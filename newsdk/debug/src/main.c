/*
* Copyright (c) 2018 Nordic Semiconductor ASA
*
* SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
*/

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample with Power management
 */
#include <stdio.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <drivers/spi.h>
#include <soc.h>
#include <device.h>
#include <pm/pm.h>
#include <hal/nrf_gpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <sys/byteorder.h>

#include <bluetooth/services/nus.h>
#include <string.h>
#include <logging/log.h>

#include <bluetooth/services/hrs.h>
#include <bluetooth/hci_vs.h>
#include <bluetooth/conn.h>
// FOTA 
#include <mgmt/mcumgr/smp_bt.h>
#include "os_mgmt/os_mgmt.h"
#include "img_mgmt/img_mgmt.h"
// Watchdog
#include <drivers/watchdog.h>
#include <sys/reboot.h>
#include <task_wdt/task_wdt.h>


#define LOG_MODULE_NAME fittag
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define CONSOLE_LABEL DT_LABEL(DT_CHOSEN(zephyr_console))
#define SLEEP_TIME_MS   1000
struct wdt_timeout_cfg wdt_config;
struct device *wdt;
int wdt_channel_id;
const struct device *gpio;

static void wdt_callback(struct device *wdt_dev, int channel_id)
{
	static bool handled_event;

	if (handled_event) {
		return;
	}

	wdt_feed(wdt_dev, channel_id);

	LOG_INF("Handled things..ready to reset\n");
	handled_event = true;
}

void install_watchdog() {
	/* Reset SoC when watchdog timer expires. */
	int err;
	wdt_config.flags = WDT_FLAG_RESET_SOC;

	/* Expire watchdog after 1000 milliseconds. */
	wdt_config.window.min = 0U;
	wdt_config.window.max = 605000U;

	/* Set up watchdog callback. Jump into it when watchdog expired. */
	wdt_config.callback = wdt_callback;

	wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
	// if (wdt_channel_id == -ENOTSUP) {
	// 	/* IWDG driver for STM32 doesn't support callback */
	// 	wdt_config.callback = NULL;
	// 	wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
	// }
	if (wdt_channel_id < 0) {
		LOG_ERR("Watchdog install error\n");
		return;
	}

	err = wdt_setup(wdt, 0);
	wdt_feed(wdt, wdt_channel_id);
}

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define WAITTIME	3000		//WRC 3 seconds

// #define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
// #define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
// #define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

#define THROUGHPUT_PACKETS_TO_SEND  (200)
#define NUMBER_THROUGHPUT_TESTS  (1)
#define THROUGH_PACKET_SIZE 50
#define RECV_THRESHOLD_CNT 100
#define RECV_THRESHOLD (THROUGH_PACKET_SIZE * RECV_THRESHOLD_CNT)
#define NUM_SPEED_CACULATIONS (THROUGHPUT_PACKETS_TO_SEND/RECV_THRESHOLD_CNT)

static K_SEM_DEFINE(ble_init_ok, 0, 1);
static K_SEM_DEFINE(throughput_sem, 0, 1);

static volatile bool data_length_req;

// #define BT_LE_ADV_CONN_SLOW BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, \
// 									0X0C80, \
// 									0X0F00, NULL)  //2s
#define BT_LE_ADV_CONN_SLOW BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, \
									BT_GAP_ADV_SLOW_INT_MIN, \
									BT_GAP_ADV_SLOW_INT_MAX, NULL)  //1s

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

const struct device *led1;
const struct device *cons;



static struct k_work_delayable ble_params_update;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[50];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);
static K_FIFO_DEFINE(fifo_transfer);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      0x84, 0xaa, 0x60, 0x74, 0x52, 0x8a, 0x8b, 0x86,
		      0xd3, 0x4c, 0xb7, 0x1d, 0x1d, 0xdc, 0x53, 0x8d),
};

#define LIS3MDL_I2C_ADDR 0x1EU
#define LIS3MDL_SPI_READ		(1 << 7)
#define LIS3MDL_WHOAMI_REG 		0x0FU
#define LIS3MDL_CTRL_REG1 		0x20U
#define LIS3MDL_CTRL_REG2 		0x21U
#define LIS3MDL_CTRL_REG3 		0x22U
#define LIS3MDL_CTRL_REG4 		0x23U
#define LIS3MDL_CTRL_REG5 		0x24U
#define LIS3MDL_STATUS_REG 		0x27U
#define LIS3MDL_OUT_X_L 		0x28U
#define LIS3MDL_OUT_X_H 		0x29U
#define LIS3MDL_OUT_Y_L 		0x2AU
#define LIS3MDL_OUT_Y_H 		0x2BU
#define LIS3MDL_OUT_Z_L 		0x2CU
#define LIS3MDL_OUT_Z_H 		0x2DU

#define LIS3MDL_ID 0x3DU

#define NUM_SENSOR 7

const struct device *cs1;
const struct device *cs3;
const struct device *spi;

struct spi_config spi_ctg1 = {0};
struct spi_config spi_ctg2 = {0};
struct spi_config spi_ctg3 = {0};
struct spi_config spi_ctg4 = {0};
struct spi_config spi_ctg5 = {0};
struct spi_config spi_ctg6 = {0};
struct spi_config spi_ctg7 = {0};

struct spi_cs_control spi_cs1;
struct spi_cs_control spi_cs2;
struct spi_cs_control spi_cs3;
struct spi_cs_control spi_cs4;
struct spi_cs_control spi_cs5;
struct spi_cs_control spi_cs6;
struct spi_cs_control spi_cs7;

struct sensor_data_t {
	uint8_t sensor_id;
	uint16_t x_value;
	uint16_t y_value;
	uint16_t z_value;
	uint32_t timestamp;
};



void lis3mdl_spi_read(struct spi_config spi_ctg, uint8_t reg_addr, uint8_t *value, uint8_t len) {

	uint8_t buffer_tx[2] = { reg_addr | LIS3MDL_SPI_READ, 0 };
	const struct spi_buf tx_buf = {
			.buf = buffer_tx,
			.len = 2,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 1,
		},
		{
			.buf = value,
			.len = len,
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2
	};

	int err = spi_transceive(spi, &spi_ctg, &tx, &rx);
	if (err){
		LOG_WRN("Fail SPI read");
	}
}

void lis3mdl_spi_write(struct spi_config spi_ctg, uint8_t reg_addr, uint8_t *value, uint8_t len) {
	uint8_t buffer_tx[1] = { reg_addr & ~LIS3MDL_SPI_READ };
	const struct spi_buf tx_buf[2] = {
		{
			.buf = buffer_tx,
			.len = 1,
		},
		{
			.buf = value,
			.len = len,
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2
	};

	int err = spi_transceive(spi, &spi_ctg, &tx, NULL);
	if (err){
		LOG_WRN("Fail SPI write");
	}
}

bool magnet_present(struct spi_config spi_ctg) {
	uint8_t whoamI;

	lis3mdl_spi_read(spi_ctg, LIS3MDL_WHOAMI_REG, (uint8_t * ) &whoamI, sizeof(whoamI));
	return whoamI == LIS3MDL_ID;
}

void lis3mdl_data_rate(struct spi_config spi_ctg) {

	uint8_t ctrl_reg1 = 0b00000010; // FAST MODE
	uint8_t ctrl_reg4 = 0b00000000;
	lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG1, (uint8_t * ) &ctrl_reg1, sizeof(ctrl_reg1));
	lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG4, (uint8_t * ) &ctrl_reg4, sizeof(ctrl_reg4));
}

void lis3mdl_full_scale_set(struct spi_config spi_ctg) {

	uint8_t ctrl_reg2 = 0b01100000; // 16 gauss
	lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG2, (uint8_t * ) &ctrl_reg2, sizeof(ctrl_reg2));
}

void lis3mdl_opearting_mode(struct spi_config spi_ctg) {

	uint8_t ctrl_reg3 = 0b00000000; // Continuous conversion mode
	lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG3, (uint8_t * ) &ctrl_reg3, sizeof(ctrl_reg3));
}

void lis3mdl_block_data_update_set(struct spi_config spi_ctg) {

	uint8_t ctrl_reg5 = 0x40; // Could this be the culprit???
	lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG5, (uint8_t * ) &ctrl_reg5, sizeof(ctrl_reg5));
}

void lis3mdl_init(struct spi_config spi_ctg) {

	lis3mdl_block_data_update_set(spi_ctg); // No idea what this does
	lis3mdl_data_rate(spi_ctg);
	lis3mdl_full_scale_set(spi_ctg);
	lis3mdl_opearting_mode(spi_ctg);
}

uint8_t lis3mdl_status(struct spi_config spi_ctg) {
	uint8_t ret;
	lis3mdl_spi_read(spi_ctg, LIS3MDL_STATUS_REG, (uint8_t * ) &ret, sizeof(ret));
	// i2c_reg_read_byte(i2c_device, LIS3MDL_I2C_ADDR, LIS3MDL_STATUS_REG, &ret);
	return ((ret >> 4) & 0x01);
}

float convert(int16_t lsb) {
	return (((float) lsb) / 1711.0f);
}

void lis3mdl_get_x(struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
	uint8_t data[2];

	lis3mdl_spi_read(spi_ctg, LIS3MDL_OUT_X_L, (uint8_t * ) &data[0], sizeof(data[0]));
	lis3mdl_spi_read(spi_ctg, LIS3MDL_OUT_X_H, (uint8_t * ) &data[1], sizeof(data[1]));

	sensor_data->x_value = (data[1] << 8 | data[0]);

}

void lis3mdl_get_y(struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
	uint8_t data[2];
	lis3mdl_spi_read(spi_ctg, LIS3MDL_OUT_Y_L, (uint8_t * ) &data[0], sizeof(data[0]));
	lis3mdl_spi_read(spi_ctg, LIS3MDL_OUT_Y_H, (uint8_t * ) &data[1], sizeof(data[1]));

	sensor_data->y_value = (data[1] << 8 | data[0]);
}

void lis3mdl_get_z(struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
	uint8_t data[2];
	lis3mdl_spi_read(spi_ctg, LIS3MDL_OUT_Z_L, (uint8_t * ) &data[0], sizeof(data[0]));
	lis3mdl_spi_read(spi_ctg, LIS3MDL_OUT_Z_H, (uint8_t * ) &data[1], sizeof(data[1]));

	sensor_data->z_value = (data[1] << 8 | data[0]);
}

void lis3mdl_get_xyz(struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
	uint8_t status;
	uint8_t data[6];
	// if (sensor_data->sensor_id == 0) {
	// 	do{
	// 		status = lis3mdl_status(spi_ctg);
	// 	}while(!status);
	// }

	lis3mdl_spi_read(spi_ctg, LIS3MDL_OUT_X_L | (1 << 6), (uint8_t * ) &data, sizeof(data));
	sensor_data->x_value = (data[1] << 8 | data[0]);
	sensor_data->y_value = (data[3] << 8 | data[2]);
	sensor_data->z_value = (data[5] << 8 | data[4]);
}

void sensor_mode(uint8_t mode) {

	struct spi_config spi_ctg;

	for (int i=0; i < NUM_SENSOR; i++) {
		switch (i)
		{
		case 0:
			spi_ctg = spi_ctg1;
			break;
		case 1:
			spi_ctg = spi_ctg2;
			break;
		case 2:
			spi_ctg = spi_ctg3;
			break;
		case 3:
			spi_ctg = spi_ctg4;
			break;
		case 4:
			spi_ctg = spi_ctg5;
			break;
		case 5:
			spi_ctg = spi_ctg6;
			break;
		case 6:
			spi_ctg = spi_ctg7;
			break;
		default:
			break;
		}

		if (mode == 0) {

			lis3mdl_poweroff(spi_ctg);
			k_msleep(20);
		} else {
			lis3mdl_init(spi_ctg);
			k_msleep(20);
		}
	}
}

void lis3mdl_poweroff(struct spi_config spi_ctg) {
	// uint8_t ctrl_reg2 = 0x08; // reboot sensor
	uint8_t ctrl_reg = 0b00000011; // power down
	// lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG2, (uint8_t * ) &ctrl_reg2, sizeof(ctrl_reg2));
	lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG3, (uint8_t * ) &ctrl_reg, sizeof(ctrl_reg));
}

void hal_spi_init(void) {
	spi = device_get_binding("SPI_2");

	if (spi == NULL) {
		LOG_ERR("Cannot create Spi instance. No spi device found.");
		return;
	}

	spi_ctg1.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg1.frequency = 8000000;
	spi_ctg1.slave = 0;
	spi_cs1.gpio_dev = device_get_binding("GPIO_1");
	spi_cs1.gpio_pin = 4;
	spi_cs1.delay = 0;
	spi_cs1.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg1.cs = &spi_cs1;

	spi_ctg2.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg2.frequency = 8000000;
	spi_ctg2.slave = 0;
	spi_cs2.gpio_dev = device_get_binding("GPIO_1");
	spi_cs2.gpio_pin = 5;
	spi_cs2.delay = 0;
	spi_cs2.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg2.cs = &spi_cs2;

	spi_ctg3.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg3.frequency = 8000000;
	spi_ctg3.slave = 0;
	spi_cs3.gpio_dev = device_get_binding("GPIO_1");
	spi_cs3.gpio_pin = 6;
	spi_cs3.delay = 0;
	spi_cs3.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg3.cs = &spi_cs3;

	spi_ctg4.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg4.frequency = 8000000;
	spi_ctg4.slave = 0;
	spi_cs4.gpio_dev = device_get_binding("GPIO_1");
	spi_cs4.gpio_pin = 7;
	spi_cs4.delay = 0;
	spi_cs4.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg4.cs = &spi_cs4;

	spi_ctg5.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg5.frequency = 8000000;
	spi_ctg5.slave = 0;
	spi_cs5.gpio_dev = device_get_binding("GPIO_1");
	spi_cs5.gpio_pin = 8;
	spi_cs5.delay = 0;
	spi_cs5.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg5.cs = &spi_cs5;

	spi_ctg6.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg6.frequency = 8000000;
	spi_ctg6.slave = 0;
	spi_cs6.gpio_dev = device_get_binding("GPIO_1");
	spi_cs6.gpio_pin = 9;
	spi_cs6.delay = 0;
	spi_cs6.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg6.cs = &spi_cs6;

	spi_ctg7.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg7.frequency = 8000000;
	spi_ctg7.slave = 0;
	spi_cs7.gpio_dev = device_get_binding("GPIO_1");
	spi_cs7.gpio_pin = 10;
	spi_cs7.delay = 0;
	spi_cs7.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg7.cs = &spi_cs7;

	sensor_mode(1);
	sensor_mode(0);
	
}

static int connection_configuration_set(const struct bt_le_conn_param *conn_param)
{
	int err;
	struct bt_conn_info info = {0};

	err = bt_conn_get_info(current_conn, &info);
	if (err) {
		LOG_ERR("Failed to get connection info %d", err);
		return err;
	}

	if (info.le.interval != conn_param->interval_max) {
		err = bt_conn_le_param_update(current_conn, conn_param);
		if (err) {
			LOG_ERR("Connection parameters update failed: %d",
				    err);
			return err;
		}

		LOG_INF("Connection parameters update pending");
	}

	return 0;
}
#define INTERVAL_MIN	6	/* x * 1.25 ms */
#define INTERVAL_MAX	12	/* x * 1.25 ms */

void params_update()
{
     const struct bt_le_conn_param *conn_param =
            BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, 0, 42);

    connection_configuration_set(conn_param);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int8_t rssi = 0xFF;
	int8_t wdt_err;
	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected");

	current_conn = bt_conn_ref(conn);
	install_watchdog();
	LOG_INF("Watchdog starting...");
	pm_device_state_set(spi, PM_DEVICE_STATE_ACTIVE,NULL,NULL);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int wdt_err;
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected:(reason %u)", reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	
	}

	pm_device_state_set(spi, PM_DEVICE_STATE_LOW_POWER,NULL,NULL);  
}

static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}


static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	LOG_INF("Connection parameters update request received.");
	LOG_INF("Minimum interval: %d, Maximum interval: %d",
	       param->interval_min, param->interval_max);
	LOG_INF("Latency: %d, Timeout: %d", param->latency, param->timeout);

	return true;
}

static void le_phy_updated(struct bt_conn *conn,
			   struct bt_conn_le_phy_info *param)
{
	LOG_INF("LE PHY updated: TX PHY %s, RX PHY %s",
	       phy2str(param->tx_phy), phy2str(param->rx_phy));

	//k_sem_give(&throughput_sem);
}

static void le_data_length_updated(struct bt_conn *conn,
				   struct bt_conn_le_data_len_info *info)
{
	if (!data_length_req) {
		return;
	}

	LOG_INF("LE data len updated: TX (len: %d time: %d)"
	       " RX (len: %d time: %d)", info->tx_max_len,
	       info->tx_max_time, info->rx_max_len, info->rx_max_time);

	data_length_req = false;
	//k_sem_give(&throughput_sem);
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	LOG_INF("Connection parameters updated.\n");
	LOG_INF("Interval: %d, latency: %d, timeout: %d",
	       interval, latency, timeout);

	k_sem_give(&throughput_sem);
}


static struct bt_conn_cb conn_callbacks = {
	.connected    = connected,
	.disconnected = disconnected,
    .le_param_req = le_param_req,
    .le_param_updated = le_param_updated,
    .le_phy_updated = le_phy_updated,
    .le_data_len_updated = le_data_length_updated
};

static  uint32_t sent_cnt = 0;
static inline void bt_sent_cb(struct bt_conn *conn)
{
    sent_cnt++;
}
static inline uint32_t get_sent_cnt(void)
{
    return sent_cnt;
}

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			uint16_t len)
{
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));
	struct uart_data_t *buff;
	buff = k_malloc(sizeof(*buff));
	memcpy(buff->data,data,len);
	buff->len=len;

	k_fifo_put(&fifo_transfer, buff);
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
	.sent = bt_sent_cb,
};

void read_conn_rssi(uint16_t handle, int8_t *rssi)
{
	struct net_buf *buf, *rsp = NULL;
	struct bt_hci_cp_read_rssi *cp;
	struct bt_hci_rp_read_rssi *rp;

	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_READ_RSSI, sizeof(*cp));
	if (!buf) {
		LOG_ERR("Unable to allocate command buffer");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);
	if (err) {
		uint8_t reason = rsp ?
			((struct bt_hci_rp_read_rssi *)rsp->data)->status : 0;
		LOG_INF("Read RSSI err: %d reason 0x%02x", err, reason);
		return;
	}

	rp = (void *)rsp->data;
	*rssi = rp->rssi;

	net_buf_unref(rsp);
}

static void set_tx_power(uint8_t handle_type, uint16_t handle, int8_t tx_pwr_lvl)
{
	struct bt_hci_cp_vs_write_tx_power_level *cp;
	struct bt_hci_rp_vs_write_tx_power_level *rp;
	struct net_buf *buf, *rsp = NULL;
	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
				sizeof(*cp));
	if (!buf) {
		LOG_INF("Unable to allocate command buffer");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);
	cp->handle_type = handle_type;
	cp->tx_power_level = tx_pwr_lvl;

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
				   buf, &rsp);
	if (err) {
		uint8_t reason = rsp ?
			((struct bt_hci_rp_vs_write_tx_power_level *)
			  rsp->data)->status : 0;
		LOG_INF("Set Tx power err: %d reason 0x%02x", err, reason);
		return;
	}

	rp = (void *)rsp->data;
	LOG_INF("Actual Tx Power: %d", rp->selected_tx_power);

	net_buf_unref(rsp);
}

static void get_tx_power(uint8_t handle_type, uint16_t handle, int8_t *tx_pwr_lvl)
{
	struct bt_hci_cp_vs_read_tx_power_level *cp;
	struct bt_hci_rp_vs_read_tx_power_level *rp;
	struct net_buf *buf, *rsp = NULL;
	int err;

	*tx_pwr_lvl = 0xFF;
	buf = bt_hci_cmd_create(BT_HCI_OP_VS_READ_TX_POWER_LEVEL,
				sizeof(*cp));
	if (!buf) {
		LOG_ERR("Unable to allocate command buffer");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);
	cp->handle_type = handle_type;

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_READ_TX_POWER_LEVEL,
				   buf, &rsp);
	if (err) {
		uint8_t reason = rsp ?
			((struct bt_hci_rp_vs_read_tx_power_level *)
			  rsp->data)->status : 0;
		LOG_ERR("Read Tx power err: %d reason 0x%02x\n", err, reason);
		return;
	}

	rp = (void *)rsp->data;
	*tx_pwr_lvl = rp->tx_power_level;

	net_buf_unref(rsp);
}

void ble_transfer(struct sensor_data_t *data, uint16_t count) {

	static char buf[THROUGH_PACKET_SIZE];
    uint32_t send_count = 0;
    int err;
    uint32_t send_count_uplimit = count;
	uint8_t err_tick;

	sent_cnt = 0; // clear sent counting
	err_tick = 0; // Check connection is still there
	uint32_t start = k_uptime_get_32();
    while(send_count < send_count_uplimit){
		memset(buf,0,sizeof(buf));
		sprintf(buf, "%d %f %f %f %u\n",data[send_count].sensor_id, convert(data[send_count].x_value), convert(data[send_count].y_value), convert(data[send_count].z_value),  data[send_count].timestamp);
		if (err_tick > 20) {

			// force disconnect from gateway if data cant send after a few tries, more functionalities can be added here
			// IE: fs for unsent data 
			bt_conn_disconnect(current_conn ,BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			break;  
		}
        err = bt_nus_send(NULL, buf, sizeof(buf));
        if (err) {
            LOG_WRN("Failed to send data over BLE connection");
			err_tick++;
        }
        else {
            send_count++;
        }

        if(send_count - get_sent_cnt() > (CONFIG_BT_L2CAP_TX_BUF_COUNT)){
             LOG_WRN("Buffer getting tight, wait sometime here");
             k_sleep(K_MSEC(1));
        }
		k_sleep(K_MSEC(1));
    }
	uint32_t end = k_uptime_get_32() - start;
	LOG_INF("BLE write time:%u", end);
	// Send acknoledge when data is done transferring
	memset(buf,0,sizeof(buf));
	sprintf(buf, "0\n");
	err = bt_nus_send(NULL, buf, sizeof(buf));
	if (err) {
            LOG_WRN("Failed to send data over BLE connection");
    }
	// bt_conn_disconnect(current_conn ,BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

void send_data(uint16_t count) {
	struct sensor_data_t *magnet;
	magnet = k_malloc(sizeof(struct sensor_data_t) * count);

	uint8_t count_temp = 0;
	struct spi_config spi_ctg;

	sensor_mode(1); // turn on all sensors

	for (int i=0; i < count; i++) {
			if (count_temp == 7) {
				count_temp = 0;
			}

			switch (count_temp)
			{
			case 0:
				spi_ctg = spi_ctg1;
				break;
			case 1:
				spi_ctg = spi_ctg2;
				break;
			case 2:
				spi_ctg = spi_ctg3;
				break;
			case 3:
				spi_ctg = spi_ctg4;
				break;
			case 4:
				spi_ctg = spi_ctg5;
				break;
			case 5:
				spi_ctg = spi_ctg6;
				break;
			case 6:
				spi_ctg = spi_ctg7;
				break;
			default:
				break;

			}

		magnet[i].sensor_id = count_temp;
		while(1) {
			uint32_t temp_time = k_cyc_to_us_floor32(k_cycle_get_32());
			if (i < 7) {
				lis3mdl_get_xyz(spi_ctg, &(magnet[i]));
				magnet[i].timestamp = temp_time;
				break;
			}

			else if (temp_time - magnet[i-6].timestamp > 999) {
				lis3mdl_get_xyz(spi_ctg, &(magnet[i]));
				magnet[i].timestamp = temp_time; // get timestamp in microseconds 
				break;
			}
		}
		count_temp++;
	}

	sensor_mode(0); // turn off all sensors
	ble_transfer(magnet, count);
	k_free(magnet);
	
}

static char* process_command(struct uart_data_t *buf) {
	char string[20]; // string test
	memcpy(string, buf->data, sizeof(buf->data));
	string[sizeof(buf->data)] = '\0';
	LOG_INF("Command received: %s", string);
	char *token;
	char *rest = string;
	char **array = (char**)k_malloc(3*sizeof(char*));
	for (int j=0; j < 3; j++)
		array[j] = (char*) k_malloc(sizeof(char)*10);

	int i = 0;
	while((token = strtok_r(rest, " ", &rest))) {
		strcpy(array[i], token);
		i++;
		if (i > 2) {
			break;
		}
	}

	return array;
}

void main(void)
{
	wdt = device_get_binding(DT_LABEL(DT_INST(0, nordic_nrf_watchdog)));
	if (!wdt) {
		LOG_ERR("Cannot get WDT device\n");
		return;
	}
	// gpio = device_get_binding("GPIO_1");
	// if (gpio == NULL) {
	// 	return;
	// }
	int err = 0;
	int8_t txp = 3;
	int8_t txp_get = 0xFF;
	
	pm_constraint_set(PM_STATE_SOFT_OFF);
	hal_spi_init();
	
	os_mgmt_register_group();
	img_mgmt_register_group();
	smp_bt_register();

	bt_conn_cb_register(&conn_callbacks);
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Failed to initialize UEnable BT (err: %d)", err);
		return;
	}

	LOG_INF("Bluetooth initialized");

	get_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0, &txp_get);
	LOG_INF("-> default. TXP = %d", txp_get);

	set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0, txp);

	k_sem_give(&ble_init_ok);

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_SLOW, ad, ARRAY_SIZE(ad), sd,
				ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
	}
	
	pm_device_state_set(spi, PM_DEVICE_STATE_LOW_POWER,NULL,NULL);  
	// pm_device_state_set(gpio, PM_DEVICE_STATE_OFF,NULL,NULL);
}

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);
	struct sensor_data_t *magnet;
	uint16_t count;

	for (;;) {
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_transfer,
							K_FOREVER);

		params_update();
		char **command = (char**)k_malloc(3*sizeof(char*));
		for (int j=0; j < 3; j++)
			command[j] = (char*) k_malloc(sizeof(char)*10);

		command = process_command(buf);
		k_sem_take(&throughput_sem, K_FOREVER);
		if (strcmp("sensor", command[0]) == 0) { // why is this statement not working?????
			if (strcmp(NULL, command[1]) == 0) {

			}
		}

		if (strcmp(NULL, command[1]) == 0) {
			if (strcmp("sample", command[0]) == 0) {
				memset(buf, 0, sizeof(buf));
				sprintf(buf, "Need sample size(1 - 10000)\n");
				int err = bt_nus_send(NULL, buf, sizeof(buf));
				if (err) {
					LOG_WRN("Failed to send data over BLE connection");
				}
			}
		}

		// This part is shitty...
		else if (strcmp("sample", command[0]) == 0) {
			if (strcmp(NULL, command[1])!=0) {
				count = atoi(command[1]);
				if (count > 10000){

				} else
					send_data(count);
			}
		}
		// sensor_mode(0); // turn off all sensors
		k_free(command);
		k_free(buf);

	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);
