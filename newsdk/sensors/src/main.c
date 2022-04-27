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

#include <bluetooth/services/nus.h>
#include <string.h>
#include <logging/log.h>

// #include <mgmt/mcumgr/smp_bt.h>
// #include "os_mgmt/os_mgmt.h"
// #include "img_mgmt/img_mgmt.h"

#define LOG_MODULE_NAME fittag
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define CONSOLE_LABEL DT_LABEL(DT_CHOSEN(zephyr_console))
#define SLEEP_TIME_MS   1000
// #define LED0_NODE DT_ALIAS(led0)

// #if DT_NODE_HAS_STATUS(LED0_NODE, okay)
// #define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
// #define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
// #define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
// #else
// /* A build error here means your board isn't set up to blink an LED. */
// #error "Unsupported board: led0 devicetree alias is not defined"
// #define LED0	""
// #define PIN	0
// #define FLAGS	0
// #endif

// #define LED1_NODE DT_ALIAS(led1)
// #if DT_NODE_HAS_STATUS(LED1_NODE, okay)
// #define LED1	DT_GPIO_LABEL(LED1_NODE, gpios)
// #define PIN1	DT_GPIO_PIN(LED1_NODE, gpios)
// #define FLAGS1	DT_GPIO_FLAGS(LED1_NODE, gpios)
// #else
// /* A build error here means your board isn't set up to blink an LED. */
// #error "Unsupported board: led1 devicetree alias is not defined"
// #define LED1	""
// #define PIN1	0
// #define FLAGS1	0
// #endif

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define WAITTIME	3000		//WRC 3 seconds

// #define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
// #define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
// #define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

// #define NUM_SAMPLE 10

// #define THROUGHPUT_PACKETS_TO_SEND  (200)
// #define NUMBER_THROUGHPUT_TESTS  (1)
// #define THROUGH_PACKET_SIZE 50
// #define RECV_THRESHOLD_CNT 100
// #define RECV_THRESHOLD (THROUGH_PACKET_SIZE * RECV_THRESHOLD_CNT)
// #define NUM_SPEED_CACULATIONS (THROUGHPUT_PACKETS_TO_SEND/RECV_THRESHOLD_CNT)

// static K_SEM_DEFINE(ble_init_ok, 0, 1);

// // #define BT_LE_ADV_CONN_SLOW BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, \
// // 									0X0C80, \
// // 									0X0F00, NULL)  //2s
// #define BT_LE_ADV_CONN_SLOW BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, \
// 									BT_GAP_ADV_FAST_INT_MIN_2, \
// 									BT_GAP_ADV_FAST_INT_MAX_2, NULL)  //1s
// static struct bt_conn *current_conn;
// static struct bt_conn *auth_conn;

// const struct device *led1;

// struct uart_data_t {
// 	void *fifo_reserved;
// 	uint8_t data[UART_BUF_SIZE];
// 	uint16_t len;
// };

// static K_FIFO_DEFINE(fifo_uart_tx_data);
// static K_FIFO_DEFINE(fifo_uart_rx_data);
// static K_FIFO_DEFINE(fifo_transfer);

// static const struct bt_data ad[] = {
// 	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
// 	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
// };

// // static const struct bt_data sd[] = {
// // 	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
// // };

// static const struct bt_data sd[] = {
// 	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
// 		      0x84, 0xaa, 0x60, 0x74, 0x52, 0x8a, 0x8b, 0x86,
// 		      0xd3, 0x4c, 0xb7, 0x1d, 0x1d, 0xdc, 0x53, 0x8d),
// };

#define MMC5983_SPI_READ		(1 << 7)
#define MMC5983_OUT_X_L 		0x00U
#define MMC5983_OUT_X_H 		0x01U
#define MMC5983_OUT_Y_L 		0x02U
#define MMC5983_OUT_Y_H 		0x03U
#define MMC5983_OUT_Z_L 		0x04U
#define MMC5983_OUT_Z_H 		0x05U
#define MMC5983_OUT_EXT			0x06U
#define MMC5983_OUT_TEMP		0x07U
#define MMC5983_STATUS 			0x08U
#define MMC5983_CTRL_REG_0		0x09U
#define MMC5983_CTRL_REG_1		0x0AU
#define MMC5983_CTRL_REG_2		0x0BU
#define MMC5983_CTRL_REG_3		0x0CU
#define MMC59832_WHOAMI_REG		0x2FU

#define NUM_SENSOR 1

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
	uint32_t sensor_id;
	uint32_t x_value;
	uint32_t y_value;
	uint32_t z_value;
	uint32_t timestamp;
};

void hal_spi_init(void) {
	spi = device_get_binding("SPI_2");

	if (spi == NULL) {
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
}


void mmc5983_spi_read(struct spi_config spi_ctg, uint8_t reg_addr, uint8_t *value, uint8_t len) {

	uint8_t buffer_tx[2] = { reg_addr | MMC5983_SPI_READ, 0 };
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

void mmc5983_spi_write(struct spi_config spi_ctg, uint8_t reg_addr, uint8_t *value, uint8_t len) {
	uint8_t buffer_tx[1] = { reg_addr & ~MMC5983_SPI_READ };
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

	mmc5983_spi_read(spi_ctg, MMC59832_WHOAMI_REG, (uint8_t * ) &whoamI, sizeof(whoamI));
	return whoamI == 0x30;
}

void mmc5983_init(struct spi_config spi_ctg) {
	uint8_t reg_0 = 0x20;
	uint8_t reg_1 = 0x03;
	uint8_t reg_2 = 0x0f;
	mmc5983_spi_write(spi_ctg, MMC5983_CTRL_REG_0, (uint8_t *) &reg_0, sizeof(reg_0)); //
	mmc5983_spi_write(spi_ctg, MMC5983_CTRL_REG_1, (uint8_t *) &reg_1, sizeof(reg_1)); //
	mmc5983_spi_write(spi_ctg, MMC5983_CTRL_REG_2, (uint8_t *) &reg_2, sizeof(reg_2)); //  
}

float convert(int16_t lsb) {
	return (((float) lsb) / 16384.0);
}

void mmc5983_get_x(struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
	uint8_t data[2];

	mmc5983_spi_read(spi_ctg, MMC5983_OUT_X_L, (uint8_t * ) &data[0], sizeof(data[0]));
	mmc5983_spi_read(spi_ctg, MMC5983_OUT_X_H, (uint8_t * ) &data[1], sizeof(data[1]));

	sensor_data->x_value = (data[1] << 8 | data[0]);

}

void mmc5983_get_y(struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
	uint8_t data[2];
	mmc5983_spi_read(spi_ctg, MMC5983_OUT_Y_L, (uint8_t * ) &data[0], sizeof(data[0]));
	mmc5983_spi_read(spi_ctg, MMC5983_OUT_Y_H, (uint8_t * ) &data[1], sizeof(data[1]));

	sensor_data->y_value = (data[1] << 8 | data[0]);
}

void mmc5983_get_z(struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
	uint8_t data[2];
	mmc5983_spi_read(spi_ctg, MMC5983_OUT_Z_L, (uint8_t * ) &data[0], sizeof(data[0]));
	mmc5983_spi_read(spi_ctg, MMC5983_OUT_Z_H, (uint8_t * ) &data[1], sizeof(data[1]));

	sensor_data->z_value = (data[1] << 8 | data[0]);
}

void mmc5983_get_xyz(struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
	uint8_t status;
	uint8_t data[6];

	mmc5983_spi_read(spi_ctg, MMC5983_OUT_X_L | (1 << 7), (uint8_t * ) &data, sizeof(data));
	sensor_data->x_value = (uint32_t) (data[1] << 10 | data[0] << 2 | data[7] & 0xC0);
	sensor_data->y_value = (uint32_t) (data[3] << 10 | data[2] << 2 | data[7] & 0x30);
	sensor_data->z_value = (uint32_t) (data[5] << 10 | data[4] << 2 | data[7] & 0x0C);
}

void mmc5983_powerdown(struct spi_config spi_ctg) {
	uint8_t ctrl_reg = 0x07; // power down

	mmc5983_spi_write(spi_ctg, MMC5983_CTRL_REG_2, (uint8_t * ) &ctrl_reg, sizeof(ctrl_reg));
}

void mmc5983_powerup(struct spi_config spi_ctg, uint8_t modr) {

	mmc5983_spi_write(spi_ctg, MMC5983_CTRL_REG_2, (uint8_t * ) &modr, sizeof(modr));
}

void mmc5983_reset(struct spi_config spi_ctg) {
	uint8_t reset = 0x80;
	mmc5983_spi_write(spi_ctg, MMC5983_CTRL_REG_1, (uint8_t * ) &reset, sizeof(reset));
	k_msleep(10);
}

void mmc5983_SET(struct spi_config spi_ctg) {
	uint8_t set = 0x08;
	mmc5983_spi_write(spi_ctg, MMC5983_CTRL_REG_0, (uint8_t * ) &set, sizeof(set));
}

void mmc5983_RESET(struct spi_config spi_ctg) {
	uint8_t reset = 0x10;
	mmc5983_spi_write(spi_ctg, MMC5983_CTRL_REG_0, (uint8_t * ) &reset, sizeof(reset));
}

void mmc5983_sampling(struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
	uint8_t status;
	uint8_t temp;
	uint32_t start, end;
	mmc5983_spi_read(spi_ctg, MMC5983_CTRL_REG_0, (uint8_t *) &temp, sizeof(temp));
	temp = temp | 0x01;
	mmc5983_spi_write(spi_ctg, MMC5983_CTRL_REG_0, (uint8_t *) &temp, sizeof(temp)); //Initialise reading
	do {
		mmc5983_spi_read(spi_ctg, MMC5983_STATUS, (uint8_t *) &status, sizeof(status));
		// printk("%d\n", (status & 0x01));

	} while ((status & 0x01)!= 1);
	uint8_t data[7];

	mmc5983_spi_read(spi_ctg, MMC5983_OUT_X_L, (uint8_t * ) &data, sizeof(data));
	sensor_data->x_value = (uint32_t) (data[1] << 10 | data[0] << 2 | data[7] & 0xC0);
	sensor_data->y_value = (uint32_t) (data[3] << 10 | data[2] << 2 | data[7] & 0x30);
	sensor_data->z_value = (uint32_t) (data[5] << 10 | data[4] << 2 | data[7] & 0x0C);
}

void main(void)
{
	int rc;
	int err = 0;
	int count = 10;
	const struct device *cons;
	struct sensor_data_t *magnet;
	magnet = k_malloc(sizeof(struct sensor_data_t) * count);
	// cons=device_get_binding(CONSOLE_LABEL);
	// uint32_t temp_time; 
	// printk("CONS=%d\n",cons);
	// bool led_is_on = true;
	int ret;
	hal_spi_init();
	if(magnet_present(spi_ctg1)) {
		printk("Sensor detected\n");
	}
	mmc5983_init(spi_ctg1);

	// if(magnet_present) {
	// 	printk("Sensor detected\n");
	// }

	while(1) {
		for (int i=0; i < count; i++) { 
			// int i = 0;
			magnet[i].sensor_id = 0;
			magnet[i].timestamp = k_cyc_to_us_floor32(k_cycle_get_32());
			mmc5983_sampling(spi_ctg1, &(magnet[i]));	
		}

		for (int i=0; i< count; i++) {
			printk("%d %f %f %f %u\n",magnet[i].sensor_id, convert(magnet[i].x_value), convert(magnet[i].y_value), convert(magnet[i].z_value), magnet[i].timestamp);
		}
		k_sleep(K_MSEC(1));
	}
}

