
#include <zephyr/types.h>
#include <zephyr.h>
#include <drivers/spi.h>
#include <sys/time_units.h>

#include <device.h>
#include <soc.h>


#include <stdio.h>
#include <string.h>

#include <logging/log.h>

// #define LOG_MODULE_NAME peripheral_uart
// LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* The devicetree node identifier for the "led0" alias. */
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

#define NUM_SAMPLE 10
#define CONSOLE_LABEL DT_LABEL(DT_CHOSEN(zephyr_console))
#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

// #define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
// #define PRIORITY 7

// #define DEVICE_NAME CONFIG_BT_DEVICE_NAME
// #define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

// #define RUN_STATUS_LED DK_LED1
// #define RUN_LED_BLINK_INTERVAL 1000

// #define CON_STATUS_LED DK_LED2

// #define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
// #define KEY_PASSKEY_REJECT DK_BTN2_MSK

// #define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
// #define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
// #define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

// #define THROUGHPUT_PACKETS_TO_SEND  (200)
// #define NUMBER_THROUGHPUT_TESTS  (1)
// #define THROUGH_PACKET_SIZE 244
// #define RECV_THRESHOLD_CNT 100
// #define RECV_THRESHOLD (THROUGH_PACKET_SIZE * RECV_THRESHOLD_CNT)
// #define NUM_SPEED_CACULATIONS (THROUGHPUT_PACKETS_TO_SEND/RECV_THRESHOLD_CNT)

// static volatile bool data_length_req;

// static K_SEM_DEFINE(ble_init_ok, 0, 1);
// static K_SEM_DEFINE(throughput_sem, 0, 1);

// static struct bt_conn *current_conn;
// static struct bt_conn *auth_conn;

// static const struct device *uart;
// static struct k_delayed_work uart_work;
// static struct k_delayed_work ble_params_update;

// struct uart_data_t {
// 	void *fifo_reserved;
// 	uint8_t data[UART_BUF_SIZE];
// 	uint16_t len;
// };

// static K_FIFO_DEFINE(fifo_uart_tx_data);
// static K_FIFO_DEFINE(fifo_uart_rx_data);

// static const struct bt_data ad[] = {
// 	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
// 	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
// };

// static const struct bt_data sd[] = {
// 	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
// 		      0x84, 0xaa, 0x60, 0x74, 0x52, 0x8a, 0x8b, 0x86,
// 		      0xd3, 0x4c, 0xb7, 0x1d, 0x1d, 0xdc, 0x53, 0x8d),
// };

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
	uint32_t sensor_id;
	uint16_t x_value;
	uint16_t y_value;
	uint16_t z_value;
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
}


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
		LOG_ERR("Fail SPI read");
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
		LOG_ERR("Fail SPI write");
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

	uint8_t ctrl_reg5 = 0x40;
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
	if (sensor_data->sensor_id == 0) {
		do{
			status = lis3mdl_status(spi_ctg);
		}while(!status);
	}

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
		} else {
			lis3mdl_init(spi_ctg);
			k_msleep(20);
		}
	}
}

void lis3mdl_poweroff(struct spi_config spi_ctg) {
	uint8_t ctrl_reg = 0b00000010; // power down

	lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG3, (uint8_t * ) ctrl_reg, sizeof(ctrl_reg));
}

void main(void)
{
	const struct device *cons;
	cons=device_get_binding(CONSOLE_LABEL);

	hal_spi_init();
	// lis3mdl_init(spi_ctg1);
	sensor_mode(1);
	struct sensor_data_t *magnet;
	while(1) {
		// uint8_t count = 0;
		// uint32_t start_time = k_uptime_get_32(); 
		magnet = k_malloc(sizeof(*magnet) * NUM_SAMPLE);
		uint8_t count = 0;
		struct spi_config spi_ctg;
		// magnet[0].x_value = 0;
		for (int i = 0; i < NUM_SAMPLE; i++) {
			if (count == 7) {
				count = 0;
			}
			switch (count)
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

			magnet[i].sensor_id = count;
			lis3mdl_get_xyz(spi_ctg, &magnet[i]);
			magnet[i].timestamp = k_uptime_get_32();
			
			printk("%d %f %f %f %d\n",magnet[i].sensor_id, convert(magnet[i].x_value), convert(magnet[i].y_value), convert(magnet[i].z_value),  magnet[i].timestamp);
			// printk("terst\n");
			count++;
			k_msleep(100);		
		}
		
		// uint32_t end_time = k_uptime_get_32();
		k_free(magnet);
		k_msleep(100);
	}
}
