#include <drivers/spi.h> 
#include <logging/log.h>
#include <hal/nrf_gpio.h>

#include <soc.h>
#include <device.h>

#include "hal_spi.h"

#define LIS3MDL_SPI_READ		(1 << 7)
#define LOG_MODULE_NAME spi
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

const struct device *spi;
struct spi_sensor_grp *spi_group;

uint8_t spi_count;
uint8_t sensor_avail;

struct spi_config spi_ctg1;
struct spi_config spi_ctg2;
struct spi_config spi_ctg3;
struct spi_config spi_ctg4;
struct spi_config spi_ctg5;
struct spi_config spi_ctg6;
struct spi_config spi_ctg7;

struct spi_cs_control spi_cs1;
struct spi_cs_control spi_cs2;
struct spi_cs_control spi_cs3;
struct spi_cs_control spi_cs4;
struct spi_cs_control spi_cs5;
struct spi_cs_control spi_cs6;
struct spi_cs_control spi_cs7;

void hal_spi_init(void) {
    spi = device_get_binding("SPI_2");

	if (spi == NULL) {
		LOG_ERR("No SPI device binding.");
		return;
	}

	spi_ctg1.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg1.frequency = 10000000;
	spi_ctg1.slave = 0;
	spi_cs1.gpio_dev = device_get_binding("GPIO_1");
	spi_cs1.gpio_pin = 4;
	spi_cs1.delay = 0;
	spi_cs1.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg1.cs = &spi_cs1;

	spi_ctg2.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg2.frequency = 10000000;
	spi_ctg2.slave = 0;
	spi_cs2.gpio_dev = device_get_binding("GPIO_1");
	spi_cs2.gpio_pin = 5;
	spi_cs2.delay = 0;
	spi_cs2.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg2.cs = &spi_cs2;

	spi_ctg3.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg3.frequency = 10000000;
	spi_ctg3.slave = 0;
	spi_cs3.gpio_dev = device_get_binding("GPIO_1");
	spi_cs3.gpio_pin = 6;
	spi_cs3.delay = 0;
	spi_cs3.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg3.cs = &spi_cs3;

	spi_ctg4.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg4.frequency = 10000000;
	spi_ctg4.slave = 0;
	spi_cs4.gpio_dev = device_get_binding("GPIO_1");
	spi_cs4.gpio_pin = 7;
	spi_cs4.delay = 0;
	spi_cs4.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg4.cs = &spi_cs4;

	spi_ctg5.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg5.frequency = 10000000;
	spi_ctg5.slave = 0;
	spi_cs5.gpio_dev = device_get_binding("GPIO_1");
	spi_cs5.gpio_pin = 8;
	spi_cs5.delay = 0;
	spi_cs5.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg5.cs = &spi_cs5;

	spi_ctg6.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg6.frequency = 10000000;
	spi_ctg6.slave = 0;
	spi_cs6.gpio_dev = device_get_binding("GPIO_1");
	spi_cs6.gpio_pin = 9;
	spi_cs6.delay = 0;
	spi_cs6.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg6.cs = &spi_cs6;

	spi_ctg7.operation =
			SPI_WORD_SET(8) | SPI_TRANSFER_MSB |SPI_OP_MODE_MASTER;
	spi_ctg7.frequency = 10000000;
	spi_ctg7.slave = 0;
	spi_cs7.gpio_dev = device_get_binding("GPIO_1");
	spi_cs7.gpio_pin = 10;
	spi_cs7.delay = 0;
	spi_cs7.gpio_dt_flags = GPIO_ACTIVE_LOW;
	spi_ctg7.cs = &spi_cs7;

	spi_group = k_malloc(sizeof(struct spi_sensor_grp) * 7);
	struct spi_config spi_ctg;
	sensor_avail = 0;
	for (int i=0; i < 7; i++) {
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
		spi_group[i].sensor_id = i;
		spi_group[i].spi_ctg = spi_ctg;
		spi_group[i].state = 0;
		sensor_avail |= (1 << i);
	}	
}

void hal_spi_read(struct spi_config spi_ctg, uint8_t reg_addr, uint8_t *value, uint8_t len) {

	uint8_t buffer_tx[2] = { reg_addr, 0 };
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

void hal_spi_write(struct spi_config spi_ctg, uint8_t reg_addr, uint8_t *value, uint8_t len) {
	uint8_t buffer_tx[1] = { reg_addr};
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

