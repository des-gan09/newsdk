
#include "lis3mdl.h"
#include "hal/hal_spi.h"
#include <zephyr/types.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <logging/log.h>


#define LOG_MODULE_NAME lis3mdl
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

struct spi_config *spi_ctgx;

void lis3mdl_data_rate(struct spi_config spi_ctg) {
    uint8_t ctrl_reg1 = 0b00000010; // FAST MODE
	uint8_t ctrl_reg4 = 0b00000000;
	hal_spi_write(spi_ctg, (LIS3MDL_CTRL_REG1 & ~LIS3MDL_SPI_READ), (uint8_t * ) &ctrl_reg1, sizeof(ctrl_reg1));
	hal_spi_write(spi_ctg, (LIS3MDL_CTRL_REG4 & ~LIS3MDL_SPI_READ), (uint8_t * ) &ctrl_reg4, sizeof(ctrl_reg4));
}
void lis3mdl_operating_mode(struct spi_config spi_ctg) {
    uint8_t ctrl_reg3 = 0b00000000; // Continuous conversion mode
	hal_spi_write(spi_ctg, (LIS3MDL_CTRL_REG3 & ~LIS3MDL_SPI_READ), (uint8_t * ) &ctrl_reg3, sizeof(ctrl_reg3));
}
	
void lis3mdl_block_data_update_set(struct spi_config spi_ctg) {
	uint8_t ctrl_reg5 = 0x40;
	hal_spi_write(spi_ctg, (LIS3MDL_CTRL_REG5 & ~LIS3MDL_SPI_READ), (uint8_t * ) &ctrl_reg5, sizeof(ctrl_reg5));
}

void lis3mdl_full_scale_set(struct spi_config spi_ctg) {

	uint8_t ctrl_reg2 = 0b01100000; // 16 gauss
	hal_spi_write(spi_ctg, (LIS3MDL_CTRL_REG2 & ~LIS3MDL_SPI_READ), (uint8_t * ) &ctrl_reg2, sizeof(ctrl_reg2));
}

bool lis3mdl_present(struct spi_config spi_ctg) {
	uint8_t whoamI;

	hal_spi_read(spi_ctg, LIS3MDL_WHOAMI_REG | LIS3MDL_SPI_READ, (uint8_t * ) &whoamI, sizeof(whoamI));

	return whoamI == LIS3MDL_ID;
}

void lis3mdl_init(struct spi_config spi_ctg) {

	lis3mdl_block_data_update_set(spi_ctg); // No idea what this does
	lis3mdl_data_rate(spi_ctg);
	lis3mdl_full_scale_set(spi_ctg);
	lis3mdl_operating_mode(spi_ctg);
}

uint8_t lis3mdl_status(struct spi_config spi_ctg) {
	uint8_t ret;
	hal_spi_read(spi_ctg, LIS3MDL_STATUS_REG, (uint8_t * ) &ret, sizeof(ret));
	// i2c_reg_read_byte(i2c_device, LIS3MDL_I2C_ADDR, LIS3MDL_STATUS_REG, &ret);
	// printk("%x\n", ret);
	return ((ret >> 3) & 0x01);
}

void lis3mdl_get_x (struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
    uint8_t data[2];

	hal_spi_read(spi_ctg, LIS3MDL_OUT_X_L | LIS3MDL_SPI_READ, (uint8_t * ) &data[0], sizeof(data[0]));
	hal_spi_read(spi_ctg, LIS3MDL_OUT_X_H | LIS3MDL_SPI_READ, (uint8_t * ) &data[1], sizeof(data[1]));

	sensor_data->x_value = (data[1] << 8 | data[0]);
}

void lis3mdl_get_y(struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
	uint8_t data[2];

	hal_spi_read(spi_ctg, LIS3MDL_OUT_Y_L | LIS3MDL_SPI_READ , (uint8_t * ) &data[0], sizeof(data[0]));
	hal_spi_read(spi_ctg, LIS3MDL_OUT_Y_H | LIS3MDL_SPI_READ , (uint8_t * ) &data[1], sizeof(data[1]));

	sensor_data->y_value = (data[1] << 8 | data[0]);
}

void lis3mdl_get_z(struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
	uint8_t data[2];

	hal_spi_read(spi_ctg, LIS3MDL_OUT_Z_L | LIS3MDL_SPI_READ, (uint8_t * ) &data[0], sizeof(data[0]));
	hal_spi_read(spi_ctg, LIS3MDL_OUT_Z_H | LIS3MDL_SPI_READ, (uint8_t * ) &data[1], sizeof(data[1]));

	sensor_data->z_value = (data[1] << 8 | data[0]);
}

void lis3mdl_get_xyz(struct spi_config spi_ctg, struct sensor_data_t *sensor_data) {
	uint8_t status;
	uint8_t data[6];

	hal_spi_read(spi_ctg, (LIS3MDL_OUT_X_L | 1 << 6)| LIS3MDL_SPI_READ, (uint8_t * ) &data, sizeof(data));
	sensor_data->x_value = (data[1] << 8 | data[0]);
	sensor_data->y_value = (data[3] << 8 | data[2]);
	sensor_data->z_value = (data[5] << 8 | data[4]);
}

float lis3mdl_convert(int16_t lsb) {
	return (((float) lsb) / 1711.0f);
}

float lis3mdl_convert12(int16_t lsb) {
	return (((float) lsb) / 2281.0f);
}

void lis3mdl_poweroff(struct spi_config spi_ctg) {
	// uint8_t ctrl_reg2 = 0x08; // reboot sensor
	uint8_t ctrl_reg = 0b00000011; // power down
	// lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG2, (uint8_t * ) &ctrl_reg2, sizeof(ctrl_reg2));
	hal_spi_write(spi_ctg, (LIS3MDL_CTRL_REG3 & ~LIS3MDL_SPI_READ), (uint8_t * ) &ctrl_reg, sizeof(ctrl_reg));
}

void lis3mdl_selftest_settings(struct spi_config spi_ctg) {
	uint8_t ctrl_reg1 = 0x1c;
	uint8_t ctrl_reg2 = 0x40;
	uint8_t ctrl_reg3 = 0x00;

	hal_spi_write(spi_ctg, (LIS3MDL_CTRL_REG1 & ~LIS3MDL_SPI_READ), 
			(uint8_t * ) &ctrl_reg1, sizeof(ctrl_reg1));
	hal_spi_write(spi_ctg, (LIS3MDL_CTRL_REG2 & ~LIS3MDL_SPI_READ),
			(uint8_t * ) &ctrl_reg2, sizeof(ctrl_reg2));
	k_msleep(20);
	hal_spi_write(spi_ctg, (LIS3MDL_CTRL_REG3 & ~LIS3MDL_SPI_READ),
			(uint8_t * ) &ctrl_reg3, sizeof(ctrl_reg3));

	k_msleep(20); // wait for stable output
}

bool lis3mdl_selftest(struct spi_config spi_ctg) {
	struct sensor_data_t junk;
	junk.sensor_id = 0;
	junk.timestamp = 0;
	struct sensor_data_t *nost;
	nost = k_malloc(sizeof(struct sensor_data_t) * 5);
	lis3mdl_selftest_settings(spi_ctg);
	do {
		lis3mdl_get_xyz(spi_ctg, &junk);
	} while (!lis3mdl_status(spi_ctg));

	for (int i=0; i < 5; i++) {
		nost[i].sensor_id = 0;
		nost[i].timestamp = 0;
		do {
			lis3mdl_get_xyz(spi_ctg, &(nost[i]));
		}while (!lis3mdl_status(spi_ctg));
	}
    
	float nost_x = 0.0;
	float nost_y = 0.0;
	float nost_z = 0.0;
	
	for (int i=0; i < 5; i++) {
		nost_x = nost_x + lis3mdl_convert12(nost[i].x_value);
		nost_y = nost_y + lis3mdl_convert12(nost[i].y_value);
		nost_z = nost_z + lis3mdl_convert12(nost[i].z_value);
	}

	nost_x = nost_x / 5.0;
	nost_y = nost_y / 5.0;
	nost_z = nost_z / 5.0;
	// turn on self-test
	uint8_t ctrl_reg1 = 0x1d;
	hal_spi_write(spi_ctg, (LIS3MDL_CTRL_REG1 & ~LIS3MDL_SPI_READ), 
			(uint8_t * ) &ctrl_reg1, sizeof(ctrl_reg1));
	k_msleep(60);

	do {
		lis3mdl_get_xyz(spi_ctg, &junk);
	} while (!lis3mdl_status(spi_ctg));

	for (int i=0; i < 5; i++) {
		nost[i].sensor_id = 0;
		nost[i].timestamp = 0;
		do {
			lis3mdl_get_xyz(spi_ctg, &(nost[i]));
		}while (!lis3mdl_status(spi_ctg));
	}

	float st_x = 0.0;
	float st_y = 0.0;
	float st_z = 0.0;

	for (int i=0; i < 5; i++) {
		st_x = st_x + lis3mdl_convert12(nost[i].x_value);
		st_y = st_y + lis3mdl_convert12(nost[i].y_value);
		st_z = st_z + lis3mdl_convert12(nost[i].z_value);
	}
	st_x = st_x / 5.0;
	st_y = st_y / 5.0;
	st_z = st_z / 5.0;

	float x = fabs(st_x - nost_x);
	float y = fabs(st_y - nost_y);
	float z = fabs(st_z - nost_z);
    
	// printk("X:%f\n", x);

    ctrl_reg1 = 0x1c;
	hal_spi_write(spi_ctg, (LIS3MDL_CTRL_REG1 & ~LIS3MDL_SPI_READ), 
			(uint8_t * ) &ctrl_reg1, sizeof(ctrl_reg1));
	lis3mdl_poweroff(spi_ctg);
	if (x >= MIN_ST_X && x <= MAX_ST_X) {
		if (y >= MIN_ST_Y && y <= MAX_ST_Y){
			if (z >= MIN_ST_Z && z <= MAX_ST_Z){
				return true;
			}
		}
	}

	free(nost);
	return false;
}

 void lis3mdl_validation() {
	spi_count = 0;
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

		if(lis3mdl_selftest(spi_ctg)) {
			LOG_INF("SENSOR %d OK.", i + 1);
			spi_count++;
		} else {
			LOG_WRN("SENSOR %d FAILED.", i + 1);
		}

	}
	spi_ctgx = k_malloc(sizeof(struct spi_config) * spi_count);
	uint8_t temp_count = 0;
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

		if(lis3mdl_selftest(spi_ctg)) {
			// LOG_INF("Sensor %d %d added.", spi_ctg.cs->gpio_pin, i);
			spi_ctgx[temp_count] = spi_ctg;
			temp_count++;
		}
	}
 }

void sensoroff() {
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
		lis3mdl_poweroff(spi_ctg);
	}
}
