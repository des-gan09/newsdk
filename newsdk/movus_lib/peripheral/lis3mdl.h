#ifndef LIS3MDL_H
#define LIS3MDL_H

#include "hal/hal_spi.h"

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

#define MIN_ST_X 1.0
#define MIN_ST_Y 1.0
#define MIN_ST_Z 0.1
#define MAX_ST_X 3.0
#define MAX_ST_Y 3.0
#define MAX_ST_Z 1.0

#define NUM_SENSOR 7

struct sensor_data_t {
	uint8_t sensor_id;
	uint16_t x_value;
	uint16_t y_value;
	uint16_t z_value;
	uint32_t timestamp;
};

void lis3mdl_data_rate(struct spi_config spi_ctg);
void lis3mdl_full_scale_set(struct spi_config spi_ctg);
void lis3mdl_operating_mode(struct spi_config spi_ctg);
void lis3mdl_block_data_update_set(struct spi_config spi_ctg);
extern bool lis3mdl_present(struct spi_config spi_ctg);
extern void lis3mdl_init(struct spi_config spi_ctg);
extern uint8_t lis3mdl_status(struct spi_config spi_ctg);
extern void lis3mdl_get_x(struct spi_config spi_ctg, struct sensor_data_t *sensor_data);
extern void lis3mdl_get_y(struct spi_config spi_ctg, struct sensor_data_t *sensor_data);
extern void lis3mdl_get_z(struct spi_config spi_ctg, struct sensor_data_t *sensor_data);
extern void lis3mdl_get_xyz(struct spi_config spi_ctg, struct sensor_data_t *sensor_data);
extern void lis3mdl_poweroff(struct spi_config spi_ctg);
extern float lis3mdl_convert(int16_t lsb);
extern float lis3mdl_convert12(int16_t lsb);
extern void lis3mdl_selftest_settings(struct spi_config spi_ctg);
extern bool lis3mdl_selftest(struct spi_config spi_ctg);
extern void lis3mdl_validation(); 

#endif