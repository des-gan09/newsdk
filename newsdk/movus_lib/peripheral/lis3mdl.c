
#include "lis3mdl.h"
#include "hal/hal_spi.h"
#include <zephyr/types.h>
#include <stdbool.h>

void lis3mdl_data_rate(struct spi_config spi_ctg) {
    uint8_t ctrl_reg2 = 0b01100000; // 16 gauss
	hal_spi_write(spi_ctg, (LIS3MDL_CTRL_REG2 & ~LIS3MDL_SPI_READ), (uint8_t * ) &ctrl_reg2, sizeof(ctrl_reg2));
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
	printk("%x\n", (LIS3MDL_OUT_X_L | 1 << 6)| LIS3MDL_SPI_READ);
	sensor_data->x_value = (data[1] << 8 | data[0]);
	sensor_data->y_value = (data[3] << 8 | data[2]);
	sensor_data->z_value = (data[5] << 8 | data[4]);
}

float lis3mdl_convert(int16_t lsb) {
	return (((float) lsb) / 1711.0f);
}

void lis3mdl_poweroff(struct spi_config spi_ctg) {
	// uint8_t ctrl_reg2 = 0x08; // reboot sensor
	uint8_t ctrl_reg = 0b00000011; // power down
	// lis3mdl_spi_write(spi_ctg, LIS3MDL_CTRL_REG2, (uint8_t * ) &ctrl_reg2, sizeof(ctrl_reg2));
	hal_spi_write(spi_ctg, (LIS3MDL_CTRL_REG3 & ~LIS3MDL_SPI_READ), (uint8_t * ) &ctrl_reg, sizeof(ctrl_reg));
}
