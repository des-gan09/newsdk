#ifndef HAL_SPI_H
#define HAL_SPI_H

#include <device.h>
#include <drivers/spi.h>

struct spi_sensor_grp {
	uint8_t sensor_id;
    struct spi_config spi_ctg;
    uint8_t state;
};

extern const struct device *spi;

extern uint8_t spi_count;
extern struct spi_config spi_ctg1;
extern struct spi_config spi_ctg2;
extern struct spi_config spi_ctg3;
extern struct spi_config spi_ctg4;
extern struct spi_config spi_ctg5;
extern struct spi_config spi_ctg6;
extern struct spi_config spi_ctg7;

extern struct spi_cs_control spi_cs1;
extern struct spi_cs_control spi_cs2;
extern struct spi_cs_control spi_cs3;
extern struct spi_cs_control spi_cs4;
extern struct spi_cs_control spi_cs5;
extern struct spi_cs_control spi_cs6;
extern struct spi_cs_control spi_cs7;

extern struct spi_sensor_grp *spi_group;

extern void hal_spi_init(void);

extern void hal_spi_read(struct spi_config spi_ctg, 
                uint8_t reg_addr, uint8_t *value, uint8_t len);

extern void hal_spi_write(struct spi_config spi_ctg, 
                uint8_t reg_addr, uint8_t *value, uint8_t len);

#endif 