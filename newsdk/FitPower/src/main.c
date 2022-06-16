#include <soc.h>
#include <stdio.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <string.h>
#include <logging/log.h>
#include <drivers/spi.h> 
#include "peripheral/lis3mdl.h"
#include "hal/hal_spi.h"
#include "hal/ble.h"
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
// #include "ble_transfer.h"
// #include "sample.h"



#define LOG_MODULE_NAME fittag
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

void sensor(uint8_t mode) {

	struct spi_config spi_ctg;

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
		switch (mode)
		{
		case 0:
			lis3mdl_poweroff(spi_ctg);
			k_msleep(20);
			break;
		case 1:
			lis3mdl_init(spi_ctg);
			k_msleep(20);
			break;
		// case 2:
		// 	lis3mdl_powerlow(spi_ctg);
		// 	k_msleep(20);
		default:
			break;
		}
	}
}

void main(void)
{
	hal_spi_init();
	ble_init();
	sensor(1);
	sensor(0);
	lis3mdl_validation();
}