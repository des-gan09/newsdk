#include <soc.h>
#include <stdio.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <string.h>
#include <logging/log.h>
#include <drivers/spi.h> 
#include "peripheral/lis3mdl.h"
#include "hal/hal_spi.h"
#include <stdbool.h>

#define LOG_MODULE_NAME fittag
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

void main(void)
{
	hal_spi_init();

}