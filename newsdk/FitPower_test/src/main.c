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
#include "ble_transfer.h"
#include "sample.h"
#include "hal/wdt.h"
#include "hal/radio.h"
#include <pm/pm.h>

#define LOG_MODULE_NAME fittag
LOG_MODULE_REGISTER(LOG_MODULE_NAME);
void main(void)
{
	install_watchdog();
	hal_spi_init();
#if CONFIG_FITPOWER_SERIAL_SAMPLING
	continue;
#else
	ble_init();
	// tx_pwr_init();
#endif
	// lis3mdl_validation();
	sensor_mode(1);
	sensoroff(0);
	pm_device_state_set(spi, PM_DEVICE_STATE_LOW_POWER,NULL,NULL);
}