#include <soc.h>
#include <stdio.h>
#include <zephyr.h>
#include <pm/pm.h>

// #define LOG_MODULE_NAME fittag
// LOG_MODULE_REGISTER(LOG_MODULE_NAME);
void main(void)
{
// 	install_watchdog();
	// hal_spi_init();
// #if CONFIG_FITPOWER_SERIAL_SAMPLING
// 	continue;
// #else
// 	ble_init();
// 	// tx_pwr_init();
// #endif
// 	// lis3mdl_validation();
	// sensoroff();
// 	pm_device_state_set(spi, PM_DEVICE_STATE_LOW_POWER,NULL,NULL);
	pm_constraint_set(PM_STATE_SOFT_OFF);
	int i = 3;

	pm_power_state_force((struct pm_state_info){PM_STATE_SOFT_OFF,0,0});

}