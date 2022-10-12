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
#include <pm/pm.h>



#define LOG_MODULE_NAME fittag
LOG_MODULE_REGISTER(LOG_MODULE_NAME);
void main(void)
{

	hal_spi_init();	

	int count = 200;		
	sensor_mode(1); // turn on all sensors
	check_sensor();

	struct sensor_data_t *magnet;
	magnet = k_malloc(sizeof(struct sensor_data_t) * count);
	uint8_t count_temp,row = 0;
	int sampled = 0;

	while (sampled < count) {
		if (count_temp == 7) {
			count_temp = 0;
		}

		// if (row == sensor_avail) {
		// 	row = 0;
		// }
		// if (!((row >> count_temp) & 0x01)) {
		if (lis3mdl_status(spi_group[count_temp].spi_ctg)) {
			lis3mdl_get_xyz(spi_group[count_temp].spi_ctg, &(magnet[sampled]));				
			uint32_t temp_time = k_cyc_to_us_floor32(k_cycle_get_32());
			magnet[sampled].timestamp = temp_time; // get timestamp in microseconds
			magnet[sampled].sensor_id = spi_group[count_temp].sensor_id;
			sampled++;
			// row |= (1 << spi_group[count_temp].sensor_id);
			// printk("%x\n", row);
		}
		// }
		count_temp++;
	}
	// for (int i=0; i < count; i++) {
	// 	// if (count_temp == 7) {
	// 	// 	count_temp = 0;
	// 	// }
	// 	magnet[i].sensor_id = spi_group[count_temp].sensor_id;
	// 	if (spi_group[count_temp].state == 1) {
	// 		lis3mdl_get_xyz(spi_group[count_temp].spi_ctg, &(magnet[i]));				
	// 		uint32_t temp_time = k_cyc_to_us_floor32(k_cycle_get_32());
	// 		magnet[i].timestamp = temp_time; // get timestamp in microseconds 
	// 	}
	// 	count_temp++;
	// }
	for (int i=0; i < count; i++) {
		printk("%d %u %u %u %u\n",magnet[i].sensor_id, 
			magnet[i].x_value, 
			magnet[i].y_value, 
			magnet[i].z_value,  
			magnet[i].timestamp);
	}
	k_free(magnet);
	sensor_mode(0);
	pm_device_state_set(spi, PM_DEVICE_STATE_LOW_POWER,NULL,NULL);
}