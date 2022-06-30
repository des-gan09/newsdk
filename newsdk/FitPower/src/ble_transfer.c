// BLE transfer thread
#include "ble_transfer.h"
#include "sample.h"
#include "peripheral/lis3mdl.h"
#include "hal/ble.h"

void ble_transfer_thread() {
	 
	struct sensor_data_t data;
	// struct k_sem stream_sem;
	// k_sem_take(&stream_sem, K_FOREVER);
	// for(;;) {
	// 	// struct stream_data_t *stream = k_fifo_get(&fifo_stream,
	// 	// 					K_FOREVER);
	// 	// data =  stream->data;
	// 	// printk("%d %u %u %u %u\n", data.sensor_id, 
	// 	// 		data.x_value,
	// 	// 		data.y_value,
	// 	// 		data.z_value,
	// 	// 		data.timestamp);
	// }
}

K_THREAD_DEFINE(ble_transfer_thread_id, STACKSIZE, ble_transfer_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);

