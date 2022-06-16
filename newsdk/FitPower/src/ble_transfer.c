// BLE transfer thread
#include "ble_transfer.h"
#include "hal/ble.h"

void ble_transfer_thread() {

}

K_THREAD_DEFINE(ble_transfer_thread_id, STACKSIZE, ble_transfer_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);

