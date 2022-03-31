/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample with Power management
 */
#include <stdio.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <soc.h>
#include <device.h>
#include <pm/pm.h>
#include <hal/nrf_gpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#define CONSOLE_LABEL DT_LABEL(DT_CHOSEN(zephyr_console))

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define WAITTIME	3000		//WRC 3 seconds

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

static K_SEM_DEFINE(ble_init_ok, 0, 1);

#define BT_LE_ADV_CONN_SLOW BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, \
                                      0X0C80, \
                                       0X0F00, NULL)  
static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Connected\n");

	current_conn = bt_conn_ref(conn);

	
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected:(reason %u)", reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	
	}
}


static struct bt_conn_cb conn_callbacks = {
	.connected    = connected,
	.disconnected = disconnected,
      

};



static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	uint8_t buffer[100];
	int16_t buff_len;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	printk("Received data \n");
	memcpy(buffer,data,len);
	buff_len=len;
	if (bt_nus_send(NULL, buffer, buff_len)) {
			printk("Failed to send data over BLE connection\n");
		}

}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void main(void)
{
	int rc;
	int err = 0;
    const struct device *cons;
    cons=device_get_binding(CONSOLE_LABEL);
    printk("CONS=%d\n",cons);

	/* Prevent deep sleep (system off) from being entered */
    pm_constraint_set(PM_STATE_SOFT_OFF);
        
	bt_conn_cb_register(&conn_callbacks);

	err = bt_enable(NULL);
	
	if (err) {
 		printk("Failed to initialize UEnable BT (err: %d)\n", err);
		return;
	}
	
	printk("Bluetooth initialized\n");
    
	k_sem_give(&ble_init_ok);

	err = bt_nus_init(&nus_cb);
	if (err) {
 		printk("Failed to initialize UART service (err: %d)\n", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_SLOW, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
	}

	printk("Starting Nordic UART service example\n");
	printk("Bluetooth initialized\n");
	printk("TURNING OFF CONSOLE\n");
	rc=pm_device_state_set(cons, PM_DEVICE_STATE_LOW_POWER,NULL,NULL); 
	k_sleep(K_MSEC(WAITTIME));
	k_sleep(K_MSEC(WAITTIME));  //Added 2 waititmes because advertising is set to 2 sec and want to capture it advertising in PPK-2

	/* Before we disabled entry to deep sleep. Here we need to override
	 * that, then force a sleep so that the deep sleep takes effect.
	 */
	pm_power_state_force((struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});
}
