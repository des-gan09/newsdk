#ifndef BLE_H
#define BLE_H

#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>

#include <bluetooth/services/hrs.h>
#include <bluetooth/hci_vs.h>
#include <bluetooth/conn.h>

#include <zephyr.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[50];
	uint16_t len;
};

void bt_sent_cb(struct bt_conn *conn);
void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			uint16_t len);

int connection_configuration_set(const struct bt_le_conn_param *conn_param);
void params_update();
void connected(struct bt_conn *conn, uint8_t err);
void disconnected(struct bt_conn *conn, uint8_t reason);

bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param);
void le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param);
void le_data_length_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info);
void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout);
void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			uint16_t len);
uint32_t get_sent_cnt(void);      

extern void ble_init(void);

#endif