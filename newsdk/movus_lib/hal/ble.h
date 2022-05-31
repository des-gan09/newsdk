#ifndef BLE_H
#define BLE_H

#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>

#include <bluetooth/services/hrs.h>
#include <bluetooth/hci_vs.h>
#include <bluetooth/conn.h>

static struct bt_conn_cb conn_callbacks = {
	.connected    = connected,
	.disconnected = disconnected,
    .le_param_req = le_param_req,
    .le_param_updated = le_param_updated,
    .le_phy_updated = le_phy_updated,
    .le_data_len_updated = le_data_length_updated
};

static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}

static int connection_configuration_set(const struct bt_le_conn_param *conn_param);
extern void params_update();
static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);
static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			uint16_t len);
static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param);
static void le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param);
static void le_data_length_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info);
static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout);
static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			uint16_t len);
static inline uint32_t get_sent_cnt(void);      