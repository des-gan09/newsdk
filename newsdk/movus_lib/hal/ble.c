#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>

#include <bluetooth/services/nus.h>
#include <bluetooth/services/hrs.h>
#include <bluetooth/hci_vs.h>
#include <bluetooth/conn.h>

#include <logging/log.h>
#include "ble.h"

#define LOG_MODULE_NAME ble
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

// Set advert at 1 second interval
#define BT_LE_ADV_CONN_SLOW BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, \
									BT_GAP_ADV_SLOW_INT_MIN, \
									BT_GAP_ADV_SLOW_INT_MAX, NULL) 

K_SEM_DEFINE(throughput_sem, 0, 1);
K_FIFO_DEFINE(fifo_transfer);

struct bt_conn *current_conn;
struct bt_conn *auth_conn;

volatile bool data_length_req;

const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      0x84, 0xaa, 0x60, 0x74, 0x52, 0x8a, 0x8b, 0x86,
		      0xd3, 0x4c, 0xb7, 0x1d, 0x1d, 0xdc, 0x53, 0x8d),
};

uint32_t sent_cnt = 0;
inline void bt_sent_cb(struct bt_conn *conn)
{
    sent_cnt++;
}
uint32_t get_sent_cnt(void)
{
    return sent_cnt;
}

void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			uint16_t len)
{
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));
	struct uart_data_t *buff;
	buff = k_malloc(sizeof(*buff));
	memcpy(buff->data,data,len);
	buff->len=len;

	k_fifo_put(&fifo_transfer, buff);
}

struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
	.sent = bt_sent_cb,
};

int connection_configuration_set(const struct bt_le_conn_param *conn_param) {
    int err;
	struct bt_conn_info info = {0};

	err = bt_conn_get_info(current_conn, &info);
	if (err) {
		LOG_ERR("Failed to get connection info %d", err);
		return err;
	}

	if (info.le.interval != conn_param->interval_max) {
		err = bt_conn_le_param_update(current_conn, conn_param);
		if (err) {
			LOG_ERR("Connection parameters update failed: %d",
				    err);
			return err;
		}

		LOG_INF("Connection parameters update pending");
	}

	return 0;
}

#define INTERVAL_MIN	    6	    /* x * 1.25 ms  */
#define INTERVAL_MAX	    6	    /* x * 1.25 ms  */
#define INTERVAL_TIMEOUT    42      /* x * 10ms     */

void params_update() {
    const struct bt_le_conn_param *conn_param =
            BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, 0, INTERVAL_TIMEOUT);

    connection_configuration_set(conn_param);
}

const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}

void connected(struct bt_conn *conn, uint8_t err) {
	char addr[BT_ADDR_LE_STR_LEN];
	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected");

	current_conn = bt_conn_ref(conn);
	// install_watchdog();
	// LOG_INF("Watchdog starting...");
	// pm_device_state_set(spi, PM_DEVICE_STATE_ACTIVE,NULL,NULL);
}

void disconnected(struct bt_conn *conn, uint8_t reason) {
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected:(reason %u)", reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	
	}

	// pm_device_state_set(spi, PM_DEVICE_STATE_LOW_POWER,NULL,NULL);  
}

bool ble_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	LOG_INF("Connection parameters update request received.");
	LOG_INF("Minimum interval: %d, Maximum interval: %d",
	       param->interval_min, param->interval_max);
	LOG_INF("Latency: %d, Timeout: %d", param->latency, param->timeout);

	return true;
}

void le_phy_updated(struct bt_conn *conn,
			   struct bt_conn_le_phy_info *param) {
	LOG_INF("LE PHY updated: TX PHY %s, RX PHY %s",
	       phy2str(param->tx_phy), phy2str(param->rx_phy));

	//k_sem_give(&throughput_sem);
}

void le_data_length_updated(struct bt_conn *conn,
				   struct bt_conn_le_data_len_info *info) {
	if (!data_length_req) {
		return;
	}

	LOG_INF("LE data len updated: TX (len: %d time: %d)"
	       " RX (len: %d time: %d)", info->tx_max_len,
	       info->tx_max_time, info->rx_max_len, info->rx_max_time);

	data_length_req = false;
	//k_sem_give(&throughput_sem);
}

void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout) {
	LOG_INF("Connection parameters updated.\n");
	LOG_INF("Interval: %d, latency: %d, timeout: %d",
	       interval, latency, timeout);

	k_sem_give(&throughput_sem);
}

struct bt_conn_cb conn_callbacks = {
	.connected    = connected,
	.disconnected = disconnected,
    .le_param_req = ble_param_req,
    .le_param_updated = le_param_updated,
    .le_phy_updated = le_phy_updated,
    .le_data_len_updated = le_data_length_updated
};

void ble_init(void) {
    int err;
    bt_conn_cb_register(&conn_callbacks);
    err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Failed to initialize UEnable BT (err: %d)", err);
		return;
	}
    LOG_INF("Bluetooth initialized");
    err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_SLOW, ad, ARRAY_SIZE(ad), sd,
				ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
	}
}
