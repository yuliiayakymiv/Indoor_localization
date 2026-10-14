#include "ble_scanner.h"
#include "beacon_format.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

struct beacon_stats {
	int32_t rssi_sum;
	uint16_t count;
	uint32_t last_report_ms;
};

static ble_scanner_cb_t user_cb;
static uint32_t interval_ms;
static struct beacon_stats stats[IL_MAX_BEACONS + 1];

/* Returns false (stop parsing) once our manufacturer data is found */
static bool parse_ad(struct bt_data *data, void *user_data)
{
	int *beacon_id = user_data;

	if (data->type == BT_DATA_MANUFACTURER_DATA &&
	    data->data_len == IL_MFG_DATA_LEN &&
	    data->data[0] == IL_COMPANY_ID_LO &&
	    data->data[1] == IL_COMPANY_ID_HI &&
	    data->data[2] == IL_MARKER_0 &&
	    data->data[3] == IL_MARKER_1) {
		*beacon_id = data->data[4];
		return false;
	}
	return true;
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *ad)
{
	int beacon_id = -1;

	ARG_UNUSED(addr);
	ARG_UNUSED(adv_type);

	bt_data_parse(ad, parse_ad, &beacon_id);
	if (beacon_id < 0 || beacon_id > IL_MAX_BEACONS) {
		return; /* not our beacon */
	}

	struct beacon_stats *s = &stats[beacon_id];
	uint32_t now = k_uptime_get_32();

	s->rssi_sum += rssi;
	s->count++;

	if (now - s->last_report_ms >= interval_ms) {
		int8_t avg = (int8_t)(s->rssi_sum / s->count);

		s->rssi_sum = 0;
		s->count = 0;
		s->last_report_ms = now;

		if (user_cb) {
			user_cb((uint8_t)beacon_id, avg, now);
		}
	}
}

int ble_scanner_start(ble_scanner_cb_t cb, uint32_t report_interval_ms)
{
	/* Passive scan WITHOUT duplicate filter: we need every packet for RSSI */
	static const struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_PASSIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};
	int err;

	user_cb = cb;
	interval_ms = report_interval_ms;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (%d)\n", err);
		return err;
	}

	err = bt_le_scan_start(&scan_param, scan_cb);
	if (err) {
		printk("Scanning failed to start (%d)\n", err);
		return err;
	}

	return 0;
}

int ble_scanner_format_line(char *buf, size_t size,
			    uint8_t beacon_id, int8_t rssi, uint32_t time_ms)
{
	return snprintk(buf, size, "BEACON_%02u,%d,%u\n",
			beacon_id, rssi, time_ms);
}
