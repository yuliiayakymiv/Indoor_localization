/*
 * Reflector role of a beacon (beacon build only, CONFIG_IL_BEACON_ID > 0).
 *
 * Based on Nordic's ras_reflector sample (../../cs_reflector). Differences:
 *  - the advertisement also carries the distances this beacon measured to the other
 *    beacons (see il_beacon.h), so the mobile node can build the coordinate plane;
 *  - after a disconnect the beacon advertises again instead of rebooting (a reboot would
 *    repeat the beacon-to-beacon measurement).
 */

#include <zephyr/kernel.h>

#if CONFIG_IL_BEACON_ID > 0

#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/sys/byteorder.h>
#include <bluetooth/services/ras.h>
#include <dk_buttons_and_leds.h>

#include "il_beacon.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(il_reflector, LOG_LEVEL_INF);

BUILD_ASSERT(IS_ENABLED(CONFIG_BT_PERIPHERAL) && IS_ENABLED(CONFIG_BT_RAS_RRSP),
	     "A beacon build needs -DEXTRA_CONF_FILE=beacon.conf");

#define CON_STATUS_LED DK_LED1
#define IL_BEACON_NAME "IL_B" STRINGIFY(CONFIG_IL_BEACON_ID)

/* Up to 3 other beacons */
#define MAX_DIST_ENTRIES_LEN (3 * IL_ADV_ENTRY_LEN)

static uint8_t mfg_data[sizeof(uint16_t) + MAX_DIST_ENTRIES_LEN];

static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_RANGING_SERVICE_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE, IL_BEACON_NAME, sizeof(IL_BEACON_NAME) - 1),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(uint16_t)),
};

static bool reflector_active;
/* The link to the mobile node; reference owned by the connection callbacks. */
static struct bt_conn *periph_conn;
/* Set when the mobile node has created the CS config on periph_conn. */
static struct bt_conn *config_done_for;

/* New connections for the main loop; every entry carries its own reference. */
K_MSGQ_DEFINE(new_conn_q, sizeof(struct bt_conn *), 2, sizeof(void *));

static void adv_work_handler(struct k_work *work)
{
	int err;

	if (!reflector_active || periph_conn != NULL) {
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err && err != -EALREADY) {
		LOG_ERR("Advertising failed to start (err %d)", err);
	}
}

static K_WORK_DEFINE(adv_work, adv_work_handler);

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info info;
	struct bt_conn *ref;

	if (bt_conn_get_info(conn, &info) != 0 || info.role != BT_CONN_ROLE_PERIPHERAL) {
		return; /* a link this beacon created itself (beacon-to-beacon measurement) */
	}

	if (err) {
		LOG_WRN("Incoming connection failed (err 0x%02X)", err);
		return;
	}

	LOG_INF("Mobile node (or another beacon) connected");

	config_done_for = NULL;
	periph_conn = bt_conn_ref(conn);

	ref = bt_conn_ref(conn);
	if (k_msgq_put(&new_conn_q, &ref, K_NO_WAIT) != 0) {
		bt_conn_unref(ref);
	}

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	if (conn != periph_conn) {
		return;
	}

	LOG_INF("Disconnected (reason 0x%02X)", reason);

	bt_conn_unref(periph_conn);
	periph_conn = NULL;

	dk_set_led_off(CON_STATUS_LED);
}

static void recycled_cb(void)
{
	/* A connection object is free again: advertising can be restarted. */
	k_work_submit(&adv_work);
}

static void config_create_cb(struct bt_conn *conn, uint8_t status,
			     struct bt_conn_le_cs_config *config)
{
	ARG_UNUSED(config);

	if (conn != periph_conn) {
		return;
	}

	if (status == BT_HCI_ERR_SUCCESS) {
		config_done_for = conn;
	} else {
		LOG_WRN("CS config creation failed. (HCI status 0x%02x)", status);
	}
}

BT_CONN_CB_DEFINE(reflector_conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.recycled = recycled_cb,
	.le_cs_config_complete = config_create_cb,
};

void il_reflector_run(const uint8_t *dist_entries, size_t len)
{
	int err;

	len = MIN(len, MAX_DIST_ENTRIES_LEN);
	sys_put_le16(IL_ADV_COMPANY_ID, mfg_data);
	if (len > 0) {
		memcpy(&mfg_data[sizeof(uint16_t)], dist_entries, len);
	}
	ad[ARRAY_SIZE(ad) - 1].data_len = sizeof(uint16_t) + len;

	LOG_INF("Beacon %s: advertising, waiting for the mobile node", IL_BEACON_NAME);

	dk_set_led_off(CON_STATUS_LED);
	reflector_active = true;
	k_work_submit(&adv_work);

	while (true) {
		struct bt_conn *conn;

		k_msgq_get(&new_conn_q, &conn, K_FOREVER);

		const struct bt_le_cs_set_default_settings_param default_settings = {
			.enable_initiator_role = false,
			.enable_reflector_role = true,
			.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
			.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
		};

		err = bt_le_cs_set_default_settings(conn, &default_settings);
		if (err) {
			LOG_ERR("Failed to configure default CS settings (err %d)", err);
		}

		/* The initiator creates the CS config; stop waiting if it disconnects. */
		while (periph_conn == conn && config_done_for != conn) {
			k_sleep(K_MSEC(50));
		}

		if (periph_conn == conn) {
			const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
				.config_id = 0,
				.max_procedure_len = 1000,
				.min_procedure_interval = 1,
				.max_procedure_interval = 100,
				.max_procedure_count = 0,
				.min_subevent_len = 10000,
				.max_subevent_len = 75000,
				.tone_antenna_config_selection =
					BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
				.phy = BT_LE_CS_PROCEDURE_PHY_2M,
				.tx_power_delta = 0x80,
				.preferred_peer_antenna =
					BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
				.snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
				.snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
			};

			err = bt_le_cs_set_procedure_parameters(conn, &procedure_params);
			if (err) {
				LOG_ERR("Failed to set procedure parameters (err %d)", err);
			}
		}

		bt_conn_unref(conn);
	}
}

#endif /* CONFIG_IL_BEACON_ID > 0 */
