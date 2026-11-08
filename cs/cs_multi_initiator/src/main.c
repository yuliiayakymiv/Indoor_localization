/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Multi-beacon Channel Sounding initiator (Indoor localization, NRF-02 + NRF-04)
 *
 *  Based on Nordic's ras_initiator sample (NCS 3.4.2) and our single-beacon version in
 *  ../cs_initiator (calibration constants are copied from there).
 *
 *  What it does:
 *   1. Scans for beacons that advertise the name "IL_B<id>" (id = 1..CONFIG_IL_NUM_BEACONS)
 *      and keeps one connection to each of them.
 *   2. Ranges to the beacons ONE AT A TIME (round-robin): enable CS on beacon i, collect
 *      CONFIG_IL_SAMPLES_PER_TURN distance estimates (or time out), disable CS, go to i+1.
 *      Nordic recommends sequential CS sessions; concurrent CS on several links is not
 *      reliably scheduled by the controller.
 *   3. After every turn prints one measurement line on the UART (format: see README):
 *        $RNG,<seq>,<t_ms>,<beacon_id>,<dist_mm>,<status>*<XX>
 *
 *  4. After every round computes X/Y with 2D trilateration (../../trilateration, Sofia's
 *     code) and prints:
 *        $POS,<seq>,<t_ms>,<x_mm>,<y_mm>,<z_mm>,<status>*<XX>
 *     Beacon coordinates: beacon_pos_mm[] below.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/ras.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/cs_de.h>

#include "trilat_bridge.h"

#include <dk_buttons_and_leds.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

#define CON_STATUS_LED DK_LED1

#define CS_CONFIG_ID 0

#if defined(CONFIG_SAMPLE_RAS_INITIATOR_STEP_MODE_2_SUB_MODE_1)
#define CS_CONFIG_MODE BT_CONN_LE_CS_MAIN_MODE_2_SUB_MODE_1
#elif defined(CONFIG_SAMPLE_RAS_INITIATOR_STEP_MODE_2)
#define CS_CONFIG_MODE BT_CONN_LE_CS_MAIN_MODE_2_NO_SUB_MODE
#elif defined(CONFIG_SAMPLE_RAS_INITIATOR_STEP_MODE_1)
#define CS_CONFIG_MODE BT_CONN_LE_CS_MAIN_MODE_1_NO_SUB_MODE
#elif defined(CONFIG_SAMPLE_RAS_INITIATOR_STEP_MODE_3)
#define CS_CONFIG_MODE BT_CONN_LE_CS_MAIN_MODE_3_NO_SUB_MODE
#else
BUILD_ASSERT(false, "Invalid ranging mode");
#endif

#define NUM_MODE_0_STEPS       3
#define PROCEDURE_COUNTER_NONE (-1)
#define MAX_AP                 (CONFIG_BT_RAS_MAX_ANTENNA_PATHS)

BUILD_ASSERT(MAX_AP == 1, "This application uses antenna path 0 only");

#define LOCAL_PROCEDURE_MEM                                                                        \
	((BT_RAS_MAX_STEPS_PER_PROCEDURE * sizeof(struct bt_le_cs_subevent_step)) +                \
	 (BT_RAS_MAX_STEPS_PER_PROCEDURE * BT_RAS_MAX_STEP_DATA_LEN))

/* ---- Multi-beacon settings ---- */
#define NUM_BEACONS        CONFIG_IL_NUM_BEACONS
#define SAMPLES_PER_TURN   CONFIG_IL_SAMPLES_PER_TURN
#define TURN_TIMEOUT_MS    CONFIG_IL_TURN_TIMEOUT_MS
#define BEACON_NAME_PREFIX "IL_B"
/* Median over the last N estimates of one beacon (spans ~2 turns). */
#define DE_SLIDING_WINDOW_SIZE (5)
/* A distance older than this is reported as "stale" */
#define DISTANCE_MAX_AGE_MS (3000)
/* Timeout of each step of the per-beacon setup (security, discovery, CS config...) */
#define SETUP_STEP_TIMEOUT_MS (5000)

/* ---- Beacon coordinates (SYS-04) ---------------------------------------------------------
 * Position of every beacon in the room coordinate system, in MILLIMETERS: {x, y, z}.
 * Row 1 = beacon IL_B1, row 2 = IL_B2, ... Replace the TEMPORARY values below with the real
 * ones from SYS-04, then rebuild and reflash only this (mobile) board.
 * z = height of the beacon antenna above the floor.
 */
static const int32_t beacon_pos_mm[][3] = {
	{    0,    0, 0 }, /* IL_B1  TEMPORARY */
	{ 3000,    0, 0 }, /* IL_B2  TEMPORARY */
	{    0, 3000, 0 }, /* IL_B3  TEMPORARY */
	{ 3000, 3000, 0 }, /* IL_B4  TEMPORARY, used only with CONFIG_IL_NUM_BEACONS=4 */
};

/* Height of the mobile board antenna above the floor, mm (on the toy car). */
#define MOBILE_Z_MM (0)

/* ---- Distance calibration (copied from ../cs_initiator, see the comment there) ----
 * Actual_distance = (Raw_measurement - OFFSET) / SLOPE
 * Fitted on two XIAO nRF54L15 boards; re-measure if the hardware changes.
 */
#define IFFT_SLOPE           (1.30f)
#define IFFT_OFFSET_M        (0.70f)
#define IFFT_WEIGHT          (0.50f)

#define PHASE_SLOPE_SLOPE    (1.786f)
#define PHASE_SLOPE_OFFSET_M (1.307f)
#define PHASE_WEIGHT         (0.00f)

#define RTT_SLOPE            (2.35f)
#define RTT_OFFSET_M         (4.05f)
#define RTT_WEIGHT           (0.50f)

#define CHANNEL_INDEX_OFFSET (2)
#define TONE_QI_OK_TONE_COUNT_THRESHOLD (15)

/* ---- NRF-02: status codes of a measurement line ---- */
enum il_status {
	IL_STATUS_OK = 0,            /* fresh, valid distance */
	IL_STATUS_NO_ESTIMATE = 1,   /* CS ran, but no usable estimate (bad quality) */
	IL_STATUS_TIMEOUT = 2,       /* no CS result from this beacon during its turn */
	IL_STATUS_NOT_CONNECTED = 3, /* beacon is not connected */
};

struct distance_estimate_buffer {
	cs_de_dist_estimates_t estimates[DE_SLIDING_WINDOW_SIZE];
	uint8_t num_valid;
	uint8_t index;
};

struct beacon {
	uint8_t id;                 /* 1..NUM_BEACONS, from the advertised name */
	struct bt_conn *conn;       /* NULL when not connected */
	bool ready;                 /* CS fully set up, can be ranged */
	uint32_t ras_feature_bits;
	struct bt_conn_le_cs_config cs_config;
	struct distance_estimate_buffer de_buf;
	uint8_t new_estimates;      /* estimates received in the current turn */

	/* latest result, used by the trilateration step */
	float distance_m;
	enum il_status status;
	uint32_t updated_ms;
};

static struct beacon beacons[NUM_BEACONS];

BUILD_ASSERT(ARRAY_SIZE(beacon_pos_mm) >= NUM_BEACONS,
	     "Add coordinates for every beacon to beacon_pos_mm");

/* Beacon being set up (connection phase) and beacon being ranged (CS phase).
 * Only one of each at a time, so the step buffers below can be shared.
 */
static struct beacon *setup_beacon;
static struct beacon *active_beacon;
static struct beacon *connecting_beacon;

static K_SEM_DEFINE(sem_remote_capabilities_obtained, 0, 1);
static K_SEM_DEFINE(sem_config_created, 0, 1);
static K_SEM_DEFINE(sem_cs_security_enabled, 0, 1);
static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_discovery_done, 0, 1);
static K_SEM_DEFINE(sem_mtu_exchange_done, 0, 1);
static K_SEM_DEFINE(sem_security, 0, 1);
static K_SEM_DEFINE(sem_ras_features, 0, 1);
static K_SEM_DEFINE(sem_procedure_enable, 0, 1);
static K_SEM_DEFINE(sem_local_steps, 1, 1);
static K_SEM_DEFINE(sem_distance_estimate_updated, 0, 1);

static K_MUTEX_DEFINE(distance_estimate_buffer_mutex);

NET_BUF_SIMPLE_DEFINE_STATIC(latest_local_steps, LOCAL_PROCEDURE_MEM);
NET_BUF_SIMPLE_DEFINE_STATIC(latest_peer_steps, BT_RAS_PROCEDURE_MEM);
static int32_t most_recent_local_ranging_counter = PROCEDURE_COUNTER_NONE;
static int32_t dropped_ranging_counter = PROCEDURE_COUNTER_NONE;

static uint16_t m_n_iqs[CONFIG_BT_RAS_MAX_ANTENNA_PATHS][CS_DE_NUM_CHANNELS];
static cs_de_report_t m_cs_de_report;

static uint8_t setup_status; /* HCI status of the last setup callback */
static bool procedure_enabled_state;

static struct beacon *beacon_by_conn(struct bt_conn *conn)
{
	for (int i = 0; i < NUM_BEACONS; i++) {
		if (beacons[i].conn == conn) {
			return &beacons[i];
		}
	}
	return NULL;
}

static int wait_sem(struct k_sem *sem, uint32_t timeout_ms)
{
	return k_sem_take(sem, K_MSEC(timeout_ms));
}

static void store_distance_estimates_in_buffer(cs_de_dist_estimates_t *p_estimates,
					       struct distance_estimate_buffer *buffer)
{
	int lock_state = k_mutex_lock(&distance_estimate_buffer_mutex, K_FOREVER);

	__ASSERT_NO_MSG(lock_state == 0);

	memcpy(&buffer->estimates[buffer->index], p_estimates, sizeof(cs_de_dist_estimates_t));

	buffer->index = (buffer->index + 1) % DE_SLIDING_WINDOW_SIZE;

	if (buffer->num_valid < DE_SLIDING_WINDOW_SIZE) {
		buffer->num_valid++;
	}

	k_mutex_unlock(&distance_estimate_buffer_mutex);
}

static int float_cmp(const void *a, const void *b)
{
	float fa = *(const float *)a;
	float fb = *(const float *)b;

	return (fa > fb) - (fa < fb);
}

static float median_inplace(int count, float *values)
{
	if (count == 0) {
		return NAN;
	}

	qsort(values, count, sizeof(float), float_cmp);

	if (count % 2 == 0) {
		return (values[count/2] + values[count/2 - 1]) / 2;
	} else {
		return values[count/2];
	}
}
static bool m_is_tone_quality_ok(uint16_t num_iqs[CS_DE_NUM_CHANNELS], uint8_t channel_map[10])
{
	uint8_t ok_tones_count = 0;

	for (uint8_t i = 0; i < CS_DE_NUM_CHANNELS; ++i) {
		if (BT_LE_CS_CHANNEL_BIT_GET(channel_map, i + CHANNEL_INDEX_OFFSET) &&
		    num_iqs[i] >= 1) {
			ok_tones_count += 1;
		}
	}
	return (ok_tones_count >= TONE_QI_OK_TONE_COUNT_THRESHOLD);
}

static void cumulate_mean(float *avg, float new_value, uint16_t *N)
{
	float a = 1.0f / (*N);
	float b = 1.0f - a;
	*avg = a * new_value + b * (*avg);
}

static void extract_pcts(cs_de_report_t *p_report, uint8_t channel_index,
			 uint8_t antenna_permutation_index,
			 struct bt_hci_le_cs_step_data_tone_info *local_tone_info,
			 struct bt_hci_le_cs_step_data_tone_info *remote_tone_info)
{

	for (uint8_t tone_index = 0; tone_index < p_report->n_ap; tone_index++) {
		int antenna_path = bt_le_cs_get_antenna_path(p_report->n_ap,
							     antenna_permutation_index, tone_index);
		if (antenna_path < 0) {
			LOG_WRN("Invalid antenna path.");
			return;
		}

		if (local_tone_info[tone_index].quality_indicator !=
			    BT_HCI_LE_CS_TONE_QUALITY_HIGH ||
		    remote_tone_info[tone_index].quality_indicator !=
			    BT_HCI_LE_CS_TONE_QUALITY_HIGH) {
			return;
		}

		struct bt_le_cs_iq_sample local_iq =
			bt_le_cs_parse_pct(local_tone_info[tone_index].phase_correction_term);
		struct bt_le_cs_iq_sample remote_iq =
			bt_le_cs_parse_pct(remote_tone_info[tone_index].phase_correction_term);

		m_n_iqs[antenna_path][channel_index]++;

		if (m_n_iqs[antenna_path][channel_index] == 1) {
			p_report->iq_tones[antenna_path].i_local[channel_index] = local_iq.i;
			p_report->iq_tones[antenna_path].q_local[channel_index] = local_iq.q;
			p_report->iq_tones[antenna_path].i_remote[channel_index] = remote_iq.i;
			p_report->iq_tones[antenna_path].q_remote[channel_index] = remote_iq.q;
		} else {
			cumulate_mean(&p_report->iq_tones[antenna_path].i_local[channel_index],
				      local_iq.i, &m_n_iqs[antenna_path][channel_index]);
			cumulate_mean(&p_report->iq_tones[antenna_path].q_local[channel_index],
				      local_iq.q, &m_n_iqs[antenna_path][channel_index]);
			cumulate_mean(&p_report->iq_tones[antenna_path].i_remote[channel_index],
				      remote_iq.i, &m_n_iqs[antenna_path][channel_index]);
			cumulate_mean(&p_report->iq_tones[antenna_path].q_remote[channel_index],
				      remote_iq.q, &m_n_iqs[antenna_path][channel_index]);
		}
	}
}

static void extract_rtt_timings(cs_de_report_t *p_report,
				struct bt_hci_le_cs_step_data_mode_1 *local_rtt_data,
				struct bt_hci_le_cs_step_data_mode_1 *peer_rtt_data)
{
	if (local_rtt_data->packet_quality_aa_check !=
		    BT_HCI_LE_CS_PACKET_QUALITY_AA_CHECK_SUCCESSFUL ||
	    local_rtt_data->packet_rssi == BT_HCI_LE_CS_PACKET_RSSI_NOT_AVAILABLE ||
	    local_rtt_data->tod_toa_reflector == BT_HCI_LE_CS_TIME_DIFFERENCE_NOT_AVAILABLE ||
	    peer_rtt_data->packet_quality_aa_check !=
		    BT_HCI_LE_CS_PACKET_QUALITY_AA_CHECK_SUCCESSFUL ||
	    peer_rtt_data->packet_rssi == BT_HCI_LE_CS_PACKET_RSSI_NOT_AVAILABLE ||
	    peer_rtt_data->tod_toa_reflector == BT_HCI_LE_CS_TIME_DIFFERENCE_NOT_AVAILABLE) {
		return;
	}

	if (p_report->role == BT_CONN_LE_CS_ROLE_INITIATOR) {
		p_report->rtt_accumulated_half_ns +=
			local_rtt_data->toa_tod_initiator - peer_rtt_data->tod_toa_reflector;
	} else {
		p_report->rtt_accumulated_half_ns +=
			peer_rtt_data->toa_tod_initiator - local_rtt_data->tod_toa_reflector;
	}

	p_report->rtt_count++;
}

static bool process_ranging_header(struct ras_ranging_header *ranging_header, void *user_data)
{
	cs_de_report_t *p_report = (cs_de_report_t *)user_data;

	p_report->n_ap = MAX(1, ((ranging_header->antenna_paths_mask & BIT(0)) +
				 ((ranging_header->antenna_paths_mask & BIT(1)) >> 1) +
				 ((ranging_header->antenna_paths_mask & BIT(2)) >> 2) +
				 ((ranging_header->antenna_paths_mask & BIT(3)) >> 3)));
	return true;
}

static bool process_step_data(struct bt_le_cs_subevent_step *local_step,
			      struct bt_le_cs_subevent_step *peer_step, void *user_data)
{
	cs_de_report_t *p_report = (cs_de_report_t *)user_data;

	if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_2) {
		struct bt_hci_le_cs_step_data_mode_2 *local_step_data =
			(struct bt_hci_le_cs_step_data_mode_2 *)local_step->data;
		struct bt_hci_le_cs_step_data_mode_2 *peer_step_data =
			(struct bt_hci_le_cs_step_data_mode_2 *)peer_step->data;

		extract_pcts(p_report, local_step->channel - CHANNEL_INDEX_OFFSET,
			     local_step_data->antenna_permutation_index, local_step_data->tone_info,
			     peer_step_data->tone_info);
	} else if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_1) {
		struct bt_hci_le_cs_step_data_mode_1 *local_step_data =
			(struct bt_hci_le_cs_step_data_mode_1 *)local_step->data;
		struct bt_hci_le_cs_step_data_mode_1 *peer_step_data =
			(struct bt_hci_le_cs_step_data_mode_1 *)peer_step->data;

		extract_rtt_timings(p_report, local_step_data, peer_step_data);
	} else if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_3) {
		struct bt_hci_le_cs_step_data_mode_3 *local_step_data =
			(struct bt_hci_le_cs_step_data_mode_3 *)local_step->data;
		struct bt_hci_le_cs_step_data_mode_3 *peer_step_data =
			(struct bt_hci_le_cs_step_data_mode_3 *)peer_step->data;

		extract_pcts(p_report, local_step->channel - CHANNEL_INDEX_OFFSET,
			     local_step_data->antenna_permutation_index, local_step_data->tone_info,
			     peer_step_data->tone_info);

		extract_rtt_timings(p_report,
				    (struct bt_hci_le_cs_step_data_mode_1 *)local_step_data,
				    (struct bt_hci_le_cs_step_data_mode_1 *)peer_step_data);
	}

	return true;
}

/* Returns the median of the last estimates of one beacon. */
static cs_de_dist_estimates_t get_distance(struct beacon *b)
{
	cs_de_dist_estimates_t averaged_result = {};
	uint8_t num_ifft = 0;
	uint8_t num_phase_slope = 0;
	uint8_t num_rtt = 0;

	static float temp_ifft[DE_SLIDING_WINDOW_SIZE];
	static float temp_phase_slope[DE_SLIDING_WINDOW_SIZE];
	static float temp_rtt[DE_SLIDING_WINDOW_SIZE];

	struct distance_estimate_buffer *buffer = &b->de_buf;

	int lock_state = k_mutex_lock(&distance_estimate_buffer_mutex, K_FOREVER);

	__ASSERT_NO_MSG(lock_state == 0);

	for (uint8_t i = 0; i < buffer->num_valid; i++) {
		if (isfinite(buffer->estimates[i].ifft)) {
			temp_ifft[num_ifft++] = buffer->estimates[i].ifft;
		}
		if (isfinite(buffer->estimates[i].phase_slope)) {
			temp_phase_slope[num_phase_slope++] = buffer->estimates[i].phase_slope;
		}
		if (isfinite(buffer->estimates[i].rtt)) {
			temp_rtt[num_rtt++] = buffer->estimates[i].rtt;
		}
	}

	k_mutex_unlock(&distance_estimate_buffer_mutex);

	averaged_result.ifft = median_inplace(num_ifft, temp_ifft);
	averaged_result.phase_slope = median_inplace(num_phase_slope, temp_phase_slope);
	averaged_result.rtt = median_inplace(num_rtt, temp_rtt);

	return averaged_result;
}

/* Releases the shared step buffers so the next procedure can use them. */
static void release_step_buffers(void)
{
	net_buf_simple_reset(&latest_local_steps);
	if (active_beacon && !(active_beacon->ras_feature_bits & RAS_FEAT_REALTIME_RD)) {
		net_buf_simple_reset(&latest_peer_steps);
	}
	k_sem_give(&sem_local_steps);
}

static void ranging_data_cb(struct bt_conn *conn, uint16_t ranging_counter, int err)
{
	struct beacon *b = active_beacon;

	if (b == NULL || b->conn != conn) {
		/* Late data from a beacon whose turn is over: ignore it. */
		return;
	}

	if (err) {
		LOG_ERR("B%u: error receiving ranging data, counter %d (err %d)", b->id,
			ranging_counter, err);
		return;
	}

	if (ranging_counter != most_recent_local_ranging_counter) {
		LOG_DBG("B%u: ranging data dropped, counter mismatch (peer %u, local %d)", b->id,
			ranging_counter, most_recent_local_ranging_counter);
		release_step_buffers();
		return;
	}

	if (latest_local_steps.len == 0) {
		LOG_WRN("B%u: all subevents of ranging counter %u were aborted", b->id,
			ranging_counter);
		release_step_buffers();
		return;
	}

	memset(&m_cs_de_report, 0x0, sizeof(cs_de_report_t));
	memset(m_n_iqs, 0, sizeof(m_n_iqs));

	bt_ras_rreq_rd_subevent_data_parse(&latest_peer_steps, &latest_local_steps,
					   b->cs_config.role, process_ranging_header, NULL,
					   process_step_data, &m_cs_de_report);

	for (uint8_t ap = 0; ap < m_cs_de_report.n_ap; ap++) {
		m_cs_de_report.distance_estimates[ap].ifft = NAN;
		m_cs_de_report.distance_estimates[ap].phase_slope = NAN;
		m_cs_de_report.distance_estimates[ap].rtt = NAN;
		m_cs_de_report.distance_estimates[ap].best = NAN;

		if (m_is_tone_quality_ok(m_n_iqs[ap], b->cs_config.channel_map)) {
			m_cs_de_report.tone_quality[ap] = CS_DE_TONE_QUALITY_OK;
		} else {
			m_cs_de_report.tone_quality[ap] = CS_DE_TONE_QUALITY_BAD;
		}
	}

	release_step_buffers();

	cs_de_quality_t quality = cs_de_calc(&m_cs_de_report);

	/* Only antenna path 0 is used (MAX_AP == 1). */
	if (quality == CS_DE_QUALITY_OK &&
	    (m_cs_de_report.tone_quality[0] == CS_DE_TONE_QUALITY_OK ||
	     isfinite(m_cs_de_report.distance_estimates[0].rtt))) {
		store_distance_estimates_in_buffer(&m_cs_de_report.distance_estimates[0],
						   &b->de_buf);
	}

	/* Count every finished procedure, good or bad, so the turn can end. */
	b->new_estimates++;
	k_sem_give(&sem_distance_estimate_updated);
}

static void subevent_result_cb(struct bt_conn *conn, struct bt_conn_le_cs_subevent_result *result)
{
	if (active_beacon == NULL || active_beacon->conn != conn) {
		return;
	}

	if (dropped_ranging_counter == result->header.procedure_counter) {
		return;
	}

	if (most_recent_local_ranging_counter !=
	    bt_ras_rreq_get_ranging_counter(result->header.procedure_counter)) {
		int sem_state = k_sem_take(&sem_local_steps, K_NO_WAIT);

		if (sem_state < 0) {
			dropped_ranging_counter = result->header.procedure_counter;
			LOG_DBG("Dropped subevent results. Waiting for ranging data from peer.");
			return;
		}

		most_recent_local_ranging_counter =
			bt_ras_rreq_get_ranging_counter(result->header.procedure_counter);
	}

	if (result->header.subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED) {
		/* The steps from this subevent will not be used. */
	} else if (result->step_data_buf) {
		if (result->step_data_buf->len <= net_buf_simple_tailroom(&latest_local_steps)) {
			uint16_t len = result->step_data_buf->len;
			uint8_t *step_data = net_buf_simple_pull_mem(result->step_data_buf, len);

			net_buf_simple_add_mem(&latest_local_steps, step_data, len);
		} else {
			LOG_ERR("Not enough memory to store step data. (%d > %d)",
				latest_local_steps.len + result->step_data_buf->len,
				latest_local_steps.size);
			net_buf_simple_reset(&latest_local_steps);
			dropped_ranging_counter = result->header.procedure_counter;
			return;
		}
	}

	dropped_ranging_counter = PROCEDURE_COUNTER_NONE;

	if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE) {
		most_recent_local_ranging_counter =
			bt_ras_rreq_get_ranging_counter(result->header.procedure_counter);
	} else if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_ABORTED) {
		LOG_WRN("B%u: procedure %u aborted", active_beacon->id,
			result->header.procedure_counter);
		net_buf_simple_reset(&latest_local_steps);
		k_sem_give(&sem_local_steps);
	}
}

static void ranging_data_ready_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	if (active_beacon == NULL || active_beacon->conn != conn) {
		return;
	}

	if (ranging_counter == most_recent_local_ranging_counter) {
		int err = bt_ras_rreq_cp_get_ranging_data(conn, &latest_peer_steps,
							  ranging_counter, ranging_data_cb);
		if (err) {
			LOG_ERR("Get ranging data failed (err %d)", err);
			net_buf_simple_reset(&latest_local_steps);
			net_buf_simple_reset(&latest_peer_steps);
			k_sem_give(&sem_local_steps);
		}
	}
}

static void ranging_data_overwritten_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	LOG_DBG("Ranging data overwritten %i", ranging_counter);
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	if (err) {
		LOG_ERR("MTU exchange failed (err %d)", err);
		return;
	}

	LOG_INF("MTU exchange success (%u)", bt_gatt_get_mtu(conn));
	k_sem_give(&sem_mtu_exchange_done);
}

static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
{
	int err;
	struct bt_conn *conn = bt_gatt_dm_conn_get(dm);

	err = bt_ras_rreq_alloc_and_assign_handles(dm, conn);
	if (err) {
		LOG_ERR("RAS RREQ alloc init failed (err %d)", err);
	}
	setup_status = err ? 1 : 0;

	err = bt_gatt_dm_data_release(dm);
	if (err) {
		LOG_ERR("Could not release the discovery data (err %d)", err);
	}

	k_sem_give(&sem_discovery_done);
}

static void discovery_service_not_found_cb(struct bt_conn *conn, void *context)
{
	LOG_WRN("Ranging service not found");
	setup_status = 1;
	k_sem_give(&sem_discovery_done);
}

static void discovery_error_found_cb(struct bt_conn *conn, int err, void *context)
{
	LOG_WRN("The discovery procedure failed (err %d)", err);
	setup_status = 1;
	k_sem_give(&sem_discovery_done);
}

static struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_completed_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found = discovery_error_found_cb,
};

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	if (err) {
		LOG_ERR("Security failed: level %u err %d %s", level, err,
			bt_security_err_to_str(err));
		setup_status = 1;
	} else {
		LOG_INF("Security changed: level %u", level);
		setup_status = 0;
	}
	k_sem_give(&sem_security);
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	/* Ignore peer parameter preferences. */
	return false;
}

static void update_status_led(void)
{
	int ready = 0;

	for (int i = 0; i < NUM_BEACONS; i++) {
		ready += beacons[i].ready ? 1 : 0;
	}
	/* LED on only when every beacon is connected and set up */
	if (ready == NUM_BEACONS) {
		dk_set_led_on(CON_STATUS_LED);
	} else {
		dk_set_led_off(CON_STATUS_LED);
	}
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	struct beacon *b = connecting_beacon;

	connecting_beacon = NULL;

	if (b == NULL || b->conn != conn) {
		return; /* not a connection we created */
	}

	if (err) {
		LOG_WRN("B%u: connection failed (err 0x%02X)", b->id, err);
		bt_conn_unref(b->conn);
		b->conn = NULL;
	} else {
		LOG_INF("B%u: connected", b->id);
	}

	setup_status = err;
	k_sem_give(&sem_connected);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	struct beacon *b = beacon_by_conn(conn);

	if (b == NULL) {
		return;
	}

	LOG_WRN("B%u: disconnected (reason 0x%02X)", b->id, reason);

	b->ready = false;
	b->status = IL_STATUS_NOT_CONNECTED;
	bt_conn_unref(b->conn);
	b->conn = NULL;
	update_status_led();

	/* Unblock the main thread if it waits on this beacon. */
	if (active_beacon == b) {
		k_sem_give(&sem_distance_estimate_updated);
		k_sem_give(&sem_procedure_enable);
	}
	if (setup_beacon == b) {
		setup_status = 1;
		k_sem_give(&sem_security);
		k_sem_give(&sem_mtu_exchange_done);
		k_sem_give(&sem_discovery_done);
		k_sem_give(&sem_ras_features);
		k_sem_give(&sem_remote_capabilities_obtained);
		k_sem_give(&sem_config_created);
		k_sem_give(&sem_cs_security_enabled);
	}
}

static void remote_capabilities_cb(struct bt_conn *conn, uint8_t status,
				   struct bt_conn_le_cs_capabilities *params)
{
	ARG_UNUSED(params);

	if (status != BT_HCI_ERR_SUCCESS) {
		LOG_WRN("CS capability exchange failed. (HCI status 0x%02x)", status);
	}
	setup_status = status;
	k_sem_give(&sem_remote_capabilities_obtained);
}

static void config_create_cb(struct bt_conn *conn, uint8_t status,
			     struct bt_conn_le_cs_config *config)
{
	struct beacon *b = beacon_by_conn(conn);

	if (status == BT_HCI_ERR_SUCCESS && b) {
		b->cs_config = *config;
		LOG_INF("B%u: CS config created (mode %u)", b->id, config->mode);
	} else {
		LOG_WRN("CS config creation failed. (HCI status 0x%02x)", status);
	}
	setup_status = status;
	k_sem_give(&sem_config_created);
}

static void security_enable_cb(struct bt_conn *conn, uint8_t status)
{
	if (status != BT_HCI_ERR_SUCCESS) {
		LOG_WRN("CS security enable failed. (HCI status 0x%02x)", status);
	}
	setup_status = status;
	k_sem_give(&sem_cs_security_enabled);
}

static void procedure_enable_cb(struct bt_conn *conn, uint8_t status,
				struct bt_conn_le_cs_procedure_enable_complete *params)
{
	if (status == BT_HCI_ERR_SUCCESS) {
		procedure_enabled_state = (params->state == 1);
		LOG_DBG("CS procedures %s", procedure_enabled_state ? "enabled" : "disabled");
	} else {
		LOG_WRN("CS procedures enable/disable failed. (HCI status 0x%02x)", status);
	}
	setup_status = status;
	k_sem_give(&sem_procedure_enable);
}

static void ras_features_read_cb(struct bt_conn *conn, uint32_t feature_bits, int err)
{
	struct beacon *b = beacon_by_conn(conn);

	if (err) {
		LOG_WRN("Error while reading RAS feature bits (err %d)", err);
	} else if (b) {
		b->ras_feature_bits = feature_bits;
	}
	setup_status = err ? 1 : 0;
	k_sem_give(&sem_ras_features);
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.le_param_req = le_param_req,
	.security_changed = security_changed,
	.le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
	.le_cs_config_complete = config_create_cb,
	.le_cs_security_enable_complete = security_enable_cb,
	.le_cs_procedure_enable_complete = procedure_enable_cb,
	.le_cs_subevent_data_available = subevent_result_cb,
};

/* ------------------------------------------------------------------------- */
/* Scanning: find "IL_B<id>" beacons that are not connected yet               */
/* ------------------------------------------------------------------------- */

#define ACL_CONN_INTERVAL 0x10 /* 20 ms */

static const struct bt_le_conn_param *conn_param =
	BT_LE_CONN_PARAM(ACL_CONN_INTERVAL, ACL_CONN_INTERVAL, 0, BT_GAP_MS_TO_CONN_TIMEOUT(4000));

static bool parse_name_cb(struct bt_data *data, void *user_data)
{
	int *beacon_id = user_data;
	const size_t prefix_len = sizeof(BEACON_NAME_PREFIX) - 1;

	if (data->type != BT_DATA_NAME_COMPLETE && data->type != BT_DATA_NAME_SHORTENED) {
		return true;
	}

	/* Expect exactly "IL_B" + one or two digits */
	if (data->data_len <= prefix_len || data->data_len > prefix_len + 2 ||
	    memcmp(data->data, BEACON_NAME_PREFIX, prefix_len) != 0) {
		return false;
	}

	int id = 0;

	for (size_t i = prefix_len; i < data->data_len; i++) {
		if (data->data[i] < '0' || data->data[i] > '9') {
			return false;
		}
		id = id * 10 + (data->data[i] - '0');
	}
	*beacon_id = id;
	return false;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	int id = -1;
	int err;

	if (connecting_beacon != NULL) {
		return;
	}

	if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	bt_data_parse(ad, parse_name_cb, &id);
	if (id < 1 || id > NUM_BEACONS) {
		return;
	}

	struct beacon *b = &beacons[id - 1];

	if (b->conn != NULL) {
		return; /* already connected */
	}

	if (bt_le_scan_stop()) {
		return;
	}

	LOG_INF("Found beacon B%u (RSSI %d), connecting...", id, rssi);

	connecting_beacon = b;
	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, conn_param, &b->conn);
	if (err) {
		LOG_WRN("B%u: create connection failed (err %d)", id, err);
		connecting_beacon = NULL;
		b->conn = NULL;
		setup_status = 1;
		k_sem_give(&sem_connected);
	}
}

/* ------------------------------------------------------------------------- */
/* Per-beacon setup (same steps as the single-beacon sample)                  */
/* ------------------------------------------------------------------------- */

#define SETUP_WAIT(sem, what)                                                                      \
	do {                                                                                       \
		if (wait_sem(&(sem), SETUP_STEP_TIMEOUT_MS) != 0 || setup_status != 0) {            \
			LOG_WRN("B%u: setup failed at: %s", b->id, what);                          \
			goto fail;                                                                 \
		}                                                                                  \
	} while (0)

#define SETUP_CALL(call, what)                                                                     \
	do {                                                                                       \
		int _err = (call);                                                                 \
		if (_err) {                                                                        \
			LOG_WRN("B%u: %s failed (err %d)", b->id, what, _err);                     \
			goto fail;                                                                 \
		}                                                                                  \
	} while (0)

static void cs_config_get(struct bt_le_cs_create_config_params *config_params);

static int setup_beacon_cs(struct beacon *b)
{
	static struct bt_gatt_exchange_params mtu_exchange_params = {.func = mtu_exchange_cb};

	setup_beacon = b;

	k_sem_reset(&sem_security);
	k_sem_reset(&sem_mtu_exchange_done);
	k_sem_reset(&sem_discovery_done);
	k_sem_reset(&sem_ras_features);
	k_sem_reset(&sem_remote_capabilities_obtained);
	k_sem_reset(&sem_config_created);
	k_sem_reset(&sem_cs_security_enabled);

	SETUP_CALL(bt_conn_set_security(b->conn, BT_SECURITY_L2), "encryption");
	SETUP_WAIT(sem_security, "encryption");

	SETUP_CALL(bt_gatt_exchange_mtu(b->conn, &mtu_exchange_params), "MTU exchange");
	SETUP_WAIT(sem_mtu_exchange_done, "MTU exchange");

	SETUP_CALL(bt_gatt_dm_start(b->conn, BT_UUID_RANGING_SERVICE, &discovery_cb, NULL),
		   "discovery");
	SETUP_WAIT(sem_discovery_done, "discovery");

	const struct bt_le_cs_set_default_settings_param default_settings = {
		.enable_initiator_role = true,
		.enable_reflector_role = false,
		.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
		.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
	};

	SETUP_CALL(bt_le_cs_set_default_settings(b->conn, &default_settings), "CS settings");

	SETUP_CALL(bt_ras_rreq_read_features(b->conn, ras_features_read_cb), "RAS features");
	SETUP_WAIT(sem_ras_features, "RAS features");

	const bool realtime_rd = b->ras_feature_bits & RAS_FEAT_REALTIME_RD;

	if (realtime_rd) {
		/* All beacons share latest_peer_steps: only the active one sends data. */
		SETUP_CALL(bt_ras_rreq_realtime_rd_subscribe(b->conn, &latest_peer_steps,
							     ranging_data_cb),
			   "real-time RD subscribe");
	} else {
		SETUP_CALL(bt_ras_rreq_rd_overwritten_subscribe(b->conn,
								ranging_data_overwritten_cb),
			   "RD overwritten subscribe");
		SETUP_CALL(bt_ras_rreq_rd_ready_subscribe(b->conn, ranging_data_ready_cb),
			   "RD ready subscribe");
		SETUP_CALL(bt_ras_rreq_on_demand_rd_subscribe(b->conn), "on-demand RD subscribe");
		SETUP_CALL(bt_ras_rreq_cp_subscribe(b->conn), "CP subscribe");
	}

	SETUP_CALL(bt_le_cs_read_remote_supported_capabilities(b->conn), "CS capabilities");
	SETUP_WAIT(sem_remote_capabilities_obtained, "CS capabilities");

	struct bt_le_cs_create_config_params config_params;

	cs_config_get(&config_params);
	bt_le_cs_set_valid_chmap_bits(config_params.channel_map);

	SETUP_CALL(bt_le_cs_create_config(b->conn, &config_params,
					  BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE),
		   "CS config");
	SETUP_WAIT(sem_config_created, "CS config");

	SETUP_CALL(bt_le_cs_security_enable(b->conn), "CS security");
	SETUP_WAIT(sem_cs_security_enabled, "CS security");

	/* scale factor of conn_interval units to proc_interval units is 1.25/0.625 = 2 */
	const uint16_t acl_interval_in_proc_interval_units = ACL_CONN_INTERVAL * 2;
	const uint16_t desired_procedure_interval = realtime_rd ? 5 : 10;
	const uint16_t desired_max_procedure_length =
		acl_interval_in_proc_interval_units * (desired_procedure_interval - 1);

	const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
		.config_id = CS_CONFIG_ID,
		.max_procedure_len = desired_max_procedure_length,
		.min_procedure_interval = desired_procedure_interval,
		.max_procedure_interval = desired_procedure_interval,
		.max_procedure_count = 0, /* run until we disable it at the end of the turn */
		.min_subevent_len = 16000,
		.max_subevent_len = 16000,
		.tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
		.phy = BT_LE_CS_PROCEDURE_PHY_2M,
		.tx_power_delta = 0x80,
		.preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
		.snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
		.snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
	};

	SETUP_CALL(bt_le_cs_set_procedure_parameters(b->conn, &procedure_params),
		   "procedure parameters");

	b->ready = true;
	setup_beacon = NULL;
	LOG_INF("B%u: ready for ranging", b->id);
	update_status_led();
	return 0;

fail:
	setup_beacon = NULL;
	if (b->conn) {
		bt_conn_disconnect(b->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
	return -EIO;
}

/* Scans for one missing beacon for up to timeout_ms, connects and sets it up. */
static void connect_one_missing_beacon(uint32_t timeout_ms)
{
	int err;

	k_sem_reset(&sem_connected);
	setup_status = 0;

	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE_CONTINUOUS, device_found);
	if (err && err != -EALREADY) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return;
	}

	if (wait_sem(&sem_connected, timeout_ms) != 0) {
		/* Nothing found in time */
		(void)bt_le_scan_stop();
		if (connecting_beacon) {
			/* Connection attempt still pending: cancel it */
			struct beacon *b = connecting_beacon;

			connecting_beacon = NULL;
			if (b->conn) {
				bt_conn_disconnect(b->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
				bt_conn_unref(b->conn);
				b->conn = NULL;
			}
		}
		return;
	}

	for (int i = 0; i < NUM_BEACONS; i++) {
		struct beacon *b = &beacons[i];

		if (b->conn != NULL && !b->ready && setup_status == 0) {
			(void)setup_beacon_cs(b);
		}
	}
}

static int connected_count(void)
{
	int n = 0;

	for (int i = 0; i < NUM_BEACONS; i++) {
		n += beacons[i].conn ? 1 : 0;
	}
	return n;
}

/* ------------------------------------------------------------------------- */
/* Ranging turn: CS on one beacon only                                        */
/* ------------------------------------------------------------------------- */

static int set_procedure_enable(struct beacon *b, bool enable)
{
	struct bt_le_cs_procedure_enable_param params = {
		.config_id = CS_CONFIG_ID,
		.enable = enable ? 1 : 0,
	};
	int err;

	k_sem_reset(&sem_procedure_enable);
	err = bt_le_cs_procedure_enable(b->conn, &params);
	if (err) {
		return err;
	}
	if (wait_sem(&sem_procedure_enable, 1000) != 0) {
		return -ETIMEDOUT;
	}
	return setup_status ? -EIO : 0;
}

static float calibrated_distance(struct beacon *b, bool *valid);

static void ranging_turn(struct beacon *b)
{
	int err;

	/* Fresh state for the shared step buffers */
	net_buf_simple_reset(&latest_local_steps);
	net_buf_simple_reset(&latest_peer_steps);
	k_sem_reset(&sem_local_steps);
	k_sem_give(&sem_local_steps);
	most_recent_local_ranging_counter = PROCEDURE_COUNTER_NONE;
	dropped_ranging_counter = PROCEDURE_COUNTER_NONE;
	k_sem_reset(&sem_distance_estimate_updated);

	b->new_estimates = 0;
	active_beacon = b;

	err = set_procedure_enable(b, true);
	if (err) {
		LOG_WRN("B%u: CS enable failed (err %d)", b->id, err);
		active_beacon = NULL;
		b->status = IL_STATUS_TIMEOUT;
		return;
	}

	int64_t deadline = k_uptime_get() + TURN_TIMEOUT_MS;

	while (b->new_estimates < SAMPLES_PER_TURN && b->conn) {
		int64_t left = deadline - k_uptime_get();

		if (left <= 0 ||
		    k_sem_take(&sem_distance_estimate_updated, K_MSEC(left)) != 0) {
			break;
		}
	}

	if (b->conn) {
		err = set_procedure_enable(b, false);
		if (err) {
			LOG_WRN("B%u: CS disable failed (err %d)", b->id, err);
		}
	}

	/* Let an in-flight ranging data transfer finish before the next beacon */
	k_sleep(K_MSEC(20));
	active_beacon = NULL;

	if (b->conn == NULL) {
		b->status = IL_STATUS_NOT_CONNECTED;
		return;
	}

	if (b->new_estimates == 0) {
		b->status = IL_STATUS_TIMEOUT;
		return;
	}

	bool valid;
	float d = calibrated_distance(b, &valid);

	if (valid) {
		b->distance_m = d;
		b->status = IL_STATUS_OK;
		b->updated_ms = k_uptime_get_32();
	} else {
		b->status = IL_STATUS_NO_ESTIMATE;
	}
}


static void cs_config_get(struct bt_le_cs_create_config_params *config_params)
{
	memset(config_params, 0, sizeof(struct bt_le_cs_create_config_params));
	config_params->id = CS_CONFIG_ID;
	config_params->mode = CS_CONFIG_MODE;
	config_params->min_main_mode_steps = 2;
	config_params->max_main_mode_steps = 5;
	config_params->main_mode_repetition = 0;
	config_params->mode_0_steps = NUM_MODE_0_STEPS;
	config_params->role = BT_CONN_LE_CS_ROLE_INITIATOR;
	config_params->rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY;
	config_params->cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY;
	config_params->channel_map_repetition = 1;
	config_params->channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B;
	config_params->ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT;
	config_params->ch3c_jump = 2;
}

/* Apply the linear calibration; returns false if the raw value is unusable
 * (not finite, or below the offset, which would give a negative distance).
 */
static bool calibrate(float raw, float offset, float slope, float *out)
{
	if (!isfinite(raw) || raw <= offset) {
		return false;
	}

	*out = (raw - offset) / slope;
	return true;
}

/* Calibrated, weighted distance to one beacon (median of its last estimates). */
static float calibrated_distance(struct beacon *b, bool *valid)
{
	cs_de_dist_estimates_t d = get_distance(b);
	float ifft = 0.0f, phase = 0.0f, rtt = 0.0f;
	float weighted_sum = 0.0f, weight_sum = 0.0f;

	if (IFFT_WEIGHT > 0.0f && calibrate(d.ifft, IFFT_OFFSET_M, IFFT_SLOPE, &ifft)) {
		weighted_sum += ifft * IFFT_WEIGHT;
		weight_sum += IFFT_WEIGHT;
	}
	if (PHASE_WEIGHT > 0.0f &&
	    calibrate(d.phase_slope, PHASE_SLOPE_OFFSET_M, PHASE_SLOPE_SLOPE, &phase)) {
		weighted_sum += phase * PHASE_WEIGHT;
		weight_sum += PHASE_WEIGHT;
	}
	if (RTT_WEIGHT > 0.0f && calibrate(d.rtt, RTT_OFFSET_M, RTT_SLOPE, &rtt)) {
		weighted_sum += rtt * RTT_WEIGHT;
		weight_sum += RTT_WEIGHT;
	}

	LOG_DBG("B%u raw IFFT:%.2f Phase:%.2f RTT:%.2f", b->id, (double)d.ifft,
		(double)d.phase_slope, (double)d.rtt);

	*valid = weight_sum > 0.0f;
	return *valid ? weighted_sum / weight_sum : NAN;
}

/* ------------------------------------------------------------------------- */
/* NRF-02: measurement message on the UART                                    */
/*   $RNG,<seq>,<t_ms>,<beacon_id>,<dist_mm>,<status>*<XX>\r\n                 */
/* ------------------------------------------------------------------------- */

static uint32_t rng_seq;

static uint8_t nmea_checksum(const char *s)
{
	uint8_t cs = 0;

	while (*s) {
		cs ^= (uint8_t)*s++;
	}
	return cs;
}

static void print_measurement(const struct beacon *b)
{
	char body[64];
	int32_t dist_mm = -1;

	if (b->status == IL_STATUS_OK) {
		dist_mm = (int32_t)lroundf(b->distance_m * 1000.0f);
	}

	/* body = everything between '$' and '*' */
	snprintk(body, sizeof(body), "RNG,%u,%u,%u,%d,%u", rng_seq++, k_uptime_get_32(), b->id,
		 dist_mm, (unsigned int)b->status);
	printk("$%s*%02X\r\n", body, nmea_checksum(body));
}

/**
 * @brief Latest distances for the trilateration step.
 *
 * @param dist_m   Output, NUM_BEACONS entries, index i = beacon id i+1 (meters).
 * @param valid    Output, true if dist_m[i] is fresh (status OK and not older than
 *                 DISTANCE_MAX_AGE_MS).
 * @return number of valid distances.
 */
static int il_get_distances(float dist_m[NUM_BEACONS], bool valid[NUM_BEACONS])
{
	uint32_t now = k_uptime_get_32();
	int n = 0;

	for (int i = 0; i < NUM_BEACONS; i++) {
		const struct beacon *b = &beacons[i];

		valid[i] = b->status == IL_STATUS_OK && b->updated_ms != 0 &&
			   (now - b->updated_ms) <= DISTANCE_MAX_AGE_MS;
		dist_m[i] = valid[i] ? b->distance_m : NAN;
		n += valid[i] ? 1 : 0;
	}
	return n;
}

/* ------------------------------------------------------------------------- */
/* Position: 2D trilateration (Sofia's code) + $POS message (SYS-03)          */
/*   $POS,<seq>,<t_ms>,<x_mm>,<y_mm>,<z_mm>,<status>*<XX>\r\n                 */
/* ------------------------------------------------------------------------- */

enum il_pos_status {
	IL_POS_OK = 0,              /* position computed */
	IL_POS_TOO_FEW_BEACONS = 1, /* fewer than 3 fresh distances */
	IL_POS_BAD_GEOMETRY = 2,    /* beacons on one line, or calculation failed */
};

static uint32_t pos_seq;

static void compute_and_print_position(void)
{
	float dist_m[NUM_BEACONS];
	bool valid[NUM_BEACONS];
	float anchor_xy_m[NUM_BEACONS][2];
	float horiz_m[NUM_BEACONS];
	int n = 0;
	float x_m = 0.0f, y_m = 0.0f;
	enum il_pos_status status;

	il_get_distances(dist_m, valid);

	for (int i = 0; i < NUM_BEACONS; i++) {
		if (!valid[i]) {
			continue;
		}

		/* The CS distance is 3D; remove the height difference to get the distance
		 * on the floor plane: d_floor = sqrt(d^2 - dz^2)
		 */
		float dz_m = (beacon_pos_mm[i][2] - MOBILE_Z_MM) / 1000.0f;
		float d2 = dist_m[i] * dist_m[i] - dz_m * dz_m;

		anchor_xy_m[n][0] = beacon_pos_mm[i][0] / 1000.0f;
		anchor_xy_m[n][1] = beacon_pos_mm[i][1] / 1000.0f;
		horiz_m[n] = d2 > 0.0f ? sqrtf(d2) : 0.0f;
		n++;
	}

	if (n < 3) {
		status = IL_POS_TOO_FEW_BEACONS;
	} else if (il_trilaterate_2d(anchor_xy_m, horiz_m, n, &x_m, &y_m) &&
		   isfinite(x_m) && isfinite(y_m)) {
		status = IL_POS_OK;
	} else {
		status = IL_POS_BAD_GEOMETRY;
	}

	char body[64];
	int32_t x_mm = 0, y_mm = 0;

	if (status == IL_POS_OK) {
		x_mm = (int32_t)lroundf(x_m * 1000.0f);
		y_mm = (int32_t)lroundf(y_m * 1000.0f);
	}

	/* z is 0 for the 2D demo */
	snprintk(body, sizeof(body), "POS,%u,%u,%d,%d,%d,%u", pos_seq++, k_uptime_get_32(), x_mm,
		 y_mm, 0, (unsigned int)status);
	printk("$%s*%02X\r\n", body, nmea_checksum(body));
}

int main(void)
{
	int err;

	LOG_INF("Indoor localization: multi-beacon CS initiator, %d beacons", NUM_BEACONS);

	dk_leds_init();

	for (int i = 0; i < NUM_BEACONS; i++) {
		beacons[i].id = i + 1;
		beacons[i].status = IL_STATUS_NOT_CONNECTED;
		beacons[i].distance_m = NAN;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	/* Startup: wait (up to ~15 s) until every beacon is connected */
	int64_t startup_deadline = k_uptime_get() + 15000;

	while (connected_count() < NUM_BEACONS && k_uptime_get() < startup_deadline) {
		connect_one_missing_beacon(3000);
	}

	LOG_INF("%d of %d beacons connected, starting ranging", connected_count(), NUM_BEACONS);

	while (true) {
		/* Round-robin over all beacons */
		for (int i = 0; i < NUM_BEACONS; i++) {
			struct beacon *b = &beacons[i];

			if (b->ready && b->conn) {
				ranging_turn(b);
			} else {
				b->status = IL_STATUS_NOT_CONNECTED;
			}
			print_measurement(b);
		}

		/* X/Y from the latest distances of all beacons */
		compute_and_print_position();

		/* Try to (re)connect a missing beacon, short scan so ranging is not blocked */
		if (connected_count() < NUM_BEACONS) {
			connect_one_missing_beacon(connected_count() == 0 ? 3000 : 300);
		}
	}

	return 0;
}
