#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Called at most once per report interval for each beacon,
 * with RSSI averaged over that interval.
 * Runs in the Bluetooth thread: keep it short (format + send is fine).
 */
typedef void (*ble_scanner_cb_t)(uint8_t beacon_id, int8_t rssi, uint32_t time_ms);

/* Enables Bluetooth and starts passive scanning. Returns 0 on success. */
int ble_scanner_start(ble_scanner_cb_t cb, uint32_t report_interval_ms);

/* Formats "BEACON_01,-62,123456\n". Returns length (without '\0'). */
int ble_scanner_format_line(char *buf, size_t size,
			    uint8_t beacon_id, int8_t rssi, uint32_t time_ms);

#ifdef __cplusplus
}
#endif
