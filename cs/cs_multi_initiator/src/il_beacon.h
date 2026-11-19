/*
 * Beacon build only (CONFIG_IL_BEACON_ID > 0): reflector role of a beacon.
 */

#ifndef IL_BEACON_H_
#define IL_BEACON_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Bluetooth company id of the manufacturer data in the beacon advertisement
 * (0xFFFF = reserved for tests).
 *
 * Manufacturer data = company id (2 bytes, little endian), then one entry per measured
 * beacon: <peer beacon id: 1 byte> <distance in mm: 2 bytes, little endian>.
 */
#define IL_ADV_COMPANY_ID 0xFFFF
#define IL_ADV_ENTRY_LEN  3

/**
 * @brief Advertise as "IL_B<id>" and answer the mobile node as a CS reflector. Never returns.
 *
 * @param dist_entries  Measured beacon distances, IL_ADV_ENTRY_LEN bytes each (see above).
 * @param len           Length of dist_entries in bytes (0 if nothing was measured).
 */
void il_reflector_run(const uint8_t *dist_entries, size_t len);

/** @brief true while a mobile node or another beacon is connected to this reflector. */
bool il_reflector_busy(void);

#endif /* IL_BEACON_H_ */
