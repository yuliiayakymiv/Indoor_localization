#pragma once

/*
 * Shared format of our beacon advertising packet (used by beacon and scanner).
 * Manufacturer Specific Data, 5 bytes:
 *   [0..1] company ID 0xFFFF (reserved for testing), little-endian
 *   [2..3] 'I','L' - marker that this is our beacon
 *   [4]    beacon ID
 */
#define IL_COMPANY_ID_LO  0xFF
#define IL_COMPANY_ID_HI  0xFF
#define IL_MARKER_0       'I'
#define IL_MARKER_1       'L'
#define IL_MFG_DATA_LEN   5

#define IL_MAX_BEACONS    8
