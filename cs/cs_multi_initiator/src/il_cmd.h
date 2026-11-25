#pragma once

/*
 * Mobile node: receive side of the UARTs (CONFIG_IL_COMMANDS).
 *
 * Console (USB, or pins D6/D7 with uart_pins.overlay) - text commands, one per line, the
 * NMEA checksum "*XX" is optional (checked if present):
 *     $MODE,CAR            2D, beacons B1..B3 (B4 ignored)
 *     $MODE,DRONE          3D, all beacons, z with the laser
 *     $LSR,<range_mm>,<roll_cdeg>,<pitch_cdeg>
 *                          one laser rangefinder sample (if the rangefinder is read by the
 *                          Raspberry Pi instead of the flight controller; also for tests)
 * MAVLink UART RX (pin D7, CONFIG_IL_LASER_FROM_FC): bytes go to il_mavlink_rx_byte().
 * If both are the same UART, every byte goes to both parsers.
 */

#include "trilat_bridge.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_IL_COMMANDS)
/** Starts the receive interrupts and the parser thread. */
void il_cmd_init(void);
#else
static inline void il_cmd_init(void) {}
#endif

/* Implemented in main.c */
void il_on_mode_command(enum il_mode mode);

#ifdef __cplusplus
}
#endif
