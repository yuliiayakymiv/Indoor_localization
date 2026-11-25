#pragma once

/*
 * MAVLink link of the mobile node (CONFIG_IL_MAVLINK).
 *
 * TX: binary MAVLink v2 on its own UART (uart21 = pins D6 TX / D7 RX, 115200 baud); the text
 * output ($RNG, $POS, log) stays on the console (USB). Packets are made by the team's
 * ../../mavlink/mavlink_packer.cpp:
 *   - HEARTBEAT once a second (from its own thread),
 *   - VISION_POSITION_ESTIMATE after every computed position (with its uncertainty).
 *
 * RX (CONFIG_IL_LASER_FROM_FC): bytes from the flight controller (fed by il_cmd.c, which
 * owns the UART receive interrupts) are parsed; ATTITUDE and a downward DISTANCE_SENSOR
 * give the laser samples for the height (il_on_laser_sample() in main.c).
 */

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/devicetree.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_IL_MAVLINK)

/* The UART for MAVLink: the "il,mavlink-uart" chosen node if an overlay sets one,
 * otherwise uart21 (XIAO pins D6 = TX, D7 = RX).
 */
#if DT_HAS_CHOSEN(il_mavlink_uart)
#define IL_MAVLINK_UART DT_CHOSEN(il_mavlink_uart)
#else
#define IL_MAVLINK_UART DT_NODELABEL(uart21)
#endif

/* Starts the output (and the heartbeat). false if the UART is not ready. */
bool il_mavlink_init(void);

/* Position in meters, beacon frame (z up), and its 1-sigma uncertainty. */
void il_mavlink_send_position(float x_m, float y_m, float z_m, float sigma_xy_m,
			      float sigma_z_m);

/* One received byte from the flight controller. */
void il_mavlink_rx_byte(uint8_t c);

#else

static inline bool il_mavlink_init(void) { return true; }
static inline void il_mavlink_send_position(float x_m, float y_m, float z_m, float sigma_xy_m,
					    float sigma_z_m)
{
	(void)x_m; (void)y_m; (void)z_m; (void)sigma_xy_m; (void)sigma_z_m;
}
static inline void il_mavlink_rx_byte(uint8_t c) { (void)c; }

#endif

/* Implemented in main.c: one laser sample (range along the beam, attitude in radians). */
void il_on_laser_sample(float range_m, float roll_rad, float pitch_rad);

#ifdef __cplusplus
}
#endif
