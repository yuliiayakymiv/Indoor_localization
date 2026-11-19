#pragma once

/*
 * MAVLink output of the mobile node (CONFIG_IL_MAVLINK).
 *
 * Binary MAVLink v2 goes to its own UART (uart21 = pins D6 TX / D7 RX, 115200 baud), the text
 * output ($RNG, $POS, log) stays on the console (USB). Packets are made by the team's
 * ../../mavlink/mavlink_packer.cpp:
 *   - HEARTBEAT once a second (from its own thread),
 *   - VISION_POSITION_ESTIMATE after every computed position.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_IL_MAVLINK)

/* Starts the output (and the heartbeat). false if the UART is not ready. */
bool il_mavlink_init(void);

/* Position in meters, plane of the beacons, z up (the packer converts z to NED). */
void il_mavlink_send_position(float x_m, float y_m, float z_m);

#else

static inline bool il_mavlink_init(void) { return true; }
static inline void il_mavlink_send_position(float x_m, float y_m, float z_m)
{
	(void)x_m; (void)y_m; (void)z_m;
}

#endif

#ifdef __cplusplus
}
#endif
