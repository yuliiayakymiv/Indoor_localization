/*
 * Bridge between main.c and the team's MAVLink packer (../../../mavlink/mavlink_packer.cpp).
 * See mavlink_bridge.h.
 */

#include "mavlink_bridge.h"

#if defined(CONFIG_IL_MAVLINK)

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <common/mavlink.h>
#include "mavlink_packer.hpp"

LOG_MODULE_REGISTER(il_mavlink, LOG_LEVEL_INF);

#if DT_SAME_NODE(IL_MAVLINK_UART, DT_CHOSEN(zephyr_console))
#warning "MAVLink and the text console share one UART: the packets are mixed with the text"
#endif

namespace {

const struct device *const uart_dev = DEVICE_DT_GET(IL_MAVLINK_UART);
bool ready;
K_MUTEX_DEFINE(tx_lock); /* heartbeat thread and main thread must not interleave packets */

void send(const uint8_t *buf, uint16_t len)
{
	k_mutex_lock(&tx_lock, K_FOREVER);
	for (uint16_t i = 0; i < len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
	k_mutex_unlock(&tx_lock);
}

void heartbeat_thread(void *, void *, void *)
{
	uint8_t buf[MavlinkPacker::MAVLINK_PACKET_MAX];

	while (true) {
		k_sleep(K_MSEC(1000));
		if (ready) {
			send(buf, MavlinkPacker::pack_heartbeat(buf));
		}
	}
}

/* Attitude from the flight controller (ATTITUDE, #30) */
float att_roll, att_pitch;
int64_t att_time_ms = -100000;
uint32_t laser_samples, laser_no_attitude;

} // namespace

K_THREAD_DEFINE(il_mavlink_hb, 1024, heartbeat_thread, NULL, NULL, NULL, 7, 0, 0);

extern "C" bool il_mavlink_init(void)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("MAVLink UART %s is not ready", uart_dev->name);
		return false;
	}
	ready = true;
	LOG_INF("MAVLink output on %s", uart_dev->name);
	return true;
}

extern "C" void il_mavlink_send_position(float x_m, float y_m, float z_m, float sigma_xy_m,
					 float sigma_z_m)
{
	if (!ready) {
		return;
	}

	uint8_t buf[MavlinkPacker::MAVLINK_PACKET_MAX];
	const Point3D p = {x_m, y_m, z_m};
	uint64_t time_usec = (uint64_t)k_uptime_get() * 1000ULL; /* time since boot */

	send(buf, MavlinkPacker::pack_vision_position(p, time_usec, sigma_xy_m, sigma_z_m, buf));
}

extern "C" void il_mavlink_rx_byte(uint8_t c)
{
#if defined(CONFIG_IL_LASER_FROM_FC)
	static mavlink_message_t msg;
	static mavlink_status_t status;

	if (!mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status)) {
		return;
	}

	switch (msg.msgid) {
	case MAVLINK_MSG_ID_ATTITUDE: {
		mavlink_attitude_t a;

		mavlink_msg_attitude_decode(&msg, &a);
		att_roll = a.roll;
		att_pitch = a.pitch;
		att_time_ms = k_uptime_get();
		break;
	}
	case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
		mavlink_distance_sensor_t d;

		mavlink_msg_distance_sensor_decode(&msg, &d);
		if (d.orientation != MAV_SENSOR_ROTATION_PITCH_270) {
			break; /* not looking down */
		}
		if (d.current_distance < d.min_distance || d.current_distance > d.max_distance) {
			break; /* out of range: no surface seen */
		}
		if (k_uptime_get() - att_time_ms > 500) {
			/* no fresh attitude: the tilt is unknown, do not guess */
			if ((laser_no_attitude++ % 100) == 0) {
				LOG_WRN("Laser samples without ATTITUDE from the flight controller "
					"(stream SRx_EXTRA1)");
			}
			break;
		}
		laser_samples++;
		il_on_laser_sample(d.current_distance / 100.0f, att_roll, att_pitch);
		break;
	}
	default:
		break;
	}
#else
	(void)c;
#endif
}

#endif /* CONFIG_IL_MAVLINK */
