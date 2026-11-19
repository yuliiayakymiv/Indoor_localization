/*
 * Bridge between main.c and the team's MAVLink packer (../../../mavlink/mavlink_packer.cpp,
 * used as is). See mavlink_bridge.h.
 */

#include "mavlink_bridge.h"

#if defined(CONFIG_IL_MAVLINK)

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include "mavlink_packer.hpp"

LOG_MODULE_REGISTER(il_mavlink, LOG_LEVEL_INF);

/* The UART for MAVLink: the "il,mavlink-uart" chosen node if an overlay sets one,
 * otherwise uart21 (XIAO pins D6 = TX, D7 = RX).
 */
#if DT_HAS_CHOSEN(il_mavlink_uart)
#define IL_MAVLINK_UART DT_CHOSEN(il_mavlink_uart)
#else
#define IL_MAVLINK_UART DT_NODELABEL(uart21)
#endif

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

extern "C" void il_mavlink_send_position(float x_m, float y_m, float z_m)
{
	if (!ready) {
		return;
	}

	uint8_t buf[MavlinkPacker::MAVLINK_PACKET_MAX];
	const Point3D p = {x_m, y_m, z_m};
	uint64_t time_usec = (uint64_t)k_uptime_get() * 1000ULL; /* time since boot */

	send(buf, MavlinkPacker::pack_vision_position(p, time_usec, buf));
}

#endif /* CONFIG_IL_MAVLINK */
