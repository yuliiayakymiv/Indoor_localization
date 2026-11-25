/*
 * Mobile node: UART receive side (text commands on the console, MAVLink from the flight
 * controller). See il_cmd.h.
 */

#include "il_cmd.h"

#if defined(CONFIG_IL_COMMANDS)

#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include "mavlink_bridge.h"

LOG_MODULE_REGISTER(il_cmd, LOG_LEVEL_INF);

#define CONSOLE_NODE DT_CHOSEN(zephyr_console)

#if defined(CONFIG_IL_LASER_FROM_FC)
#define HAVE_MAVLINK_RX 1
#define MAVLINK_SHARES_CONSOLE DT_SAME_NODE(IL_MAVLINK_UART, CONSOLE_NODE)
#else
#define HAVE_MAVLINK_RX 0
#define MAVLINK_SHARES_CONSOLE 0
#endif

struct rx_port {
	const struct device *dev;
	struct ring_buf rb;
	uint8_t buf[512];
	uint32_t overruns;
};

static struct rx_port console_rx = {.dev = DEVICE_DT_GET(CONSOLE_NODE)};
#if HAVE_MAVLINK_RX && !MAVLINK_SHARES_CONSOLE
static struct rx_port mavlink_rx = {.dev = DEVICE_DT_GET(IL_MAVLINK_UART)};
#endif

static K_SEM_DEFINE(rx_sem, 0, 1);

static void uart_isr(const struct device *dev, void *user_data)
{
	struct rx_port *port = user_data;
	uint8_t tmp[32];

	if (!uart_irq_update(dev)) {
		return;
	}
	while (uart_irq_rx_ready(dev)) {
		int n = uart_fifo_read(dev, tmp, sizeof(tmp));

		if (n <= 0) {
			break;
		}
		if (ring_buf_put(&port->rb, tmp, n) < (uint32_t)n) {
			port->overruns++;
		}
	}
	k_sem_give(&rx_sem);
}

static bool start_port(struct rx_port *port)
{
	if (!device_is_ready(port->dev)) {
		LOG_ERR("UART %s not ready, no commands from it", port->dev->name);
		return false;
	}
	ring_buf_init(&port->rb, sizeof(port->buf), port->buf);
	if (uart_irq_callback_user_data_set(port->dev, uart_isr, port) != 0) {
		LOG_ERR("UART %s: no interrupt-driven RX", port->dev->name);
		return false;
	}
	uart_irq_rx_enable(port->dev);
	return true;
}

/* ---- text commands ---- */

static char line[96];
static size_t line_len;

static bool checksum_ok(const char *msg)
{
	/* msg starts after '$'; "*XX" at the end is optional */
	const char *star = strchr(msg, '*');

	if (star == NULL) {
		return true;
	}
	uint8_t cs = 0;

	for (const char *p = msg; p < star; p++) {
		cs ^= (uint8_t)*p;
	}
	return strtoul(star + 1, NULL, 16) == cs;
}

static void handle_line(char *s)
{
	char *start = strrchr(s, '$');

	if (start == NULL) {
		return;
	}
	start++;
	if (!checksum_ok(start)) {
		LOG_WRN("Command with a wrong checksum ignored");
		return;
	}
	char *star = strchr(start, '*');

	if (star) {
		*star = '\0';
	}

	if (strncmp(start, "MODE,", 5) == 0) {
		const char *m = start + 5;

		if (strcmp(m, "CAR") == 0 || strcmp(m, "car") == 0) {
			il_on_mode_command(IL_MODE_CAR);
		} else if (strcmp(m, "DRONE") == 0 || strcmp(m, "drone") == 0) {
			il_on_mode_command(IL_MODE_DRONE);
		} else {
			LOG_WRN("Unknown mode '%s' (CAR or DRONE)", m);
		}
	} else if (strncmp(start, "LSR,", 4) == 0) {
		char *p = start + 4;
		long range_mm = strtol(p, &p, 10);
		long roll_cdeg = (*p == ',') ? strtol(p + 1, &p, 10) : 0;
		long pitch_cdeg = (*p == ',') ? strtol(p + 1, &p, 10) : 0;
		const float cdeg_to_rad = 3.14159265f / 18000.0f;

		il_on_laser_sample(range_mm / 1000.0f, roll_cdeg * cdeg_to_rad,
				   pitch_cdeg * cdeg_to_rad);
	}
}

static void text_byte(uint8_t c)
{
	if (c == '\r' || c == '\n') {
		if (line_len > 0) {
			line[line_len] = '\0';
			handle_line(line);
			line_len = 0;
		}
		return;
	}
	if (c < 0x20 || c > 0x7e) {
		line_len = 0; /* binary (MAVLink on a shared UART): not a text line */
		return;
	}
	if (line_len < sizeof(line) - 1) {
		line[line_len++] = (char)c;
	} else {
		line_len = 0; /* too long: drop */
	}
}

static void rx_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	uint8_t tmp[64];

	while (true) {
		k_sem_take(&rx_sem, K_MSEC(200));

		uint32_t n;

		while ((n = ring_buf_get(&console_rx.rb, tmp, sizeof(tmp))) > 0) {
			for (uint32_t i = 0; i < n; i++) {
				text_byte(tmp[i]);
				if (MAVLINK_SHARES_CONSOLE) {
					il_mavlink_rx_byte(tmp[i]);
				}
			}
		}
#if HAVE_MAVLINK_RX && !MAVLINK_SHARES_CONSOLE
		while ((n = ring_buf_get(&mavlink_rx.rb, tmp, sizeof(tmp))) > 0) {
			for (uint32_t i = 0; i < n; i++) {
				il_mavlink_rx_byte(tmp[i]);
			}
		}
#endif
	}
}

K_THREAD_STACK_DEFINE(rx_stack, 2048);
static struct k_thread rx_thread_data;

void il_cmd_init(void)
{
	bool any = start_port(&console_rx);

#if HAVE_MAVLINK_RX && !MAVLINK_SHARES_CONSOLE
	any = start_port(&mavlink_rx) || any;
#endif
	if (!any) {
		return;
	}
	k_thread_create(&rx_thread_data, rx_stack, K_THREAD_STACK_SIZEOF(rx_stack), rx_thread,
			NULL, NULL, NULL, 6, 0, K_NO_WAIT);
	k_thread_name_set(&rx_thread_data, "il_rx");
	LOG_INF("Commands on %s: $MODE,CAR / $MODE,DRONE / $LSR,<mm>,<roll cdeg>,<pitch cdeg>",
		console_rx.dev->name);
}

#endif /* CONFIG_IL_COMMANDS */
