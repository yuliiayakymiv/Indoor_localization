/*
 * Sleep button (D0 to GND, see boards/xiao_nrf54l15_nrf54l15_cpuapp.overlay).
 *
 * Press: the LED goes off, the RF switch is powered down and the chip enters System OFF
 * (a few uA). Press again: the chip wakes up and starts from the beginning, like after
 * power-on (a beacon measures the other beacons again, the drone looks for the beacons).
 */
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/poweroff.h>
#include <dk_buttons_and_leds.h>

#include "led_status.h"

LOG_MODULE_REGISTER(sleep_button, LOG_LEVEL_INF);

#define USER_NODE DT_PATH(zephyr_user)

#if IS_ENABLED(CONFIG_IL_SLEEP_BUTTON) && DT_NODE_HAS_PROP(USER_NODE, sleep_button_gpios)

#define DEBOUNCE_MS 30

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(USER_NODE, sleep_button_gpios);
static struct gpio_callback button_cb;

static void rf_switch_off(void)
{
	const struct device *gpio2 = DEVICE_DT_GET(DT_NODELABEL(gpio2));

	/* P2.03 powers the RF switch, P2.05 selects the antenna (see antenna.c) */
	gpio_pin_configure(gpio2, 3, GPIO_OUTPUT_LOW);
	gpio_pin_configure(gpio2, 5, GPIO_OUTPUT_LOW);
}

static void sleep_work_handler(struct k_work *work)
{
	k_msleep(DEBOUNCE_MS);
	if (gpio_pin_get_dt(&button) != 1) {
		/* a glitch, not a press */
		gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
		return;
	}

	LOG_INF("Sleep button: going to sleep, press it again to wake up");
	led_status_set(LED_STATUS_OFF);
	dk_set_led_off(DK_LED1);

	/* Wait for the release, otherwise the still pressed button wakes the chip at once */
	while (gpio_pin_get_dt(&button) == 1) {
		k_msleep(10);
	}
	k_msleep(DEBOUNCE_MS);

	rf_switch_off();

	/* Wake-up source: the button pressed (pin low) */
	gpio_pin_interrupt_configure_dt(&button, GPIO_INT_LEVEL_ACTIVE);

	k_msleep(100); /* let the log go out */
	sys_poweroff();
}

static K_WORK_DEFINE(sleep_work, sleep_work_handler);

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	gpio_pin_interrupt_configure_dt(&button, GPIO_INT_DISABLE);
	k_work_submit(&sleep_work);
}

static int sleep_button_init(void)
{
	int err;

	if (!gpio_is_ready_dt(&button)) {
		LOG_ERR("Sleep button GPIO not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (err) {
		LOG_ERR("Sleep button configure error %d", err);
		return err;
	}

	gpio_init_callback(&button_cb, button_pressed, BIT(button.pin));
	gpio_add_callback_dt(&button, &button_cb);

	/* Edge, not level: a button still held after the wake-up does not send it back to sleep */
	return gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
}

SYS_INIT(sleep_button_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif
