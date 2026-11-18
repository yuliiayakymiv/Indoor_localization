/*
 * Status LED patterns, driven from the system work queue. See led_status.h.
 */
#include <zephyr/kernel.h>
#include <dk_buttons_and_leds.h>

#include "led_status.h"

#define STATUS_LED DK_LED1

/* on time, off time in ms for each blinking status */
static const uint16_t pattern[][2] = {
	[LED_STATUS_ON] = {0, 0},
	[LED_STATUS_MEASURING] = {100, 100},
	[LED_STATUS_LINKED] = {500, 500},
	[LED_STATUS_CONNECTED] = {100, 900},
};

static atomic_t current = ATOMIC_INIT(LED_STATUS_ON);
static bool led_on;

static void led_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(led_work, led_work_handler);

static void led_work_handler(struct k_work *work)
{
	enum led_status s = (enum led_status)atomic_get(&current);

	if (s == LED_STATUS_ON) {
		led_on = true;
		dk_set_led_on(STATUS_LED);
		return;
	}

	led_on = !led_on;
	dk_set_led(STATUS_LED, led_on);
	k_work_reschedule(&led_work, K_MSEC(led_on ? pattern[s][0] : pattern[s][1]));
}

void led_status_set(enum led_status status)
{
	if (atomic_set(&current, status) == status) {
		return; /* no change, keep the running pattern */
	}
	k_work_reschedule(&led_work, K_NO_WAIT);
}

void led_status_init(void)
{
	atomic_set(&current, LED_STATUS_ON);
	k_work_reschedule(&led_work, K_NO_WAIT);
}
