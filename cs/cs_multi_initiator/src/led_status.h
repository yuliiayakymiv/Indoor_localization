/*
 * Status LED (the one user LED of the XIAO).
 *
 * The LED is on as soon as the board runs, so a dark LED means no power or no firmware.
 * Blinking patterns on top of that show what the board is doing.
 */
#pragma once

enum led_status {
	LED_STATUS_ON,        /* steady: running (beacon: waiting for the mobile node) */
	LED_STATUS_MEASURING, /* fast blink, 5 Hz: beacon measures the other beacons */
	LED_STATUS_LINKED,    /* blink, 1 Hz: drone with all beacons connected */
	LED_STATUS_CONNECTED, /* short flash once a second: beacon with a connection */
};

/* Call once after dk_leds_init(). Starts with LED_STATUS_ON. */
void led_status_init(void);
void led_status_set(enum led_status status);
