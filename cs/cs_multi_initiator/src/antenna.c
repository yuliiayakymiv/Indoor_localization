/*
 * Antenna selection on the Seeed XIAO nRF54L15.
 *
 * The 2.4 GHz path goes through an RF switch:
 *   P2.03 (rfsw_pwr) - powers the switch; without power NO antenna is connected
 *   P2.05 (rfsw_ctl) - physical LOW = onboard ceramic antenna, HIGH = U.FL (external)
 *
 * The pins are driven by their physical level on purpose: the Zephyr and Seeed board
 * files disagree on the polarity flag of rfsw_ctl, so the logical value is not reliable.
 * Runs at APPLICATION level, after the board's regulator setup, before main().
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(antenna, LOG_LEVEL_INF);

#define RFSW_PWR_PIN 3
#define RFSW_CTL_PIN 5

static int antenna_select(void)
{
	const struct device *gpio2 = DEVICE_DT_GET(DT_NODELABEL(gpio2));

	if (!device_is_ready(gpio2)) {
		LOG_ERR("gpio2 not ready, antenna not switched");
		return -ENODEV;
	}

	int err = gpio_pin_configure(gpio2, RFSW_PWR_PIN, GPIO_OUTPUT_HIGH);

	if (!err) {
		err = gpio_pin_configure(gpio2, RFSW_CTL_PIN,
					 IS_ENABLED(CONFIG_IL_EXTERNAL_ANTENNA) ? GPIO_OUTPUT_HIGH
										: GPIO_OUTPUT_LOW);
	}
	if (err) {
		LOG_ERR("RF switch GPIO error %d", err);
		return err;
	}

	LOG_INF("Antenna: %s", IS_ENABLED(CONFIG_IL_EXTERNAL_ANTENNA) ? "external (U.FL)"
								      : "onboard");
	return 0;
}

SYS_INIT(antenna_select, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
