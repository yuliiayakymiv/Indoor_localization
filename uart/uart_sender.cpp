#include "uart_sender.hpp"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>

namespace {

const struct device* uart_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#if DT_NODE_EXISTS(DT_ALIAS(led0))
const struct gpio_dt_spec led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif

}

namespace UartSender {

bool init() {
    if (!device_is_ready(uart_dev)) {
        return false;
    }

#if DT_NODE_EXISTS(DT_ALIAS(led0))
    if (!gpio_is_ready_dt(&led)) {
        return false;
    }

    if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE) < 0) {
        return false;
    }
#endif

    return true;
}

bool send(const uint8_t* data, size_t length) {
    if (data == nullptr || length == 0) {
        return false;
    }

    if (!device_is_ready(uart_dev)) {
        return false;
    }

    for (size_t i = 0; i < length; i++) {
        uart_poll_out(uart_dev, data[i]);
    }

    return true;
}

void blink_led() {
#if DT_NODE_EXISTS(DT_ALIAS(led0))
    gpio_pin_set_dt(&led, 1);
    k_msleep(30);
    gpio_pin_set_dt(&led, 0);
#endif
}

}
