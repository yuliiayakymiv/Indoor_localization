#include <zephyr/kernel.h>
#include "uart_sender.hpp"

int main(void)
{
    if (!UartSender::init()) {
        return 1;
    }

    const uint8_t test_data[] = {0xFD, 0x01, 0x02, 0x03};

    while (true) {
        if (UartSender::send(test_data, sizeof(test_data))) {
            UartSender::blink_led();
        }

        k_msleep(100);
    }

    return 0;
}