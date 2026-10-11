#pragma once
#include <stdint.h>
#include <stddef.h>

namespace UartSender {
    bool init();
    bool send(const uint8_t* data, size_t length);
    void blink_led();
}
  