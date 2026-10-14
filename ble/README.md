# BLE (маяк + сканер)

- `beacon/` — прошивка плати-маяка. ID маяка: `BEACON_ID` у `beacon/src/main.c`.
- `ble/` — модуль сканера, підключається в прошивку машинки.
- `scanner/` — тестова прошивка: виводить знайдені маяки в лог.

Формат рядка: `BEACON_01,-62,123456\n` (ID, середній RSSI в dBm, час у мс), не частіше ніж раз на 500 мс.

## Збірка
```
west build -p always -b nrf54l15dk/nrf54l15/cpuapp beacon
west build -p always -b nrf54l15dk/nrf54l15/cpuapp scanner
west flash
```

## Підключення до прошивки машинки (з uart_sender)
CMakeLists.txt: додати `../ble/ble_scanner.c` у `target_sources` і `../ble` у `target_include_directories`.
prj.conf: `CONFIG_BT=y`, `CONFIG_BT_OBSERVER=y`.

```cpp
#include "ble_scanner.h"
#include "uart_sender.hpp"

static void on_beacon(uint8_t id, int8_t rssi, uint32_t t) {
    char line[32];
    int len = ble_scanner_format_line(line, sizeof(line), id, rssi, t);
    UartSender::send(reinterpret_cast<const uint8_t*>(line), len);
}

// у main():
UartSender::init();
ble_scanner_start(on_beacon, 500);
```
