# LoRa Ra-02 Integration — XIAO nRF54L15

## Hardware

- MCU: Seeed Studio XIAO nRF54L15
- Radio: Ra-02 (SX1278, SPI)
- Framework: Zephyr
- SPI controller: `spi00` (enabled in board Devicetree)

## Wiring

| XIAO | nRF GPIO | Ra-02 |
|---|---|---|
| D8 | P2.01 | SCK |
| D9 | P2.04 | MISO |
| D10 | P2.02 | MOSI |
| D1 | P1.05 | NSS |
| D0 | P1.04 | RST |
| D2 | P1.06 | DIO0 |
| GND | — | GND |
| 3V3 | — | 3.3V |

SPI pin mapping verified in:

- `zephyr/boards/seeed/xiao_nrf54l15/xiao_nrf54l15-pinctrl.dtsi`
- `zephyr/boards/seeed/xiao_nrf54l15/seeed_xiao_connector.dtsi`

NSS, RST and DIO0 are proposed GPIO assignments; they still need firmware configuration.

## Firmware tasks

1. Configure Ra-02 as an SPI device in the Devicetree overlay.
2. Configure NSS, RST and DIO0.
3. Read SX1278 register `0x42` (expected value: `0x12`).
4. Implement LoRa packet transmission.
5. Connect BLE scanner output to LoRa transmission.

## Current status

- SPI pin mapping: verified.
- SPI controller: enabled.
- Physical SPI communication: not yet verified.
- LoRa transmission: not yet implemented.
- Existing BLE/RSSI firmware: preserve until integration is ready.

**Important:** Ra-02 requires stable 3.3 V power. Verify the regulator's current capacity. Connect the antenna before transmitting.