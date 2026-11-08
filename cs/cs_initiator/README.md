# channel_sounding_ras_initiator (Seeed XIAO nRF54L15, NCS 3.4.2)

BLE Channel Sounding **initiator** (ranging requestor). It scans for a RAS reflector, connects,
runs Channel Sounding and prints distance estimates over the UART.

Based on Nordic's `samples/bluetooth/channel_sounding/ras_initiator` (NCS v3.4.2).
Only change vs. the stock sample: `distance_estimates_print()` in `src/main.c` applies our
linear calibration (`PHASE_*`, `RTT_*`, `IFFT_*` constants at the top of the file) and prints
`[AP0] IFFT:..m Phase:..m RTT:..m -> Best:..m`.

Pair it with `../channel_sounding_ras_reflector` on a second board.

## Requirements
- nRF Connect SDK **v3.4.2** and its toolchain (nRF Connect extension -> Manage SDKs).
- Board target: `xiao_nrf54l15/nrf54l15/cpuapp`. This board is built into NCS 3.4.2.
- **Do not set a board root** (no `BOARD_ROOT`, no `west config build.board_root`, no
  `-DBOARD_ROOT`). Pointing at an external `platform-seeedboards` folder makes the build fail with
  "Board(s): {'xiao_nrf54l15'}, defined multiple times".

## Build
From an "nRF Connect SDK v3.4.2" terminal, in this folder:

    west build -b xiao_nrf54l15/nrf54l15/cpuapp --pristine

(or in VS Code: add this folder as an application, board `xiao_nrf54l15/nrf54l15/cpuapp`,
SDK/toolchain 3.4.2, use sysbuild, leave "Board root" empty).

Output: `build\channel_sounding_ras_initiator\zephyr\zephyr.hex`

If a build ever behaves strangely after changing SDK versions or settings, delete the `build`
folder and build again.

## Flash (use pyOCD, not `west flash`)
See the header of `flash.ps1` for the one-time pyOCD install. Then:

    .\flash.ps1 -List
    .\flash.ps1 -Probe <ID>

The OpenOCD runner that `west flash` uses for this board can leave the end of the image
unprogrammed, which shows up as `MPU FAULT, MMFAR Address: 0x9` right after the SoftDevice
Controller banner. If you ever see that, re-flash with pyOCD (use `-Erase` once).

If flashing fails with "Examination failed / Failed to read memory at 0xe000ed00": unplug the
board and any battery, wait 10 s, replug directly into the PC (no hub/carrier), then
`.\flash.ps1 -Probe <ID> -Erase`.

## Run
Serial terminal at 115200 baud on the board's COM port. Flash the reflector on a second board,
reset both, keep them ~1 m apart. Expected log after boot: connection, CS config, then
`[AP0] ... Best: ..m` lines.

## Calibration
Log line: `[AP0] Distance: X m  (raw IFFT:.. Phase:.. RTT:..)`. `Distance` is the calibrated value; the
raw numbers are the uncalibrated estimates, kept so the calibration can be redone.

Constants are at the top of `src/main.c` (`*_SLOPE`, `*_OFFSET_M`, `*_WEIGHT`), distance =
(raw - offset) / slope. Provisional fit on NCS 3.4.2 (two XIAO boards, onboard antennas; distances were
not tape-measured exactly):

| True distance (m) | IFFT raw | Phase raw | RTT raw |
|---|---|---|---|
| 0.500 | 1.75 | 2.47 | 5.23 |
| 0.856 | 1.84 | 2.17 | 5.93 |
| 1.000 | 1.75 | 3.13 | 5.86 |
| 1.317 | 2.05 | 3.19 | 6.20 |
| 1.500 | 2.63 | 6.99 | 8.91 |
| 2.000 | 3.80 | 6.34 | 9.21 |
| 2.567 | 3.50 | 6.67 | 10.45 |
| 3.523 | 5.56 | 7.20 | 11.75 |

IFFT and RTT are averaged (weight 0.5 each): about 0.25 m rms error with leave-one-out validation
(max about 0.4 m). Phase is not monotonic at short range, so its weight is 0.0.
The fit is specific to this hardware and indoor environment. To improve it: measure with a tape,
more distances (also beyond 4 m), several runs per distance in different positions, wait ~10 s after
moving the boards, same antenna orientation, away from walls/floor/bodies, then re-fit.

## Layout
- `src/main.c` - application
- `prj.conf`, `Kconfig`, `Kconfig.sysbuild`, `sysbuild/` - stock Nordic configuration
- `boards/` - stock overlays for Nordic boards (XIAO needs none)
