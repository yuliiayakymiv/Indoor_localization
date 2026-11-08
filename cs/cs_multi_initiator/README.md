# cs_multi_initiator (Seeed XIAO nRF54L15, NCS 3.4.2)

Mobile node of the indoor localization system (tasks **NRF-02**, **NRF-04** + 2D position).
Connects to several Channel Sounding beacons at once and measures the distance to them
**one at a time** (round-robin), then prints one measurement line per beacon and the computed X/Y on the UART.

Based on `../cs_initiator` (Nordic `ras_initiator` sample + our calibration).
Beacons run `../cs_reflector`, each built with its own id.

## How it works
1. Scans for beacons advertising the name `IL_B1`, `IL_B2`, ... `IL_B<N>` and keeps one
   connection to each (N = `CONFIG_IL_NUM_BEACONS`, default 3). Waits up to ~15 s at
   startup, then starts ranging with whatever is connected and keeps trying to
   (re)connect missing beacons between rounds.
2. For each beacon in turn: enable CS → collect `CONFIG_IL_SAMPLES_PER_TURN` estimates
   (default 3) or stop after `CONFIG_IL_TURN_TIMEOUT_MS` (default 1500 ms) → disable CS.
   Only one CS session runs at a time (Nordic's recommendation: concurrent CS on several
   links is not reliably scheduled).
3. Distance = calibrated median of the last 5 estimates of that beacon
   (calibration constants are the ones from `../cs_initiator`).
4. The LED is on when all beacons are connected and set up.

5. After every round: 2D position with Sofia's trilateration (`../../trilateration`,
   `Trilateration::calculate_2d_position`, used as is through `src/trilat_bridge.cpp`).
   The 3D CS distance is first reduced to the floor plane with the beacon/mobile heights.

## Beacon coordinates
At the top of `src/main.c`, array `beacon_pos_mm` ({x, y, z} in mm, row 1 = `IL_B1`).
**The current values are temporary.** Put the real ones from SYS-04, rebuild and reflash
only the mobile board. `MOBILE_Z_MM` = antenna height of the mobile board.

## Position message (SYS-03)
Once per round:

    $POS,<seq>,<t_ms>,<x_mm>,<y_mm>,<z_mm>,<status>*<XX>

`z_mm` is 0 for the 2D demo. `status`: `0` OK, `1` fewer than 3 fresh distances,
`2` beacons on one line / calculation failed (then x, y are 0).

## Measurement message (NRF-02)
One line per beacon per round, NMEA-style:

    $RNG,<seq>,<t_ms>,<beacon_id>,<dist_mm>,<status>*<XX>

| Field | Meaning |
|---|---|
| `seq` | message counter (gaps = lost lines) |
| `t_ms` | time since boot of the mobile node, ms |
| `beacon_id` | 1..N (from the beacon name `IL_B<id>`) |
| `dist_mm` | calibrated distance in mm, `-1` if not valid |
| `status` | `0` OK, `1` CS ran but no usable estimate, `2` timeout (no result this turn), `3` beacon not connected |
| `XX` | checksum: XOR of all characters between `$` and `*`, 2 hex digits |

Example: `$RNG,42,15320,2,1875,0*4D`

Normal Zephyr log lines (`[00:00:01.234,567] <inf> app_main: ...`) are printed on the same
UART; a parser should only take lines that start with `$RNG`.

## Build
Beacons (one build per board, change the id):

    cd ../cs_reflector
    west build -b xiao_nrf54l15/nrf54l15/cpuapp --pristine -- -DCONFIG_IL_BEACON_ID=1

Mobile node:

    west build -b xiao_nrf54l15/nrf54l15/cpuapp --pristine

To use 4 beacons later: add `-- -DCONFIG_IL_NUM_BEACONS=4` (or change the default in `Kconfig`).

Output: `build\cs_multi_initiator\zephyr\zephyr.hex`

## Flash
Same as `../cs_initiator`: use pyOCD (`.\flash.ps1 -List`, then `.\flash.ps1 -Probe <ID>`),
not `west flash`.

## Status
Compiles on NCS 3.4.2 for `xiao_nrf54l15/nrf54l15/cpuapp`. **Not yet tested on hardware.**
