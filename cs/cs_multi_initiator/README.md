# cs_multi_initiator (Seeed XIAO nRF54L15, NCS 3.4.2)

Mobile node of the indoor localization system (tasks **NRF-02**, **NRF-04** + 2D position).
Connects to several Channel Sounding beacons at once and measures the distance to them
**one at a time** (round-robin), then prints one measurement line per beacon and the computed X/Y on the UART.

Based on `../cs_initiator` (Nordic `ras_initiator` sample + our calibration).
The **beacon** firmware is built from the same sources (`CONFIG_IL_BEACON_ID` = 1..4, see
Build); `../cs_reflector` is the older beacon that does not measure the other beacons.

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
4. The LED is on as soon as the board runs and blinks (1 Hz) when all beacons are connected
   and set up (see "Status LED" below).

5. Every result of a turn goes through the distance filter `src/cs_pos.c`: a single spike
   (more than 0.3 m from the median of the beacon's last 5 accepted distances) is dropped,
   two consistent jumps in a row are taken as real movement, and after a timeout the last
   distance is kept for up to 10 s, so one missed turn does not lose the position.
6. After every round: 2D position from the filtered distances with Sofia's trilateration
   (`../../trilateration`, `Trilateration::calculate_2d_position`, used as is through
   `src/trilat_bridge.cpp`). The 3D CS distance is first reduced to the floor plane with the
   beacon/mobile heights. The `$RNG` lines stay unfiltered (one per turn).
7. MAVLink (`CONFIG_IL_MAVLINK`, on by default for the mobile node): HEARTBEAT once a second
   and VISION_POSITION_ESTIMATE after every position, MAVLink v2 on pins D6 (TX) / D7 (RX),
   115200 baud, made by `../../mavlink/mavlink_packer.cpp` (`src/mavlink_bridge.cpp`). The text
   output stays on USB. Needs the submodule: `git submodule update --init lib/mavlink`.
   Before connecting a flight controller, check in the packer: the y axis (only z is
   negated for NED, our plane is z up / y counter-clockwise), the system / component id of the
   heartbeat (now the same as an autopilot), and the rate (one position per round, ~4-5 s).

## Beacon layout and coordinate plane
The coordinate plane is built from the distances **between** the beacons:

- `IL_B1` = (0, 0), the origin;
- `IL_B2` = (D12, 0), so the x axis goes from `IL_B1` through `IL_B2`;
- `IL_B3` = (x3, y3) with y3 > 0.

Seen from above, `IL_B1` → `IL_B2` → `IL_B3` must go counter-clockwise, otherwise y is mirrored.

**Automatic (default, `CONFIG_IL_AUTO_LAYOUT=y`).** At boot every beacon measures the distance
to each beacon with a higher id (B1 → B2, B3; B2 → B3), one at a time, LED blinking. The
distances to one peer are pooled over several separate connections (one per ranging turn,
up to 40); it stops when it has 20 with a spread (interquartile range) of at most 0.3 m, and
the result is the median. It then puts the results in its advertisement (manufacturer data,
company id 0xFFFF, entries `<peer id><distance mm, 2 bytes LE>`, see `src/il_beacon.h`) and
becomes a normal CS reflector. The mobile node only scans until it has all distances, logs
`Beacon distances: ...` and the beacon coordinates, and then connects. While it waits it logs
every 5 s which pairs are still missing and which beacons it hears. Start order: reset the
beacons, then the mobile node. After moving a beacon, reset all boards.

One bad link does not block the system forever:
- a peer that was connected 4 times without enough distances is given up;
- after 4 minutes the beacon advertises what it has measured (so the lower-id beacons can
  measure it), and if a distance is missing it reboots 90 s later to measure again;
- if the measurement hangs, a timer reboots the beacon;
- a reflector checks every 2 s that it advertises (a single failed restart after a
  disconnect used to leave a beacon invisible until it was power-cycled).

**Manual (`-DCONFIG_IL_AUTO_LAYOUT=n`, mobile node only).** The distances are
`BEACON_D12_MM`, `BEACON_D13_MM`, `BEACON_D23_MM` at the top of `src/main.c` (tape measure).

`beacon_z_mm` = antenna height of each beacon, `MOBILE_Z_MM` = antenna height of the mobile
board (both 0 now: everything on one level).

## Position message (SYS-03)
Once per round:

    $POS,<seq>,<t_ms>,<x_mm>,<y_mm>,<z_mm>,<status>*<XX>

`z_mm` is 0 for the 2D demo. `status`: `0` OK, `1` fewer than 3 fresh distances,
`2` beacons on one line, the beacon distances do not form a triangle, or the calculation
failed (then x, y are 0).

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

With `-DCONFIG_IL_RAW_OUTPUT=y` every OK `$RNG` line is followed by the uncalibrated
estimates, for calibration (`-1` = not available):

    $RAW,<seq of the $RNG line>,<beacon_id>,<ifft_mm>,<phase_slope_mm>,<rtt_mm>*<XX>

Normal Zephyr log lines (`I: ...`) are printed on the same UART; a parser should only take
lines that start with `$RNG` / `$POS`.

## Build
Beacons, in this folder, one build folder per board (change the id):

    west build -b xiao_nrf54l15/nrf54l15/cpuapp -d build_b1 --pristine -- -DEXTRA_CONF_FILE=beacon.conf -DCONFIG_IL_BEACON_ID=1

Output: `build_b1\cs_multi_initiator\zephyr\zephyr.hex`.

A beacon build has no serial output (`CONFIG_SERIAL=n` in `beacon.conf`), so the beacons can
run on a battery without USB. Check a beacon by its LED (see "Status LED" below). To see its
log for debugging (on USB only, not on the battery), add `log_on.conf`:

    west build -b xiao_nrf54l15/nrf54l15/cpuapp -d b1log --pristine -- "-DEXTRA_CONF_FILE=beacon.conf;log_on.conf" -DCONFIG_IL_BEACON_ID=1

### Status LED

The user LED is on as soon as the firmware runs, so a dark LED means no power (or no firmware).

| Board  | LED                              | Meaning                                         |
|--------|----------------------------------|-------------------------------------------------|
| both   | steady on                        | running; beacon: advertising, waiting           |
| beacon | fast blink (5 Hz)                | measuring the other beacons at boot             |
| beacon | short flash once a second        | something is connected (the mobile node, or a beacon measuring it) |
| mobile | blink (1 Hz)                     | all beacons connected and set up                |

The charge LED next to it is driven by the charger chip, not by the firmware: it is only lit
while the battery charges from USB.

### Sleep button

A button between **D0** and **GND** (pin set in `boards/xiao_nrf54l15_nrf54l15_cpuapp.overlay`).
Press and release: the LED goes off and the board sleeps (System OFF, a few uA). Press again:
the board wakes up and starts from the beginning, like after power-on (a beacon measures the
other beacons again). Works on every build (beacons and mobile node); without a button nothing
happens. Turn it off with `-DCONFIG_IL_SLEEP_BUTTON=n`.

Mobile node:

    west build -b xiao_nrf54l15/nrf54l15/cpuapp --pristine

Mobile node on the drone battery (no USB): the output goes to the pins instead of the USB port,
D6 = TX, D7 = RX, 115200 baud, 3.3 V logic (see `uart_pins.overlay`):

    west build -b xiao_nrf54l15/nrf54l15/cpuapp --pristine -- -DEXTRA_DTC_OVERLAY_FILE=uart_pins.overlay

Wiring: D6 (TX) -> RX of the Raspberry Pi / flight controller, D7 (RX) -> its TX, GND -> GND.
With this build nothing comes out of the USB port; flashing over USB still works.

Antenna: by default the boards use the **external** FPC antenna on the U.FL connector
(`CONFIG_IL_EXTERNAL_ANTENNA=y`, see `src/antenna.c`). Plug the antenna in before powering the
board. For the onboard ceramic antenna add `-DCONFIG_IL_EXTERNAL_ANTENNA=n`. Use the same
antenna on all boards and recalibrate after switching (the constants were fitted with onboard antennas).

To use 4 beacons later: add `-- -DCONFIG_IL_NUM_BEACONS=4` (or change the default in `Kconfig`).

Output: `build\cs_multi_initiator\zephyr\zephyr.hex`

Distance estimation uses a 2048-point FFT (`CONFIG_BT_CS_DE_2048_NFFT`): with 512 points the
distance came in steps of about 22.5 cm.

## Flash
Same as `../cs_initiator`: use pyOCD (`.\flash.ps1 -List`, then `.\flash.ps1 -Probe <ID>`),
not `west flash`. For a beacon pass its hex:
`.\flash.ps1 -Probe <ID> -Hex build_b1\cs_multi_initiator\zephyr\zephyr.hex`.

## Status
Compiles on NCS 3.4.2 for `xiao_nrf54l15/nrf54l15/cpuapp` (mobile node and beacons).
Tested on hardware with 3 beacons on 16.11.2026:
- the automatic layout works end to end; at short range (sides 0.6–1.1 m) `$POS` gave status 0;
- **the beacon-to-beacon distances are not accurate enough yet**: with sides of about
  1.4 / 3.8 / 3.8 m they came out as 1147 / 3019 / 4223 mm (errors up to 0.8 m, not a
  triangle), so there was no position. Drone-to-beacon distances are stable (±5 cm on a good
  link) but 0.5–1 m too long. Next step: recalibrate with `CONFIG_IL_RAW_OUTPUT=y` at known
  distances of 1–4 m (the constants were fitted with the 512-point FFT at short range);
- once `SoftDevice Controller ASSERT: 135, 301` (controller crash, the board reboots) after
  ~110 s, while switching CS from B1 to B2; cause unknown.

Multi-link scheduling notes (all subevents aborted, subevent abort reason `0x3`):
- each beacon gets its own connection interval (40/45/50/55 ms for B1..B4), so the
  ACL events of the links drift against each other instead of always colliding with
  the CS subevents of the active beacon;
- CS subevent length 6 ms, `CONFIG_BT_CTLR_SDC_CENTRAL_ACL_EVENT_SPACING_DEFAULT=13000`.

If one beacon is never found again after the mobile node restarts, check its power
supply (a power bank may switch off at the low current of the board).
