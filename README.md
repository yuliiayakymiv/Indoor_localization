# Indoor_localization

Indoor positioning of a small drone / toy car with **Bluetooth Channel Sounding** (CS) on
Seeed Studio **XIAO nRF54L15** boards (nRF Connect SDK v3.4.2, Zephyr).

Three fixed beacons and one mobile board. The beacons measure the distances between each
other at boot, so the coordinate plane builds itself (no beacon coordinates are entered by
hand). The mobile board measures its distance to every beacon, filters the distances,
computes its 2D position by trilateration and sends it out:

- as text (`$RNG`, `$POS`, `$BCN`) on USB, read by a Raspberry Pi that shows a live map on a
  tablet over Wi-Fi;
- as MAVLink (`HEARTBEAT`, `VISION_POSITION_ESTIMATE`) on UART pin D6, for the flight
  controller.

```
  beacons (power banks)          on the car / drone                     in your hand
 ┌───────────────────┐        ┌──────────────────────────────────┐
 │ IL_B1 IL_B2 IL_B3 │ ◄─CS─► │ mobile XIAO ──USB──► Raspberry Pi │ ──Wi-Fi──► tablet:
 └───────────────────┘        │      │ D6 (UART, MAVLink)         │            http://<Pi>:8000
                              │      └──► flight controller / Pi  │
                              └──────────────────────────────────┘
```

## Repository layout

| Folder | What it is |
|---|---|
| [`cs/cs_multi_initiator`](cs/cs_multi_initiator/README.md) | **Main firmware.** Mobile node and, with `beacon.conf`, the beacons. Message formats, the coordinate plane, filtering, MAVLink, all options. |
| [`cs/cs_initiator`](cs/cs_initiator/README.md) | Single-beacon CS initiator (Nordic `ras_initiator` + our calibration). Where the calibration constants come from. |
| [`cs/cs_reflector`](cs/cs_reflector/README.md) | Older beacon firmware (does not measure the other beacons). |
| `trilateration/` | Position math: 2D (used) / 3D trilateration, Kalman filter (`kalman.hpp`, not wired in yet). |
| `mavlink/` | Packs the position into MAVLink (`VISION_POSITION_ESTIMATE`, heartbeat); `test_mavlink_packer.cpp` runs on a PC. |
| `lib/mavlink` | MAVLink C library (git submodule, header-only). |
| `uart/`, `uart_test/` | UART sender and its hardware test app. |
| `CMakeLists.txt` (root) | Zephyr app that builds trilateration + MAVLink + UART together. |
| `nRF54L15_files/` | Early test app (trilateration on the board). |
| [`tools/map/`](tools/map/README.md) | **Live map** server (laptop or Raspberry Pi, opens on a tablet), systemd unit for the Pi, Pi setup. |
| `tools/mavlink_check.py` | Checks the MAVLink output of pin D6 (on the Pi's UART or a USB-UART adapter): finds the frames, verifies checksums, decodes them. |
| `tools/laptop_receiver.py` | Old serial reader from the RSSI stage. |

After cloning, fetch the submodule:

    git submodule update --init

## Build and flash

Requirements: nRF Connect SDK **v3.4.2** with its toolchain, board target
`xiao_nrf54l15/nrf54l15/cpuapp`, and **pyOCD** for flashing (`west flash` with the Seeed
OpenOCD runner can leave the image incomplete; see the header of `cs/cs_multi_initiator/flash.ps1`).
On Windows build from a path without spaces or non-ASCII characters (e.g. `C:\dev\...`).

From an "nRF Connect SDK v3.4.2" terminal in `cs/cs_multi_initiator`:

    # beacons, one per board (id 1, 2, 3)
    west build -b xiao_nrf54l15/nrf54l15/cpuapp -d build_b1 --pristine -- "-DEXTRA_CONF_FILE=beacon.conf" -DCONFIG_IL_BEACON_ID=1

    # mobile node
    west build -b xiao_nrf54l15/nrf54l15/cpuapp --pristine

    # flash
    .\flash.ps1 -List
    .\flash.ps1 -Probe <ID> -Hex build_b1\cs_multi_initiator\zephyr\zephyr.hex   # beacon 1
    .\flash.ps1 -Probe <ID>                                                       # mobile node

If pyOCD says "Target not detected", retry with `-f 1000000`; a beacon that only answers
(B3) sometimes needs a few tries or a reset-button press during the attempt.

## Running a test

1. **Raspberry Pi** (once): copy `tools/` and install the `il-map` service, see
   [`tools/map/README.md`](tools/map/README.md). Plug the mobile XIAO into the Pi by USB.
2. **Beacons:** a triangle with sides of **2–3 m**, close to equilateral, B1 → B2 → B3
   **counter-clockwise** seen from above (otherwise the map is mirrored). All on the **same
   height** (e.g. on boxes), antennas **vertical**, facing the middle of the triangle,
   0.5–1 m away from walls, metal and laptops, nothing between any two beacons.
3. Power the beacons on in the order **B3, B2, B1**, then (re)start the mobile node
   (re-plug its USB). The beacons measure each other for 1–4 minutes (LED blinks fast),
   then the mobile node connects to all three and starts ranging.
4. Open `http://<Pi IP>:8000` on the tablet (same Wi-Fi). The old text view is still on
   TCP port 5000.
5. On the car / drone: the mobile node's antenna **vertical and above** the Pi, the power
   bank and the frame. A shielded antenna shows as RSSI around -95 in the log and many
   timeouts.

## Status (19 November 2026)

Done and tested on hardware (tasks 1, 2, 3 in 2D, 5):

- Ranging to 3 beacons, self-built coordinate plane, distance filter, 2D position, live map
  on a tablet from a Raspberry Pi on the car, without a laptop.
- **Distance accuracy** (mobile node to beacon, antennas vertical, nobody close by):
  ±0.10–0.26 m at 1–3 m. Single spots with strong reflections gave up to +0.9 m; turning
  the car by 90° changed the distance by ~0.25 m; people or a laptop next to a board moved
  it by ~0.5 m.
- **Beacon layout** (beacon to beacon): with the beacons on equal boxes and a free line of
  sight the sides came out 0.22–0.35 m short (2.20 / 2.16 / 2.11 m for 2.50 / 2.38 / 2.46 m);
  on a bad placement one side was 1.2 m short.

MAVLink (task 4): HEARTBEAT and VISION_POSITION_ESTIMATE leave pin D6 and decode with a
valid checksum on the Pi's UART (`tools/mavlink_check.py`). Not tested with a flight
controller yet.

Known problems:

- After lost links the mobile node can get stuck with one beacon: `CS enable failed
  (err -13)`, `opcode 0x2094 status 0x0c` and many `Failed to allocate new reassembly
  buffer`. Re-plugging the mobile node clears it; the firmware should disable CS and retry
  instead.
- If two beacons cannot measure each other, the higher one keeps re-measuring (and is not
  visible) until it gives up after 4 minutes and reboots to retry; move the beacons or
  clear the line of sight.
- MAVLink over jumper wires to the Pi: part of the frames had bad checksums (likely a loose
  GND); re-seat the wires, expect `HEARTBEAT: 5 (1.0/s); bad CRC 0` per 5 s.
- One position per round (~4.5–5 s, 0.2 Hz) is slow for a flight controller's EKF.

Next (3D and drone): a 4th beacon at another height and `calculate_3d_position`; MAVLink
y axis / component id fixes and a test in Mission Planner; a higher update rate; the drone
demo (beacons at the room's edges: a vertical antenna is weak straight up).
