# Indoor_localization

Indoor positioning of a small drone / toy car with **Bluetooth Channel Sounding** (CS) on
Seeed Studio **XIAO nRF54L15** boards (nRF Connect SDK v3.4.2, Zephyr).

Three fixed beacons and one mobile board. The mobile board measures its distance to every
beacon with CS, computes its 2D position by trilateration and prints it on the UART. The
position is meant to be sent to the flight controller as MAVLink.

```
   IL_B3                       mobile board (drone)
     \                         - ranges to B1, B2, B3 one at a time (CS)
      \        drone           - trilateration -> $POS x, y
       \        o              - UART -> laptop / flight controller
        \
   IL_B1 ------------ IL_B2
```

## Repository layout

| Folder | What it is |
|---|---|
| [`cs/cs_multi_initiator`](cs/cs_multi_initiator/README.md) | **Main firmware.** Mobile node (drone) and, with `beacon.conf`, the beacons. Beacons measure the distances between each other at boot, so the coordinate plane builds itself. Prints `$RNG` (distance per beacon) and `$POS` (x, y). |
| [`cs/cs_initiator`](cs/cs_initiator/README.md) | Single-beacon CS initiator (Nordic `ras_initiator` + our calibration). Where the calibration constants come from. |
| [`cs/cs_reflector`](cs/cs_reflector/README.md) | Older beacon firmware (Nordic `ras_reflector`, only changed to advertise as `IL_B<id>`). Does not measure the other beacons. |
| `trilateration/` | Position math shared by all targets: 2D / 3D trilateration, Kalman filter (`kalman.hpp`, not wired in yet). |
| `mavlink/` | Packs the position into MAVLink (`VISION_POSITION_ESTIMATE`, heartbeat); `test_mavlink_packer.cpp` runs on a PC. |
| `lib/mavlink` | MAVLink C library (git submodule, header-only). |
| `uart/` | UART sender used to send the MAVLink packets. |
| `uart_test/` | Small Zephyr app that tests the UART sender on hardware. |
| `CMakeLists.txt` (root) | Zephyr app that builds trilateration + MAVLink + UART together. |
| `nRF54L15_files/` | Early test app (trilateration on the board). |
| `tools/laptop_receiver.py` | Reads the board's serial port on a laptop (`pip install -r tools/requirements.txt`). |
| `tools/map/` | Live map in the browser (laptop / Raspberry Pi server, open on a tablet): `python tools/map/map_server.py`. |

After cloning, fetch the submodule:

    git submodule update --init

## Quick start (CS localization)

Requirements: nRF Connect SDK **v3.4.2** with its toolchain, board target
`xiao_nrf54l15/nrf54l15/cpuapp`, and **pyOCD** for flashing (`west flash` with the Seeed
OpenOCD runner can leave the image incomplete; see the header of `cs/cs_multi_initiator/flash.ps1`).

From an "nRF Connect SDK v3.4.2" terminal in `cs/cs_multi_initiator`:

    # beacons, one per board (id 1, 2, 3)
    west build -b xiao_nrf54l15/nrf54l15/cpuapp -d build_b1 --pristine -- -DEXTRA_CONF_FILE=beacon.conf -DCONFIG_IL_BEACON_ID=1

    # mobile node
    west build -b xiao_nrf54l15/nrf54l15/cpuapp --pristine

    # flash
    .\flash.ps1 -List
    .\flash.ps1 -Probe <ID> -Hex build_b1\cs_multi_initiator\zephyr\zephyr.hex   # beacon 1
    .\flash.ps1 -Probe <ID>                                                       # mobile node

Place the beacons as a triangle (IL_B1 → IL_B2 → IL_B3 counter-clockwise seen from above),
reset the beacons, then the mobile node. Read its output with a serial terminal at 115200 baud,
for example `python -m serial.tools.miniterm COM24 115200`.

Message formats, the coordinate plane and all options:
[`cs/cs_multi_initiator/README.md`](cs/cs_multi_initiator/README.md).

## Status (November 2026)

- Ranging to 3 beacons and the self-built coordinate plane work end to end on hardware.
- **Accuracy is not good enough yet:** beacon-to-beacon distances are off by up to 0.8 m at
  1.4–3.8 m, and drone-to-beacon distances are 0.5–1 m too long. Next step is recalibrating
  the distance at known distances of 1–4 m.
- The CS firmware sends the position as MAVLink on pins D6/D7 (`VISION_POSITION_ESTIMATE` +
  heartbeat, see `cs/cs_multi_initiator/README.md`); not tested with a flight controller yet.
- Beacon-to-beacon distances also depend strongly on where the boards stand (reflections):
  at some spots the error was ~0.8 m with a small spread. Calibration with the boards fixed
  in one orientation is in progress.
