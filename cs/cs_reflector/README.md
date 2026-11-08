# channel_sounding_ras_reflector (Seeed XIAO nRF54L15, NCS 3.4.2)

BLE Channel Sounding **reflector** (ranging responder). Advertises as "IL_B<id>" (see Build) and
answers the initiator (`../channel_sounding_ras_initiator`). This is Nordic's unmodified
`samples/bluetooth/channel_sounding/ras_reflector` from NCS v3.4.2; it needs no XIAO-specific
changes.

## Requirements
- nRF Connect SDK **v3.4.2** and its toolchain.
- Board target: `xiao_nrf54l15/nrf54l15/cpuapp` (built into NCS 3.4.2).
- **Do not set a board root** (no `BOARD_ROOT`, no `west config build.board_root`).

## Build
From an "nRF Connect SDK v3.4.2" terminal, in this folder. Each beacon board gets its own id
(advertised name `IL_B<id>`, used by `../cs_multi_initiator`):

    west build -b xiao_nrf54l15/nrf54l15/cpuapp --pristine -- -DCONFIG_IL_BEACON_ID=1

(Without `-DCONFIG_IL_BEACON_ID` the id is 1.)

Output: `build\channel_sounding_ras_reflector\zephyr\zephyr.hex`
(Delete the old `build` folder first if it came from NCS 3.1.0.)

## Flash (use pyOCD, not `west flash`)
See the header of `flash.ps1` for the one-time pyOCD install. With two boards connected, pass the
probe ID of the reflector board:

    .\flash.ps1 -List
    .\flash.ps1 -Probe <ID> -Erase

`west flash` (Seeed OpenOCD runner) can leave the end of the image unprogrammed and crash the
firmware at boot.

## Layout
- `src/main.c`, `prj.conf`, `Kconfig.sysbuild`, `sysbuild/`, `boards/`, `android_ranging.conf` - stock Nordic files
