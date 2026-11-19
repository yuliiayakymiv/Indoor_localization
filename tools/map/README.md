# Live map

A small web server that reads the mobile node's serial output and shows the beacons and the
current position on a map. It runs on a laptop or a Raspberry Pi; the map opens in a browser
on any device in the same network (laptop, tablet, phone).

Only Python 3 and `pyserial` are needed (`pip install -r ../requirements.txt`).

    python map_server.py                       # finds the mobile node's port by itself
    python map_server.py --port COM24          # Windows
    python map_server.py --port /dev/ttyACM0   # Raspberry Pi
    python map_server.py --replay run.log      # play back a saved serial log
    python map_server.py --demo                # made-up data, no board needed

It prints the addresses to open, for example `http://192.168.1.23:8000`. Change the port with
`--http-port`.

What the map shows:
- beacons `B1..B3` at the coordinates the mobile node prints at boot (`Beacon IL_B<n> at x=...`)
  and repeats every 5 rounds (`$BCN`), so the map can be opened at any time;
- the current position from `$POS` (green; grey when there was no new position for 12 s)
  and its trail;
- a circle around each beacon with its last distance from `$RNG` (can be turned off);
- a banner when the beacon layout failed ("the distances do not form a triangle").

The server reads the text console, so the mobile node must print it on the port the server
opens (USB by default). Only one program can open a serial port: close other serial
terminals first. The beacons appear within ~25 s after the server starts (next `$BCN`).

## Raspberry Pi (on the car / drone)

The mobile node is plugged into the Pi by USB (`/dev/ttyACM0`, text + power). The Pi runs the
server at boot; the tablet opens `http://<Pi IP>:8000`. `--tcp 5000` also sends the same text
lines as the old `pi_receiver.py`, so a "Serial WiFi Terminal" app on `<Pi IP>:5000` still works
(only one program can read `/dev/ttyACM0`, so do not run `pi_receiver.py` next to it).

    scp -r tools pi@<Pi IP>:~/tools
    ssh pi@<Pi IP>
    sudo apt install -y python3-serial
    sudo cp ~/tools/map/il-map.service /etc/systemd/system/
    sudo systemctl daemon-reload && sudo systemctl enable --now il-map
    journalctl -u il-map -f          # its log

### MAVLink on the Pi's UART (pins 8/10), optional

To check the MAVLink output of the nRF on the Pi: XIAO D6 (TX) -> Pi pin 10 (GPIO15, RX),
XIAO GND -> Pi pin 6 (GND); then `python3 ~/tools/mavlink_check.py --port /dev/ttyAMA0`.

On a Pi 5 this UART must be enabled, and **the Linux console must be moved off it first**,
otherwise the bytes from the nRF arrive at a login console during boot and the Pi may not
come up (it did not join the Wi-Fi in our test):

    sudo cp /boot/firmware/cmdline.txt /boot/firmware/cmdline.txt.bak
    sudo sed -i 's/console=serial0,115200 //' /boot/firmware/cmdline.txt   # keep console=tty1
    sudo systemctl mask serial-getty@ttyAMA0.service serial-getty@serial0.service
    echo 'dtparam=uart0=on' | sudo tee -a /boot/firmware/config.txt
    sudo reboot

On a Pi 5 `/dev/serial0` stays on the separate debug connector (`ttyAMA10`); the pins are
`/dev/ttyAMA0`. To undo, restore `cmdline.txt` and remove the `dtparam=uart0=on` line (the SD
card's `bootfs` partition can be edited on any computer).

Network notes:
- Windows asks whether Python may accept connections: allow it for private networks.
- Some university / public Wi-Fi networks block traffic between devices. Then use a phone
  hotspot, or the Raspberry Pi's own access point.
