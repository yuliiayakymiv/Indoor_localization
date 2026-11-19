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

Network notes:
- Windows asks whether Python may accept connections: allow it for private networks.
- Some university / public Wi-Fi networks block traffic between devices. Then use a phone
  hotspot, or the Raspberry Pi's own access point.
