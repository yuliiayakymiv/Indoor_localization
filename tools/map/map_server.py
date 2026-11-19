#!/usr/bin/env python3
"""
Live map of the mobile node (drone / car) for the indoor localization demo.

Reads the text output of the mobile node (USB serial, 115200 baud) and serves a web page
with the beacons and the current position. Open it from any device in the same network
(laptop, tablet, phone): http://<ip of this computer>:8000

What it reads from the serial output:
    I: Beacon IL_B<n> at x=<mm> y=<mm> z=<mm> mm      beacon positions (built at boot)
    $BCN,<beacon>,<x_mm>,<y_mm>,<z_mm>*<XX>             the same, repeated every 5 rounds
    E: Beacon layout: <reason>                          layout failed (also repeated)
    $RNG,<seq>,<t_ms>,<beacon>,<dist_mm>,<status>*<XX>       distance to one beacon
    $POS,<seq>,<t_ms>,<x_mm>,<y_mm>,<z_mm>,<status>*<XX>     position
Lines with a wrong checksum are ignored. A "*** Booting" line (board restart) clears the map.

Usage:
    pip install pyserial
    python map_server.py                       # finds the board's serial port by itself
    python map_server.py --port COM24          # Windows
    python map_server.py --port /dev/ttyACM0   # Raspberry Pi
    python map_server.py --replay run.log      # play back a saved log (no board needed)
    python map_server.py --demo                # made-up data (no board needed)

Only one program can open the serial port at a time: close other serial terminals first.
"""

import argparse
import json
import math
import os
import queue
import random
import re
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
BAUD = 115200
TRAIL_LEN = 200

POS_STATUS = {0: "ok", 1: "мало маяків", 2: "погана геометрія / розкладка"}
RNG_STATUS = {0: "ok", 1: "немає оцінки", 2: "таймаут", 3: "не підключений"}

RE_BEACON = re.compile(r"Beacon IL_B(\d+) at x=(-?\d+) y=(-?\d+) z=(-?\d+) mm")
RE_LAYOUT_BAD = re.compile(r"Beacon layout: (.*)")
RE_DISTANCES = re.compile(r"Beacon distances: (.*)")


# --------------------------------------------------------------------------------------
# Shared state + broadcast to the browsers (Server-Sent Events)
# --------------------------------------------------------------------------------------

class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.clients = []          # one queue per open page
        self.reset()

    def reset(self):
        self.beacons = {}          # id -> {x, y, z} in mm
        self.layout_error = None
        self.layout_info = None
        self.pos = None            # last $POS
        self.trail = []            # [[x, y], ...] of OK positions
        self.ranges = {}           # id -> last $RNG
        self.source = ""
        self.last_line_time = None

    def snapshot(self):
        return {
            "type": "snapshot",
            "beacons": self.beacons,
            "layout_error": self.layout_error,
            "layout_info": self.layout_info,
            "pos": self.pos,
            "trail": self.trail,
            "ranges": self.ranges,
            "source": self.source,
            "server_time": time.time(),
        }

    def publish(self, msg):
        data = json.dumps(msg, ensure_ascii=False)
        with self.lock:
            for q in list(self.clients):
                try:
                    q.put_nowait(data)
                except queue.Full:
                    pass  # a stuck browser: skip it, it gets a snapshot on reconnect


STATE = State()


def nmea_ok(line):
    """True if the $...*XX checksum matches (XOR of everything between $ and *)."""
    if "*" not in line:
        return False
    body, cs = line[1:].rsplit("*", 1)
    try:
        want = int(cs[:2], 16)
    except ValueError:
        return False
    got = 0
    for ch in body:
        got ^= ord(ch)
    return got == want


def handle_line(line):
    line = line.strip()
    if not line:
        return
    STATE.last_line_time = time.time()

    if "*** Booting" in line:
        with STATE.lock:
            src = STATE.source
            STATE.reset()
            STATE.source = src
        STATE.publish({"type": "reset"})
        return

    if line.startswith("$"):
        if not nmea_ok(line):
            return
        f = line[1:].split("*", 1)[0].split(",")
        try:
            if f[0] == "POS" and len(f) >= 7:
                pos = {"seq": int(f[1]), "t_ms": int(f[2]), "x": int(f[3]), "y": int(f[4]),
                       "z": int(f[5]), "status": int(f[6]),
                       "status_text": POS_STATUS.get(int(f[6]), "?"), "rx": time.time()}
                with STATE.lock:
                    STATE.pos = pos
                    if pos["status"] == 0:
                        STATE.trail.append([pos["x"], pos["y"]])
                        del STATE.trail[:-TRAIL_LEN]
                STATE.publish({"type": "pos", "pos": pos})
            elif f[0] == "RNG" and len(f) >= 6:
                rng = {"seq": int(f[1]), "t_ms": int(f[2]), "beacon": int(f[3]),
                       "dist": int(f[4]), "status": int(f[5]),
                       "status_text": RNG_STATUS.get(int(f[5]), "?"), "rx": time.time()}
                with STATE.lock:
                    STATE.ranges[str(rng["beacon"])] = rng
                STATE.publish({"type": "rng", "rng": rng})
            elif f[0] == "BCN" and len(f) >= 5:
                bid = str(int(f[1]))
                beacon = {"x": int(f[2]), "y": int(f[3]), "z": int(f[4])}
                with STATE.lock:
                    changed = STATE.beacons.get(bid) != beacon or STATE.layout_error
                    STATE.beacons[bid] = beacon
                    STATE.layout_error = None
                if changed:
                    STATE.publish({"type": "beacon", "id": bid, "beacon": beacon})
        except (ValueError, IndexError):
            pass
        return

    m = RE_BEACON.search(line)
    if m:
        bid, x, y, z = m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4))
        with STATE.lock:
            STATE.beacons[bid] = {"x": x, "y": y, "z": z}
            STATE.layout_error = None
        STATE.publish({"type": "beacon", "id": bid, "beacon": STATE.beacons[bid]})
        return

    m = RE_DISTANCES.search(line)
    if m:
        with STATE.lock:
            STATE.layout_info = m.group(1)
        STATE.publish({"type": "layout_info", "text": m.group(1)})
        return

    m = RE_LAYOUT_BAD.search(line)
    if m:
        with STATE.lock:
            STATE.layout_error = m.group(1)
        STATE.publish({"type": "layout_error", "text": m.group(1)})


# --------------------------------------------------------------------------------------
# Data sources
# --------------------------------------------------------------------------------------

def find_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    seeed = [p for p in ports if p.vid == 0x2886 or "XIAO" in (p.description or "")
             or "CMSIS-DAP" in (p.description or "")]
    if len(seeed) == 1:
        return seeed[0].device
    print("Choose the port of the mobile node with --port. Serial ports found:")
    for p in ports:
        print(f"  {p.device:12} {p.description}")
    if len(seeed) > 1:
        print("Several XIAO boards are plugged in; the mobile node is the one that prints "
              "'multi-beacon CS initiator' at boot.")
    sys.exit(1)


def run_serial(port):
    import serial
    STATE.source = f"serial {port}"
    while True:
        try:
            with serial.Serial(port, BAUD, timeout=1) as ser:
                print(f"Reading {port} at {BAUD} baud")
                buf = b""
                while True:
                    chunk = ser.read(ser.in_waiting or 1)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        handle_line(raw.decode("utf-8", errors="replace"))
        except Exception as e:  # unplugged board, port busy, ...
            print(f"Serial port {port}: {e}. Retrying in 2 s")
            STATE.publish({"type": "error", "text": f"{port}: {e}"})
            time.sleep(2)


def run_replay(path, speed):
    STATE.source = f"replay {os.path.basename(path)}"
    while True:
        with open(path, encoding="utf-8", errors="replace") as fh:
            lines = fh.read().splitlines()
        last_t = None
        for line in lines:
            m = re.match(r"\$(POS|RNG),\d+,(\d+),", line)
            if m and speed > 0:
                t = int(m.group(2)) / 1000.0
                if last_t is not None and 0 < t - last_t < 10:
                    time.sleep((t - last_t) / speed)
                last_t = t
            handle_line(line)
        print("Replay finished, starting again in 3 s")
        time.sleep(3)
        handle_line("*** Booting (replay restart) ***")


def run_demo():
    STATE.source = "demo"

    def nmea(body):
        cs = 0
        for ch in body:
            cs ^= ord(ch)
        return f"${body}*{cs:02X}"

    beacons = {1: (0, 0), 2: (2500, 0), 3: (1478, 1575)}
    for bid, (x, y) in beacons.items():
        handle_line(f"I: Beacon IL_B{bid} at x={x} y={y} z=0 mm")
    seq = pos_seq = 0
    t0 = time.time()
    while True:
        t = time.time() - t0
        x = 1300 + 700 * math.cos(t / 6)
        y = 700 + 450 * math.sin(t / 4)
        for bid, (bx, by) in beacons.items():
            if random.random() < 0.15:
                handle_line(nmea(f"RNG,{seq},{int(t * 1000)},{bid},-1,2"))
            else:
                d = math.hypot(x - bx, y - by) + random.gauss(0, 60)
                handle_line(nmea(f"RNG,{seq},{int(t * 1000)},{bid},{int(d)},0"))
            seq += 1
            time.sleep(0.4)
        jx, jy = random.gauss(0, 50), random.gauss(0, 50)
        handle_line(nmea(f"POS,{pos_seq},{int(t * 1000)},{int(x + jx)},{int(y + jy)},0,0"))
        pos_seq += 1


# --------------------------------------------------------------------------------------
# HTTP server
# --------------------------------------------------------------------------------------

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # keep the console readable

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "index.html"), "rb") as fh:
                body = fh.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/state":
            with STATE.lock:
                body = json.dumps(STATE.snapshot(), ensure_ascii=False).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/events":
            self.events()
        else:
            self.send_error(404)

    def events(self):
        q = queue.Queue(maxsize=500)
        with STATE.lock:
            STATE.clients.append(q)
            first = json.dumps(STATE.snapshot(), ensure_ascii=False)
        try:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            self.wfile.write(f"data: {first}\n\n".encode())
            self.wfile.flush()
            while True:
                try:
                    data = q.get(timeout=15)
                    self.wfile.write(f"data: {data}\n\n".encode())
                except queue.Empty:
                    self.wfile.write(b": keepalive\n\n")  # keeps proxies / tablets awake
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass
        finally:
            with STATE.lock:
                if q in STATE.clients:
                    STATE.clients.remove(q)


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        if isinstance(sys.exc_info()[1], ConnectionError):
            return  # a browser closed the page / tablet went to sleep: nothing to report
        super().handle_error(request, client_address)


def lan_ips():
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))  # no packet is sent; only picks the LAN interface
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    return sorted(ip for ip in ips if not ip.startswith("127."))


def main():
    ap = argparse.ArgumentParser(description="Live map of the indoor localization mobile node")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--port", help="serial port of the mobile node (COM24, /dev/ttyACM0, ...)")
    src.add_argument("--replay", metavar="LOG", help="play back a saved serial log")
    src.add_argument("--demo", action="store_true", help="made-up data, no board needed")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed (0 = no delays)")
    ap.add_argument("--http-port", type=int, default=8000)
    args = ap.parse_args()
    sys.stdout.reconfigure(line_buffering=True)  # show the URLs at once, also when piped

    if args.demo:
        target, targs = run_demo, ()
    elif args.replay:
        target, targs = run_replay, (args.replay, args.speed)
    else:
        target, targs = run_serial, (args.port or find_port(),)
    threading.Thread(target=target, args=targs, daemon=True).start()

    server = Server(("0.0.0.0", args.http_port), Handler)
    print(f"Map: http://localhost:{args.http_port}")
    for ip in lan_ips():
        print(f"     http://{ip}:{args.http_port}   (from a tablet / phone in the same Wi-Fi)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
