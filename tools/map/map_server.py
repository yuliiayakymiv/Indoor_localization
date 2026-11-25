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
    $MOD,<CAR|DRONE>,<beacons used>*<XX>                     current mode
    $HGT,<t_ms>,<laser_h_mm>,<surface_mm>,<surface>,<z_mm>*<XX>   laser height (drone)
    $OBS,<seq>,<t_ms>,<x_mm>,<y_mm>,<h_mm>*<XX>              obstacle under the drone
Lines with a wrong checksum are ignored. A "*** Booting" line (board restart) clears the map
(the obstacles are kept: they are a map of the room, not of one run).

Mode (car / drone): the page has a switch; the server sends "$MODE,CAR" / "$MODE,DRONE" to
the board and repeats it if the board restarts with another mode. CAR ignores the 4th beacon.

Usage:
    pip install pyserial
    python map_server.py                       # finds the board's serial port by itself
    python map_server.py --port COM24          # Windows
    python map_server.py --port /dev/ttyACM0   # Raspberry Pi
    python map_server.py --replay run.log      # play back a saved log (no board needed)
    python map_server.py --demo                # made-up data (no board needed)
    python map_server.py --port /dev/ttyACM0 --tcp 5000   # + the text of pi_receiver.py on :5000

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
SURFACE = {0: "невідомо", 1: "підлога", 2: "перешкода"}
MAX_OBSTACLES = 2000
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
        self.obstacles = []        # [{x, y, h, t}] in mm; kept across board restarts
        self.desired_mode = None   # "car" / "drone" chosen on the page
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
        self.mode = None           # mode reported by the board ($MOD): "car" / "drone"
        self.mode_beacons = None   # beacons that mode uses
        self.height = None         # last $HGT

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
            "mode": self.mode,
            "mode_beacons": self.mode_beacons,
            "desired_mode": self.desired_mode,
            "height": self.height,
            "obstacles": self.obstacles,
            "can_send": SERIAL.connected() or DEMO_MODE[0] is not None,
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


class SerialOut:
    """The open serial port of the board, for the $MODE command (written from HTTP threads)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.ser = None
        self.last_mode_sent = 0.0

    def connected(self):
        return self.ser is not None

    def send(self, body):
        cs = 0
        for ch in body:
            cs ^= ord(ch)
        line = f"${body}*{cs:02X}\r\n".encode()
        with self.lock:
            if self.ser is None:
                return False
            try:
                self.ser.write(line)
                self.ser.flush()
                return True
            except Exception as e:  # unplugged
                print(f"Serial write failed: {e}")
                return False


SERIAL = SerialOut()
DEMO_MODE = [None]  # --demo: the mode of the simulated board


def request_mode(mode):
    """Page asked for a mode: remember it and send it to the board (or the demo)."""
    with STATE.lock:
        STATE.desired_mode = mode
    STATE.publish({"type": "desired_mode", "mode": mode})
    if DEMO_MODE[0] is not None:
        DEMO_MODE[0] = mode
        return True
    SERIAL.last_mode_sent = time.time()
    return SERIAL.send(f"MODE,{mode.upper()}")


def resend_mode_if_needed():
    """The board reports another mode than chosen (e.g. it restarted): ask again, every 3 s."""
    with STATE.lock:
        want, have = STATE.desired_mode, STATE.mode
    if want and have and want != have and time.time() - SERIAL.last_mode_sent > 3:
        SERIAL.last_mode_sent = time.time()
        SERIAL.send(f"MODE,{want.upper()}")


# --------------------------------------------------------------------------------------
# Optional text output on a TCP port (--tcp), same lines as pi_receiver.py --rng --raw,
# so a "Serial WiFi Terminal" on the tablet keeps working next to the map
# --------------------------------------------------------------------------------------

class TcpText:
    def __init__(self, port):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("0.0.0.0", port))
        self.server.listen(4)
        self.clients = []
        self.lock = threading.Lock()
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            client, _ = self.server.accept()
            client.setblocking(False)
            with self.lock:
                self.clients.append(client)

    def send(self, text):
        data = (text + "\r\n").encode()
        with self.lock:
            for client in self.clients[:]:
                try:
                    client.send(data)
                except BlockingIOError:
                    pass  # slow client: skip this line
                except OSError:
                    self.clients.remove(client)
                    client.close()


TCP = None  # TcpText when --tcp is given


def tcp_text(kind, rec=None, line=None):
    """Format like pi_receiver.py: positions, distances, and '# ' + other lines."""
    if TCP is None:
        return
    if kind == "pos":
        if rec["status"] != 0:
            TCP.send(f"X=   ---   Y=   ---   [{POS_STATUS_TCP.get(rec['status'], rec['status'])}]"
                     f"  t={rec['t_ms']} ms  #{rec['seq']}")
        else:
            TCP.send(f"X={rec['x']:7.1f}  Y={rec['y']:7.1f}  Z={rec['z']:5.1f}  "
                     f"t={rec['t_ms']} ms  #{rec['seq']}")
    elif kind == "rng":
        if rec["status"] != 0:
            TCP.send(f"    B{rec['beacon']}: [{RNG_STATUS.get(rec['status'], rec['status'])}]")
        else:
            TCP.send(f"    B{rec['beacon']}: {rec['dist'] / 1000:.2f} m")
    elif kind == "raw" and line:
        TCP.send(f"# {line}")


POS_STATUS_TCP = {0: "OK", 1: "мало маяків", 2: "погана геометрія"}  # pi_receiver wording


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
        tcp_text("raw", line=line)
        with STATE.lock:
            src = STATE.source
            STATE.reset()
            STATE.source = src
        STATE.publish({"type": "reset"})
        SERIAL.last_mode_sent = 0.0  # the board restarted: send the chosen mode at its $MOD
        return

    # A log line can be glued in front of a message: take the last '$'
    start = line.rfind("$")
    if start >= 0 and "*" in line[start:] and nmea_ok(line[start:]):
        msg = line[start:]
        f = msg[1:].split("*", 1)[0].split(",")
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
                tcp_text("pos", pos)
            elif f[0] == "RNG" and len(f) >= 6:
                rng = {"seq": int(f[1]), "t_ms": int(f[2]), "beacon": int(f[3]),
                       "dist": int(f[4]), "status": int(f[5]),
                       "status_text": RNG_STATUS.get(int(f[5]), "?"), "rx": time.time()}
                with STATE.lock:
                    STATE.ranges[str(rng["beacon"])] = rng
                STATE.publish({"type": "rng", "rng": rng})
                tcp_text("rng", rng)
            elif f[0] == "MOD" and len(f) >= 2:
                mode = f[1].lower()
                used = int(f[2]) if len(f) >= 3 and f[2] else None
                with STATE.lock:
                    changed = STATE.mode != mode or STATE.mode_beacons != used
                    STATE.mode, STATE.mode_beacons = mode, used
                    if mode == "car":
                        STATE.height = None
                if changed:
                    STATE.publish({"type": "mode", "mode": mode, "beacons": used})
                    tcp_text("raw", line=f"MODE {mode.upper()}")
                resend_mode_if_needed()
            elif f[0] == "HGT" and len(f) >= 6:
                hgt = {"t_ms": int(f[1]), "laser_h": int(f[2]), "surface_h": int(f[3]),
                       "surface": int(f[4]), "surface_text": SURFACE.get(int(f[4]), "?"),
                       "z": int(f[5]), "rx": time.time()}
                with STATE.lock:
                    STATE.height = hgt
                STATE.publish({"type": "height", "height": hgt})
            elif f[0] == "OBS" and len(f) >= 6:
                obs = {"x": int(f[3]), "y": int(f[4]), "h": int(f[5]), "t": time.time()}
                with STATE.lock:
                    STATE.obstacles.append(obs)
                    del STATE.obstacles[:-MAX_OBSTACLES]
                STATE.publish({"type": "obstacle", "obstacle": obs})
                tcp_text("raw", line=f"OBSTACLE h={obs['h'] / 1000:.2f} m at "
                                     f"x={obs['x'] / 1000:.2f} y={obs['y'] / 1000:.2f}")
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

    tcp_text("raw", line=line)  # log lines and broken messages, like pi_receiver --raw

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
                with SERIAL.lock:
                    SERIAL.ser = ser
                SERIAL.last_mode_sent = 0.0  # a (re)opened port: send the chosen mode again
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
            with SERIAL.lock:
                SERIAL.ser = None
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
            m = re.match(r"\$(POS|RNG|OBS),\d+,(\d+),", line)
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
    """Made-up board: 4 beacons (B4 high), CAR / DRONE switch, laser height and a box."""
    STATE.source = "demo"

    def nmea(body):
        cs = 0
        for ch in body:
            cs ^= ord(ch)
        return f"${body}*{cs:02X}"

    beacons = {1: (0, 0, 300), 2: (2500, 0, 300), 3: (1478, 1575, 300), 4: (1250, 600, 2300)}
    box = (900, 300, 1500, 800, 700)  # x0, y0, x1, y1, height (mm)
    for bid, (x, y, z) in beacons.items():
        handle_line(f"I: Beacon IL_B{bid} at x={x} y={y} z={z} mm")
    seq = pos_seq = obs_seq = 0
    t0 = time.time()
    mode_reported = None
    while True:
        mode = DEMO_MODE[0]
        if mode != mode_reported:
            used = 3 if mode == "car" else 4
            handle_line(nmea(f"MOD,{mode.upper()},{used}"))
            mode_reported = mode
        t = time.time() - t0
        x = 1300 + 700 * math.cos(t / 6)
        y = 700 + 450 * math.sin(t / 4)
        z = 100 if mode == "car" else 1200 + 200 * math.sin(t / 5)
        ids = [1, 2, 3] if mode == "car" else [1, 2, 3, 4]
        for bid in ids:
            bx, by, bz = beacons[bid]
            if random.random() < 0.1:
                handle_line(nmea(f"RNG,{seq},{int(t * 1000)},{bid},-1,2"))
            else:
                d = math.dist((x, y, z), (bx, by, bz)) + random.gauss(0, 60)
                handle_line(nmea(f"RNG,{seq},{int(t * 1000)},{bid},{int(d)},0"))
            seq += 1
            jx, jy = random.gauss(0, 50), random.gauss(0, 50)
            jz = 0 if mode == "car" else random.gauss(0, 20)
            handle_line(nmea(f"POS,{pos_seq},{int(t * 1000)},{int(x + jx)},{int(y + jy)},"
                             f"{int(z + jz)},0"))
            pos_seq += 1
            if mode == "drone":
                over = box[0] < x < box[2] and box[1] < y < box[3]
                below = box[4] if over else 0
                laser = z - below + random.gauss(0, 10)
                surface = 2 if over else 1
                handle_line(nmea(f"HGT,{int(t * 1000)},{int(laser)},{int(z - laser)},"
                                 f"{surface},{int(z)}"))
                if over:
                    handle_line(nmea(f"OBS,{obs_seq},{int(t * 1000)},{int(x)},{int(y)},"
                                     f"{int(below + random.gauss(0, 20))}"))
                    obs_seq += 1
            time.sleep(0.5)


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

    def do_POST(self):
        length = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            body = {}
        if self.path == "/mode":
            mode = str(body.get("mode", "")).lower()
            if mode not in ("car", "drone"):
                self.send_error(400, "mode must be car or drone")
                return
            sent = request_mode(mode)
            self.reply_json({"ok": True, "sent": sent})
        elif self.path == "/obstacles/clear":
            with STATE.lock:
                STATE.obstacles = []
            STATE.publish({"type": "obstacles_cleared"})
            self.reply_json({"ok": True})
        else:
            self.send_error(404)

    def reply_json(self, obj):
        data = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

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
    ap.add_argument("--tcp", type=int, metavar="PORT",
                    help="also send the text lines of pi_receiver.py (--rng --raw) on this TCP "
                         "port, e.g. 5000, for a serial terminal app on the tablet")
    args = ap.parse_args()
    sys.stdout.reconfigure(line_buffering=True)  # show the URLs at once, also when piped

    global TCP
    if args.tcp:
        TCP = TcpText(args.tcp)

    if args.demo:
        DEMO_MODE[0] = "car"
        STATE.desired_mode = "car"
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
    if args.tcp:
        print(f"Text lines (like pi_receiver.py) on TCP port {args.tcp}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
