#!/usr/bin/env python3
"""
Checks the MAVLink output of the mobile node (pins D6 = TX / D7 = RX, 115200 baud) through a
USB-UART adapter, without a flight controller.

Wiring (adapter set to 3.3 V!):  XIAO D6 (TX) -> adapter RX,  XIAO GND -> adapter GND.
Do not connect VCC: the XIAO is powered by its own USB.

    pip install pyserial
    python mavlink_check.py --port COM7          # Windows (the adapter's port, not the XIAO's)
    python mavlink_check.py --port /dev/ttyUSB0  # Linux / Raspberry Pi
    python mavlink_check.py --list               # show serial ports

Prints every HEARTBEAT and VISION_POSITION_ESTIMATE with the checksum verified, and every
5 s a summary: packets per message type, rate, bad checksums, bytes that were not MAVLink.
Only the two messages the firmware sends are decoded; others are counted by id.
"""

import argparse
import struct
import sys
import time

# CRC_EXTRA of the messages we decode (from the MAVLink common.xml definitions)
MSGS = {
    0: ("HEARTBEAT", 50),
    102: ("VISION_POSITION_ESTIMATE", 158),
}
MAV_TYPE = {2: "QUADROTOR", 18: "ONBOARD_CONTROLLER", 6: "GCS", 1: "FIXED_WING"}
MAV_AUTOPILOT = {0: "GENERIC", 3: "ARDUPILOTMEGA", 8: "INVALID", 12: "PX4"}


def crc_accumulate(byte, crc):
    tmp = byte ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def x25(data, crc=0xFFFF):
    for b in data:
        crc = crc_accumulate(b, crc)
    return crc


class Parser:
    """MAVLink v1 / v2 frame parser (no signing support needed here)."""

    def __init__(self):
        self.buf = bytearray()
        self.junk = 0
        self.bad_crc = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            # find a start byte
            i = 0
            while i < len(self.buf) and self.buf[i] not in (0xFD, 0xFE):
                i += 1
            self.junk += i
            del self.buf[:i]
            if len(self.buf) < 2:
                return out
            v2 = self.buf[0] == 0xFD
            plen = self.buf[1]
            hdr = 10 if v2 else 6
            if v2 and len(self.buf) >= 3 and self.buf[2] & 0x01:
                total = hdr + plen + 2 + 13  # signed packet
            else:
                total = hdr + plen + 2
            if len(self.buf) < total:
                return out
            frame = bytes(self.buf[:total])
            if v2:
                seq, sysid, compid = frame[4], frame[5], frame[6]
                msgid = frame[7] | (frame[8] << 8) | (frame[9] << 16)
            else:
                seq, sysid, compid, msgid = frame[2], frame[3], frame[4], frame[5]
            payload = frame[hdr:hdr + plen]
            crc_rx = frame[hdr + plen] | (frame[hdr + plen + 1] << 8)
            extra = MSGS.get(msgid, (None, None))[1]
            if extra is not None:
                crc = x25(frame[1:hdr + plen])
                crc = crc_accumulate(extra, crc)
                if crc != crc_rx:
                    self.bad_crc += 1
                    del self.buf[:1]  # false start byte inside other data: resync
                    continue
            del self.buf[:total]
            out.append({"v2": v2, "seq": seq, "sysid": sysid, "compid": compid,
                        "msgid": msgid, "payload": payload, "crc_checked": extra is not None})


def decode(m):
    p = m["payload"]
    if m["msgid"] == 0:
        p = p + bytes(9 - len(p))  # MAVLink 2 drops trailing zero bytes
        custom_mode, mtype, autopilot, base_mode, status, ver = struct.unpack("<IBBBBB", p[:9])
        return (f"type={MAV_TYPE.get(mtype, mtype)} autopilot={MAV_AUTOPILOT.get(autopilot, autopilot)} "
                f"state={status}")
    if m["msgid"] == 102:
        p = p + bytes(32 - len(p)) if len(p) < 32 else p
        usec, x, y, z, roll, pitch, yaw = struct.unpack("<Qffffff", p[:32])
        return f"t={usec / 1e6:9.3f}s  x={x:7.3f} m  y={y:7.3f} m  z={z:7.3f} m"
    return f"{len(p)} payload bytes"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port of the USB-UART adapter")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    args = ap.parse_args()

    import serial
    from serial.tools import list_ports
    if args.list or not args.port:
        for p in list_ports.comports():
            print(f"  {p.device:12} {p.description}")
        if not args.port:
            print("Pass the adapter's port with --port")
        return

    sys.stdout.reconfigure(line_buffering=True)
    parser = Parser()
    counts, last_summary = {}, time.time()
    start = time.time()
    print(f"Listening on {args.port} at {args.baud} baud. Ctrl+C to stop.")
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        try:
            while True:
                for m in parser.feed(ser.read(256)):
                    name = MSGS.get(m["msgid"], (f"msg {m['msgid']}", None))[0]
                    counts[name] = counts.get(name, 0) + 1
                    ok = "CRC ok" if m["crc_checked"] else "CRC not checked"
                    print(f"[{time.time() - start:7.1f}s] MAVLink{'2' if m['v2'] else '1'} "
                          f"sys={m['sysid']} comp={m['compid']} seq={m['seq']:3}  {name:25} "
                          f"{decode(m)}  ({ok})")
                now = time.time()
                if now - last_summary >= 5:
                    dt = now - last_summary
                    summary = ", ".join(f"{k}: {v} ({v / dt:.1f}/s)" for k, v in counts.items()) or "no packets"
                    print(f"--- last {dt:.0f}s: {summary}; bad CRC {parser.bad_crc}; "
                          f"non-MAVLink bytes {parser.junk}")
                    if not counts and parser.junk == 0:
                        print("    nothing at all on the line: check D6 -> adapter RX and GND, "
                              "and that the mobile node runs")
                    counts, last_summary = {}, now
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
