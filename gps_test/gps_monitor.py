#!/usr/bin/env python3
"""
gps_monitor.py - live GPS status from the IMU module's MTData2 stream.

Shows fix type, satellite count, position, speed and DOP, pulled from two
sources in the stream:
  - StatusByte  (XDI 0xE010): bit2 = GNSS fix  (most reliable yes/no)
  - GpsSol      (XDI 0x8840): u-blox NAV-SOL wrapper, gives gpsFix + numSV
  - LatLon/Alt/Velocity (0x5040/0x5020/0xD010): the fused position

Fix type and satellite count are single bytes in GpsSol, so they decode the
same regardless of byte order. Multi-byte GpsSol fields (pDOP, iTOW) are read
as u-blox little-endian and marked "~" - confirm against a real fix.

LISTEN ONLY: never writes to the module (OTA bootloader + command parser live
on the same UART).

Usage:
  python3 gps_monitor.py /dev/ttyUSB0
  python3 gps_monitor.py /dev/ttyUSB0 --baud 115200 --raw gps.bin
  python3 gps_monitor.py --file capture.bin        # replay a saved capture
Deps: pyserial (live mode only)
"""

import sys
import struct
import argparse
import time

PRE, BUS, MID = 0xFA, 0xFF, 0x36
MAX_EXT = 1024
FIX_NAME = {0: "NO FIX", 1: "DEAD-RECK", 2: "2D FIX", 3: "3D FIX",
            4: "GPS+DR", 5: "TIME ONLY"}

# Note: on this module the GpsSol block fills only gpsFix and numSV; the
# quality fields (pDOP, pAcc, sAcc, ECEF) are left zero/garbage, so we do not
# trust them. numSV and fix are single bytes and decode reliably.


def haversine_km(lat1, lon1, lat2, lon2):
    from math import radians, sin, cos, asin, sqrt
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat / 2) ** 2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) ** 2
    return 2 * 6371.0 * asin(sqrt(a))


class Framer:
    """Byte stream -> MTData2 payloads, checksum-validated, resyncs on junk."""

    def __init__(self):
        self.buf = bytearray()
        self.ok = 0
        self.bad = 0

    def feed(self, chunk):
        self.buf.extend(chunk)
        out = []
        while True:
            i = self.buf.find(bytes([PRE, BUS]))
            if i < 0:
                del self.buf[:-1]
                break
            if i:
                del self.buf[:i]
            if len(self.buf) < 5:
                break
            mid, ln = self.buf[2], self.buf[3]
            hdr = 4
            if mid == PRE:
                del self.buf[:1]
                continue
            if ln == 0xFF:
                if len(self.buf) < 6:
                    break
                ln = (self.buf[4] << 8) | self.buf[5]
                hdr = 6
                if ln > MAX_EXT:
                    del self.buf[:1]
                    continue
            total = hdr + ln + 1
            if len(self.buf) < total:
                break
            frame = bytes(self.buf[:total])
            if sum(frame[1:]) & 0xFF != 0:
                self.bad += 1
                del self.buf[:1]
                continue
            del self.buf[:total]
            self.ok += 1
            if mid == MID:
                out.append(frame[hdr:hdr + ln])
        return out


def blocks(payload):
    i, out = 0, {}
    while i + 3 <= len(payload):
        xdi = (payload[i] << 8) | payload[i + 1]
        size = payload[i + 2]
        data = payload[i + 3:i + 3 + size]
        i += 3 + size
        if len(data) != size:
            break
        out[xdi] = data
    return out


class GpsState:
    def __init__(self):
        self.status = None          # StatusByte
        self.fix = None             # gpsFix from GpsSol
        self.numsv = None           # satellites used
        self.pdop = None            # ~ (little-endian assumption)
        self.lat = self.lon = self.alt = None
        self.vel = None             # (vx,vy,vz)
        self.t_sol = None           # last GpsSol time
        self.t_any = None

    def update(self, b):
        self.t_any = time.time()
        if 0xE010 in b and len(b[0xE010]) >= 1:
            self.status = b[0xE010][0]
        if 0x5040 in b and len(b[0x5040]) >= 8:
            self.lat, self.lon = struct.unpack(">2f", b[0x5040][:8])
        if 0x5020 in b and len(b[0x5020]) >= 4:
            self.alt = struct.unpack(">f", b[0x5020][:4])[0]
        if 0xD010 in b and len(b[0xD010]) >= 12:
            self.vel = struct.unpack(">3f", b[0xD010][:12])
        if 0x8840 in b:
            d = b[0x8840]
            if len(d) >= 48:                     # NAV-SOL is 52 bytes
                self.fix = d[10]                 # single byte, endian-free
                self.numsv = d[47]               # single byte, endian-free
                self.pdop = struct.unpack("<H", d[44:46])[0] / 100.0  # ~
                self.t_sol = time.time()


def fix_label(st):
    # prefer GpsSol fix; fall back to StatusByte bit2
    if st.fix is not None:
        name = FIX_NAME.get(st.fix, f"?{st.fix}")
    elif st.status is not None:
        name = "3D FIX" if (st.status & 0x04) else "NO FIX"
    else:
        name = "?"
    return name


def status_bits(s):
    if s is None:
        return "status --"
    return (f"status 0x{s:02X}: selftest={int(bool(s & 1))} "
            f"filter_valid={int(bool(s & 2))} gnss_fix={int(bool(s & 4))}")


def render(st, home=None):
    fix = fix_label(st)
    good = st.fix in (2, 3, 4) or (st.status is not None and st.status & 0x04)
    sv = st.numsv if st.numsv is not None else "--"
    bar = ""
    if isinstance(st.numsv, int):
        bar = "#" * min(st.numsv, 20)
    hspeed = (st.vel[0] ** 2 + st.vel[1] ** 2) ** 0.5 if st.vel else None
    speed = f"{hspeed:6.2f} m/s horiz" if hspeed is not None else ""
    lines = [
        "\033[H\033[J" + "=" * 52,
        " GPS MONITOR   (listen-only)   Ctrl+C to stop",
        "=" * 52,
        f"  FIX        : {fix}   {'[LOCKED]' if good else '[no lock]'}",
        f"  SATELLITES : {sv}   {bar}",
        f"  position   : lat {st.lat if st.lat is None else round(st.lat,6)}"
        f"  lon {st.lon if st.lon is None else round(st.lon,6)}",
        f"  altitude   : {st.alt if st.alt is None else round(st.alt,1)} m",
        f"  velocity   : {speed}",
        f"  {status_bits(st.status)}",
        "-" * 52,
    ]
    # spoof check: if we claim a fix, is it plausible for a stationary receiver?
    if good and st.lat is not None:
        flags = []
        if home:
            dkm = haversine_km(st.lat, st.lon, home[0], home[1])
            lines.insert(-1, f"  dist from home : {dkm:,.0f} km")
            if dkm > 100:
                flags.append(f"position {dkm:,.0f} km from home")
        if hspeed is not None and hspeed > 5:
            flags.append(f"moving {hspeed:.0f} m/s while stationary")
        if isinstance(st.numsv, int) and st.numsv > 24:
            flags.append(f"{st.numsv} sats (implausibly high)")
        if flags:
            lines.append("  *** LIKELY SPOOFED ***  " + "; ".join(flags))
            lines.append("-" * 52)
    if st.t_sol is None:
        lines.append("  waiting for a GpsSol block (0x8840)...")
    else:
        age = time.time() - st.t_sol
        lines.append(f"  last GpsSol {age:4.1f}s ago")
    if not good:
        lines.append("  hint: GPS needs sky view. Indoors you usually get")
        lines.append("        NO FIX. Take it to a window or outside and wait")
        lines.append("        30 s to a few minutes for the first fix.")
    print("\n".join(lines), flush=True)


def main():
    ap = argparse.ArgumentParser(description="Live GPS status from the module (listen-only).")
    ap.add_argument("port", nargs="?")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--file", help="replay a raw capture instead of a live port")
    ap.add_argument("--raw", help="also save raw bytes (live mode)")
    ap.add_argument("--home", nargs=2, type=float, metavar=("LAT","LON"),
                    help="your real location; enables spoof check (e.g. --home 50.45 30.52)")
    a = ap.parse_args()
    if not a.port and not a.file:
        ap.error("give a serial port or --file")

    fr = Framer()
    st = GpsState()
    last_draw = 0
    home = tuple(a.home) if a.home else None

    def pump(chunk):
        nonlocal last_draw
        for p in fr.feed(chunk):
            st.update(blocks(p))
        if time.time() - last_draw > 0.25:
            render(st, home)
            last_draw = time.time()

    try:
        if a.file:
            with open(a.file, "rb") as f:
                data = f.read()
            for i in range(0, len(data), 512):
                pump(data[i:i + 512])
            render(st, home)
            print(f"\n(file done) frames ok={fr.ok} bad_checksum={fr.bad}")
        else:
            try:
                import serial
            except ImportError:
                sys.exit("pyserial missing:  pip install pyserial")
            rawf = open(a.raw, "wb") if a.raw else None
            with serial.Serial(a.port, a.baud, timeout=0.2) as ser:   # never written
                while True:
                    chunk = ser.read(4096)
                    if chunk:
                        if rawf:
                            rawf.write(chunk)
                        pump(chunk)
    except KeyboardInterrupt:
        print(f"\nstopped. frames ok={fr.ok} bad_checksum={fr.bad}")


if __name__ == "__main__":
    main()
