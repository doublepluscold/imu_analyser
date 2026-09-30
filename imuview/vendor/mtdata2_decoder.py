#!/usr/bin/env python3
"""
mtdata2_decoder.py - decoder for the IMU module's RS-232 output.

Reverse-engineering of the firmware showed the module emulates an Xsens MTi and
sends Xsens MTData2 packets on USART1 (external RS-232), 115200 8N1:

    FA  FF  36  LEN  [XDI_hi XDI_lo SIZE DATA...]...  CHECKSUM
    preamble, BusID, MID=MTData2, payload length, data blocks, checksum
    checksum: (BusID + MID + LEN + all payload bytes + CHECKSUM) & 0xFF == 0
    multi-byte values are big-endian

Expected blocks in this firmware (94-byte packet, LEN = 0x59):
    0x2030 EulerAngles       3 x float32  roll, pitch, yaw  [deg]
    0x4020 Acceleration      3 x float32  x, y, z           [m/s^2]
    0x8020 RateOfTurn        3 x float32  x, y, z           [rad/s]
    0xE010 StatusByte        1 byte
    0x5040 LatLon            2 x float32  lat, lon          [deg]
    0x5020 AltitudeEllipsoid 1 x float32                    [m]
    0xD010 VelocityXYZ       3 x float32  x, y, z           [m/s]
    0x2060 (non-standard)    1 x float32  meaning unknown
Units are the Xsens defaults; confirm them against real motion.
The parser is generic: any block layout is decoded, unknown XDIs are shown raw.

LISTEN ONLY. This script never transmits: USART1 also carries the OTA
bootloader and a command parser, so do not send bytes to the module.

Usage:
  live:   python3 mtdata2_decoder.py /dev/ttyUSB0
          python3 mtdata2_decoder.py /dev/ttyUSB0 --baud 115200 --csv log.csv
  file:   python3 mtdata2_decoder.py --file capture.bin
  quiet:  add --every 10   (print every 10th packet, still log all to CSV)
Deps: pyserial (only for live mode)
"""

import sys
import struct
import argparse
import time

XDI = {
    0x1010: ("UtcTime", None),
    0x1020: ("PacketCounter", ">H"),
    0x1060: ("SampleTimeFine", ">I"),
    0x1070: ("SampleTimeCoarse", ">I"),
    0x2010: ("Quaternion", ">4f"),
    0x2030: ("Euler", ">3f"),
    0x2060: ("X2060", ">f"),
    0x3010: ("BaroPressure", ">I"),
    0x4010: ("DeltaV", ">3f"),
    0x4020: ("Acc", ">3f"),
    0x4030: ("FreeAcc", ">3f"),
    0x5020: ("AltEllipsoid", ">f"),
    0x5040: ("LatLon", ">2f"),
    0x8020: ("Gyro", ">3f"),
    0x8030: ("DeltaQ", ">4f"),
    0x8840: ("GpsSol", None),          # legacy Xsens GPS solution block, shown raw
    0xC020: ("Mag", ">3f"),
    0xD010: ("Vel", ">3f"),
    0xE010: ("Status", ">B"),
    0xE020: ("StatusWord", ">I"),
}

PREAMBLE, BUSID, MID_MTDATA2 = 0xFA, 0xFF, 0x36
MAX_EXT_LEN = 1024          # longer "extended length" = false sync


def parse_payload(payload):
    """Split MTData2 payload into {name: tuple} (unknown XDI -> hex string)."""
    out = {}
    i = 0
    while i + 3 <= len(payload):
        xdi = (payload[i] << 8) | payload[i + 1]
        size = payload[i + 2]
        data = payload[i + 3:i + 3 + size]
        i += 3 + size
        if len(data) != size:
            out["_truncated"] = True
            break
        name, fmt = XDI.get(xdi, (f"XDI_{xdi:04X}", None))
        if fmt and struct.calcsize(fmt) == size:
            out[name] = struct.unpack(fmt, data)
        else:
            out[name] = data.hex()
    return out


class Framer:
    """Byte-stream -> validated MTData2 packets. Resyncs on bad checksum."""

    def __init__(self):
        self.buf = bytearray()
        self.ok = 0
        self.bad = 0
        self.other = 0

    def feed(self, chunk):
        self.buf.extend(chunk)
        packets = []
        while True:
            start = self.buf.find(bytes([PREAMBLE, BUSID]))
            if start < 0:
                del self.buf[:-1]
                break
            if start:
                del self.buf[:start]
            if len(self.buf) < 5:
                break
            mid, ln = self.buf[2], self.buf[3]
            hdr = 4
            if mid == PREAMBLE:                 # "FA FF FA ..." -> false sync
                del self.buf[:1]
                continue
            if ln == 0xFF:                      # extended length
                if len(self.buf) < 6:
                    break
                ln = (self.buf[4] << 8) | self.buf[5]
                hdr = 6
                if ln > MAX_EXT_LEN:            # implausible -> false sync, don't wait for it
                    del self.buf[:1]
                    continue
            total = hdr + ln + 1
            if len(self.buf) < total:
                break
            frame = bytes(self.buf[:total])
            if sum(frame[1:]) & 0xFF != 0:
                self.bad += 1
                del self.buf[:1]                # false sync, slide by one
                continue
            del self.buf[:total]
            if mid == MID_MTDATA2:
                self.ok += 1
                packets.append(parse_payload(frame[hdr:hdr + ln]))
            else:
                self.other += 1
                packets.append({"_MID": f"0x{mid:02X}", "_raw": frame.hex()})
        return packets


CSV_COLS = ["t", "roll", "pitch", "yaw", "ax", "ay", "az", "gx", "gy", "gz",
            "status", "lat", "lon", "alt", "vx", "vy", "vz", "x2060"]


def to_row(t, p):
    g = lambda k, n: list(p.get(k, (None,) * n)) if isinstance(p.get(k), tuple) else [None] * n
    return [t] + g("Euler", 3) + g("Acc", 3) + g("Gyro", 3) + g("Status", 1) + \
        g("LatLon", 2) + g("AltEllipsoid", 1) + g("Vel", 3) + g("X2060", 1)


def fmt(p):
    if "_MID" in p:
        return f"[other msg MID={p['_MID']}] {p['_raw']}"
    parts = []
    for k, v in p.items():
        if isinstance(v, tuple):
            parts.append(f"{k}=" + ",".join(f"{x:.4f}" if isinstance(x, float) else str(x) for x in v))
        else:
            parts.append(f"{k}={v}")
    return "  ".join(parts)


def main():
    ap = argparse.ArgumentParser(description="Decode Xsens MTData2 from the IMU module (listen-only).")
    ap.add_argument("port", nargs="?", help="serial port, e.g. /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--file", help="decode a raw capture file instead of a port")
    ap.add_argument("--csv", help="write decoded rows to CSV")
    ap.add_argument("--raw", help="also save raw bytes to this file (live mode)")
    ap.add_argument("--every", type=int, default=1, help="print every Nth packet")
    a = ap.parse_args()
    if not a.port and not a.file:
        ap.error("give a serial port or --file")

    fr = Framer()
    csvf = open(a.csv, "w") if a.csv else None
    if csvf:
        csvf.write(",".join(CSV_COLS) + "\n")
    rawf = open(a.raw, "wb") if a.raw else None
    n = 0
    t0 = time.time()

    def handle(chunk):
        nonlocal n
        for p in fr.feed(chunk):
            n += 1
            t = round(time.time() - t0, 4)
            if csvf and "_MID" not in p:
                csvf.write(",".join("" if v is None else str(v) for v in to_row(t, p)) + "\n")
            if n % a.every == 0:
                print(f"#{n:<6} {fmt(p)}", flush=True)

    try:
        if a.file:
            with open(a.file, "rb") as f:
                handle(f.read())
        else:
            try:
                import serial
            except ImportError:
                sys.exit("pyserial missing:  pip install pyserial  (or apt install python3-serial)")
            with serial.Serial(a.port, a.baud, timeout=0.2) as ser:   # never written to
                print(f"Listening on {a.port} @ {a.baud} (Ctrl+C to stop)")
                while True:
                    chunk = ser.read(4096)
                    if chunk:
                        if rawf:
                            rawf.write(chunk)
                        handle(chunk)
    except KeyboardInterrupt:
        pass
    finally:
        dt = max(time.time() - t0, 1e-9)
        print(f"\npackets ok={fr.ok} bad_checksum={fr.bad} other_msgs={fr.other}"
              + ("" if a.file else f"  rate ~{fr.ok/dt:.1f} Hz"))
        if csvf:
            csvf.close()
        if rawf:
            rawf.close()


if __name__ == "__main__":
    main()
