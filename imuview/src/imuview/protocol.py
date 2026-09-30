"""Wire formats: framers, the demux that runs them over one byte stream, and parsers.

Reference frame (little-endian):
  0xA5 0x5A | len:u8 | type:u8 | seq:u16 | t_us:u32 | payload | crc16:u16
  len   = number of bytes from type to end of payload (= 7 + payload length)
  crc16 = CRC-16/CCITT-FALSE over len..payload
  0x01 IMU int16   : n_imu:u8, then per IMU ax ay az gx gy gz
  0x02 IMU int32   : same layout
  0x03 IMU float32 : same layout, already in m/s^2 and rad/s
  0x04 IMU float32 + quaternion: n_imu:u8, per IMU ax ay az gx gy gz qw qx qy qz
  0x10..0x1F reserved for GNSS, 0x20..0x2F for baro/mag/other (counted as unknown for now)

NMEA ("$...*hh\\r\\n") and UBX (0xB5 0x62 ...) framers only check the checksum and
count frames by type. Decoding them into GnssFix is future work: add a parser to PARSERS.
"""

import binascii
import math
import struct
from collections import Counter
from dataclasses import dataclass

from .frames import G
from .messages import ImuSample, Message

NEED_MORE = -1  # match() results; a positive value is the length of a valid frame
NO_MATCH = 0
BAD_CHECKSUM = -2


@dataclass
class Frame:
    framer: str  # "ref" | "nmea" | "ubx"
    kind: str  # "0x01" | "GPGGA" | "NAV-PVT" ...
    data: bytes  # the whole frame, sync bytes and checksum included
    pc_rx_time_ns: int
    bytes_after: int = 0  # bytes received after the end of this frame, up to pc_rx_time_ns


def crc16(data) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). binascii does it in C."""
    return binascii.crc_hqx(data, 0xFFFF)


# ---------- framers ----------


class RefFramer:
    name = "ref"
    sync = b"\xa5"

    def match(self, buf, pos) -> int:
        n = len(buf) - pos
        if n < 2:
            return NEED_MORE
        if buf[pos + 1] != 0x5A:
            return NO_MATCH
        if n < 3:
            return NEED_MORE
        length = buf[pos + 2]
        if length < 7:
            return NO_MATCH
        total = length + 5
        if n < total:
            return NEED_MORE
        crc = buf[pos + 3 + length] | buf[pos + 4 + length] << 8
        if crc16(buf[pos + 2 : pos + 3 + length]) != crc:
            return BAD_CHECKSUM
        return total

    def kind(self, frame: bytes) -> str:
        return f"0x{frame[3]:02X}"


class NmeaFramer:
    name = "nmea"
    sync = b"$"
    max_len = 128  # the standard says 82, some receivers send longer proprietary sentences

    def match(self, buf, pos) -> int:
        end = min(len(buf), pos + self.max_len)
        star = buf.find(b"*", pos, end)
        body = buf[pos + 1 : star if star >= 0 else end]
        if any(c < 0x20 or c > 0x7E for c in body):
            return NO_MATCH
        if star < 0:
            return NEED_MORE if end - pos < self.max_len else NO_MATCH
        if len(buf) < star + 5:
            return NEED_MORE
        if buf[star + 3 : star + 5] != b"\r\n":
            return NO_MATCH
        try:
            wanted = int(bytes(buf[star + 1 : star + 3]), 16)
        except ValueError:
            return NO_MATCH
        x = 0
        for c in body:
            x ^= c
        return star + 5 - pos if x == wanted else BAD_CHECKSUM

    def kind(self, frame: bytes) -> str:
        return frame[1:].split(b",")[0].split(b"*")[0].decode("ascii")  # talker + type, e.g. GPGGA


UBX_CLASSES = {0x01, 0x02, 0x04, 0x05, 0x06, 0x09, 0x0A, 0x0B, 0x0D, 0x10, 0x13, 0x21, 0x27, 0x28}
UBX_NAMES = {
    (0x01, 0x02): "NAV-POSLLH",
    (0x01, 0x03): "NAV-STATUS",
    (0x01, 0x07): "NAV-PVT",
    (0x01, 0x12): "NAV-VELNED",
    (0x01, 0x21): "NAV-TIMEUTC",
    (0x01, 0x35): "NAV-SAT",
    (0x02, 0x15): "RXM-RAWX",
    (0x05, 0x00): "ACK-NAK",
    (0x05, 0x01): "ACK-ACK",
    (0x0A, 0x04): "MON-VER",
}


class UbxFramer:
    name = "ubx"
    sync = b"\xb5"
    max_payload = 2048  # bigger lengths are treated as a false sync, so we never wait long

    def match(self, buf, pos) -> int:
        n = len(buf) - pos
        if n < 2:
            return NEED_MORE
        if buf[pos + 1] != 0x62:
            return NO_MATCH
        if n < 3:
            return NEED_MORE
        if buf[pos + 2] not in UBX_CLASSES:
            return NO_MATCH
        if n < 6:
            return NEED_MORE
        length = buf[pos + 4] | buf[pos + 5] << 8
        if length > self.max_payload:
            return NO_MATCH
        total = length + 8
        if n < total:
            return NEED_MORE
        a = b = 0  # 8-bit Fletcher over class, id, length, payload
        for c in buf[pos + 2 : pos + 6 + length]:
            a = (a + c) & 0xFF
            b = (b + a) & 0xFF
        if buf[pos + 6 + length] != a or buf[pos + 7 + length] != b:
            return BAD_CHECKSUM
        return total

    def kind(self, frame: bytes) -> str:
        cls, mid = frame[2], frame[3]
        return UBX_NAMES.get((cls, mid), f"{cls:02X}-{mid:02X}")


# ---------- demux ----------


class Demux:
    """Runs several framers over one byte stream.

    At each position we ask every framer whose sync byte matches. A complete frame
    with a good checksum wins and its bytes are skipped. If some framer needs more
    bytes we wait for the next chunk. Otherwise that single byte is "unclaimed" and
    we move on by one byte (this is also how we resync after a CRC error).
    """

    def __init__(self, framers):
        self.framers = framers
        self.buf = bytearray()
        self.frames = Counter()  # (framer, kind) -> count
        self.bad_checksum = Counter()  # framer -> count
        self.total_bytes = 0
        self.unclaimed_bytes = 0
        self.unclaimed_sample = bytearray()  # first unclaimed bytes, for `imu sniff`
        self.sync_bytes = {f.sync[0] for f in framers}

    def feed(self, data: bytes, pc_rx_time_ns: int) -> list[Frame]:
        self.buf += data
        self.total_bytes += len(data)
        buf = self.buf
        out = []
        pos = 0
        while pos < len(buf):
            if buf[pos] not in self.sync_bytes:  # skip bytes no framer could start with
                nxt = self._next_sync(pos)
                self._unclaimed(buf[pos:nxt])
                pos = nxt
                continue
            result, framer = self._try_framers(pos)
            if result > 0:
                frame = bytes(buf[pos : pos + result])
                after = len(buf) - pos - result
                out.append(Frame(framer.name, framer.kind(frame), frame, pc_rx_time_ns, after))
                self.frames[(framer.name, out[-1].kind)] += 1
                pos += result
            elif result == NEED_MORE:
                break
            else:
                self._unclaimed(buf[pos : pos + 1])
                pos += 1
        del buf[:pos]
        return out

    def _next_sync(self, pos) -> int:
        found = [i for f in self.framers if (i := self.buf.find(f.sync, pos)) >= 0]
        return min(found) if found else len(self.buf)

    def _try_framers(self, pos):
        waiting = None
        for f in self.framers:
            if self.buf[pos] != f.sync[0]:
                continue
            r = f.match(self.buf, pos)
            if r > 0:
                return r, f
            if r == NEED_MORE:
                waiting = f
            elif r == BAD_CHECKSUM:
                self.bad_checksum[f.name] += 1
        return (NEED_MORE, waiting) if waiting else (NO_MATCH, None)

    def _unclaimed(self, data):
        self.unclaimed_bytes += len(data)
        if len(self.unclaimed_sample) < 256:
            self.unclaimed_sample += data[: 256 - len(self.unclaimed_sample)]


def build_demux(parsers) -> Demux:
    """The parsers' framers, plus NMEA and UBX (always, so we notice raw GPS data)."""
    framers = {p.framer.name: p.framer for p in parsers}
    framers.setdefault("nmea", NmeaFramer())
    framers.setdefault("ubx", UbxFramer())
    return Demux(list(framers.values()))


# ---------- time ----------


class Unwrapper:
    """Turns the MCU's u32 microsecond counter (wraps every ~71.6 min) into 64 bit."""

    def __init__(self):
        self.last = None
        self.offset = 0

    def __call__(self, t32: int) -> int:
        if self.last is not None and t32 < self.last and self.last - t32 > 2**31:
            self.offset += 2**32
        self.last = t32
        return self.offset + t32


# ---------- reference parser ----------

IMU_LAYOUTS = {0x01: ("<6h", 12), 0x02: ("<6i", 24), 0x03: ("<6f", 24), 0x04: ("<10f", 40)}


class RefParser:
    name = "ref"
    version = "1"

    def __init__(self, imu_config: dict | None = None):
        """imu_config: {"default": {...}, "0": {...}, ...} with accel_lsb_g and gyro_lsb_dps."""
        self.imu_config = imu_config or {}
        self.framer = RefFramer()
        self.unwrap = Unwrapper()
        self.last_seq = None
        self.seq_gaps = 0
        self.frames_lost = 0
        self.bad_length = 0
        self.unknown_types = Counter()

    def scales(self, imu_id) -> tuple[float, float]:
        cfg = {**self.imu_config.get("default", {}), **self.imu_config.get(str(imu_id), {})}
        accel = cfg.get("accel_lsb_g", 1 / 2048) * G
        gyro = math.radians(cfg.get("gyro_lsb_dps", 1 / 16.4))
        return accel, gyro

    def parse(self, frame: Frame) -> list[Message]:
        d = frame.data
        length, ftype = d[2], d[3]
        seq, t32 = struct.unpack_from("<HI", d, 4)
        payload = d[10 : 3 + length]
        self._check_seq(seq)
        t_us = self.unwrap(t32)
        if ftype not in IMU_LAYOUTS:
            self.unknown_types[ftype] += 1
            return []
        return self._imu(ftype, payload, seq, t_us, frame.pc_rx_time_ns)

    def _check_seq(self, seq):
        if self.last_seq is not None:
            lost = (seq - self.last_seq - 1) & 0xFFFF
            if lost:
                self.seq_gaps += 1
                self.frames_lost += lost
        self.last_seq = seq

    def _imu(self, ftype, payload, seq, t_us, pc_ns) -> list[Message]:
        fmt, size = IMU_LAYOUTS[ftype]
        n = payload[0] if payload else 0
        if len(payload) != 1 + n * size:
            self.bad_length += 1
            return []
        out = []
        for i in range(n):
            v = struct.unpack_from(fmt, payload, 1 + i * size)
            a_scale, g_scale = (1.0, 1.0) if ftype >= 0x03 else self.scales(i)
            out.append(
                ImuSample(
                    mcu_time_us=t_us,
                    pc_rx_time_ns=pc_ns,
                    seq=seq,
                    imu_id=i,
                    accel=(v[0] * a_scale, v[1] * a_scale, v[2] * a_scale),
                    gyro=(v[3] * g_scale, v[4] * g_scale, v[5] * g_scale),
                    quat=tuple(v[6:10]) if ftype == 0x04 else None,
                )
            )
        return out

    def stats(self) -> dict:
        return {
            "seq_gaps": self.seq_gaps,
            "frames_lost": self.frames_lost,
            "bad_length": self.bad_length,
            "unknown_types": {f"0x{k:02X}": v for k, v in self.unknown_types.items()},
        }


# A new wire format = a new parser class with .name, .version, .framer, .parse(), .stats().
# config "parser" picks the main one; "extra_parsers" adds more (e.g. a future NMEA/UBX
# parser that turns GPS frames into GnssFix). Each gets the frames of its own framer.
# protocol_mtdata2 adds "mtdata2" when imported (it imports this module).
PARSERS = {"ref": RefParser}


# ---------- encoder (used by the simulator and the tests) ----------


def encode_frame(ftype: int, seq: int, t_us: int, payload: bytes) -> bytes:
    body = struct.pack("<BBHI", 7 + len(payload), ftype, seq & 0xFFFF, t_us & 0xFFFFFFFF) + payload
    return b"\xa5\x5a" + body + struct.pack("<H", crc16(body))


def encode_imu_payload(ftype: int, rows) -> bytes:
    """rows: one tuple per IMU with 6 values (10 for type 0x04), already raw units."""
    fmt, _ = IMU_LAYOUTS[ftype]
    return bytes([len(rows)]) + b"".join(struct.pack(fmt, *r) for r in rows)
