"""Xsens MTData2, as emulated by the module's firmware on USART1 (115200 8N1).

    FA FF 36 LEN [XDI_hi XDI_lo SIZE DATA...]... CS        (LEN == 0xFF: 2 more length bytes)
    checksum: sum(bytes from FF to CS) & 0xFF == 0, values big-endian

Block decoding is done by vendor/mtdata2_decoder.py (parse_payload and its XDI table, verified
on the live device). Its Framer consumes a whole byte stream and returns packets, which does not
fit the Demux interface (match a frame at a position, report its length), so Mtdata2Framer below
re-states the same framing rules; a test checks that both find the same frames.

XDI low bits (Xsens spec): bits 0-1 precision (0 float32, 1 fp12.20, 2 fp16.32, 3 float64),
bits 2-3 coordinate system (0 ENU, 4 NED, 8 NWU). They are recorded per block. So far the device
only sends float32 / ENU, and it is an emulation: treat ENU as a hypothesis until checked.
"""

import struct
from collections import Counter, deque

import mtdata2_decoder as vendor

from .messages import FixType, GnssFix, GnssRaw, ImuSample, Message, RawBlock
from .protocol import BAD_CHECKSUM, NEED_MORE, NO_MATCH, PARSERS, Frame, Unwrapper

PREAMBLE, BUSID, MID_MTDATA2 = vendor.PREAMBLE, vendor.BUSID, vendor.MID_MTDATA2
EXT_LEN = 0xFF

PRECISION = {0: "float32", 1: "fp1220", 2: "fp1632", 3: "float64"}
COORDS = {0: "ENU", 4: "NED", 8: "NWU", 12: "reserved"}

STATUS_BITS = {0: "selftest", 1: "filter_valid", 2: "gnss_fix"}
ZERO_LEVEL = 1e-9  # |accel|, |gyro| components below this: no IMU board

# Blocks that become fields of ImuSample / GnssFix. Everything else goes to `extra`.
USED = {"Acc", "Gyro", "Euler", "Status", "LatLon", "AltEllipsoid", "Vel", "GpsSol",
        "PacketCounter", "SampleTimeFine", "Quaternion", "Mag"}  # fmt: skip


class RepeatDetector:
    """Flags a vector equal (exactly) to one of the last `depth` values of the same field.

    The device re-sends stale values: accel, gyro and Euler each repeat the value of the frame
    two before on the wire in 13-17% of valid frames, independently of each other (ping-pong TX
    buffers, fields updated by different tasks). With a live, noisy sensor an exact float match
    cannot happen by chance. A constant signal (no IMU board) repeats every time, as it should.
    """

    def __init__(self, depth=4):
        self.recent = deque(maxlen=depth)
        self.repeats = 0

    def __call__(self, vec) -> bool:
        key = tuple(vec)
        rep = key in self.recent
        self.recent.append(key)
        self.repeats += rep
        return rep


def header(buf, pos=0):
    """(mid, payload length, header length) of the frame at pos; buf must hold >= 6 bytes."""
    mid, ln = buf[pos + 2], buf[pos + 3]
    if ln == EXT_LEN:
        return mid, buf[pos + 4] << 8 | buf[pos + 5], 6
    return mid, ln, 4


def decode_status(status: int) -> dict:
    return {name: bool(status >> bit & 1) for bit, name in STATUS_BITS.items()}


def iter_blocks(payload: bytes):
    """Yields (xdi, data) for each block. A block cut short ends the iteration."""
    i = 0
    while i + 3 <= len(payload):
        xdi, size = payload[i] << 8 | payload[i + 1], payload[i + 2]
        data = payload[i + 3 : i + 3 + size]
        if len(data) != size:
            return
        yield xdi, data
        i += 3 + size


class Mtdata2Framer:
    """Same rules as vendor Framer.feed(): FA FF sync, 'FA FF FA' is a false sync, extended
    length above MAX_EXT_LEN is a false sync, checksum over FF..CS."""

    name = "mtdata2"
    sync = bytes([PREAMBLE])

    def __init__(self):
        self.bad = Counter()  # (mid, payload length) -> frames with a bad checksum

    def match(self, buf, pos) -> int:
        n = len(buf) - pos
        if n < 2:
            return NEED_MORE
        if buf[pos + 1] != BUSID:
            return NO_MATCH
        if n < 5:
            return NEED_MORE
        if buf[pos + 2] == PREAMBLE:
            return NO_MATCH
        if buf[pos + 3] == EXT_LEN and n < 6:
            return NEED_MORE
        mid, ln, hdr = header(buf, pos)
        if hdr == 6 and ln > vendor.MAX_EXT_LEN:
            return NO_MATCH
        total = hdr + ln + 1
        if n < total:
            return NEED_MORE
        if sum(buf[pos + 1 : pos + total]) & 0xFF:
            self.bad[(mid, ln)] += 1
            return BAD_CHECKSUM
        return total

    def kind(self, frame: bytes) -> str:
        return f"0x{frame[2]:02X}"


class Mtdata2Parser:
    name = "mtdata2"
    version = "1"

    def __init__(self, imu_config: dict | None = None, baud: int = 115200):
        self.framer = Mtdata2Framer()
        self.baud = baud
        self.time_source = "host"  # the pipeline sets "synthetic" for imported dumps
        self.frames = Counter()  # MID -> valid frames
        self.extended = 0
        self.xdi_counts = Counter()
        self.xdi_flags = {}  # xdi -> {"precision": ..., "coords": ...}
        self.malformed = 0  # payload whose last block is cut short
        self.status_values = Counter()
        self.zero_imu = 0
        self.accel_repeat = RepeatDetector()
        self.gyro_repeat = RepeatDetector()
        self.imu_absent = None  # None until the first IMU block, then True / False
        self.stf = Unwrapper()
        self.last_seq = None
        self.seq_gaps = 0
        self.frames_lost = 0
        self.events = []  # (level, text) for the pipeline's event log

    def drain_events(self) -> list[tuple[str, str]]:
        out, self.events = self.events, []
        return out

    def host_time_ns(self, frame: Frame) -> int:
        """Chunk time minus the time the bytes after this frame took on the wire (10 bits each)."""
        return frame.pc_rx_time_ns - frame.bytes_after * 10_000_000_000 // self.baud

    def parse(self, frame: Frame) -> list[Message]:
        d = frame.data
        mid, ln, hdr = header(d)
        self.frames[mid] += 1
        if hdr == 6:
            self.extended += 1
        if mid != MID_MTDATA2:
            return []
        payload = d[hdr : hdr + ln]
        for xdi, _ in iter_blocks(payload):
            self._count_block(xdi)
        p = vendor.parse_payload(payload)
        if p.pop("_truncated", False):
            self.malformed += 1

        times = {
            "pc_rx_time_ns": frame.pc_rx_time_ns,
            "host_time_ns": self.host_time_ns(frame),
            "time_source": self.time_source,
        }
        if "PacketCounter" in p:
            times["seq"] = self._check_seq(p["PacketCounter"][0])
        if "SampleTimeFine" in p:  # 10 kHz ticks
            times["mcu_time_us"] = self.stf(p["SampleTimeFine"][0]) * 100
            times["time_source"] = "device"

        out = []
        status = p["Status"][0] if "Status" in p else None
        if status is not None:
            self.status_values[status] += 1
        extra = {self._key(k): self._value(v) for k, v in p.items() if k not in USED}
        if {"Acc", "Gyro", "Euler"} & p.keys():
            nan3 = (float("nan"),) * 3
            accel, gyro = p.get("Acc", nan3), p.get("Gyro", nan3)
            self._check_imu_present(accel, gyro)
            out.append(
                ImuSample(
                    **times,
                    imu_id=0,
                    accel=_vec(accel),
                    gyro=_vec(gyro),
                    euler_deg=_vec(p["Euler"]) if "Euler" in p else None,
                    quat=_vec(p["Quaternion"]) if "Quaternion" in p else None,
                    mag=_vec(p["Mag"]) if "Mag" in p else None,
                    status=status,
                    extra=extra or None,
                    accel_repeat=self.accel_repeat(accel) if "Acc" in p else None,
                    gyro_repeat=self.gyro_repeat(gyro) if "Gyro" in p else None,
                )
            )
            extra = {}
        if "LatLon" in p:
            fix = status is not None and decode_status(status)["gnss_fix"]
            out.append(
                GnssFix(
                    **times,
                    valid_time_us=None,
                    gnss_tow_ms=None,
                    gnss_week=None,
                    fix_type=int(FixType.FIX_3D if fix else FixType.NONE),
                    num_sv=None,
                    lat_deg=float(p["LatLon"][0]),
                    lon_deg=float(p["LatLon"][1]),
                    height_m=float(p["AltEllipsoid"][0]) if "AltEllipsoid" in p else float("nan"),
                    vel_ned=None,  # the device frame of Vel is not verified
                    vel_xyz=_vec(p["Vel"]) if "Vel" in p else None,
                    h_acc_m=None,
                    v_acc_m=None,
                )
            )
        if "GpsSol" in p:
            out.append(GnssRaw(**times, kind="0x8840", data_hex=p["GpsSol"]))
        if extra:  # blocks of a frame without IMU data: keep them
            out.append(RawBlock(**times, blocks=extra))
        return out

    def _count_block(self, xdi):
        self.xdi_counts[xdi] += 1
        if xdi not in self.xdi_flags:
            self.xdi_flags[xdi] = {
                "precision": PRECISION[xdi & 0x3],
                "coords": COORDS[xdi & 0xC],
            }
            name = vendor.XDI.get(xdi, ("unknown",))[0]
            self.events.append(
                (
                    "info",
                    f"first XDI 0x{xdi:04X} ({name}; low bits: {self.xdi_flags[xdi]['precision']}, "
                    f"{self.xdi_flags[xdi]['coords']} per spec)",
                )  # fmt: skip
            )
            if xdi & 0xF:  # the vendor table only lists low nibble 0
                self.events.append(("warn", f"XDI 0x{xdi:04X} has non-default precision/"
                                            "coordinate bits; kept raw, not decoded"))  # fmt: skip

    def _check_imu_present(self, accel, gyro):
        # The real dry run (no IMU board) sends constant ~1e-14, not exact zeros: firmware
        # arithmetic on a zero input. Anything below ZERO_LEVEL counts as zero.
        absent = all(abs(v) < ZERO_LEVEL for v in (*accel, *gyro))
        self.zero_imu += absent
        if absent != self.imu_absent:
            if absent:
                msg = "IMU board seems absent: accel and gyro ~zero (< 1e-9)"
                self.events.append(("warn", msg))
            elif self.imu_absent is not None:
                self.events.append(("info", "IMU data is back (accel/gyro non-zero)"))
            self.imu_absent = absent

    def _check_seq(self, seq):
        if self.last_seq is not None:
            lost = (seq - self.last_seq - 1) & 0xFFFF
            if lost:
                self.seq_gaps += 1
                self.frames_lost += lost
        self.last_seq = seq
        return seq

    @staticmethod
    def _key(name: str) -> str:
        if name.startswith("XDI_"):
            return "0x" + name[4:]
        xdi = next((k for k, (n, _) in vendor.XDI.items() if n == name), None)
        return f"0x{xdi:04X}" if xdi is not None else name

    @staticmethod
    def _value(v):
        if isinstance(v, tuple):
            v = [_plain(x) for x in v]
            return v[0] if len(v) == 1 else v
        return v

    def stats(self) -> dict:
        return {
            "frames_by_mid": {f"0x{k:02X}": v for k, v in sorted(self.frames.items())},
            "extended_frames": self.extended,
            "bad_checksum_by_mid_len": {
                f"0x{m:02X}/{n}": c for (m, n), c in sorted(self.framer.bad.items())
            },
            "xdi_counts": {f"0x{k:04X}": v for k, v in sorted(self.xdi_counts.items())},
            "xdi_flags": {f"0x{k:04X}": v for k, v in sorted(self.xdi_flags.items())},
            "malformed_payloads": self.malformed,
            "status_values": {str(k): v for k, v in sorted(self.status_values.items())},
            "imu_all_zero_frames": self.zero_imu,
            "repeated_accel": self.accel_repeat.repeats,
            "repeated_gyro": self.gyro_repeat.repeats,
            "seq_gaps": self.seq_gaps,
            "frames_lost": self.frames_lost,
        }


PARSERS["mtdata2"] = Mtdata2Parser


def _plain(x):
    return float(x) if isinstance(x, float) else x


def _vec(t):
    return tuple(float(x) for x in t)


# ---------- encoder (simulator and tests) ----------


def encode_block(xdi: int, data: bytes) -> bytes:
    return struct.pack(">HB", xdi, len(data)) + data


def encode_frame(payload: bytes, mid: int = MID_MTDATA2, extended: bool | None = None) -> bytes:
    """extended=None: only when the payload needs it (> 254 bytes)."""
    if extended is None:
        extended = len(payload) >= EXT_LEN
    if extended:
        body = bytes([BUSID, mid, EXT_LEN]) + struct.pack(">H", len(payload)) + payload
    else:
        body = bytes([BUSID, mid, len(payload)]) + payload
    return bytes([PREAMBLE]) + body + bytes([-sum(body) & 0xFF])


def encode_imu_payload(
    euler=(0.0, 0.0, 0.0),
    acc=(0.0, 0.0, 9.81),
    gyro=(0.0, 0.0, 0.0),
    status=2,
    latlon=(29.9999, 29.9999),
    alt=0.0,
    vel=(0.0, 0.0, 0.0),
    x2060=0.0,
    extra_blocks=(),
) -> bytes:
    """The block layout of the real device (LEN 89), plus optional (xdi, bytes) blocks."""
    blocks = [
        encode_block(0x2030, struct.pack(">3f", *euler)),
        encode_block(0x4020, struct.pack(">3f", *acc)),
        encode_block(0x8020, struct.pack(">3f", *gyro)),
        encode_block(0xE010, bytes([status])),
        encode_block(0x5040, struct.pack(">2f", *latlon)),
        encode_block(0x5020, struct.pack(">f", alt)),
        encode_block(0xD010, struct.pack(">3f", *vel)),
        encode_block(0x2060, struct.pack(">f", x2060)),
        *(encode_block(x, d) for x, d in extra_blocks),
    ]
    return b"".join(blocks)


def encode_gpssol_payload(data: bytes = bytes(52)) -> bytes:
    """A separate GpsSol frame (the device sends LEN 55 = 3 + 52 bytes)."""
    return encode_block(0x8840, data)
