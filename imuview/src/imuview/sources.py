"""Where bytes come from.

A source has:
    read(timeout) -> list of Chunk / Message, or None when there is nothing more
    describe() -> dict   (saved into meta.json)
    close()
and optionally:
    protocol     wire format it produces (simulators); otherwise config["parser"] decides
    baud         line rate, for host time reconstruction
    time_source  "host" (default) or "synthetic" (times made up from byte positions)
Messages in the list are "injected": they did not come from bytes (fake GNSS in the sim,
commands during replay, CSV imports).

SerialSource is LISTEN-ONLY. The module's USART1 also hosts an OTA bootloader and a command
parser; a stray byte can brick it. Nothing in this project may write to the port.
MasterSerialSource (the master ESP over USB) is a SerialSource too, so it listens only as well.
"""

import dataclasses
import math
import socket
import struct
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .frames import (
    NED_TO_ENU,
    G,
    board_rotation,
    quat_from_euler,
    quat_from_gyro,
    quat_from_matrix,
    quat_mul,
    quat_normalize,
    quat_to_euler,
    quat_to_matrix,
)
from .messages import FixType, GnssFix
from .netproto import HEADER, MAGIC, MAX_PAYLOAD, VERSION, Datagram, DatagramError, parse_datagram
from .protocol import RefParser, encode_frame, encode_imu_payload
from .protocol_mtdata2 import encode_frame as mt_encode_frame
from .protocol_mtdata2 import encode_gpssol_payload
from .protocol_mtdata2 import encode_imu_payload as mt_imu_payload


@dataclass
class Chunk:
    pc_rx_time_ns: int  # time.monotonic_ns() when the bytes arrived
    data: bytes
    module_id: int = 0  # which module sent them (0 = USB-serial)


class SimSource:
    """Fake sensor module that sends bytes like the real firmware would.

    protocol "ref": reference protocol, n_imu IMUs, device clock in every frame.
    protocol "mtdata2": Xsens MTData2 like our module (one IMU, no device clock, frames timed
    on a `baud` line, optional GpsSol frames, corruption / truncation / extended-length frames
    to test the parser and the bad-checksum analysis). Its sensor is mounted z-up
    (sensor_alignment_deg default [180, 0, 0]) so level and still reads +g on z like the device.

    Motion: 5 s still at start (so you can calibrate), then repeating 12 s of slow
    rotation about all axes followed by 4 s still. Each IMU gets noise and a constant
    gyro bias, and is mounted with the board alignment from the config (or
    sensor_alignment_deg). accel_offset / accel_matrix add a known accel error in the sensor
    frame: raw = inv(M) @ true + b, so that the calibration M (raw - b) recovers the truth.
    """

    def __init__(
        self,
        rate_hz=1000.0,
        n_imu=1,
        fake_gnss=False,
        realtime=True,
        seconds=None,
        seed=0,
        imu_config=None,
        t0_us=0,
        ftype=0x01,
        protocol="ref",
        baud=115200,
        sensor_alignment_deg=None,
        accel_offset=(0.0, 0.0, 0.0),
        accel_matrix=None,
        corrupt_prob=0.0,
        truncate_prob=0.0,
        extended_every=0,
        gpssol_every=0,
        attitude_deg=(0.0, 0.0, 0.0),
        motion="pattern",
        gyro_scale=1.0,
        gps_origin=None,
    ):
        """gps_origin = (lat, lon) makes the mtdata2 sim report a valid GNSS fix (status bit 2)
        moving on a 30 m circle around that point, so the GUI's 3D movement can be shown."""
        self.gps_origin = gps_origin
        self.protocol = protocol
        self.baud = baud
        self.time_source = "host"
        if protocol == "mtdata2":
            n_imu = 1
            if sensor_alignment_deg is None:
                sensor_alignment_deg = [180, 0, 0]
        self.corrupt_prob = corrupt_prob
        self.truncate_prob = truncate_prob
        self.extended_every = extended_every
        self.gpssol_every = gpssol_every
        self.accel_offset = np.asarray(accel_offset, dtype=float)
        self.accel_matrix = np.eye(3) if accel_matrix is None else np.asarray(accel_matrix, float)
        self.accel_matrix_inv = np.linalg.inv(self.accel_matrix)
        self.frames_sent = 0
        self.frames_corrupted = 0
        self.frames_truncated = 0
        self.frames_extended = 0
        self.line_free_us = t0_us
        self.pending = deque()  # (line end time us, frame bytes)
        self.rate_hz = rate_hz
        self.n_imu = n_imu
        self.fake_gnss = fake_gnss
        self.realtime = realtime
        self.seconds = seconds
        self.seed = seed
        self.ftype = ftype
        self.rng = np.random.default_rng(seed)
        imu_config = imu_config or {}
        self.parser = RefParser(imu_config)  # only used for its scale factors
        self.dt_us = round(1e6 / rate_hz)
        self.t0_us = t0_us
        self.t_us = t0_us  # MCU clock, 64 bit here, sent as u32
        self.seq = 0
        # true body attitude (FRD -> NED), from roll, pitch, yaw [deg]
        self.q = quat_from_euler(*np.radians(attitude_deg))
        self.motion = motion  # "pattern" | "still" | "rot_x" | "rot_y" | "rot_z"
        self.gyro_scale = gyro_scale  # gyro scale error (1.0 = none), for the scale check
        self.gyro_bias = self.rng.normal(0.0, math.radians(0.5), (n_imu, 3))
        self.body_to_sensor = []
        for i in range(n_imu):
            cfg = {**imu_config.get("default", {}), **imu_config.get(str(i), {})}
            align = sensor_alignment_deg or cfg.get("board_alignment_deg", [0, 0, 0])
            self.body_to_sensor.append(board_rotation(*align).T)
        self.scales = [self.parser.scales(i) for i in range(n_imu)]
        self.noise_std = np.array([0.05] * 3 + [math.radians(0.1)] * 3)  # accel, gyro
        self.next_gnss_us = t0_us + 200_000
        self.wall0 = None
        self.pc0 = time.monotonic_ns()

    def describe(self) -> dict:
        d = {
            "type": "sim",
            "protocol": self.protocol,
            "rate_hz": self.rate_hz,
            "n_imu": self.n_imu,
            "fake_gnss": self.fake_gnss,
            "seed": self.seed,
            "frame_type": f"0x{self.ftype:02X}",
            "true_gyro_bias_rad_s": self.gyro_bias.tolist(),
        }
        if self.protocol == "mtdata2":
            d |= {
                "baud": self.baud,
                "time_source": self.time_source,
                "true_accel_offset": self.accel_offset.tolist(),
                "true_accel_matrix": self.accel_matrix.tolist(),
                "corrupt_prob": self.corrupt_prob,
                "truncate_prob": self.truncate_prob,
                "extended_every": self.extended_every,
                "gpssol_every": self.gpssol_every,
                "frames_sent": self.frames_sent,
                "frames_corrupted": self.frames_corrupted,
                "frames_truncated": self.frames_truncated,
                "frames_extended": self.frames_extended,
            }
        return d

    def close(self):
        pass

    def elapsed(self) -> float:
        return (self.t_us - self.t0_us) * 1e-6

    def read(self, timeout=0.05):
        if self.seconds is not None and self.elapsed() >= self.seconds:
            return None
        if self.realtime:
            if self.wall0 is None:
                self.wall0 = time.monotonic()
            end = self.t0_us + (time.monotonic() - self.wall0) * 1e6
            if end - self.t_us < self.dt_us:
                time.sleep(min(timeout, 0.002))
                return []
        else:
            end = self.t_us + 10_000  # 10 ms of data per read
        if self.seconds is not None:
            end = min(end, self.t0_us + self.seconds * 1e6)
        if self.protocol == "mtdata2":
            return self._read_mtdata2(end)
        data = bytearray()
        gnss = []
        while self.t_us < end:
            data += self._imu_frame()
            if self.fake_gnss and self.t_us >= self.next_gnss_us:
                gnss.append(self._gnss_fix())
                self.next_gnss_us += 200_000  # 5 Hz
            self.t_us += self.dt_us
        if self.realtime:
            pc_ns = time.monotonic_ns()
        else:
            pc_ns = self.pc0 + (self.t_us - self.t0_us) * 1000 + 1_000_000  # 1 ms "USB delay"
        for fix in gnss:
            fix.pc_rx_time_ns = pc_ns
        return [Chunk(pc_ns, bytes(data)), *gnss]

    def body_rate(self, t) -> np.ndarray:
        """True angular rate [rad/s] in body frame at time t [s].

        pattern: 5 s still, then 12 s rotation about all axes / 4 s still, repeating.
        still: never moves. rot_x/y/z: 3 s still, +90 deg about that body axis in 3 s, still.
        """
        if self.motion == "still":
            return np.zeros(3)
        if self.motion.startswith("rot_"):
            if not 3.0 <= t < 6.0:
                return np.zeros(3)
            rate = np.zeros(3)  # smooth: 90 deg * (1 - cos) profile over 3 s
            rate["xyz".index(self.motion[-1])] = (
                math.radians(90) / 3.0 * (1 - math.cos(2 * math.pi * (t - 3.0) / 3.0))
            )
            return rate
        if t < 5.0 or (t - 5.0) % 16.0 > 12.0:
            return np.zeros(3)
        envelope = math.sin(math.pi * ((t - 5.0) % 16.0) / 12.0)  # smooth start and stop
        deg = [40 * math.sin(0.5 * t), 30 * math.sin(0.7 * t + 1), 60 * math.sin(0.3 * t + 2)]
        return envelope * np.radians(deg)

    def _truth_step(self):
        """Advance the true attitude by one sample. Returns (omega, specific force), body frame."""
        omega = self.body_rate(self.elapsed())
        self.q = quat_normalize(quat_mul(self.q, quat_from_gyro(omega, self.dt_us * 1e-6)))
        f_body = quat_to_matrix(self.q).T @ np.array([0.0, 0.0, -G])  # still: only gravity
        return omega, f_body

    def _sensor_values(self, i, omega, f_body, noise):
        r = self.body_to_sensor[i]
        accel = self.accel_matrix_inv @ (r @ f_body) + self.accel_offset + noise[:3]
        gyro = self.gyro_scale * (r @ omega) + self.gyro_bias[i] + noise[3:]
        return accel, gyro

    def _imu_frame(self) -> bytes:
        omega, f_body = self._truth_step()
        noise = self.rng.normal(0.0, 1.0, (self.n_imu, 6)) * self.noise_std
        rows = []
        for i in range(self.n_imu):
            r = self.body_to_sensor[i]
            accel, gyro = self._sensor_values(i, omega, f_body, noise[i])
            if self.ftype in (0x01, 0x02):
                a_lsb, g_lsb = self.scales[i]
                limit = 32767 if self.ftype == 0x01 else 2**31 - 1
                raw = np.concatenate([accel / a_lsb, gyro / g_lsb])
                rows.append(tuple(int(v) for v in np.clip(np.round(raw), -limit, limit)))
            elif self.ftype == 0x03:
                rows.append((*accel, *gyro))
            else:  # 0x04 also sends the true orientation of this IMU (sensor -> world)
                q_sensor = quat_mul(self.q, quat_from_matrix(r.T))
                rows.append((*accel, *gyro, *q_sensor))
        frame = encode_frame(self.ftype, self.seq, self.t_us, encode_imu_payload(self.ftype, rows))
        self.seq += 1
        return frame

    # ----- MTData2 -----

    def _read_mtdata2(self, end):
        while self.t_us < end:
            for frame in self._mtdata2_frames():
                start = max(self.t_us, self.line_free_us)
                self.line_free_us = start + len(frame) * 10e6 / self.baud  # 8N1: 10 bits/byte
                self.pending.append((self.line_free_us, frame))
            self.t_us += self.dt_us
        data, last = bytearray(), None
        while self.pending and self.pending[0][0] <= end:
            last, frame = self.pending.popleft()
            data += frame
        if not data:
            return []
        if self.realtime:
            pc_ns = time.monotonic_ns()
        else:
            pc_ns = self.pc0 + round((last - self.t0_us) * 1000) + 1_000_000  # 1 ms "USB delay"
        return [Chunk(pc_ns, bytes(data))]

    def _mtdata2_frames(self) -> list[bytes]:
        omega, f_body = self._truth_step()
        noise = self.rng.normal(0.0, 1.0, 6) * self.noise_std
        accel, gyro = self._sensor_values(0, omega, f_body, noise)
        # like Xsens: orientation of the SENSOR frame in ENU, ZYX Euler angles
        s2enu = NED_TO_ENU @ quat_to_matrix(self.q) @ self.body_to_sensor[0].T
        roll, pitch, yaw = (math.degrees(a) for a in quat_to_euler(quat_from_matrix(s2enu)))
        extra = []
        n = self.frames_sent
        if self.extended_every and n % self.extended_every == self.extended_every - 1:
            extra.append((0x7FF0, bytes(range(200))))  # unknown block, pushes LEN past 254
            self.frames_extended += 1
        gps = {}
        if self.gps_origin is not None:
            t = self.elapsed()
            east, north = (
                30.0 * math.sin(2 * math.pi * t / 40),
                30.0 * math.cos(2 * math.pi * t / 40),
            )
            gps = {
                "status": 7,  # selftest, filter valid, GNSS fix
                "latlon": (
                    self.gps_origin[0] + north / 111_320,
                    self.gps_origin[1]
                    + east / (111_320 * math.cos(math.radians(self.gps_origin[0]))),
                ),
                "alt": 100.0 + 5.0 * math.sin(2 * math.pi * t / 25),
            }
        payload = mt_imu_payload(
            euler=(roll, pitch, yaw), acc=accel, gyro=gyro, status=gps.pop("status", 2),
            x2060=129.5, extra_blocks=extra, **gps,
        )  # fmt: skip
        frames = [mt_encode_frame(payload)]
        if self.gpssol_every and n % self.gpssol_every == self.gpssol_every - 1:
            frames.append(mt_encode_frame(encode_gpssol_payload(struct.pack(">I", n) + bytes(48))))
        out = []
        for f in frames:
            self.frames_sent += 1
            u = self.rng.random()
            if u < self.truncate_prob:
                f = f[: int(self.rng.integers(4, len(f) - 1))]
                self.frames_truncated += 1
            elif u < self.truncate_prob + self.corrupt_prob:
                f = bytearray(f)
                i = int(self.rng.integers(1, len(f)))
                f[i] ^= int(self.rng.integers(1, 256))
                f = bytes(f)
                self.frames_corrupted += 1
            out.append(f)
        return out

    def _gnss_fix(self) -> GnssFix:
        noise = self.rng.normal(0.0, 1.0, 3)
        return GnssFix(
            mcu_time_us=self.t_us,  # arrival
            pc_rx_time_ns=0,  # set by read()
            valid_time_us=self.t_us - 80_000,  # the fix is 80 ms old when it arrives
            gnss_tow_ms=345_600_000 + (self.t_us - self.t0_us - 80_000) // 1000,
            gnss_week=2400,
            fix_type=int(FixType.FIX_3D),
            num_sv=int(self.rng.integers(10, 15)),
            lat_deg=47.0 + noise[0] * 1.5 / 111_320,
            lon_deg=8.0 + noise[1] * 1.5 / 75_900,
            height_m=500.0 + noise[2] * 3.0,
            vel_ned=tuple(float(v) for v in self.rng.normal(0.0, 0.05, 3)),
            h_acc_m=1.5,
            v_acc_m=3.0,
        )


class MultiSimSource:
    """Several simulated MTData2 modules at once, like the master ESP forwarding them.

    Each module is an ordinary SimSource (own seed, own start heading); everything it produces
    is tagged with its module_id. Ids are first_id, first_id + 1, ... The sources are stepped
    one after another, so in real time they run side by side and in fast mode they advance together.
    """

    protocol = "mtdata2"
    time_source = "host"

    def __init__(self, n_modules=3, first_id=1, seed=0, **sim_kwargs):
        sim_kwargs.pop("protocol", None)
        self.sims = {}
        for k in range(n_modules):
            heading = (0.0, 0.0, 60.0 * k)  # modules start pointing in different directions
            kw = dict(sim_kwargs)
            if kw.get("gps_origin"):  # each module circles around its own point
                kw["gps_origin"] = (kw["gps_origin"][0] + 0.0004 * k, kw["gps_origin"][1])
            self.sims[first_id + k] = SimSource(
                protocol="mtdata2", seed=seed + k, attitude_deg=heading, **kw
            )
        self.baud = next(iter(self.sims.values())).baud
        self.finished = set()

    def describe(self) -> dict:
        return {
            "type": "sim-multi",
            "protocol": self.protocol,
            "modules": list(self.sims),
            "sims": {str(i): s.describe() for i, s in self.sims.items()},
            "time_source": self.time_source,
        }

    def read(self, timeout=0.05):
        out = []
        for module_id, sim in self.sims.items():
            if module_id in self.finished:
                continue
            items = sim.read(0.0)
            if items is None:
                self.finished.add(module_id)
                continue
            for item in items:
                if isinstance(item, Chunk):
                    out.append(dataclasses.replace(item, module_id=module_id))
                else:  # injected message (fake GNSS)
                    item.module_id = module_id
                    out.append(item)
        if len(self.finished) == len(self.sims):
            return None
        if not out:
            time.sleep(min(timeout, 0.002))
        return out

    def close(self):
        for sim in self.sims.values():
            sim.close()


# ---------- real device ----------


class SerialSource:
    """LISTEN-ONLY serial port reader.

    There is deliberately no write method, and the port object is private. The port is created
    closed, configured (no flow control, DTR/RTS low, exclusive) and only then opened, so pyserial
    never touches the modem lines with other settings. Tests check that nothing writes.
    """

    time_source = "host"

    def __init__(self, port: str, baud: int = 115200, serial_cls=None):
        import serial

        cls = serial_cls or serial.Serial
        ser = cls()  # no port given: not opened yet
        ser.port = port
        ser.baudrate = baud
        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE
        ser.stopbits = serial.STOPBITS_ONE
        ser.xonxoff = False
        ser.rtscts = False
        ser.dsrdtr = False
        ser.dtr = False
        ser.rts = False
        ser.timeout = 0.05
        if hasattr(ser, "exclusive"):
            ser.exclusive = True  # one reader at a time
        ser.open()
        self._ser = ser
        self.port = port
        self.baud = baud
        self.bytes_read = 0

    def describe(self) -> dict:
        return {"type": "serial", "port": self.port, "baud": self.baud, "listen_only": True,
                "time_source": self.time_source}  # fmt: skip

    def read(self, timeout=0.05):
        # blocks until at least one byte or the port timeout, then takes what is buffered
        data = self._ser.read(max(1, self._ser.in_waiting))
        if not data:
            return []
        self.bytes_read += len(data)
        return [Chunk(time.monotonic_ns(), data)]

    def close(self):
        self._ser.close()


class _DatagramBook:
    """Per-module counting shared by every source that receives netproto datagrams (UDP, USB).

    Counts accepted datagrams, damaged ones, and gaps in each module's sequence number (lost
    packets) or late/repeated numbers (out of order). Only counting: no I/O.
    """

    def _book_init(self):
        self.datagrams = Counter()  # module_id -> accepted datagrams
        self.lost = Counter()  # module_id -> datagrams missing by sequence number
        self.out_of_order = Counter()  # module_id -> late or repeated datagrams
        self.bad = 0  # damaged datagrams (wrong magic, version or length)
        self.last_error = None
        self._last_seq = {}

    def _register(self, d: Datagram, now_ns: int, out: list):
        """Count d; append a Chunk to out unless it is late or repeated."""
        self.datagrams[d.module_id] += 1
        last = self._last_seq.get(d.module_id)
        if last is not None:
            missing = (d.seq - last - 1) & 0xFFFF  # u16 wraps
            if missing == 0xFFFF or missing >= 0x8000:  # same or older number: late / repeated
                self.out_of_order[d.module_id] += 1
                return
            self.lost[d.module_id] += missing
        self._last_seq[d.module_id] = d.seq
        out.append(Chunk(now_ns, d.payload, d.module_id))

    def _book_stats(self) -> dict:
        return {
            "datagrams": {str(k): v for k, v in sorted(self.datagrams.items())},
            "bad_datagrams": self.bad,
            "lost_datagrams": sum(self.lost.values()),
            "lost_by_module": {str(k): v for k, v in sorted(self.lost.items())},
            "out_of_order_datagrams": sum(self.out_of_order.values()),
        }


class UdpSource(_DatagramBook):
    """LISTEN-ONLY UDP receiver: datagrams of netproto.py from the master ESP, one per module frame.

    The socket is only ever read (recvfrom); there is no method that sends. Every datagram becomes a
    Chunk tagged with the module_id from its header, so the pipeline treats each module like a
    serial port of its own. Damaged datagrams are counted and dropped; gaps in the per-module
    sequence number are counted as lost packets.
    """

    time_source = "host"
    MAX_PER_READ = 1000  # keeps the main loop responsive when a burst arrives

    def __init__(self, host="0.0.0.0", port=5005):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
            sock.bind((host, port))  # "Address already in use" is an error on purpose
        except OSError:
            sock.close()
            raise
        self._sock = sock
        self.host = host
        self.port = sock.getsockname()[1]  # the real port when 0 was asked for
        self.bytes_read = 0
        self._book_init()

    def describe(self) -> dict:
        return {"type": "udp", "host": self.host, "port": self.port, "listen_only": True,
                "time_source": self.time_source}  # fmt: skip

    def stats(self) -> dict:
        return {**self._book_stats(), "bytes": self.bytes_read}

    def read(self, timeout=0.05):
        out = []
        self._sock.settimeout(timeout)
        for _ in range(self.MAX_PER_READ):
            try:
                data, _ = self._sock.recvfrom(65535)
            except (TimeoutError, BlockingIOError):
                break
            self._accept(data, time.monotonic_ns(), out)
            self._sock.settimeout(0)  # after the first one take only what is already queued
        return out

    def _accept(self, data: bytes, now_ns: int, out: list):
        try:
            d = parse_datagram(data)
        except DatagramError as e:
            self.bad += 1
            self.last_error = str(e)
            return
        self.bytes_read += len(d.payload)
        self._register(d, now_ns, out)

    def close(self):
        self._sock.close()


def _is_single_mtdata2_frame(buf: bytes) -> bool:
    """True if buf is exactly one MTData2 frame: FA FF 36 LEN payload CS, sum from FF to CS = 0.

    Same rule as mtdata2_is_single_frame() in the master firmware, which only forwards such frames.
    """
    n = len(buf)
    if n < 5 or buf[:3] != b"\xfa\xff\x36":
        return False
    hdr, plen = 4, buf[3]
    if plen == 0xFF:  # extended length: 2 bytes big-endian
        if n < 7:
            return False
        hdr, plen = 6, (buf[4] << 8) | buf[5]
    return hdr + plen + 1 == n and sum(buf[1:]) & 0xFF == 0


class MasterSerialSource(SerialSource, _DatagramBook):
    """LISTEN-ONLY: the master ESP32 over its USB serial port (no Ethernet needed).

    The master writes two kinds of things into that port, mixed in one byte stream:
      - binary datagrams in exactly the format of netproto.py, so everything behind this source is
        identical to the UDP path:   'I' 'V' 1 module_id seq len payload
      - text lines that start with "# " and end with a newline: the master's own diagnostics (link
        state, and per slave its status packet plus a diagnosis such as UART_SILENT).
    Anything else (ROM boot messages, a half-written line, noise) is skipped and counted.

    Being a SerialSource, it opens the port with DTR/RTS low and has no way to write. A reading
    can start or end anywhere inside a datagram, so the bytes go through a small re-framer: a
    datagram is accepted only when the header is sane AND its payload is one valid MTData2 frame
    (sync, length, checksum). A false start inside binary data, or a datagram cut off in the
    middle, is therefore dropped without swallowing the real datagram behind it.
    """

    time_source = "host"
    PAYLOAD_SYNC = b"\xfa\xff\x36"
    MAX_TEXT_LINE = 400
    STALE_S = 3.5  # the master prints once a second: older than this = it stopped

    def __init__(self, port: str, baud: int = 115200, serial_cls=None, strict_payload=True):
        super().__init__(port, baud, serial_cls)  # baud: meaningless on USB, pyserial wants it
        self._book_init()
        self.strict_payload = strict_payload
        self._buf = bytearray()
        self._lock = threading.Lock()  # diag data is read by the GUI thread
        self.payload_bytes = 0
        self.skipped_bytes = 0  # bytes that were neither a datagram nor a text line
        self.diag = deque(maxlen=200)  # the last text lines, without the "# "
        self.peers = {}  # module_id -> key=value fields of its line, plus "_t" (monotonic seconds)
        self.link = {}  # fields of the master's "link" line
        self._link_t = None
        self.on_diag = None  # optional callable(line), called from the reading thread

    def describe(self) -> dict:
        return {"type": "master-serial", "port": self.port, "listen_only": True,
                "time_source": self.time_source}  # fmt: skip

    def stats(self) -> dict:
        return {
            **self._book_stats(),
            "bytes": self.payload_bytes,
            "port_bytes": self.bytes_read,
            "skipped_bytes": self.skipped_bytes,
            "diag_lines": len(self.diag),
        }

    # ----- reading -----

    def read(self, timeout=0.05):
        data = self._ser.read(max(1, self._ser.in_waiting))
        if not data:
            return []
        self.bytes_read += len(data)
        out = []
        self._feed(data, time.monotonic_ns(), out)
        return out

    def _feed(self, data: bytes, now_ns: int, out: list):
        buf = self._buf
        buf += data
        while buf:
            first = buf[0]
            if first == 0x49:  # 'I': start of a datagram?
                step = self._take_datagram(buf, now_ns, out)
            elif first == 0x23:  # '#': start of a text line?
                step = self._take_text(buf)
            else:
                step = False
            if step is None:  # a real start, but the rest has not arrived yet
                break
            if step is False:  # not a start (or a broken one): drop up to the next candidate
                cut = 1 if first in (0x49, 0x23) else 0
                nxt = len(buf)
                for c in (b"I", b"#"):
                    k = buf.find(c, 1)
                    if k != -1:
                        nxt = min(nxt, k)
                cut = max(cut, nxt)
                self.skipped_bytes += cut
                del buf[:cut]

    def _take_datagram(self, buf: bytearray, now_ns: int, out: list):
        """True: consumed a datagram. None: need more bytes. False: not a datagram here."""
        magic_ver = MAGIC + bytes([VERSION])
        have = min(len(buf), len(magic_ver))
        if bytes(buf[:have]) != magic_ver[:have]:
            return False
        if len(buf) < HEADER.size:
            return None
        _, _, module_id, seq, length = HEADER.unpack_from(buf)
        if length > MAX_PAYLOAD:
            self.bad += 1
            self.last_error = f"length field {length} too large"
            return False
        sync = len(self.PAYLOAD_SYNC)
        if self.strict_payload and len(buf) >= HEADER.size + sync:
            if bytes(buf[HEADER.size : HEADER.size + sync]) != self.PAYLOAD_SYNC:
                self.bad += 1
                self.last_error = "payload does not start like an MTData2 frame"
                return False
        total = HEADER.size + length
        if len(buf) < total:
            return None
        payload = bytes(buf[HEADER.size : total])
        if self.strict_payload and not _is_single_mtdata2_frame(payload):
            # The master only sends whole, valid frames, so this is a datagram cut off earlier whose
            # missing tail was filled with the next datagram's bytes. Skip one byte and look again:
            # the next real datagram is still in the buffer.
            self.bad += 1
            self.last_error = "payload is not one valid MTData2 frame"
            return False
        del buf[:total]
        self.payload_bytes += len(payload)
        self._register(Datagram(module_id, seq, payload), now_ns, out)
        return True

    def _take_text(self, buf: bytearray):
        """True: consumed a line. None: need more bytes. False: not a text line."""
        if len(buf) >= 2 and buf[1] != 0x20:
            return False  # our lines start with "# "
        nl = buf.find(b"\n", 0, self.MAX_TEXT_LINE)
        body = buf[: nl if nl >= 0 else len(buf)]
        if any((c < 0x20 and c not in (0x09, 0x0D)) or c > 0x7E for c in body):
            return False  # binary data: this '#' was not a line start
        if nl < 0:
            return None if len(buf) < self.MAX_TEXT_LINE else False
        line = bytes(buf[:nl]).decode("ascii").rstrip("\r")
        del buf[: nl + 1]
        self._on_line(line)
        return True

    # ----- the master's diagnostics -----

    def _on_line(self, line: str):
        text = line[2:].strip()
        fields = {}
        for tok in text.split():
            if "=" in tok:
                k, v = tok.split("=", 1)
                fields[k] = v
        now = time.monotonic()
        with self._lock:
            self.diag.append(text)
            if text.startswith("link "):
                self.link, self._link_t = fields, now
            elif text.startswith("id=") and fields.get("id", "").isdigit():
                self.peers[int(fields["id"])] = {**fields, "_t": now}
        if self.on_diag:
            self.on_diag(text)

    def diag_lines(self) -> list[str]:
        with self._lock:
            return list(self.diag)

    DIAG_TEXT = {
        "UART_SILENT": "UART мовчить (проводка, точка підключення?)",
        "NO_VALID_FRAMES": "UART є, але кадрів нема (швидкість? рівні?)",
        "RESTARTED": "слейв перезавантажується (живлення?)",
        "NO_ACK": "майстер не підтверджує прийом (MAC, канал, відстань)",
        "QUEUE_DROPS": "радіо не встигає за потоком",
        "BEACON": "тестовий режим радіо (UART ігнорується)",
    }

    def link_summary(self) -> str:
        """One line for the status bar: what is wrong, or that everything is fine."""
        with self._lock:
            peers = {k: dict(v) for k, v in self.peers.items()}
            link_t = self._link_t
        if self.bytes_read == 0:
            return "ESP-майстер: з порту нічого не приходить (той порт? прошивка майстра?)"
        if link_t is None:
            if self.datagrams:
                return "ESP-майстер: дані йдуть (діагностики майстра немає)"
            return "З порту йдуть байти, але це не схоже на майстра (інший порт?)"
        if time.monotonic() - link_t > self.STALE_S:
            return "ESP-майстер замовк"
        if not peers:
            return (
                "ESP-майстер на зв'язку, але жодного слейва не чути "
                "(живлення? канал? тест esp32c3_beacon)"
            )
        problems = []
        for mid, kv in sorted(peers.items()):
            hb = kv.get("hb")
            if hb == "lost":
                problems.append(f"#{mid}: слейв замовк")
            elif hb == "none":
                problems.append(f"#{mid}: слейв не шле статус (стара прошивка?)")
            elif kv.get("diag") in self.DIAG_TEXT:
                problems.append(f"#{mid}: {self.DIAG_TEXT[kv['diag']]}")
        if problems:
            return "ESP: " + "; ".join(problems)
        rssi = []
        for kv in peers.values():
            try:
                rssi.append(int(kv["rssi"]))
            except (KeyError, ValueError):
                pass
        tail = f", RSSI {min(rssi)} дБм" if rssi else ""
        return f"ESP: {len(peers)} мод., усе гаразд{tail}"


# ---------- files ----------


class RawFileSource:
    """Replays a session's raw.bin (optionally in real time), plus its recorded commands."""

    def __init__(self, session_dir, realtime=False, speed=1.0, with_commands=True):
        import json

        from .logger import read_raw, read_stream
        from .messages import Command

        self.session = Path(session_dir)
        meta_path = self.session / "meta.json"
        self.meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
        src = self.meta.get("source", {})
        self.protocol = src.get("protocol")  # sim sessions: the protocol the sim produced
        self.baud = src.get("baud") or self.meta.get("config", {}).get("serial", {}).get("baud")
        self.time_source = src.get("time_source", "host")
        self.chunks = read_raw(self.session / "raw.bin")
        self.commands = deque(read_stream(self.session, Command) if with_commands else [])
        self.realtime = realtime
        self.speed = speed
        self.first_pc = None
        self.wall0 = None

    def describe(self) -> dict:
        return {"type": "replay", "session": str(self.session), "original_source":
                self.meta.get("source"), "time_source": self.time_source}  # fmt: skip

    def read(self, timeout=0.05):
        chunk = next(self.chunks, None)
        if chunk is None:
            rest, self.commands = list(self.commands), deque()
            return rest or None
        if self.realtime:
            if self.first_pc is None:
                self.first_pc, self.wall0 = chunk.pc_rx_time_ns, time.monotonic()
            wait = (chunk.pc_rx_time_ns - self.first_pc) * 1e-9 / self.speed
            delay = self.wall0 + wait - time.monotonic()
            if delay > 0:
                time.sleep(delay)
        out = []
        while self.commands and self.commands[0].pc_rx_time_ns <= chunk.pc_rx_time_ns:
            out.append(self.commands.popleft())
        return [*out, chunk]

    def close(self):
        pass


class BytesSource:
    """A raw dump without timestamps (mtdata2_decoder.py --raw). Time is made up from the byte
    position at `byte_rate` bytes/s: time_source "synthetic"."""

    time_source = "synthetic"

    def __init__(self, path, byte_rate=11520.0, chunk=256, t0_ns=0):
        self.path = str(path)
        self.data = Path(path).read_bytes()
        self.byte_rate = byte_rate
        self.baud = byte_rate * 10  # so host time reconstruction gives the exact byte time
        self.chunk = chunk
        self.t0_ns = t0_ns
        self.pos = 0

    def describe(self) -> dict:
        return {"type": "import-raw", "file": self.path, "bytes": len(self.data),
                "byte_rate": self.byte_rate, "baud": self.baud,
                "time_source": self.time_source}  # fmt: skip

    def read(self, timeout=0.05):
        if self.pos >= len(self.data):
            return None
        end = min(self.pos + self.chunk, len(self.data))
        chunk = Chunk(self.t0_ns + round(end / self.byte_rate * 1e9), self.data[self.pos : end])
        self.pos = end
        return [chunk]

    def close(self):
        pass


class MessageSource:
    """Hands out stored messages (sessions without raw bytes, e.g. CSV imports).
    Derived fields (*_cal, *_body) are cleared so the pipeline recomputes them."""

    def __init__(self, messages, describe: dict, batch=500):
        self.messages = messages
        self.info = describe
        self.batch = batch
        self.pos = 0
        self.time_source = next((m.time_source for m in messages), "host")

    @classmethod
    def from_session(cls, session_dir):
        from .logger import read_stream
        from .messages import GnssRaw, ImuSample, RawBlock

        msgs = []
        for c in (ImuSample, GnssFix, GnssRaw, RawBlock):
            msgs += read_stream(session_dir, c)
        for m in msgs:
            if isinstance(m, ImuSample):
                m.accel_cal = m.gyro_cal = m.accel_body = m.gyro_body = m.quat_body = None
        msgs.sort(key=lambda m: (m.pc_rx_time_ns, m.t_us))  # arrival order, then frame time
        return cls(msgs, {"type": "replay-messages", "session": str(session_dir)})

    def describe(self) -> dict:
        return self.info | {"time_source": self.time_source}

    def read(self, timeout=0.05):
        if self.pos >= len(self.messages):
            return None
        out = self.messages[self.pos : self.pos + self.batch]
        self.pos += self.batch
        return out

    def close(self):
        pass


# frame sizes of the device, to spread CSV rows of one USB chunk over time
CSV_IMU_FRAME_BYTES = 94
CSV_GPSSOL_FRAME_BYTES = 60


def csv_messages(path, baud=115200, chunk_gap_s=0.001) -> list:
    """Messages from a CSV written by mtdata2_decoder.py.

    Columns: t,roll,pitch,yaw,ax,ay,az,gx,gy,gz,status,lat,lon,alt,vx,vy,vz,x2060.
    `t` is the time the decoder handled the frame, so all frames of one USB chunk carry nearly
    the same t. Rows closer than chunk_gap_s form one chunk; each gets the chunk time minus the
    wire time of the frames after it (94 bytes per IMU frame, 60 per GpsSol frame). Bytes of
    bad frames are unknown, so this is approximate. Rows with empty ax are GpsSol frames whose
    bytes the CSV did not keep: they become GnssRaw with data_hex None.
    """
    import csv

    from .messages import GnssRaw, ImuSample
    from .protocol_mtdata2 import decode_status

    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    groups, cur, last_t = [], [], None
    for r in rows:
        t = float(r["t"])
        if cur and t - last_t > chunk_gap_s:
            groups.append(cur)
            cur = []
        cur.append(r)
        last_t = t
    if cur:
        groups.append(cur)

    def num(r, k):
        return float(r[k]) if r.get(k, "") != "" else None

    out = []
    for g in groups:
        chunk_ns = round(float(g[0]["t"]) * 1e9)
        sizes = [CSV_IMU_FRAME_BYTES if r["ax"] != "" else CSV_GPSSOL_FRAME_BYTES for r in g]
        after = [sum(sizes[i + 1 :]) for i in range(len(g))]
        for r, n_after in zip(g, after, strict=True):
            times = {
                "pc_rx_time_ns": chunk_ns,
                "host_time_ns": chunk_ns - n_after * 10_000_000_000 // baud,
                "time_source": "host",
            }
            if r["ax"] == "":
                out.append(GnssRaw(**times, kind="0x8840", data_hex=None))
                continue
            status = int(float(r["status"])) if r["status"] != "" else None
            x2060 = num(r, "x2060")
            out.append(
                ImuSample(
                    **times,
                    imu_id=0,
                    accel=(num(r, "ax"), num(r, "ay"), num(r, "az")),
                    gyro=(num(r, "gx"), num(r, "gy"), num(r, "gz")),
                    euler_deg=(num(r, "roll"), num(r, "pitch"), num(r, "yaw")),
                    status=status,
                    extra={"0x2060": x2060} if x2060 is not None else None,
                )
            )
            if r["lat"] != "":
                fix = status is not None and decode_status(status)["gnss_fix"]
                vel = (num(r, "vx"), num(r, "vy"), num(r, "vz"))
                out.append(
                    GnssFix(
                        **times,
                        valid_time_us=None,
                        gnss_tow_ms=None,
                        gnss_week=None,
                        fix_type=int(FixType.FIX_3D if fix else FixType.NONE),
                        num_sv=None,
                        lat_deg=num(r, "lat"),
                        lon_deg=num(r, "lon"),
                        height_m=num(r, "alt") if r["alt"] != "" else float("nan"),
                        vel_ned=None,
                        vel_xyz=vel if vel[0] is not None else None,
                        h_acc_m=None,
                        v_acc_m=None,
                    )
                )
    return out


def session_source(session):
    """raw.bin when the session has bytes, else its stored messages (e.g. CSV imports)."""
    raw = Path(session) / "raw.bin"
    if raw.exists() and raw.stat().st_size > 0:
        return RawFileSource(session)
    return MessageSource.from_session(session)
