"""Everything the GUI shows, kept per module and indexed by time. No Qt in here.

The pipeline thread calls on_* (through the router); the GUI thread asks for windows of data and
for "the pose of module N at time t". One lock protects both sides. Live mode asks for the
newest time, analysis mode for any time: the same queries serve both.

Time is seconds since the first message of the store (t = msg.t_us * 1e-6 - t0); all modules share
one host clock, so their times are comparable.
"""

import bisect
import math
import threading
import time
from dataclasses import dataclass

import numpy as np

from ..frames import G, geodetic_to_ned, quat_rotate, quat_to_euler
from ..messages import GnssFix, GnssRaw, ImuSample, State
from ..sources import Chunk

MIN_FIX_TYPE = 2  # 2D fix or better: the position can be used
ALIVE_S = 1.5  # a module counts as alive if data came in within this time


class Series:
    """Rows sorted by time: parallel lists t and rows (tuples)."""

    def __init__(self):
        self.t: list[float] = []
        self.rows: list[tuple] = []

    def add(self, t: float, row: tuple):
        if self.t and t < self.t[-1]:  # out of order (should not happen): keep the list sorted
            k = bisect.bisect_right(self.t, t)
            self.t.insert(k, t)
            self.rows.insert(k, row)
        else:
            self.t.append(t)
            self.rows.append(row)

    def __len__(self):
        return len(self.t)

    def index_at(self, t: float) -> int:
        """Index of the last row with time <= t, or -1."""
        return bisect.bisect_right(self.t, t) - 1

    def at(self, t: float):
        k = self.index_at(t)
        return (self.t[k], self.rows[k]) if k >= 0 else None

    def window(self, t0: float, t1: float):
        """(times, rows as array) for t0 <= t <= t1."""
        a, b = bisect.bisect_left(self.t, t0), bisect.bisect_right(self.t, t1)
        if b <= a:
            return np.zeros(0), np.zeros((0, 0))
        return np.asarray(self.t[a:b]), np.asarray(self.rows[a:b], dtype=float)

    def trim_before(self, t: float):
        k = bisect.bisect_left(self.t, t)
        if k:
            del self.t[:k], self.rows[:k]


# columns of the imu series
IMU_ACCEL_BODY, IMU_GYRO_BODY, IMU_ACCEL_RAW, IMU_GYRO_RAW, IMU_EULER_DEV, IMU_STATUS = (
    slice(0, 3), slice(3, 6), slice(6, 9), slice(9, 12), slice(12, 15), 15,
)  # fmt: skip
# columns of the gps series
GPS_LAT, GPS_LON, GPS_H, GPS_FIX, GPS_NSV, GPS_VEL, GPS_ENU = (
    0,
    1,
    2,
    3,
    4,
    slice(5, 8),
    slice(8, 11),
)


@dataclass
class Pose:
    """What the 3D view needs for one module at one moment."""

    t: float
    q: tuple  # body (FRD) -> world (NED)
    position: tuple | None  # (east, north, up) in metres from the store origin, None = no GPS
    accel_body: tuple | None
    free_accel: tuple | None  # body frame, gravity removed
    euler_deg: tuple  # roll, pitch, yaw of the host estimate
    gps_valid: bool


class ModuleData:
    def __init__(self, module_id: int):
        self.module_id = module_id
        self.imu = Series()
        self.state = Series()
        self.gps = Series()
        self.gps_raw = Series()  # (hex,)
        self.chunks = Series()  # (bytes,)
        self.last_rx_wall = 0.0  # time.monotonic() of the newest data, live mode
        self.counts = {"imu": 0, "gps": 0}


class Store:
    def __init__(self, keep_s: float | None = 1800.0):
        """keep_s: live mode forgets data older than this (None = keep all, for analysis)."""
        self.keep_s = keep_s
        self.lock = threading.RLock()
        self.modules: dict[int, ModuleData] = {}
        self.t0_us: int | None = None
        self.t_last = 0.0
        self.origin: tuple | None = None  # (lat, lon, h) of the first valid fix of any module
        self._adds = 0

    # ----- feeding (pipeline thread) -----

    def attach(self, router):
        router.subscribe(ImuSample, self.on_imu)
        router.subscribe(State, self.on_state)
        router.subscribe(GnssFix, self.on_gnss)
        router.subscribe(GnssRaw, self.on_gnss_raw)
        router.subscribe(Chunk, self.on_chunk)

    def _t(self, t_us: int) -> float:
        if self.t0_us is None:
            self.t0_us = t_us
        return (t_us - self.t0_us) * 1e-6

    def _module(self, module_id: int) -> ModuleData:
        if module_id not in self.modules:
            self.modules[module_id] = ModuleData(module_id)
        return self.modules[module_id]

    def _touch(self, m: ModuleData, t: float):
        m.last_rx_wall = time.monotonic()
        self.t_last = max(self.t_last, t)
        self._adds += 1
        if self.keep_s and self._adds % 2000 == 0:
            for mod in self.modules.values():
                for s in (mod.imu, mod.state, mod.gps, mod.gps_raw, mod.chunks):
                    s.trim_before(self.t_last - self.keep_s)

    def on_imu(self, s: ImuSample):
        if s.imu_id != 0:
            return
        nan3 = (math.nan,) * 3
        body_a = s.accel_body if s.accel_body is not None else s.accel
        body_g = s.gyro_body if s.gyro_body is not None else s.gyro
        row = (*body_a, *body_g, *s.accel, *s.gyro, *(s.euler_deg or nan3),
               -1 if s.status is None else s.status)  # fmt: skip
        with self.lock:
            m = self._module(s.module_id)
            t = self._t(s.t_us)
            m.imu.add(t, row)
            m.counts["imu"] += 1
            self._touch(m, t)

    def on_state(self, st: State):
        if st.source != "vehicle":
            return
        roll, pitch, yaw = (math.degrees(a) for a in quat_to_euler(st.q))
        with self.lock:
            m = self._module(st.module_id)
            t = self._t(st.t_us)
            m.state.add(t, (*st.q, roll, pitch, yaw))
            self._touch(m, t)

    def on_gnss(self, f: GnssFix):
        with self.lock:
            m = self._module(f.module_id)
            t = self._t(f.t_us)
            enu = (math.nan,) * 3
            if f.fix_type >= MIN_FIX_TYPE and math.isfinite(f.lat_deg) and math.isfinite(f.lon_deg):
                h = f.height_m if math.isfinite(f.height_m) else 0.0
                if self.origin is None:
                    self.origin = (f.lat_deg, f.lon_deg, h)
                ned = geodetic_to_ned(f.lat_deg, f.lon_deg, h, self.origin)
                enu = (float(ned[1]), float(ned[0]), float(-ned[2]))
            vel = f.vel_xyz or f.vel_ned or (math.nan,) * 3
            m.gps.add(t, (f.lat_deg, f.lon_deg, f.height_m, f.fix_type, f.num_sv or 0, *vel, *enu))
            m.counts["gps"] += 1
            self._touch(m, t)

    def on_gnss_raw(self, g: GnssRaw):
        with self.lock:
            m = self._module(g.module_id)
            m.gps_raw.add(self._t(g.t_us), (g.data_hex or "",))

    def on_chunk(self, c: Chunk):
        with self.lock:
            m = self._module(c.module_id)
            m.chunks.add(self._t(c.pc_rx_time_ns // 1000), (c.data,))

    # ----- asking (GUI thread) -----

    def module_ids(self) -> list[int]:
        with self.lock:
            return sorted(self.modules)

    def span(self) -> tuple[float, float]:
        """First and last time with data (seconds)."""
        with self.lock:
            firsts = [m.imu.t[0] for m in self.modules.values() if len(m.imu)]
            lasts = [m.imu.t[-1] for m in self.modules.values() if len(m.imu)]
        return (min(firsts), max(lasts)) if firsts else (0.0, 0.0)

    def now(self) -> float:
        """Newest time (live view time)."""
        with self.lock:
            return self.t_last

    def alive(self, module_id: int, at: float | None = None) -> bool:
        """Live (at=None): data in the last ALIVE_S of wall time. Analysis: near time `at`."""
        with self.lock:
            m = self.modules.get(module_id)
            if m is None:
                return False
            if at is None:
                return time.monotonic() - m.last_rx_wall < ALIVE_S
            row = m.imu.at(at)
            return row is not None and at - row[0] < ALIVE_S

    def rate(self, module_id: int, at: float | None = None, window: float = 1.0) -> float:
        """IMU samples per second over the last `window` seconds before `at` (None = newest)."""
        with self.lock:
            m = self.modules.get(module_id)
            if m is None or not len(m.imu):
                return 0.0
            at = self.t_last if at is None else at
            hi = m.imu.index_at(at)
            lo = m.imu.index_at(at - window)
            return max(0, hi - lo) / window

    def series(self, module_id: int, kind: str, t0: float, t1: float):
        """(times, (N, 3) values) for kind: accel, gyro, euler, gps_enu (east, north, up)."""
        with self.lock:
            m = self.modules.get(module_id)
            if m is None:
                return np.zeros(0), np.zeros((0, 3))
            if kind in ("accel", "gyro"):
                t, r = m.imu.window(t0, t1)
                if not len(t):
                    return t, np.zeros((0, 3))
                return t, (
                    r[:, IMU_ACCEL_BODY] if kind == "accel" else np.degrees(r[:, IMU_GYRO_BODY])
                )
            if kind == "euler":
                t, r = m.state.window(t0, t1)
                return (t, r[:, 4:7]) if len(t) else (t, np.zeros((0, 3)))
            if kind == "gps_enu":  # metres from the store origin
                t, r = m.gps.window(t0, t1)
                if not len(t):
                    return t, np.zeros((0, 3))
                ok = ~np.isnan(r[:, GPS_ENU.start])
                return t[ok], r[ok, GPS_ENU]
        raise ValueError(kind)

    def pose_at(self, module_id: int, t: float | None = None, use_gps=True, g=G) -> Pose | None:
        """Newest pose at time t (None = newest data). None if nothing before t yet."""
        with self.lock:
            m = self.modules.get(module_id)
            if m is None:
                return None
            t = self.t_last if t is None else t
            st = m.state.at(t)
            if st is None:
                return None
            q = tuple(st[1][:4])
            euler = tuple(st[1][4:7])
            imu = m.imu.at(t)
            accel = tuple(imu[1][IMU_ACCEL_BODY]) if imu else None
            free = None
            if accel is not None:
                g_body = quat_rotate(_conj(q), np.array([0.0, 0.0, g]))  # gravity in body axes
                free = tuple(float(v) for v in np.asarray(accel) + g_body)
            pos, valid = None, False
            if use_gps:
                k = m.gps.index_at(t)
                while k >= 0 and math.isnan(m.gps.rows[k][GPS_ENU.start]):
                    k -= 1  # newest row with a valid position
                if k >= 0:
                    pos, valid = tuple(m.gps.rows[k][GPS_ENU]), True
            return Pose(t, q, pos, accel, free, euler, valid)

    def trail(self, module_id: int, t: float, seconds: float, n: int = 8):
        """Up to n past (time, position, q) over the last `seconds`, oldest first (the ghosts)."""
        with self.lock:
            m = self.modules.get(module_id)
            if m is None:
                return []
            a, b = bisect.bisect_left(m.gps.t, t - seconds), bisect.bisect_right(m.gps.t, t)
            idx = [k for k in range(a, b) if not math.isnan(m.gps.rows[k][GPS_ENU.start])]
            if not idx:
                return []
            picks = sorted({idx[round(i * (len(idx) - 1) / max(1, n - 1))] for i in range(n)})
            out = []
            for k in picks:
                st = m.state.at(m.gps.t[k])
                if st is not None:
                    out.append((m.gps.t[k], tuple(m.gps.rows[k][GPS_ENU]), tuple(st[1][:4])))
            return out

    def raw_table(self, module_id: int, t: float | None = None) -> list[tuple[str, str, str]]:
        """Decoded MTData2 blocks at time t: [(xdi, name, value text)]."""
        with self.lock:
            m = self.modules.get(module_id)
            if m is None:
                return []
            t = self.t_last if t is None else t
            rows = []
            imu, fix, sol = m.imu.at(t), m.gps.at(t), m.gps_raw.at(t)

            def f3(v):
                return ", ".join(f"{x:+.5f}" for x in v)

            if imu:
                r = imu[1]
                status = int(r[IMU_STATUS])
                bits = (f"selftest {status & 1}, filter {status >> 1 & 1}, gnss {status >> 2 & 1}"
                        if status >= 0 else "-")  # fmt: skip
                rows += [
                    ("0x2030", "EulerAngles (deg)", f3(r[IMU_EULER_DEV])),
                    ("0x4020", "Acceleration (raw)", f3(r[IMU_ACCEL_RAW])),
                    ("0x8020", "RateOfTurn (raw)", f3(r[IMU_GYRO_RAW])),
                    ("0xE010", "StatusByte", f"0x{status:02X}  {bits}" if status >= 0 else "-"),
                ]
            if fix:
                r = fix[1]
                note = "" if r[GPS_FIX] >= MIN_FIX_TYPE else "   (no fix: placeholder)"
                rows += [
                    ("0x5040", "LatLon", f"{r[GPS_LAT]:.6f}, {r[GPS_LON]:.6f}{note}"),
                    ("0x5020", "AltitudeEllipsoid", f"{r[GPS_H]:.2f}"),
                    ("0xD010", "VelocityXYZ", f3(r[GPS_VEL])),
                ]
            if sol and sol[1][0]:
                rows.append(("0x8840", "GpsSol (raw hex)", sol[1][0][:96]))
            return rows

    def raw_hex(self, module_id: int, t: float | None = None, n: int = 12) -> list[str]:
        """The last n received byte chunks up to time t, as hex lines (oldest first)."""
        with self.lock:
            m = self.modules.get(module_id)
            if m is None:
                return []
            t = self.t_last if t is None else t
            k = m.chunks.index_at(t)
            rows = m.chunks.rows[max(0, k - n + 1) : k + 1]
            times = m.chunks.t[max(0, k - n + 1) : k + 1]
            return [f"{tt:9.3f}  {r[0].hex(' ')}" for tt, r in zip(times, rows, strict=True)]


def _conj(q):
    return (q[0], -q[1], -q[2], -q[3])
