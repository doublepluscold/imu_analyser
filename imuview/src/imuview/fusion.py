"""Attitude estimation (Mahony, Madgwick, passthrough) and gyro bias calibration.

Estimator interface (anything with these works in the pipeline, e.g. a future INS EKF):
    name: str
    process(msg) -> State | None     # ignore messages you can't use
    params() -> dict                 # saved into meta.json

Both filters are the textbook versions, which compare the measured accel direction with
world +z seen from the body. In NED world +z is down, and at rest the accelerometer
reads (0, 0, -g) in FRD, so we feed them `down` = -accel / |accel| (gravity direction).
Without a magnetometer yaw is not observable and slowly drifts. That is expected.

Filters use the body-frame vectors (accel_body / gyro_body) that the pipeline fills; a sample
built without the pipeline (tests) falls back to accel / gyro.

dt: from the device clock when the stream has one. Otherwise from the reconstructed host time,
clamped to [0.5, 3] x the running median interval, because host times jitter (USB chunks).
"""

import math
import statistics
from collections import deque

import numpy as np

from .frames import quat_from_accel, quat_mul, quat_normalize
from .messages import ImuSample, State, times_of

MAX_DT = 0.1  # s; a longer gap means lost data, we skip the step instead of integrating


class IntervalClock:
    """Turns message times into integration steps.

    step(t_us, trusted) -> (dt [s] or None to skip, raw interval [s] or None).
    trusted (device clock): dt = raw interval.
    untrusted (host / synthetic): dt = raw clamped to [lo, hi] x median of recent intervals.
    A raw interval above hi x median is a gap: `gaps` counts them, `on_gap(raw, median)` is called.
    """

    def __init__(self, lo=0.5, hi=3.0, window=51, min_history=5):
        self.lo, self.hi = lo, hi
        self.recent = deque(maxlen=window)
        self.min_history = min_history
        self.last_t = None
        self.gaps = 0
        self.on_gap = None

    def median(self) -> float | None:
        return statistics.median(self.recent) if len(self.recent) >= self.min_history else None

    def step(self, t_us, trusted=True):
        last, self.last_t = self.last_t, t_us
        if last is None:
            return None, None
        raw = (t_us - last) * 1e-6
        if trusted:
            return (raw if 0 < raw <= MAX_DT else None), raw
        med = self.median()
        if raw > 0:
            self.recent.append(raw)
        if med is None:
            return (raw if 0 < raw <= MAX_DT else None), raw
        if raw > self.hi * med:
            self.gaps += 1
            if self.on_gap:
                self.on_gap(raw, med)
        return min(max(raw, self.lo * med), self.hi * med), raw


def mahony_step(q, gyro, down, dt, kp, ki, integral):
    """One Mahony update. down = unit gravity direction in body. Returns (q, integral)."""
    w, x, y, z = q
    v = np.array([2 * (x * z - w * y), 2 * (w * x + y * z), w * w - x * x - y * y + z * z])
    # e = down x v: rotation that would align measured and estimated gravity
    # (written out by hand, np.cross is very slow for single vectors)
    e = np.array(
        [
            down[1] * v[2] - down[2] * v[1],
            down[2] * v[0] - down[0] * v[2],
            down[0] * v[1] - down[1] * v[0],
        ]
    )
    if ki > 0:
        integral = integral + ki * e * dt
    omega = gyro + kp * e + integral
    q = q + 0.5 * quat_mul(q, [0.0, *omega]) * dt
    return quat_normalize(q), integral


def madgwick_step(q, gyro, down, dt, beta):
    """One Madgwick (IMU version) update. down = unit gravity direction in body."""
    w, x, y, z = q
    f = np.array(
        [
            2 * (x * z - w * y) - down[0],
            2 * (w * x + y * z) - down[1],
            2 * (0.5 - x * x - y * y) - down[2],
        ]
    )
    j = np.array(
        [
            [-2 * y, 2 * z, -2 * w, 2 * x],
            [2 * x, 2 * w, 2 * z, 2 * y],
            [0.0, -4 * x, -4 * y, 0.0],
        ]
    )
    grad = j.T @ f
    norm = math.sqrt(grad @ grad)
    q_dot = 0.5 * quat_mul(q, [0.0, *gyro])
    if norm > 0:
        q_dot = q_dot - beta * grad / norm
    return quat_normalize(q + q_dot * dt)


class AttitudeEstimator:
    """Orientation of one IMU. algo = "mahony" | "madgwick" | "passthrough"."""

    def __init__(self, algo="mahony", imu_id=0, source="vehicle", kp=1.0, ki=0.05, beta=0.05):
        if algo not in ("mahony", "madgwick", "passthrough"):
            raise ValueError(f"unknown estimator {algo!r}")
        self.name = algo
        self.imu_id = imu_id
        self.source = source
        self.kp, self.ki, self.beta = kp, ki, beta
        self.gyro_bias = np.zeros(3)  # set by the gyro calibration
        self.q = None
        self.integral = np.zeros(3)
        self.clock = IntervalClock()

    def params(self) -> dict:
        p = {"algo": self.name, "imu_id": self.imu_id}
        if self.name == "mahony":
            p |= {"kp": self.kp, "ki": self.ki}
        if self.name == "madgwick":
            p |= {"beta": self.beta}
        return p

    def process(self, msg) -> State | None:
        if not isinstance(msg, ImuSample) or msg.imu_id != self.imu_id:
            return None
        if self.name == "passthrough":
            quat = msg.quat_body if msg.quat_body is not None else msg.quat
            if quat is None:
                return None
            self.q = quat_normalize(quat)
        else:
            self._update(msg)
        if self.q is None:
            return None
        return State(
            **times_of(msg),
            q=tuple(float(v) for v in self.q),
            estimator=self.name,
            source=self.source,
        )

    def _update(self, msg):
        accel, gyro = body_vectors(msg)
        if not (np.isfinite(accel).all() and np.isfinite(gyro).all()):
            return
        gyro = gyro - self.gyro_bias
        trusted = msg.time_source == "device"
        dt, raw = self.clock.step(msg.t_us, trusted)
        if self.q is None or raw is None or raw <= 0:  # first sample or clock jumped back
            if np.linalg.norm(accel) > 1e-6:
                self.q = quat_from_accel(accel)
                self.integral = np.zeros(3)
            return
        if dt is None:
            return
        norm = math.sqrt(accel @ accel)
        down = -accel / norm if norm > 1e-6 else np.zeros(3)  # free fall -> gyro only
        if self.name == "mahony":
            self.q, self.integral = mahony_step(
                self.q, gyro, down, dt, self.kp, self.ki, self.integral
            )
        else:
            beta = self.beta if down.any() else 0.0
            self.q = madgwick_step(self.q, gyro, down, dt, beta)


def body_vectors(s: ImuSample) -> tuple[np.ndarray, np.ndarray]:
    accel = s.accel_body if s.accel_body is not None else s.accel
    gyro = s.gyro_body if s.gyro_body is not None else s.gyro
    return np.array(accel, dtype=float), np.array(gyro, dtype=float)


class GyroCalibrator:
    """Averages body-frame gyro over `seconds` of message time. Fails if the IMU moves.

    feed() returns None while collecting, then "done" (see .bias) or "moved".
    """

    def __init__(self, seconds=3.0, max_gyro_dps=5.0, max_accel_std=0.2):
        self.seconds = seconds
        self.max_gyro = math.radians(max_gyro_dps)
        self.max_accel_std = max_accel_std
        self.t0 = None
        self.gyro = []
        self.accel_norm = []
        self.bias = None

    def feed(self, s: ImuSample) -> str | None:
        accel, gyro = body_vectors(s)
        if self.t0 is None:
            self.t0 = s.t_us
        if np.linalg.norm(gyro) > self.max_gyro:
            return "moved"
        self.gyro.append(gyro)
        self.accel_norm.append(np.linalg.norm(accel))
        if (s.t_us - self.t0) * 1e-6 < self.seconds:
            return None
        if np.std(self.accel_norm) > self.max_accel_std:
            return "moved"
        self.bias = np.mean(self.gyro, axis=0)
        return "done"
