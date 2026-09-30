"""All coordinate frame math lives here, nowhere else.

Conventions
  body  = FRD (x forward, y right, z down)
  world = local NED (x north, y east, z down)
  quaternion q = (w, x, y, z), Hamilton product, rotates body vectors into world:
      v_ned = q * v_frd * conj(q)
  Euler angles: yaw-pitch-roll (ZYX), R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
  Accelerometer measures specific force: lying level and still it reads (0, 0, -g).
"""

import math

import numpy as np

G = 9.80665  # m/s^2

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


# ---------- quaternions ----------


def quat_mul(a, b) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def quat_conj(q) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    q = q / math.sqrt(q @ q)
    return q if q[0] >= 0 else -q  # keep w >= 0 so logs don't flip sign


def quat_to_matrix(q) -> np.ndarray:
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def quat_from_matrix(m) -> np.ndarray:
    m = np.asarray(m)
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = 2 * math.sqrt(1 + t)
        q = [s / 4, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2 * math.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [(m[2, 1] - m[1, 2]) / s, s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2 * math.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s]
    else:
        s = 2 * math.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4]
    return quat_normalize(q)


def quat_rotate(q, v) -> np.ndarray:
    """Rotate vector v by q (body -> world)."""
    return quat_to_matrix(q) @ np.asarray(v)


def quat_from_euler(roll, pitch, yaw) -> np.ndarray:
    """Radians, ZYX order."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def quat_to_euler(q) -> tuple[float, float, float]:
    """Returns (roll, pitch, yaw) in radians, ZYX order."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def quat_from_gyro(omega, dt) -> np.ndarray:
    """Rotation by body rate omega [rad/s] during dt, as a quaternion."""
    omega = np.asarray(omega, dtype=float)
    angle = np.linalg.norm(omega) * dt
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = omega / np.linalg.norm(omega)
    return np.concatenate([[math.cos(angle / 2)], axis * math.sin(angle / 2)])


def quat_from_accel(accel) -> np.ndarray:
    """Initial attitude from one accelerometer reading (still sensor). Yaw is set to 0."""
    fx, fy, fz = accel
    roll = math.atan2(-fy, -fz)
    pitch = math.atan2(fx, math.hypot(fy, fz))
    return quat_from_euler(roll, pitch, 0.0)


def angle_between(q1, q2) -> float:
    """Smallest rotation angle [rad] between two orientations."""
    d = abs(float(np.dot(q1, q2)))
    return 2 * math.acos(min(1.0, d))


NED_TO_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])  # v_enu = M v_ned


def matrix_from_euler_deg(roll, pitch, yaw) -> np.ndarray:
    """ZYX: R = Rz(yaw) Ry(pitch) Rx(roll), degrees."""
    return quat_to_matrix(quat_from_euler(*np.radians([roll, pitch, yaw])))


def euler_deg_from_matrix(m) -> tuple[float, float, float]:
    return tuple(math.degrees(a) for a in quat_to_euler(quat_from_matrix(m)))


# ---------- board alignment and tare ----------


def board_rotation(roll_deg, pitch_deg, yaw_deg) -> np.ndarray:
    """Sensor -> body rotation matrix, like Betaflight board alignment.

    v_body = R @ v_sensor. Example: sensor axes are FLU (z up) -> roll = 180.
    """
    q = quat_from_euler(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
    return quat_to_matrix(q)


def tare(q_ref, q) -> np.ndarray:
    """Orientation q relative to reference q_ref (identity right after tare)."""
    return quat_mul(quat_conj(q_ref), q)


# ---------- geodetic (for GPS later) ----------


def geodetic_to_ecef(lat_deg, lon_deg, h) -> np.ndarray:
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(lat) ** 2)
    return np.array(
        [
            (n + h) * math.cos(lat) * math.cos(lon),
            (n + h) * math.cos(lat) * math.sin(lon),
            (n * (1 - WGS84_E2) + h) * math.sin(lat),
        ]
    )


def _ecef_to_ned_matrix(lat_deg, lon_deg) -> np.ndarray:
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    return np.array(
        [
            [-sl * co, -sl * so, cl],
            [-so, co, 0.0],
            [-cl * co, -cl * so, -sl],
        ]
    )


def geodetic_to_ned(lat_deg, lon_deg, h, origin) -> np.ndarray:
    """Position relative to origin = (lat0_deg, lon0_deg, h0) as NED meters."""
    lat0, lon0, h0 = origin
    d = geodetic_to_ecef(lat_deg, lon_deg, h) - geodetic_to_ecef(lat0, lon0, h0)
    return _ecef_to_ned_matrix(lat0, lon0) @ d


def ned_to_geodetic(ned, origin) -> tuple[float, float, float]:
    """Inverse of geodetic_to_ned (iterative, exact to well below a millimeter)."""
    lat0, lon0, h0 = origin
    x, y, z = geodetic_to_ecef(lat0, lon0, h0) + _ecef_to_ned_matrix(lat0, lon0).T @ np.asarray(ned)
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - WGS84_E2))
    for _ in range(5):
        n = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1 - WGS84_E2 * n / (n + h)))
    return math.degrees(lat), math.degrees(lon), h


# ---------- viewer edge ----------


def to_rerun_quat(q) -> list[float]:
    """Rerun wants (x, y, z, w). World stays NED; the viewer is told 'z is down'."""
    return [q[1], q[2], q[3], q[0]]
