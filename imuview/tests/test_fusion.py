import math

import numpy as np
import pytest

from imuview.frames import (
    G,
    angle_between,
    board_rotation,
    geodetic_to_ned,
    ned_to_geodetic,
    quat_from_euler,
    quat_from_matrix,
    quat_mul,
    quat_to_euler,
    quat_to_matrix,
    tare,
)
from imuview.fusion import AttitudeEstimator, GyroCalibrator
from imuview.messages import ImuSample


def sample(t_us, accel=(0.0, 0.0, -G), gyro=(0.0, 0.0, 0.0), quat=None):
    return ImuSample(mcu_time_us=t_us, pc_rx_time_ns=0, imu_id=0, accel=accel, gyro=gyro, quat=quat)


def specific_force(q):
    """What a still accelerometer reads with attitude q."""
    return tuple(quat_to_matrix(q).T @ [0.0, 0.0, -G])


def rpy_deg(q):
    return [math.degrees(a) for a in quat_to_euler(q)]


# ---------- frames ----------


def test_euler_round_trip():
    q = quat_from_euler(0.3, -0.2, 2.5)
    assert quat_to_euler(q) == pytest.approx((0.3, -0.2, 2.5))
    assert quat_from_matrix(quat_to_matrix(q)) == pytest.approx(q)


def test_level_accel_reads_minus_g():
    assert specific_force([1, 0, 0, 0]) == pytest.approx((0, 0, -G))
    nose_up = quat_from_euler(0, math.radians(30), 0)
    assert specific_force(nose_up)[0] == pytest.approx(G * 0.5)  # gravity pulls "backwards"


def test_board_alignment_examples():
    # sensor mounted upside down (z up): roll 180 turns its +g reading into FRD -g
    assert board_rotation(180, 0, 0) @ [0, 0, G] == pytest.approx([0, 0, -G])
    # sensor turned 90 deg clockwise (seen from above): its x axis points to body right
    assert board_rotation(0, 0, 90) @ [1, 0, 0] == pytest.approx([0, 1, 0], abs=1e-12)


def test_tare_gives_identity_then_relative_rotation():
    ref = quat_from_euler(0.2, 0.1, 1.0)
    assert tare(ref, ref) == pytest.approx([1, 0, 0, 0])
    later = quat_mul(ref, quat_from_euler(0, 0, 0.5))  # then yawed 0.5 rad about body z
    assert quat_to_euler(tare(ref, later)) == pytest.approx((0, 0, 0.5))
    assert angle_between(tare(ref, later), [1, 0, 0, 0]) == pytest.approx(0.5)


def test_geodetic_ned_round_trip():
    origin = (47.0, 8.0, 500.0)
    ned = geodetic_to_ned(47.0 + 1e-4, 8.0, 510.0, origin)
    assert ned == pytest.approx([11.12, 0.0, -10.0], abs=0.01)  # 1e-4 deg lat ~ 11.1 m north
    back = ned_to_geodetic([100.0, -200.0, 5.0], origin)
    assert geodetic_to_ned(*back, origin) == pytest.approx([100.0, -200.0, 5.0], abs=1e-6)


# ---------- filters ----------


@pytest.mark.parametrize("algo", ["mahony", "madgwick"])
def test_static_accel_only_converges(algo):
    truth = quat_from_euler(math.radians(20), math.radians(-10), 0)
    est = AttitudeEstimator(algo, kp=2.0, ki=0.0, beta=0.1)
    est.process(sample(0))  # starts level
    for k in range(1, 5000):  # 5 s at 1 kHz
        state = est.process(sample(k * 1000, accel=specific_force(truth)))
    roll, pitch, _ = rpy_deg(state.q)
    assert roll == pytest.approx(20, abs=0.5)
    assert pitch == pytest.approx(-10, abs=0.5)


@pytest.mark.parametrize("algo", ["mahony", "madgwick"])
def test_constant_yaw_rate_integrates(algo):
    est = AttitudeEstimator(algo)
    for k in range(3001):  # 30 deg/s for 3 s
        state = est.process(sample(k * 1000, gyro=(0, 0, math.radians(30))))
    assert rpy_deg(state.q) == pytest.approx([0, 0, 90], abs=0.1)


@pytest.mark.parametrize("algo", ["mahony", "madgwick"])
def test_constant_roll_rate_with_matching_accel(algo):
    est = AttitudeEstimator(algo)
    for k in range(1001):  # 45 deg/s for 1 s
        truth = quat_from_euler(math.radians(45 * k / 1000), 0, 0)
        state = est.process(sample(k * 1000, specific_force(truth), (math.radians(45), 0, 0)))
    assert rpy_deg(state.q) == pytest.approx([45, 0, 0], abs=0.2)


def test_dt_comes_from_mcu_time_and_gaps_are_skipped():
    est = AttitudeEstimator("mahony", kp=0, ki=0)
    est.process(sample(0))
    est.process(sample(1_000_000, gyro=(0, 0, 1.0)))  # 1 s gap > MAX_DT: not integrated
    state = est.process(sample(1_010_000, gyro=(0, 0, 1.0)))  # 10 ms at 1 rad/s
    assert quat_to_euler(state.q)[2] == pytest.approx(0.01, abs=1e-6)


def test_passthrough_uses_device_quaternion():
    q = tuple(quat_from_euler(0.1, 0.2, 0.3))
    state = AttitudeEstimator("passthrough").process(sample(0, quat=q))
    assert state.q == pytest.approx(q)
    assert AttitudeEstimator("passthrough").process(sample(0)) is None


def test_other_imus_and_messages_are_ignored():
    est = AttitudeEstimator("mahony", imu_id=1)
    assert est.process(sample(0)) is None


# ---------- gyro calibration ----------


def test_calibration_measures_bias():
    bias = np.array([0.01, -0.02, 0.005])
    rng = np.random.default_rng(0)
    cal = GyroCalibrator(seconds=3.0)
    for k in range(10_000):
        result = cal.feed(sample(k * 1000, gyro=tuple(bias + rng.normal(0, 0.002, 3))))
        if result:
            break
    assert result == "done" and k == 3000
    assert cal.bias == pytest.approx(bias, abs=2e-4)


def test_calibration_aborts_on_rotation_and_on_shaking():
    cal = GyroCalibrator()
    assert cal.feed(sample(0, gyro=(0, 0, math.radians(20)))) == "moved"
    cal = GyroCalibrator(seconds=1.0)
    results = [cal.feed(sample(k * 1000, accel=(0, 0, -G + (k % 2)))) for k in range(1001)]
    assert results[-1] == "moved"
