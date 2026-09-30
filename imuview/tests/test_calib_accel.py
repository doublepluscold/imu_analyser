import math

import numpy as np
import pytest

from imuview import calib_tools as ct
from imuview.calibration import Calibration
from imuview.frames import G, matrix_from_euler_deg

AXES = {
    "level": (0, 0, 1),
    "back": (0, 0, -1),
    "left": (0, 1, 0),
    "right": (0, -1, 0),
    "nosedown": (1, 0, 0),
    "noseup": (-1, 0, 0),
}  # fmt: skip (sensor-frame readings, +g along the up side)


def raw_reading(true, b, m):
    """What the sensor reports: true = M (raw - b)  ->  raw = M^-1 true + b."""
    return np.linalg.inv(m) @ true + b


def samples_for(dirs, b, m, tilt_deg=3.0, seed=0, names=None):
    rng = np.random.default_rng(seed)
    out = []
    for k, d in enumerate(dirs):
        d = np.asarray(d, float)
        d = d + rng.normal(0, math.radians(tilt_deg), 3)  # imperfectly held
        d /= np.linalg.norm(d)
        pose = names[k] if names else None
        out.append(ct.PoseSample(pose, raw_reading(G * d, b, m), np.full(3, 0.01), 500, pose or ""))
    return out


B = np.array([0.15, -0.22, 0.31])
M_DIAG = np.diag([1.004, 0.992, 1.011])


def test_six_parameter_fit_recovers_offset_and_scale_with_tilted_poses():
    # the sensor is built with M = M_DIAG
    s = samples_for(AXES.values(), B, M_DIAG, names=list(AXES))
    fit = ct.fit_accel(s, G, "six")
    assert not fit["errors"]
    assert np.allclose(fit["offset_b"], B, atol=0.03)
    assert np.allclose(fit["scale_diag"], np.diag(M_DIAG), atol=0.003)
    assert fit["max_rel_err_after"] < 1e-6  # exactly determined
    assert any("exactly determined" in w for w in fit["warnings"])
    assert fit["rms_after_ms2"] < fit["rms_before_ms2"]


def test_axis_method_agrees_on_axis_aligned_poses():
    s = samples_for(AXES.values(), B, M_DIAG, tilt_deg=0.0, names=list(AXES))
    six = ct.fit_accel(s, G, "six")
    axis = ct.fit_accel(s, G, "axis")
    assert np.allclose(axis["offset_b"], B, atol=1e-9)
    assert np.allclose(axis["offset_b"], six["offset_b"], atol=1e-6)
    assert np.allclose(axis["scale_diag"], six["scale_diag"], atol=1e-6)
    assert set(ct.compare_models(s)) == {"six", "axis"}  # no nine with 6 poses


def test_nine_parameter_fit_needs_many_poses_and_finds_cross_axis():
    m = np.array([[1.004, 0.006, -0.004], [0.006, 0.992, 0.008], [-0.004, 0.008, 1.011]])
    rng = np.random.default_rng(3)
    dirs = rng.normal(size=(16, 3))
    s = samples_for(dirs, B, m, tilt_deg=0.0)
    fit = ct.fit_accel(s, G, "nine")
    assert not fit["errors"], fit["errors"]
    assert np.allclose(fit["offset_b"], B, atol=1e-4)
    assert np.allclose(fit["matrix_M"], m, atol=1e-4)
    assert fit["redundancy"] == 7
    # six parameters cannot absorb the cross terms as well
    assert ct.fit_accel(s, G, "six")["rms_after_ms2"] > fit["rms_after_ms2"]


def test_nine_refused_with_six_poses_and_with_unobservable_poses():
    s6 = samples_for(AXES.values(), B, M_DIAG, names=list(AXES))
    assert "clearly different" in ct.fit_accel(s6, G, "nine")["errors"][0]
    # 14 poses, all in one plane: enough poses but the cross terms are not observable
    ang = np.linspace(0, 2 * math.pi, 14, endpoint=False)
    flat = [(math.cos(a), math.sin(a), 0.0) for a in ang]
    s = samples_for(flat, B, M_DIAG, tilt_deg=0.0)
    err = ct.fit_accel(s, G, "nine")["errors"]
    assert err and "not observable" in err[0]


def test_sanity_checks_reject_absurd_offset_and_flag_repeated_pose():
    s = samples_for(AXES.values(), np.array([4.0, 0, 0]), M_DIAG, names=list(AXES))
    fit = ct.fit_accel(s, G, "six")
    assert any("exceeds" in e for e in fit["errors"])
    s = samples_for(AXES.values(), B, M_DIAG, names=list(AXES))
    s[1] = s[0]  # the same pose twice
    fit = ct.fit_accel(s, G, "six")
    assert any("same way" in w for w in fit["warnings"])


def test_fewer_than_six_poses_is_an_error():
    s = samples_for(list(AXES.values())[:5], B, M_DIAG)
    assert ct.fit_accel(s, G)["errors"]


@pytest.mark.parametrize("roll,pitch", [(0, 0), (2.0, -1.5), (-4.0, 3.0), (7.0, 6.0)])
def test_level_trim_recovers_a_known_tilt(roll, pitch):
    align = [180.0, 0.0, 180.0]
    r_board = matrix_from_euler_deg(*align)
    # the body is tilted by (roll, pitch) relative to level: a_body = R(roll,pitch)^T (0,0,-g)
    tilt = matrix_from_euler_deg(roll, pitch, 0.0)
    a_body = tilt.T @ np.array([0, 0, -G])
    a_sensor = r_board.T @ a_body
    raw = raw_reading(a_sensor, B, M_DIAG)
    out = ct.level_trim(raw, B, M_DIAG, align)
    assert np.allclose(out["residual_after"], [0, 0, -1], atol=1e-5)
    # applying the trim on top of the alignment makes the body frame level again
    trim = matrix_from_euler_deg(*out["level_trim_deg"], 0.0)
    assert np.allclose(trim @ a_body, [0, 0, -G], atol=1e-3)
    assert out["tilt_before_deg"] == pytest.approx(
        math.degrees(math.acos(np.cos(math.radians(roll)) * np.cos(math.radians(pitch)))), abs=0.05
    )


def test_build_calibration_roundtrips_through_json(tmp_path):
    s = samples_for(AXES.values(), B, M_DIAG, names=list(AXES))
    fit = ct.fit_accel(s, G, "six")
    level = {"level_trim_deg": [0.5, -0.25]}
    cal = ct.build_calibration(fit, [1e-4, 2e-5, -3e-5], level, {"sources": ["test"]})
    cal.save(tmp_path / "imu_calib.json")
    back = Calibration.load(tmp_path / "imu_calib.json")
    assert np.allclose(back.accel_offset, B, atol=0.03)
    assert np.allclose(back.gyro_bias, [1e-4, 2e-5, -3e-5])
    assert back.level_trim_deg == (0.5, -0.25)
    assert back.info["units"] == "m/s^2" and back.info["accel_fit"]["model"] == "six"
    # a calibrated reading of pose "level" has length g
    a = back.apply_accel(s[0].mean)
    assert np.linalg.norm(a) == pytest.approx(G, rel=1e-6)


def test_report_writes_markdown_and_png(tmp_path):
    s = samples_for(AXES.values(), B, M_DIAG, names=list(AXES))
    fit = ct.fit_accel(s, G, "six")
    md = ct.report(accel=fit, out_dir=tmp_path, name="r")
    assert md.exists() and (tmp_path / "r.png").stat().st_size > 1000
    assert "offset b" in md.read_text()
