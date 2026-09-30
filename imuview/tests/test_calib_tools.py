import tomllib

import numpy as np
import pytest

from imuview import calib_tools as ct
from imuview.frames import G, board_rotation
from imuview.pipeline import Pipeline, load_config
from imuview.sources import SimSource

MOUNT = [180, 0, 90]  # sensor mounting used by the simulated device (sensor -> body)
POSE_ATTITUDE = {  # body roll, pitch, yaw [deg] that puts each side up
    "level": (0, 0, 0),
    "back": (180, 0, 0),
    "left": (-90, 0, 0),
    "right": (90, 0, 0),
    "nosedown": (0, -90, 0),
    "noseup": (0, 90, 0),
}


def record(tmp_path, label, seconds=3.0, **kw):
    """A session recorded from the simulated MTData2 device, like `imu calib record`."""
    kw.setdefault("sensor_alignment_deg", MOUNT)
    source = SimSource(protocol="mtdata2", rate_hz=100, realtime=False, seconds=seconds,
                       gpssol_every=20, **kw)  # fmt: skip
    session = tmp_path / label
    Pipeline(source, load_config(), session_dir=session, label=label).run()
    return ct.load(session), source


def test_check_detects_units_and_the_euler_convention(tmp_path):
    d, _ = record(tmp_path, "tilted", motion="still", attitude_deg=(20, 10, 30), corrupt_prob=0.3)
    r = ct.check(d)
    assert r["units_guess"]["accel"] == "m/s^2" and r["units_guess"]["g"] == G
    assert r["accel_norm"]["mean"] == pytest.approx(G, abs=0.01)
    assert r["time_source"] == "host" and r["still"]["still"]
    assert r["rate_hz"] == pytest.approx(70, rel=0.1)  # 100 Hz minus 30% bad frames
    e = r["euler_vs_accel_deg"]
    assert min(e, key=e.get) == "ENU, ZYX (Xsens spec)" and e["ENU, ZYX (Xsens spec)"] < 1.0
    assert all(v > 5 for k, v in e.items() if k != "ENU, ZYX (Xsens spec)")
    assert not r["imu_board_absent"] and "|" in ct.check_markdown([r])


def test_units_detection_refuses_other_magnitudes():
    assert ct.detect_units(9.81) == ("m/s^2", G)
    assert ct.detect_units(1.002) == ("g", 1.0)
    assert ct.detect_units(4.0) == (None, None)


def test_gyro_bias_noise_stability_and_scale(tmp_path):
    still, src = record(tmp_path, "still", seconds=35.0, motion="still")
    g = ct.gyro_still(still)
    # bias is in the sensor frame; sigma of the mean = 1.7e-3 / sqrt(3500) = 3e-5 rad/s
    assert np.allclose(g["bias"], src.gyro_bias[0], atol=1.5e-4)
    assert np.allclose(g["noise_std"], np.radians(0.1), rtol=0.1)
    assert g["long_enough"] and max(abs(v) for v in g["drift_in_sigma_of_mean"]) < 4
    assert "rad/s" in g["units_hint"]

    rot, _ = record(tmp_path, "rot_z", seconds=9.0, motion="rot_z", gyro_scale=1.02)
    r = ct.gyro_rotation(rot, g["bias"])
    assert r["angle_euler_deg_zyx"] == pytest.approx(90, abs=0.5)
    assert r["scale_gyro_over_euler"] == pytest.approx(1.02, abs=0.006)
    body_z_in_sensor = board_rotation(*MOUNT).T @ [0, 0, 1]
    assert abs(np.dot(r["axis_sensor"], body_z_in_sensor)) > 0.999
    assert "rotation" in ct.gyro_markdown(g, [r])


def test_axes_recovers_the_mounting_from_six_poses(tmp_path):
    datas = [
        record(tmp_path, f"pose_{p}", motion="still", attitude_deg=a, corrupt_prob=0.2)[0]
        for p, a in POSE_ATTITUDE.items()
    ]
    r = ct.axes(datas)
    assert "error" not in r
    assert np.array_equal(r["signed_permutation"], np.round(board_rotation(*MOUNT)))
    assert np.allclose(board_rotation(*r["board_alignment_deg"]), board_rotation(*MOUNT))
    assert all(p["residual_deg_perm"] < 1.0 for p in r["poses"])
    assert r["pairwise_min_angle_deg"] == pytest.approx(90, abs=1)
    assert r["covered"] == sorted(POSE_ATTITUDE)
    assert "sensor +z -> body" in ct.axes_markdown(r)


def test_axes_with_explicit_pose_names_and_too_few_poses(tmp_path):
    a, _ = record(tmp_path, "one", motion="still")
    b, _ = record(tmp_path, "two", motion="still", attitude_deg=(0, 0, 45))  # same up side
    r = ct.axes([a, b], ["level", "level"])
    assert "error" in r


def test_write_board_alignment_keeps_the_rest_of_the_config(tmp_path):
    cfg = tmp_path / "config.toml"
    ct.write_board_alignment(cfg, [180, 0, 90])
    assert tomllib.loads(cfg.read_text())["imu"]["default"]["board_alignment_deg"] == [180, 0, 90]
    cfg.write_text('parser = "mtdata2"\n\n[imu.default]\nboard_alignment_deg = [0, 0, 0]\n'
                   'gyro_lsb_dps = 0.06\n\n[estimator]\nalgo = "mahony"\n')  # fmt: skip
    ct.write_board_alignment(cfg, [0, 180, -90])
    t = tomllib.loads(cfg.read_text())
    assert t["imu"]["default"] == {"board_alignment_deg": [0, 180, -90], "gyro_lsb_dps": 0.06}
    assert t["estimator"]["algo"] == "mahony" and t["parser"] == "mtdata2"
