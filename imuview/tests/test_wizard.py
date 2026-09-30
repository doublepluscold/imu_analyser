import numpy as np
import pytest

from imuview import calib_tools as ct
from imuview.frames import G, board_rotation
from imuview.gui.wizard import ORDER, CalibWizard
from imuview.messages import ImuSample

ALIGN = [180.0, 0.0, 180.0]
B = np.array([0.12, -0.2, 0.3])
M = np.diag([1.004, 0.992, 1.011])
GYRO_BIAS = np.array([1e-3, -2e-3, 5e-4])
RATE = 80.0


class Sensor:
    """Feeds the wizard like a module held in a pose, with noise; time runs on."""

    def __init__(self, wiz, seed=0):
        self.wiz, self.t, self.rng = wiz, 0.0, np.random.default_rng(seed)
        self.pose, self.a0 = None, None
        self.r_board = board_rotation(*ALIGN)

    def reading(self, pose, tilt_deg=2.0):
        up_body = np.array(ct.POSES[pose]) + self.rng.normal(0, np.radians(tilt_deg), 3)
        up_body /= np.linalg.norm(up_body)
        return np.linalg.inv(M) @ (self.r_board.T @ (G * up_body)) + B

    def hold(self, pose, seconds, moving=False):
        if pose != self.pose:  # the module is moved to another side only when the pose changes
            self.pose, self.a0 = pose, self.reading(pose)
        a0 = self.a0
        for _ in range(int(seconds * RATE)):
            a = a0 + self.rng.normal(0, 0.012, 3)
            w = GYRO_BIAS + self.rng.normal(0, 4e-4, 3)
            if moving:
                a = a + self.rng.normal(0, 2.0, 3)
                w = w + self.rng.normal(0, 0.5, 3)
            self.feed(a, w)

    def feed(self, a, w):
        self.t += 1 / RATE
        self.wiz.feed(
            ImuSample(
                pc_rx_time_ns=0,
                host_time_ns=int(self.t * 1e9),
                time_source="host",
                module_id=self.wiz.module_id,
                imu_id=0,
                accel=tuple(a),
                gyro=tuple(w),
            )
        )


def run_all(wiz, sensor):
    for pose in ORDER:
        wiz.press_enter()
        sensor.hold(pose, 1.0, moving=True)  # user is still placing it
        sensor.hold(pose, 5.0)
    wiz.press_enter()
    sensor.hold("level", 12.0)


def test_full_run_recovers_the_injected_error():
    wiz = CalibWizard(module_id=3, board_alignment_deg=ALIGN)
    sensor = Sensor(wiz)
    assert wiz.status().phase == "wait_enter" and wiz.status().n_steps == 7
    run_all(wiz, sensor)
    st = wiz.status()
    assert st.phase == "done", st
    acc = wiz.result["accel"]
    assert not acc["errors"], acc["errors"]
    assert np.allclose(acc["offset_b"], B, atol=0.05)
    assert np.allclose(acc["scale_diag"], np.diag(M), atol=0.004)
    assert np.allclose(wiz.result["gyro_bias"], GYRO_BIAS, atol=2e-4)
    assert wiz.result["level"]["tilt_before_deg"] < 4  # level pose was held within ~2-3 deg
    assert wiz.calibration() is not None


def test_nothing_is_collected_before_enter_and_while_moving():
    wiz = CalibWizard(board_alignment_deg=ALIGN)
    sensor = Sensor(wiz)
    sensor.hold("level", 6.0)  # still, but Enter not pressed
    assert wiz.status().progress == 0 and wiz.status().step == 0
    wiz.press_enter()
    sensor.hold("level", 10.0, moving=True)  # shaking: never completes
    assert wiz.status().step == 0 and wiz.status().phase == "collecting"
    sensor.hold("level", 1.5)
    assert 0 < wiz.status().progress < 1
    sensor.hold("level", 3.5)
    assert wiz.status().step == 1 and wiz.status().done_poses == ["level"]


def test_wrong_side_is_refused_and_can_be_redone():
    wiz = CalibWizard(board_alignment_deg=ALIGN)
    sensor = Sensor(wiz)
    wiz.press_enter()
    sensor.hold("level", 5.0)  # step 0 asks for level: fine
    wiz.press_enter()
    sensor.hold("back", 5.0)  # step 1 asks for left, user turns it upside down
    st = wiz.status()
    assert (
        st.step == 1 and st.phase == "wait_enter" and "не схоже на положення 'left'" in st.message
    )
    wiz.press_enter()
    sensor.hold("left", 5.0)
    assert wiz.status().step == 2


def test_wrong_side_passes_when_the_check_is_off():
    wiz = CalibWizard(board_alignment_deg=ALIGN, check_pose=False)
    sensor = Sensor(wiz)
    wiz.press_enter()
    sensor.hold("back", 5.0)
    assert wiz.status().step == 1


def test_other_modules_are_ignored_and_save_writes_files(tmp_path):
    wiz = CalibWizard(module_id=1, board_alignment_deg=ALIGN)
    other = Sensor(CalibWizard(module_id=2, board_alignment_deg=ALIGN))
    other.wiz = wiz  # same sensor, but its samples carry module 2
    wiz.press_enter()
    for _ in range(800):
        other.t += 1 / RATE
        wiz.feed(
            ImuSample(
                pc_rx_time_ns=0,
                host_time_ns=int(other.t * 1e9),
                module_id=2,
                imu_id=0,
                accel=(0, 0, 9.8),
                gyro=(0, 0, 0),
            )
        )
    assert wiz.status().progress == 0
    sensor = Sensor(wiz)
    run_all(wiz, sensor)
    cal, md = wiz.save(tmp_path / "imu_calib.json", tmp_path)
    assert (tmp_path / "imu_calib.json").exists() and md.exists()
    assert cal.sha256


def test_back_redoes_a_step():
    wiz = CalibWizard(board_alignment_deg=ALIGN)
    sensor = Sensor(wiz)
    wiz.press_enter()
    sensor.hold("level", 5.0)
    assert wiz.status().step == 1
    wiz.back()
    assert wiz.status().step == 0 and wiz.status().done_poses == []


@pytest.mark.parametrize("bad", ["level"])
def test_unit_sanity_refused(bad):
    wiz = CalibWizard(board_alignment_deg=ALIGN)
    wiz.press_enter()
    sensor = Sensor(wiz)
    for _ in range(600):
        sensor.feed(np.array([0, 0, 3.0]), np.zeros(3))  # |a| = 3: nothing like g
    assert "не схоже" in wiz.status().message and wiz.status().phase == "wait_enter"


def test_redo_a_finished_step_keeps_the_others_and_returns_to_the_end():
    wiz = CalibWizard(board_alignment_deg=ALIGN)
    sensor = Sensor(wiz)
    run_all(wiz, sensor)
    assert wiz.status().phase == "done"
    wiz.goto(2)
    st = wiz.status()
    assert (
        st.phase == "wait_enter"
        and st.step == 2
        and st.done_steps == [True, True, False] + [True] * 4
    )
    wiz.press_enter()
    sensor.hold("right", 5.0)
    assert wiz.status().phase == "done" and wiz.result["accel"]["n_poses"] == 6


def test_target_orientation_puts_the_requested_side_up():
    from imuview.frames import quat_from_euler, quat_to_euler
    from imuview.gui.wizard import MATCH_DEG, target_quat, up_error_deg

    q_now = quat_from_euler(0.3, -0.2, 1.1)
    for pose in ORDER:
        q_t = target_quat(pose, q_now)
        assert up_error_deg(q_t, pose) < 1e-6
        assert up_error_deg(q_now, pose) > 0 or pose == "level"
    # level pose: same heading as now, tilt removed
    assert quat_to_euler(target_quat("level", q_now))[2] == pytest.approx(1.1, abs=1e-6)
    assert up_error_deg(quat_from_euler(0, 0, 0), "level") < MATCH_DEG
    assert up_error_deg(quat_from_euler(0, 0, 0), "back") == pytest.approx(180.0)
    assert len(Sensor(CalibWizard(board_alignment_deg=ALIGN)).wiz.history()) == 0
