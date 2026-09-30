"""Step-by-step calibration guide (Mission Planner style). No Qt, no I/O except save().

Steps: level, then the five other sides (left, right, nose down, nose up, back), then a still
gyro step. For every step the program shows a prompt, the user places the module and presses
Enter; the wizard then waits until the module is still (rolling-window test) and collects
`hold_s` seconds of still samples. Movement restarts the collection. A wrong side (the reading
does not match the requested pose, using the board alignment of the config) is refused.

Only RAW sensor values are used (s.accel, s.gyro), repeated (stale) values are skipped.
The fit itself lives in calib_tools.py, the same code as `imu calib accel`.
"""

import math
import threading
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from .. import calib_tools as ct
from ..frames import G, board_rotation
from ..messages import ImuSample

POSE_TEXT = {
    "level": "Постав модуль РІВНО (верхом догори).",
    "left": "Постав на ЛІВИЙ бік (лівий бік донизу).",
    "right": "Постав на ПРАВИЙ бік (правий бік донизу).",
    "nosedown": "Постав НОСОМ ДОНИЗУ (перед дивиться в підлогу).",
    "noseup": "Постав НОСОМ ДОГОРИ (перед дивиться в стелю).",
    "back": "Переверни ДОГОРИ ДНОМ (верхом донизу).",
}
ORDER = ["level", "left", "right", "nosedown", "noseup", "back"]  # Mission Planner order

WINDOW_S = 0.6  # stillness is judged over this much time
MAX_ACCEL_STD = 0.08  # m/s^2, any axis
MAX_GYRO_DEV = 0.03  # rad/s, largest departure from the window mean
MAX_POSE_ANGLE_DEG = 35.0  # measured "up" vs the requested pose


@dataclass
class Step:
    kind: str  # "pose" | "gyro"
    pose: str | None
    prompt: str
    hold_s: float


@dataclass
class Status:
    step: int
    n_steps: int
    kind: str
    prompt: str
    phase: str  # wait_enter | collecting | done | error
    progress: float  # 0..1 of the current hold
    still: bool
    accel_norm: float | None
    accel_std: float | None
    message: str = ""
    done_poses: list[str] = field(default_factory=list)
    done_steps: list[bool] = field(default_factory=list)  # one flag per step, for the stepper


class CalibWizard:
    def __init__(self, module_id=0, board_alignment_deg=(0, 0, 0), hold_s=3.0, gyro_s=10.0,
                 check_pose=True):  # fmt: skip
        self.module_id = module_id
        self.board = board_rotation(*board_alignment_deg)
        self.board_alignment_deg = list(board_alignment_deg)
        self.check_pose = check_pose
        self.steps = [Step("pose", p, POSE_TEXT[p], hold_s) for p in ORDER]
        self.steps.append(
            Step("gyro", None, f"Не чіпай модуль: зсув гіроскопа, {gyro_s:g} с.", gyro_s)
        )
        self.lock = threading.RLock()
        self.index = 0
        self.phase = "wait_enter"
        self.message = ""
        self.window: deque = deque()  # (t, accel, gyro) of the rolling stillness window
        self.hold: list = []  # (t, accel, gyro) collected while still
        self.poses: dict[str, ct.PoseSample] = {}
        self.gyro_bias = None
        self.gyro_std = None
        self.result: dict | None = None
        self._last = (None, None, False)  # accel norm, accel std, still: for the display
        self.norm_hist: deque = deque(maxlen=1200)  # (t, |a|) for the small graph

    # ----- user actions (GUI thread) -----

    def press_enter(self):
        with self.lock:
            if self.phase == "wait_enter":
                self.phase, self.message = "collecting", "чекаю, поки модуль заспокоїться"
                self.window.clear()
                self.hold.clear()

    def _has_data(self, i: int) -> bool:
        step = self.steps[i]
        return self.gyro_bias is not None if step.kind == "gyro" else step.pose in self.poses

    def goto(self, index: int):
        """Do step `index` again (its old data is dropped; the other steps keep theirs)."""
        with self.lock:
            if not 0 <= index < len(self.steps):
                return
            step = self.steps[index]
            if step.kind == "gyro":
                self.gyro_bias = self.gyro_std = None
            else:
                self.poses.pop(step.pose, None)
            self.index = index
            self.result = None
            self.phase, self.message = "wait_enter", ""
            self.hold.clear()

    def back(self):
        """Redo the previous step."""
        with self.lock:
            if self.index > 0 and self.phase != "done":
                self.goto(self.index - 1)
            elif self.phase == "done":
                self.goto(len(self.steps) - 1)

    def cancel(self):
        with self.lock:
            self.phase, self.message = "error", "скасовано"

    # ----- data (pipeline thread) -----

    def feed(self, s: ImuSample):
        if s.module_id != self.module_id or s.imu_id != 0:
            return
        with self.lock:
            if s.accel_repeat or s.gyro_repeat:  # stale copy of an older value
                return
            t = s.t_us * 1e-6
            a, w = np.asarray(s.accel, float), np.asarray(s.gyro, float)
            self.window.append((t, a, w))
            self.norm_hist.append((t, float(np.linalg.norm(a))))
            while self.window and t - self.window[0][0] > WINDOW_S:
                self.window.popleft()
            still = self._is_still()
            self._last = (float(np.linalg.norm(a)), self._window_std(), still)
            if self.phase != "collecting":
                return
            if not still:
                if self.hold:
                    self.message = "рух, починаю спочатку"
                self.hold.clear()
                return
            self.hold.append((t, a, w))
            step = self.steps[self.index]
            if self.hold[-1][0] - self.hold[0][0] >= step.hold_s:
                self._finish_step(step)

    def _window_std(self):
        if len(self.window) < 5:
            return None
        return float(np.array([x[1] for x in self.window]).std(axis=0).max())

    def _is_still(self) -> bool:
        if len(self.window) < 5 or self.window[-1][0] - self.window[0][0] < 0.8 * WINDOW_S:
            return False
        acc = np.array([x[1] for x in self.window])
        gyr = np.array([x[2] for x in self.window])
        return bool(
            acc.std(axis=0).max() < MAX_ACCEL_STD
            and np.abs(gyr - gyr.mean(axis=0)).max() < MAX_GYRO_DEV
        )

    def _finish_step(self, step: Step):
        acc = np.array([x[1] for x in self.hold])
        gyr = np.array([x[2] for x in self.hold])
        if step.kind == "gyro":
            self.gyro_bias, self.gyro_std = gyr.mean(axis=0), gyr.std(axis=0)
            self._advance()
            return
        mean = acc.mean(axis=0)
        norm = float(np.linalg.norm(mean))
        if not (0.8 * G < norm < 1.2 * G or 0.8 < norm < 1.2):
            return self._refuse(f"|a| = {norm:.3f}: це не схоже ні на 9.8 м/с², ні на 1 g")
        if self.check_pose:
            up_body = np.array(ct.POSES[step.pose])
            expect = self.board.T @ up_body  # where "up" should point in the sensor frame
            ang = math.degrees(math.acos(np.clip(mean @ expect / norm, -1, 1)))
            if ang > MAX_POSE_ANGLE_DEG:
                seen = ct._dominant(mean)
                return self._refuse(
                    f"це не схоже на положення '{step.pose}' (відхилення {ang:.0f}°, показ "
                    f"дивиться вздовж {seen}); постав ще раз і натисни Enter"
                )
        self.poses[step.pose] = ct.PoseSample(
            step.pose, mean, acc.std(axis=0), len(acc), step.pose, True
        )
        self._advance()

    def _refuse(self, text):
        self.phase, self.message = "wait_enter", text
        self.hold.clear()

    def _advance(self):
        """Next step without data (the one after this, else the first gap); done when none."""
        self.hold.clear()
        self.message = ""
        n = len(self.steps)
        order = [*range(self.index + 1, n), *range(0, self.index)]
        nxt = next((i for i in order if not self._has_data(i)), None)
        if nxt is None:
            self.index = n
            self.phase = "done"
            self._compute()
        else:
            self.index = nxt
            self.phase = "wait_enter"

    # ----- result -----

    def _compute(self):
        samples = [self.poses[p] for p in ORDER if p in self.poses]
        unit = "g" if samples[0].mean @ samples[0].mean < 4 else "m/s^2"
        g = 1.0 if unit == "g" else G
        fit = ct.fit_accel(samples, g, "six")
        self.result = {"accel": fit, "gyro_bias": self.gyro_bias.tolist(),
                       "gyro_std": self.gyro_std.tolist(), "level": None,
                       "comparison": {m: f.get("rms_after_ms2") for m, f in
                                      ct.compare_models(samples, g).items()}}  # fmt: skip
        if not fit["errors"]:
            self.result["level"] = ct.level_trim(
                self.poses["level"].mean, fit["offset_b"], fit["matrix_M"], self.board_alignment_deg
            )
        else:
            self.message = "; ".join(fit["errors"])

    def calibration(self):
        """The finished Calibration (not saved yet), or None if the fit failed."""
        if not self.result or self.result["accel"]["errors"]:
            return None
        r = self.result
        return ct.build_calibration(
            r["accel"], r["gyro_bias"], r["level"],
            {"sources": {"wizard": f"module {self.module_id}"}},
        )  # fmt: skip

    def save(self, path="calib/imu_calib.json", report_dir=None):
        """Write the calibration JSON and a markdown + PNG report. Returns (cal, report path)."""
        cal = self.calibration()
        if cal is None:
            raise ValueError("no valid calibration to save: " + self.message)
        cal.save(path)
        md = ct.report(
            self.result["accel"], self.result["level"],
            {"bias": self.result["gyro_bias"], "noise_std": self.result["gyro_std"],
             "still": {"still": True}},
            out_dir=report_dir,
        )  # fmt: skip
        return cal, md

    def history(self) -> list:
        """[(t, |a|)] of the newest samples."""
        with self.lock:
            return list(self.norm_hist)

    def status(self) -> Status:
        with self.lock:
            idx = min(self.index, len(self.steps) - 1)
            step = self.steps[idx]
            prog = 0.0
            if self.phase == "collecting" and self.hold:
                prog = min(1.0, (self.hold[-1][0] - self.hold[0][0]) / step.hold_s)
            if self.phase == "done":
                prog = 1.0
            norm, std, still = self._last
            prompt = "Калібрацію завершено." if self.phase == "done" else step.prompt
            done = [p for p in ORDER if p in self.poses]
            flags = [self._has_data(i) for i in range(len(self.steps))]
            return Status(self.index, len(self.steps), step.kind, prompt, self.phase, prog,
                          still, norm, std, self.message, done, flags)  # fmt: skip


# ---------- target orientation of a step (for the 3D preview) ----------

MATCH_DEG = 12.0  # the module counts as placed when its "up" is this close to the target


def up_error_deg(q, pose: str) -> float:
    """Angle between where the pose's "up" side points now and world up (NED -z), degrees.
    q is body -> world (NED)."""
    from ..frames import quat_to_matrix

    up_body = np.array(ct.POSES[pose], float)
    up_world = quat_to_matrix(q) @ up_body
    return math.degrees(math.acos(np.clip(-up_world[2], -1.0, 1.0)))


def target_quat(pose: str, q_now) -> np.ndarray:
    """Orientation the module should have for this pose: its "up" side up, heading as now."""
    from ..frames import quat_from_euler, quat_mul, quat_to_euler, quat_to_matrix

    a = np.array(ct.POSES[pose], float)
    b = np.array([0.0, 0.0, -1.0])
    axis = np.cross(a, b)
    s = float(np.linalg.norm(axis))
    if s < 1e-9:
        axis = np.array([1.0, 0.0, 0.0])  # opposite or equal: any axis
        angle = math.pi if a @ b < 0 else 0.0
    else:
        axis, angle = axis / s, math.atan2(s, a @ b)
    q_min = np.array([math.cos(angle / 2), *(math.sin(angle / 2) * axis)])
    # keep the nose heading of the current orientation (rotate about the vertical only)
    yaw_now = quat_to_euler(q_now)[2]
    fwd = quat_to_matrix(q_min) @ np.array([1.0, 0.0, 0.0])
    yaw_min = math.atan2(fwd[1], fwd[0]) if abs(fwd[2]) < 0.99 else 0.0
    return quat_mul(quat_from_euler(0.0, 0.0, yaw_now - yaw_min), q_min)
