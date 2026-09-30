"""Host-side IMU calibration: a correction applied to the stream, never to the device.

Works in the stream's native sensor frame, before board alignment, so changing the board
alignment later does not invalidate it:
    a_cal = M (a_raw - b)
    w_cal = w_raw - gyro_bias
Level trim is a small body-frame rotation (roll, pitch) applied after board alignment.

calib/imu_calib.json holds these plus how they were obtained (see Calibration.to_json).
"""

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

TOOL_VERSION = "1"


@dataclass
class Calibration:
    accel_offset: np.ndarray = field(default_factory=lambda: np.zeros(3))  # b, sensor units
    accel_matrix: np.ndarray = field(default_factory=lambda: np.eye(3))  # M
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))  # sensor units
    level_trim_deg: tuple[float, float] = (0.0, 0.0)  # roll, pitch; body frame
    info: dict = field(default_factory=dict)  # units, g, fit model, quality, sources, date...
    path: str | None = None
    sha256: str | None = None

    def apply_accel(self, a) -> np.ndarray:
        return self.accel_matrix @ (np.asarray(a, dtype=float) - self.accel_offset)

    def apply_gyro(self, w) -> np.ndarray:
        return np.asarray(w, dtype=float) - self.gyro_bias

    def apply_arrays(self, accel: np.ndarray, gyro: np.ndarray):
        """Vectorized: (N, 3) raw arrays -> (N, 3) calibrated arrays."""
        return (accel - self.accel_offset) @ self.accel_matrix.T, gyro - self.gyro_bias

    def to_json(self) -> dict:
        return {
            "tool_version": TOOL_VERSION,
            "frame": "native sensor frame of the stream, before board alignment",
            "model": "a_cal = M (a_raw - b); w_cal = w_raw - gyro_bias",
            "accel_offset_b": self.accel_offset.tolist(),
            "accel_matrix_M": self.accel_matrix.tolist(),
            "gyro_bias": self.gyro_bias.tolist(),
            "level_trim_deg": list(self.level_trim_deg),
            **self.info,
        }

    def save(self, path) -> str:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        text = json.dumps(self.to_json(), indent=2) + "\n"
        path.write_text(text)
        self.path, self.sha256 = str(path), hashlib.sha256(text.encode()).hexdigest()
        return self.sha256

    @classmethod
    def load(cls, path) -> "Calibration":
        raw = Path(path).read_bytes()
        d = json.loads(raw)
        known = {"tool_version", "frame", "model", "accel_offset_b", "accel_matrix_M",
                 "gyro_bias", "level_trim_deg"}  # fmt: skip
        return cls(
            accel_offset=np.array(d.get("accel_offset_b", [0, 0, 0]), dtype=float),
            accel_matrix=np.array(d.get("accel_matrix_M", np.eye(3).tolist()), dtype=float),
            gyro_bias=np.array(d.get("gyro_bias", [0, 0, 0]), dtype=float),
            level_trim_deg=tuple(d.get("level_trim_deg", [0.0, 0.0])),
            info={k: v for k, v in d.items() if k not in known},
            path=str(path),
            sha256=hashlib.sha256(raw).hexdigest(),
        )

    def describe(self) -> dict:
        """For meta.json."""
        return {"path": self.path, "sha256": self.sha256}
