"""3D models: STL loading, the model registry, automatic scale and centre.

A mesh is a "triangle soup": float32 array (N, 3, 3) = N triangles x 3 vertices x xyz.
A model is prepared in this order:
  1. centre: subtract the centre (default: area-weighted centroid of the surface);
  2. rotate by rotation_deg so that the model's front points along +x, its right side along +y
     and its underside along +z (body frame FRD, the frame the orientation estimate is in);
  3. scale so that the longest side is `size` (scene units = metres), times `scale`.
The registry file models/models.toml overrides any of it per model:

    [[model]]
    key = "shahed"               # default: file name without .stl
    display_name = "Shahed"      # default: the key
    file = "shahed.stl"
    rotation_deg = [0, 90, 0]    # roll, pitch, yaw applied to the STL (default [0, 0, 0])
    scale = 1.0                  # multiplies the automatic size (default 1)
    center = [0, 0, 0]           # point of the STL to turn around (default "auto")
    color = [0.8, 0.8, 0.8]      # default light grey

STL files in the folder without an entry are listed too, with these defaults.
"""

import math
import re
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..frames import board_rotation

DEFAULT_SIZE_M = 4.0  # longest side of a model in the scene
DEFAULT_COLOR = (0.78, 0.80, 0.85)


# ---------- STL ----------


def load_stl(path) -> np.ndarray:
    """Binary or ASCII STL -> (N, 3, 3) float32 triangles."""
    data = Path(path).read_bytes()
    if len(data) >= 84:
        n = int(np.frombuffer(data[80:84], "<u4")[0])
        if len(data) == 84 + 50 * n:  # the size must match exactly, "solid" headers lie
            rec = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
            return np.frombuffer(data, rec, count=n, offset=84)["v"].astype(np.float32).copy()
    text = data.decode("ascii", errors="ignore")
    if not text.lstrip().startswith("solid"):
        raise ValueError(f"{path}: not an STL file")
    num = r"([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)"
    found = re.findall(rf"vertex\s+{num}\s+{num}\s+{num}", text)
    if not found or len(found) % 3:
        raise ValueError(f"{path}: no triangles found")
    return np.array(found, dtype=np.float32).reshape(-1, 3, 3)


def surface_centroid(tris: np.ndarray) -> np.ndarray:
    """Centre of the surface: triangle centres weighted by triangle area."""
    t = tris.astype(np.float64)
    area = 0.5 * np.linalg.norm(np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]), axis=1)
    if area.sum() <= 0:
        return t.reshape(-1, 3).mean(axis=0)
    return (t.mean(axis=1) * area[:, None]).sum(axis=0) / area.sum()


def bbox(tris: np.ndarray):
    v = tris.reshape(-1, 3)
    return v.min(axis=0), v.max(axis=0)


# ---------- registry ----------


AXES = {
    "+X": (1.0, 0.0, 0.0), "-X": (-1.0, 0.0, 0.0),
    "+Y": (0.0, 1.0, 0.0), "-Y": (0.0, -1.0, 0.0),
    "+Z": (0.0, 0.0, 1.0), "-Z": (0.0, 0.0, -1.0),
}  # fmt: skip


def axes_rotation(forward: str, up: str) -> np.ndarray:
    """Model -> body (FRD) rotation from two axes of the STL: which one is the nose and which one
    is the top. Raises ValueError if they lie on one line."""
    f, u = np.array(AXES[forward]), np.array(AXES[up])
    if abs(float(f @ u)) > 0.5:
        raise ValueError("nose axis and up axis must not lie on one line")
    d = -u  # body z points down
    r = np.cross(d, f)  # body y (right) = z x x
    return np.array([f, r, d])


def axes_from_matrix(r: np.ndarray) -> tuple[str, str]:
    """Inverse of axes_rotation for a matrix that only permutes and flips axes."""
    names = {v: k for k, v in AXES.items()}
    if not np.allclose(r, np.round(r), atol=1e-6):
        raise ValueError("rotation is not a multiple of 90 degrees")
    try:
        f = names[tuple(float(round(x)) for x in r[0])]
        u = names[tuple(float(round(-x)) for x in r[2])]
    except KeyError as e:
        raise ValueError("rotation is not a multiple of 90 degrees") from e
    return f, u


@dataclass
class ModelSpec:
    key: str
    display_name: str
    file: str | None = None  # None: built in (or a copy of one, see `base`)
    rotation_deg: tuple = (0.0, 0.0, 0.0)  # old way (models.toml); forward/up axes win if set
    scale: float = 1.0  # times the automatic size
    scale_xyz: tuple | None = None  # extra scale per body axis (x nose, y right, z down)
    center: tuple | None = None  # point of the STL to turn around; None = surface centroid
    color: tuple = DEFAULT_COLOR
    forward_axis: str | None = None  # axis of the STL that is the nose, e.g. "+X"
    up_axis: str | None = None  # axis of the STL that is the top, e.g. "+Z"
    builtin: bool = False
    base: str | None = None  # built-in geometry a copy is made of
    broken: bool = False  # file missing or unreadable: a placeholder is shown


@dataclass
class Mesh:
    tris: np.ndarray  # (N, 3, 3) float32, body frame FRD, centred, scaled to metres
    face_colors: np.ndarray  # (N, 4) float32 rgba
    spec: ModelSpec = field(default_factory=lambda: ModelSpec("?", "?"))

    def size(self) -> np.ndarray:
        lo, hi = bbox(self.tris)
        return hi - lo

    def sample_points(self, n=600) -> np.ndarray:
        """About n vertices, evenly spread (for the small thumbnails)."""
        v = self.tris.reshape(-1, 3)
        step = max(1, len(v) // n)
        return v[::step]


def prepare(tris: np.ndarray, spec: ModelSpec, size=DEFAULT_SIZE_M) -> np.ndarray:
    t = tris.astype(np.float64)
    c = surface_centroid(t) if spec.center is None else np.asarray(spec.center, float)
    t = t - c
    if spec.forward_axis and spec.up_axis:
        r = axes_rotation(spec.forward_axis, spec.up_axis)  # v_body = R v_model
    else:
        r = board_rotation(*spec.rotation_deg)
    t = t @ r.T
    lo, hi = bbox(t)
    longest = float((hi - lo).max())
    if longest <= 0:
        raise ValueError(f"model {spec.key} has no size")
    t = t * (size * spec.scale / longest)
    if spec.scale_xyz is not None:
        t = t * np.asarray(spec.scale_xyz, float)
    return t.astype(np.float32)


def _box(center, size) -> np.ndarray:
    """12 triangles of an axis-aligned box."""
    c, s = np.asarray(center, float), np.asarray(size, float) / 2
    corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * s + c
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for a, b, c2, d in quads:
        tris += [[corners[a], corners[b], corners[c2]], [corners[a], corners[c2], corners[d]]]
    return np.array(tris, dtype=np.float32)


def _rot_z(tris, deg):
    r = board_rotation(0, 0, deg)
    return (tris.astype(np.float64) @ r.T).astype(np.float32)


def _builtin_quad(size):
    body = _box((0, 0, 0), (0.30, 0.30, 0.10))
    parts, colors = [body], [(0.55, 0.60, 0.70, 1)]
    for deg in (45, 135, 225, 315):
        arm = _rot_z(_box((0.30, 0, 0), (0.60, 0.05, 0.03)), deg)
        motor = _rot_z(_box((0.60, 0, -0.03), (0.12, 0.12, 0.05)), deg)
        parts += [arm, motor]
        colors += [(0.78, 0.80, 0.86, 1), (0.95, 0.60, 0.12, 1)]
    parts.append(_box((0.20, 0, 0), (0.12, 0.10, 0.04)))  # nose marker
    colors.append((0.9, 0.15, 0.15, 1))
    return parts, colors


def _builtin_board(size):
    parts = [_box((0, 0, 0), (0.60, 0.40, 0.06)), _box((0.22, 0, -0.04), (0.10, 0.30, 0.03))]
    return parts, [(0.1, 0.45, 0.25, 1), (0.9, 0.15, 0.15, 1)]  # green board, red front edge


BUILTIN = {
    "quad": (ModelSpec("quad", "Quadcopter", builtin=True), _builtin_quad),
    "board": (ModelSpec("board", "IMU board", builtin=True), _builtin_board),
}


class ModelRegistry:
    """The models the GUI can show. Built from a folder (models.toml, bare .stl files) or from a
    list of specs (the model library). Every registry has its own `version`, so views know when
    to rebuild what they drew."""

    _versions = 0

    def __init__(self, models_dir=None, size=DEFAULT_SIZE_M, specs=None):
        ModelRegistry._versions += 1
        self.version = ModelRegistry._versions
        self.dir = Path(models_dir) if models_dir else None
        self.size = size
        self.specs: dict[str, ModelSpec] = {k: v[0] for k, v in BUILTIN.items()}
        self.cache: dict[str, Mesh] = {}
        self.native_cache: dict[str, tuple] = {}
        if specs is not None:
            for spec in specs:
                self.specs[spec.key] = spec
        elif self.dir and self.dir.is_dir():
            self._scan()

    def _scan(self):
        listed = {}
        toml = self.dir / "models.toml"
        if toml.exists():
            for entry in tomllib.loads(toml.read_text(encoding="utf-8")).get("model", []):
                file = entry["file"]
                key = entry.get("key") or Path(file).stem
                listed[file] = ModelSpec(
                    key=key,
                    display_name=entry.get("display_name", key),
                    file=file,
                    rotation_deg=tuple(entry.get("rotation_deg", (0, 0, 0))),
                    scale=float(entry.get("scale", 1.0)),
                    center=tuple(entry["center"]) if "center" in entry else None,
                    color=tuple(entry.get("color", DEFAULT_COLOR)),
                )
        for path in sorted(self.dir.glob("*.stl")):  # no entry needed: defaults
            listed.setdefault(path.name, ModelSpec(path.stem, path.stem, path.name))
        for spec in listed.values():
            if (self.dir / spec.file).exists():
                self.specs[spec.key] = spec

    def keys(self) -> list[str]:
        return list(self.specs)

    def names(self) -> dict[str, str]:
        return {k: s.display_name for k, s in self.specs.items()}

    def native(self, key: str) -> tuple[np.ndarray, np.ndarray | None]:
        """The model as stored: (triangles, per-face colours or None). Raises for a bad file."""
        if key not in self.native_cache:
            spec = self.specs[key]
            geometry = spec.base or (key if spec.file is None else None)
            if geometry is not None:
                parts, colors = BUILTIN[geometry][1](self.size)
                tris = np.concatenate(parts)
                fc = np.concatenate(
                    [
                        np.tile(np.array(c, np.float32), (len(p), 1))
                        for p, c in zip(parts, colors, strict=True)
                    ]
                )
            else:
                tris, fc = load_stl(self.dir / spec.file), None
            self.native_cache[key] = (tris, fc)
        return self.native_cache[key]

    def mesh(self, key: str) -> Mesh:
        if key not in self.specs:
            key = "quad"
        if key not in self.cache:
            spec = self.specs[key]
            try:
                tris, fc = self.native(key)
                if spec.builtin and spec.file is None and spec.base is None:
                    # built-in models are already in FRD and centred: only scale to the size
                    lo, hi = bbox(tris)
                    tris = (tris * (self.size * spec.scale / float((hi - lo).max()))).astype(
                        np.float32
                    )
                else:
                    tris = prepare(tris, spec, self.size)
                if fc is None:
                    fc = np.tile(np.array([*spec.color, 1.0], np.float32), (len(tris), 1))
                spec.broken = False
            except (OSError, ValueError, KeyError, TypeError):  # bad file or bad numbers
                spec.broken = True
                tris = _box((0, 0, 0), (self.size, self.size * 0.5, self.size * 0.15))
                fc = np.tile(np.array([0.45, 0.47, 0.52, 1.0], np.float32), (len(tris), 1))
            self.cache[key] = Mesh(tris, fc, spec)
        return self.cache[key]


# ---------- pose of a model in the scene ----------

NED_TO_SCENE = np.array(
    [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]
)  # x east, y north, z up


def pose_matrix(q_body_to_ned, position_enu=(0.0, 0.0, 0.0)) -> np.ndarray:
    """4x4 matrix taking model (FRD) points into the scene (x east, y north, z up)."""
    from ..frames import quat_to_matrix

    m = np.eye(4)
    m[:3, :3] = NED_TO_SCENE @ quat_to_matrix(q_body_to_ned)
    m[:3, 3] = position_enu
    return m


def yaw_of(q) -> float:
    w, x, y, z = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
