"""Calibration stages 0-2 of docs/module-context.md: data check, gyro, axes.

Everything works on raw values in the stream's native sensor frame. Inputs are sessions
(folders), CSVs of mtdata2_decoder.py, or raw dumps; see load().

Nothing here assumes axes, units or frames: they are measured and reported, with the
hypotheses they rest on named in the report.
"""

import json
import math
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np

from .frames import (
    G,
    angle_between,
    euler_deg_from_matrix,
    matrix_from_euler_deg,
    quat_from_gyro,
    quat_from_matrix,
    quat_mul,
    quat_normalize,
)
from .protocol_mtdata2 import ZERO_LEVEL, RepeatDetector

# ---------- loading ----------


@dataclass
class ImuData:
    name: str
    label: str | None
    t: np.ndarray  # s, primary time (device, else host / synthetic), starts at 0
    time_source: str
    accel: np.ndarray  # (N, 3) raw, as sent
    gyro: np.ndarray  # (N, 3) raw, as sent
    euler: np.ndarray  # (N, 3) device Euler as sent, NaN if absent
    status: np.ndarray  # (N,) int, -1 if absent
    x2060: np.ndarray  # (N,) NaN if absent
    accel_repeat: np.ndarray  # (N,) bool
    gyro_repeat: np.ndarray  # (N,) bool
    meta: dict = field(default_factory=dict)
    n_gnss_raw: int = 0  # GpsSol frames (rows without IMU data in a CSV)

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0]) if len(self.t) > 1 else 0.0

    def slice(self, t0, t1) -> "ImuData":
        m = (self.t >= t0) & (self.t < t1)
        return ImuData(
            self.name, self.label, self.t[m], self.time_source, self.accel[m], self.gyro[m],
            self.euler[m], self.status[m], self.x2060[m], self.accel_repeat[m],
            self.gyro_repeat[m], self.meta, 0,
        )  # fmt: skip


def _from_samples(name, label, samples, meta, n_gnss_raw, module_id=None) -> ImuData:
    samples = [
        s for s in samples if s.imu_id == 0 and (module_id is None or s.module_id == module_id)
    ]
    if not samples:
        raise ValueError(f"{name}: no IMU samples")
    t_us = np.array([s.t_us for s in samples], dtype=np.int64)
    nan3 = (math.nan,) * 3
    acc_rep, gyr_rep = RepeatDetector(), RepeatDetector()
    accel_repeat, gyro_repeat = [], []
    for s in samples:  # sessions written before the parser flagged repeats
        a, g = acc_rep(s.accel), gyr_rep(s.gyro)
        accel_repeat.append(s.accel_repeat if s.accel_repeat is not None else a)
        gyro_repeat.append(s.gyro_repeat if s.gyro_repeat is not None else g)
    return ImuData(
        name=name,
        label=label,
        t=(t_us - t_us[0]) * 1e-6,
        time_source=samples[0].time_source,
        accel=np.array([s.accel for s in samples], dtype=float),
        gyro=np.array([s.gyro for s in samples], dtype=float),
        euler=np.array([s.euler_deg or nan3 for s in samples], dtype=float),
        status=np.array([-1 if s.status is None else s.status for s in samples]),
        x2060=np.array([(s.extra or {}).get("0x2060", math.nan) for s in samples], dtype=float),
        accel_repeat=np.array(accel_repeat, dtype=bool),
        gyro_repeat=np.array(gyro_repeat, dtype=bool),
        meta=meta,
        n_gnss_raw=n_gnss_raw,
    )


def load(path, config=None, module_id=None) -> ImuData:
    """A session folder, a CSV of mtdata2_decoder.py, or a raw dump (synthetic time).
    module_id picks one module of a multi-module session (None = all rows, fine for one module)."""
    from .logger import read_stream
    from .messages import GnssRaw, ImuSample

    path = Path(path)
    if path.is_dir():
        meta_path = path / "meta.json"
        meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
        samples = read_stream(path, ImuSample)
        n_raw = len(read_stream(path, GnssRaw))
        return _from_samples(path.name, meta.get("label"), samples, meta, n_raw, module_id)
    if path.suffix.lower() == ".csv":
        from .sources import csv_messages

        msgs = csv_messages(path)
        samples = [m for m in msgs if isinstance(m, ImuSample)]
        n_raw = sum(isinstance(m, GnssRaw) for m in msgs)
        return _from_samples(path.name, path.stem, samples, {"source": "csv"}, n_raw)
    # raw dump: run it through the parser, no recording
    from .pipeline import Pipeline, load_config
    from .sources import BytesSource

    samples, raws = [], []
    p = Pipeline(BytesSource(path), config or load_config(), record=False)
    p.router.subscribe(ImuSample, samples.append)
    p.router.subscribe(GnssRaw, raws.append)
    p.run()
    return _from_samples(path.name, path.stem, samples, {"stats": p.stats()}, len(raws))


# ---------- helpers ----------


def detect_units(accel_norm_mean: float) -> tuple[str | None, float | None]:
    """Still |a|: ~9.8 -> m/s^2, ~1.0 -> g. Anything else: unknown (stop and report)."""
    if 9.3 <= accel_norm_mean <= 10.3:
        return "m/s^2", G
    if 0.95 <= accel_norm_mean <= 1.05:
        return "g", 1.0
    return None, None


def fresh(x: np.ndarray, repeat: np.ndarray) -> np.ndarray:
    """Values that are not stale repeats (for noise and bias statistics)."""
    return x[~repeat]


def is_still(d: ImuData, max_gyro_std=0.02, max_accel_std_rel=0.01) -> dict:
    """Crude stillness test of a whole session (gyro std in raw units, |a| std relative)."""
    g = fresh(d.gyro, d.gyro_repeat)
    an = np.linalg.norm(fresh(d.accel, d.accel_repeat), axis=1)
    gyro_std = float(np.linalg.norm(g.std(axis=0))) if len(g) > 1 else math.nan
    rel = float(an.std() / an.mean()) if len(an) > 1 and an.mean() > 0 else math.nan
    return {
        "still": bool(gyro_std < max_gyro_std and rel < max_accel_std_rel),
        "gyro_std_norm": gyro_std,
        "accel_norm_std_rel": rel,
    }


# Hypotheses for what the device Euler angles mean. Each gives the specific force direction
# (unit) the accelerometer should read, still, in the sensor frame.
def _pred_enu_zyx(e):  # Xsens default: sensor -> ENU, R = Rz Ry Rx; still reads +z_world
    return matrix_from_euler_deg(*e).T @ [0.0, 0.0, 1.0]


def _pred_ned_zyx(e):  # sensor -> NED, R = Rz Ry Rx; still reads -z_world (up)
    return matrix_from_euler_deg(*e).T @ [0.0, 0.0, -1.0]


def _pred_enu_xyz(e):  # extrinsic order swapped: R = Rx Ry Rz
    r, p, y = np.radians(e)
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return (rx @ ry @ rz).T @ [0.0, 0.0, 1.0]


def _pred_enu_swapped(e):  # roll and pitch swapped
    return _pred_enu_zyx((e[1], e[0], e[2]))


EULER_HYPOTHESES = {
    "ENU, ZYX (Xsens spec)": _pred_enu_zyx,
    "NED, ZYX": _pred_ned_zyx,
    "ENU, XYZ order": _pred_enu_xyz,
    "ENU, roll/pitch swapped": _pred_enu_swapped,
}


def euler_vs_accel(d: ImuData) -> dict | None:
    """Angle [deg] between the measured still accel direction and the one predicted from
    the device Euler angles, per hypothesis (doc item 4)."""
    e = d.euler[~np.isnan(d.euler).any(axis=1)]
    if len(e) == 0:
        return None
    a = fresh(d.accel, d.accel_repeat).mean(axis=0)
    if np.linalg.norm(a) < ZERO_LEVEL:
        return None
    a = a / np.linalg.norm(a)
    e_mean = e.mean(axis=0)  # fine while still and away from the +-180 yaw wrap
    if np.ptp(e[:, 2]) > 180:
        e_mean[2] = e[0, 2]
    out = {}
    for name, pred in EULER_HYPOTHESES.items():
        v = np.asarray(pred(e_mean), dtype=float)
        out[name] = round(math.degrees(math.acos(float(np.clip(v @ a, -1, 1)))), 3)
    return out


# ---------- stage 0: check ----------


def check(d: ImuData) -> dict:
    t = d.t
    dt = np.diff(t)
    med = float(np.median(dt)) if len(dt) else math.nan
    an = np.linalg.norm(fresh(d.accel, d.accel_repeat), axis=1)
    units, g = detect_units(float(an.mean()))
    absent = bool((np.abs(d.accel) < ZERO_LEVEL).all() and (np.abs(d.gyro) < ZERO_LEVEL).all())
    x = d.x2060[~np.isnan(d.x2060)]
    stats = d.meta.get("stats", {})
    parser = stats.get("parser", {}).get("mtdata2", {})
    rows = len(t) + d.n_gnss_raw
    return {
        "name": d.name,
        "label": d.label,
        "time_source": d.time_source,
        "samples": len(t),
        "duration_s": round(d.duration, 3),
        "rate_hz": round((len(t) - 1) / d.duration, 2) if d.duration > 0 else None,
        "interval_ms": {
            "median": round(1e3 * med, 3),
            "p5": round(1e3 * float(np.percentile(dt, 5)), 3) if len(dt) else None,
            "p95": round(1e3 * float(np.percentile(dt, 95)), 3) if len(dt) else None,
            "max": round(1e3 * float(dt.max()), 3) if len(dt) else None,
            "gaps_over_3x_median": int((dt > 3 * med).sum()) if len(dt) else 0,
        },
        "rows_without_imu_fraction": round(d.n_gnss_raw / rows, 4) if rows else None,
        "std_accel": d.accel.std(axis=0).tolist(),
        "std_gyro": d.gyro.std(axis=0).tolist(),
        "std_euler": np.nanstd(d.euler, axis=0).tolist(),
        "frozen_axes": {
            "accel": [bool(v == 0) for v in d.accel.std(axis=0)],
            "gyro": [bool(v == 0) for v in d.gyro.std(axis=0)],
        },
        "repeat_fraction": {
            "accel": round(float(d.accel_repeat.mean()), 4),
            "gyro": round(float(d.gyro_repeat.mean()), 4),
        },
        "imu_board_absent": absent,
        "accel_norm": {"mean": float(an.mean()), "std": float(an.std())},
        "units_guess": {"accel": units, "g": g, "gyro": "rad/s per spec, unverified"},
        "still": is_still(d),
        "status_values": {
            str(k): int(v) for k, v in zip(*np.unique(d.status, return_counts=True), strict=True)
        },  # fmt: skip
        "x2060": {
            "mean": float(x.mean()),
            "std": float(x.std()),
            "min": float(x.min()),
            "max": float(x.max()),
        }
        if len(x)
        else None,  # fmt: skip
        "euler_vs_accel_deg": euler_vs_accel(d) if not absent else None,
        "checksum_errors": stats.get("checksum_errors"),
        "bad_frames_by_mid_len": parser.get("bad_checksum_by_mid_len"),
    }


def check_markdown(results: list[dict]) -> str:
    lines = [
        "| session | src | N | s | Hz | dt med/p95/max ms | gaps | no-IMU rows | |a| mean | "
        "units | rep a/g | still | absent |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in results:
        i = r["interval_ms"]
        lines.append(
            f"| {r['label'] or r['name']} | {r['time_source']} | {r['samples']} | "
            f"{r['duration_s']:.1f} | {r['rate_hz']} | {i['median']:.1f}/{i['p95']:.1f}/"
            f"{i['max']:.1f} | {i['gaps_over_3x_median']} | {r['rows_without_imu_fraction']} | "
            f"{r['accel_norm']['mean']:.4f} | {r['units_guess']['accel']} | "
            f"{r['repeat_fraction']['accel']:.3f}/{r['repeat_fraction']['gyro']:.3f} | "
            f"{'yes' if r['still']['still'] else 'no'} | "
            f"{'YES' if r['imu_board_absent'] else 'no'} |"
        )
    lines.append("")
    for r in results:
        if r["euler_vs_accel_deg"]:
            best = min(r["euler_vs_accel_deg"], key=r["euler_vs_accel_deg"].get)
            vals = ", ".join(f"{k}: {v:.2f}" for k, v in r["euler_vs_accel_deg"].items())
            lines.append(f"- {r['label'] or r['name']}: Euler vs accel [deg] {vals} (best: {best})")
        frozen = [k for k, v in r["frozen_axes"].items() if any(v)]
        if frozen and not r["imu_board_absent"]:
            lines.append(f"- {r['label'] or r['name']}: FROZEN axes in {frozen}")
        if r["units_guess"]["accel"] is None and not r["imu_board_absent"]:
            lines.append(f"- {r['label'] or r['name']}: |a| fits neither m/s^2 nor g: STOP")
    return "\n".join(lines)


# ---------- stage 2: gyro ----------


def gyro_still(d: ImuData, window_s=10.0) -> dict:
    g = fresh(d.gyro, d.gyro_repeat)
    t = fresh(d.t, d.gyro_repeat)
    bias = g.mean(axis=0)
    std = g.std(axis=0)
    first = g[t < t[0] + window_s].mean(axis=0)
    last = g[t >= t[-1] - window_s].mean(axis=0)
    n_win = max(1, int((t < t[0] + window_s).sum()))
    return {
        "session": d.label or d.name,
        "duration_s": round(d.duration, 2),
        "samples_used": len(g),
        "repeats_dropped": int(d.gyro_repeat.sum()),
        "bias": bias.tolist(),
        "noise_std": std.tolist(),
        "bias_first_window": first.tolist(),
        "bias_last_window": last.tolist(),
        "bias_drift": (last - first).tolist(),
        "drift_in_sigma_of_mean": ((last - first) / (std * math.sqrt(2 / n_win))).tolist(),
        "window_s": window_s,
        "long_enough": d.duration >= 30.0,
        "units_hint": units_hint(std),
        "still": is_still(d),
    }


def units_hint(noise_std) -> str:
    """Typical MEMS gyro noise at ~100 Hz is 1e-4..1e-2 rad/s. In deg/s it is 57x larger."""
    n = float(np.linalg.norm(noise_std))
    if 1e-5 < n < 2e-2:
        return f"noise {n:.2e} fits rad/s (would be implausibly low in deg/s)"
    if 2e-2 <= n < 1.0:
        return f"noise {n:.2e} fits deg/s better than rad/s"
    return f"noise {n:.2e}: no guess"


def integrate_gyro(t, gyro, bias) -> np.ndarray:
    """Attitude change (quaternion, sensor frame) from integrating gyro over time t."""
    q = np.array([1.0, 0.0, 0.0, 0.0])
    w = gyro - bias
    for k in range(len(t) - 1):
        w_mid = 0.5 * (w[k] + w[k + 1])  # trapezoid
        q = quat_mul(q, quat_from_gyro(w_mid, t[k + 1] - t[k]))
    return quat_normalize(q)


def gyro_rotation(d: ImuData, bias, edge_s=1.0) -> dict:
    """Scale check: gyro-integrated rotation vs device Euler change (ZYX) over the session.
    Record: >= edge_s still, one rotation about one axis, >= edge_s still."""
    m = ~d.gyro_repeat
    t, g = d.t[m], d.gyro[m]
    q = integrate_gyro(t, g, np.asarray(bias))
    angle_gyro = math.degrees(2 * math.acos(min(1.0, abs(q[0]))))
    axis = q[1:] / (np.linalg.norm(q[1:]) or 1.0)
    e = d.euler
    start = np.nanmean(e[d.t < edge_s], axis=0)
    end = np.nanmean(e[d.t > d.t[-1] - edge_s], axis=0)
    out = {
        "session": d.label or d.name,
        "angle_gyro_deg": round(angle_gyro, 3),
        "axis_sensor": axis.round(4).tolist(),
        "euler_start": start.tolist(),
        "euler_end": end.tolist(),
        "peak_rate": float(np.linalg.norm(g - bias, axis=1).max()),
    }
    if not np.isnan(start).any() and not np.isnan(end).any():
        q0 = quat_from_matrix(matrix_from_euler_deg(*start))
        q1 = quat_from_matrix(matrix_from_euler_deg(*end))
        angle_euler = math.degrees(angle_between(q0, q1))
        out["angle_euler_deg_zyx"] = round(angle_euler, 3)
        out["scale_gyro_over_euler"] = round(angle_gyro / angle_euler, 5) if angle_euler else None
    return out


def gyro_markdown(still: dict, rots: list[dict]) -> str:
    f = lambda v: ", ".join(f"{x:+.3e}" for x in v)  # noqa: E731
    lines = [
        f"### Gyro, still session `{still['session']}` ({still['duration_s']} s, "
        f"{still['samples_used']} samples, {still['repeats_dropped']} repeats dropped)",
        "",
        f"- bias: {f(still['bias'])}",
        f"- noise std: {f(still['noise_std'])}",
        f"- bias first {still['window_s']:.0f} s: {f(still['bias_first_window'])}",
        f"- bias last {still['window_s']:.0f} s: {f(still['bias_last_window'])}",
        f"- drift: {f(still['bias_drift'])} "
        f"({', '.join(f'{x:+.1f}' for x in still['drift_in_sigma_of_mean'])} sigma of the mean)",
        f"- units: {still['units_hint']}",
    ]
    if not still["long_enough"]:
        lines.append("- WARNING: shorter than 30 s, stability numbers are weak")
    if not still["still"]["still"]:
        lines.append(f"- WARNING: session does not look still: {still['still']}")
    if rots:
        lines += [
            "",
            "| rotation | gyro angle deg | Euler angle deg (ZYX) | ratio | axis (sensor) |",
            "|---|---|---|---|---|",
        ]
        for r in rots:
            lines.append(
                f"| {r['session']} | {r['angle_gyro_deg']:.2f} | "
                f"{r.get('angle_euler_deg_zyx', float('nan')):.2f} | "
                f"{r.get('scale_gyro_over_euler')} | {r['axis_sensor']} |"
            )
    return "\n".join(lines)


# ---------- stage 1: axes ----------

# Mission Planner poses: which body direction points UP (FRD body: x fwd, y right, z down).
# Still, the accelerometer reads +g along "up".
POSES = {
    "level": (0.0, 0.0, -1.0),  # top up
    "back": (0.0, 0.0, 1.0),  # upside down
    "left": (0.0, 1.0, 0.0),  # left side down -> right side up
    "right": (0.0, -1.0, 0.0),  # right side down -> left side up
    "nosedown": (-1.0, 0.0, 0.0),  # front down -> rear up
    "noseup": (1.0, 0.0, 0.0),  # front up
}
BODY_AXES = ["x (forward)", "y (right)", "z (down)"]


def pose_of(d: ImuData, explicit: str | None = None) -> str:
    name = (explicit or d.label or d.name).lower()
    for p in sorted(POSES, key=len, reverse=True):  # "nosedown" before "down"...
        if p in name.replace("_", "").replace("-", ""):
            return p
    raise ValueError(f"{d.name}: cannot tell the pose; name it one of {list(POSES)}")


def kabsch(measured: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Rotation R minimizing sum |R m_i - t_i|^2 (rows are unit vectors)."""
    h = measured.T @ target
    u, _, vt = np.linalg.svd(h)
    dsign = np.sign(np.linalg.det(vt.T @ u.T))
    return vt.T @ np.diag([1.0, 1.0, dsign]) @ u.T


def nearest_signed_permutation(r: np.ndarray) -> np.ndarray:
    p = np.zeros((3, 3))
    used = set()
    for i in np.argsort(-np.abs(r).max(axis=1)):  # most certain rows first
        j = max((j for j in range(3) if j not in used), key=lambda j: abs(r[i, j]))
        p[i, j] = np.sign(r[i, j])
        used.add(j)
    return p


def axes(datas: list[ImuData], poses: list[str] | None = None) -> dict:
    """Propose the sensor -> body mapping from still poses with known 'up' sides."""
    rows = []
    for k, d in enumerate(datas):
        pose = pose_of(d, poses[k] if poses else None)
        a = fresh(d.accel, d.accel_repeat).mean(axis=0)
        rows.append(
            {
                "session": d.label or d.name,
                "pose": pose,
                "mean_accel": a.tolist(),
                "norm": float(np.linalg.norm(a)),
                "dominant": _dominant(a),
                "still": is_still(d),
            }
        )
    m = np.array([np.array(r["mean_accel"]) / r["norm"] for r in rows])
    u = np.array([POSES[r["pose"]] for r in rows])
    covered = sorted({r["pose"] for r in rows})
    result = {"poses": rows, "covered": covered}
    rank = np.linalg.matrix_rank(np.array([POSES[p] for p in covered]), tol=0.5)
    if rank < 2:
        result["error"] = "need poses with at least two different up axes"
        return result
    r_fit = kabsch(m, u)
    perm = nearest_signed_permutation(r_fit)
    for row, mi, ui in zip(rows, m, u, strict=True):
        row["residual_deg_fit"] = round(
            math.degrees(math.acos(np.clip((r_fit @ mi) @ ui, -1, 1))), 3
        )
        row["residual_deg_perm"] = round(
            math.degrees(math.acos(np.clip((perm @ mi) @ ui, -1, 1))), 3
        )
    mapping = []
    for i in range(3):  # sensor axis i -> body axis j with sign
        j = int(np.argmax(np.abs(perm[:, i])))
        sign = "+" if perm[j, i] > 0 else "-"
        mapping.append(f"sensor +{'xyz'[i]} -> body {sign}{BODY_AXES[j]}")
    det = float(np.linalg.det(perm))
    result |= {
        "rotation_fit": r_fit.round(5).tolist(),
        "signed_permutation": perm.astype(int).tolist(),
        "det": det,
        "mapping": mapping,
        "board_alignment_deg": [round(v, 6) + 0.0 for v in euler_deg_from_matrix(perm)],
        "mount_tilt_deg": round(
            math.degrees(math.acos(np.clip((np.trace(perm.T @ r_fit) - 1) / 2, -1, 1))), 3
        ),  # fmt: skip
        "pairwise_min_angle_deg": _min_pair_angle(m),
    }
    if det < 0:
        result["error"] = "sensor axes are LEFT-handed: no rotation maps them, check signs"
    return result


def _dominant(a) -> str:
    i = int(np.argmax(np.abs(a)))
    return f"{'+' if a[i] > 0 else '-'}{'xyz'[i]}"


def _min_pair_angle(m) -> float | None:
    best = None
    for i in range(len(m)):
        for j in range(i + 1, len(m)):
            ang = math.degrees(math.acos(np.clip(m[i] @ m[j], -1, 1)))
            best = ang if best is None else min(best, ang)
    return round(best, 2) if best is not None else None


def axes_markdown(r: dict) -> str:
    lines = [
        "| session | pose (up side) | mean accel (sensor) | |a| | dominant | still | resid deg |",
        "|---|---|---|---|---|---|---|",
    ]
    for p in r["poses"]:
        a = ", ".join(f"{v:+.3f}" for v in p["mean_accel"])
        lines.append(
            f"| {p['session']} | {p['pose']} | {a} | {p['norm']:.4f} | {p['dominant']} | "
            f"{'yes' if p['still']['still'] else 'NO'} | {p.get('residual_deg_perm', '')} |"
        )
    lines.append("")
    if "error" in r:
        lines.append(f"**{r['error']}**")
    if "mapping" in r:
        lines += [f"- {m}" for m in r["mapping"]]
        lines.append(f"- board_alignment_deg (roll, pitch, yaw) = {r['board_alignment_deg']}")
        lines.append(f"- mounting tilt left after the permutation: {r['mount_tilt_deg']} deg")
        lines.append(f"- smallest angle between two poses: {r['pairwise_min_angle_deg']} deg")
        missing = sorted(set(POSES) - set(r["covered"]))
        if missing:
            lines.append(f"- poses not covered: {missing}")
    return "\n".join(lines)


def write_board_alignment(config_path, deg) -> None:
    """Set [imu.default] board_alignment_deg in a TOML config (created if missing)."""
    import tomllib

    path = Path(config_path)
    lines = path.read_text().splitlines() if path.exists() else []
    new = f"board_alignment_deg = [{', '.join(f'{v:g}' for v in deg)}]"
    head = next((i for i, x in enumerate(lines) if x.strip() == "[imu.default]"), None)
    if head is None:
        lines += ([""] if lines else []) + ["[imu.default]", new]
    else:
        end = next((i for i in range(head + 1, len(lines)) if lines[i].lstrip().startswith("[")),
                   len(lines))  # fmt: skip
        key = next((i for i in range(head + 1, end)
                    if lines[i].split("=")[0].strip() == "board_alignment_deg"), None)  # fmt: skip
        if key is None:
            lines.insert(head + 1, new)
        else:
            lines[key] = new
    text = "\n".join(lines) + "\n"
    tomllib.loads(text)  # still valid TOML
    path.write_text(text)


# ---------- reports ----------


def save_report(kind: str, markdown: str, data, base="calib/reports") -> Path:
    d = Path(base) / datetime.now().strftime("%Y-%m-%d")
    d.mkdir(parents=True, exist_ok=True)
    stem = f"{kind}_{datetime.now().strftime('%H%M%S')}"
    (d / f"{stem}.md").write_text(markdown + "\n")
    (d / f"{stem}.json").write_text(json.dumps(data, indent=2, default=float) + "\n")
    return d / f"{stem}.md"


# ---------- accelerometer calibration (stage 3), level trim, report, apply ----------
#
# Model (same as Calibration): a_cal = M (a_raw - b), in the sensor frame.
# Input is one MEAN raw accel vector per still pose. The fits use only the fact that a still
# sensor must read |a| = g, so they do not depend on how exactly the pose was held.

# limits in the spirit of Mission Planner's sanity checks; ours, not copied from MP
MAX_OFFSET = 3.5  # |b_i| in m/s^2
SCALE_RANGE = (0.8, 1.25)  # diagonal of M
MAX_REL_ERR = 0.01  # |a_cal| within 1 % of g in every pose (docs/module-context.md section 10)
MIN_POSES_NINE = 12
MAX_COND_NINE = 100.0  # Jacobian condition number above which 9 parameters are not observable
DISTINCT_DEG = 15.0  # poses closer than this count as one


@dataclass
class PoseSample:
    pose: str | None  # "level", "back", ... or None (free pose for the 9-parameter fit)
    mean: np.ndarray  # mean raw accel, sensor frame
    std: np.ndarray
    n: int
    name: str = ""
    still: bool = True


def pose_samples(datas: list[ImuData], poses: list[str | None] | None = None, trim_s=1.0):
    """One PoseSample per still session; the first and last trim_s seconds are dropped."""
    out = []
    for k, d in enumerate(datas):
        if d.duration > 4 * trim_s:
            d = d.slice(d.t[0] + trim_s, d.t[-1] - trim_s)
        a = fresh(d.accel, d.accel_repeat)
        explicit = poses[k] if poses else None
        try:
            pose = pose_of(d, explicit)
        except ValueError:
            pose = None
        out.append(
            PoseSample(
                pose, a.mean(axis=0), a.std(axis=0), len(a), d.label or d.name, is_still(d)["still"]
            )  # fmt: skip
        )
    return out


def _norms(a: np.ndarray, b, m) -> np.ndarray:
    return np.linalg.norm((a - b) @ m.T, axis=1)


def _sym(p6) -> np.ndarray:
    m00, m11, m22, m01, m02, m12 = p6
    return np.array([[m00, m01, m02], [m01, m11, m12], [m02, m12, m22]])


def _axis_fit(a: np.ndarray, g: float):
    """Simple per-axis method: offset = mean of the two opposite readings, scale = 2g / span."""
    b, s = np.zeros(3), np.ones(3)
    for k in range(3):
        hi, lo = a[:, k].max(), a[:, k].min()
        b[k] = (hi + lo) / 2
        s[k] = 2 * g / (hi - lo)
    return b, np.diag(s)


def distinct_poses(a: np.ndarray, deg=DISTINCT_DEG) -> int:
    """How many of the mean vectors point in clearly different directions."""
    unit = a / np.linalg.norm(a, axis=1, keepdims=True)
    kept = []
    for u in unit:
        if all(math.degrees(math.acos(np.clip(u @ v, -1, 1))) > deg for v in kept):
            kept.append(u)
    return len(kept)


def fit_accel(samples: list[PoseSample], g: float = G, model: str = "six") -> dict:
    """Fit a_cal = M (a - b).

    model "six":  b and a diagonal M (6 parameters), least squares on |a_cal| = g. Default.
    model "nine": b and a symmetric M (9 parameters, adds cross-axis terms). Only with
                  >= 12 clearly different poses and an observable Jacobian, else an error.
    model "axis": simple per-axis method (needs the 6 axis-aligned poses); for comparison.
    """
    from scipy.optimize import least_squares

    a = np.array([s.mean for s in samples])
    result = {"model": model, "g": g, "n_poses": len(a), "warnings": [], "errors": []}
    if len(a) < 6:
        result["errors"].append(f"need at least 6 poses, got {len(a)}")
        return result
    if model == "axis":
        b, m = _axis_fit(a, g)
        n_par = 6
    elif model in ("six", "nine"):
        b0 = a.mean(axis=0)
        if model == "six":
            n_par = 6

            def unpack(p):
                return p[:3], np.diag(p[3:])

            x0 = np.r_[b0, np.ones(3)]
        else:
            n_par = 9
            distinct = distinct_poses(a)
            if len(a) < MIN_POSES_NINE or distinct < MIN_POSES_NINE:
                result["errors"].append(
                    f"9 parameters need >= {MIN_POSES_NINE} clearly different poses "
                    f"({len(a)} given, {distinct} different); use model six"
                )
                return result

            def unpack(p):
                return p[:3], _sym(p[3:])

            x0 = np.r_[b0, 1, 1, 1, 0, 0, 0]

        def resid(p):
            bb, mm = unpack(p)
            return _norms(a, bb, mm) - g

        fit = least_squares(resid, x0, method="lm" if len(a) >= n_par else "trf")
        b, m = unpack(fit.x)
        sv = np.linalg.svd(fit.jac, compute_uv=False)
        cond = float(sv[0] / sv[-1]) if sv[-1] > 0 else math.inf
        result["jacobian_condition"] = cond
        if model == "nine" and (len(sv) < n_par or cond > MAX_COND_NINE):
            result["errors"].append(
                f"9-parameter fit not observable (Jacobian condition {cond:.3g} > "
                f"{MAX_COND_NINE:g}): poses do not cover all directions; use model six"
            )
            return result
    else:
        raise ValueError(f"unknown model {model!r}")

    before = np.linalg.norm(a, axis=1) / g - 1
    after = _norms(a, b, m) / g - 1
    result |= {
        "offset_b": b.tolist(),
        "matrix_M": m.tolist(),
        "scale_diag": np.diag(m).tolist(),
        "redundancy": len(a) - n_par,
        "rel_err_before": before.tolist(),
        "rel_err_after": after.tolist(),
        "rms_before_ms2": float(np.sqrt(np.mean((before * g) ** 2))),
        "rms_after_ms2": float(np.sqrt(np.mean((after * g) ** 2))),
        "max_rel_err_after": float(np.abs(after).max()),
        "poses": [s.pose for s in samples],
        "names": [s.name for s in samples],
    }
    if result["redundancy"] <= 0:
        result["warnings"].append(
            f"{len(a)} poses for {n_par} parameters: the fit is exactly determined, so the "
            "residual after calibration proves nothing; check |a| on an extra pose"
        )
    checks = check_accel(result, samples)
    result["checks"] = checks
    result["errors"] += [c["text"] for c in checks if c["level"] == "error"]
    result["warnings"] += [c["text"] for c in checks if c["level"] == "warn"]
    return result


def check_accel(fit: dict, samples: list[PoseSample]) -> list[dict]:
    """Sanity checks (Mission Planner style). Each: {level: ok|warn|error, text}."""
    out = []

    def add(level, text):
        out.append({"level": level, "text": text})

    b = np.array(fit["offset_b"])
    scale = np.array(fit["scale_diag"])
    if np.abs(b).max() > MAX_OFFSET:
        add("error", f"offset {np.round(b, 3).tolist()} exceeds {MAX_OFFSET} m/s^2")
    else:
        add("ok", f"offsets within {MAX_OFFSET} m/s^2: {np.round(b, 3).tolist()}")
    if scale.min() < SCALE_RANGE[0] or scale.max() > SCALE_RANGE[1]:
        add("error", f"scale {np.round(scale, 4).tolist()} outside {SCALE_RANGE}")
    else:
        add("ok", f"scales within {SCALE_RANGE}: {np.round(scale, 4).tolist()}")
    if fit["max_rel_err_after"] > MAX_REL_ERR:
        worst = 100 * fit["max_rel_err_after"]
        add("warn", f"|a| after calibration differs from g by up to {worst:.2f} % "
                    f"(limit {100 * MAX_REL_ERR:g} %)")  # fmt: skip
    else:
        add("ok", f"|a| within {100 * MAX_REL_ERR:g} % of g in every pose "
                  f"(max {100 * fit['max_rel_err_after']:.3f} %)")  # fmt: skip
    dominant = [_dominant(s.mean) for s in samples]
    if len(set(dominant)) < len(dominant) and fit["model"] != "nine":
        add("warn", f"two poses point the same way (dominant axes {dominant})")
    for s in samples:
        if not s.still:
            add("warn", f"pose {s.name or s.pose} was not still")
    return out


def level_trim(mean_accel_raw, cal_offset, cal_matrix, board_alignment_deg) -> dict:
    """Roll / pitch trim [deg] that makes the level pose read (0, 0, -g) in the body frame.

    Uses the calibrated accel of the level pose, turned into the body frame by the board
    alignment; the pipeline then applies board_rotation(roll, pitch, 0) @ that rotation.
    """
    a_cal = np.asarray(cal_matrix) @ (np.asarray(mean_accel_raw) - np.asarray(cal_offset))
    r_board = matrix_from_euler_deg(*board_alignment_deg)
    u = r_board @ a_cal
    u = u / np.linalg.norm(u)
    roll = math.atan2(-u[1], -u[2])
    z1 = u[1] * math.sin(roll) + u[2] * math.cos(roll)  # after the roll: (u_x, 0, z1), z1 < 0
    pitch = math.atan2(u[0], -z1)
    trim = (math.degrees(roll), math.degrees(pitch))
    check = matrix_from_euler_deg(trim[0], trim[1], 0.0) @ u
    return {
        "level_trim_deg": [round(trim[0], 4), round(trim[1], 4)],
        "tilt_before_deg": round(math.degrees(math.acos(np.clip(-u[2], -1, 1))), 3),
        "residual_after": np.round(check, 6).tolist(),  # should be (0, 0, -1)
        "board_alignment_deg": list(board_alignment_deg),
    }


def build_calibration(accel=None, gyro_bias=None, level=None, info=None):
    """Put the fit results into a Calibration (ready for .save())."""
    from .calibration import Calibration

    cal = Calibration(info=dict(info or {}))
    if accel:
        cal.accel_offset = np.array(accel["offset_b"], dtype=float)
        cal.accel_matrix = np.array(accel["matrix_M"], dtype=float)
        cal.info["g"] = accel["g"]
        cal.info["units"] = "m/s^2" if abs(accel["g"] - G) < 0.5 else "g"
        cal.info["accel_fit"] = {
            k: accel[k]
            for k in (
                "model",
                "n_poses",
                "redundancy",
                "rms_before_ms2",
                "rms_after_ms2",
                "max_rel_err_after",
                "warnings",
            )  # fmt: skip
        }
    if gyro_bias is not None:
        cal.gyro_bias = np.asarray(gyro_bias, dtype=float)
    if level:
        cal.level_trim_deg = tuple(level["level_trim_deg"])
    cal.info["created"] = datetime.now().isoformat(timespec="seconds")
    return cal


def accel_markdown(fit: dict) -> str:
    lines = [f"model: **{fit['model']}**, poses: {fit['n_poses']}, g = {fit['g']:.5f}"]
    for e in fit["errors"]:
        lines.append(f"- ERROR: {e}")
    if "offset_b" not in fit:
        return "\n".join(lines)
    lines += [
        f"- offset b = {np.round(fit['offset_b'], 5).tolist()}",
        "- matrix M = " + str(np.round(fit["matrix_M"], 5).tolist()),
        f"- redundancy (poses - parameters): {fit['redundancy']}",
        f"- RMS |a|-g [m/s^2]: before {fit['rms_before_ms2']:.4f}, "
        f"after {fit['rms_after_ms2']:.4f}",
        "",
        "| pose | session | |a|/g-1 before [%] | after [%] |",
        "|---|---|---|---|",
    ]
    for p, n, b, a in zip(fit["poses"], fit["names"], fit["rel_err_before"], fit["rel_err_after"],
                          strict=True):  # fmt: skip
        lines.append(f"| {p} | {n} | {100 * b:+.3f} | {100 * a:+.3f} |")
    lines.append("")
    lines += [f"- {c['level'].upper()}: {c['text']}" for c in fit["checks"]]
    shown = {c["text"] for c in fit["checks"]}
    lines += [f"- WARNING: {w}" for w in fit["warnings"] if w not in shown]
    return "\n".join(lines)


def compare_models(samples: list[PoseSample], g: float = G) -> dict:
    """six and axis side by side (nine too when enough poses were given)."""
    out = {m: fit_accel(samples, g, m) for m in ("six", "axis")}
    if len(samples) >= MIN_POSES_NINE:
        out["nine"] = fit_accel(samples, g, "nine")
    return out


def report(accel=None, level=None, gyro=None, out_dir=None, name=None) -> Path:
    """Markdown + PNG of a calibration: |a| per pose before / after, numbers, checks."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    d = Path(out_dir) if out_dir else Path("calib/reports") / datetime.now().strftime("%Y-%m-%d")
    d.mkdir(parents=True, exist_ok=True)
    stem = name or f"report_{datetime.now().strftime('%H%M%S')}"
    md = ["## Calibration report", ""]
    if accel:
        md += ["### Accelerometer", accel_markdown(accel), ""]
    if gyro:
        md += ["### Gyro", f"- bias (sensor units): {np.round(gyro['bias'], 7).tolist()}",
               f"- noise std: {np.round(gyro['noise_std'], 7).tolist()}",
               f"- still: {gyro['still']['still']}", ""]  # fmt: skip
    if level:
        md += ["### Level", f"- trim roll, pitch [deg]: {level['level_trim_deg']}",
               f"- tilt before trim: {level['tilt_before_deg']} deg", ""]  # fmt: skip
    (d / f"{stem}.md").write_text("\n".join(md) + "\n")
    if accel and "rel_err_before" in accel:
        fig, ax = plt.subplots(figsize=(7, 3.6))
        x = np.arange(len(accel["poses"]))
        ax.bar(x - 0.2, 100 * np.array(accel["rel_err_before"]), 0.4, label="raw")
        ax.bar(x + 0.2, 100 * np.array(accel["rel_err_after"]), 0.4, label="calibrated")
        ax.axhline(100 * MAX_REL_ERR, color="gray", ls="--", lw=0.8)
        ax.axhline(-100 * MAX_REL_ERR, color="gray", ls="--", lw=0.8)
        ax.set_xticks(x, [p or "?" for p in accel["poses"]])
        ax.set_ylabel("|a| / g - 1  [%]")
        ax.set_title(f"accel calibration ({accel['model']}), dashed = 1 % limit")
        ax.legend()
        fig.tight_layout()
        fig.savefig(d / f"{stem}.png", dpi=110)
        plt.close(fig)
    return d / f"{stem}.md"


def latest_report(kind: str, base="calib/reports") -> Path | None:
    """Newest <kind>_*.json under calib/reports/<date>/ (names sort by date and time)."""
    found = sorted(Path(base).glob(f"*/{kind}_*.json"), key=lambda p: (p.parent.name, p.name))
    return found[-1] if found else None
