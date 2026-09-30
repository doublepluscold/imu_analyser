"""Rerun live view.

Entity tree
  world                     NED, viewer told "z is down"; axes + ground grid
  world/vehicle             Transform3D: rotation from State.q, translation from State.pos_ned
  world/vehicle/model       the quad (child, so it moves with the vehicle)
  world/trajectory          line of past positions (only once an estimator fills pos_ned)
  world/diag/imu<k>         one small model per IMU, own filter, fixed offset next to the vehicle
  plots/gyro/imu<k>, plots/accel/imu<k>   as sent by the device (sensor frame, raw)
  plots/accel_norm/imu<k>   |a| raw, |a| calibrated, and a line at g
  plots/device_euler        Euler angles computed by the device itself, as sent
  plots/status              device status bits
  plots/attitude, plots/disagreement/imu<k>, plots/gnss/* (only fixes with the fix flag)
  events                    text log

Timelines: time (device clock if the stream has one, else reconstructed host time; starts at 0
with the first message) and pc_time (arrival of the USB chunk at the PC, wall clock).
Data is buffered and sent every ~50 ms with rr.send_columns (much cheaper than rr.log per sample).
"""

import math
import time
from collections import defaultdict

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from .frames import G, angle_between, quat_to_euler, tare, to_rerun_quat
from .messages import Command, Event, GnssFix, ImuSample, State

RED, GRAY, DARK = [230, 40, 40], [150, 150, 150], [60, 60, 60]
XYZ_COLORS = [[230, 60, 60], [60, 200, 60], [60, 120, 240]]
DIAG_SPACING = 0.5  # m between diagnostic models


def blueprint(show_gnss=False) -> rrb.Blueprint:
    last_10s = rrb.VisibleTimeRange(
        "time",
        start=rrb.TimeRangeBoundary.cursor_relative(seconds=-10.0),
        end=rrb.TimeRangeBoundary.cursor_relative(),
    )
    plots = [
        rrb.TimeSeriesView(origin="plots/gyro", name="Gyro [rad/s]", time_ranges=last_10s),
        rrb.TimeSeriesView(origin="plots/accel", name="Accel (as sent)", time_ranges=last_10s),
        rrb.TimeSeriesView(origin="plots/accel_norm", name="|a| and g", time_ranges=last_10s),
        rrb.TimeSeriesView(
            origin="plots/device_euler", name="Device Euler [deg]", time_ranges=last_10s
        ),
        rrb.TimeSeriesView(origin="plots/status", name="Device status", time_ranges=last_10s),
        rrb.TimeSeriesView(
            origin="plots/attitude", name="Roll/Pitch/Yaw [deg]", time_ranges=last_10s
        ),
        rrb.TimeSeriesView(
            origin="plots/disagreement", name="IMU disagreement [deg]", time_ranges=last_10s
        ),
    ]
    if show_gnss:
        plots.append(rrb.TimeSeriesView(origin="plots/gnss", name="GNSS", time_ranges=last_10s))
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                rrb.Spatial3DView(origin="world", name="3D", line_grid=rrb.LineGrid3D(spacing=0.1)),
                rrb.Vertical(*plots),
                column_shares=[3, 2],
            ),
            rrb.TextLogView(origin="events", name="Events"),
            row_shares=[5, 1],
        ),
        collapse_panels=True,
    )


def log_quad_model(path, arm=0.25, scale=1.0):
    """Procedural X-quad in body FRD coordinates. Front half is red."""
    a = arm * scale / math.sqrt(2)
    motors = [[a, a, 0], [a, -a, 0], [-a, a, 0], [-a, -a, 0]]  # FR, FL, RR, RL
    colors = [RED, RED, GRAY, GRAY]
    up = -0.02 * scale  # z is down, so "above" is negative z
    rr.log(
        f"{path}/arms",
        rr.LineStrips3D([[[0, 0, 0], m] for m in motors], radii=0.012 * scale, colors=colors),
        static=True,
    )
    rr.log(
        f"{path}/motors",
        rr.Cylinders3D(
            lengths=0.03 * scale,
            radii=0.022 * scale,
            centers=[[m[0], m[1], up] for m in motors],
            colors=colors,
        ),
        static=True,
    )
    rr.log(
        f"{path}/props",
        rr.Cylinders3D(
            lengths=0.003 * scale,
            radii=0.11 * scale,
            centers=[[m[0], m[1], 2.2 * up] for m in motors],
            colors=[[230, 40, 40, 90]] * 2 + [[150, 150, 150, 90]] * 2,
        ),
        static=True,
    )
    rr.log(
        f"{path}/body",
        rr.Boxes3D(half_sizes=[[0.07 * scale, 0.045 * scale, 0.025 * scale]], colors=[DARK]),
        static=True,
    )
    rr.log(
        f"{path}/nose",
        rr.Arrows3D(vectors=[[0.3 * scale, 0, 0]], colors=[RED], radii=0.012 * scale),
        static=True,
    )


class Buffer:
    """Columns waiting to be sent: time, pc time, and one list per value."""

    def __init__(self):
        self.t, self.pc, self.values = [], [], []

    def add(self, t_us, pc_ns, value):
        self.t.append(t_us)
        self.pc.append(pc_ns)
        self.values.append(value)


class RerunViewer:
    def __init__(self, config, spawn=True, rrd_path=None, show_gnss=False, wall_offset_ns=0):
        cfg = config["viewer"]
        self.primary = config["estimator"]["primary_imu"]
        self.plot_min_dt = 1e6 / cfg["plot_hz"] if cfg["plot_hz"] else 0  # us, 0 = full rate
        self.g = G  # reference line on the |a| plot; set_g() once the units are known
        self.t0_us = None  # first message time: the "time" timeline starts at 0
        self.pose_min_dt = 1e6 / cfg["transform_hz"] if cfg["transform_hz"] else 0
        self.wall_offset_ns = wall_offset_ns  # wall clock minus monotonic clock
        self.buffers = defaultdict(Buffer)  # entity path -> Buffer
        self.last_sent_t = {}  # entity path -> mcu time of last accepted sample (decimation)
        self.last_q = {}  # state source -> latest q (not tared)
        self.tare_ref = {}  # state source -> q at the moment of tare
        self.diag_ids = set()
        self.trajectory = []
        self.trajectory_sent = 0
        self.send_seconds = 0.0  # time spent in rerun calls, for throughput stats

        rr.init("imuview")
        sinks = []
        if spawn:
            rr.spawn(connect=False)
            sinks.append(rr.GrpcSink())
        if rrd_path:
            sinks.append(rr.FileSink(str(rrd_path)))
        rr.set_sinks(*sinks)
        rr.send_blueprint(blueprint(show_gnss))

        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_DOWN, static=True)
        rr.log(
            "world/axes",
            rr.Arrows3D(
                vectors=[[0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5]],
                colors=XYZ_COLORS,
                labels=["N", "E", "D"],
            ),
            static=True,
        )
        rr.log("world/vehicle", rr.TransformAxes3D(0.2), static=True)
        if cfg.get("model"):
            rr.log("world/vehicle/model", rr.Asset3D(path=cfg["model"]), static=True)
        else:
            log_quad_model("world/vehicle/model")
        rr.log("plots/attitude", rr.SeriesLines(names=["roll", "pitch", "yaw"]), static=True)

    def attach(self, router):
        router.subscribe(ImuSample, self.on_imu)
        router.subscribe(State, self.on_state)
        router.subscribe(GnssFix, self.on_gnss)
        router.subscribe(Event, self.on_event)
        router.subscribe(Command, self.on_command)

    # ----- incoming data -----

    def set_g(self, g):
        self.g = g

    def _t(self, t_us) -> int:
        if self.t0_us is None:
            self.t0_us = t_us
        return t_us - self.t0_us

    def _add(self, path, t_us, pc_ns, value, min_dt=0.0):
        if min_dt and t_us - self.last_sent_t.get(path, -math.inf) < min_dt:
            return
        self.last_sent_t[path] = t_us
        self.buffers[path].add(t_us, pc_ns, value)

    def _series(self, path, names, colors=None):
        if path not in self.last_sent_t:
            rr.log(path, rr.SeriesLines(names=names, colors=colors), static=True)

    def on_imu(self, s: ImuSample):
        t, pc = self._t(s.t_us), s.pc_rx_time_ns
        for kind, v in (("gyro", s.gyro), ("accel", s.accel)):
            path = f"plots/{kind}/imu{s.imu_id}"
            self._series(path, ["x", "y", "z"], XYZ_COLORS)
            self._add(path, t, pc, v, self.plot_min_dt)
        path = f"plots/accel_norm/imu{s.imu_id}"
        self._series(path, ["|a| raw", "|a| cal", "g"], [GRAY, RED, DARK])
        cal = math.dist(s.accel_cal, (0, 0, 0)) if s.accel_cal is not None else math.nan
        self._add(path, t, pc, [math.dist(s.accel, (0, 0, 0)), cal, self.g], self.plot_min_dt)
        if s.euler_deg is not None and s.imu_id == self.primary:
            self._series("plots/device_euler", ["roll", "pitch", "yaw"], XYZ_COLORS)
            self._add("plots/device_euler", t, pc, s.euler_deg, self.plot_min_dt)
        if s.status is not None and s.imu_id == self.primary:
            self._series("plots/status", ["selftest", "filter_valid", "gnss_fix"], XYZ_COLORS)
            bits = [(s.status >> b & 1) + 0.02 * b for b in range(3)]  # offset so lines don't hide
            self._add("plots/status", t, pc, bits, self.plot_min_dt)

    def on_state(self, st: State):
        self.last_q[st.source] = st.q
        q = tare(self.tare_ref[st.source], st.q) if st.source in self.tare_ref else st.q
        t, pc = self._t(st.t_us), st.pc_rx_time_ns
        pos = st.pos_ned or (0.0, 0.0, 0.0)
        if st.source == "vehicle":
            self._add("world/vehicle", t, pc, (to_rerun_quat(q), pos), self.pose_min_dt)
            rpy = [math.degrees(a) for a in quat_to_euler(q)]
            self._add("plots/attitude", t, pc, rpy, self.plot_min_dt)
            if st.pos_ned is not None:
                self.trajectory.append(st.pos_ned)
            return
        k = int(st.source.removeprefix("imu"))
        if k not in self.diag_ids:
            self.diag_ids.add(k)
            log_quad_model(f"world/diag/imu{k}/model", scale=0.4)
        # follows the vehicle position (later, with GPS), never its rotation
        vehicle_pos = np.array(self.trajectory[-1]) if self.trajectory else np.zeros(3)
        offset = vehicle_pos + [-0.6, DIAG_SPACING * (k - 1), 0.0]
        self._add(f"world/diag/imu{k}", t, pc, (to_rerun_quat(q), offset), self.pose_min_dt)
        primary = self.last_q.get(f"imu{self.primary}")
        if k != self.primary and primary is not None:
            deg = math.degrees(angle_between(st.q, primary))
            self._add(f"plots/disagreement/imu{k}", t, pc, [deg], self.plot_min_dt)

    def on_gnss(self, fix: GnssFix):
        if fix.fix_type < 2:  # no fix: positions are placeholders (~29.9999 deg), never plot
            return
        t, pc = self._t(fix.t_us), fix.pc_rx_time_ns
        if fix.num_sv is not None:
            self._add("plots/gnss/num_sv", t, pc, [fix.num_sv])
        self._add("plots/gnss/h_acc_m", t, pc, [fix.h_acc_m or 0.0])

    def on_command(self, c: Command):
        if c.kind == "tare":
            self.tare_ref = dict(self.last_q)

    def on_event(self, ev: Event):
        rr.set_time("time", duration=(ev.t_us - (self.t0_us or ev.t_us)) * 1e-6)
        rr.set_time(
            "pc_time", timestamp=np.datetime64(ev.pc_rx_time_ns + self.wall_offset_ns, "ns")
        )
        rr.log("events", rr.TextLog(ev.text, level=ev.level.upper()))

    # ----- sending -----

    def flush(self):
        start = time.perf_counter()
        for path, b in self.buffers.items():
            if not b.t:
                continue
            index = [
                rr.TimeColumn("time", duration=np.array(b.t) * 1e-6),
                rr.TimeColumn(
                    "pc_time",
                    timestamp=np.array(b.pc, dtype=np.int64).astype("datetime64[ns]")
                    + np.timedelta64(self.wall_offset_ns, "ns"),
                ),
            ]
            if path.startswith("world/"):
                quats = np.array([v[0] for v in b.values])
                trans = np.array([v[1] for v in b.values])
                columns = rr.Transform3D.columns(quaternion=quats, translation=trans)
            else:
                values = np.array(b.values, dtype=float)
                columns = rr.Scalars.columns(scalars=values.ravel()).partition(
                    np.full(len(values), values.shape[1])
                )
            rr.send_columns(path, indexes=index, columns=columns)
            self.buffers[path] = Buffer()
        if len(self.trajectory) > self.trajectory_sent:
            rr.log("world/trajectory", rr.LineStrips3D([self.trajectory[-10_000:]], radii=0.01))
            self.trajectory_sent = len(self.trajectory)
        self.send_seconds += time.perf_counter() - start

    def close(self):
        self.flush()
        rr.disconnect()
