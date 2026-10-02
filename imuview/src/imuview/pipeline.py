"""Source -> Demux -> Parser -> Calibration -> BoardAlignment -> Router -> Estimators -> sinks.

Live, sim, replay and import all use this same Pipeline; only the Source differs.
Raw sensor values are never modified: calibration and alignment fill separate fields.
"""

import copy
import queue
import time
import tomllib
from collections import Counter, defaultdict
from datetime import datetime

import numpy as np

from . import protocol_mtdata2  # noqa: F401  (registers the "mtdata2" parser)
from .calibration import Calibration
from .frames import board_rotation, quat_from_matrix, quat_mul
from .fusion import AttitudeEstimator, GyroCalibrator
from .logger import Recorder, git_commit, new_session_dir
from .messages import STREAMS, Command, Event, ImuSample, Message, times_of
from .protocol import PARSERS, build_demux
from .sources import Chunk

DEFAULT_CONFIG = {
    "parser": "mtdata2",
    "serial": {"port": "/dev/ttyUSB0", "baud": 115200},
    "calib_file": "",  # host calibration JSON (calib/imu_calib.json); "" = none
    "extra_parsers": [],  # e.g. a GNSS parser later
    "log_dir": "logs",
    "imu": {
        # per-IMU sections "0", "1", ... override "default"
        "default": {
            "accel_lsb_g": 1 / 2048,  # +-16 g range, int16
            "gyro_lsb_dps": 1 / 16.4,  # +-2000 deg/s range, int16
            # roll, pitch, yaw: sensor -> body. 180 roll: the MTData2 sensor reads +g on z when
            # level (z up), body FRD wants -g. Provisional until `imu calib axes` is confirmed.
            "board_alignment_deg": [180.0, 0.0, 180.0],
        },
    },
    "estimator": {"algo": "mahony", "primary_imu": 0, "kp": 1.0, "ki": 0.05, "beta": 0.05},
    "calibration": {"seconds": 3.0, "max_gyro_dps": 5.0, "max_accel_std": 0.2},
    "modules": {"names": {}},  # module_id (as text) -> name shown in the GUI, e.g. {"1": "Drone 1"}
    "network": {"host": "0.0.0.0", "port": 5005},  # UdpSource, listen only
    "master": {"port": "/dev/ttyACM0"},  # MasterSerialSource: the master ESP over USB, listen only
    "viewer": {"plot_hz": 0, "transform_hz": 0, "model": ""},  # 0 = full rate
}

CONVENTIONS = {
    "body": "FRD",
    "world": "NED (local tangent plane)",
    "quaternion": "(w, x, y, z), Hamilton, body -> world",
    "euler": "ZYX (yaw, pitch, roll)",
    "accel": "specific force, still and level = (0, 0, -g)",
    "imu.parquet": (
        "accel/gyro: exactly as sent, sensor frame, never modified; "
        "*_cal: host calibration applied (sensor frame); "
        "*_body: calibrated (if any) then board alignment + level trim; gyro-key bias NOT removed"
    ),
    "time": "t = device time if the stream has one, else reconstructed host time (time_source)",
}


def load_config(path=None) -> dict:
    config = copy.deepcopy(DEFAULT_CONFIG)
    if path:
        with open(path, "rb") as f:
            _merge(config, tomllib.load(f))
    return config


def _merge(base: dict, extra: dict):
    for k, v in extra.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge(base[k], v)
        else:
            base[k] = v


class Router:
    """Calls every function subscribed to the type of the object (or a parent type)."""

    def __init__(self):
        self.subscribers = defaultdict(list)

    def subscribe(self, cls, fn):
        self.subscribers[cls].append(fn)

    def unsubscribe(self, owner):
        for fns in self.subscribers.values():
            fns[:] = [f for f in fns if getattr(f, "__self__", None) is not owner]

    def publish(self, obj):
        for cls in type(obj).__mro__:
            for fn in self.subscribers.get(cls, ()):
                fn(obj)


class Pipeline:
    def __init__(
        self,
        source,
        config,
        viewer=None,
        record=True,
        session_dir=None,
        calibration: Calibration | None = None,
        label: str | None = None,
        module_id: int = 0,
        router: "Router | None" = None,
        owns_source: bool = True,
    ):
        """module_id / router / owns_source are for MultiPipeline: one Pipeline per module, all
        publishing to one shared router, and the source is closed by the MultiPipeline."""
        self.module_id = module_id
        self.owns_source = owns_source
        self.source = source
        self.config = config
        self.label = label
        if calibration is None and config.get("calib_file"):
            calibration = Calibration.load(config["calib_file"])
        self.calibration = calibration
        # a simulator knows its own wire format; otherwise the config decides
        parser_name = getattr(source, "protocol", None) or config["parser"]
        self.parser = PARSERS[parser_name](config["imu"])
        extra = [PARSERS[name](config["imu"]) for name in config["extra_parsers"]]
        baud = getattr(source, "baud", None) or config["serial"]["baud"]
        for p in (self.parser, *extra):
            if hasattr(p, "baud"):
                p.baud = baud
            if hasattr(p, "time_source"):
                p.time_source = getattr(source, "time_source", "host")
        self.parsers = {p.framer.name: p for p in [self.parser, *extra]}  # framer -> parser
        self.demux = build_demux(self.parsers.values())
        self.router = router if router is not None else Router()
        est = config["estimator"]
        self.primary = est["primary_imu"]
        self.vehicle = self.make_estimator(self.primary, "vehicle")
        self.diag = {}  # imu_id -> AttitudeEstimator (one per IMU, for diagnostics)
        self.alignment = {}  # imu_id -> sensor->body rotation matrix
        self.gyro_bias = {}  # imu_id -> bias from the last calibration
        self.calibrators = None  # imu_id -> GyroCalibrator while calibrating
        self.commands = queue.Queue()  # keys from the keyboard thread
        self.counts = Counter()  # message type -> count
        self.time_span = {}  # message type -> [first, last] t_us
        self.time_sources = Counter()
        self.seen = set()  # (framer, kind) already announced
        self.injected = set()  # streams that did not come from raw bytes
        self.last_times = {
            "pc_rx_time_ns": time.monotonic_ns(),
            "mcu_time_us": 0,
            "module_id": module_id,
        }
        self.prev_errors = Counter()
        self.last_tick = time.monotonic()
        self.last_flush = time.monotonic()
        self.running = True
        self.viewer = viewer
        if viewer:
            viewer.attach(self.router)
        self.recorder = None
        if record:
            self.start_recording(session_dir)

    def make_estimator(self, imu_id, source):
        e = self.config["estimator"]
        est = AttitudeEstimator(e["algo"], imu_id, source, e["kp"], e["ki"], e["beta"])
        if source == "vehicle":  # one gap report is enough
            est.clock.on_gap = lambda raw, med: self.event(
                "warn", f"time gap {raw * 1e3:.1f} ms (median interval {med * 1e3:.1f} ms)"
            )
        return est

    # ----- recording -----

    def start_recording(self, session_dir=None):
        session_dir = session_dir or new_session_dir(self.config["log_dir"], self.label)
        meta = {
            "label": self.label,
            "start_time": datetime.now().isoformat(),
            "wall_minus_monotonic_ns": time.time_ns() - time.monotonic_ns(),
            "source": self.source.describe(),
            "parsers": {p.name: p.version for p in self.parsers.values()},
            "estimator": self.vehicle.params(),
            "calibration": self.calibration.describe() if self.calibration else None,
            "config": self.config,
            "conventions": CONVENTIONS,
            "ned_origin": None,  # set by the first good GNSS fix, later
            "git_commit": git_commit(),
        }
        self.recorder = Recorder(session_dir, meta)
        self.recorder.attach(self.router)
        self.event("info", f"recording to {session_dir}")

    def stop_recording(self):
        if self.recorder:
            self.router.unsubscribe(self.recorder)
            self.recorder.close(
                {
                    "stop_time": datetime.now().isoformat(),
                    "stats": self.stats(),
                    "gyro_bias_rad_s": {k: v.tolist() for k, v in self.gyro_bias.items()},
                    "injected_streams": sorted(self.injected),
                }
            )
            self.event("info", f"recording stopped: {self.recorder.dir}")
            self.recorder = None

    # ----- main loop -----

    def run(self):
        try:
            while self.running:
                items = self.source.read(0.05)
                if items is None:
                    break
                for item in items:
                    self.handle(item)
                self.poll_keys()
                self.tick()
        finally:
            self.close()

    def handle(self, item):
        if isinstance(item, Chunk):
            self.router.publish(item)  # raw.bin
            for frame in self.demux.feed(item.data, item.pc_rx_time_ns):
                if (frame.framer, frame.kind) not in self.seen:
                    self.seen.add((frame.framer, frame.kind))
                    self.event("info", f"first {frame.framer} frame of type {frame.kind}")
                parser = self.parsers.get(frame.framer)
                if parser:
                    for msg in parser.parse(frame):
                        msg.module_id = self.module_id
                        self.on_message(msg)
                    for level, text in getattr(parser, "drain_events", list)():
                        self.event(level, text)
        else:  # injected message (fake GNSS, replayed command)
            if type(item) in STREAMS:
                self.injected.add(STREAMS[type(item)])
            self.on_message(item)

    def on_message(self, msg: Message):
        if isinstance(msg, ImuSample):
            msg = self.align(self.apply_calibration(msg))
        name = type(msg).__name__
        self.counts[name] += 1
        self.time_sources[msg.time_source] += 1
        self.time_span.setdefault(name, [msg.t_us, 0])[1] = msg.t_us
        if not isinstance(msg, Command):
            self.last_times = times_of(msg)
        self.router.publish(msg)
        if isinstance(msg, Command):
            self.on_command(msg)
        if isinstance(msg, ImuSample):
            if msg.imu_id not in self.diag:
                self.diag[msg.imu_id] = self.make_estimator(msg.imu_id, f"imu{msg.imu_id}")
            if self.calibrators is not None:
                self.calibrate(msg)
        for est in (self.vehicle, *self.diag.values()):
            state = est.process(msg)
            if state:
                self.router.publish(state)

    def set_calibration(self, cal: Calibration | None):
        """Another calibration for the samples that follow (the level trim is rebuilt too)."""
        self.calibration = cal
        self.alignment.clear()

    def apply_calibration(self, s: ImuSample) -> ImuSample:
        """Host calibration in the native sensor frame; fills *_cal, raw fields untouched."""
        cal = self.calibration
        if cal is not None and s.imu_id == self.primary:
            s.accel_cal = tuple(cal.apply_accel(s.accel).tolist())
            s.gyro_cal = tuple(cal.apply_gyro(s.gyro).tolist())
        return s

    def align(self, s: ImuSample) -> ImuSample:
        """Sensor frame -> body frame (board alignment, then level trim); fills *_body."""
        if s.imu_id not in self.alignment:
            imu = self.config["imu"]
            cfg = {**imu.get("default", {}), **imu.get(str(s.imu_id), {})}
            r = board_rotation(*cfg.get("board_alignment_deg", [0, 0, 0]))
            if self.calibration is not None and s.imu_id == self.primary:
                trim_roll, trim_pitch = self.calibration.level_trim_deg
                r = board_rotation(trim_roll, trim_pitch, 0.0) @ r
            self.alignment[s.imu_id] = r
        r = self.alignment[s.imu_id]
        accel = s.accel_cal if s.accel_cal is not None else s.accel
        gyro = s.gyro_cal if s.gyro_cal is not None else s.gyro
        s.accel_body = tuple((r @ accel).tolist())
        s.gyro_body = tuple((r @ gyro).tolist())
        if s.quat is not None:  # device gives sensor -> world; we want body -> world
            s.quat_body = tuple(quat_mul(s.quat, quat_from_matrix(r.T)).tolist())
        return s

    # ----- commands and calibration -----

    def command(self, key: str):
        """Called from the keyboard thread."""
        self.commands.put(key)

    def poll_keys(self):
        while not self.commands.empty():
            key = self.commands.get()
            if key == "q":
                self.running = False
            elif key == "r":
                if self.recorder:
                    self.stop_recording()
                else:
                    self.start_recording()
            elif key in ("c", "t"):
                kind = "calibrate" if key == "c" else "tare"
                self.on_message(Command(**self.last_times, kind=kind))

    def on_command(self, c: Command):
        if c.kind == "calibrate":
            self.calibrators = {}
            secs = self.config["calibration"]["seconds"]
            self.event("info", f"gyro calibration started, keep the sensor still for {secs} s")
        elif c.kind == "tare":
            self.event("info", "tare: current orientation is the new reference")

    def calibrate(self, s: ImuSample):
        cal = self.calibrators.setdefault(s.imu_id, GyroCalibrator(**self.config["calibration"]))
        if cal.bias is not None:
            return
        result = cal.feed(s)
        if result == "moved":
            self.calibrators = None
            self.event("warn", f"gyro calibration aborted: IMU {s.imu_id} moved")
        elif result == "done":
            self.gyro_bias[s.imu_id] = cal.bias
            for est in (self.vehicle, self.diag[s.imu_id]):
                if est.imu_id == s.imu_id:
                    est.gyro_bias = cal.bias
            dps = ", ".join(f"{v:+.3f}" for v in np.degrees(cal.bias))
            self.event("info", f"IMU {s.imu_id} gyro bias [deg/s]: {dps}")
            if all(c.bias is not None for c in self.calibrators.values()):
                self.calibrators = None

    # ----- events, stats, periodic work -----

    def event(self, level, text):
        ev = Event(**self.last_times, level=level, text=text)
        print(f"[{level}] {text}", flush=True)
        self.router.publish(ev)

    def tick(self):
        now = time.monotonic()
        if self.viewer and now - self.last_flush > 0.05:
            self.viewer.flush()
            self.last_flush = now
        if now - self.last_tick < 1.0:
            return
        self.last_tick = now
        if self.recorder:
            self.recorder.flush()
        errors = Counter({f"{k} checksum errors": v for k, v in self.demux.bad_checksum.items()})
        errors["seq gaps"] = self.parser.seq_gaps
        errors["frames lost"] = self.parser.frames_lost
        errors["unknown message types"] = sum(getattr(self.parser, "unknown_types", {}).values())
        for name, total in errors.items():
            new = total - self.prev_errors[name]
            if new > 0:
                self.event("warn", f"{name}: +{new} (total {total})")
        self.prev_errors = errors

    def stats(self) -> dict:
        messages = {}
        for name, n in self.counts.items():
            first, last = self.time_span[name]
            rate = (n - 1) / ((last - first) * 1e-6) if last > first else 0.0
            messages[name] = {"count": n, "rate_hz": round(rate, 2)}
        total = self.demux.total_bytes
        return {
            "messages": messages,
            "frames": {f"{f} {k}": n for (f, k), n in sorted(self.demux.frames.items())},
            "checksum_errors": dict(self.demux.bad_checksum),
            "parser": {p.name: p.stats() for p in self.parsers.values()},
            "time_sources": dict(self.time_sources),
            "bytes_total": total,
            "bytes_unclaimed": self.demux.unclaimed_bytes,
            "unclaimed_percent": round(100 * self.demux.unclaimed_bytes / total, 3) if total else 0,
        }

    def close(self):
        if getattr(self, "closed", False):
            return
        self.closed = True
        self.stop_recording()
        if self.owns_source:
            self.source.close()
        if self.viewer:
            self.viewer.close()
