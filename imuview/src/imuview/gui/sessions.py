"""Live connection and analysis loading. Threads, but no Qt."""

import threading
import time
from pathlib import Path

from ..calibration import Calibration
from ..multi import MultiPipeline
from ..sources import MasterSerialSource, MultiSimSource, SerialSource, UdpSource, session_source
from .store import Store

SOURCE_KINDS = ("master", "udp", "serial", "sim")


def make_source(kind: str, config: dict, *, host=None, port=None, serial_port=None, baud=None,
                sim_modules=3, sim_gps=True):  # fmt: skip
    """The ways to get data. All of them only listen.

    master: the master ESP over USB (modules reach it by Wi-Fi / ESP-NOW); udp: the same master
    over Ethernet; serial: ONE module straight from its UART (debugging); sim: simulated modules.
    """
    if kind == "udp":
        net = config["network"]
        return UdpSource(host or net["host"], int(port or net["port"]))
    if kind == "master":
        return MasterSerialSource(serial_port or config["master"]["port"])
    if kind == "serial":
        ser = config["serial"]
        return SerialSource(serial_port or ser["port"], int(baud or ser["baud"]))
    if kind == "sim":
        return MultiSimSource(
            n_modules=sim_modules,
            rate_hz=80.0,
            realtime=True,
            imu_config=config["imu"],
            gpssol_every=15,
            gps_origin=(50.45, 30.52) if sim_gps else None,
        )
    raise ValueError(f"unknown source {kind!r}")


class LiveSession:
    """A MultiPipeline running in its own thread, feeding a Store. Recording is on by default."""

    def __init__(self, source, config, store: Store, record=True,
                 calibration: Calibration | None = None, label="demo"):  # fmt: skip
        self.store = store
        self.pipeline = MultiPipeline(source, config, record=record, calibration=calibration,
                                      label=label)  # fmt: skip
        store.attach(self.pipeline.router)
        self.error: str | None = None
        self.error_exc: Exception | None = None
        self.thread = threading.Thread(target=self._run, daemon=True, name="pipeline")

    def start(self):
        self.thread.start()

    def _run(self):
        try:
            self.pipeline.run()
        except Exception as e:  # shown in the GUI; the thread must not die silently
            self.error = f"{type(e).__name__}: {e}"
            self.error_exc = e
            self.pipeline.close()

    @property
    def running(self) -> bool:
        return self.thread.is_alive()

    @property
    def recording(self) -> bool:
        return self.pipeline.recorder is not None

    @property
    def session_dir(self) -> Path | None:
        return self.pipeline.recorder.dir if self.pipeline.recorder else None

    def set_recording(self, on: bool):
        self.pipeline.command("rec_on" if on else "rec_off")

    def set_calibration(self, cal: Calibration | None):
        self.pipeline.command(("calibration", cal))

    def stop(self, timeout=3.0):
        self.pipeline.command("q")
        self.thread.join(timeout)


class SessionPlayer:
    """A recorded session, run through the pipeline once into a Store, then played by time.

    Deterministic: the raw bytes (or stored messages) go through the same parser, calibration and
    estimators every time, so two loads give identical data, and a given time always gives the same
    pose. The slider just moves `t`.
    """

    def __init__(self, session_dir, config, calibration: Calibration | None = None):
        self.session = Path(session_dir)
        self.config = config
        self.calibration = calibration
        self.store = Store(keep_s=None)
        self.loaded = False
        self.error: str | None = None
        self.t = 0.0
        self.t_start = 0.0
        self.t_end = 0.0
        self.playing = False
        self.speed = 1.0
        self._thread = None

    def load(self, background=True):
        if background:
            self._thread = threading.Thread(target=self._load, daemon=True, name="load-session")
            self._thread.start()
        else:
            self._load()

    def _load(self):
        try:
            mp = MultiPipeline(session_source(self.session), self.config, record=False,
                               calibration=self.calibration)  # fmt: skip
            self.store.attach(mp.router)
            try:
                while mp.step(0.0):
                    pass
            finally:
                mp.close()
            self.t_start, self.t_end = self.store.span()
            self.t = self.t_start
            self.loaded = True
        except Exception as e:
            self.error = f"{type(e).__name__}: {e}"

    # ----- time control -----

    def seek(self, t: float):
        self.t = min(max(t, self.t_start), self.t_end)

    def play(self):
        if self.t >= self.t_end:
            self.t = self.t_start
        self.playing = True

    def pause(self):
        self.playing = False

    def tick(self, wall_dt: float):
        """Advance by wall_dt seconds of real time (scaled by speed); stops at the end."""
        if not self.playing:
            return
        self.t += wall_dt * self.speed
        if self.t >= self.t_end:
            self.t, self.playing = self.t_end, False

    @property
    def fraction(self) -> float:
        span = self.t_end - self.t_start
        return (self.t - self.t_start) / span if span > 0 else 0.0


def wait_until(predicate, timeout=5.0, step=0.02) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(step)
    return predicate()
