"""MultiPipeline: several sensor modules at once.

One source delivers data of many modules (UdpSource from the master ESP, MultiSimSource, or a
replayed session). Every Chunk / message carries a module_id. MultiPipeline keeps one ordinary
Pipeline per module_id, created the moment the id first appears, so each module has its own
Demux, parser, estimator and calibration state. All of them publish to ONE shared Router and
ONE Recorder: the session is a single folder in which every row has a module_id column and
raw.bin records carry the module id too.

Listeners (GUI, logger) subscribe once to `router` and see all modules.
"""

import queue
import time
from datetime import datetime

from .calibration import Calibration
from .logger import Recorder, git_commit, new_session_dir
from .messages import Event
from .pipeline import CONVENTIONS, Pipeline, Router
from .sources import Chunk


def module_name(config: dict, module_id: int) -> str:
    """Human name of a module from [modules.names] in the config, default "Module N"."""
    names = config.get("modules", {}).get("names", {})
    return names.get(str(module_id)) or f"Module {module_id}"


class MultiPipeline:
    def __init__(
        self,
        source,
        config,
        record=True,
        session_dir=None,
        calibration: Calibration | None = None,
        calibrations: dict[int, Calibration] | None = None,
        label: str | None = None,
    ):
        self.source = source
        self.config = config
        self.label = label
        self.default_calibration = calibration
        self.calibrations = calibrations or {}  # module_id -> Calibration (overrides the default)
        self.router = Router()
        self.pipelines: dict[int, Pipeline] = {}
        self.on_new_module = []  # callbacks fn(module_id)
        self.commands = queue.Queue()  # from other threads; handled in poll_keys (pipeline thread)
        self.running = True
        self.closed = False
        self.recorder = None
        self.last_tick = time.monotonic()
        if record:
            self.start_recording(session_dir)

    # ----- modules -----

    def pipeline(self, module_id: int) -> Pipeline:
        """The Pipeline of a module; created on first use."""
        if module_id not in self.pipelines:
            self.pipelines[module_id] = Pipeline(
                self.source,
                self.config,
                record=False,
                calibration=self.calibrations.get(module_id, self.default_calibration),
                module_id=module_id,
                router=self.router,
                owns_source=False,
            )
            self.event(f"{module_name(self.config, module_id)} (id {module_id}) appeared")
            for fn in self.on_new_module:
                fn(module_id)
        return self.pipelines[module_id]

    def event(self, text, level="info"):
        print(f"[{level}] {text}", flush=True)
        self.router.publish(
            Event(pc_rx_time_ns=time.monotonic_ns(), time_source="host", level=level, text=text)
        )

    # ----- recording -----

    def start_recording(self, session_dir=None):
        session_dir = session_dir or new_session_dir(self.config["log_dir"], self.label)
        cal = self.default_calibration
        meta = {
            "label": self.label,
            "multi_module": True,
            "start_time": datetime.now().isoformat(),
            "wall_minus_monotonic_ns": time.time_ns() - time.monotonic_ns(),
            "source": self.source.describe(),
            "module_names": {
                str(k): module_name(self.config, k) for k in self.config["modules"]["names"]
            },
            "calibration": cal.describe() if cal else None,
            "config": self.config,
            "conventions": CONVENTIONS,
            "git_commit": git_commit(),
        }
        self.recorder = Recorder(session_dir, meta)
        self.recorder.attach(self.router)
        self.event(f"recording to {session_dir}")

    def stop_recording(self):
        if self.recorder:
            self.router.unsubscribe(self.recorder)
            self.recorder.close(
                {
                    "stop_time": datetime.now().isoformat(),
                    "modules": sorted(self.pipelines),
                    "stats": self.stats(),
                }
            )
            self.event(f"recording stopped: {self.recorder.dir}")
            self.recorder = None

    # ----- main loop -----

    def run(self):
        try:
            while self.running:
                if not self.step(0.05):
                    break
        finally:
            self.close()

    def step(self, timeout=0.05) -> bool:
        """One read from the source. False when the source has nothing more."""
        items = self.source.read(timeout)
        if items is None:
            return False
        for item in items:
            if isinstance(item, Chunk) or hasattr(item, "module_id"):
                self.pipeline(item.module_id).handle(item)
        self.poll_keys()
        self.tick()
        return True

    def stop(self):
        self.running = False

    # ----- commands: safe to call from any thread, carried out by the pipeline thread -----

    def command(self, cmd):
        """ "q" quit, "r" toggle recording, "rec_on" / "rec_off", "c" / "t" (gyro calibrate, tare:
        to every module), or ("calibration", Calibration) to use a new host calibration."""
        self.commands.put(cmd)

    def poll_keys(self):
        while not self.commands.empty():
            cmd = self.commands.get()
            if cmd == "q":
                self.running = False
            elif cmd == "r":
                self.toggle_recording()
            elif cmd == "rec_on" and not self.recorder:
                self.start_recording()
            elif cmd == "rec_off":
                self.stop_recording()
            elif isinstance(cmd, tuple) and cmd[0] == "calibration":
                self.set_calibration(cmd[1])
            elif cmd in ("c", "t"):
                for p in self.pipelines.values():
                    p.command(cmd)
        for p in self.pipelines.values():
            p.poll_keys()

    def toggle_recording(self):
        if self.recorder:
            self.stop_recording()
        else:
            self.start_recording()

    def set_calibration(self, cal: Calibration | None):
        """New calibration for every module, from now on (raw values stay raw)."""
        self.default_calibration = cal
        self.calibrations = {}
        for p in self.pipelines.values():
            p.set_calibration(cal)
        self.event(f"calibration {'set' if cal else 'removed'}" + (f": {cal.path}" if cal else ""))

    def tick(self):
        for p in self.pipelines.values():
            p.tick()
        now = time.monotonic()
        if self.recorder and now - self.last_tick >= 1.0:
            self.recorder.flush()
        if now - self.last_tick >= 1.0:
            self.last_tick = now

    def stats(self) -> dict:
        out = {"modules": {str(k): p.stats() for k, p in sorted(self.pipelines.items())}}
        if hasattr(self.source, "stats"):
            out["source"] = self.source.stats()
        return out

    def close(self):
        if self.closed:
            return
        self.closed = True
        for p in self.pipelines.values():
            p.close()
        self.stop_recording()
        self.source.close()
