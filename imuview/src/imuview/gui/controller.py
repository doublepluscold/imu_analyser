"""The application state the windows are built from. Plain Python, tested without a window.

Modes:   live (data arriving now), calibration (live data, wizard on top), analysis (a session
         file and a time slider).
Layouts: single (active module, big), grid (one 3D view per module), scene (all in one space).
Views:   3d (model + plots) or raw (MTData2 table and hex).
"""

import tomllib
from dataclasses import dataclass
from pathlib import Path

from ..calibration import Calibration
from ..messages import ImuSample
from ..multi import module_name
from ..pipeline import load_config
from .models import ModelRegistry
from .sessions import LiveSession, SessionPlayer, make_source
from .store import Pose, Store
from .wizard import CalibWizard

MODES = ("live", "calibration", "analysis")
LAYOUTS = ("single", "grid", "scene")
VIEWS = ("3d", "plots", "both", "raw")  # model, plots, model + plots, MTData table and hex
CONN_STATES = ("disconnected", "connected", "running")
MAX_MODULES = 8  # the grid and the thumbnail column are laid out for this many


@dataclass
class Options:
    use_gps: bool = True  # show movement from GPS; off = orientation + force arrow only
    subtract_gravity: bool = True  # force arrow: free acceleration instead of specific force
    trail_seconds: float = 2.0
    record: bool = True
    layout: str = "single"
    view: str = "both"
    window_s: float = 10.0  # width of the plots
    show_trail: bool = True


class AppController:
    def __init__(self, config: dict | None = None, models_dir="models", calib_path=None):
        self.config = config or load_config()
        self.registry = ModelRegistry(models_dir)
        self.library = None  # ModelLibrary (user models in the app data folder), set by the app
        self.options = Options()
        self.mode = "live"
        self.active: int | None = None
        self.default_model = "quad"
        self.module_models: dict[int, str] = {}
        self.live_store = Store()
        self.live: LiveSession | None = None
        self.player: SessionPlayer | None = None
        self.wizard: CalibWizard | None = None
        self.calibration: Calibration | None = None
        self.calib_path = calib_path or self.config.get("calib_file") or None
        if self.calib_path and Path(self.calib_path).exists():
            self.calibration = Calibration.load(self.calib_path)
        self.message = ""
        self.running = False  # "Старт" pressed: the log is written if options.record

    def set_registry(self, registry):
        """New set of models (after the manager dialog). Choices that no longer exist fall back."""
        self.registry = registry
        keys = set(registry.keys())
        self.module_models = {m: k for m, k in self.module_models.items() if k in keys}
        if self.default_model not in keys:
            self.default_model = "quad"

    # ----- which store, which time -----

    @property
    def store(self) -> Store:
        if self.mode == "analysis" and self.player is not None:
            return self.player.store
        return self.live_store

    def view_time(self) -> float | None:
        """Time to show: None = newest (live), else the slider time (analysis)."""
        if self.mode == "analysis" and self.player is not None:
            return self.player.t
        return None

    # ----- mode, connection -----

    def set_mode(self, mode: str):
        if mode not in MODES:
            raise ValueError(mode)
        if mode == "analysis" and self.player is None:
            self.message = "open a session first"
        self.mode = mode
        if mode != "calibration":
            self.wizard = None
        self._fix_active()

    def start_live(self, kind: str, **params):
        """Connect (udp / serial / sim). The old connection, if any, is stopped first."""
        self.stop_live()
        self.live_store = Store()
        self.active = None
        source = make_source(kind, self.config, **params)  # may raise OSError
        # connected = data is shown; nothing is written until start_run()
        self.live = LiveSession(source, self.config, self.live_store, record=False,
                                calibration=self.calibration)  # fmt: skip
        self.live.start()
        self.running = False
        self.message = f"listening ({kind})"

    @property
    def conn_state(self) -> str:
        """disconnected -> connected -> running (the state machine the toolbar follows)."""
        if self.live is None:
            return "disconnected"
        return "running" if self.running else "connected"

    @property
    def recording(self) -> bool:
        return self.live is not None and self.live.recording

    def start_run(self):
        """ "Старт": the log starts if "Писати лог" is on."""
        if self.live is None:
            return
        self.running = True
        if self.options.record:
            self.live.set_recording(True)

    def stop_run(self):
        """ "Стоп": the log is closed, the connection stays."""
        if self.live is None:
            return
        self.running = False
        self.live.set_recording(False)

    def stop_live(self):
        self.running = False
        if self.live is not None:
            saved = self.live.session_dir
            self.live.stop()
            self.message = "stopped" + (f", saved {saved}" if saved else "")
            self.live = None

    def set_record(self, on: bool):
        """The log switch. While running it takes effect at once (the GUI locks it then)."""
        self.options.record = on
        if self.live and self.running:
            self.live.set_recording(on)

    def open_session(self, path, background=True):
        self.player = SessionPlayer(path, self.config, self.calibration)
        self.player.load(background=background)
        self.mode = "analysis"
        self.active = None
        self.message = f"loading {path}"

    def poll(self) -> None:
        """Called by the GUI timer: keeps `active` valid, reports finished loads and errors."""
        if self.live and self.live.error:
            self.message = f"connection error: {self.live.error}"
        if self.player:
            if self.player.error:
                self.message = f"cannot open session: {self.player.error}"
            elif self.player.loaded and self.message.startswith("loading"):
                self.message = f"loaded {self.player.session.name}"
        self._fix_active()

    # ----- modules -----

    def module_ids(self) -> list[int]:
        return self.store.module_ids()[:MAX_MODULES]

    def name(self, module_id: int) -> str:
        return module_name(self.config, module_id)

    def _fix_active(self):
        ids = self.module_ids()
        if ids and self.active not in ids:
            self.active = ids[0]
        if not ids:
            self.active = None

    def set_active(self, module_id: int):
        if module_id in self.module_ids():
            self.active = module_id

    def set_model(self, key: str, module_id: int | None = None, everywhere=False):
        """Model of one module (default: the active one); everywhere = all modules and new ones."""
        if key not in self.registry.keys():
            raise KeyError(key)
        if everywhere:
            self.default_model = key
            self.module_models = {}
        else:
            target = module_id if module_id is not None else self.active
            if target is None:
                self.default_model = key
            else:
                self.module_models[target] = key

    def model_of(self, module_id: int) -> str:
        return self.module_models.get(module_id, self.default_model)

    def pose(self, module_id: int) -> Pose | None:
        return self.store.pose_at(module_id, self.view_time(), use_gps=self.options.use_gps)

    def alive(self, module_id: int) -> bool:
        return self.store.alive(module_id, self.view_time())

    def shown_modules(self) -> list[int]:
        """Modules drawn in the big area: the active one, or all of them."""
        if self.options.layout == "single":
            return [self.active] if self.active is not None else []
        return self.module_ids()

    # ----- options -----

    def set_layout(self, layout: str):
        if layout not in LAYOUTS:
            raise ValueError(layout)
        self.options.layout = layout

    def set_view(self, view: str):
        if view not in VIEWS:
            raise ValueError(view)
        self.options.view = view

    # ----- calibration -----

    def start_wizard(self, **kw) -> CalibWizard:
        if self.active is None:
            raise RuntimeError("no module is sending data")
        imu = self.config["imu"]
        alignment = {**imu.get("default", {}), **imu.get("0", {})}.get("board_alignment_deg",
                                                                         [0, 0, 0])  # fmt: skip
        self.wizard = CalibWizard(self.active, alignment, **kw)
        if self.live:
            self.live.pipeline.router.subscribe(ImuSample, self.wizard.feed)
        self.mode = "calibration"
        return self.wizard

    def finish_wizard(self, path="calib/imu_calib.json"):
        """Save the wizard's result and use it for the live modules from now on."""
        cal, report = self.wizard.save(path)
        self.calibration = cal
        self.calib_path = str(path)
        if self.live:
            self.live.set_calibration(cal)
        self.message = f"calibration saved to {path}, report {report}"
        return cal, report

    def end_wizard(self):
        if self.wizard and self.live:
            self.live.pipeline.router.unsubscribe(self.wizard)
        self.wizard = None
        self.mode = "live"

    # ----- names from the config file -----

    def load_names(self, path):
        data = tomllib.loads(Path(path).read_text(encoding="utf-8"))
        self.config.setdefault("modules", {}).setdefault("names", {}).update(
            data.get("modules", {}).get("names", {})
        )

    def shutdown(self):
        self.stop_live()
