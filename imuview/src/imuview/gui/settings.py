"""What the user set up is kept between runs (QSettings: ini file in the user config folder)."""

from PySide6.QtCore import QSettings

DEFAULTS = {
    "tab": "live",
    "transport": "sim",
    "port/serial": "/dev/ttyUSB0",
    "port/udp": "5005",
    "port/sim": "3",
    "record": True,
    "model": "quad",
    "layout": "single",
    "view": "both",
    "use_gps": True,
    "show_trail": True,
    "trail_seconds": 2.0,
    "subtract_gravity": True,
    "apply_all": False,
    "side_open": True,
    "section/data": True,
    "section/viz": True,
}


class Settings:
    def __init__(self, qs: QSettings | None = None):
        self.qs = qs or QSettings("imuview", "imuview")

    def get(self, key: str):
        default = DEFAULTS.get(key)
        if default is None:
            return self.qs.value(key)
        return self.qs.value(key, default, type=type(default))

    def set(self, key: str, value):
        self.qs.setValue(key, value)

    # window geometry and splitters are binary / lists, handled by the window
    def get_bytes(self, key: str):
        v = self.qs.value(key)
        return v if v else None

    def get_sizes(self, key: str) -> list[int] | None:
        v = self.qs.value(key)
        try:
            sizes = [int(x) for x in v]
            return sizes if sizes and all(s >= 0 for s in sizes) else None
        except (TypeError, ValueError):
            return None

    def sync(self):
        self.qs.sync()

    # ----- controller <-> settings -----

    def load_into(self, c):
        o = c.options
        o.use_gps = self.get("use_gps")
        o.show_trail = self.get("show_trail")
        o.trail_seconds = float(self.get("trail_seconds"))
        o.subtract_gravity = self.get("subtract_gravity")
        o.record = self.get("record")
        layout, view = self.get("layout"), self.get("view")
        from .controller import LAYOUTS, VIEWS

        if layout in LAYOUTS:
            o.layout = layout
        if view in VIEWS:
            o.view = view
        model = self.get("model")
        if model in c.registry.keys():
            c.default_model = model

    def store_from(self, c):
        o = c.options
        for key, value in (
            ("use_gps", o.use_gps), ("show_trail", o.show_trail),
            ("trail_seconds", o.trail_seconds), ("subtract_gravity", o.subtract_gravity),
            ("record", o.record), ("layout", o.layout), ("view", o.view),
            ("model", c.default_model),
        ):  # fmt: skip
            self.set(key, value)
