"""Plots of the active module: accel, gyro, euler (and GPS speed / altitude). Imports Qt."""

import numpy as np
import pyqtgraph as pg

MAX_POINTS = 1200  # per curve; more would not be visible anyway
from . import theme  # noqa: E402
from .theme import T  # noqa: E402

XYZ_COLORS = [theme.rgb(c) for c in theme.XYZ]


def decimate(t: np.ndarray, v: np.ndarray):
    if len(t) <= MAX_POINTS:
        return t, v
    step = int(np.ceil(len(t) / MAX_POINTS))
    return t[::step], v[::step]


def style_plot(p):
    """Plot colours from the theme tokens."""
    p.showGrid(x=True, y=True, alpha=0.18)
    for name in ("left", "bottom"):
        ax = p.getAxis(name)
        ax.setPen(pg.mkPen(T.border))
        ax.setTextPen(pg.mkPen(T.text_muted))
    p.titleLabel.opts["color"] = T.text


class PlotsPanel(pg.GraphicsLayoutWidget):
    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.setBackground(T.bg1)
        self.plots = {}
        self.curves = {}

        def title(text, names):
            dots = " ".join(
                f"<span style='color: {theme.XYZ[i]}'>{n}</span>" for i, n in enumerate(names)
            )
            return f"{text}  {dots}"

        specs = [
            ("accel", title("Прискорення, м/с²", "xyz"), ("x", "y", "z")),
            ("gyro", title("Гіроскоп, °/с", "xyz"), ("x", "y", "z")),
            ("euler", title("Кути (roll pitch yaw), °", "RPY"), ("roll", "pitch", "yaw")),
            ("gps_enu", title("GPS від старту, м", "ENU"), ("E", "N", "U")),
        ]
        for row, (kind, title, names) in enumerate(specs):
            p = self.addPlot(row=row, col=0, title=title)
            style_plot(p)
            p.setMenuEnabled(False)
            p.setMouseEnabled(x=False, y=True)
            self.plots[kind] = p
            self.curves[kind] = [
                p.plot(pen=pg.mkPen(XYZ_COLORS[i], width=1.5), name=n) for i, n in enumerate(names)
            ]
            if kind != "gps_enu":
                p.setXLink(self.plots["accel"]) if kind != "accel" else None

    def refresh(self):
        c = self.controller
        mid = c.active
        t_now = c.view_time()
        win = c.options.window_s
        if mid is None:
            for curves in self.curves.values():
                for cv in curves:
                    cv.setData([], [])
            return
        if t_now is None:
            t_now = c.store.now()
        for kind, curves in self.curves.items():
            t, v = c.store.series(mid, kind, t_now - win, t_now)
            show = kind != "gps_enu" or (c.options.use_gps and len(t) > 0)
            self.plots[kind].setVisible(bool(show))
            if not show:
                continue
            t, v = decimate(t - t_now, v)
            for i, cv in enumerate(curves):
                cv.setData(t, v[:, i]) if len(t) else cv.setData([], [])
        self.plots["accel"].setXRange(-win, 0, padding=0.01)


class MiniEulerPlot(pg.PlotWidget):
    """Small euler plot for a cell of the grid layout."""

    def __init__(self, controller, module_id, parent=None):
        super().__init__(parent)
        self.controller, self.module_id = controller, module_id
        self.setBackground(T.bg1)
        self.setMenuEnabled(False)
        self.setMouseEnabled(x=False, y=False)
        style_plot(self.getPlotItem())
        self.setFixedHeight(110)
        self.curves = [self.plot(pen=pg.mkPen(XYZ_COLORS[i], width=1.5)) for i in range(3)]

    def refresh(self):
        c = self.controller
        t_now = c.view_time()
        if t_now is None:
            t_now = c.store.now()
        win = c.options.window_s
        t, v = c.store.series(self.module_id, "euler", t_now - win, t_now)
        t, v = decimate(t - t_now, v)
        for i, cv in enumerate(self.curves):
            cv.setData(t, v[:, i]) if len(t) else cv.setData([], [])
        self.setXRange(-win, 0, padding=0.01)
