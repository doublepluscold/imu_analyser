"""Main window: top bar, sub-bar, module cards, content pages, side panel, status bar."""

import time
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QFileDialog,
    QHBoxLayout,
    QMainWindow,
    QSplitter,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from .calibration_page import CalibrationPage
from .controller import AppController
from .errors import friendly
from .module_cards import ModuleStrip
from .plots_view import PlotsPanel
from .raw_view import RawPanel
from .scene_area import SceneArea
from .settings import Settings
from .side_panel import SidePanel
from .status_bar import StatusBar, dir_size
from .theme import T
from .timeline import TimelineBar
from .toolbar import SubBar, TopBar
from .widgets import Banner

FRAME_MS = 33  # about 30 frames per second
PAGE_CONTENT, PAGE_RAW, PAGE_CALIB = 0, 1, 2


class MainWindow(QMainWindow):
    def __init__(self, controller: AppController, settings: Settings | None = None):
        super().__init__()
        self.c = controller
        self.settings = settings or Settings()
        self.setWindowTitle("imuview")
        self.resize(1450, 880)
        self.last_params = None  # (kind, params, target text) of the last connection
        self.error_shown = False
        self.tab = "live"
        self.settings.load_into(self.c)

        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        self.top = TopBar()
        self.sub = SubBar()
        self.banner = Banner()
        outer.addWidget(self.top)
        outer.addWidget(self.sub)

        body = QWidget()
        body_lay = QHBoxLayout(body)
        body_lay.setContentsMargins(T.sp3, T.sp3, T.sp3, T.sp3)
        body_lay.setSpacing(T.sp3)
        self.strip = ModuleStrip(controller)
        self.strip.selected.connect(controller.set_active)
        self.scene = SceneArea(controller)
        self.plots = PlotsPanel(controller)
        self.split = QSplitter(Qt.Horizontal)
        self.split.addWidget(self.scene)
        self.split.addWidget(self.plots)
        self.split.setSizes([900, 450])
        self.raw = RawPanel(controller)
        self.calib = CalibrationPage(controller, self.show_message)
        self.pages = QStackedWidget()
        self.pages.addWidget(self.split)
        self.pages.addWidget(self.raw)
        self.pages.addWidget(self.calib)
        self.side = SidePanel(controller)
        body_lay.addWidget(self.strip)
        body_lay.addWidget(self.pages, 1)
        body_lay.addWidget(self.side)
        banner_row = QHBoxLayout()
        banner_row.setContentsMargins(T.sp3, T.sp3, T.sp3, 0)
        banner_row.addWidget(self.banner)
        outer.addLayout(banner_row)
        outer.addWidget(body, 1)
        self.timeline = TimelineBar(controller)
        outer.addWidget(self.timeline)
        self.status = StatusBar()
        outer.addWidget(self.status)

        self.save_timer = QTimer(self)
        self.save_timer.setSingleShot(True)
        self.save_timer.timeout.connect(self.save_settings)
        self._wire()
        self._restore()
        self.last_tick = time.monotonic()
        self.last_size_check = 0.0
        self.log_bytes = 0
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(FRAME_MS)
        self.sync_controls()

    # ----- wiring -----

    def _wire(self):
        self.top.tab_changed.connect(self.set_tab)
        self.top.connect_clicked.connect(self.toggle_connection)
        self.top.start_clicked.connect(self.toggle_run)
        self.top.log_toggled.connect(self._on_log)
        self.top.changed.connect(self.schedule_save)
        self.sub.layout_changed.connect(self._on_layout)
        self.sub.view_changed.connect(self._on_view)
        self.sub.open_clicked.connect(self.open_session_dialog)
        self.side.changed.connect(self.schedule_save)
        self.side.manage_models.connect(self.open_model_manager)
        self.side.open_changed.connect(lambda _: self.schedule_save())
        self.banner.retry.connect(self.reconnect)
        self.split.splitterMoved.connect(lambda *_: self.schedule_save())
        QShortcut(QKeySequence("Ctrl+O"), self, activated=self.open_session_dialog)
        QShortcut(QKeySequence(Qt.Key_Return), self, activated=self.calib.on_enter)
        QShortcut(QKeySequence(Qt.Key_Enter), self, activated=self.calib.on_enter)
        QShortcut(QKeySequence(Qt.Key_Space), self, activated=self._space)

    def _restore(self):
        s = self.settings
        geo = s.get_bytes("window/geometry")
        if geo:
            self.restoreGeometry(geo)
        sizes = s.get_sizes("window/split")
        if sizes and len(sizes) == 2 and all(x >= 120 for x in sizes):
            self.split.setSizes(sizes)
        self.top.set_transport(s.get("transport"), s.get(f"port/{s.get('transport')}"))
        for kind in ("master", "udp", "serial", "sim"):
            self.top._port_memory[kind] = s.get(f"port/{kind}")
        self.top.log_check.setChecked(self.c.options.record)
        self.sub.layout_seg.set_value(self.c.options.layout)
        self.sub.view_seg.set_value(self.c.options.view)
        self.side.pull()
        self.side.apply_all.setChecked(s.get("apply_all"))
        self.side.data.set_open(s.get("section/data"))
        self.side.viz.set_open(s.get("section/viz"))
        self.side.set_open(s.get("side_open"))
        tab = s.get("tab")
        self.set_tab(tab if tab in ("live", "calibration") else "live")

    def save_settings(self):
        s = self.settings
        s.store_from(self.c)
        s.set("window/geometry", self.saveGeometry())
        sizes = self.split.sizes()
        if all(x >= 120 for x in sizes):  # not while one side is hidden by the view switch
            s.set("window/split", sizes)
        s.set("tab", self.tab)
        s.set("transport", self.top.transport.currentData())
        s.set(f"port/{self.top.transport.currentData()}", self.top.port_text())
        s.set("apply_all", self.side.apply_all.isChecked())
        s.set("side_open", self.side.is_open())
        s.set("section/data", self.side.data.is_open())
        s.set("section/viz", self.side.viz.is_open())
        s.sync()

    def schedule_save(self):
        self.save_timer.start(600)

    # ----- actions -----

    def _space(self):
        if self.tab == "analysis":
            self.timeline.toggle()

    def set_tab(self, tab):
        if tab == "analysis" and self.c.player is None:
            self.open_session_dialog()
            if self.c.player is None:
                self.top.tabs.set_value(self.tab)
                return
        self.tab = tab
        if tab == "calibration":
            self.c.mode = "calibration"
        else:
            self.c.set_mode(tab)
        self.top.tabs.set_value(tab)
        self.sync_controls()
        self.schedule_save()

    def _on_layout(self, key):
        self.c.set_layout(key)
        self.schedule_save()

    def _on_view(self, key):
        self.c.set_view(key)
        self.sync_controls()
        self.schedule_save()

    def _on_log(self, on):
        self.c.set_record(on)
        self.schedule_save()

    def toggle_connection(self):
        if self.c.live:
            self.c.stop_live()
        else:
            self.connect_now()
        self.sync_controls()

    def connect_now(self) -> bool:
        kind, target = self.top.transport.currentData(), self.top.port_text()
        try:
            kind, params = self.top.connection()
            self.c.start_live(kind, **params)
        except Exception as e:  # port busy, no such device, bad number...
            msg, details = friendly(e, kind, target)
            self.banner.show_error(msg, details)
            self.c.message = msg
            return False
        self.last_params = (kind, target)
        self.error_shown = False
        self.banner.hide()
        if self.c.mode == "analysis":
            self.set_tab("live")
        return True

    def reconnect(self):
        self.c.stop_live()
        self.connect_now()
        self.sync_controls()

    def toggle_run(self):
        if self.c.conn_state == "running":
            self.c.stop_run()
        elif self.c.conn_state == "connected":
            self.c.start_run()
        self.sync_controls()

    def open_session_dialog(self):
        start = "logs" if Path("logs").is_dir() else "."
        path = QFileDialog.getExistingDirectory(self, "Папка сесії (logs/...)", start)
        if path:
            self.c.open_session(path)
            self.sub.session_label.setText(Path(path).name)
            self.set_tab("analysis")

    def open_model_manager(self):
        if self.c.library is None:
            self.c.message = "Бібліотека моделей недоступна"
            return
        from .model_manager import ModelManager

        if ModelManager(self.c, self).exec():
            self.side.fill_models()
            self.schedule_save()

    def show_message(self, text):
        self.c.message = text

    # ----- state -> widgets -----

    def sync_controls(self):
        c = self.c
        self.top.sync(c.conn_state, c.recording)
        self.sub.setVisible(self.tab != "calibration")
        self.sub.set_analysis(self.tab == "analysis")
        self.side.setVisible(self.tab != "calibration")
        self.timeline.setVisible(self.tab == "analysis")
        self.plots.setVisible(self.plots_visible())
        self.scene.setVisible(self.scene_visible())
        self.sub.layout_seg.set_value(c.options.layout)
        self.sub.view_seg.set_value(c.options.view)

    def scene_visible(self):
        return self.c.options.view in ("3d", "both")

    def plots_visible(self):
        o = self.c.options
        return o.view == "plots" or (o.view == "both" and o.layout != "grid")

    def _check_errors(self):
        live = self.c.live
        if live and live.error and not self.error_shown:
            self.error_shown = True
            kind, target = self.last_params or ("", "")
            msg, details = friendly(live.error_exc or RuntimeError(live.error), kind, target)
            self.banner.show_error(f"З'єднання перервано. {msg}", details)
            self.c.stop_live()
            self.sync_controls()

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        c = self.c
        c.poll()
        self._check_errors()
        if c.player:
            c.player.tick(dt)
        self.top.sync(c.conn_state, c.recording)
        if c.mode == "calibration":
            self.pages.setCurrentIndex(PAGE_CALIB)
            self.calib.refresh()
        elif c.options.view == "raw":
            self.pages.setCurrentIndex(PAGE_RAW)
            self.raw.refresh()
        else:
            self.pages.setCurrentIndex(PAGE_CONTENT)
            if self.scene_visible():
                self.scene.refresh()
            if self.plots_visible():
                self.plots.refresh()
        self.strip.refresh()
        self.side.sync_model()
        self.timeline.refresh()
        if c.live and c.live.session_dir and now - self.last_size_check > 1.0:
            self.last_size_check = now
            self.log_bytes = dir_size(c.live.session_dir)
        rate = c.store.rate(c.active, c.view_time()) if c.active is not None else None
        self.status.set_state(c.conn_state, c.recording)
        self.status.set_info(c.message, rate, self.log_bytes if c.recording else None)

    def closeEvent(self, e):
        self.timer.stop()
        self.save_settings()
        self.c.shutdown()
        super().closeEvent(e)
