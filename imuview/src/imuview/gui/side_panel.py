"""Right panel with collapsible sections; the whole panel folds to a narrow strip."""

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QHBoxLayout,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from . import theme
from .theme import T
from .widgets import Collapsible, button, label

OPEN_WIDTH = 288
CLOSED_WIDTH = 44


class SidePanel(QWidget):
    manage_models = Signal()
    changed = Signal()  # anything was edited (to be saved)
    open_changed = Signal(bool)

    def __init__(self, controller):
        super().__init__()
        self.c = controller
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        head = QHBoxLayout()
        head.setContentsMargins(0, 0, 0, 0)
        self.title = label("Параметри", heading=True)
        self.toggle = button("", "flat", "settings-2", tip="Згорнути / розгорнути панель")
        self.toggle.setFixedWidth(T.control_h)
        head.addWidget(self.title, 1)
        head.addWidget(self.toggle)
        outer.addLayout(head)
        self.toggle.clicked.connect(lambda: self.set_open(not self._open))

        self.body = QFrame()
        self.body.setObjectName("panel")
        body_lay = QVBoxLayout(self.body)
        body_lay.setContentsMargins(T.sp3, T.sp3, T.sp3, T.sp3)
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setWidget(self.body)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        outer.addWidget(self.scroll, 1)

        self.data = Collapsible("Дані")
        self.gps = QCheckBox("Читати GPS")
        self.gravity = QCheckBox("Віднімати гравітацію")
        self.data.body_layout.addWidget(self.gps)
        self.data.body_layout.addWidget(self.gravity)

        self.viz = Collapsible("Візуалізація")
        self.trail = QCheckBox("Шлейф")
        self.trail_len = QDoubleSpinBox()
        self.trail_len.setRange(0.5, 10.0)
        self.trail_len.setSingleStep(0.5)
        self.trail_len.setSuffix(" с")
        row = QHBoxLayout()
        row.addWidget(self.trail, 1)
        row.addWidget(self.trail_len)
        self.viz.body_layout.addLayout(row)
        self.viz.body_layout.addWidget(label("3D-модель", muted=True))
        self.model = QComboBox()
        self.viz.body_layout.addWidget(self.model)
        self.manage = button("Керувати моделями…", icon_name="box")
        self.viz.body_layout.addWidget(self.manage)
        self.apply_all = QCheckBox("Застосувати до всіх модулів")
        self.viz.body_layout.addWidget(self.apply_all)

        for sec in (self.data, self.viz):
            body_lay.addWidget(sec)
            sec.toggled.connect(lambda _: self.changed.emit())
        body_lay.addStretch(1)

        self.gps.toggled.connect(self._on_gps)
        self.gravity.toggled.connect(self._on_gravity)
        self.trail.toggled.connect(self._on_trail)
        self.trail_len.valueChanged.connect(self._on_trail_len)
        self.model.currentIndexChanged.connect(self._on_model)
        self.manage.clicked.connect(self.manage_models)
        self.apply_all.toggled.connect(lambda _: self.changed.emit())
        self._open = True
        self._model_for = None  # module whose model the combo shows
        self.fill_models()
        self.pull()

    # ----- open / closed -----

    def is_open(self) -> bool:
        return self._open

    def set_open(self, open_: bool):
        self._open = open_
        self.scroll.setVisible(open_)
        self.title.setVisible(open_)
        self.setFixedWidth(OPEN_WIDTH if open_ else CLOSED_WIDTH)
        self.toggle.setIcon(theme.icon("settings-2", T.accent if not open_ else T.text))
        self.open_changed.emit(open_)

    # ----- controller <-> widgets -----

    def pull(self):
        """Show the controller's options (after the settings were loaded)."""
        o = self.c.options
        for w, v in ((self.gps, o.use_gps), (self.gravity, o.subtract_gravity),
                     (self.trail, o.show_trail)):  # fmt: skip
            w.blockSignals(True)
            w.setChecked(v)
            w.blockSignals(False)
        self.trail_len.blockSignals(True)
        self.trail_len.setValue(o.trail_seconds)
        self.trail_len.blockSignals(False)
        self.trail_len.setEnabled(o.show_trail)

    def fill_models(self):
        self.model.blockSignals(True)
        self.model.clear()
        for key, name in self.c.registry.names().items():
            self.model.addItem(name, key)
        self.model.blockSignals(False)
        self._model_for = "unset"

    def sync_model(self):
        """Keep the combo on the model of the active module (only when it changed)."""
        key = self.c.model_of(self.c.active) if self.c.active is not None else self.c.default_model
        marker = (self.c.active, key)
        if marker != self._model_for:
            self._model_for = marker
            self.model.blockSignals(True)
            self.model.setCurrentIndex(max(0, self.model.findData(key)))
            self.model.blockSignals(False)

    def _on_gps(self, on):
        self.c.options.use_gps = on
        self.changed.emit()

    def _on_gravity(self, on):
        self.c.options.subtract_gravity = on
        self.changed.emit()

    def _on_trail(self, on):
        self.c.options.show_trail = on
        self.trail_len.setEnabled(on)
        self.changed.emit()

    def _on_trail_len(self, v):
        self.c.options.trail_seconds = float(v)
        self.changed.emit()

    def _on_model(self):
        key = self.model.currentData()
        if key:
            self.c.set_model(key, everywhere=self.apply_all.isChecked())
            self._model_for = (self.c.active, key)
            self.changed.emit()
