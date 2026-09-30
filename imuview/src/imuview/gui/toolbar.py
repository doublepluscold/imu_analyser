"""Top bar (tabs + connection + Start) and the sub-bar (layout and view)."""

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QCheckBox, QComboBox, QFrame, QHBoxLayout, QLabel

from . import theme
from .controller import LAYOUTS
from .theme import T
from .widgets import Dot, Segmented, button, label, separator, set_variant

TABS = [("live", "Лайв"), ("calibration", "Калібрація"), ("analysis", "Аналіз")]
TRANSPORTS = [("serial", "USB-serial"), ("udp", "Мережа (UDP)"), ("sim", "Демо (симулятор)")]
LAYOUT_NAMES = {"single": "Один модуль", "grid": "Сітка", "scene": "Одна сцена"}
VIEW_OPTIONS = [("3d", "3D"), ("plots", "Графіки"), ("both", "3D + графіки"), ("raw", "MTData")]


def list_serial_ports() -> list[str]:
    try:
        from serial.tools import list_ports

        return [p.device for p in list_ports.comports()]
    except Exception:  # pyserial missing or the OS refused: an empty list is fine
        return []


class TopBar(QFrame):
    tab_changed = Signal(str)
    connect_clicked = Signal()
    start_clicked = Signal()
    log_toggled = Signal(bool)
    changed = Signal()  # transport or port edited (to be saved)

    def __init__(self):
        super().__init__()
        self.setObjectName("topbar")
        self.setFixedHeight(56)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(T.sp4, 0, T.sp4, 0)
        lay.setSpacing(T.sp2)
        self.tabs = Segmented(TABS)
        self.tabs.changed.connect(self.tab_changed)
        lay.addWidget(self.tabs)
        lay.addStretch(1)

        self.transport = QComboBox()
        for key, text in TRANSPORTS:
            self.transport.addItem(text, key)
        self.transport.setMinimumWidth(170)
        self.port = QComboBox()
        self.port.setEditable(True)
        self.port.setMinimumWidth(170)
        self.refresh_btn = button("", "flat", "refresh-cw", tip="Оновити список портів")
        self.refresh_btn.setFixedWidth(T.control_h)
        self.connect_btn = button("Під'єднати", icon_name="plug")
        self.rec_dot = Dot(T.border, 10)
        self.rec_text = QLabel("REC")
        self.rec_text.setFont(theme.mono_font(9, bold=True))
        self.log_check = QCheckBox("Писати лог")
        self.start_btn = button("Старт", "primary", "play")
        self.start_btn.setMinimumWidth(110)
        for w in (self.transport, self.port, self.refresh_btn, self.connect_btn, separator()):
            lay.addWidget(w)
        lay.addWidget(self.log_check)
        lay.addWidget(self.rec_dot)
        lay.addWidget(self.rec_text)
        lay.addWidget(self.start_btn)

        self.transport.currentIndexChanged.connect(self._transport_changed)
        self.port.editTextChanged.connect(lambda _: self.changed.emit())
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.connect_clicked)
        self.start_btn.clicked.connect(self.start_clicked)
        self.log_check.toggled.connect(self.log_toggled)
        self._port_memory = {}
        self._current = "sim"

    # ----- transport and port -----

    def _transport_changed(self):
        self._port_memory[self._current] = self.port.currentText()
        self._current = self.transport.currentData()
        self.fill_ports()
        self.changed.emit()

    def fill_ports(self, preferred: str | None = None):
        kind = self._current
        self.port.blockSignals(True)
        self.port.clear()
        if kind == "serial":
            self.port.addItems(list_serial_ports())
            self.port.setEditable(True)
        elif kind == "udp":
            self.port.addItems(["5005"])
            self.port.setEditable(True)
        else:
            for n in range(1, 9):
                self.port.addItem(
                    f"{n} {'модуль' if n == 1 else 'модулі' if n < 5 else 'модулів'}", n
                )
            self.port.setEditable(False)
        text = preferred or self._port_memory.get(kind)
        if text:
            if self.port.isEditable():
                self.port.setCurrentText(text)
            else:
                k = self.port.findData(int(text)) if text.isdigit() else -1
                self.port.setCurrentIndex(max(0, k))
        self.port.blockSignals(False)
        self.refresh_btn.setVisible(kind == "serial")

    def refresh_ports(self):
        keep = self.port.currentText()
        self.fill_ports(keep)

    def set_transport(self, kind: str, port_text: str):
        self._port_memory[kind] = port_text
        self.transport.blockSignals(True)
        self.transport.setCurrentIndex(max(0, self.transport.findData(kind)))
        self.transport.blockSignals(False)
        self._current = kind
        self.fill_ports(port_text)

    def port_text(self) -> str:
        if self._current == "sim":
            return str(self.port.currentData() or 3)
        return self.port.currentText().strip()

    def connection(self) -> tuple[str, dict]:
        """(kind, parameters for AppController.start_live). ValueError for a bad UDP port."""
        kind = self._current
        if kind == "serial":
            return kind, {"serial_port": self.port.currentText().strip()}
        if kind == "udp":
            n = int(self.port.currentText())
            if not 1 <= n <= 65535:
                raise ValueError("port out of range")
            return kind, {"port": n}
        return kind, {"sim_modules": int(self.port.currentData() or 3), "sim_gps": True}

    # ----- state machine: widgets are disabled, never hidden -----

    def sync(self, state: str, recording: bool):
        connected = state != "disconnected"
        running = state == "running"
        for w in (self.transport, self.port, self.refresh_btn):
            w.setEnabled(not connected)
        self.connect_btn.setText("Від'єднати" if connected else "Під'єднати")
        self.connect_btn.setIcon(theme.icon("unplug" if connected else "plug", T.text))
        self.connect_btn.setEnabled(not running)  # stop first
        self.start_btn.setEnabled(connected)
        self.log_check.setEnabled(not running)
        if running:
            self.start_btn.setText("Стоп")
            set_variant(self.start_btn, "danger", "square")
        else:
            self.start_btn.setText("Старт")
            set_variant(self.start_btn, "primary", "play")
        self.rec_dot.set_color(T.danger if recording else T.border)
        self.rec_text.setStyleSheet(f"color: {T.danger if recording else T.text_muted};")


class SubBar(QFrame):
    layout_changed = Signal(str)
    view_changed = Signal(str)
    open_clicked = Signal()

    def __init__(self):
        super().__init__()
        self.setObjectName("subbar")
        self.setFixedHeight(48)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(T.sp4, 0, T.sp4, 0)
        lay.setSpacing(T.sp2)
        self.layout_seg = Segmented([(k, LAYOUT_NAMES[k]) for k in LAYOUTS])
        self.view_seg = Segmented(VIEW_OPTIONS)
        self.layout_seg.changed.connect(self.layout_changed)
        self.view_seg.changed.connect(self.view_changed)
        self.layout_label = label("Розкладка", muted=True)
        self.view_label = label("Вигляд", muted=True)
        self.open_btn = button("Відкрити сесію…", icon_name="folder-open", tip="Ctrl+O")
        self.session_label = label("", muted=True)
        lay.addWidget(self.layout_label)
        lay.addWidget(self.layout_seg)
        lay.addSpacing(T.sp4)
        lay.addWidget(self.view_label)
        lay.addWidget(self.view_seg)
        lay.addStretch(1)
        lay.addWidget(separator())
        lay.addWidget(self.open_btn)
        lay.addWidget(self.session_label)
        self.set_analysis(False)

    def set_analysis(self, on: bool):
        self.open_btn.setVisible(on)
        self.session_label.setVisible(on)
        self.findChildren(QFrame, "sep")[0].setVisible(on)
