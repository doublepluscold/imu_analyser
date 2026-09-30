"""Bottom bar: state chip, packet rate, log size, message."""

from pathlib import Path

from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel

from . import theme
from .theme import T
from .widgets import Chip, label


def dir_size(path: Path | None) -> int:
    if not path or not Path(path).is_dir():
        return 0
    return sum(f.stat().st_size for f in Path(path).iterdir() if f.is_file())


def human_size(n: int) -> str:
    for unit in ("Б", "КБ", "МБ", "ГБ"):
        if n < 1024 or unit == "ГБ":
            return f"{n:.0f} {unit}" if unit == "Б" else f"{n:.1f} {unit}"
        n /= 1024


class StatusBar(QFrame):
    def __init__(self):
        super().__init__()
        self.setObjectName("statusbar")
        self.setFixedHeight(36)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(T.sp3, 0, T.sp3, 0)
        lay.setSpacing(T.sp4)
        self.chip = Chip("Відключено", "muted")
        self.message = label("", muted=True)
        self.rate = QLabel("")
        self.log = QLabel("")
        for w in (self.rate, self.log):
            w.setFont(theme.mono_font(9))
        lay.addWidget(self.chip)
        lay.addWidget(self.message, 1)
        lay.addWidget(self.rate)
        lay.addWidget(self.log)
        self._last_size_check = 0.0

    def set_state(self, state: str, recording: bool):
        if state == "disconnected":
            self.chip.set("Відключено", "muted")
        elif recording:
            self.chip.set("Запис", "danger")
        elif state == "running":
            self.chip.set("Працює", "ok")
        else:
            self.chip.set("Підключено", "info")

    def set_info(self, message: str, rate_hz: float | None, log_bytes: int | None):
        self.message.setText(message)
        self.rate.setText("" if rate_hz is None else f"{rate_hz:5.0f} Гц")
        self.log.setText("" if log_bytes is None else f"лог {human_size(log_bytes)}")
