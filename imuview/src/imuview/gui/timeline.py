"""Bottom bar of the analysis mode: play / pause, slider, speed, time. Imports Qt."""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QLabel, QPushButton, QSlider, QWidget

SLIDER_STEPS = 10000
SPEEDS = [0.25, 0.5, 1.0, 2.0, 4.0, 8.0]


def fmt_time(seconds: float) -> str:
    m, s = divmod(max(0.0, seconds), 60)
    return f"{int(m):02d}:{s:05.2f}"


class TimelineBar(QWidget):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        lay = QHBoxLayout(self)
        lay.setContentsMargins(6, 2, 6, 2)
        self.play = QPushButton("▶ Грати")
        self.play.setFixedWidth(90)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, SLIDER_STEPS)
        self.speed = QComboBox()
        for s in SPEEDS:
            self.speed.addItem(f"{s:g}x", s)
        self.speed.setCurrentIndex(SPEEDS.index(1.0))
        self.label = QLabel("00:00.00 / 00:00.00")
        self.label.setMinimumWidth(130)
        for w in (self.play, self.slider, self.speed, self.label):
            lay.addWidget(w, 1 if w is self.slider else 0)
        self.play.clicked.connect(self.toggle)
        self.slider.sliderMoved.connect(self.moved)
        self.speed.currentIndexChanged.connect(self.speed_changed)
        self._dragging = False
        self.slider.sliderPressed.connect(lambda: setattr(self, "_dragging", True))
        self.slider.sliderReleased.connect(lambda: setattr(self, "_dragging", False))

    @property
    def player(self):
        return self.controller.player

    def toggle(self):
        p = self.player
        if p and p.loaded:
            p.pause() if p.playing else p.play()

    def moved(self, value):
        p = self.player
        if p and p.loaded:
            p.seek(p.t_start + value / SLIDER_STEPS * (p.t_end - p.t_start))

    def speed_changed(self):
        if self.player:
            self.player.speed = self.speed.currentData()

    def refresh(self):
        p = self.player
        ok = bool(p and p.loaded)
        self.setEnabled(ok)
        if not ok:
            return
        self.play.setText("⏸ Пауза" if p.playing else "▶ Грати")
        if not self._dragging:
            self.slider.setValue(round(p.fraction * SLIDER_STEPS))
        self.label.setText(f"{fmt_time(p.t - p.t_start)} / {fmt_time(p.t_end - p.t_start)}")
