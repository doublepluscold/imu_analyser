"""The column of small clickable module cards: a spinning point-cloud of the model, the name
and a live / lost dot. Clicking a card makes that module the active one. Imports Qt."""

import math

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QLabel, QScrollArea, QVBoxLayout, QWidget

from ..frames import quat_to_matrix
from . import theme
from .gl_common import module_color
from .models import NED_TO_SCENE
from .theme import T

CARD_W, CARD_H = 168, 128


def project(points_frd: np.ndarray, q, az_deg=35.0, el_deg=28.0) -> np.ndarray:
    """Orthographic (x, y) of model points for the thumbnail, looking from a fixed direction."""
    scene = (NED_TO_SCENE @ quat_to_matrix(q) @ points_frd.T).T  # x east, y north, z up
    az, el = math.radians(az_deg), math.radians(el_deg)
    x = scene[:, 0] * math.cos(az) - scene[:, 1] * math.sin(az)
    depth = scene[:, 0] * math.sin(az) + scene[:, 1] * math.cos(az)
    y = scene[:, 2] * math.cos(el) + depth * math.sin(el)
    return np.column_stack([x, y])


class ModuleCard(QWidget):
    clicked = Signal(int)

    def __init__(self, controller, module_id):
        super().__init__()
        self.controller, self.module_id = controller, module_id
        self.setFixedSize(CARD_W, CARD_H)
        self.setCursor(Qt.PointingHandCursor)
        self.points = None
        self.model_key = None

    def mousePressEvent(self, e):
        self.clicked.emit(self.module_id)

    def paintEvent(self, _):
        c, mid = self.controller, self.module_id
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        active = c.active == mid
        p.setBrush(theme.qcolor(T.bg3 if active else T.bg2))
        p.setPen(QPen(theme.qcolor(T.accent), 2) if active else QPen(theme.qcolor(T.border), 1))
        p.drawRoundedRect(QRectF(2, 2, CARD_W - 4, CARD_H - 4), 8, 8)
        key = c.model_of(mid)
        marker = (key, c.registry.version)
        if marker != self.model_key:
            self.points, self.model_key = c.registry.mesh(key).sample_points(500), marker
        pose = c.pose(mid)
        alive = c.alive(mid)
        if pose is not None:
            xy = project(self.points, pose.q)
            px_per_m = (CARD_H - 50) / c.registry.size
            col = QColor(*(int(255 * v) for v in module_color(mid)))
            col.setAlpha(230 if alive else 90)
            p.setPen(QPen(col, 2))
            for x, y in xy:
                p.drawPoint(QPointF(CARD_W / 2 + x * px_per_m, CARD_H / 2 - 8 - y * px_per_m))
        p.setPen(theme.qcolor(T.text))
        f = QFont(self.font())
        f.setBold(active)
        p.setFont(f)
        p.drawText(QRectF(10, CARD_H - 26, CARD_W - 40, 20), Qt.AlignVCenter, c.name(mid))
        rate = c.store.rate(mid, c.view_time())
        p.setPen(theme.qcolor(T.text_muted))
        p.setFont(theme.mono_font(8))
        p.drawText(
            QRectF(10, 8, CARD_W - 20, 16), Qt.AlignRight | Qt.AlignVCenter, f"{rate:.0f} Гц"
        )
        p.setPen(Qt.NoPen)
        p.setBrush(theme.qcolor(T.ok if alive else T.danger))
        p.drawEllipse(QPointF(CARD_W - 18, CARD_H - 18), 5, 5)
        self.setToolTip("Дані надходять" if alive else "Немає даних понад 1.5 с")


class ModuleStrip(QScrollArea):
    """Vertical list of cards; rebuilt only when the set of modules changes."""

    selected = Signal(int)

    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.cards: dict[int, ModuleCard] = {}
        self.setWidgetResizable(True)
        self.setFixedWidth(CARD_W + 26)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        body = QWidget()
        self.layout_ = QVBoxLayout(body)
        self.layout_.setAlignment(Qt.AlignTop)
        self.setWidget(body)
        self.empty = QLabel("Немає модулів.\nПід'єднайся до джерела даних.")
        self.empty.setWordWrap(True)
        self.empty.setProperty("muted", True)
        self.layout_.addWidget(self.empty)

    def refresh(self):
        ids = self.controller.module_ids()
        if list(self.cards) != ids:
            for card in self.cards.values():
                card.setParent(None)
            self.cards = {}
            for mid in ids:
                card = ModuleCard(self.controller, mid)
                card.clicked.connect(self.selected)
                self.layout_.addWidget(card)
                self.cards[mid] = card
        self.empty.setVisible(not self.cards)
        for card in self.cards.values():
            card.update()
