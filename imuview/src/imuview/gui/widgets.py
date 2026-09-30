"""Small reusable widgets styled by theme.py. Imports Qt."""

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QButtonGroup,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from . import theme
from .theme import T


def restyle(w: QWidget):
    """Re-apply the stylesheet after a dynamic property changed."""
    w.style().unpolish(w)
    w.style().polish(w)
    w.update()


def button(text="", variant=None, icon_name=None, icon_color=None, tip=None) -> QPushButton:
    """variant: None (secondary) | "primary" (yellow, the main action) | "danger" | "flat"."""
    b = QPushButton(text)
    if variant:
        b.setProperty("variant", variant)
    if icon_name:
        color = icon_color or (T.on_accent if variant == "primary" else T.text)
        b.setIcon(theme.icon(icon_name, color))
        b.setIconSize(theme.icon_size())
    if tip:
        b.setToolTip(tip)
    return b


def set_variant(b: QPushButton, variant, icon_name=None, icon_color=None):
    b.setProperty("variant", variant)
    if icon_name:
        color = icon_color or (
            T.on_accent if variant == "primary" else T.on_danger if variant == "danger" else T.text
        )
        b.setIcon(theme.icon(icon_name, color))
    restyle(b)


def label(text="", muted=False, heading=False, mono=False) -> QLabel:
    lb = QLabel(text)
    if muted:
        lb.setProperty("muted", True)
    if heading:
        lb.setProperty("heading", True)
    if mono:
        lb.setProperty("mono", True)
    return lb


def separator() -> QFrame:
    f = QFrame()
    f.setObjectName("sep")
    f.setFixedWidth(1)
    return f


class Segmented(QFrame):
    """Segmented switch: exactly one of several options is active (yellow)."""

    changed = Signal(str)

    def __init__(self, options: list[tuple[str, str]], parent=None):
        super().__init__(parent)
        self.setObjectName("segmented")
        lay = QHBoxLayout(self)
        lay.setContentsMargins(2, 2, 2, 2)
        lay.setSpacing(2)
        self.group = QButtonGroup(self)
        self.group.setExclusive(True)
        self.buttons: dict[str, QPushButton] = {}
        for key, text in options:
            b = QPushButton(text)
            b.setProperty("segment", True)
            b.setCheckable(True)
            b.setCursor(Qt.PointingHandCursor)
            b.clicked.connect(lambda _=False, k=key: self._clicked(k))
            self.group.addButton(b)
            lay.addWidget(b)
            self.buttons[key] = b
        self.setFixedHeight(T.control_h)
        self.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)

    def _clicked(self, key):
        self.changed.emit(key)

    def value(self) -> str | None:
        return next((k for k, b in self.buttons.items() if b.isChecked()), None)

    def set_value(self, key: str):
        """Sets the active option without emitting `changed`."""
        if key in self.buttons:
            self.buttons[key].setChecked(True)

    def set_enabled_keys(self, keys):
        for k, b in self.buttons.items():
            b.setEnabled(k in keys)


class Chip(QLabel):
    """Coloured status pill. kind: ok | warn | danger | info | muted."""

    COLORS = {"ok": T.ok, "warn": T.warn, "danger": T.danger, "info": T.info,
              "muted": T.text_muted, "accent": T.accent}  # fmt: skip

    def __init__(self, text="", kind="muted", parent=None):
        super().__init__(text, parent)
        self.setAlignment(Qt.AlignCenter)
        self.setFixedHeight(24)
        self.set(text, kind)

    def set(self, text, kind):
        c = self.COLORS[kind]
        r, g, b = theme.rgb(c)
        self.setText(text)
        self.setStyleSheet(
            f"background: rgba({r},{g},{b},40); color: {c};"
            f"border: 1px solid rgba({r},{g},{b},120);"
            f"border-radius: 12px; padding: 0 {T.sp3}px; font-weight: 600;"
        )


class Dot(QLabel):
    """Round status dot with a tooltip."""

    def __init__(self, color=T.text_muted, size=10, parent=None):
        super().__init__(parent)
        self.size_ = size
        self.setFixedSize(size, size)
        self.set_color(color)

    def set_color(self, color, tip=None):
        self.setStyleSheet(f"background: {color}; border-radius: {self.size_ // 2}px;")
        if tip is not None:
            self.setToolTip(tip)


def card(parent=None) -> QFrame:
    f = QFrame(parent)
    f.setObjectName("card")
    return f


def panel(parent=None) -> QFrame:
    f = QFrame(parent)
    f.setObjectName("panel")
    return f


class Collapsible(QWidget):
    """Section with a header that folds the body. `toggled(bool)` tells the new open state."""

    toggled = Signal(bool)

    def __init__(self, title: str, open_=True, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(T.sp2)
        self.head = QToolButton()
        self.head.setText(title)
        self.head.setCheckable(True)
        self.head.setChecked(open_)
        self.head.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.head.setStyleSheet(
            f"QToolButton {{ border: none; font-weight: 600; text-align: left; padding: 4px 0; }}"
            f"QToolButton:hover {{ color: {T.accent}; }}"
        )
        self.head.setCursor(Qt.PointingHandCursor)
        self.head.clicked.connect(self._toggle)
        self.body = QWidget()
        self.body_layout = QVBoxLayout(self.body)
        self.body_layout.setContentsMargins(0, 0, 0, T.sp2)
        self.body_layout.setSpacing(T.sp2)
        lay.addWidget(self.head)
        lay.addWidget(self.body)
        self._sync()

    def _sync(self):
        self.head.setIcon(theme.icon("chevron-down" if self.head.isChecked() else "chevron-right",
                                     T.text_muted, 16))  # fmt: skip
        self.body.setVisible(self.head.isChecked())

    def _toggle(self):
        self._sync()
        self.toggled.emit(self.head.isChecked())

    def is_open(self) -> bool:
        return self.head.isChecked()

    def set_open(self, open_: bool):
        self.head.setChecked(open_)
        self._sync()


class Banner(QFrame):
    """Closable message strip. Raw exception text goes to "Деталі", not into the message."""

    retry = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.details_text = ""
        self.setStyleSheet(
            f"QFrame#banner {{ background: rgba(255,92,108,36); border: 1px solid {T.danger};"
            f"border-radius: {T.radius_button}px; }}"
        )
        self.setObjectName("banner")
        lay = QHBoxLayout(self)
        lay.setContentsMargins(T.sp3, T.sp2, T.sp2, T.sp2)
        ic = QLabel()
        ic.setPixmap(theme.icon("triangle-alert", T.danger, 18).pixmap(18, 18))
        self.text = QLabel("")
        self.text.setWordWrap(True)
        self.retry_btn = button("Перепід'єднати", icon_name="refresh-cw")
        self.details_btn = button("Деталі", "flat")
        self.close_btn = button("", "flat", "x")
        self.close_btn.setFixedWidth(T.control_h)
        for w in (ic, self.text):
            lay.addWidget(w, 1 if w is self.text else 0)
        for w in (self.retry_btn, self.details_btn, self.close_btn):
            lay.addWidget(w)
        self.retry_btn.clicked.connect(self.retry)
        self.close_btn.clicked.connect(self.hide)
        self.details_btn.clicked.connect(self._details)
        self.hide()

    def show_error(self, message: str, details: str = ""):
        self.text.setText(message)
        self.details_text = details
        self.details_btn.setVisible(bool(details))
        self.show()

    def _details(self):
        from PySide6.QtWidgets import QMessageBox

        box = QMessageBox(self)
        box.setWindowTitle("Деталі помилки")
        box.setText(self.details_text)
        box.exec()
