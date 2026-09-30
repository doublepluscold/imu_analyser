"""Every colour, size and font of the GUI lives here. QSS is generated from these tokens;
pyqtgraph plots and the 3D view take their colours from the same tokens. Imports Qt."""

import sys
import tempfile
from pathlib import Path

from PySide6.QtCore import QSize, Qt
from PySide6.QtGui import QColor, QFont, QFontDatabase, QIcon, QPainter, QPalette, QPixmap
from PySide6.QtSvg import QSvgRenderer
from PySide6.QtWidgets import QApplication


class T:
    """Design tokens."""

    # surfaces
    bg0, bg1, bg2, bg3 = "#081120", "#0D182B", "#13223B", "#1B2E4D"
    border = "#243656"
    text, text_muted = "#E8EDF7", "#8B99B5"
    # accent (main action and active elements only)
    accent, accent_hover, accent_press, on_accent = "#FFC83D", "#FFD466", "#E0A800", "#081120"
    # status
    ok, warn, danger, info = "#3DDC97", "#FF9F43", "#FF5C6C", "#4DA3FF"
    on_danger = "#FFFFFF"
    # plot lines: x/y/z and roll/pitch/yaw
    line_x, line_y, line_z = "#FF6B6B", "#4ADE80", "#4DA3FF"
    # geometry (multiples of 4)
    radius_card, radius_button = 8, 6
    control_h = 32
    sp1, sp2, sp3, sp4 = 4, 8, 12, 16
    # fonts
    font_ui, font_mono, font_fallback = "Inter", "JetBrains Mono", "Noto Sans"
    font_pt = 10


XYZ = (T.line_x, T.line_y, T.line_z)


def rgb(hex_color: str) -> tuple[int, int, int]:
    h = hex_color.lstrip("#")
    return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)


def rgbf(hex_color: str, alpha: float = 1.0) -> tuple[float, float, float, float]:
    """For OpenGL: floats 0..1 with alpha."""
    r, g, b = rgb(hex_color)
    return r / 255, g / 255, b / 255, alpha


def qcolor(hex_color: str, alpha: int = 255) -> QColor:
    c = QColor(hex_color)
    c.setAlpha(alpha)
    return c


# ---------- paths, fonts ----------


def assets_dir() -> Path:
    """assets/ next to the sources, or inside the PyInstaller bundle."""
    base = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parents[3]))
    return base / "assets"


def load_fonts() -> dict:
    """Registers the bundled .ttf files. Returns which families are available."""
    found = set()
    for ttf in sorted((assets_dir() / "fonts").glob("*.ttf")):
        fid = QFontDatabase.addApplicationFont(str(ttf))
        if fid >= 0:
            found.update(QFontDatabase.applicationFontFamilies(fid))
    ui = T.font_ui if T.font_ui in found else T.font_fallback
    mono = T.font_mono if T.font_mono in found else QFontDatabase.systemFont(
        QFontDatabase.FixedFont).family()  # fmt: skip
    return {"ui": ui, "mono": mono, "bundled": sorted(found)}


_FONTS = {"ui": T.font_fallback, "mono": "monospace"}


def mono_font(pt: float | None = None, bold=False) -> QFont:
    f = QFont(_FONTS["mono"])
    f.setPointSizeF(pt or T.font_pt)
    f.setBold(bold)
    return f


# ---------- icons ----------

_ICON_CACHE: dict = {}


def icon(name: str, color: str | None = None, size: int = 18) -> QIcon:
    """Lucide SVG from assets/icons, recoloured. Disabled state is drawn in the muted colour."""
    key = (name, color, size)
    if key in _ICON_CACHE:
        return _ICON_CACHE[key]
    svg = (assets_dir() / "icons" / f"{name}.svg").read_text()

    def pix(c):
        renderer = QSvgRenderer(svg.replace("currentColor", c).encode())
        ratio = 2  # sharp on high-DPI screens
        pm = QPixmap(size * ratio, size * ratio)
        pm.fill(Qt.transparent)
        p = QPainter(pm)
        renderer.render(p)
        p.end()
        pm.setDevicePixelRatio(ratio)
        return pm

    ic = QIcon()
    ic.addPixmap(pix(color or T.text), QIcon.Normal)
    ic.addPixmap(pix(T.text_muted), QIcon.Disabled)
    _ICON_CACHE[key] = ic
    return ic


def icon_size(px: int = 18) -> QSize:
    return QSize(px, px)


def _write_qss_images() -> dict:
    """QSS `image: url()` needs files: write the few recoloured SVGs it uses."""
    d = Path(tempfile.gettempdir()) / "imuview-theme"
    d.mkdir(exist_ok=True)
    out = {}
    for name, color in (("check", T.on_accent), ("chevron-down", T.text_muted)):
        svg = (assets_dir() / "icons" / f"{name}.svg").read_text().replace("currentColor", color)
        path = d / f"{name}.svg"
        path.write_text(svg)
        out[name] = path.as_posix()
    return out


# ---------- stylesheet ----------


def build_qss() -> str:
    img = _write_qss_images()
    t = T
    return f"""
* {{ font-size: {t.font_pt}pt; }}
QMainWindow, QDialog {{ background: {t.bg0}; }}
QWidget {{ color: {t.text}; }}
QToolTip {{ background: {t.bg3}; color: {t.text}; border: 1px solid {t.border}; padding: 4px; }}
QLabel {{ background: transparent; }}
QLabel[muted="true"] {{ color: {t.text_muted}; }}
QLabel[heading="true"] {{ font-size: {t.font_pt + 3}pt; font-weight: 600; }}
QLabel[mono="true"] {{ font-family: "{_FONTS["mono"]}"; }}

QFrame#topbar {{ background: {t.bg1}; border-bottom: 1px solid {t.border}; }}
QFrame#subbar {{ background: {t.bg0}; border-bottom: 1px solid {t.border}; }}
QFrame#panel {{ background: {t.bg1}; border: 1px solid {t.border}; border-radius: {t.radius_card}px; }}
QFrame#card {{ background: {t.bg2}; border: 1px solid {t.border}; border-radius: {t.radius_card}px; }}
QFrame#card[selected="true"] {{ border: 2px solid {t.accent}; }}
QFrame#sep {{ background: {t.border}; max-width: 1px; min-width: 1px; }}
QStatusBar, QFrame#statusbar {{ background: {t.bg1}; border-top: 1px solid {t.border}; }}

QPushButton {{
    background: {t.bg2}; border: 1px solid {t.border}; border-radius: {t.radius_button}px;
    padding: 0 {t.sp3}px; min-height: {t.control_h}px; max-height: {t.control_h}px;
}}
QPushButton:hover {{ background: {t.bg3}; }}
QPushButton:pressed {{ background: {t.bg1}; }}
QPushButton:disabled {{ color: {t.text_muted}; background: {t.bg1}; border-color: {t.bg2}; }}
QPushButton[variant="primary"] {{
    background: {t.accent}; color: {t.on_accent}; border: 1px solid {t.accent}; font-weight: 600;
}}
QPushButton[variant="primary"]:hover {{ background: {t.accent_hover}; border-color: {t.accent_hover}; }}
QPushButton[variant="primary"]:pressed {{ background: {t.accent_press}; border-color: {t.accent_press}; }}
QPushButton[variant="primary"]:disabled {{ background: {t.bg2}; color: {t.text_muted}; border-color: {t.bg2}; }}
QPushButton[variant="danger"] {{
    background: {t.danger}; color: {t.on_danger}; border: 1px solid {t.danger}; font-weight: 600;
}}
QPushButton[variant="danger"]:hover {{ background: #FF7785; }}
QPushButton[variant="flat"] {{ background: transparent; border: 1px solid transparent; }}
QPushButton[variant="flat"]:hover {{ background: {t.bg3}; }}
QPushButton[segment="true"] {{
    background: transparent; border: none; border-radius: {t.radius_button - 2}px;
    min-height: {t.control_h - 4}px; max-height: {t.control_h - 4}px; padding: 0 {t.sp3}px;
    color: {t.text_muted};
}}
QPushButton[segment="true"]:hover {{ color: {t.text}; background: {t.bg3}; }}
QPushButton[segment="true"]:checked {{ background: {t.accent}; color: {t.on_accent}; font-weight: 600; }}
QPushButton[segment="true"]:disabled {{ color: {t.bg3}; }}
QFrame#segmented {{
    background: {t.bg2}; border: 1px solid {t.border}; border-radius: {t.radius_button}px;
}}

QComboBox, QLineEdit, QSpinBox, QDoubleSpinBox {{
    background: {t.bg2}; border: 1px solid {t.border}; border-radius: {t.radius_button}px;
    padding: 0 {t.sp2}px; min-height: {t.control_h}px; max-height: {t.control_h}px;
    selection-background-color: {t.accent}; selection-color: {t.on_accent};
}}
QComboBox:hover, QLineEdit:hover, QSpinBox:hover, QDoubleSpinBox:hover {{ background: {t.bg3}; }}
QComboBox:focus, QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {{ border: 1px solid {t.accent}; }}
QComboBox:disabled, QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled {{
    color: {t.text_muted}; background: {t.bg1}; border-color: {t.bg2};
}}
QComboBox::drop-down {{ border: none; width: 24px; }}
QComboBox::down-arrow {{ image: url({img["chevron-down"]}); width: 12px; height: 12px; }}
QComboBox QAbstractItemView {{
    background: {t.bg2}; border: 1px solid {t.border}; selection-background-color: {t.bg3};
    selection-color: {t.text}; outline: none;
}}
QSpinBox::up-button, QSpinBox::down-button, QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
    width: 16px; border: none; background: transparent;
}}

QCheckBox {{ spacing: {t.sp2}px; min-height: {t.control_h - 8}px; }}
QCheckBox::indicator {{
    width: 16px; height: 16px; border-radius: 4px; border: 1px solid {t.border}; background: {t.bg2};
}}
QCheckBox::indicator:hover {{ border-color: {t.text_muted}; }}
QCheckBox::indicator:checked {{ background: {t.accent}; border-color: {t.accent}; image: url({img["check"]}); }}
QCheckBox::indicator:disabled {{ background: {t.bg1}; border-color: {t.bg2}; }}
QCheckBox::indicator:checked:disabled {{ background: {t.accent_press}; border-color: {t.accent_press}; }}
QCheckBox:disabled {{ color: {t.text_muted}; }}

QSlider::groove:horizontal {{ height: 4px; background: {t.bg3}; border-radius: 2px; }}
QSlider::sub-page:horizontal {{ background: {t.accent}; border-radius: 2px; }}
QSlider::handle:horizontal {{
    background: {t.text}; width: 14px; height: 14px; margin: -6px 0; border-radius: 7px;
}}
QSlider::handle:horizontal:hover {{ background: {t.accent_hover}; }}
QSlider:disabled {{ }}

QProgressBar {{
    background: {t.bg2}; border: 1px solid {t.border}; border-radius: {t.radius_button}px;
    text-align: center; min-height: 16px; max-height: 16px; color: {t.text};
}}
QProgressBar::chunk {{ background: {t.accent}; border-radius: {t.radius_button - 1}px; }}

QScrollArea {{ border: none; background: transparent; }}
QScrollBar:vertical {{ background: transparent; width: 10px; margin: 0; }}
QScrollBar::handle:vertical {{ background: {t.bg3}; border-radius: 5px; min-height: 24px; }}
QScrollBar::handle:vertical:hover {{ background: {t.border}; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 0; }}
QScrollBar::handle:horizontal {{ background: {t.bg3}; border-radius: 5px; min-width: 24px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}

QSplitter::handle {{ background: {t.bg0}; }}
QSplitter::handle:hover {{ background: {t.border}; }}

QTableWidget, QPlainTextEdit, QListWidget {{
    background: {t.bg1}; border: 1px solid {t.border}; border-radius: {t.radius_button}px;
    gridline-color: {t.border}; alternate-background-color: {t.bg2}; outline: none;
}}
QListWidget::item {{ padding: 4px; border-radius: {t.radius_button}px; }}
QListWidget::item:selected {{ background: {t.bg3}; border: 1px solid {t.accent}; color: {t.text}; }}
QListWidget::item:hover {{ background: {t.bg3}; }}
QHeaderView::section {{
    background: {t.bg2}; color: {t.text_muted}; border: none; border-bottom: 1px solid {t.border};
    padding: 4px 8px;
}}
QMenu {{ background: {t.bg2}; border: 1px solid {t.border}; padding: 4px; }}
QMenu::item {{ padding: 6px 16px; border-radius: 4px; }}
QMenu::item:selected {{ background: {t.bg3}; }}
QMessageBox {{ background: {t.bg1}; }}
"""


def apply_theme(app: QApplication) -> dict:
    """Fusion style, dark palette, bundled fonts, generated stylesheet."""
    fonts = load_fonts()
    _FONTS.update(fonts)
    app.setStyle("Fusion")
    p = QPalette()
    roles = {
        QPalette.Window: T.bg0, QPalette.WindowText: T.text, QPalette.Base: T.bg1,
        QPalette.AlternateBase: T.bg2, QPalette.Text: T.text, QPalette.Button: T.bg2,
        QPalette.ButtonText: T.text, QPalette.Highlight: T.accent,
        QPalette.HighlightedText: T.on_accent, QPalette.ToolTipBase: T.bg3,
        QPalette.ToolTipText: T.text, QPalette.PlaceholderText: T.text_muted,
        QPalette.Link: T.info,
    }  # fmt: skip
    for role, color in roles.items():
        p.setColor(role, QColor(color))
    for role in (QPalette.WindowText, QPalette.Text, QPalette.ButtonText):
        p.setColor(QPalette.Disabled, role, QColor(T.text_muted))
    app.setPalette(p)
    f = QFont(fonts["ui"])
    f.setPointSize(T.font_pt)
    app.setFont(f)
    app.setStyleSheet(build_qss())
    return fonts
