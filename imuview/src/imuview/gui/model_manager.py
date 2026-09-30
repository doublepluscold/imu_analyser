"""3D model manager dialog: list on the left, preview and parameters on the right.
Changes are made on a draft and only "Зберегти" applies them to the session. Imports Qt."""

import math
from pathlib import Path

import numpy as np
import pyqtgraph.opengl as gl
from PySide6.QtCore import QSize, Qt, QTimer
from PySide6.QtGui import QIcon, QPainter, QPen, QPixmap, QVector3D
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from ..frames import board_rotation
from . import theme
from .gl_common import (
    LIT,
    mesh_data,
    overlay_label,
    place_label,
    project_point,
    register_lit_shader,
    transform3d,
)
from .models import AXES, axes_from_matrix, axes_rotation, bbox, surface_centroid
from .module_cards import project
from .theme import T
from .widgets import Collapsible, button, label

FRD_TO_SCENE = np.diag([1.0, -1.0, -1.0, 1.0])  # body x nose / y right / z down -> x, y, z up
AXIS_NAMES = list(AXES)
AXIS_TEXT = {
    "+X": "+X", "-X": "−X", "+Y": "+Y", "-Y": "−Y", "+Z": "+Z", "-Z": "−Z",
}  # fmt: skip
COLLINEAR_TEXT = "Вісь носа і вісь верху не можуть лежати на одній прямій."


def thumbnail(mesh, size=44) -> QIcon:
    """Point-cloud picture of a model for the list."""
    pm = QPixmap(size * 2, size * 2)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing)
    p.setBrush(theme.qcolor(T.bg1))
    p.setPen(Qt.NoPen)
    p.drawRoundedRect(0, 0, size * 2, size * 2, 12, 12)
    p.setPen(QPen(theme.qcolor(T.accent if not mesh.spec.broken else T.text_muted), 3))
    xy = project(mesh.sample_points(350), (1.0, 0.0, 0.0, 0.0))
    scale = size * 2 * 0.8 / max(float(np.ptp(xy[:, 0])), float(np.ptp(xy[:, 1])), 1e-6)
    cx, cy = (xy.min(axis=0) + xy.max(axis=0)) / 2
    for x, y in xy:
        p.drawPoint(int(size + (x - cx) * scale), int(size - (y - cy) * scale))
    p.end()
    pm.setDevicePixelRatio(2)
    return QIcon(pm)


class ModelPreview(gl.GLViewWidget):
    """The model with its body axes (nose arrow labelled) and the pivot marker at the origin.
    The mesh is centred on the pivot, so turning it shows what it turns around."""

    def __init__(self, parent=None):
        register_lit_shader()
        super().__init__(parent)
        self.setBackgroundColor(T.bg1)
        self.setMinimumSize(380, 300)
        self.mesh_item = None
        self.angle = 0.0
        grid = gl.GLGridItem(size=QVector3D(8, 8, 1), color=(*theme.rgb(T.text_muted), 45))
        self.addItem(grid)
        self.axes = []
        self.tags = []  # (label, tip position in body axes)
        for vec, color, text in (((1, 0, 0), T.danger, "Ніс"), ((0, 1, 0), T.ok, "Право"),
                                 ((0, 0, 1), T.info, "Низ")):  # fmt: skip
            length = 2.6
            line = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], np.array(vec) * length]), width=3,
                                     color=theme.rgbf(color), antialias=True)  # fmt: skip
            tag = overlay_label(self, color)
            tag.setText(text)
            self.axes.append(line)
            self.addItem(line)
            self.tags.append((tag, np.array(vec) * (length + 0.3)))
        # the pivot marker is a widget over the view, so the model cannot hide it
        self.pivot_dot = QLabel(self)
        self.pivot_dot.setFixedSize(16, 16)
        self.pivot_dot.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.pivot_dot.setToolTip("Центр обертання")
        self.pivot_dot.setStyleSheet(
            f"background: {T.accent}; border: 3px solid {T.bg0}; border-radius: 8px;"
        )
        self.setCameraPosition(distance=12, elevation=22, azimuth=-130)
        self._apply_angle()

    def set_mesh(self, mesh):
        if self.mesh_item is not None:
            self.removeItem(self.mesh_item)
        fc = mesh.face_colors if mesh.spec.file is None or mesh.spec.base else None
        color = (*mesh.spec.color, 1.0)
        md = mesh_data(mesh.tris, fc)
        self.mesh_item = gl.GLMeshItem(meshdata=md, smooth=False, shader=LIT, color=color,
                                       glOptions="opaque")  # fmt: skip
        self.addItem(self.mesh_item)
        self._apply_angle()

    def set_angle(self, deg: float):
        self.angle = deg
        self._apply_angle()

    def paintGL(self):
        super().paintGL()
        self._place_tags()

    def _place_tags(self):
        pos = project_point(self, (0.0, 0.0, 0.0))
        if pos is not None:
            self.pivot_dot.move(int(pos[0] - 8), int(pos[1] - 8))
            self.pivot_dot.show()

        a = math.radians(self.angle)
        rot = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        for tag, tip in self.tags:
            place_label(self, tag, rot @ (FRD_TO_SCENE[:3, :3] @ tip))

    def _apply_angle(self):
        a = math.radians(self.angle)
        rz = np.eye(4)
        rz[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
        m = rz @ FRD_TO_SCENE
        for item in [self.mesh_item, *self.axes]:
            if item is not None:
                item.setTransform(transform3d(m))
        self.update()


def _spin(lo, hi, step, decimals, suffix=""):
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setSingleStep(step)
    s.setDecimals(decimals)
    s.setSuffix(suffix)
    s.setKeyboardTracking(False)
    return s


class ModelManager(QDialog):
    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.c = controller
        self.lib = controller.library
        self.setWindowTitle("Керування 3D-моделями")
        self.resize(1080, 720)
        self.setModal(True)
        self.draft = self.lib.draft()
        self.registry = self.draft.registry()
        self.registry.specs = self.draft.specs  # the dialog edits the draft's specs in place
        self.original = {k: (s.center, s.scale_xyz) for k, s in self.draft.specs.items()}
        self.key = None
        self._loading = False
        self.valid = True

        root = QHBoxLayout(self)
        root.addLayout(self._left(), 0)
        root.addLayout(self._right(), 1)
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._spin_step)
        self._fill_list(self.c.default_model)

    # ----- building -----

    def _left(self):
        lay = QVBoxLayout()
        lay.addWidget(label("Моделі", heading=True))
        self.list = QListWidget()
        self.list.setIconSize(QSize(44, 44))
        self.list.setFixedWidth(270)
        self.list.currentItemChanged.connect(self._selected)
        lay.addWidget(self.list, 1)
        row = QHBoxLayout()
        self.add_btn = button("Додати", icon_name="plus")
        self.dup_btn = button("Дублювати", icon_name="copy")
        self.del_btn = button("Видалити", icon_name="trash-2")
        for b in (self.add_btn, self.dup_btn, self.del_btn):
            row.addWidget(b)
        lay.addLayout(row)
        self.add_btn.clicked.connect(self.on_add)
        self.dup_btn.clicked.connect(self.on_duplicate)
        self.del_btn.clicked.connect(self.on_delete)
        return lay

    def _right(self):
        lay = QVBoxLayout()
        head = QHBoxLayout()
        self.name = QLineEdit()
        self.name.editingFinished.connect(self._name_edited)
        head.addWidget(label("Назва", muted=True))
        head.addWidget(self.name, 1)
        lay.addLayout(head)
        self.preview = ModelPreview()
        lay.addWidget(self.preview, 1)
        play = QHBoxLayout()
        self.play = button("Програти обертання", icon_name="rotate-cw")
        self.play.setCheckable(True)
        self.play.toggled.connect(self._play_toggled)
        self.angle = QSlider(Qt.Horizontal)
        self.angle.setRange(0, 360)
        self.angle.valueChanged.connect(lambda v: self.preview.set_angle(v))
        play.addWidget(self.play)
        play.addWidget(self.angle, 1)
        lay.addLayout(play)
        self.note = QLabel("")
        self.note.setWordWrap(True)
        lay.addWidget(self.note)

        self.editor = QWidget()
        form = QVBoxLayout(self.editor)
        form.setContentsMargins(0, 0, 0, 0)
        form.addWidget(self._scale_box())
        form.addWidget(self._pivot_box())
        form.addWidget(self._orient_box())
        lay.addWidget(self.editor)

        btns = QHBoxLayout()
        btns.addStretch(1)
        self.cancel_btn = button("Скасувати", icon_name="x")
        self.save_btn = button("Зберегти", "primary", "save")
        btns.addWidget(self.cancel_btn)
        btns.addWidget(self.save_btn)
        lay.addLayout(btns)
        self.cancel_btn.clicked.connect(self.reject)
        self.save_btn.clicked.connect(self.on_save)
        return lay

    def _scale_box(self):
        box = Collapsible("Масштаб", True)
        row = QHBoxLayout()
        self.prop = QCheckBox("Пропорційно")
        self.prop.setChecked(True)
        self.scale = _spin(0.05, 20.0, 0.05, 2, " ×")
        self.scale_slider = QSlider(Qt.Horizontal)
        self.scale_slider.setRange(5, 500)
        row.addWidget(self.prop)
        row.addWidget(self.scale)
        row.addWidget(self.scale_slider, 1)
        self.axis_scale = [_spin(0.05, 20.0, 0.05, 2, " ×") for _ in range(3)]
        self.axis_labels = [label(ax, muted=True) for ax in "XYZ"]
        for lb, sp in zip(self.axis_labels, self.axis_scale, strict=True):
            row.addWidget(lb)
            row.addWidget(sp)
        box.body_layout.addLayout(row)
        self.prop.toggled.connect(self._prop_toggled)
        self.scale.valueChanged.connect(self._scale_changed)
        self.scale_slider.valueChanged.connect(lambda v: self.scale.setValue(v / 100))
        for sp in self.axis_scale:
            sp.valueChanged.connect(self._axis_scale_changed)
        return box

    def _pivot_box(self):
        box = Collapsible("Центр обертання", True)
        row = QHBoxLayout()
        self.pivot = [_spin(-1e6, 1e6, 0.5, 3) for _ in range(3)]
        for ax, sp in zip("XYZ", self.pivot, strict=True):
            row.addWidget(label(ax, muted=True))
            row.addWidget(sp)
            sp.valueChanged.connect(self._pivot_changed)
        box.body_layout.addLayout(row)
        row2 = QHBoxLayout()
        self.bbox_btn = button("По центру bbox", icon_name="crosshair")
        self.mass_btn = button("Центр мас (наближено)", icon_name="crosshair")
        self.reset_btn = button("Скинути", icon_name="undo-2")
        for b in (self.bbox_btn, self.mass_btn, self.reset_btn):
            row2.addWidget(b)
        box.body_layout.addLayout(row2)
        self.bbox_btn.clicked.connect(lambda: self._set_pivot(self._native_bbox_centre()))
        self.mass_btn.clicked.connect(lambda: self._set_pivot(self._native_centroid()))
        self.reset_btn.clicked.connect(self._reset_pivot)
        return box

    def _orient_box(self):
        box = Collapsible("Орієнтація", True)
        row = QHBoxLayout()
        self.fwd = QComboBox()
        self.up = QComboBox()
        for cb in (self.fwd, self.up):
            for k in AXIS_NAMES:
                cb.addItem(AXIS_TEXT[k], k)
        self.swap = button("Поміняти перед/зад", icon_name="arrow-left-right")
        row.addWidget(label("Вісь носа", muted=True))
        row.addWidget(self.fwd)
        row.addWidget(label("Вісь верху", muted=True))
        row.addWidget(self.up)
        row.addWidget(self.swap)
        box.body_layout.addLayout(row)
        row2 = QHBoxLayout()
        row2.addWidget(label("Повернути на 90°:", muted=True))
        self.rot = {}
        for key, text in (("yaw", "Рискання"), ("roll", "Крен"), ("pitch", "Тангаж")):
            b = button(text, icon_name="rotate-cw")
            b.clicked.connect(lambda _=False, k=key: self._quick_rotate(k))
            row2.addWidget(b)
            self.rot[key] = b
        row2.addStretch(1)
        box.body_layout.addLayout(row2)
        self.fwd.currentIndexChanged.connect(self._axes_changed)
        self.up.currentIndexChanged.connect(self._axes_changed)
        self.swap.clicked.connect(self._swap)
        return box

    # ----- list -----

    def _fill_list(self, select: str | None = None):
        self.list.blockSignals(True)
        self.list.clear()
        for key, spec in self.draft.specs.items():
            mesh = self.registry.mesh(key)
            suffix = (
                "  · вбудована"
                if spec.builtin
                else ("  · файл не читається" if spec.broken else "")
            )
            item = QListWidgetItem(thumbnail(mesh), f"{spec.display_name}{suffix}")
            item.setData(Qt.UserRole, key)
            item.setSizeHint(QSize(240, 54))
            self.list.addItem(item)
        self.list.blockSignals(False)
        keys = list(self.draft.specs)
        k = keys.index(select) if select in keys else 0
        self.list.setCurrentRow(k)
        self._selected(self.list.currentItem(), None)

    def _refresh_item(self, key):
        for i in range(self.list.count()):
            it = self.list.item(i)
            if it.data(Qt.UserRole) == key:
                spec = self.draft.specs[key]
                it.setIcon(thumbnail(self.registry.mesh(key)))
                it.setText(f"{spec.display_name}{'  · файл не читається' if spec.broken else ''}")

    def _selected(self, item, _prev):
        if item is None:
            return
        self.key = item.data(Qt.UserRole)
        self._load_editor()

    # ----- editor <-> spec -----

    @property
    def spec(self):
        return self.draft.specs[self.key]

    def _load_editor(self):
        spec = self.spec
        self._loading = True
        editable = not spec.builtin
        self.editor.setEnabled(editable)
        self.name.setEnabled(editable)
        self.name.setText(spec.display_name)
        s = spec.scale_xyz or (1.0, 1.0, 1.0)
        uniform = len({round(v, 6) for v in s}) == 1
        self.prop.setChecked(uniform)
        self.scale.setValue(s[0])
        self.scale_slider.setValue(round(s[0] * 100))
        for sp, v in zip(self.axis_scale, s, strict=True):
            sp.setValue(v)
        self._show_scale_mode(uniform)
        centre = spec.center if spec.center is not None else self._native_centroid()
        for sp, v in zip(self.pivot, centre, strict=True):
            sp.setValue(float(v))
        self.fwd.setCurrentIndex(AXIS_NAMES.index(spec.forward_axis or "+X"))
        self.up.setCurrentIndex(AXIS_NAMES.index(spec.up_axis or "-Z"))
        self._loading = False
        self.del_btn.setEnabled(editable)
        self.dup_btn.setEnabled(True)
        self._set_valid(True)
        self._rebuild_preview()
        if spec.builtin:
            self._note("Вбудовану модель не можна змінити або видалити. Зроби копію і редагуй її.",
                       "muted")  # fmt: skip
        elif spec.broken:
            self._note("Файл моделі не знайдено або він пошкоджений: показана заглушка.", "warn")
        else:
            self._note("", "muted")

    def _note(self, text, kind):
        color = {"muted": T.text_muted, "warn": T.warn, "danger": T.danger}[kind]
        self.note.setStyleSheet(f"color: {color};")
        self.note.setText(text)

    def _set_valid(self, ok: bool):
        self.valid = ok
        self.save_btn.setEnabled(ok)
        self.list.setEnabled(ok)
        for b in (self.add_btn, self.dup_btn, self.del_btn):
            b.setEnabled(ok and (b is not self.del_btn or not self.spec.builtin))

    def _rebuild_preview(self):
        self.registry.cache.pop(self.key, None)
        mesh = self.registry.mesh(self.key)
        self.preview.set_mesh(mesh)

    def _native_tris(self):
        try:
            return self.registry.native(self.key)[0]
        except (OSError, ValueError, KeyError):
            return np.zeros((1, 3, 3), np.float32)

    def _native_centroid(self):
        return tuple(float(x) for x in surface_centroid(self._native_tris()))

    def _native_bbox_centre(self):
        lo, hi = bbox(self._native_tris())
        return tuple(float(x) for x in (lo + hi) / 2)

    def _changed(self, **fields):
        """Write to the draft and redraw. Not while the editor is being filled."""
        if self._loading or self.spec.builtin:
            return
        self.draft.update(self.key, **fields)
        self._rebuild_preview()
        self._refresh_item(self.key)

    def _name_edited(self):
        text = self.name.text().strip()
        if not self._loading and not self.spec.builtin and text and text != self.spec.display_name:
            self.draft.update(self.key, display_name=text)
            self._refresh_item(self.key)

    # scale
    def _show_scale_mode(self, uniform: bool):
        self.scale.setVisible(uniform)
        self.scale_slider.setVisible(uniform)
        for sp, lb in zip(self.axis_scale, self.axis_labels, strict=True):
            sp.setVisible(not uniform)
            lb.setVisible(not uniform)

    def _prop_toggled(self, on):
        self._show_scale_mode(on)
        if self._loading:
            return
        if on:  # back to one factor: the nose axis value is kept
            v = self.axis_scale[0].value()
            self._loading = True
            self.scale.setValue(v)
            self.scale_slider.setValue(round(v * 100))
            self._loading = False
            self._changed(scale_xyz=(v, v, v))
        else:
            v = self.scale.value()
            self._loading = True
            for sp in self.axis_scale:
                sp.setValue(v)
            self._loading = False

    def _scale_changed(self, v):
        if self._loading:
            return
        self._loading = True
        self.scale_slider.setValue(round(v * 100))
        self._loading = False
        self._changed(scale_xyz=(v, v, v))

    def _axis_scale_changed(self, _):
        self._changed(scale_xyz=tuple(sp.value() for sp in self.axis_scale))

    # pivot
    def _pivot_changed(self, _):
        self._changed(center=tuple(sp.value() for sp in self.pivot))

    def _set_pivot(self, xyz):
        self._loading = True
        for sp, v in zip(self.pivot, xyz, strict=True):
            sp.setValue(float(v))
        self._loading = False
        self._changed(center=tuple(float(v) for v in xyz))

    def _reset_pivot(self):
        c0 = self.original.get(self.key, (None, None))[0]
        self._set_pivot(c0 if c0 is not None else self._native_centroid())

    # orientation
    def _axes_changed(self, _=None):
        if self._loading:
            return
        f, u = self.fwd.currentData(), self.up.currentData()
        try:
            axes_rotation(f, u)
        except ValueError:
            self._set_valid(False)
            self._note(COLLINEAR_TEXT, "danger")
            return
        self._set_valid(True)
        self._note("", "muted")
        self._changed(forward_axis=f, up_axis=u)

    def _swap(self):
        f = self.fwd.currentData()
        opposite = ("-" if f[0] == "+" else "+") + f[1]
        self.fwd.setCurrentIndex(AXIS_NAMES.index(opposite))

    def _quick_rotate(self, kind: str):
        """Turn the model by 90 degrees about a body axis (changes the nose / top axes)."""
        spec = self.spec
        try:
            r = axes_rotation(self.fwd.currentData(), self.up.currentData())
        except ValueError:
            return
        q = board_rotation(*{"roll": (90, 0, 0), "pitch": (0, 90, 0), "yaw": (0, 0, 90)}[kind])
        f, u = axes_from_matrix(np.round(q @ r))
        self._loading = True
        self.fwd.setCurrentIndex(AXIS_NAMES.index(f))
        self.up.setCurrentIndex(AXIS_NAMES.index(u))
        self._loading = False
        if not spec.builtin:
            self._changed(forward_axis=f, up_axis=u)

    # ----- spin -----

    def _play_toggled(self, on):
        if on:
            self.spin_timer.start(33)
        else:
            self.spin_timer.stop()

    def _spin_step(self):
        self.angle.setValue((self.angle.value() + 3) % 361)

    # ----- list buttons -----

    def on_add(self):
        path, _ = QFileDialog.getOpenFileName(self, "Додати модель", "", "STL (*.stl *.STL)")
        if path:
            self.add_file(path)

    def add_file(self, path) -> str | None:
        try:
            key = self.draft.add(Path(path))
        except (OSError, ValueError) as e:
            QMessageBox.warning(self, "Не вдалося додати модель",
                                f"Файл не схожий на STL або пошкоджений.\n\n{e}")  # fmt: skip
            return None
        self.original[key] = (self.draft.specs[key].center, self.draft.specs[key].scale_xyz)
        self._fill_list(key)
        return key

    def on_duplicate(self):
        key = self.draft.duplicate(self.key)
        self.original[key] = (self.draft.specs[key].center, self.draft.specs[key].scale_xyz)
        self._fill_list(key)

    def on_delete(self):
        if self.spec.builtin:
            return
        box = QMessageBox(self)
        box.setWindowTitle("Видалити модель")
        box.setText(f"Видалити «{self.spec.display_name}»? Копію файла в програмі буде стерто "
                    "після «Зберегти».")  # fmt: skip
        yes = box.addButton("Видалити", QMessageBox.DestructiveRole)
        box.addButton("Скасувати", QMessageBox.RejectRole)
        box.exec()
        if box.clickedButton() is yes:
            self.delete_current()

    def delete_current(self):
        self.draft.delete(self.key)
        self._fill_list()

    # ----- finish -----

    def on_save(self):
        if not self.valid:
            return
        registry = self.lib.commit(self.draft)
        self.c.set_registry(registry)
        self.accept()

    def reject(self):
        self.spin_timer.stop()
        self.draft.discard()
        super().reject()

    def accept(self):
        self.spin_timer.stop()
        super().accept()
