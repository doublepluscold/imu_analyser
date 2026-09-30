"""3D view: one or several modules, each a model with orientation, optional GPS movement,
a fading trail (ghost copies) and, without GPS, the force arrow.

SceneView only draws what AppController/Store say; it holds no data of its own.
"""

import numpy as np
import pyqtgraph.opengl as gl
from PySide6.QtGui import QVector3D

from . import theme
from .gl_common import (
    LIT,
    mesh_data,
    module_color,
    overlay_label,
    place_label,
    register_lit_shader,
    transform3d,
)
from .models import pose_matrix
from .theme import T

GHOSTS = 8  # copies along the trail; oldest is the faintest
ARROW_M_PER_MS2 = 0.3  # arrow length per m/s^2 of acceleration
ARROW_MAX_M = 4.0
ROW_SPACING_M = 6.0  # models without GPS are put in a row in the shared scene


def arrow_points(vec, head=0.25) -> np.ndarray:
    """Polyline for an arrow from the origin to vec (body frame): shaft and two head strokes."""
    v = np.asarray(vec, float)
    n = float(np.linalg.norm(v))
    if n < 1e-6:
        return np.zeros((0, 3))
    if n > ARROW_MAX_M:
        v, n = v * ARROW_MAX_M / n, ARROW_MAX_M
    d = v / n
    side = np.cross(d, [0.0, 0.0, 1.0])
    if np.linalg.norm(side) < 1e-3:
        side = np.cross(d, [0.0, 1.0, 0.0])
    side /= np.linalg.norm(side)
    h = min(head, 0.4 * n)
    back = v - d * h
    return np.array([[0, 0, 0], v, back + side * h * 0.5, v, back - side * h * 0.5])


class Actor:
    """Everything drawn for one module."""

    def __init__(self, view, module_id, mesh, tint, trail_tint):
        self.view, self.module_id, self.key = view, module_id, mesh.spec.key
        self.version = view.controller.registry.version
        self.mesh = mesh
        color = (*tint, 1.0)
        md = mesh_data(mesh.tris, mesh.face_colors if mesh.spec.file is None else None)
        self.main = gl.GLMeshItem(
            meshdata=md, smooth=False, shader=LIT, color=color, glOptions="opaque"
        )
        base = (*trail_tint, 1.0)  # ghosts and trail in the module's colour
        self.ghosts = []
        for i in range(GHOSTS):
            alpha = 0.06 + 0.30 * (i + 1) / GHOSTS
            g = gl.GLMeshItem(meshdata=gl.MeshData(vertexes=mesh.tris), smooth=False, shader=LIT,
                              color=(*base[:3], alpha), glOptions="translucent")  # fmt: skip
            g.setVisible(False)
            self.ghosts.append(g)
        self.trail = gl.GLLinePlotItem(width=2, antialias=True, color=(*base[:3], 0.9))
        self.arrow = gl.GLLinePlotItem(
            width=4, antialias=True, color=theme.rgbf(T.accent), mode="line_strip"
        )
        self.label = overlay_label(view, T.text)
        self.items = [*self.ghosts, self.main, self.trail, self.arrow]
        for it in self.items:
            view.addItem(it)
        self.hide()

    def hide(self):
        for it in self.items:
            it.setVisible(False)
        self.label.hide()

    def remove(self):
        for it in self.items:
            self.view.removeItem(it)
        self.label.deleteLater()


class SceneView(gl.GLViewWidget):
    def __init__(self, controller, parent=None, show_labels=False):
        register_lit_shader()
        super().__init__(parent)
        self.controller = controller
        self.show_labels = show_labels
        self.actors: dict[int, Actor] = {}
        self.setBackgroundColor(T.bg1)
        self.setMinimumSize(200, 150)
        self.grid_spacing = None
        self._far = False
        self.grid = None
        self._set_grid(1.0)
        self.setCameraPosition(distance=11, elevation=22, azimuth=-125)

    def _set_grid(self, spacing):
        if self.grid_spacing == spacing:
            return
        if self.grid is not None:
            self.removeItem(self.grid)
        size = 16 if spacing <= 1 else 160
        self.grid = gl.GLGridItem(
            size=QVector3D(size, size, 1), color=(*theme.rgb(T.text_muted), 50)
        )
        self.grid.setSpacing(spacing, spacing, 1)
        self.addItem(self.grid)
        self.grid_spacing = spacing

    def set_modules(self, module_ids: list[int]):
        """Which modules this view draws; actors are (re)built when the model of one changes."""
        c = self.controller
        for mid in list(self.actors):
            stale = mid in self.actors and (
                self.actors[mid].key != c.model_of(mid)
                or self.actors[mid].version != c.registry.version
            )
            if mid not in module_ids or stale:
                self.actors.pop(mid).remove()
        for mid in module_ids:
            if mid not in self.actors:
                mesh = c.registry.mesh(c.model_of(mid))
                tint = module_color(mid) if c.options.layout == "scene" else mesh.spec.color
                self.actors[mid] = Actor(self, mid, mesh, tint, module_color(mid))

    def refresh(self):
        """Move everything to where the data says it is now."""
        c = self.controller
        ids = list(self.actors)
        poses = {mid: c.pose(mid) for mid in ids}
        anchor, any_gps = [], False
        for k, mid in enumerate(ids):
            a, pose = self.actors[mid], poses[mid]
            if pose is None:
                a.hide()
                continue
            if pose.position is not None:
                pos, any_gps = np.array(pose.position), True
            else:
                row = (k - (len(ids) - 1) / 2) * ROW_SPACING_M if len(ids) > 1 else 0.0
                pos = np.array([row, 0.0, 0.0])
            anchor.append(pos)
            m = pose_matrix(pose.q, pos)
            a.main.setVisible(True)
            a.main.setTransform(transform3d(m))
            gps_now = pose.position is not None
            self._ghosts(a, mid, pose, gps_now)
            self._arrow(a, pose, m, gps_now)
            if self.show_labels:
                a.label.setText(c.name(mid))
                place_label(self, a.label, pos + [0, 0, 2.6])
            else:
                a.label.hide()
        self._set_grid(10.0 if any_gps else 1.0)
        if any_gps != self._far:  # zoom out for GPS movement, back in without
            self._far = any_gps
            self.opts["distance"] = 26 if any_gps else 11
        if len(anchor) > 1:  # keep every module in view
            span = float(np.max(np.linalg.norm(np.array(anchor) - np.mean(anchor, axis=0), axis=1)))
            self.opts["distance"] = max(self.opts["distance"], 2.6 * span)
        if anchor:
            centre = np.mean(anchor, axis=0)
            self.opts["center"] = QVector3D(*centre)
            snap = 10.0 if any_gps else 1.0
            self.grid.resetTransform()
            self.grid.translate(round(centre[0] / snap) * snap, round(centre[1] / snap) * snap, 0)
        self.update()

    def _ghosts(self, a: Actor, mid, pose, gps_now):
        opt = self.controller.options
        trail = []
        if gps_now and opt.show_trail and opt.trail_seconds > 0:
            t = pose.t
            trail = self.controller.store.trail(mid, t, opt.trail_seconds, GHOSTS + 1)[:-1]
        for i, g in enumerate(a.ghosts):
            k = i - (GHOSTS - len(trail))  # the newest ghost slots get the newest samples
            if 0 <= k < len(trail):
                g.setVisible(True)
                g.setTransform(transform3d(pose_matrix(trail[k][2], trail[k][1])))
            else:
                g.setVisible(False)
        if len(trail) > 1:
            a.trail.setVisible(True)
            a.trail.setData(pos=np.array([p for _, p, _ in trail] + [pose.position]))
        else:
            a.trail.setVisible(False)

    def _arrow(self, a: Actor, pose, m, gps_now):
        opt = self.controller.options
        vec = pose.free_accel if opt.subtract_gravity else pose.accel_body
        if gps_now or vec is None:
            a.arrow.setVisible(False)
            return
        pts = arrow_points(np.asarray(vec) * ARROW_M_PER_MS2)
        if not len(pts):
            a.arrow.setVisible(False)
            return
        a.arrow.setVisible(True)
        a.arrow.setData(pos=pts)
        a.arrow.setTransform(transform3d(m))
