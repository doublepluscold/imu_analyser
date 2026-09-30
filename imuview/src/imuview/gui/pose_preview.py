"""Live 3D preview for the calibration page: the module as it is now (solid) and the orientation
the current step asks for (translucent ghost, green when they match). Imports Qt."""

import pyqtgraph.opengl as gl
from PySide6.QtGui import QVector3D

from . import theme
from .gl_common import LIT, mesh_data, register_lit_shader, transform3d
from .models import pose_matrix
from .theme import T
from .wizard import MATCH_DEG, target_quat, up_error_deg


class PoseView(gl.GLViewWidget):
    def __init__(self, controller, parent=None):
        register_lit_shader()
        super().__init__(parent)
        self.c = controller
        self.key = None
        self.now = self.ghost = None
        self.matched = None
        self.setBackgroundColor(T.bg1)
        self.setMinimumSize(320, 260)
        grid = gl.GLGridItem(size=QVector3D(10, 10, 1), color=(*theme.rgb(T.text_muted), 50))
        grid.setSpacing(1, 1, 1)
        self.addItem(grid)
        self.setCameraPosition(distance=9, elevation=24, azimuth=-125)
        self.opts["center"] = QVector3D(0, 0, 0)

    def _build(self, key):
        for it in (self.now, self.ghost):
            if it is not None:
                self.removeItem(it)
        mesh = self.c.registry.mesh(key)
        md = mesh_data(mesh.tris, mesh.face_colors if mesh.spec.file is None else None)
        color = (*mesh.spec.color, 1.0) if mesh.spec.file else (1, 1, 1, 1)
        self.now = gl.GLMeshItem(
            meshdata=md, smooth=False, shader=LIT, color=color, glOptions="opaque"
        )
        ghost_md = gl.MeshData(vertexes=mesh.tris)
        self.ghost = gl.GLMeshItem(
            meshdata=ghost_md,
            smooth=False,
            shader=LIT,
            color=theme.rgbf(T.info, 0.35),
            glOptions="translucent",
        )
        self.addItem(self.ghost)
        self.addItem(self.now)
        self.matched = None

    def refresh(self, pose_name: str | None) -> bool | None:
        """Draw for the given step's pose. Returns True when placed right, None without data."""
        c = self.c
        if c.active is None:
            return None
        key = (c.model_of(c.active), c.registry.version)
        if key != self.key:
            self._build(key[0])
            self.key = key
        pose = c.store.pose_at(c.active, c.view_time(), use_gps=False)
        if pose is None or pose_name is None:
            return None
        self.now.setTransform(transform3d(pose_matrix(pose.q)))
        self.ghost.setTransform(transform3d(pose_matrix(target_quat(pose_name, pose.q))))
        ok = up_error_deg(pose.q, pose_name) < MATCH_DEG
        if ok != self.matched:
            self.matched = ok
            self.ghost.setColor(theme.rgbf(T.ok if ok else T.info, 0.45 if ok else 0.35))
        self.update()
        return ok
