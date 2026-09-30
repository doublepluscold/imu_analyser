"""OpenGL helpers shared by the views. Imports Qt."""

import os
import sys

import numpy as np
from PySide6.QtCore import Qt


def configure_platform():
    """Call BEFORE importing pyqtgraph.opengl / creating the QApplication.

    On Linux under Wayland, PyOpenGL and Qt pick different GL backends ("no valid context") unless
    the X11 (xcb) plugin and GLX are forced. Windows needs nothing."""
    if sys.platform.startswith("linux"):
        os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
        os.environ.setdefault("PYOPENGL_PLATFORM", "glx")


LIT = "lit"


def register_lit_shader():
    """pyqtgraph's 'shaded' puts the light behind the camera, so faces that look at you are dark.
    'lit' is the same shader with the light above and in front, and lit from both sides."""
    from pyqtgraph.opengl import shaders

    if LIT in shaders.ShaderProgram.names:
        return
    shaders.ShaderProgram(LIT, [
        shaders.VertexShader("""
            uniform mat4 u_mvp;
            uniform mat3 u_normal;
            attribute vec4 a_position;
            attribute vec3 a_normal;
            attribute vec4 a_color;
            varying vec4 v_color;
            varying vec3 v_normal;
            void main() {
                v_normal = normalize(u_normal * a_normal);
                v_color = a_color;
                gl_Position = u_mvp * a_position;
            }
        """),
        shaders.FragmentShader("""
            #ifdef GL_ES
            precision mediump float;
            #endif
            varying vec4 v_color;
            varying vec3 v_normal;
            void main() {
                float p = abs(dot(v_normal, normalize(vec3(0.35, 0.6, 0.8))));
                vec3 rgb = v_color.rgb * (0.5 + 0.5 * p);
                gl_FragColor = vec4(rgb, v_color.a);
            }
        """),
    ])  # fmt: skip


def mesh_data(tris, face_colors=None):
    """MeshData for a triangle soup. Per-face colours go in as per-vertex colours: with
    faceColors pyqtgraph drew most faces of a multi-colour model black."""
    import pyqtgraph.opengl as gl

    if face_colors is None:
        return gl.MeshData(vertexes=tris)
    vertex_colors = np.repeat(np.asarray(face_colors, np.float32)[:, None, :], 3, axis=1)
    return gl.MeshData(vertexes=tris, vertexColors=vertex_colors)


def transform3d(matrix: np.ndarray):
    from pyqtgraph import Transform3D

    return Transform3D(*np.asarray(matrix, float).ravel().tolist())


def project_point(view, xyz):
    """Pixel position (x, y) in a GLViewWidget of a 3D point, or None if it is behind the camera.
    Used for text labels: QLabels over the view (GLTextItem does not draw reliably)."""
    from PySide6.QtGui import QVector4D

    region = view.getViewport()
    m = view.projectionMatrix(region, region) * view.viewMatrix()
    p = m.map(QVector4D(float(xyz[0]), float(xyz[1]), float(xyz[2]), 1.0))
    if p.w() <= 1e-9:
        return None
    return (p.x() / p.w() + 1) / 2 * view.width(), (1 - p.y() / p.w()) / 2 * view.height()


def overlay_label(parent, color: str):
    from PySide6.QtWidgets import QLabel

    lb = QLabel(parent)
    lb.setStyleSheet(f"color: {color}; background: transparent; font-weight: 600;")
    lb.setAttribute(Qt.WA_TransparentForMouseEvents)
    lb.hide()
    return lb


def place_label(view, label, xyz, dx=0, dy=-8):
    pos = project_point(view, xyz)
    if pos is None:
        label.hide()
        return
    label.adjustSize()
    label.move(int(pos[0] - label.width() / 2 + dx), int(pos[1] + dy - label.height() / 2))
    label.show()


# one colour per module, used for trails, labels and (in the shared scene) the models
MODULE_COLORS = [
    (0.95, 0.35, 0.30), (0.30, 0.65, 0.95), (0.35, 0.85, 0.45), (0.95, 0.75, 0.25),
    (0.75, 0.45, 0.95), (0.30, 0.85, 0.85), (0.95, 0.55, 0.75), (0.75, 0.75, 0.75),
]  # fmt: skip


def module_color(module_id: int):
    return MODULE_COLORS[module_id % len(MODULE_COLORS)]
