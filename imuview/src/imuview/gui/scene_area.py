"""The big 3D area: one SceneView (single module / one shared scene) or a grid of them."""

from PySide6.QtWidgets import QGridLayout, QStackedWidget, QVBoxLayout, QWidget

from .plots_view import MiniEulerPlot
from .scene_view import SceneView
from .widgets import label


class GridArea(QWidget):
    """One 3D view (and a small euler plot) per module, side by side."""

    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.grid = QGridLayout(self)
        self.grid.setContentsMargins(0, 0, 0, 0)
        self.cells: dict[int, tuple] = {}

    def refresh(self):
        c = self.controller
        ids = c.module_ids()
        if list(self.cells) != ids:
            for _, view, plot, title in self.cells.values():
                for w in (view, plot, title):
                    w.setParent(None)
                    w.deleteLater()
            self.cells = {}
            cols = 1 if len(ids) <= 1 else 2 if len(ids) <= 4 else 3
            for k, mid in enumerate(ids):
                cell = QWidget()
                box = QVBoxLayout(cell)
                box.setContentsMargins(2, 2, 2, 2)
                title = label(c.name(mid), muted=True)
                view = SceneView(c)
                plot = MiniEulerPlot(c, mid)
                for w in (title, view, plot):
                    box.addWidget(w)
                box.setStretch(1, 1)
                self.grid.addWidget(cell, k // cols, k % cols)
                self.cells[mid] = (cell, view, plot, title)
        for mid, (_, view, plot, title) in self.cells.items():
            view.set_modules([mid])
            view.refresh()
            plot.refresh()
            title.setText(f"{c.name(mid)}{'' if c.alive(mid) else '  (немає даних)'}")


class SceneArea(QStackedWidget):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.view = SceneView(controller)
        self.grid = GridArea(controller)
        self.addWidget(self.view)
        self.addWidget(self.grid)

    def refresh(self):
        c = self.controller
        layout = c.options.layout
        if layout == "grid":
            self.setCurrentWidget(self.grid)
            self.grid.refresh()
            return
        self.setCurrentWidget(self.view)
        self.view.show_labels = layout == "scene"
        self.view.set_modules(c.shown_modules())
        self.view.refresh()
