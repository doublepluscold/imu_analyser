"""Widgets without OpenGL (the 3D view needs a real GL context, so it is checked by eye)."""

import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6.QtWidgets import QApplication  # noqa: E402
from test_gui_logic import record_session  # noqa: E402

from imuview.frames import quat_from_euler  # noqa: E402
from imuview.gui.calibration_page import CalibrationPage  # noqa: E402
from imuview.gui.controller import AppController  # noqa: E402
from imuview.gui.module_cards import ModuleStrip, project  # noqa: E402
from imuview.gui.plots_view import MiniEulerPlot, PlotsPanel  # noqa: E402
from imuview.gui.raw_view import RawPanel  # noqa: E402
from imuview.gui.scene_view import ARROW_MAX_M, arrow_points  # noqa: E402
from imuview.gui.timeline import SLIDER_STEPS, TimelineBar, fmt_time  # noqa: E402
from imuview.pipeline import load_config  # noqa: E402


@pytest.fixture(scope="module")
def qapp():
    return QApplication.instance() or QApplication([])


@pytest.fixture()
def controller(tmp_path, qapp):
    config = record_session(tmp_path / "s")
    c = AppController(config, models_dir="models")
    c.open_session(tmp_path / "s", background=False)
    c.poll()
    return c


def test_module_strip_builds_one_card_per_module_and_click_selects(controller, qapp):
    strip = ModuleStrip(controller)
    picked = []
    strip.selected.connect(picked.append)
    strip.refresh()
    assert list(strip.cards) == [1, 2, 3]
    strip.cards[3].clicked.emit(3)
    assert picked == [3]
    controller.set_active(3)
    strip.cards[3].grab()  # paints without error, also with another model
    controller.set_model("shahed", 3)
    strip.cards[3].grab()


def test_plots_follow_the_active_module_and_the_slider_time(controller, qapp):
    panel = PlotsPanel(controller)
    controller.player.seek(controller.player.t_start + 4.0)
    panel.refresh()
    x, y = panel.curves["accel"][2].getData()
    assert len(x) > 100 and x.max() <= 0.0 and x.min() >= -10.0 - 1e-6
    assert np.mean(y) == pytest.approx(-9.8, abs=0.5)  # body z reads -g, level
    controller.set_active(2)
    panel.refresh()
    assert len(panel.curves["euler"][0].getData()[0]) > 100
    mini = MiniEulerPlot(controller, 1)
    mini.refresh()
    assert len(mini.curves[0].getData()[0]) > 100


def test_gps_plot_hidden_without_gps(controller, qapp):
    panel = PlotsPanel(controller)
    panel.show()
    panel.refresh()
    assert panel.plots["gps_enu"].isVisible()
    controller.options.use_gps = False
    panel.refresh()
    assert not panel.plots["gps_enu"].isVisible()


def test_raw_panel_shows_blocks_and_hex(controller, qapp):
    panel = RawPanel(controller)
    controller.player.seek(controller.player.t_start + 2.0)
    panel.refresh()
    xdis = [panel.table.item(r, 0).text() for r in range(panel.table.rowCount())]
    assert {"0x2030", "0x4020", "0x8020", "0xE010"} <= set(xdis)
    assert "fa ff 36" in panel.hex.toPlainText()


def test_timeline_bar_drives_the_player(controller, qapp):
    bar = TimelineBar(controller)
    bar.refresh()
    assert bar.isEnabled()
    bar.moved(SLIDER_STEPS // 2)
    p = controller.player
    assert p.t == pytest.approx(p.t_start + (p.t_end - p.t_start) / 2, abs=1e-3)
    bar.toggle()
    assert p.playing
    bar.toggle()
    assert not p.playing
    bar.speed.setCurrentIndex(bar.speed.findData(4.0))
    assert p.speed == 4.0
    assert fmt_time(65.5) == "01:05.50"


def test_calibration_page_follows_the_wizard(controller, qapp):
    page = CalibrationPage(controller, lambda text: None)
    page.refresh()
    assert page.stack.currentIndex() == 0 and not page.start.isEnabled()  # no connection
    controller.live = object()  # a connection exists (the page only checks for one)
    page.refresh()
    assert page.start.isEnabled()
    from imuview.gui.wizard import CalibWizard

    controller.wizard = CalibWizard(1, [180, 0, 180])
    controller.mode = "calibration"
    controller.wizard.press_enter()
    page.refresh()
    assert page.stack.currentIndex() == 1
    assert page.items[0].state_ == "current" and page.items[1].state_ == "todo"
    assert not page.ready.isEnabled() and page.ready.text() == "Збираю…"
    assert page.items[0].text().endswith("Рівно")
    controller.live = None
    controller.wizard = None


def test_summary_page_lists_every_step(qapp, tmp_path):
    from test_wizard import ALIGN, Sensor, run_all

    from imuview.gui.wizard import CalibWizard

    c = AppController(load_config(), models_dir="models")
    wiz = CalibWizard(module_id=1, board_alignment_deg=ALIGN)
    run_all(wiz, Sensor(wiz))
    c.wizard, c.mode = wiz, "calibration"
    page = CalibrationPage(c, lambda text: None)
    page.refresh()
    assert page.stack.currentIndex() == 2
    assert page.table.rowCount() == 6 and page.sum_save.isEnabled()
    assert all(it.state_ == "done" for it in page.items)
    assert "точна" in page.sum_notes.text()  # the exactly-determined note, in Ukrainian
    # clicking a finished step does it again
    page._step_clicked(2)
    assert wiz.status().phase == "wait_enter" and wiz.status().step == 2
    assert wiz.result is None and not wiz.status().done_steps[2]


def test_thumbnail_projection_turns_with_the_model():
    pts = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 0.0]])  # nose point and centre
    a = project(pts, quat_from_euler(0, 0, 0))
    b = project(pts, quat_from_euler(0, 0, np.pi / 2))
    assert np.allclose(a[1], 0) and not np.allclose(a[0], b[0])
    assert np.allclose(np.linalg.norm(a[0]), np.linalg.norm(b[0]), rtol=0.6)


def test_arrow_points():
    assert arrow_points((0, 0, 0)).shape == (0, 3)
    pts = arrow_points((2.0, 0, 0))
    assert pts.shape == (5, 3) and np.allclose(pts[1], (2, 0, 0)) and np.allclose(pts[0], 0)
    long = arrow_points((100.0, 0, 0))
    assert np.linalg.norm(long[1]) == pytest.approx(ARROW_MAX_M)
