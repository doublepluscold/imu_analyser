"""Logic of the model manager dialog (the GL preview is not drawn: the dialog is not shown)."""

import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6.QtWidgets import QApplication  # noqa: E402
from test_gui_logic import long_box, write_binary_stl  # noqa: E402

from imuview.gui.controller import AppController  # noqa: E402
from imuview.gui.library import ModelLibrary  # noqa: E402
from imuview.gui.model_manager import COLLINEAR_TEXT, ModelManager  # noqa: E402
from imuview.gui.models import bbox  # noqa: E402
from imuview.pipeline import load_config  # noqa: E402

USER_ROLE = 0x0100


@pytest.fixture(scope="module")
def qapp():
    return QApplication.instance() or QApplication([])


@pytest.fixture()
def setup(qapp, tmp_path):
    c = AppController(load_config(), models_dir="models")
    c.library = ModelLibrary(tmp_path / "data", seed_dir="models")
    c.set_registry(c.library.registry())
    src = tmp_path / "box.stl"
    write_binary_stl(src, long_box())
    return c, src, tmp_path / "data"


def select(dlg, key):
    for i in range(dlg.list.count()):
        if dlg.list.item(i).data(USER_ROLE) == key:
            dlg.list.setCurrentRow(i)
            return
    raise KeyError(key)


def test_builtin_is_locked_but_copyable(setup):
    c, _, _ = setup
    dlg = ModelManager(c)
    select(dlg, "quad")
    assert not dlg.editor.isEnabled() and not dlg.del_btn.isEnabled() and dlg.dup_btn.isEnabled()
    dlg.on_duplicate()
    assert dlg.key.startswith("quad-copy") and dlg.editor.isEnabled() and dlg.del_btn.isEnabled()
    dlg.reject()


def test_nothing_reaches_the_session_before_save(setup):
    c, src, data = setup
    before = set(c.registry.keys())
    dlg = ModelManager(c)
    key = dlg.add_file(src)
    dlg.scale.setValue(2.0)
    assert (data / "box.stl").exists() and key in dlg.draft.specs
    dlg.reject()  # cancel
    assert set(c.registry.keys()) == before and not (data / "box.stl").exists()
    assert key not in ModelLibrary(data).user

    dlg = ModelManager(c)
    key = dlg.add_file(src)
    dlg.on_save()
    assert key in c.registry.keys() and key in ModelLibrary(data).user


def test_scale_pivot_and_axes_edit_the_mesh_live(setup):
    c, src, _ = setup
    dlg = ModelManager(c)
    key = dlg.add_file(src)
    plain = dlg.registry.mesh(key).size()
    dlg.scale.setValue(2.0)
    assert dlg.registry.mesh(key).size()[0] == pytest.approx(2 * plain[0], rel=1e-4)
    assert dlg.scale_slider.value() == 200  # slider follows the spinbox
    dlg.prop.setChecked(False)
    assert all(sp.value() == pytest.approx(2.0) for sp in dlg.axis_scale)
    dlg.axis_scale[1].setValue(1.0)  # y (right) back to 1
    size = dlg.registry.mesh(key).size()
    assert size[1] == pytest.approx(plain[1], rel=1e-4)
    assert size[0] == pytest.approx(2 * plain[0], rel=1e-4)
    dlg.prop.setChecked(True)
    # pivot at the nose end: the mesh lies behind the origin
    lo, hi = bbox(dlg.registry.native(key)[0])
    dlg._set_pivot((hi[0], (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2))
    lo2, hi2 = bbox(dlg.registry.mesh(key).tris)
    assert hi2[0] == pytest.approx(0.0, abs=1e-3) and lo2[0] < -1
    dlg.bbox_btn.click()
    lo2, hi2 = bbox(dlg.registry.mesh(key).tris)
    assert lo2[0] == pytest.approx(-hi2[0], abs=1e-3)
    assert dlg.pivot[0].value() == pytest.approx((lo[0] + hi[0]) / 2, abs=1e-2)
    dlg.mass_btn.click()
    dlg.reset_btn.click()  # back to what it was when the model was added
    dlg.reject()


def test_collinear_axes_block_saving(setup):
    c, src, _ = setup
    dlg = ModelManager(c)
    dlg.add_file(src)
    dlg.fwd.setCurrentIndex(0)  # +X
    dlg.up.setCurrentIndex(1)  # -X: same line
    assert not dlg.valid and not dlg.save_btn.isEnabled()
    assert COLLINEAR_TEXT in dlg.note.text()
    assert not dlg.list.isEnabled()  # cannot switch away with an invalid edit
    dlg.up.setCurrentIndex(4)  # +Z
    assert dlg.valid and dlg.save_btn.isEnabled()
    dlg.reject()


def test_swap_and_quick_rotation(setup):
    c, src, _ = setup
    dlg = ModelManager(c)
    dlg.add_file(src)  # nose +X, top +Z
    assert (dlg.spec.forward_axis, dlg.spec.up_axis) == ("+X", "+Z")
    dlg.swap.click()
    assert dlg.spec.forward_axis == "-X"
    dlg.swap.click()
    dlg._quick_rotate("yaw")  # 90 deg about the top axis: the nose now points along a side axis
    assert dlg.spec.forward_axis in ("+Y", "-Y") and dlg.spec.up_axis == "+Z"
    for _ in range(3):
        dlg._quick_rotate("yaw")
    assert (dlg.spec.forward_axis, dlg.spec.up_axis) == ("+X", "+Z")  # four turns: back
    dlg._quick_rotate("roll")
    assert dlg.spec.up_axis in ("+Y", "-Y") and dlg.spec.forward_axis == "+X"
    dlg.reject()


def test_delete_removes_the_model_only_after_save(setup):
    c, _, data = setup
    scout_file = data / c.library.user["scout"].file
    dlg = ModelManager(c)
    select(dlg, "scout")
    dlg.delete_current()
    assert "scout" not in dlg.draft.specs
    dlg.reject()
    assert "scout" in c.registry.keys() and scout_file.exists()
    dlg = ModelManager(c)
    select(dlg, "scout")
    dlg.delete_current()
    dlg.on_save()
    assert "scout" not in c.registry.keys() and not scout_file.exists()
    c.default_model = "scout"
    c.set_registry(c.library.registry())
    assert c.default_model == "quad"  # a removed model falls back


def test_rename_and_bad_stl_message(setup, monkeypatch):
    c, src, tmp = setup
    dlg = ModelManager(c)
    key = dlg.add_file(src)
    dlg.name.setText("Мій дрон")
    dlg._name_edited()
    assert dlg.draft.specs[key].display_name == "Мій дрон"
    shown = []
    from PySide6.QtWidgets import QMessageBox

    monkeypatch.setattr(QMessageBox, "warning", lambda *a, **k: shown.append(a[2]))
    bad = tmp / "bad.stl"
    bad.write_bytes(b"not stl")
    assert dlg.add_file(bad) is None and shown
    dlg.reject()


def test_thumbnail_is_drawn_for_every_model(setup):
    from imuview.gui.model_manager import thumbnail

    c, _, _ = setup
    for key in c.registry.keys():
        assert not thumbnail(c.registry.mesh(key)).isNull()
    assert np.isfinite(c.registry.mesh("shahed").tris).all()
