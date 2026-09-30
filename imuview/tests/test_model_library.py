import json

import numpy as np
import pytest
from test_gui_logic import long_box, write_ascii_stl, write_binary_stl

from imuview.frames import board_rotation
from imuview.gui.library import ModelLibrary
from imuview.gui.models import (
    ModelRegistry,
    ModelSpec,
    axes_from_matrix,
    axes_rotation,
    bbox,
    prepare,
)


@pytest.fixture()
def lib(tmp_path):
    return ModelLibrary(tmp_path / "data" / "models")


def test_axes_rotation_makes_a_right_handed_body_frame():
    r = axes_rotation("+X", "-Z")  # already FRD
    assert np.allclose(r, np.eye(3))
    for f in ("+X", "-X", "+Y", "-Y", "+Z", "-Z"):
        for u in ("+X", "-X", "+Y", "-Y", "+Z", "-Z"):
            if f[1] == u[1]:
                with pytest.raises(ValueError, match="one line"):
                    axes_rotation(f, u)
                continue
            r = axes_rotation(f, u)
            assert np.allclose(r @ r.T, np.eye(3)) and np.linalg.det(r) == pytest.approx(1.0)
            assert axes_from_matrix(r) == (f, u)


def test_axes_reproduce_the_old_rotation_deg_models():
    # shahed: nose +Z, top +X; scout: nose -Y, top +Z (see models/models.toml)
    assert np.allclose(axes_rotation("+Z", "+X"), board_rotation(0, 90, 0))
    assert np.allclose(axes_rotation("-Y", "+Z"), board_rotation(180, 0, -90))
    with pytest.raises(ValueError):
        axes_from_matrix(board_rotation(0, 0, 30))


def test_first_run_seeds_the_shipped_stl_models_identically(lib, tmp_path):
    lib2 = ModelLibrary(tmp_path / "fresh", seed_dir="models")
    assert {"shahed", "scout"} <= set(lib2.user)
    assert lib2.user["shahed"].display_name == "Шахед"
    assert (lib2.user["shahed"].forward_axis, lib2.user["shahed"].up_axis) == ("+Z", "+X")
    assert (tmp_path / "fresh" / "shahed.stl").exists()
    old = ModelRegistry("models")
    new = lib2.registry()
    for key in ("shahed", "scout"):
        assert np.allclose(old.mesh(key).tris, new.mesh(key).tris, atol=1e-5)
    # second start reads models.json instead of seeding again
    again = ModelLibrary(tmp_path / "fresh", seed_dir="models")
    assert set(again.user) == set(lib2.user)
    data = json.loads((tmp_path / "fresh" / "models.json").read_text())
    entry = next(m for m in data["models"] if m["id"] == "shahed")
    assert set(entry) >= {
        "id",
        "name",
        "file",
        "builtin",
        "scale",
        "pivot",
        "forward_axis",
        "up_axis",
    }


def test_add_copies_the_file_and_nothing_changes_before_commit(lib, tmp_path):
    src = tmp_path / "plane.stl"
    write_binary_stl(src, long_box())
    draft = lib.draft()
    key = draft.add(src)
    assert (lib.dir / "plane.stl").exists() and key in draft.specs and key not in lib.user
    assert ModelLibrary(lib.dir).user == {}  # not saved yet
    draft.discard()
    assert not (lib.dir / "plane.stl").exists()  # cancelled: the copy is gone again

    draft = lib.draft()
    key = draft.add(src, "Мій літак")
    reg = lib.commit(draft)
    assert key in reg.keys() and reg.names()[key] == "Мій літак"
    src.unlink()  # the original is not needed any more
    assert np.isfinite(ModelLibrary(lib.dir).registry().mesh(key).tris).all()


def test_add_rejects_files_that_are_not_stl(lib, tmp_path):
    bad = tmp_path / "x.stl"
    bad.write_bytes(b"hello")
    draft = lib.draft()
    with pytest.raises(ValueError):
        draft.add(bad)
    assert not list(lib.dir.glob("*.stl")) and draft.keys() == ["quad", "board"]


def test_ascii_stl_can_be_added(lib, tmp_path):
    src = tmp_path / "a.stl"
    write_ascii_stl(src, long_box())
    draft = lib.draft()
    assert draft.registry().mesh(draft.add(src)).size().max() == pytest.approx(4.0, rel=1e-4)


def test_builtin_models_are_protected_but_can_be_copied(lib):
    draft = lib.draft()
    with pytest.raises(ValueError):
        draft.delete("quad")
    with pytest.raises(ValueError):
        draft.update("quad", scale_xyz=(2, 2, 2))
    copy_key = draft.duplicate("quad")
    assert draft.specs[copy_key].base == "quad" and not draft.specs[copy_key].builtin
    draft.update(copy_key, scale_xyz=(2.0, 1.0, 1.0))
    reg = lib.commit(draft)
    big, plain = reg.mesh(copy_key).size(), reg.mesh("quad").size()
    assert big[0] == pytest.approx(2 * plain[0], rel=1e-3)  # only the nose axis grew


def test_duplicate_of_a_file_model_has_its_own_file_and_delete_removes_it(lib, tmp_path):
    src = tmp_path / "p.stl"
    write_binary_stl(src, long_box())
    d = lib.draft()
    k = d.add(src)
    lib.commit(d)
    d = lib.draft()
    k2 = d.duplicate(k)
    assert d.specs[k2].file != d.specs[k].file
    d.delete(k)
    lib.commit(d)
    assert not (lib.dir / f"{src.stem}.stl").exists() and (lib.dir / d.specs[k2].file).exists()


def test_pivot_scale_and_orientation_change_the_prepared_mesh():
    tris = long_box()  # 6 x 2 x 1, centre at (10, 5, 3)
    base = ModelSpec("m", "m", forward_axis="+X", up_axis="+Z", center=(10.0, 5.0, 3.0))
    assert np.allclose(bbox(prepare(tris, base))[0] + bbox(prepare(tris, base))[1], 0, atol=1e-5)
    moved = ModelSpec(
        "m", "m", forward_axis="+X", up_axis="+Z", center=(13.0, 5.0, 3.0)
    )  # nose end
    lo, hi = bbox(prepare(tris, moved))
    assert lo[0] == pytest.approx(-4.0, abs=1e-4) and hi[0] == pytest.approx(0.0, abs=1e-4)
    flipped = ModelSpec("m", "m", forward_axis="-X", up_axis="+Z", center=(13.0, 5.0, 3.0))
    lo, hi = bbox(prepare(tris, flipped))  # nose and tail swapped: the pivot is now at the tail end
    assert lo[0] == pytest.approx(0.0, abs=1e-4) and hi[0] == pytest.approx(4.0, abs=1e-4)
    wide = ModelSpec("m", "m", forward_axis="+X", up_axis="+Z", scale_xyz=(1, 2, 1))
    assert np.ptp(prepare(tris, wide).reshape(-1, 3), axis=0)[1] == pytest.approx(
        2 * np.ptp(prepare(tris, base).reshape(-1, 3), axis=0)[1], rel=1e-4
    )


def test_bad_files_never_crash_loading(tmp_path):
    d = tmp_path / "models"
    d.mkdir()
    (d / "models.json").write_text("{ this is not json")
    lib = ModelLibrary(d, seed_dir="models")
    assert (d / "models.json.bad").exists() and "shahed" in lib.user  # started again from seed

    (d / "models.json").write_text(
        json.dumps(
            {
                "models": [
                    {
                        "id": "ok",
                        "name": "Ok",
                        "file": "gone.stl",
                        "scale": [1, 1, 1],
                        "pivot": None,
                        "forward_axis": "+X",
                        "up_axis": "+Z",
                    },
                    {
                        "id": "weird",
                        "name": "Weird",
                        "file": "gone.stl",
                        "scale": "big",
                        "pivot": [1, "x"],
                        "forward_axis": "+X",
                        "up_axis": "+X",
                    },
                    {"name": "no id"},
                    "junk",
                    {"id": "quad", "name": "fake builtin"},
                ]
            }
        )
    )
    lib = ModelLibrary(d)
    assert set(lib.user) == {"ok", "weird"} and len(lib.problems) == 3
    w = lib.user["weird"]
    assert (w.forward_axis, w.up_axis) == ("+X", "+Z") and w.scale_xyz == (1.0, 1.0, 1.0)
    assert w.center is None
    reg = lib.registry()
    mesh = reg.mesh("ok")  # the file is missing: a placeholder, not a crash
    assert reg.specs["ok"].broken and len(mesh.tris) == 12
    assert reg.mesh("quad").spec.builtin


def test_registry_versions_differ(lib):
    assert lib.registry().version != lib.registry().version
