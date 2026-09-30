import errno
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6.QtCore import QSettings  # noqa: E402

from imuview.gui.controller import AppController  # noqa: E402
from imuview.gui.errors import friendly  # noqa: E402
from imuview.gui.settings import Settings  # noqa: E402
from imuview.pipeline import load_config  # noqa: E402


def settings_at(path):
    return Settings(QSettings(str(path), QSettings.IniFormat))


def test_options_survive_a_restart(tmp_path):
    ini = tmp_path / "s.ini"
    a = AppController(load_config(), models_dir="models")
    a.options.use_gps = False
    a.options.trail_seconds = 4.5
    a.options.subtract_gravity = False
    a.options.layout, a.options.view, a.options.record = "scene", "plots", False
    a.set_model("shahed", everywhere=True)
    s = settings_at(ini)
    s.store_from(a)
    s.set("transport", "udp")
    s.set("port/udp", "6000")
    s.sync()

    b = AppController(load_config(), models_dir="models")
    s2 = settings_at(ini)
    s2.load_into(b)
    o = b.options
    assert (o.use_gps, o.trail_seconds, o.subtract_gravity) == (False, 4.5, False)
    assert (o.layout, o.view, o.record) == ("scene", "plots", False)
    assert b.default_model == "shahed"
    assert s2.get("transport") == "udp" and s2.get("port/udp") == "6000"
    assert s2.get("port/serial") == "/dev/ttyUSB0"  # default for what was never set


def test_bad_saved_values_fall_back_to_defaults(tmp_path):
    s = settings_at(tmp_path / "s.ini")
    s.set("layout", "nonsense")
    s.set("view", "nonsense")
    s.set("model", "no-such-model")
    s.set("window/split", ["x", "y"])
    c = AppController(load_config(), models_dir="models")
    s.load_into(c)
    assert (c.options.layout, c.options.view, c.default_model) == ("single", "both", "quad")
    assert s.get_sizes("window/split") is None


def test_friendly_errors_hide_the_raw_text():
    msg, raw = friendly(OSError(errno.EBUSY, "Device or resource busy"), "serial", "/dev/ttyUSB0")
    assert "зайнятий" in msg and "Errno" not in msg and "Device or resource busy" in raw
    msg, _ = friendly(FileNotFoundError(errno.ENOENT, "No such file"), "serial", "COM9")
    assert "не знайдено" in msg and "COM9" in msg
    msg, _ = friendly(OSError(errno.EACCES, "Permission denied"), "serial", "/dev/ttyUSB0")
    assert "dialout" in msg
    msg, _ = friendly(OSError(errno.EADDRINUSE, "Address already in use"), "udp", "5005")
    assert "UDP" in msg and "5005" in msg
    msg, _ = friendly(ValueError("x"), "udp", "abc")
    assert "числом" in msg
    assert friendly(RuntimeError("boom"), "sim")[0] == "Не вдалося під'єднатися."
