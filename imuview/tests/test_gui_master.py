"""The GUI side of the USB master transport (offscreen Qt, no hardware)."""

import errno
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6.QtCore import QSettings  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from imuview.gui.controller import AppController  # noqa: E402
from imuview.gui.errors import friendly  # noqa: E402
from imuview.gui.sessions import SOURCE_KINDS, make_source, wait_until  # noqa: E402
from imuview.gui.settings import Settings  # noqa: E402
from imuview.gui.toolbar import TRANSPORTS, TopBar  # noqa: E402
from imuview.netproto import encode_datagram  # noqa: E402
from imuview.pipeline import load_config  # noqa: E402
from imuview.protocol_mtdata2 import encode_frame, encode_imu_payload  # noqa: E402
from imuview.sources import MasterSerialSource, SerialSource  # noqa: E402


@pytest.fixture(scope="module")
def qapp():
    return QApplication.instance() or QApplication([])


@pytest.fixture()
def pty_pair():
    master, slave = os.openpty()
    yield master, os.ttyname(slave)
    os.close(master)
    os.close(slave)


def datagram(module_id, seq):
    return encode_datagram(module_id, seq, encode_frame(encode_imu_payload(acc=(0.0, 0.0, 1.0))))


def test_master_is_a_known_transport_and_the_old_ones_are_still_there():
    keys = [k for k, _ in TRANSPORTS]
    assert keys == ["master", "udp", "serial", "sim"]
    assert set(SOURCE_KINDS) == set(keys)


def test_make_source_builds_the_right_class(pty_pair):
    _, path = pty_pair
    config = load_config()
    src = make_source("master", config, serial_port=path)
    try:
        assert isinstance(src, MasterSerialSource)
    finally:
        src.close()
    src = make_source("serial", config, serial_port=path)
    try:
        assert type(src) is SerialSource  # the module-direct debug transport is unchanged
    finally:
        src.close()
    assert config["master"]["port"].startswith("/dev/")  # default port exists in the config


def test_toolbar_hands_the_port_to_the_master_transport(qapp):
    bar = TopBar()
    bar.set_transport("master", "/dev/ttyACM3")
    assert bar.connection() == ("master", {"serial_port": "/dev/ttyACM3"})
    bar.set_transport("udp", "6000")
    assert bar.connection() == ("udp", {"port": 6000})
    bar.set_transport("serial", "/dev/ttyUSB1")
    assert bar.connection() == ("serial", {"serial_port": "/dev/ttyUSB1"})


def test_port_choice_is_remembered_per_transport(tmp_path):
    s = Settings(QSettings(str(tmp_path / "s.ini"), QSettings.IniFormat))
    assert s.get("port/master") == "/dev/ttyACM0"
    s.set("port/master", "COM7")
    assert (
        Settings(QSettings(str(tmp_path / "s.ini"), QSettings.IniFormat)).get("port/master")
        == "COM7"
    )


def test_a_missing_master_port_gets_the_usual_friendly_message():
    exc = OSError(errno.ENOENT, "No such file or directory")
    msg, raw = friendly(exc, "master", "/dev/ttyACM0")
    assert "не знайдено" in msg and "/dev/ttyACM0" in msg
    msg, _ = friendly(OSError(errno.EACCES, "Permission denied"), "master", "/dev/ttyACM0")
    assert "dialout" in msg


def test_controller_shows_what_the_master_reports(pty_pair, tmp_path):
    master, path = pty_pair
    config = load_config()
    config["log_dir"] = str(tmp_path)
    c = AppController(config, models_dir="models")
    c.start_live("master", serial_port=path)
    try:
        assert c.conn_state == "connected"
        c.poll()
        assert "нічого не приходить" in c.message  # port open, master silent

        line = (
            "# id={i} mac=11:22:33:44:55:0{i} rx=5 bad=0 sent=5 drop=0 rssi=-{r} hb=ok up=9 "
            "uart={u} "
            "frames={f} badcs=0 tx=5 err=0 nack=0 qdrop=0 ch=1 rst=0 diag={d}\n"
        )
        os.write(master, b"# link out=usb slaves=2 queue=0 qdrop=0 out_err=0 bad_unknown=0\n")
        os.write(master, line.format(i=1, r=45, u=470, f=5, d="OK").encode())
        os.write(master, line.format(i=2, r=60, u=0, f=0, d="UART_SILENT").encode())
        for seq in range(10):
            os.write(master, datagram(1, seq))
        assert wait_until(lambda: c.module_ids() == [1], 5)
        c.poll()
        assert "#2" in c.message and "UART мовчить" in c.message  # the failing module is named
    finally:
        c.stop_live()


def test_healthy_master_reports_all_good(pty_pair, tmp_path):
    master, path = pty_pair
    config = load_config()
    config["log_dir"] = str(tmp_path)
    c = AppController(config, models_dir="models")
    c.start_live("master", serial_port=path)
    try:
        os.write(master, b"# link out=usb slaves=1 queue=0 qdrop=0 out_err=0 bad_unknown=0\n")
        os.write(
            master,
            b"# id=1 mac=11:22:33:44:55:01 rx=5 bad=0 sent=5 drop=0 rssi=-47 hb=ok up=9 uart=470 "
            b"frames=5 badcs=0 tx=5 err=0 nack=0 qdrop=0 ch=1 rst=0 diag=OK\n",
        )
        assert wait_until(lambda: (c.poll() or True) and "усе гаразд" in c.message, 5)
        assert "-47" in c.message
    finally:
        c.stop_live()
