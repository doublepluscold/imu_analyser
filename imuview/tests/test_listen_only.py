"""Nothing in this project may send bytes to the device (OTA bootloader on the same USART)."""

import os
import re
import threading
import time
from pathlib import Path

import pytest
import serial

from imuview.cli import TimeLimit
from imuview.messages import ImuSample
from imuview.pipeline import Pipeline, load_config
from imuview.protocol_mtdata2 import encode_frame, encode_imu_payload
from imuview.sources import SerialSource

SRC = Path(__file__).parent.parent / "src"
FORBIDDEN = re.compile(r"\.(write|writelines|send_break)\s*\(|break_condition|sendBreak")


def forbid(*args, **kwargs):
    raise AssertionError("something tried to send to the serial port")


@pytest.fixture
def no_tx(monkeypatch):
    # set on serial.Serial itself: shadows every inherited version, subclasses included
    for name in ("write", "writelines", "send_break", "sendBreak"):
        monkeypatch.setattr(serial.Serial, name, forbid, raising=False)
    monkeypatch.setattr(serial.Serial, "break_condition", property(lambda s: False, forbid))


class SpySerial(serial.Serial):
    """Real pyserial on a pty, recording the settings in force when open() is called."""

    settings_at_open = None

    def open(self):
        SpySerial.settings_at_open = {
            "dtr": self.dtr, "rts": self.rts, "xonxoff": self.xonxoff,
            "rtscts": self.rtscts, "dsrdtr": self.dsrdtr, "baudrate": self.baudrate,
        }  # fmt: skip
        super().open()


def test_serial_source_has_no_way_to_send():
    public = {n for n in dir(SerialSource) if not n.startswith("_")}
    assert not public & {"write", "writelines", "send", "send_break", "ser", "port_object"}


def test_live_path_on_a_pty_never_sends(no_tx, tmp_path):
    master, slave = os.openpty()
    try:
        source = SerialSource(os.ttyname(slave), 115200, serial_cls=SpySerial)
        assert SpySerial.settings_at_open == {
            "dtr": False, "rts": False, "xonxoff": False,
            "rtscts": False, "dsrdtr": False, "baudrate": 115200,
        }  # fmt: skip
        frames = b"".join(encode_frame(encode_imu_payload(acc=(0, 0, i))) for i in range(50))

        def device():
            for k in range(0, len(frames), 200):  # the "device" talks from the other end
                os.write(master, frames[k : k + 200])
                time.sleep(0.01)

        t = threading.Thread(target=device)
        t.start()
        got = []
        pipeline = Pipeline(TimeLimit(source, 1.0), load_config(), session_dir=tmp_path)
        pipeline.router.subscribe(ImuSample, got.append)
        pipeline.run()  # closes the port at the end
        t.join()
        assert len(got) == 50
        assert all(s.time_source == "host" for s in got)
        assert (tmp_path / "raw.bin").stat().st_size >= len(frames)
        with pytest.raises(AssertionError, match="tried to send"):  # the guard itself works
            source._ser.write(b"x")
    finally:
        os.close(master)
        os.close(slave)


def test_no_serial_send_calls_in_the_source_tree():
    offenders = []
    for path in SRC.rglob("*.py"):
        text = path.read_text()
        if not re.search(r"^\s*(import serial|from serial)", text, re.M):
            continue  # files that never touch pyserial may write files
        for n, line in enumerate(text.splitlines(), 1):
            if FORBIDDEN.search(line.split("#")[0]):
                offenders.append(f"{path.relative_to(SRC)}:{n}: {line.strip()}")
    assert not offenders
    assert "send_break" not in "".join(p.read_text() for p in SRC.rglob("*.py"))
