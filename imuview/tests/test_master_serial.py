"""The master ESP over USB: binary datagrams and "# " text lines in one byte stream."""

import os
import random
import threading
import time
from collections import deque

import pytest
import serial

from imuview.cli import TimeLimit
from imuview.messages import ImuSample
from imuview.multi import MultiPipeline
from imuview.netproto import encode_datagram
from imuview.pipeline import load_config
from imuview.protocol_mtdata2 import encode_frame, encode_imu_payload
from imuview.sources import MasterSerialSource, SerialSource


class FakeSerial:
    """Stands in for pyserial: hands out the chunks it was given, exactly as given."""

    def __init__(self, chunks=()):
        self.chunks = deque(chunks)
        self.closed = False

    def open(self):
        pass

    def close(self):
        self.closed = True

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, n=1):
        if not self.chunks:
            return b""
        c = self.chunks.popleft()
        if len(c) > n:
            self.chunks.appendleft(c[n:])
            c = c[:n]
        return c


def make_source(chunks=(), **kw):
    fake = FakeSerial(chunks)
    src = MasterSerialSource("fake", serial_cls=lambda: fake, **kw)
    return src, fake


def drain(src):
    out = []
    while src._ser.chunks:
        out += src.read()
    return out


def frame(i):
    return encode_frame(encode_imu_payload(acc=(0.0, 0.0, float(i))))


def datagram(module_id, seq, i):
    return encode_datagram(module_id, seq, frame(i))


def pairs(chunks):
    return [(c.module_id, c.data) for c in chunks]


# ---------------- re-framing ----------------


def test_datagrams_come_out_as_chunks_with_their_module_id():
    stream = datagram(1, 0, 10) + datagram(2, 0, 20) + datagram(1, 1, 11)
    src, _ = make_source([stream])
    got = drain(src)
    assert pairs(got) == [(1, frame(10)), (2, frame(20)), (1, frame(11))]
    assert src.stats()["datagrams"] == {"1": 2, "2": 1}
    assert src.stats()["bad_datagrams"] == 0 and src.stats()["skipped_bytes"] == 0


@pytest.mark.parametrize("size", [1, 2, 3, 5, 8, 13, 50, 101, 102, 103, 5000])
def test_result_does_not_depend_on_how_the_port_chops_the_stream(size):
    stream = b"".join(datagram(1 + i % 3, i // 3, i) for i in range(30))
    cut = 2 * 102  # the master writes whole datagrams and lines: a line sits between datagrams
    stream = stream[:cut] + b"# link out=usb slaves=3\n" + stream[cut:]
    chunks = [stream[k : k + size] for k in range(0, len(stream), size)]
    src, _ = make_source(chunks)
    got = drain(src)
    assert len(got) == 30
    assert [c.data for c in got] == [frame(i) for i in range(30)]
    assert src.stats()["skipped_bytes"] == 0
    assert src.diag_lines() == ["link out=usb slaves=3"]


def test_noise_between_datagrams_is_skipped_and_nothing_real_is_lost():
    rng = random.Random(7)
    not_one = [b for b in range(256) if b != 0x01]  # so noise never forms the header 'IV' 0x01
    parts = []
    for i in range(200):
        noise = bytes(rng.choice(not_one) for _ in range(rng.randrange(0, 30)))
        parts += [noise, datagram(1, i, i)]
    src, _ = make_source([b"".join(parts)])
    got = drain(src)
    assert [c.data for c in got] == [frame(i) for i in range(200)]
    assert src.stats()["skipped_bytes"] > 0  # the noise was thrown away and counted, not fatal


def test_a_stream_that_starts_in_the_middle_of_a_datagram():
    # the laptop opens the port while the master is mid-write
    full = datagram(1, 5, 1)
    stream = full[40:] + datagram(1, 6, 2) + datagram(1, 7, 3)
    src, _ = make_source([stream])
    got = drain(src)
    assert [c.data for c in got] == [frame(2), frame(3)]


@pytest.mark.parametrize("cut", range(1, 102))
def test_a_datagram_cut_off_anywhere_does_not_swallow_the_next_one(cut):
    first, second, third = datagram(1, 0, 1), datagram(1, 1, 2), datagram(1, 2, 3)
    assert len(first) == 102
    src, _ = make_source([first[:cut] + second + third])
    got = drain(src)
    assert [c.data for c in got] == [frame(2), frame(3)]  # broken one gone, rest intact


def test_a_header_that_is_not_followed_by_a_frame_is_rejected():
    fake_header = b"IV\x01\x09\x00\x00\x5e\x00"  # claims 94 bytes of module 9
    src, _ = make_source([fake_header + b"\x00" * 94 + datagram(1, 0, 1)])
    got = drain(src)
    assert [c.data for c in got] == [frame(1)]
    assert src.stats()["bad_datagrams"] >= 1


def test_oversized_length_field_is_rejected_quickly():
    bad = b"IV\x01\x01\x00\x00" + (5000).to_bytes(2, "little")
    src, _ = make_source([bad + datagram(1, 0, 1)])
    assert [c.data for c in drain(src)] == [frame(1)]


def test_lost_and_repeated_datagrams_are_counted_by_sequence_number():
    stream = datagram(1, 0, 0) + datagram(1, 3, 3) + datagram(1, 3, 3) + datagram(2, 0, 9)
    src, _ = make_source([stream])
    got = drain(src)
    assert [(c.module_id, c.data) for c in got] == [(1, frame(0)), (1, frame(3)), (2, frame(9))]
    s = src.stats()
    assert s["lost_by_module"] == {"1": 2} and s["out_of_order_datagrams"] == 1


def test_boot_messages_of_the_chip_are_skipped():
    boot = b"ESP-ROM:esp32s3-20210327\nBuild:Mar 27 2021\nrst:0x1 (POWERON),boot:0x8 (SPI_FAST)\n"
    boot += b"I (123) boot: ESP-IDF v5.1\n"
    src, _ = make_source([boot + datagram(1, 0, 1)])
    assert [c.data for c in drain(src)] == [frame(1)]


# ---------------- the master's text lines ----------------


def test_text_lines_become_diagnostics_and_per_module_fields():
    lines = (
        b"# master up: wifi mac=AA:BB:CC:DD:EE:FF ch=1 out=usb ids_restored=0\n"
        b"# link out=usb slaves=2 queue=0 qdrop=0 out_err=0 bad_unknown=0\n"
        b"# id=1 mac=11:22:33:44:55:66 rx=100 bad=0 sent=100 drop=0 rssi=-45 hb=ok up=12 "
        b"uart=11520 frames=60 badcs=22 tx=60 err=0 nack=0 qdrop=0 ch=1 rst=0 diag=OK\n"
        b"# id=2 mac=11:22:33:44:55:77 rx=0 bad=0 sent=0 drop=0 rssi=-70 hb=ok up=5 uart=0 "
        b"frames=0 badcs=0 tx=0 err=0 nack=0 qdrop=0 ch=1 rst=0 diag=UART_SILENT\n"
    )
    seen = []
    src, _ = make_source([lines])
    src.on_diag = seen.append
    drain(src)
    assert src.link["slaves"] == "2"
    assert src.peers[1]["rssi"] == "-45" and src.peers[1]["diag"] == "OK"
    assert src.peers[2]["diag"] == "UART_SILENT" and src.peers[2]["uart"] == "0"
    assert len(seen) == 4 and seen[0].startswith("master up")
    assert src.stats()["skipped_bytes"] == 0


def test_a_hash_byte_inside_binary_data_is_not_taken_for_a_text_line():
    # a datagram whose payload holds "# " followed by bytes that are not text: still one datagram
    payload = encode_frame(encode_imu_payload(acc=(0.0, 0.0, 1.0), x2060=0.0))
    pos = payload.find(b"\x00")
    assert pos > 0
    tricky = payload[:pos] + b"# " + payload[pos + 2 :]
    # recompute the checksum so the frame stays valid
    body = bytearray(tricky)
    body[-1] = (-sum(body[1:-1])) & 0xFF
    dg = encode_datagram(1, 0, bytes(body))
    src, _ = make_source([b"# link slaves=1\n" + dg + datagram(1, 1, 2)])
    got = drain(src)
    assert [c.data for c in got] == [bytes(body), frame(2)]
    assert src.link == {"slaves": "1"}


def test_a_text_line_without_newline_does_not_hold_the_stream_forever():
    src, _ = make_source([b"# " + b"x" * 500 + datagram(1, 0, 1)])
    assert [c.data for c in drain(src)] == [frame(1)]


def test_a_half_written_text_line_followed_by_a_datagram():
    src, _ = make_source([b"# id=1 mac=AA rx=1" + datagram(1, 0, 1) + datagram(1, 1, 2)])
    got = drain(src)
    assert [c.data for c in got] == [frame(1), frame(2)]


# ---------------- what the status bar says ----------------


def line(**kw):
    base = {
        "id": 1, "mac": "AA:BB:CC:DD:EE:FF", "rx": 10, "bad": 0, "sent": 10, "drop": 0,
        "rssi": -45, "hb": "ok", "up": 10, "uart": 100, "frames": 10, "badcs": 0, "tx": 10,
        "err": 0, "nack": 0, "qdrop": 0, "ch": 1, "rst": 0, "diag": "OK",
    }  # fmt: skip
    base.update(kw)
    return ("# " + " ".join(f"{k}={v}" for k, v in base.items()) + "\n").encode()


LINK = b"# link out=usb slaves=1 queue=0 qdrop=0 out_err=0 bad_unknown=0\n"


def summary(*chunks):
    src, _ = make_source(list(chunks))
    drain(src)
    return src.link_summary()


def test_summary_when_the_port_is_silent():
    src, _ = make_source([])
    assert "нічого не приходить" in src.link_summary()


def test_summary_when_the_port_is_not_the_master():
    assert "не схоже на майстра" in summary(b"hello world, this is some other device\r\n")


def test_summary_when_the_master_hears_no_slave():
    assert "жодного слейва не чути" in summary(b"# link out=usb slaves=0\n")


def test_summary_names_the_problem_of_each_module():
    assert "UART мовчить" in summary(LINK, line(diag="UART_SILENT", uart=0, frames=0))
    assert "кадрів нема" in summary(LINK, line(diag="NO_VALID_FRAMES", frames=0))
    assert "перезавантажується" in summary(LINK, line(diag="RESTARTED"))
    assert "слейв замовк" in summary(LINK, line(hb="lost"))
    assert "#2:" in summary(LINK, line(), line(id=2, diag="NO_ACK"))


def test_summary_when_all_is_well_reports_the_weakest_signal():
    text = summary(LINK, line(rssi=-45), line(id=2, rssi=-61))
    assert "2 мод." in text and "усе гаразд" in text and "-61" in text


def test_summary_when_the_master_stopped_talking(monkeypatch):
    src, _ = make_source([LINK + line()])
    drain(src)
    t0 = time.monotonic()
    monkeypatch.setattr("imuview.sources.time.monotonic", lambda: t0 + 10)
    assert "замовк" in src.link_summary()


def test_old_master_without_text_lines_is_still_usable():
    src, _ = make_source([datagram(1, 0, 1)])
    drain(src)
    assert "дані йдуть" in src.link_summary()


# ---------------- through the real pipeline ----------------


def test_two_modules_over_usb_reach_the_pipeline_as_separate_modules(tmp_path):
    stream = b"".join(datagram(1 + i % 2, i // 2, i) for i in range(40))
    chunks = [stream[k : k + 64] for k in range(0, len(stream), 64)]  # USB moves 64 bytes at a time
    src, _ = make_source(chunks)
    got = {}
    mp = MultiPipeline(TimeLimit(src, 0.5), load_config(), session_dir=tmp_path)
    mp.router.subscribe(ImuSample, lambda m: got.setdefault(m.module_id, []).append(m))
    mp.run()
    assert sorted(got) == [1, 2]
    assert len(got[1]) == 20 and len(got[2]) == 20
    assert all(m.time_source == "host" for m in got[1])


# ---------------- listen-only ----------------


def test_the_usb_source_has_no_way_to_send():
    public = {n for n in dir(MasterSerialSource) if not n.startswith("_")}
    assert not public & {"write", "writelines", "send", "send_break", "ser", "port_object"}
    assert issubclass(MasterSerialSource, SerialSource)  # same safe open: DTR/RTS low, exclusive


def test_a_real_port_gets_opened_like_the_module_port_and_is_never_written(monkeypatch, tmp_path):
    def forbid(*a, **k):
        raise AssertionError("something tried to send to the serial port")

    for name in ("write", "writelines", "send_break", "sendBreak"):
        monkeypatch.setattr(serial.Serial, name, forbid, raising=False)

    opened = {}

    class Spy(serial.Serial):
        def open(self):
            opened.update(dtr=self.dtr, rts=self.rts, rtscts=self.rtscts, dsrdtr=self.dsrdtr)
            super().open()

    master, slave = os.openpty()
    try:
        src = MasterSerialSource(os.ttyname(slave), serial_cls=Spy)
        assert opened == {"dtr": False, "rts": False, "rtscts": False, "dsrdtr": False}
        stream = b"# link out=usb slaves=1\n" + b"".join(datagram(1, i, i) for i in range(30))

        def device():
            for k in range(0, len(stream), 64):
                os.write(master, stream[k : k + 64])
                time.sleep(0.004)

        t = threading.Thread(target=device)
        t.start()
        got = []
        mp = MultiPipeline(TimeLimit(src, 1.0), load_config(), session_dir=tmp_path)
        mp.router.subscribe(ImuSample, got.append)
        mp.run()
        t.join()
        assert len(got) == 30 and {m.module_id for m in got} == {1}
        assert src.link == {"out": "usb", "slaves": "1"}
        with pytest.raises(AssertionError, match="tried to send"):  # the guard itself works
            src._ser.write(b"x")
    finally:
        os.close(master)
        os.close(slave)
