import re
import socket
import time
from pathlib import Path

import pyarrow.parquet as pq
import pytest

from imuview.logger import RawWriter, read_raw
from imuview.messages import ImuSample, State
from imuview.multi import MultiPipeline, module_name
from imuview.netproto import DatagramError, encode_datagram, parse_datagram
from imuview.pipeline import load_config
from imuview.protocol_mtdata2 import encode_frame, encode_imu_payload
from imuview.sources import Chunk, MultiSimSource, RawFileSource, UdpSource

SRC = Path(__file__).parent.parent / "src" / "imuview"


def test_datagram_roundtrip_and_errors():
    data = encode_datagram(3, 65537, b"\xfa\xff\x36")
    d = parse_datagram(data)
    assert (d.module_id, d.seq, d.payload) == (3, 1, b"\xfa\xff\x36")  # seq is u16
    with pytest.raises(DatagramError, match="magic"):
        parse_datagram(b"XX" + data[2:])
    with pytest.raises(DatagramError, match="version"):
        parse_datagram(data[:2] + b"\x09" + data[3:])
    with pytest.raises(DatagramError, match="length"):
        parse_datagram(data[:-1])
    with pytest.raises(DatagramError, match="short"):
        parse_datagram(b"IV")


def test_raw_bin_keeps_module_id_and_reads_old_files(tmp_path):
    w = RawWriter(tmp_path / "raw.bin")
    w.write(Chunk(10, b"abc", 0))
    w.write(Chunk(20, b"defg", 7))
    w.close()
    assert [(c.pc_rx_time_ns, c.data, c.module_id) for c in read_raw(tmp_path / "raw.bin")] == [
        (10, b"abc", 0),
        (20, b"defg", 7),
    ]
    # a record of the old format (len field without module id) is module 0
    import struct

    old = tmp_path / "old.bin"
    old.write_bytes(struct.pack("<QI", 5, 2) + b"xy")
    assert [c.module_id for c in read_raw(old)] == [0]


def test_udp_source_receives_counts_and_drops_bad():
    src = UdpSource("127.0.0.1", 0)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        addr = ("127.0.0.1", src.port)
        tx.sendto(encode_datagram(1, 0, b"aa"), addr)
        tx.sendto(encode_datagram(2, 0, b"bb"), addr)
        tx.sendto(b"garbage", addr)
        tx.sendto(encode_datagram(1, 3, b"cc"), addr)  # seq 1 and 2 of module 1 are missing
        tx.sendto(encode_datagram(1, 3, b"cc"), addr)  # repeated
        got = []
        deadline = time.monotonic() + 2
        while len(got) < 3 and time.monotonic() < deadline:
            got += src.read(0.1)
        assert [(c.module_id, c.data) for c in got] == [(1, b"aa"), (2, b"bb"), (1, b"cc")]
        stats = src.stats()
        assert stats["bad_datagrams"] == 1
        assert stats["lost_by_module"] == {"1": 2}
        assert stats["out_of_order_datagrams"] == 1
    finally:
        tx.close()
        src.close()


def test_source_tree_never_sends_udp():
    text = "".join(p.read_text() for p in SRC.rglob("*.py"))
    assert not re.search(r"\.(sendto|sendall|send)\(", text)


def run_multi(tmp_path, n=3, seconds=4.0):
    config = load_config()
    config["modules"]["names"] = {"1": "Drone 1"}
    source = MultiSimSource(n_modules=n, rate_hz=80.0, realtime=False, seconds=seconds)
    seen = []
    mp = MultiPipeline(source, config, session_dir=tmp_path)
    mp.on_new_module.append(seen.append)
    per_module = {}
    mp.router.subscribe(ImuSample, lambda m: per_module.setdefault(m.module_id, []).append(m))
    mp.run()
    return mp, seen, per_module


def test_multi_pipeline_routes_by_module_and_records_one_session(tmp_path):
    mp, seen, per_module = run_multi(tmp_path)
    assert sorted(seen) == [1, 2, 3]
    assert sorted(per_module) == [1, 2, 3]
    assert all(len(v) > 200 for v in per_module.values())
    imu = pq.read_table(tmp_path / "imu.parquet").to_pylist()
    assert {r["module_id"] for r in imu} == {1, 2, 3}
    for mid, msgs in per_module.items():
        assert sum(r["module_id"] == mid for r in imu) == len(msgs)
    states = pq.read_table(tmp_path / "state.parquet").to_pylist()
    assert {r["module_id"] for r in states} == {1, 2, 3}
    assert {c.module_id for c in read_raw(tmp_path / "raw.bin")} == {1, 2, 3}
    assert module_name(mp.config, 1) == "Drone 1" and module_name(mp.config, 2) == "Module 2"


def test_modules_are_independent(tmp_path):
    # modules start pointing in different directions, so their attitudes must differ
    _, _, per_module = run_multi(tmp_path)
    first = {k: v[-1].euler_deg for k, v in per_module.items()}
    assert len({round(e[2]) for e in first.values()}) == 3


def test_replay_of_multi_session_is_deterministic(tmp_path):
    _, _, per_module = run_multi(tmp_path / "a")
    out = tmp_path / "b"
    config = load_config()
    replay = MultiPipeline(RawFileSource(tmp_path / "a"), config, session_dir=out)
    replay.run()
    a = pq.read_table(tmp_path / "a" / "imu.parquet").to_pylist()
    b = pq.read_table(out / "imu.parquet").to_pylist()
    assert len(a) == len(b) > 0
    key = lambda r: (r["module_id"], r["pc_rx_time_ns"], r["accel_x"])  # noqa: E731
    assert sorted(map(key, a)) == sorted(map(key, b))
    sa = pq.read_table(tmp_path / "a" / "state.parquet").to_pylist()
    sb = pq.read_table(out / "state.parquet").to_pylist()
    assert sorted((r["module_id"], r["q_w"], r["q_x"]) for r in sa) == sorted(
        (r["module_id"], r["q_w"], r["q_x"]) for r in sb
    )


def test_single_module_zero_keeps_working(tmp_path):
    config = load_config()
    frames = b"".join(encode_frame(encode_imu_payload(acc=(0, 0, 9.8))) for _ in range(20))

    class One:
        items = [Chunk(1, frames, 0)]

        def read(self, timeout=0):
            items, self.items = self.items, None
            return items

        def describe(self):
            return {"type": "test"}

        def close(self):
            pass

    mp = MultiPipeline(One(), config, session_dir=tmp_path)
    got = []
    mp.router.subscribe(State, got.append)
    mp.run()
    assert got and {s.module_id for s in got} == {0}
