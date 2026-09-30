import json
from collections import Counter

import numpy as np
import pyarrow.parquet as pq
import pytest
from test_protocol import NAV_PVT, imu_frame, ubx

from imuview.frames import board_rotation, geodetic_to_ned
from imuview.logger import StreamWriter, from_row, read_stream, recover_session, to_row
from imuview.messages import Command, GnssFix, ImuSample, State
from imuview.pipeline import Pipeline, load_config
from imuview.protocol import PARSERS, UbxFramer
from imuview.sources import Chunk, SimSource


class ListSource:
    """Hands out a fixed list of items once."""

    def __init__(self, items):
        self.items = items

    def read(self, timeout=0.0):
        items, self.items = self.items, None
        return items

    def describe(self):
        return {"type": "test"}

    def close(self):
        pass


def fix(t_us=0, lat=47.0, lon=8.0):
    return GnssFix(
        mcu_time_us=t_us,
        pc_rx_time_ns=0,
        valid_time_us=None,
        gnss_tow_ms=None,
        gnss_week=None,
        fix_type=3,
        num_sv=9,
        lat_deg=lat,
        lon_deg=lon,
        height_m=500.0,
        vel_ned=None,
        h_acc_m=None,
        v_acc_m=None,
    )


def test_multirate_sim_routes_and_logs_streams_separately(tmp_path):
    config = load_config()
    source = SimSource(n_imu=2, fake_gnss=True, realtime=False, seconds=3.0)
    p = Pipeline(source, config, session_dir=tmp_path)
    got = Counter()
    p.router.subscribe(ImuSample, lambda m: got.update(["imu"]))
    p.router.subscribe(GnssFix, lambda m: got.update(["gnss"]))
    p.run()

    assert got == {"imu": 6000, "gnss": 14}
    imu = pq.read_table(tmp_path / "imu.parquet")
    gnss = pq.read_table(tmp_path / "gnss.parquet")
    state = pq.read_table(tmp_path / "state.parquet")
    assert (imu.num_rows, gnss.num_rows, state.num_rows) == (6000, 14, 9000)  # vehicle + 2 diag
    assert "lat_deg" not in imu.column_names and "accel_x" not in gnss.column_names
    for row in gnss.to_pylist():
        assert row["valid_time_us"] < row["mcu_time_us"]  # measured before it arrived

    meta = json.loads((tmp_path / "meta.json").read_text())
    assert meta["injected_streams"] == ["gnss"]
    assert meta["stats"]["messages"]["ImuSample"]["rate_hz"] == pytest.approx(2000, rel=1e-3)
    assert meta["stats"]["messages"]["GnssFix"]["rate_hz"] == pytest.approx(5, rel=1e-3)
    assert meta["conventions"]["body"] == "FRD"


def test_board_alignment_brings_imus_into_body_frame(tmp_path):
    config = load_config()
    config["imu"]["default"]["board_alignment_deg"] = [0, 0, 0]  # identity for IMU 0
    config["imu"]["1"] = {"board_alignment_deg": [180, 0, 90]}  # second IMU mounted differently
    source = SimSource(n_imu=2, realtime=False, seconds=8.0, imu_config=config["imu"])
    Pipeline(source, config, session_dir=tmp_path).run()
    table = pq.read_table(tmp_path / "imu.parquet").to_pydict()
    ids = np.array(table["imu_id"])
    accel = np.array([table["accel_body_x"], table["accel_body_y"], table["accel_body_z"]]).T
    diff = accel[ids == 0] - accel[ids == 1]
    assert np.abs(diff.mean(axis=0)).max() < 0.02  # m/s^2, same body-frame accel
    raw = np.array([table["accel_x"], table["accel_y"], table["accel_z"]]).T
    assert np.abs((raw[ids == 0] - raw[ids == 1]).mean(axis=0)).max() > 5  # raw stays as sent


def test_calibration_command_measures_sim_bias(tmp_path):
    config = load_config()
    config["imu"]["default"]["board_alignment_deg"] = [0, 0, 0]  # identity for IMU 0
    config["imu"]["1"] = {"board_alignment_deg": [0, 0, 90]}
    source = SimSource(n_imu=2, realtime=False, seconds=4.0, imu_config=config["imu"])
    p = Pipeline(source, config, session_dir=tmp_path)
    p.command("c")
    p.run()
    for i in range(2):
        r = board_rotation(*([0, 0, 90] if i else [0, 0, 0]))
        assert p.gyro_bias[i] == pytest.approx(r @ source.gyro_bias[i], abs=2e-4)  # rad/s
    assert p.vehicle.gyro_bias == pytest.approx(p.gyro_bias[0])
    commands = read_stream(tmp_path, Command)
    assert [c.kind for c in commands] == ["calibrate"]


def test_rows_round_trip_for_every_message_type():
    msgs = [
        ImuSample(
            mcu_time_us=1,
            pc_rx_time_ns=2,
            seq=3,
            imu_id=1,
            accel=(1.0, 2.0, 3.0),
            gyro=(4.0, 5.0, 6.0),
        ),
        fix(),
        Command(mcu_time_us=1, pc_rx_time_ns=2, kind="tare"),
        State(
            mcu_time_us=1,
            pc_rx_time_ns=2,
            q=(1.0, 0.0, 0.0, 0.0),
            estimator="x",
            pos_ned=(1.0, 2.0, 3.0),
            covariance=[1.0, 0.5],
        ),
    ]
    for m in msgs:
        assert from_row(type(m), to_row(m)) == m


def test_crash_keeps_flushed_data(tmp_path):
    w = StreamWriter(tmp_path, ImuSample)
    for i in range(15):
        w.add(ImuSample(mcu_time_us=i, pc_rx_time_ns=0, imu_id=0, accel=(0, 0, 0), gyro=(0, 0, 0)))
        if i == 9:
            w.flush()  # rows 10..14 were never flushed: lost in the "crash"
    w.f.write(b"\xff\xff\xff\xff\x40\x00\x00\x00half a batch")  # torn write
    w.f.flush()  # ... and the program dies here, without close()
    assert recover_session(tmp_path) == ["imu"]
    assert [m.mcu_time_us for m in read_stream(tmp_path, ImuSample)] == list(range(10))


# ---------- GPS extension points ----------


class FakeUbxGnssParser:
    """Stands in for a future UBX parser: every UBX frame becomes a GnssFix."""

    name = "test-ubx"
    version = "0"

    def __init__(self, imu_config=None):
        self.framer = UbxFramer()

    def parse(self, frame):
        return [fix(t_us=frame.pc_rx_time_ns)]

    def stats(self):
        return {}


def test_extra_parser_plugin_turns_frames_into_gnss_stream(tmp_path, monkeypatch):
    monkeypatch.setitem(PARSERS, "test-ubx", FakeUbxGnssParser)
    config = load_config()
    config["parser"] = "ref"
    config["extra_parsers"] = ["test-ubx"]
    data = b"".join(
        imu_frame(i, i * 1000) + (ubx(*NAV_PVT) if i % 10 == 0 else b"") for i in range(100)
    )
    p = Pipeline(ListSource([Chunk(123, data)]), config, session_dir=tmp_path)
    p.run()
    assert pq.read_table(tmp_path / "gnss.parquet").num_rows == 10
    assert pq.read_table(tmp_path / "imu.parquet").num_rows == 100
    meta = json.loads((tmp_path / "meta.json").read_text())
    assert meta["injected_streams"] == []  # came from bytes, so reparse will rebuild it
    assert set(meta["parsers"]) == {"ref", "test-ubx"}


class StubIns:
    """Stands in for a future GNSS/INS estimator: position = last fix, attitude = level."""

    name = "stub-ins"

    def __init__(self):
        self.origin = None
        self.pos = None

    def params(self):
        return {}

    def process(self, msg):
        if isinstance(msg, GnssFix):
            if self.origin is None:
                self.origin = (msg.lat_deg, msg.lon_deg, msg.height_m)
            ned = geodetic_to_ned(msg.lat_deg, msg.lon_deg, msg.height_m, self.origin)
            self.pos = tuple(ned.tolist())
        if isinstance(msg, ImuSample) and msg.imu_id == 0 and self.pos is not None:
            return State(
                mcu_time_us=msg.mcu_time_us,
                pc_rx_time_ns=msg.pc_rx_time_ns,
                q=(1.0, 0.0, 0.0, 0.0),
                estimator=self.name,
                pos_ned=self.pos,
            )
        return None


def test_position_estimator_fills_state_and_viewer_translation(tmp_path):
    from imuview.viewer import RerunViewer

    config = load_config()
    viewer = RerunViewer(config, spawn=False, rrd_path=tmp_path / "record.rrd")
    source = SimSource(n_imu=1, fake_gnss=True, realtime=False, seconds=2.0)
    p = Pipeline(source, config, viewer=viewer, session_dir=tmp_path / "s")
    p.vehicle = StubIns()
    p.run()

    states = [s for s in read_stream(tmp_path / "s", State) if s.source == "vehicle"]
    assert states and all(s.pos_ned is not None for s in states)
    assert max(abs(v) for s in states for v in s.pos_ned) < 20  # meters around the origin
    assert len(viewer.trajectory) == len(states)
    assert (tmp_path / "record.rrd").stat().st_size > 0
