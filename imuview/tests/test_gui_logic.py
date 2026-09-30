import math
import socket
import time

import numpy as np
import pytest

from imuview.frames import quat_from_euler
from imuview.gui.controller import AppController
from imuview.gui.models import (
    ModelRegistry,
    ModelSpec,
    bbox,
    load_stl,
    pose_matrix,
    prepare,
    surface_centroid,
)
from imuview.gui.sessions import SessionPlayer, wait_until
from imuview.gui.store import Store
from imuview.messages import GnssFix, ImuSample, State
from imuview.multi import MultiPipeline
from imuview.netproto import encode_datagram
from imuview.pipeline import load_config
from imuview.protocol import PARSERS, build_demux
from imuview.sources import MultiSimSource


def record_session(path, n=3, seconds=6.0, gps=True):
    config = load_config()
    source = MultiSimSource(n_modules=n, rate_hz=80.0, realtime=False, seconds=seconds,
                            imu_config=config["imu"], gpssol_every=15,
                            gps_origin=(50.45, 30.52) if gps else None)  # fmt: skip
    MultiPipeline(source, config, session_dir=path).run()
    return config


# ---------- STL and models ----------


def write_binary_stl(path, tris):
    import struct

    with open(path, "wb") as f:
        f.write(b"\0" * 80 + struct.pack("<I", len(tris)))
        for t in tris:
            f.write(
                struct.pack("<3f", 0, 0, 0) + struct.pack("<9f", *np.asarray(t).ravel()) + b"\0\0"
            )


def write_ascii_stl(path, tris):
    lines = ["solid t"]
    for t in tris:
        lines.append("facet normal 0 0 0\n outer loop")
        lines += [f"  vertex {v[0]:e} {v[1]:e} {v[2]:e}" for v in t]
        lines.append(" endloop\nendfacet")
    lines.append("endsolid t")
    path.write_text("\n".join(lines))


def long_box(cx=10.0):
    """A box 6 long (x), 2 wide (y), 1 high (z), not centred on the origin."""
    from imuview.gui.models import _box

    return _box((cx, 5, 3), (6, 2, 1))


def test_stl_binary_and_ascii_give_the_same_triangles(tmp_path):
    tris = long_box()
    write_binary_stl(tmp_path / "b.stl", tris)
    write_ascii_stl(tmp_path / "a.stl", tris)
    b, a = load_stl(tmp_path / "b.stl"), load_stl(tmp_path / "a.stl")
    assert b.shape == a.shape == (12, 3, 3)
    assert np.allclose(a, b, atol=1e-5) and np.allclose(b, tris)


def test_bad_stl_is_an_error(tmp_path):
    (tmp_path / "x.stl").write_bytes(b"not an stl at all")
    with pytest.raises(ValueError):
        load_stl(tmp_path / "x.stl")


def test_auto_centre_and_scale_fit_the_bounding_box():
    tris = long_box()
    out = prepare(tris, ModelSpec("m", "m"), size=4.0)
    lo, hi = bbox(out)
    assert (hi - lo).max() == pytest.approx(4.0, rel=1e-5)
    assert np.allclose(surface_centroid(out), 0, atol=1e-5)  # turns around itself
    # a box is symmetric, so the bbox is centred too
    assert np.allclose(lo, -hi, atol=1e-5)


def test_manual_scale_rotation_and_centre_override_the_automatic_ones():
    tris = long_box()
    spec = ModelSpec("m", "m", rotation_deg=(0, 0, 90), scale=0.5, center=(10.0, 5.0, 3.0))
    lo, hi = bbox(prepare(tris, spec, size=4.0))
    size = hi - lo
    assert size.max() == pytest.approx(2.0, rel=1e-5)  # scale 0.5 of size 4
    assert size[1] == pytest.approx(2.0, rel=1e-5)  # yaw 90: the long side went from x to y
    assert size[0] == pytest.approx(2.0 / 3, rel=1e-5)


def test_registry_lists_builtin_toml_and_bare_stl_files(tmp_path):
    write_binary_stl(tmp_path / "plane.stl", long_box())
    write_binary_stl(tmp_path / "bare.stl", long_box())
    (tmp_path / "models.toml").write_text(
        '[[model]]\nkey = "p"\ndisplay_name = "Plane"\nfile = "plane.stl"\nscale = 0.5\n'
        '[[model]]\nkey = "gone"\nfile = "missing.stl"\n'
    )
    reg = ModelRegistry(tmp_path)
    assert reg.names()["p"] == "Plane"
    assert reg.names()["bare"] == "bare"  # no entry needed
    assert "gone" not in reg.keys() and {"quad", "board"} <= set(reg.keys())
    assert reg.mesh("p").size().max() == pytest.approx(4.0 * 0.5, rel=1e-4)
    assert reg.mesh("bare").size().max() == pytest.approx(4.0, rel=1e-4)
    assert reg.mesh("nonexistent").spec.key == "quad"  # falls back


def test_the_shipped_models_are_nose_forward():
    reg = ModelRegistry("models")
    for key in ("shahed", "scout"):
        m = reg.mesh(key)
        assert m.size().max() == pytest.approx(4.0, rel=1e-4)
        size = m.size()
        assert size[0] >= size[2]  # longer than thick: nose axis is x, not z


def test_pose_matrix_orientation_and_position():
    # level, heading north: body x -> north = scene +y; body z (down) -> scene -z
    m = pose_matrix(quat_from_euler(0, 0, 0), (1, 2, 3))
    assert np.allclose(m @ [1, 0, 0, 1], [1, 3, 3, 1])
    assert np.allclose(m @ [0, 0, 1, 1], [1, 2, 2, 1])
    # yaw 90 deg: heading east = scene +x
    m = pose_matrix(quat_from_euler(0, 0, math.pi / 2))
    assert np.allclose(m @ [1, 0, 0, 1], [1, 0, 0, 1], atol=1e-9)
    assert np.allclose(m[:3, :3] @ m[:3, :3].T, np.eye(3), atol=1e-9)


# ---------- store ----------


def feed_store(store, module_id, seconds=3.0, rate=50.0, accel=(0, 0, -9.80665), q=(1, 0, 0, 0),
               fix=None):  # fmt: skip
    n = int(seconds * rate)
    for i in range(n):
        t_us = int(i / rate * 1e6) + 1_000_000
        kw = {"module_id": module_id, "pc_rx_time_ns": t_us * 1000, "host_time_ns": t_us * 1000,
              "time_source": "host"}  # fmt: skip
        store.on_imu(
            ImuSample(
                imu_id=0,
                accel=accel,
                gyro=(0, 0, 0),
                accel_body=accel,
                gyro_body=(0, 0, 0),
                euler_deg=(0, 0, 0),
                status=7,
                **kw,
            )
        )
        store.on_state(State(q=q, estimator="mahony", **kw))
        if fix is not None and i % 10 == 0:
            store.on_gnss(fix(i, **kw))


def gnss(i, **kw):
    return GnssFix(valid_time_us=None, gnss_tow_ms=None, gnss_week=None, fix_type=3, num_sv=8,
                   lat_deg=50.0 + i * 1e-6, lon_deg=30.0, height_m=100.0 + i * 0.01,
                   vel_ned=None, h_acc_m=None, v_acc_m=None, vel_xyz=(1.0, 0.0, 0.0),
                   **kw)  # fmt: skip


def test_free_acceleration_is_zero_when_still_and_shows_a_manoeuvre():
    store = Store()
    feed_store(store, 1)
    p = store.pose_at(1)
    assert np.allclose(p.free_accel, 0, atol=1e-9)  # level and still: gravity cancels
    assert p.accel_body[2] == pytest.approx(-9.80665)
    feed_store(store, 2, accel=(2.0, 0, -9.80665))  # 2 m/s^2 forward on top of gravity
    assert np.allclose(store.pose_at(2).free_accel, (2.0, 0, 0), atol=1e-9)
    # tilted 90 deg nose down: gravity lies along body x; specific force reads (-g, 0, 0) at rest
    q = quat_from_euler(0, math.radians(90), 0)
    feed_store(store, 3, accel=(9.80665, 0, 0), q=tuple(q))
    assert np.allclose(store.pose_at(3).free_accel, 0, atol=1e-6)


def test_gps_position_is_relative_to_the_first_fix_and_respects_the_switch():
    store = Store()
    feed_store(store, 1, fix=gnss)
    p = store.pose_at(1)
    assert p.gps_valid
    east, north, up = p.position
    assert abs(east) < 1e-6 and north > 0 and up > 0  # moved north and up since the first fix
    assert store.pose_at(1, use_gps=False).position is None  # GPS switched off
    assert np.allclose(store.pose_at(1, t=0.0).position, (0, 0, 0), atol=0.2)


def test_no_fix_gives_no_position():
    store = Store()

    def nofix(i, **kw):
        f = gnss(i, **kw)
        f.fix_type = 0
        return f

    feed_store(store, 1, fix=nofix)
    assert store.pose_at(1).position is None and store.origin is None


def test_trail_returns_past_positions_oldest_first_within_the_window():
    store = Store()
    feed_store(store, 1, seconds=6.0, fix=gnss)
    trail = store.trail(1, t=5.5, seconds=2.0, n=6)
    times = [x[0] for x in trail]
    assert times == sorted(times) and 3.4 <= times[0] and times[-1] <= 5.5 + 1e-9
    assert 2 <= len(trail) <= 6
    assert store.trail(9, t=5.0, seconds=2.0) == []


def test_series_windows_and_alive():
    store = Store()
    feed_store(store, 1, seconds=4.0)
    t, v = store.series(1, "accel", 1.0, 2.0)
    assert len(t) == pytest.approx(50, abs=2) and v.shape[1] == 3 and t[0] >= 1.0 - 1e-9
    assert store.series(1, "gyro", 0, 10)[1].shape[1] == 3
    assert store.alive(1) and not store.alive(2)
    assert store.alive(1, at=3.0) and not store.alive(1, at=30.0)
    lo, hi = store.span()
    assert lo == pytest.approx(0, abs=0.05) and hi == pytest.approx(3.98, abs=0.05)


def test_store_trims_old_data_in_live_mode_only():
    live, keep = Store(keep_s=1.0), Store(keep_s=None)
    for s in (live, keep):
        feed_store(s, 1, seconds=60.0, rate=50.0)
    assert len(live.modules[1].imu) < 500 and len(keep.modules[1].imu) == 3000


def test_raw_table_and_hex():
    store = Store()
    feed_store(store, 1, fix=gnss)
    rows = {r[0]: r for r in store.raw_table(1)}
    assert {"0x2030", "0x4020", "0x8020", "0xE010", "0x5040", "0x5020", "0xD010"} <= set(rows)
    assert "gnss 1" in rows["0xE010"][2]
    from imuview.sources import Chunk

    store.on_chunk(Chunk(1_500_000_000, b"\xfa\xff\x36", 1))
    assert "fa ff 36" in store.raw_hex(1)[-1]


# ---------- analysis: determinism ----------


def test_player_loads_deterministically_and_seeks(tmp_path):
    config = record_session(tmp_path / "s")
    a, b = SessionPlayer(tmp_path / "s", config), SessionPlayer(tmp_path / "s", config)
    a.load(background=False)
    b.load(background=False)
    assert a.loaded and b.loaded and a.error is None
    assert a.store.module_ids() == [1, 2, 3]
    assert (a.t_start, a.t_end) == (b.t_start, b.t_end) and a.t_end - a.t_start > 5
    for t in (a.t_start, 2.5, a.t_end):
        for m in (1, 2, 3):
            pa, pb = a.store.pose_at(m, t), b.store.pose_at(m, t)
            assert pa == pb
    ta, va = a.store.series(2, "accel", 0, 100)
    tb, vb = b.store.series(2, "accel", 0, 100)
    assert np.array_equal(ta, tb) and np.array_equal(va, vb)


def test_player_time_control(tmp_path):
    config = record_session(tmp_path / "s", n=1)
    p = SessionPlayer(tmp_path / "s", config)
    p.load(background=False)
    p.seek(-5)
    assert p.t == p.t_start
    p.seek(1e9)
    assert p.t == p.t_end
    p.seek(p.t_start)
    p.tick(1.0)
    assert p.t == p.t_start  # paused: does not move
    p.play()
    p.speed = 2.0
    p.tick(1.0)
    assert p.t == pytest.approx(p.t_start + 2.0)
    assert 0 < p.fraction < 1
    p.tick(1e6)
    assert p.t == p.t_end and not p.playing
    p.play()  # at the end: starts over
    assert p.t == p.t_start and p.playing


def test_old_single_module_session_opens_in_analysis():
    import glob

    sessions = sorted(glob.glob("logs/*_pose_level"))
    if not sessions:
        pytest.skip("no recorded sessions")
    p = SessionPlayer(sessions[0], load_config())
    p.load(background=False)
    assert p.loaded and p.store.module_ids() == [0]
    pose = p.store.pose_at(0, p.t_end)
    assert pose is not None and abs(pose.euler_deg[0]) < 10  # lying level


# ---------- controller ----------


def test_controller_switching_layouts_models_and_modules(tmp_path):
    config = record_session(tmp_path / "s")
    c = AppController(config, models_dir="models")
    c.open_session(tmp_path / "s", background=False)
    wait_until(lambda: c.player.loaded)
    c.poll()
    assert c.mode == "analysis" and c.module_ids() == [1, 2, 3] and c.active == 1
    c.set_active(2)
    assert c.active == 2
    c.set_active(99)  # unknown module: ignored
    assert c.active == 2
    assert c.shown_modules() == [2]
    c.set_layout("scene")
    assert c.shown_modules() == [1, 2, 3]
    c.set_model("shahed")  # active module only
    assert c.model_of(2) == "shahed" and c.model_of(1) == "quad"
    c.set_model("scout", everywhere=True)
    assert {c.model_of(m) for m in (1, 2, 3)} == {"scout"}
    with pytest.raises(KeyError):
        c.set_model("no-such-model")
    with pytest.raises(ValueError):
        c.set_layout("bogus")
    assert c.pose(1) is not None and c.alive(1)
    c.options.use_gps = False
    assert c.pose(1).position is None
    c.player.seek(c.player.t_end)
    assert c.view_time() == c.player.t_end
    c.set_mode("live")
    assert c.view_time() is None and c.module_ids() == []


def test_controller_names_come_from_the_config():
    config = load_config()
    config["modules"]["names"] = {"1": "Дрон 1"}
    c = AppController(config, models_dir="models")
    assert c.name(1) == "Дрон 1" and c.name(2) == "Module 2"


def test_live_udp_end_to_end_with_recording(tmp_path):
    config = load_config()
    config["log_dir"] = str(tmp_path)
    c = AppController(config, models_dir="models")
    c.start_live("udp", host="127.0.0.1", port=0)
    port = c.live.pipeline.source.port
    assert c.conn_state == "connected" and not c.recording  # nothing is written before Start
    c.start_run()
    assert wait_until(lambda: c.recording, 3)
    sim = MultiSimSource(n_modules=2, rate_hz=80.0, realtime=False, seconds=3.0,
                         imu_config=config["imu"])  # fmt: skip
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    demuxes = {1: build_demux([PARSERS["mtdata2"](config["imu"])]),
               2: build_demux([PARSERS["mtdata2"](config["imu"])])}  # fmt: skip
    seq = {1: 0, 2: 0}
    try:
        while (items := sim.read(0)) is not None:
            for it in items:
                for fr in demuxes[it.module_id].feed(it.data, it.pc_rx_time_ns):
                    tx.sendto(encode_datagram(it.module_id, seq[it.module_id], fr.data),
                              ("127.0.0.1", port))  # fmt: skip
                    seq[it.module_id] += 1
            time.sleep(0.0005)
        assert wait_until(lambda: c.module_ids() == [1, 2] and c.pose(2) is not None, 5)
        c.poll()
        assert c.active == 1 and c.alive(1)
        assert c.conn_state == "running" and c.recording
        rec_dir = c.live.session_dir
        c.stop_live()
    finally:
        tx.close()
    import pyarrow.parquet as pq

    rows = pq.read_table(rec_dir / "imu.parquet").to_pylist()
    assert {r["module_id"] for r in rows} == {1, 2}


def test_live_recording_can_be_switched_off(tmp_path):
    config = load_config()
    config["log_dir"] = str(tmp_path)
    c = AppController(config, models_dir="models")
    c.start_live("sim", sim_modules=2, sim_gps=False)
    assert wait_until(lambda: c.module_ids() == [0 + 1, 2], 5)
    assert c.conn_state == "connected" and not c.live.recording
    c.start_run()
    assert wait_until(lambda: c.live.recording, 3)
    c.set_record(False)  # takes effect at once while running
    assert wait_until(lambda: not c.live.recording, 3)
    c.stop_live()
    assert c.live is None and "stopped" in c.message


def test_connection_state_machine_and_log_switch(tmp_path):
    config = load_config()
    config["log_dir"] = str(tmp_path)
    c = AppController(config, models_dir="models")
    assert c.conn_state == "disconnected"
    c.start_run()  # no connection: nothing happens
    assert c.conn_state == "disconnected"
    c.start_live("sim", sim_modules=1, sim_gps=False)
    assert c.conn_state == "connected"
    c.options.record = False  # "Писати лог" off: Start runs without a log
    c.start_run()
    assert c.conn_state == "running"
    time.sleep(0.3)
    assert not c.recording
    c.stop_run()
    assert c.conn_state == "connected"
    c.stop_live()
    assert c.conn_state == "disconnected" and not list(tmp_path.iterdir())


def test_store_rate():
    store = Store()
    feed_store(store, 1, seconds=3.0, rate=50.0)
    assert store.rate(1) == pytest.approx(50.0, abs=2)
    assert store.rate(1, at=1.5) == pytest.approx(50.0, abs=2)
    assert store.rate(9) == 0.0
