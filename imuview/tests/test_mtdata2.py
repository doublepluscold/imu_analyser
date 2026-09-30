import math
import random
import struct
from collections import Counter
from pathlib import Path

import mtdata2_decoder as vendor
import numpy as np
import pytest

from imuview.frames import G
from imuview.fusion import IntervalClock
from imuview.messages import FixType, GnssFix, GnssRaw, ImuSample, RawBlock
from imuview.pipeline import Pipeline, load_config
from imuview.protocol import build_demux
from imuview.protocol_mtdata2 import (
    Mtdata2Parser,
    encode_block,
    encode_frame,
    encode_gpssol_payload,
    encode_imu_payload,
)
from imuview.sources import SimSource

LEGACY = Path(__file__).parent.parent / "data" / "legacy"


def imu(i=0, **kw):
    kw.setdefault("acc", (0.1 * i, -0.2, 9.8))
    kw.setdefault("gyro", (0.01, 0.02 * i, -0.03))
    kw.setdefault("euler", (1.0, 2.0, float(i)))
    return encode_frame(encode_imu_payload(**kw))


def run(data: bytes, chunk_sizes=None, baud=115200, t_of_end=None):
    """Feed bytes through demux + parser. pc time of a chunk = t_of_end(byte index) or index."""
    parser = Mtdata2Parser(baud=baud)
    demux = build_demux([parser])
    msgs, pos = [], 0
    sizes = iter(chunk_sizes or [len(data)])
    while pos < len(data):
        n = next(sizes, len(data))
        end = min(pos + n, len(data))
        pc = t_of_end(end) if t_of_end else end
        for frame in demux.feed(data[pos:end], pc_rx_time_ns=pc):
            if frame.framer == "mtdata2":
                msgs += parser.parse(frame)
        pos = end
    return msgs, demux, parser


def of(msgs, cls):
    return [m for m in msgs if isinstance(m, cls)]


def test_device_frame_layout_is_94_bytes():
    assert len(imu()) == 94  # LEN 0x59, like the real device
    assert len(encode_frame(encode_gpssol_payload())) == 60  # LEN 55


def test_valid_frames_decode_every_field():
    data = b"".join(imu(i, status=6, latlon=(47.5, 8.25), alt=500.0, vel=(1, 2, 3), x2060=129.5)
                    for i in range(5))  # fmt: skip
    msgs, demux, parser = run(data)
    samples = of(msgs, ImuSample)
    assert len(samples) == 5 and demux.bad_checksum["mtdata2"] == 0
    s = samples[3]
    assert s.accel == pytest.approx((0.3, -0.2, 9.8), abs=1e-6)
    assert s.gyro == pytest.approx((0.01, 0.06, -0.03), abs=1e-6)
    assert s.euler_deg == pytest.approx((1.0, 2.0, 3.0))
    assert s.status == 6 and s.extra == {"0x2060": 129.5}
    assert s.mcu_time_us is None and s.time_source == "host"
    fixes = of(msgs, GnssFix)
    assert len(fixes) == 5 and fixes[0].fix_type == FixType.FIX_3D  # status bit 2
    assert fixes[0].lat_deg == pytest.approx(47.5) and fixes[0].height_m == 500.0
    assert fixes[0].vel_xyz == (1, 2, 3) and fixes[0].vel_ned is None  # frame not verified
    st = parser.stats()
    assert st["xdi_counts"]["0x4020"] == 5 and st["frames_by_mid"] == {"0x36": 5}
    assert st["xdi_flags"]["0x4020"] == {"precision": "float32", "coords": "ENU"}


def test_no_gnss_fix_flag_means_no_fix():
    msgs, _, _ = run(imu(status=2))  # what the device sends without GPS: LatLon ~29.9999
    assert of(msgs, GnssFix)[0].fix_type == FixType.NONE


def test_bad_checksum_is_counted_by_mid_and_len_and_stream_recovers():
    frames = [imu(i) for i in range(10)]
    bad = bytearray(frames[4])
    bad[40] ^= 0x55
    frames[4] = bytes(bad)
    msgs, demux, parser = run(b"".join(frames))
    assert [round(s.accel[0], 3) for s in of(msgs, ImuSample)] == [
        0.0, 0.1, 0.2, 0.3, 0.5, 0.6, 0.7, 0.8, 0.9,
    ]  # fmt: skip
    assert demux.bad_checksum["mtdata2"] >= 1
    assert parser.stats()["bad_checksum_by_mid_len"]["0x36/89"] == 1


def test_truncated_frame_loses_only_itself():
    frames = [imu(i) for i in range(10)]
    frames[3] = frames[3][:50]  # cut short on the wire
    msgs, demux, _ = run(b"".join(frames))
    assert len(of(msgs, ImuSample)) == 9
    assert demux.bad_checksum["mtdata2"] == 1  # the stub + next frame's head fail the sum once


def test_extended_length_frame_and_unknown_xdi_are_kept():
    big = encode_imu_payload(extra_blocks=[(0x7FF0, bytes(range(200)))])
    assert len(big) > 254
    data = imu(0) + encode_frame(big) + encode_frame(encode_imu_payload(), extended=True) + imu(1)
    msgs, _, parser = run(data)
    samples = of(msgs, ImuSample)
    assert len(samples) == 4
    assert samples[1].extra["0x7FF0"] == bytes(range(200)).hex()
    assert parser.stats()["extended_frames"] == 2
    assert parser.stats()["xdi_counts"]["0x7FF0"] == 1


def test_unknown_blocks_of_frames_without_imu_data_are_kept():
    data = encode_frame(encode_gpssol_payload(b"\x01\x02")) + encode_frame(
        encode_block(0x1234, b"\xaa\xbb") + encode_block(0x3010, struct.pack(">I", 101325))
    )
    msgs, _, parser = run(data)
    assert of(msgs, GnssRaw)[0].data_hex == "0102"
    blocks = of(msgs, RawBlock)[0].blocks
    assert blocks == {"0x1234": "aabb", "0x3010": 101325}
    assert parser.stats()["xdi_counts"]["0x1234"] == 1


def test_non_default_precision_or_frame_bits_are_recorded_and_kept_raw():
    payload = encode_imu_payload() + encode_block(0x4024, struct.pack(">3f", 1, 2, 3))  # NED acc
    msgs, _, parser = run(encode_frame(payload))
    assert parser.stats()["xdi_flags"]["0x4024"] == {"precision": "float32", "coords": "NED"}
    assert of(msgs, ImuSample)[0].extra["0x4024"] == struct.pack(">3f", 1, 2, 3).hex()
    assert any(level == "warn" and "0x4024" in text for level, text in parser.drain_events())


def test_frames_split_across_chunks_at_every_position():
    data = imu(0) + encode_frame(encode_gpssol_payload()) + imu(1) + imu(2)
    whole, _, _ = run(data)
    for cut in range(1, len(data)):
        msgs, demux, _ = run(data, [cut])
        assert [type(m) for m in msgs] == [type(m) for m in whole], cut
        assert demux.bad_checksum["mtdata2"] == 0


def test_random_chunks_and_damage_match_the_vendor_framer():
    rng = random.Random(1)
    parts = []
    for i in range(400):
        f = bytearray(imu(i) if i % 7 else encode_frame(encode_gpssol_payload()))
        r = rng.random()
        if r < 0.15:
            f[rng.randrange(1, len(f))] ^= rng.randrange(1, 256)
        elif r < 0.25:
            f = f[: rng.randrange(4, len(f))]
        parts.append(bytes(f))
        if rng.random() < 0.05:
            parts.append(bytes(rng.randrange(256) for _ in range(rng.randrange(1, 30))))
    data = b"".join(parts)
    sizes = [rng.randrange(1, 300) for _ in range(len(data))]
    msgs, demux, parser = run(data, sizes)
    fr = vendor.Framer()
    packets = []
    pos = 0
    for n in sizes:
        packets += fr.feed(data[pos : pos + n])
        pos += n
        if pos >= len(data):
            break
    assert sum(parser.frames.values()) == fr.ok + fr.other
    assert demux.bad_checksum["mtdata2"] == fr.bad
    assert len(of(msgs, ImuSample)) == sum("Acc" in p for p in packets)


@pytest.mark.parametrize("name", ["still.bin", "dryrun_uart.bin", "dryrun_stlink.bin"])
def test_legacy_dumps_match_the_vendor_framer(name):
    path = LEGACY / name
    if not path.exists():
        pytest.skip(f"{path} not present")
    data = path.read_bytes()
    fr = vendor.Framer()
    fr.feed(data)
    _, demux, parser = run(data, [4096] * (len(data) // 4096 + 1))
    assert (sum(parser.frames.values()), demux.bad_checksum["mtdata2"]) == (fr.ok, fr.bad)


def test_imu_board_absent_warning():
    residue = (2.28574213e-15, 5.74420602e-15, -1.13686839e-14)  # real dry run, not exact 0
    data = imu(0) + imu(1, acc=residue, gyro=(0, 0, 0)) + imu(2, acc=(0, 0, 0), gyro=residue)
    _, _, parser = run(data + imu(3))
    events = parser.drain_events()
    assert ("warn", "IMU board seems absent: accel and gyro ~zero (< 1e-9)") in events
    assert any("back" in text for _, text in events)
    assert parser.stats()["imu_all_zero_frames"] == 2


def test_packet_counter_and_sample_time_fine_become_seq_and_device_time():
    frames = []
    for pc, stf in [(1, 100), (2, 200), (4, 400), (5, 2**32 - 50), (6, 50)]:
        blocks = [(0x1020, struct.pack(">H", pc)), (0x1060, struct.pack(">I", stf))]
        frames.append(encode_frame(encode_imu_payload(extra_blocks=blocks)))
    msgs, _, parser = run(b"".join(frames))
    s = of(msgs, ImuSample)
    assert [m.seq for m in s] == [1, 2, 4, 5, 6]
    assert all(m.time_source == "device" for m in s)
    assert [m.mcu_time_us for m in s][:3] == [10_000, 20_000, 40_000]  # 100 us ticks
    assert s[4].mcu_time_us - s[3].mcu_time_us == 100 * 100  # across the u32 wrap
    assert (parser.seq_gaps, parser.frames_lost) == (1, 1)
    assert "0x1020" not in (s[0].extra or {})


# ---------- time without a device clock ----------


def test_host_time_is_the_end_of_each_frame_on_the_wire():
    """Saturated 115200 line, random USB chunks stamped when their last byte arrived."""
    baud = 115200
    ns_per_byte = 10e9 / baud
    frames = [imu(i) if i % 5 else encode_frame(encode_gpssol_payload()) for i in range(200)]
    data = b"".join(frames)
    ends = np.cumsum([len(f) for f in frames])  # byte index after each frame
    t0 = 5_000_000_000
    rng = random.Random(3)
    sizes = [rng.randrange(1, 400) for _ in range(len(data))]
    msgs, _, _ = run(data, sizes, baud, t_of_end=lambda end: t0 + round(end * ns_per_byte))
    times = []
    for m in msgs:
        if not times or m.host_time_ns != times[-1]:
            times.append(m.host_time_ns)  # an IMU frame gives ImuSample + GnssFix, same time
    truth = t0 + ends * ns_per_byte
    assert np.abs(np.array(times) - truth).max() < 1000  # < 1 us
    chunk_times = {m.pc_rx_time_ns for m in msgs}
    assert len(chunk_times) < len(times)  # several frames per chunk, yet distinct host times


def test_interval_clock_clamps_host_time_and_reports_gaps():
    clock = IntervalClock()
    gaps = []
    clock.on_gap = lambda raw, med: gaps.append((raw, med))
    t, dts = 0, []
    for k in range(40):
        t += 12_500 if k % 3 else 4_000  # jitter: bunched frames
        if k == 30:
            t += 200_000  # lost frames
        dts.append(clock.step(t, trusted=False)[0])
    med = clock.median()
    assert all(0.5 * med - 1e-12 <= d <= 3 * med + 1e-12 for d in dts[10:])
    assert len(gaps) == 1 and gaps[0][0] == pytest.approx(0.204)
    assert clock.step(t + 500_000, trusted=True) == (None, 0.5)  # device time: > MAX_DT skip


# ---------- simulator ----------


def test_sim_mtdata2_through_pipeline(tmp_path):
    config = load_config()
    config["imu"]["default"]["board_alignment_deg"] = [180, 0, 0]  # sim sensor is z-up
    source = SimSource(protocol="mtdata2", rate_hz=80, realtime=False, seconds=4.0,
                       gpssol_every=15, corrupt_prob=0.2, truncate_prob=0.05,
                       extended_every=50, seed=2)  # fmt: skip
    p = Pipeline(source, config, session_dir=tmp_path)
    got = Counter()
    for cls in (ImuSample, GnssFix, GnssRaw):
        p.router.subscribe(cls, lambda m, c=cls: got.update([c.__name__]))
    samples = []
    p.router.subscribe(ImuSample, samples.append)
    p.run()
    sent = source.frames_sent
    damaged = source.frames_corrupted + source.frames_truncated
    assert got["ImuSample"] + got["GnssRaw"] == sent - damaged
    assert p.parser.stats()["extended_frames"] > 0
    still = [s for s in samples if (s.t_us - samples[0].t_us) * 1e-6 < 4.0]  # first 5 s: still
    raw = np.mean([s.accel for s in still], axis=0)
    body = np.mean([s.accel_body for s in still], axis=0)
    assert raw == pytest.approx([0, 0, G], abs=0.05)  # z-up sensor reads +g, like the device
    assert body == pytest.approx([0, 0, -G], abs=0.05)  # FRD body: level and still = -g on z
    assert all(s.time_source == "host" for s in samples)
    rate = (len(samples) - 1) / ((samples[-1].t_us - samples[0].t_us) * 1e-6)
    assert rate == pytest.approx(80 * (1 - damaged / sent), rel=0.1)
    assert math.isclose(p.stats()["messages"]["ImuSample"]["rate_hz"], rate, rel_tol=0.05)


def test_stale_accel_and_gyro_are_flagged_separately():
    a = [(0, 0, 9.8 + k * 1e-3) for k in range(6)]
    g = [(k * 1e-4, 0, 0) for k in range(6)]
    frames = [
        imu(acc=a[0], gyro=g[0]),
        imu(acc=a[1], gyro=g[1]),
        imu(acc=a[0], gyro=g[2]),  # accel re-sent from two frames before
        imu(acc=a[3], gyro=g[1]),  # gyro re-sent
        imu(acc=a[4], gyro=g[4]),
    ]
    msgs, _, parser = run(b"".join(frames))
    s = of(msgs, ImuSample)
    assert [m.accel_repeat for m in s] == [False, False, True, False, False]
    assert [m.gyro_repeat for m in s] == [False, False, False, True, False]
    assert parser.stats()["repeated_accel"] == parser.stats()["repeated_gyro"] == 1
