from imuview.framing_analysis import analyze, correct_cs
from imuview.protocol_mtdata2 import encode_frame, encode_gpssol_payload, encode_imu_payload
from imuview.sources import SimSource

JUNK = bytes.fromhex("fa705403110000871ec82a0003000300000000000000000502")  # seen on the device


def stream():
    """200 IMU frames, GpsSol every 20 (+ junk after it), known damage."""
    frames = [encode_frame(encode_imu_payload(acc=(0, 0, 9.8 + k * 1e-3))) for k in range(200)]
    kinds = ["ok"] * 200
    for k in range(30, 200, 17):  # payload byte flipped, full length stays
        f = bytearray(frames[k])
        f[30] ^= 0x10
        frames[k], kinds[k] = bytes(f), "corrupt"
    for k in range(40, 200, 41):  # cut short
        frames[k], kinds[k] = frames[k][:50], "trunc"
    for k in range(35, 200, 23):  # ping-pong race: CS belongs to the frame 2 later
        if kinds[k] == kinds[k + 2] == "ok" and k + 2 < 200:
            frames[k] = frames[k][:-1] + bytes([correct_cs(frames[k + 2])])
            kinds[k] = "race"
    out = []
    for k, f in enumerate(frames):
        out.append(f)
        if k % 20 == 19:
            out.append(encode_frame(encode_gpssol_payload()) + JUNK)
    return b"".join(out), kinds


def test_analysis_finds_the_injected_damage():
    data, kinds = stream()
    r = analyze(data, duration_s=2.0)
    n = {k: kinds.count(k) for k in set(kinds)}
    assert r["valid_frames"] == {"0x36/55": 10, "0x36/89": n["ok"]}
    assert r["bad_full_length"] == n["corrupt"] + n["race"]
    assert r["bad_truncated"] == n["trunc"]
    assert r["false_syncs"] == 0 and r["extended_frames"] == 0
    assert r["unclaimed_runs_top"][0] == {"len": 25, "count": 10, "hex": JUNK.hex()}
    assert r["byte_accounting"]["uncovered_total"] == 10 * 25
    hits = r["cs_equals_correct_cs_of_frame_at_lag"]["hits"]
    assert hits.get(2, 0) == n["race"] and sum(hits.values()) <= n["race"] + 1
    assert r["rates"]["frames_per_s"]["0x36/89"] == 100.0


def test_analysis_of_simulated_corruption_matches_the_simulator():
    sim = SimSource(protocol="mtdata2", rate_hz=80, realtime=False, seconds=10.0,
                    gpssol_every=16, corrupt_prob=0.2, truncate_prob=0.05, seed=4)  # fmt: skip
    data = bytearray()
    while (items := sim.read()) is not None:
        data += b"".join(c.data for c in items)
    r = analyze(bytes(data), duration_s=10.0)
    sent_ok = sim.frames_sent - sim.frames_corrupted - sim.frames_truncated
    assert sum(r["valid_frames"].values()) == sent_ok
    damaged = r["bad_full_length"] + r["bad_truncated"]
    # a flipped header byte turns a frame into unclaimed bytes, not a "bad real header"
    assert sim.frames_corrupted + sim.frames_truncated - 15 <= damaged
    assert damaged <= sim.frames_corrupted + sim.frames_truncated
    assert abs(r["bad_truncated"] - sim.frames_truncated) <= 3
