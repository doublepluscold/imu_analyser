import math
import random
import struct

import pytest

from imuview.frames import G
from imuview.protocol import (
    Demux,
    NmeaFramer,
    RefParser,
    UbxFramer,
    build_demux,
    crc16,
    encode_frame,
    encode_imu_payload,
)


def imu_frame(seq, t_us, rows=((0, 0, -2048, 0, 0, 164),), ftype=0x01):
    return encode_frame(ftype, seq, t_us, encode_imu_payload(ftype, list(rows)))


def nmea(body: str) -> bytes:
    x = 0
    for c in body.encode():
        x ^= c
    return f"${body}*{x:02X}\r\n".encode()


def ubx(cls, mid, payload: bytes) -> bytes:
    body = bytes([cls, mid]) + struct.pack("<H", len(payload)) + payload
    a = b = 0
    for c in body:
        a = (a + c) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


def run(data: bytes, chunk_sizes=None, parser=None):
    """Feed bytes through demux + parser. Returns (messages, demux, parser)."""
    parser = parser or RefParser()
    demux = build_demux([parser])
    msgs = []
    pos = 0
    sizes = iter(chunk_sizes or [len(data)])
    while pos < len(data):
        n = next(sizes, len(data))
        for frame in demux.feed(data[pos : pos + n], pc_rx_time_ns=pos):
            if frame.framer == "ref":
                msgs += parser.parse(frame)
        pos += n
    return msgs, demux, parser


def test_crc_is_ccitt_false():
    assert crc16(b"123456789") == 0x29B1  # standard check value


def test_one_frame_scaled_to_si():
    msgs, demux, _ = run(imu_frame(7, 1000))
    assert len(msgs) == 1
    m = msgs[0]
    assert (m.seq, m.mcu_time_us, m.imu_id) == (7, 1000, 0)
    assert m.accel[2] == pytest.approx(-G)  # -2048 LSB at 1/2048 g
    assert m.gyro[2] == pytest.approx(math.radians(10.0))  # 164 LSB at 1/16.4 dps
    assert demux.unclaimed_bytes == 0


def test_several_imus_in_one_frame():
    rows = [(1, 2, 3, 4, 5, 6), (7, 8, 9, 10, 11, 12), (0, 0, 0, 0, 0, 0)]
    msgs, _, _ = run(imu_frame(0, 5, rows))
    assert [m.imu_id for m in msgs] == [0, 1, 2]
    assert all(m.mcu_time_us == 5 for m in msgs)


@pytest.mark.parametrize("chunk", [1, 2, 3, 7, 50])
def test_fragmented_frames(chunk):
    data = b"".join(imu_frame(i, i * 1000) for i in range(20))
    msgs, demux, parser = run(data, [chunk] * len(data))
    assert [m.seq for m in msgs] == list(range(20))
    assert demux.unclaimed_bytes == 0 and parser.seq_gaps == 0


def test_garbage_between_frames():
    rng = random.Random(1)
    data, garbage = b"", 0
    for i in range(50):
        junk = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 30)))
        data += junk + imu_frame(i, i)
        garbage += len(junk)
    msgs, demux, _ = run(data, [rng.randrange(1, 64) for _ in range(len(data))])
    assert [m.seq for m in msgs] == list(range(50))
    assert demux.unclaimed_bytes == garbage


def test_crc_error_drops_frame_and_resyncs():
    frames = [bytearray(imu_frame(i, i)) for i in range(5)]
    frames[2][12] ^= 0xFF  # corrupt a payload byte
    msgs, demux, parser = run(b"".join(frames))
    assert [m.seq for m in msgs] == [0, 1, 3, 4]
    assert demux.bad_checksum["ref"] == 1
    assert demux.unclaimed_bytes == len(frames[2])
    assert (parser.seq_gaps, parser.frames_lost) == (1, 1)


def test_resync_by_one_byte_finds_frame_inside_corrupted_frame():
    inner = imu_frame(1, 1)
    outer = bytearray(encode_frame(0x01, 0, 0, b"\x00" + inner))  # bogus but framed
    outer[-1] ^= 0xFF  # break outer CRC: inner frame must still be found
    msgs, demux, _ = run(bytes(outer) + imu_frame(2, 2))
    assert [m.seq for m in msgs] == [1, 2]
    assert demux.bad_checksum["ref"] == 1


def test_seq_gaps_and_seq_wrap():
    seqs = [65534, 65535, 0, 1, 5, 6]
    msgs, _, parser = run(b"".join(imu_frame(s, i) for i, s in enumerate(seqs)))
    assert len(msgs) == 6
    assert (parser.seq_gaps, parser.frames_lost) == (1, 3)


def test_timestamp_wrap_unwraps_to_64_bit():
    times = [2**32 - 2000, 2**32 - 1000, 0, 1000]
    msgs, _, _ = run(b"".join(imu_frame(i, t) for i, t in enumerate(times)))
    assert [m.mcu_time_us for m in msgs] == [2**32 - 2000, 2**32 - 1000, 2**32, 2**32 + 1000]


def test_unknown_types_are_counted_and_skipped():
    data = encode_frame(0x12, 0, 0, b"gnss?") + encode_frame(0x7F, 1, 0, b"") + imu_frame(2, 0)
    msgs, _, parser = run(data)
    assert len(msgs) == 1
    assert dict(parser.unknown_types) == {0x12: 1, 0x7F: 1}
    assert parser.seq_gaps == 0


def test_bad_payload_length_is_counted():
    bad = encode_frame(0x01, 0, 0, bytes([2]) + b"\x00" * 12)  # says 2 IMUs, has data for 1
    msgs, _, parser = run(bad + imu_frame(1, 1))
    assert len(msgs) == 1 and parser.bad_length == 1


def test_int32_float_and_quaternion_types():
    msgs, _, _ = run(
        imu_frame(0, 0, [(0, 0, -2048, 0, 0, 0)], ftype=0x02)
        + imu_frame(1, 0, [(0.5, 0, -9.8, 0.1, 0, 0)], ftype=0x03)
        + imu_frame(2, 0, [(0, 0, -9.8, 0, 0, 0, 1, 0, 0, 0)], ftype=0x04)
    )
    assert msgs[0].accel[2] == pytest.approx(-G)
    assert msgs[1].accel[0] == pytest.approx(0.5) and msgs[1].gyro[0] == pytest.approx(0.1)
    assert msgs[2].quat == (1.0, 0.0, 0.0, 0.0)


# ---------- NMEA / UBX framers ----------

GGA = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"
NAV_PVT = (0x01, 0x07, bytes(92))


def test_nmea_valid_and_corrupted():
    good = nmea(GGA)
    bad = bytearray(nmea("GNRMC,1,2,3"))
    bad[5] ^= 0x01  # still printable, checksum now wrong
    _, demux, _ = run(good + bytes(bad) + nmea("GNRMC,1,2,3"))
    assert demux.frames[("nmea", "GPGGA")] == 1
    assert demux.frames[("nmea", "GNRMC")] == 1
    assert demux.bad_checksum["nmea"] == 1


def test_ubx_valid_and_corrupted():
    good = ubx(*NAV_PVT)
    bad = bytearray(good)
    bad[20] ^= 0x10
    _, demux, _ = run(good + bytes(bad) + ubx(0x05, 0x01, b"\x06\x8a"))
    assert demux.frames[("ubx", "NAV-PVT")] == 1
    assert demux.frames[("ubx", "ACK-ACK")] == 1
    assert demux.bad_checksum["ubx"] == 1


def test_framers_alone_reject_wrong_sync():
    assert NmeaFramer().match(bytearray(b"$GP\x00GGA*00\r\n"), 0) == 0
    assert UbxFramer().match(bytearray(b"\xb5\x62\xee\x00\x00\x00"), 0) == 0  # unknown class


@pytest.mark.parametrize("seed", range(5))
def test_interleaved_streams_split_randomly(seed):
    rng = random.Random(seed)
    parts, n_ref = [], 0
    for _ in range(200):
        r = rng.random()
        if r < 0.7:
            parts.append(imu_frame(n_ref, n_ref))
            n_ref += 1
        elif r < 0.85:
            parts.append(nmea(GGA))
        else:
            parts.append(ubx(*NAV_PVT))
    data = b"".join(parts)
    msgs, demux, parser = run(data, [rng.randrange(1, 40) for _ in range(len(data))])
    assert len(msgs) == n_ref and parser.seq_gaps == 0
    assert demux.frames[("nmea", "GPGGA")] + demux.frames[("ubx", "NAV-PVT")] == 200 - n_ref
    assert demux.unclaimed_bytes == 0 and not demux.bad_checksum


def test_demux_counts_all_unclaimed_bytes():
    demux = Demux([UbxFramer()])
    demux.feed(b"hello world", 0)
    assert demux.unclaimed_bytes == 11 and demux.unclaimed_sample == b"hello world"
