"""Contract between the ESP firmware (esp_radio/) and this program.

The byte vectors below were produced by the real C++ in esp_radio/common (build_test_frame and
encode_heartbeat, the radio self-test of the slave). If that code changes, regenerate them.
"""

import math
from pathlib import Path

import pytest

from imuview.messages import ImuSample
from imuview.netproto import encode_datagram, parse_datagram
from imuview.protocol import build_demux
from imuview.protocol_mtdata2 import Mtdata2Parser
from imuview.sources import MasterSerialSource, _is_single_mtdata2_frame

ESP = Path(__file__).parent.parent.parent / "esp_radio"

# n -> hex of build_test_frame(n): the frame the slave sends in TEST_BEACON mode
TEST_FRAMES = {
    0: (
        "faff365920300c000000004149f3fac334000040200c0000000000000000411c"
        "f5c380200c000000000000000000000000e01001025040080000000000000000"
        "50200400000000d0100c000000000000000000000000206004000000000c"
    ),
    100: (
        "faff365920300cc1722d43c0a8603ec2b4000040200c0000000000000000411c"
        "f5c380200c000000000000000000000000e01001025040080000000000000000"
        "50200400000000d0100c000000000000000000000000206004000000005b"
    ),
    1234: (
        "faff365920300cc17bc05340c9dfaec315666840200c0000000000000000411c"
        "f5c380200c000000000000000000000000e01001025040080000000000000000"
        "50200400000000d0100c00000000000000000000000020600400000000ef"
    ),
}
# encode_heartbeat(uptime 12345 ms, uart 11520 B, 60 frames, 22 bad checksums, 59 sent, ch 1)
HEARTBEAT = (
    "4956530139300000002d00003c00000016000000000000003b00000000000000000000000000000001000000"
)


def euler_of(n):
    t = n / 20.0
    return (
        20.0 * math.sin(0.8 * t),
        15.0 * math.sin(0.5 * t + 1.0),
        math.fmod(18.0 * t, 360.0) - 180.0,
    )


@pytest.mark.parametrize("n", sorted(TEST_FRAMES))
def test_the_slaves_test_frame_is_a_real_imu_frame_for_the_parser(n):
    frame = bytes.fromhex(TEST_FRAMES[n])
    assert len(frame) == 94 and _is_single_mtdata2_frame(frame)
    parser = Mtdata2Parser()
    demux = build_demux([parser])
    msgs = []
    for fr in demux.feed(frame, pc_rx_time_ns=1):
        msgs += parser.parse(fr)
    samples = [m for m in msgs if isinstance(m, ImuSample)]
    assert len(samples) == 1
    got = samples[0].euler_deg
    for a, b in zip(got, euler_of(n), strict=True):
        assert a == pytest.approx(b, abs=1e-3)  # float32 on the wire
    assert samples[0].accel == pytest.approx((0.0, 0.0, 9.81), abs=1e-4)


def test_a_test_frame_survives_the_whole_usb_path():
    frame = bytes.fromhex(TEST_FRAMES[100])
    assert parse_datagram(encode_datagram(1, 7, frame)).payload == frame
    from test_master_serial import drain, make_source

    src, _ = make_source([encode_datagram(1, 0, frame)])
    assert [c.data for c in drain(src)] == [frame]


def test_a_heartbeat_is_never_mistaken_for_a_frame_or_a_datagram():
    hb = bytes.fromhex(HEARTBEAT)
    assert len(hb) == 44 and hb[:4] == b"IVS\x01"
    assert not _is_single_mtdata2_frame(hb)
    with pytest.raises(ValueError):
        parse_datagram(hb)  # 'IVS' is not the datagram magic + version
    from test_master_serial import drain, make_source

    src, _ = make_source([hb + encode_datagram(1, 0, bytes.fromhex(TEST_FRAMES[0]))])
    assert len(drain(src)) == 1  # the heartbeat bytes are skipped, the datagram behind is found


@pytest.mark.skipif(not ESP.exists(), reason="firmware sources not next to this folder")
def test_diagnosis_codes_of_the_firmware_are_the_ones_the_laptop_translates():
    names = (ESP / "common" / "heartbeat.cpp").read_text()
    for code in MasterSerialSource.DIAG_TEXT:
        assert f'"{code}"' in names, code
    assert '"OK"' in names


@pytest.mark.skipif(not ESP.exists(), reason="firmware sources not next to this folder")
def test_status_line_format_of_the_master_has_the_fields_the_laptop_reads():
    main = (ESP / "esp32s3" / "src" / "main.cpp").read_text()
    for field in (
        "link out=",
        "slaves=%u",
        "id=%u mac=",
        "rssi=%d",
        "hb=ok",
        "hb=lost",
        "hb=none",
        "diag=%s",
    ):
        assert field in main, field
    assert 'snprintf(line, sizeof line, "# ")' in main  # every text line starts with "# "
