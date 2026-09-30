"""UDP datagram format between the master ESP and this program.

    [magic: 'IV'] [version: u8] [module_id: u8] [seq: u16 LE] [len: u16 LE] [len bytes of MTData2]

- one datagram carries the raw MTData2 bytes of ONE module (normally one whole frame: an IMU
  frame is 94 bytes and a GpsSol frame 60, an ESP-NOW payload holds up to 250);
- module_id 1..255 is handed out by the master in the order the modules connect (0 is what the
  USB-serial module uses inside this program);
- seq counts the datagrams of that module, so gaps show lost packets;
- the receiving side only receives, it never sends anything back.
"""

import struct
from dataclasses import dataclass

MAGIC = b"IV"
VERSION = 1
HEADER = struct.Struct("<2sBBHH")  # magic, version, module_id, seq, len
MAX_PAYLOAD = 1400  # one Ethernet frame; the real payload is about 100 bytes


class DatagramError(ValueError):
    """The datagram does not follow the format above."""


@dataclass(frozen=True)
class Datagram:
    module_id: int
    seq: int
    payload: bytes


def encode_datagram(module_id: int, seq: int, payload: bytes) -> bytes:
    if not 0 <= module_id <= 255:
        raise ValueError("module_id must fit in one byte")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload longer than {MAX_PAYLOAD} bytes")
    return HEADER.pack(MAGIC, VERSION, module_id, seq & 0xFFFF, len(payload)) + payload


def parse_datagram(data: bytes) -> Datagram:
    """Raises DatagramError for anything that is not exactly one valid datagram."""
    if len(data) < HEADER.size:
        raise DatagramError(f"too short ({len(data)} bytes)")
    magic, version, module_id, seq, length = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise DatagramError("bad magic")
    if version != VERSION:
        raise DatagramError(f"unknown version {version}")
    if length != len(data) - HEADER.size:
        raise DatagramError(f"length field {length} does not match {len(data) - HEADER.size} bytes")
    return Datagram(module_id, seq, bytes(data[HEADER.size :]))
