"""Typed messages that flow through the pipeline, plus State (the estimator output).

Every message carries three kinds of time:
  mcu_time_us    device clock in microseconds, unwrapped to 64 bit; None if the wire format
                 has no device time (the MTData2 stream of our module has none)
  pc_rx_time_ns  time.monotonic_ns() of the chunk that completed the frame
  host_time_ns   host time of the frame itself: chunk time minus the bytes that followed the
                 frame in that chunk, at the line rate (so frames of one chunk get distinct times)
  time_source    "device" | "host" | "synthetic": which of them `t_us` is based on.
                 "synthetic" = time made up from byte positions (imported raw dumps).
  seq            frame sequence number, if the wire format has one
  module_id      which sensor module the message belongs to. 0 = the USB-serial module,
                 1..255 = modules that arrive over the network (the master ESP hands out the ids).
"""

from dataclasses import dataclass
from enum import IntEnum

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # (w, x, y, z)

TIME_SOURCES = ("device", "host", "synthetic")


@dataclass(kw_only=True)
class Message:
    mcu_time_us: int | None = None
    pc_rx_time_ns: int
    host_time_ns: int | None = None
    time_source: str = "device"
    seq: int | None = None
    module_id: int = 0

    @property
    def t_us(self) -> int:
        """The primary time of this message in microseconds (device if present, else host)."""
        if self.mcu_time_us is not None:
            return self.mcu_time_us
        ns = self.host_time_ns if self.host_time_ns is not None else self.pc_rx_time_ns
        return ns // 1000


@dataclass(kw_only=True)
class ImuSample(Message):
    """One IMU sample.

    accel / gyro are exactly what the device sent, in its own sensor frame and units. They are
    never modified. The pipeline fills the other vectors:
      *_cal   sensor frame, host calibration applied (None without a calibration)
      *_body  body frame (FRD): calibrated if available, then board alignment. Estimators use these.
    """

    imu_id: int
    accel: Vec3  # as sent; m/s^2 for the ref protocol, units to be verified for MTData2
    gyro: Vec3  # as sent; rad/s for the ref protocol
    mag: Vec3 | None = None  # uT
    quat: Quat | None = None  # orientation computed by the device itself (sensor -> world)
    euler_deg: Vec3 | None = None  # device Euler angles as sent (roll, pitch, yaw), meaning TBD
    status: int | None = None  # device status byte (MTData2 0xE010)
    extra: dict | None = None  # other blocks, e.g. {"0x2060": value}; unknown ones as hex
    # identical to one of the last few samples: stale value re-sent by the device.
    # accel and gyro are flagged separately, the firmware updates them independently.
    accel_repeat: bool | None = None
    gyro_repeat: bool | None = None
    accel_cal: Vec3 | None = None
    gyro_cal: Vec3 | None = None
    accel_body: Vec3 | None = None
    gyro_body: Vec3 | None = None
    quat_body: Quat | None = None  # device quaternion turned into body -> world


class FixType(IntEnum):  # same numbering as u-blox NAV-PVT
    NONE = 0
    DEAD_RECKONING = 1
    FIX_2D = 2
    FIX_3D = 3
    GNSS_DR = 4
    TIME_ONLY = 5


@dataclass(kw_only=True)
class GnssFix(Message):
    """A GNSS solution.

    mcu_time_us / pc_rx_time_ns are ARRIVAL times. The fix describes an earlier
    epoch: valid_time_us is the MCU time of that epoch (None if unknown). A future
    estimator must apply the measurement at valid_time_us, not at arrival.
    Positions of a fix with fix_type NONE are placeholders: never plot or use them.
    """

    valid_time_us: int | None
    gnss_tow_ms: int | None  # GPS time of week of the epoch
    gnss_week: int | None
    fix_type: int  # FixType
    num_sv: int | None
    lat_deg: float
    lon_deg: float
    height_m: float  # above the WGS84 ellipsoid
    vel_ned: Vec3 | None  # m/s
    h_acc_m: float | None
    v_acc_m: float | None
    vel_xyz: Vec3 | None = None  # velocity as the device sent it (frame per its XDI bits)


@dataclass(kw_only=True)
class GnssRaw(Message):
    """A GNSS block we do not decode yet, kept as hex (e.g. MTData2 0x8840 GpsSol)."""

    kind: str  # e.g. "0x8840"
    data_hex: str | None  # None when the source lost the bytes (CSV import)


@dataclass(kw_only=True)
class RawBlock(Message):
    """Blocks of a frame that carries no IMU data and that no field takes: {"0xXXXX": value}.
    Undecoded blocks are hex strings. Nothing the device sends is dropped."""

    blocks: dict


@dataclass(kw_only=True)
class Command(Message):
    """A user action (keyboard). Logged so replay can re-apply it at the same moment."""

    kind: str  # "calibrate" | "tare"


@dataclass(kw_only=True)
class Event(Message):
    """Something worth showing in the event log (calibration, CRC errors, new type seen...)."""

    level: str  # "info" | "warn" | "error"
    text: str


@dataclass(kw_only=True)
class State:
    """Estimator output. Viewer and logger only ever see this, never filter internals."""

    mcu_time_us: int | None = None
    pc_rx_time_ns: int
    host_time_ns: int | None = None
    time_source: str = "device"
    module_id: int = 0
    q: Quat  # body (FRD) -> world (NED)
    estimator: str  # "mahony", "madgwick", "passthrough", later e.g. "ins_ekf"
    source: str = "vehicle"  # "vehicle" or "imu<k>" for the per-IMU diagnostic filters
    pos_ned: Vec3 | None = None  # m, filled by a future GNSS/INS estimator
    vel_ned: Vec3 | None = None  # m/s
    covariance: list[float] | None = None  # row-major, flattened

    t_us = Message.t_us


def times_of(msg) -> dict:
    """The time fields (and the module) of a message, to copy into something derived from it."""
    return {
        "module_id": msg.module_id,
        "mcu_time_us": msg.mcu_time_us,
        "pc_rx_time_ns": msg.pc_rx_time_ns,
        "host_time_ns": msg.host_time_ns,
        "time_source": msg.time_source,
    }


# One parquet file per stream. A new message type only needs a line here.
STREAMS = {
    ImuSample: "imu",
    GnssFix: "gnss",
    GnssRaw: "gnss_raw",
    RawBlock: "raw_blocks",
    State: "state",
    Command: "commands",
}
