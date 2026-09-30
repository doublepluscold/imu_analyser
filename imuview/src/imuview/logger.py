"""Session logging.

logs/YYYY-MM-DD_HH-MM-SS/
  raw.bin        every received byte: records of [pc_rx_time_ns:u64][len|module_id<<24:u32][bytes]
  <stream>.parquet  one file per message type (imu, state, commands, gnss ...)
  events.jsonl   event log, one JSON object per line
  meta.json      settings at start, statistics at the end

While recording, each stream is written as an Arrow IPC stream (<stream>.arrows) and
flushed about once per second. That format stays readable if the program crashes.
On a clean stop it is converted to parquet. recover_session() converts leftovers.
"""

import dataclasses
import functools
import json
import struct
import subprocess
import time
import types
import typing
from datetime import datetime
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq

from .messages import STREAMS, Event, Quat, Vec3
from .sources import Chunk

# raw.bin record: [pc_rx_time_ns: u64][len | module_id << 24: u32][bytes]. The module id sits in
# the top byte of the length field (a record is far shorter than 16 MB), so files written before
# modules existed (top byte 0) read back as module 0.
RAW_HEADER = struct.Struct("<QI")
RAW_LEN_MASK = 0xFFFFFF


def new_session_dir(base="logs", label=None) -> Path:
    name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if label:
        name += "_" + "".join(c if c.isalnum() or c in "-_" else "-" for c in label)
    path = Path(base) / name
    n = 1
    while path.exists():  # two sessions in the same second (imports, tests)
        n += 1
        path = Path(base) / f"{name}_{n}"
    path.mkdir(parents=True)
    return path


def git_commit() -> str | None:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=Path(__file__).parent,
            capture_output=True,
            text=True,
            timeout=2,
        )
        return out.stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


# ---------- raw.bin ----------


class RawWriter:
    def __init__(self, path):
        self.f = open(path, "wb")

    def write(self, chunk: Chunk):
        n = len(chunk.data)
        if n > RAW_LEN_MASK or not 0 <= chunk.module_id <= 255:
            raise ValueError("chunk too long or module_id out of range for raw.bin")
        header = RAW_HEADER.pack(chunk.pc_rx_time_ns, n | (chunk.module_id << 24))
        self.f.write(header + chunk.data)

    def flush(self):
        self.f.flush()

    def close(self):
        self.f.close()


def read_raw(path):
    """Yields Chunks from raw.bin. A record cut short by a crash is ignored."""
    with open(path, "rb") as f:
        while True:
            header = f.read(RAW_HEADER.size)
            if len(header) < RAW_HEADER.size:
                return
            pc_ns, field = RAW_HEADER.unpack(header)
            n = field & RAW_LEN_MASK
            data = f.read(n)
            if len(data) < n:
                return
            yield Chunk(pc_ns, data, field >> 24)


# ---------- dataclass <-> table row ----------
# Vec3 fields become three columns (accel_x, accel_y, accel_z), Quat four (q_w ... q_z).


@functools.cache  # computed once per message class
def _layout(cls):
    """[(field name, kind, column names, arrow type)] derived from the dataclass fields."""
    hints = typing.get_type_hints(cls)
    out = []
    for f in dataclasses.fields(cls):
        t = hints[f.name]
        if isinstance(t, types.UnionType):  # "X | None"
            t = next(a for a in typing.get_args(t) if a is not type(None))
        if t == Vec3:
            out.append((f.name, "vec", [f"{f.name}_{a}" for a in "xyz"], pa.float64()))
        elif t == Quat:
            out.append((f.name, "vec", [f"{f.name}_{a}" for a in "wxyz"], pa.float64()))
        elif t == list[float]:
            out.append((f.name, "value", [f.name], pa.list_(pa.float64())))
        elif t is dict or typing.get_origin(t) is dict:
            out.append((f.name, "json", [f.name], pa.string()))
        else:
            arrow = {bool: pa.bool_(), int: pa.int64(), float: pa.float64(), str: pa.string()}[t]
            out.append((f.name, "value", [f.name], arrow))
    return out


def schema_for(cls) -> pa.Schema:
    return pa.schema([(c, arrow) for _, _, cols, arrow in _layout(cls) for c in cols])


def to_row(msg) -> dict:
    row = {}
    for name, kind, cols, _ in _layout(type(msg)):
        value = getattr(msg, name)
        if kind == "vec":
            row.update(zip(cols, value if value is not None else [None] * len(cols), strict=True))
        elif kind == "json":
            row[name] = None if value is None else json.dumps(value)
        else:
            row[name] = value
    return row


def from_row(cls, row: dict):
    """Missing columns (sessions written by an older version) become None or the default."""
    kwargs = {}
    for name, kind, cols, _ in _layout(cls):
        if cols[0] not in row:
            continue
        if kind == "vec":
            kwargs[name] = None if row[cols[0]] is None else tuple(row[c] for c in cols)
        elif kind == "json":
            kwargs[name] = None if row[name] is None else json.loads(row[name])
        else:
            kwargs[name] = row[name]
    return cls(**kwargs)


def read_stream(session_dir, cls) -> list:
    """All messages of one stream from <stream>.parquet."""
    path = Path(session_dir) / f"{STREAMS[cls]}.parquet"
    if not path.exists():
        return []
    return [from_row(cls, r) for r in pq.read_table(path).to_pylist()]


# ---------- per-stream writer ----------


class StreamWriter:
    def __init__(self, session_dir: Path, cls):
        self.name = STREAMS[cls]
        self.schema = schema_for(cls)
        self.arrows_path = session_dir / f"{self.name}.arrows"
        self.f = open(self.arrows_path, "wb")
        self.writer = pa.ipc.new_stream(self.f, self.schema)
        self.rows = []

    def add(self, msg):
        self.rows.append(to_row(msg))

    def flush(self):
        if self.rows:
            self.writer.write_batch(pa.RecordBatch.from_pylist(self.rows, schema=self.schema))
            self.f.flush()
            self.rows = []

    def close(self):
        self.flush()
        self.writer.close()
        self.f.close()
        arrows_to_parquet(self.arrows_path)


def arrows_to_parquet(arrows_path: Path):
    batches, schema = [], None
    with open(arrows_path, "rb") as f:
        try:
            reader = pa.ipc.open_stream(f)
            schema = reader.schema
            for batch in reader:
                batches.append(batch)
        except (pa.ArrowInvalid, OSError):
            pass  # file cut short by a crash: keep the complete batches
    if schema is not None:
        table = pa.Table.from_batches(batches, schema=schema)
        pq.write_table(table, arrows_path.with_suffix(".parquet"), compression="zstd")
    arrows_path.unlink()


def recover_session(session_dir) -> list[str]:
    """Convert <stream>.arrows left behind by a crash. Returns the recovered stream names."""
    found = sorted(Path(session_dir).glob("*.arrows"))
    for p in found:
        arrows_to_parquet(p)
    return [p.stem for p in found]


# ---------- recorder ----------


class Recorder:
    """Subscribes to the router and writes one session folder."""

    def __init__(self, session_dir: Path, meta: dict):
        self.dir = Path(session_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.meta = meta
        self.write_meta()
        self.raw = RawWriter(self.dir / "raw.bin")
        self.events = open(self.dir / "events.jsonl", "w")
        self.streams = {}
        self.last_flush = time.monotonic()

    def attach(self, router):
        router.subscribe(Chunk, self.raw.write)
        router.subscribe(Event, self.on_event)
        for cls in STREAMS:
            router.subscribe(cls, self.on_record)

    def on_record(self, msg):
        cls = type(msg)
        if cls not in self.streams:
            self.streams[cls] = StreamWriter(self.dir, cls)
        self.streams[cls].add(msg)

    def on_event(self, ev: Event):
        self.events.write(json.dumps(dataclasses.asdict(ev)) + "\n")

    def flush(self):
        for s in self.streams.values():
            s.flush()
        self.raw.flush()
        self.events.flush()
        self.last_flush = time.monotonic()

    def write_meta(self):
        (self.dir / "meta.json").write_text(json.dumps(self.meta, indent=2, default=str) + "\n")

    def close(self, final: dict):
        for s in self.streams.values():
            s.close()
        self.raw.close()
        self.events.close()
        self.meta |= final
        self.meta["streams"] = sorted(STREAMS[c] for c in self.streams)
        self.write_meta()
