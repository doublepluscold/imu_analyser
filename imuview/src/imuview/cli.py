"""Command line: imu sim | live | replay | reparse | import | info | calib ...

Nothing here ever writes to the serial port (see SerialSource).
"""

import argparse
import json
import sys
import threading
import time
from pathlib import Path

try:  # single key presses from the terminal: not available on Windows
    import os
    import select
    import termios
    import tty
except ImportError:
    termios = None

import numpy as np
import pyarrow.parquet as pq

from .calibration import Calibration
from .logger import new_session_dir, recover_session
from .multi import MultiPipeline
from .pipeline import Pipeline, load_config
from .sources import (
    BytesSource,
    MasterSerialSource,
    MessageSource,
    MultiSimSource,
    RawFileSource,
    SerialSource,
    SimSource,
    UdpSource,
    csv_messages,
    session_source,
)

KEYS_HELP = "keys: c = calibrate gyro, t = tare, r = start/stop recording, q = quit"


class Keyboard:
    """Reads single key presses from the terminal in a background thread."""

    def __init__(self, on_key):
        self.on_key = on_key
        self.fd = sys.stdin.fileno() if termios and sys.stdin.isatty() else None
        self.old = None

    def __enter__(self):
        if self.fd is not None:
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)  # no Enter needed, Ctrl+C still works
            threading.Thread(target=self._loop, daemon=True).start()
        return self

    def _loop(self):
        while True:
            ready, _, _ = select.select([self.fd], [], [], 0.2)
            if ready:
                self.on_key(os.read(self.fd, 1).decode(errors="ignore").lower())

    def __exit__(self, *exc):
        if self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


class TimeLimit:
    """Wraps a source and ends it after `seconds` of wall time."""

    def __init__(self, source, seconds):
        self.source, self.seconds, self.start = source, seconds, None

    def __getattr__(self, name):
        return getattr(self.source, name)

    def read(self, timeout=0.05):
        self.start = self.start or time.monotonic()
        if time.monotonic() - self.start >= self.seconds:
            return None
        return self.source.read(timeout)


def load_calibration(args, config):
    path = getattr(args, "calib", None) or config.get("calib_file")
    return Calibration.load(path) if path else None


def run_pipeline(source, config, args, show_gnss=False, label=None, session_dir=None,
                 keyboard=True):  # fmt: skip
    no_record = getattr(args, "no_record", False)
    no_viewer = getattr(args, "no_viewer", True)
    rrd = getattr(args, "rrd", False)
    label = label or getattr(args, "label", None)
    if not no_record and session_dir is None:
        session_dir = new_session_dir(config["log_dir"], label)
    session_dir = None if no_record else session_dir
    viewer = None
    if not no_viewer or rrd:
        from .viewer import RerunViewer  # imported here so headless runs skip rerun startup

        viewer = RerunViewer(
            config,
            spawn=not no_viewer,
            rrd_path=session_dir / "record.rrd" if rrd and session_dir else None,
            show_gnss=show_gnss,
            wall_offset_ns=time.time_ns() - time.monotonic_ns(),
        )
    calibration = load_calibration(args, config)
    if viewer and calibration and calibration.info.get("g"):
        viewer.set_g(calibration.info["g"])
    pipeline = Pipeline(
        source,
        config,
        viewer,
        record=not no_record,
        session_dir=session_dir,
        calibration=calibration,
        label=label,
    )
    started = time.monotonic()
    try:
        if keyboard:
            print(KEYS_HELP)
            with Keyboard(pipeline.command):
                pipeline.run()
        else:
            pipeline.run()
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.close()  # idempotent; run() closes too, but not if Ctrl+C hit before it
    print_stats(pipeline.stats(), time.monotonic() - started, viewer)
    if session_dir:
        print(f"session: {session_dir}")
    return pipeline, session_dir


def print_stats(stats, wall_seconds, viewer=None):
    print(f"\n--- stats ({wall_seconds:.1f} s wall time) ---")
    for name, m in stats["messages"].items():
        print(f"  {name:12s} {m['count']:9d}  {m['rate_hz']:9.1f} Hz")
    print(f"  frames: {stats['frames']}")
    print(f"  checksum errors: {stats['checksum_errors']}")
    print(f"  parser: {json.dumps(stats['parser'])}")
    print(f"  time sources: {stats['time_sources']}")
    print(f"  unclaimed bytes: {stats['bytes_unclaimed']} ({stats['unclaimed_percent']} %)")
    if viewer:
        print(f"  time spent sending to rerun: {viewer.send_seconds:.2f} s")


def run_multi(source, config, args, label=None):
    """Headless run of several modules: one session, rows tagged with module_id."""
    mp = MultiPipeline(
        source,
        config,
        record=not args.no_record,
        calibration=load_calibration(args, config),
        label=label or args.label,
    )
    started = time.monotonic()
    try:
        with Keyboard(mp.command):
            print(KEYS_HELP)
            mp.run()
    except KeyboardInterrupt:
        pass
    finally:
        mp.close()
    print(f"\n--- stats ({time.monotonic() - started:.1f} s wall time) ---")
    stats = mp.stats()
    for mid, st in stats["modules"].items():
        msgs = {k: v["count"] for k, v in st["messages"].items()}
        print(f"  module {mid}: {msgs} checksum errors: {st['checksum_errors']}")
    if "source" in stats:
        print(f"  source: {stats['source']}")
    return mp


def cmd_net(args):
    config = load_config(args.config)
    if args.serial:  # the master ESP over USB instead of UDP
        source = MasterSerialSource(args.serial)
        shown = []

        def show_link(_line):  # print the status sentence only when it changes
            text = source.link_summary()
            if not shown or shown[-1] != text:
                shown.append(text)
                print(f"[master] {text}")

        source.on_diag = show_link
        print(f"listening on {args.serial} (master ESP over USB, listen-only, nothing is sent)")
    else:
        host = args.host or config["network"]["host"]
        port = args.port or config["network"]["port"]
        source = UdpSource(host, port)
        print(f"listening on udp {host}:{source.port} (listen-only, nothing is ever sent)")
    if args.seconds:
        source = TimeLimit(source, args.seconds)
    run_multi(source, config, args)


def cmd_gui(args):
    from .gui.app import main as gui_main

    argv = []
    for name in ("config", "calib", "models", "source", "session"):
        if getattr(args, name):
            argv += [f"--{name}", getattr(args, name)]
    raise SystemExit(gui_main(argv))


def cmd_sim(args):
    config = load_config(args.config)
    if args.modules > 1:
        source = MultiSimSource(
            n_modules=args.modules,
            rate_hz=args.rate or 80.0,
            fake_gnss=args.with_fake_gnss,
            realtime=not args.fast,
            seconds=args.seconds,
            imu_config=config["imu"],
            corrupt_prob=args.corrupt,
            truncate_prob=args.truncate,
            gpssol_every=args.gpssol_every,
            gps_origin=(50.45, 30.52) if args.gps else None,
        )
        run_multi(source, config, args)
        return
    mt = args.protocol == "mtdata2"
    source = SimSource(
        rate_hz=args.rate or (80.0 if mt else 1000.0),
        n_imu=args.n_imu,
        fake_gnss=args.with_fake_gnss,
        realtime=not args.fast,
        seconds=args.seconds,
        imu_config=config["imu"],
        ftype=int(args.frame_type, 0),
        protocol=args.protocol,
        corrupt_prob=args.corrupt,
        truncate_prob=args.truncate,
        extended_every=args.extended_every,
        gpssol_every=args.gpssol_every if mt else 0,
        gps_origin=(50.45, 30.52) if args.gps and mt else None,
    )
    run_pipeline(source, config, args, show_gnss=args.with_fake_gnss)


def serial_source(args, config):
    port = args.port or config["serial"]["port"]
    baud = args.baud or config["serial"]["baud"]
    source = SerialSource(port, baud)
    print(f"listening on {port} @ {baud} (listen-only, nothing is ever sent)")
    return TimeLimit(source, args.seconds) if args.seconds else source


def cmd_live(args):
    config = load_config(args.config)
    run_pipeline(serial_source(args, config), config, args)


def cmd_replay(args):
    config = load_config(args.config)
    source = session_source(args.session)
    if isinstance(source, RawFileSource):
        source.realtime, source.speed = not args.fast, args.speed
    args.no_record = not args.record
    run_pipeline(source, config, args, label=f"{Path(args.session).name}-replay")


def cmd_reparse(args):
    config = load_config(args.config)
    if args.parser:
        config["parser"] = args.parser
    source = session_source(args.session)
    meta = json.loads((Path(args.session) / "meta.json").read_text())
    label = args.label or f"{meta.get('label') or Path(args.session).name}-reparse"
    out = Path(args.out) if args.out else new_session_dir(config["log_dir"], label)
    out.mkdir(parents=True, exist_ok=True)
    args.no_viewer, args.no_record = True, False
    _, session = run_pipeline(source, config, args, label=label, session_dir=out, keyboard=False)
    add_meta(session, {"reparsed_from": str(args.session)})


def add_meta(session, extra: dict):
    path = Path(session) / "meta.json"
    meta = json.loads(path.read_text())
    path.write_text(json.dumps(meta | extra, indent=2, default=str) + "\n")


def cmd_import_csv(args):
    config = load_config(args.config)
    baud = config["serial"]["baud"]
    msgs = csv_messages(args.file, baud=baud)
    source = MessageSource(msgs, {"type": "import-csv", "file": str(args.file), "baud": baud})
    args.no_viewer, args.no_record = True, False
    _, session = run_pipeline(source, config, args, keyboard=False)
    add_meta(session, {"imported_from": str(args.file)})


def cmd_import_raw(args):
    config = load_config(args.config)
    size = Path(args.file).stat().st_size
    if args.duration:
        byte_rate = size / args.duration
    else:
        byte_rate = args.byte_rate or config["serial"]["baud"] / 10
    source = BytesSource(args.file, byte_rate=byte_rate)
    args.no_viewer, args.no_record = True, False
    _, session = run_pipeline(source, config, args, keyboard=False)
    add_meta(session, {"imported_from": str(args.file)})


def cmd_framing(args):
    from .framing_analysis import analyze
    from .logger import read_raw

    path = Path(args.input)
    duration = args.duration
    if path.is_dir():  # a session: raw.bin records carry arrival times
        chunks = list(read_raw(path / "raw.bin"))
        data = b"".join(c.data for c in chunks)
        if duration is None and len(chunks) > 1:
            duration = (chunks[-1].pc_rx_time_ns - chunks[0].pc_rx_time_ns) * 1e-9
    else:
        data = path.read_bytes()
    report = analyze(data, duration, args.baud)
    text = json.dumps(report, indent=2)
    print(text)
    if args.out:
        Path(args.out).write_text(text + "\n")


def cmd_calib_check(args):
    from . import calib_tools as ct

    config = load_config(args.config)
    results = [ct.check(ct.load(p, config)) for p in args.inputs]
    md = "## Data check (stage 0)\n\n" + ct.check_markdown(results)
    print(md)
    print(f"\nreport: {ct.save_report('check', md, results)}")


def cmd_calib_gyro(args):
    from . import calib_tools as ct

    config = load_config(args.config)
    still = ct.gyro_still(ct.load(args.still, config))
    rots = [ct.gyro_rotation(ct.load(p, config), still["bias"]) for p in args.rotations]
    md = "## Gyro (stage 2)\n\n" + ct.gyro_markdown(still, rots)
    print(md)
    print(f"\nreport: {ct.save_report('gyro', md, {'still': still, 'rotations': rots})}")


def cmd_calib_axes(args):
    from . import calib_tools as ct

    config = load_config(args.config)
    poses = args.poses.split(",") if args.poses else None
    if poses and len(poses) != len(args.sessions):
        raise SystemExit("--poses needs one pose per session")
    result = ct.axes([ct.load(p, config) for p in args.sessions], poses)
    md = "## Axes (stage 1)\n\n" + ct.axes_markdown(result)
    print(md)
    print(f"\nreport: {ct.save_report('axes', md, result)}")
    if args.write:
        if "error" in result:
            raise SystemExit(f"not written: {result['error']}")
        ct.write_board_alignment(args.write, result["board_alignment_deg"])
        print(f"board_alignment_deg written to {args.write}")
    else:
        print("nothing written; after checking the table run again with --write config.toml")


def board_alignment_of(config) -> list:
    imu = config["imu"]
    return {**imu.get("default", {}), **imu.get("0", {})}.get("board_alignment_deg", [0, 0, 0])


def cmd_calib_accel(args):
    from . import calib_tools as ct

    config = load_config(args.config)
    datas = [ct.load(p, config, args.module_id) for p in args.sessions]
    poses = args.poses.split(",") if args.poses else None
    if poses and len(poses) != len(datas):
        raise SystemExit("--poses needs one pose per session")
    samples = ct.pose_samples(datas, poses)
    _, g = ct.detect_units(sum(float(np.linalg.norm(s.mean)) for s in samples) / len(samples))
    if g is None:
        raise SystemExit("units of the accelerometer are not clear (|a| is neither ~g nor ~9.8)")
    fit = ct.fit_accel(samples, g, args.model)
    fit["comparison"] = {
        m: {"rms_after_ms2": f.get("rms_after_ms2"), "errors": f["errors"]}
        for m, f in ct.compare_models(samples, g).items()
    }
    md = "## Accel (stage 3)\n\n" + ct.accel_markdown(fit)
    md += "\n\nmodels side by side (RMS |a|-g after, m/s^2): " + ", ".join(
        f"{m} {c['rms_after_ms2']:.4f}" if c["rms_after_ms2"] is not None else f"{m} n/a"
        for m, c in fit["comparison"].items()
    )
    print(md)
    print(f"\nreport: {ct.save_report('accel', md, fit)}")


def cmd_calib_level(args):
    from . import calib_tools as ct
    from .calibration import Calibration

    config = load_config(args.config)
    d = ct.load(args.session, config, args.module_id)
    s = ct.pose_samples([d], ["level"])[0]
    if args.accel_json:
        fit = json.loads(Path(args.accel_json).read_text())
        b, m = fit["offset_b"], fit["matrix_M"]
    elif args.calib:
        cal = Calibration.load(args.calib)
        b, m = cal.accel_offset, cal.accel_matrix
    else:
        b, m = [0, 0, 0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        print("no accel calibration given: trim computed from the raw reading")
    res = ct.level_trim(s.mean, b, m, board_alignment_of(config))
    md = "## Level\n\n" + json.dumps(res, indent=2)
    print(md)
    print(f"\nreport: {ct.save_report('level', md, res)}")


def _pick(arg, kind):
    from . import calib_tools as ct

    path = Path(arg) if arg else ct.latest_report(kind)
    return (path, json.loads(path.read_text())) if path else (None, None)


def cmd_calib_report(args):
    from . import calib_tools as ct

    (pa, accel), (pl, level), (pg, gyro) = (
        _pick(args.accel, "accel"), _pick(args.level, "level"), _pick(args.gyro, "gyro"),
    )  # fmt: skip
    md = ct.report(accel, level, gyro["still"] if gyro else None)
    print(f"report: {md} (+ .png)   inputs: {[str(x) for x in (pa, pl, pg) if x]}")


def cmd_calib_apply(args):
    """Collect the newest accel / gyro / level results into calib/imu_calib.json."""
    from . import calib_tools as ct

    (pa, accel), (pl, level), (pg, gyro) = (
        _pick(args.accel, "accel"), _pick(args.level, "level"), _pick(args.gyro, "gyro"),
    )  # fmt: skip
    if accel is None and gyro is None and level is None:
        raise SystemExit("nothing to apply: run calib accel / gyro / level first")
    if accel and accel["errors"]:
        raise SystemExit(f"accel fit has errors, not applied: {accel['errors']}")
    if accel and "offset_b" not in accel:
        raise SystemExit("accel report has no fit")
    sources = {"accel": str(pa), "gyro": str(pg), "level": str(pl)}
    cal = ct.build_calibration(accel, gyro["still"]["bias"] if gyro else None, level,
                               {"sources": sources})  # fmt: skip
    sha = cal.save(args.out)
    print(f"written {args.out} (sha256 {sha[:12]}) from {sources}")
    print("use it with: --calib", args.out, " or calib_file in the config")


def cmd_calib_record(args):
    config = load_config(args.config)
    args.no_record = False
    args.no_viewer = not args.viewer
    run_pipeline(serial_source(args, config), config, args, keyboard=False)


def cmd_info(args):
    session = Path(args.session)
    recovered = recover_session(session)
    if recovered:
        print(f"recovered after crash: {recovered}")
    meta = json.loads((session / "meta.json").read_text())
    print(f"session:   {session}")
    print(f"start:     {meta.get('start_time')}   stop: {meta.get('stop_time', '? (crashed)')}")
    print(f"source:    {meta.get('source')}")
    print(f"parsers:   {meta.get('parsers')}   estimator: {meta.get('estimator')}")
    print(f"raw.bin:   {(session / 'raw.bin').stat().st_size} bytes")
    for p in sorted(session.glob("*.parquet")):
        print(f"{p.name:15s}{pq.ParquetFile(p).metadata.num_rows} rows")
    if "stats" in meta:
        print(json.dumps(meta["stats"], indent=2))


def add_run_options(p, record=True):
    p.add_argument("--config", help="TOML config file")
    p.add_argument("--calib", help="host calibration JSON to apply (raw values stay raw)")
    p.add_argument("--no-viewer", action="store_true", help="do not open rerun")
    if record:
        p.add_argument("--no-record", action="store_true", help="start without recording")
        p.add_argument("--label", help="label saved in meta.json and in the folder name")
    p.add_argument("--rrd", action="store_true", help="also save record.rrd in the session")


def add_port_options(p):
    p.add_argument("--port", help="serial port (default from config: /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, help="default from config: 115200")
    p.add_argument("--seconds", type=float, help="stop after this many seconds")


def main(argv=None):
    ap = argparse.ArgumentParser(prog="imu", description="IMU live viewer, logger and replay")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("gui", help="desktop GUI (live, calibration, analysis)")
    p.add_argument("--config")
    p.add_argument("--calib")
    p.add_argument("--models")
    p.add_argument("--source", choices=["master", "udp", "serial", "sim"])
    p.add_argument("--session")
    p.set_defaults(func=cmd_gui)

    p = sub.add_parser("sim", help="simulated sensor module")
    p.add_argument("--protocol", choices=["ref", "mtdata2"], default="ref")
    p.add_argument("--rate", type=float, help="IMU rate [Hz] (default 1000 ref, 80 mtdata2)")
    p.add_argument("--corrupt", type=float, default=0.0, help="mtdata2: P(frame gets a bad byte)")
    p.add_argument("--truncate", type=float, default=0.0, help="mtdata2: P(frame is cut short)")
    p.add_argument("--extended-every", type=int, default=0, help="mtdata2: every Nth frame > 254 B")
    p.add_argument("--gpssol-every", type=int, default=15, help="mtdata2: GpsSol frame every N")
    p.add_argument("--n-imu", type=int, default=3)
    p.add_argument("--modules", type=int, default=1, help="mtdata2: simulate N modules at once")
    p.add_argument("--gps", action="store_true", help="mtdata2: valid GNSS fix, moves on a circle")
    p.add_argument("--with-fake-gnss", action="store_true", help="also emit GnssFix at 5 Hz")
    p.add_argument("--seconds", type=float, help="stop after this much simulated time")
    p.add_argument("--fast", action="store_true", help="as fast as possible, not real time")
    p.add_argument("--frame-type", default="0x01", help="0x01 int16 .. 0x04 float+quaternion")
    add_run_options(p)
    p.set_defaults(func=cmd_sim)

    p = sub.add_parser("live", help="read the device (listen-only), view and record")
    add_port_options(p)
    add_run_options(p)
    p.set_defaults(func=cmd_live)

    p = sub.add_parser("net", help="listen to the master ESP (several modules), record")
    p.add_argument("--serial", help="read the master over this USB serial port instead of UDP")
    p.add_argument("--host", help="address to bind (default from config: 0.0.0.0)")
    p.add_argument("--port", type=int, help="default from config: 5005")
    p.add_argument("--seconds", type=float, help="stop after this many seconds")
    p.add_argument("--config", help="TOML config file")
    p.add_argument("--calib", help="host calibration JSON applied to every module")
    p.add_argument("--no-record", action="store_true")
    p.add_argument("--label")
    p.set_defaults(func=cmd_net)

    p = sub.add_parser("replay", help="view a recorded session again")
    p.add_argument("session")
    p.add_argument("--speed", type=float, default=1.0)
    p.add_argument("--fast", action="store_true", help="as fast as possible")
    p.add_argument("--record", action="store_true", help="also save the replay as a new session")
    add_run_options(p, record=False)
    p.set_defaults(func=cmd_replay)

    p = sub.add_parser("reparse", help="run a session through the parsers again -> new session")
    p.add_argument("session")
    p.add_argument("--parser", help="parser to use instead of the config's")
    p.add_argument("--out", help="output folder (default: new session in log_dir)")
    p.add_argument("--label")
    p.add_argument("--config", help="TOML config file")
    p.add_argument("--calib", help="host calibration JSON to apply")
    p.set_defaults(func=cmd_reparse)

    p = sub.add_parser("import", help="turn files of mtdata2_decoder.py into sessions")
    isub = p.add_subparsers(dest="kind", required=True)
    q = isub.add_parser("csv", help="CSV (t,roll,pitch,yaw,ax,...,x2060)")
    q.add_argument("file")
    q.add_argument("--label", required=True)
    q.add_argument("--config")
    q.add_argument("--calib")
    q.set_defaults(func=cmd_import_csv)
    q = isub.add_parser("raw", help="raw byte dump (no timestamps: time is synthetic)")
    q.add_argument("file")
    q.add_argument("--label", required=True)
    q.add_argument("--byte-rate", type=float, help="bytes/s for synthetic time (default baud/10)")
    q.add_argument("--duration", type=float, help="recording length [s]; sets the byte rate")
    q.add_argument("--config")
    q.add_argument("--calib")
    q.set_defaults(func=cmd_import_raw)

    p = sub.add_parser("calib", help="calibration tools")
    csub = p.add_subparsers(dest="calib_cmd", required=True)
    q = csub.add_parser("record", help="passive recording of one calibration session")
    q.add_argument("--label", required=True, help="e.g. still, rot_x, pose_level")
    add_port_options(q)
    q.add_argument("--viewer", action="store_true", help="also open rerun")
    q.add_argument("--config")
    q.add_argument("--calib", help="apply this calibration while recording (raw is kept)")
    q.set_defaults(func=cmd_calib_record)

    p = sub.add_parser("framing", help="why frames fail the checksum (raw dump or session)")
    p.add_argument("input", help="raw byte dump, or a session folder")
    p.add_argument("--duration", type=float, help="recording length [s] (dumps have no time)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--out", help="also write the JSON report here")
    p.set_defaults(func=cmd_framing)

    q = csub.add_parser("accel", help="stage 3: accel offset and scale from still poses")
    q.add_argument("sessions", nargs="+", help="one still session per pose (>= 6)")
    q.add_argument("--poses", help="comma list of pose names, default from the labels")
    q.add_argument("--model", choices=["six", "nine", "axis"], default="six")
    q.add_argument("--module-id", type=int, help="module of a multi-module session")
    q.add_argument("--config")
    q.set_defaults(func=cmd_calib_accel)
    q = csub.add_parser("level", help="roll / pitch trim from the level pose")
    q.add_argument("session")
    q.add_argument("--accel-json", help="accel report to calibrate with (else --calib, else none)")
    q.add_argument("--calib")
    q.add_argument("--module-id", type=int)
    q.add_argument("--config")
    q.set_defaults(func=cmd_calib_level)
    for name, func, text in (("report", cmd_calib_report, "markdown + PNG of the newest results"),
                             ("apply", cmd_calib_apply, "write calib/imu_calib.json")):  # fmt: skip
        q = csub.add_parser(name, help=text)
        q.add_argument("--accel", help="accel report json (default: newest)")
        q.add_argument("--gyro", help="gyro report json (default: newest)")
        q.add_argument("--level", help="level report json (default: newest)")
        if name == "apply":
            q.add_argument("--out", default="calib/imu_calib.json")
        q.set_defaults(func=func)
    q = csub.add_parser("check", help="stage 0: rate, intervals, frozen values, units, repeats")
    q.add_argument("inputs", nargs="+", help="sessions, CSVs or raw dumps")
    q.add_argument("--config")
    q.set_defaults(func=cmd_calib_check)
    q = csub.add_parser("gyro", help="stage 2: bias, noise, stability; scale from rotations")
    q.add_argument("still", help="still session (>= 30 s)")
    q.add_argument("rotations", nargs="*", help="rot_* sessions: still, one rotation, still")
    q.add_argument("--config")
    q.set_defaults(func=cmd_calib_gyro)
    q = csub.add_parser("axes", help="stage 1: sensor -> body mapping from still poses")
    q.add_argument("sessions", nargs="+")
    q.add_argument("--poses", help="comma list, one per session: level,back,left,right,"
                                   "nosedown,noseup (default: taken from the labels)")  # fmt: skip
    q.add_argument("--write", metavar="CONFIG", help="write board_alignment_deg to this TOML")
    q.add_argument("--config")
    q.set_defaults(func=cmd_calib_axes)

    p = sub.add_parser("info", help="print session stats")
    p.add_argument("session")
    p.set_defaults(func=cmd_info)

    args = ap.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
