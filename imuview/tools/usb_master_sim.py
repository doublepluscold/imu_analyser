"""Stand-in for the master ESP32 over USB: no hardware needed to try the GUI or `imu net --serial`.

It makes a pseudo-terminal and writes into it what the real master writes to its USB port:
binary datagrams (src/imuview/netproto.py) and text lines that start with "# " (diagnostics).

    uv run python tools/usb_master_sim.py --modules 3 --gps --link /tmp/ttyIMU
    uv run imu net --serial /tmp/ttyIMU --no-record          # headless
    uv run imu gui                                           # transport "Wi-Fi: майстер ESP (USB)",
                                                             # port /tmp/ttyIMU

Try the failure messages too:  --silent 2   (module 2 has a dead UART: UART_SILENT)
                               --lost 2     (module 2 stops sending: its slave went quiet)
                               --no-slaves  (the master hears nobody)

This is a TEST TOOL (Linux/macOS). The imuview program itself never writes to a serial port; that is
why this file is outside src/ (a test checks that src/ has no write calls next to pyserial).
"""

import argparse
import os
import random
import time
import tty

from imuview.netproto import encode_datagram
from imuview.pipeline import load_config
from imuview.protocol import PARSERS, build_demux
from imuview.sources import Chunk, MultiSimSource


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--modules", type=int, default=3)
    ap.add_argument("--gps", action="store_true", help="modules report a moving GNSS fix")
    ap.add_argument("--seconds", type=float, help="stop after this many seconds")
    ap.add_argument("--loss", type=float, default=0.0, help="drop this fraction of datagrams")
    ap.add_argument("--silent", type=int, nargs="*", default=[], help="modules with a dead UART")
    ap.add_argument("--lost", type=int, nargs="*", default=[], help="modules that went quiet")
    ap.add_argument("--no-slaves", action="store_true", help="master hears no slave at all")
    ap.add_argument("--link", help="also make this symlink to the port, e.g. /tmp/ttyIMU")
    args = ap.parse_args()

    master, slave = os.openpty()
    tty.setraw(slave)  # no echo, no CR/LF translation: the data is binary
    os.set_blocking(master, False)  # like the real master: if nobody reads, data is dropped
    path = os.ttyname(slave)
    if args.link:
        if os.path.lexists(args.link):
            os.remove(args.link)
        os.symlink(path, args.link)
    print(f"master simulator on {path}" + (f" (also {args.link})" if args.link else ""))
    print("connect imuview to that port; Ctrl+C stops")

    def put(data: bytes) -> bool:
        try:
            return os.write(master, data) == len(data)
        except BlockingIOError:
            return False

    def text(line: str):
        put(f"# {line}\n".encode("ascii"))

    config = load_config()
    n = 0 if args.no_slaves else args.modules
    source = MultiSimSource(
        n_modules=max(n, 1),
        rate_hz=80.0,
        realtime=True,
        seconds=args.seconds,
        imu_config=config["imu"],
        gpssol_every=15,
        gps_origin=(50.45, 30.52) if args.gps else None,
    )
    ids = list(source.sims) if n else []
    demuxes = {i: build_demux([PARSERS["mtdata2"](config["imu"])]) for i in source.sims}
    seqs = dict.fromkeys(source.sims, 0)
    rx = dict.fromkeys(source.sims, 0)
    drop = random.Random(1)
    t0 = time.monotonic()
    next_report = t0

    text("master up: wifi mac=AA:BB:CC:DD:EE:FF ch=1 out=usb ids_restored=0")
    try:
        while (items := source.read(0.05)) is not None:
            now = time.monotonic()
            for item in items:
                if not isinstance(item, Chunk) or not ids:
                    continue
                if item.module_id in args.silent or item.module_id in args.lost:
                    continue
                for frame in demuxes[item.module_id].feed(item.data, item.pc_rx_time_ns):
                    seq = seqs[item.module_id]
                    seqs[item.module_id] = (seq + 1) & 0xFFFF
                    rx[item.module_id] += 1
                    if drop.random() >= args.loss:
                        put(encode_datagram(item.module_id, seq, frame.data))
            if now >= next_report:
                next_report += 1.0
                up = int(now - t0)
                text(f"link out=usb slaves={len(ids)} queue=0 qdrop=0 out_err=0 bad_unknown=0")
                for i in ids:
                    lost = i in args.lost
                    silent = i in args.silent
                    uart = 0 if silent else rx[i] * 94
                    diag = "UART_SILENT" if silent else "OK"
                    tail = (
                        "hb=lost"
                        if lost
                        else f"hb=ok up={up} uart={uart} frames={rx[i]} badcs={rx[i] // 2} "
                        f"tx={rx[i]} err=0 nack=0 qdrop=0 ch=1 rst=0 diag={diag}"
                    )
                    text(
                        f"id={i} mac=11:22:33:44:55:{i:02X} rx={rx[i]} bad=0 sent={rx[i]} drop=0 "
                        f"rssi={-40 - 3 * i} {tail}"
                    )
                if not ids:
                    text("no slave heard yet: is it powered, on channel 1, and sending?")
    except KeyboardInterrupt:
        pass
    finally:
        if args.link and os.path.islink(args.link):
            os.remove(args.link)


if __name__ == "__main__":
    main()
