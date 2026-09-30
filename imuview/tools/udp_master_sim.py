"""Stand-in for the master ESP while its firmware does not exist: simulated MTData2 modules are
sent to imuview as UDP datagrams (format in src/imuview/netproto.py), one frame per datagram.

    uv run python tools/udp_master_sim.py --modules 3 --gps [--host 127.0.0.1] [--port 5005]

This is a TEST TOOL. The imuview program itself never sends anything; that is why this file is
outside src/ (a test checks that src/ contains no send calls).
"""

import argparse
import socket
import time

from imuview.netproto import encode_datagram
from imuview.pipeline import load_config
from imuview.protocol import PARSERS, build_demux
from imuview.sources import Chunk, MultiSimSource


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--modules", type=int, default=3)
    ap.add_argument("--gps", action="store_true", help="modules report a moving GNSS fix")
    ap.add_argument("--seconds", type=float, help="stop after this many seconds")
    ap.add_argument("--loss", type=float, default=0.0, help="drop this fraction of datagrams")
    args = ap.parse_args()

    config = load_config()
    source = MultiSimSource(
        n_modules=args.modules,
        rate_hz=80.0,
        realtime=True,
        seconds=args.seconds,
        imu_config=config["imu"],
        gpssol_every=15,
        gps_origin=(50.45, 30.52) if args.gps else None,
    )
    demuxes = {i: build_demux([PARSERS["mtdata2"](config["imu"])]) for i in source.sims}
    seqs = dict.fromkeys(source.sims, 0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    drop = __import__("random").Random(1)
    print(f"sending {args.modules} simulated modules to udp {args.host}:{args.port}, Ctrl+C stops")
    try:
        while (items := source.read(0.05)) is not None:
            for item in items:
                if not isinstance(item, Chunk):
                    continue
                for frame in demuxes[item.module_id].feed(item.data, item.pc_rx_time_ns):
                    seq = seqs[item.module_id]
                    seqs[item.module_id] = (seq + 1) & 0xFFFF
                    if drop.random() >= args.loss:
                        data = encode_datagram(item.module_id, seq, frame.data)
                        sock.sendto(data, (args.host, args.port))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
