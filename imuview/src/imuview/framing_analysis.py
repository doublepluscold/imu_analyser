"""Why do MTData2 frames fail the checksum? Offline analysis of a raw byte stream.

Walks the stream like the framer does (valid frame: skip it; bad checksum: slide one byte) and
records every sync attempt with its position. Then tests the hypotheses:

  extended / other MIDs   frames with LEN 0xFF, or MIDs other than MTData2
  real header vs false    a bad attempt whose (MID, LEN) matches valid frames is a real frame
                          with a bad sum; anything else is a false sync (FA FF inside data)
  truncated / full length distance from a real bad header to the next header, vs its length
  byte accounting         bytes in valid frames, bad full frames, truncated frames, and
                          unclaimed runs (with their most common contents)
  periodicity             P(bad) by position after each minority frame kind, run lengths
  checksum lag            does the CS of a bad frame equal the correct CS of the frame at lag
                          +-k (a buffer rewritten while it is being sent: ping-pong race)?
  repeats                 are sample bytes identical to the frame at lag +-k?
  plausibility            values of bad frames (decoded ignoring the sum) vs good frames
"""

from collections import Counter
from dataclasses import dataclass

import mtdata2_decoder as vendor
import numpy as np

from .protocol import BAD_CHECKSUM, NEED_MORE, NO_MATCH
from .protocol_mtdata2 import Mtdata2Framer, header

LAGS = (-3, -2, -1, 1, 2, 3)


@dataclass
class Attempt:
    pos: int
    mid: int
    ln: int
    total: int  # claimed frame length
    ok: bool
    kind: str = ""  # "valid" | "bad" (real header) | "false"
    seg: int = 0  # bytes until the next frame header (valid or real bad)


def scan(data: bytes) -> list[Attempt]:
    framer = Mtdata2Framer()
    out, pos = [], 0
    n = len(data)
    while pos < n:
        pos = data.find(b"\xfa\xff", pos)
        if pos < 0:
            break
        r = framer.match(data, pos)
        if r == NO_MATCH or r == NEED_MORE:  # NEED_MORE here = runs past the end of the file
            if r == NEED_MORE and pos + 6 <= n:
                mid, ln, hdr = header(data, pos)
                out.append(Attempt(pos, mid, ln, hdr + ln + 1, False, "cut"))
            pos += 1
            continue
        mid, ln, hdr = header(data, pos)
        if r == BAD_CHECKSUM:
            out.append(Attempt(pos, mid, ln, hdr + ln + 1, False))
            pos += 1
        else:
            out.append(Attempt(pos, mid, ln, r, True, "valid"))
            pos += r
    return out


def correct_cs(frame: bytes) -> int:
    return -sum(frame[1:-1]) & 0xFF


def analyze(data: bytes, duration_s: float | None = None, baud: int = 115200) -> dict:
    atts = scan(data)
    valid_kinds = Counter((a.mid, a.ln) for a in atts if a.ok)
    for a in atts:
        if not a.ok and not a.kind:
            a.kind = "bad" if (a.mid, a.ln) in valid_kinds else "false"
    frames = [a for a in atts if a.kind in ("valid", "bad")]  # real frame starts, in order
    for a, b in zip(frames, frames[1:] + [None], strict=True):
        a.seg = (b.pos if b else len(data)) - a.pos

    bad = [a for a in frames if a.kind == "bad"]
    truncated = [a for a in bad if a.seg < a.total]
    full = [a for a in bad if a.seg >= a.total]
    valid = [a for a in frames if a.ok]

    # byte accounting
    covered = np.zeros(len(data), dtype=bool)
    for a in frames:
        covered[a.pos : a.pos + min(a.total, a.seg)] = True
    lead = frames[0].pos if frames else len(data)
    unclaimed_runs = Counter()
    for a in frames:
        end = a.pos + min(a.total, a.seg)
        nxt = a.pos + a.seg
        if nxt > end:
            unclaimed_runs[bytes(data[end:nxt])] += 1
    b_valid = sum(a.total for a in valid)
    b_bad_full = sum(a.total for a in full)
    b_trunc = sum(a.seg for a in truncated)

    # false syncs inside valid frames
    fa_inside = sum(data.count(b"\xfa\xff", a.pos + 1, a.pos + a.total) for a in valid)

    main_kind = valid_kinds.most_common(1)[0][0] if valid_kinds else None
    report = {
        "bytes": len(data),
        "sync_attempts": len(atts),
        "valid_frames": {f"0x{m:02X}/{n}": c for (m, n), c in sorted(valid_kinds.items())},
        "bad_real_header": dict(Counter(f"0x{a.mid:02X}/{a.ln}" for a in bad).most_common()),
        "false_syncs": len([a for a in atts if a.kind == "false"]),
        "false_sync_kinds": dict(
            Counter(f"0x{a.mid:02X}/{a.ln}" for a in atts if a.kind == "false").most_common(5)
        ),
        "fa_ff_inside_valid_frames": fa_inside,
        "extended_frames": sum(1 for a in atts if a.ln > 254 or a.total - a.ln == 7),
        "other_mids": dict(Counter(f"0x{a.mid:02X}" for a in valid if a.mid != 0x36)),
        "bad_full_length": len(full),
        "bad_truncated": len(truncated),
        "bad_fraction": round(len(bad) / len(frames), 4) if frames else None,
        "bad_fraction_of_sync_attempts": round((len(atts) - len(valid)) / len(atts), 4)
        if atts
        else None,  # fmt: skip
        "byte_accounting": {
            "valid_frames": b_valid,
            "bad_full_frames": b_bad_full,
            "truncated_frames": b_trunc,
            "lead_before_first_frame": lead,
            "unclaimed_between_frames": sum(len(k) * v for k, v in unclaimed_runs.items()),
            "uncovered_total": int((~covered).sum()),
        },
        "unclaimed_runs_top": [
            {"len": len(k), "count": v, "hex": k[:32].hex()}
            for k, v in unclaimed_runs.most_common(5)
        ],
    }
    if duration_s:
        report["rates"] = {
            "duration_s": duration_s,
            "bytes_per_s": round(len(data) / duration_s, 1),
            "line_capacity_bytes_per_s": baud / 10,
            "line_use_percent": round(100 * len(data) / duration_s / (baud / 10), 1),
            "frames_per_s": {
                f"0x{m:02X}/{n}": round(
                    sum(1 for a in frames if (a.mid, a.ln) == (m, n)) / duration_s, 2
                )
                for (m, n) in valid_kinds
            },
            "valid_frames_per_s": round(len(valid) / duration_s, 2),
        }
    if main_kind:
        report |= _sequence_tests(data, frames, main_kind)
    return report


def _sequence_tests(data, frames, main_kind) -> dict:
    main = [a for a in frames if (a.mid, a.ln) == main_kind]
    seq = "".join("." if a.ok else "X" for a in main)

    # P(bad) by position after each minority frame kind
    after = {}
    for kind in {(a.mid, a.ln) for a in frames} - {main_kind}:
        tot, badc, last = Counter(), Counter(), None
        k = 0
        for a in frames:
            if (a.mid, a.ln) == kind:
                last = k
            elif (a.mid, a.ln) == main_kind:
                if last is not None and k - last <= 25:
                    tot[k - last] += 1
                    badc[k - last] += not a.ok
                k += 1
        after[f"0x{kind[0]:02X}/{kind[1]}"] = {
            p: round(badc[p] / tot[p], 2) for p in sorted(tot) if tot[p] >= 10
        }
    runs = Counter(len(r) for r in seq.replace(".", " ").split())
    p_bad = seq.count("X") / len(seq) if seq else 0
    xx = sum(1 for a, b in zip(seq, seq[1:], strict=False) if a == b == "X")
    bad_idx = [i for i, c in enumerate(seq) if c == "X"]
    gaps = Counter(b - a for a, b in zip(bad_idx, bad_idx[1:], strict=False))

    # checksum lag and repeats (main kind only, full frames)
    full = [bytes(data[a.pos : a.pos + a.total]) if a.seg >= a.total else None for a in main]
    cs_lag, rep_good, rep_bad = Counter(), Counter(), Counter()
    n_bad = n_good = 0
    for k in range(3, len(main) - 3):
        f = full[k]
        if f is None:
            continue
        payload = f[4:-1]
        if not main[k].ok:
            n_bad += 1
            for lag in LAGS:
                g = full[k + lag]
                if g is not None and correct_cs(g) == f[-1]:
                    cs_lag[lag] += 1
        else:
            n_good += 1
        for lag in LAGS:
            g = full[k + lag]
            if g is not None and g[4:-1] == payload:
                (rep_good if main[k].ok else rep_bad)[lag] += 1

    # plausibility of values in bad frames (decoded without the sum)
    def norms(ok):
        out = []
        for a, f in zip(main, full, strict=True):
            if f is None or a.ok != ok:
                continue
            p = vendor.parse_payload(f[4:-1])
            if isinstance(p.get("Acc"), tuple):
                out.append(float(np.linalg.norm(p["Acc"])))
        return np.array(out)

    good_n, bad_n = norms(True), norms(False)

    def summary(x):
        if len(x) == 0:
            return None
        return {"n": len(x), "mean": round(float(x.mean()), 5), "std": round(float(x.std()), 5)}

    return {
        "main_kind": f"0x{main_kind[0]:02X}/{main_kind[1]}",
        "p_bad_after_kind": after,
        "bad_run_lengths": dict(sorted(runs.items())),
        "p_bad": round(p_bad, 4),
        "p_bad_given_prev_bad": round(xx / max(1, seq[:-1].count("X")), 4),
        "bad_spacing_top": dict(gaps.most_common(8)),
        "cs_equals_correct_cs_of_frame_at_lag": {
            "bad_frames": n_bad,
            "chance_per_lag": round(n_bad / 256, 1),
            "hits": dict(sorted(cs_lag.items())),
        },
        "payload_identical_to_frame_at_lag": {
            "good_frames": n_good,
            "good_hits": dict(sorted(rep_good.items())),
            "bad_frames": n_bad,
            "bad_hits": dict(sorted(rep_bad.items())),
        },
        "accel_norm_good": summary(good_n),
        "accel_norm_bad": summary(bad_n),
    }
