#!/usr/bin/env python3
"""11-17 hardware day verdicts (docs/plans/2026-11-17-hw-day.md D): the criteria fixed before the measurement, one
PASS / FAIL line per item and a last line ALL PASS / NOT ALL PASS. School scale: Hz, counts, cm, gaps.

usage: judge.py a2m12 SUMMARY.json [--minutes N]     scan_check summary (A1)
       judge.py probe SUMMARY.json                   scan_check --probe summary (A3)
       judge.py permit TIMELINE.jsonl                tools/hwday/scan_permit_check.py timeline (A4)
       judge.py stall RECORD.jsonl --phase idle|load teleop bridge record (B5)
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

# A1 -- the A2M12 itself (nominal 10 Hz; the 7b permit closes when the scan period passes 1.5 x 0.1 s, when silence
# passes 0.35 s and drops a scan whose stamp-to-receipt passes 0.15 s, config/real/obstacle_layer_real.yaml)
A2M12 = {
    "period_p50_s": (0.08, 0.125),        # 8-12.5 Hz
    "period_p99_max_s": 0.15,             # the permit's slow-period line
    "receipt_gap_max_s": 0.35,            # the permit's silence line: no gap this long in the whole run ("끊김 0")
    "stamp_to_receipt_p99_max_ms": 150.0,  # the permit drops later scans
    "beams_min": 400,                     # points per revolution (catalogue 16 kHz / 10 Hz = 1600)
    "valid_fraction_min": 0.5,            # indoor room; NaN = no return
    "refused_max": 10,                    # the clock pair's first ESTABLISH_COUNT (10) scans only
    "adapter_dropped_max": 0,
}
PERMIT = {"empty_open_min": 0.9, "blocked_s_min": 3.0, "end_open_min": 0.9}
STALL = {"idle_over_40ms_max": 0, "load_over_80ms_max": 0}


def line(ok: bool, what: str, value) -> bool:
    print(f"{'PASS' if ok else 'FAIL'}  {what}: {value}")
    return ok


def a2m12(path: Path, minutes: float) -> bool:
    s = json.loads(path.read_text())
    if not s.get("scans"):
        line(False, "scans", f"none ({s.get('refused')})")
        return False
    if s.get("synthetic_frames") or s.get("not_for_judgement") or s.get("synthetic_inputs"):
        line(False, "synthetic frames", f"{s.get('synthetic_frames')} (a rehearsal sender, not the A2M12)")
        return False
    c = A2M12
    p50, p99 = s["period_s"]["p50"], s["period_s"]["p99"]
    oks = [
        line(c["period_p50_s"][0] <= p50 <= c["period_p50_s"][1], "rotation (period p50)",
             f"{p50 * 1e3:.1f} ms = {1 / p50:.2f} Hz"),
        line(p99 <= c["period_p99_max_s"], "period p99", f"{p99 * 1e3:.1f} ms (max {s['period_s']['max'] * 1e3:.1f})"),
        line(s["receipt_gap_max_s"] <= c["receipt_gap_max_s"], f"no cut over {minutes:g} min (largest receipt gap)",
             f"{s['receipt_gap_max_s'] * 1e3:.1f} ms"),
        line(s["stamp_to_receipt_ms"]["p99"] <= c["stamp_to_receipt_p99_max_ms"], "stamp->receipt p99",
             f"{s['stamp_to_receipt_ms']['p99']:.1f} ms"),
        line(s["beams"]["min"] >= c["beams_min"], "points per revolution", f"{s['beams']['min']}-{s['beams']['max']}"),
        line(1 - s["unknown_fraction"] >= c["valid_fraction_min"], "valid range fraction",
             f"{1 - s['unknown_fraction']:.3f}"),
        line(sum(s["refused"].values()) <= c["refused_max"], "refused scans", s["refused"]),
        line(s["adapter_dropped"] <= c["adapter_dropped_max"], "adapter drops (scan_id gaps)", s["adapter_dropped"]),
        line(s["scans"] >= 0.9 * minutes * 60 * 10 * 0.8, "scans received", f"{s['scans']} in {minutes:g} min"),
    ]
    print(f"info  angles {s['angle_min_rad']:.3f}..{s['angle_max_rad']:.3f} rad, increment "
          f"{s['angle_increment_rad']:.5f}, range {s['range_min_m']}..{s['range_max_m']} m, scan_time "
          f"{s['scan_time_s']:.3f} s")
    return all(oks)


def probe(path: Path) -> bool:
    p = json.loads(path.read_text()).get("probe") or {}
    return line(bool(p.get("ok")), f"angle convention ({p.get('side')})",
                {k: p.get(k) for k in ("seen_side", "support", "nearest_m", "nearest_angle_deg", "why") if k in p})


def permit(path: Path) -> bool:
    rows = [json.loads(x) for x in path.read_text().splitlines() if x.strip()]
    rows = [r for r in rows if "t_s" in r]
    if not rows:
        return line(False, "timeline", "empty")
    t = np.array([r["t_s"] for r in rows])
    fwd = np.array([r["forward"] > 0 for r in rows])
    empty = fwd[(t >= 5) & (t < 20)]
    end = fwd[t >= t.max() - 10]
    # the longest closed stretch in the middle (someone standing in front)
    mid = (t >= 20) & (t < t.max() - 10)
    best = run = 0.0
    prev = None
    for ti, f, m in zip(t, fwd, mid, strict=True):
        if m and not f:
            run = run + (ti - prev if prev is not None else 0.0)
            best = max(best, run)
        else:
            run = 0.0
        prev = ti
    c = PERMIT
    return all([
        line(len(empty) > 0 and empty.mean() >= c["empty_open_min"], "empty floor opens forward (5-20 s)",
             f"{empty.mean() if len(empty) else 0:.2f} of samples"),
        line(best >= c["blocked_s_min"], "a person in front closes it (20 s to the last 10 s)", f"{best:.1f} s closed"),
        line(len(end) > 0 and end.mean() >= c["end_open_min"], "clear again reopens (last 10 s)",
             f"{end.mean() if len(end) else 0:.2f} of samples"),
    ])


def stall(path: Path, phase: str) -> bool:
    rows = [json.loads(x) for x in path.read_text().splitlines() if x.strip()]
    starts = np.array([r["cycle_start_ns"] for r in rows if "cycle_start_ns" in r], dtype=np.int64)
    if len(starts) < 100:
        return line(False, "cycles", len(starts))
    gaps = np.diff(starts) / 1e6
    gc = [r.get("gc_ms", 0.0) for r in rows if "cycle_start_ns" in r]
    print(f"info  {phase}: {len(starts)} cycles, gap p50 {np.percentile(gaps, 50):.1f} p99 {np.percentile(gaps, 99):.1f} "
          f"max {gaps.max():.1f} ms, >40 ms {int((gaps > 40).sum())}, >80 ms {int((gaps > 80).sum())}, "
          f">124 ms {int((gaps > 124).sum())}, gc max {max(gc):.1f} ms")
    if phase == "idle":
        return line(int((gaps > 40).sum()) <= STALL["idle_over_40ms_max"], "idle: no cycle gap over 40 ms",
                    int((gaps > 40).sum()))
    return line(int((gaps >= 80).sum()) <= STALL["load_over_80ms_max"], "load: no cycle gap of 80 ms or more",
                int((gaps >= 80).sum()))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="kind", required=True)
    a = sub.add_parser("a2m12")
    a.add_argument("path", type=Path)
    a.add_argument("--minutes", type=float, default=10)
    b = sub.add_parser("probe")
    b.add_argument("path", type=Path)
    c = sub.add_parser("permit")
    c.add_argument("path", type=Path)
    d = sub.add_parser("stall")
    d.add_argument("path", type=Path)
    d.add_argument("--phase", choices=("idle", "load"), required=True)
    args = ap.parse_args(argv)
    try:
        ok = {"a2m12": lambda: a2m12(args.path, args.minutes), "probe": lambda: probe(args.path),
              "permit": lambda: permit(args.path), "stall": lambda: stall(args.path, args.phase)}[args.kind]()
    except (OSError, ValueError, KeyError) as e:
        print(f"FAIL  cannot judge {args.path}: {e}")
        ok = False
    print("ALL PASS" if ok else "NOT ALL PASS")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
