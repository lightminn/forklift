#!/usr/bin/env python3
"""The laptop A2M12 check (docs/design/2026-11-12-real-scan-obstacle-layer.md 1.4-1.5): binds the socket the ROS
adapter sends to, reads scans through ScanFeed for N seconds and reports what the scan path gives -- rate, beams,
angles, unknown (NaN) share, the stamp-to-receipt distribution, refusals and the adapter's drops -- and, with
``--probe``, which side the nearest return is on (the angle convention). Scan checks only: no obstacle layer and no
permission (that is 7b). ``--record`` keeps every received frame before its verdict, for replay.

usage: scan_check.py --socket /tmp/forklift-scan/scan.sock [--seconds 30] [--probe right|left|front|back]
                     [--record out.scan.jsonl] [--out summary.json]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

from forklift_core.sensors.scan_feed import ScanFeed, bind_socket

PROBE_NEAR_M = 1.0  # the probe's box is put this close or closer
SECTORS = {"front": (-math.pi / 4, math.pi / 4), "left": (math.pi / 4, 3 * math.pi / 4),
           "right": (-3 * math.pi / 4, -math.pi / 4)}  # back: the rest (ROS: zero ahead, positive to the left)


def _dist(values) -> dict:
    v = np.asarray(values, dtype=float)
    if v.size == 0:
        return {"p50": None, "p99": None, "max": None}
    return {"p50": float(np.percentile(v, 50)), "p99": float(np.percentile(v, 99)), "max": float(v.max())}


def summarize(scans, feed) -> dict:
    """What the accepted scans show (and what was refused)."""
    out = {"scans": len(scans), "refused": dict(feed.refused)}
    if not scans:
        return out
    beams = [len(s.ranges) for s in scans]
    unknown = sum(int(np.isnan(s.ranges).sum()) for s in scans) / max(1, sum(beams))
    stamps = np.array([s.stamp_mono_ns for s in scans], dtype=float) / 1e9
    recvs = np.array([s.recv_mono_ns for s in scans], dtype=float) / 1e9
    ids = [s.scan_id for s in scans]
    pairs = [s.meta["sys_minus_mono_ns"] for s in scans]
    out.update({
        "beams": {"min": int(min(beams)), "max": int(max(beams))},
        "angle_min_rad": float(np.median([s.angles[0] for s in scans])),
        "angle_max_rad": float(np.median([s.angles[-1] for s in scans])),
        "angle_increment_rad": float(np.median([s.angles[1] - s.angles[0] for s in scans if len(s.angles) > 1])),
        "scan_time_s": float(np.median([s.meta["scan_time"] for s in scans])),
        "range_min_m": float(np.median([s.meta["range_min"] for s in scans])),
        "range_max_m": float(np.median([s.meta["range_max"] for s in scans])),
        "unknown_fraction": float(unknown),
        "period_s": _dist(np.diff(stamps)),
        "receipt_gap_max_s": float(np.diff(recvs).max()) if len(recvs) > 1 else None,
        "stamp_to_receipt_ms": {**_dist([s.stamp_to_recv_ns / 1e6 for s in scans]),
                                "note": "driver stamp to adapter receipt -- not the acquisition age (design 1.1)"},
        "adapter_dropped": int(sum(max(0, b - a - 1) for a, b in zip(ids, ids[1:]))),
        "sys_minus_mono_span_ms": float((max(pairs) - min(pairs)) / 1e6),
    })
    return out


def _side(angle: float) -> str:
    for name, (lo, hi) in SECTORS.items():
        if lo <= angle <= hi:
            return name
    return "back"


def probe(scans, side: str) -> dict:
    """The nearest valid return in each scan; its side must be ``side`` (a box put there, PROBE_NEAR_M or closer)."""
    near = []
    for s in scans:
        if np.all(np.isnan(s.ranges)):
            continue
        i = int(np.nanargmin(s.ranges))
        near.append((float(s.ranges[i]), float(s.angles[i])))
    if not near:
        return {"side": side, "ok": False, "why": "no return in any scan"}
    dist = float(np.median([d for d, _a in near]))
    ang = float(np.angle(np.mean(np.exp(1j * np.array([a for _d, a in near])))))
    if dist > PROBE_NEAR_M:
        return {"side": side, "ok": False, "nearest_m": dist, "nearest_angle_rad": ang,
                "why": f"no return within {PROBE_NEAR_M} m: put the box closer"}
    seen = _side(ang)
    return {"side": side, "ok": seen == side, "seen_side": seen, "nearest_m": dist, "nearest_angle_rad": ang,
            "nearest_angle_deg": math.degrees(ang)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--socket", required=True)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--probe", choices=("right", "left", "front", "back"))
    ap.add_argument("--record")
    ap.add_argument("--out")
    args = ap.parse_args(argv)
    Path(args.socket).parent.mkdir(parents=True, exist_ok=True)
    sock = bind_socket(args.socket)
    record = open(args.record, "w", encoding="utf-8") if args.record else None  # noqa: SIM115
    scans = []
    try:
        feed = ScanFeed(sock=sock, record=record)
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            scans += feed.poll()
            time.sleep(0.005)
        scans += feed.poll()
    finally:
        sock.close()
        Path(args.socket).unlink(missing_ok=True)
        if record is not None:
            record.close()
    summary = summarize(scans, feed)
    if args.probe:
        summary["probe"] = probe(scans, args.probe)
    text = json.dumps(summary, indent=2)
    print(text)
    if args.out:
        Path(args.out).write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
