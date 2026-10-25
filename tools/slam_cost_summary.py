#!/usr/bin/env python3
"""Summarise a SLAM cost replay (tools/slam_remap_drive.py output) for the slam_link_failed plan
(docs/plans/2026-10-25-slam-link-failed.md 3-4).

Cost: keyframe slam_wall_s per spliced unit, its maximum, the mean of the last 10 % of keyframes, and a least-squares
line against sim time extrapolated to sim 8 h (the plan's 350 passes at the measured 83 s pass period) -- a change
that only flattens the finite window still shows its slope there. Accuracy: the SLAM pose (the reply's map<-odom
composed with the odometry sent) against the record's truth, no alignment, position and heading, per unit and its
growth. Validity (an R0 replay against the live run): the same keyframe stamps as the live bridge processed, and the
replay's mean cost per tenth of the run within 0.7-1.4 of the live one.

    python tools/slam_cost_summary.py DRIVE_JSONL [--live-bridge DIR] > summary.json
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path

EIGHT_HOURS_S = 8 * 3600.0
RATIO_LO, RATIO_HI = 0.7, 1.4


def _fit(xs, ys):
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx else 0.0
    return my - b * mx, b


def cost_summary(rows: list) -> dict:
    t = [float(r["stamp_s"]) for r in rows]
    w = [float(r["slam_wall_s"]) for r in rows]
    a, b = _fit(t, w)
    units = sorted({int(r.get("unit", 0)) for r in rows})
    tail = w[-max(1, len(w) // 10):]
    return {"keyframes": len(rows), "max_s": max(w), "last_tenth_mean_s": statistics.fmean(tail),
            "unit_mean_s": [statistics.fmean(float(r["slam_wall_s"]) for r in rows if int(r.get("unit", 0)) == u)
                            for u in units],
            "slope_s_per_s": b, "intercept_s": a, "at_8h_s": a + b * EIGHT_HOURS_S}


def slam_pose(r) -> tuple:
    mx, my, myaw = (float(v) for v in r["map_from_odom"])
    ox, oy, oyaw = (float(v) for v in r["odom_base"])
    c, s = math.cos(myaw), math.sin(myaw)
    return (mx + c * ox - s * oy, my + s * ox + c * oy, myaw + oyaw)


def accuracy(rows: list) -> dict:
    pos, yaw, by_unit = [], [], {}
    for r in rows:
        x, y, th = slam_pose(r)
        tx, ty, tth = (float(v) for v in r["truth_base"])
        e = math.hypot(x - tx, y - ty)
        pos.append(e)
        yaw.append(abs(math.degrees(math.atan2(math.sin(th - tth), math.cos(th - tth)))))
        by_unit.setdefault(int(r.get("unit", 0)), []).append(e)
    unit_med = [statistics.median(by_unit[u]) for u in sorted(by_unit)]
    return {"pos_median_m": statistics.median(pos), "pos_max_m": max(pos), "yaw_median_deg": statistics.median(yaw),
            "yaw_max_deg": max(yaw), "unit_pos_median_m": unit_med, "growth_m": unit_med[-1] - unit_med[0]}


def validity(rows: list, live: list) -> dict:
    live_kf = [r for r in live if r.get("status") == "processed"]
    same = [round(float(r["stamp_s"]), 6) for r in rows] == [round(float(r["stamp_s"]), 6) for r in live_kf]
    ratios = []
    if same and live_kf:
        n = len(live_kf)
        for k in range(10):
            lo, hi = k * n // 10, max(k * n // 10 + 1, (k + 1) * n // 10)
            ratios.append(statistics.fmean(float(r["slam_wall_s"]) for r in rows[lo:hi])
                          / statistics.fmean(float(r["wall_s"]) for r in live_kf[lo:hi]))
    late = ratios[3:]  # the first tenths are 30-40 ms, where the ratio is noise
    return {"same_keyframes": same, "tenth_ratio": ratios,
            "ok": bool(same and late and all(RATIO_LO <= x <= RATIO_HI for x in late))}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("drive", type=Path)
    ap.add_argument("--live-bridge", type=Path, default=None, help="the live run's bridge folder (records.jsonl)")
    a = ap.parse_args(argv)
    rows = [json.loads(line) for line in a.drive.read_text().splitlines() if line.strip()]
    out = {"drive": str(a.drive), "cost": cost_summary(rows), "accuracy": accuracy(rows)}
    if a.live_bridge is not None:
        live = [json.loads(line) for line in (a.live_bridge / "records.jsonl").read_text().splitlines() if line.strip()]
        out["validity"] = validity(rows, live)
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
