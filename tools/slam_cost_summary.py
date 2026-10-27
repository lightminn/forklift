#!/usr/bin/env python3
"""Summarise a SLAM cost replay (tools/slam_remap_drive.py output) for the slam_link_failed plan
(docs/plans/2026-10-25-slam-link-failed.md 3-4).

Cost: keyframe slam_wall_s per spliced unit (mean and maximum), the maximum, the mean of the last 10 % of keyframes,
and least-squares lines against sim time extrapolated to sim 8 h (the plan's 350 passes at the measured 83 s pass
period): one through every keyframe, and, with three complete units or more, one through the per-unit means and one
through the per-unit maxima at the units' mid times -- a change that only flattens the finite window, or leaves one
slow keyframe per pass growing, still shows its slope there. A spliced replay stops at its sim limit inside its last
unit, so with more than one unit the last is partial and left out of the lines (the maximum and the tail keep it).
Accuracy: the SLAM pose (the reply's map<-odom composed with the odometry sent) against the record's truth, no
alignment, position and heading; the growth compares the first and the last complete unit over the record scans both
hold. Validity (an R0 replay against the live run): within the scans the runner recorded, the replay's keyframes are
the live bridge's processed ones (same scan ids, stamps to 1 us -- a missing one fails, a late bridge record after the
runner's last is left out), sent with the runner's own odometry, and the replay's mean cost per tenth of the run
within 0.7-1.4 of the live one.

For a mixed splice of several runs (slam-bounded-graph plan 4.3), the growth is also taken per source run -- the
median error at the run's last complete appearance less its first (``source_growth_m``, the largest over runs) -- and
against a baseline replay of the same input as the excess error per unit on the same (unit, run, scan) keys. A graph
dump (``nodes.csv``, ``edges.csv`` of tools/slam_graph_dump.cpp) gives the components and the nodes reachable from
the first.

    python tools/slam_cost_summary.py DRIVE_JSONL [--live-epoch EPOCH_DIR] [--baseline DRIVE_JSONL]
        [--graph-dump DIR] > summary.json
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
    units = complete_units(rows)
    tail = w[-max(1, len(w) // 10):]
    by = {u: [r for r in rows if int(r.get("unit", 0)) == u] for u in units}
    mid = [statistics.fmean(float(r["stamp_s"]) for r in by[u]) for u in units]
    means = [statistics.fmean(float(r["slam_wall_s"]) for r in by[u]) for u in units]
    maxes = [max(float(r["slam_wall_s"]) for r in by[u]) for u in units]
    out = {"keyframes": len(rows), "max_s": max(w), "last_tenth_mean_s": statistics.fmean(tail),
           "unit_mean_s": means, "unit_max_s": maxes,
           "units_complete": len(units), "slope_s_per_s": b, "intercept_s": a, "at_8h_s": a + b * EIGHT_HOURS_S,
           "mean_at_8h_s": None, "max_at_8h_s": None}
    if len(units) >= 3:
        am, bm = _fit(mid, means)
        ax, bx = _fit(mid, maxes)
        out.update(mean_at_8h_s=am + bm * EIGHT_HOURS_S, max_at_8h_s=ax + bx * EIGHT_HOURS_S)
    return out


def complete_units(rows: list) -> list:
    """The units a spliced replay finished: all but the last when there is more than one."""
    units = sorted({int(r.get("unit", 0)) for r in rows})
    return units[:-1] if len(units) > 1 else units


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
        by_unit.setdefault(int(r.get("unit", 0)), {})[int(r.get("src", -1))] = e
    units = complete_units(rows)
    unit_med = [statistics.median(by_unit[u].values()) for u in units]
    common = set(by_unit[units[0]]) & set(by_unit[units[-1]])
    growth = (statistics.median(by_unit[units[-1]][k] for k in common)
              - statistics.median(by_unit[units[0]][k] for k in common)) if common else None
    by_run = {}
    for r in rows:
        if int(r.get("unit", 0)) in units:
            by_run.setdefault(int(r.get("run", 0)), {}).setdefault(int(r.get("unit", 0)), []).append(r)
    source_growth = []
    for run, appear in by_run.items():
        if len(appear) >= 2:
            first, last = appear[min(appear)], appear[max(appear)]
            source_growth.append(statistics.median(_pos_err(r) for r in last)
                                 - statistics.median(_pos_err(r) for r in first))
    return {"pos_median_m": statistics.median(pos), "pos_max_m": max(pos), "yaw_median_deg": statistics.median(yaw),
            "yaw_max_deg": max(yaw), "unit_pos_median_m": unit_med, "growth_m": growth,
            "source_growth_m": max(source_growth) if source_growth else None}


def _pos_err(r) -> float:
    x, y, _ = slam_pose(r)
    tx, ty, _ = (float(v) for v in r["truth_base"])
    return math.hypot(x - tx, y - ty)


def excess_over_baseline(rows: list, base: list) -> dict:
    """The candidate's position error less the baseline's on the same (unit, run, scan) keys, its median per complete
    unit of the candidate and the growth from the first to the last."""
    ref = {(int(r.get("unit", 0)), int(r.get("run", 0)), int(r["src"])): _pos_err(r) for r in base}
    per_unit = {}
    for r in rows:
        key = (int(r.get("unit", 0)), int(r.get("run", 0)), int(r["src"]))
        if key[0] in complete_units(rows) and key in ref:
            per_unit.setdefault(key[0], []).append(_pos_err(r) - ref[key])
    med = [statistics.median(per_unit[u]) for u in sorted(per_unit)]
    return {"unit_excess_median_m": med, "excess_growth_m": med[-1] - med[0] if med else None,
            "matched": sum(len(v) for v in per_unit.values())}


def graph_connectivity(dump: Path) -> dict:
    """Components of the dumped pose graph and the nodes reachable from the lowest id."""
    import csv

    with open(Path(dump) / "nodes.csv") as f:
        nodes = [int(r["unique_id"]) for r in csv.DictReader(f)]
    with open(Path(dump) / "edges.csv") as f:
        edges = [(int(r["source"]), int(r["target"])) for r in csv.DictReader(f)]
    parent = {n: n for n in nodes}

    def find(n):
        while parent[n] != n:
            parent[n] = parent[parent[n]]
            n = parent[n]
        return n

    for a, b in edges:
        if a in parent and b in parent:
            parent[find(a)] = find(b)
    roots = {find(n) for n in nodes}
    first = find(min(nodes)) if nodes else None
    reach = sum(1 for n in nodes if find(n) == first)
    return {"nodes": len(nodes), "edges": len(edges), "components": len(roots), "reachable_from_first": reach,
            "connected": len(roots) == 1}


def validity(rows: list, live: list, runner: list) -> dict:
    n = len(runner)
    live_kf = [r for r in live if r.get("status") == "processed" and int(r["scan_id"]) < n]
    same = ([int(r["src"]) for r in rows] == [int(r["scan_id"]) for r in live_kf]
            and [round(float(r["stamp_s"]), 6) for r in rows] == [round(float(r["stamp_s"]), 6) for r in live_kf])
    same_odom = all([float(v) for v in r["odom_base"]] == [float(v) for v in runner[int(r["src"])]["odom_base"]]
                    for r in rows if int(r["src"]) < n)
    ratios = []
    if same and live_kf:
        n = len(live_kf)
        for k in range(10):
            lo, hi = k * n // 10, max(k * n // 10 + 1, (k + 1) * n // 10)
            ratios.append(statistics.fmean(float(r["slam_wall_s"]) for r in rows[lo:hi])
                          / statistics.fmean(float(r["wall_s"]) for r in live_kf[lo:hi]))
    late = ratios[3:]  # the first tenths are 30-40 ms, where the ratio is noise
    return {"same_keyframes": same, "same_odometry": same_odom, "tenth_ratio": ratios,
            "ok": bool(same and same_odom and late and all(RATIO_LO <= x <= RATIO_HI for x in late))}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("drive", type=Path)
    ap.add_argument("--baseline", type=Path, default=None, help="a baseline replay of the same input (drive.jsonl)")
    ap.add_argument("--graph-dump", type=Path, default=None, help="tools/slam_graph_dump.cpp output folder")
    ap.add_argument("--live-epoch", type=Path, default=None,
                    help="the live run's epoch folder (bridge/records.jsonl, run/spool slam.records)")
    a = ap.parse_args(argv)
    rows = [json.loads(line) for line in a.drive.read_text().splitlines() if line.strip()]
    out = {"drive": str(a.drive), "cost": cost_summary(rows), "accuracy": accuracy(rows)}
    if a.baseline is not None:
        base = [json.loads(line) for line in a.baseline.read_text().splitlines() if line.strip()]
        out["excess"] = excess_over_baseline(rows, base)
    if a.graph_dump is not None and (a.graph_dump / "edges.csv").exists():
        out["graph"] = graph_connectivity(a.graph_dump)
    if a.live_epoch is not None:
        import sys

        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
        from forklift_core.live.spool import load_list

        live = [json.loads(line) for line in (a.live_epoch / "bridge/records.jsonl").read_text().splitlines()
                if line.strip()]
        out["validity"] = validity(rows, live, load_list(a.live_epoch / "run/spool", "slam.records"))
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
