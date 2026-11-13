#!/usr/bin/env python3
"""Are two cost replays of the same input the same SLAM? (online-SLAM plan 5 S1 equivalence (가), docs/design/
2026-11-13-slam-patch-build.md 4: R0 against P0 judged directly, R0-U0 and U0-P0 to tell a build difference from a
patch difference.)

Equivalent when: the processed scans are the same, in the same order; every reply's map<-odom differs by at most 1e-6
m in position and 1e-6 rad in heading; the end graph (tools/slam_graph_dump.cpp) has the same node and edge counts;
the effective parameters are the same; each run's own slam_toolbox check (tools/slam_patch_check.py) passed; and each
run finished with every part at exit 0 (status.json: driver, container -- whose checks include the trace's end record
-- and summary). A difference that is not a finite number fails, never passes. Cost is reported, never judged. A
missing graph dump is an error, not a pass.

    python tools/slam_equivalence.py RUN_A RUN_B > equivalence.json    # each a slam_cost_replay.sbatch OUT
"""

from __future__ import annotations

import csv
import json
import math
import statistics
import sys
from pathlib import Path

POSITION_M = 1e-6
HEADING_RAD = 1e-6


def _rows(run: Path) -> list:
    return [json.loads(x) for x in (Path(run) / "drive.jsonl").read_text().splitlines() if x.strip()]


def _graph(run: Path) -> tuple:
    dump = Path(run) / "bridge" / "dump"
    if not (dump / "nodes.csv").is_file() or not (dump / "edges.csv").is_file():
        raise SystemExit(f"{run}: no graph dump (bridge/dump/nodes.csv, edges.csv) -- an equivalence run needs it")
    with open(dump / "nodes.csv") as f:
        nodes = sum(1 for _ in csv.DictReader(f))
    with open(dump / "edges.csv") as f:
        edges = sum(1 for _ in csv.DictReader(f))
    return nodes, edges


def _status_problems(run: Path) -> list:
    status = run / "status.json"
    if not status.is_file():
        return [f"{run}: no status.json"]
    s = json.loads(status.read_text())
    if s.get("state") != "done":
        return [f"{run}: status {s.get('state')!r}, not done"]
    return [f"{run}: {part} exit {s.get(part)!r}" for part in ("driver_exit", "container_exit", "summary_exit")
            if s.get(part) != 0]


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a)) if math.isfinite(a) else math.nan


def compare(a: Path, b: Path) -> dict:
    ra, rb = _rows(a), _rows(b)
    problems = []
    keys_a = [(r["scan_id"], r["stamp_s"]) for r in ra]
    keys_b = [(r["scan_id"], r["stamp_s"]) for r in rb]
    if keys_a != keys_b:
        first = next((i for i, (x, y) in enumerate(zip(keys_a, keys_b)) if x != y), min(len(keys_a), len(keys_b)))
        problems.append(f"processed scan sequences differ ({len(keys_a)} vs {len(keys_b)} scans, first at {first})")
    pos, yaw = [], []
    for x, y in zip(ra, rb):
        if (x["scan_id"], x["stamp_s"]) != (y["scan_id"], y["stamp_s"]):
            break
        mx, my = x["map_from_odom"], y["map_from_odom"]
        pos.append(math.hypot(mx[0] - my[0], mx[1] - my[1]))
        yaw.append(abs(_wrap(mx[2] - my[2])))
    bad = sum(1 for v in pos + yaw if not math.isfinite(v))
    if bad:
        problems.append(f"{bad} pose differences are not finite numbers")
    pos, yaw = [v for v in pos if math.isfinite(v)], [v for v in yaw if math.isfinite(v)]
    max_pos, max_yaw = max(pos, default=0.0), max(yaw, default=0.0)
    if max_pos > POSITION_M:
        problems.append(f"position differs by up to {max_pos:.3g} m (> {POSITION_M:g})")
    if max_yaw > HEADING_RAD:
        problems.append(f"heading differs by up to {max_yaw:.3g} rad (> {HEADING_RAD:g})")
    ga, gb = _graph(a), _graph(b)
    if ga != gb:
        problems.append(f"end graph nodes/edges {ga} vs {gb}")
    pa = (Path(a) / "bridge" / "params_effective.yaml").read_text()
    pb = (Path(b) / "bridge" / "params_effective.yaml").read_text()
    if pa != pb:
        problems.append("effective parameters differ")
    for run in (a, b):
        problems += _status_problems(Path(run))
        check = Path(run) / "bridge" / "patch_check.json"
        if not check.is_file() or not json.loads(check.read_text()).get("ok"):
            problems.append(f"{run}: its slam_toolbox check did not pass (bridge/patch_check.json)")
    cost = {name: statistics.fmean(float(r["slam_wall_s"]) for r in rows) if rows else None
            for name, rows in (("a", ra), ("b", rb))}
    return {"a": str(a), "b": str(b), "equivalent": not problems, "problems": problems, "scans": len(keys_a),
            "max_position_diff_m": max_pos, "max_heading_diff_rad": max_yaw, "graph": {"a": ga, "b": gb},
            "mean_keyframe_s": cost}


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 2:
        raise SystemExit(__doc__)
    out = compare(Path(argv[0]), Path(argv[1]))
    print(json.dumps(out, indent=1))
    return 0 if out["equivalent"] else 1


if __name__ == "__main__":
    sys.exit(main())
