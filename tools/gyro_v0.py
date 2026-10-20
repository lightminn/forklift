"""Gyro-heading plan V-G0 (docs/plans/2026-10-21-gyro-heading-odometry.md): the pre-registered matrix and its verdict.

    python tools/gyro_v0.py manifest --run r6=RUN_DIR --run r7=RUN_DIR --out-root DIR --manifest M.json
    python tools/gyro_v0.py summary M.json [--json OUT]

``manifest`` writes every planned remap: per run the steering arm (one per seed; it does not depend on the gyro bundle)
and the gyro arm per bundle and seed, seeds 1000-1019, each with its output directory and the ``slam_remap.sbatch``
arguments. ``summary`` reads each planned output: a run counts only when its status shows the driver, container and
evaluation exited 0, the graph was serialized, the driver sent every scan of the record, the V2' check paired every
node and has a yaw value, and the processed replies equal the graph's nodes; any other (or missing) run is V2' = inf
(plan V-G0: a failed gyro run loses its pair and is the largest value in the median). The verdict, per bundle A+ and A-
separately and per run: (1) gyro V2' max < steering V2' max on >= 16 of 20 seeds, (2) the median gyro position max
<= 0.10 m, (3) the median gyro yaw max <= 0.03 rad; G2's runtime-safety design follows only when all four cells pass.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

SEEDS = tuple(range(1000, 1020))
REQUIRED_RUNS = ("r6", "r7")
JUDGED = ("A+", "A-")
RECORDED = ("B1+", "B1-", "B2", "B3")
WINS, POSITION_M, YAW_RAD = 16, 0.10, 0.03


def plan(runs: dict, out_root: Path) -> list[dict]:
    entries = []
    for key, run_dir in runs.items():
        for seed in SEEDS:
            entries.append({"run": key, "run_dir": str(run_dir), "arm": "steering", "bundle": None, "seed": seed,
                            "out": str(out_root / key / f"steering_s{seed}"),
                            "odom_args": f"--arm steering --bundle A+ --seed {seed}", "noise_seed": seed})
            for bundle in JUDGED + RECORDED:
                entries.append({"run": key, "run_dir": str(run_dir), "arm": "gyro", "bundle": bundle, "seed": seed,
                                "out": str(out_root / key / f"gyro_{bundle}_s{seed}"),
                                "odom_args": f"--arm gyro --bundle {bundle} --seed {seed}", "noise_seed": seed})
    return entries


def record_scans(run_dir: str, cache: dict) -> int:
    if run_dir not in cache:
        from forklift_core.live.spool import load_list

        cache[run_dir] = len(load_list(Path(run_dir) / "spool", "slam.records"))
    return cache[run_dir]


def node_stamps(dump: Path) -> list:
    import csv

    with open(dump / "nodes.csv", newline="") as handle:
        return [round(float(row["stamp_s"]), 6) for row in csv.DictReader(handle)]


def outcome(entry: dict, cache: dict) -> dict:
    """The run's V2' maxima, or inf with why it does not count."""
    out = Path(entry["out"])
    try:
        status = json.loads((out / "status.json").read_text())
        v2 = json.loads((out / "v2.json").read_text())
        replies = [json.loads(line) for line in (out / "drive.jsonl").read_text().splitlines() if line]
        config = next(json.loads(line[len("config "):]) for line in (out / "drive.log").read_text().splitlines()
                      if line.startswith("config "))
        odom = json.loads((out / "odom.json").read_text().splitlines()[-1])
        nodes = node_stamps(out / "bridge" / "dump")
    except (OSError, ValueError, StopIteration, KeyError, IndexError) as exc:
        return {"position_max_m": math.inf, "yaw_max_rad": math.inf, "invalid": f"unreadable: {exc!r}"}
    processed = [round(float(r["stamp_s"]), 6) for r in replies]
    checks = {"done": status.get("state") == "done",
              "driver": status.get("driver_exit") == 0, "container": status.get("container_exit") == 0,
              "eval": status.get("eval_exit") == 0, "serialized": status.get("serialize_ok") is True,
              "all_scans": status.get("sent") == record_scans(entry["run_dir"], cache),
              "paired": bool(v2.get("nodes")) and v2.get("paired") == v2.get("nodes"),
              "yaw": v2.get("yaw_max_rad") is not None,
              # processed replies and the saved graph's nodes, one to one by stamp
              "nodes": (len(set(processed)) == len(processed) == len(nodes) == len(set(nodes)) == v2.get("nodes")
                        and set(processed) == set(nodes)),
              # what actually ran is what the matrix planned
              "config": (config.get("noise_seed") == entry["noise_seed"]
                         and config.get("run") == str(Path(entry["run_dir"]).resolve())
                         and config.get("odom_file") == str(out / "odom.npz")),
              "odometry": (odom.get("arm") == entry["arm"] and odom.get("seed") == entry["seed"]
                           and (entry["arm"] == "steering" or odom.get("bundle") == entry["bundle"]))}
    bad = [k for k, ok in checks.items() if not ok]
    if bad:
        return {"position_max_m": math.inf, "yaw_max_rad": math.inf, "invalid": bad}
    return {"position_max_m": float(v2["position_max_m"]), "yaw_max_rad": float(v2["yaw_max_rad"]), "invalid": None}


def matrix_problems(entries: list[dict]) -> list[str]:
    """What the entries lack of the pre-registered matrix (both records, distinct, every arm x bundle x seed once)."""
    problems = []
    dirs = {}
    for e in entries:
        dirs.setdefault(e["run"], set()).add(str(Path(e["run_dir"]).resolve()))
    if sorted(dirs) != sorted(REQUIRED_RUNS):
        problems.append(f"runs {sorted(dirs)} are not {list(REQUIRED_RUNS)}")
    if any(len(v) != 1 for v in dirs.values()) or len({next(iter(v)) for v in dirs.values()}) != len(dirs):
        problems.append("each run needs one record of its own")
    keys = [(e["run"], e["arm"], e["bundle"], e["seed"]) for e in entries]
    expected = {(r, "steering", None, s) for r in REQUIRED_RUNS for s in SEEDS} | {
        (r, "gyro", b, s) for r in REQUIRED_RUNS for b in JUDGED + RECORDED for s in SEEDS}
    if len(keys) != len(set(keys)):
        problems.append("duplicate entries")
    if set(keys) != expected:
        problems.append(f"{len(expected - set(keys))} planned remaps missing, {len(set(keys) - expected)} unplanned")
    if len({e["out"] for e in entries}) != len(entries):
        problems.append("two entries share an output directory")
    return problems


def verdict(entries: list[dict]) -> dict:
    problems = matrix_problems(entries)
    if problems:
        return {"cells": {}, "pass": False, "matrix_problems": problems}
    cache: dict = {}
    results = {(e["run"], e["arm"], e["bundle"], e["seed"]): outcome(e, cache) for e in entries}
    runs = sorted({e["run"] for e in entries})
    cells = {}
    for run in runs:
        for bundle in JUDGED + RECORDED:
            gyro = [results[(run, "gyro", bundle, s)] for s in SEEDS]
            steer = [results[(run, "steering", None, s)] for s in SEEDS]
            wins = sum(1 for g, t in zip(gyro, steer) if g["position_max_m"] < t["position_max_m"])
            cell = {"wins": wins, "median_position_max_m": statistics.median(g["position_max_m"] for g in gyro),
                    "median_yaw_max_rad": statistics.median(g["yaw_max_rad"] for g in gyro),
                    "steering_median_position_max_m": statistics.median(t["position_max_m"] for t in steer),
                    "invalid": {"gyro": sum(1 for g in gyro if g["invalid"]), "steering": sum(1 for t in steer if t["invalid"])},
                    "judged": bundle in JUDGED}
            cell["pass"] = (wins >= WINS and cell["median_position_max_m"] <= POSITION_M
                            and cell["median_yaw_max_rad"] <= YAW_RAD)
            cells[f"{run}/{bundle}"] = cell
    judged = [c for c in cells.values() if c["judged"]]
    return {"cells": cells, "pass": bool(judged) and all(c["pass"] for c in judged),
            "invalid_runs": {f"{k[0]}/{k[1]}/{k[2]}/{k[3]}": v["invalid"] for k, v in results.items() if v["invalid"]}}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="command", required=True)
    m = sub.add_parser("manifest")
    m.add_argument("--run", action="append", required=True, help="KEY=RUN_DIR")
    m.add_argument("--out-root", type=Path, required=True)
    m.add_argument("--manifest", type=Path, required=True)
    s = sub.add_parser("summary")
    s.add_argument("manifest", type=Path)
    s.add_argument("--json", type=Path, default=None)
    a = ap.parse_args(argv)
    if a.command == "manifest":
        runs = dict(item.split("=", 1) for item in a.run)
        entries = plan(runs, a.out_root)
        a.manifest.write_text(json.dumps({"plan": "docs/plans/2026-10-21-gyro-heading-odometry.md V-G0",
                                          "entries": entries}, indent=1))
        print(f"{len(entries)} planned remaps")
        return 0
    result = verdict(json.loads(a.manifest.read_text())["entries"])
    text = json.dumps(result, indent=1, default=str)
    if a.json:
        a.json.write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
