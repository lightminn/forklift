"""Section 8a of docs/plans/2026-10-28-adoption-zone-check.md: the adoption checks a live run recorded (each with
the simulation's pallet truth), per stage -- the axis error of the effective target, the decision quantity
max(0, edge_m), and how many checks a given margin would have refused.

    python -m tools.adoption_zone_checks <dir> [<dir> ...] --out checks.json [--margins 0.15 0.30]

A run is a ``**/run/result.json`` with ``missions/*/records.json``; a run that did not localize on a survey bundle
(no ``slam_map_bundle`` in its arguments: mapping, or a fallback) is kept apart. A check without truth is undetermined.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

STEP_M = 0.05


def _up(value):
    return None if value is None else math.ceil(round(value / STEP_M, 9)) * STEP_M


def summarize_dirs(dirs, margins=(0.30,)) -> dict:
    stages: dict = {}
    refused = {f"{m:.2f}": 0 for m in margins}
    sessions = missions = undetermined = put_down = 0
    excluded = []
    for run_path in sorted(p for d in dirs for p in Path(d).glob("**/result.json") if p.parent.name == "run"):
        run = json.loads(run_path.read_text())
        if not (run.get("arguments") or {}).get("slam_map_bundle"):
            excluded.append(str(run_path.parent))
            continue
        sessions += 1
        reached_put_down = False
        for rec in sorted((run_path.parent / "missions").glob("*/records.json")):
            checks = (((json.loads(rec.read_text()) or {}).get("state") or {}).get("adoption_zone") or {}).get(
                "checks") or []
            missions += bool(checks)
            for c in checks:
                if c.get("truth") is None:
                    undetermined += 1
                    continue
                err = max(abs(c["site"][0] - c["truth"][0]), abs(c["site"][1] - c["truth"][1]))
                over = max(0.0, float(c["edge_m"]))
                s = stages.setdefault(c["stage"], {"n": 0, "max_abs_m": 0.0, "max_edge_over_m": 0.0})
                s["n"] += 1
                s["max_abs_m"] = max(s["max_abs_m"], err)
                s["max_edge_over_m"] = max(s["max_edge_over_m"], over)
                for m in margins:
                    refused[f"{m:.2f}"] += float(c["edge_m"]) > m
                reached_put_down |= c.get("zone_source") == "put_down"
        put_down += reached_put_down
    worst_err = max((s["max_abs_m"] for s in stages.values()), default=None)
    worst_edge = max((s["max_edge_over_m"] for s in stages.values()), default=None)
    # margin_m: the pre-registered rule (the error, rounded up); edge_margin_m: what the decision quantity itself
    # would need (a re-pick square's centre carries the earlier put-down's error too), reported beside it
    return {"sessions": sessions, "missions": missions, "put_down_sessions": put_down,
            "undetermined_checks": undetermined, "excluded_runs": excluded, "stages": stages,
            "max_abs_m": worst_err, "max_edge_over_m": worst_edge, "margin_m": _up(worst_err),
            "edge_margin_m": _up(worst_edge), "refused_at": refused}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dirs", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--margins", type=float, nargs="+", default=[0.30])
    args = ap.parse_args(argv)
    out = summarize_dirs(args.dirs, tuple(args.margins))
    args.out.write_text(json.dumps(out, indent=1))
    print(json.dumps({k: out[k] for k in ("sessions", "missions", "put_down_sessions", "undetermined_checks",
                                          "max_abs_m", "max_edge_over_m", "margin_m", "edge_margin_m",
                                          "refused_at")}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
