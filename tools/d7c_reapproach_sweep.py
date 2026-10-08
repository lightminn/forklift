"""Plan D7c CPU docking re-alignment report: the re-approach plan from a missed docking line.

For the factory hall seeds 1, 3 and 5 (the v10 scene of the D7 CPU judgement: props under
1.15 m removed, ground-truth rectangles, the measured chassis, the D7 adopted planner
options), the docking line starts at the delivery straight's start L. The truck stands at
L o (0, lateral, yaw) for lateral {0.12, 0.18, 0.24, 0.30} m x yaw {-0.08, 0, 0.08} rad --
just outside the 0.12 m / 0.08 rad straight acceptance -- and plan_docking_reapproach plans
back to L inside its box (36 cases); an accepted plan is then dry-run with the runner's
transport tracker at the docking stop's tolerance (0.03 m, 0.05 rad), as the runner will
before installing it. Reported only: the pass judgement is Isaac's (plan D7).

    python tools/d7c_reapproach_sweep.py --output <dir>
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from dataclasses import replace
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools import bay_planning_sweep as BAY  # noqa: E402
from tools.d7_combo_sweep import compose  # noqa: E402

SEEDS = (1, 3, 5)
LATERALS = (0.12, 0.18, 0.24, 0.30)
YAWS = (-0.08, 0.0, 0.08)


def stop_tracker_config(settings: dict):
    """The runner's transport tracker (run_transport.py, the trackers dict) with the
    docking stop's tolerances (arm_docking: 0.03 m, 0.05 rad)."""
    from forklift_core.control.path_tracking import TrackerConfig

    return TrackerConfig(
        cruise_speed_mps=settings["transport_speed_mps"], max_curvature_inv_m=settings["tracker_curvature_inv_m"],
        max_acceleration_mps2=settings["drive_acceleration_mps2"], lookahead_m=0.28, position_tolerance_m=0.03,
        yaw_tolerance_rad=0.05, cusp_position_tolerance_m=0.03, cusp_yaw_tolerance_rad=0.05,
        cusp_brake_window_m=0.008, max_lateral_acceleration_mps2=settings.get("max_lateral_acceleration_mps2"),
        max_reverse_speed_mps=settings.get("max_reverse_speed_mps"), overshoot_tolerance_m=0.03,
        stop_speed_mps=0.012, max_cross_track_error_m=0.35,
    )


def run(args) -> dict:
    from forklift_core.control.rollout import bicycle_rollout
    from forklift_core.planning import Pose2D
    from forklift_core.planning.pallet_mission import (
        D7_PLANNER_OPTIONS,
        SearchBudget,
        make_transport_planner_config,
        plan_docking_reapproach,
        site_poses,
    )

    settings = yaml.safe_load(args.settings.read_text(encoding="utf-8"))
    base = {"pallet_geometry": str(args.pallet_geometry), "forklift_urdf": str(args.forklift_urdf),
            "factory_layout": str(args.factory_layout), "reserve_m": args.insertion_reserve_m,
            "alignment_straight_m": 2.1, "delivery_straight_m": 1.5, "withdrawal_m": 0.55, "obstacles": 4,
            "min_top_m": 1.15}
    config = make_transport_planner_config(
        curvature_limit_inv_m=settings["planner_curvature_inv_m"], clearance_m=settings["planning_clearance_m"],
        max_expansions=30000, **D7_PLANNER_OPTIONS,
    )
    tracker = stop_tracker_config(settings)
    rows = []
    for seed in args.seeds:
        scenario, _, geometry, _ = BAY.factory_scene({**base, "seed": seed})
        line = site_poses(scenario.destination, geometry)["predelivery"]
        line_t = (line.x_m, line.y_m, line.yaw_rad)
        for lateral in args.laterals:
            for yaw in args.yaws:
                start = Pose2D(*compose(line_t, 0.0, lateral, yaw))
                started = time.monotonic()
                path, record = plan_docking_reapproach(
                    scenario, start, line, config, geometry=geometry, keep_m=geometry.delivery_straight_m,
                    deadline=SearchBudget(args.search_budget_s),
                )
                record["planning_wall_s"] = round(time.monotonic() - started, 4)
                dry = None
                if path is not None:
                    rolled = time.monotonic()
                    roll = bicycle_rollout(path.poses, path.directions, path.curvatures_inv_m, tracker,
                                           (start.x_m, start.y_m, start.yaw_rad))
                    dry = {"status": roll.status, "position_error_m": roll.position_error_m,
                           "yaw_error_rad": roll.yaw_error_rad, "time_s": roll.time_s,
                           "wall_s": round(time.monotonic() - rolled, 4),
                           "arrived": roll.status == "arrived" and roll.position_error_m <= 0.03
                           and abs(roll.yaw_error_rad) <= 0.05}
                # The helper's own statistics stay, a refused plan's included (Codex D7c P3).
                rows.append({"seed": seed, "lateral_m": lateral, "yaw_rad": yaw, **record, "dry_run": dry})
    accepted = [r for r in rows if r["refused"] is None and r["dry_run"]["arrived"]]
    lengths = sorted(r["length_m"] for r in accepted)
    summary = {
        "cases": len(rows), "accepted": len(accepted),
        "refused": {**{k: sum(1 for r in rows if r["refused"] == k) for k in ("no_path", "leaves_box", "too_long")},
                    "dry_run": sum(1 for r in rows if r["refused"] is None and not r["dry_run"]["arrived"])},
        "dry_run_time_s_max": max((r["dry_run"]["time_s"] for r in accepted), default=None),
        "length_m": None if not lengths else {"median": statistics.median(lengths), "max": lengths[-1]},
        "gear_changes_max": max((r["gear_changes"] for r in accepted), default=None),
        "planner_options": D7_PLANNER_OPTIONS,
    }
    out = {"summary": summary, "rows": rows}
    if args.output is not None:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "reapproach.json").write_text(json.dumps(out, indent=2))
    return out


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--seeds", type=int, nargs="+", default=list(SEEDS))
    parser.add_argument("--laterals", type=float, nargs="+", default=list(LATERALS))
    parser.add_argument("--yaws", type=float, nargs="+", default=list(YAWS))
    parser.add_argument("--settings", type=Path, default=ROOT / "config/isaac_transport_measured.yaml")
    parser.add_argument("--forklift-urdf", type=Path, default=ROOT / "sim/models/dls08_measured/forklift.urdf")
    parser.add_argument("--pallet-geometry", type=Path, default=ROOT / "config/pallet_geometry_epal6.yaml")
    parser.add_argument("--factory-layout", type=Path, default=ROOT / "config/factory_south_hall.yaml")
    parser.add_argument("--insertion-reserve-m", type=float, default=0.016)
    parser.add_argument("--search-budget-s", type=float, default=20.0)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    print(json.dumps(run(parse_args(argv))["summary"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
