"""CPU check of the return-leg turnaround (docs/plans/2026-10-12-return-forward-turnaround.md).

From each factory seed's mission-plan withdrawal end (tools/bay_planning_sweep.plan_factory_mission,
the D7 travel config the recorded runs plan with) and from where each recorded run made its first
return plan -- the standing pose after the settle phase's SLAM release (result.json slam_summary
holds, "rear_after"), just before the live return plan; run/paths.json is overwritten by later
replans (Codex implementation review 2026-10-12) -- plan_return_leg is called without and with
forward_turnaround on the truth rectangles (no grid, no drive permission: the runner checks each arc
on its snapshot, this does not). One JSON row per start: both plans' status, forward / reverse
length, gear changes, weighted cost, the chosen candidate and the wall time.

    python tools/return_turnaround_check.py --seeds 0-24 --runs DIR... --output OUT.json
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools import bay_planning_sweep as BAY  # noqa: E402


def job_for(seed: int) -> dict:
    """The recorded runs' scene arguments (meeting_video.sbatch, d7_isaac.sbatch)."""
    return dict(
        seed=seed, pallet_geometry=str(ROOT / "config/pallet_geometry_epal6.yaml"),
        forklift_urdf=str(ROOT / "sim/models/dls08_measured/forklift.urdf"), reserve_m=0.016,
        alignment_straight_m=2.1, delivery_straight_m=1.5, withdrawal_m=0.55, obstacles=4,
        factory_layout=str(ROOT / "config/factory_south_hall.yaml"), min_top_m=1.15,
    )


def compare(scenario, geometry, config, travel, start) -> dict:
    from forklift_core.planning.pallet_mission import (
        RETURN_TURNAROUND_REVERSE_WEIGHT,
        SearchBudget,
        path_terms,
        plan_return_leg,
    )

    row = {"start": [start.x_m, start.y_m, start.yaw_rad]}
    for name, on in (("direct", False), ("turnaround", True)):
        t0 = time.monotonic()
        leg = plan_return_leg(scenario, start, scenario.start_rear, config, geometry=geometry,
                              travel_config=travel, deadline=SearchBudget(20.0), forward_turnaround=on)
        entry = {"status": leg.status, "wall_s": round(time.monotonic() - t0, 3)}
        if leg.success:
            entry.update(path_terms(leg, travel, RETURN_TURNAROUND_REVERSE_WEIGHT))
        if on and leg.turnaround is not None:
            record = leg.turnaround
            entry.update(chosen=record["chosen"], evaluated=record["evaluated"], timed_out=record["timed_out"],
                         expansions=record["expansions"],
                         candidates={c["name"]: c["status"] for c in record["candidates"]})
        row[name] = entry
    return row


def main(argv=None) -> int:
    from forklift_core.planning import Pose2D
    from forklift_core.planning.pallet_mission import (
        D7_PLANNER_OPTIONS,
        SearchBudget,
        make_transport_planner_config,
    )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=BAY._seeds, default=[])
    parser.add_argument("--runs", type=Path, nargs="*", default=[])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    config = replace(make_transport_planner_config(curvature_limit_inv_m=0.29, clearance_m=0.10,
                                                   max_expansions=30000), **D7_PLANNER_OPTIONS)
    travel = replace(config, obstacle_heuristic_resolution_m=0.25)
    rows = []
    for seed in args.seeds:
        scenario, factory, geometry, _ = BAY.factory_scene(job_for(seed))
        plan, _ = BAY.plan_factory_mission(scenario, factory, geometry, config, travel, SearchBudget(20.0), [], True)
        row = {"source": f"mission_plan seed {seed}", "seed": seed}
        if plan is None or not plan.success or plan.return_home is None:
            row["mission"] = "planning_failure" if plan is None else plan.status
        else:
            row.update(compare(scenario, geometry, config, travel, Pose2D(*plan.return_home.poses[0])))
        rows.append(row)
        print(json.dumps(row)[:400], flush=True)
    for run in args.runs:
        result = json.loads((run / "result.json").read_text())
        paths = json.loads((run / "paths.json").read_text())
        seed = int(result["seed"])
        scenario, _, geometry, _ = BAY.factory_scene(job_for(seed))
        row = {"source": str(run), "seed": seed}
        holds = (result.get("slam_summary") or {}).get("holds") or []
        settle = [h for h in holds if h.get("event") == "release" and h.get("phase") == "settle"]
        if "return_home" not in paths or not paths["return_home"].get("poses"):
            row["mission"] = "no_return"
        elif not settle:
            row["mission"] = "no_settle_release"
        else:
            row["start_source"] = f"settle release at {settle[0]['time_s']:.2f} s"
            row["recorded_return"] = {"length_m": paths["return_home"]["length_m"],
                                      "reverse_samples": sum(1 for d in paths["return_home"]["directions"] if d < 0)}
            row.update(compare(scenario, geometry, config, travel, Pose2D(*settle[0]["rear_after"])))
        rows.append(row)
        print(json.dumps(row)[:400], flush=True)
    args.output.write_text(json.dumps(rows, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
