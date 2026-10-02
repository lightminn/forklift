"""CPU proxy for run-time observation viewpoints (2026-10-03 plan).

Plan: docs/plans/2026-10-03-runtime-observation-viewpoints.md. For each seed it
rebuilds the scenario from a runner record, generates the run-time viewpoints
exactly as the runner does (map and pickup zone only), and replays the G4
design tool's sequential policy over the fixed list, then the fixed list plus
the run-time viewpoints. The truth pallet enters only the scoring (front-visible
pixels and the mission plan from each view), never the generation.

Two numbers per seed: whether the first good view ends the search (the G4
metric), and how many good views at least 0.5 m apart remain reachable if every
detection is rejected. Both are nominal plan availability, not Isaac success.

    python -m tools.observation_viewpoint_eval \\
        --record artifacts/<run>/seed_3001/result.json --seeds 200:400 --json out.json
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import sys
from multiprocessing import Pool
from pathlib import Path

from forklift_core.planning import Pose2D, Rectangle
from forklift_core.planning.observation_viewpoints import ViewpointConfig, runtime_viewpoints
from tools import observation_candidate_design as design

# The runner's fixed list (sim/isaac/run_transport.py default --observation-waypoints).
FIXED = (
    (-0.10, 0.90, 0.0),
    (-1.20, 0.30, 0.0),
    (-0.10, -0.60, 0.0),
    (-1.50, -0.60, 0.0),
    (-2.00, -0.30, 0.0),
    (0.00, 2.10, -0.25),
    (0.40, 1.20, 0.0),
    (-0.60, 1.80, -0.25),
)
# The next frozen evaluation (plan, check 4): never generated here.
RESERVED_SEEDS = range(4000, 4030)
REAR_TO_CAMERA_FROM_BASE_M = 0.75  # scene_rig.DEFAULT_CAMERA_XYZ_M[0]

_EVALUATOR: design.Evaluator | None = None


def viewpoints_for(evaluator: design.Evaluator, seed: int) -> list[tuple]:
    config = evaluator.config
    scenario = evaluator.scenario(seed)
    geometry = config.geometry
    occupied = [prop.rectangle for prop in scenario.props] + [
        Rectangle(
            scenario.pickup.x_m,
            scenario.pickup.y_m,
            geometry.pallet_depth_m,
            geometry.pallet_width_m,
            scenario.pickup.yaw_rad,
        )
    ]
    views = runtime_viewpoints(
        occupied,
        geometry.unloaded_footprint,
        scenario.bounds,
        margin_m=config.planner.clearance_m,
        pallet_depth_m=geometry.pallet_depth_m,
        pallet_width_m=geometry.pallet_width_m,
        rear_to_camera_m=config.rear_axle_offset_m + REAR_TO_CAMERA_FROM_BASE_M,
        half_fov_rad=math.atan((config.intrinsics.width / 2) / config.intrinsics.fx),
    )
    return [(v.pose.x_m, v.pose.y_m, v.pose.yaw_rad) for v in views]


def good_views(evaluator: design.Evaluator, seed: int, candidates, proxy: int):
    """Walk the whole list as if every detection were rejected."""
    separation = ViewpointConfig().min_separation_m
    start, good, travelled = None, [], 0.0
    for candidate in candidates:
        success, _, length = evaluator.leg(seed, start, candidate)
        if not success:
            continue
        travelled += length
        if (
            all(math.hypot(candidate[0] - g[0], candidate[1] - g[1]) >= separation for g in good)
            and evaluator.view(seed, candidate) >= proxy
            and evaluator.approach(seed, candidate) == "success"
        ):
            good.append(candidate)
        start = Pose2D(*candidate)
    return good, travelled


def evaluate(evaluator: design.Evaluator, seed: int, proxy: int) -> dict:
    if seed in RESERVED_SEEDS:
        raise ValueError("seeds 4000-4029 are reserved for the next frozen evaluation")
    extra = viewpoints_for(evaluator, seed)
    base = evaluator.sequence(seed, FIXED, proxy)
    new = base if base["served"] else evaluator.sequence(seed, list(FIXED) + extra, proxy)
    fixed_good, _ = good_views(evaluator, seed, FIXED, proxy)
    new_good, walked = good_views(evaluator, seed, list(FIXED) + extra, proxy)
    return {
        "seed": seed,
        "runtime_viewpoints": extra,
        "fixed_served": base["served"],
        "served": new["served"],
        "served_by": new["by"],
        "travelled_m": new["travelled_m"],
        "fixed_good_views": len(fixed_good),
        "good_views": len(new_good),
        "worst_walk_m": walked,
    }


def _init(record_path: str) -> None:
    global _EVALUATOR
    _EVALUATOR = design.Evaluator(
        design.RunConfig.from_record(json.loads(Path(record_path).read_text()))
    )


def _job(job):
    seed, proxy = job
    return evaluate(_EVALUATOR, seed, proxy)


def parse_seeds(text: str) -> list[int]:
    seeds = []
    for part in text.split(","):
        start, _, stop = part.partition(":")
        seeds += list(range(int(start), int(stop))) if stop else [int(start)]
    return seeds


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", required=True, help="a runner result.json with all three prop assets")
    parser.add_argument("--seeds", required=True, help="e.g. 200:400 or 3012,3015")
    parser.add_argument("--proxy", type=int, default=1500)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args(argv)
    seeds = parse_seeds(args.seeds)
    reserved = [s for s in seeds if s in RESERVED_SEEDS]
    if reserved:
        parser.error(f"reserved for the next frozen evaluation: {reserved}")
    _init(args.record)  # fail here, not in every pool worker
    jobs = [(s, args.proxy) for s in seeds]
    if args.workers > 1:
        with Pool(args.workers, _init, (args.record,)) as pool:
            rows = pool.map(_job, jobs)
    else:
        rows = [_job(j) for j in jobs]
    n = len(rows)
    print(f"first good view: fixed {sum(r['fixed_served'] for r in rows)}/{n}, "
          f"with run-time {sum(r['served'] for r in rows)}/{n}")
    for label, key in (("fixed", "fixed_good_views"), ("with run-time", "good_views")):
        counts = collections.Counter(min(r[key], 3) for r in rows)
        print(f"good views >=1/>=2/>=3 ({label}): "
              f"{sum(v for k, v in counts.items() if k >= 1)}/"
              f"{sum(v for k, v in counts.items() if k >= 2)}/{counts[3]}")
    if args.json:
        args.json.write_text(json.dumps(rows, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
