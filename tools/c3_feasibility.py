"""C3 feasibility on the CPU depth rig (docs/plans/2026-10-27-insertion-lateral-drift.md, T1).

Can a second near observation be taken on the alignment straight, and can its re-draw be adopted? For each fork-tip
to entry-face gap g and twelve truck poses perturbed around the insertion line (lateral and yaw, the same samples for
every g), the EPAL 6 rig scene is rendered from the carriage_low_measured mount (Isaac intrinsics, 1 mm depth
rounding, no noise) and:

- detection: the pocket detector at the near capture's settings (``DetectorParams.derived_for``, range_min 0.8 m) and
  at the near-field tracker's (range_min 0.1 m); success = valid, the pocket midpoint's lateral error (truck frame) is
  recorded;
- re-draw: from the true relative pose, ``straight_from_pose`` with the runner's near-capture box (lateral 0.05 m,
  heading ``--near-capture-box-yaw-rad`` 0.05, length >= 0.3 m) and the dry run the runner applies
  (``bicycle_rollout`` with the approach tracker's config, arrived within 6 mm and 0.015 rad).

The plan's rule (fixed before results): the smallest g in {0.5, 0.6, 0.7} where detection is valid in >= 10 of 12 with
lateral p95 <= 10 mm AND the re-draw passes in >= 10 of 12.

    python -m tools.c3_feasibility --out c3.json
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
from pathlib import Path

import numpy as np

from forklift_core.control.path_tracking import TrackerConfig
from forklift_core.control.rollout import bicycle_rollout
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from forklift_core.planning.pallet_mission import Pose2D, straight_from_pose
from tools import scene_rig
from tools.nearfield_mount_study import (
    GEOMETRY,
    HALF_DEPTH,
    PALLET,
    PRIOR,
    isaac_intrinsics,
    truck_frame,
)

GAPS_M = (0.4, 0.5, 0.6, 0.7, 0.8, 1.0)
JUDGED_GAPS_M = (0.5, 0.6, 0.7)
SEEDS = 12
LATERAL_M, YAW_RAD = 0.050, math.radians(1.5)
TILT_RAD = 0.10  # carriage_low_measured
CAMERA_Z_M = 0.27
CAMERA_BACK_TO_OPTICAL_M = 0.015
REAR_AXLE_TO_TIP_M = 1.29  # geometry.axle_to_fork_tip_m of the measured chassis
APPROACH_GAP_M = 0.10  # the insertion starts with the tip this far before the face
ALIGNMENT_STRAIGHT_M = 2.1
BOX = dict(max_lateral_m=0.05, max_yaw_rad=0.05, min_length_m=0.3)
DRY_POSITION_M, DRY_YAW_RAD = 0.006, 0.015
# The approach tracker of a riib run (result.json tracker_configs.approach, rsbr_b35d7f53 aisle_rack s4).
APPROACH_CONFIG = TrackerConfig(
    cruise_speed_mps=0.6, max_curvature_inv_m=0.36, max_acceleration_mps2=0.3, lookahead_m=0.28,
    position_tolerance_m=0.008, yaw_tolerance_rad=0.02, stop_speed_mps=0.012, max_cross_track_error_m=0.35,
    cusp_position_tolerance_m=0.03, cusp_yaw_tolerance_rad=0.05, cusp_brake_window_m=0.008,
    overshoot_tolerance_m=0.03)
PARAMS = {
    "near_capture": DetectorParams.derived_for(PRIOR),
    "near_field": DetectorParams.derived_for(PRIOR, range_min_m=0.1),
}


def perturbations(seeds: int = SEEDS):
    """Truck pose relative to the insertion line: (lateral m, yaw rad), the same for every g."""
    rng = np.random.default_rng(20261027)
    return [(float(rng.uniform(-LATERAL_M, LATERAL_M)), float(rng.uniform(-YAW_RAD, YAW_RAD))) for _ in range(seeds)]


def pallet_pose_in_truck(gap_m: float, tip_x: float, lateral_m: float, yaw_rad: float):
    """Pallet centre (x, y, yaw) in the truck frame when the truck's tip stands ``gap_m`` before the face along the
    insertion line, offset ``lateral_m`` to its left and turned ``yaw_rad`` from it."""
    # Insertion-line frame: face at x = 0, pallet centre at (HALF_DEPTH, 0), tip at (-gap, lateral), heading yaw.
    cx, cy = HALF_DEPTH + gap_m, -lateral_m  # pallet centre relative to the tip, line frame
    c, s = math.cos(-yaw_rad), math.sin(-yaw_rad)
    x = tip_x + c * cx - s * cy
    y = s * cx + c * cy
    return x, y, -yaw_rad


def _finite_or_none(value):
    """A timed-out rollout reports NaN errors; JSON has no NaN, so they are written as null."""
    value = float(value)
    return value if math.isfinite(value) else None


def redraw(gap_m: float, lateral_m: float, yaw_rad: float) -> dict:
    """The runner's re-draw acceptance from the true relative pose (rear axle frame of the insertion line)."""
    rear_x = -(gap_m + REAR_AXLE_TO_TIP_M * math.cos(yaw_rad))
    rear_y = lateral_m - REAR_AXLE_TO_TIP_M * math.sin(yaw_rad)
    current = Pose2D(rear_x, rear_y, yaw_rad)
    line_end = Pose2D(-(APPROACH_GAP_M + REAR_AXLE_TO_TIP_M), 0.0, 0.0)
    line_start = Pose2D(line_end.x_m - ALIGNMENT_STRAIGHT_M, 0.0, 0.0)
    straight, offsets = straight_from_pose(current, line_start, line_end, **BOX)
    out = {"offsets": offsets, "box": straight is not None}
    if straight is None:
        out["ok"] = False
        return out
    dry = bicycle_rollout(straight.poses, straight.directions, straight.curvatures_inv_m, APPROACH_CONFIG,
                          np.array([current.x_m, current.y_m, current.yaw_rad]))
    out["dry"] = {"status": dry.status, "position_error_m": _finite_or_none(dry.position_error_m),
                  "yaw_error_rad": _finite_or_none(dry.yaw_error_rad)}
    if dry.trajectory:
        # Where the fork tip ends across the line (the line is y = 0): rear lateral + lever x sin(yaw).
        xe, ye, he = dry.trajectory[-1]
        out["dry"]["end_tip_lateral_m"] = float(ye + REAR_AXLE_TO_TIP_M * math.sin(he))
    out["ok"] = (dry.status == "arrived" and dry.position_error_m <= DRY_POSITION_M
                 and abs(dry.yaw_error_rad) <= DRY_YAW_RAD)
    # Diagnostic only (not the plan's rule): the tracker's own tolerance as the acceptance.
    out["ok_at_tracker_tolerance"] = (dry.status == "arrived" and dry.position_error_m <= 0.008
                                      and abs(dry.yaw_error_rad) <= 0.02)
    return out


def evaluate(gap_m: float, samples) -> dict:
    truck, tip_x, carriage_front = truck_frame("measured")
    camera = scene_rig.Camera((round(carriage_front + CAMERA_BACK_TO_OPTICAL_M, 4), 0.0, CAMERA_Z_M), TILT_RAD)
    k = isaac_intrinsics()
    rows = []
    for seed, (lateral, yaw) in enumerate(samples):
        x, y, pyaw = pallet_pose_in_truck(gap_m, tip_x, lateral, yaw)
        placed = scene_rig.place(PALLET, x_m=x, y_m=y, yaw_rad=pyaw)
        truth = scene_rig.true_pockets(GEOMETRY, x_m=x, y_m=y, yaw_rad=pyaw)
        mid_true = (truth[0] + truth[1]) / 2
        scene = scene_rig.render([*truck, *placed], camera=camera, quantize=True, noise_k=0.0,
                                 min_range_m=0.1, intrinsics=k)
        row = {"seed": seed, "lateral_m": lateral, "yaw_rad": yaw}
        for name, params in PARAMS.items():
            obs = detect_pockets(scene, PRIOR, dataclasses.replace(params, seed=seed)).observation
            entry = {"status": obs.status, "reason": obs.reason}
            if obs.status == "valid":
                # The rule's pocket lateral error: left to left and right to right, the worse of the two.
                entry["lateral_error_m"] = float(max(abs(obs.left.center_m[1] - truth[0][1]),
                                                     abs(obs.right.center_m[1] - truth[1][1])))
                mid = (np.array(obs.left.center_m) + np.array(obs.right.center_m)) / 2
                entry["midpoint_lateral_error_m"] = float(abs(mid[1] - mid_true[1]))  # diagnostic
            row[name] = entry
        row["redraw"] = redraw(gap_m, lateral, yaw)
        rows.append(row)
    summary = {"gap_m": gap_m, "camera_face_m": round(gap_m + tip_x - camera.xyz_m[0], 4)}
    for name in PARAMS:
        errs = [r[name]["lateral_error_m"] for r in rows if r[name]["status"] == "valid"]
        summary[name] = {"valid": len(errs),
                         "lateral_p95_mm": None if not errs else 1000 * float(np.percentile(errs, 95, method="higher"))}
    summary["redraw_ok"] = sum(r["redraw"]["ok"] for r in rows)
    summary["redraw_ok_at_tracker_tolerance"] = sum(r["redraw"].get("ok_at_tracker_tolerance", False) for r in rows)
    summary["dry_status"] = {k: sum(1 for r in rows if r["redraw"].get("dry", {}).get("status") == k)
                             for k in ("arrived", "failed", "timeout")}
    tips = [abs(r["redraw"]["dry"]["end_tip_lateral_m"]) for r in rows
            if r["redraw"].get("dry", {}).get("status") == "arrived" and "end_tip_lateral_m" in r["redraw"]["dry"]]
    summary["arrived_end_tip_lateral_max_mm"] = None if not tips else 1000 * max(tips)
    summary["box_ok"] = sum(r["redraw"]["box"] for r in rows)
    return {"summary": summary, "rows": rows}


def judge(results: list, detector: str) -> float | None:
    for r in results:
        s = r["summary"]
        if s["gap_m"] not in JUDGED_GAPS_M:
            continue
        d = s[detector]
        if d["valid"] >= 10 and d["lateral_p95_mm"] is not None and d["lateral_p95_mm"] <= 10.0 and s["redraw_ok"] >= 10:
            return s["gap_m"]
    return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    samples = perturbations()
    results = [evaluate(g, samples) for g in GAPS_M]
    verdict = {name: judge(results, name) for name in PARAMS}
    json.dump({"samples": samples, "results": results, "verdict_gap_m": verdict}, args.out.open("w"), indent=1,
              allow_nan=False)
    for r in results:
        print(json.dumps(r["summary"]))
    print("verdict", json.dumps(verdict))
    return 0


if __name__ == "__main__":
    sys.exit(main())
