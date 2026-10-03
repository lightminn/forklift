"""Near-field camera mount study on the CPU depth rig (4순위 A).

Plan: docs/plans/2026-10-03-near-field-mount-study.md (v2). For each mount and
chassis, aligned, on a 1 cm face-gap grid down to the insertion target:

- before entry (face_gap > 0): the front pocket detector, success = valid, M2
  error <= tau and yaw error <= yaw_tau;
- after entry: the roof tracker, success = valid, error <= tau, yaw <= yaw_tau
  and reported sigma <= tau, its prior chained from this seed's previous roof
  estimate (shifted by the known 1 cm advance; height from the pallet model);
- the handoff window: cells where both are successful and agree within
  RoofHandoff's tolerances (its 1.4 m start gate is a 0.27 m-mount constant and
  is reported, not applied);
- per seed, the observation sequence actually used (front until the first
  handoff cell, roof after) and its longest gap and terminal gap.

A far grid compares front detection at the runner's detector settings against
the current mount, cell by cell, as a regression warning only.

    python -m tools.nearfield_mount_study --out out.json --workers 8
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from forklift_core.perception.pallet_geometry import (
    load_pallet_geometry,
    target_insertion_depth_m,
)
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from forklift_core.perception.roof_tracking import track_roof
from tools import scene_rig
from tools.measure_pocket_evidence import blades, pocket_error_m, structure

ROOT = Path(__file__).resolve().parents[1]
CHASSIS = {
    "provisional": ROOT / "sim/models/dls08_provisional/forklift.urdf",
    "measured": ROOT / "sim/models/dls08_measured/forklift.urdf",
}
HEIGHTS = (0.20, 0.24, 0.27)
TILTS = (0.0, 0.05, 0.10, 0.15, 0.20)
CURRENT = (0.75, 0.0, 0.50, 0.0)  # the mission runner's mount (base x, y, z, tilt)
# Isaac's integer-index K (perception_adapter.IsaacIntrinsics.integer_index).
FX, CX, CY = 465.741156, 319.5, 239.5
CAMERA_BACK_TO_OPTICAL_M = 0.015  # 25 mm body, front face 10 mm ahead of the optics
SEEDS = 12
TAU_M, YAW_TAU = 0.020, 0.010
MIN_RANGE_M = 0.175
HANDOFF = dict(position=0.025, yaw=0.03, size=0.02)
PRIOR_XY_M, PRIOR_YAW = 0.020, 0.010
FAR_X = tuple(round(2.0 + 0.25 * k, 3) for k in range(11))
FAR_Y = (-0.4, 0.0, 0.4)
FAR_YAW = (-math.radians(12), 0.0, math.radians(12))

GEOMETRY = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
PRIOR = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
NEAR_PARAMS = DetectorParams.derived_for(PRIOR, range_min_m=0.1)
FAR_PARAMS = DetectorParams.derived_for(PRIOR)  # the runner's own settings
HALF_DEPTH = GEOMETRY.overall_depth_m / 2
PALLET = structure(GEOMETRY, "pallet")


def isaac_intrinsics():
    return dataclasses.replace(scene_rig.intrinsics(), fx=FX, fy=FX, cx=CX, cy=CY)


def truck_frame(chassis: str):
    truck = scene_rig.truck_boxes(CHASSIS[chassis], lift_m=0.0)
    side = blades(truck)
    tip_x = truck[side["left"]].centre_m[0] + truck[side["left"]].size_m[0] / 2
    carriage_front = max(
        b.centre_m[0] + b.size_m[0] / 2 for i, b in enumerate(truck) if i not in side.values()
    )
    return truck, tip_x, carriage_front


def yaw_error(observation, truth_yaw: float) -> float:
    delta = observation.insertion_yaw_rad - truth_yaw
    return abs(math.atan2(math.sin(delta), math.cos(delta)))


def front_ok(observation, truth) -> bool:
    return (
        observation.status == "valid"
        and pocket_error_m(observation, truth) <= TAU_M
        and yaw_error(observation, 0.0) <= YAW_TAU
    )


def roof_ok(observation, truth) -> bool:
    return (
        observation.status == "valid"
        and pocket_error_m(observation, truth) <= TAU_M
        and yaw_error(observation, 0.0) <= YAW_TAU
        and observation.position_sigma_m is not None
        and observation.position_sigma_m <= TAU_M
    )


def agree(front, roof) -> bool:
    pairs = ((front.left, roof.left), (front.right, roof.right))
    position = max(math.dist(a.center_m, b.center_m) for a, b in pairs)
    size = max(max(abs(a.width_m - b.width_m), abs(a.height_m - b.height_m)) for a, b in pairs)
    return (
        position <= HANDOFF["position"]
        and abs(front.insertion_yaw_rad - roof.insertion_yaw_rad) <= HANDOFF["yaw"]
        and size <= HANDOFF["size"]
    )


def prior_offsets(seed: int):
    rng = np.random.default_rng(1000 + seed)  # the same samples for every mount
    dx, dy = rng.uniform(-PRIOR_XY_M, PRIOR_XY_M, 2)
    return float(dx), float(dy), float(rng.uniform(-PRIOR_YAW, PRIOR_YAW))


def gaps(flags, step=0.01):
    """Longest run of False and the run ending at the last cell (the target)."""
    longest = run = 0
    for flag in flags:
        run = 0 if flag else run + 1
        longest = max(longest, run)
    return round(longest * step, 3), round(run * step, 3)


def near(chassis: str, z: float, tilt: float) -> dict:
    truck, tip_x, carriage_front = truck_frame(chassis)
    camera_x = round(carriage_front + CAMERA_BACK_TO_OPTICAL_M, 4)
    camera = scene_rig.Camera((camera_x, 0.0, z), tilt)
    k = isaac_intrinsics()
    target = target_insertion_depth_m(GEOMETRY.overall_depth_m, tip_x - carriage_front)
    # Pallet centre x for face_gap +1.20 m down to -target, 1 cm (descending).
    top = tip_x + HALF_DEPTH + 1.20
    count = int(round((1.20 + target) / 0.01)) + 1
    xs = [round(top - 0.01 * i, 4) for i in range(count)]
    offsets = [prior_offsets(s) for s in range(SEEDS)]
    roof_prior = [None] * SEEDS
    cells = []
    for x in xs:
        face_gap = round(x - HALF_DEPTH - tip_x, 4)
        placed = scene_rig.place(PALLET, x_m=x)
        if any(scene_rig.boxes_interpenetrate(t, p) for t in truck for p in placed):
            cells.append({"face_gap": face_gap, "penetrating": True})
            continue
        truth = scene_rig.true_pockets(GEOMETRY, x_m=x)
        truth_mid = (truth[0] + truth[1]) / 2
        scene = scene_rig.render(
            [*truck, *placed], camera=camera, quantize=True, noise_k=0.0,
            min_range_m=MIN_RANGE_M, intrinsics=k,
        )
        front, roof, both = [], [], []
        for s in range(SEEDS):
            f = detect_pockets(scene, PRIOR, dataclasses.replace(NEAR_PARAMS, seed=s)).observation
            if roof_prior[s] is None:
                dx, dy, dyaw = offsets[s]
                expected, eyaw = truth_mid + [dx, dy, 0.0], dyaw
            else:
                (px, py), eyaw = roof_prior[s]
                expected, eyaw = np.array([px, py, truth_mid[2]]), eyaw
            r = track_roof(scene, GEOMETRY, expected, eyaw).observation
            fo, ro = front_ok(f, truth), roof_ok(r, truth)
            if r.status == "valid":
                mid = (np.array(r.left.center_m) + r.right.center_m) / 2
                # The next cell is 1 cm closer: shift by the known advance.
                roof_prior[s] = ((mid[0] - 0.01, mid[1]), r.insertion_yaw_rad)
            elif roof_prior[s] is not None:
                (px, py), pyaw = roof_prior[s]
                roof_prior[s] = ((px - 0.01, py), pyaw)
            front.append(fo)
            roof.append(ro)
            both.append(fo and ro and f.status == r.status == "valid" and agree(f, r))
        cells.append({"face_gap": face_gap, "front": front, "roof": roof, "handoff": both,
                      "front_mid_x": round(float(truth_mid[0]), 4)})
    usable = [c for c in cells if not c.get("penetrating")]
    per_seed = []
    for s in range(SEEDS):
        handed = False
        used = []
        first_handoff = None
        for c in usable:
            if not handed and c["face_gap"] > 0 and c["handoff"][s]:
                handed, first_handoff = True, c["face_gap"]
            if handed:
                used.append(c["roof"][s])
            else:
                used.append(c["front"][s] if c["face_gap"] > 0 else False)
        longest, terminal = gaps(used)
        per_seed.append({"handoff_at": first_handoff, "max_gap_m": longest, "terminal_gap_m": terminal})
    post = [c for c in usable if c["face_gap"] <= 0]
    return {
        "chassis": chassis, "camera": [camera_x, 0.0, z, tilt], "tip_x": tip_x,
        "carriage_front": carriage_front, "target_m": target,
        "handoff_cells": sum(any(c["handoff"]) for c in usable),
        "worst_max_gap_m": max(p["max_gap_m"] for p in per_seed),
        "worst_terminal_gap_m": max(p["terminal_gap_m"] for p in per_seed),
        "seeds_without_handoff": sum(p["handoff_at"] is None for p in per_seed),
        "post_entry_min_roof_seeds": min((sum(c["roof"]) for c in post), default=0),
        "per_seed": per_seed, "cells": cells,
    }


def far(chassis: str, camera_xyz_tilt) -> dict:
    x0, y0, z0, tilt = camera_xyz_tilt
    camera = scene_rig.Camera((x0, y0, z0), tilt)
    k = isaac_intrinsics()
    out = {}
    for x in FAR_X:
        for y in FAR_Y:
            for yaw in FAR_YAW:
                placed = scene_rig.place(PALLET, x_m=x, y_m=y, yaw_rad=yaw)
                truth = scene_rig.true_pockets(GEOMETRY, x_m=x, y_m=y, yaw_rad=yaw)
                scene = scene_rig.render(placed, camera=camera, quantize=True, noise_k=0.0,
                                         min_range_m=MIN_RANGE_M, intrinsics=k)
                ok = 0
                for s in range(SEEDS):
                    o = detect_pockets(scene, PRIOR, dataclasses.replace(FAR_PARAMS, seed=s)).observation
                    ok += o.status == "valid" and pocket_error_m(o, truth) <= TAU_M and yaw_error(o, yaw) <= YAW_TAU
                out[f"{x},{y},{round(yaw, 4)}"] = ok
    return {"chassis": chassis, "camera": list(camera_xyz_tilt), "far": out}


def _job(job):
    kind, *a = job
    t0 = time.process_time()
    result = near(*a) if kind == "near" else far(*a)
    result["kind"], result["cpu_s"] = kind, round(time.process_time() - t0, 1)
    return result


def jobs():
    out = []
    for chassis in CHASSIS:
        _, _, front = truck_frame(chassis)
        x = round(front + CAMERA_BACK_TO_OPTICAL_M, 4)
        out.append(("far", chassis, CURRENT))
        for z in HEIGHTS:
            for tilt in TILTS:
                out.append(("near", chassis, z, tilt))
                out.append(("far", chassis, (x, 0.0, z, tilt)))
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--smoke", action="store_true", help="one near and one far job")
    args = parser.parse_args(argv)
    todo = jobs()
    if args.smoke:
        todo = [("near", "provisional", 0.27, 0.10), ("far", "provisional", CURRENT)]
    if args.workers > 1:
        with Pool(args.workers) as pool:
            results = pool.map(_job, todo, chunksize=1)
    else:
        results = [_job(j) for j in todo]
    args.out.write_text(json.dumps(results))
    for r in results:
        if r["kind"] == "near":
            print(r["chassis"], r["camera"], "handoff_cells", r["handoff_cells"],
                  "worst_gap", r["worst_max_gap_m"], "terminal", r["worst_terminal_gap_m"],
                  "no_handoff", r["seeds_without_handoff"], "post_min_roof", r["post_entry_min_roof_seeds"], "cpu", r["cpu_s"])
        else:
            print(r["chassis"], r["camera"], "far_all12", sum(v == SEEDS for v in r["far"].values()), "/", len(r["far"]), "cpu", r["cpu_s"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
