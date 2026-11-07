"""The offline cross-check of the observation start check (the lead's decision after the second B, item 5; revision plan
9.22): on one session's records from ``--pose-diag-capture``, the planner's start check
(``pallet_mission.observation_start_checker``, hybrid_astar's ``invalid_start``) at two poses on two grids --

    (pre-move pose, post-reverse pose) x (pre-move grid, submitted grid)

-- each at clearance 0 and at the planner's clearance, split by what refuses it: the bounds, the rectangles (the props
and the pallet zone the leg checks; each refusing one named), the grid (and per layer: live, the memory's observed
cells, the SLAM survey layer, minus the cells left out under the truck), and beside it the drive permission's
reversing shape (body and blades apart) on the same grids. The poses are rear-axle poses in the map frame: the
pre-move one is the rear the pre-move grid was built for (its ``own_pose``), the post-reverse one the submitted
world's ``start_rear``. Each grid carries its own correction (recorded beside it).

    python tools/start_check_cross.py --premove RUN/pose_diag_premove.npz --prepare RUN/prepare_input_1.pkl \\
        --forklift-urdf URDF --rear-axle-offset-m 0.34 --out cross.json
"""

from __future__ import annotations

import argparse
import json
import math
import pickle
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "sim" / "isaac"))

from forklift_core.planning.geometry import Bounds, FootprintCollisionChecker  # noqa: E402
from forklift_core.planning.grid_collision import CompositeChecker, OccupancyGrid  # noqa: E402
from forklift_core.planning.pallet_mission import leg_obstacles, observation_start_checker  # noqa: E402

FAR = Bounds(-1e4, 1e4, -1e4, 1e4)
LAYERS = ("live", "memory_observed", "slam_layer")


def _grid(arrays, name: str = "occupied") -> OccupancyGrid | None:
    if name not in arrays:
        return None
    cells = np.asarray(arrays[name], bool)
    if name != "occupied" and "own_left_out" in arrays:
        cells = cells & ~np.asarray(arrays["own_left_out"], bool)
    ox, oy = (float(v) for v in arrays["origin"])
    return OccupancyGrid(ox, oy, float(arrays["resolution_m"]), cells)


def _part_poses(pose, shape) -> list:
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [((x + lon * c - lat * s, y + lon * s + lat * c, yaw), fp) for fp, lat, lon in shape]


def cell(pose, grid_arrays, payload: dict, clearances, parts) -> dict:
    """One cell of the cross: the start check and its parts at each clearance."""
    world, geometry = payload["world"], payload["geometry"]
    gk = payload["grid_kwargs"]
    occ = _grid(grid_arrays)
    free, planner_clearance = observation_start_checker(
        world, payload["config"], geometry=geometry, pickup_bounds=payload.get("pickup_bounds"), occupancy=occ,
        pickup_obstacle=gk.get("pickup_obstacle"))
    rects, footprint, bounds = leg_obstacles("observe", world, geometry, pickup_obstacle=gk.get("pickup_obstacle"),
                                             pickup_bounds=payload.get("pickup_bounds"))
    out = {"pose": [float(v) for v in pose], "planner_clearance_m": planner_clearance, "by_clearance": {}}
    for cl in clearances:
        row = {"start_check_free": bool(free(pose, cl)),
               "bounds_free": bool(FootprintCollisionChecker([], footprint, bounds).free(pose, cl)),
               "rectangles_refusing": [k for k, r in enumerate(rects)
                                       if not FootprintCollisionChecker([r], footprint, FAR).free(pose, cl)],
               "grid_free": None if occ is None else bool(CompositeChecker([], occ, footprint, FAR).free(pose, cl)),
               "layers_refusing": [n for n in LAYERS if _grid(grid_arrays, n) is not None
                                   and not CompositeChecker([], _grid(grid_arrays, n), footprint, FAR).free(pose, cl)]}
        if parts is not None:
            live = _grid(grid_arrays, "live")
            row["permission_parts_live_free"] = None if live is None else all(
                CompositeChecker([], live, fp, FAR).free(p, cl) for p, fp in _part_poses(pose, parts))
            row["permission_parts_full_grid_free"] = None if occ is None else all(
                CompositeChecker([], occ, fp, FAR).free(p, cl) for p, fp in _part_poses(pose, parts))
        out["by_clearance"][f"{cl:g}"] = row
    out["rectangles"] = [[r.x_m, r.y_m, r.length_m, r.width_m, r.yaw_rad] for r in rects]
    return out


def permission_parts(geometry, blades_base_m, rear_axle_offset_m: float):
    """The drive permission's reversing shape (obstacle_layer: the body to the carriage and each blade apart), as
    (Footprint, lateral, longitudinal) about the rear axle."""
    from forklift_core.planning.geometry import Footprint

    un = geometry.unloaded_footprint
    body = Footprint(geometry.axle_to_fork_tip_m - geometry.carriage_limit_m, un.rear_m, un.half_width_m)
    off = abs(rear_axle_offset_m)
    return [(body, 0.0, 0.0)] + [
        (Footprint((x1 - x0) / 2, (x1 - x0) / 2, (y1 - y0) / 2), (y0 + y1) / 2, (x0 + off + x1 + off) / 2)
        for x0, x1, y0, y1 in blades_base_m]


def shapes_table(geometry, parts) -> dict:
    un = geometry.unloaded_footprint
    return {"planner_hull": {"front_m": un.front_m, "rear_m": un.rear_m, "half_width_m": un.half_width_m},
            "permission_reversing_parts": None if parts is None else [
                {"front_m": fp.front_m, "rear_m": fp.rear_m, "half_width_m": fp.half_width_m, "lateral_m": lat,
                 "longitudinal_m": lon} for fp, lat, lon in parts],
            "planner_layers": "live | memory observed | SLAM survey layer (with use_slam), the cells under the truck "
                              "left out; plus the props and the pallet zone as rectangles and the bounds",
            "permission_layers": "the live snapshot only (occupied, unknown and edge block it), plus the union zones "
                                 "in the unconfirmed position check's rear check"}


def cross(premove: dict, prepare: dict, parts=None) -> dict:
    payload = pickle.loads(prepare["payload"]) if isinstance(prepare["payload"], bytes) else prepare["payload"]
    pre_meta = json.loads(str(premove["meta"]))
    sent = payload["grid_kwargs"]["occupancy"]
    post_arrays = dict(prepare["arrays"])
    if "occupied" not in post_arrays and sent is not None:
        post_arrays.update(occupied=sent.occupied, origin=[sent.origin_x_m, sent.origin_y_m],
                           resolution_m=sent.resolution_m)
    pre_arrays = {k: premove[k] for k in premove if k != "meta"}
    start = payload["world"].start_rear
    poses = {"pre_move": tuple(pre_meta["own_pose"]), "post_reverse": (start.x_m, start.y_m, start.yaw_rad)}
    grids = {"pre_move": pre_arrays, "submitted": post_arrays}
    clearances = (0.0, float(payload["config"].clearance_m))
    return {"poses": {k: list(v) for k, v in poses.items()},
            "grid_corrections": {"pre_move": pre_meta.get("grid_applied"),
                                 "submitted": prepare.get("meta", {}).get("grid_applied")},
            "cells": {f"{p}@{g}": cell(poses[p], grids[g], payload, clearances, parts)
                      for p in poses for g in grids},
            "shapes": shapes_table(payload["geometry"], parts)}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--premove", type=Path, required=True)
    ap.add_argument("--prepare", type=Path, required=True, help="prepare_input_<k>.pkl (its .npz beside it)")
    ap.add_argument("--forklift-urdf", type=Path, default=None)
    ap.add_argument("--rear-axle-offset-m", type=float, default=0.34)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    with open(a.prepare, "rb") as fh:
        prep = pickle.load(fh)
    with np.load(a.prepare.with_suffix(".npz"), allow_pickle=False) as npz:
        prep["arrays"] = {k: npz[k] for k in npz.files if k != "meta"}
    with np.load(a.premove, allow_pickle=False) as npz:
        pre = {k: npz[k] for k in npz.files}
    parts = None
    if a.forklift_urdf is not None:
        from insertion_geometry import read_fork_blades_m

        payload = pickle.loads(prep["payload"])
        parts = permission_parts(payload["geometry"], read_fork_blades_m(a.forklift_urdf), a.rear_axle_offset_m)
    out = cross(pre, prep, parts)
    a.out.write_text(json.dumps(out, indent=1) + "\n")
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
