"""Offline checks of the live riib-default validation fixtures (docs/plans/2026-11-08-live-riib-defaults-validation.md 2):

- the low-clearance guard under the default and the fixture configuration on a straight unloaded leg beside a
  pallet-height prop (low_clearance.guard_cap), and
- the return start at the withdrawal end for several --withdrawal-m values, with the live geometry and planner config
  taken from a captured live preparation payload (#353's prepare_input_<k>.pkl), against D-1 of each session scene:
  free at clearance 0 and at the planner's, the boxed first step, and pallet_mission.return_start_backoff.

    PYTHONPATH=src python docs/validation/assets/live_riib/fixture_offline.py PREPARE_INPUT.pkl LINE_M.json RANDOM_M.json
"""

import hashlib
import json
import math
import pickle
import sys

import numpy as np

from forklift_core.control import low_clearance as lc
from forklift_core.control.drive_permission import StoppingModel
from forklift_core.planning import Bounds, Pose2D, Rectangle
from forklift_core.planning.grid_collision import make_checker
from forklift_core.planning.hybrid_astar import first_step_free
from forklift_core.planning.pallet_mission import (
    RETURN_START_BACKOFF_CAP_M,
    boxed_fallback_config,
    return_start_backoff,
)

STOP = StoppingModel(0.15, 1.5, 0.05)


def sha(path):
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def guard_table(unloaded):
    configs = (("default", lc.LowClearanceConfig()), ("fixture", lc.LowClearanceConfig(margin_m=0.30, deviation_m=0.015)))
    print("guard: straight unloaded leg, prop 0.6 x 0.8 m (height irrelevant here), speed 0.3 m/s;"
          " cap at truck offsets toward the prop 0/0.01/0.02/0.03/0.05 m ('-' = none)")
    for name, cfg in configs:
        for c_plan in (0.10, 0.20, 0.30):
            prop = Rectangle(2.5, unloaded.half_width_m + c_plan + 0.4, 0.6, 0.8, 0.0)
            path = np.column_stack((np.linspace(0, 5, 501), np.zeros(501), np.zeros(501)))
            leg = lc.LegClearance(path, unloaded, search_props=[prop], slot_props=[prop])
            caps = []
            for dev in (0.0, 0.01, 0.02, 0.03, 0.05):
                r = lc.guard_cap((1.0, dev, 0.0), 0.0, 1, 0.3, unloaded, leg, progress_m=1.0, leg_remaining_m=4.0,
                                 end_allowance_m=0.008, stopping=STOP, config=cfg)
                caps.append("-" if not np.isfinite(r.allowed_mps) else f"{r.allowed_mps:.2f}")
            print(f"  {name:8s} planned {c_plan:.2f}: {' '.join(caps)}")


def withdrawal_table(payload, scene_paths):
    g, c = payload["geometry"], payload["config"]
    fp = g.unloaded_footprint
    print(f"withdrawal: inserted_offset {g.inserted_offset_m:.3f} m, pallet {g.pallet_depth_m} x {g.pallet_width_m} m,"
          f" footprint {fp}, clearance {c.clearance_m} m")
    for path in scene_paths:
        d = json.load(open(path))
        props = [Rectangle(**{k: p["rectangle"][k] for k in ("x_m", "y_m", "length_m", "width_m", "yaw_rad")})
                 for p in d["static_props"]]
        bounds = Bounds(**d["bounds"])
        dests = d["destinations"] if isinstance(d["destinations"], list) else list(d["destinations"].values())
        x, y, yaw = dests[0]["x_m"], dests[0]["y_m"], dests[0]["yaw_rad"]
        obstacles = props + [Rectangle(x, y, g.pallet_depth_m, g.pallet_width_m, yaw)]
        checker = make_checker(obstacles, fp, bounds, None)
        for w in (0.30, 0.35, 0.40, 0.45, 0.55):
            r = -(g.inserted_offset_m + w)
            start = Pose2D(x + r * math.cos(yaw), y + r * math.sin(yaw), yaw)
            pose = (start.x_m, start.y_m, start.yaw_rad)
            free0, free1 = checker.free(pose, 0.0), checker.free(pose, c.clearance_m)
            step = first_step_free(pose, checker, boxed_fallback_config(c)) if free1 else None
            res = return_start_backoff(start, obstacles, fp, bounds, c.clearance_m, props, 0.0,
                                       RETURN_START_BACKOFF_CAP_M, first_step_config=c)
            print(f"  {d['name']:10s} D-1 w={w:.2f}: free@0 {free0!s:5s} free@{c.clearance_m} {free1!s:5s}"
                  f" first_step {step!s:5s} backoff {res['status']} advice {res.get('advice_m')} length {res.get('length_m')}")


def main():
    prep, scenes = sys.argv[1], sys.argv[2:]
    for p in (prep, *scenes):
        print(f"input {p} sha256 {sha(p)}")
    with open(prep, "rb") as fh:
        payload = pickle.loads(pickle.load(fh)["payload"])
    guard_table(payload["geometry"].unloaded_footprint)
    withdrawal_table(payload, scenes)


if __name__ == "__main__":
    main()
