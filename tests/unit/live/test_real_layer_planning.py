"""docs/design/2026-11-19-real-safety-wiring.md 2 (stage 5a): the planning grid's split of the registered pallet's
returns on the real planner. One scan of the S10-A field with the pallet ahead goes through the real layer into the
assembly's occupancy hooks and ``plan_transport``: without the split the approach straight collides with the face's
returns (placed with their error bound), with it the mission plans, and a separate box beside the approach still
blocks it (Codex 5a impl round 3 P3)."""

import json
import math
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

from forklift_core.live import real_autonomy as ra
from forklift_core.live.real_assembly import docking_occupancy, scenario_of
from forklift_core.live.real_layer import ManualWorker, RealLayer, RealScan, real_layer_configs
from forklift_core.perception.obstacle_grid import AgeErrorTable, snapshot_to_occupancy
from forklift_core.planning.geometry import Footprint, Rectangle
from forklift_core.planning.pallet_mission import PalletSite, SyntheticMissionGeometry, plan_transport

ROOT = Path(__file__).resolve().parents[3]
LAYER = yaml.safe_load((ROOT / "config/real/obstacle_layer_real.yaml").read_text())
G = SyntheticMissionGeometry()
PAL = (3.0, 0.0, 0.0)
KEY = ("P1", PAL, 0)


def _corners(cx, cy, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * u - s * v, cy + s * u + c * v)
            for u, v in ((length / 2, width / 2), (length / 2, -width / 2), (-length / 2, -width / 2),
                         (-length / 2, width / 2))]


def _raycast(rects, n=720):
    """A LiDAR at the start's rear axle (the map origin) seeing the rectangles' edges."""
    ang = np.linspace(-math.pi, math.pi, n, endpoint=False)
    r = np.full(n, np.inf)
    for k, a in enumerate(ang):
        dx, dy = math.cos(a), math.sin(a)
        for cs in rects:
            for (ax, ay), (bx, by) in zip(cs, cs[1:] + cs[:1]):
                ex, ey = bx - ax, by - ay
                den = dx * ey - dy * ex
                if abs(den) < 1e-12:
                    continue
                tt = (ax * ey - ay * ex) / den
                uu = (ax * dy - ay * dx) / den
                if tt > 0.0 and 0.0 <= uu <= 1.0:
                    r[k] = min(r[k], tt)
    return ang, r


def _occupancy(rects, split: bool):
    age = json.loads((ROOT / LAYER["odometry_age"]).read_text())
    table = AgeErrorTable(tuple(age["ages_s"]), tuple(age["cumulative_position_m"]), tuple(age["cumulative_yaw_rad"]))
    hall = SimpleNamespace(x_min_m=-1.0, x_max_m=6.0, y_min_m=-2.5, y_max_m=2.5)
    grid, perm = real_layer_configs(LAYER, error_table=table, budget_s=0.1, scan_period_s=0.1, state_delay_s=0.05,
                                    v_scan_mps=0.225, kappa_max_inv_m=1.0, hall=hall)
    body = Footprint(0.35, 0.2, 0.3)
    lay = RealLayer(grid, perm, sensors={"lidar": ((0.0, 0.0, 0.0), True, {"measured": False})}, unloaded=body,
                    loaded=body, body=body, placer=lambda a, b: (0.0, 0.0, 0.0), budget_s=0.1, overdue_end_s=1.0,
                    scan_period_s=0.1, scan_max_age_s=0.35, worker=ManualWorker())
    # the loop's rectangle (RealLoop._layer_tick): the registered pallet grown by the range bound
    bound = grid.sensor_bound_m
    lay.planning_pallet = ((*PAL, G.pallet_depth_m / 2 + bound, G.pallet_width_m / 2 + bound), KEY)
    ang, r = _raycast(rects)
    lay.add_scan(RealScan("lidar", 1.0, 1.1, ang, r, np.ones(len(ang), bool), np.zeros(len(ang), bool),
                          np.full(len(ang), np.inf)), 1.1)
    lay.worker.step()
    lay.install_ready(1.12)
    return lay.planning_grid(KEY) if split else snapshot_to_occupancy(lay.snapshot)


def _plan(occ):
    """As the assembly's grid hooks hand it to the planner (RealHooks.grid_kwargs for a mission)."""
    scenario = scenario_of(replace(ra.load_s10a(ROOT / "config/real/s10a.yaml"), check_pallet_pose=PAL))
    target = PalletSite(*PAL)
    return plan_transport(scenario, geometry=G, occupancy=occ, docking_occupancy=docking_occupancy(occ, target, G),
                          pickup_obstacle=Rectangle(PAL[0], PAL[1], G.pallet_depth_m, G.pallet_width_m, PAL[2]))


PALLET = _corners(*PAL, G.pallet_depth_m, G.pallet_width_m)


def test_without_the_split_the_face_returns_block_the_approach():
    plan = _plan(_occupancy([PALLET], split=False))
    assert not plan.success and plan.status.startswith("approach:"), plan.status


def test_with_the_split_the_mission_plans():
    assert _plan(_occupancy([PALLET], split=True)).success


def test_a_box_beside_the_approach_still_blocks_with_the_split():
    box = _corners(2.2, 0.75, 0.0, 0.3, 0.3)
    plan = _plan(_occupancy([PALLET, box], split=True))
    assert not plan.success and plan.status.startswith("approach:"), plan.status
