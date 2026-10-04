"""Priority-5 obstacle layer glue (no Isaac): REP-117 conversion, noise cut, scan to permission."""

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

from forklift_core.perception.obstacle_grid import AgeErrorTable
from forklift_core.planning.geometry import Bounds, Footprint

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("obstacle_layer", ROOT / "sim/isaac/obstacle_layer.py")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules["obstacle_layer"] = MODULE
SPEC.loader.exec_module(MODULE)

CONFIG = MODULE.load_layer_config(ROOT / "config/obstacle_layer.yaml")
AGE = json.loads((ROOT / "config/obstacle_odometry_age.json").read_text())
TABLE = AgeErrorTable(tuple(AGE["ages_s"]), tuple(AGE["cumulative_position_m"]), tuple(AGE["cumulative_yaw_rad"]))


def layer():
    return MODULE.ObstacleLayer(
        CONFIG, hall=Bounds(-5, 10, -5, 5), error_table=TABLE,
        unloaded=Footprint(1.29, 0.17, 0.36), loaded=Footprint(1.53, 0.17, 0.40),
        body_front_m=0.884, rear_axle_x_in_base_m=-0.34, noise_seed=1,
    )


def test_only_low_planes_may_clear():
    clear = {s.name: s.may_clear for s in CONFIG["sensors"]}
    assert clear == {"high": False, "low_fl": True, "low_rr": True}


def test_ranges_follow_rep117_and_the_noise_cut():
    lay = layer()
    n = len(lay.beam_angles)
    distances = np.full(n, 3.0)
    hits = np.ones(n, bool)
    hits[:10] = False
    distances[10:20] = 0.05
    r = lay.ranges(distances, hits)
    assert np.isposinf(r[:10]).all() and np.isneginf(r[10:20]).all()
    assert np.abs(r[20:] - 3.0).max() <= 0.06 + 1e-12


def test_an_open_floor_lets_the_truck_drive_and_a_wall_stops_it():
    lay = layer()
    n = len(lay.beam_angles)
    pose = (0.0, 0.0, 0.0)
    path = np.column_stack((np.linspace(0, 4, 81), np.zeros(81), np.zeros(81)))
    for k in range(3):
        t = 0.1 * k
        raw = {s.name: (np.full(n, np.nan), np.zeros(n, bool), np.zeros(n, bool)) for s in CONFIG["sensors"]}
        lay.add_scans(t, raw, odom_rear=pose, loaded=False)
    lay.refresh(0.2, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=path, loaded=False)
    speed, _ = lay.limit(0.2, current_pose=pose, curvature_inv_m=0.0, direction=1, loaded=False, cap_mps=0.6)
    assert speed >= 0.6
    # A wall 1.62 m ahead of the rear axle (0.33 m past the fork tips), seen by every sensor.
    lay2 = layer()
    for k in range(3):
        t = 0.1 * k
        raw = {}
        for s in CONFIG["sensors"]:
            ox = s.xyz_m[0] + 0.34
            ang = lay2.beam_angles + s.yaw_rad
            d = np.where(np.cos(ang) > 0.05, (1.62 - ox) / np.cos(ang), np.nan)
            ok = np.isfinite(d) & (np.abs(s.xyz_m[1] + d * np.sin(ang)) < 3) & (d > 0)
            raw[s.name] = (np.where(ok, d, np.nan), ok, np.zeros(len(ang), bool))
        lay2.add_scans(t, raw, odom_rear=pose, loaded=False)
    lay2.refresh(0.2, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=path, loaded=False)
    speed, why = lay2.limit(0.2, current_pose=pose, curvature_inv_m=0.0, direction=1, loaded=False, cap_mps=0.6)
    assert why == "occupied" and speed < 0.6
    occ = lay2.planner_grid(0.2, (0.0, 0.0, 0.0), 0)
    i, j = occ.cell_of(1.62, 0.0)
    assert occ.occupied[i, j]


def test_beam_limits_follow_the_tilt():
    origin = np.array([0.0, 0.0, 0.10])
    level = np.array([[1.0, 0.0, 0.0]])
    up = np.array([[math.cos(0.04), 0.0, math.sin(0.04)]])
    down = np.array([[math.cos(0.04), 0.0, -math.sin(0.04)]])
    assert np.isinf(MODULE.beam_limits(origin, level, band_top_m=0.17)[0])
    assert math.isclose(MODULE.beam_limits(origin, up, band_top_m=0.17)[0], 0.07 / math.sin(0.04))
    assert math.isclose(MODULE.beam_limits(origin, down, band_top_m=0.17)[0], 0.05 / math.sin(0.04))
