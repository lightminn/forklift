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


def test_the_docking_exemption_frees_only_cells_wholly_inside_the_region():
    import numpy as np

    from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, GridSnapshot

    state = np.full((40, 40), OCCUPIED, dtype=np.uint8)
    snap = GridSnapshot(state, np.full((40, 40), np.nan), 3.0, (0.0, 0.0, 0.0), 0, 1, {}, 0.0, 0.0, 0.05)
    out = MODULE.exempt_region(snap, (1.0, 1.0, 0.5, 0.3, 0.0))
    freed = out.state == FREE
    assert freed.sum() == 10 * 6  # 0.5 x 0.3 m on a 0.05 m grid, aligned
    assert np.all(out.free_stamp[freed] == 3.0)
    assert (snap.state == OCCUPIED).all()  # the input is untouched
    turned = MODULE.exempt_region(snap, (1.0, 1.0, 0.5, 0.3, 0.3))
    assert 0 < (turned.state == FREE).sum() < 60


def test_the_permission_checks_the_hull_forward_and_the_blades_in_reverse():
    blades = ((0.87, 1.29, 0.1175, 0.1725), (0.87, 1.29, -0.1725, -0.1175))
    lay = MODULE.ObstacleLayer(
        CONFIG, hall=Bounds(-5, 10, -5, 5), error_table=TABLE,
        unloaded=Footprint(1.29, 0.17, 0.36), loaded=Footprint(1.53, 0.17, 0.40),
        body_front_m=0.884, rear_axle_x_in_base_m=-0.34, noise_seed=1, blades_rear_m=blades,
    )
    forward, _ = lay.footprints(False, 1)
    reverse, _ = lay.footprints(False, -1)
    assert forward == Footprint(1.29, 0.17, 0.36)
    assert isinstance(reverse, list) and len(reverse) == 3
    assert lay.footprints(True, -1)[0] == Footprint(1.53, 0.17, 0.40)


def _wall_raw(lay, wall_x, half=3.0):
    raw = {}
    for s in CONFIG["sensors"]:
        ox = s.xyz_m[0] + 0.34
        ang = lay.beam_angles + s.yaw_rad
        d = np.where(np.cos(ang) > 0.05, (wall_x - ox) / np.cos(ang), np.nan)
        ok = np.isfinite(d) & (np.abs(s.xyz_m[1] + d * np.sin(ang)) < half) & (d > 0)
        raw[s.name] = (np.where(ok, d, np.nan), ok, np.zeros(len(ang), bool))
    return raw


def _open_raw(lay, reach=4.0):
    # Every beam ends 4 m out on a far wall (that cell is occupied, everything closer seen free).
    return {s.name: (np.full(len(lay.beam_angles), reach), np.ones(len(lay.beam_angles), bool),
                     np.zeros(len(lay.beam_angles), bool)) for s in CONFIG["sensors"]}


def test_the_planning_memory_keeps_an_obstacle_out_of_view_and_drops_it_when_seen_free():
    # D2/D3 delta (2026-10-06): the live grid forgets a hit after 0.3 s; the plan must not.
    lay = layer()
    assert lay.memory is not None
    pose = (0.0, 0.0, 0.0)
    path = np.column_stack((np.linspace(0, 4, 81), np.zeros(81), np.zeros(81)))
    for k in range(3):
        lay.add_scans(0.1 * k, _wall_raw(lay, 1.62), odom_rear=pose, loaded=False)
    lay.refresh(0.2, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=path, loaded=False)
    # Nine seconds with no beam through the wall's cells (no-return beams would
    # say "free to the far end" and rightly clear it): the live grid drops the
    # 0.3 s old hits.
    live = lay.grid.snapshot(9.9, (0.0, 0.0, 0.0), 0)
    occ = lay.planner_grid(9.9, (0.0, 0.0, 0.0), 0)
    i, j = occ.cell_of(1.62, 0.0)
    assert occ.occupied[i, j] and not live.occupied[i, j]  # remembered, though no longer live
    # The wall is gone and the low planes see through: cleared.
    for k in range(100, 103):
        lay.add_scans(0.1 * k, _open_raw(lay), odom_rear=pose, loaded=False)
    lay.refresh(10.2, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=path, loaded=False)
    occ = lay.planner_grid(10.2, (0.0, 0.0, 0.0), 0)
    i, j = occ.cell_of(1.62, 0.0)
    assert not occ.occupied[i, j]


def test_remembered_cells_under_the_truck_are_left_out_of_the_plan_copy_only():
    lay = layer()
    lay.memory.add_hits(np.array([0.3]), np.array([0.0]), 0.0)  # under the body at (0, 0, 0)
    occ = lay.planner_grid(0.1, (0.0, 0.0, 0.0), 0, own_pose=(0.0, 0.0, 0.0), loaded=False)
    i, j = occ.cell_of(0.3, 0.0)
    assert not occ.occupied[i, j]
    a, b = lay.memory.cell(0.3, 0.0)
    assert lay.memory.occupied()[a, b]  # the memory itself keeps it
