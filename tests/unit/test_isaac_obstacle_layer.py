"""Priority-5 obstacle layer glue (no Isaac): REP-117 conversion, noise cut, scan to permission."""

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

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


def test_a_sinking_beam_stops_clearing_at_the_band_bottom():
    # Plan v10 D0 (Codex v10 2nd P1-3): with only the 1.05 m plane, a beam tilted down
    # must stop speaking at h_lo, not at the floor, or it slips under a cover between
    # supports. h_lo = 1.05 - 5 m x tan(1 deg) leaves the 5 m clearing reach at 1 deg.
    origin = np.array([0.0, 0.0, 1.05])
    down = np.array([[math.cos(0.04), 0.0, -math.sin(0.04)]])
    assert math.isclose(MODULE.beam_limits(origin, down, band_top_m=1.15, band_bottom_m=0.963)[0], 0.087 / math.sin(0.04))
    assert math.isclose(MODULE.beam_limits(origin, down, band_top_m=1.15)[0], 1.0 / math.sin(0.04))  # floor margin
    one_deg = np.array([[math.cos(math.radians(1)), 0.0, -math.sin(math.radians(1))]])
    h_lo = 1.05 - 5.0 * math.tan(math.radians(1))
    assert MODULE.beam_limits(origin, one_deg, band_top_m=1.15, band_bottom_m=h_lo)[0] >= 5.0 - 1e-9
    cfg = MODULE.load_layer_config(ROOT / "config/obstacle_layer_single.yaml")
    measured = 1.05 - 5.0 * math.tan(math.radians(0.0697))  # the measured-chassis gate's largest tilt, every tick
    assert math.isclose(cfg["band_bottom_m"], measured, abs_tol=5e-4)


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


def test_with_several_own_poses_only_cells_under_all_of_them_are_left_out():
    """Obstacle recovery v4: an escape's end region rebuilds one grid that holds each pose's own."""
    lay = layer()
    lay.memory.add_hits(np.array([0.3, 1.5]), np.array([0.0, 0.0]), 0.0)
    poses = [(0.0, 0.0, 0.0), (0.4, 0.0, 0.0)]  # no blades here: the hull's front at 1.29 / 1.69
    occ = lay.planner_grid(0.1, (0.0, 0.0, 0.0), 0, own_pose=poses, loaded=False)
    i, j = occ.cell_of(0.3, 0.0)
    assert not occ.occupied[i, j]  # under the body at both poses
    i, j = occ.cell_of(1.5, 0.0)
    assert occ.occupied[i, j]  # under the hull at the second pose only
    alone = lay.planner_grid(0.1, (0.0, 0.0, 0.0), 0, own_pose=poses[1], loaded=False)
    assert not alone.occupied[i, j]


def _grid_kwargs_harness(tmp_path, slam=None, with_map=False, **outer):
    """Run the runner's grid_kwargs (AST) against a real obstacle layer."""
    import ast
    import math as _math
    import time as _time
    from types import SimpleNamespace

    from forklift_core.perception.planning_memory import SLAM_OCCUPIED
    from forklift_core.planning.geometry import Rectangle

    script = ROOT / "sim/isaac/run_transport.py"
    fn = next(n for n in ast.walk(ast.parse(script.read_text()))
              if isinstance(n, ast.FunctionDef) and n.name == "grid_kwargs")
    lay = layer()
    obstacle = {"layer": lay, "applied": (0.0, 0.0, 0.0), "version": 0, "plans": [], "last_stamp": 1.0,
                "run_wall_start": _time.time() - 5}
    if with_map:
        data = np.full((40, 40), -1, dtype=np.int8)
        data[20, 20] = SLAM_OCCUPIED
        np.savez(tmp_path / "latest_map.npz", data=data, origin=np.array([0.0, 0.0, 0.0]), resolution_m=0.05,
                 index=1, after_scan_id=0, stamp_ns=int(0.5e9))
    scope = {
        "np": np, "math": _math, "os": __import__("os"), "Rectangle": Rectangle, "grid_planning": True, "obstacle": obstacle,
        "args": SimpleNamespace(slam_map_dir=tmp_path if with_map else None),
        "geometry": SimpleNamespace(pallet_depth_m=0.6, pallet_width_m=0.8, loaded_footprint=Footprint(1.53, 0.17, 0.40),
                                    unloaded_footprint=Footprint(1.29, 0.17, 0.36)),
        "planner_config": SimpleNamespace(clearance_m=0.1),
        "pickup_zone": Rectangle(3.0, 0.0, 1.0, 1.0, 0.0),
        **outer,
    }
    if slam is not None:
        scope["slam"] = slam
    exec(compile(ast.Module([fn], []), str(script), "exec"), scope)
    return scope["grid_kwargs"], obstacle


def test_grid_kwargs_runs_before_the_drive_loop_and_with_a_slam_map(tmp_path):
    # l5_video5/7: grid_kwargs crashed at the first plan (a free 'slam', a renamed attribute).
    from types import SimpleNamespace

    gk, obstacle = _grid_kwargs_harness(tmp_path)
    out = gk("observe")
    assert "occupancy" in out and obstacle["plans"][-1]["memory"] is True
    tracker = SimpleNamespace(applied=((0.0, 0.0, 0.0),), mode="tracking")
    gk, obstacle = _grid_kwargs_harness(tmp_path, slam={"tracker": tracker, "scan_id": 5}, with_map=True,
                                        rear=np.array([-2.0, 0.0, 0.0]), loaded=False)
    out = gk(None)
    assert obstacle["plans"][-1]["slam_used"] is True and obstacle["plans"][-1]["slam_map"]["index"] == 1
    assert out["occupancy"].occupied[out["occupancy"].cell_of(1.025, 1.025)]
    gk(None)  # the same instant: the cached bundle
    assert len(obstacle["plans"]) == 2 and obstacle["plans"][0]["memory_revision"] == obstacle["plans"][1]["memory_revision"]


def test_grid_kwargs_takes_an_end_region_for_its_own_cells_and_the_return_band(tmp_path):
    """Obstacle recovery v4: an escape scores its end region on one grid -- the remembered cells and
    the return's delivered-pallet band are left out only under every region pose; its record is labelled."""
    from forklift_core.planning.geometry import Rectangle

    gk, obstacle = _grid_kwargs_harness(tmp_path, rear=np.array([0.0, 0.0, 0.0]), loaded=False)
    lay = obstacle["layer"]
    lay.memory.add_hits(np.array([0.3, 1.5]), np.array([0.0, 0.0]), 0.0)
    target = Rectangle(1.0, 0.0, 0.6, 0.8, 0.0)  # a delivered pallet whose band covers x 0.3..1.7
    region = [(0.0, 0.0, 0.0), (0.4, 0.0, 0.0)]
    out = gk("return", target, own=region, band_pose=region, label="escape_end")
    occ = out["occupancy"]
    assert obstacle["plans"][-1]["kind"] == "escape_end"
    assert not occ.occupied[occ.cell_of(0.3, 0.0)]  # under the truck at both poses
    assert occ.occupied[occ.cell_of(1.5, 0.0)]  # under it at the second pose only: kept
    one = gk("return", target, own=region[1], band_pose=region[1])["occupancy"]
    assert not one.occupied[one.cell_of(1.5, 0.0)]


def test_a_map_swapped_after_it_was_opened_is_judged_by_the_file_read(tmp_path):
    # Codex re-review 3 P2: the path was stat()ed after it had been opened; a
    # bridge swap between the two gave an older file the newer file's mtime.
    import os
    import time as _time
    from types import SimpleNamespace

    tracker = SimpleNamespace(applied=((0.0, 0.0, 0.0),), mode="tracking")
    gk, obstacle = _grid_kwargs_harness(tmp_path, slam={"tracker": tracker, "scan_id": 5}, with_map=True,
                                        rear=np.array([-2.0, 0.0, 0.0]), loaded=False)
    path = tmp_path / "latest_map.npz"
    old = _time.time() - 60
    os.utime(path, (old, old))  # another run's file
    fresh = tmp_path / "fresh.npz"
    with np.load(path) as m:
        np.savez(fresh, **{k: m[k] for k in m.files})
    real_load = np.load

    def load_then_swap(f, *a, **k):
        out = real_load(f, *a, **k)
        os.replace(fresh, path)  # the bridge writes a new file now
        return out

    gk.__globals__["np"] = SimpleNamespace(**{n: getattr(np, n) for n in dir(np) if not n.startswith("__")})
    gk.__globals__["np"].load = load_then_swap
    gk(None)
    note = obstacle["plans"][-1]["slam_map_note"]
    assert note is not None and note["why"] == "file_before_run"
    assert obstacle["plans"][-1]["slam_map"] is None


SINGLE = MODULE.load_layer_config(ROOT / "config/obstacle_layer_single.yaml")


def single_layer(**known):
    cfg = dict(SINGLE)
    if known:
        cfg["known_pallets"] = {**SINGLE["known_pallets"], **known}
    blades = ((0.944, 1.29, 0.1175, 0.1725), (0.944, 1.29, -0.1725, -0.1175))  # synthetic: the 0.29 m fork spacing as recorded then, not the current model
    return MODULE.ObstacleLayer(
        cfg, hall=Bounds(-5, 10, -5, 5), error_table=TABLE,
        unloaded=Footprint(1.29, 0.17, 0.36), loaded=Footprint(1.53, 0.17, 0.40),
        body_front_m=0.944, rear_axle_x_in_base_m=-0.34, noise_seed=1, blades_rear_m=blades,
    )


def open_scans(lay, until_s, pose=(0.0, 0.0, 0.0)):
    n = len(lay.beam_angles)
    for k in range(int(round(until_s / 0.1)) + 1):
        raw = {s.name: (np.full(n, np.nan), np.zeros(n, bool), np.zeros(n, bool)) for s in lay.sensors}
        lay.add_scans(0.1 * k, raw, odom_rear=pose, loaded=False)


def test_the_single_layer_marks_a_delivered_pallet_the_plane_cannot_see():
    from forklift_core.perception.known_obstacles import KnownRect

    # The committed single layer keeps known pallets off until D8 (user, 2026-10-08).
    assert not single_layer().known_enabled
    lay = single_layer(enabled=True)
    assert lay.known_enabled and lay.shared_with_slam and lay.band_bottom_m == 1.044
    path = np.column_stack((np.linspace(0, 4, 81), np.zeros(81), np.zeros(81)))
    open_scans(lay, 0.2)
    lay.known = [KnownRect((1.29 + 0.2 + 0.3, 0.0, 0.6, 0.8, 0.0), fix_stamp_s=0.0, r_fix_m=0.05, rho_m=None,
                           frame_correction=(0.0, 0.0, 0.0))]
    lay.refresh(0.2, (0.0, 0.0, 0.0), 0, current_pose=(0.0, 0.0, 0.0), path_ahead=path, loaded=False, direction=1)
    speed, why = lay.limit(0.2, current_pose=(0.0, 0.0, 0.0), curvature_inv_m=0.0, direction=1, loaded=False, cap_mps=0.6)
    assert speed < 0.6 and why in ("occupied", "unknown")


def test_a_withdrawal_needs_its_certificate_and_then_starts():
    from forklift_core.perception.known_obstacles import KnownRect, fork_pocket_gaps

    pose = (0.0, 0.0, 0.0)
    back = np.column_stack((np.linspace(0, -0.6, 121), np.zeros(121), np.zeros(121)))
    pallet = KnownRect((1.29, 0.0, 0.6, 0.8, 0.0), fix_stamp_s=0.0, r_fix_m=0.01, rho_m=None, frame_correction=(0, 0, 0))
    gaps = fork_pocket_gaps(0.0, 0.0, blade_centre_m=0.145, blade_half_width_m=0.0275, blade_length_m=0.30,
                            pocket_inner_m=0.0725, pocket_outer_m=0.300)
    # Unmeasured b_w / e_w: refused, and the delivered pallet holds the truck.
    lay = single_layer(b_w_m=None, e_w_m=None, enabled=True)
    open_scans(lay, 0.2)
    lay.known = [pallet]
    assert not lay.certify_withdrawal(back, lay.parts_shape, 0.48, gaps)
    lay.refresh(0.2, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=back, loaded=False, direction=-1)
    held, _ = lay.limit(0.2, current_pose=pose, curvature_inv_m=0.0, direction=-1, loaded=False, cap_mps=0.12)
    assert held == 0.0
    # Measured and within the 0.0265 m budget: certified, the corridor starts the truck.
    lay = single_layer(b_w_m=0.010, e_w_m=0.010, enabled=True)
    open_scans(lay, 0.2)
    lay.known = [pallet]
    assert lay.certify_withdrawal(back, lay.parts_shape, 0.48, gaps)
    assert lay.permission.config.step_m == 0.005
    lay.refresh(0.2, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=back, loaded=False, direction=-1)
    speed, _ = lay.limit(0.2, current_pose=pose, curvature_inv_m=0.0, direction=-1, loaded=False, cap_mps=0.12)
    assert speed >= 0.12
    lay.end_withdrawal()
    assert lay.permission.config.step_m == 0.02 and lay.corridor is None


def test_codex_stage1_a_certified_withdrawal_hands_the_return_a_fresh_pallet():
    # Codex stage-1 P1-6: after 0.55 m / 4.6 s the delivered pallet had grown to 0.39 m and
    # turned unknown the moment the corridor ended. The certified stop re-fixes it at b_w + e_w.
    from forklift_core.perception.known_obstacles import KnownRect, fork_pocket_gaps

    back = np.column_stack((np.linspace(0, -0.6, 121), np.zeros(121), np.zeros(121)))
    pallet = KnownRect((1.29, 0.0, 0.6, 0.8, 0.0), fix_stamp_s=0.0, r_fix_m=0.01, rho_m=None, frame_correction=(0, 0, 0))
    gaps = fork_pocket_gaps(0.0, 0.0, blade_centre_m=0.145, blade_half_width_m=0.0275, blade_length_m=0.30,
                            pocket_inner_m=0.0725, pocket_outer_m=0.300)
    # Without the full drift bound the certified stop does not re-fix (Codex stage-1 2nd P1-2).
    lat_only = single_layer(b_w_m=0.010, e_w_m=0.010, e_w_full_m=None, enabled=True)
    lat_only.known = [pallet]
    assert lat_only.certify_withdrawal(back, lat_only.parts_shape, 0.48, gaps)
    lat_only.end_withdrawal(now_s=4.6)
    assert lat_only.known[0].fix_stamp_s == 0.0
    lay = single_layer(b_w_m=0.010, e_w_m=0.010, e_w_full_m=0.010, enabled=True)
    lay.known = [pallet]
    assert lay.certify_withdrawal(back, lay.parts_shape, 0.48, gaps)
    pose = (-0.55, 0.0, 0.0)
    onward = np.column_stack((np.linspace(-0.55, -2.5, 81), np.zeros(81), np.zeros(81)))
    open_scans(lay, 4.6, pose=pose)
    lay.end_withdrawal(now_s=4.6)
    assert lay.known[0].fix_stamp_s == 4.6 and lay.known[0].r_fix_m == pytest.approx(0.020)
    lay.refresh(4.6, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=onward, loaded=False, direction=-1)
    speed, why = lay.limit(4.6, current_pose=pose, curvature_inv_m=0.0, direction=-1, loaded=False, cap_mps=0.3)
    assert speed > 0.0, why


def test_a_candidate_is_checked_on_the_permission_snapshot_without_changing_it():
    """Obstacle recovery v4 D1: the escape's path and stop-arc checks use the snapshot and shapes the
    permission uses, and leave its installed path check alone."""
    lay = layer()
    pose = (0.0, 0.0, 0.0)
    path = np.column_stack((np.linspace(0, 4, 81), np.zeros(81), np.zeros(81)))
    for k in range(3):
        raw = {}
        for s in CONFIG["sensors"]:
            ox = s.xyz_m[0] + 0.34
            ang = lay.beam_angles + s.yaw_rad
            d = np.where(np.cos(ang) > 0.05, (1.62 - ox) / np.cos(ang), np.nan)
            ok = np.isfinite(d) & (np.abs(s.xyz_m[1] + d * np.sin(ang)) < 3) & (d > 0)
            raw[s.name] = (np.where(ok, d, np.nan), ok, np.zeros(len(ang), bool))
        lay.add_scans(0.1 * k, raw, odom_rear=pose, loaded=False)
    installed = lay.refresh(0.2, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=path, loaded=False)
    snap = lay.snapshot
    ahead = lay.check_candidate(path, current_pose=pose, direction=1, loaded=False)
    assert ahead == installed and ahead.blocked == "occupied"
    back = np.column_stack((np.linspace(0, -0.5, 26), np.zeros(26), np.zeros(26)))
    assert lay.check_candidate(back, current_pose=pose, direction=-1, loaded=False).blocked != "occupied"
    arc = lay.check_arc_candidate(current_pose=pose, curvature_inv_m=0.0, direction=1, loaded=False, cap_mps=0.6)
    speed, why = lay.limit(0.2, current_pose=pose, curvature_inv_m=0.0, direction=1, loaded=False, cap_mps=0.6)
    assert arc == lay.permission.last_estop
    assert lay.snapshot is snap and lay.permission.path_check is installed


def test_a_turnaround_arc_needs_its_start_verified_and_no_known_obstacle_further_on():
    """Return turnaround (docs/plans/2026-10-12-return-forward-turnaround.md): cells clear only within
    max_clear_m of the sensor, so a 90-degree full-lock reverse arc (5.4 m, the body sweeping farther)
    starts on what the permission's own walk verifies; a known occupied cell anywhere on it refuses it."""
    from forklift_core.control.drive_permission import arc_poses
    from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN, GridSnapshot

    lay = layer()
    pose = (0.0, 0.0, 0.0)
    samples, _ = arc_poses(pose, 0.29, -1, math.pi / 2 / 0.29, 0.05)
    res, x0, y0 = 0.05, -10.0, -10.0
    xs = x0 + (np.arange(400) + 0.5) * res
    near = np.hypot(*np.meshgrid(xs, xs, indexing="ij")) < 4.0  # what the scan has cleared

    def snap(extra=None):
        state = np.where(near, FREE, UNKNOWN).astype(np.uint8)
        if extra is not None:
            i, j = int((extra[0] - x0) / res), int((extra[1] - y0) / res)
            state[i - 2:i + 3, j - 2:j + 3] = OCCUPIED
        free = np.where(state == FREE, 1.0, np.nan)
        return GridSnapshot(state, free, 1.0, (0.0, 0.0, 0.0), 0, 1, {"high": 1.0}, x0, y0, res)

    lay.snapshot = snap()
    whole = lay.check_candidate(samples, current_pose=pose, direction=-1, loaded=False, lookahead_m=6.0)
    assert whole.blocked == "unknown"  # the whole arc cannot be verified from its start
    assert lay.check_turnaround_arc(samples, current_pose=pose) is None
    end = samples[-1]
    lay.snapshot = snap(extra=(end[0], end[1]))  # an obstacle seen where the arc ends
    assert lay.check_turnaround_arc(samples, current_pose=pose) == "occupied_ahead"
    lay.snapshot = snap(extra=tuple(samples[20][:2]))  # within the start's walk
    assert lay.check_turnaround_arc(samples, current_pose=pose) == "occupied"


def _start_check_harness(lay, tracker, pallet_pose, held=(False, False)):
    """The runner's prepare_start_ok and prep_obstacle_of (AST) with its grid_kwargs, against a real layer."""
    import ast
    import math as _math
    import time as _time
    from dataclasses import replace as _replace
    from types import SimpleNamespace

    from forklift_core.localization.slam_pose import compose, invert
    from forklift_core.mission.targets import Pose, TargetsBook
    from forklift_core.planning.geometry import Rectangle
    from forklift_core.safety.pallet_permit import SESSION_PRIOR_GROW_M, preparation_obstacle, session_pallet_props

    script = ROOT / "sim/isaac/run_transport.py"
    tree = ast.parse(script.read_text())
    fns = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
           and n.name in ("grid_kwargs", "prep_obstacle_of", "prepare_start_ok", "session_scenario_")]
    assert len(fns) == 4
    obstacle = {"layer": lay, "applied": tuple(tracker.applied[0]), "version": 0, "plans": [], "last_stamp": 0.2,
                "run_wall_start": _time.time() - 5}
    tb = TargetsBook()
    tb.add_pallet("P-1", "epal6", Pose(*pallet_pose), depth_m=1.2, width_m=0.8)
    tb.update_pose("P-1", Pose(*pallet_pose), basis="recognised", uncertainty_m=0.21)
    core = SimpleNamespace(commands=SimpleNamespace(mission=SimpleNamespace(pallet_id="P-1")), targets=tb,
                           _held_estimates=lambda: held)
    scenario = SimpleNamespace(props=(), bounds=Bounds(-5, 10, -5, 5), pickup=SimpleNamespace(x_m=0, y_m=0, yaw_rad=0))
    geometry = SimpleNamespace(pallet_depth_m=1.2, pallet_width_m=0.8, loaded_footprint=Footprint(1.53, 0.17, 0.40),
                               unloaded_footprint=Footprint(1.29, 0.17, 0.36))
    scope = {
        "np": np, "math": _math, "os": __import__("os"), "Rectangle": Rectangle, "grid_planning": True,
        "obstacle": obstacle, "args": SimpleNamespace(slam_map_dir=None), "geometry": geometry,
        "planner_config": SimpleNamespace(clearance_m=0.1), "pickup_zone": None,
        "slam": {"tracker": tracker, "scan_id": 4},
        "compose": compose, "invert": invert, "replace": _replace, "core": core, "grid_world": lambda sc: sc,
        # plan 2026-10-22-session-scene-pallet-props 2.4: the preparation's inputs per pallet (P-1 alone here)
        "live_session_": {"pallets": {"P-1": {"zone": None, "reference": (99.0, 99.0, 0.0)}}, "scenario": scenario,
                          "nominal": {}, "shape": (1.2, 0.8, 0.144)},
        "live_preparation_obstacle": preparation_obstacle, "live_session_pallet_props": session_pallet_props,
        "SESSION_PRIOR_GROW_M": SESSION_PRIOR_GROW_M,
        "prep_check_cache_": {}, "rear": np.zeros(3),
        "loaded": False,  # grid_kwargs masks the truck's own cells only inside the drive loop (Codex round 2)
    }
    exec(compile(ast.Module(fns, []), str(script), "exec"), scope)
    return scope, obstacle


def _scanned_wall_layer():
    """Wall hits up to 0.71 s, then open scans to 1.00 s (Codex D4b impl round 2 P2-1's timing: the release at
    1.02 s re-projects after the last scan)."""
    lay = layer()
    pose = (0.0, 0.0, 0.0)
    path = np.column_stack((np.linspace(0, 4, 81), np.zeros(81), np.zeros(81)))
    for stamp in (0.69, 0.70, 0.71):
        lay.add_scans(stamp, _wall_raw(lay, 1.62), odom_rear=pose, loaded=False)
    lay.refresh(0.71, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=path, loaded=False)
    for stamp in (0.8, 0.9, 1.0):
        lay.add_scans(stamp, _wall_raw(lay, 2.4), odom_rear=pose, loaded=False)
    lay.refresh(1.0, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=path, loaded=False)
    return lay


@pytest.mark.parametrize("received", [(0.05, 0.6, 0.02), (0.0, 0.7, 0.0)])
def test_the_start_check_previews_the_release_exactly_and_leaves_the_layer_alone(received):
    """Persistent console plan D4b (Codex impl round 1 P2-3, P2-4; round 2 P2-1): held, the next preparation's start
    check is judged as it will be after the end release -- the release's re-projection at its time t under the
    received correction (the core's _reproject: version + 1, refresh at t), then the preparation's own grid under it
    -- with the own pose in the cache key, and without changing the shared layer."""
    import copy
    import pickle
    from types import SimpleNamespace

    from forklift_core.localization.slam_pose import compose, invert

    t_release = 1.02
    held_tracker = SimpleNamespace(mode="holding", applied=((0.0, 0.0, 0.0), 0.0, 0), received=(received, 1.0, 9))
    base = _scanned_wall_layer()
    lay = copy.deepcopy(base)
    scope, obstacle = _start_check_harness(lay, held_tracker, (8.0, 0.0, 0.0))
    obstacle["last_stamp"] = 1.0
    before = pickle.dumps(lay.__dict__)
    frame = compose(received, invert((0.0, 0.0, 0.0)))
    seen = set()
    for x in np.arange(-1.0, 1.31, 0.1):
        pose = np.array([x, 0.0, 0.0])
        scope["rear"] = pose  # the truck stands where it is checked (the own pose varies within one stamp)
        preview = scope["prepare_start_ok"](pose, extra_m=0.0, preview=True, t=t_release)
        # The release made for real on another copy: tracking under the received correction, the core's
        # _reproject (version + 1, refresh at t from the released pose), then the preparation's own check.
        real_tracker = SimpleNamespace(mode="tracking", applied=(received, 1.0, 9), received=(received, 1.0, 9))
        real_layer = copy.deepcopy(base)
        real_scope, real_obstacle = _start_check_harness(real_layer, real_tracker, (8.0, 0.0, 0.0))
        released = tuple(float(v) for v in compose(frame, tuple(pose)))
        real_obstacle.update(applied=tuple(received), version=1, last_stamp=1.0)
        real_layer.refresh(t_release, tuple(received), 1, current_pose=released, path_ahead=np.array([released]),
                           loaded=False, direction=0)
        real_scope["rear"] = np.array(released)
        occupancy = real_scope["grid_kwargs"]("observe")["occupancy"]  # the preparation's call, as it runs
        from forklift_core.planning.grid_collision import make_checker
        from forklift_core.planning.pallet_mission import leg_obstacles

        geometry = real_scope["geometry"]
        obstacles, footprint, bounds = leg_obstacles(
            "observe", real_scope["live_session_"]["scenario"], geometry,
            pickup_obstacle=real_scope["prep_obstacle_of"](real_scope["core"].targets.pallets["P-1"], False),
            pickup_bounds=None)
        real = bool(make_checker(obstacles, footprint, bounds, occupancy).free(released, 0.1))
        assert preview == real, x
        seen.add(preview)
    assert seen == {True, False}  # the sweep crosses the wall's clearance
    assert pickle.dumps(lay.__dict__) == before  # the shared layer untouched


def test_the_start_check_for_another_target_is_that_preparations_own_check():
    """Plan 2026-10-24-leg-end-budget-stands R1: with ``target_id`` the start check is the one the next preparation of
    that pallet runs -- its pickup zone as live_prepare draws it (a prior pallet at its first place: the bay's zone),
    the others (this mission's delivered one among them) as its neighbours -- and it differs from the check for this
    mission's pallet where the zone and the neighbour outline differ."""
    from dataclasses import dataclass
    from types import SimpleNamespace

    from forklift_core.mission.targets import Pose
    from forklift_core.planning.geometry import Rectangle
    from forklift_core.planning.grid_collision import make_checker
    from forklift_core.planning.pallet_mission import leg_obstacles

    tracker = SimpleNamespace(mode="tracking", applied=((0.0, 0.0, 0.0), 0.0, 0), received=None)
    scope, _ = _start_check_harness(layer(), tracker, (-3.0, 0.0, 0.0))
    tb = scope["core"].targets
    tb.carried("P-1")
    tb.put_down("P-1", Pose(-3.0, 0.0, 0.0), uncertainty_m=0.21)  # this mission's, delivered
    second = (3.0, 0.0, 0.0)
    tb.add_pallet("P-2", "epal6", Pose(*second), depth_m=1.2, width_m=0.8)
    scope["live_session_"]["pallets"]["P-2"] = {"zone": Rectangle(3.0, 0.0, 2.6, 2.0, 0.0), "reference": second}
    scope["live_session_"]["nominal"] = {"P-1": (-3.0, 0.0, 0.0), "P-2": second}

    @dataclass(frozen=True)
    class Scene:  # session_scenario_ adds the other pallets with dataclasses.replace
        props: tuple
        bounds: object
        pickup: object

    base = scope["live_session_"]["scenario"]
    scope["live_session_"]["scenario"] = Scene((), base.bounds, base.pickup)
    geometry = scope["geometry"]
    seen = set()
    for x in np.arange(0.0, 2.01, 0.05):
        pose = np.array([x, 0.0, 0.0])
        scope["rear"] = pose
        occupancy = scope["grid_kwargs"]("observe")["occupancy"]
        obstacles, footprint, bounds = leg_obstacles(
            "observe", scope["session_scenario_"]("P-2"), geometry,
            pickup_obstacle=scope["prep_obstacle_of"](tb.pallets["P-2"], True), pickup_bounds=None)
        own = bool(make_checker(obstacles, footprint, bounds, occupancy).free(tuple(pose), 0.1))
        assert scope["prepare_start_ok"](pose, target_id="P-2") == own, x
        seen.add((own, scope["prepare_start_ok"](pose)))
    assert (False, True) in seen  # P-2's zone refuses where P-2 as a neighbour still lets this mission's check pass


def test_the_start_check_carries_the_pallet_only_when_the_hold_found_it():
    from types import SimpleNamespace

    received = (0.0, 0.5, 0.0)
    tracker = SimpleNamespace(mode="holding", applied=((0.0, 0.0, 0.0), 0.0, 0), received=(received, 0.2, 3))
    # a pallet beside the truck, its grown outline 0.45 m clear of the hull's clearance band: carried with the pose
    # (+0.5 m in y) the gap stays; left where it is, the moved pose's band reaches 0.05 m into it
    pallet = (0.4, 1.52, 0.0)
    pose = np.array([-0.4, 0.0, 0.0])
    held_scope, _ = _start_check_harness(layer(), tracker, pallet, held=(True, False))
    held_scope["rear"] = pose
    map_scope, _ = _start_check_harness(layer(), tracker, pallet, held=(False, False))
    map_scope["rear"] = pose
    assert held_scope["prepare_start_ok"](pose) is True  # the gap kept
    assert map_scope["prepare_start_ok"](pose) is False  # a map estimate stays: the pose moved 0.5 m across it


def test_the_start_check_preview_takes_the_release_reprojection_at_its_time():
    """Codex D4b impl round 3 (its round 2 P2-1 counterexample): a hit at 0.71 s already in the memory, a no-return
    scan at 1.00 s, both on a plane that may not clear; the release at 1.02 s re-projects first. Without that step
    the preview saw 58 occupied cells and refused; the real preparation after the release sees 13 and starts."""
    import copy
    from types import SimpleNamespace

    from forklift_core.perception.obstacle_grid import ObstacleScan
    from forklift_core.planning.grid_collision import make_checker
    from forklift_core.planning.pallet_mission import leg_obstacles

    zero = (0.0, 0.0, 0.0)

    def scan(stamp, distance):
        return ObstacleScan(stamp_s=stamp, sensor="high", odom_rear=zero, laser_in_rear=zero,
                            angles_rad=np.array([0.0]), ranges_m=np.array([distance]),
                            self_hit=np.array([False]), may_clear=False)

    base = layer()
    base.grid.add_scan(scan(0.71, 1.30))
    base.planner_grid(0.71, zero, 0)  # in the memory under the applied (old) frame
    base.grid.add_scan(scan(1.00, np.nan))
    base.refresh(1.00, zero, 0, current_pose=zero, path_ahead=np.array([zero]), loaded=False)
    received = (0.0, 0.7, 0.0)

    held = SimpleNamespace(mode="holding", applied=(zero, 0.0, 0), received=(received, 1.0, 9))
    scope, obstacle = _start_check_harness(copy.deepcopy(base), held, (8.0, 0.0, 0.0))
    obstacle["last_stamp"] = 1.0
    scope["rear"] = np.zeros(3)
    preview = scope["prepare_start_ok"](np.zeros(3), preview=True, t=1.02)
    stale = _start_check_harness(copy.deepcopy(base), held, (8.0, 0.0, 0.0))[0]
    stale["obstacle"]["last_stamp"] = 1.0
    stale["rear"] = np.zeros(3)
    without_release = stale["prepare_start_ok"](np.zeros(3), preview=True, t=None)  # no release step

    real_layer = copy.deepcopy(base)
    tracking = SimpleNamespace(mode="tracking", applied=(received, 1.0, 9), received=(received, 1.0, 9))
    real_scope, real_obstacle = _start_check_harness(real_layer, tracking, (8.0, 0.0, 0.0))
    real_obstacle.update(applied=received, version=1, last_stamp=1.0)
    released = (0.0, 0.7, 0.0)
    real_layer.refresh(1.02, received, 1, current_pose=released, path_ahead=np.array([released]), loaded=False,
                       direction=0)
    real_scope["rear"] = np.array(released)
    occupancy = real_scope["grid_kwargs"]("observe")["occupancy"]
    obstacles, footprint, bounds = leg_obstacles(
        "observe", real_scope["live_session_"]["scenario"], real_scope["geometry"],
        pickup_obstacle=real_scope["prep_obstacle_of"](real_scope["core"].targets.pallets["P-1"], False),
        pickup_bounds=None)
    real = bool(make_checker(obstacles, footprint, bounds, occupancy).free(released, 0.1))
    assert (preview, real, without_release) == (True, True, False)


def _arcs():
    """Reverse arcs from the origin: free, and long enough to reach the wall behind (x = -3.5)."""
    out = {}
    for name, kappa, length in (("straight", 0.0, 1.0), ("left", 0.4, 1.2), ("long", 0.0, 3.5)):
        s = np.linspace(0.0, length, 41)
        yaw = -kappa * s
        x = -np.where(kappa == 0, s, np.sin(kappa * s) / max(kappa, 1e-9))
        y = np.where(kappa == 0, 0.0, -(1 - np.cos(kappa * s)) / max(kappa, 1e-9))
        out[name] = np.column_stack((x, y, yaw))
    return out


def _wall_behind_layer():
    lay = layer()
    pose = (0.0, 0.0, 0.0)
    raw = {}
    for s in CONFIG["sensors"]:
        ox = s.xyz_m[0] + 0.34
        ang = lay.beam_angles + s.yaw_rad
        d = np.where(np.cos(ang) < -0.05, (-3.5 - ox) / np.cos(ang), np.nan)
        ok = np.isfinite(d) & (np.abs(s.xyz_m[1] + d * np.sin(ang)) < 3.0) & (d > 0)
        raw[s.name] = (np.where(ok, d, 4.0), np.ones(len(ang), bool), np.zeros(len(ang), bool))
    for k in range(3):
        lay.add_scans(0.1 * k, raw, odom_rear=pose, loaded=False)
    lay.refresh(0.2, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=np.array([pose]), loaded=False,
                direction=-1)
    return lay


def test_the_turnaround_arc_check_is_one_function_frozen_and_picklable():
    """Persistent console plan D11: the layer's check and the frozen checker give the same verdicts; the checker
    pickles (the worker gets it) and keeps the snapshot it was made with when the layer changes."""
    import pickle

    from forklift_core.control.turnaround_check import TurnaroundArcCheck

    lay = _wall_behind_layer()
    checker = lay.turnaround_arc_check()
    assert isinstance(checker, TurnaroundArcCheck)
    arcs = _arcs()
    expected = {name: lay.check_turnaround_arc(poses, current_pose=tuple(poses[0])) for name, poses in arcs.items()}
    assert len(set(map(str, expected.values()))) >= 2  # the cases differ (a verdict passes, another does not)
    assert {name: checker(poses) for name, poses in arcs.items()} == expected
    again = pickle.loads(pickle.dumps(checker))
    assert {name: again(poses) for name, poses in arcs.items()} == expected
    assert again.callback_spec["kind"] == "turnaround_arc"
    lay.snapshot = None  # the layer moves on; the frozen checker does not
    assert {name: checker(poses) for name, poses in arcs.items()} == expected
    assert layer().turnaround_arc_check()(arcs["straight"]) == "no_snapshot"



def test_a_candidate_permit_is_judged_in_its_own_context_leaving_the_layer_alone():
    """Scene-mode replan plan (docs/plans/2026-10-23-scene-replan.md 2.1 step 6): a candidate's first gear leg on the
    processed snapshot, in a fresh DrivePermission -- verified distance, blocked, the start allowance at the command's
    curvature and at the measured one. The layer's permission (its path check, snapshot and last stop arc) stays as it
    is, and a candidate is allowed where the installed path has the truck held at 0 (Codex plan round 2 P3)."""
    lay = layer()
    pose = (0.0, 0.0, 0.0)
    path = np.column_stack((np.linspace(0, 4, 81), np.zeros(81), np.zeros(81)))
    for k in range(3):
        lay.add_scans(0.1 * k, _wall_raw(lay, 1.62), odom_rear=pose, loaded=False)
    installed = lay.refresh(0.2, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=path, loaded=False)
    held, why = lay.limit(0.2, current_pose=pose, curvature_inv_m=0.0, direction=1, loaded=False, cap_mps=0.6)
    snap, estop = lay.snapshot, lay.permission.last_estop
    assert installed.blocked == "occupied"
    back = np.column_stack((np.linspace(0, -1.0, 51), np.zeros(51), np.zeros(51)))
    ok = lay.candidate_permit(0.2, back, current_pose=pose, direction=-1, loaded=False, curvature_inv_m=0.0,
                              measured_curvature_inv_m=0.05, cap_mps=0.6)
    assert ok["ok"] and ok["blocked"] is None and ok["allowed_mps"] > 0 and ok["measured_allowed_mps"] > 0
    assert ok["verified_m"] >= min(lay.permission.config.lookahead_m, 1.0) - 1e-9
    ahead = lay.candidate_permit(0.2, path, current_pose=pose, direction=1, loaded=False, curvature_inv_m=0.0,
                                 measured_curvature_inv_m=0.0, cap_mps=0.6)
    assert not ahead["ok"] and ahead["blocked"] == "occupied" and ahead["why"] == "blocked:occupied"
    # the layer's own permission untouched
    assert lay.snapshot is snap and lay.permission.path_check is installed and lay.permission.last_estop is estop
    assert lay.limit(0.2, current_pose=pose, curvature_inv_m=0.0, direction=1, loaded=False,
                     cap_mps=0.6) == (held, why)


def test_a_candidate_permit_needs_the_whole_lookahead_or_the_leg_verified_and_a_snapshot():
    lay = layer()
    pose = (0.0, 0.0, 0.0)
    short = np.column_stack((np.linspace(0, -0.3, 16), np.zeros(16), np.zeros(16)))
    none = lay.candidate_permit(0.0, short, current_pose=pose, direction=-1, loaded=False, curvature_inv_m=0.0,
                                measured_curvature_inv_m=0.0, cap_mps=0.6)
    assert not none["ok"] and none["why"] == "no_snapshot"
    for k in range(3):
        lay.add_scans(0.1 * k, _open_raw(lay), odom_rear=pose, loaded=False)
    lay.refresh(0.2, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=short, loaded=False)
    leg = lay.candidate_permit(0.2, short, current_pose=pose, direction=-1, loaded=False, curvature_inv_m=0.0,
                               measured_curvature_inv_m=0.0, cap_mps=0.6)
    assert leg["ok"] and leg["verified_m"] == pytest.approx(0.3, abs=1e-6)  # a 0.3 m leg: its whole length


def test_a_strict_candidate_check_covers_only_the_permission_off_zone_within_the_lookahead():
    """Leg-failures plan A1 (2026-10-25; Isaac ct2_07ca86aa random_m s1, ct2_41a2a7e8 line_m s2: verified 2.4 of 3.0 m,
    the zone 9-11 m ahead, refused for 60 s): strict verifies [max(0, zone start), min(lookahead, leg)] only -- an
    unknown before the zone is the driving permission's; the zone beyond the lookahead leaves nothing to verify; the
    zone's own walk is not lengthened to a whole lookahead. (A 2.5 m leg: the test layer's ground is known to ~4 m.)"""
    from dataclasses import replace as dc_replace

    from forklift_core.perception.obstacle_grid import OCCUPIED, UNKNOWN

    lay = layer()
    pose = (0.0, 0.0, 0.0)
    path = np.column_stack((np.linspace(0, 2.5, 51), np.zeros(51), np.zeros(51)))
    for k in range(3):
        lay.add_scans(0.1 * k, _open_raw(lay, reach=9.0), odom_rear=pose, loaded=False)
    lay.refresh(0.2, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=path, loaded=False)
    snap = lay.snapshot

    def band(state, x0, x1, value):
        out = state.copy()
        i0 = int(round((x0 - snap.origin_x_m) / snap.resolution_m))
        i1 = int(round((x1 - snap.origin_x_m) / snap.resolution_m))
        out[i0:i1, :] = value
        return out

    kw = dict(current_pose=pose, direction=1, loaded=False, curvature_inv_m=0.0, measured_curvature_inv_m=0.0,
              cap_mps=0.6, strict=True)
    assert lay.candidate_permit(0.2, path, strict_from_m=2.0, **kw)["ok"]  # all known: the zone's part passes
    # the recorded case: an unknown ahead of the truck, the zone far beyond the lookahead
    lay.snapshot = dc_replace(snap, state=band(snap.state, 2.4, 2.6, UNKNOWN))
    assert not lay.candidate_permit(0.2, path, **kw)["ok"]  # the old rule (zone start 0): refused
    far = lay.candidate_permit(0.2, path, strict_from_m=9.25, **kw)
    assert far["ok"] and far["strict_checked_m"] == 0.0  # nothing of the zone within the lookahead
    # the zone starting inside the lookahead: [2.0, 2.5] verified, an unknown before it left to the permission
    lay.snapshot = dc_replace(snap, state=band(snap.state, 1.6, 1.75, UNKNOWN))
    before = lay.candidate_permit(0.2, path, strict_from_m=2.0, **kw)
    assert before["ok"] and before["strict_checked_m"] == pytest.approx(0.5, abs=1e-6)
    lay.snapshot = dc_replace(snap, state=band(snap.state, 2.4, 2.6, UNKNOWN))
    inside = lay.candidate_permit(0.2, path, strict_from_m=2.0, **kw)
    assert not inside["ok"] and inside["why"] == "blocked:unknown"
    # past the zone's own sweep (its last pose's nose at 3.79 m): a whole lookahead from 2.0 would reach it
    lay.snapshot = dc_replace(snap, state=band(snap.state, 3.85, 3.95, UNKNOWN))
    assert lay.candidate_permit(0.2, path, strict_from_m=2.0, **kw)["ok"]
    # an occupied cell is refused wherever it is within the lookahead, strict or not
    lay.snapshot = dc_replace(snap, state=band(snap.state, 1.6, 1.75, OCCUPIED))
    occupied = lay.candidate_permit(0.2, path, strict_from_m=9.25, **kw)
    assert not occupied["ok"] and occupied["why"] == "blocked:occupied"


def test_a_candidate_permit_leaves_an_unknown_beyond_the_verified_distance_to_the_permission_unless_strict():
    """Scene replan plan 7판 (Isaac sri_b907b91 s0: verified 2.42 of 3.0 m, start allowed 0.354 m/s, refused): an
    unknown ahead is the driving permission's, as on any path -- unless the leg reaches where the permission does not
    act (strict). An occupied cell behind the unknown is still found (walked past it, Codex Isaac round 2)."""
    from dataclasses import replace as dc_replace

    from forklift_core.perception.obstacle_grid import OCCUPIED, UNKNOWN

    lay = layer()
    pose = (0.0, 0.0, 0.0)
    path = np.column_stack((np.linspace(0, 4, 81), np.zeros(81), np.zeros(81)))
    for k in range(3):
        lay.add_scans(0.1 * k, _open_raw(lay, reach=9.0), odom_rear=pose, loaded=False)  # no wall within the walk
    lay.refresh(0.2, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=path, loaded=False)
    snap = lay.snapshot

    def band(state, x0, x1, value):
        out = state.copy()
        i0 = int(round((x0 - snap.origin_x_m) / snap.resolution_m))
        i1 = int(round((x1 - snap.origin_x_m) / snap.resolution_m))
        out[i0:i1, :] = value
        return out

    kw = dict(current_pose=pose, direction=1, loaded=False, curvature_inv_m=0.0, measured_curvature_inv_m=0.0,
              cap_mps=0.6)
    lay.snapshot = dc_replace(snap, state=band(snap.state, 3.4, 3.6, UNKNOWN))
    strict = lay.candidate_permit(0.2, path, strict=True, **kw)
    relaxed = lay.candidate_permit(0.2, path, strict=False, **kw)
    assert not strict["ok"] and strict["why"] == "blocked:unknown" and strict["allowed_mps"] > 0
    assert relaxed["ok"] and relaxed["blocked"] == "unknown" and relaxed["verified_m"] < relaxed["need_m"]
    assert relaxed["strict"] is False and relaxed["occupied_ahead"] is False
    behind = band(band(snap.state, 3.4, 3.6, UNKNOWN), 3.75, 3.85, OCCUPIED)
    lay.snapshot = dc_replace(snap, state=behind)
    past = lay.candidate_permit(0.2, path, strict=False, **kw)
    assert not past["ok"] and past["why"] == "blocked:occupied" and past["occupied_ahead"] is True
    lay.snapshot = dc_replace(snap, state=band(snap.state, 0.9, 1.1, UNKNOWN))
    start = lay.candidate_permit(0.2, path, strict=False, **kw)  # unknown inside the stopping distance: no start
    assert not start["ok"] and start["why"].startswith("start:")
