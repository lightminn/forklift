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
    blades = ((0.944, 1.29, 0.1175, 0.1725), (0.944, 1.29, -0.1725, -0.1175))
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

    lay = single_layer()
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
    lay = single_layer(b_w_m=None, e_w_m=None)
    open_scans(lay, 0.2)
    lay.known = [pallet]
    assert not lay.certify_withdrawal(back, lay.parts_shape, 0.48, gaps)
    lay.refresh(0.2, (0.0, 0.0, 0.0), 0, current_pose=pose, path_ahead=back, loaded=False, direction=-1)
    held, _ = lay.limit(0.2, current_pose=pose, curvature_inv_m=0.0, direction=-1, loaded=False, cap_mps=0.12)
    assert held == 0.0
    # Measured and within the 0.0265 m budget: certified, the corridor starts the truck.
    lay = single_layer(b_w_m=0.010, e_w_m=0.010)
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
    lat_only = single_layer(b_w_m=0.010, e_w_m=0.010, e_w_full_m=None)
    lat_only.known = [pallet]
    assert lat_only.certify_withdrawal(back, lat_only.parts_shape, 0.48, gaps)
    lat_only.end_withdrawal(now_s=4.6)
    assert lat_only.known[0].fix_stamp_s == 0.0
    lay = single_layer(b_w_m=0.010, e_w_m=0.010, e_w_full_m=0.010)
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
