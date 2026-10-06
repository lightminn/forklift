"""Drive permission from the obstacle grid (priority-5 plan D4); synthetic grids only.

The Codex L0 review counterexamples are kept as tests: an obstacle cell just
outside the body, a steering-held stop that leaves a curving path, FREE
evidence older than its limit, and a short path end.
"""

import math

import numpy as np

from forklift_core.control.drive_permission import (
    DrivePermission,
    PermissionConfig,
    StoppingModel,
    arc_poses,
)
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN, GridSnapshot
from forklift_core.planning.geometry import Footprint

FOOT = Footprint(1.29, 0.17, 0.36)
BODY = Footprint(0.75, 0.17, 0.36)
STOP = StoppingModel(latency_s=0.3, decel_mps2=1.0, margin_m=0.05)
CONFIG = PermissionConfig(STOP, envelope_offset_m=0.05, evidence_max_age_s=0.2)
STRAIGHT = np.column_stack((np.linspace(0, 6, 121), np.zeros(121), np.zeros(121)))


def snapshot(fill=FREE, stamp=0.0, free_at=0.0):
    state = np.full((200, 120), fill, dtype=np.uint8)  # x -1..9, y -3..3
    free_stamp = np.where(state == FREE, free_at, np.nan)
    return GridSnapshot(state, free_stamp, stamp, (0.0, 0.0, 0.0), 0, 1, {"low": stamp}, -1.0, -3.0, 0.05)


def set_cell(snap, x, y, value):
    i, j = int(math.floor((x + 1.0) / 0.05)), int(math.floor((y + 3.0) / 0.05))
    snap.state[i, j] = value
    snap.free_stamp[i, j] = np.nan if value != FREE else 0.0


def allowed(perm, t, pose=(0.0, 0.0, 0.0), curvature=0.0, direction=1, cap=0.6, sensors=None):
    return perm.allowed_speed(
        t, sensors if sensors is not None else {"low": t}, current_pose=pose, curvature_inv_m=curvature,
        direction=direction, footprint=FOOT, own_footprint=BODY, speed_cap_mps=cap,
    )


def test_the_stopping_model_inverts():
    for v in (0.0, 0.1, 0.3, 0.6):
        assert math.isclose(STOP.speed_for(STOP.distance_m(v)), v, abs_tol=1e-12)
    assert STOP.speed_for(0.01) == 0.0


def test_arcs_follow_the_curvature_in_both_directions():
    poses, s = arc_poses((0.0, 0.0, 0.0), 0.5, 1, math.pi, 0.05)  # a quarter turn of radius 2
    assert math.isclose(poses[-1, 0], 2.0, abs_tol=1e-9) and math.isclose(poses[-1, 1], 2.0, abs_tol=1e-9)
    back, _ = arc_poses((0.0, 0.0, 0.0), 0.0, -1, 1.0, 0.05)
    assert math.isclose(back[-1, 0], -1.0)


def test_a_free_corridor_allows_the_cap():
    perm = DrivePermission(CONFIG)
    perm.update(snapshot(), STRAIGHT, FOOT, BODY, current_pose=(0, 0, 0))
    speed, _ = allowed(perm, 0.05)
    assert speed >= 0.6


def test_an_obstacle_cell_just_outside_the_body_is_not_exempt():
    # Codex: body front 0.75, cell [0.775, 0.825] ahead was ignored with an envelope-grown exclusion.
    perm = DrivePermission(CONFIG)
    snap = snapshot()
    set_cell(snap, 0.80, 0.0, OCCUPIED)
    check = perm.update(snap, STRAIGHT, BODY, BODY, current_pose=(0, 0, 0))
    assert check.blocked == "occupied" and check.verified_m < 0.1
    speed, why = perm.allowed_speed(0.05, {"low": 0.05}, current_pose=(0, 0, 0), curvature_inv_m=0.0, direction=1,
                                    footprint=BODY, own_footprint=BODY, speed_cap_mps=0.6)
    assert speed == 0.0 and why == "occupied"


def test_a_steering_held_stop_is_checked_not_the_curving_plan():
    # Codex: the plan turns left, the wheels are straight; an obstacle on the straight line.
    left_turn, _ = arc_poses((0.0, 0.0, 0.0), 0.8, 1, 1.5, 0.02)
    snap = snapshot()
    set_cell(snap, 1.50, -0.35, OCCUPIED)
    perm = DrivePermission(CONFIG)
    check = perm.update(snap, left_turn, FOOT, BODY, current_pose=(0, 0, 0))
    assert check.blocked is None  # the plan itself is clear
    speed, why = allowed(perm, 0.05, curvature=0.0)
    assert why == "occupied" and speed <= STOP.speed_for(0.25)


def test_free_evidence_expires_cell_by_cell():
    perm = DrivePermission(CONFIG)
    snap = snapshot(stamp=0.15, free_at=0.0)  # the FREE cells came from a scan at t = 0
    perm.update(snap, STRAIGHT, FOOT, BODY, current_pose=(0, 0, 0))
    assert allowed(perm, 0.19)[0] > 0
    assert allowed(perm, 0.22) == (0.0, "evidence_stale")


def test_a_silent_sensor_stops_the_truck():
    perm = DrivePermission(CONFIG)
    perm.update(snapshot(stamp=1.0, free_at=1.0), STRAIGHT, FOOT, BODY, current_pose=(0, 0, 0))
    assert allowed(perm, 1.05, sensors={"low": 1.0, "rear": 0.7}) == (0.0, "sensor_silent:rear")


def test_a_short_path_end_still_limits_the_speed():
    # Codex: verified to an end 0.01 m away used to give an unlimited speed.
    perm = DrivePermission(CONFIG)
    short = np.array([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0]])
    check = perm.update(snapshot(), short, FOOT, BODY, current_pose=(0, 0, 0))
    assert check.reached_end
    speed, _ = allowed(perm, 0.05)
    assert speed <= max(STOP.speed_for(0.01 + STOP.margin_m), CONFIG.end_creep_mps) + 1e-12


def test_unknown_ahead_blocks_but_the_cells_under_the_body_do_not():
    perm = DrivePermission(CONFIG)
    snap = snapshot()
    for x in np.arange(-0.15, 0.6, 0.05):
        for y in np.arange(-0.34, 0.35, 0.05):
            set_cell(snap, x, y, UNKNOWN)
    assert perm.update(snap, STRAIGHT, FOOT, BODY, current_pose=(0, 0, 0)).blocked is None
    set_cell(snap, 2.5, 0.3, UNKNOWN)
    check = perm.update(snap, STRAIGHT, FOOT, BODY, current_pose=(0, 0, 0))
    assert check.blocked == "unknown" and check.verified_m < 1.3


def test_a_cell_the_body_only_partly_covers_is_still_checked():
    # Codex L0b: body end x = 0.763, obstacle at x = 0.780 in the cell [0.75, 0.80].
    body = Footprint(0.763, 0.17, 0.36)
    perm = DrivePermission(CONFIG)
    snap = snapshot()
    set_cell(snap, 0.78, 0.0, OCCUPIED)
    perm.update(snap, STRAIGHT, body, body, current_pose=(0, 0, 0))
    speed, why = perm.allowed_speed(0.05, {"low": 0.05}, current_pose=(0, 0, 0), curvature_inv_m=0.0, direction=1,
                                    footprint=body, own_footprint=body, speed_cap_mps=0.6)
    assert speed == 0.0 and why == "occupied"


def test_the_sweep_between_samples_is_covered():
    # Codex L0b: kappa 0.6 from the origin grazes [1.30, 1.35] x [-0.40, -0.35] at s = 8.3 mm.
    snap = snapshot()
    set_cell(snap, 1.32, -0.37, OCCUPIED)
    perm = DrivePermission(CONFIG)
    perm.update(snap, STRAIGHT[:2] * [0, 0, 0] + STRAIGHT[:2], FOOT, BODY, current_pose=(0, 0, 0))
    speed, why = allowed(perm, 0.05, curvature=0.6)
    assert why == "occupied" and speed < 0.6


def test_the_occupancy_grid_keeps_its_own_copy():
    from forklift_core.planning.grid_collision import GridFootprintChecker, OccupancyGrid
    from forklift_core.planning.geometry import Bounds

    raw = np.zeros((200, 200), dtype=bool)
    grid = OccupancyGrid(-5.0, -5.0, 0.05, raw)
    raw[100, 100] = True  # the caller edits its array afterwards
    checker = GridFootprintChecker(grid, FOOT, Bounds(-5, 5, -5, 5))
    assert checker.free((-0.5, 0.0, 0.0)) and not grid.occupied.flags.writeable


def test_an_unknown_cell_beside_the_body_blocks_a_turning_stop():
    # Codex checkpoint: half width 0.36, kappa 0.6, 0.3 m/s; the side cell
    # [0.45, 0.50] x [0.35, 0.40] is entered after 7.5 cm of the stop.
    perm = DrivePermission(CONFIG)
    snap = snapshot()
    set_cell(snap, 0.475, 0.375, UNKNOWN)
    perm.update(snap, STRAIGHT, BODY, BODY, current_pose=(0, 0, 0))
    speed, why = perm.allowed_speed(0.05, {"low": 0.05}, current_pose=(0, 0, 0), curvature_inv_m=0.6, direction=1,
                                    footprint=BODY, own_footprint=BODY, speed_cap_mps=0.3)
    assert why == "unknown" and speed < 0.3


# Body + fork blades (D4: the gap between the blades is not the truck). Blades
# as the provisional model: x 0.870..1.290 from the rear axle, y +-0.1445,
# half width 0.0275.
BLADE = Footprint(0.21, 0.21, 0.0275)
SHAPE = [(BODY, 0.0, 0.0), (BLADE, 0.1445, 1.08), (BLADE, -0.1445, 1.08)]


def test_reversing_away_from_an_unseen_fork_gap_is_allowed():
    snap = snapshot()
    set_cell(snap, 0.95, 0.0, UNKNOWN)  # between the blades, ahead of the body face (0.75)
    perm = DrivePermission(CONFIG)
    back = np.column_stack((np.linspace(0, -0.6, 13), np.zeros(13), np.zeros(13)))  # grid starts at x -1
    assert perm.update(snap, back, SHAPE, BODY, current_pose=(0.0, 0.0, 0.0), direction=-1).blocked is None
    # The hull would have refused it.
    assert perm.update(snap, back, FOOT, BODY, current_pose=(0.0, 0.0, 0.0), direction=-1).blocked == "unknown"


def test_driving_forward_into_an_object_between_the_blades_is_refused():
    snap = snapshot()
    set_cell(snap, 0.82, 0.0, OCCUPIED)  # in the gap, within the body face's stop
    perm = DrivePermission(CONFIG)
    check = perm.update(snap, STRAIGHT, SHAPE, BODY, current_pose=(0.0, 0.0, 0.0), direction=1)
    assert check.blocked == "occupied" and check.verified_m < 0.1


def test_an_unseen_cell_under_a_blade_tip_blocks_forward():
    snap = snapshot()
    set_cell(snap, 1.31, 0.1445, UNKNOWN)
    perm = DrivePermission(CONFIG)
    check = perm.update(snap, STRAIGHT, SHAPE, BODY, current_pose=(0.0, 0.0, 0.0), direction=1)
    assert check.blocked == "unknown" and check.verified_m < 0.1


def test_shape_meets_uses_the_parts_not_their_hull():
    from forklift_core.control.drive_permission import shape_meets
    from forklift_core.planning.geometry import Rectangle

    in_gap = Rectangle(1.1, 0.0, 0.1, 0.1, 0.0)
    on_blade = Rectangle(1.1, 0.15, 0.04, 0.04, 0.0)
    assert not shape_meets(in_gap, SHAPE, (0.0, 0.0, 0.0))
    assert shape_meets(in_gap, FOOT, (0.0, 0.0, 0.0))
    assert shape_meets(on_blade, SHAPE, (0.0, 0.0, 0.0))



def test_a_cell_whose_corners_lie_in_the_union_but_not_in_one_part_is_not_wholly_own():
    # Codex checkpoint P1: the body + blades union is not convex.
    from forklift_core.control.drive_permission import DrivePermission

    snap = snapshot()
    # A cell straddling the body's front corner and the left blade's root: its
    # corners can lie in the union while its interior pokes out of both.
    whole, partial = DrivePermission._own_cells(snap, (0.0, 0.0, 0.0), SHAPE)
    for a, b in whole:
        x0, y0 = -1.0 + a * 0.05, -3.0 + b * 0.05
        assert any(
            (lon - fp.rear_m <= x0 and x0 + 0.05 <= lon + fp.front_m
             and abs(lat) - fp.half_width_m <= min(abs(y0), abs(y0 + 0.05))
             and max(abs(y0 - lat), abs(y0 + 0.05 - lat)) <= fp.half_width_m)
            for fp, lat, lon in SHAPE
        )


def test_shape_cells_union_matches_a_row_wise_unique():
    from forklift_core.control.drive_permission import footprint_cells, shape_cells
    from forklift_core.perception.obstacle_grid import GridSnapshot

    rng = np.random.default_rng(3)
    for _ in range(50):
        snap = _snap_for_shape_test(rng)
        pose = (float(rng.uniform(-1, 1)), float(rng.uniform(-1, 1)), float(rng.uniform(-3, 3)))
        shape = ((Footprint(0.6, 0.5, 0.36), 0.0, 0.0), (Footprint(0.21, 0.21, 0.0275), 0.145, 0.74),
                 (Footprint(0.21, 0.21, 0.0275), -0.145, 0.74))
        got, _ = shape_cells(snap, pose, shape, 0.01, direction=1)
        parts = [footprint_cells(snap, pose, fp, 0.01, direction=1, lateral_m=lat, longitudinal_m=lon)[0]
                 for fp, lat, lon in shape]
        assert np.array_equal(got, np.unique(np.concatenate(parts), axis=0))


def _snap_for_shape_test(rng):
    state = np.full((120, 120), FREE, dtype=np.int8)
    free_stamp = np.zeros((120, 120))
    return GridSnapshot(state, free_stamp, 0.0, (0.0, 0.0, 0.0), 0, 1, {"low": 0.0}, -3.0, -3.0, 0.05)
