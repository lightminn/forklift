"""Obstacle recovery v4 E2 (docs/plans/2026-10-04-lidar-obstacle-map.md): the straight escape a
recovery drives before replanning at zero clearance. Synthetic grids only; the band beside the
truck is the shape of smoke 2382's return-leg stop."""

import math

import numpy as np
import pytest

from forklift_core.control import escape as E
from forklift_core.control.drive_permission import (
    DrivePermission,
    PermissionConfig,
    StoppingModel,
)
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, GridSnapshot
from forklift_core.planning.geometry import Bounds, Footprint, Rectangle
from forklift_core.planning.grid_collision import (
    FootprintCollisionChecker,
    GridFootprintChecker,
    OccupancyGrid,
    make_checker,
)

FOOT = Footprint(1.29, 0.17, 0.36)
WIDE = Bounds(-100.0, 100.0, -100.0, 100.0)
YAW = -2.945  # heading west, a little south, as at 2382's stop
PSI = 0.025960997689755683  # odometry age table, 0.3 s


def test_a_straight_keeps_the_heading_in_both_directions():
    fwd = E.straight((1.0, 2.0, 0.5), 1, 0.3)
    assert fwd.shape == (16, 3) and np.allclose(fwd[:, 2], 0.5)
    assert np.allclose(
        fwd[-1, :2], [1.0 + 0.3 * math.cos(0.5), 2.0 + 0.3 * math.sin(0.5)]
    )
    back = E.straight((1.0, 2.0, 0.5), -1, 0.3)
    assert np.allclose(
        back[-1, :2], [1.0 - 0.3 * math.cos(0.5), 2.0 - 0.3 * math.sin(0.5)]
    )


def test_the_end_region_is_the_tolerance_box_and_the_threshold_its_sum():
    region = E.end_region((0.0, 0.0, 0.0))
    assert len(region) == 27 and (0.0, 0.0, 0.0) in region
    assert max(abs(p[0]) for p in region) == pytest.approx(0.01)
    assert max(abs(p[1]) for p in region) == pytest.approx(0.01)
    assert max(abs(p[2]) for p in region) == pytest.approx(0.02)
    assert E.end_threshold_m(0.1, PSI, 1.0, 0.05) == pytest.approx(
        0.1 + 2 * math.sin(PSI / 2) + 0.05
    )
    assert E.end_threshold_m(0.1, PSI, 1.0, 0.05) == pytest.approx(0.1760, abs=1e-4)


def fake_checks(refuse=None):
    """Checks that pass everything except the named (check, direction, length); the end region's
    clearance grows 0.2 m per metre of escape."""
    refuse = refuse or set()
    measured = []

    def length_of(poses):
        return float(np.hypot(*(np.asarray(poses)[-1, :2] - np.asarray(poses)[0, :2])))

    def sweep_free(poses, direction):
        return ("sweep", direction, round(length_of(poses), 3)) not in refuse

    def path_blocked(poses, direction, extra, ramp):
        key = ("ramped" if extra else "path", direction)
        return "occupied" if key in refuse else None

    def arc_blocked(kappa, direction):
        return "occupied" if ("arc", direction) in refuse else None

    def end_clearance(region):
        centre = region[13]
        d = math.hypot(centre[0], centre[1])
        measured.append(round(d, 3))
        return ("end", round(d, 3)) not in refuse, 0.02 + 0.2 * d

    return dict(
        sweep_free=sweep_free,
        path_blocked=path_blocked,
        arc_blocked=arc_blocked,
        end_clearance=end_clearance,
    ), measured


def evaluate(checks, **kw):
    return E.evaluate(
        (0.0, 0.0, 0.0),
        kappa_now=-0.02,
        stop_length_m=0.1,
        planner_clearance_m=0.1,
        psi_rad=PSI,
        cell_m=0.05,
        **checks,
        **kw,
    )


def test_the_shortest_qualifying_candidate_is_chosen_and_each_check_can_refuse():
    checks, measured = fake_checks()
    cands = evaluate(checks)
    assert len(cands) == 8 and len(measured) == 8
    # 0.02 + 0.2 L >= 0.15 + 0.026 L + ...: only 1.0 and 1.5 qualify in either direction.
    assert [(c.direction, c.length_m) for c in cands if c.qualifies] == [
        (1, 1.0),
        (1, 1.5),
        (-1, 1.0),
        (-1, 1.5),
    ]
    pick = E.choose(cands, blocked_direction=-1)
    assert (pick.direction, pick.length_m) == (
        1,
        1.0,
    )  # of equal length, against the blocked leg
    assert (E.choose(cands, blocked_direction=1).direction) == -1
    for refused in (
        {("sweep", 1, 1.0)},
        {("path", 1)},
        {("ramped", 1)},
        {("arc", 1)},
        {("end", 1.0)},
    ):
        checks, _ = fake_checks(refused)
        pick = E.choose(evaluate(checks), blocked_direction=-1)
        assert (pick.direction, pick.length_m) != (1, 1.0), refused
    checks, measured = fake_checks({("path", 1), ("path", -1)})
    assert E.choose(evaluate(checks), blocked_direction=-1) is None
    assert measured == []  # the end region is measured only after the other checks pass
    record = cands[2].record()
    assert (
        record["qualifies"] and record["length_m"] == 1.0 and len(record["arcs"]) == 3
    )


def band_case(extra=None):
    """The truck's rear-right corner ~0.03 m below a band of occupied cells along +x (2382's shape,
    whose stop kept 0.027 m): reversing swings that corner into the band, driving forward leaves it
    at 0.19 m per metre. extra: more (i, j) cells."""
    occ = np.zeros((240, 160), dtype=bool)  # x -8..4, y -4..4
    res, ox, oy = 0.05, -8.0, -4.0
    xs, ys = ox + (np.arange(240) + 0.5) * res, oy + (np.arange(160) + 0.5) * res
    occ[np.ix_((xs > -6.0) & (xs < 1.5), (ys > 0.44) & (ys < 0.6))] = True
    for i, j in extra or ():
        occ[i, j] = True
    return OccupancyGrid(ox, oy, res, occ)


def permission_checks(grid, pose):
    perm = DrivePermission(
        PermissionConfig(
            StoppingModel(0.15, 1.5, 0.05),
            envelope_offset_m=0.01,
            evidence_max_age_s=0.2,
            envelope_ramp_m=0.0,
        )
    )
    snap = GridSnapshot(
        np.where(grid.occupied, OCCUPIED, FREE).astype(np.uint8),
        np.zeros(grid.occupied.shape),
        0.0,
        (0.0, 0.0, 0.0),
        0,
        1,
        {},
        grid.origin_x_m,
        grid.origin_y_m,
        grid.resolution_m,
    )
    rects = FootprintCollisionChecker([], FOOT, WIDE)
    hull = GridFootprintChecker(grid, FOOT, WIDE)

    def end_clearance(region):
        values = [E.clearance_m(hull.free, p) for p in region]
        return min(values) >= 0.0, min(values)

    return dict(
        sweep_free=lambda poses, d: (
            E.rectangles_sweep_free(rects, poses)
            and E.grid_sweep_blocked(grid, FOOT, pose, d, poses) is None
        ),
        path_blocked=lambda poses, d, extra, ramp: (
            perm.check_path(
                snap,
                poses,
                FOOT,
                FOOT,
                current_pose=pose,
                direction=d,
                extra_margin_m=extra,
                extra_ramp_m=ramp,
            ).blocked
        ),
        arc_blocked=lambda k, d: (
            perm.check_arc(snap, pose, k, d, FOOT, FOOT, 0.15).blocked
        ),
        end_clearance=end_clearance,
    ), hull


def test_beside_a_band_the_forward_escape_is_chosen_and_ends_with_room_to_replan():
    grid = band_case()
    pose = (0.0, 0.0, YAW)
    checks, hull = permission_checks(grid, pose)
    start = E.clearance_m(hull.free, pose)
    assert (
        0.0 < start < 0.1
    )  # inside the planning clearance: the normal replan is invalid_start
    stop = StoppingModel(0.15, 1.5, 0.05).distance_m(0.15) + 0.02
    cands = E.evaluate(
        pose,
        kappa_now=-0.0174,
        stop_length_m=stop,
        planner_clearance_m=0.1,
        psi_rad=PSI,
        cell_m=0.05,
        **checks,
    )
    back = [c for c in cands if c.direction == -1]
    assert all(c.path == "occupied" and not c.qualifies for c in back)
    pick = E.choose(cands, blocked_direction=-1)
    assert pick is not None and pick.direction == 1
    end = E.straight(pose, 1, pick.length_m)[-1]
    assert (
        E.clearance_m(hull.free, end) >= 0.1
    )  # the normal replan's start is valid there


def test_a_start_already_on_a_cell_is_not_refused_for_it_but_a_rectangle_is():
    pose = (0.0, 0.0, YAW)
    rear = (
        0.17 * math.cos(YAW + math.pi),
        0.17 * math.sin(YAW + math.pi),
    )  # the rear face's middle
    i, j = int((rear[0] + 8.0) / 0.05), int((rear[1] + 4.0) / 0.05)
    grid = band_case([(i, j)])  # a cell the rear face already straddles
    assert not GridFootprintChecker(grid, FOOT, WIDE).free(pose, 0.0)
    # Driving away from it is not refused for it (the permission's own-cell rule); backing into it is.
    assert E.grid_sweep_blocked(grid, FOOT, pose, 1, E.straight(pose, 1, 0.6)) is None
    assert (
        E.grid_sweep_blocked(grid, FOOT, pose, -1, E.straight(pose, -1, 0.6))
        == "occupied"
    )
    # The delivered pallet, which the single LiDAR never sees, ahead of the forward straight.
    pallet = Rectangle(-2.4, -0.45, 0.6, 0.8, YAW)
    rects = FootprintCollisionChecker([pallet], FOOT, WIDE)
    assert not E.rectangles_sweep_free(rects, E.straight(pose, 1, 1.0))
    assert E.rectangles_sweep_free(rects, E.straight(pose, -1, 1.0))
    composite = make_checker([pallet], FOOT, WIDE, grid)
    assert E.clearance_m(composite.free, (5.0, 5.0, 0.0)) == 0.5


def test_clearance_is_minus_one_off_a_free_pose_and_capped():
    grid = band_case()
    hull = GridFootprintChecker(grid, FOOT, WIDE)
    assert E.clearance_m(hull.free, (-2.0, 0.5, 0.0)) == -1.0
    assert E.clearance_m(hull.free, (0.0, -3.0, 0.0)) == 0.5
