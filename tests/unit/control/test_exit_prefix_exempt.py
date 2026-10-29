"""Exit-prefix exemption (docs/plans/2026-10-28-exit-prefix-extract-rules.md D1, D2): the raw watch's region (the
remaining exempt path and the stop arcs at the measured and commanded curvature), the cells it verified, and the one
snapshot transform that reads only the OCCUPIED cells of a verified corridor FREE, stamped with the verifying scan."""

import dataclasses
import math

import numpy as np
import pytest

from forklift_core.control import exit_prefix_exempt as xe
from forklift_core.control.drive_permission import DrivePermission, PermissionConfig, StoppingModel
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN, GridSnapshot
from forklift_core.planning import Footprint

FOOT = Footprint(1.0, 0.2, 0.3)
RES = 0.05
ORIGIN = (-2.0, -2.0)
P0 = (0.0, 0.0, 0.0)
STOP = 0.088


def grid(cells=(), shape=(120, 80), *, stamp=9.0, version=0, unknown=()):
    state = np.full(shape, FREE, dtype=np.uint8)
    free_stamp = np.full(shape, stamp)
    for i, j in unknown:
        state[i, j] = UNKNOWN
        free_stamp[i, j] = np.nan
    for i, j in cells:
        state[i, j] = OCCUPIED
        free_stamp[i, j] = np.nan  # as the grid leaves a marked cell (obstacle_grid: free_stamp[OCCUPIED] = nan)
    return GridSnapshot(state, free_stamp, 10.0, (0.0, 0.0, 0.0), version, 1, {}, ORIGIN[0], ORIGIN[1], RES)


def cell_at(x, y):
    return (int(math.floor((x - ORIGIN[0]) / RES)), int(math.floor((y - ORIGIN[1]) / RES)))


def reverse_path(length=0.4, start=P0):
    s = np.arange(0.0, length + 1e-9, 0.02)
    return np.column_stack((start[0] - s, np.full(len(s), start[1]), np.full(len(s), start[2])))


def region(kappa_cmd=0.0, kappa_meas=0.0, length=0.4):
    return xe.region_poses(reverse_path(length), P0, measured_kappa=kappa_meas, commanded_kappa=kappa_cmd,
                           direction=-1, stop_m=STOP, scan_m=0.018)


# -- the region and the watch -----------------------------------------------------------------------------------


def test_the_region_covers_the_remaining_path_and_both_stop_arcs():
    poses = region(kappa_cmd=-0.36)
    # the path's end, and the commanded arc's end (it turns, the straight does not)
    assert np.min(poses[:, 0]) <= -0.4 + 1e-9
    assert np.max(np.abs(poses[:, 2])) == pytest.approx(0.36 * (STOP + 0.018), abs=1e-6)


def test_a_hit_inside_the_swept_outline_rings_and_one_beside_it_does_not():
    poses = region()
    behind = np.array([[-0.2 - 0.3, 0.0]])  # 0.3 m behind the rear edge, on the path
    beside = np.array([[0.5, 0.3 + 0.13]])  # 0.13 m beside the outline: outside outline + 0.06
    assert list(xe.watch_hits(behind, np.array([False]), poses, FOOT, 0.06)) == [0]
    assert list(xe.watch_hits(beside, np.array([False]), poses, FOOT, 0.06)) == []
    assert list(xe.watch_hits(behind, np.array([True]), poses, FOOT, 0.06)) == []  # its own hit


def test_the_commanded_arc_catches_the_corner_a_straight_sweep_misses():
    # Codex 1st review P1-2: reversing at -0.36 the front corner swings out past the straight sweep + margin
    arc = xe.region_poses(np.asarray([P0]), P0, measured_kappa=0.0, commanded_kappa=-0.36, direction=-1,
                          stop_m=STOP, scan_m=0.018)
    e = arc[np.argmax(np.abs(arc[:, 2]))]
    c, s = math.cos(e[2]), math.sin(e[2])
    corners = [(e[0] + FOOT.front_m * c - side * s, e[1] + FOOT.front_m * s + side * c)
               for side in (FOOT.half_width_m + 0.03, -(FOOT.half_width_m + 0.03))]
    tip = np.array([max(corners, key=lambda q: abs(q[1]))])  # the corner that swings out, 0.03 m beyond it
    assert abs(tip[0, 1]) > FOOT.half_width_m + 0.06  # outside the straight sweep's margin
    assert list(xe.watch_hits(tip, np.array([False]), region(kappa_cmd=0.0), FOOT, 0.06)) == []
    assert list(xe.watch_hits(tip, np.array([False]), region(kappa_cmd=-0.36), FOOT, 0.06)) == [0]


def test_the_corridor_is_every_cell_the_region_plus_margin_overlaps():
    snap = grid()
    cells = xe.corridor_cells(snap, region(), FOOT, 0.06)
    keys = {tuple(c) for c in cells.tolist()}
    assert cell_at(-0.5, 0.0) in keys  # behind, on the path
    assert cell_at(0.5, 0.3 + 0.04) in keys  # beside, within the margin
    assert cell_at(0.5, 0.3 + 0.2) not in keys
    assert len(keys) == len(cells)  # each cell once


# -- the transform ---------------------------------------------------------------------------------------------------


def verified(*, stamp=9.95, version=0, poses=None):
    return xe.VerifiedCorridor(stamp, version, region() if poses is None else poses)


def apply(snap, v, now=10.0):
    return xe.apply_verified(snap, v, FOOT, 0.06, now_s=now, max_age_s=0.2)


def test_only_the_occupied_cells_of_the_verified_corridor_are_read_free_with_the_scan_stamp():
    inside, unknown_inside, outside = cell_at(-0.5, 0.0), cell_at(-0.45, 0.1), cell_at(0.5, 0.8)
    snap = grid([inside, outside], unknown=[unknown_inside])
    out, cleared, skipped = apply(snap, verified())
    assert skipped is None and cleared == 1
    assert out.state[inside] == FREE and out.free_stamp[inside] == 9.95
    assert out.state[unknown_inside] == UNKNOWN and np.isnan(out.free_stamp[unknown_inside])
    assert out.state[outside] == OCCUPIED
    assert snap.state[inside] == OCCUPIED  # the input is left as it was


@pytest.mark.parametrize("ver, stamp, now, reason", [(None, 10.0, 10.0, "unverified"), (1, 10.0, 10.0, "version"),
                                                     (0, 10.0, 10.21, "stale"), (0, float("nan"), 10.0, "invalid"),
                                                     (0, 10.5, 10.0, "invalid"), (0, 10.0, float("nan"), "invalid"),
                                                     (0, float("inf"), 10.0, "invalid")])
def test_no_cell_is_freed_without_a_fresh_verification_of_this_correction(ver, stamp, now, reason):
    # Codex 4th review P2-3: a NaN or future stamp passed the age test and was written as fresh evidence
    inside = cell_at(-0.5, 0.0)
    snap = grid([inside], version=0)
    v = None if ver is None else verified(stamp=stamp, version=ver)
    out, cleared, skipped = apply(snap, v, now)
    assert skipped == reason and cleared == 0
    assert out is snap


def permission(max_age=0.2):
    return DrivePermission(PermissionConfig(StoppingModel(0.15, 1.5, 0.05), 0.01, max_age, envelope_ramp_m=0.0))


def allowed(snap, now):
    p = permission()
    p.update(snap, reverse_path(1.0), FOOT, FOOT, current_pose=P0, direction=-1)
    return p.allowed_speed(now, {}, current_pose=P0, curvature_inv_m=0.0, direction=-1, footprint=FOOT,
                           own_footprint=FOOT, speed_cap_mps=0.18)


def test_a_freed_cell_carries_the_scans_age_into_the_permission():
    # Codex 2nd review P1-1: a marked cell's stamp is NaN, which the walk does not read as old evidence; the freed
    # cell must age like any other free cell
    halo = cell_at(-0.25, 0.0)  # right behind the rear edge: the reverse start reads it
    snap = grid([halo], stamp=10.0)  # every other cell seen free at 10.0
    assert allowed(snap, 10.0) == (0.0, "occupied")
    out, _, _ = apply(snap, verified(stamp=9.95))
    a, why = allowed(out, 10.0)
    assert a > 0 and why == "ok"
    assert allowed(dataclasses.replace(snap, state=np.where(snap.state == OCCUPIED, FREE, snap.state)),
                   10.18)[1] == "ok"  # the rest alone is still fresh at 10.18 ...
    assert allowed(out, 10.18) == (0.0, "evidence_stale")  # ... the freed cell, 0.23 s old, is not


def test_the_freed_cells_follow_the_verified_region_when_the_grid_origin_moves():
    # Codex 3rd review P1: the layer's window origin follows the truck (2 mm moved it from -5.00 to -4.95); the region
    # is kept in the map frame and its cells found on the snapshot at hand, so the same place is freed
    inside, beside = (-0.5, 0.0), (0.5, 0.8)
    for origin in ((-2.0, -2.0), (-1.95, -2.0)):
        shape = (120, 80)
        state = np.full(shape, FREE, dtype=np.uint8)
        stamp = np.full(shape, 10.0)
        cells = {}
        for name, (x, y) in (("inside", inside), ("beside", beside)):
            ij = (int(math.floor((x - origin[0]) / RES)), int(math.floor((y - origin[1]) / RES)))
            state[ij] = OCCUPIED
            stamp[ij] = np.nan
            cells[name] = ij
        snap = GridSnapshot(state, stamp, 10.0, (0.0, 0.0, 0.0), 0, 1, {}, origin[0], origin[1], RES)
        out, cleared, skipped = apply(snap, verified())
        assert skipped is None and cleared == 1
        assert out.state[cells["inside"]] == FREE and out.state[cells["beside"]] == OCCUPIED
