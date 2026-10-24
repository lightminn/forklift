"""Start-band release (docs/plans/2026-10-24-start-band-release.md): an occupied cell the stopped truck's outline is
already within the walk's own start inflation of does not block a motion whose swept outline stays as far from it;
every other cell keeps the walk's full margin; off, the walk is unchanged."""

import dataclasses
import math

import numpy as np
import pytest

from forklift_core.control import start_release as sr
from forklift_core.control.drive_permission import DrivePermission, PermissionConfig, StoppingModel
from forklift_core.control.escape import rectangles_sweep_free, start_band_rectangles, straight
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, GridSnapshot
from forklift_core.planning import Bounds, Footprint, Rectangle
from forklift_core.planning.geometry import FootprintCollisionChecker

FOOT = Footprint(1.0, 0.2, 0.3)
RES = 0.05
ORIGIN = (-2.0, -2.0)


def grid(cells, shape=(120, 80)):
    state = np.full(shape, FREE, dtype=np.uint8)
    for i, j in cells:
        state[i, j] = OCCUPIED
    return GridSnapshot(state, np.full(shape, 9.0), 10.0, (0.0, 0.0, 0.0), 0, 1, {}, ORIGIN[0], ORIGIN[1], RES)


def cell_at(x, y):
    return (int(math.floor((x - ORIGIN[0]) / RES)), int(math.floor((y - ORIGIN[1]) / RES)))


def brute_cell_distance(cell, pose, fp, n=60):
    """Sampled distance between a cell and the footprint (for the exact one's check)."""
    x0, y0 = ORIGIN[0] + cell[0] * RES, ORIGIN[1] + cell[1] * RES
    g = np.linspace(0, RES, n)
    px, py = (a.ravel() for a in np.meshgrid(x0 + g, y0 + g, indexing="ij"))
    return float(sr._points_to_rect(px, py, pose, fp, 0.0, 0.0).min())


@pytest.mark.parametrize("pose", [(0.0, 0.0, 0.0), (0.1, -0.05, 0.3), (-0.2, 0.1, -1.1)])
def test_the_cell_distance_is_the_exact_one(pose):
    snap = grid(set())
    cells = [cell_at(x, y) for x in (-0.6, 0.0, 0.7, 1.3) for y in (-0.6, -0.33, 0.34, 0.8)]
    exact = sr.cell_distances(snap, cells, pose, FOOT)
    brute = np.array([brute_cell_distance(c, pose, FOOT) for c in cells])
    assert np.all(exact <= brute + 1e-9) and np.allclose(exact, brute, atol=RES / 59 * 1.5)


def test_the_footprint_to_rectangle_distance_is_exact_and_zero_on_overlap():
    r = Rectangle(0.5, 0.4, 0.2, 0.1, 0.0)  # y 0.35-0.45, the side at 0.3: 0.05 apart
    assert sr.footprint_rect_distance((0.0, 0.0, 0.0), FOOT, r) == pytest.approx(0.05)
    assert sr.footprint_rect_distance((0.0, 0.1, 0.0), FOOT, r) == 0.0
    rot = Rectangle(1.2, 0.0, 0.2, 0.2, math.pi / 4)  # its corner toward the front face at 1.0
    assert sr.footprint_rect_distance((0.0, 0.0, 0.0), FOOT, rot) == pytest.approx(1.2 - 0.1 * math.sqrt(2) - 1.0)


# The truck stands 0.015 m right of the grid line y = 0.30: a short wall of cells from that line beside the front half
# of its outline is 0.015 m off its side (within the 0.02 band)
P0 = (0.0, -0.015, 0.0)
SIDE = {cell_at(x, 0.32) for x in np.arange(0.5, 1.0, RES)}


def perm(on: bool):
    cfg = PermissionConfig(StoppingModel(0.15, 1.5, 0.05), envelope_offset_m=0.01, evidence_max_age_s=5.0,
                           envelope_ramp_m=0.0)
    return DrivePermission(dataclasses.replace(cfg, start_band_m=0.02 if on else 0.0))


def path(direction, length=0.6, kappa=0.0):
    s = np.linspace(0.0, length, 31)
    if kappa == 0.0:
        return np.column_stack((direction * s, np.full_like(s, P0[1]), np.zeros_like(s)))
    yaw = direction * s * kappa
    return np.column_stack((np.sin(yaw) / kappa, P0[1] + (1 - np.cos(yaw)) / kappa, yaw))


def test_the_band_holds_the_cells_within_the_start_inflation_only():
    snap = grid(SIDE | {cell_at(0.5, 0.37)})  # one more, 0.065 m off: not in the band
    cells, d0 = sr.start_band_cells(snap, P0, FOOT, 0.02)
    assert {tuple(c) for c in cells} == SIDE and np.all((d0 > 0) & (d0 <= 0.02))


def test_off_the_stopped_truck_is_held_in_both_directions_and_on_it_backs_away():
    snap = grid(SIDE)
    off, on = perm(False), perm(True)
    for d in (-1, 1):
        assert off.check_path(snap, path(d), FOOT, FOOT, current_pose=P0, direction=d).verified_m == 0.0
    back = on.check_path(snap, path(-1), FOOT, FOOT, current_pose=P0, direction=-1)
    assert back.blocked is None and back.verified_m == pytest.approx(0.6)
    arc = on.check_arc(snap, P0, 0.0, -1, FOOT, FOOT, 0.15)
    assert arc.blocked is None  # the stop arc too: the same band rule, the same stopping model


def test_a_turn_toward_the_cells_is_stopped_where_the_outline_first_comes_nearer():
    snap = grid(SIDE)
    toward = perm(True).check_path(snap, path(1, 0.6, kappa=0.5), FOOT, FOOT, current_pose=P0, direction=1)
    assert toward.blocked == "occupied" and toward.verified_m < 0.05
    away = perm(True).check_path(snap, path(1, 0.6, kappa=-0.5), FOOT, FOOT, current_pose=P0, direction=1)
    assert away.blocked is None


def test_a_cell_outside_the_band_keeps_the_full_margin():
    far = {cell_at(x, 0.37) for x in np.arange(0.5, 1.0, RES)}  # 0.065 m off: beyond the band
    snap = grid(far)
    for d in (-1, 1):
        a = perm(False).check_path(snap, path(d), FOOT, FOOT, current_pose=P0, direction=d)
        b = perm(True).check_path(snap, path(d), FOOT, FOOT, current_pose=P0, direction=d)
        assert (a.verified_m, a.blocked, a.blocked_cells) == (b.verified_m, b.blocked, b.blocked_cells)


def test_a_new_cell_met_on_the_way_still_blocks_with_its_margin():
    ahead = cell_at(-0.6, 0.0)  # behind the rear face, met reversing
    snap = grid(SIDE | {ahead})
    back = perm(True).check_path(snap, path(-1), FOOT, FOOT, current_pose=P0, direction=-1)
    assert back.blocked == "occupied" and ahead in back.blocked_cells and 0.0 < back.verified_m < 0.4


def test_off_the_walk_is_the_one_before():
    snap = grid(SIDE | {cell_at(0.2, -0.33)})
    a = DrivePermission(PermissionConfig(StoppingModel(0.15, 1.5, 0.05), 0.01, 5.0, envelope_ramp_m=0.0))
    for d in (-1, 1):
        x = a.check_path(snap, path(d), FOOT, FOOT, current_pose=P0, direction=d)
        y = perm(False).check_path(snap, path(d), FOOT, FOOT, current_pose=P0, direction=d)
        assert x == y


def test_the_release_lengths_follow_the_continuous_sweep_corners_included():
    # rotating in place toward a cell off the front corner: the corner sweeps nearer at once
    corner = cell_at(1.02, 0.32)  # [1.00, 1.05] x [0.30, 0.35]; the front-left corner at (0.985, 0.285)
    snap = grid({corner})
    pose = (-0.015, -0.015, 0.0)
    cells, d0 = sr.start_band_cells(snap, pose, FOOT, 0.03)
    assert len(cells) == 1 and d0[0] == pytest.approx(math.hypot(0.015, 0.015))
    turn = np.column_stack((np.full(21, pose[0]), np.full(21, pose[1]), np.linspace(0.0, 0.2, 21)))
    arc = np.r_[0.0, np.cumsum(np.abs(np.diff(turn[:, 2])) * 1.0)]
    s_star = sr.release_lengths(snap, cells, d0, turn, arc, FOOT)
    assert np.isfinite(s_star[0]) and s_star[0] < 0.01


def test_the_dense_error_bound():
    assert sr.dense_error_m(1.6) == pytest.approx(0.001 + 1.6 * 0.0005)


# ---- the escape's rectangles ----

def test_the_escape_releases_only_a_trigger_rectangle_the_start_nearly_touches():
    near = Rectangle(0.75, 0.33, 0.5, 0.05, 0.0)  # y 0.305-0.355: 0.005 m off the side
    far = Rectangle(0.75, 0.6, 0.5, 0.05, 0.0)
    over = Rectangle(0.75, 0.29, 0.5, 0.05, 0.0)  # overlaps the outline
    n, f = start_band_rectangles([near, far, over], FOOT, (0.0, 0.0, 0.0))
    assert n == [near] and f == [far, over]


def test_the_escape_sweep_passes_moving_away_and_not_toward():
    near = Rectangle(1.25, 0.0, 0.1, 1.0, 0.0)  # 0.2 m wide band 0.005 m ahead of the front face
    near = Rectangle(1.055, 0.0, 0.1, 1.0, 0.0)
    bounds = Bounds(-5, 5, -5, 5)
    base = FootprintCollisionChecker([], FOOT, bounds)
    whole = FootprintCollisionChecker([near], FOOT, bounds)
    back, fwd = straight((0.0, 0.0, 0.0), -1, 0.4), straight((0.0, 0.0, 0.0), 1, 0.4)
    assert not rectangles_sweep_free(whole, back)  # without the release: the start sample itself
    assert rectangles_sweep_free(base, back, [near], FOOT)
    assert not rectangles_sweep_free(base, fwd, [near], FOOT)
