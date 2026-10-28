"""Start-band release (docs/plans/2026-10-24-start-band-release.md): the permission latches an anchor where the truck
first stands; an occupied cell the anchor's outline is within the band of does not block a motion whose swept outline
never comes nearer to it than it has been since; every other cell keeps the walk's full margin; off, the walk is
unchanged."""

import dataclasses
import math

import numpy as np
import pytest

from forklift_core.control import start_release as sr
from forklift_core.control.drive_permission import DrivePermission, PermissionConfig, StoppingModel
from forklift_core.control.escape import rectangles_sweep_free, start_band_rectangles, straight
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN, GridSnapshot
from forklift_core.planning import Bounds, Footprint, Rectangle
from forklift_core.planning.geometry import FootprintCollisionChecker

FOOT = Footprint(1.0, 0.2, 0.3)
RES = 0.05
ORIGIN = (-2.0, -2.0)


def grid(cells, shape=(120, 80), *, stamp=9.0, version=0, unknown=()):
    state = np.full(shape, FREE, dtype=np.uint8)
    for i, j in unknown:
        state[i, j] = UNKNOWN
    for i, j in cells:
        state[i, j] = OCCUPIED
    return GridSnapshot(state, np.full(shape, stamp), 10.0, (0.0, 0.0, 0.0), version, 1, {}, ORIGIN[0], ORIGIN[1], RES)


def cell_at(x, y):
    return (int(math.floor((x - ORIGIN[0]) / RES)), int(math.floor((y - ORIGIN[1]) / RES)))


def brute_cell_distance(cell, pose, fp, n=60):
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
    r = Rectangle(0.5, 0.4, 0.2, 0.1, 0.0)
    assert sr.footprint_rect_distance((0.0, 0.0, 0.0), FOOT, r) == pytest.approx(0.05)
    assert sr.footprint_rect_distance((0.0, 0.1, 0.0), FOOT, r) == 0.0
    rot = Rectangle(1.2, 0.0, 0.2, 0.2, math.pi / 4)
    assert sr.footprint_rect_distance((0.0, 0.0, 0.0), FOOT, rot) == pytest.approx(1.2 - 0.1 * math.sqrt(2) - 1.0)


# The truck stands 0.015 m right of the grid line y = 0.30; a short wall of cells from that line beside the front half
# of its outline is 0.015 m off its side (within the 0.02 band)
P0 = (0.0, -0.015, 0.0)
SIDE = {cell_at(x, 0.32) for x in np.arange(0.5, 1.0, RES)}


def perm(on: bool, *, standing_at=P0, version=0):
    cfg = PermissionConfig(StoppingModel(0.15, 1.5, 0.05), envelope_offset_m=0.01, evidence_max_age_s=5.0,
                           envelope_ramp_m=0.0)
    p = DrivePermission(dataclasses.replace(cfg, start_band_m=0.02 if on else 0.0))
    if standing_at is not None:
        p.note_standing(True, standing_at, version)
    return p


def path(direction, length=0.6, kappa=0.0, start=P0):
    s = np.linspace(0.0, length, 31)
    if kappa == 0.0:
        return np.column_stack((start[0] + direction * s, np.full_like(s, start[1]), np.zeros_like(s)))
    yaw = direction * s * kappa
    return np.column_stack((start[0] + np.sin(yaw) / kappa, start[1] + (1 - np.cos(yaw)) / kappa, yaw))


def test_the_band_holds_the_cells_within_it_and_no_nearer_than_the_discretisation_floor():
    snap = grid(SIDE | {cell_at(0.5, 0.37)})
    cells, d0 = sr.start_band_cells(snap, P0, FOOT, 0.02)
    assert {tuple(c) for c in cells} == SIDE and np.all(d0.min(axis=1) <= 0.02)
    # Codex counterexample: a start distance of half a micrometre is never released
    tiny = (0.0, 0.3 - 0.3 - 5e-7, 0.0)  # the side at 0.2999995
    assert not len(sr.start_band_cells(grid(SIDE), tiny, FOOT, 0.02)[0])
    assert sr.min_d0_m(FOOT) == pytest.approx(2 * sr.dense_error_m(sr.shape_radius(FOOT)))


def test_off_the_stopped_truck_is_held_in_both_directions_and_on_it_backs_away():
    snap = grid(SIDE)
    off, on = perm(False), perm(True)
    for d in (-1, 1):
        assert off.check_path(snap, path(d), FOOT, FOOT, current_pose=P0, direction=d).verified_m == 0.0
    back = on.check_path(snap, path(-1), FOOT, FOOT, current_pose=P0, direction=-1)
    assert back.blocked is None and back.verified_m == pytest.approx(0.6)
    assert on.check_arc(snap, P0, 0.0, -1, FOOT, FOOT, 0.15).blocked is None  # the stop arc's walk too


def test_without_a_standing_anchor_nothing_is_released():
    # a moving truck meeting a mark within the band: the walk's own rule (the release is for a truck that stood)
    snap = grid(SIDE)
    moving = perm(True, standing_at=None)
    assert moving.check_path(snap, path(-1), FOOT, FOOT, current_pose=P0, direction=-1).verified_m == 0.0


def test_a_drift_toward_the_cells_since_the_anchor_is_held_at_once():
    # agy, Codex P2-3: the start distance is the anchor's, never the current pose's -- no ratchet
    snap = grid(SIDE)
    p = perm(True)
    nearer = (0.0, -0.012, 0.0)  # 3 mm toward the wall since it stood
    c = p.check_path(snap, path(-1, start=nearer), FOOT, FOOT, current_pose=nearer, direction=-1)
    assert c.blocked == "occupied" and c.verified_m == 0.0


def test_the_floor_rises_with_the_distance_reached_and_never_comes_down():
    snap = grid(SIDE)
    p = perm(True)
    away = (0.0, -0.018, 0.0)  # 3 mm farther: the best rises to 0.018 (still within the band, the anchor stays)
    assert p.check_path(snap, path(-1, start=away), FOOT, FOOT, current_pose=away, direction=-1).blocked is None
    assert p.release_anchor is not None
    back_in = (0.0, -0.016, 0.0)  # nearer than the best again (but farther than the anchor's 0.015)
    c = p.check_path(snap, path(-1, start=back_in), FOOT, FOOT, current_pose=back_in, direction=-1)
    assert c.blocked == "occupied" and c.verified_m == 0.0 and p.release_anchor is not None


def test_along_the_sweep_away_then_toward_is_cut_where_it_turns_back():
    snap = grid(SIDE)
    s = np.linspace(0.0, 0.2, 41)
    y = P0[1] - 0.004 * np.sin(s / 0.2 * math.pi)  # 4 mm away, then back
    wiggle = np.column_stack((-s, y, np.zeros_like(s)))
    c = perm(True).check_path(snap, wiggle, FOOT, FOOT, current_pose=P0, direction=-1)
    assert c.blocked == "occupied" and 0.0 < c.verified_m < 0.15


def test_a_turn_toward_the_cells_is_cut_where_the_outline_first_comes_nearer():
    snap = grid(SIDE)
    toward = perm(True).check_path(snap, path(1, 0.6, kappa=0.5), FOOT, FOOT, current_pose=P0, direction=1)
    assert toward.blocked == "occupied" and toward.verified_m < 0.05
    away = perm(True).check_path(snap, path(1, 0.6, kappa=-0.5), FOOT, FOOT, current_pose=P0, direction=1)
    assert away.blocked is None


def test_a_cell_outside_the_band_keeps_the_full_margin():
    snap = grid({cell_at(x, 0.37) for x in np.arange(0.5, 1.0, RES)})
    for d in (-1, 1):
        a = perm(False).check_path(snap, path(d), FOOT, FOOT, current_pose=P0, direction=d)
        b = perm(True).check_path(snap, path(d), FOOT, FOOT, current_pose=P0, direction=d)
        assert (a.verified_m, a.blocked, a.blocked_cells) == (b.verified_m, b.blocked, b.blocked_cells)


def test_a_new_cell_met_on_the_way_still_blocks_with_its_margin():
    ahead = cell_at(-0.6, -0.015)
    snap = grid(SIDE | {ahead})
    back = perm(True).check_path(snap, path(-1), FOOT, FOOT, current_pose=P0, direction=-1)
    assert back.blocked == "occupied" and ahead in back.blocked_cells and 0.0 < back.verified_m < 0.4


def test_a_correction_change_or_moving_away_drops_the_anchor():
    p = perm(True)
    c = p.check_path(grid(SIDE, version=1), path(-1), FOOT, FOOT, current_pose=P0, direction=-1)
    assert c.verified_m == 0.0 and p.release_anchor is None
    p = perm(True)
    far = (-0.5, -0.06, 0.0)  # every start-band cell beyond the band now
    p.check_path(grid(SIDE), path(-1, start=far), FOOT, FOOT, current_pose=far, direction=-1)
    assert p.release_anchor is None


def test_a_new_standing_latches_a_new_anchor_only_after_moving():
    p = perm(True)
    p.note_standing(True, (0.3, 0.0, 0.0), 0)  # still standing: the first anchor stays
    assert p.release_anchor["pose"] == P0
    p.note_standing(False, (0.3, 0.0, 0.0), 0)
    p.note_standing(True, (0.3, 0.0, 0.0), 0)
    assert p.release_anchor["pose"] == (0.3, 0.0, 0.0)


def test_the_cut_walk_keeps_only_the_evidence_and_the_unknowns_of_what_it_walked():
    # Codex P2-4: an old FREE beyond the cut does not make the cut walk stale; the cut is the walk's own end
    snap = grid(SIDE)
    stamp = np.array(snap.free_stamp, copy=True)
    for i in range(cell_at(2.0, 0)[0], snap.state.shape[0]):
        stamp[i, :] = 0.0  # old evidence at x >= 2 m only
    snap = dataclasses.replace(snap, free_stamp=stamp)
    toward = perm(True).check_path(snap, path(1, 1.5, kappa=0.5), FOOT, FOOT, current_pose=P0, direction=1)
    assert toward.blocked == "occupied" and toward.oldest_free_s > 0.0
    # the released cells are never evidence
    assert np.isinf(sr.released(grid(SIDE), np.array(sorted(SIDE))).free_stamp[sorted(SIDE)[0]])


def test_off_the_walk_is_the_one_before():
    snap = grid(SIDE | {cell_at(0.2, -0.33)})
    a = DrivePermission(PermissionConfig(StoppingModel(0.15, 1.5, 0.05), 0.01, 5.0, envelope_ramp_m=0.0))
    for d in (-1, 1):
        x = a.check_path(snap, path(d), FOOT, FOOT, current_pose=P0, direction=d)
        y = perm(False).check_path(snap, path(d), FOOT, FOOT, current_pose=P0, direction=d)
        assert x == y


def test_the_dense_error_bound_and_the_corner_sweep_in_rotation():
    assert sr.dense_error_m(1.6) == pytest.approx(0.001 + 1.6 * 0.0005 + sr.SAGITTA_M)
    corner = cell_at(1.02, 0.32)
    snap = grid({corner})
    pose = (-0.015, -0.015, 0.0)
    cells, d0 = sr.start_band_cells(snap, pose, FOOT, 0.03)
    assert len(cells) == 1 and d0.min() == pytest.approx(math.hypot(0.015, 0.015))
    turn = np.column_stack((np.full(21, pose[0]), np.full(21, pose[1]), np.linspace(0.0, 0.2, 21)))
    arc = np.r_[0.0, np.cumsum(np.abs(np.diff(turn[:, 2])))]
    first, hit = sr.release_length(snap, cells, d0, np.full_like(d0, 0.03), turn, arc, FOOT)
    assert np.isfinite(first) and first < 0.01 and len(hit) == 1


# ---- the escape's rectangles ----

def test_the_escape_releases_only_a_trigger_rectangle_the_start_pad_meets_but_does_not_overlap():
    near = Rectangle(0.75, 0.33, 0.5, 0.05, 0.0)  # 0.005 m off the side
    far = Rectangle(0.75, 0.6, 0.5, 0.05, 0.0)
    over = Rectangle(0.75, 0.29, 0.5, 0.05, 0.0)
    corner = Rectangle(1.0 + 0.009 + 0.025, 0.3 + 0.009 + 0.025, 0.05, 0.05, 0.0)  # 9 mm per axis off the corner
    n, f = start_band_rectangles([near, far, over, corner], FOOT, (0.0, 0.0, 0.0))
    assert n == [near, corner] and f == [far, over]  # Codex P3: the pad's own per-axis test, not a Euclidean one


def test_the_escape_sweep_passes_moving_away_and_not_toward():
    near = Rectangle(1.055, 0.0, 0.1, 1.0, 0.0)
    bounds = Bounds(-5, 5, -5, 5)
    base = FootprintCollisionChecker([], FOOT, bounds)
    whole = FootprintCollisionChecker([near], FOOT, bounds)
    back, fwd = straight((0.0, 0.0, 0.0), -1, 0.4), straight((0.0, 0.0, 0.0), 1, 0.4)
    assert not rectangles_sweep_free(whole, back)
    assert rectangles_sweep_free(base, back, [near], FOOT)
    assert not rectangles_sweep_free(base, fwd, [near], FOOT)
    # away then back toward (Codex P2-3: never nearer than reached, not only than the start)
    out_and_in = np.vstack([straight((0.0, 0.0, 0.0), -1, 0.03), straight((-0.03, 0.0, 0.0), 1, 0.02)[1:]])
    assert not rectangles_sweep_free(base, out_and_in, [near], FOOT)


# ---- Codex v2 review ----

def shifted(cells, di):
    """The same physical cells on a grid whose origin moved di cells along x (a rolling grid)."""
    state = np.full((120, 80), FREE, dtype=np.uint8)
    for i, j in cells:
        state[i - di, j] = OCCUPIED
    return GridSnapshot(state, np.full((120, 80), 9.0), 10.0, (0.0, 0.0, 0.0), 0, 1, {},
                        ORIGIN[0] + di * RES, ORIGIN[1], RES)


def test_a_cell_keeps_its_history_when_the_rolling_grid_moves():
    p = perm(True)
    away = (0.0, -0.018, 0.0)
    assert p.check_path(grid(SIDE), path(-1, start=away), FOOT, FOOT, current_pose=away, direction=-1).blocked is None
    back_in = (0.0, -0.016, 0.0)
    c = p.check_path(shifted(SIDE, 1), path(-1, start=back_in), FOOT, FOOT, current_pose=back_in, direction=-1)
    assert c.blocked == "occupied" and c.verified_m == 0.0  # the 0.018 reached is remembered across the move


PARTS = [(Footprint(0.3, 0.2, 0.3), 0.0, 0.0), (Footprint(0.7, 0.0, 0.05), 0.25, 0.3),
         (Footprint(0.7, 0.0, 0.05), -0.25, 0.3)]


def test_switching_between_shapes_keeps_their_histories_apart():
    snap = grid(SIDE)
    p = perm(True)
    p.check_path(snap, path(-1), PARTS, PARTS, current_pose=P0, direction=-1)  # body + blades reversing
    c = p.check_path(snap, path(1), FOOT, FOOT, current_pose=P0, direction=1)  # the hull forward: no shape clash
    assert c.blocked in (None, "occupied")
    assert {k[0] for k in p.release_anchor["best"]} == {sr.history_keys(snap, [(0, 0)], PARTS)[0][0],
                                                         sr.history_keys(snap, [(0, 0)], FOOT)[0][0]}


def test_a_standing_truck_re_latches_after_a_correction_dropped_its_anchor():
    p = perm(True)
    assert p.check_path(grid(SIDE), path(-1), FOOT, FOOT, current_pose=P0, direction=-1).blocked is None
    assert p.check_path(grid(SIDE, version=1), path(-1), FOOT, FOOT, current_pose=P0, direction=-1).verified_m == 0.0
    assert p.release_anchor is None
    p.note_standing(True, P0, 1)  # still standing, the scene replan's release raised the version
    assert p.release_anchor is not None and p.release_anchor["version"] == 1
    back = p.check_path(grid(SIDE, version=1), path(-1), FOOT, FOOT, current_pose=P0, direction=-1)
    assert back.blocked is None


def test_the_release_counts_the_walks_it_passed_and_the_ones_it_cut():
    snap = grid(SIDE)
    on, off = perm(True), perm(False)
    on.check_path(snap, path(-1), FOOT, FOOT, current_pose=P0, direction=-1)
    on.check_path(snap, path(1, 0.6, kappa=0.5), FOOT, FOOT, current_pose=P0, direction=1)  # turns toward: cut
    off.check_path(snap, path(-1), FOOT, FOOT, current_pose=P0, direction=-1)
    assert on.release_stats == {"checks": 2, "passed": 1, "capped": 1}
    assert off.release_stats == {"checks": 0, "passed": 0, "capped": 0}


# ---- a fixed approach budget (docs/plans/2026-10-27-escape-presteer.md, C4'' -- the lead's (a')) ----

AHEAD = cell_at(1.06, 0.0)  # the cell [1.05, 1.10) x: 12 mm ahead of the front face at the anchor below
ANCHOR = (0.038, 0.0, 0.0)


def _toward(approach_m, n=11, start=ANCHOR):
    """Poses moving forward by approach_m from start toward AHEAD."""
    xs = start[0] + np.linspace(0.0, approach_m, n)
    return np.column_stack((xs, np.zeros(n), np.zeros(n))), xs - start[0]


def _band():
    snap = grid({AHEAD})
    cells, d0 = sr.start_band_cells(snap, ANCHOR, FOOT, 0.03)
    assert len(cells) == 1 and d0.min() == pytest.approx(0.012, abs=1e-6)
    return snap, cells, d0, np.full_like(d0, 0.03)


def test_without_a_budget_the_walk_is_cut_at_the_first_approach():
    snap, cells, d0, cap = _band()
    poses, arc = _toward(0.0004)
    first, _ = sr.release_length(snap, cells, d0, cap, poses, arc, FOOT)
    assert np.isfinite(first) and first < 0.0004


def test_a_budget_lets_the_outline_come_that_much_nearer_and_no_more():
    snap, cells, d0, cap = _band()
    poses, arc = _toward(0.0015)  # 1.5 mm nearer: within a 2 mm budget
    assert sr.release_length(snap, cells, d0, cap, poses, arc, FOOT, budget_m=0.002)[0] == np.inf
    poses, arc = _toward(0.004, n=41)  # 4 mm: cut past the budget
    first, _ = sr.release_length(snap, cells, d0, cap, poses, arc, FOOT, budget_m=0.002)
    assert np.isfinite(first) and 0.0015 <= first <= 0.0021


def test_the_budget_counts_from_the_farthest_the_outline_has_been_and_never_goes_below_min_d0():
    snap, cells, d0, cap = _band()
    # back 5 mm then forward 6 mm: 1 mm nearer than the anchor but 6 mm nearer than the farthest -- cut
    xs = ANCHOR[0] + np.r_[np.linspace(0.0, -0.005, 11), np.linspace(-0.005, 0.001, 13)[1:]]
    poses = np.column_stack((xs, np.zeros_like(xs), np.zeros_like(xs)))
    arc = np.r_[0.0, np.cumsum(np.abs(np.diff(xs)))]
    first, _ = sr.release_length(snap, cells, d0, cap, poses, arc, FOOT, budget_m=0.002)
    assert np.isfinite(first)
    # 4.5 mm ahead: a 2 mm budget would allow 2.5 mm, below min_d0 (3.24 mm here) -- min_d0 holds
    anchor2 = (0.0455, 0.0, 0.0)
    cells2, d02 = sr.start_band_cells(snap, anchor2, FOOT, 0.03)
    floor_min = sr.min_d0_m(FOOT)
    assert d02.min() - 0.002 < floor_min < d02.min()
    poses, arc = _toward(0.004, n=81, start=anchor2)
    first2, _ = sr.release_length(snap, cells2, d02, np.full_like(d02, 0.03), poses, arc, FOOT, budget_m=0.002,
                                  floor_min_m=floor_min)
    assert np.isfinite(first2)
    reached = float(sr.part_distances(snap, cells2, [tuple(poses[np.searchsorted(arc, first2)])], FOOT).min())
    assert reached >= floor_min - 1e-6


def test_the_permission_passes_the_budget_from_its_config():
    snap = grid({AHEAD}, version=0)
    base = PermissionConfig(StoppingModel(0.0, 1.5, 0.0), 0.0, 5.0, envelope_ramp_m=0.0, start_band_m=0.03,
                            lookahead_m=0.3)
    moved = (ANCHOR[0] + 0.0004, 0.0, 0.0)  # crept 0.4 mm toward the cell while standing
    path_ = np.column_stack((np.linspace(moved[0], moved[0] - 0.3, 31), np.zeros(31), np.zeros(31)))
    results = {}
    for budget in (0.0, 0.002):
        p = DrivePermission(dataclasses.replace(base, start_band_approach_m=budget))
        p.note_standing(True, ANCHOR, 0)
        p.note_standing(False, ANCHOR, 0)
        results[budget] = p.check_path(snap, path_, FOOT, FOOT, current_pose=moved, direction=-1).blocked
    assert results[0.0] == "occupied" and results[0.002] is None
