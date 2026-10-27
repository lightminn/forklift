"""The global matcher (docs/plans/2026-10-27-autonomous-survey-and-relocalization.md B1-B2): multi-resolution
correlative search with branch and bound over every free cell the laser may stand in, competitors kept beyond the
wrong-confirmation tolerance, refinement of every kept candidate, and the confirmation rule."""

import math

import numpy as np
import pytest

from forklift_core.localization import global_match as G
from forklift_core.localization.slam_map import Grid

RES = 0.05


def _grid(width_m, height_m, boxes):
    """A survey-like grid: free inside, occupied walls one cell thick around, plus filled ``boxes`` (x0, y0, x1, y1)."""
    w, h = int(round(width_m / RES)), int(round(height_m / RES))
    data = np.zeros((h, w), np.int8)
    data[0, :] = data[-1, :] = 100
    data[:, 0] = data[:, -1] = 100
    for x0, y0, x1, y1 in boxes:
        data[int(y0 / RES):int(math.ceil(y1 / RES)), int(x0 / RES):int(math.ceil(x1 / RES))] = 100
    return Grid(data, (0.0, 0.0, 0.0), RES)


def _scan(grid, pose, beams=360, range_max=12.0, step=0.01):
    """Ranges a 2D LiDAR at ``pose`` reads off the grid's occupied cells (ray marching)."""
    occ = grid.occupied()
    angles = pose[2] + np.arange(beams) * 2 * math.pi / beams
    t = np.arange(0.05, range_max, step)
    xs = pose[0] + np.cos(angles)[:, None] * t
    ys = pose[1] + np.sin(angles)[:, None] * t
    rows, cols, inside = grid.cells_of(np.stack([xs.ravel(), ys.ravel()], 1))
    hit = (inside & occ[rows, cols]).reshape(beams, -1)
    first = np.where(hit.any(1), hit.argmax(1), -1)
    return np.where(first >= 0, t[np.maximum(first, 0)], np.inf), 0.0, 2 * math.pi / beams


def _points(grid, pose, **kw):
    ranges, a0, inc = _scan(grid, pose, **kw)
    return G.scan_points(ranges, a0, inc, range_min_m=0.2, range_max_m=12.0)


ROOM = (6.0, 4.0, [(4.0, 2.6, 4.6, 3.95), (0.05, 0.05, 1.0, 0.5)])  # two boxes break the symmetry


def _close(pose, truth, m=0.10, rad=0.05):
    d = math.atan2(math.sin(pose[2] - truth[2]), math.cos(pose[2] - truth[2]))
    return math.hypot(pose[0] - truth[0], pose[1] - truth[1]) <= m and abs(d) <= rad


def test_an_asymmetric_room_is_found_and_confirmed():
    grid = _grid(*ROOM)
    truth = (2.3, 1.7, 0.6)
    match = G.refine(grid, G.global_match(grid, _points(grid, truth)), _points(grid, truth))
    assert match.status == "ok" and _close(match.candidates[0].pose, truth)
    assert G.decide(match, e_min=0.8, r_max=0.9).confirmed


def test_the_search_is_exact_against_brute_force():
    # branch and bound returns the best lattice score over every candidate cell and heading
    grid = _grid(3.0, 2.0, [(2.0, 1.2, 2.5, 1.95)])
    cfg = G.MatchConfig(yaw_step_rad=math.radians(5.0), clearance_m=0.2)
    pts = _points(grid, (1.0, 0.8, 0.3))
    match = G.global_match(grid, pts, cfg)
    best = G.brute_force_best(grid, pts, cfg)
    assert match.candidates[0].score == pytest.approx(best, abs=1e-12)


def test_a_symmetric_room_keeps_the_turned_competitor_and_is_not_confirmed():
    grid = _grid(6.0, 4.0, [])
    truth = (2.0, 1.4, 0.0)
    pts = _points(grid, truth)
    # an empty box overflows a small frontier: no answer at all, never a confirmation
    assert G.global_match(grid, pts, G.MatchConfig(max_nodes=400_000)).status == "overflow"
    cfg = G.MatchConfig(max_nodes=5_000_000, yaw_step_rad=math.radians(2.0))
    match = G.refine(grid, G.global_match(grid, pts, cfg), pts, cfg)
    assert any(_close(c.pose, truth) for c in match.candidates)
    mirror = (6.0 - truth[0], 4.0 - truth[1], math.pi)
    assert any(_close(c.pose, mirror) for c in match.candidates)
    d = G.decide(match, e_min=0.8, r_max=0.9)
    assert not d.confirmed and d.reason == "competitor"


def test_a_lone_wall_is_not_confirmed():
    grid = _grid(12.0, 4.0, [])
    pts = _points(grid, (6.0, 0.6, math.pi / 2), range_max=12.0)
    pts = pts[pts[:, 0] < -0.3]  # keep only the wall behind-right: a single straight wall in view
    d = G.decide(G.refine(grid, G.global_match(grid, pts), pts), e_min=0.8, r_max=0.9)
    assert not d.confirmed


def test_an_observation_without_information_is_refused_never_nan():
    grid = _grid(*ROOM)
    empty = G.scan_points(np.full(360, np.inf), 0.0, 2 * math.pi / 360, range_min_m=0.2, range_max_m=12.0)
    match = G.global_match(grid, empty)
    assert match.status == "uninformative" and not match.candidates
    d = G.decide(match, e_min=0.0, r_max=1.0)
    assert not d.confirmed and d.reason == "uninformative"


def test_points_beyond_the_match_range_are_dropped():
    ranges = np.array([1.0, 7.9, 8.1, 11.0, np.inf, -np.inf])
    pts = G.scan_points(ranges, 0.0, 0.1, range_min_m=0.2, range_max_m=12.0, match_range_m=8.0)
    assert len(pts) == 2


def test_competitors_are_suppressed_only_inside_both_tolerances():
    # a near tie 0.7 m away survives (merging at 1 m would lose it); within 0.3 m and 0.1 rad it is suppressed
    cands = [G.Candidate((0.0, 0.0, 0.0), 1.0, 1.0), G.Candidate((0.2, 0.0, 0.05), 0.99, 1.0),
             G.Candidate((0.7, 0.0, 0.0), 0.98, 1.0), G.Candidate((0.1, 0.0, 0.2), 0.97, 1.0)]
    kept = G.suppress(cands, sep_m=0.3, sep_rad=0.1)
    assert [c.pose for c in kept] == [(0.0, 0.0, 0.0), (0.7, 0.0, 0.0), (0.1, 0.0, 0.2)]


def test_evenly_spaced_competitors_are_a_repeated_structure():
    top = G.Candidate((0.0, 0.0, 0.0), 1.0, 0.95)
    row = [G.Candidate((2.0, 0.05, 0.0), 0.8, 0.9), G.Candidate((-2.0, -0.02, 0.0), 0.78, 0.9),
           G.Candidate((4.05, 0.0, 0.0), 0.7, 0.9)]
    assert G.periodic([top] + row, ratio_min=0.6)
    scattered = [G.Candidate((2.0, 1.3, 0.0), 0.8, 0.9), G.Candidate((-0.4, 3.1, 0.0), 0.78, 0.9)]
    assert not G.periodic([top] + scattered, ratio_min=0.6)
    d = G.decide(G.GlobalMatch("ok", (top, *row)), e_min=0.8, r_max=0.85, r_struct=0.6)
    assert not d.confirmed and d.reason == "periodic"


def test_the_laser_stands_only_where_the_clearance_holds():
    grid = _grid(*ROOM)
    mask = G.candidate_cells(grid, clearance_m=0.3)
    rows, cols, _ = grid.cells_of(np.array([[0.2, 2.0], [3.0, 2.0], [4.3, 2.5]]))
    assert not mask[rows[0], cols[0]] and mask[rows[1], cols[1]] and not mask[rows[2], cols[2]]


def test_stationary_scans_stack_in_the_first_laser_frame():
    grid = _grid(*ROOM)
    r, a0, inc = _scan(grid, (2.3, 1.7, 0.6))
    one = G.scan_points(r, a0, inc, range_min_m=0.2, range_max_m=12.0, cell_m=0.05)
    stacked = G.stack_scans([((0.0, 0.0, 0.0), r, a0, inc), ((0.0, 0.0, 0.0), r, a0, inc)], range_min_m=0.2,
                            range_max_m=12.0)
    assert len(stacked) == len(one)  # identical scans fall in the same 5 cm voxels


def test_the_competitor_ratio_is_the_lattice_one_and_a_wandering_top_is_refused():
    # Codex B4a re-review P2: refinement places the pose; it neither re-ranks nor hides competitors
    top = G.Candidate((1.0, 1.0, 0.0), 0.99, 0.95, True, (1.02, 1.0, 0.0), 0.80)
    rival = G.Candidate((4.0, 1.0, 0.0), 0.40, 0.90, True, (4.0, 1.0, 0.0), 0.76)
    d = G.decide(G.GlobalMatch("ok", (top, rival)), e_min=0.8, r_max=0.96)
    assert d.confirmed and d.ratio == pytest.approx(0.76 / 0.80)  # 0.95 on the lattice, though 0.40 / 0.99 refined
    assert not G.decide(G.GlobalMatch("ok", (top, rival)), e_min=0.8, r_max=0.94).confirmed
    walked = G.Candidate((1.4, 1.0, 0.0), 0.99, 0.95, True, (1.0, 1.0, 0.0), 0.80)
    d = G.decide(G.GlobalMatch("ok", (walked,)), e_min=0.8, r_max=0.9)
    assert not d.confirmed and d.reason == "refine_moved"


def test_refinement_keeps_the_lattice_pose_and_score_through_the_real_path():
    # Codex B4a round 3 P2: refine() must carry the lattice fields to decide(), which reads the ratio and the move
    grid = _grid(*ROOM)
    truth = (2.3, 1.7, 0.6)
    pts = _points(grid, truth)
    lattice = G.global_match(grid, pts)
    refined = G.refine(grid, lattice, pts)
    for a, b in zip(lattice.candidates, refined.candidates, strict=True):
        assert b.lattice_pose == a.pose and b.lattice_score == a.score
    # a candidate placed 0.25 m off the truth: the ICP pulls it home, and a 0.2 m tolerance calls that a move
    off = G.Candidate((truth[0] + 0.25, truth[1], truth[2]), 0.5, math.nan)
    moved = G.refine(grid, G.GlobalMatch("ok", (off,)), pts)
    top = moved.candidates[0]
    assert top.lattice_pose == off.pose and _close(top.pose, truth, m=0.05)
    d = G.decide(moved, e_min=0.0, r_max=1.0, sep_m=0.2)
    assert not d.confirmed and d.reason == "refine_moved"
