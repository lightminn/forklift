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


def _random_grid(rng, width_m, height_m, n_boxes):
    boxes = []
    for _ in range(n_boxes):
        x0, y0 = rng.uniform(0.0, width_m - 0.4), rng.uniform(0.0, height_m - 0.4)
        boxes.append((x0, y0, x0 + rng.uniform(0.1, 0.6), y0 + rng.uniform(0.1, 0.6)))
    return _grid(width_m, height_m, boxes)


def _assert_bounds_cover_leaves(grid, pts, cfg):
    pts = G._thin(pts, RES)
    if len(pts) < cfg.min_points:  # a laser inside a random box sees nothing: the matcher refuses it (uninformative)
        return 0
    s = G._Search(grid, pts, cfg)
    rows, cols = np.nonzero(s.cand)
    for level in range(1, cfg.levels + 1):
        m = (1 << level) - 1
        for yaw in range(s.n_yaw):
            h = np.full(len(rows), yaw)
            leaves = s.scores(0, h, rows, cols)
            bounds = s.scores(level, h, rows & ~m, cols & ~m)
            assert np.all(bounds >= leaves - 1e-12), (level, yaw)
    return 1


def test_every_node_bound_covers_every_leaf_under_it_even_at_the_grid_edge():
    """B-P1b: the pyramid bound of a node is >= the score of each leaf below it. Walls on the grid's first row and
    column put endpoints of near-edge leaves into blocks that start outside the grid; a pyramid built before padding
    read those blocks as 0 (hall map val2_000: level-5 bound 0.457 over a 0.975 leaf)."""
    grid = _grid(3.2, 2.4, [(2.0, 1.2, 2.5, 1.95)])
    cfg = G.MatchConfig(yaw_step_rad=math.radians(30.0), clearance_m=0.1, levels=4)
    assert _assert_bounds_cover_leaves(grid, _points(grid, (0.4, 0.35, 0.3)), cfg)  # walls at x = 0, y = 0 are close


@pytest.mark.parametrize("seed", range(6))
def test_bounds_cover_leaves_on_random_maps_near_every_edge_and_at_the_heading_wrap(seed):
    """Random boxes, the laser near each of the four walls and in the middle, and headings on both sides of the wrap
    (the yaw lattice's first and last index, 0 and just under 2 pi) -- the bound holds for every node and level."""
    rng = np.random.default_rng([20261028, seed])
    grid = _random_grid(rng, 3.6, 2.8, 6)
    cfg = G.MatchConfig(yaw_step_rad=math.radians(15.0), clearance_m=0.1, levels=4)
    checked = 0
    for x, y in ((0.3, 1.4), (3.3, 1.4), (1.8, 0.3), (1.8, 2.5), (1.8, 1.4)):
        for yaw in (0.0, 2 * math.pi - 1e-3, rng.uniform(0, 2 * math.pi)):
            checked += _assert_bounds_cover_leaves(grid, _points(grid, (x, y, yaw), beams=180), cfg)
    assert checked >= 9  # most poses see the room


@pytest.mark.parametrize("seed", range(4))
def test_the_search_is_exact_against_brute_force_with_a_keep_ratio_near_one(seed):
    """A keep ratio just under 1 sets the threshold at the greedy floor itself (not at the best leaf -- the floor can
    be well below it): the search still finds the brute-force best. Random maps, the laser near the edges; all fail
    with the pyramid pooled before padding."""
    rng = np.random.default_rng([20261028, 100 + seed])
    grid = _random_grid(rng, 3.2, 2.4, 4)
    cfg = G.MatchConfig(yaw_step_rad=math.radians(10.0), clearance_m=0.1, levels=4, keep_ratio=1.0 - 1e-9)
    for pose in ((0.3, 0.3, rng.uniform(0, 2 * math.pi)), (2.9, 2.1, rng.uniform(0, 2 * math.pi)),
                 (1.6, 1.2, 2 * math.pi - 1e-3)):
        pts = _points(grid, pose, beams=180)
        if len(G._thin(pts, RES)) < cfg.min_points or not G.candidate_cells(grid, cfg.clearance_m).any():
            continue
        assert G.global_match(grid, pts, cfg).candidates[0].score == pytest.approx(G.brute_force_best(grid, pts, cfg),
                                                                                    abs=1e-12)


def test_a_symmetric_room_keeps_the_turned_competitor_and_is_not_confirmed():
    grid = _grid(6.0, 4.0, [])
    truth = (2.0, 1.4, 0.0)
    pts = _points(grid, truth)
    # an empty box overflows a small frontier: no answer at all, never a confirmation
    assert G.global_match(grid, pts, G.MatchConfig(max_nodes=10_000)).status == "overflow"
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


def test_the_padding_reaches_every_rotated_endpoint():
    """Codex review (B-P1b): the pad was sized from the largest unrotated coordinate, but a heading turns an endpoint
    at (5.4, 5.5) to 7.7 m along one axis; a scan of only that far wall segment indexed past the padded field."""
    grid = _grid(10.0, 10.0, [])
    ys = np.arange(4.0, 5.6, RES)
    pts = np.stack([np.full(len(ys), 5.4), ys], 1)  # 32 endpoints, the farthest 7.7 m away
    s = G._Search(grid, G._thin(pts, RES), G.MatchConfig())
    assert s.pad > max(np.abs(s.oc).max(), np.abs(s.orow).max()) + (1 << G.MatchConfig().levels)
    assert G.global_match(grid, pts).status in ("ok", "overflow")


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
