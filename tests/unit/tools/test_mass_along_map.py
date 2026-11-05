"""tools/mass_along_map.py on synthetic maps (docs/plans/2026-11-06-mass-along-followup-intervals-map.md, section 3)."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import mass_along_map as S  # noqa: E402
import mass_along_mechanism as M  # noqa: E402
from forklift_core.localization.slam_pose import compose  # noqa: E402

Q = (2.0, -1.0, 0.4)


def room_points(rng, q=Q):
    segs = S.segments_for("room")
    local = S.thin(S.sample_points(segs, rng))
    local = local[np.hypot(local[:, 0], local[:, 1]) <= S.REGION_M]
    return segs, local


def test_along_batch_matches_the_composed_transform():
    rng = np.random.default_rng(1)
    gs = np.column_stack([rng.uniform(-0.3, 0.3, 20), rng.uniform(-0.3, 0.3, 20), rng.uniform(-0.03, 0.03, 20)])
    t = (5.0, 3.0, 2.2)
    got = S.along_batch(Q, gs, t)
    want = [S.along_at(S.world_transform(Q, g), t) for g in gs]
    assert got == pytest.approx(want, abs=1e-12)


def test_the_map_offset_reads_as_the_estimate_error_of_d1():
    # a map shifted by F: the correction makes the estimate F o T, and D1's err3(T, F o T) is along_at(F, T)
    f = S.world_transform(Q, (0.03, -0.01, 0.004))
    t = (4.0, 1.0, -0.7)
    assert S.along_at(f, t) == pytest.approx(M.err3(t, compose(f, t))[0], abs=1e-12)


def test_alignment_recovers_an_off_lattice_transform_and_s_contains_it():
    rng = np.random.default_rng(2)
    segs, local = room_points(rng)
    g = (0.013, -0.027, math.radians(0.33))
    # the segments are in Q's frame: place them in the world at Q, then move by F
    world_segs = [tuple(S.se2_apply(Q, np.array([[a, b], [c, d]])).ravel()) for a, b, c, d in segs]
    grid = S.rasterize(world_segs, S.world_transform(Q, g), (0.017, 0.031), size=20.0)
    # rasterize centres its grid on the world origin: keep Q inside it
    al = S.align(grid, Q, local, 3e-4)
    assert al["g_opt"][0] == pytest.approx(g[0], abs=0.008)
    assert al["g_opt"][1] == pytest.approx(g[1], abs=0.008)
    assert al["g_opt"][2] == pytest.approx(g[2], abs=math.radians(0.15))
    assert not al["touches_box"]
    assert S.contains(al["S"], g)
    iv = S.value_interval(Q, al, Q)
    assert iv["lo"] <= g[0] <= iv["hi"]


@pytest.mark.parametrize("step", [S.COARSE, S.FINE])
def test_the_vertex_slack_bounds_the_cost_drop_inside_its_cell(step):
    rng = np.random.default_rng(3)
    segs, local = room_points(rng)
    grid = S.rasterize(segs, (0.0, 0.0, 0.0), (0.02, 0.01))
    q = (0.0, 0.0, 0.0)
    vertices = S.lattice(step, lo=(-0.1, -0.1, -0.02), hi=(0.1, 0.1, 0.02)) * np.array(step)
    vertices = vertices[rng.choice(len(vertices), 40, replace=False)]
    cv, sv = S.costs(grid, q, local, vertices, S.half(step))
    for _ in range(5):
        jitter = rng.uniform(-0.5, 0.5, vertices.shape) * np.array(step)
        cj = S.costs(grid, q, local, vertices + jitter)
        assert np.all(cj >= cv - sv - 1e-12)


def test_two_narrow_valleys_both_stay_in_s():
    # a repeated column row 0.25 m apart: the alignment along the row has competing minima inside the box
    rng = np.random.default_rng(4)
    segs = S.segments_for("repeat")
    local = S.thin(S.sample_points(segs, rng))
    local = local[np.hypot(local[:, 0], local[:, 1]) <= S.REGION_M]
    q = (0.0, 0.0, 0.0)
    # drop the end wall's points so only the columns constrain the along axis
    cols = local[np.abs(local[:, 0]) < 3.0]
    al = S.align(S.rasterize(segs, q, (0.01, 0.02)), q, cols, 1e-3)
    a_range = al["S_range"]["a"]
    assert a_range[0] <= -0.24 and a_range[1] >= 0.24
    iv = S.value_interval(q, al, q)
    assert iv["lo"] < 0 < iv["hi"]


def test_support_needs_the_whole_reach_known_and_refuses_thin_support():
    data = np.zeros((200, 200), np.int8)
    data[:, 120:] = -1  # unknown from x = 1.0 m (origin -5, res 0.05)
    grid = S.MapGrid(data, (-5.0, -5.0, 0.0), 0.05)
    q = (0.0, 0.0, 0.0)
    pts = np.array([[0.0, 0.0], [0.2, 0.0], [-1.0, 0.0]])
    ok = S.support(pts, q, [grid])
    # the reach at r = 0 is 0.424 + 0.10: x = 0 is 1.0 m from the unknown edge (ok), x = 0.2 is 0.8 m (ok),
    # but x = 0.6 would not be
    assert ok.tolist() == [True, True, True]
    assert not S.support(np.array([[0.6, 0.0]]), q, [grid])[0]
    assert S.support_problem(np.zeros((10, 2))) is not None
    line = np.column_stack([np.linspace(-2, 2, 400), np.zeros(400)])
    assert "spread" in S.support_problem(line)


def test_thin_keeps_one_point_per_cell():
    pts = np.array([[0.01, 0.01], [0.02, 0.03], [0.07, 0.01]])
    out = S.thin(pts)
    assert len(out) == 2
    assert out[np.argsort(out[:, 0])][0] == pytest.approx([0.015, 0.02])


def test_scan_points_skip_rep117_and_out_of_range():
    laser = {"angle_min_rad": 0.0, "angle_increment_rad": math.pi / 2, "range_min_m": 0.15, "range_max_m": 12.0}
    log = {"scan_stamps_s": np.array([1.0, 2.0]),
           "scan_ranges_m": np.array([[1.0, np.inf, -np.inf, 0.1], [2.0, 12.0, np.nan, 3.0]], np.float32),
           "laser_pose_world": np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 0.0]])}
    pts = S.scan_points(log, laser, {1.0})
    assert pts == pytest.approx(np.array([[1.0, 0.0]]))
    pts = S.scan_points(log, laser, {2.0})
    assert pts == pytest.approx(np.array([[3.0, 1.0], [1.0, -2.0]]), abs=1e-6)


def test_map_choice_is_the_last_map_at_or_before_the_scan_id():
    maps = [{"index": 0, "after_scan_id": 5, "origin": [0, 0, 0], "resolution_m": 0.05},
            {"index": 1, "after_scan_id": 9, "origin": [0, 0, 0], "resolution_m": 0.05},
            {"index": 2, "after_scan_id": 12, "origin": [0, 0, 0], "resolution_m": 0.05}]
    arrays = {f"map_{i:04d}": np.zeros((4, 4), np.int8) for i in range(3)}
    assert S.map_for(maps, arrays, 11)[1]["index"] == 1
    assert S.map_for(maps, arrays, 12)[1]["index"] == 2
    assert S.map_for(maps, arrays, 4) is None


def test_event_poses_keep_exactly_the_d1_events_and_values():
    rec = {"scan_id": 3, "stamp_s": 1.0, "status": "processed", "mode": "tracking", "odom_base": [2.0, 0.0, 0.0],
           "truth_base": [2.03, 0.0, 0.0], "map_from_odom": [0.03, 0, 0], "applied_map_from_odom": [0.03, 0, 0]}
    c_minus, c_plus = (0.03, 0.0, 0.0), (0.05, 0.0, 0.01)
    odom = (3.0, 0.2, 0.1)
    truth = (3.02, 0.21, 0.105)
    control = np.array([[2.5, *compose(c_plus, odom), *truth]])
    events = [(0.0, "first", None, (0.0, 0.0, 0.0), None), (1.0, "processed", (0.0, 0.0, 0.0), c_minus, None),
              (2.0, "release", c_minus, c_plus, 0.05), (3.0, "release", c_plus, c_minus, 0.05)]  # the last has no row
    d1 = M.correction_effects(events, [rec], control, -0.34)
    ev = S.event_poses(events, [rec], control, -0.34)
    assert [e["t"] for e in ev] == [e["t"] for e in d1] == [1.0, 2.0]
    assert [e["post_along"] for e in ev] == pytest.approx([e["post"][0] for e in d1])
    assert ev[1]["truth"] == pytest.approx(truth)


def test_pair_difference_is_undecided_when_the_widened_interval_holds_zero():
    a = {"opt": 0.0, "lo": -0.004, "hi": 0.004}
    b = {"opt": -0.03, "lo": -0.034, "hi": -0.026}
    d = S.pair_difference(a, b, 0.005)
    assert d["sign"] == -1 and d["hi"] == pytest.approx(-0.026 + 0.004 + 0.005)
    assert S.pair_difference(a, b, 0.03)["sign"] == 0
    assert S.pair_difference(a, None, 0.005) is None


def test_floor_band_is_the_smallest_tested_distance_at_least_as_far():
    prec = {"floor": {"event": {"0": 0.001, "5": 0.004, "10": 0.009}}}
    assert S.floor_for(prec, "event", 0.0) == 0.001
    assert S.floor_for(prec, "event", 3.2) == 0.004
    assert S.floor_for(prec, "event", 12.0) == 0.009
    assert S.floor_for(None, "event", 1.0) is None


def test_s_covers_every_sampled_transform_within_min_plus_delta():
    # the box refinement's guarantee, checked by sampling: any transform with cost <= min + delta lies in a final box
    rng = np.random.default_rng(5)
    segs, local = room_points(rng)
    q = (0.0, 0.0, 0.0)
    grid = S.rasterize(segs, (0.012, -0.007, math.radians(0.2)), (0.031, 0.004))
    delta = 2e-4
    al = S.align(grid, q, local, delta)
    g0 = np.array(al["g_opt"])
    samples = g0 + rng.uniform(-1, 1, (3000, 3)) * np.array([0.06, 0.06, math.radians(0.6)])
    c = S.costs(grid, q, local, samples)
    near = samples[c <= al["cost_min"] + delta]
    assert len(near) > 50
    assert all(S.contains(al["S"], g) for g in near)
