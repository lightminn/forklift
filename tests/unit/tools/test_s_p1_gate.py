"""The S-P1 gate harness (tools/s_p1_gate.py): the geometric depth it feeds the real low layer and the kinematic run
it judges by the pre-registered rule."""

import math

import numpy as np

from forklift_core.safety import low_layer as L
from tools import s_p1_gate as G


def test_the_camera_sees_the_floor_from_where_the_mount_says():
    pts, origin = G.render_points(G.World(), (0.0, 0.0, 0.0), 0.10, stride=4)
    assert np.allclose(pts[:, 2], 0.0, atol=1e-9) and np.allclose(origin, [0.959, 0.0, 0.27])
    # the nearest floor in view: the mount at rear x 0.959, 0.27 m up, tilted 0.10 rad, half vertical fov 27.26 deg
    near = 0.959 + 0.27 / math.tan(math.atan(240 / G.scene_rig.FOCAL_PX) + 0.10)
    assert abs(pts[:, 0].min() - near) < 0.02 and pts[:, 0].max() <= 0.959 + 3.0 + 1e-6


def test_a_box_in_front_occludes_the_floor_behind_it():
    world = G.World([(2.0, 0.0, 0.6, 0.8, 0.0, 0.144)])
    pts, _ = G.render_points(world, (0.0, 0.0, 0.0), 0.10, stride=2)
    on_box = pts[(np.abs(pts[:, 1]) < 0.3) & (pts[:, 2] > 0.1)]
    assert len(on_box) and np.allclose(on_box[:, 2].max(), 0.144, atol=1e-6)
    behind = pts[(np.abs(pts[:, 1]) < 0.05) & (pts[:, 0] > 2.31) & (pts[:, 0] < 2.6) & (pts[:, 2] < 0.01)]
    assert len(behind) == 0  # shadowed by the box


def test_a_floating_box_leaves_floor_hits_under_it_but_its_cells_are_not_free():
    # the S-P1 pre-check counterexample: rays reach the floor under a box at z 0.05-0.12; its column is not crossed
    world = G.World([(1.9, 0.0, 0.4, 0.8, 0.05, 0.12)])
    pts, origin = G.render_points(world, (0.0, 0.0, 0.0), 0.10, stride=1)
    under = pts[(pts[:, 0] > 1.75) & (pts[:, 0] < 2.05) & (np.abs(pts[:, 1]) < 0.3) & (pts[:, 2] < 0.01)]
    assert len(under) > 0
    layer = L.LowLayer(L.LowLayerConfig(), origin_xy=(-6.0, -6.0), size_cells=(320, 320))
    assert layer.integrate(pts, 0.0, origin=origin).ok
    assert layer.state_at(1.9, 0.0) != L.FREE and layer.state_at(2.0, 0.1) != L.FREE


def test_dropout_rectangles_remove_returns():
    world = G.World(dropout=[(1.9, 2.6, -1.0, 1.0)])
    pts, _ = G.render_points(world, (0.0, 0.0, 0.0), 0.10, stride=2)
    assert not ((pts[:, 0] >= 1.9) & (pts[:, 0] <= 2.6) & (np.abs(pts[:, 1]) <= 1.0)).any()


def test_box_overlap_is_the_rectangles_intersection():
    foot = G.Footprint(1.0, 0.2, 0.3)
    assert G.box_overlaps(foot, (0.0, 0.0, 0.0), (0.5, 0.0, 0.2, 0.2))
    assert not G.box_overlaps(foot, (0.0, 0.0, 0.0), (0.5, 0.6, 0.2, 0.2))
    assert G.box_overlaps(foot, (0.0, 0.0, math.pi / 2), (0.0, 0.5, 0.2, 0.2))


def test_the_stop_model_is_drive_permissions():
    assert G.stopping_distance_m(0.3) == np.float64(0.125) or abs(G.stopping_distance_m(0.3) - 0.125) < 1e-9
    assert abs(G.stopping_distance_m(0.6) - 0.26) < 1e-9


def test_a_run_reports_by_the_preregistered_rule():
    r = G.run([(0.3, 0.0)], 0.3, stride=4, max_time_s=40.0)
    assert set(r) >= {"finished", "permanent_stop", "stopped_fraction", "first_block", "overlap", "pass"}
    assert r["pass"] == (r["finished"] and not r["permanent_stop"] and r["stopped_fraction"] <= 0.5)


def test_a_refused_reverse_after_a_gear_change_is_not_driven_as_braking():
    # S-P1 implementation review P1: forward allowed (the strip ahead declared clear), reverse refused (nothing behind
    # the rear overhang was ever swept): the truck stands at the gear change instead of "braking" into reverse
    r = G.run([(0.03, 0.0), (-0.03, 0.0)], 0.3, start_clear_m=0.3, stride=4, max_time_s=40.0)
    assert not r["finished"] and r["travel_m"] <= 0.031


def test_the_forks_shade_the_floor_right_ahead_of_their_tips():
    # S-P1 implementation review P2: with a steeper mount the floor just ahead of the tips is behind the tines
    import pytest
    with_forks, _ = G.render_points(G.World(), (0.0, 0.0, 0.0), 0.25, stride=1)
    mp = pytest.MonkeyPatch()
    mp.setattr(G, "FORK_BOXES", [])
    try:
        bare, _ = G.render_points(G.World(), (0.0, 0.0, 0.0), 0.25, stride=1)
    finally:
        mp.undo()
    floor_with = with_forks[with_forks[:, 2] < 1e-6]
    assert len(floor_with) < len(bare)  # rays that hit a tine never reach the floor
    near = lambda p: p[(p[:, 0] < 1.5) & (np.abs(p[:, 1]) > 0.15) & (np.abs(p[:, 1]) < 0.4)]  # noqa: E731
    assert len(near(floor_with)) < len(near(bare))  # the shade lies just ahead of and beside the tips
    tine_hits = with_forks[with_forks[:, 2] > 0.02]
    assert len(tine_hits) and tine_hits[:, 0].max() <= 1.29 + 1e-6
