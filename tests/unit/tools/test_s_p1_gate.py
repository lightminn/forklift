"""The S-P1 gate harness (tools/s_p1_gate.py): the geometric depth it feeds the real low layer and the kinematic run
it judges by the pre-registered rule."""

import math

import numpy as np

from tools import s_p1_gate as G


def test_the_camera_sees_the_floor_from_where_the_mount_says():
    pts = G.render_points(G.World(), (0.0, 0.0, 0.0), 0.10, stride=4)
    assert np.allclose(pts[:, 2], 0.0, atol=1e-9)
    # the nearest floor in view: the mount at rear x 0.959, 0.27 m up, tilted 0.10 rad, half vertical fov 27.26 deg
    near = 0.959 + 0.27 / math.tan(math.atan(240 / G.scene_rig.FOCAL_PX) + 0.10)
    assert abs(pts[:, 0].min() - near) < 0.02 and pts[:, 0].max() <= 0.959 + 3.0 + 1e-6


def test_a_box_in_front_occludes_the_floor_behind_it():
    world = G.World([(2.0, 0.0, 0.6, 0.8, 0.144)])
    pts = G.render_points(world, (0.0, 0.0, 0.0), 0.10, stride=2)
    on_box = pts[(np.abs(pts[:, 1]) < 0.3) & (pts[:, 2] > 0.1)]
    assert len(on_box) and np.allclose(on_box[:, 2].max(), 0.144, atol=1e-6)
    behind = pts[(np.abs(pts[:, 1]) < 0.05) & (pts[:, 0] > 2.31) & (pts[:, 0] < 2.6) & (pts[:, 2] < 0.01)]
    assert len(behind) == 0  # shadowed by the box


def test_box_overlap_is_the_rectangles_intersection():
    foot = G.Footprint(1.0, 0.2, 0.3)
    assert G.box_overlaps(foot, (0.0, 0.0, 0.0), (0.5, 0.0, 0.2, 0.2, 0.1))
    assert not G.box_overlaps(foot, (0.0, 0.0, 0.0), (0.5, 0.6, 0.2, 0.2, 0.1))
    assert G.box_overlaps(foot, (0.0, 0.0, math.pi / 2), (0.0, 0.5, 0.2, 0.2, 0.1))


def test_a_run_reports_by_the_preregistered_rule():
    r = G.run([(0.3, 0.0)], 0.3, stride=4, max_time_s=40.0)
    assert set(r) >= {"finished", "permanent_stop", "stopped_fraction", "first_block", "overlap", "pass"}
    assert r["pass"] == (r["finished"] and not r["permanent_stop"] and r["stopped_fraction"] <= 0.5)
