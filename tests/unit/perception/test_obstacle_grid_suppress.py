"""docs/design/2026-11-19-real-safety-wiring.md 2: ObstacleGrid.snapshot(suppress=) -- the planning grid's split of a
registered pallet's returns. Normal finite hits whose end point lies in a suppress rectangle do not mark; clearing,
the bounds and the near sector are untouched; None (the default) is the snapshot as before, byte for byte."""

import math

import numpy as np

from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, ObstacleGrid

from test_obstacle_grid import cell, config, scan, wall_ranges


def grid_with(*scans):
    g = ObstacleGrid(config())
    for s in scans:
        g.add_scan(s)
    return g


def test_none_is_the_snapshot_as_before_byte_for_byte():
    g = grid_with(scan(0.0, wall_ranges()), scan(0.1, wall_ranges(2.5)))
    a = g.snapshot(0.15, (0.0, 0.0, 0.0))
    b = g.snapshot(0.15, (0.0, 0.0, 0.0), suppress=None)
    assert np.array_equal(a.state, b.state)
    assert np.array_equal(np.nan_to_num(a.free_stamp, nan=-1.0), np.nan_to_num(b.free_stamp, nan=-1.0))


# sha256 of three snapshots' state and free_stamp bytes and metadata, computed with origin/main's obstacle_grid.py
# (before suppress=) on the scene below (Codex 5a impl P3: the bytes, not only the values of two calls after it)
MAIN_BYTES_SHA256 = "8f37e0111137fc566835def2d6cbc4283078d687bec2c5a605e1004a74ba61d2"


def test_none_and_the_default_are_main_byte_for_byte():
    import hashlib

    g = grid_with(scan(0.0, wall_ranges()), scan(0.1, wall_ranges(2.5)), scan(0.2, wall_ranges(1.8)))
    for kw in ({}, {"suppress": None}):
        h = hashlib.sha256()
        for t, corr, ver in ((0.15, (0.0, 0.0, 0.0), 0), (0.25, (0.05, -0.02, 0.01), 1), (0.6, (0.0, 0.0, 0.0), 2)):
            snap = g.snapshot(t, corr, ver, **kw)
            h.update(snap.state.tobytes())
            h.update(snap.free_stamp.tobytes())
            h.update(repr((snap.origin_x_m, snap.origin_y_m, snap.resolution_m, snap.stamp_s, snap.state.shape,
                           snap.state.dtype, snap.free_stamp.dtype)).encode())
        assert h.hexdigest() == MAIN_BYTES_SHA256, kw


def test_returns_inside_the_rectangle_do_not_mark_and_the_rest_still_do():
    g = grid_with(scan(0.0, wall_ranges()))
    # the wall at x = 3 spans y in about +-2.5; suppress its middle metre (a pallet face there)
    rect = (3.0, 0.0, 0.0, 0.15, 0.5)  # centre x, y, yaw, half length (along x), half width (along y)
    snap = g.snapshot(0.05, (0.0, 0.0, 0.0), suppress=[rect])
    assert cell(snap, 3.0, 0.0) != OCCUPIED and cell(snap, 2.9, 0.0) != OCCUPIED
    assert cell(snap, 3.0, 1.5) == OCCUPIED  # the wall outside the rectangle
    assert cell(snap, 1.5, 0.0) == FREE  # clearing up to the face is unchanged


def test_older_scans_are_reclassified_too():
    """A scan from before the mission (only marking by now) is split at the snapshot, not when it came in."""
    g = grid_with(scan(0.0, wall_ranges()))
    old = g.snapshot(1.0, (0.0, 0.0, 0.0))
    assert cell(old, 3.0, 0.0) == OCCUPIED
    snap = g.snapshot(1.0, (0.0, 0.0, 0.0), suppress=[(3.0, 0.0, 0.0, 0.15, 0.5)])
    assert cell(snap, 3.0, 0.0) != OCCUPIED


def test_a_rotated_rectangle_is_honoured():
    g = grid_with(scan(0.0, wall_ranges()))
    rect = (3.0, 0.0, math.pi / 2, 0.5, 0.15)  # the same area, its length along y
    snap = g.snapshot(0.05, (0.0, 0.0, 0.0), suppress=[rect])
    assert cell(snap, 3.0, 0.0) != OCCUPIED and cell(snap, 3.0, 1.5) == OCCUPIED


def test_a_grown_rectangle_grows_each_hit_by_its_own_range_bound():
    """Codex 5a impl round 6 P2-1 / round 7 P2: with the sixth element True, a hit is suppressed when it ends within
    the rectangle grown by its own range bound -- a far scan's face return measured 0.1 m long goes, while the same
    point seen from 1.1 m (a 2 cm bound) stays: a far scan's wide bound does not clear what a near scan saw."""
    from dataclasses import replace

    cfg = replace(config(), range_bound_model="a2_spec")
    rect = (4.0, 0.0, 0.0, 0.05, 0.3, True)
    far = ObstacleGrid(cfg)
    far.add_scan(scan(0.0, wall_ranges(4.1)))
    assert cell(far.snapshot(0.1, (0.0, 0.0, 0.0), suppress=[rect]), 4.1, 0.0) != OCCUPIED
    assert cell(far.snapshot(0.1, (0.0, 0.0, 0.0), suppress=[rect[:5]]), 4.1, 0.0) == OCCUPIED  # not grown
    both = ObstacleGrid(cfg)
    both.add_scan(scan(0.0, wall_ranges(4.1)))
    both.add_scan(scan(0.05, wall_ranges(1.1), odom=(3.0, 0.0, 0.0)))
    assert cell(both.snapshot(0.1, (0.0, 0.0, 0.0), suppress=[rect]), 4.1, 0.0) == OCCUPIED
