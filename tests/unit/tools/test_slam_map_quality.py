"""slam-bounded-graph plan criterion 4: the planner's grid against a truth map -- the recorded clean scans placed at the
Isaac truth laser poses -- as preservation (truth obstacle cells the grid holds as occupied) and registration (the
grid's occupied cells near a truth obstacle)."""

import math

import numpy as np
import pytest

from tools import slam_map_quality as mq


def _wall_scans(n=20):
    """A laser at x = 0..1.9 m facing +y, a wall at y = 2 m: beams at -0.5..0.5 rad."""
    angles = np.linspace(-0.5, 0.5, 41)
    poses, ranges = [], []
    for k in range(n):
        x = 0.1 * k
        poses.append((x, 0.0, math.pi / 2))
        ranges.append(np.array([2.0 / math.cos(a) for a in angles]))
    return poses, ranges, angles


def test_truth_cells_lie_on_the_wall():
    poses, ranges, angles = _wall_scans()
    cells = mq.truth_cells(poses, ranges, angles, resolution_m=0.05, min_hits=2, range_max_m=12.0)
    ys = {iy for _, iy in cells}
    assert ys <= {39, 40} and len(cells) > 30


def _grid_from_cells(cells, origin=(-1.0, -1.0), res=0.05, shape=(80, 80)):
    data = np.zeros(shape, np.int8)
    for ix, iy in cells:
        c = int(math.floor(((ix + 0.5) * res - origin[0]) / res))
        r = int(math.floor(((iy + 0.5) * res - origin[1]) / res))
        if 0 <= r < shape[0] and 0 <= c < shape[1]:
            data[r, c] = 100
    return mq.Grid(data, (origin[0], origin[1], 0.0), res)


def test_a_grid_holding_the_wall_scores_full_and_a_shifted_or_empty_one_does_not():
    poses, ranges, angles = _wall_scans()
    truth = mq.truth_cells(poses, ranges, angles, resolution_m=0.05, min_hits=2, range_max_m=12.0)
    good = mq.quality(mq.grid_cells(_grid_from_cells(truth)), truth, resolution_m=0.05)
    assert good["preservation"] == pytest.approx(1.0) and good["registration"] == pytest.approx(1.0)
    shifted = {(ix, iy + 6) for ix, iy in truth}  # 0.30 m off
    bad = mq.quality(mq.grid_cells(_grid_from_cells(shifted)), truth, resolution_m=0.05)
    assert bad["preservation"] < 0.1 and bad["registration"] < 0.1
    empty = mq.quality(set(), truth, resolution_m=0.05)
    assert empty["preservation"] == 0.0 and empty["registration"] is None


def test_map_at_unit_start_is_the_latest_written_before_it():
    rows = [{"index": 0, "after_scan_id": 1}, {"index": 3, "after_scan_id": 40},
            {"repeat_of": 3, "repeats": 2, "last_index": 5, "last_after_scan_id": 90},
            {"index": 6, "after_scan_id": 120}]
    assert mq.map_index_before(rows, 100) == 3  # repeats of map 3 up to scan 90: map 3's file
    assert mq.map_index_before(rows, 130) == 6
    assert mq.map_index_before(rows, 0) is None
