"""Real-time plan 5.26 G3-1b: _clear_group judged at the beams' cells gives the original's state and free_stamp byte
for byte -- in both kernel modes, with and without the body rectangle, at the grid's edges, across band edges and
for several same-instant scans; numba and its NumPy stand-in agree; non-increasing band edges take the original."""

import math

import numpy as np
import pytest

from forklift_core import kernels
from forklift_core.perception import obstacle_grid as og

TABLE = og.AgeErrorTable((0.1, 0.2, 0.3, 1.0, 3.0), (0.024, 0.030, 0.039, 0.091, 0.141),
                         (0.014, 0.022, 0.026, 0.039, 0.059))


def _group(grid, rng, n_scans, *, own, near_edge):
    cfg = grid.config
    group = []
    rear = (float(rng.uniform(cfg.x_min_m, cfg.x_max_m)) if near_edge else float(rng.uniform(-1, 1)),
            float(rng.uniform(cfg.y_min_m, cfg.y_max_m)) if near_edge else float(rng.uniform(-1, 1)),
            float(rng.uniform(-math.pi, math.pi)))
    for k in range(n_scans):
        n = 360
        ang = np.linspace(-math.pi, math.pi, n, endpoint=False)
        ranges = rng.uniform(0.3, 6.0, n)
        ranges[rng.random(n) < 0.2] = np.inf
        ranges[rng.random(n) < 0.05] = np.nan
        laser = (rear[0] + rng.uniform(-0.3, 0.3), rear[1] + rng.uniform(-0.3, 0.3))
        usable = rng.random(n) > 0.05
        limit = np.full(n, rng.uniform(3.0, 6.0))
        foot = (0.9, 0.3, 0.35) if own else None
        scan = og.ObstacleScan(0.1 * k, f"s{k}", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), ang, ranges,
                               np.zeros(n, bool), True, own_footprint=foot)
        geo = (rear, laser, ranges, np.cos(ang), np.sin(ang), usable, limit)
        group.append((scan, float(rng.uniform(0.0, 0.25)), geo))
    return group


def _run(grid, group, cells):
    state = np.zeros(grid.config.shape, dtype=np.uint8)
    state[::7, ::5] = og.OCCUPIED
    stamp = np.full(grid.config.shape, np.nan)
    if cells:
        grid._clear_group_cells(group, state, stamp)
    else:
        _original(grid, group, state, stamp)
    return state, stamp


def _original(grid, group, state, stamp):
    keep = og.CLEAR_KERNEL
    og.CLEAR_KERNEL = False
    try:
        grid._clear_group(group, state, stamp)
    finally:
        og.CLEAR_KERNEL = keep


@pytest.mark.parametrize("mode", ["reference", "numba"])
def test_cells_equal_the_original_byte_for_byte(mode):
    kernels.set_mode(mode)
    try:
        rng = np.random.default_rng(5)
        freed = 0
        for case in range(24):
            grid = og.ObstacleGrid(og.GridConfig(x_min_m=-6, x_max_m=6, y_min_m=-6, y_max_m=6, error=TABLE))
            group = _group(grid, rng, 1 + case % 3, own=case % 2 == 0, near_edge=case % 4 == 3)
            a = _run(grid, group, cells=False)
            b = _run(grid, group, cells=True)
            assert a[0].tobytes() == b[0].tobytes(), case
            assert a[1].tobytes() == b[1].tobytes(), case
            freed += int((a[0] == og.FREE).sum())
        assert freed > 1000  # the equality is not of empty results
    finally:
        kernels.set_mode("reference")


def test_numba_and_the_numpy_stand_in_agree():
    pytest.importorskip("numba")
    from forklift_core.perception import grid_numba

    rng = np.random.default_rng(7)
    mask = rng.random((60, 50)) > 0.2
    ci, cj = np.nonzero(rng.random((60, 50)) > 0.5)
    for r in (0.0, 1.3, 2.4, 3.9):
        offs = og._reach_offsets(r)
        want = og._cells_eroded_numpy(mask, ci, cj, offs)
        rows = grid_numba.row_intervals(offs)
        got = grid_numba.cells_eroded_rows(grid_numba.next_off(mask), ci, cj, rows)
        np.testing.assert_array_equal(want, got)
        np.testing.assert_array_equal(want, grid_numba.cells_eroded(mask, ci, cj, np.asarray(offs, np.int64)))


def test_non_increasing_band_edges_take_the_original(monkeypatch):
    monkeypatch.setattr(og, "CLEAR_KERNEL", True)
    grid = og.ObstacleGrid(og.GridConfig(x_min_m=-3, x_max_m=3, y_min_m=-3, y_max_m=3, error=TABLE,
                                         rho_bands_m=(1.5, 1.5, 4.0)))
    called = []
    monkeypatch.setattr(og.ObstacleGrid, "_clear_group_cells", lambda *a: called.append(1))
    rng = np.random.default_rng(1)
    grid._clear_group(_group(grid, rng, 1, own=False, near_edge=False), np.zeros(grid.config.shape, np.uint8),
                      np.full(grid.config.shape, np.nan))
    assert called == []
