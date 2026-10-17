"""Real-time plan S3 (docs/plans/2026-10-17-realtime-sim.md, D2, V1): the compiled grid kernels give the NumPy
reference's masks bit for bit -- erosion and dilation by disk and reach offsets (row runs and the general loop),
the gap closing along rows, columns and diagonals, the beam mask -- on random masks and beams."""

import importlib.util
import math

import numpy as np
import pytest

from forklift_core import kernels
from forklift_core.perception import obstacle_grid as og

pytestmark = pytest.mark.skipif(importlib.util.find_spec("numba") is None, reason="numba is not installed")


def under(mode, fn, *args, **kwargs):
    previous = kernels.set_mode(mode)
    try:
        return fn(*args, **kwargs)
    finally:
        kernels.set_mode(previous)


@pytest.mark.parametrize("seed", range(25))
def test_erode_dilate_and_close_match_the_reference(seed):
    rng = np.random.default_rng(seed)
    shape = (int(rng.integers(5, 90)), int(rng.integers(5, 90)))
    mask = rng.random(shape) < rng.choice([0.05, 0.5, 0.95])
    radius = float(rng.choice([0.0, 0.7, 1.5, 4.2, rng.uniform(0, 6)]))
    for fn, kw in ((og._erode, {}), (og._dilate, {}),
                   (og._erode, {"offsets": og._disk_offsets(radius)})):
        a = under("reference", fn, mask, radius, **kw)
        b = under("numba", fn, mask, radius, **kw)
        assert a.dtype == b.dtype and np.array_equal(a, b)
    gap = int(rng.integers(0, 12))
    occupied = mask & (rng.random(shape) < 0.2)
    a = under("reference", og._line_close, occupied, gap)
    b = under("numba", og._line_close, occupied, gap)
    assert np.array_equal(a, b)


def test_a_general_offset_list_takes_the_loop():
    from forklift_core.perception import grid_numba

    ring = np.array([[0, -2], [0, 2], [1, 0]], dtype=np.int64)  # row 0 is not one run
    assert grid_numba.row_intervals(ring) is None
    mask = np.random.default_rng(1).random((30, 30)) < 0.6
    assert np.array_equal(under("reference", og._erode, mask, 0.0, offsets=ring),
                          under("numba", og._erode, mask, 0.0, offsets=ring))


@pytest.mark.parametrize("seed", range(10))
def test_the_beam_mask_matches_the_reference(seed):
    rng = np.random.default_rng(50 + seed)
    cfg = og.GridConfig(-3.0, 3.0, -3.0, 3.0, og.AgeErrorTable((0.1, 1.0), (0.005, 0.01), (0.001, 0.002)),
                        resolution_m=0.05, max_mark_m=5.0, max_clear_m=5.0, range_min_m=0.2, free_max_age_s=0.2,
                        occupied_max_age_s=0.3, sensor_bound_m=0.06, free_r_cap_m=0.2, free_rho_m=4.0)
    grid = og.ObstacleGrid(cfg)
    angles = np.linspace(-math.pi, math.pi, 1600, endpoint=False) + rng.uniform(-0.01, 0.01)
    laser = (float(rng.integers(-40, 40)) * 0.05 + rng.choice([0.0, 1e-17, -1e-17]), float(rng.uniform(-1, 1)), 0.0)
    lengths = np.maximum(rng.uniform(-0.5, 5.0, len(angles)), 0.0)
    a = under("reference", grid._ray_mask, laser, np.cos(angles), np.sin(angles), lengths)
    b = under("numba", grid._ray_mask, laser, np.cos(angles), np.sin(angles), lengths)
    assert np.array_equal(a, b)
