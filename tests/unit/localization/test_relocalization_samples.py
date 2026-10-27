"""Start poses for the global matcher's calibration (plan B4a): the footprint on free cells only, keep-out
rectangles, the aisle ridge stratum, the independence rule across splits, reproducibility."""

import math

import numpy as np
import pytest

from forklift_core.localization import relocalization_samples as S
from forklift_core.localization.slam_map import Grid
from forklift_core.planning.geometry import Rectangle

RES = 0.05


def _grid(width_m=12.0, height_m=8.0, unknown_strip=False):
    w, h = int(width_m / RES), int(height_m / RES)
    data = np.zeros((h, w), np.int8)
    data[0, :] = data[-1, :] = data[:, 0] = data[:, -1] = 100
    data[60:100, 100:140] = 100  # a block in the middle
    if unknown_strip:
        data[:, 200:203] = -1
    return Grid(data, (0.0, 0.0, 0.0), RES)


def test_the_footprint_must_lie_on_free_cells_only():
    g = _grid()
    assert S.footprint_free(g, (3.0, 1.5, 0.0))
    # nose 1.29 - 0.34 = 0.95 m ahead of base: from x 4.1 it reaches the block at x 5.0
    assert not S.footprint_free(g, (4.1, 4.0, 0.0))
    assert S.footprint_free(g, (3.9, 4.0, 0.0))
    # a single corner clipping one occupied cell is enough to refuse
    g2 = Grid(g.data.copy(), g.origin, RES)
    g2.data[38, 78] = 100  # x 3.90-3.95, y 1.90-1.95
    corners = S.footprint_corners((3.0, 1.5, 0.0))
    assert corners[:, 0].max() == pytest.approx(3.95) and corners[:, 1].max() == pytest.approx(1.86)
    assert S.footprint_free(g2, (3.0, 1.5, 0.0))  # the cell starts at y 1.90: not touched
    g2.data[37, 78] = 100  # y 1.85-1.90: the corner at y 1.86 enters it
    assert not S.footprint_free(g2, (3.0, 1.5, 0.0))


def test_unknown_cells_and_the_grid_edge_refuse_too():
    g = _grid(unknown_strip=True)
    assert not S.footprint_free(g, (10.0, 1.5, 0.0))  # spans the unknown strip at x 10.0-10.15
    assert not S.footprint_free(g, (0.3, 1.5, math.pi))  # beyond x 0


def test_keep_out_rectangles_refuse_the_low_obstacles_the_grid_does_not_show():
    g = _grid()
    pallet = Rectangle(4.4, 1.5, 1.2, 0.8)  # x 3.8-5.0, the nose reaches 3.95
    assert S.footprint_free(g, (3.0, 1.5, 0.0))
    assert not S.footprint_free(g, (3.0, 1.5, 0.0), keep_out=[S.rectangle_corners(pallet)])


def test_the_ridge_runs_down_the_middle_of_an_aisle():
    data = np.full((40, 200), 100, np.int8)
    data[10:30, :] = 0  # a 1 m wide corridor from y 0.5 to 1.5
    ridge = S.ridge_cells(Grid(data, (0.0, 0.0, 0.0), RES))
    rows = np.unique(np.argwhere(ridge[:, 50:150])[:, 0])
    assert set(rows) <= {19, 20}  # the centre line at y 0.95-1.05


def test_starts_are_independent_across_splits_and_reproducible():
    g = _grid()
    dev = S.sample_starts(g, count=6, seed=1027, sep_m=1.5)
    again = S.sample_starts(g, count=6, seed=1027, sep_m=1.5)
    assert [p.base for p in dev] == [p.base for p in again]
    val = S.sample_starts(g, count=6, seed=1028, taken=[p.base for p in dev], sep_m=1.5)
    everything = [p.base for p in dev + val]
    for i, a in enumerate(everything):
        assert S.independent(a, everything[:i] + everything[i + 1:], sep_m=1.5)
        assert S.footprint_free(g, a)
    assert [p.stratum for p in dev[:6]] == ["A", "A", "B", "C", "A", "A"]


def test_a_turn_on_the_spot_is_not_an_independent_start():
    # Codex B-P1 pre-registration P2: the 360 deg scan from one laser position is one observation at any heading
    base = (3.0, 2.0, 0.0)
    lx, ly = S.laser_xy(base)
    turned = (lx - S.LASER_X_M * math.cos(1.26), ly - S.LASER_X_M * math.sin(1.26), 1.26)  # same laser spot, 72 deg
    assert S.laser_xy(turned) == pytest.approx((lx, ly))
    assert not S.independent(turned, [base])
    assert S.independent((6.2, 2.0, 0.0), [base])


def test_the_wall_band_lies_beside_walls_not_in_the_open():
    g = _grid()
    band = S.wall_band_cells(g)
    rows, cols, _ = g.cells_of(np.array([[0.6, 4.0], [3.0, 4.0]]))
    assert band[rows[0], cols[0]] and not band[rows[1], cols[1]]


def test_a_map_too_small_for_the_count_raises():
    with pytest.raises(ValueError):
        S.sample_starts(_grid(), count=10_000, seed=0, max_attempts=2_000)


def test_a_validation_map_is_drawn_until_full():
    g = _grid()
    full = S.sample_starts(g, count=10_000, seed=3, max_attempts=3_000, fill=True)
    assert 0 < len(full) < 10_000
    everything = [p.base for p in full]
    assert all(S.independent(a, everything[:i] + everything[i + 1:]) for i, a in enumerate(everything))


def test_a_blocked_stratum_does_not_end_the_fill_while_another_has_room():
    # Codex B4a re-review P3: B (the crest) blocked by earlier starts must not stop A and C
    g = _grid()
    ridge = S.ridge_cells(g)
    crest = [(g.origin[0] + (c + 0.5) * RES, g.origin[1] + (r + 0.5) * RES, 0.0) for r, c in np.argwhere(ridge)[::7]]
    out = S.sample_starts(g, count=10_000, seed=4, taken=crest, sep_m=1.0, fill=True, exhaust_after=500,
                          max_attempts=200_000)
    strata = {k: sum(p.stratum == k for p in out) for k in "ABC"}
    assert strata["A"] > 2 and strata["C"] > 0


def test_the_draw_reports_why_it_stopped():
    g = _grid()
    rep = {}
    S.sample_starts(g, count=3, seed=1, report=rep)
    assert rep["stop_reason"] == "count" and rep["attempts"] >= 3
    rep = {}
    S.sample_starts(g, count=10_000, seed=1, fill=True, exhaust_after=300, report=rep)
    assert rep["stop_reason"] == "exhausted" and rep["exhausted"] == ["A", "B", "C"]
    rep = {}
    S.sample_starts(g, count=10_000, seed=1, fill=True, max_attempts=50, report=rep)
    assert rep["stop_reason"] == "cap"
