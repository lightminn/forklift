"""Rolling LiDAR obstacle grid (priority-5 plan D2); synthetic scans only."""

import math

import numpy as np
import pytest

from forklift_core.perception.obstacle_grid import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    AgeErrorTable,
    GridConfig,
    ObstacleGrid,
    ObstacleScan,
)

TABLE = AgeErrorTable((0.1, 0.2, 0.3, 1.0, 3.0), (0.024, 0.030, 0.039, 0.091, 0.141), (0.014, 0.022, 0.026, 0.039, 0.059))
ANGLES = np.linspace(-math.pi, math.pi, 720, endpoint=False)


def config(**overrides):
    values = dict(x_min_m=-1, x_max_m=6, y_min_m=-3, y_max_m=3, error=TABLE)
    values.update(overrides)
    return GridConfig(**values)


def scan(stamp, ranges, *, odom=(0.0, 0.0, 0.0), self_hit=None, sensor="low", may_clear=True):
    ranges = np.asarray(ranges, dtype=float)
    return ObstacleScan(
        stamp, sensor, odom, (0.0, 0.0, 0.0), ANGLES, ranges,
        np.zeros(len(ANGLES), bool) if self_hit is None else self_hit, may_clear,
    )


def wall_ranges(x_wall=3.0):
    """Beams within +-40 deg hit a wall at x = x_wall; the rest see nothing."""
    out = np.full(len(ANGLES), np.inf)
    front = np.abs(ANGLES) < math.radians(40)
    out[front] = x_wall / np.cos(ANGLES[front])
    return out


def cell(snap, x, y):
    i = int(math.floor((x - snap.origin_x_m) / snap.resolution_m))
    j = int(math.floor((y - snap.origin_y_m) / snap.resolution_m))
    return snap.state[i, j]


def test_the_table_steps_up_to_the_next_listed_age_and_ends():
    assert TABLE.at(0.05) == (0.024, 0.014)
    assert TABLE.at(0.1) == (0.024, 0.014)
    assert TABLE.at(0.15) == (0.030, 0.022)
    assert TABLE.at(3.5) is None
    with pytest.raises(ValueError):
        AgeErrorTable((0.1, 0.2), (0.03, 0.02), (0.01, 0.02))


def test_a_fresh_scan_marks_the_wall_clears_the_way_and_leaves_behind_unknown():
    grid = ObstacleGrid(config())
    grid.add_scan(scan(0.0, wall_ranges()))
    snap = grid.snapshot(0.05, (0.0, 0.0, 0.0))
    assert cell(snap, 3.0, 0.0) == OCCUPIED
    assert cell(snap, 1.5, 0.0) == FREE
    assert cell(snap, 4.5, 0.0) == UNKNOWN  # behind the wall
    # Inflation: r = 0.024 + sensor 0.06 (+ cell quantisation) reaches 0.1 m short of the wall.
    assert cell(snap, 2.9, 0.0) == OCCUPIED
    assert cell(snap, 2.6, 0.0) == FREE


def test_self_hits_neither_mark_nor_clear():
    grid = ObstacleGrid(config())
    own = np.ones(len(ANGLES), bool)
    grid.add_scan(scan(0.0, wall_ranges(), self_hit=own))
    snap = grid.snapshot(0.05, (0.0, 0.0, 0.0))
    assert not snap.occupied.any() and not snap.free.any()


def test_an_unexplained_too_close_return_marks_the_near_sector():
    ranges = np.full(len(ANGLES), np.inf)
    ranges[0] = -np.inf  # straight behind (angle -pi)
    grid = ObstacleGrid(config())
    grid.add_scan(scan(0.0, ranges))
    snap = grid.snapshot(0.05, (0.0, 0.0, 0.0))
    assert cell(snap, -0.1, 0.0) == OCCUPIED


def test_the_newest_observation_wins_and_old_free_is_not_evidence():
    grid = ObstacleGrid(config())
    grid.add_scan(scan(0.0, wall_ranges(2.0)))  # a box at 2 m ...
    grid.add_scan(scan(0.5, wall_ranges(4.0)))  # ... gone half a second later
    snap = grid.snapshot(0.55, (0.0, 0.0, 0.0))
    assert cell(snap, 2.0, 0.0) == FREE
    # A second later the newer scan is too old to count as free: unknown, while
    # its wall stays occupied until occupied_max_age_s.
    snap = grid.snapshot(1.5, (0.0, 0.0, 0.0))
    # Nothing fresh covers it any more: the old box mark is back (conservative --
    # older scans only mark), and nothing there is FREE.
    assert cell(snap, 2.0, 0.0) == OCCUPIED
    assert cell(snap, 1.0, 0.0) == UNKNOWN
    assert cell(snap, 4.0, 0.0) == OCCUPIED
    snap = grid.snapshot(3.6, (0.0, 0.0, 0.0))
    assert cell(snap, 4.0, 0.0) == UNKNOWN and not grid.scans


def test_old_hits_grow_with_age_and_range():
    grid = ObstacleGrid(config())
    grid.add_scan(scan(0.0, wall_ranges(3.0)))
    fresh = grid.snapshot(0.05, (0.0, 0.0, 0.0)).occupied.sum()
    old = grid.snapshot(2.5, (0.0, 0.0, 0.0)).occupied.sum()
    assert old > 1.5 * fresh


def test_the_correction_moves_every_stored_scan_together():
    grid = ObstacleGrid(config())
    grid.add_scan(scan(0.0, wall_ranges(3.0)))
    a = grid.snapshot(0.06, (0.0, 0.0, 0.0))
    b = grid.snapshot(0.06, (0.0, 1.0, 0.0))  # correction moved the map 1 m in y
    # The wall spans y +-2.52 at x = 3 from the origin.
    assert cell(a, 3.0, -2.2) == OCCUPIED and cell(b, 3.0, -1.2) == OCCUPIED
    assert cell(b, 3.0, -2.2) != OCCUPIED


def test_each_scan_is_placed_at_its_own_odometry_pose():
    grid = ObstacleGrid(config())
    grid.add_scan(scan(0.0, wall_ranges(2.0), odom=(1.0, 0.0, 0.0)))  # drove 1 m first
    snap = grid.snapshot(0.05, (0.0, 0.0, 0.0))
    assert cell(snap, 3.0, 0.0) == OCCUPIED and cell(snap, 2.0, 0.0) == FREE


def test_far_hits_clear_only_to_the_mark_limit_and_mark_nothing():
    grid = ObstacleGrid(config(max_mark_m=2.0, max_clear_m=2.0))
    grid.add_scan(scan(0.0, wall_ranges(3.0)))
    snap = grid.snapshot(0.05, (0.0, 0.0, 0.0))
    assert not snap.occupied.any()
    assert cell(snap, 1.2, 0.0) == FREE and cell(snap, 2.5, 0.0) == UNKNOWN


def test_scans_out_of_order_are_rejected():
    grid = ObstacleGrid(config())
    grid.add_scan(scan(1.0, wall_ranges()))
    with pytest.raises(ValueError):
        grid.add_scan(scan(0.5, wall_ranges()))


def test_a_high_plane_marks_but_never_clears_what_a_low_plane_saw():
    grid = ObstacleGrid(config())
    grid.add_scan(scan(0.0, wall_ranges(2.0), sensor="low"))  # a low box at 2 m
    grid.add_scan(scan(0.05, wall_ranges(4.0), sensor="high", may_clear=False))  # beams pass over it
    snap = grid.snapshot(0.06, (0.0, 0.0, 0.0))
    assert cell(snap, 2.0, 0.0) == OCCUPIED and cell(snap, 4.0, 0.0) == OCCUPIED
    assert cell(snap, 3.0, 0.0) == UNKNOWN  # behind the low box nothing low was seen


def test_a_beam_that_reads_long_does_not_clear_the_surface_it_hit():
    grid = ObstacleGrid(config())
    ranges = wall_ranges(3.0)
    ranges[np.isfinite(ranges)] += 0.06  # every hit reads the full range error bound long
    grid.add_scan(scan(0.0, ranges))
    snap = grid.snapshot(0.05, (0.0, 0.0, 0.0))
    for x in (2.98, 3.0, 3.02):
        assert cell(snap, x, 0.0) == OCCUPIED


def test_cells_beside_the_body_can_be_free_and_under_it_cannot():
    grid = ObstacleGrid(config())
    s = scan(0.0, wall_ranges(4.0))
    s = ObstacleScan(s.stamp_s, s.sensor, s.odom_rear, s.laser_in_rear, s.angles_rad, s.ranges_m,
                     s.self_hit, True, (0.5, 0.2, 0.3))
    grid.add_scan(s)
    snap = grid.snapshot(0.05, (0.0, 0.0, 0.0))
    assert cell(snap, 0.2, 0.0) == UNKNOWN  # under the body
    assert cell(snap, 0.2, 0.42) == FREE  # just beside it


def test_closing_fills_a_pocket_narrower_than_twice_the_gap_and_wins_over_free():
    # Two blocks 0.3 m apart with free space seen between them (a pallet pocket).
    ranges = np.full(len(ANGLES), np.inf)
    for y0 in (-0.35, 0.35):
        sel = np.abs(np.tan(ANGLES) * 3.0 - y0) < 0.1
        sel &= np.cos(ANGLES) > 0
        ranges[sel] = 3.0 / np.cos(ANGLES[sel])
    open_grid = ObstacleGrid(config())
    open_grid.add_scan(scan(0.0, ranges))
    assert cell(open_grid.snapshot(0.05, (0.0, 0.0, 0.0)), 3.0, 0.0) != OCCUPIED
    closed_grid = ObstacleGrid(config(close_gap_m=0.25))
    closed_grid.add_scan(scan(0.0, ranges))
    snap = closed_grid.snapshot(0.05, (0.0, 0.0, 0.0))
    assert cell(snap, 3.0, 0.0) == OCCUPIED
    assert cell(snap, 1.5, 0.0) == FREE  # open floor in front stays free


def test_a_tilted_beam_neither_marks_nor_clears_past_its_band_limit():
    grid = ObstacleGrid(config())
    s = scan(0.0, wall_ranges(3.0))
    limit = np.full(len(ANGLES), 1.0)  # every beam leaves the band at 1 m
    s = ObstacleScan(s.stamp_s, s.sensor, s.odom_rear, s.laser_in_rear, s.angles_rad, s.ranges_m,
                     s.self_hit, True, None, limit)
    grid.add_scan(s)
    snap = grid.snapshot(0.05, (0.0, 0.0, 0.0))
    assert cell(snap, 0.5, 0.0) == FREE
    assert cell(snap, 2.0, 0.0) == UNKNOWN and cell(snap, 3.0, 0.0) == UNKNOWN


def test_line_closing_fills_a_pocket_end_to_end():
    from forklift_core.perception.obstacle_grid import _line_close

    occ = np.zeros((40, 40), dtype=bool)
    occ[10:30, 10:12] = True  # two long blocks 0.30 m (6 cells) apart, a pocket between
    occ[10:30, 18:20] = True
    filled = _line_close(occ, 10)
    assert filled[10:30, 10:20].all()  # every pocket cell, ends included (Codex: 10 cells were left)
    assert not filled[:, 25:].any() and not filled[:8].any()


def test_far_apart_objects_and_wide_gaps_stay_open():
    from forklift_core.perception.obstacle_grid import _line_close

    occ = np.zeros((60, 60), dtype=bool)
    occ[5:10, 5:10] = True
    occ[5:10, 40:45] = True  # 30 cells (1.5 m) away
    occ[30:50, 5:7] = True
    occ[30:50, 19:21] = True  # 12 empty cells (0.6 m) between: a gap wider than 0.5 m
    filled = _line_close(occ, 10)
    assert not filled[5:10, 15:35].any()
    assert not filled[30:50, 8:18].any()


def test_an_l_shaped_group_does_not_fill_its_open_corner():
    from forklift_core.perception.obstacle_grid import _line_close

    occ = np.zeros((60, 60), dtype=bool)
    occ[10:12, 10:50] = True  # one arm
    occ[10:50, 10:12] = True  # the other
    filled = _line_close(occ, 10)
    assert not filled[25:45, 25:45].any()  # a convex hull would fill this


def test_same_instant_sensors_shrink_together_so_each_covers_the_others_shadow():
    # Sensor A sees only beams pointing to +y (the -y half is its own body);
    # sensor B the reverse. Alone, each one's shadow shrinks the cells near y = 0.
    ranges = np.full(len(ANGLES), np.inf)
    own_a = np.sin(ANGLES) < 0
    own_b = np.sin(ANGLES) > 0
    def make(own, name):
        return ObstacleScan(0.0, name, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), ANGLES, ranges, own, True)
    alone = ObstacleGrid(config())
    alone.add_scan(make(own_a, "a"))
    assert cell(alone.snapshot(0.05, (0.0, 0.0, 0.0)), 2.0, 0.02) != FREE
    both = ObstacleGrid(config())
    both.add_scan(make(own_a, "a"))
    both.add_scan(make(own_b, "b"))
    assert cell(both.snapshot(0.05, (0.0, 0.0, 0.0)), 2.0, 0.02) == FREE
