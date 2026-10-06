"""Planning memory and SLAM static layer (priority-5 D2/D3 delta, Codex design review)."""

import numpy as np

from forklift_core.perception.planning_memory import SLAM_OCCUPIED, PlanningMemory


def memory():
    return PlanningMemory(0.0, 0.0, 2.0, 2.0, 0.05, hit_radius_m=0.0)


def test_a_hit_stays_until_a_newer_clear():
    m = memory()
    m.add_hits(np.array([1.0]), np.array([1.0]), 0.0)
    i, j = m.cell(1.0, 1.0)
    assert m.occupied()[i, j]
    m.add_clears(np.array([1.0]), np.array([1.0]), np.array([0.05]))
    assert not m.occupied()[i, j]


def test_a_stale_hit_is_never_registered_again():
    # Codex design review P1: the snapshot re-raised an old hit after a clear;
    # the memory takes each scan once, with its own stamp.
    m = memory()
    m.add_hits(np.array([1.0]), np.array([1.0]), 0.0)
    m.add_clears(np.array([1.0]), np.array([1.0]), np.array([0.05]))
    m.add_hits(np.array([1.0]), np.array([1.0]), 0.0)  # the same old scan offered again
    i, j = m.cell(1.0, 1.0)
    assert not m.occupied()[i, j]
    m.add_hits(np.array([1.0]), np.array([1.0]), 0.3)  # a new hit
    assert m.occupied()[i, j]


def test_the_slam_layer_marks_every_cell_an_occupied_square_overlaps_and_yields_to_a_clear():
    m = memory()
    data = np.full((4, 4), -1, dtype=np.int8)
    data[1, 2] = SLAM_OCCUPIED  # row = y, col = x
    m.set_slam(data, (0.5, 0.5), 0.05, index=3)
    occ = m.occupied()
    i, j = m.cell(0.5 + 2.5 * 0.05, 0.5 + 1.5 * 0.05)
    assert occ[i, j] and occ[i + 1, j] and occ[i - 1, j]  # grown by one cell
    assert m.slam_info == {"index": 3}
    # A removed tall object: the live evidence clears the cell; an old map does not bring it back.
    m.add_clears(np.array([0.5 + 2.5 * 0.05]), np.array([0.5 + 1.5 * 0.05]), np.array([1.0]))
    assert not m.occupied()[i, j]
    m.set_slam(data, (0.5, 0.5), 0.05, index=4)
    assert not m.occupied()[i, j]
    assert m.occupied(use_slam=False)[i, j] == False  # noqa: E712
