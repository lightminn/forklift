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
    assert m.slam_info["index"] == 3
    # A removed tall object: the live evidence clears the cell; an old map does not bring it back.
    m.add_clears(np.array([0.5 + 2.5 * 0.05]), np.array([0.5 + 1.5 * 0.05]), np.array([1.0]))
    assert not m.occupied()[i, j]
    m.set_slam(data, (0.5, 0.5), 0.05, index=4)
    assert not m.occupied()[i, j]
    assert m.occupied(use_slam=False)[i, j] == False  # noqa: E712


def test_a_slam_square_outside_the_hall_marks_nothing():
    # Codex review P1: negative slice ends filled the hall's far side.
    m = memory()
    data = np.full((2, 2), -1, dtype=np.int8)
    data[0, 0] = SLAM_OCCUPIED
    m.set_slam(data, (-0.2, -0.2), 0.05)
    assert not m.occupied().any()


def test_an_old_clear_never_hides_a_new_slam_obstacle_and_a_republished_map_brings_back_nothing():
    # Codex review P1: last_clear (t=1) against a tall obstacle that appears at t=10.
    m = memory()
    x, y = 1.0 + 0.025, 1.0 + 0.025
    m.add_clears(np.array([x]), np.array([y]), np.array([1.0]))
    data = np.full((40, 40), -1, dtype=np.int8)
    data[20, 20] = SLAM_OCCUPIED
    m.set_slam(data, (0.0, 0.0), 0.05, received_s=10.0, index=1)
    i, j = m.cell(x, y)
    assert m.occupied()[i, j]
    m.add_clears(np.array([x]), np.array([y]), np.array([12.0]))  # removed, seen free
    assert not m.occupied()[i, j]
    m.set_slam(data, (0.0, 0.0), 0.05, received_s=13.0, index=2)  # the same old map again
    assert not m.occupied()[i, j]


def test_lifting_the_pallet_retracts_its_remembered_hits_by_the_estimate():
    m = memory()
    m.add_hits(np.array([1.0, 1.0, 1.9]), np.array([1.0, 1.3, 1.0]), 0.0)
    gone = m.retract_rect(1.0, 1.0, 0.6, 0.8, 0.0, 0.2)
    occ = m.occupied()
    assert gone >= 2 and not occ[m.cell(1.0, 1.0)] and not occ[m.cell(1.0, 1.3)]
    assert occ[m.cell(1.9, 1.0)]  # outside the grown rectangle: kept
