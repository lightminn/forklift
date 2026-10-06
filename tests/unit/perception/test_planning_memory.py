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
    m.set_slam(data, (0.5, 0.5), 0.05, stamp_s=0.0, index=3)
    occ = m.occupied()
    i, j = m.cell(0.5 + 2.5 * 0.05, 0.5 + 1.5 * 0.05)
    assert occ[i, j] and occ[i + 1, j] and occ[i - 1, j]  # grown by one cell
    assert m.slam_info["index"] == 3
    # A removed tall object: the live evidence clears the cell; an old map does not bring it back.
    m.add_clears(np.array([0.5 + 2.5 * 0.05]), np.array([0.5 + 1.5 * 0.05]), np.array([1.0]))
    assert not m.occupied()[i, j]
    m.set_slam(data, (0.5, 0.5), 0.05, stamp_s=0.5, index=4)
    assert not m.occupied()[i, j]
    assert m.occupied(use_slam=False)[i, j] == False  # noqa: E712


def test_a_slam_square_outside_the_hall_marks_nothing():
    # Codex review P1: negative slice ends filled the hall's far side.
    m = memory()
    data = np.full((2, 2), -1, dtype=np.int8)
    data[0, 0] = SLAM_OCCUPIED
    m.set_slam(data, (-0.2, -0.2), 0.05, stamp_s=0.0)
    assert not m.occupied().any()


def test_an_old_clear_never_hides_a_new_slam_obstacle_and_a_republished_map_brings_back_nothing():
    # Codex review P1: last_clear (t=1) against a tall obstacle that appears at t=10.
    m = memory()
    x, y = 1.0 + 0.025, 1.0 + 0.025
    m.add_clears(np.array([x]), np.array([y]), np.array([1.0]))
    data = np.full((40, 40), -1, dtype=np.int8)
    data[20, 20] = SLAM_OCCUPIED
    m.set_slam(data, (0.0, 0.0), 0.05, stamp_s=10.0, index=1)
    i, j = m.cell(x, y)
    assert m.occupied()[i, j]
    m.add_clears(np.array([x]), np.array([y]), np.array([12.0]))  # removed, seen free
    assert not m.occupied()[i, j]
    m.set_slam(data, (0.0, 0.0), 0.05, stamp_s=13.0, index=2)  # the same old map again
    assert not m.occupied()[i, j]


def test_lifting_the_pallet_retracts_its_own_endpoints_and_keeps_a_box_beside_it():
    # Codex review P1: an outside box 0.135 m from the pallet end lost its memory.
    m = PlanningMemory(0.0, 0.0, 2.0, 2.0, 0.05, hit_radius_m=0.1)
    m.add_hits(np.array([1.0, 1.0]), np.array([1.0, 1.3]), 0.0)  # the pallet's own endpoints
    m.add_hits(np.array([1.45]), np.array([1.0]), 0.0)  # a box 0.135 m beyond the pallet's 0.3 m half depth
    gone = m.retract_endpoints(1.0, 1.0, 0.6, 0.8, 0.0, 0.05)
    occ = m.occupied()
    assert gone == 2 and not occ[m.cell(1.0, 1.0)] and not occ[m.cell(1.0, 1.3)]
    assert occ[m.cell(1.45, 1.0)] and occ[m.cell(1.40, 1.0)]  # the box and its spread stay


def test_a_new_raw_slam_cell_inside_an_old_grown_ring_is_new_evidence():
    # Codex review P1: dating by the grown mask hid a new occupancy in the old ring.
    m = memory()
    data = np.full((40, 40), -1, dtype=np.int8)
    data[20, 20] = SLAM_OCCUPIED
    m.set_slam(data, (0.0, 0.0), 0.05, stamp_s=10.0, index=1)
    xn, yn = 0.05 * 21 + 0.025, 0.05 * 20 + 0.025  # the neighbour in the old ring
    m.add_clears(np.array([xn]), np.array([yn]), np.array([12.0]))
    i, j = m.cell(xn, yn)
    assert not m.occupied()[i, j]
    data[20, 21] = SLAM_OCCUPIED
    m.set_slam(data, (0.0, 0.0), 0.05, stamp_s=20.0, index=2)
    assert m.occupied()[i, j]


def test_a_map_read_late_keeps_its_own_date():
    # Codex review P1: stamped 10, a clear at 12, read at 13 -> the clear wins.
    m = memory()
    x, y = 1.0 + 0.025, 1.0 + 0.025
    data = np.full((40, 40), -1, dtype=np.int8)
    data[20, 20] = SLAM_OCCUPIED
    m.add_clears(np.array([x]), np.array([y]), np.array([12.0]))
    m.set_slam(data, (0.0, 0.0), 0.05, stamp_s=10.0, index=1)
    assert not m.occupied()[m.cell(x, y)]


def test_a_new_source_square_hidden_by_its_neighbours_projection_is_still_new_evidence():
    # Codex re-review 3 P1: origin 0.025 m off the planning grid, a source row
    # [100, -1, 100] -> [100, 100, 100] projected onto the same planning cells.
    m = memory()
    data = np.full((40, 40), -1, dtype=np.int8)
    data[20, 19] = data[20, 21] = SLAM_OCCUPIED
    m.set_slam(data, (0.025, 0.0), 0.05, stamp_s=10.0, index=1)
    xm, ym = 0.025 + 20.5 * 0.05, 20.5 * 0.05  # the middle square's centre
    i, j = m.cell(xm, ym)
    before = m.slam.copy()
    m.add_clears(np.array([xm]), np.array([ym]), np.array([12.0]))
    assert not m.occupied()[i, j]
    data[20, 20] = SLAM_OCCUPIED
    m.set_slam(data, (0.025, 0.0), 0.05, stamp_s=20.0, index=2)
    assert (m.slam == before).all()  # the projection alone cannot tell
    assert m.occupied()[i, j]
    # The neighbours keep their own date when the map grows and its origin moves.
    grown = np.full((42, 41), -1, dtype=np.int8)
    grown[22, 20:23] = SLAM_OCCUPIED
    m.set_slam(grown, (-0.025, -0.1), 0.05, stamp_s=30.0, index=3)
    assert m.slam_since[i, j] == 20.0


def test_an_endpoint_just_outside_the_retraction_stays_even_when_its_cell_centre_is_inside():
    # Codex re-review 3 P1: pallet front 1.530, retraction edge 1.580, a box's
    # endpoint at 1.592 in the cell centred 1.575.
    m = PlanningMemory(0.0, 0.0, 2.0, 2.0, 0.05, hit_radius_m=0.1)
    m.add_hits(np.array([1.5, 1.592]), np.array([1.0, 1.0]), 0.0)
    gone = m.retract_endpoints(1.23, 1.0, 0.6, 0.8, 0.0, 0.05)
    assert gone == 1
    assert m.occupied()[m.cell(1.592, 1.0)] and m.endpoint[m.cell(1.592, 1.0)] == 0.0
    assert not np.isfinite(m.endpoint[m.cell(1.5, 1.0)])
