import numpy as np
import pytest

from forklift_core.planning.fleet_traffic import overlap_matrix
from forklift_core.planning.geometry import Footprint
from forklift_core.planning.pallet_mission import SyntheticMissionGeometry
from forklift_core.planning.world_gen import make_world
from forklift_core.planning.world_people import (
    FIELD_EXTRA_M,
    GIVE_WAY_S,
    PERSON_RADIUS,
    STOP_BEFORE_M,
    People,
    protective_block_m,
)

TRUCK = Footprint(1.29, 0.17, 0.36)


def test_a_person_ahead_shortens_the_distance_a_truck_may_drive():
    pose = (0.0, 0.0, 0.0)
    # At 1 m/s the field reaches 1/(2*0.3) + FIELD_EXTRA_M = 2.67 m past the nose.
    ahead = protective_block_m(pose, TRUCK, 1, 1.0, 0.3, [[3.0, 0.2]])
    assert ahead == pytest.approx(3.0 - 1.29 - PERSON_RADIUS - STOP_BEFORE_M)
    # Beyond the field (stopping distance 0 + FIELD_EXTRA_M) nothing happens.
    far = 1.29 + FIELD_EXTRA_M + PERSON_RADIUS + 0.1
    assert protective_block_m(pose, TRUCK, 1, 0.0, 0.3, [[far, 0.0]]) is None
    # Faster means a longer field.
    assert protective_block_m(pose, TRUCK, 1, 1.0, 0.3, [[far, 0.0]]) is not None
    # Off to the side, or behind a truck driving forwards: no stop.
    assert protective_block_m(pose, TRUCK, 1, 0.0, 0.3, [[2.0, 1.5]]) is None
    assert protective_block_m(pose, TRUCK, 1, 0.0, 0.3, [[-1.0, 0.0]]) is None


def test_reversing_looks_behind_from_the_rear():
    pose = (0.0, 0.0, 0.0)
    behind = protective_block_m(pose, TRUCK, -1, 0.3, 0.3, [[-1.5, 0.0]])
    assert behind == pytest.approx(1.5 - 0.17 - PERSON_RADIUS - STOP_BEFORE_M)
    assert protective_block_m(pose, TRUCK, -1, 0.3, 0.3, [[1.5, 0.0]]) is None


def test_people_keep_to_free_floor_and_wait_for_a_truck_ahead():
    geometry = SyntheticMissionGeometry()
    world = make_world("shop", 100, geometry=geometry, trucks=5)
    people = People(world, geometry, 6, np.random.default_rng(1))
    origin = people.origin
    for _ in range(600):
        people.step(0.1, np.zeros((0, 6)))
        cells = ((people.pos - origin) // people.grid).astype(int)
        cells = np.clip(cells, 0, np.array(people.free.shape) - 1)
        assert people.free[cells[:, 0], cells[:, 1]].all()
    assert people.walked_m > 100.0
    # A truck right in front of a walker holds it in place.
    people.paths[0] = [people.pos[0] + np.array([5.0, 0.0])]
    before = people.pos[0].copy()
    blocker = np.array([[before[0] + 1.0, before[1], 1.0, 0.5, 1.0, 0.0]])
    people.step(0.1, blocker)
    assert np.allclose(people.pos[0], before)


def test_a_person_held_by_a_truck_gives_way_round_it():
    geometry = SyntheticMissionGeometry()
    world = make_world("shop", 100, geometry=geometry, trucks=5)
    people = People(world, geometry, 1, np.random.default_rng(3))
    # A walker heading into a truck parked across its way (the truck waits for it).
    for _ in range(50):
        people.step(0.1, np.zeros((0, 6)))
    people.paths[0] = [people.pos[0] + np.array([5.0, 0.0])]
    start = people.pos[0].copy()
    truck = np.array([[start[0] + 1.6, start[1], 0.73, 0.36, 0.0, 1.0]])
    for _ in range(int(GIVE_WAY_S / 0.1)):
        people.step(0.1, truck)
    assert np.allclose(people.pos[0], start)
    for _ in range(60):
        people.step(0.1, truck)
        # Never into the truck's body.
        disc = np.array([[*people.pos[0], PERSON_RADIUS, PERSON_RADIUS, 1.0, 0.0]])
        assert not overlap_matrix(disc, truck).any()
    assert np.hypot(*(people.pos[0] - start)) > 1.0
