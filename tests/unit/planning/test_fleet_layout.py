"""Five trucks on the whole warehouse: assignments and per-truck obstacles; geometry only."""

from pathlib import Path

import pytest

from forklift_core.planning.factory_layout import FactoryAssets
from forklift_core.planning.fleet_layout import load_fleet_layout, make_fleet_scenario
from forklift_core.planning.fleet_traffic import footprint_rectangle, rectangles_overlap
from forklift_core.planning.geometry import Pose2D, collision_free_pose
from forklift_core.planning.pallet_mission import (
    AssetSpec,
    SyntheticMissionGeometry,
    site_poses,
)

LAYOUT_PATH = Path(__file__).parents[3] / "config/fleet_full_warehouse.yaml"
BAY_ASSETS = (
    AssetSpec("test://barrel.usd", 0.600, 0.708, 0.901),
    AssetSpec("test://crate.usd", 0.450, 0.662, 0.188),
    AssetSpec("test://cardbox.usd", 0.797, 0.637, 0.503),
)
FACTORY_ASSETS = FactoryAssets(
    AssetSpec("test://SM_PaletteA_01.usd", 1.21, 1.00, 0.21),
    (
        AssetSpec("test://SM_CardBoxA_02.usd", 0.797, 0.637, 0.503),
        AssetSpec("test://SM_CardBoxC_01.usd", 0.50, 0.50, 0.25),
    ),
    (
        AssetSpec("test://SM_BarelPlastic_A_01.usd", 0.600, 0.708, 0.901),
        AssetSpec("test://SM_CratePlastic_D_01.usd", 0.450, 0.662, 0.188),
        AssetSpec("test://SM_PushcartA_02.usd", 0.94, 1.65, 0.38),
        AssetSpec("test://S_TrafficCone.usd", 0.34, 0.33, 0.46),
    ),
)
GEOMETRY = SyntheticMissionGeometry()


@pytest.fixture(scope="module")
def layout():
    return load_fleet_layout(LAYOUT_PATH)


@pytest.mark.parametrize("seed", [0, 1, 7])
def test_every_truck_gets_its_own_aisle_dock_and_parking(layout, seed):
    fleet = make_fleet_scenario(
        seed, layout, BAY_ASSETS, FACTORY_ASSETS, geometry=GEOMETRY
    )
    assert len(fleet.missions) == 5
    assert len({m.aisle for m in fleet.missions}) == 5
    assert (
        len(
            {
                (round(m.destination.x_m), round(m.destination.y_m))
                for m in fleet.missions
            }
        )
        == 5
    )
    assert len({(m.parking.x_m, m.parking.y_m) for m in fleet.missions}) == 5


def test_assignment_is_seeded(layout):
    a = make_fleet_scenario(3, layout, BAY_ASSETS, FACTORY_ASSETS, geometry=GEOMETRY)
    b = make_fleet_scenario(3, layout, BAY_ASSETS, FACTORY_ASSETS, geometry=GEOMETRY)
    c = make_fleet_scenario(4, layout, BAY_ASSETS, FACTORY_ASSETS, geometry=GEOMETRY)
    assert [m.aisle for m in a.missions] == [m.aisle for m in b.missions]
    assert a.missions[0].destination == b.missions[0].destination
    assert [m.aisle for m in a.missions] != [m.aisle for m in c.missions] or (
        a.missions[0].destination != c.missions[0].destination
    )


def test_only_the_own_aisle_is_open_and_the_mission_poses_are_free(layout):
    fleet = make_fleet_scenario(
        0, layout, BAY_ASSETS, FACTORY_ASSETS, geometry=GEOMETRY
    )
    footprint = GEOMETRY.unloaded_footprint
    for mission in fleet.missions:
        rects = [p.rectangle for p in mission.scenario.props]
        assert collision_free_pose(mission.parking, rects, footprint, fleet.bounds)
        assert collision_free_pose(mission.aisle_entry, rects, footprint, fleet.bounds)
        poses = site_poses(mission.pickup, GEOMETRY)
        assert collision_free_pose(
            poses["prealign"], rects, footprint, mission.pickup_bounds
        )
        for other in fleet.missions:
            if other is mission:
                continue
            # Another truck's aisle mouth is closed to this truck.
            inside = Pose2D(other.aisle_entry.x_m, 12.0, other.aisle_entry.yaw_rad)
            assert not collision_free_pose(inside, rects, footprint, fleet.bounds)
        own = Pose2D(mission.aisle_entry.x_m, 12.0, mission.aisle_entry.yaw_rad)
        assert collision_free_pose(own, rects, footprint, fleet.bounds)


def test_parked_trucks_are_grown_by_the_traffic_gap(layout):
    fleet = make_fleet_scenario(
        0, layout, BAY_ASSETS, FACTORY_ASSETS, geometry=GEOMETRY, parked_gap_m=0.6
    )
    footprint = GEOMETRY.unloaded_footprint
    for mission in fleet.missions:
        parked = [
            p.rectangle
            for p in mission.scenario.props
            if p.asset.uri == "rule:parking"
        ]
        assert len(parked) == 4
        assert all(
            r.length_m == pytest.approx(footprint.front_m + footprint.rear_m + 1.2)
            for r in parked
        )
        # Its own place and the straight run out of and into it stay free.
        rects = [p.rectangle for p in mission.scenario.props]
        for pose in (mission.parking, mission.parking_exit, mission.parking_entry):
            assert collision_free_pose(pose, rects, footprint, fleet.bounds)


def test_parking_places_do_not_overlap(layout):
    footprint = GEOMETRY.unloaded_footprint
    bodies = [footprint_rectangle(p, footprint, 0.3) for p in layout.parking]
    for i, a in enumerate(bodies):
        for b in bodies[i + 1 :]:
            assert not rectangles_overlap(a, b)


def test_too_many_trucks_is_refused(layout):
    with pytest.raises(ValueError, match="more trucks"):
        make_fleet_scenario(
            0, layout, BAY_ASSETS, FACTORY_ASSETS, geometry=GEOMETRY, trucks=6
        )
