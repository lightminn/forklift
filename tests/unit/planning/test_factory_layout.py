"""Seeded factory floor around the original transport bay; geometry only."""

import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from forklift_core.planning import (
    Bounds,
    Footprint,
    Pose2D,
    Rectangle,
    collision_free_pose,
)
from forklift_core.planning.factory_layout import (
    FactoryAssets,
    Region,
    load_factory_layout,
    make_factory_scenario,
)
from forklift_core.planning.pallet_mission import (
    AssetSpec,
    SyntheticMissionGeometry,
    make_scenario,
    site_poses,
)

LAYOUT_PATH = Path(__file__).parents[3] / "config/factory_south_hall.yaml"
BAY_ASSETS = (
    AssetSpec("test://barrel.usd", 0.600, 0.708, 0.901),
    AssetSpec("test://crate.usd", 0.450, 0.662, 0.188),
    AssetSpec("test://cardbox.usd", 0.797, 0.637, 0.503),
)
# Bounds measured 2026-09-26 from the Simple_Warehouse prop USDs.
PALLET = AssetSpec("test://SM_PaletteA_01.usd", 1.21, 1.00, 0.21)
LOADS = (
    AssetSpec("test://SM_CardBoxA_02.usd", 0.797, 0.637, 0.503),
    AssetSpec("test://SM_CardBoxC_01.usd", 0.50, 0.50, 0.25),
)
CLUTTER = (
    AssetSpec("test://SM_BarelPlastic_A_01.usd", 0.600, 0.708, 0.901),
    AssetSpec("test://SM_CratePlastic_D_01.usd", 0.450, 0.662, 0.188),
    AssetSpec("test://SM_PushcartA_02.usd", 0.94, 1.65, 0.38),
    AssetSpec("test://S_TrafficCone.usd", 0.34, 0.33, 0.46),
)
FACTORY_ASSETS = FactoryAssets(PALLET, LOADS, CLUTTER)
ORIGINAL_BAY = Bounds(-3.0, 4.7, -1.75, 3.05)


@pytest.fixture(scope="module")
def layout():
    return load_factory_layout(LAYOUT_PATH)


def build(seed, layout):
    return make_factory_scenario(seed, layout, BAY_ASSETS, FACTORY_ASSETS)


def rectangle_of(region):
    return Rectangle(
        (region.x_min_m + region.x_max_m) / 2,
        (region.y_min_m + region.y_max_m) / 2,
        region.x_max_m - region.x_min_m,
        region.y_max_m - region.y_min_m,
    )


def footprint_of(rect, margin=0.0):
    return (
        Pose2D(rect.x_m, rect.y_m, rect.yaw_rad),
        Footprint(
            rect.length_m / 2 + margin,
            rect.length_m / 2 + margin,
            rect.width_m / 2 + margin,
        ),
    )


def corners(rect):
    c, s = math.cos(rect.yaw_rad), math.sin(rect.yaw_rad)
    return [
        (
            rect.x_m + c * a * rect.length_m / 2 - s * b * rect.width_m / 2,
            rect.y_m + s * a * rect.length_m / 2 + c * b * rect.width_m / 2,
        )
        for a in (-1, 1)
        for b in (-1, 1)
    ]


def test_layout_file_holds_the_measured_hall_and_keeps_the_bay_clear(layout):
    assert layout.hall == Bounds(-25.6, 4.7, -22.9, 8.3)
    bay = Region(-3.0, 4.7, -1.75, 3.05)
    assert any(zone.contains(bay) for zone in layout.keep_clear)


@pytest.mark.parametrize("seed", [0, 3, 11])
def test_same_seed_keeps_the_original_bay_scenario(layout, seed):
    bay = make_scenario(seed, BAY_ASSETS, 4)
    factory = build(seed, layout)
    assert factory.transport.start_rear == bay.start_rear
    assert factory.transport.pickup == bay.pickup
    assert factory.transport.props[: len(bay.props)] == bay.props
    assert factory.pickup_bounds == bay.bounds == ORIGINAL_BAY
    assert factory.transport.bounds == layout.hall


def test_hall_is_filled_with_pallets_loads_and_clutter(layout):
    factory = build(0, layout)
    pallets = [p for p in factory.work_items if p.asset == PALLET]
    clutter = [p for p in factory.work_items if p.asset in CLUTTER]
    # 66 slots at occupancy 0.9; seeds 0-19 gave 54-64 pallets and 179-268
    # boxes when this test was written (2026-09-26).
    assert len(pallets) >= 50
    assert len(factory.loads) >= 150
    assert len(clutter) == sum(zone.count for zone in layout.clutter)
    assert len(factory.work_items) == len(pallets) + len(clutter)


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_items_stay_in_the_hall_apart_and_out_of_the_kept_clear_zone(layout, seed):
    factory = build(seed, layout)
    hall = layout.hall
    forbidden = [rectangle_of(zone) for zone in layout.keep_clear]
    forbidden.append(rectangle_of(Region(-3.0, 4.7, -1.75, 3.05)))
    items = [p.rectangle for p in factory.work_items]
    for index, rect in enumerate(items):
        pose, footprint = footprint_of(rect)
        assert collision_free_pose(pose, [], footprint, hall), rect
        assert collision_free_pose(pose, forbidden, footprint, hall), rect
        others = items[:index] + items[index + 1 :]
        assert collision_free_pose(pose, others, footprint, hall), rect


@pytest.mark.parametrize("seed", [0, 5])
def test_loads_sit_on_their_pallet_in_whole_layers(layout, seed):
    factory = build(seed, layout)
    pallets = [p.rectangle for p in factory.work_items if p.asset == PALLET]
    assert factory.loads
    for load in factory.loads:
        # The pallet under a load is the one its centre falls on.
        under = [
            p
            for p in pallets
            if math.hypot(p.x_m - load.rectangle.x_m, p.y_m - load.rectangle.y_m) < 0.6
        ]
        assert len(under) == 1
        pallet = under[0]
        c, s = math.cos(pallet.yaw_rad), math.sin(pallet.yaw_rad)
        for x, y in corners(load.rectangle):
            dx, dy = x - pallet.x_m, y - pallet.y_m
            assert abs(dx * c + dy * s) <= pallet.length_m / 2 + 1e-9
            assert abs(-dx * s + dy * c) <= pallet.width_m / 2 + 1e-9
        layer = (load.base_height_m - PALLET.height_m) / load.asset.height_m
        assert layer == pytest.approx(round(layer), abs=1e-9)
        assert round(layer) >= 0
        top = load.base_height_m + load.asset.height_m
        assert top <= PALLET.height_m + layout.load_height_m[1] + 1e-9


@pytest.mark.parametrize("seed", [0, 1, 2, 3])
def test_destination_is_a_dock_with_a_clear_delivery_corridor(layout, seed):
    factory = build(seed, layout)
    destination = factory.transport.destination
    nearest = min(
        layout.docks,
        key=lambda d: math.hypot(d.x_m - destination.x_m, d.y_m - destination.y_m),
    )
    assert (
        math.hypot(nearest.x_m - destination.x_m, nearest.y_m - destination.y_m)
        <= layout.dock_jitter_m + 1e-9
    )
    assert abs(destination.yaw_rad - nearest.yaw_rad) <= layout.dock_yaw_jitter_rad
    geometry = SyntheticMissionGeometry()
    poses = site_poses(destination, geometry)
    obstacles = [p.rectangle for p in factory.transport.props]
    for name, footprint in [
        ("predelivery", geometry.loaded_footprint),
        ("delivery", geometry.loaded_footprint),
        ("withdrawn", geometry.unloaded_footprint),
    ]:
        assert collision_free_pose(
            poses[name], obstacles, footprint, layout.hall, margin_m=0.1
        ), name


def test_seeded_factory_is_reproducible_and_varies_by_seed(layout):
    first, again, other = build(4, layout), build(4, layout), build(5, layout)
    assert first == again
    assert first.work_items != other.work_items


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4])
def test_survey_waypoints_are_free_for_the_unloaded_truck(layout, seed):
    factory = build(seed, layout)
    geometry = SyntheticMissionGeometry()
    obstacles = [p.rectangle for p in factory.transport.props]
    for pose in layout.survey_route:
        assert collision_free_pose(
            pose, obstacles, geometry.unloaded_footprint, layout.hall, margin_m=0.1
        ), pose


def test_a_load_wider_than_the_pallet_is_rejected(layout):
    wide = AssetSpec("test://wide.usd", 1.5, 1.2, 0.3)
    with pytest.raises(ValueError, match="load"):
        make_factory_scenario(
            0, layout, BAY_ASSETS, FactoryAssets(PALLET, (wide,), CLUTTER)
        )


def test_storage_inside_the_kept_clear_zone_is_rejected(layout):
    with pytest.raises(ValueError, match="keep_clear"):
        replace(layout, storage=layout.storage + (Region(0.0, 2.0, 0.0, 2.0),))


def test_region_outside_the_hall_is_rejected(layout):
    with pytest.raises(ValueError, match="hall"):
        replace(layout, storage=(Region(-30.0, -24.0, 0.0, 2.6),))


def test_unknown_and_missing_keys_are_rejected(tmp_path):
    text = LAYOUT_PATH.read_text(encoding="utf-8")
    unknown = tmp_path / "unknown.yaml"
    unknown.write_text(text + "\nsurprise: 1\n", encoding="utf-8")
    with pytest.raises(ValueError, match="surprise"):
        load_factory_layout(unknown)
    missing = tmp_path / "missing.yaml"
    missing.write_text(text.replace("slot_occupancy: 0.9\n", ""), encoding="utf-8")
    with pytest.raises(ValueError, match="slot_occupancy"):
        load_factory_layout(missing)


def test_impossible_clutter_count_fails_without_silent_reduction(layout):
    crowded = replace(
        layout,
        clutter=(replace(layout.clutter[0], count=500),),
    )
    with pytest.raises(ValueError, match="clutter"):
        make_factory_scenario(0, crowded, BAY_ASSETS, FACTORY_ASSETS, max_attempts=300)


def test_survey_route_joins_every_waypoint_without_collision(layout):
    from forklift_core.planning import collision_free_path
    from forklift_core.planning.factory_layout import plan_survey_route
    from forklift_core.planning.pallet_mission import make_transport_planner_config

    factory = build(0, layout)
    geometry = SyntheticMissionGeometry()
    config = replace(
        make_transport_planner_config(max_expansions=30000),
        obstacle_heuristic_resolution_m=0.25,
    )
    result = plan_survey_route(factory, layout.survey_route, config, geometry=geometry)
    assert result.success, result.status
    obstacles = [p.rectangle for p in factory.transport.props]
    assert collision_free_path(
        result.poses, obstacles, geometry.unloaded_footprint, layout.hall
    )
    # Every waypoint is visited exactly, in order.
    index = 0
    for waypoint in layout.survey_route:
        hits = [
            i
            for i in range(index, len(result.poses))
            if math.hypot(
                result.poses[i, 0] - waypoint.x_m, result.poses[i, 1] - waypoint.y_m
            )
            < 1e-7
        ]
        assert hits, waypoint
        index = hits[0]
    first = layout.survey_route[0]
    np.testing.assert_allclose(result.poses[0, :2], [first.x_m, first.y_m])
    heading_error = result.poses[0, 2] - first.yaw_rad
    assert math.cos(heading_error) == pytest.approx(1.0)
    assert len(result.directions) == len(result.poses)
    assert result.length_m > 80


def test_survey_route_reports_the_leg_that_failed(layout):
    from forklift_core.planning.factory_layout import plan_survey_route
    from forklift_core.planning.pallet_mission import make_transport_planner_config

    factory = build(0, layout)
    # The second waypoint sits on a stored pallet.
    blocked = factory.work_items[0].rectangle
    route = (layout.survey_route[0], Pose2D(blocked.x_m, blocked.y_m, 0.0))
    result = plan_survey_route(factory, route, make_transport_planner_config())
    assert not result.success
    assert result.status == "leg0:invalid_goal"
    assert len(result.poses) == 0
