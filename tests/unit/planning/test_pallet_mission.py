"""Synthetic mission orchestration; physical handling is an adapter test."""

import math
from dataclasses import asdict

import numpy as np
import pytest

from forklift_core.planning import (
    Bounds,
    Footprint,
    PlannerConfig,
    Pose2D,
    Rectangle,
    collision_free_path,
    collision_free_pose,
)
from forklift_core.planning.pallet_mission import (
    AssetSpec,
    PalletSite,
    PlacedProp,
    SyntheticMissionGeometry,
    TransportScenario,
    make_scenario,
    plan_transport,
    site_poses,
)

ASSETS = (
    AssetSpec("test://barrel.usd", 0.600, 0.708, 0.901),
    AssetSpec("test://crate.usd", 0.450, 0.662, 0.188),
    AssetSpec("test://cardbox.usd", 0.797, 0.637, 0.503),
)


def test_default_clearance_blocks_a_near_margin_delivery_straight_post():
    geometry = SyntheticMissionGeometry()
    lateral = (
        geometry.loaded_footprint.half_width_m + 0.025 + 0.05
    )  # prop half-width + surface gap
    scenario = TransportScenario(
        0,
        Pose2D(-2.34, 0, 0),
        PalletSite(3.4, 0, 0),
        PalletSite(0.2, 1, 0),
        (
            PlacedProp(
                AssetSpec("test://post", 0.05, 0.05, 1.0),
                Rectangle(0.0, 1 + lateral, 0.05, 0.05),
            ),
        ),
        Bounds(-3, 4.7, -1.75, 3.05),
    )
    # First assert the geometric premise: margin 0 clears, margin 0.10 does not.
    default = plan_transport(scenario)
    assert not default.success
    assert default.status == "transport:straight_collision"
    zero_clearance = plan_transport(scenario, PlannerConfig(clearance_m=0.00))
    assert zero_clearance.success


def test_approach_clearance_adapts_to_a_smaller_stopping_gap():
    geometry = SyntheticMissionGeometry(approach_gap_m=0.04)
    scenario = TransportScenario(
        0,
        Pose2D(-2.34, 0, 0),
        PalletSite(3.4, 0, 0),
        PalletSite(0.2, 1, 0),
        (),
        Bounds(-3, 4.7, -1.75, 3.05),
    )
    result = plan_transport(scenario, geometry=geometry)
    assert result.success, result.status
    pickup = Rectangle(3.4, 0, geometry.pallet_depth_m, geometry.pallet_width_m)
    assert collision_free_path(
        result.approach.poses,
        [pickup],
        geometry.unloaded_footprint,
        scenario.bounds,
        margin_m=0.02,
    )


def test_t11_derived_geometry_and_serialization():
    geometry = SyntheticMissionGeometry(pallet_depth_m=0.66, pallet_width_m=0.66)
    expected = {
        "inserted_offset_m": 1.26,
        "approach_offset_m": 1.72,
        "prealign_offset_m": 2.52,
        "predelivery_offset_m": 1.96,
    }
    serialized = asdict(geometry)
    for name, value in expected.items():
        assert getattr(geometry, name) == pytest.approx(value)
        assert name in serialized
        assert serialized[name] == pytest.approx(value)
    assert geometry.loaded_footprint.front_m == pytest.approx(1.59)
    assert geometry.loaded_footprint.half_width_m == 0.36
    assert serialized["loaded_footprint"] == pytest.approx(
        {"front_m": 1.59, "rear_m": 0.17, "half_width_m": 0.36}
    )


def test_t11_loaded_envelope_collision_counterexample():
    pose = Pose2D(0, 0, 0)
    obstacles = [Rectangle(1.57, 0, 0.010, 0.010)]
    bounds = Bounds(-3, 4.7, -1.75, 3.05)
    epal = SyntheticMissionGeometry()
    t11 = SyntheticMissionGeometry(pallet_depth_m=0.66, pallet_width_m=0.66)
    assert collision_free_pose(
        pose, obstacles, epal.loaded_footprint, bounds, margin_m=0
    )
    assert not collision_free_pose(
        pose, obstacles, t11.loaded_footprint, bounds, margin_m=0
    )


def test_fork_tip_and_unloaded_envelope_are_independent():
    original = SyntheticMissionGeometry(axle_to_fork_tip_m=1.29)
    enlarged = SyntheticMissionGeometry(
        axle_to_fork_tip_m=1.29, unloaded_footprint=Footprint(1.80, 0.17, 0.36)
    )
    shifted = SyntheticMissionGeometry(axle_to_fork_tip_m=1.49)
    assert enlarged.inserted_offset_m == original.inserted_offset_m
    assert enlarged.approach_offset_m == original.approach_offset_m
    assert shifted.inserted_offset_m - original.inserted_offset_m == pytest.approx(0.20)
    assert shifted.approach_offset_m - original.approach_offset_m == pytest.approx(0.20)
    assert enlarged.loaded_footprint.front_m == 1.80


@pytest.mark.parametrize("steering_rad", [-0.45, 0.45])
@pytest.mark.parametrize("side", [-1, 1])
def test_unloaded_footprint_covers_steered_tire_corners(steering_rad, side):
    # Provisional tire centres are +/-0.255 m, with radius 0.135 m and
    # half-width 0.05 m. The outside corner reaches 0.358743 m at full steer.
    local_x = side * math.copysign(0.135, math.sin(steering_rad))
    local_y = side * 0.05
    corner_x = (
        0.64 + local_x * math.cos(steering_rad) - local_y * math.sin(steering_rad)
    )
    corner_y = (
        side * 0.255
        + local_x * math.sin(steering_rad)
        + local_y * math.cos(steering_rad)
    )
    obstacle = Rectangle(corner_x, corner_y, 0.001, 0.001)
    assert not collision_free_pose(
        Pose2D(0, 0, 0),
        [obstacle],
        SyntheticMissionGeometry().unloaded_footprint,
        Bounds(-3, 4.7, -1.75, 3.05),
    )


def test_seeded_spawn_is_reproducible_and_randomizes_sites_and_assets():
    first = make_scenario(12, ASSETS)
    assert first == make_scenario(12, ASSETS)
    assert first != make_scenario(13, ASSETS)
    assert len(first.props) == 4
    assert all(prop.asset in ASSETS for prop in first.props)
    assert len({prop.asset.uri for prop in first.props}) > 1
    for index, prop in enumerate(first.props):
        rectangle = prop.rectangle
        assert collision_free_pose(
            Pose2D(rectangle.x_m, rectangle.y_m, rectangle.yaw_rad),
            [other.rectangle for other in first.props[index + 1 :]],
            Footprint(
                rectangle.length_m / 2, rectangle.length_m / 2, rectangle.width_m / 2
            ),
            first.bounds,
        )


def test_site_offsets_are_in_site_heading_frame():
    poses = site_poses(PalletSite(2, 3, math.pi / 2))
    np.testing.assert_allclose(
        [poses["inserted"].x_m, poses["inserted"].y_m],
        [2, 1.77],
        atol=1e-12,
    )
    assert poses["approach"].y_m == pytest.approx(1.31)
    assert poses["prealign"].y_m == pytest.approx(0.51)
    assert poses["extracted"].y_m == pytest.approx(1.12)
    assert poses["predelivery"].y_m == pytest.approx(1.07)
    assert poses["withdrawn"].y_m == pytest.approx(1.22)


@pytest.mark.parametrize("seed", [0, 7, 12])
def test_spawn_preserves_docking_and_loaded_extraction_corridors(seed):
    scenario = make_scenario(seed, ASSETS)
    obstacles = [prop.rectangle for prop in scenario.props]
    geometry = SyntheticMissionGeometry()
    for site, first_name, last_name, footprint in [
        (scenario.pickup, "prealign", "inserted", geometry.unloaded_footprint),
        (scenario.pickup, "inserted", "extracted", geometry.loaded_footprint),
        (scenario.destination, "predelivery", "delivery", geometry.loaded_footprint),
        (scenario.destination, "delivery", "withdrawn", geometry.unloaded_footprint),
    ]:
        poses = site_poses(site)
        path = np.array(
            [
                [poses[name].x_m, poses[name].y_m, poses[name].yaw_rad]
                for name in (first_name, last_name)
            ]
        )
        assert collision_free_path(
            path, obstacles, footprint, scenario.bounds, margin_m=0.10
        )


@pytest.mark.parametrize("seed", [0, 17])
def test_complete_seeded_plan_has_exact_straights_and_loaded_clearance(seed):
    scenario = make_scenario(seed, ASSETS)
    geometry = SyntheticMissionGeometry()
    result = plan_transport(scenario)
    assert result.success, result.status
    pickup, destination = site_poses(scenario.pickup), site_poses(scenario.destination)
    expected_lengths = {"insert": 0.46, "extract": 0.65, "withdraw": 0.55}
    for name, expected_length in expected_lengths.items():
        path = getattr(result, name)
        assert path.length_m == pytest.approx(expected_length)
        assert np.all(path.curvatures_inv_m == 0)
        assert (
            np.max(np.linalg.norm(np.diff(path.poses[:, :2], axis=0), axis=1))
            <= 0.04 + 1e-12
        )
    assert np.all(result.insert.directions == 1)
    assert np.all(result.extract.directions == -1)
    assert np.all(result.withdraw.directions == -1)
    for name, goal in [
        ("approach", pickup["approach"]),
        ("insert", pickup["inserted"]),
        ("extract", pickup["extracted"]),
        ("transport", destination["delivery"]),
        ("withdraw", destination["withdrawn"]),
    ]:
        path = getattr(result, name)
        np.testing.assert_allclose(
            path.poses[-1], [goal.x_m, goal.y_m, goal.yaw_rad], atol=1e-7
        )
        assert np.all(
            np.linalg.norm(np.diff(path.poses[:, :2], axis=0), axis=1) > 1e-10
        )
    for name in ("extract", "transport"):
        assert collision_free_path(
            getattr(result, name).poses,
            [prop.rectangle for prop in scenario.props],
            geometry.loaded_footprint,
            scenario.bounds,
        )
    for name, distance in [("approach", 0.8), ("transport", 0.7)]:
        path = getattr(result, name)
        tail_segments = math.ceil(distance / 0.04)
        assert np.all(path.directions[-tail_segments:] == 1)
        assert np.all(path.curvatures_inv_m[-tail_segments:] == 0)


def test_seed_seven_with_steered_tire_envelope_returns_bounded_failure():
    # Widening the unloaded half-width from 0.315 to 0.36 m changes reserved
    # spawn corridors and feasible motion. Seed 7 is retained as a failure
    # fixture, rather than claiming its earlier narrow-envelope success.
    # The fixed-seed benchmark records its full-search no_path outcome;
    # this quick unit check covers the bounded-search failure contract.
    scenario = make_scenario(7, ASSETS)
    result = plan_transport(
        scenario, PlannerConfig(clearance_m=0.10, max_expansions=100)
    )
    assert not result.success
    assert result.status == "approach:expansion_limit"
    assert all(
        getattr(result, name) is None
        for name in ("approach", "insert", "extract", "transport", "withdraw")
    )


def test_failed_stage_exposes_no_runnable_partial_paths():
    wall = PlacedProp(
        AssetSpec("test://wall", 0.2, 4.8, 2), Rectangle(0, 0.65, 0.2, 4.8)
    )
    scenario = TransportScenario(
        0,
        Pose2D(-2.34, 0, 0),
        PalletSite(3.4, 0, 0),
        PalletSite(0.2, 1, 0),
        (wall,),
        Bounds(-3, 4.7, -1.75, 3.05),
    )
    result = plan_transport(scenario, PlannerConfig(max_expansions=100))
    assert not result.success
    assert result.status.startswith("approach:")
    assert all(
        getattr(result, name) is None
        for name in ("approach", "insert", "extract", "transport", "withdraw")
    )


def test_delivery_uses_loaded_width_and_discards_earlier_successful_stages():
    # At delivery the 0.80 m load clips this prop; the 0.72 m unloaded envelope
    # clears it. An unloaded footprint accidentally used for transport fails
    # to detect the final approach collision.
    prop = PlacedProp(
        AssetSpec("test://narrow", 0.08, 0.02, 0.1), Rectangle(0, 1.38, 0.08, 0.02)
    )
    scenario = TransportScenario(
        0,
        Pose2D(-2.34, 0, 0),
        PalletSite(3.4, 0, 0),
        PalletSite(0.2, 1, 0),
        (prop,),
        Bounds(-3, 4.7, -1.75, 3.05),
    )
    geometry = SyntheticMissionGeometry()
    goal = site_poses(scenario.destination)["delivery"]
    assert collision_free_pose(
        goal, [prop.rectangle], geometry.unloaded_footprint, scenario.bounds
    )
    assert not collision_free_pose(
        goal, [prop.rectangle], geometry.loaded_footprint, scenario.bounds
    )
    result = plan_transport(scenario, PlannerConfig(clearance_m=0))
    assert not result.success
    assert result.status == "transport:straight_collision"
    assert (
        result.approach is None and result.insert is None and result.transport is None
    )


def test_impossible_spawn_fails_without_seed_replacement():
    with pytest.raises(ValueError, match="place"):
        make_scenario(0, [AssetSpec("test://huge", 20, 20, 1)], max_attempts=5)


@pytest.mark.parametrize(
    "build",
    [
        lambda: AssetSpec("", 1, 1, 1),
        lambda: AssetSpec("test://asset", 0, 1, 1),
        lambda: PalletSite(float("nan"), 0, 0),
        lambda: make_scenario(1, ASSETS, obstacle_count=-1),
    ],
)
def test_invalid_mission_inputs_raise(build):
    with pytest.raises(ValueError):
        build()
