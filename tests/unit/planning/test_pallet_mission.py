"""Synthetic mission orchestration; physical handling is an adapter test."""

import math
from dataclasses import asdict, replace

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
    pallet_mission,
)
from forklift_core.planning.pallet_mission import (
    AssetSpec,
    PalletSite,
    PlacedProp,
    SyntheticMissionGeometry,
    TransportScenario,
    make_scenario,
    make_transport_planner_config,
    plan_transport,
    site_poses,
)

ASSETS = (
    AssetSpec("test://barrel.usd", 0.600, 0.708, 0.901),
    AssetSpec("test://crate.usd", 0.450, 0.662, 0.188),
    AssetSpec("test://cardbox.usd", 0.797, 0.637, 0.503),
)


@pytest.fixture
def observation_scenario():
    return TransportScenario(
        0,
        Pose2D(-3, 0, 0),
        PalletSite(4, 3, 0),
        PalletSite(4, -3, 0),
        (),
        Bounds(-5, 6, -4, 4),
    )


def test_observation_leg_reaches_independent_waypoint(observation_scenario):
    result = pallet_mission.plan_observation_leg(observation_scenario, Pose2D(2, 0, 0))
    assert result.success, result.status
    np.testing.assert_allclose(result.poses[0], [-3, 0, 0], atol=1e-7)
    np.testing.assert_allclose(result.poses[-1], [2, 0, 0], atol=1e-7)


def test_observation_leg_optional_start_preserves_default_plan(observation_scenario):
    waypoint = Pose2D(2, 0, 0)
    baseline = asdict(
        pallet_mission.plan_observation_leg(observation_scenario, waypoint)
    )
    for start in (None, observation_scenario.start_rear):
        result = pallet_mission.plan_observation_leg(
            observation_scenario, waypoint, start_rear=start
        )
        for field_name, expected in baseline.items():
            np.testing.assert_array_equal(asdict(result)[field_name], expected)


def test_observation_leg_starts_at_measured_rear_pose(observation_scenario):
    result = pallet_mission.plan_observation_leg(
        observation_scenario, Pose2D(2, 0, 0), start_rear=Pose2D(0, 0, 0)
    )
    assert result.success, result.status
    np.testing.assert_allclose(result.poses[0], [0, 0, 0], atol=1e-7)
    np.testing.assert_allclose(result.poses[-1], [2, 0, 0], atol=1e-7)


def test_observation_leg_rejects_waypoint_inside_real_pallet(observation_scenario):
    result = pallet_mission.plan_observation_leg(observation_scenario, Pose2D(4, 3, 0))
    assert not result.success
    assert result.status == "invalid_goal"


@pytest.mark.parametrize(
    "config",
    [
        None,
        PlannerConfig(primitive_length_m=0.25, clearance_m=0.15),
        PlannerConfig(primitive_length_m=0.5, clearance_m=0.15),
    ],
)
def test_observation_leg_avoids_props(observation_scenario, config):
    # The lower face at y=0.30 still overlaps the straight truck's y=0.36
    # envelope. This placement admits a detour with both primitive lengths.
    obstacle = Rectangle(0, 0.45, 0.3, 0.3)
    scenario = replace(
        observation_scenario,
        props=(PlacedProp(AssetSpec("test://box", 0.3, 0.3, 1), obstacle),),
    )
    geometry = SyntheticMissionGeometry()
    result = pallet_mission.plan_observation_leg(
        scenario, Pose2D(2, 0, 0), config, geometry=geometry
    )
    assert result.success, result.status
    assert not collision_free_path(
        np.array([[-3, 0, 0], [2, 0, 0]]),
        [obstacle],
        geometry.unloaded_footprint,
        scenario.bounds,
    )
    assert collision_free_path(
        result.poses,
        [obstacle, Rectangle(4, 3, 0.6, 0.8)],
        geometry.unloaded_footprint,
        scenario.bounds,
        margin_m=0.10 if config is None else config.clearance_m,
    )


def test_observation_default_matches_quarter_metre_plan(observation_scenario):
    scenario = replace(
        observation_scenario,
        props=(
            PlacedProp(
                AssetSpec("test://box", 0.3, 0.3, 1),
                Rectangle(0, 0.45, 0.3, 0.3),
            ),
        ),
    )
    waypoint = Pose2D(2, 0, 0)
    expected = pallet_mission.plan_observation_leg(
        scenario, waypoint, PlannerConfig(primitive_length_m=0.25, clearance_m=0.10)
    )
    actual = pallet_mission.plan_observation_leg(scenario, waypoint)
    assert expected.success and actual.success
    for name, value in asdict(expected).items():
        np.testing.assert_array_equal(asdict(actual)[name], value)


def test_transport_default_matches_quarter_metre_plan():
    scenario = make_scenario(0, ASSETS)
    expected = plan_transport(
        scenario, PlannerConfig(primitive_length_m=0.25, clearance_m=0.10)
    )
    actual = plan_transport(scenario)
    assert expected.success and actual.success
    for stage in ("approach", "insert", "extract", "transport", "withdraw"):
        for name, value in asdict(getattr(expected, stage)).items():
            np.testing.assert_array_equal(asdict(getattr(actual, stage))[name], value)


def test_explicit_planner_config_is_preserved(monkeypatch, observation_scenario):
    # Capture the real planner boundary, still executing its collision/search code.
    received = []
    real_plan = pallet_mission.plan_hybrid_astar

    def capture(*args):
        received.append(args[-1])
        return real_plan(*args)

    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", capture)
    config = PlannerConfig(primitive_length_m=0.5, clearance_m=0.12, max_expansions=321)
    result = pallet_mission.plan_observation_leg(
        observation_scenario, Pose2D(2, 0, 0), config
    )
    assert result.success
    assert received.pop() is config
    scenario = TransportScenario(
        0,
        Pose2D(-2.34, 0, 0),
        PalletSite(0, 0, 0),
        PalletSite(3.4, 0, 0),
        (),
        Bounds(-3, 4.7, -1.75, 3.05),
    )
    result = plan_transport(scenario, config)
    assert result.success, result.status
    approach_config, transport_config = received
    assert approach_config == replace(config, clearance_m=0.05)
    assert transport_config is config


def test_transport_config_factory_preserves_general_defaults_and_overrides():
    assert PlannerConfig().primitive_length_m == 0.5
    assert make_transport_planner_config() == PlannerConfig(
        primitive_length_m=0.25, clearance_m=0.10
    )
    assert make_transport_planner_config(
        primitive_length_m=0.5,
        clearance_m=0.03,
        max_expansions=30000,
        xy_resolution_m=0.1,
        reverse_penalty=1.5,
    ) == PlannerConfig(
        primitive_length_m=0.5,
        clearance_m=0.03,
        max_expansions=30000,
        xy_resolution_m=0.1,
        reverse_penalty=1.5,
    )


@pytest.mark.parametrize("config", [None, PlannerConfig(clearance_m=0.2)])
def test_observation_leg_preserves_full_clearance(observation_scenario, config):
    # Fork tip at x=3.29 leaves 0.07 m to the pallet face at x=3.36.
    scenario = replace(observation_scenario, pickup=PalletSite(3.66, 0, 0))
    result = pallet_mission.plan_observation_leg(scenario, Pose2D(2, 0, 0), config)
    assert not result.success
    assert result.status == "invalid_goal"


@pytest.mark.parametrize("seed", [0, 17])
def test_optional_pickup_and_start_preserve_the_complete_default_plan(seed):
    scenario = make_scenario(seed, ASSETS)
    baseline = plan_transport(scenario)
    assert baseline.success, baseline.status
    for overrides in (
        {"target_pickup": None, "start_rear": None},
        {"target_pickup": scenario.pickup, "start_rear": scenario.start_rear},
    ):
        result = plan_transport(scenario, **overrides)
        assert result.success == baseline.success
        assert result.status == baseline.status
        for name in ("approach", "insert", "extract", "transport", "withdraw"):
            expected = asdict(getattr(baseline, name))
            actual = asdict(getattr(result, name))
            for field_name in expected:
                np.testing.assert_array_equal(actual[field_name], expected[field_name])


def test_target_pickup_changes_waypoints_but_keeps_ground_truth_obstacle():
    geometry = SyntheticMissionGeometry()
    scenario = TransportScenario(
        0,
        Pose2D(-2.34, 0, 0),
        PalletSite(3.4, 0, 0),
        PalletSite(0.2, 1, 0),
        (),
        Bounds(-3, 4.7, -1.75, 3.05),
    )
    target = PalletSite(3.4, 1, 0)
    baseline = plan_transport(scenario)
    result = plan_transport(scenario, target_pickup=target)
    assert baseline.success, baseline.status
    assert result.success, result.status
    for name, x_m in (("approach", 1.71), ("insert", 2.17), ("extract", 1.52)):
        path = getattr(result, name)
        np.testing.assert_allclose(path.poses[-1], [x_m, 1, 0], atol=1e-12)
        assert not np.array_equal(path.poses[-1], getattr(baseline, name).poses[-1])
    for first, second in (
        ("approach", "insert"),
        ("insert", "extract"),
        ("extract", "transport"),
        ("transport", "withdraw"),
    ):
        np.testing.assert_array_equal(
            getattr(result, first).poses[-1], getattr(result, second).poses[0]
        )
    for name in ("transport", "withdraw"):
        np.testing.assert_array_equal(
            getattr(result, name).poses[-1], getattr(baseline, name).poses[-1]
        )
    pallet = Rectangle(3.4, 0, geometry.pallet_depth_m, geometry.pallet_width_m)
    assert collision_free_path(
        result.approach.poses,
        [pallet],
        geometry.unloaded_footprint,
        scenario.bounds,
        margin_m=0.05,
    )
    # This pose clears the estimated pallet but overlaps the real pallet.
    blocked_start = Pose2D(2.9, 0, 0)
    assert collision_free_pose(
        blocked_start,
        [Rectangle(3.4, 1, 0.6, 0.8)],
        geometry.unloaded_footprint,
        scenario.bounds,
        margin_m=0.05,
    )
    blocked = plan_transport(
        replace(scenario, start_rear=blocked_start), target_pickup=target
    )
    assert not blocked.success
    assert blocked.status == "approach:invalid_start"


def test_start_rear_replaces_approach_start_after_observation():
    scenario = make_scenario(0, ASSETS)
    observed_start = Pose2D(-2.0, 0, 0)
    result = plan_transport(scenario, start_rear=observed_start)
    assert result.success, result.status
    np.testing.assert_array_equal(result.approach.poses[0], [-2.0, 0, 0])
    baseline = plan_transport(scenario)
    assert baseline.success, baseline.status
    np.testing.assert_array_equal(baseline.approach.poses[0], [-2.34, 0, 0])
    np.testing.assert_array_equal(
        result.approach.poses[-1], baseline.approach.poses[-1]
    )


def test_default_clearance_blocks_a_near_margin_delivery_straight_post():
    geometry = SyntheticMissionGeometry()
    lateral = (
        geometry.loaded_footprint.half_width_m + 0.025 + 0.05
    )  # prop half-width + surface gap
    scenario = TransportScenario(
        0,
        Pose2D(-2.34, 0, 0),
        PalletSite(0, 0, 0),
        PalletSite(3.4, 0, 0),
        (
            PlacedProp(
                AssetSpec("test://post", 0.05, 0.05, 1.0),
                Rectangle(3.2, lateral, 0.05, 0.05),
            ),
        ),
        Bounds(-3, 4.7, -1.75, 3.05),
    )
    # Collinear sites let search reach predelivery at either primitive length;
    # only the final loaded straight approaches the post's 0.05 m surface gap.
    destination = site_poses(scenario.destination, geometry)
    obstacles = [prop.rectangle for prop in scenario.props]
    assert collision_free_pose(
        destination["predelivery"],
        obstacles,
        geometry.loaded_footprint,
        scenario.bounds,
        margin_m=0.10,
    )
    tail = np.array(
        [
            [destination[name].x_m, destination[name].y_m, destination[name].yaw_rad]
            for name in ("predelivery", "delivery")
        ]
    )
    assert collision_free_path(
        tail, obstacles, geometry.loaded_footprint, scenario.bounds, margin_m=0.00
    )
    assert not collision_free_path(
        tail, obstacles, geometry.loaded_footprint, scenario.bounds, margin_m=0.10
    )
    default = plan_transport(scenario)
    assert not default.success
    assert default.status == "transport:straight_collision"
    zero_clearance = plan_transport(
        scenario, PlannerConfig(primitive_length_m=0.25, clearance_m=0.00)
    )
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
