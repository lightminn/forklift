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
    # Reserve 16 mm since 2026-10-08 (ADR 0004 D3 amendment): T11 inserts 0.39 m.
    expected = {
        "inserted_offset_m": 1.23,
        "approach_offset_m": 1.72,
        "prealign_offset_m": 2.52,
        "predelivery_offset_m": 1.93,
    }
    serialized = asdict(geometry)
    for name, value in expected.items():
        assert getattr(geometry, name) == pytest.approx(value)
        assert name in serialized
        assert serialized[name] == pytest.approx(value)
    assert geometry.loaded_footprint.front_m == pytest.approx(1.56)
    assert geometry.loaded_footprint.half_width_m == 0.36
    assert serialized["loaded_footprint"] == pytest.approx(
        {"front_m": 1.56, "rear_m": 0.17, "half_width_m": 0.36}
    )


def test_t11_loaded_envelope_collision_counterexample():
    pose = Pose2D(0, 0, 0)
    obstacles = [Rectangle(1.545, 0, 0.010, 0.010)]  # between EPAL's 1.53 m and T11's 1.56 m loaded front
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


@pytest.mark.parametrize("seed", [0, 3, 5])
def test_return_leg_starts_withdrawn_and_ends_at_the_requested_pose(seed):
    scenario = make_scenario(seed, ASSETS)
    geometry = SyntheticMissionGeometry()
    result = plan_transport(scenario, return_to=scenario.start_rear)
    assert result.success, result.status
    destination = site_poses(scenario.destination)
    start = destination["withdrawn"]
    np.testing.assert_allclose(
        result.return_home.poses[0], [start.x_m, start.y_m, start.yaw_rad], atol=1e-7
    )
    np.testing.assert_allclose(
        result.return_home.poses[-1],
        [scenario.start_rear.x_m, scenario.start_rear.y_m, scenario.start_rear.yaw_rad],
        atol=1e-7,
    )
    # It leaves the delivered pallet behind, so it drives unloaded throughout.
    assert collision_free_path(
        result.return_home.poses,
        [prop.rectangle for prop in scenario.props],
        geometry.unloaded_footprint,
        scenario.bounds,
    )


def test_omitting_return_to_leaves_the_five_stage_plan_untouched():
    scenario = make_scenario(0, ASSETS)
    without = plan_transport(scenario)
    with_return = plan_transport(scenario, return_to=scenario.start_rear)
    assert without.return_home is None
    assert with_return.return_home is not None
    for name in ("approach", "insert", "extract", "transport", "withdraw"):
        np.testing.assert_array_equal(
            getattr(without, name).poses, getattr(with_return, name).poses
        )


def test_delivered_pallet_obstructs_the_return_leg():
    # Home sits on the far side of the destination, so the straight line back
    # runs through the pallet that was just set down. An empty world isolates
    # the pallet as the only thing the return leg has to avoid.
    scenario = TransportScenario(
        0,
        Pose2D(-10, 0, 0),
        PalletSite(-6, 0, 0),
        PalletSite(0, 0, 0),
        (),
        Bounds(-12, 8, -5, 5),
    )
    geometry = SyntheticMissionGeometry()
    home = Pose2D(3.5, 0, 0)
    result = plan_transport(scenario, return_to=home)
    assert result.success, result.status
    delivered = Rectangle(
        scenario.destination.x_m,
        scenario.destination.y_m,
        geometry.pallet_depth_m,
        geometry.pallet_width_m,
        scenario.destination.yaw_rad,
    )
    withdrawn = site_poses(scenario.destination)["withdrawn"]
    straight = np.array(
        [[x, 0.0, 0.0] for x in np.linspace(withdrawn.x_m, home.x_m, 200)]
    )
    # The shortcut this leg must not take.
    assert not collision_free_path(
        straight, [delivered], geometry.unloaded_footprint, scenario.bounds
    )
    assert collision_free_path(
        result.return_home.poses,
        [delivered],
        geometry.unloaded_footprint,
        scenario.bounds,
    )
    assert result.return_home.length_m > abs(home.x_m - withdrawn.x_m)


def test_unreachable_return_fails_the_whole_mission_without_partial_paths():
    scenario = make_scenario(0, ASSETS)
    result = plan_transport(
        scenario,
        PlannerConfig(clearance_m=0.10, max_expansions=100),
        return_to=Pose2D(scenario.start_rear.x_m, scenario.start_rear.y_m, 0.0),
    )
    assert not result.success
    assert result.status.startswith("return_home:")
    assert all(
        getattr(result, name) is None
        for name in ("approach", "insert", "extract", "transport", "withdraw")
    )
    assert result.return_home is None


# The bay embedded in a larger floor: same seed, same props, wider bounds.
HALL = Bounds(-9.0, 4.7, -7.0, 7.0)


def test_pickup_bounds_reproduce_the_bay_plan_inside_a_larger_floor():
    bay = make_scenario(0, ASSETS)
    hall = replace(bay, bounds=HALL)
    in_bay = plan_transport(bay)
    in_hall = plan_transport(hall, pickup_bounds=bay.bounds)
    assert in_bay.success and in_hall.success, (in_bay.status, in_hall.status)
    for name in ("approach", "insert", "extract"):
        np.testing.assert_array_equal(
            getattr(in_bay, name).poses, getattr(in_hall, name).poses
        )


def test_pickup_bounds_confine_only_the_pickup_side_stages():
    bay = make_scenario(0, ASSETS)
    # A destination outside the bay: transport has to leave the pickup bounds.
    hall = replace(bay, bounds=HALL, destination=PalletSite(-6.0, -4.0, -math.pi / 2))
    result = plan_transport(hall, pickup_bounds=bay.bounds)
    assert result.success, result.status
    for name in ("approach", "insert", "extract"):
        poses = getattr(result, name).poses
        assert np.all(poses[:, 0] >= bay.bounds.x_min_m)
        assert np.all(poses[:, 1] <= bay.bounds.y_max_m)
        assert np.all(poses[:, 1] >= bay.bounds.y_min_m)
    assert result.transport.poses[:, 0].min() < bay.bounds.x_min_m


def test_pickup_bounds_apply_to_the_observation_leg():
    bay = make_scenario(0, ASSETS)
    hall = replace(bay, bounds=HALL)
    waypoint = Pose2D(-0.10, 0.90, 0)
    in_bay = pallet_mission.plan_observation_leg(bay, waypoint)
    in_hall = pallet_mission.plan_observation_leg(
        hall, waypoint, pickup_bounds=bay.bounds
    )
    assert in_bay.success, in_bay.status
    np.testing.assert_array_equal(in_bay.poses, in_hall.poses)


def test_travel_config_changes_only_the_travel_legs():
    bay = make_scenario(0, ASSETS)
    # A wall between the bay and a destination behind it, so both travel legs
    # have to search instead of closing with one analytic shot.
    wall = PlacedProp(
        AssetSpec("test://wall", 0.3, 10.5, 2), Rectangle(-4.0, -1.75, 0.3, 10.5)
    )
    hall = replace(
        bay,
        bounds=HALL,
        props=bay.props + (wall,),
        destination=PalletSite(-6.5, -4.5, -math.pi / 2),
    )
    plain = plan_transport(hall, pickup_bounds=bay.bounds, return_to=bay.start_rear)
    travel = replace(
        make_transport_planner_config(), obstacle_heuristic_resolution_m=0.25
    )
    guided = plan_transport(
        hall,
        pickup_bounds=bay.bounds,
        return_to=bay.start_rear,
        travel_config=travel,
    )
    assert plain.success and guided.success, (plain.status, guided.status)
    for name in ("approach", "insert", "extract", "withdraw"):
        np.testing.assert_array_equal(
            getattr(plain, name).poses, getattr(guided, name).poses
        )
    # Measured 2026-09-26: 24/1400 expansions without the grid term, 552/24
    # with it. Any difference shows the travel legs used the other config.
    assert (plain.transport.expanded_nodes, plain.return_home.expanded_nodes) != (
        guided.transport.expanded_nodes,
        guided.return_home.expanded_nodes,
    )


def test_mission_geometry_uses_its_own_carriage_limit():
    provisional = SyntheticMissionGeometry()
    measured = SyntheticMissionGeometry(carriage_limit_m=0.346)
    # 0.60 m pallet: min(0.36, 0.346 - 0.016) = 0.330 (reserve 16 mm since 2026-10-08)
    assert provisional.inserted_offset_m == pytest.approx(1.29 + 0.30 - 0.36)
    assert measured.inserted_offset_m == pytest.approx(1.29 + 0.30 - 0.33)
    assert measured.loaded_footprint.front_m == pytest.approx(1.56)


def test_trace_records_every_stage_without_changing_the_plan():
    scenario = make_scenario(0, ASSETS)
    plain = pallet_mission.plan_transport(scenario, return_to=scenario.start_rear)
    trace = []
    traced = pallet_mission.plan_transport(
        scenario, return_to=scenario.start_rear, trace=trace
    )
    assert (plain.success, plain.status) == (traced.success, traced.status)
    for name in ("approach", "insert", "extract", "transport", "withdraw"):
        a, b = getattr(plain, name), getattr(traced, name)
        assert (a is None) == (b is None)
        if a is not None:
            np.testing.assert_array_equal(a.poses, b.poses)
    names = [entry["stage"] for entry in trace]
    assert names[:2] == ["approach_search", "approach_straight"]
    for entry in trace:
        assert set(entry) >= {
            "stage",
            "status",
            "length_m",
            "expansions",
            "gear_changes",
        }


def test_trace_keeps_the_stages_before_a_failure():
    scenario = make_scenario(0, ASSETS)
    blocked = replace(scenario, destination=replace(scenario.destination, x_m=99.0))
    trace = []
    plan = pallet_mission.plan_transport(blocked, trace=trace)
    assert not plan.success and plan.transport is None
    stages = {entry["stage"]: entry for entry in trace}
    assert stages["approach_search"]["status"] == "success"
    assert stages["approach_search"]["length_m"] > 0
    assert trace[-1]["status"] != "success"


# --- analytic-connection fallback (docs/plans/2026-10-02-planner-tracker-robustness.md, P1) ---

# The bay catalogue as the G2 rerun measured it (FACTORY_PROPS order).
G2_CATALOGUE = (
    AssetSpec(
        "SM_BarelPlastic_A_01.usd",
        0.5999998721480395,
        0.7080822595637528,
        0.9013595379585126,
    ),
    AssetSpec(
        "SM_CratePlastic_D_01.usd",
        0.449549798453976,
        0.6620987553425408,
        0.1880701023026532,
    ),
    AssetSpec(
        "SM_CardBoxA_02.usd", 0.7970254338452492, 0.6365090800111943, 0.5032872468988465
    ),
)
# G2a seed 4: the handoff pose, from which the nominal pickup failed with
# approach:expansion_limit at interval 8 (docs/validation/2026-10-01-g2-rerun.md).
SEED4_HANDOFF = Pose2D(-0.08475987919388789, 0.8969545667655244, 0.0008659313366127473)


def g2_config():
    return pallet_mission.make_transport_planner_config(
        curvature_limit_inv_m=0.5, clearance_m=0.10, max_expansions=30000
    )


def test_a_failed_search_is_retried_and_the_trace_says_which_retry_won():
    scenario = make_scenario(4, G2_CATALOGUE, 4)
    config = g2_config()
    assert config.analytic_expansion_interval == 8
    trace = []
    result = plan_transport(scenario, config, start_rear=SEED4_HANDOFF, trace=trace)
    assert result.success, result.status
    # The approach search needed a retry; the trace says which one won.
    search = next(t for t in trace if t["stage"] == "approach_search")
    assert len(search["search_attempts"]) > 1
    assert search["search_attempts"][0][:2] == [8, "expansion_limit"]
    # Joining the straight tail must not drop the interval from the result.
    transport = next(t for t in trace if t["stage"] == "transport_search")
    assert (
        result.approach.analytic_expansion_interval
        == (search["analytic_expansion_interval"])
    )
    assert (
        result.transport.analytic_expansion_interval
        == (transport["analytic_expansion_interval"])
    )


def test_the_trace_keeps_an_earlier_fallback_when_a_later_search_fails(monkeypatch):
    real_plan = pallet_mission.plan_hybrid_astar
    goals = []

    def transport_fails(start, goal, *rest):
        if goal not in goals:
            goals.append(goal)
        if len(goals) == 1:
            return real_plan(start, goal, *rest)
        return pallet_mission.PlanResult(
            False,
            "no_path",
            np.zeros((0, 3)),
            np.zeros(0, np.int8),
            np.zeros(0),
            0.0,
            0,
        )

    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", transport_fails)
    scenario = make_scenario(4, G2_CATALOGUE, 4)
    trace = []
    result = plan_transport(
        scenario, g2_config(), start_rear=SEED4_HANDOFF, trace=trace
    )
    assert not result.success
    by_stage = {t["stage"]: t for t in trace}
    assert by_stage["approach_search"]["status"] == "success"
    assert len(by_stage["approach_search"]["search_attempts"]) > 1
    assert by_stage["transport_search"]["status"] == "no_path"
    # The last search is the extended ladder's, at the original interval.
    assert by_stage["transport_search"]["analytic_expansion_interval"] == 8
    # The cost of every retry is on record, not only the last search's.
    approach = by_stage["approach_search"]["search_attempts"]
    assert approach[0][:2] == [8, "expansion_limit"]
    assert approach[-1][1] == "success"
    # Fine lattice first (P4), then denser connections on the original
    # lattice, then the extended ladder: the fine lattice already closed, so
    # only its shorter-primitive search runs
    # (docs/plans/2026-10-03-transport-stage-fixes.md, T2).
    transport = by_stage["transport_search"]["search_attempts"]
    assert [
        (entry[0], entry[1], entry[3], round(entry[4], 6), entry[6])
        for entry in transport
    ] == [
        (8, "no_path", 0.2, round(np.radians(10), 6), 0.25),
        (8, "no_path", 0.1, round(np.radians(5), 6), 0.25),
        (4, "no_path", 0.2, round(np.radians(10), 6), 0.25),
        (2, "no_path", 0.2, round(np.radians(10), 6), 0.25),
        (1, "no_path", 0.2, round(np.radians(10), 6), 0.25),
        (8, "no_path", 0.1, round(np.radians(5), 6), 0.10),
    ]
    assert all(entry[2] > 0 for entry in approach)


def test_a_search_that_succeeds_at_eight_is_not_retried(monkeypatch):
    received = []
    real_plan = pallet_mission.plan_hybrid_astar

    def capture(*args):
        received.append(args[-1].analytic_expansion_interval)
        return real_plan(*args)

    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", capture)
    scenario = make_scenario(0, G2_CATALOGUE, 4)
    trace = []
    result = plan_transport(scenario, g2_config(), trace=trace)
    assert result.success, result.status
    assert received == [8, 8]  # approach and transport, once each
    searches = [t for t in trace if t["stage"].endswith("_search")]
    assert [t["analytic_expansion_interval"] for t in searches] == [8, 8]


@pytest.mark.parametrize("seed", [0, 2, 5])
def test_the_fallback_changes_nothing_when_eight_succeeds(monkeypatch, seed):
    scenario = make_scenario(seed, G2_CATALOGUE, 4)
    with_fallback = plan_transport(scenario, g2_config())
    monkeypatch.setattr(pallet_mission, "FALLBACK_ANALYTIC_INTERVALS", ())
    monkeypatch.setattr(pallet_mission, "FALLBACK_FINE_LATTICE", None)
    without = plan_transport(scenario, g2_config())
    assert with_fallback.success and without.success
    for stage in ("approach", "insert", "extract", "transport", "withdraw"):
        a, b = getattr(with_fallback, stage), getattr(without, stage)
        np.testing.assert_array_equal(a.poses, b.poses)
        np.testing.assert_array_equal(a.directions, b.directions)
        np.testing.assert_array_equal(a.curvatures_inv_m, b.curvatures_inv_m)


def test_a_failure_other_than_search_exhaustion_is_not_retried(monkeypatch):
    received = []

    def invalid(*args):
        received.append(args[-1].analytic_expansion_interval)
        return pallet_mission.PlanResult(
            False,
            "invalid_goal",
            np.zeros((0, 3)),
            np.zeros(0, np.int8),
            np.zeros(0),
            0.0,
            0,
        )

    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", invalid)
    result = pallet_mission._search(None, None, [], None, None, g2_config())
    assert result.status == "invalid_goal" and received == [8]


def _exhausted(received):
    def plan(*args):
        received.append(args[-1])
        return pallet_mission.PlanResult(
            False,
            "expansion_limit",
            np.zeros((0, 3)),
            np.zeros(0, np.int8),
            np.zeros(0),
            0.0,
            30000,
        )

    return plan


def test_the_fine_lattice_is_tried_first_at_the_same_interval(monkeypatch):
    received = []
    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", _exhausted(received))
    config = g2_config()
    pallet_mission._search(None, None, [], None, None, config)
    lattices = [(c.xy_resolution_m, c.yaw_resolution_rad) for c in received]
    intervals = [c.analytic_expansion_interval for c in received]
    base = (config.xy_resolution_m, config.yaw_resolution_rad)
    fine = (0.1, np.radians(5))
    assert lattices == [base, fine, base, base, base, fine, fine]
    assert intervals == [8, 8, 4, 2, 1, 8, 8]
    # The earlier ladder keeps the caller's budget and primitive; the extended
    # one runs after it: the fine lattice at four times the budget, then with
    # a 0.10 m primitive (docs/plans/2026-10-03-transport-stage-fixes.md, T2).
    assert [c.max_expansions for c in received] == [config.max_expansions] * 5 + [
        4 * config.max_expansions
    ] * 2
    assert [c.primitive_length_m for c in received] == [
        config.primitive_length_m
    ] * 6 + [0.10]
    for retry in received[1:5]:
        assert (
            replace(
                retry,
                xy_resolution_m=config.xy_resolution_m,
                yaw_resolution_rad=config.yaw_resolution_rad,
                analytic_expansion_interval=config.analytic_expansion_interval,
            )
            == config
        )


def test_the_extended_ladder_can_be_left_out(monkeypatch):
    received = []
    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", _exhausted(received))
    pallet_mission._search(None, None, [], None, None, g2_config(), extended=False)
    assert [c.analytic_expansion_interval for c in received] == [8, 8, 4, 2, 1]
    assert {c.max_expansions for c in received} == {g2_config().max_expansions}


def test_a_lattice_already_as_fine_is_not_searched_again(monkeypatch):
    received = []
    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", _exhausted(received))
    fine = replace(g2_config(), xy_resolution_m=0.1, yaw_resolution_rad=np.radians(5))
    pallet_mission._search(None, None, [], None, None, fine)
    # The earlier ladder skips the duplicate fine search; the extended ladder
    # still runs on it, with more budget and then the shorter primitive.
    assert [c.analytic_expansion_interval for c in received] == [8, 4, 2, 1, 8, 8]
    assert all(c.xy_resolution_m == 0.1 for c in received)
    assert [c.max_expansions for c in received[-2:]] == [4 * fine.max_expansions] * 2
    assert received[-1].primitive_length_m == 0.10


def test_a_partly_finer_lattice_only_refines_the_coarser_axis(monkeypatch):
    received = []
    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", _exhausted(received))
    config = replace(g2_config(), xy_resolution_m=0.05)
    pallet_mission._search(None, None, [], None, None, config)
    assert (received[1].xy_resolution_m, received[1].yaw_resolution_rad) == (
        0.05,
        np.radians(5),
    )


def test_the_fine_lattice_rescues_the_second_evaluation_seed_2007():
    """Offline replay of seed 2007's handoff (docs/plans/2026-10-02-second-eval-failure-fixes.md)."""
    scenario = make_scenario(2007, G2_CATALOGUE, 4)
    target = pallet_mission.PalletSite(
        3.3596509836450688, 0.6171614345292817, -0.11854830685801701
    )
    start = Pose2D(-1.2235211175476588, 0.30154677671545727, -0.006682678318098347)
    trace = []
    result = plan_transport(
        scenario, g2_config(), target_pickup=target, start_rear=start, trace=trace
    )
    assert result.success, result.status
    search = next(t for t in trace if t["stage"] == "approach_search")
    assert [entry[:2] for entry in search["search_attempts"]] == [
        [8, "expansion_limit"],
        [8, "success"],
    ]
    assert search["search_attempts"][1][3] == 0.1


def test_when_the_fine_lattice_also_runs_out_a_denser_interval_still_rescues(
    monkeypatch,
):
    """Dev seed 1020's handoff: fine lattice exhausts, interval 4 on the base lattice plans.

    The extended ladder runs only after this, so the plan is unchanged.
    """
    scenario = make_scenario(1020, G2_CATALOGUE, 4)
    target = pallet_mission.PalletSite(
        3.1790195627598172, 0.28139591002602726, -0.1785698927944206
    )
    start = Pose2D(-0.11716749215274674, -0.6009603022045468, -0.012208772502526596)
    trace = []
    result = plan_transport(
        scenario, g2_config(), target_pickup=target, start_rear=start, trace=trace
    )
    assert result.success, result.status
    search = next(t for t in trace if t["stage"] == "approach_search")
    assert [(e[0], e[1], e[3]) for e in search["search_attempts"]] == [
        (8, "expansion_limit", 0.2),
        (8, "expansion_limit", 0.1),
        (4, "success", 0.2),
    ]
    assert result.approach.analytic_expansion_interval == 4


def test_a_start_boxed_in_by_the_swept_margin_is_searched_with_finer_collision_steps():
    """Fine-lattice dev run seed 1006: the start is free at the approach clearance,
    but within the extra swept-footprint margin of a prop, so every primitive is
    rejected and the search ends after one expansion
    (docs/plans/2026-10-02-second-eval-failure-fixes.md, P6)."""
    scenario = make_scenario(1006, G2_CATALOGUE, 4)
    target = pallet_mission.PalletSite(
        2.9369561817130974, 0.8203058927930917, -0.08517051692935226
    )
    start = Pose2D(-1.222732341882794, 0.30449959478431576, -0.04754033353286844)
    trace = []
    result = plan_transport(
        scenario, g2_config(), target_pickup=target, start_rear=start, trace=trace
    )
    assert result.success, result.status
    attempts = next(t for t in trace if t["stage"] == "approach_search")[
        "search_attempts"
    ]
    assert attempts[0][:3] == [8, "no_path", 1]
    assert attempts[-1][1] == "success"
    assert attempts[-1][5] == 0.01  # collision step of the winning search


def test_a_search_that_expands_more_than_its_root_keeps_the_collision_step(monkeypatch):
    received = []
    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", _exhausted(received))
    pallet_mission._search(None, None, [], None, None, g2_config())
    assert {c.collision_step_m for c in received} == {g2_config().collision_step_m}


def test_a_boxed_start_repeats_the_whole_ladder_at_the_finer_collision_step(
    monkeypatch,
):
    received = []

    def boxed(*args):
        received.append(args[-1])
        return pallet_mission.PlanResult(
            False,
            "no_path",
            np.zeros((0, 3)),
            np.zeros(0, np.int8),
            np.zeros(0),
            0.0,
            1,
        )

    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", boxed)
    pallet_mission._search(None, None, [], None, None, g2_config())
    # Every retry at the base step would be boxed in the same way, so the
    # ladder moves straight to the finer step.
    steps = [c.collision_step_m for c in received]
    base = g2_config().collision_step_m
    assert steps == [base] + [0.01] * 6
    # The fine lattice closed, so its larger-budget search is skipped.
    assert [c.analytic_expansion_interval for c in received] == [8, 8, 8, 4, 2, 1, 8]


def _no_path_after(expanded, received):
    def plan(*args):
        received.append(args[-1])
        return pallet_mission.PlanResult(
            False,
            "no_path",
            np.zeros((0, 3)),
            np.zeros(0, np.int8),
            np.zeros(0),
            0.0,
            expanded,
        )

    return plan


def test_a_search_that_got_past_its_root_keeps_the_collision_step(monkeypatch):
    received = []
    monkeypatch.setattr(
        pallet_mission, "plan_hybrid_astar", _no_path_after(2, received)
    )
    pallet_mission._search(None, None, [], None, None, g2_config())
    assert {c.collision_step_m for c in received} == {g2_config().collision_step_m}


def test_a_collision_step_already_as_fine_is_not_coarsened(monkeypatch):
    received = []
    monkeypatch.setattr(
        pallet_mission, "plan_hybrid_astar", _no_path_after(1, received)
    )
    pallet_mission._search(
        None, None, [], None, None, replace(g2_config(), collision_step_m=0.005)
    )
    assert {c.collision_step_m for c in received} == {0.005}


def test_the_transport_leg_alone_matches_the_mission_plan():
    """plan_transport_leg from the extracted pose is plan_transport's transport
    (docs/plans/2026-10-03-transport-stage-fixes.md)."""
    for seed in (0, 2):
        scenario = make_scenario(seed, G2_CATALOGUE, 4)
        mission = plan_transport(scenario, g2_config())
        assert mission.success
        start = Pose2D(*mission.transport.poses[0])
        leg = pallet_mission.plan_transport_leg(scenario, start, g2_config())
        assert leg.success
        np.testing.assert_array_equal(leg.poses, mission.transport.poses)
        np.testing.assert_array_equal(leg.directions, mission.transport.directions)
        np.testing.assert_array_equal(
            leg.curvatures_inv_m, mission.transport.curvatures_inv_m
        )


def test_the_fine_lattice_budget_plans_the_fourth_evaluation_seed_4013_transport():
    """Its handoff's transport closed the base lattice and ran out of the fine
    one at 30,000 expansions; four times that plans it."""
    scenario = make_scenario(4013, G2_CATALOGUE, 4)
    start = Pose2D(1.1398248485991234, 0.32419656228973015, -0.03151695982182697)
    leg = pallet_mission.plan_transport_leg(scenario, start, g2_config())
    assert leg.success, leg.status
    assert [tuple(e[:2]) for e in leg.search_attempts] == [
        (8, "no_path"),
        (8, "expansion_limit"),
        (4, "no_path"),
        (2, "no_path"),
        (1, "no_path"),
        (8, "success"),
    ]
    winning = leg.search_attempts[-1]
    assert winning[3] == 0.1 and winning[6] == 0.25 and winning[7] == 120000


def test_a_short_primitive_leaves_the_fourth_evaluation_seed_4020_start():
    """Two props narrow the start; 0.25 m primitives close every lattice at the
    0.10 m clearance, 0.10 m ones on the fine lattice find the way."""
    scenario = make_scenario(4020, G2_CATALOGUE, 4)
    leg = pallet_mission.plan_observation_leg(
        scenario, Pose2D(0.40, 1.20, 0.0), g2_config()
    )
    assert leg.success, leg.status
    winning = leg.search_attempts[-1]
    assert winning[1] == "success" and winning[3] == 0.1 and winning[6] == 0.10


def test_the_return_leg_alone_matches_the_mission_plan():
    """plan_return_leg from the withdrawn pose is plan_transport's return_home."""
    scenario = make_scenario(0, G2_CATALOGUE, 4)
    mission = plan_transport(scenario, g2_config(), return_to=scenario.start_rear)
    assert mission.success and mission.return_home is not None
    leg = pallet_mission.plan_return_leg(
        scenario, Pose2D(*mission.return_home.poses[0]), scenario.start_rear, g2_config()
    )
    assert leg.success
    np.testing.assert_array_equal(leg.poses, mission.return_home.poses)
    np.testing.assert_array_equal(leg.directions, mission.return_home.directions)


def _plan(poses, directions, curvatures):
    from forklift_core.planning.hybrid_astar import PlanResult

    poses = np.asarray(poses, dtype=float)
    length = float(np.hypot(*np.diff(poses[:, :2], axis=0).T).sum())
    return PlanResult(True, "ok", poses, np.asarray(directions, np.int8),
                      np.asarray(curvatures, float), length, 0)


def test_the_near_capture_point_is_where_the_final_straight_begins():
    from forklift_core.planning.pallet_mission import final_straight_prefix

    # 1 m arc-ish lead-in (curved), then 1.2 m straight along +x.
    lead = [(-1.0 + 0.1 * k, 0.1 * (10 - k) ** 2 / 100, 0.0) for k in range(10)]
    straight = [(0.1 * k, 0.0, 0.0) for k in range(13)]
    poses = lead + straight
    curv = [0.5] * len(lead) + [0.0] * len(straight)
    plan = _plan(poses, [1] * len(poses), curv)
    prefix = final_straight_prefix(plan, keep_m=0.8)
    assert prefix is not None
    np.testing.assert_allclose(prefix.poses[-1], (0.4, 0.0, 0.0), atol=1e-9)
    assert prefix.length_m == pytest.approx(plan.length_m - 0.8, abs=1e-9)
    assert len(prefix.poses) == len(prefix.directions) == len(prefix.curvatures_inv_m)


def test_no_near_capture_when_the_end_is_not_a_long_enough_forward_straight():
    from forklift_core.planning.pallet_mission import final_straight_prefix

    straight = [(0.1 * k, 0.0, 0.0) for k in range(6)]  # only 0.5 m
    assert final_straight_prefix(_plan(straight, [1] * 6, [0.0] * 6), keep_m=0.8) is None
    curved = [(0.1 * k, 0.0, 0.0) for k in range(13)]
    assert final_straight_prefix(_plan(curved, [1] * 13, [0.3] * 13), keep_m=0.8) is None
    reverse = [(-0.1 * k, 0.0, 0.0) for k in range(13)]
    assert final_straight_prefix(_plan(reverse, [-1] * 13, [0.0] * 13), keep_m=0.8) is None


def test_a_final_straight_one_ulp_short_still_counts():
    """Codex v3.6 P1: arc to (4,0,0) then a straight that sums to 0.7999999."""
    from forklift_core.planning.pallet_mission import final_straight_prefix

    arc = [(4.0 - 0.1 * (5 - k), 0.01 * (5 - k) ** 2, 0.0) for k in range(6)]
    xs = np.linspace(4.0, 4.8, 21)
    straight = [(x, 0.0, 0.0) for x in xs[1:]]
    poses = arc + straight
    plan = _plan(poses, [1] * len(poses), [0.5] * len(arc) + [0.0] * len(straight))
    prefix = final_straight_prefix(plan, keep_m=0.8)
    assert prefix is not None
    np.testing.assert_allclose(prefix.poses[-1][:2], (4.0, 0.0), atol=1e-9)


def test_the_final_straight_is_redrawn_from_where_the_truck_stands():
    from forklift_core.planning import Pose2D
    from forklift_core.planning.pallet_mission import straight_from_pose

    start, end = Pose2D(0.0, 0.0, 0.0), Pose2D(0.8, 0.0, 0.0)
    plan, record = straight_from_pose(
        Pose2D(0.02, 0.03, 0.04), start, end, max_lateral_m=0.05, max_yaw_rad=0.05, min_length_m=0.3
    )
    assert plan is not None
    np.testing.assert_allclose(plan.poses[0], (0.02, 0.0, 0.0), atol=1e-12)
    np.testing.assert_allclose(plan.poses[-1], (0.8, 0.0, 0.0), atol=1e-12)
    assert plan.length_m == pytest.approx(0.78)
    assert record["lateral_m"] == pytest.approx(0.03)
    assert np.all(plan.directions == 1) and np.all(plan.curvatures_inv_m == 0)
    for bad in (Pose2D(0.0, 0.06, 0.0), Pose2D(0.0, 0.0, 0.06), Pose2D(0.6, 0.0, 0.0)):
        plan, _ = straight_from_pose(
            bad, start, end, max_lateral_m=0.05, max_yaw_rad=0.05, min_length_m=0.3
        )
        assert plan is None


def test_a_truck_behind_the_line_start_gets_a_straight_from_its_own_projection():
    # Plan v10 D7c (Codex v10 2nd P1-4): a negative along was cut to 0, so the path
    # began ahead of the truck and the tracker failed on its first tick (cross_track).
    from forklift_core.planning import Pose2D
    from forklift_core.planning.pallet_mission import straight_from_pose

    start, end = Pose2D(0.0, 0.0, 0.0), Pose2D(1.5, 0.0, 0.0)
    plan, record = straight_from_pose(
        Pose2D(-1.0, 0.02, 0.0), start, end, max_lateral_m=0.05, max_yaw_rad=0.05, min_length_m=0.3
    )
    assert plan is not None
    np.testing.assert_allclose(plan.poses[0], (-1.0, 0.0, 0.0), atol=1e-12)
    np.testing.assert_allclose(plan.poses[-1], (1.5, 0.0, 0.0), atol=1e-12)
    assert plan.length_m == pytest.approx(2.5) and record["length_left_m"] == pytest.approx(2.5)
    assert record["along_m"] == pytest.approx(-1.0)


def test_an_approach_that_is_all_one_straight_is_cut_where_keep_m_is_left():
    from forklift_core.planning.pallet_mission import final_straight_prefix

    poses = [(0.1 * k, 0.0, 0.0) for k in range(24)]  # 2.3 m forward straight
    prefix = final_straight_prefix(_plan(poses, [1] * 24, [0.0] * 24), keep_m=0.8)
    assert prefix is not None
    np.testing.assert_allclose(prefix.poses[-1], (1.5, 0.0, 0.0), atol=1e-9)
