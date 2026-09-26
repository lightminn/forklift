"""Observation-derived approach targets; no simulator pallet truth is supplied."""

import math
from dataclasses import replace

import numpy as np
import pytest

from forklift_core.geometry import RigidTransform
from forklift_core.perception.pallet_prior import PalletPrior
from forklift_core.perception.pocket_observation import Pocket, PocketObservation
from forklift_core.planning.observed_approach import observed_approach_goal
from forklift_core.planning.pallet_mission import SyntheticMissionGeometry

PRIOR = PalletPrior(
    0.3,
    0.05,
    0.05,
    0.2,
    0.18,
    0.3,
    0.08,
    0.12,
    0.8,
    0.6,
    "synthetic",
    "observation-test-v1",
)


def observation(**changes):
    fields = dict(
        stamp_ns=1_000_000_000,
        clock_domain="ros_sim",
        frame_id="base_link",
        source_provenance="synthetic",
        status="valid",
        left=Pocket((2, 0.7, 0.5), 0.2, 0.2),
        right=Pocket((2, 0.1, 0.5), 0.2, 0.2),
        insertion_yaw_rad=0.0,
        position_sigma_m=None,
        yaw_sigma_rad=None,
        reason=None,
    )
    return PocketObservation(**(fields | changes))


def transform(rotation=None, translation=(0, 0, 0), **frames):
    return RigidTransform(
        frames.get("source_frame", "base_link"),
        frames.get("target_frame", "world"),
        np.eye(3) if rotation is None else rotation,
        translation,
    )


def target(obs=None, world_from_base=None, prior=PRIOR, **changes):
    settings = dict(
        transform_stamp_ns=1_000_000_000,
        transform_clock_domain="ros_sim",
        now_ns=1_100_000_000,
        now_clock_domain="ros_sim",
        max_age_ns=250_000_000,
    )
    return observed_approach_goal(
        observation() if obs is None else obs,
        prior,
        transform() if world_from_base is None else world_from_base,
        **(settings | changes),
    )


def assert_no_target(result):
    assert not result.success
    assert result.pallet_site is None
    assert result.approach_rear is None
    assert result.prealign_rear is None
    assert result.front_midpoint_world_m is None


def test_front_midpoint_becomes_pallet_centre_then_rear_axle_target():
    result = target()
    assert result.success and result.status == "valid" and result.reason is None
    np.testing.assert_allclose(result.front_midpoint_world_m, [2, 0.4, 0.5])
    assert result.pallet_site.x_m == pytest.approx(2.3)
    assert result.pallet_site.y_m == pytest.approx(0.4)
    assert result.approach_rear.x_m == pytest.approx(0.61)
    assert result.approach_rear.y_m == pytest.approx(0.4)
    assert result.approach_rear.yaw_rad == 0
    assert result.prealign_rear.x_m == pytest.approx(-0.19)


def test_acquisition_world_transform_rotates_and_translates_observed_goal():
    rotation = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    result = target(world_from_base=transform(rotation, (3, -1, 0.2)))
    np.testing.assert_allclose(result.front_midpoint_world_m, [2.6, 1, 0.7])
    assert result.pallet_site.x_m == pytest.approx(2.6)
    assert result.pallet_site.y_m == pytest.approx(1.3)
    assert result.approach_rear.x_m == pytest.approx(2.6)
    assert result.approach_rear.y_m == pytest.approx(-0.39)
    assert result.approach_rear.yaw_rad == pytest.approx(math.pi / 2)


def test_small_pitch_uses_full_3d_rotation_including_pocket_height():
    angle = 0.02
    c, s = math.cos(angle), math.sin(angle)
    # Rz(pi/2) @ Ry(angle): height contributes to world y, not only z.
    rotation = [[0, -1, 0], [c, 0, s], [-s, 0, c]]
    result = target(world_from_base=transform(rotation, (3, -1, 0.2)))
    assert result.success
    assert result.front_midpoint_world_m[1] == pytest.approx(-1 + 2 * c + 0.5 * s)
    assert result.front_midpoint_world_m[2] == pytest.approx(0.2 - 2 * s + 0.5 * c)
    assert result.pallet_site.y_m == pytest.approx(-1 + 2.3 * c + 0.5 * s)
    assert result.approach_rear.y_m == pytest.approx(-1 + 2.3 * c + 0.5 * s - 1.69)


def test_goal_changes_with_observation_position_and_insertion_yaw():
    # Observed front lies on y=2 with insertion along +y; left is world -x.
    rotated = observation(
        left=Pocket((-0.3, 2, 0.5), 0.2, 0.2),
        right=Pocket((0.3, 2, 0.5), 0.2, 0.2),
        insertion_yaw_rad=math.pi / 2,
    )
    result = target(rotated)
    assert result.approach_rear.x_m == pytest.approx(0, abs=1e-12)
    assert result.approach_rear.y_m == pytest.approx(0.61)
    assert result.approach_rear.yaw_rad == pytest.approx(math.pi / 2)
    moved = replace(
        rotated,
        left=Pocket((0.7, 4, 0.5), 0.2, 0.2),
        right=Pocket((1.3, 4, 0.5), 0.2, 0.2),
    )
    moved_result = target(moved)
    assert moved_result.approach_rear.x_m == pytest.approx(1)
    assert moved_result.approach_rear.y_m == pytest.approx(2.61)


def test_explicit_prior_depth_and_robot_offset_are_used_independently():
    result = target(
        prior=replace(PRIOR, overall_depth_m=1.0),
        geometry=SyntheticMissionGeometry(axle_to_fork_tip_m=1.40),
    )
    assert result.pallet_site.x_m == pytest.approx(2.5)
    assert result.approach_rear.x_m == pytest.approx(0.7)


@pytest.mark.parametrize("status", ["no_pallet", "invalid"])
def test_nonvalid_observation_never_produces_a_goal(status):
    result = target(
        observation(
            status=status,
            left=None,
            right=None,
            insertion_yaw_rad=None,
            position_sigma_m=None,
            yaw_sigma_rad=None,
            reason="detector_rejected",
        )
    )
    assert_no_target(result)
    assert result.status == status
    assert result.reason == "detector_rejected"


@pytest.mark.parametrize(
    "changes,reason",
    [
        ({"now_ns": 999_999_999}, "future_observation"),
        ({"now_ns": 1_250_000_001}, "stale_observation"),
        ({"transform_stamp_ns": 999_999_999}, "transform_timestamp_mismatch"),
        ({"transform_stamp_ns": 1_000_000_001}, "transform_timestamp_mismatch"),
        ({"now_clock_domain": "ros_system"}, "clock_domain_mismatch"),
        ({"transform_clock_domain": "ros_system"}, "clock_domain_mismatch"),
    ],
)
def test_time_and_clock_mismatches_are_explicit_rejections(changes, reason):
    result = target(**changes)
    assert_no_target(result)
    assert result.status == "invalid"
    assert result.reason == reason


def test_age_limit_is_inclusive_and_exact_acquisition_time_is_required():
    assert target(now_ns=1_250_000_000).success
    assert target(now_ns=1_000_000_000, max_age_ns=0).success


def test_ground_truth_observation_is_not_a_perception_goal():
    result = target(observation(source_provenance="synthetic_ground_truth"))
    assert_no_target(result)
    assert result.reason == "synthetic_ground_truth_observation"


@pytest.mark.parametrize(
    "rotation",
    [
        [[1, 0, 0], [0, 0, -1], [0, 1, 0]],
        [[0, 0, 1], [0, 1, 0], [-1, 0, 0]],
        [[1, 0, 0], [0, -1, 0], [0, 0, -1]],
    ],
)
def test_nonplanar_or_inverted_transform_cannot_yield_planar_goal(rotation):
    result = target(world_from_base=transform(rotation))
    assert_no_target(result)
    assert result.reason == "nonplanar_transform"


@pytest.mark.parametrize(
    "frames",
    [
        {"source_frame": "camera_optical"},
        {"target_frame": "odom"},
    ],
)
def test_frame_misconfiguration_raises(frames):
    with pytest.raises(ValueError, match="frame"):
        target(world_from_base=transform(**frames))


def test_explicit_world_frame_name_is_supported():
    assert target(
        world_from_base=transform(target_frame="map"), world_frame_id="map"
    ).success


@pytest.mark.parametrize(
    "changes",
    [
        {"now_ns": -1},
        {"now_ns": 1.5},
        {"now_ns": True},
        {"transform_stamp_ns": True},
        {"max_age_ns": -1},
        {"max_age_ns": 1.5},
        {"max_tilt_rad": float("nan")},
        {"max_tilt_rad": -0.1},
        {"max_tilt_rad": math.pi},
        {"now_clock_domain": "unregistered"},
    ],
)
def test_invalid_timing_or_orientation_configuration_raises(changes):
    with pytest.raises(ValueError):
        target(**changes)


@pytest.mark.parametrize("depth", [0, -0.1, float("nan")])
def test_directly_constructed_invalid_prior_depth_is_rejected(depth):
    with pytest.raises(ValueError):
        target(prior=replace(PRIOR, overall_depth_m=depth))
