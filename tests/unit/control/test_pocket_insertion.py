"""Insertion decisions consume fresh sensor geometry, never a cached goal."""

from dataclasses import replace

import numpy as np
import pytest

from forklift_core.control.pocket_insertion import (
    PocketInsertionConfig,
    PocketInsertionController,
)
from forklift_core.perception.pocket_observation import Pocket, PocketObservation


def observation(stamp_ns, *, x=0.9, y=0.0, yaw=0.0, z=0.056, width=0.12):
    return PocketObservation(
        stamp_ns=stamp_ns,
        clock_domain="synthetic",
        frame_id="base_link",
        source_provenance="synthetic",
        status="valid",
        left=Pocket((x, y + 0.145, z), width, 0.072),
        right=Pocket((x, y - 0.145, z), width, 0.072),
        insertion_yaw_rad=yaw,
        position_sigma_m=0.001,
        yaw_sigma_rad=0.001,
        reason=None,
    )


def update(controller, obs, *, now_ns=None, speed=0.0, domain="synthetic"):
    return controller.update(
        obs,
        now_ns=obs.stamp_ns if now_ns is None else now_ns,
        clock_domain=domain,
        measured_speed_mps=speed,
    )


def acquire(controller, *, start_ns=0, **kwargs):
    commands = [
        update(controller, observation(start_ns + index * 50_000_000, **kwargs))
        for index in range(3)
    ]
    assert [command.speed_mps for command in commands[:2]] == [0.0, 0.0]
    return commands[-1]


def test_three_fresh_stopped_observations_are_required_before_motion():
    controller = PocketInsertionController(PocketInsertionConfig())
    command = acquire(controller)
    assert command.status == "tracking"
    assert command.speed_mps == pytest.approx(0.04)
    assert command.curvature_inv_m == 0.0
    assert command.confirmed_frames == 3
    assert command.stamp_ns == 100_000_000
    assert command.clock_domain == "synthetic"


@pytest.mark.parametrize("speed", [0.012, -0.012, 0.04])
def test_acquisition_waits_until_measured_motion_has_stopped(speed):
    controller = PocketInsertionController(PocketInsertionConfig())
    for index in range(4):
        command = update(controller, observation(index * 50_000_000), speed=speed)
        assert command.speed_mps == 0.0
        assert command.status == "acquiring"
        assert command.confirmed_frames == 0
    assert acquire(controller, start_ns=200_000_000).speed_mps > 0.0


@pytest.mark.parametrize(
    "failure",
    [
        "missing",
        "invalid",
        "stale",
        "future",
        "duplicate",
        "backward",
        "domain",
        "ground_truth",
        "jump",
        "gap",
        "overspeed",
    ],
)
def test_observation_failure_stops_immediately_and_requires_new_confirmation(failure):
    controller = PocketInsertionController(PocketInsertionConfig())
    acquire(controller)
    obs = observation(150_000_000)
    now_ns = 150_000_000
    speed = 0.04
    if failure == "missing":
        obs = None
    elif failure == "invalid":
        obs = replace(
            obs,
            status="invalid",
            left=None,
            right=None,
            insertion_yaw_rad=None,
            position_sigma_m=None,
            yaw_sigma_rad=None,
            reason="occluded",
        )
    elif failure == "stale":
        obs = observation(149_999_999)
        now_ns = 300_000_000
    elif failure == "future":
        obs = observation(150_000_001)
    elif failure == "duplicate":
        obs = observation(100_000_000)
    elif failure == "backward":
        obs = observation(90_000_000)
    elif failure == "domain":
        obs = replace(obs, clock_domain="ros_sim")
    elif failure == "ground_truth":
        obs = replace(obs, source_provenance="synthetic_ground_truth")
    elif failure == "jump":
        obs = observation(150_000_000, x=0.94)
    elif failure == "gap":
        obs = observation(250_000_001)
        now_ns = obs.stamp_ns
    elif failure == "overspeed":
        speed = 0.056
    command = update(controller, obs, now_ns=now_ns, speed=speed)
    assert command.status == "lost"
    assert command.speed_mps == command.curvature_inv_m == 0.0
    assert command.reason
    # A fresh frame while still coasting cannot reacquire.
    command = update(controller, observation(350_000_000), speed=0.02)
    assert command.speed_mps == 0.0
    assert command.confirmed_frames == 0
    assert acquire(controller, start_ns=400_000_000).speed_mps > 0.0


def test_repeated_frames_never_count_as_confirmation():
    controller = PocketInsertionController(PocketInsertionConfig())
    obs = observation(0)
    for now_ns in [0, 1, 2, 3]:
        command = update(controller, obs, now_ns=now_ns)
        assert command.speed_mps == 0.0
        assert command.confirmed_frames < 3
    assert acquire(controller, start_ns=50_000_000).speed_mps > 0.0


@pytest.mark.parametrize(
    "kwargs",
    [
        {"x": 1.101},
        {"x": 0.584},
        {"yaw": 0.026},
        {"y": 0.013},
        {"width": 0.055},
        {"z": 0.09},
        {"z": 0.015},
    ],
)
def test_out_of_envelope_or_inadequate_openings_cannot_authorize_motion(kwargs):
    controller = PocketInsertionController(PocketInsertionConfig())
    for index in range(4):
        command = update(controller, observation(index * 50_000_000, **kwargs))
        assert command.status == "lost"
        assert command.speed_mps == 0.0


def test_each_fork_clearance_is_checked_individually():
    controller = PocketInsertionController(PocketInsertionConfig())
    obs = observation(0)
    obs = replace(
        obs,
        left=replace(obs.left, center_m=(0.9, 0.2, 0.056)),
        right=replace(obs.right, center_m=(0.9, -0.2, 0.056)),
    )
    assert update(controller, obs).status == "lost"


def test_geometry_change_checks_opening_dimensions_as_well_as_centers():
    controller = PocketInsertionController(PocketInsertionConfig())
    acquire(controller)
    command = update(controller, observation(150_000_000, width=0.16), speed=0.04)
    assert command.status == "lost"
    assert command.speed_mps == 0.0


def test_goal_requires_fresh_observation_and_measured_stop():
    controller = PocketInsertionController(PocketInsertionConfig())
    acquire(controller, x=0.61)
    command = update(controller, observation(150_000_000, x=0.594), speed=0.02)
    assert command.status == "tracking"
    assert command.speed_mps == 0.0
    command = update(controller, observation(200_000_000, x=0.591), speed=0.0)
    assert command.status == "complete"
    assert command.speed_mps == 0.0
    command = update(controller, None, now_ns=250_000_000)
    assert command.status == "lost"
    assert command.speed_mps == 0.0


def test_completed_controller_never_restarts_motion_if_target_moves():
    controller = PocketInsertionController(PocketInsertionConfig())
    assert acquire(controller, x=0.59).status == "complete"
    for index in range(1, 8):
        command = update(
            controller,
            observation(100_000_000 + index * 50_000_000, x=0.59 + index * 0.01),
        )
        assert command.speed_mps == 0.0


def test_small_alignment_error_produces_bounded_corrective_curvature():
    cfg = PocketInsertionConfig(max_curvature_inv_m=0.02)
    left = acquire(PocketInsertionController(cfg), y=0.01, yaw=0.02)
    right = acquire(PocketInsertionController(cfg), y=-0.01, yaw=-0.02)
    assert 0 < left.curvature_inv_m <= 0.02
    assert -0.02 <= right.curvature_inv_m < 0
    assert 0 < left.speed_mps <= 0.055


def test_backward_controller_clock_requires_recovery_above_high_watermark():
    controller = PocketInsertionController(PocketInsertionConfig())
    acquire(controller)
    command = update(controller, observation(90_000_000), now_ns=90_000_000)
    assert command.status == "lost"
    assert command.speed_mps == 0.0
    command = update(controller, observation(95_000_000), now_ns=95_000_000)
    assert command.status == "lost"
    assert acquire(controller, start_ns=150_000_000).speed_mps > 0.0


def test_controller_clock_domain_cannot_change_to_bypass_timestamps():
    controller = PocketInsertionController(PocketInsertionConfig())
    acquire(controller)
    obs = replace(observation(150_000_000), clock_domain="ros_sim")
    command = update(controller, obs, domain="ros_sim")
    assert command.status == "lost"
    assert command.speed_mps == 0.0


@pytest.mark.parametrize(
    "name,value",
    [
        ("confirmation_frames", True),
        ("confirmation_frames", 2.5),
        ("confirmation_frames", 0),
        ("max_observation_age_ns", True),
        ("max_observation_age_ns", 2.5),
        ("cruise_speed_mps", np.nan),
        ("max_speed_mps", np.inf),
        ("max_lateral_error_m", True),
        ("fork_width_m", 0),
        ("fork_bottom_z_m", 0.06),
        ("target_front_x_m", 1.2),
        ("cruise_speed_mps", 0.06),
        ("max_yaw_error_rad", 2.0),
    ],
)
def test_invalid_configuration_is_rejected(name, value):
    with pytest.raises(ValueError):
        PocketInsertionConfig(**{name: value})


@pytest.mark.parametrize("value", [True, -1, 2.5, np.nan])
def test_invalid_controller_timestamp_is_an_explicit_contract_error(value):
    controller = PocketInsertionController(PocketInsertionConfig())
    with pytest.raises(ValueError):
        update(controller, observation(0), now_ns=value)


@pytest.mark.parametrize("x", [0.585, 0.595])
def test_exact_goal_tolerance_edges_never_command_reverse_or_forward(x):
    controller = PocketInsertionController(PocketInsertionConfig())
    command = acquire(controller, x=x)
    assert command.status == "complete"
    assert command.speed_mps == 0.0


@pytest.mark.parametrize("speed", [float("nan"), float("inf"), True, "0.04"])
def test_invalid_measured_speed_returns_stop(speed):
    controller = PocketInsertionController(PocketInsertionConfig())
    acquire(controller)
    command = update(controller, observation(150_000_000), speed=speed)
    assert command.status == "lost"
    assert command.speed_mps == command.curvature_inv_m == 0.0


def test_old_but_inclusive_maximum_age_observations_can_confirm():
    controller = PocketInsertionController(PocketInsertionConfig())
    for stamp_ns in [0, 50_000_000, 100_000_000]:
        command = update(
            controller, observation(stamp_ns), now_ns=stamp_ns + 150_000_000
        )
    assert command.status == "tracking"
    assert command.speed_mps > 0


def test_adjustable_base_frame_fork_height_does_not_assume_ground_origin():
    cfg = PocketInsertionConfig(fork_bottom_z_m=-0.002, fork_top_z_m=0.022)
    command = acquire(PocketInsertionController(cfg), z=0.026)
    assert command.status == "tracking"
    assert command.speed_mps > 0


def test_start_range_can_be_extended_for_synthetic_camera_visibility():
    cfg = PocketInsertionConfig(max_start_front_x_m=1.75)
    command = acquire(PocketInsertionController(cfg), x=1.7)
    assert command.status == "tracking"
    assert command.speed_mps > 0


def faster_approach_config(**overrides):
    values = dict(
        max_start_front_x_m=1.76,
        cruise_speed_mps=0.6,
        max_speed_mps=0.75,
        insertion_speed_mps=0.055,
        slowdown_front_x_m=1.35,
    )
    values.update(overrides)
    return PocketInsertionConfig(**values)


def test_faster_approach_slows_at_threshold_before_fork_entry():
    controller = PocketInsertionController(faster_approach_config())
    assert acquire(controller, x=1.36).speed_mps == pytest.approx(0.6)
    command = update(controller, observation(150_000_000, x=1.35), speed=0.6)
    assert command.status == "tracking"
    assert command.speed_mps == pytest.approx(0.055)
    command = update(controller, observation(200_000_000, x=1.34), speed=0.5)
    assert command.speed_mps == pytest.approx(0.055)


@pytest.mark.parametrize("x,expected_speed", [(1.7, 0.6), (0.95, 0.055), (0.61, 0.02)])
def test_speed_schedule_keeps_distance_braking_near_the_observed_goal(
    x, expected_speed
):
    command = acquire(PocketInsertionController(faster_approach_config()), x=x)
    assert command.speed_mps == pytest.approx(expected_speed)


def test_observation_loss_commands_zero_even_at_fast_approach_speed():
    controller = PocketInsertionController(faster_approach_config())
    assert acquire(controller, x=1.7).speed_mps == pytest.approx(0.6)
    command = update(controller, None, now_ns=150_000_000, speed=0.6)
    assert command.status == "lost"
    assert command.speed_mps == command.curvature_inv_m == 0.0


def test_faster_approach_configuration_preserves_measured_stop_completion():
    controller = PocketInsertionController(faster_approach_config())
    acquire(controller, x=0.61)
    command = update(controller, observation(150_000_000, x=0.594), speed=0.04)
    assert command.status == "tracking"
    assert command.speed_mps == 0.0
    command = update(controller, observation(200_000_000, x=0.591), speed=0.0)
    assert command.status == "complete"
    assert command.speed_mps == 0.0


@pytest.mark.parametrize(
    "overrides",
    [
        {"insertion_speed_mps": 0.012},
        {"insertion_speed_mps": 0.61},
        {"insertion_speed_mps": float("nan")},
        {"insertion_speed_mps": True},
        {"slowdown_front_x_m": 0.59},
        {"slowdown_front_x_m": 1.77},
        {"slowdown_front_x_m": float("inf")},
        {"slowdown_front_x_m": False},
        {"cruise_speed_mps": 0.76},
    ],
)
def test_speed_schedule_rejects_invalid_limits(overrides):
    with pytest.raises(ValueError):
        faster_approach_config(**overrides)


def test_slowdown_at_start_limit_keeps_entire_allowed_range_at_insertion_speed():
    config = faster_approach_config(slowdown_front_x_m=1.76)
    command = acquire(PocketInsertionController(config), x=1.76)
    assert command.speed_mps == pytest.approx(0.055)
