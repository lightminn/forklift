"""Synthetic independent bicycle rollouts and actuator geometry contracts."""

import numpy as np
import pytest

from forklift_core.control import (
    AckermannGeometry,
    RearAxlePathTracker,
    TrackerConfig,
    ackermann_command,
)


def route(direction=1):
    # Analytic circular arc followed by a one-metre tangent straight.
    angle = np.linspace(0, direction * 0.6, 101)
    arc = np.column_stack((np.sin(angle) / 0.4, (1 - np.cos(angle)) / 0.4, angle))
    distance = np.linspace(0.01, 1, 101)
    straight = arc[-1] + np.column_stack(
        (
            direction * distance * np.cos(angle[-1]),
            direction * distance * np.sin(angle[-1]),
            np.zeros(len(distance)),
        )
    )
    poses = np.vstack((arc, straight))
    return (
        poses,
        np.full(len(poses), direction),
        np.r_[np.full(len(arc), 0.4), np.zeros(len(straight))],
    )


def rollout(tracker, initial, count=20000):
    pose = np.array(initial, dtype=float)
    speed = 0.0
    history = []
    for _ in range(count):
        command = tracker.update(pose, speed, 0.02)
        history.append(command)
        assert (
            abs(command.speed_mps - speed)
            <= tracker.config.max_acceleration_mps2 * 0.02 + 1e-9
        )
        assert abs(command.curvature_inv_m) <= tracker.config.max_curvature_inv_m + 1e-9
        speed = command.speed_mps
        # Independent midpoint bicycle integration, not the planner integrator.
        change = speed * command.curvature_inv_m * 0.02
        pose[:2] += (
            speed
            * 0.02
            * np.array([np.cos(pose[2] + change / 2), np.sin(pose[2] + change / 2)])
        )
        pose[2] += change
        if command.status in {"arrived", "failed"}:
            break
    return pose, history


@pytest.mark.parametrize("direction", [1, -1])
def test_curved_route_and_precise_final_straight_arrive(direction):
    poses, directions, curvatures = route(direction)
    tracker = RearAxlePathTracker(
        poses,
        directions,
        curvatures,
        TrackerConfig(
            cruise_speed_mps=0.08,
            position_tolerance_m=0.008,
            yaw_tolerance_rad=0.02,
        ),
    )
    initial = poses[0] + [0, 0.025, 0.012]
    final, history = rollout(tracker, initial)
    assert history[-1].status == "arrived", (final, history[-1])
    assert np.linalg.norm(final[:2] - poses[-1, :2]) <= 0.008
    assert abs(final[2] - poses[-1, 2]) <= 0.02
    assert np.all(np.diff([c.progress_m for c in history]) >= -1e-10)


def test_cusp_requires_stopping_before_gear_change():
    x = np.r_[np.linspace(0, 1, 51), np.linspace(0.98, 0.3, 35)]
    poses = np.column_stack((x, np.zeros_like(x), np.zeros_like(x)))
    directions = np.r_[np.ones(51), -np.ones(35)]
    tracker = RearAxlePathTracker(poses, directions, np.zeros(len(x)))
    final, history = rollout(tracker, poses[0])
    speeds = np.array([c.speed_mps for c in history])
    first_reverse = np.flatnonzero(speeds < 0)[0]
    assert speeds[first_reverse - 1] == 0
    assert history[first_reverse - 1].status == "braking"
    assert history[-1].status == "arrived"
    assert abs(final[0] - 0.3) < 0.035


def test_final_position_with_wrong_yaw_is_not_success():
    tracker = RearAxlePathTracker([[0, 0, 0], [1, 0, 0]], [1, 1], [0, 0])
    for x in np.linspace(0, 1, 101):
        command = tracker.update([x, 0, 0.4], 0, 0.02)
    assert command.status != "arrived"


def test_crossing_does_not_jump_to_later_leg():
    # The rear axle at the origin also lies on the later return leg.
    poses = [[0, 0, 0], [1, 0, 0], [2, 0, 0], [1, 0, 0], [0, 0, 0]]
    tracker = RearAxlePathTracker(poses, [1, 1, 1, -1, -1], [0] * 5)
    command = tracker.update([0, 0, 0], 0, 0.02)
    assert command.segment_index == 0
    assert command.progress_m == 0
    assert command.speed_mps > 0


@pytest.mark.parametrize(
    "pose,speed,dt",
    [([0, np.nan, 0], 0, 0.02), ([0, 0, 0], np.inf, 0.02), ([0, 0, 0], 0, 0)],
)
def test_invalid_update_is_rejected(pose, speed, dt):
    tracker = RearAxlePathTracker([[0, 0, 0], [1, 0, 0]], [1, 1], [0, 0])
    with pytest.raises(ValueError):
        tracker.update(pose, speed, dt)


@pytest.mark.parametrize(
    "poses,directions,curvatures",
    [
        ([[0, 0, 0]], [1], [0]),
        ([[0, 0, 0], [1, 0, np.nan]], [1, 1], [0, 0]),
        ([[0, 0, 0], [1, 0, 0]], [0, 1], [0, 0]),
        ([[0, 0, 0], [1, 0, 0]], [1, 1], [0, np.inf]),
        ([[0, 0, 0], [0, 0, 0]], [1, 1], [0, 0]),
    ],
)
def test_invalid_path_is_rejected(poses, directions, curvatures):
    with pytest.raises(ValueError):
        RearAxlePathTracker(poses, directions, curvatures)


def test_tracking_loss_latches_failure_not_success():
    tracker = RearAxlePathTracker([[0, 0, 0], [1, 0, 0]], [1, 1], [0, 0])
    assert tracker.update([0, 3, 0], 0, 0.02).status == "failed"
    assert tracker.update([1, 0, 0], 0, 0.02).status == "failed"


def geometry():
    return AckermannGeometry(
        wheelbase_m=0.64,
        track_m=0.51,
        wheel_radius_m=0.135,
        max_steering_rad=0.45,
        max_wheel_rate_rad_s=8,
    )


def test_ackermann_inside_wheels_slower_and_reverse_signs():
    forward = ackermann_command(0.18, 0.5, geometry())
    reverse = ackermann_command(-0.18, 0.5, geometry())
    assert 0 < forward.steering_rad[1] < forward.steering_rad[0] < 0.45
    fl, fr, rl, rr = forward.wheel_rates_rad_s
    assert 0 < rl < rr
    assert rl < fl < fr and rr < fr
    assert np.allclose(reverse.wheel_rates_rad_s, -np.array(forward.wheel_rates_rad_s))
    assert reverse.steering_rad == forward.steering_rad
    # Rear wheel differential must produce the requested angular velocity.
    assert (rr - rl) * 0.135 / 0.51 == pytest.approx(0.18 * 0.5)


def test_ackermann_limits_preserve_feasible_curvature_by_speed_scaling():
    command = ackermann_command(10, 0.5, geometry())
    assert max(abs(v) for v in command.wheel_rates_rad_s) <= 8
    assert 0 < command.speed_mps < 10
    assert command.curvature_inv_m == 0.5
    assert max(abs(v) for v in command.steering_rad) <= 0.45
    extreme = ackermann_command(0.18, -10, geometry())
    assert max(abs(v) for v in extreme.steering_rad) <= 0.45 + 1e-12
    assert -0.7 < extreme.curvature_inv_m < 0


@pytest.mark.parametrize("speed,curvature", [(np.nan, 0), (0, np.inf)])
def test_ackermann_rejects_nonfinite_command(speed, curvature):
    with pytest.raises(ValueError):
        ackermann_command(speed, curvature, geometry())


@pytest.mark.parametrize("value", [True, "0.2", 0, -1, np.nan])
def test_invalid_tracker_configuration_is_rejected(value):
    with pytest.raises(ValueError):
        TrackerConfig(cruise_speed_mps=value)


@pytest.mark.parametrize("value", [True, "0.64", 0, -1, np.nan])
def test_invalid_geometry_is_rejected(value):
    with pytest.raises(ValueError):
        AckermannGeometry(value, 0.51, 0.135, 0.45, 8)


def test_measured_motion_holds_cusp_even_if_commanded_stop_is_small():
    tracker = RearAxlePathTracker(
        [[0, 0, 0], [1, 0, 0], [0.5, 0, 0]], [1, 1, -1], [0, 0, 0]
    )
    for x in np.linspace(0, 1, 21):
        command = tracker.update([x, 0, 0], 0.1, 0.02)
    for _ in range(120):
        command = tracker.update([1, 0, 0], 0.1, 0.02)
        assert command.status == "braking"
        assert command.speed_mps >= 0
        assert command.segment_index == 0
    assert command.speed_mps == 0
    stopped = tracker.update([1, 0, 0], 0, 0.02)
    assert stopped.speed_mps == 0
    assert tracker.update([1, 0, 0], 0, 0.02).speed_mps < 0


def test_lagged_speed_actuator_stops_at_endpoint_at_120hz():
    poses, directions, curvatures = route()
    tracker = RearAxlePathTracker(
        poses,
        directions,
        curvatures,
        TrackerConfig(
            cruise_speed_mps=0.14,
            max_acceleration_mps2=0.18,
            position_tolerance_m=0.008,
            yaw_tolerance_rad=0.02,
        ),
    )
    pose = poses[0].copy()
    speed = previous_command = 0.0
    dt = 1 / 120
    for _ in range(9000):
        command = tracker.update(pose, speed, dt)
        assert abs(command.speed_mps - previous_command) <= 0.18 * dt + 1e-10
        previous_command = command.speed_mps
        # A 100ms actuator response must not weaken the requested deceleration.
        speed += (command.speed_mps - speed) * dt / 0.1
        change = speed * command.curvature_inv_m * dt
        pose[:2] += (
            speed
            * dt
            * np.array([np.cos(pose[2] + change / 2), np.sin(pose[2] + change / 2)])
        )
        pose[2] += change
        if command.status in {"arrived", "failed"}:
            break
    assert command.status == "arrived", command
    assert np.linalg.norm(pose[:2] - poses[-1, :2]) < 0.008


def test_curved_cusp_keeps_endpoint_heading_before_reversing():
    angle = np.linspace(0, 0.8, 161)
    arc = np.column_stack((np.sin(angle) / 0.5, (1 - np.cos(angle)) / 0.5, angle))
    poses = np.vstack((arc, arc[-2::-1]))
    directions = np.r_[np.ones(len(arc)), -np.ones(len(arc) - 1)]
    tracker = RearAxlePathTracker(
        poses,
        directions,
        np.full(len(poses), 0.5),
        TrackerConfig(
            cruise_speed_mps=0.14,
            max_curvature_inv_m=0.62,
            lookahead_m=0.28,
            position_tolerance_m=0.008,
            yaw_tolerance_rad=0.02,
        ),
    )
    final, history = rollout(tracker, poses[0])
    assert history[-1].status == "arrived", history[-1]
    assert np.linalg.norm(final[:2] - poses[-1, :2]) < 0.008


@pytest.mark.parametrize("cruise_speed", [0.30, 0.35])
@pytest.mark.parametrize("direction", [1, -1])
@pytest.mark.parametrize("has_cusp", [False, True])
def test_faster_cruise_with_speed_and_steering_lag_arrives(
    cruise_speed, direction, has_cusp
):
    # Independent analytic path; curved cusp prevents a final straight from
    # concealing endpoint heading error and premature physical gear changes.
    angle = np.linspace(0, direction * 0.8, 161)
    arc = np.column_stack((np.sin(angle) / 0.5, (1 - np.cos(angle)) / 0.5, angle))
    if has_cusp:
        poses = np.vstack((arc, arc[-2::-1]))
        directions = np.r_[
            np.full(len(arc), direction), np.full(len(arc) - 1, -direction)
        ]
        curvatures = np.full(len(poses), 0.5)
    else:
        distance = np.linspace(0.01, 0.8, 81)
        poses = np.vstack(
            (
                arc,
                arc[-1]
                + np.column_stack(
                    (
                        direction * distance * np.cos(angle[-1]),
                        direction * distance * np.sin(angle[-1]),
                        np.zeros(len(distance)),
                    )
                ),
            )
        )
        directions = np.full(len(poses), direction)
        curvatures = np.r_[np.full(len(arc), 0.5), np.zeros(len(distance))]
    tracker = RearAxlePathTracker(
        poses,
        directions,
        curvatures,
        TrackerConfig(
            cruise_speed_mps=cruise_speed,
            max_curvature_inv_m=0.62,
            max_acceleration_mps2=0.30,
            lookahead_m=0.28,
            position_tolerance_m=0.008,
            yaw_tolerance_rad=0.02,
            stop_speed_mps=0.012,
            max_cross_track_error_m=0.35,
        ),
    )
    pose = poses[0].copy()
    speed = actual_steering = steering_setpoint = previous_command = 0.0
    maximum_speed = 0.0
    saw_reverse = False
    dt = 1 / 120
    for _ in range(9000):
        command = tracker.update(pose, speed, dt)
        assert abs(command.speed_mps - previous_command) <= 0.30 * dt + 1e-10
        if command.speed_mps * direction < 0 and not saw_reverse:
            assert previous_command == 0
            assert abs(speed) <= 0.012
            saw_reverse = True
        previous_command = command.speed_mps
        speed += (command.speed_mps - speed) * dt / 0.1
        maximum_speed = max(maximum_speed, abs(speed))
        target_steering = np.arctan(0.64 * command.curvature_inv_m)
        steering_setpoint += np.clip(target_steering - steering_setpoint, -dt, dt)
        actual_steering += (steering_setpoint - actual_steering) * dt / 0.1
        change = speed * np.tan(actual_steering) / 0.64 * dt
        pose[:2] += (
            speed
            * dt
            * np.array([np.cos(pose[2] + change / 2), np.sin(pose[2] + change / 2)])
        )
        pose[2] += change
        if command.status in {"arrived", "failed"}:
            break
    assert command.status == "arrived", command
    assert np.linalg.norm(pose[:2] - poses[-1, :2]) < 0.008
    assert abs(pose[2] - poses[-1, 2]) < 0.02
    assert maximum_speed > 0.95 * cruise_speed
    assert saw_reverse == has_cusp


DOCKING = dict(position_tolerance_m=0.008, yaw_tolerance_rad=0.02)
CUSP_PATH = (
    [[0, 0, 0], [0.5, 0, 0], [1, 0, 0], [0.6, 0, 0], [0.3, 0, 0]],
    [1, 1, 1, -1, -1],
    [0, 0, 0, 0, 0],
)


def drive_past_cusp_off_line(tracker, lateral=0.015, yaw=0.03):
    # Reach the gear-change cusp at x = 1 but 15 mm to the side and slightly
    # yawed, then stand still: the stop a real truck produced (2026-09-26).
    for x in np.linspace(0, 1, 51):
        tracker.update([x, lateral, yaw], 0.1, 0.02)
    return [tracker.update([1.0, lateral, yaw], 0.0, 0.02) for _ in range(200)]


def test_without_cusp_tolerance_an_offset_stop_at_a_cusp_holds_forever():
    tracker = RearAxlePathTracker(*CUSP_PATH, TrackerConfig(**DOCKING))
    commands = drive_past_cusp_off_line(tracker)
    # After the commanded speed has slewed down it stays at zero, never
    # reverses and never fails: the deadlock seen in Isaac.
    assert all(c.speed_mps == 0 for c in commands[-100:])
    assert all(c.speed_mps >= 0 for c in commands)
    assert all(c.status != "failed" for c in commands)


def test_cusp_tolerance_releases_an_offset_stop_into_the_next_leg():
    config = TrackerConfig(
        **DOCKING, cusp_position_tolerance_m=0.03, cusp_yaw_tolerance_rad=0.05
    )
    tracker = RearAxlePathTracker(*CUSP_PATH, config)
    commands = drive_past_cusp_off_line(tracker)
    assert any(c.speed_mps < 0 for c in commands)
    assert all(c.status != "failed" for c in commands)


def test_cusp_tolerance_does_not_loosen_the_final_goal():
    config = TrackerConfig(
        **DOCKING, cusp_position_tolerance_m=0.03, cusp_yaw_tolerance_rad=0.05
    )
    tracker = RearAxlePathTracker([[0, 0, 0], [1, 0, 0]], [1, 1], [0, 0], config)
    for x in np.linspace(0, 1, 51):
        tracker.update([x, 0.015, 0.0], 0.1, 0.02)
    for _ in range(50):
        command = tracker.update([1.0, 0.015, 0.0], 0.0, 0.02)
    assert command.status != "arrived"


@pytest.mark.parametrize(
    "field", ["cusp_position_tolerance_m", "cusp_yaw_tolerance_rad"]
)
@pytest.mark.parametrize("value", [True, 0, -0.01, np.nan])
def test_invalid_cusp_tolerance_is_rejected(field, value):
    with pytest.raises(ValueError):
        TrackerConfig(**{field: value})


def stop_near_final_goal(config, along, across=0.0):
    # Drive the one-metre straight, then stand still `along` metres past the
    # goal (negative = short of it) and `across` metres to its left.
    tracker = RearAxlePathTracker([[0, 0, 0], [1, 0, 0]], [1, 1], [0, 0], config)
    for x in np.linspace(0, 1 + along, 51):
        tracker.update([x, across, 0.0], 0.1, 0.02)
    return [tracker.update([1 + along, across, 0.0], 0.0, 0.02) for _ in range(100)]


def test_without_overshoot_tolerance_stopping_just_past_the_goal_deadlocks():
    commands = stop_near_final_goal(TrackerConfig(**DOCKING), along=0.009)
    assert all(c.status != "arrived" for c in commands)
    assert all(c.speed_mps == 0 for c in commands[-50:])


def test_overshoot_tolerance_accepts_a_stop_just_past_the_goal():
    config = TrackerConfig(**DOCKING, overshoot_tolerance_m=0.03)
    assert stop_near_final_goal(config, along=0.009)[-1].status == "arrived"
    assert stop_near_final_goal(config, along=0.025)[-1].status == "arrived"
    assert stop_near_final_goal(config, along=0.035)[-1].status != "arrived"


def test_overshoot_tolerance_keeps_the_sideways_and_short_limits():
    config = TrackerConfig(**DOCKING, overshoot_tolerance_m=0.03)
    assert stop_near_final_goal(config, along=0.0, across=0.012)[-1].status != (
        "arrived"
    )
    # Short of the goal the tracker keeps driving instead of accepting.
    short = stop_near_final_goal(config, along=-0.02)
    assert short[-1].status != "arrived"
    assert any(c.speed_mps > 0 for c in short)


@pytest.mark.parametrize("value", [True, 0, -0.01, np.nan])
def test_invalid_overshoot_tolerance_is_rejected(value):
    with pytest.raises(ValueError):
        TrackerConfig(overshoot_tolerance_m=value)


def straight_then_arc(direction=1):
    # 6 m straight, then a quarter turn of radius 2 m (curvature 0.5).
    straight = np.column_stack((np.linspace(0, 6, 121), np.zeros(121), np.zeros(121)))
    angle = np.linspace(0, np.pi / 2, 80)[1:]
    arc = np.column_stack((6 + 2 * np.sin(angle), 2 * (1 - np.cos(angle)), angle))
    poses = np.vstack((straight, arc))
    if direction < 0:
        poses = poses[::-1]
    curvature = np.r_[np.zeros(121), np.full(79, 0.5)]
    if direction < 0:
        curvature = curvature[::-1] * -1
    curvature[0] = curvature[1]
    return poses, np.full(len(poses), direction), curvature


def test_lateral_acceleration_limit_slows_down_before_and_on_a_curve():
    poses, directions, curvatures = straight_then_arc()
    config = TrackerConfig(
        cruise_speed_mps=2.2,
        max_acceleration_mps2=0.5,
        max_curvature_inv_m=0.62,
        max_lateral_acceleration_mps2=0.5,
    )
    tracker = RearAxlePathTracker(poses, directions, curvatures, config)
    _, history = rollout(tracker, poses[0])
    arc_limit = np.sqrt(0.5 / 0.5)  # 1.0 m/s on the 2 m radius
    on_arc = [c.speed_mps for c in history if c.progress_m > 6.0 + 0.05]
    assert on_arc and max(on_arc) <= arc_limit + 1e-6
    assert max(c.speed_mps for c in history) > 1.5  # the straight is still fast
    assert history[-1].status == "arrived"


def test_without_a_lateral_limit_the_curve_is_taken_at_cruise():
    poses, directions, curvatures = straight_then_arc()
    config = TrackerConfig(
        cruise_speed_mps=2.2, max_acceleration_mps2=0.5, max_curvature_inv_m=0.62
    )
    tracker = RearAxlePathTracker(poses, directions, curvatures, config)
    _, history = rollout(tracker, poses[0])
    assert max(c.speed_mps for c in history if 6.2 < c.progress_m < 8.0) > 1.5


def test_reverse_speed_cap_limits_only_reversing():
    poses, directions, curvatures = straight_then_arc(direction=-1)
    config = TrackerConfig(
        cruise_speed_mps=2.2,
        max_acceleration_mps2=0.5,
        max_curvature_inv_m=0.62,
        max_reverse_speed_mps=0.6,
    )
    tracker = RearAxlePathTracker(poses, directions, curvatures, config)
    _, history = rollout(tracker, poses[0])
    assert min(c.speed_mps for c in history) >= -0.6 - 1e-9
    assert history[-1].status == "arrived"


@pytest.mark.parametrize(
    "field", ["max_lateral_acceleration_mps2", "max_reverse_speed_mps"]
)
@pytest.mark.parametrize("value", [True, 0, -1.0, np.nan])
def test_invalid_speed_limits_are_rejected(field, value):
    with pytest.raises(ValueError):
        TrackerConfig(**{field: value})


def test_nominal_duration_is_length_over_cruise_without_caps():
    poses, directions, curvatures = straight_then_arc()
    config = TrackerConfig(cruise_speed_mps=2.0, max_curvature_inv_m=0.62)
    tracker = RearAxlePathTracker(poses, directions, curvatures, config)
    length = np.sum(np.hypot(*np.diff(poses[:, :2], axis=0).T))
    assert tracker.nominal_duration_s() == pytest.approx(length / 2.0)


def test_nominal_duration_counts_the_slow_curve():
    poses, directions, curvatures = straight_then_arc()
    config = TrackerConfig(
        cruise_speed_mps=2.0,
        max_curvature_inv_m=0.62,
        max_lateral_acceleration_mps2=0.125,  # 0.5 m/s on the 2 m arc
    )
    tracker = RearAxlePathTracker(poses, directions, curvatures, config)
    arc_time = (np.pi / 2 * 2) / 0.5
    assert tracker.nominal_duration_s() > arc_time
