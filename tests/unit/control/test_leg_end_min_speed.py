"""The leg end the tracker's 1.5 x remaining creep cannot finish (docs/plans/2026-10-22-leg-end-missed-stop.md).

The speed cap 1.5 x remaining only approaches zero, and every receiver of a stop outside tolerance (the stall
replan, the docking stop, the cusp stall K2, observe's loose arrival) waits for the remaining length to reach zero.
Odometry noise or an overshoot used to get it there. Team-spec counted odometry stands still exactly (Isaac
jao_tq_on_s1: 6.1e-5 m/s for 58 s), a deadband plant does not move on such a command, and reported odometry did not
see a 3 mm/s creep (Riibotics docking stops). In the last 4 mm of such a leg end the tracker now drives at a floor.

The closed loop here: the tracker, an Ackermann kinematic plant that does not move under a deadband, the odometry
(team-spec 0.192 mm counts, or reported noise with the low-speed rates lost) and the runner's stop detector.
"""

import math

import numpy as np
import pytest

from forklift_core.control import RearAxlePathTracker, TrackerConfig
from forklift_core.control import path_tracking
from forklift_core.localization.slam_pose import StopDetector

DT = 1 / 120
COUNT_M = 2 * math.pi / 4096 * 0.125  # one MT6701 x4 count on a 0.125 m wheel
# The measured runner's transport / return followers (run_transport.py), goal tolerance 8 mm.
BASE = dict(cruise_speed_mps=0.6, max_curvature_inv_m=0.36, max_acceleration_mps2=0.3, lookahead_m=0.28,
            position_tolerance_m=0.008, yaw_tolerance_rad=0.05, stop_speed_mps=0.012, max_cross_track_error_m=0.35,
            cusp_position_tolerance_m=0.03, cusp_yaw_tolerance_rad=0.05, cusp_brake_window_m=0.008,
            overshoot_tolerance_m=0.03)
GOAL = TrackerConfig(**BASE)
DOCK_STOP = TrackerConfig(**{**BASE, "position_tolerance_m": 0.03})  # arm_docking()'s stop: 30 mm
INSERT = TrackerConfig(**{**BASE, "overshoot_tolerance_m": None, "yaw_tolerance_rad": 0.02, "cruise_speed_mps": 0.055})


def straight(direction=1, n=26):
    x = np.linspace(0.0, 1.0, n) * direction
    return np.column_stack((x, np.zeros(n), np.zeros(n))), np.full(n, direction, np.int8), np.zeros(n)


def curved(kappa=0.36, n=26):
    s = np.linspace(0.0, 1.0, n)
    yaw = kappa * s
    return np.column_stack((np.sin(yaw) / kappa, (1 - np.cos(yaw)) / kappa, yaw)), np.ones(n, np.int8), np.full(n, kappa)


def along(upto, lateral, yaw, direction=1):
    return [[direction * x, lateral, yaw] for x in np.linspace(0.0, upto, 40)]


def drive(path, config, setup, start, *, odometry="team_spec", seconds=10.0, deadband=0.001, blind_mps=0.0, seed=0):
    """Run the loop from ``start`` after building the tracker's progress over ``setup``. Returns (outcome, time,
    tracker, last command): "arrived", "end" (remaining zero, command zero and the stop detector stopped -- the
    receivers' state) or "stuck"."""
    tracker = RearAxlePathTracker(*path, config)
    for pose in setup:
        tracker.update(pose, 0.1, 0.02)
    tracker.restart_speed_slew(0.0)
    truth = np.array(start, float)
    odom = truth.copy()
    rng = np.random.default_rng(seed)
    stop = StopDetector(tick_s=DT)
    counted = 0.0
    command = None
    for k in range(int(seconds / DT)):
        command = tracker.update(odom, stop.tracker_speed(0.012), DT)
        if command.status == "arrived":
            return "arrived", k * DT, tracker, command
        if (command.status == "tracking" and tracker.remaining_to_goal_m() <= 1e-6 and command.speed_mps == 0.0
                and stop.stopped):
            return "end", k * DT, tracker, command
        v = command.speed_mps if abs(command.speed_mps) >= deadband else 0.0
        step = v * DT
        truth = truth + [step * math.cos(truth[2]), step * math.sin(truth[2]), step * command.curvature_inv_m]
        if odometry == "team_spec":
            counted += step
            n = math.floor(counted / COUNT_M) if counted >= 0 else -math.floor(-counted / COUNT_M)
            ds = n * COUNT_M
            counted -= ds
            speed = ds / DT
        else:
            seen = 0.0 if abs(v) < blind_mps else v
            speed = seen + rng.normal(0.0, 0.2 * 0.125 / math.sqrt(2))
            ds = speed * DT
        odom = odom + [ds * math.cos(odom[2]), ds * math.sin(odom[2]), ds * command.curvature_inv_m]
        stop.update(commanded_speed=command.speed_mps, speed=speed, yaw_rate=ds * command.curvature_inv_m / DT)
    return "stuck", seconds, tracker, command


# --- The defect, reproduced in both odometry models (plan 1.4).

def test_team_spec_odometry_finishes_a_goal_missed_sideways():
    # jao_tq_on_s1: 44 mm across the return goal, 0.04 mm of path left; the command stood at 6e-5 m/s.
    outcome, t, tracker, command = drive(straight(), GOAL, along(0.99996, 0.044, 0.0), [0.99996, 0.044, 0.0],
                                         seconds=20)
    assert outcome == "end" and t < 1.0
    assert tracker.remaining_to_goal_m() == 0.0 and command.speed_mps == 0.0


def test_a_deadband_above_the_old_creep_no_longer_holds_the_truck():
    outcome, t, _, _ = drive(straight(), GOAL, along(0.99996, 0.044, 0.0), [0.99996, 0.044, 0.0], seconds=20,
                             deadband=0.005)
    assert outcome == "end" and t < 1.0


def test_reported_odometry_blind_to_the_creep_reaches_the_docking_stop():
    # n2_line_1_s1-like: the progress latched 2.6 mm from the stop, the pose 50 mm behind it; the reported rates
    # read nothing under 6.25 mm/s.
    setup = along(0.9974, 0.02, 0.0) + [[0.9474, 0.02, 0.0]]
    outcome, t, _, _ = drive(straight(), DOCK_STOP, setup, [0.9474, 0.02, 0.0], odometry="reported", seconds=30,
                             blind_mps=0.00625)
    assert outcome in ("arrived", "end") and t < 10.0


def test_reported_noise_still_reaches_the_end_as_before():
    outcome, t, _, _ = drive(straight(), GOAL, along(0.99996, 0.044, 0.0), [0.99996, 0.044, 0.0],
                             odometry="reported", seconds=20)
    assert outcome == "end" and t < 1.0


# --- Arrivals the earlier designs broke (plan 2, review record): no later than the old tracker.

@pytest.mark.parametrize("name, path, setup, start, old_s", [
    ("7.5 mm short, 8.05 across, heading in", straight(), along(0.9925, 0.00805, -0.02), [0.9925, 0.00805, -0.02], 1.517),
    ("its reverse", straight(-1), along(0.9925, 0.00805, 0.02, -1), [-0.9925, 0.00805, 0.02], 1.717),
    ("progress 0.9975, pose 0.980", straight(), along(0.9975, 0.00805, -0.02) + [[0.980, 0.00805, -0.02]],
     [0.980, 0.00805, -0.02], 4.975),
    ("progress 0.998, pose 0.996", straight(), along(0.998, 0.00805, -0.02) + [[0.996, 0.00805, -0.02]],
     [0.996, 0.00805, -0.02], 1.650),
    ("ordinary approach 2 mm across", straight(), along(0.9, 0.002, 0.0), [0.9, 0.002, 0.0], 2.1334),
    ("ordinary approach 7.5 mm across", straight(), along(0.9, 0.0075, 0.0), [0.9, 0.0075, 0.0], 2.5334),
])
def test_arrivals_are_kept(name, path, setup, start, old_s):
    outcome, t, _, _ = drive(path, GOAL, setup, start)
    assert outcome == "arrived" and t <= old_s + 1e-9


def test_a_curved_end_arrives_as_before():
    s_end = 1.0 - 0.00802
    yaw = 0.36 * s_end
    start = [math.sin(yaw) / 0.36 + 0.005 * math.sin(yaw), (1 - math.cos(yaw)) / 0.36 - 0.005 * math.cos(yaw), yaw]
    setup = [[math.sin(0.36 * s) / 0.36, (1 - math.cos(0.36 * s)) / 0.36, 0.36 * s] for s in np.linspace(0, s_end, 40)]
    outcome, t, _, _ = drive(curved(), GOAL, setup, start)
    assert outcome == "arrived" and t <= 0.425 + 1e-9


# --- Boundary arrivals on an ideal plant (Codex 4th-6th reviews): the floor must not take them past the circle.

def test_an_insert_inside_the_tolerance_still_arrives():
    outcome, t, _, _ = drive(straight(), INSERT, along(0.997, 0.0079427, 0.02), [0.997, 0.0079427, 0.02],
                             deadband=0.0, seconds=20)
    assert outcome == "arrived" and t <= 1.942 + 1e-9


def test_an_insert_a_hair_outside_keeps_the_old_creep():
    # 8.000845 mm across with a straight heading: within the 0.5 mm margin, no floor -- the old approach to the end.
    outcome, t, _, _ = drive(straight(), INSERT, along(0.997, 0.008000845, 0.0), [0.997, 0.008000845, 0.0],
                             deadband=0.0, seconds=20)
    assert outcome == "end" and t > 2.5


def test_a_low_braking_acceleration_still_arrives():
    low = TrackerConfig(**{**BASE, "overshoot_tolerance_m": None, "yaw_tolerance_rad": 0.02,
                           "cruise_speed_mps": 0.055, "max_acceleration_mps2": 0.03})
    setup = along(0.9995, 0.009, 0.0) + [[0.995, 0.008002, 0.0]]
    outcome, t, _, _ = drive(straight(), low, setup, [0.995, 0.008002, 0.0], deadband=0.0, seconds=14)
    assert outcome == "arrived" and t < 7.292


# --- The rule itself, one tick at a time.

def stalled(config=GOAL, lateral=0.044, yaw=0.0, path=None, end_x=0.99996):
    tracker = RearAxlePathTracker(*(path or straight()), config)
    for pose in along(end_x, lateral, yaw):
        tracker.update(pose, 0.1, 0.02)
    tracker.restart_speed_slew(path_tracking.LEG_END_MIN_SPEED_MPS)
    return tracker


def test_the_floor_applies_to_a_leg_end_missed_sideways():
    command = stalled().update([0.99996, 0.044, 0.0], 0.0, DT)
    assert command.status == "tracking"
    assert command.speed_mps == pytest.approx(path_tracking.LEG_END_MIN_SPEED_MPS)


def test_the_floor_follows_the_leg_direction():
    tracker = RearAxlePathTracker(*straight(-1), GOAL)
    for pose in along(0.99996, 0.044, 0.0, -1):
        tracker.update(pose, -0.1, 0.02)
    tracker.restart_speed_slew(-path_tracking.LEG_END_MIN_SPEED_MPS)
    command = tracker.update([-0.99996, 0.044, 0.0], 0.0, DT)
    assert command.speed_mps == pytest.approx(-path_tracking.LEG_END_MIN_SPEED_MPS)


def test_no_floor_once_the_path_has_run_out():
    tracker = stalled(end_x=1.0)
    tracker.restart_speed_slew(0.0)
    assert tracker.update([1.0, 0.044, 0.0], 0.0, DT).speed_mps == 0.0


def test_no_floor_when_the_heading_is_turned_away():
    # Heading pi: a floor would only move the truck (Codex 4th review, 200 mm without progress).
    tracker = stalled()
    tracker.restart_speed_slew(0.0)
    command = tracker.update([0.99996, 0.044, math.pi], 0.0, DT)
    assert abs(command.speed_mps) < path_tracking.LEG_END_MIN_SPEED_MPS / 10


def settled_speed(tracker, pose, ticks=12):
    """The command after the slew has had time to reach the target, the pose held."""
    tracker.restart_speed_slew(0.0)
    for _ in range(ticks):
        command = tracker.update(pose, 0.0, DT)
    return abs(command.speed_mps)


def test_no_floor_for_an_end_a_straight_run_brings_inside():
    # 11.2 mm across, 3.5 mm to go, turned -0.7 rad towards the path: a straight run (tan) ends 8.25 mm across,
    # inside the tolerance plus margin; a sin extrapolation would read 8.95 mm and add the floor.
    tracker = RearAxlePathTracker(*straight(), GOAL)
    for pose in along(0.9965, 0.0112, 0.0):
        tracker.update(pose, 0.1, 0.02)
    assert settled_speed(tracker, [0.9965, 0.0112, -0.7]) == pytest.approx(1.5 * 0.0035, abs=1e-6)


def test_the_floor_never_exceeds_the_configured_caps():
    for config in (TrackerConfig(**{**BASE, "cruise_speed_mps": 0.005}),
                   TrackerConfig(**{**BASE, "max_acceleration_mps2": 0.03})):
        tracker = stalled(config)
        tracker.restart_speed_slew(0.0)
        speeds = [abs(tracker.update([0.99996, 0.044, 0.0], 0.0, DT).speed_mps) for _ in range(20)]
        cap = min(path_tracking.LEG_END_MIN_SPEED_MPS, config.cruise_speed_mps,
                  math.sqrt(2 * config.max_acceleration_mps2 * path_tracking.LEG_END_BEHIND_M))
        assert max(speeds) <= cap + 1e-12


def test_behind_the_progress_counts_only_on_the_last_segment():
    # A 0.8 m arc kappa 0.36 drawn with three samples, 0.802 / 0.002 / 0 m from the end; the projection 2 mm short of
    # the first segment's end, 7.9 mm across, heading 0: no real lag, the last-segment axis would read 1.12 mm.
    k = 0.36
    s = np.array([0.802, 0.002, 0.0])
    yaw = -k * s
    poses = np.column_stack((np.sin(yaw) / k, (1 - np.cos(yaw)) / k, yaw))  # a circle ending at the origin, heading 0
    tracker = RearAxlePathTracker(poses, np.ones(3, np.int8), np.full(3, k), GOAL)
    seg = poses[1, :2] - poses[0, :2]
    u = seg / np.linalg.norm(seg)
    normal = np.array([-u[1], u[0]])
    p = poses[1, :2] - 0.002 * u - 0.0079 * normal
    for f in np.linspace(0.0, 1.0, 40):
        tracker.update([*(poses[0, :2] + f * (p - poses[0, :2])), 0.0], 0.1, 0.02)
    remaining = tracker.remaining_to_goal_m()
    assert remaining < path_tracking.LEG_END_SLACK_M
    assert settled_speed(tracker, [*p, 0.0]) <= 1.5 * remaining + 1e-9  # the old creep, not the floor


def test_arrival_and_the_brake_against_motion_are_unchanged():
    # Inside the tolerance: the old arrival. Measured motion back along the leg above the stop speed: the old brake.
    tracker = stalled(lateral=0.002, end_x=0.995)
    tracker.restart_speed_slew(0.0)
    command = tracker.update([0.995, 0.002, 0.0], 0.0, DT)
    assert command.status in ("braking", "arrived") and command.speed_mps == 0.0
    tracker = stalled()
    command = tracker.update([0.99996, 0.044, 0.0], -0.03, DT)
    assert command.status == "braking" and tracker.braking_against_motion
