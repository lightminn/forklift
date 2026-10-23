"""The recorded final-goal creep stall (docs/validation/2026-10-23-joint-angle-odometry-rejudge.md section 4):
jao3_tq_on_s7, the return_home leg's end. The real tracker, built from the run's own return_home path and tracker
configuration, is followed kinematically to that leg's end; the recorded stall's control poses are then fed standing.
It stays "tracking" with 0.024 mm left and only the 1.5 x remaining creep: the end lies 8.32 mm across, inside the
leg-end floor's 0.5 mm margin above the 8 mm tolerance, so the floor is off, and 8.35 mm from the goal is never an
arrival. The truck stood 53 s at -3.6e-5 m/s with its team-spec counts frozen and ended obstacle_no_progress.

The fixture tests/fixtures/goal_creep_stall_jao3_tq_on_s7.json is cut from the run's records (its "source")."""

import json
import math
from pathlib import Path

import numpy as np
import pytest

from forklift_core.control import RearAxlePathTracker, TrackerConfig
from forklift_core.control import path_tracking

ROOT = Path(__file__).resolve().parents[3]
FIXTURE = ROOT / "tests/fixtures/goal_creep_stall_jao3_tq_on_s7.json"
DT = 1 / 120


def _tracker_at_the_end():
    """The recorded tracker followed kinematically (command = measured speed) into its last LEG_END_SLACK_M."""
    f = json.loads(FIXTURE.read_text())
    p = f["path"]
    tracker = RearAxlePathTracker(np.asarray(p["poses"], float), np.asarray(p["directions"], int),
                                  np.asarray(p["curvatures_inv_m"], float), TrackerConfig(**f["tracker_config"]))
    pose = np.asarray(p["poses"][0], float)
    speed = 0.0
    for _ in range(20000):
        command = tracker.update(pose, speed, DT)
        assert command.status not in ("failed", "arrived")
        if tracker.leg_remaining_m() < path_tracking.LEG_END_SLACK_M and tracker.remaining_to_goal_m() < 0.05:
            return f, tracker
        speed = command.speed_mps
        pose = pose + DT * speed * np.array([math.cos(pose[2]), math.sin(pose[2]), command.curvature_inv_m])
    raise AssertionError("the follow did not reach the leg's end")


def _stand(tracker, poses, ticks):
    """Feed the recorded stall poses in turn, standing, for this many ticks; the last command."""
    command = None
    for k in range(ticks):
        command = tracker.update(np.asarray(poses[k % len(poses)], float), 0.0, DT)
    return command


def test_the_recorded_stall_keeps_only_a_micro_creep_that_never_arrives():
    f, tracker = _tracker_at_the_end()
    command = _stand(tracker, f["stall_control_poses"], 2400)  # 20 s standing
    remaining = tracker.remaining_to_goal_m()
    assert command.status == "tracking"
    assert not command.at_cusp
    assert remaining == pytest.approx(2.41e-5, abs=1e-6)
    assert command.position_error_m > f["tracker_config"]["position_tolerance_m"]
    assert abs(command.yaw_error_rad) > f["tracker_config"]["yaw_tolerance_rad"]
    # Only the creep: 1.5 x remaining, the recorded command; the floor is off (the end 8.32 mm across lies inside
    # tolerance + LEG_END_ACROSS_MARGIN_M).
    assert command.speed_mps == pytest.approx(f["recorded_command_mps"], rel=0.05)
    assert abs(command.speed_mps) == pytest.approx(1.5 * remaining, rel=1e-6)


def test_the_recorded_stall_is_out_of_reach_at_every_stall_pose():
    # Plan 2026-10-24-goal-creep-stall G1: the filter is true at each of the stall's control poses (8.32 mm across,
    # 8.15 mm after the curvature allowance over the 30 mm overshoot: above the 8 mm tolerance), and the pose stands
    # 0.1-0.9 mm short of the end normal -- R1's "at the progress point" is not met, R2 takes it.
    f, tracker = _tracker_at_the_end()
    for pose in f["stall_control_poses"]:
        tracker.update(np.asarray(pose, float), 0.0, DT)
        assert tracker.leg_end_out_of_reach()
        assert 5e-5 < tracker.distance_to_end_normal_m() < 1e-3
