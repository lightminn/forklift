"""The leg-failures V2 steering plant (tools/d7_reapproach_steer.py): the joint curvature inverts the Ackermann
command, an ideal plant arrives on an S, a slow steering rate leaves the S's leg end with the heading error the plant
exists to show, and the Isaac stop's leg is the nearest leg end."""

import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

from d7_reapproach_steer import isaac_offsets, joint_curvature, simulate  # noqa: E402

from forklift_core.control.path_tracking import AckermannGeometry, TrackerConfig, ackermann_command  # noqa: E402

GEOMETRY = AckermannGeometry(wheelbase_m=0.66, track_m=0.53, wheel_radius_m=0.125, max_steering_rad=0.2618,
                             max_wheel_rate_rad_s=8.0)
CONFIG = TrackerConfig(cruise_speed_mps=0.3, max_curvature_inv_m=0.36, max_acceleration_mps2=0.3, lookahead_m=0.28,
                       position_tolerance_m=0.03, yaw_tolerance_rad=0.05, stop_speed_mps=0.012,
                       max_cross_track_error_m=0.35, cusp_position_tolerance_m=0.03, cusp_yaw_tolerance_rad=0.05,
                       cusp_brake_window_m=0.008, overshoot_tolerance_m=0.03)
CONSTANTS = {"rate_rad_s": 1.0, "decel_mps2": 1.5, "max_wheel_rate_rad_s": 8.0}


def s_path(kappa=0.29, arc=0.9, tail=0.6, step=0.01):
    """Forward: an arc at -kappa, at once an arc at +kappa (the reversal), then a straight."""
    poses, curv = [np.array([0.0, 0.0, 0.0])], [-kappa]
    for k_, length in ((-kappa, arc), (kappa, arc), (0.0, tail)):
        for _ in range(int(round(length / step))):
            x, y, yaw = poses[-1]
            mid = yaw + k_ * step / 2
            poses.append(np.array([x + step * math.cos(mid), y + step * math.sin(mid), yaw + k_ * step]))
            curv.append(k_)
    poses = np.array(poses)
    return poses, np.ones(len(poses), np.int8), np.array(curv)


def test_the_joint_curvature_inverts_the_ackermann_command():
    for kappa in (-0.36, -0.2, 0.0, 0.1, 0.29):
        steering = ackermann_command(0.3, kappa, GEOMETRY).steering_rad
        assert joint_curvature(steering, GEOMETRY) == pytest.approx(kappa, abs=1e-9)


def test_an_ideal_plant_arrives_and_a_slow_steering_rate_leaves_a_heading_error():
    poses, dirs, curv = s_path()
    ideal = simulate(poses, dirs, curv, CONFIG, poses[0], GEOMETRY, {**CONSTANTS, "rate_rad_s": 50.0},
                     delay_s=DT, response=None)
    assert ideal["status"] == "arrived"
    slow = simulate(poses, dirs, curv, replace(CONFIG, position_tolerance_m=0.008, yaw_tolerance_rad=0.002),
                    poses[0], GEOMETRY, {**CONSTANTS, "rate_rad_s": 0.05}, delay_s=0.12)
    assert slow["status"] != "arrived"


DT = 1.0 / 120.0


def test_the_isaac_stop_is_judged_on_the_nearest_leg_end():
    poses = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.0, 0.2, 0.1], [0.0, 0.4, 0.2]])
    case = {"path": {"poses": poses.tolist(), "directions": [1, 1, 1, -1, -1]}, "stop_pose": [1.95, 0.0, -0.06]}
    out = isaac_offsets(case)
    assert out["leg"] == 0 and out["legs"] == 2 and out["heading_error_rad"] == pytest.approx(-0.06)
    assert out["progress_fraction"] == pytest.approx(1.0)
    assert isaac_offsets({"path": case["path"], "stop_pose": None}) is None
