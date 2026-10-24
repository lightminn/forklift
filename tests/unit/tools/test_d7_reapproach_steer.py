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


def test_the_control_error_is_taken_in_the_control_frame_aligned_at_the_attempt_start():
    """Codex plan round 9 P2: the runner's tracker reads the SLAM control pose. The series is control minus the truth
    carried by the transform that makes them agree at the first row -- a frame offset alone is no error."""
    from d7_reapproach_steer import control_error_series

    rot = math.pi / 2
    rows = []
    for t, (tx, ty), extra in ((10.0, (1.0, 0.0), (0.0, 0.0)), (11.0, (2.0, 0.0), (0.0, 0.0)),
                               (12.0, (3.0, 0.0), (0.0, 0.02))):
        # control = truth rotated by 90 deg and shifted (5, 5), plus an estimate error at the last row
        cx, cy = 5.0 + math.cos(rot) * tx - math.sin(rot) * ty, 5.0 + math.sin(rot) * tx + math.cos(rot) * ty
        rows.append([t, cx + extra[0], cy + extra[1], rot, tx, ty, 0.0])
    series = control_error_series(rows, 10.0, 12.0)
    assert [r[0] for r in series] == [0.0, 1.0, 2.0]
    np.testing.assert_allclose(np.asarray(series)[:, 1:], [[0, 0, 0], [0, 0, 0], [0, 0.02, 0]], atol=1e-12)


def test_a_fed_lateral_error_moves_the_truck_the_other_way():
    poses = np.column_stack((np.linspace(0, 2, 201), np.zeros(201), np.zeros(201)))
    dirs, curv = np.ones(201, np.int8), np.zeros(201)
    free = simulate(poses, dirs, curv, CONFIG, poses[0], GEOMETRY, CONSTANTS)
    fed = simulate(poses, dirs, curv, CONFIG, poses[0], GEOMETRY, CONSTANTS,
                   control_error=[[0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.02, 0.0], [99.0, 0.0, 0.02, 0.0]])
    assert free["status"] == "arrived" and fed["status"] == "arrived"
    # the reported pose is the estimate's (on the path); the truck itself ended about 2 cm to the right
    assert abs(fed["pose"][1]) < 0.01


def test_a_model_stall_at_another_cusp_is_not_the_same_failure():
    """Codex plan round 10 P2 (ct2_07ca86aa_fixed_random_m_s2 m3 retry 1: Isaac at the first cusp, the fed model at
    the second): counted as a stall, not as one at the same leg end."""
    from d7_reapproach_steer import feed_summary

    def mode(status, leg):
        return {"status": status, "leg": leg}

    rows = [{"isaac": "stall", "isaac_stop": {"leg": 0}, "none": mode("arrived", 1), "position": mode("stall", 0),
             "position_yaw": mode("stall", 0)},
            {"isaac": "stall", "isaac_stop": {"leg": 0}, "none": mode("arrived", 1), "position": mode("stall", 1),
             "position_yaw": mode("stall", 1)},
            {"isaac": "arrived", "isaac_stop": None, "none": mode("arrived", 1), "position": mode("stall", 0),
             "position_yaw": mode("arrived", 1)}]
    out = feed_summary(rows)
    assert out["position"] == {"stalls": 2, "stall_any": 2, "stall_same_leg": 1, "controls": 1, "control_arrived": 0}
    assert out["none"]["stall_any"] == 0 and out["position_yaw"]["control_arrived"] == 1


def _mission(tmp_path, run, mission, retries, rounds, outcome, reason, ended_s=200.0):
    import json

    base = tmp_path / run / "live/epoch_1/run"
    md = base / "missions" / mission
    md.mkdir(parents=True, exist_ok=True)
    (base / "result.json").write_text(json.dumps({"tracker_configs": {"transport": {"cruise_speed_mps": 0.3}}}))
    path = {"poses": [[0, 0, 0], [1, 0, 0]], "directions": [1, 1], "curvatures_inv_m": [0, 0], "length_m": 1.0}
    state = {"docking_retries": retries, "paths_replaced": [], "paths": {"transport": path},
             "delivery_docking": {"rounds": [{"time_s": t} for t in rounds]}, "mission_scenario": {"x": 1}}
    (md / "records.json").write_text(json.dumps({"state": state}))
    (md / "result.json").write_text(json.dumps({"result": {"outcome": outcome, "reason": reason, "ended_s": ended_s}}))
    for name, rows in (("slam.control", [[150.0, 0.9, 0.0, 0.07, 0.9, 0.0, 0.07]]),
                       ("state.samples", [{"time_s": 99.9, "steering_actual_rad": [0.1, 0.1]},
                                          {"time_s": 100.05, "steering_actual_rad": [0.2, 0.2]}])):
        (base / "spool" / name).mkdir(parents=True, exist_ok=True)
        (base / "spool" / name / "000001.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    return str(base)


def test_an_arrival_needs_a_docking_round_inside_the_attempt_and_a_stop_without_one_is_excluded(tmp_path):
    """Codex plan round 6 P2: no next retry is not an arrival; P3: the start steering is the last sample before."""
    from d7_reapproach_steer import extract

    ev = {"trigger": "b", "time_s": 100.0, "counted": True, "start": [0, 0, 0], "line_start": [1, 0, 0],
          "plan": {"length_m": 1.0}}
    arrived = _mission(tmp_path, "r1", "m1", [ev], [120.0], "done", None)
    lost = _mission(tmp_path, "r2", "m1", [ev], [], "failed", "slam_link_failed")
    stalled = _mission(tmp_path, "r3", "m1", [ev], [], "failed", "transport_recovery_after_docking")
    (a,), (b,), (c,) = extract([arrived]), extract([lost]), extract([stalled])
    assert a["isaac"] == "arrived" and a["rounds_in_window"] == 1
    assert "isaac" not in b and b["excluded"].startswith("no_arrival_evidence")
    assert c["isaac"] == "stall" and c["stop_pose"] == [0.9, 0.0, 0.07]
    assert a["steering0"] == [0.1, 0.1] and a["steering0_lag_s"] == pytest.approx(0.1) and not a["steering0_fallback"]


def test_v2_does_not_stand_without_the_four_named_failure_attempts():
    from d7_reapproach_steer import FIRST_CANDIDATE, check

    poses, dirs, curv = s_path()
    path = {"poses": poses.tolist(), "directions": dirs.tolist(), "curvatures_inv_m": curv.tolist()}
    base = {"path": path, "start": poses[0].tolist(), "steering0": [0.0, 0.0], "command0": [0.0, 0.0],
            "config": CONFIG.__dict__, "scenario": {"x": 1}, "trigger": "b"}
    one_stall = dict(base, run="ct2_41a2a7e8_fixed_random_m_s2", mission="m1", retry=0, isaac="stall",
                     stop_pose=poses[-1].tolist())
    control = dict(base, run="other", mission="m1", retry=0, isaac="arrived")
    called = []
    out = check([one_stall, control], GEOMETRY, CONSTANTS, planner=lambda c, k: called.append(k))
    assert not out["complete"] and not out["reproduced"] and out["pick"] == FIRST_CANDIDATE and called == []
