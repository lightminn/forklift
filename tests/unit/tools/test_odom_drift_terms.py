"""Plan 2026-10-21 task A: the odometry drift split (tools/odom_drift_terms.py) on synthetic logs."""

import math

import numpy as np
import pytest

from tools import odom_drift_terms as T

DT = 1 / 120


def result(seed=None):
    return {"arguments": {"drive_geometry": {"wheelbase_m": 0.66, "track_m": 0.53, "wheel_radius_m": 0.125},
                          "odometry_wheel_radius_m": None, "rear_axle_offset_m": -0.34, "slam_noise_seed": seed,
                          "slam_noise_spec": None},
            "slam_feedback": {"start_rear": [-0.34, 0.0, 0.0]},
            "transitions": [{"from": "observe", "to": "approach", "time_s": 0.0},
                            {"from": "approach", "to": "insert", "time_s": 2.0},
                            {"from": "insert", "to": "lift", "time_s": 3.0}]}


def log(n=480, v=0.5, rate_bias=None, slip=None):
    """A straight drive along x at v; the base starts at the origin (rear axle at -0.34)."""
    t = np.arange(n) * DT
    x = v * t
    rate = np.full(n, v / 0.125)
    reported = rate.copy()
    if rate_bias is not None:
        a, b, k = rate_bias
        reported[(t >= a) & (t < b)] *= k
    turned = np.concatenate([[0.0], np.cumsum((rate[1:] + rate[:-1]) / 2 * DT)])
    if slip is not None:
        a, b, k = slip  # the wheels turn k times more than the body moves in [a, b)
        extra = np.concatenate([[0.0], np.cumsum(((t[1:] >= a) & (t[1:] < b)) * (k - 1) * rate[1:] * DT)])
        turned = turned + extra
        reported = np.where((t >= a) & (t < b), reported * k, reported)
    wheels = np.zeros((n, 4), np.float32)
    wheels[:, 2] = wheels[:, 3] = reported
    angles = np.zeros((n, 4))
    angles[:, 2] = angles[:, 3] = turned
    poses = np.zeros((n, 7))
    poses[:, 0] = x
    poses[:, 3] = 1.0
    return {"joint_stamps_s": t, "wheel_rates_rad_s": wheels, "steering_rad": np.zeros((n, 2)),
            "wheel_angles_rad": angles, "base_pose_world": poses}


def window(lg, res):
    return T.terms(T.trajectories(lg, res), *T.phase_window(res))


def test_a_clean_straight_drive_has_no_drift_and_no_terms():
    w = window(log(), result())
    assert w["truth_forward_m"] == pytest.approx(1.5, abs=2e-3)  # approach start to the insert's end, 3 s
    for term in w["terms"].values():
        assert abs(term["forward_m"]) < 1e-6
    assert abs(w["drift"]["forward_m"]) < 1e-6


def test_reported_rates_above_the_joint_angles_land_in_rate_angle():
    w = window(log(rate_bias=(0.5, 1.0, 1.1)), result())  # 0.25 m driven in the window, reported 10 % high
    assert w["terms"]["rate_angle"]["forward_m"] == pytest.approx(0.025, abs=1e-3)
    assert abs(w["terms"]["angle_truth"]["forward_m"]) < 1e-6
    assert w["sum_residual_m"] < 1e-12


def test_wheels_turning_more_than_the_body_moves_land_in_angle_truth():
    w = window(log(slip=(0.5, 1.0, 1.1)), result())
    assert w["terms"]["angle_truth"]["forward_m"] == pytest.approx(0.025, abs=1e-3)
    assert abs(w["terms"]["rate_angle"]["forward_m"]) < 1e-3


def test_the_noise_term_is_the_rebuilt_noise_and_the_terms_still_sum():
    w = window(log(), result(seed=1))
    assert w["terms"]["noise"]["forward_m"] != 0.0
    assert w["sum_residual_m"] < 1e-12
    assert w["drift"]["forward_m"] == pytest.approx(sum(t["forward_m"] for t in w["terms"].values()), abs=1e-12)


def test_the_rebuilt_odometry_is_checked_against_every_recorded_sample():
    # Codex task A review P2: no comparison, an unmatched stamp or a wrong yaw must not read as a reproduction.
    lg, res = log(), result(seed=1)
    traj = T.trajectories(lg, res)
    stamps = [float(t) for t in lg["joint_stamps_s"][::40]]
    good = [{"stamp_s": t, "odom_base": list(T.base_from_rear(traj["recorded"][int(round(t * 120))], -0.34))}
            for t in stamps]
    check = T.reproduction(traj, good)
    assert check["compared"] == len(good) and check["unmatched"] == 0 and check["ok"]
    assert check["max_position_m"] < 1e-12 and check["max_yaw_rad"] < 1e-12
    assert not T.reproduction(traj, [])["ok"]  # nothing compared
    shifted = [dict(r, stamp_s=r["stamp_s"] + 0.003) for r in good]
    assert T.reproduction(traj, shifted)["unmatched"] == len(good) and not T.reproduction(traj, shifted)["ok"]
    yawed = [dict(r, odom_base=[r["odom_base"][0], r["odom_base"][1], r["odom_base"][2] + 0.5]) for r in good]
    out = T.reproduction(traj, yawed)
    assert out["max_yaw_rad"] == pytest.approx(0.5) and not out["ok"]
    moved = [dict(r, odom_base=[r["odom_base"][0] + 0.01, *r["odom_base"][1:]]) for r in good]
    assert T.reproduction(traj, moved)["max_position_m"] == pytest.approx(0.01)


def test_the_window_reports_the_rolling_distances_apart_from_the_trajectories():
    # Codex task A review P2: the wheels' rolled distance against the truth path, not the trajectories' forward
    # difference (the steering moves that one too).
    w = window(log(slip=(0.5, 1.0, 1.1)), result())
    d = w["distance"]
    assert d["truth_path_m"] == pytest.approx(1.5, abs=2e-3)
    assert d["joint_angle_m"] - d["truth_path_m"] == pytest.approx(0.025, abs=1e-3)
    assert d["reported_rate_m"] == pytest.approx(d["joint_angle_m"], abs=1e-3)


# Joint-angle odometry plan 2026-10-22 (3rd edition), step 3: team-spec runs, window classes, the hold window.

from forklift_core.localization.encoders import TeamEncoderModel, encoder_model_spec, sensor_phases  # noqa: E402


def team_spec(lg, seed=3):
    """The log with the team-spec encoder arrays the runner records, and the meta naming the model."""
    model = TeamEncoderModel(encoder_model_spec("team_spec_quantized"), sensor_phases(seed))
    rows = [model.read(lg["wheel_angles_rad"][k, 2:4], lg["steering_rad"][k]) for k in range(len(lg["joint_stamps_s"]))]
    out = dict(lg, wheel_increments_rad=np.array([r[0] for r in rows], float),
               steering_measured_rad=np.array([r[1] for r in rows], float),
               wheel_counts=np.array([r[2] for r in rows], np.int64))
    return out, {"encoder_model": model.record()}


def test_a_team_spec_run_is_rebuilt_from_its_counts_and_splits_into_quantize_and_angle_truth():
    lg, meta = team_spec(log(slip=(0.5, 1.0, 1.1)))
    res = result(seed=3)  # the noise seed does not draw wheel noise on a team-spec run
    traj = T.trajectories(lg, res, meta)
    assert traj["input_term"] == "quantize"
    w = T.terms(traj, *T.phase_window(res))
    assert set(w["terms"]) == {"noise", "quantize", "angle_truth"}
    assert w["terms"]["noise"]["forward_m"] == 0.0
    # The counts are 0.19 mm of rolling each: the quantize term stays within a few counts over the window.
    assert abs(w["terms"]["quantize"]["forward_m"]) < 1e-3
    assert w["terms"]["angle_truth"]["forward_m"] == pytest.approx(0.025, abs=1e-3)
    assert w["sum_residual_m"] < 1e-12
    # The recorded odometry is update_increments over the counted increments and the measured steering.
    from forklift_core.localization.slam_pose import IncrementalWheelOdometry
    from forklift_core.localization.wheel_odometry import AckermannOdometryGeometry
    odo = IncrementalWheelOdometry(AckermannOdometryGeometry(0.66, 0.53, 0.125), initial_pose=(-0.34, 0.0, 0.0))
    last = None
    for k, t in enumerate(lg["joint_stamps_s"]):
        last = odo.update_increments(t, lg["wheel_increments_rad"][k], lg["steering_measured_rad"][k])
    assert np.allclose(traj["recorded"][-1], last, atol=1e-12)


def test_a_team_spec_log_that_disagrees_with_its_model_is_refused():
    lg, meta = team_spec(log())
    lg["wheel_counts"] = lg["wheel_counts"] + 1
    with pytest.raises(ValueError):
        T.trajectories(lg, result(seed=3), meta)


def test_a_reported_run_without_meta_keeps_the_rate_angle_split():
    traj = T.trajectories(log(), result(seed=1), {"encoder_model": {"name": "reported"}})
    assert traj["input_term"] == "rate_angle"
    assert set(T.terms(traj, 0.0, 3.0)["terms"]) == {"noise", "rate_angle", "angle_truth"}


def test_the_window_classes():
    res = result()
    assert T.window_status(res) == ("complete", 0.0, 3.0)
    stuck = dict(res, transitions=res["transitions"][:2])  # insert entered, never left
    assert T.window_status(stuck)[0] == "mid_failure"
    never = dict(res, transitions=res["transitions"][:1])  # approach, no insert
    assert T.window_status(never)[0] == "mid_failure"
    none = dict(res, transitions=[{"from": "observe", "to": "failed", "time_s": 5.0}])
    assert T.window_status(none) == ("no_window", None, None)


def test_the_hold_window_starts_at_the_accepted_near_capture():
    res = result()
    res["near_capture"] = {"status": "done", "near_attempt": 2}
    res["observation_attempts"] = [{"capture_control": {"time_s": 0.2}}, {"capture_control": {"time_s": 1.0}}]
    assert T.window_status(res, hold=True) == ("complete", 1.0, 3.0)
    del res["near_capture"]
    assert T.window_status(res, hold=True) == ("no_window", None, None)


def test_a_mid_failure_or_an_unreproduced_run_is_counted_but_not_summarised(tmp_path):
    rows = [{"encoder_model": "team_spec_quantized", "status": "complete", "hold": {"drift": {"forward_m": 0.004}}},
            {"encoder_model": "team_spec_quantized", "status": "complete", "hold": {"drift": {"forward_m": -0.015}}},
            {"encoder_model": "team_spec_quantized", "status": "mid_failure", "hold": None},
            {"encoder_model": "team_spec_quantized", "status": "reproduction_failure", "hold": None},
            {"encoder_model": "reported", "status": "no_window", "hold": None}]
    s = T.summary(rows)
    tq = s["team_spec_quantized"]
    assert tq["counts"] == {"complete": 2, "mid_failure": 1, "reproduction_failure": 1}
    assert tq["valid"] == 2
    assert tq["hold_forward_abs_median_m"] == pytest.approx(abs((0.004 - 0.015) / 2))
    assert tq["hold_forward_max_abs_m"] == pytest.approx(0.015)  # the sign does not hide the worst one
    assert s["reported"]["valid"] == 0 and s["reported"]["hold_forward_max_abs_m"] is None
