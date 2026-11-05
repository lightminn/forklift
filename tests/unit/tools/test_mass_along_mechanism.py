"""tools/mass_along_mechanism.py on synthetic records (docs/plans/2026-11-06-mass-along-mechanism-diagnosis.md)."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import mass_along_judge as J  # noqa: E402
import mass_along_mechanism as M  # noqa: E402
from forklift_core.localization.slam_pose import compose, invert  # noqa: E402


def test_a_release_effect_is_rebuilt_from_the_corrected_row_without_composing_twice():
    # Codex plan review 2: the control row is the corrected rear estimate E; E- = C- o invert(C+) o E
    c_minus, c_plus = (0.0, 0.0, 0.0), (0.05, 0.0, 0.01)
    odom = (1.0, 0.2, 0.1)
    truth = (1.02, 0.21, 0.105)
    e_plus = compose(c_plus, odom)
    control = np.array([[0.5, *e_plus, *truth]])
    events = [(0.0, "first", None, c_minus, None), (0.5, "release", c_minus, c_plus, 0.05)]
    eff = M.correction_effects(events, [], control, -0.34)
    assert len(eff) == 1 and eff[0]["delay_s"] == 0.0
    assert eff[0]["post"] == pytest.approx(M.err3(truth, e_plus))
    assert eff[0]["pre"] == pytest.approx(M.err3(truth, compose(c_minus, odom)))


def test_a_processed_effect_uses_its_own_record_at_the_rear_axle():
    rec = {"scan_id": 3, "stamp_s": 1.0, "status": "processed", "mode": "tracking", "odom_base": [2.0, 0.0, 0.0],
           "truth_base": [2.03, 0.0, 0.0], "map_from_odom": [0.03, 0, 0], "applied_map_from_odom": [0.03, 0, 0]}
    events = [(0.0, "first", None, (0.0, 0.0, 0.0), None), (1.0, "processed", (0.0, 0.0, 0.0), (0.03, 0.0, 0.0), None)]
    eff = M.correction_effects(events, [rec], np.zeros((0, 7)), -0.34)
    assert eff[0]["pre"][0] == pytest.approx(-0.03) and eff[0]["post"][0] == pytest.approx(0.0, abs=1e-12)


def test_perfect_kinematics_leave_no_steering_residual():
    # D3: per-sample trapezoid on a curve whose steering changes over the window
    L, W = 0.66, 0.53
    t = np.arange(0.0, 2.0, 1 / 120)
    delta = 0.2 + 0.1 * np.sin(t)
    # left/right from a common curvature: solve each wheel's Ackermann angle for the same kappa
    kappa = np.tan(delta) / L
    dl = np.arctan(kappa * L / (1 - kappa * W / 2))
    dr = np.arctan(kappa * L / (1 + kappa * W / 2))
    k = M.kinematic_curvature(dl, dr, L, W)
    assert np.allclose(k, kappa, rtol=1e-12)
    v = 0.4
    ds = v / 120
    yaw = np.concatenate([[0.0], np.cumsum(0.5 * (k[1:] + k[:-1]) * ds)])
    x = np.concatenate([[0.0], np.cumsum(ds * np.cos(0.5 * (yaw[1:] + yaw[:-1])))])
    y = np.concatenate([[0.0], np.cumsum(ds * np.sin(0.5 * (yaw[1:] + yaw[:-1])))])
    control = np.column_stack([t, x, y, yaw, x, y, yaw])
    log = {"joint_stamps_s": t, "steering_rad": np.column_stack([dl, dr]), "steering_measured_rad": np.column_stack([dl, dr])}
    r = M.steering_residuals(log, control, (0.0, 2.0), L, W)
    assert r["actual.windows"] > 0 and abs(r["actual.residual_rad"]) < 1e-9
    assert r["measured.residual_rad"] == pytest.approx(r["actual.residual_rad"])


def test_understeer_shows_as_a_negative_residual():
    L, W = 0.66, 0.53
    t = np.arange(0.0, 1.0, 1 / 120)
    kappa = np.full_like(t, 0.5)
    dl = np.arctan(kappa * L / (1 - kappa * W / 2))
    dr = np.arctan(kappa * L / (1 + kappa * W / 2))
    ds = 0.4 / 120
    yaw = np.concatenate([[0.0], np.cumsum(0.9 * kappa[1:] * ds)])  # turns at 90 % of the kinematic rate
    x = np.concatenate([[0.0], np.cumsum(ds * np.cos(yaw[1:]))])
    y = np.concatenate([[0.0], np.cumsum(ds * np.sin(yaw[1:]))])
    control = np.column_stack([t, x, y, yaw, x, y, yaw])
    log = {"joint_stamps_s": t, "steering_rad": np.column_stack([dl, dr])}
    r = M.steering_residuals(log, control, (0.0, 1.0), L, W)
    assert r["actual.residual_rad"] < -0.01


def test_the_inherited_parts_sum_to_e_inh():
    e_c, t_c, t_p = (1.0, 0.1, 0.05), (1.02, 0.05, 0.02), (1.8, 0.6, 0.5)
    parts = M.inherited_parts(e_c, t_c, t_p)
    assert abs(parts["residual"]) < 1e-12
    assert parts["e_inh"] == pytest.approx(J.along(t_p, compose(e_c, compose(invert(t_c), t_p))))


def test_keyframe_intervals_compare_relative_motions_in_the_start_frame():
    # a pure 2 % odometry scale on a straight leg heading 0.5 rad: dx = -2 % of the leg, dy = dyaw = 0
    off = -0.34
    recs = []
    for i, s in enumerate((0.0, 1.0)):
        truth = (s * math.cos(0.5), s * math.sin(0.5), 0.5)
        odom = (0.98 * s * math.cos(0.5), 0.98 * s * math.sin(0.5), 0.5)
        recs.append({"status": "processed", "stamp_s": float(i), "odom_base": list(odom), "truth_base": list(truth)})
    t = np.linspace(0, 1, 11)
    control = np.column_stack([t, t * math.cos(0.5), t * math.sin(0.5), 0 * t + 0.5, t * math.cos(0.5), t * math.sin(0.5), 0 * t + 0.5])
    iv = M.keyframe_intervals(recs, control, off)
    assert iv[0]["dx"] == pytest.approx(-0.02) and abs(iv[0]["dy"]) < 1e-12 and abs(iv[0]["dyaw"]) < 1e-12
    assert iv[0]["path_m"] == pytest.approx(1.0)


def test_the_encoder_split_separates_quantisation_from_slip():
    t = np.array([0.0, 0.5, 1.0])
    angles = np.zeros((3, 4))
    angles[:, 2] = angles[:, 3] = [0.0, 4.0, 8.0]  # rad of joint rotation
    inc = np.array([[0.0, 0.0], [4.01, 4.01], [4.0, 4.0]])  # 0.01 rad over-read
    log = {"joint_stamps_s": t, "wheel_angles_rad": angles, "wheel_increments_rad": inc}
    iv = [{"t0": 0.0, "t1": 1.0, "along_true_m": 0.95}]
    out = M.encoder_split(log, iv, 0.125)
    assert out[0]["model_err_rad"] == pytest.approx(0.01)
    assert out[0]["joint_vs_truth_m"] == pytest.approx(8.0 * 0.125 - 0.95)


def test_attitude_scales():
    t = np.array([0.0, 1.0])
    a = 0.002
    q = np.array([[1, 0, 0, 0], [math.cos(a / 2), 0, math.sin(a / 2), 0]], float)
    log = {"joint_stamps_s": t, "base_pose_world": np.column_stack([np.zeros((2, 3)), q])}
    out = M.attitude(log, [0.0, 1.0], 1.05)
    assert out["pitch_maxabs_rad"] == pytest.approx(a) and out["origin_shift_max_m"] == pytest.approx(1.05 * a)
