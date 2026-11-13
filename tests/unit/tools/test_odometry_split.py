"""Baseline fragility odometry split (docs/validation/2026-11-13-baseline-fragility/odometry_split.py): the
quantisation's share and the kinematic model's error against the plant over the capture -> lift leg, and the
lag/gain check. Synthetic logs, no Isaac."""

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from forklift_core.localization.slam_pose import IncrementalWheelOdometry
from forklift_core.localization.wheel_odometry import AckermannOdometryGeometry

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    "odometry_split", ROOT / "docs/validation/2026-11-13-baseline-fragility/odometry_split.py")
O = importlib.util.module_from_spec(spec)
spec.loader.exec_module(O)

GEO = {"wheelbase_m": 0.66, "track_m": 0.53, "wheel_radius_m": 0.125}


def make_run(root: Path, name: str, *, plant_gain=1.0, plant_lag=0, steer_quant=0.0, n=600, steer_amp=0.05,
             accelerate=False):
    """A truck rolling forward at 0.3 m/s with a sinusoidal steering; the plant turns plant_gain times the kinematic
    yaw rate, plant_lag ticks late; the measured steering is the true one plus steer_quant."""
    dt = 1 / 120
    t = (np.arange(n) + 1) * dt
    steer = np.column_stack([steer_amp * np.sin(t)] * 2)
    ds = (0.05 + 0.5 * np.arange(n) / n if accelerate else np.full(n, 0.3)) * dt  # metres per tick
    geometry = AckermannOdometryGeometry(**GEO)
    angles = np.zeros((n, 4))
    angles[:, 2:4] = (np.concatenate([[0.0], np.cumsum(ds[1:])]) / GEO["wheel_radius_m"])[:, None]
    # plant: integrate the kinematic yaw rate, scaled and delayed
    k = O._curvature_inv_m(steer, geometry)
    kmid = np.concatenate([[k[0]], (k[1:] + k[:-1]) / 2])
    klag = np.concatenate([np.full(plant_lag, kmid[0]), kmid[: n - plant_lag]]) if plant_lag else kmid
    dyaw = plant_gain * ds * klag
    dyaw[0] = 0.0
    yaw = np.cumsum(dyaw)
    step = np.concatenate([[0.0], ds[1:]])
    x = np.cumsum(step * np.cos(yaw))
    y = np.cumsum(step * np.sin(yaw))
    quat = np.column_stack([x, y, np.zeros(n), np.cos(yaw / 2), np.zeros(n), np.zeros(n), np.sin(yaw / 2)])
    increments = np.vstack([np.zeros((1, 2)), np.diff(angles[:, 2:4], axis=0)])
    measured = steer + steer_quant
    # the estimate: the recorded-input odometry from the capture sample, placed on the capture's truth (SLAM held)
    i_cap, i_lift = 100, n - 1
    odo = IncrementalWheelOdometry(geometry, initial_pose=(x[i_cap], y[i_cap], yaw[i_cap]))
    est = np.zeros((n, 3))
    est[: i_cap + 1] = np.column_stack([x, y, yaw])[: i_cap + 1]
    odo.update_increments(t[i_cap], increments[i_cap], measured[i_cap])
    for i in range(i_cap + 1, n):
        est[i] = odo.update_increments(t[i], increments[i], measured[i])
    rows = [i for i in range(n) if i != i_cap]  # no control row at the capture instant
    sc = np.column_stack([t[rows], est[rows], x[rows], y[rows], yaw[rows]])
    run = root / name / "run"
    run.mkdir(parents=True)
    np.save(run / "slam_control.npy", sc)
    np.savez(run / "slam_log.npz", joint_stamps_s=t, wheel_angles_rad=angles, wheel_increments_rad=increments,
             steering_rad=steer, steering_measured_rad=measured, base_pose_world=quat.astype(np.float64))
    result = {"chassis_model": {"drive_geometry": GEO}, "arguments": {"odometry_wheel_radius_m": None},
              "near_capture": {"near_attempt": 1},
              "observation_attempts": [{"capture_control": {
                  "time_s": float(t[i_cap]), "truth_rear": [x[i_cap], y[i_cap], yaw[i_cap]],
                  "control_rear": [x[i_cap], y[i_cap], yaw[i_cap]]}}]}
    (run / "result.json").write_text(json.dumps(result))
    return {"root": root.name, "name": name, "scene": name.split("_")[0], "arm": "B0", "t_lift": float(t[i_lift]),
            "bias_course": {"dpsi_rad": float(est[i_lift, 2] - yaw[i_lift])}}


def split_of(r, raw_root):
    run = raw_root / r["root"] / r["name"] / "run"
    result = json.loads((run / "result.json").read_text())
    with np.load(run / "slam_log.npz") as log:
        return O.run_split(result, np.load(run / "slam_control.npy"), log, r["t_lift"])


def test_a_kinematic_plant_with_exact_inputs_leaves_nothing(tmp_path):
    s = split_of(make_run(tmp_path, "a_r0"), tmp_path.parent)
    assert s["recorded_vs_estimate_m"] < 1e-12 and abs(s["recorded_vs_estimate_rad"]) < 1e-12
    assert abs(s["quantisation_yaw_rad"]) < 1e-12 and abs(s["kinematic_minus_truth_yaw_rad"]) < 1e-9


def test_a_steering_offset_shows_as_steering_quantisation(tmp_path):
    s = split_of(make_run(tmp_path, "a_r0", steer_quant=0.001), tmp_path.parent)
    assert s["steering_quantisation_yaw_rad"] == pytest.approx(s["quantisation_yaw_rad"], abs=1e-12)
    assert abs(s["wheel_quantisation_yaw_rad"]) < 1e-12
    # about 0.001 rad / 0.66 m of curvature over 4.17 m
    assert s["quantisation_yaw_rad"] == pytest.approx(0.001 / 0.66 * 0.3 * 499 / 120, rel=0.03)
    assert s["estimate_minus_truth_yaw_rad"] == pytest.approx(s["quantisation_yaw_rad"], abs=1e-9)


def test_a_plant_that_turns_less_shows_as_kinematic_error(tmp_path):
    s = split_of(make_run(tmp_path, "a_r0", plant_gain=0.9), tmp_path.parent)
    assert abs(s["quantisation_yaw_rad"]) < 1e-12
    assert s["kinematic_minus_truth_yaw_rad"] == pytest.approx(0.1 * s["kinematic_turn_rad"], rel=1e-3)


def test_the_lag_and_gain_are_recovered(tmp_path):
    runs = [make_run(tmp_path, f"{sc}_r{i}", plant_gain=0.93, plant_lag=10, steer_amp=0.03 + 0.01 * i)
            for i, sc in enumerate(("a", "b", "a", "b"))]
    out = O.analyse(runs, tmp_path.parent)
    assert out["lag_gain_fits"]["all"]["lag_ticks"] == 10 and out["lag_gain_fits"]["all"]["gain"] == pytest.approx(0.93)
    assert out["lag_gain_fits"]["all"]["residual_median_rad"] < 1e-6
    assert out["lag_gain_fits"]["held_out:a"]["residual_median_rad"] < 1e-6
    assert out["quantisation_abs_max_rad"] < 1e-12


def test_the_tick_gain_of_a_plant_that_turns_less(tmp_path):
    runs = [make_run(tmp_path, f"a_r{i}", plant_gain=0.92, steer_amp=0.03 + 0.01 * i) for i in range(2)]
    assert O.analyse(runs, tmp_path.parent)["tick_gain"]["gain"] == pytest.approx(0.92, abs=1e-9)


def test_the_lag_is_on_the_curvature_not_on_the_whole_rotation(tmp_path):
    """Codex review P2: with an accelerating truck, a plant whose curvature lags (distance not) is recovered by the
    curvature-lag model, which a lag of the whole per-tick rotation would not reproduce exactly."""
    runs = [make_run(tmp_path, f"{sc}_r{i}", plant_gain=0.93, plant_lag=10, steer_amp=0.03 + 0.01 * i, accelerate=True)
            for i, sc in enumerate(("a", "b", "a", "b"))]
    out = O.analyse(runs, tmp_path.parent)
    assert out["lag_gain_fits"]["all"]["lag_ticks"] == 10 and out["lag_gain_fits"]["all"]["gain"] == pytest.approx(0.93)
    assert out["lag_gain_fits"]["all"]["residual_median_rad"] < 1e-6
    series = []
    for r in runs:
        run = tmp_path.parent / r["root"] / r["name"] / "run"
        with np.load(run / "slam_log.npz") as log:
            series.append(O.kinematic_series(json.loads((run / "result.json").read_text()),
                                             np.load(run / "slam_control.npy"), log, r["t_lift"]))
    whole = []  # the whole rotation (distance x curvature) delayed instead
    for s in series:
        rot = s["ds"] * s["k"]
        lagged = np.concatenate([np.full(10, rot[0]), rot[:-10]])
        whole.append(abs((0.93 * lagged - s["true_dyaw"])[s["history"]:].sum()))
    assert np.median(whole) > 1e-5
