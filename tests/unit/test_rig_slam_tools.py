"""Offline evaluation and summary tools of the rig SLAM comparison (plan 2026-10-07 D4)."""

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]


def _load(name):
    spec = importlib.util.spec_from_file_location(
        f"{name}_under_test", ROOT / f"tools/{name}.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


evaluate = _load("evaluate_rig_slam")
summarise = _load("summarise_rig_slam")


def test_estimates_are_placed_with_the_true_start_pose():
    start = np.array([2.0, -1.0, math.pi / 2])
    poses = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.1]])
    world = evaluate.to_world(poses, start)
    np.testing.assert_allclose(world[0], start)
    np.testing.assert_allclose(world[1], [2.0, 0.0, math.pi / 2 + 0.1], atol=1e-12)


def test_rows_are_paired_by_stamp_within_a_microsecond():
    table = np.array([0.1, 0.2, 0.3000004])
    rows = evaluate.rows_at(np.array([0.2, 0.3, 0.25]), table)
    assert rows.tolist() == [1, 2, -1]


def _write(root, record, name, ate):
    d = root / record / name
    d.mkdir(parents=True)
    report = {
        "estimate": {
            "first_pose_ate_rmse_m": ate,
            "ate_rmse_m": ate,
            "final_error_m": 0.0,
            "yaw_rmse_rad": 0.0,
        },
        "start_aligned_error_max_m": ate,
        "odometry_same_frames": {"first_pose_ate_rmse_m": 0.6},
        "frames_paired": 10,
        "frames": 10,
    }
    (d / "evaluation.json").write_text(json.dumps(report))


def test_summary_parses_config_names_and_pairs_ratios(tmp_path):
    _write(tmp_path, "nominal_seed1", "lidar_st_1_nominal", 0.05)
    _write(tmp_path, "nominal_seed1", "fusion_1_nominal", 0.02)
    _write(tmp_path, "nominal_seed1", "vision_front_1_lidar_blackout", 0.5)
    (tmp_path / "nominal_seed1" / "vision_1_nominal").mkdir()  # unfinished
    rows = summarise.collect(tmp_path)
    by = {(r["config"], r["condition"]): r for r in rows}
    assert by[("vision_front", "lidar_blackout")]["noise"] == "noisy"
    assert by[("vision", "nominal")]["status"] == "missing"
    ratios = summarise.paired_ratios(rows)
    assert [r["pair"] for r in ratios] == ["F/L-ST"]
    assert ratios[0]["ratio"] == pytest.approx(0.4)
    assert "F/L-ST" in summarise.markdown(rows, ratios)


def test_frames_without_a_slam_pose_ride_the_last_correction_on_odometry():
    odometry = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
    )
    estimate = np.full((4, 3), np.nan)
    estimate[1] = [1.0, 0.5, 0.0]  # SLAM says the odometry is 0.5 m off in y
    filled, count = evaluate.hold_correction(estimate, odometry)
    assert count == 2 and np.isnan(filled[0]).all()
    np.testing.assert_allclose(filled[3], [3.0, 0.5, 0.0])
