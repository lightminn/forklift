"""tools/slam_localization.py: map frame, start pose, parameters and scoring."""

import csv
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]


def _load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "tools" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


tool = _load("slam_localization")
CLOCK = 10.0


def _quaternion(yaw):
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def _record(path, start=(-5.34, 0.0, 0.3), n=200, speed=0.5):
    """A straight drive along the start heading, one joint sample per 0.1 s scan."""
    path.mkdir(parents=True)
    stamps = np.arange(n) / 10
    x = start[0] + speed * stamps * math.cos(start[2])
    y = start[1] + speed * stamps * math.sin(start[2])
    base = np.array(
        [[xi, yi, 0.0, *_quaternion(start[2])] for xi, yi in zip(x, y, strict=True)]
    )
    np.savez(
        path / "slam_log.npz",
        joint_stamps_s=stamps,
        scan_stamps_s=stamps,
        base_pose_world=base,
    )
    return np.column_stack((x, y, np.full(n, start[2])))


def _replay(path, stamps, poses, windows=()):
    (path / "replay").mkdir(parents=True)
    (path / "slam").mkdir()
    manifest = {
        "clock_offset_s": CLOCK,
        "messages": {"/scan": len(stamps)},
        "noise": {},
        "condition": {"name": "test", "lidar_blackout_s": [list(w) for w in windows]},
    }
    (path / "replay" / "replay_manifest.json").write_text(json.dumps(manifest))
    with (path / "replay" / "odometry.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["stamp_s", "x_m", "y_m", "yaw_rad"])
        writer.writerows([s, *p] for s, p in zip(stamps, poses, strict=True))
    with (path / "slam" / "slam_trajectory.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["ros_stamp_s", "x_m", "y_m", "yaw_rad"])
        writer.writerows(
            [s + CLOCK, *p]
            for s, p in zip(stamps, poses, strict=True)
            if np.isfinite(p).all()
        )


def test_map_frame_is_the_inverse_of_placing_with_the_start_pose():
    evaluator = tool._evaluator()
    start = np.array([-5.34, 0.2, 0.7])
    poses = np.array([[1.0, 2.0, 0.1], [-3.0, 0.5, -2.9]])
    back = evaluator.to_world(tool.map_from_world(start, poses), start)
    assert np.allclose(back[:, :2], poses[:, :2])
    assert np.allclose(tool._wrap(back[:, 2] - poses[:, 2]), 0.0)


def test_start_pose_is_the_first_played_scan_with_the_error_45_degrees_off_heading(
    tmp_path,
):
    truth = _record(tmp_path / "rec")
    pose = tool.start_pose(tmp_path / "rec", tmp_path / "rec", offset_s=6.0)
    expected = tool.map_from_world(truth[0], truth[60])[0]
    assert np.allclose(pose, expected)
    off = tool.start_pose(tmp_path / "rec", tmp_path / "rec", 6.0, 0.5, 0.1)
    # In the map frame the truck heads along +x, so the error sits at +45 deg.
    assert off[0] - pose[0] == pytest.approx(0.5 / math.sqrt(2))
    assert off[1] - pose[1] == pytest.approx(0.5 / math.sqrt(2))
    assert off[2] - pose[2] == pytest.approx(0.1)


def test_params_change_only_mode_map_and_start_pose(tmp_path):
    yaml = pytest.importorskip("yaml")
    base = tmp_path / "base.yaml"
    base.write_text(
        "slam_toolbox:\n  ros__parameters:\n    mode: mapping\n    resolution: 0.05\n"
    )
    out = tmp_path / "loc.yaml"
    tool.write_params(base, "/map/posegraph", [1.0, 2.0, 0.5], out)
    params = yaml.safe_load(out.read_text())["slam_toolbox"]["ros__parameters"]
    assert params == {
        "mode": "localization",
        "resolution": 0.05,
        "map_file_name": "/map/posegraph",
        "map_start_pose": [1.0, 2.0, 0.5],
    }


def test_evaluation_is_unaligned_and_reports_convergence(tmp_path):
    truth = _record(tmp_path / "rec")
    map_truth = _record(tmp_path / "map")  # same start pose
    estimate = tool.map_from_world(map_truth[0], truth)
    # Start 0.3 m off along map y, closing to zero by scan 30 (3 s).
    estimate[:, 1] += 0.3 * np.clip(1 - np.arange(len(truth)) / 30, 0, None)
    _replay(tmp_path / "replay", np.arange(len(truth)) / 10, estimate)
    report = tool.evaluate(tmp_path / "map", tmp_path / "rec", tmp_path / "replay")
    assert report["coverage"] == 1.0
    assert report["first_error_m"] == pytest.approx(0.3)
    assert report["max_m"] == pytest.approx(0.3)
    # First scan under 0.10 m: error 0.3 * (1 - i/30) < 0.1 -> i = 21.
    assert report["converged_at_s"] == pytest.approx(2.1)
    assert report["converged"] is True


def test_a_map_from_another_start_shows_up_as_error(tmp_path):
    truth = _record(tmp_path / "rec")
    _record(tmp_path / "map", start=(-5.34, 0.5, 0.3))
    estimate = tool.map_from_world(truth[0], truth)  # placed as if the map began at rec
    _replay(tmp_path / "replay", np.arange(len(truth)) / 10, estimate)
    report = tool.evaluate(tmp_path / "map", tmp_path / "rec", tmp_path / "replay")
    assert report["rmse_m"] == pytest.approx(0.5)
    assert report["converged"] is False


def test_blackout_windows_report_the_recovery_after_the_lidar_returns(tmp_path):
    truth = _record(tmp_path / "rec")
    estimate = tool.map_from_world(truth[0], truth)
    stamps = np.arange(len(truth)) / 10
    drift = np.where((stamps >= 5.0) & (stamps < 7.5), (stamps - 5.0) * 0.1, 0.0)
    drift[(stamps >= 7.5) & (stamps < 8.0)] = 0.2  # still off just after the window
    estimate[:, 0] += drift
    _replay(tmp_path / "replay", stamps, estimate, windows=[(5.0, 7.5)])
    report = tool.evaluate(tmp_path / "rec", tmp_path / "rec", tmp_path / "replay")
    (window,) = report["lidar_blackouts"]
    assert window["max_during_m"] == pytest.approx(0.24)
    assert window["error_at_return_m"] == pytest.approx(0.2)
    assert window["recovered_after_s"] == pytest.approx(0.5)


def test_scans_before_the_start_offset_are_not_scored(tmp_path):
    truth = _record(tmp_path / "rec")
    estimate = tool.map_from_world(truth[0], truth)
    estimate[:50] = np.nan  # nothing published before the restart
    _replay(tmp_path / "replay", np.arange(len(truth)) / 10, estimate)
    report = tool.evaluate(
        tmp_path / "rec", tmp_path / "rec", tmp_path / "replay", offset_s=5.0
    )
    assert report["scans_played"] == 150
    assert report["coverage"] == 1.0
    assert report["rmse_m"] == pytest.approx(0.0, abs=1e-9)
