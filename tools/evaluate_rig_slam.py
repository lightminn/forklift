"""Score one offline replay of a rig record against ground truth.

    python tools/evaluate_rig_slam.py --record <record> --replay <replay dir>

Plan: docs/plans/2026-10-07-visual-slam-and-fusion.md D4. Works for both
replay kinds the plan runs on the same record:

- RTAB-Map through forklift_ros.rig_slam_bridge (``bridge_records.json``): the
  bridge's estimate at every frame -- the pose a robot would have driven on.
- slam_toolbox through the existing bag replay (``slam/slam_trajectory.csv``):
  the online map->base_link at every scan stamp (tools/evaluate_slam_replay.py
  pairs the same way).

Both are placed in the world with the true start pose -- the one alignment a
robot knows -- and scored with forklift_core trajectory_error, next to the
wheel odometry that went into that replay. Writes evaluation.json and
error_series.npz (per frame: stamp, truth, estimate, odometry, start-aligned
error) into the replay directory.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict
from pathlib import Path

import numpy as np

from forklift_core.localization.trajectory_error import trajectory_error

STAMP_TOLERANCE_S = 1e-6
CLOCK_OFFSET_S = 10.0  # forklift_ros slam_replay / rig_slam_bridge


def _yaw(quaternion_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion_wxyz.T
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def to_world(poses: np.ndarray, start: np.ndarray) -> np.ndarray:
    """Poses in a frame that coincides with base_link at the start -> world."""
    c, s = math.cos(start[2]), math.sin(start[2])
    return np.column_stack(
        (
            start[0] + c * poses[:, 0] - s * poses[:, 1],
            start[1] + s * poses[:, 0] + c * poses[:, 1],
            poses[:, 2] + start[2],
        )
    )


def rows_at(stamps: np.ndarray, table_stamps: np.ndarray) -> np.ndarray:
    """Index of each stamp in table_stamps (nearest within tolerance), else -1."""
    order = np.argsort(table_stamps)
    ordered = table_stamps[order]
    right = np.clip(np.searchsorted(ordered, stamps), 0, len(ordered) - 1)
    left = np.clip(right - 1, 0, len(ordered) - 1)
    nearest = np.where(
        np.abs(ordered[left] - stamps) < np.abs(ordered[right] - stamps), left, right
    )
    found = np.abs(ordered[nearest] - stamps) <= STAMP_TOLERANCE_S
    return np.where(found, order[nearest], -1)


def _csv(path: Path) -> np.ndarray:
    with path.open(newline="") as stream:
        rows = list(csv.reader(stream))[1:]
    return np.asarray(rows, dtype=float).reshape(-1, 4)


def load_truth(record: Path) -> tuple[np.ndarray, np.ndarray]:
    """Scan stamps and the true base_link x, y, yaw at each."""
    with np.load(record / "slam_log.npz") as data:
        joint_stamps = data["joint_stamps_s"]
        base = data["base_pose_world"].astype(float)
        scan_stamps = data["scan_stamps_s"].astype(float)
    truth_all = np.column_stack((base[:, 0], base[:, 1], _yaw(base[:, 3:7])))
    return scan_stamps, truth_all[rows_at(scan_stamps, joint_stamps)]


def _compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array(
        [a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]]
    )


def _invert(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([-(c * a[0] + s * a[1]), s * a[0] - c * a[1], -a[2]])


def hold_correction(
    estimate: np.ndarray, odometry: np.ndarray
) -> tuple[np.ndarray, int]:
    """Fill frames without a SLAM pose after the first one: last map<-odom on odometry.

    slam_toolbox gives a pose only at a scan it processed; between them (and
    through a LiDAR blackout) the robot's map->base_link is the last map->odom
    correction composed with the current wheel odometry. Frames before the
    first pose stay missing.
    """
    out = np.array(estimate, dtype=float, copy=True)
    last, filled = None, 0
    for i in range(len(out)):
        if np.isfinite(out[i]).all():
            last = i
        elif last is not None:
            map_from_odom = _compose(out[last], _invert(odometry[last]))
            out[i] = _compose(map_from_odom, odometry[i])
            filled += 1
    return out, filled


def estimates(
    replay: Path, scan_stamps: np.ndarray
) -> tuple[np.ndarray, np.ndarray, dict]:
    """(estimate poses with NaN rows where missing, odometry poses, extra info)."""
    estimate = np.full((len(scan_stamps), 3), np.nan)
    if (replay / "bridge_records.json").exists():
        records = json.loads((replay / "bridge_records.json").read_text())
        status = json.loads((replay / "bridge_status.json").read_text())
        frames = [r for r in records if "frame_id" in r and "estimate" in r]
        stamps = np.array([r["stamp_s"] for r in frames])
        rows = rows_at(scan_stamps, stamps)
        found = rows >= 0
        estimate[found] = np.array([frames[i]["estimate"] for i in rows[found]])
        odometry = np.load(replay / "record_truth.npz")["odometry"]
        lost = {
            stage: int(sum(1 for r in frames if r["front"].get(stage) == "lost"))
            for stage in status["front_status"]
        }
        info = {
            "kind": "rtabmap",
            "mode": status["mode"],
            "cameras": status["cameras"],
            "condition": status["condition"],
            "noise_seed": status["noise_seed"],
            "bridge_ok": status["ok"],
            "keyframes": status["keyframes"],
            "front_status": status["front_status"],
            "front_lost_frames": lost,
            "frame_wall_s_median": status["wall_s_median"],
            "frame_wall_s_p95": float(np.percentile([r["wall_s"] for r in frames], 95))
            if frames
            else None,
        }
        return estimate, odometry, info
    manifest = json.loads((replay / "replay" / "replay_manifest.json").read_text())
    slam = _csv(replay / "slam/slam_trajectory.csv")
    rows = rows_at(scan_stamps, slam[:, 0] - manifest["clock_offset_s"])
    found = rows >= 0
    estimate[found] = slam[rows[found], 1:]
    odometry_csv = _csv(replay / "replay/odometry.csv")
    odometry = odometry_csv[rows_at(scan_stamps, odometry_csv[:, 0]), 1:]
    estimate, extrapolated = hold_correction(estimate, odometry)
    info = {
        "kind": "slam_toolbox",
        "mode": "lidar_st",
        "condition": manifest.get("condition", {}).get("name", "nominal"),
        "noise": manifest["noise"],
        "scan_messages": manifest["messages"]["/scan"],
        # Frames with no scan (a LiDAR blackout) carry the last map<-odom on
        # the wheel odometry, as the robot's TF chain would.
        "frames_extrapolated_on_odometry": extrapolated,
    }
    return estimate, odometry, info


def evaluate(record: Path, replay: Path) -> dict:
    scan_stamps, truth = load_truth(record)
    estimate, odometry, info = estimates(replay, scan_stamps)
    start = truth[0]
    paired = np.isfinite(estimate).all(axis=1)
    estimate_world = np.full_like(estimate, np.nan)
    estimate_world[paired] = to_world(estimate[paired], start)
    odometry_world = to_world(odometry, start)
    error = np.hypot(*(estimate_world[:, :2] - truth[:, :2]).T)
    odometry_err = np.hypot(*(odometry_world[:, :2] - truth[:, :2]).T)
    np.savez(
        replay / "error_series.npz",
        stamps_s=scan_stamps,
        truth=truth,
        estimate=estimate_world,
        odometry=odometry_world,
        error_m=error,
        odometry_error_m=odometry_err,
    )
    slam = trajectory_error(estimate_world[paired], truth[paired])
    return {
        "record": str(record),
        "replay": str(replay),
        **info,
        "frames": int(len(scan_stamps)),
        "frames_paired": int(paired.sum()),
        "frames_missing": int((~paired).sum()),
        "estimate": asdict(slam),
        "start_aligned_error_max_m": float(np.nanmax(error)),
        "odometry_same_frames": asdict(
            trajectory_error(odometry_world[paired], truth[paired])
        ),
        "note": (
            "first_pose_* places the estimate with the true start pose only, as the "
            "robot would; ate_* is the least-squares rigid fit. Estimates are online "
            "(what the robot had at that frame), never re-optimised afterwards."
        ),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument("--replay", type=Path, required=True)
    args = parser.parse_args(argv)
    report = evaluate(args.record, args.replay)
    (args.replay / "evaluation.json").write_text(json.dumps(report, indent=2) + "\n")
    e, o = report["estimate"], report["odometry_same_frames"]
    print(
        f"{report['mode']:14s} first-pose ATE {e['first_pose_ate_rmse_m']:.3f} m  "
        f"LS ATE {e['ate_rmse_m']:.3f} m  max {e['ate_max_m']:.3f} m  "
        f"final {e['final_error_m']:.3f} m | odometry {o['first_pose_ate_rmse_m']:.3f} m "
        f"| paired {report['frames_paired']}/{report['frames']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
