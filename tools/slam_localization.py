"""Localization on a saved slam_toolbox map: start pose, parameters, scoring.

    python tools/slam_localization.py start-pose --map-record <rec> --record <rec> \
        [--offset-s T] [--error-m E] [--error-yaw-rad Y]
    python tools/slam_localization.py params --base <yaml> --map-file <path> \
        --start-pose X Y YAW --output <yaml>
    python tools/slam_localization.py evaluate --map-record <rec> --record <rec> \
        --replay <dir> [--offset-s T]

Plan: docs/plans/2026-10-08-slam-operating-conditions.md. The map frame of a
slam_toolbox map is the base pose at the start of the record it was built
from (the replay's odometry starts there), so a localization estimate is put
in the world with that pose and nothing else -- no alignment to the truth of
the record being localized. A mapping replay scored with --map-record equal to
--record is the same number as tools/evaluate_rig_slam.py's first-pose ATE.

``start-pose`` gives the pose of the first scan a replay started --offset-s
into the record will process, in the map frame, with an optional error placed
45 deg off the truck's heading (the plan's O3). ``evaluate`` writes
localization_evaluation.json and localization_series.npz into the replay.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

CONVERGED_M = 0.10
CONVERGED_RAD = 0.05
STAY_M = 0.15
CONVERGE_WITHIN_S = 10.0
RECOVERED_M = 0.10


def _evaluator():
    path = Path(__file__).with_name("evaluate_rig_slam.py")
    spec = importlib.util.spec_from_file_location("evaluate_rig_slam", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _wrap(angle):
    return (np.asarray(angle) + math.pi) % (2 * math.pi) - math.pi


def map_from_world(map_start: np.ndarray, poses: np.ndarray) -> np.ndarray:
    """World x, y, yaw -> the frame whose origin is map_start (inverse of to_world)."""
    poses = np.atleast_2d(np.asarray(poses, dtype=float))
    c, s = math.cos(map_start[2]), math.sin(map_start[2])
    dx, dy = poses[:, 0] - map_start[0], poses[:, 1] - map_start[1]
    return np.column_stack(
        (c * dx + s * dy, -s * dx + c * dy, _wrap(poses[:, 2] - map_start[2]))
    )


def first_played_scan(record: Path, offset_s: float) -> int:
    """Index of the first scan at or after offset_s past the record's first message."""
    with np.load(record / "slam_log.npz") as data:
        joints = data["joint_stamps_s"].astype(float)
        scans = data["scan_stamps_s"].astype(float)
    start = min(joints[0], scans[0]) + offset_s
    later = np.flatnonzero(scans >= start - 1e-9)
    if not len(later):
        raise ValueError(f"no scan {offset_s} s into {record}")
    return int(later[0])


def start_pose(
    map_record: Path,
    record: Path,
    offset_s: float = 0.0,
    error_m: float = 0.0,
    error_yaw_rad: float = 0.0,
) -> np.ndarray:
    """Map-frame pose of the first played scan, plus the plan's start error."""
    evaluator = _evaluator()
    _, map_truth = evaluator.load_truth(map_record)
    _, truth = evaluator.load_truth(record)
    pose = truth[first_played_scan(record, offset_s)].copy()
    heading = pose[2] + math.pi / 4
    pose[0] += error_m * math.cos(heading)
    pose[1] += error_m * math.sin(heading)
    pose[2] += error_yaw_rad
    return map_from_world(map_truth[0], pose)[0]


def write_params(base: Path, map_file: str, pose, output: Path) -> None:
    """The mapping parameters with only mode, map file and start pose changed."""
    import yaml

    document = yaml.safe_load(base.read_text())
    params = document["slam_toolbox"]["ros__parameters"]
    params["mode"] = "localization"
    params["map_file_name"] = str(map_file)
    params["map_start_pose"] = [float(v) for v in pose]
    output.write_text(yaml.safe_dump(document, sort_keys=False))


def _convergence(times, error, yaw_error):
    """First time the error is inside the converged band and never leaves STAY_M."""
    inside = (error < CONVERGED_M) & (np.abs(yaw_error) < CONVERGED_RAD)
    stays = np.flip(np.maximum.accumulate(np.flip(error))) < STAY_M
    hits = np.flatnonzero(inside & stays)
    return None if not len(hits) else float(times[hits[0]])


def evaluate(
    map_record: Path, record: Path, replay: Path, offset_s: float = 0.0
) -> dict:
    evaluator = _evaluator()
    _, map_truth = evaluator.load_truth(map_record)
    scan_stamps, truth = evaluator.load_truth(record)
    estimate, _, info = evaluator.estimates(replay, scan_stamps)
    manifest = json.loads((replay / "replay" / "replay_manifest.json").read_text())
    slam = evaluator._csv(replay / "slam/slam_trajectory.csv")
    resolved = (
        evaluator.rows_at(scan_stamps, slam[:, 0] - manifest["clock_offset_s"]) >= 0
    )
    world = evaluator.to_world(estimate, map_truth[0])
    first = first_played_scan(record, offset_s)
    played = np.arange(len(scan_stamps)) >= first
    has = played & np.isfinite(world).all(axis=1)
    error = np.hypot(*(world[:, :2] - truth[:, :2]).T)
    yaw_error = _wrap(world[:, 2] - truth[:, 2])
    times = scan_stamps - scan_stamps[first]
    out = {
        "map_record": str(map_record),
        "record": str(record),
        "replay": str(replay),
        "offset_s": offset_s,
        "estimate": "slam_toolbox map->base_link at each scan, held on odometry "
        "between scans, placed in the world with the map record's start pose only",
        "scans_played": int(played.sum()),
        "scans_resolved": int((played & resolved).sum()),
        "coverage": float((played & resolved).sum() / played.sum()),
        "scans_scored": int(has.sum()),
        **{k: v for k, v in info.items() if k in ("condition", "noise")},
    }
    if has.any():
        e, y, t = error[has], yaw_error[has], times[has]
        converged_at = _convergence(t, e, y)
        out.update(
            rmse_m=float(np.sqrt(np.mean(e**2))),
            max_m=float(e.max()),
            yaw_rmse_rad=float(np.sqrt(np.mean(y**2))),
            first_error_m=float(e[0]),
            final_error_m=float(e[-1]),
            converged_at_s=converged_at,
            converged=converged_at is not None and converged_at <= CONVERGE_WITHIN_S,
        )
        windows = manifest.get("condition", {}).get("lidar_blackout_s") or []
        out["lidar_blackouts"] = []
        for begin, end in windows:
            during = has & (scan_stamps >= begin) & (scan_stamps < end)
            after = has & (scan_stamps >= end)
            if not during.any():
                continue
            back = np.flatnonzero(after & (error < RECOVERED_M))
            out["lidar_blackouts"].append(
                {
                    "window_s": [begin, end],
                    "max_during_m": float(error[during].max()),
                    "error_at_return_m": float(error[np.flatnonzero(after)[0]])
                    if after.any()
                    else None,
                    "recovered_after_s": float(scan_stamps[back[0]] - end)
                    if len(back)
                    else None,
                }
            )
    np.savez_compressed(
        replay / "localization_series.npz",
        stamp_s=scan_stamps,
        truth=truth,
        estimate=world,
        error_m=error,
        yaw_error_rad=yaw_error,
        played=played,
        resolved=resolved,
    )
    (replay / "localization_evaluation.json").write_text(
        json.dumps(out, indent=2) + "\n", encoding="utf-8"
    )
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    pose = sub.add_parser("start-pose")
    pose.add_argument("--map-record", type=Path, required=True)
    pose.add_argument("--record", type=Path, required=True)
    pose.add_argument("--offset-s", type=float, default=0.0)
    pose.add_argument("--error-m", type=float, default=0.0)
    pose.add_argument("--error-yaw-rad", type=float, default=0.0)
    params = sub.add_parser("params")
    params.add_argument("--base", type=Path, required=True)
    params.add_argument("--map-file", required=True)
    params.add_argument("--start-pose", type=float, nargs=3, required=True)
    params.add_argument("--output", type=Path, required=True)
    score = sub.add_parser("evaluate")
    score.add_argument("--map-record", type=Path, required=True)
    score.add_argument("--record", type=Path, required=True)
    score.add_argument("--replay", type=Path, required=True)
    score.add_argument("--offset-s", type=float, default=0.0)
    args = parser.parse_args(argv)
    if args.command == "start-pose":
        print(
            " ".join(
                f"{v:.6f}"
                for v in start_pose(
                    args.map_record,
                    args.record,
                    args.offset_s,
                    args.error_m,
                    args.error_yaw_rad,
                )
            )
        )
    elif args.command == "params":
        write_params(args.base, args.map_file, args.start_pose, args.output)
    else:
        report = evaluate(args.map_record, args.record, args.replay, args.offset_s)
        print(
            f"coverage {report['coverage']:.3f}  rmse {report.get('rmse_m', math.nan):.3f} m"
            f"  max {report.get('max_m', math.nan):.3f} m"
            f"  converged {report.get('converged')} at {report.get('converged_at_s')}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
