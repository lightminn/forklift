"""Score a slam_toolbox replay of an Isaac SLAM record against ground truth.

    python tools/evaluate_slam_replay.py --record <run> --replay <replay dir>

Pairs every scan stamp with the ground-truth base_link pose recorded at that
physics step, the online SLAM estimate (slam/slam_trajectory.csv) and the wheel
odometry that went into the bag (odometry.csv). Reports forklift_core's
trajectory_error for both estimates, so the SLAM figure always sits next to
what odometry alone would have given. Writes evaluation.json and map_overlay.png
into the replay directory: the SLAM map with ground-truth item outlines (bright
when taller than the scan plane), and the true, SLAM and odometry paths, all
placed with the start pose -- the one alignment a robot actually knows.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from forklift_core.localization.trajectory_error import trajectory_error

STAMP_TOLERANCE_S = 1e-6


def _csv(path: Path) -> np.ndarray:
    with path.open(newline="") as stream:
        rows = list(csv.reader(stream))[1:]
    return np.asarray(rows, dtype=float).reshape(-1, 4)


def _yaw(quaternion_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion_wxyz.T
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _rows_at(stamps: np.ndarray, table_stamps: np.ndarray) -> np.ndarray:
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


def _to_world(poses: np.ndarray, start: np.ndarray) -> np.ndarray:
    c, s = math.cos(start[2]), math.sin(start[2])
    return np.column_stack(
        (
            start[0] + c * poses[:, 0] - s * poses[:, 1],
            start[1] + s * poses[:, 0] + c * poses[:, 1],
            poses[:, 2] + start[2],
        )
    )


def _overlay(replay: Path, meta: dict, start, paths: dict[str, np.ndarray]) -> None:
    info = {}
    for line in (replay / "slam/map.yaml").read_text().splitlines():
        key, _, value = line.partition(":")
        info[key.strip()] = value.strip()
    resolution = float(info["resolution"])
    origin = [float(v) for v in info["origin"].strip("[]").split(",")]
    grey = Image.open(replay / "slam/map.pgm").convert("RGB")
    scale = 2
    image = grey.resize((grey.width * scale, grey.height * scale), Image.NEAREST)
    draw = ImageDraw.Draw(image)
    c, s = math.cos(start[2]), math.sin(start[2])

    def pixel(x_world: float, y_world: float) -> tuple[float, float]:
        dx, dy = x_world - start[0], y_world - start[1]
        x_map, y_map = c * dx + s * dy, -s * dx + c * dy
        u = (x_map - origin[0]) / resolution
        v = grey.height - (y_map - origin[1]) / resolution
        return u * scale, v * scale

    scan_height = meta["laser"]["mount_xyz_m"][2]
    for item in meta["obstacles"]:
        top = item["base_m"] + item["height_m"]
        if item["base_m"] > 0:
            continue  # loads sit inside their pallet's outline
        stacked = [
            o["base_m"] + o["height_m"]
            for o in meta["obstacles"]
            if o["base_m"] > 0
            and abs(o["x_m"] - item["x_m"]) < item["length_m"] / 2
            and abs(o["y_m"] - item["y_m"]) < item["width_m"] / 2
        ]
        top = max([top, *stacked])
        cy, sy = math.cos(item["yaw_rad"]), math.sin(item["yaw_rad"])
        corners = [
            pixel(
                item["x_m"]
                + cy * a * item["length_m"] / 2
                - sy * b * item["width_m"] / 2,
                item["y_m"]
                + sy * a * item["length_m"] / 2
                + cy * b * item["width_m"] / 2,
            )
            for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1))
        ]
        colour = (0, 170, 0) if top > scan_height else (170, 210, 170)
        draw.polygon(corners, outline=colour)
    colours = [(40, 90, 255), (230, 30, 30), (255, 150, 0)]
    for path, colour in zip(paths.values(), colours, strict=True):
        draw.line([pixel(x, y) for x, y, _ in path], fill=colour, width=2)
    # The map frame points along the start heading; turn it to world axes
    # (east right, north up) before labelling.
    image = image.rotate(math.degrees(start[2]), expand=True, fillcolor=(205, 205, 205))
    draw = ImageDraw.Draw(image)
    for index, (name, colour) in enumerate(zip(paths, colours, strict=True)):
        draw.text((10, 10 + 14 * index), name, fill=colour)
    draw.text((10, 10 + 14 * len(paths)), "north up, east right", fill=(0, 0, 0))
    image.save(replay / "map_overlay.png")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument("--replay", type=Path, required=True)
    args = parser.parse_args(argv)
    meta = json.loads((args.record / "meta.json").read_text())
    manifest = json.loads((args.replay / "replay_manifest.json").read_text())
    offset = manifest["clock_offset_s"]
    with np.load(args.record / "slam_log.npz") as data:
        joint_stamps = data["joint_stamps_s"]
        base = data["base_pose_world"].astype(float)
        scan_stamps = data["scan_stamps_s"]
    truth_all = np.column_stack((base[:, 0], base[:, 1], _yaw(base[:, 3:7])))
    start = truth_all[0]
    truth = truth_all[_rows_at(scan_stamps, joint_stamps)]

    slam = _csv(args.replay / "slam/slam_trajectory.csv")
    slam_rows = _rows_at(scan_stamps, slam[:, 0] - offset)
    odometry = _csv(args.replay / "odometry.csv")
    odometry_rows = _rows_at(scan_stamps, odometry[:, 0])
    if np.any(odometry_rows < 0):
        raise ValueError("odometry.csv does not cover every scan stamp")
    paired = slam_rows >= 0
    slam_world = _to_world(slam[slam_rows[paired], 1:], start)
    odometry_world = _to_world(odometry[odometry_rows, 1:], start)
    slam_error = trajectory_error(slam_world, truth[paired])
    odometry_error = trajectory_error(odometry_world, truth)
    odometry_at_slam = trajectory_error(odometry_world[paired], truth[paired])
    report = {
        "record": str(args.record),
        "replay": str(args.replay),
        "noise": manifest["noise"],
        "scans": int(len(scan_stamps)),
        "slam_poses_paired": int(paired.sum()),
        "slam_poses_missing": int((~paired).sum()),
        "slam": asdict(slam_error),
        "odometry_same_samples": asdict(odometry_at_slam),
        "odometry_all_scans": asdict(odometry_error),
        "note": (
            "first_pose_* aligns only the start pose, as a robot would; ate_* is "
            "the least-squares rigid fit. SLAM is the online map->base_link at "
            "each scan stamp, not a re-optimised trajectory."
        ),
    }
    (args.replay / "evaluation.json").write_text(json.dumps(report, indent=2) + "\n")
    _overlay(
        args.replay,
        meta,
        start,
        {
            "ground truth": truth,
            "slam_toolbox (online)": slam_world,
            "wheel odometry": odometry_world,
        },
    )
    for name in ("slam", "odometry_same_samples"):
        e = report[name]
        print(
            f"{name:22s} first-pose ATE {e['first_pose_ate_rmse_m']:.3f} m  "
            f"LS ATE {e['ate_rmse_m']:.3f} m  max {e['ate_max_m']:.3f} m  "
            f"final {e['final_error_m']:.3f} m over {e['path_length_m']:.1f} m"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
