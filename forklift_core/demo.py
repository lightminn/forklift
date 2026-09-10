"""Run a synthetic sensor-math example: python -m forklift_core.demo."""

import json

import numpy as np

from .geometry import FramePoints, RigidTransform
from .lidar import scan_to_points
from .rgbd import PinholeIntrinsics, deproject_depth_pixels


def _json_points(points: FramePoints) -> dict:
    return {
        "frame_id": points.frame_id,
        "xyz_m": [
            [float(value) if np.isfinite(value) else None for value in row]
            for row in points.xyz_m
        ],
        "valid": points.valid.tolist(),
    }


def main() -> None:
    # All dimensions, focal lengths, depth values and extrinsics below are
    # fictional fixtures. They are not D435i calibration or pallet detections.
    intrinsics = PinholeIntrinsics(5, 3, 2, 4, 2, 1, "example_depth_optical")
    depth = np.full((3, 5), 2000, dtype=np.uint16)
    depth[1, 4] = 0
    optical = deproject_depth_pixels(
        depth,
        [[2, 1], [3, 1], [4, 1]],
        intrinsics,
        meters_per_unit=0.001,
        pixel_frame=intrinsics.frame_id,
        rectified=True,
    )
    mounting = RigidTransform(
        intrinsics.frame_id,
        "example_base_link",
        [[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
        [0.2, 0, 0.5],
    )
    scan = scan_to_points(
        [1, 2, np.inf],
        angle_min_rad=0,
        angle_increment_rad=np.pi / 2,
        range_min_m=0.1,
        range_max_m=10,
        frame_id="example_laser",
    )
    print(
        json.dumps(
            {
                "input_source": "synthetic",
                "scope": "sensor geometry only; no detection, SLAM, motion or hardware validation",
                "depth_in_base": _json_points(mounting.apply(optical)),
                "lidar_in_sensor": _json_points(scan),
            },
            indent=2,
            allow_nan=False,
        )
    )


if __name__ == "__main__":
    main()
