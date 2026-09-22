"""Pure geometry and colour functions shared by Isaac recording and deck tools."""

import math

import numpy as np

from forklift_core.geometry import RigidTransform, rotation_matrix_from_quaternion_xyzw
from forklift_core.perception.overlay import project_point
from forklift_core.sensors.rgbd import PinholeIntrinsics

PHASE_LABELS = {
    "observe": "관측",
    "approach": "접근",
    "insert": "삽입",
    "lift": "들기",
    "extract": "후진",
    "transport": "운반",
    "lower": "하역",
    "withdraw": "후진",
    "settle": "하역",
    "complete": "하역",
}


def parse_extra_views(value: str, video: bool, use_perception: bool) -> tuple[str, ...]:
    """Validate optional comma-separated views without changing empty defaults."""
    names = tuple(value.split(",")) if value else ()
    if len(set(names)) != len(names) or any(
        name not in {"quarter", "chase", "perception"} for name in names
    ):
        raise ValueError("Unknown or repeated extra view")
    if names and not video:
        raise ValueError("--extra-views requires --video")
    if "perception" in names and not use_perception:
        raise ValueError("perception extra view requires --use-perception")
    return names


def chase_pose(base_position_m, yaw_rad, previous_yaw_rad, dt_s):
    """Return camera eye, target and wrap-safe low-pass yaw (tau=0.5 s)."""
    if previous_yaw_rad is None:
        smooth_yaw = yaw_rad
    else:
        difference = math.atan2(
            math.sin(yaw_rad - previous_yaw_rad),
            math.cos(yaw_rad - previous_yaw_rad),
        )
        smooth_yaw = previous_yaw_rad + (1 - math.exp(-dt_s / 0.5)) * difference
    base = np.asarray(base_position_m, dtype=float)
    forward = np.array([math.cos(smooth_yaw), math.sin(smooth_yaw), 0.0])
    left = np.array([-forward[1], forward[0], 0.0])
    eye = base - 2.4 * forward + 0.6 * left + [0, 0, 1.6]
    target = base + 0.9 * forward + [0, 0, 0.3]
    return eye, target, smooth_yaw


def depth_colormap(depth_m):
    """Map 0.3–4 m axial depth to a small viridis-like RGB LUT; invalid is black."""
    depth = np.asarray(depth_m, dtype=float)
    anchors = np.array(
        [
            [68, 1, 84],
            [59, 82, 139],
            [33, 145, 140],
            [94, 201, 98],
            [253, 231, 37],
        ]
    )
    valid = np.isfinite(depth) & (depth > 0)
    scaled = np.clip((np.where(valid, depth, 0) - 0.3) / 3.7, 0, 1) * 4
    low = np.minimum(scaled.astype(int), 3)
    colour = anchors[low] * (1 - (scaled - low))[..., None]
    colour += anchors[low + 1] * (scaled - low)[..., None]
    result = colour.astype(np.uint8)
    result[~valid] = 0
    return result


def phase_label(phase: str) -> str:
    """Return the Korean label for a transport mission phase."""
    return PHASE_LABELS[phase]


def project_world_opening(
    corners_world_m, base_position_m, base_orientation_wxyz, mount, intrinsics
):
    """Project one fixed world opening through the frame's base and optical pose."""
    w, x, y, z = base_orientation_wxyz
    world_from_base = rotation_matrix_from_quaternion_xyzw([x, y, z, w])
    corners_base = (np.asarray(corners_world_m) - base_position_m) @ world_from_base
    calibration = PinholeIntrinsics(**intrinsics)
    base_from_optical = RigidTransform(
        calibration.frame_id, "base_link", mount["rotation"], mount["translation_m"]
    )
    pixels = [
        project_point(corner, calibration, base_from_optical) for corner in corners_base
    ]
    return pixels if all(pixel is not None for pixel in pixels) else None
