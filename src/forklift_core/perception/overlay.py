"""Project base-frame pocket openings onto RGB images for evaluation display."""

import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

from forklift_core.geometry import RigidTransform
from forklift_core.perception.pocket_observation import Pocket, PocketObservation
from forklift_core.sensors.rgbd import PinholeIntrinsics

TRUTH_COLOUR = (0, 255, 0)  # ground truth openings
ESTIMATE_COLOUR = (255, 0, 255)  # detector estimate
CAPTION_COLOUR = (255, 255, 255)
CAPTION_OUTLINE_COLOUR = (0, 0, 0)


def project_point(
    point_base: ArrayLike,
    intrinsics: PinholeIntrinsics,
    base_from_optical: RigidTransform,
) -> tuple[float, float] | None:
    """Return pixel (u, v) of a base-frame point in metres.

    None when the point is at or behind the optical plane; coordinates outside
    the image are returned unchanged so partial rectangles clip when drawn.
    """
    x, y, z = (
        np.asarray(point_base, dtype=float) - base_from_optical.translation_m
    ) @ base_from_optical.rotation
    if z <= 0:
        return None
    return (
        float(x * intrinsics.fx / z + intrinsics.cx),
        float(y * intrinsics.fy / z + intrinsics.cy),
    )


def opening_corners_m(
    pocket: Pocket, insertion_yaw_rad: float
) -> list[tuple[float, float, float]]:
    """Return base-frame corners ordered TL, TR, BR, BL along the insertion axis."""
    centre = np.asarray(pocket.center_m)
    half_left = (
        pocket.width_m
        / 2
        * np.array([-math.sin(insertion_yaw_rad), math.cos(insertion_yaw_rad), 0.0])
    )
    half_up = np.array([0.0, 0.0, pocket.height_m / 2])
    return [
        tuple(float(value) for value in corner)
        for corner in (
            centre + half_left + half_up,
            centre - half_left + half_up,
            centre - half_left - half_up,
            centre + half_left - half_up,
        )
    ]


def draw_scene_overlay(
    rgb: NDArray[np.uint8],
    *,
    truth: PocketObservation,
    estimate: PocketObservation,
    intrinsics: PinholeIntrinsics,
    base_from_optical: RigidTransform,
    caption: str,
) -> NDArray[np.uint8]:
    """Return an (H, W, 3) uint8 RGB copy with green truth and magenta estimates.

    The white caption is drawn at the top left. Openings with corners at or
    behind the optical plane are skipped. Input pixels are never modified.
    """
    try:
        from PIL import Image, ImageDraw
    except ImportError as exc:
        raise ImportError(
            "Install Pillow with pip install 'forklift-core[dataset]'"
        ) from exc

    image = Image.fromarray(rgb.copy())
    draw = ImageDraw.Draw(image)
    for observation, colour in ((truth, TRUTH_COLOUR), (estimate, ESTIMATE_COLOUR)):
        if observation.status != "valid":
            continue
        for pocket in (observation.left, observation.right):
            corners = opening_corners_m(pocket, observation.insertion_yaw_rad)
            pixels = [
                project_point(corner, intrinsics, base_from_optical)
                for corner in corners
            ]
            if all(pixel is not None for pixel in pixels):
                draw.line([*pixels, pixels[0]], fill=colour, width=2)
    # A dark stroke keeps the caption readable over both sky and pallet pixels.
    draw.text(
        (8, 8),
        caption,
        fill=CAPTION_COLOUR,
        stroke_width=1,
        stroke_fill=CAPTION_OUTLINE_COLOUR,
    )
    return np.array(image, dtype=np.uint8)
