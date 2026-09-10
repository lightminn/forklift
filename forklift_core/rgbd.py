"""Deprojection for a rectified depth grid, independent of the camera SDK."""

from dataclasses import dataclass
from numbers import Integral

import numpy as np
from numpy.typing import ArrayLike

from .geometry import FramePoints, _finite_scalar, _frame_id, _real_array


@dataclass(frozen=True)
class PinholeIntrinsics:
    """Calibration of the exact rectified grid sampled by the caller."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    frame_id: str

    def __post_init__(self) -> None:
        for name in ("width", "height"):
            value = getattr(self, name)
            if (
                isinstance(value, (bool, np.bool_))
                or not isinstance(value, Integral)
                or value <= 0
            ):
                raise ValueError(f"{name} must be a positive integer")
        for name in ("fx", "fy", "cx", "cy"):
            _finite_scalar(getattr(self, name), name)
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError("Focal lengths must be positive")
        _frame_id(self.frame_id)


def deproject_depth_pixels(
    depth: ArrayLike,
    pixels_uv: ArrayLike,
    intrinsics: PinholeIntrinsics,
    *,
    meters_per_unit: float,
    pixel_frame: str,
    rectified: bool,
) -> FramePoints:
    """Convert depth pixels to optical-frame xyz in metres.

    Optical axes: +x right, +y down, +z forward. Depth is axial z, not
    Euclidean ray length. Pixel indices must already belong to this depth
    grid. Register colour detections through the SDK/adapter before calling;
    equal image sizes do not prove RGB/depth alignment. Raw distorted grids
    need a distortion-aware deprojection adapter instead of this function.

    The scale is mandatory: obtain it from the stream/SDK metadata. Neither
    a sensor name nor the storage dtype implies a depth scale.
    """
    if rectified is not True:
        raise ValueError("Only explicitly rectified pinhole grids are supported")
    if pixel_frame != intrinsics.frame_id:
        raise ValueError("Pixel and calibration frames must match")
    scale = _finite_scalar(meters_per_unit, "meters_per_unit")
    if scale <= 0:
        raise ValueError("meters_per_unit must be positive")
    image = _real_array(depth, "depth")
    if image.shape != (intrinsics.height, intrinsics.width):
        raise ValueError("Depth image shape must match the calibration")
    pixels = np.asarray(pixels_uv)
    if pixels.ndim != 2 or pixels.shape[1] != 2 or pixels.dtype.kind not in "ui":
        raise ValueError("pixels_uv must have shape (N, 2) and contain integer indices")
    u, v = pixels[:, 0], pixels[:, 1]
    if ((u < 0) | (u >= intrinsics.width) | (v < 0) | (v >= intrinsics.height)).any():
        raise ValueError("Pixel index lies outside the depth grid")

    xyz = np.full((len(pixels), 3), np.nan)
    with np.errstate(over="ignore", invalid="ignore"):
        z = image[v, u] * scale
        valid = np.isfinite(z) & (z > 0)
        xyz[valid, 0] = (
            (u[valid].astype(float) - intrinsics.cx) * z[valid] / intrinsics.fx
        )
        xyz[valid, 1] = (
            (v[valid].astype(float) - intrinsics.cy) * z[valid] / intrinsics.fy
        )
        xyz[valid, 2] = z[valid]
    xyz[~np.isfinite(xyz).all(axis=1)] = np.nan
    return FramePoints(intrinsics.frame_id, xyz)
