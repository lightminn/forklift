"""Metric ROS-style scan geometry without assuming an RPLIDAR model."""

import numpy as np
from numpy.typing import ArrayLike

from forklift_core._validation import _finite_scalar, _frame_id, _real_array
from forklift_core.geometry import FramePoints


def scan_to_points(
    ranges_m: ArrayLike,
    *,
    angle_min_rad: float,
    angle_increment_rad: float,
    range_min_m: float,
    range_max_m: float,
    frame_id: str,
) -> FramePoints:
    """Keep one row per beam; invalid beams remain unknown (all NaN).

    Angles are radians, zero along +x and positive toward +y. Bounds are
    inclusive and must come from stream metadata. This conversion neither
    deskews the scan nor turns a 2D sensor into a 3D collision guarantee.
    """
    _frame_id(frame_id)
    angle_min = _finite_scalar(angle_min_rad, "angle_min_rad")
    increment = _finite_scalar(angle_increment_rad, "angle_increment_rad")
    lower = _finite_scalar(range_min_m, "range_min_m")
    upper = _finite_scalar(range_max_m, "range_max_m")
    if increment == 0 or lower < 0 or upper <= lower:
        raise ValueError("Scan requires a nonzero angle increment and 0 <= min < max")
    ranges = _real_array(ranges_m, "ranges_m")
    if ranges.ndim != 1:
        raise ValueError("ranges_m must be one-dimensional")
    with np.errstate(over="ignore", invalid="ignore"):
        angles = angle_min + np.arange(len(ranges)) * increment
    if not np.isfinite(angles).all():
        raise ValueError("Scan angles overflowed")
    valid = np.isfinite(ranges) & (ranges > 0) & (ranges >= lower) & (ranges <= upper)
    xyz = np.full((len(ranges), 3), np.nan)
    xyz[valid, 0] = ranges[valid] * np.cos(angles[valid])
    xyz[valid, 1] = ranges[valid] * np.sin(angles[valid])
    xyz[valid, 2] = 0
    return FramePoints(frame_id, xyz)
