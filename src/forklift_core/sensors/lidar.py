"""Metric ROS-style scan geometry without assuming an RPLIDAR model."""

from dataclasses import dataclass

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


@dataclass(frozen=True)
class PlanarScanPattern:
    """A full 360 degree planar scan: beam i points at -pi + i * 2 pi / N.

    Beam angles are radians in the laser frame, zero along +x and positive
    toward +y, matching scan_to_points. The pattern is whatever configuration
    the caller supplies; it does not describe a particular sensor model.
    """

    beam_count: int
    range_min_m: float
    range_max_m: float

    def __post_init__(self) -> None:
        if (
            isinstance(self.beam_count, bool)
            or not isinstance(self.beam_count, int)
            or self.beam_count <= 0
        ):
            raise ValueError("beam_count must be a positive integer")
        lower = _finite_scalar(self.range_min_m, "range_min_m")
        upper = _finite_scalar(self.range_max_m, "range_max_m")
        if lower < 0 or upper <= lower:
            raise ValueError("ranges must satisfy 0 <= range_min_m < range_max_m")

    @property
    def angle_min_rad(self) -> float:
        return -np.pi

    @property
    def angle_increment_rad(self) -> float:
        return 2 * np.pi / self.beam_count

    def beam_angles_rad(self) -> np.ndarray:
        return (
            self.angle_min_rad + np.arange(self.beam_count) * self.angle_increment_rad
        )

    def ranges_from_hits(self, distances_m: ArrayLike, hit: ArrayLike) -> np.ndarray:
        """Turn ray-cast results into LaserScan ranges following REP-117.

        distances_m: (N,) distance to the first surface for rays that hit, cast
        no further than range_max_m. hit: (N,) booleans. A miss becomes +inf
        (nothing within range) and a hit closer than range_min_m becomes -inf
        (too close to measure). Neither is ever reported as a distance.
        """
        distances = _real_array(distances_m, "distances_m")
        hits = np.asarray(hit, dtype=bool)
        if distances.shape != (self.beam_count,) or hits.shape != distances.shape:
            raise ValueError("distances and hits need one entry per beam")
        measured = distances[hits]
        if not np.isfinite(measured).all() or np.any(measured < 0):
            raise ValueError("hit distances must be finite and nonnegative")
        if np.any(measured > self.range_max_m):
            raise ValueError("a hit lies beyond range_max_m; cast no further than it")
        ranges = np.full(self.beam_count, np.inf)
        ranges[hits] = np.where(measured < self.range_min_m, -np.inf, measured)
        return ranges
