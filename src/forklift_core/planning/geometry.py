"""Planar collision geometry in metres; poses reference the rear axle centre."""

from collections.abc import Sequence
from dataclasses import dataclass
from math import ceil, cos, hypot, isfinite, pi, sin

import numpy as np
from numpy.typing import ArrayLike

from forklift_core._validation import _finite_scalar


def _finite(*values: float) -> None:
    for value in values:
        _finite_scalar(value, "geometry value")


@dataclass(frozen=True)
class Pose2D:
    """Rear axle position and counterclockwise heading in the world frame."""

    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self) -> None:
        _finite(self.x_m, self.y_m, self.yaw_rad)


@dataclass(frozen=True)
class Rectangle:
    """Closed obstacle centred at (x, y), with length along its local x axis."""

    x_m: float
    y_m: float
    length_m: float
    width_m: float
    yaw_rad: float = 0.0

    def __post_init__(self) -> None:
        _finite(self.x_m, self.y_m, self.length_m, self.width_m, self.yaw_rad)
        if min(self.length_m, self.width_m) <= 0:
            raise ValueError("rectangle dimensions must be positive")


@dataclass(frozen=True)
class Footprint:
    """Rectangle extending front/rear from axle; rear is a positive distance.

    Supply measured or explicitly synthetic dimensions. Include forks and the
    carried load in these extents when applicable; no chassis defaults exist.
    """

    front_m: float
    rear_m: float
    half_width_m: float

    def __post_init__(self) -> None:
        _finite(self.front_m, self.rear_m, self.half_width_m)
        if min(self.front_m, self.rear_m) < 0 or self.half_width_m <= 0:
            raise ValueError(
                "footprint extents must be nonnegative with positive width"
            )
        if self.front_m + self.rear_m <= 0:
            raise ValueError("footprint length must be positive")


@dataclass(frozen=True)
class Bounds:
    """Closed axis-aligned world region that must contain the full footprint."""

    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float

    def __post_init__(self) -> None:
        _finite(self.x_min_m, self.x_max_m, self.y_min_m, self.y_max_m)
        if self.x_min_m >= self.x_max_m or self.y_min_m >= self.y_max_m:
            raise ValueError("bounds minima must be less than maxima")


class FootprintCollisionChecker:
    """Precompute obstacle axes and apply the rectangle separating-axis test."""

    def __init__(
        self, obstacles: Sequence[Rectangle], footprint: Footprint, bounds: Bounds
    ) -> None:
        self.footprint = footprint
        self.bounds = bounds
        self.radius_m = hypot(
            max(footprint.front_m, footprint.rear_m), footprint.half_width_m
        )
        self.obstacles = [
            (
                o.x_m,
                o.y_m,
                o.length_m / 2,
                o.width_m / 2,
                cos(o.yaw_rad),
                sin(o.yaw_rad),
            )
            for o in obstacles
        ]

    def free(self, pose: Sequence[float], margin_m: float = 0.0) -> bool:
        """Check a finite (x, y, yaw) triple against precomputed obstacles.

        margin_m inflates each side of the footprint. A negative margin or
        nonfinite pose is invalid and raises ValueError.
        """
        x, y, yaw = pose
        if not all(isfinite(v) for v in (x, y, yaw, margin_m)) or margin_m < 0:
            raise ValueError("pose and margin must be finite; margin nonnegative")
        c, s = cos(yaw), sin(yaw)
        fp = self.footprint
        offset = (fp.front_m - fp.rear_m) / 2
        cx, cy = x + offset * c, y + offset * s
        half_length = (fp.front_m + fp.rear_m) / 2 + margin_m
        half_width = fp.half_width_m + margin_m
        ex = abs(c) * half_length + abs(s) * half_width
        ey = abs(s) * half_length + abs(c) * half_width
        b = self.bounds
        if (
            cx - ex < b.x_min_m
            or cx + ex > b.x_max_m
            or cy - ey < b.y_min_m
            or cy + ey > b.y_max_m
        ):
            return False
        for ox, oy, ol, ow, oc, os in self.obstacles:
            dx, dy = ox - cx, oy - cy
            # Cheap world-axis broad phase before the four exact SAT axes.
            if abs(dx) > ex + abs(oc) * ol + abs(os) * ow:
                continue
            if abs(dy) > ey + abs(os) * ol + abs(oc) * ow:
                continue
            relative_cos = abs(c * oc + s * os)
            relative_sin = abs(s * oc - c * os)
            if (
                abs(dx * c + dy * s)
                > half_length + ol * relative_cos + ow * relative_sin
            ):
                continue
            if (
                abs(-dx * s + dy * c)
                > half_width + ol * relative_sin + ow * relative_cos
            ):
                continue
            if (
                abs(dx * oc + dy * os)
                > ol + half_length * relative_cos + half_width * relative_sin
            ):
                continue
            if (
                abs(-dx * os + dy * oc)
                > ow + half_length * relative_sin + half_width * relative_cos
            ):
                continue
            return False
        return True


def _pose_tuple(pose: Pose2D | ArrayLike) -> tuple[float, float, float]:
    if isinstance(pose, Pose2D):
        return pose.x_m, pose.y_m, pose.yaw_rad
    array = np.asarray(pose, dtype=float)
    if array.shape != (3,) or not np.isfinite(array).all():
        raise ValueError("pose must have three finite x, y, yaw values")
    return tuple(array)


def collision_free_pose(
    pose: Pose2D | ArrayLike,
    obstacles: Sequence[Rectangle],
    footprint: Footprint,
    bounds: Bounds,
    *,
    margin_m: float = 0.0,
) -> bool:
    """Test the entire oriented footprint; obstacle contact counts as collision."""
    _finite(margin_m)
    if margin_m < 0:
        raise ValueError("margin_m must be nonnegative")
    return FootprintCollisionChecker(obstacles, footprint, bounds).free(
        _pose_tuple(pose), margin_m
    )


def collision_free_path(
    poses: ArrayLike,
    obstacles: Sequence[Rectangle],
    footprint: Footprint,
    bounds: Bounds,
    *,
    margin_m: float = 0.0,
    max_step_m: float = 0.05,
) -> bool:
    """Conservative collision check of piecewise linear position/yaw samples.

    Subdivides translation and shortest-angle rotation, then inflates each
    sampled footprint by a bound on point motion to its next sample. Thin
    obstacles cannot hide between samples. This checks geometry, not vehicle
    feasibility; use the planner's controls for the latter. Empty paths are
    rejected. The planner independently checks its true circular arcs.
    """
    array = np.asarray(poses, dtype=float)
    if (
        array.ndim != 2
        or array.shape[1] != 3
        or not len(array)
        or not np.isfinite(array).all()
    ):
        raise ValueError("poses must be a nonempty finite (N, 3) array")
    _finite(margin_m, max_step_m)
    if margin_m < 0 or max_step_m <= 0:
        raise ValueError("margin must be nonnegative and max_step_m positive")
    checker = FootprintCollisionChecker(obstacles, footprint, bounds)
    if not checker.free(array[-1], margin_m):
        return False
    for a, b in zip(array[:-1], array[1:], strict=True):
        dyaw = (b[2] - a[2] + pi) % (2 * pi) - pi
        motion = hypot(b[0] - a[0], b[1] - a[1]) + checker.radius_m * abs(dyaw)
        count = max(1, ceil(motion / max_step_m))
        delta = np.array([b[0] - a[0], b[1] - a[1], dyaw])
        for i in range(count):
            if not checker.free(
                a + (i + 0.5) / count * delta, margin_m + motion / (2 * count)
            ):
                return False
    return checker.free(array[0], margin_m)
