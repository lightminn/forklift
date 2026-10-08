"""Fuse frame-to-frame motion from several odometry sources by their uncertainty.

Plan: docs/plans/2026-10-07-visual-slam-and-fusion.md D3. Each source (wheel
odometry, visual odometry, LiDAR ICP odometry) reports base_link's pose in its
own frame once per frame, or nothing (lost, or its sensor delivered nothing).
Between two consecutive frames that a source both reported, its increment
(dx, dy, dyaw in the previous base frame) is an estimate of the same motion.
The fused increment is the per-component inverse-variance mean of the
increments available, each source's standard deviation growing with the
distance and the angle of its own increment (``IncrementNoise``). Only the
sources present enter, so losing one sensor leaves the others carrying the
pose -- that is the point of the fusion. Planar poses: x, y metres, yaw rad.

This is a loosely coupled, memoryless combination: no motion model, and the
components are treated as independent. It is chosen for transparency and for
lockstep determinism, not as an optimal filter.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from forklift_core._validation import _finite_scalar
from forklift_core.localization.slam_pose import compose, invert


@dataclass(frozen=True)
class IncrementNoise:
    """Standard deviation of one frame's increment from one source.

    sigma_xy = trans_floor_m + trans_per_m * d and
    sigma_yaw = rot_floor_rad + rot_per_rad * |dyaw| + rot_per_m * d, with d
    the increment's travelled distance. Assumed design values, tuned on the
    development record only (plan D3), not measured sensor specifications.
    """

    trans_per_m: float
    trans_floor_m: float
    rot_per_rad: float
    rot_per_m: float
    rot_floor_rad: float

    def __post_init__(self) -> None:
        for name in (
            "trans_per_m",
            "trans_floor_m",
            "rot_per_rad",
            "rot_per_m",
            "rot_floor_rad",
        ):
            if _finite_scalar(getattr(self, name), name) < 0:
                raise ValueError(f"{name} must be nonnegative")
        if self.trans_floor_m <= 0 or self.rot_floor_rad <= 0:
            raise ValueError("floors must be positive so every variance is")

    def sigma(self, increment) -> tuple[float, float]:
        dx, dy, dyaw = increment
        distance = math.hypot(dx, dy)
        return (
            self.trans_floor_m + self.trans_per_m * distance,
            self.rot_floor_rad
            + self.rot_per_rad * abs(dyaw)
            + self.rot_per_m * distance,
        )


def increment(previous, current) -> tuple[float, float, float]:
    """current expressed in previous: the motion between two poses of one frame."""
    dx, dy, dyaw = compose(invert(previous), current)
    return dx, dy, math.atan2(math.sin(dyaw), math.cos(dyaw))


def fuse(increments: dict[str, tuple], noise: dict[str, IncrementNoise]) -> tuple:
    """Inverse-variance mean of the given increments, per component."""
    if not increments:
        raise ValueError("at least one increment is needed")
    sums = [0.0, 0.0, 0.0]
    weights = [0.0, 0.0, 0.0]
    for name, delta in increments.items():
        sigma_xy, sigma_yaw = noise[name].sigma(delta)
        for axis, sigma in ((0, sigma_xy), (1, sigma_xy), (2, sigma_yaw)):
            w = 1.0 / (sigma * sigma)
            sums[axis] += w * delta[axis]
            weights[axis] += w
    return tuple(s / w for s, w in zip(sums, weights, strict=True))


class FusedOdometry:
    """Accumulates fused increments from the sources reported at each frame.

    ``update`` takes, per source, base_link's pose in that source's frame at
    this frame, or None. A source contributes an increment only when it
    reported this frame and the previous one. A frame with no increment at all
    keeps the fused pose (nothing moved it); the caller decides whether that is
    acceptable (pure vision without wheels, while vision is lost).
    """

    def __init__(
        self, noise: dict[str, IncrementNoise], *, initial_pose=(0.0, 0.0, 0.0)
    ):
        if not noise:
            raise ValueError("at least one source is needed")
        self.noise = dict(noise)
        self.pose = tuple(_finite_scalar(v, "initial pose") for v in initial_pose)
        self._previous: dict[str, tuple] = {}
        self.last_sources: tuple[str, ...] = ()

    def update(self, poses: dict[str, tuple | None]) -> tuple[float, float, float]:
        unknown = set(poses) - set(self.noise)
        if unknown:
            raise ValueError(f"no noise model for {sorted(unknown)}")
        increments = {}
        for name, pose in poses.items():
            previous = self._previous.get(name)
            if pose is not None and previous is not None:
                increments[name] = increment(previous, pose)
        self._previous = {name: tuple(p) for name, p in poses.items() if p is not None}
        self.last_sources = tuple(sorted(increments))
        if increments:
            self.pose = compose(self.pose, fuse(increments, self.noise))
        return self.pose


__all__ = ["FusedOdometry", "IncrementNoise", "fuse", "increment"]
