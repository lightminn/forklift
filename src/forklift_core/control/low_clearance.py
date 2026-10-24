"""Low-clearance guard (docs/plans/2026-10-25-low-clearance-guard.md).

Props below the LiDAR plane are not in the obstacle layer, so the drive permission never sees them: only the planning
clearance keeps the truck off them, and a tracking deviation (a heading error on the loaded outline's long front) can
use it up. The guard watches those props along the permission's stop arc from the estimated pose. Each check is held
to the planned clearance the same distance ahead on the leg -- ``min(margin, max(0, c - deviation))`` -- so a plan that
passes close on purpose is not stopped while it is followed, and contact is always a violation. The speed is capped
so the truck stops before the first violation; past the leg end the arc runs only as far as the truck can overrun it
(the end allowance or the physical braking distance) with the planned clearance carried on less the overrun, and
interval midpoints with half the motion between samples taken off keep the sweep between
samples (as the permission's walk does)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from forklift_core.control.drive_permission import StoppingModel, arc_poses, parts_of
from forklift_core.control.start_release import footprint_rect_distance
from forklift_core.planning.pallet_mission import LIDAR_PLANE_M, SEARCH_TAG, SLOT_TAG, TransportScenario


@dataclass(frozen=True)
class LowClearanceConfig:
    margin_m: float = 0.05  # the share of localisation error and pallet slip on the forks the guard keeps
    deviation_m: float = 0.02  # how much nearer than planned a tight plan may be followed
    step_m: float = 0.01  # arc sample spacing; half the motion between samples comes off each midpoint

    def __post_init__(self) -> None:
        for name in ("margin_m", "deviation_m", "step_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0) or (name == "step_m" and value <= 0.0):
                raise ValueError(f"{name} must be finite and non-negative (step positive)")


def low_props(scenario: TransportScenario, stage: str, plane_m: float = LIDAR_PLANE_M) -> list:
    """The rectangles, at their size, of ``scenario``'s props no taller than ``plane_m`` that a check at ``stage`` sees
    (pallet_mission.scenario_props' stage rule: "search" leaves out the slot copies, "slot" the search ones)."""
    if stage not in ("search", "slot"):
        raise ValueError(f"unknown stage {stage!r}")
    drop = SLOT_TAG if stage == "search" else SEARCH_TAG
    return [p.rectangle for p in scenario.props if not p.asset.uri.endswith(drop) and p.asset.height_m <= plane_m]


def shape_radius(shape) -> float:
    """The farthest any point of the shape lies from the rear axle."""
    return max(math.hypot(abs(lon) + max(fp.front_m, fp.rear_m), abs(lat) + fp.half_width_m)
               for fp, lat, lon in parts_of(shape))


def shape_clearance(pose, shape, rects) -> float:
    """Exact distance between the shape at ``pose`` and the nearest rectangle (0 where they meet, inf with none)."""
    if not rects:
        return math.inf
    reach = shape_radius(shape)
    best = math.inf
    for r in rects:
        if math.hypot(r.x_m - pose[0], r.y_m - pose[1]) - math.hypot(r.length_m, r.width_m) / 2 - reach > best:
            continue  # farther than the nearest already found
        best = min(best, footprint_rect_distance(tuple(float(v) for v in pose), shape, r))
        if best <= 0.0:
            return 0.0
    return best


@dataclass
class LegClearance:
    """The planned clearance ``c(s)`` along one tracker path: at every path pose, the shape's distance to the low props
    of the stage that planned that part of the path (``slot_from_m``: path length from which the slot props count --
    a transport's delivery straight; None: none). Built once when the tracker is installed."""

    poses: np.ndarray
    shape: object
    search_props: list
    slot_props: list
    slot_from_m: float | None = None
    s: np.ndarray = field(init=False)
    c: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        poses = np.asarray(self.poses, dtype=float)
        if poses.ndim != 2 or poses.shape[1] != 3 or len(poses) < 1:
            raise ValueError("poses must be an (N, 3) array")
        self.poses = poses
        self.s = np.r_[0.0, np.cumsum(np.hypot(np.diff(poses[:, 0]), np.diff(poses[:, 1])))]
        self.c = np.array([shape_clearance(p, self.shape, self.props_at(si)) for p, si in zip(poses, self.s)])

    @property
    def length_m(self) -> float:
        return float(self.s[-1])

    def props_at(self, s: float) -> list:
        return self.slot_props if self.slot_from_m is not None and s >= self.slot_from_m else self.search_props

    def at(self, s: float) -> float:
        """c at path length ``s`` (clamped to the path), linear between path poses; inf where no prop is near."""
        s = min(max(float(s), 0.0), self.length_m)
        if len(self.s) == 1:
            return float(self.c[0])
        k = int(np.clip(np.searchsorted(self.s, s, side="right") - 1, 0, len(self.s) - 2))
        a, b = float(self.c[k]), float(self.c[k + 1])
        if self.slot_from_m is not None and (self.s[k] >= self.slot_from_m) != (self.s[k + 1] >= self.slot_from_m):
            # the two poses belong to different stages: the side s is on, never a blend of the two (Codex round 8 P2)
            return a if (s >= self.slot_from_m) == (self.s[k] >= self.slot_from_m) else b
        if not (math.isfinite(a) and math.isfinite(b)):
            return min(a, b)
        span = float(self.s[k + 1] - self.s[k])
        t = 0.0 if span <= 0.0 else (s - float(self.s[k])) / span
        return a + t * (b - a)


@dataclass(frozen=True)
class GuardCap:
    allowed_mps: float  # inf: no cap
    s_star_m: float  # arc length of the first violation (inf: none)
    lower_bound_m: float  # the violating check's lower bound (inf: none)
    threshold_m: float  # its m(s)
    arc_m: float  # the arc length looked at


def guard_cap(pose, curvature_inv_m: float, direction: int, speed_cap_mps: float, shape, leg: LegClearance, *,
              progress_m: float, leg_remaining_m: float, end_allowance_m: float, stopping: StoppingModel,
              config: LowClearanceConfig) -> GuardCap:
    """The speed cap of one control tick (plan D3): the stop arc at the held curvature; the first sample, then each
    interval midpoint with half the motion between samples taken off, against
    ``min(margin, max(0, c(progress + s) - deviation))`` -- a lower bound at or below 0 always violates.

    Past the leg end (the tracker stops there) the arc runs to the farther of the end plus ``end_allowance_m`` and the
    physical braking distance at this speed (latency and deceleration, without the permission's fixed stopping margin):
    the truck can overrun the end that far, never the margin. There the planned clearance is carried on from the end,
    less the distance past it (as if the end were driven straight on into the prop)."""
    step = config.step_m
    remaining = max(0.0, float(leg_remaining_m))
    v = abs(float(speed_cap_mps))
    braking = v * stopping.latency_s + v * v / (2.0 * stopping.decel_mps2)
    length = min(stopping.distance_m(speed_cap_mps) + step,
                 max(remaining + max(0.0, end_allowance_m), braking + step))
    samples, arc = arc_poses(tuple(float(v) for v in pose), float(curvature_inv_m), 1 if direction >= 0 else -1,
                             length, step)
    radius = shape_radius(shape)
    checks = [(samples[0], float(arc[0]), 0.0, float(arc[0]))]
    for i in range(1, len(samples)):
        a, b = samples[i - 1], samples[i]
        dyaw = float(np.arctan2(np.sin(b[2] - a[2]), np.cos(b[2] - a[2])))
        motion = float(np.hypot(b[0] - a[0], b[1] - a[1])) + radius * abs(dyaw)
        mid = np.array([(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, a[2] + dyaw / 2])
        checks.append((mid, (float(arc[i - 1]) + float(arc[i])) / 2, motion / 2, float(arc[i - 1])))
    end_along = progress_m + remaining
    for check_pose, s_check, pad, s_from in checks:
        along = min(progress_m + s_check, end_along)
        lower = shape_clearance(check_pose, shape, leg.props_at(along)) - pad
        planned = leg.at(along) - max(0.0, s_check - remaining)
        threshold = min(config.margin_m, max(0.0, planned - config.deviation_m))
        if lower <= 0.0 or lower < threshold:
            return GuardCap(stopping.speed_for(s_from) if s_from > 0.0 else 0.0, s_from, lower, threshold, length)
    return GuardCap(math.inf, math.inf, math.inf, math.inf, length)
