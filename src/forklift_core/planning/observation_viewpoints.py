"""Observation viewpoints generated at run time from the map and the pickup zone.

Plan: docs/plans/2026-10-03-runtime-observation-viewpoints.md. The runner tries
its fixed observation waypoints first; these are appended after them. The
inputs are the occupancy map as unlabelled rectangles, a fixed pickup zone (the
task's placement area, not a pallet pose) and the vehicle and camera constants.
No input says which rectangle is the pallet: every rectangle is an obstacle,
and a rectangle whose centre lies in the pickup zone is never an occluder,
because it may be the pallet itself.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from .geometry import Bounds, Footprint, Pose2D, Rectangle, collision_free_pose


# The runner's fixed observation waypoints (rear axle x, y, yaw), tried in this order
# before any generated one; run_transport.py documents why each is where it is.
# Shared so CPU baselines plan from the same candidates (priority-5 plan D7).
DEFAULT_OBSERVATION_WAYPOINTS = (
    (-0.10, 0.90, 0.0),
    (-1.20, 0.30, 0.0),
    (-0.10, -0.60, 0.0),
    (-1.50, -0.60, 0.0),
    (-2.00, -0.30, 0.0),
    (0.00, 2.10, -0.25),
    (0.40, 1.20, 0.0),
    (-0.60, 1.80, -0.25),
)


@dataclass(frozen=True)
class PickupZone:
    """Where a pallet centre may be, and its heading range (task constants)."""

    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    yaw_max_rad: float

    def contains(self, x_m: float, y_m: float) -> bool:
        return self.x_min_m <= x_m <= self.x_max_m and self.y_min_m <= y_m <= self.y_max_m

    @property
    def centre(self) -> tuple[float, float]:
        return (self.x_min_m + self.x_max_m) / 2, (self.y_min_m + self.y_max_m) / 2


# The synthetic bay's placement distribution (pallet_mission.make_scenario).
BAY_PICKUP_ZONE = PickupZone(2.6, 3.6, -0.2, 1.5, math.radians(12))


@dataclass(frozen=True)
class ViewpointConfig:
    """Generation rule; values fixed by the plan before the Isaac run."""

    lattice_x_m: tuple[float, float, float] = (-2.0, 1.0, 0.25)
    lattice_y_m: tuple[float, float, float] = (-1.5, 2.75, 0.25)
    yaw_offsets_rad: tuple[float, ...] = (0.0, -0.2, 0.2)
    zone_samples_x: int = 6
    zone_samples_y: int = 10
    distance_m: tuple[float, float] = (1.3, 4.0)
    oblique_max_rad: float = math.radians(25)
    fov_margin_rad: float = math.radians(3)
    min_score: float = 0.3
    limit: int = 6
    min_separation_m: float = 0.5
    # The runner accepts an observation stop within 3 cm and 0.05 rad, and plans
    # the next leg from there: every pose in that box must keep the margin.
    arrival_position_m: float = 0.03
    arrival_yaw_rad: float = 0.05


@dataclass(frozen=True)
class Viewpoint:
    pose: Pose2D
    score: float


def _arange(start: float, stop: float, step: float) -> list[float]:
    count = int(math.floor((stop - start) / step + 1e-9)) + 1
    return [round(start + i * step, 6) for i in range(count)]


def _linspace(start: float, stop: float, count: int) -> list[float]:
    if count == 1:
        return [(start + stop) / 2]
    return [start + (stop - start) * i / (count - 1) for i in range(count)]


def _segment_hits(p: tuple[float, float], q: tuple[float, float], rect: Rectangle) -> bool:
    """Whether segment pq meets the closed rectangle (slab test in its frame)."""
    c, s = math.cos(rect.yaw_rad), math.sin(rect.yaw_rad)

    def local(point):
        dx, dy = point[0] - rect.x_m, point[1] - rect.y_m
        return c * dx + s * dy, -s * dx + c * dy

    (x0, y0), (x1, y1) = local(p), local(q)
    t0, t1 = 0.0, 1.0
    for delta, origin, half in (
        (x1 - x0, x0, rect.length_m / 2),
        (y1 - y0, y0, rect.width_m / 2),
    ):
        if abs(delta) < 1e-12:
            if abs(origin) > half:
                return False
            continue
        ta, tb = (-half - origin) / delta, (half - origin) / delta
        if ta > tb:
            ta, tb = tb, ta
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 > t1:
            return False
    return True


def view_score(
    pose: Pose2D,
    occluders: Sequence[Rectangle],
    zone: PickupZone,
    pallet_depth_m: float,
    pallet_width_m: float,
    rear_to_camera_m: float,
    half_fov_rad: float,
    config: ViewpointConfig,
) -> float:
    """Fraction of sampled pallet placements whose whole front this pose would see."""
    cx = pose.x_m + rear_to_camera_m * math.cos(pose.yaw_rad)
    cy = pose.y_m + rear_to_camera_m * math.sin(pose.yaw_rad)
    fov = half_fov_rad - config.fov_margin_rad
    seen = total = 0
    for x in _linspace(zone.x_min_m, zone.x_max_m, config.zone_samples_x):
        for y in _linspace(zone.y_min_m, zone.y_max_m, config.zone_samples_y):
            for psi in (-zone.yaw_max_rad, 0.0, zone.yaw_max_rad):
                total += 1
                c, s = math.cos(psi), math.sin(psi)
                face = (x - pallet_depth_m / 2 * c, y - pallet_depth_m / 2 * s)
                distance = math.hypot(face[0] - cx, face[1] - cy)
                if not config.distance_m[0] <= distance <= config.distance_m[1]:
                    continue
                # Line of sight against the face's outward normal -(c, s).
                vx, vy = cx - face[0], cy - face[1]
                if abs(math.atan2(vx * s - vy * c, -(vx * c + vy * s))) > config.oblique_max_rad:
                    continue
                visible = True
                for lateral in (-pallet_width_m / 2, 0.0, pallet_width_m / 2):
                    point = (face[0] - lateral * s, face[1] + lateral * c)
                    bearing = math.atan2(point[1] - cy, point[0] - cx) - pose.yaw_rad
                    bearing = math.atan2(math.sin(bearing), math.cos(bearing))
                    if abs(bearing) > fov or any(
                        _segment_hits((cx, cy), point, r) for r in occluders
                    ):
                        visible = False
                        break
                seen += visible
    return seen / total


def runtime_viewpoints(
    occupied: Sequence[Rectangle],
    footprint: Footprint,
    bounds: Bounds,
    *,
    margin_m: float,
    pallet_depth_m: float,
    pallet_width_m: float,
    rear_to_camera_m: float,
    half_fov_rad: float,
    zone: PickupZone = BAY_PICKUP_ZONE,
    config: ViewpointConfig | None = None,
) -> list[Viewpoint]:
    """Ranked, spread-out viewpoints that keep ``margin_m`` across the arrival box."""
    config = config if config is not None else ViewpointConfig()
    occluders = [r for r in occupied if not zone.contains(r.x_m, r.y_m)]
    centre = zone.centre
    p, w = config.arrival_position_m, config.arrival_yaw_rad
    arrival = [
        (dx, dy, dyaw) for dx in (-p, 0.0, p) for dy in (-p, 0.0, p) for dyaw in (-w, 0.0, w)
    ]
    scored = []
    for x in _arange(*config.lattice_x_m):
        for y in _arange(*config.lattice_y_m):
            aim = math.atan2(centre[1] - y, centre[0] - x)
            for offset in config.yaw_offsets_rad:
                pose = Pose2D(x, y, round(aim + offset, 4))
                if not all(
                    collision_free_pose(
                        Pose2D(x + dx, y + dy, pose.yaw_rad + dyaw),
                        occupied,
                        footprint,
                        bounds,
                        margin_m=margin_m,
                    )
                    for dx, dy, dyaw in arrival
                ):
                    continue
                score = view_score(
                    pose,
                    occluders,
                    zone,
                    pallet_depth_m,
                    pallet_width_m,
                    rear_to_camera_m,
                    half_fov_rad,
                    config,
                )
                if score >= config.min_score:
                    scored.append(Viewpoint(pose, score))
    # Stable: equal scores keep lattice order.
    scored.sort(key=lambda v: -v.score)
    chosen: list[Viewpoint] = []
    for viewpoint in scored:
        if all(
            math.hypot(viewpoint.pose.x_m - c.pose.x_m, viewpoint.pose.y_m - c.pose.y_m)
            >= config.min_separation_m
            for c in chosen
        ):
            chosen.append(viewpoint)
            if len(chosen) == config.limit:
                break
    return chosen
