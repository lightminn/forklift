"""Hermite path legs for the mock mission (display and playback only)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from console.mock_vehicle.world import Pose


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Leg:
    points: list[tuple[float, float, float, float, float]]  # x, y, yaw, s, curvature
    direction: int
    speed_mps: float
    s_m: float = 0.0
    index: int = field(default=0)

    @property
    def length_m(self) -> float:
        return self.points[-1][3]


def hermite_leg(p0: Pose, p1: Pose, direction: int, speed_mps: float) -> Leg:
    d = math.hypot(p1.x_m - p0.x_m, p1.y_m - p0.y_m)
    k = max(1.1 * d, 0.4)
    t0 = (math.cos(p0.yaw_rad) * direction * k, math.sin(p0.yaw_rad) * direction * k)
    t1 = (math.cos(p1.yaw_rad) * direction * k, math.sin(p1.yaw_rad) * direction * k)
    n = max(16, math.ceil(d / 0.04))
    xy = []
    for i in range(n + 1):
        t = i / n
        a, b, c, e = (
            2 * t**3 - 3 * t**2 + 1,
            t**3 - 2 * t**2 + t,
            -2 * t**3 + 3 * t**2,
            t**3 - t**2,
        )
        xy.append(
            (
                a * p0.x_m + b * t0[0] + c * p1.x_m + e * t1[0],
                a * p0.y_m + b * t0[1] + c * p1.y_m + e * t1[1],
            )
        )
    pts = [(xy[0][0], xy[0][1], p0.yaw_rad, 0.0, 0.0)]
    s = 0.0
    for i in range(1, len(xy)):
        (x0, y0), (x1, y1) = xy[i - 1], xy[i]
        ds = math.hypot(x1 - x0, y1 - y0)
        s += ds
        yaw = _wrap(math.atan2(y1 - y0, x1 - x0) + (math.pi if direction < 0 else 0.0))
        if i == len(xy) - 1:
            yaw = p1.yaw_rad
        kappa = _wrap(yaw - pts[-1][2]) / max(ds, 1e-9) * direction
        pts.append((x1, y1, yaw, s, kappa))
    return Leg(points=pts, direction=direction, speed_mps=speed_mps)


def advance(leg: Leg, ds_m: float) -> Pose:
    leg.s_m = min(leg.length_m, leg.s_m + max(ds_m, 0.0))
    pts = leg.points
    while leg.index < len(pts) - 2 and pts[leg.index + 1][3] < leg.s_m:
        leg.index += 1
    a, b = pts[leg.index], pts[min(leg.index + 1, len(pts) - 1)]
    u = (leg.s_m - a[3]) / (b[3] - a[3]) if b[3] > a[3] else 1.0
    return Pose(
        a[0] + (b[0] - a[0]) * u,
        a[1] + (b[1] - a[1]) * u,
        a[2] + _wrap(b[2] - a[2]) * u,
    )


def curvature_at(leg: Leg) -> float:
    return leg.points[min(leg.index + 1, len(leg.points) - 1)][4]
