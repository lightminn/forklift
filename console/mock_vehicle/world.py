"""Mock bay for the console's fake vehicle (not a robot model, not a safety function).

Coordinates follow the map frame: x east, y north, metres; poses are rear-axle centres.
The permission here imitates the shape of DrivePermission's answer (limit, reason) so
the console can be exercised; it is not DrivePermission and proves nothing about it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Pose:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class Chassis:
    wheelbase_m: float = 0.66
    track_m: float = 0.53
    half_width_m: float = 0.315
    rear_overhang_m: float = 0.40  # rear bumper behind the rear axle (mock value)
    nose_from_rear_m: float = 1.06  # fork tip ahead of the rear axle (mock value)
    fork_length_m: float = 0.36
    insert_depth_m: float = 0.33  # dls08_measured insertion rule (CLAUDE.md)
    max_curvature_inv_m: float = (
        0.3665  # model value, both wheels at 15 deg (dls08_measured README)
    )
    teleop_curvature_inv_m: float = (
        0.36  # tracker share in isaac_transport_measured.yaml
    )


CHASSIS = Chassis()


@dataclass
class Pallet:
    pallet_id: str
    kind: str
    pose: Pose  # centre; yaw is the direction the truck faces when inserting
    depth_m: float
    width_m: float
    state: str = "available"  # available | carried | delivered


@dataclass
class World:
    size_m: tuple[float, float]
    circles: list[tuple[float, float, float]]
    boxes: list[tuple[float, float, float, float]]
    pallets: list[Pallet]
    destinations: dict[str, Pose]
    home: Pose
    carried: str | None = field(default=None)


def default_world() -> World:
    return World(
        size_m=(14.0, 9.0),
        circles=[(5.4, 2.9, 0.3), (6.0, 3.15, 0.3), (8.5, 7.9, 0.3), (11.6, 7.3, 0.3)],
        boxes=[(7.2, 7.2, 1.2, 0.8), (3.6, 7.9, 1.0, 0.8), (9.6, 3.4, 0.8, 1.0)],
        pallets=[
            Pallet("P-01", "epal6", Pose(12.3, 5.2, 0.0), 0.6, 0.8),
            Pallet("P-02", "t11_x0_6", Pose(12.3, 1.6, 0.0), 0.66, 0.66),
        ],
        destinations={"D-A": Pose(2.4, 1.6, math.pi), "D-B": Pose(2.4, 4.4, math.pi)},
        home=Pose(3.8, 6.2, 0.0),
    )


def _ray_box(ox, oy, dx, dy, cx, cy, hw, hh, yaw, best):
    cs, sn = math.cos(-yaw), math.sin(-yaw)
    lx, ly = (ox - cx) * cs - (oy - cy) * sn, (ox - cx) * sn + (oy - cy) * cs
    ldx, ldy = dx * cs - dy * sn, dx * sn + dy * cs
    t0, t1 = -math.inf, math.inf
    for o, d, h in ((lx, ldx, hw), (ly, ldy, hh)):
        if abs(d) < 1e-12:
            if o < -h or o > h:
                return best
            continue
        a, b = (-h - o) / d, (h - o) / d
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
    if t0 <= t1 and 0.0 < t0 < best:
        return t0
    return best


def raycast(
    world: World,
    x: float,
    y: float,
    dx: float,
    dy: float,
    max_m: float,
    skip: tuple[str, ...] = (),
) -> float:
    """Distance along the unit direction (dx, dy) to the first wall, obstacle or pallet."""
    w, h = world.size_m
    best = max_m
    if dx > 1e-12:
        best = min(best, (w - x) / dx)
    if dx < -1e-12:
        best = min(best, -x / dx)
    if dy > 1e-12:
        best = min(best, (h - y) / dy)
    if dy < -1e-12:
        best = min(best, -y / dy)
    for cx, cy, r in world.circles:
        fx, fy = x - cx, y - cy
        b = fx * dx + fy * dy
        disc = b * b - (fx * fx + fy * fy - r * r)
        if disc >= 0:
            t = -b - math.sqrt(disc)
            if 0.0 < t < best:
                best = t
    for cx, cy, bw, bh in world.boxes:
        best = _ray_box(x, y, dx, dy, cx, cy, bw / 2, bh / 2, 0.0, best)
    for p in world.pallets:
        if p.pallet_id in skip or p.state == "carried":
            continue
        best = _ray_box(
            x,
            y,
            dx,
            dy,
            p.pose.x_m,
            p.pose.y_m,
            p.depth_m / 2,
            p.width_m / 2,
            p.pose.yaw_rad,
            best,
        )
    return best


def loaded_nose_m(load: Pallet) -> float:
    return CHASSIS.nose_from_rear_m - CHASSIS.insert_depth_m + load.depth_m


def clearance_ahead(
    world: World,
    pose: Pose,
    direction: int,
    curvature_inv_m: float = 0.0,
    skip: tuple[str, ...] = (),
    load: Pallet | None = None,
) -> float:
    """Smallest free distance ahead of the travel edge (fork tip, load front or rear bumper)."""
    c = CHASSIS
    heading = pose.yaw_rad + (math.pi if direction < 0 else 0.0)
    half = max(c.half_width_m, load.width_m / 2) if load else c.half_width_m
    nose = loaded_nose_m(load) if load else c.nose_from_rear_m
    edge = c.rear_overhang_m if direction < 0 else nose
    # bend the probe rays toward the commanded turn (mock of the stop arc)
    bend = math.atan(curvature_inv_m * 0.8) * (1 if direction > 0 else -1)
    best = math.inf
    for off in (-half, 0.0, half):
        ox = pose.x_m + math.cos(heading) * edge - math.sin(heading) * off
        oy = pose.y_m + math.sin(heading) * edge + math.cos(heading) * off
        for da in (-0.12, 0.0, 0.12):
            a = heading + bend + da
            best = min(
                best, raycast(world, ox, oy, math.cos(a), math.sin(a), 5.0, skip)
            )
    return best


STOP_M = 0.25
SLOW_M = 0.80


def permission(
    world: World,
    pose: Pose,
    speed_mps: float,
    curvature_inv_m: float,
    load: Pallet | None,
    skip: tuple[str, ...] = (),
) -> tuple[float, str]:
    direction = -1 if speed_mps < 0 else 1
    free = clearance_ahead(world, pose, direction, curvature_inv_m, skip, load)
    cap = (0.25 if direction < 0 else 0.5) * (0.6 if load else 1.0)
    if free < STOP_M:
        return 0.0, f"occupied:{free:.2f}m"
    if free < SLOW_M:
        return min(cap, 0.12), f"slow:{free:.2f}m"
    return cap, "ok"


# Forward camera of the fake vehicle (synthetic, D435i-like field of view). camera_link
# axes: x forward, y left, z up; pitched down about +y. Display overlays only.
_HFOV_RAD = math.radians(69.0)
_PITCH_RAD = math.radians(14.0)
CAMERA_MODEL = {
    "calibration_version": 1,
    "width": 640,
    "height": 400,
    "fx": 320.0 / math.tan(_HFOV_RAD / 2),
    "fy": 320.0 / math.tan(_HFOV_RAD / 2),
    "cx": 320.0,
    "cy": 200.0,
    "base_from_camera": {
        "frame": "camera_link (x forward, y left, z up)",
        "xyz_m": [0.70, 0.0, 1.05],
        "quaternion_xyzw": [
            0.0,
            math.sin(_PITCH_RAD / 2),
            0.0,
            math.cos(_PITCH_RAD / 2),
        ],
    },
}
