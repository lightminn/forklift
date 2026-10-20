# src/forklift_core/mission/bottom_deck_legs.py
"""Where the truck stands to see each bottom-deck evidence face (plan 2026-10-19 task 4).

The camera stands ``CAMERA_RANGE_M`` from the pallet centre, looking at it. Its direction is set from the
evidence face's outward normal by theta_e: the x hypothesis sees the two model +-y sides at 35 deg, tilted toward
the entry face the truck comes from (55 deg from the entry face); the T11 y hypothesis sees the back face and the
entry face head on (the back first). The truck pose is worked back from the camera mount: base ``mount_x_m``
behind the camera, the rear axle ``rear_axle_offset_m`` behind the base. Poses are rear-axle poses, the
observation leg planner's waypoints.
"""

import math
from dataclasses import dataclass

import numpy as np


CAMERA_RANGE_M = 1.35
THETA_E_DEG = {"x": 35.0, "y": 0.0}
# Faces as model +-y (the bottom_deck_evidence convention). For the y hypothesis the insertion runs along model
# +y (model yaw = insertion yaw - 90 deg), so the entry face is -1 and the back face +1.
FACES = {"x": (1, -1), "y": (1, -1)}
RETREAT_M = 0.3
RETREAT_MAX_M = 1.0  # a longer planned retreat is a detour: the face ends (no detour on held odometry)
REOBSERVE_RADIUS_M = 0.3  # the re-observation's zone around the pallet judged (the gate binds 0.15 m)
RETURN_RECOVERIES = 3  # replans of the return to the far observation point before it is given up
RECOVERIES_PER_FACE = 3  # path-replacing recoveries on one face before it is given up
TRAVEL_M = 0.3  # room around the stands and retreats for the side legs to manoeuvre
CAPTURES = 3  # per face; the first present or absent stops (task 4 capture policy)


@dataclass(frozen=True)
class Stand:
    face: int
    rear: tuple          # (x, y, yaw) of the rear axle
    camera_xy: tuple


def model_yaw(insertion_yaw_rad: float, axis: str) -> float:
    return insertion_yaw_rad if axis == "x" else insertion_yaw_rad - math.pi / 2


def stand(centre_xy, insertion_yaw_rad: float, axis: str, face: int, *, mount_x_m: float,
          rear_axle_offset_m: float) -> Stand:
    my = model_yaw(insertion_yaw_rad, axis)
    normal = np.array([-math.sin(my), math.cos(my)]) * (1 if face > 0 else -1)
    entry = np.array([math.cos(insertion_yaw_rad), math.sin(insertion_yaw_rad)])
    th = math.radians(THETA_E_DEG[axis])
    u = math.cos(th) * normal - math.sin(th) * entry   # centre -> camera, toward the truck's side
    u = u / np.linalg.norm(u)
    c = np.asarray(centre_xy, float)
    cam = c + CAMERA_RANGE_M * u
    rear = cam + (mount_x_m + abs(rear_axle_offset_m)) * u
    yaw = math.atan2(-u[1], -u[0])
    return Stand(face, (float(rear[0]), float(rear[1]), yaw), (float(cam[0]), float(cam[1])))


def ordered_faces(axis: str, centre_xy, insertion_yaw_rad: float, *, rear_xy, mount_x_m: float = 0.619,
                  rear_axle_offset_m: float = 0.0) -> tuple:
    """x: the nearer stand first; y: the back face first, then the entry face (the near leg follows it)."""
    if axis == "y":
        return FACES["y"]
    return tuple(sorted(FACES["x"], key=lambda f: float(np.linalg.norm(np.subtract(stand(
        centre_xy, insertion_yaw_rad, axis, f, mount_x_m=mount_x_m,
        rear_axle_offset_m=rear_axle_offset_m).rear[:2], rear_xy)))))


def retreat_pose(stand_rear, length_m: float = RETREAT_M) -> tuple:
    """A point ``length_m`` behind the stand along its heading: a retake backs off to it (a planned leg, so the
    planner checks the footprint) and arrives at the stand afresh (task 4 capture policy)."""
    x, y, yaw = (float(v) for v in stand_rear)
    return (x - length_m * math.cos(yaw), y - length_m * math.sin(yaw), yaw)


def footprint_corners(rear, footprint, margin_m: float = 0.0) -> np.ndarray:
    """The four corners of the truck footprint at a rear-axle pose, grown by ``margin_m``."""
    x, y, yaw = (float(v) for v in rear)
    h = np.array([math.cos(yaw), math.sin(yaw)])
    n = np.array([-h[1], h[0]])
    fwd, back, half = footprint.front_m + margin_m, footprint.rear_m + margin_m, footprint.half_width_m + margin_m
    c = np.array([x, y])
    return np.array([c + fwd * h + half * n, c + fwd * h - half * n, c - back * h + half * n, c - back * h - half * n])


def side_bounds(zone, hall, poses, footprint, margin_m: float, travel_m: float = TRAVEL_M):
    """The side stage's planning bounds (plan 2026-10-19 task 5, Codex A'): the pickup zone and the footprints of
    every stand and retreat point (grown by the planning margin and a travel allowance), one rectangle, clipped to
    the hall. The zone confines the pickup legs for the search's sake, not as a wall; the side legs need room
    around the pallet."""
    from forklift_core.planning import Bounds  # noqa: PLC0415

    pts = np.vstack([footprint_corners(p, footprint, margin_m) for p in poses])
    lo, hi = pts.min(axis=0) - travel_m, pts.max(axis=0) + travel_m
    return Bounds(max(hall.x_min_m, min(zone.x_min_m, float(lo[0]))), min(hall.x_max_m, max(zone.x_max_m, float(hi[0]))),
                  max(hall.y_min_m, min(zone.y_min_m, float(lo[1]))), min(hall.y_max_m, max(zone.y_max_m, float(hi[1]))))


def inside(bounds, rear, footprint, margin_m: float) -> bool:
    """The footprint grown by ``margin_m`` lies inside ``bounds`` (the next planner's start check)."""
    pts = footprint_corners(rear, footprint, margin_m)
    return bool((pts[:, 0] >= bounds.x_min_m).all() and (pts[:, 0] <= bounds.x_max_m).all()
                and (pts[:, 1] >= bounds.y_min_m).all() and (pts[:, 1] <= bounds.y_max_m).all())
