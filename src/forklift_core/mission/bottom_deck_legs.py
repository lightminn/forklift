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
RECOVERIES_PER_FACE = 3  # path-replacing recoveries on one face before it is given up
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
