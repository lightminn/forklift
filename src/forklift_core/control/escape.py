"""The straight escape an obstacle recovery looks for before replanning at zero clearance.

Plan docs/plans/2026-10-04-lidar-obstacle-map.md, "장애물 복구–허가 정합 설계 4판" (E2): when a
stop leaves the truck inside the planning clearance, the replan at zero clearance gave paths the
drive permission blocked again (smoke 2382). A short straight that every check passes and that
ends with room for a normal replan is driven first. The checks themselves (the planner's checker,
the permission, the grid rebuilt for the end pose) are passed in, so this module holds only the
candidates, the qualification and the choice.
"""

from __future__ import annotations

import itertools
import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np

LENGTHS_M = (0.3, 0.6, 1.0, 1.5)
STEP_M = 0.02
END_POSITION_TOL_M = 0.01  # along and across; the escape tracker's own end tolerance
END_YAW_TOL_RAD = 0.02
EXTRA_MARGIN_M = (
    0.05  # added to the permission's margin, growing from 0 over the escape
)


def straight(
    pose, direction: int, length_m: float, step_m: float = STEP_M
) -> np.ndarray:
    """Rear-axle poses from pose along its heading (direction +1 forward, -1 back), start included."""
    x, y, yaw = (float(v) for v in pose)
    n = max(1, int(math.ceil(length_m / step_m)))
    s = np.linspace(0.0, length_m, n + 1) * (1 if direction >= 0 else -1)
    return np.column_stack(
        (x + s * math.cos(yaw), y + s * math.sin(yaw), np.full(n + 1, yaw))
    )


def end_region(
    end,
    position_tol_m: float = END_POSITION_TOL_M,
    yaw_tol_rad: float = END_YAW_TOL_RAD,
) -> list:
    """The 27 poses the escape tracker may stop at: along, across and heading each at -tol, 0, +tol."""
    x, y, yaw = (float(v) for v in end)
    c, s = math.cos(yaw), math.sin(yaw)
    steps = (-position_tol_m, 0.0, position_tol_m)
    return [
        (x + a * c - b * s, y + a * s + b * c, yaw + w)
        for a, b, w in itertools.product(steps, steps, (-yaw_tol_rad, 0.0, yaw_tol_rad))
    ]


def end_threshold_m(
    planner_clearance_m: float, psi_rad: float, length_m: float, cell_m: float
) -> float:
    """The clearance an escape's end region must keep: the planning clearance, plus how much a mark
    swells when the truck backs length_m away from it (2 sin(psi/2) per metre of range at the
    oldest mark's age), plus one cell."""
    return planner_clearance_m + 2.0 * math.sin(psi_rad / 2.0) * length_m + cell_m


@dataclass(frozen=True)
class Candidate:
    direction: int
    length_m: float
    sweep_free: bool  # ① the leg's planner checker over the sweep
    path: str | None  # ② the permission's path check to the length plus the stop length
    path_ramped: (
        str | None
    )  # ② with the extra margin growing 0 -> EXTRA_MARGIN_M over the length
    arcs: tuple  # ③ (curvature, blocked) of each start arc at the held steering
    end_free: (
        bool | None
    )  # ④ every end-region pose free at clearance 0 (None: not reached)
    end_clearance_m: float | None  # ④ the region's smallest clearance
    threshold_m: float

    @property
    def qualifies(self) -> bool:
        return (
            self.sweep_free
            and self.path is None
            and self.path_ramped is None
            and all(blocked is None for _, blocked in self.arcs)
            and bool(self.end_free)
            and self.end_clearance_m is not None
            and self.end_clearance_m >= self.threshold_m
        )

    def record(self) -> dict:
        return {
            "direction": self.direction,
            "length_m": self.length_m,
            "sweep_free": self.sweep_free,
            "path": self.path,
            "path_ramped": self.path_ramped,
            "arcs": [[k, b] for k, b in self.arcs],
            "end_free": self.end_free,
            "end_clearance_m": self.end_clearance_m,
            "threshold_m": self.threshold_m,
            "qualifies": self.qualifies,
        }


def evaluate(
    pose,
    *,
    sweep_free: Callable[[np.ndarray, int], bool],
    path_blocked: Callable[[np.ndarray, int, float, float | None], str | None],
    arc_blocked: Callable[[float, int], str | None],
    end_clearance: Callable[[Sequence], tuple[bool, float]],
    kappa_now: float,
    stop_length_m: float,
    planner_clearance_m: float,
    psi_rad: float,
    cell_m: float,
    lengths: Sequence[float] = LENGTHS_M,
) -> list[Candidate]:
    """Every candidate (both directions, each length) with each check's result. The end region is
    only measured for a candidate that passed ①-③ (it rebuilds the grid for the end pose)."""
    out = []
    for direction in (1, -1):
        arcs = tuple(
            (float(k), arc_blocked(float(k), direction))
            for k in (kappa_now, kappa_now / 2.0, 0.0)
        )
        for length in lengths:
            sweep = straight(pose, direction, length)
            ahead = straight(pose, direction, length + stop_length_m)
            free = bool(sweep_free(sweep, direction))
            path = path_blocked(ahead, direction, 0.0, None)
            ramped = path_blocked(ahead, direction, EXTRA_MARGIN_M, length)
            threshold = end_threshold_m(planner_clearance_m, psi_rad, length, cell_m)
            end_free, clearance = None, None
            if (
                free
                and path is None
                and ramped is None
                and all(b is None for _, b in arcs)
            ):
                end_free, clearance = end_clearance(end_region(sweep[-1]))
                end_free, clearance = bool(end_free), float(clearance)
            out.append(
                Candidate(
                    direction,
                    float(length),
                    free,
                    path,
                    ramped,
                    arcs,
                    end_free,
                    clearance,
                    threshold,
                )
            )
    return out


def grid_sweep_blocked(occupancy, footprint, pose, direction: int, poses) -> str | None:
    """① on a planner grid: the sweep of footprint along poses at zero clearance, under the drive
    permission's own-cell rule (cells wholly under the outline at pose are skipped, cells it straddles
    are checked only where the sweep reaches their outside part) -- so a start already on a cell is
    not refused for it. "edge" when the sweep leaves the grid."""
    from forklift_core.control.drive_permission import (
        DrivePermission,
        PermissionConfig,
        StoppingModel,
    )
    from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, GridSnapshot

    occupied = np.asarray(occupancy.occupied, dtype=bool)
    snapshot = GridSnapshot(
        np.where(occupied, OCCUPIED, FREE).astype(np.uint8),
        np.zeros(occupied.shape),
        0.0,
        (0.0, 0.0, 0.0),
        0,
        1,
        {},
        float(occupancy.origin_x_m),
        float(occupancy.origin_y_m),
        float(occupancy.resolution_m),
    )
    length = float(
        np.sum(np.hypot(*np.diff(np.asarray(poses, dtype=float)[:, :2], axis=0).T))
    )
    walk = DrivePermission(
        PermissionConfig(
            StoppingModel(0.0, 1.0, 0.0),
            envelope_offset_m=0.0,
            evidence_max_age_s=math.inf,
            envelope_ramp_m=0.0,
            step_m=STEP_M,
            lookahead_m=length + 1.0,
        )
    )
    return walk.check_path(
        snapshot, poses, footprint, footprint, current_pose=pose, direction=direction
    ).blocked


def rectangles_sweep_free(rect_checker, poses, released=(), footprint=None) -> bool:
    """① on the leg's rectangles and bounds: every sample free with half a step of sweep pad (the
    search's own pad for a straight), the start included. ``released`` (start-band release,
    docs/plans/2026-10-24-start-band-release.md): rectangles the start already comes within that pad of, kept out
    of ``rect_checker`` -- each passes only while ``footprint`` swept along poses stays as far from it as at the
    start (never one the start overlaps)."""
    if not all(rect_checker.free(tuple(float(v) for v in p), STEP_M / 2.0) for p in poses):
        return False
    if not released:
        return True
    from forklift_core.control.start_release import sweep_keeps_away

    return all(sweep_keeps_away(poses, footprint, r) for r in released)


def start_band_rectangles(rects, footprint, pose):
    """(near, far): the rectangles the start pose comes within the sweep pad of (but does not overlap), and the rest
    -- the start-band release's split of a trigger's extra obstacles."""
    from forklift_core.control.start_release import footprint_rect_distance

    near, far = [], []
    for r in rects:
        d = footprint_rect_distance(tuple(float(v) for v in pose), footprint, r)
        (near if 0.0 < d <= STEP_M / 2.0 else far).append(r)
    return near, far


def clearance_m(
    free_at: Callable[[tuple, float], bool],
    pose,
    cap_m: float = 0.5,
    tol_m: float = 1e-3,
) -> float:
    """The largest margin (to cap_m, within tol_m) at which free_at(pose, margin) holds; -1 if the
    pose is not free at zero."""
    pose = tuple(float(v) for v in pose)
    if not free_at(pose, 0.0):
        return -1.0
    if free_at(pose, cap_m):
        return cap_m
    lo, hi = 0.0, cap_m
    while hi - lo > tol_m:
        mid = (lo + hi) / 2.0
        lo, hi = (mid, hi) if free_at(pose, mid) else (lo, mid)
    return lo


def choose(candidates: Sequence[Candidate], blocked_direction: int) -> Candidate | None:
    """The shortest qualifying candidate; of equal length, the one against the blocked leg."""
    ok = [c for c in candidates if c.qualifies]
    if not ok:
        return None
    return min(ok, key=lambda c: (c.length_m, c.direction == blocked_direction))


__all__ = [
    "END_POSITION_TOL_M",
    "END_YAW_TOL_RAD",
    "EXTRA_MARGIN_M",
    "LENGTHS_M",
    "STEP_M",
    "Candidate",
    "choose",
    "clearance_m",
    "end_region",
    "end_threshold_m",
    "evaluate",
    "grid_sweep_blocked",
    "rectangles_sweep_free",
    "straight",
]
