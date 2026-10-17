"""Known pallets for the drive permission (docs/plans/2026-10-04-lidar-obstacle-map.md, D5 v10).

The brief's single 2D LiDAR scans at 1.05 m and never sees an EPAL 6 pallet
(0.144 m) -- not the pickup pallet, not the one being carried, not the one
just delivered. Those are the operating assumption's only allowed low
objects, so they reach the permission as known rectangles instead of scan
hits: the pickup zone before recognition (unknown), the recognised pallet
and the delivered pallet (occupied, grown by how far the truck's relative
pose may have drifted since the last fix).

Radius r_k(a) = r_fix + e(a) + 2 rho sin(psi(a) / 2): the D2 point radius with
the odometry age table, rho from the rear axle to the farthest pallet corner,
r_fix the relative error at the fix. Past the free-evidence cap the pallet's
surroundings become unknown; past the table an empirical sample may stand in
but never shrinks the radius.

During a certified withdrawal the forks are inside the pallet; the cells of
the permission checker's own corridor are left out (``exclude``) so the
pallet does not hold the truck it sits on, while every other cell keeps it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN, AgeErrorTable, compose


@dataclass(frozen=True)
class KnownRect:
    """A pallet rectangle (x, y, length along yaw, width, yaw) in the control frame of its fix.

    ``frame_correction`` is the SLAM correction applied when it was fixed: the
    rectangle is kept in odometry and carried with the grid's correction.
    ``kind`` "zone" is a prior pickup area (unknown until recognition).
    """

    rect: tuple[float, float, float, float, float]
    fix_stamp_s: float
    r_fix_m: float
    rho_m: float | None  # None: from the current rear axle to the farthest corner, each refresh
    frame_correction: tuple[float, float, float]
    kind: str = "pallet"


def invert_pose(pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (-(c * x + s * y), s * x - c * y, -yaw)


def known_radius(age_s: float, *, rho_m: float, r_fix_m: float, table: AgeErrorTable, cap_m: float = 0.20,
                 empirical_m: float | None = None, empirical_max_age_s: float | None = None):
    """(radius, OCCUPIED or UNKNOWN) of a known pallet whose relative fix is age_s old."""
    bound = table.at(age_s)
    if bound is not None:
        p, y = bound
        r = r_fix_m + p + 2.0 * rho_m * math.sin(y / 2.0)
        return r, OCCUPIED if r <= cap_m else UNKNOWN
    p, y = table.position_m[-1], table.yaw_rad[-1]
    r = r_fix_m + p + 2.0 * rho_m * math.sin(y / 2.0)
    if empirical_m is not None and empirical_max_age_s is not None and age_s <= empirical_max_age_s:
        r = max(r, float(empirical_m))  # a sample never shrinks the bound (it is no new observation)
        return r, OCCUPIED if r <= cap_m else UNKNOWN
    return r, UNKNOWN


def _cells_near_rect(snapshot, rect, radius_m: float) -> np.ndarray:
    """Cells whose square may meet the rectangle grown by radius (conservative)."""
    cx, cy, length, width, yaw = rect
    nx, ny = snapshot.state.shape
    res = snapshot.resolution_m
    xs = snapshot.origin_x_m + (np.arange(nx) + 0.5) * res - cx
    ys = snapshot.origin_y_m + (np.arange(ny) + 0.5) * res - cy
    c, s = math.cos(yaw), math.sin(yaw)
    u = np.abs(xs[:, None] * c + ys[None, :] * s) - length / 2
    v = np.abs(-xs[:, None] * s + ys[None, :] * c) - width / 2
    outside = np.hypot(np.maximum(u, 0.0), np.maximum(v, 0.0))
    return outside <= radius_m + res * math.sqrt(2) / 2


def _farthest_corner_m(rect, pose) -> float:
    cx, cy, length, width, yaw = rect
    c, s = math.cos(yaw), math.sin(yaw)
    return max(
        math.hypot(cx + c * u - s * v - pose[0], cy + s * u + c * v - pose[1])
        for u in (-length / 2, length / 2) for v in (-width / 2, width / 2)
    )


def apply_known(snapshot, known, now_s: float, *, exclude: np.ndarray | None = None, table: AgeErrorTable | None = None,
                cap_m: float = 0.20, empirical_m: float | None = None, empirical_max_age_s: float | None = None,
                current_pose=None, horizon_s: float = 0.0):
    """The snapshot with known pallets marked: OCCUPIED stays, anything else turns UNKNOWN/OCCUPIED.

    horizon_s: the radius is taken at the age the snapshot may still be used at
    (the permission reuses it between refreshes), as the FREE grid does
    (Codex stage-1 P1-4). UNKNOWN also overrides the shadow memory's RETAINED
    cells -- a remembered band cell is no evidence against a pallet the plane
    cannot see (Codex stage-1 P1-3).
    """
    if not known:
        return snapshot
    state = snapshot.state.copy()
    stamp = snapshot.free_stamp.copy()
    for k in known:
        odom = compose(invert_pose(k.frame_correction), k.rect[:2] + (k.rect[4],))
        x, y, yaw = compose(snapshot.correction, odom)
        rect = (x, y, k.rect[2], k.rect[3], yaw)
        if k.kind == "zone":
            radius, mark = k.r_fix_m, UNKNOWN
        else:
            if table is None:
                raise ValueError("a known pallet needs the odometry age table")
            rho = k.rho_m
            if rho is None:
                if current_pose is None:
                    raise ValueError("rho from the current pose needs current_pose")
                rho = _farthest_corner_m(rect, current_pose)
            radius, mark = known_radius(now_s + horizon_s - k.fix_stamp_s, rho_m=rho, r_fix_m=k.r_fix_m, table=table,
                                        cap_m=cap_m, empirical_m=empirical_m, empirical_max_age_s=empirical_max_age_s)
        hit = _cells_near_rect(snapshot, rect, radius)
        if exclude is not None:
            hit &= ~exclude
        if mark == OCCUPIED:
            state[hit] = OCCUPIED
        else:
            hit &= state != OCCUPIED  # FREE and RETAINED alike
            state[hit] = UNKNOWN
        stamp[hit] = np.nan
    return replace(snapshot, state=state, free_stamp=stamp)


def corridor_mask(snapshot, path, shape, config, *, direction: int, until_m: float, tracking_m: float) -> np.ndarray:
    """Cells the permission checker itself examines along the path up to until_m, plus tracking.

    The same sampling (config.step_m) and margin (envelope + half a sample step)
    as the checker, so no cell it checks lies outside the corridor (Codex v10 5th).
    """
    from forklift_core.control.drive_permission import resample_path, shape_cells

    poses, _, _ = resample_path(np.asarray(path, dtype=float), config.step_m, until_m)
    margin = config.envelope_offset_m + config.step_m / 2 + tracking_m
    mask = np.zeros(snapshot.state.shape, dtype=bool)
    nx, ny = mask.shape
    for pose in poses:
        cells, _ = shape_cells(snapshot, tuple(pose), shape, margin, direction=direction)
        keep = (cells[:, 0] >= 0) & (cells[:, 0] < nx) & (cells[:, 1] >= 0) & (cells[:, 1] < ny)
        mask[cells[keep, 0], cells[keep, 1]] = True
    return mask


def fork_pocket_gaps(lateral_m: float, yaw_rad: float, *, blade_centre_m: float, blade_half_width_m: float,
                     blade_length_m: float, pocket_inner_m: float, pocket_outer_m: float) -> tuple[float, ...]:
    """The four blade-to-pocket-wall gaps (left inner, left outer, right inner, right outer).

    lateral_m / yaw_rad: the pallet axis relative to the truck's (pallet centre
    to the left of the truck centreline is positive). The worst point along the
    inserted blade length is taken for the yaw.
    """
    swing = abs(blade_length_m * math.sin(yaw_rad))
    left_in = (blade_centre_m - blade_half_width_m) - (lateral_m + pocket_inner_m)
    left_out = (lateral_m + pocket_outer_m) - (blade_centre_m + blade_half_width_m)
    right_in = (lateral_m - pocket_inner_m) - (-blade_centre_m + blade_half_width_m)
    right_out = (-blade_centre_m - blade_half_width_m) - (lateral_m - pocket_outer_m)
    return tuple(g - swing for g in (left_in, left_out, right_in, right_out))


def withdraw_certified(gaps_m, *, b_w: float | None, e_w: float | None, side_m: float) -> bool:
    """Withdrawal corridor certificate: the smallest of the four blade-to-pocket gaps covers
    the checker's side width, the relative error at release and the drift until the stop."""
    if b_w is None or e_w is None or not gaps_m:
        return False
    return min(gaps_m) >= side_m + b_w + e_w


__all__ = ["KnownRect", "apply_known", "corridor_mask", "fork_pocket_gaps", "invert_pose", "known_radius", "withdraw_certified"]
