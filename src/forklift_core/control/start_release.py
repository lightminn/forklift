"""Start-band release of the drive permission (docs/plans/2026-10-24-start-band-release.md).

A truck that stopped with an occupied cell a little outside its outline -- nearer than the walk's own inflation at the
start -- is held at zero in every direction: the first sample interval already reaches the cell. With the release on,
the permission latches an *anchor* at the first tick the truck stands (``DrivePermission.note_standing``); the occupied
cells within the band of the anchor's outline are *start-band cells*. Such a cell does not block while the truck's
outline, swept continuously along the walk (its corners through every rotation), never comes nearer to it than it has
been since the anchor (non-decreasing, part by part); from the last pose before the first that does, the walk is cut
there. A part that was farther than the band at the anchor keeps the band plus the stopping margin. Cells outside the
band -- every new cell met on the way -- keep the walk's full inflation. The anchor goes with a correction change, with a
new standing, and once the truck is farther than the band from every start-band cell.

Distances are exact between convex shapes (a grid cell square, each footprint part's rectangle). The sweep is checked on
poses ``DENSE_STEP_M`` / ``DENSE_YAW_RAD`` apart; between two of them the outline can come nearer than the checked poses
show by at most ``dense_error_m`` (half a step, the farthest corner times half a yaw step, and the sagitta of a walk
interval), so a cell is released only from ``MIN_D0_FACTOR`` times that distance.
"""

from __future__ import annotations

from dataclasses import replace
from math import ceil, cos, hypot, sin

import numpy as np

from forklift_core.perception.obstacle_grid import FREE, OCCUPIED

DENSE_STEP_M = 0.002
DENSE_YAW_RAD = 0.001
SAGITTA_M = 1e-4  # a 0.02 m walk interval on a curvature up to 2 m^-1: 0.02^2 * 2 / 8
MIN_D0_FACTOR = 2.0


def _parts(shape):
    from forklift_core.control.drive_permission import parts_of

    return parts_of(shape)


def _rect_corners(pose, fp, lat, lon):
    """The four corners (4, 2) of one footprint part at pose."""
    x, y, yaw = pose
    c, s = cos(yaw), sin(yaw)
    us = np.array([lon + fp.front_m, lon + fp.front_m, lon - fp.rear_m, lon - fp.rear_m])
    vs = np.array([lat + fp.half_width_m, lat - fp.half_width_m, lat - fp.half_width_m, lat + fp.half_width_m])
    return np.column_stack((x + us * c - vs * s, y + us * s + vs * c))


def _points_to_rect(px, py, pose, fp, lat, lon):
    """Distance of points to one footprint part (0 inside)."""
    x, y, yaw = pose
    c, s = cos(yaw), sin(yaw)
    u = (px - x) * c + (py - y) * s - lon
    v = -(px - x) * s + (py - y) * c - lat
    du = np.maximum(np.maximum(u - fp.front_m, -fp.rear_m - u), 0.0)
    dv = np.maximum(np.abs(v) - fp.half_width_m, 0.0)
    return np.hypot(du, dv)


def _square_vertices(snapshot, cells):
    res = float(snapshot.resolution_m)
    x0 = snapshot.origin_x_m + cells[:, 0] * res
    y0 = snapshot.origin_y_m + cells[:, 1] * res
    return x0, y0, res


def part_distances(snapshot, cells, poses, shape) -> np.ndarray:
    """Exact distances (n poses, m cells, P parts) between each cell's square and each footprint part at each pose; 0
    where they meet. Convex shapes: the distance is attained at a vertex of one of them (overlap tested apart)."""
    cells = np.asarray(cells, dtype=int).reshape(-1, 2)
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    parts = _parts(shape)
    out = np.zeros((len(poses), len(cells), len(parts)))
    if not len(cells) or not len(poses):
        return out
    x0, y0, res = _square_vertices(snapshot, cells)
    sqx = np.stack([x0, x0 + res, x0 + res, x0], axis=1)  # (m, 4)
    sqy = np.stack([y0, y0, y0 + res, y0 + res], axis=1)
    px, py, yaw = poses[:, 0][:, None, None], poses[:, 1][:, None, None], poses[:, 2][:, None, None]
    c, s = np.cos(yaw), np.sin(yaw)
    for k, (fp, lat, lon) in enumerate(parts):
        # the square's vertices in the part's frame -> distance to the part
        u = (sqx[None] - px) * c + (sqy[None] - py) * s - lon  # (n, m, 4)
        v = -(sqx[None] - px) * s + (sqy[None] - py) * c - lat
        du = np.maximum(np.maximum(u - fp.front_m, -fp.rear_m - u), 0.0)
        dv = np.maximum(np.abs(v) - fp.half_width_m, 0.0)
        d1 = np.hypot(du, dv).min(axis=2)  # (n, m)
        # the part's corners -> distance to the square
        cu = np.array([lon + fp.front_m, lon + fp.front_m, lon - fp.rear_m, lon - fp.rear_m])
        cv = np.array([lat + fp.half_width_m, lat - fp.half_width_m, lat - fp.half_width_m, lat + fp.half_width_m])
        cx = poses[:, 0][:, None] + cu[None] * np.cos(poses[:, 2])[:, None] - cv[None] * np.sin(poses[:, 2])[:, None]
        cy = poses[:, 1][:, None] + cu[None] * np.sin(poses[:, 2])[:, None] + cv[None] * np.cos(poses[:, 2])[:, None]
        dx = np.maximum(np.maximum(x0[None, :, None] - cx[:, None, :], cx[:, None, :] - (x0 + res)[None, :, None]), 0.0)
        dy = np.maximum(np.maximum(y0[None, :, None] - cy[:, None, :], cy[:, None, :] - (y0 + res)[None, :, None]), 0.0)
        d2 = np.hypot(dx, dy).min(axis=2)  # (n, m)
        # overlap: separating axes of the square (world x, y) and of the part
        cxmin, cxmax, cymin, cymax = cx.min(axis=1), cx.max(axis=1), cy.min(axis=1), cy.max(axis=1)
        sep = ((cxmax[:, None] < x0[None]) | (cxmin[:, None] > (x0 + res)[None])
               | (cymax[:, None] < y0[None]) | (cymin[:, None] > (y0 + res)[None]))
        sep |= (u.max(axis=2) < -fp.rear_m) | (u.min(axis=2) > fp.front_m)
        sep |= (v.max(axis=2) < -fp.half_width_m) | (v.min(axis=2) > fp.half_width_m)
        out[:, :, k] = np.where(sep, np.minimum(d1, d2), 0.0)
    return out


def cell_distances(snapshot, cells, pose, shape) -> np.ndarray:
    """Exact distance of each cell's square to the shape (its nearest part) at pose; 0 where they meet."""
    return part_distances(snapshot, cells, [pose], shape)[0].min(axis=1) if len(cells) else np.zeros(0)


def shape_radius(shape) -> float:
    return max(float(hypot(abs(lon) + max(fp.front_m, fp.rear_m), abs(lat) + fp.half_width_m))
               for fp, lat, lon in _parts(shape))


def dense_error_m(radius: float) -> float:
    """The most the continuous sweep can come nearer to a point than the dense poses show."""
    return 0.5 * DENSE_STEP_M + radius * 0.5 * DENSE_YAW_RAD + SAGITTA_M


def min_d0_m(shape) -> float:
    """Below this a start distance is not released (the continuous sweep could touch between checked poses)."""
    return MIN_D0_FACTOR * dense_error_m(shape_radius(shape))


def start_band_cells(snapshot, anchor_pose, shape, band_m: float):
    """(cells (k, 2), per-part start distances (k, P)) of the OCCUPIED cells outside the shape at the anchor whose
    nearest part is within band_m but no nearer than ``min_d0_m``."""
    state = np.asarray(snapshot.state)
    res = float(snapshot.resolution_m)
    reach = shape_radius(shape) + band_m + res
    x, y = float(anchor_pose[0]), float(anchor_pose[1])
    i0 = max(0, int(np.floor((x - reach - snapshot.origin_x_m) / res)))
    i1 = min(state.shape[0], int(np.ceil((x + reach - snapshot.origin_x_m) / res)) + 1)
    j0 = max(0, int(np.floor((y - reach - snapshot.origin_y_m) / res)))
    j1 = min(state.shape[1], int(np.ceil((y + reach - snapshot.origin_y_m) / res)) + 1)
    n_parts = len(_parts(shape))
    empty = (np.zeros((0, 2), dtype=int), np.zeros((0, n_parts)))
    if i1 <= i0 or j1 <= j0:
        return empty
    ii, jj = np.nonzero(state[i0:i1, j0:j1] == OCCUPIED)
    cells = np.column_stack((ii + i0, jj + j0))
    if not len(cells):
        return empty
    d0 = part_distances(snapshot, cells, [anchor_pose], shape)[0]
    nearest = d0.min(axis=1)
    keep = (nearest >= min_d0_m(shape)) & (nearest <= band_m)
    return cells[keep], d0[keep]


def history_keys(snapshot, cells, shape) -> list:
    """Each start-band cell's history key: the checked shape's parts and the cell's world index (the grid origin over
    the resolution, plus the cell's index), so a rolling grid's origin moving keeps a cell's history."""
    res = float(snapshot.resolution_m)
    oi, oj = int(round(snapshot.origin_x_m / res)), int(round(snapshot.origin_y_m / res))
    sig = tuple((float(fp.front_m), float(fp.rear_m), float(fp.half_width_m), float(lat), float(lon))
                for fp, lat, lon in _parts(shape))
    return [(sig, round(res, 6), oi + int(i), oj + int(j)) for i, j in np.asarray(cells, dtype=int).reshape(-1, 2)]


def dense_poses(samples, arc):
    """Poses along the walk's samples no more than DENSE_STEP_M / DENSE_YAW_RAD apart, with their arc lengths."""
    samples = np.asarray(samples, dtype=float)
    arc = np.asarray(arc, dtype=float)
    out, s_out = [samples[:1]], [arc[:1]]
    for a, b, sa, sb in zip(samples[:-1], samples[1:], arc[:-1], arc[1:]):
        dyaw = float(np.arctan2(np.sin(b[2] - a[2]), np.cos(b[2] - a[2])))
        n = max(1, ceil(max(float(np.hypot(*(b[:2] - a[:2]))) / DENSE_STEP_M, abs(dyaw) / DENSE_YAW_RAD)))
        f = np.arange(1, n + 1) / n
        out.append(np.column_stack((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * dyaw)))
        s_out.append(sa + f * (sb - sa))
    return np.concatenate(out), np.concatenate(s_out)


def release_length(snapshot, cells, floor, cap, samples, arc, shape, *, budget_m: float = 0.0,
                   floor_min_m: float = 0.0) -> tuple[float, np.ndarray]:
    """(the arc length of the last safe dense pose, the cells that end it) of a start-band release along the walk.

    ``floor`` (m, P): each part's best distance to each cell since the anchor; ``cap`` (m, P): the most a part is held
    to (the band for a part that started within it, the band plus the stopping margin for one that started farther).
    Along the dense sweep no part may come nearer than the largest distance it has reached -- since the anchor and
    along the sweep so far -- capped at its ``cap``. inf: the whole walk keeps away.

    ``budget_m`` (escape presteer C4'', the lead's (a')): a part may come that much nearer than that held distance -- a
    fixed allowance against the largest distance reached, so moving away and back never earns more -- but never nearer
    than ``floor_min_m`` (the release's own min_d0, the continuous sweep's error between dense poses). 0: as before."""
    if not len(cells):
        return np.inf, np.zeros((0, 2), dtype=int)
    poses, s = dense_poses(samples, arc)
    d = part_distances(snapshot, cells, poses, shape)  # (n, m, P)
    run = np.maximum.accumulate(np.concatenate([np.asarray(floor, dtype=float)[None], d], axis=0), axis=0)[1:]
    thr = np.minimum(run, np.asarray(cap, dtype=float)[None])
    if budget_m > 0.0:
        thr = np.maximum(thr - float(budget_m), float(floor_min_m))
    bad = (d < thr - 1e-9).any(axis=2)  # (n, m)
    hit = np.nonzero(bad.any(axis=1))[0]
    if not len(hit):
        return np.inf, np.zeros((0, 2), dtype=int)
    k = int(hit[0])
    return float(s[max(0, k - 1)]), cells[bad[k]]


def released(snapshot, cells):
    """The snapshot with the start-band cells read as FREE but never as evidence (stamp +inf: the walk's oldest-evidence
    minimum ignores them; the outline never enters them, so no free space is claimed there)."""
    if not len(cells):
        return snapshot
    state = np.array(snapshot.state, copy=True)
    stamp = np.array(snapshot.free_stamp, copy=True)
    state[cells[:, 0], cells[:, 1]] = FREE
    stamp[cells[:, 0], cells[:, 1]] = np.inf
    return replace(snapshot, state=state, free_stamp=stamp)


# ---- the escape's rectangles (planning side) ----

def footprint_rect_distance(pose, footprint, rect) -> float:
    """Exact distance between a footprint (all its parts) at pose and a rectangle obstacle; 0 where they meet."""
    rx, ry, hl, hw, yaw = rect.x_m, rect.y_m, rect.length_m / 2, rect.width_m / 2, rect.yaw_rad
    c, s = cos(yaw), sin(yaw)
    ru = np.array([hl, hl, -hl, -hl])
    rv = np.array([hw, -hw, -hw, hw])
    rc = np.column_stack((rx + ru * c - rv * s, ry + ru * s + rv * c))
    best = np.inf
    for fp, lat, lon in _parts(footprint):
        corners = _rect_corners(pose, fp, lat, lon)
        d1 = float(_points_to_rect(rc[:, 0], rc[:, 1], pose, fp, lat, lon).min())
        u = (corners[:, 0] - rx) * c + (corners[:, 1] - ry) * s
        v = -(corners[:, 0] - rx) * s + (corners[:, 1] - ry) * c
        d2 = float(np.hypot(np.maximum(np.abs(u) - hl, 0.0), np.maximum(np.abs(v) - hw, 0.0)).min())
        axes = [np.array([cos(pose[2]), sin(pose[2])]), np.array([-sin(pose[2]), cos(pose[2])]),
                np.array([c, s]), np.array([-s, c])]
        overlap = all(not ((rc @ a_).max() < (corners @ a_).min() - 1e-12
                           or (rc @ a_).min() > (corners @ a_).max() + 1e-12) for a_ in axes)
        best = min(best, 0.0 if overlap else min(d1, d2))
    return best


def sweep_keeps_away(poses, footprint, rect) -> bool:
    """Whether the footprint swept along poses (densely) never comes nearer to rect than it has been -- the distance
    non-decreasing from the first pose, which must be apart from it by at least ``min_d0_m``."""
    poses = np.asarray(poses, dtype=float)
    d0 = footprint_rect_distance(tuple(poses[0]), footprint, rect)
    if d0 < min_d0_m(footprint):
        return False
    arc = np.r_[0.0, np.cumsum(np.hypot(*np.diff(poses[:, :2], axis=0).T))]
    dense, _ = dense_poses(poses, arc)
    best = d0
    for p in dense[1:]:
        d = footprint_rect_distance(tuple(p), footprint, rect)
        if d < best - 1e-9:
            return False
        best = max(best, d)
    return True


__all__ = ["DENSE_STEP_M", "DENSE_YAW_RAD", "cell_distances", "dense_error_m", "dense_poses", "footprint_rect_distance",
           "history_keys", "min_d0_m", "part_distances", "release_length", "released", "shape_radius", "start_band_cells",
           "sweep_keeps_away"]
