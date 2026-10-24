"""Start-band release of the drive permission (docs/plans/2026-10-24-start-band-release.md).

A truck that stopped with an occupied cell a little outside its outline -- nearer than the walk's own inflation at the
start (the envelope plus half a sample interval) -- is held at zero in every direction: the first sample interval
already reaches the cell. With the release on, such a *start-band* cell does not block while the truck's outline,
swept continuously along the walk (its corners through every rotation), stays at least as far from it as at the start;
from the first place the swept outline comes nearer, the walk stops there as it would have. Cells farther than the
band -- every new cell met on the way -- keep the walk's full inflation, and nothing else of the permission changes.

Distances are exact between convex shapes: a grid cell (an axis-aligned square) and each part of the footprint (a
rectangle about the rear axle). The sweep is checked on poses ``DENSE_STEP_M`` / ``DENSE_YAW_RAD`` apart; between two
of them the outline moves at most ``dense_error_m(radius)`` (the step plus the farthest corner times the yaw step),
which is the most the continuous sweep can come nearer than the checked poses show.
"""

from __future__ import annotations

from dataclasses import replace
from math import ceil, cos, hypot, sin

import numpy as np

from forklift_core.perception.obstacle_grid import FREE, OCCUPIED

DENSE_STEP_M = 0.002
DENSE_YAW_RAD = 0.001


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


def _points_to_square(px, py, x0, y0, res):
    """Distance of points (k,) to axis-aligned squares [x0, x0 + res] x [y0, y0 + res] (m,), as (m, k)."""
    dx = np.maximum(np.maximum(x0[:, None] - px[None, :], px[None, :] - (x0[:, None] + res)), 0.0)
    dy = np.maximum(np.maximum(y0[:, None] - py[None, :], py[None, :] - (y0[:, None] + res)), 0.0)
    return np.hypot(dx, dy)


def cell_distances(snapshot, cells, pose, shape) -> np.ndarray:
    """Exact distance of each cell's square to the shape (all its parts) at pose; 0 where they meet."""
    cells = np.asarray(cells, dtype=int).reshape(-1, 2)
    if not len(cells):
        return np.zeros(0)
    res = float(snapshot.resolution_m)
    x0 = snapshot.origin_x_m + cells[:, 0] * res
    y0 = snapshot.origin_y_m + cells[:, 1] * res
    sq = np.stack([np.column_stack((x0, y0)), np.column_stack((x0 + res, y0)),
                   np.column_stack((x0 + res, y0 + res)), np.column_stack((x0, y0 + res))], axis=1)  # (m, 4, 2)
    best = np.full(len(cells), np.inf)
    for fp, lat, lon in _parts(shape):
        # convex shapes: the distance is attained at a vertex of one of them
        d_sq = _points_to_rect(sq[..., 0].ravel(), sq[..., 1].ravel(), pose, fp, lat, lon).reshape(-1, 4).min(axis=1)
        corners = _rect_corners(pose, fp, lat, lon)
        d_rc = _points_to_square(corners[:, 0], corners[:, 1], x0, y0, res).min(axis=1)
        inside = _rect_overlaps_squares(pose, fp, lat, lon, x0, y0, res)
        d = np.minimum(d_sq, d_rc)
        d[inside] = 0.0
        best = np.minimum(best, d)
    return best


def _rect_overlaps_squares(pose, fp, lat, lon, x0, y0, res) -> np.ndarray:
    """Whether the part and each square overlap (separating axes of both)."""
    corners = _rect_corners(pose, fp, lat, lon)
    sq = np.stack([np.column_stack((x0, y0)), np.column_stack((x0 + res, y0)),
                   np.column_stack((x0 + res, y0 + res)), np.column_stack((x0, y0 + res))], axis=1)
    x, y, yaw = pose
    axes = [np.array([1.0, 0.0]), np.array([0.0, 1.0]), np.array([cos(yaw), sin(yaw)]), np.array([-sin(yaw), cos(yaw)])]
    out = np.ones(len(x0), dtype=bool)
    for a in axes:
        pr = corners @ a
        ps = sq @ a  # (m, 4)
        out &= ~((ps.max(axis=1) < pr.min() - 1e-12) | (ps.min(axis=1) > pr.max() + 1e-12))
    return out


def footprint_rect_distance(pose, footprint, rect) -> float:
    """Exact distance between a footprint (all its parts) at pose and a rectangle obstacle; 0 where they meet."""
    rx, ry, hl, hw, yaw = rect.x_m, rect.y_m, rect.length_m / 2, rect.width_m / 2, rect.yaw_rad
    c, s = cos(yaw), sin(yaw)
    ru = np.array([hl, hl, -hl, -hl]); rv = np.array([hw, -hw, -hw, hw])
    rc = np.column_stack((rx + ru * c - rv * s, ry + ru * s + rv * c))
    best = np.inf
    for fp, lat, lon in _parts(footprint):
        corners = _rect_corners(pose, fp, lat, lon)
        # rectangle obstacle corners to the part, the part's corners to the obstacle (in its own frame)
        d1 = float(_points_to_rect(rc[:, 0], rc[:, 1], pose, fp, lat, lon).min())
        u = (corners[:, 0] - rx) * c + (corners[:, 1] - ry) * s
        v = -(corners[:, 0] - rx) * s + (corners[:, 1] - ry) * c
        d2 = float(np.hypot(np.maximum(np.abs(u) - hl, 0.0), np.maximum(np.abs(v) - hw, 0.0)).min())
        axes = [np.array([cos(pose[2]), sin(pose[2])]), np.array([-sin(pose[2]), cos(pose[2])]),
                np.array([c, s]), np.array([-s, c])]
        overlap = all(not ((rc @ a_).max() < (corners @ a_).min() - 1e-12 or (rc @ a_).min() > (corners @ a_).max() + 1e-12)
                      for a_ in axes)
        best = min(best, 0.0 if overlap else min(d1, d2))
    return best


def sweep_keeps_away(poses, footprint, rect, *, step_m: float = 0.02) -> bool:
    """Whether the footprint swept along poses (densely, as the walk's) stays at least as far from rect as at the
    first pose -- which must be apart from it (an overlapping start is never released)."""
    poses = np.asarray(poses, dtype=float)
    d0 = footprint_rect_distance(tuple(poses[0]), footprint, rect)
    if d0 <= 0.0:
        return False
    arc = np.r_[0.0, np.cumsum(np.hypot(*np.diff(poses[:, :2], axis=0).T))]
    dense, _ = dense_poses(poses, arc)
    return all(footprint_rect_distance(tuple(p), footprint, rect) >= d0 - 1e-9 for p in dense[1:])


def shape_radius(shape) -> float:
    return max(float(hypot(abs(lon) + max(fp.front_m, fp.rear_m), abs(lat) + fp.half_width_m))
               for fp, lat, lon in _parts(shape))


def dense_error_m(radius: float) -> float:
    """The most the continuous sweep can come nearer to a point than the dense poses show."""
    return 0.5 * DENSE_STEP_M + radius * 0.5 * DENSE_YAW_RAD


def start_band_cells(snapshot, own_pose, shape, band_m: float):
    """(cells (k, 2), their start distances) of the OCCUPIED cells outside the shape at own_pose but within band_m."""
    state = np.asarray(snapshot.state)
    res = float(snapshot.resolution_m)
    reach = shape_radius(shape) + band_m + res
    x, y = float(own_pose[0]), float(own_pose[1])
    i0 = max(0, int(np.floor((x - reach - snapshot.origin_x_m) / res)))
    i1 = min(state.shape[0], int(np.ceil((x + reach - snapshot.origin_x_m) / res)) + 1)
    j0 = max(0, int(np.floor((y - reach - snapshot.origin_y_m) / res)))
    j1 = min(state.shape[1], int(np.ceil((y + reach - snapshot.origin_y_m) / res)) + 1)
    if i1 <= i0 or j1 <= j0:
        return np.zeros((0, 2), dtype=int), np.zeros(0)
    ii, jj = np.nonzero(state[i0:i1, j0:j1] == OCCUPIED)
    cells = np.column_stack((ii + i0, jj + j0))
    if not len(cells):
        return np.zeros((0, 2), dtype=int), np.zeros(0)
    d0 = cell_distances(snapshot, cells, own_pose, shape)
    keep = (d0 > 0.0) & (d0 <= band_m)
    return cells[keep], d0[keep]


def dense_poses(samples, arc):
    """Poses along the walk's samples no more than DENSE_STEP_M / DENSE_YAW_RAD apart, with their arc lengths."""
    samples = np.asarray(samples, dtype=float)
    arc = np.asarray(arc, dtype=float)
    out, s_out = [samples[0]], [arc[0]]
    for a, b, sa, sb in zip(samples[:-1], samples[1:], arc[:-1], arc[1:]):
        dyaw = float(np.arctan2(np.sin(b[2] - a[2]), np.cos(b[2] - a[2])))
        n = max(1, ceil(max(float(np.hypot(*(b[:2] - a[:2]))) / DENSE_STEP_M, abs(dyaw) / DENSE_YAW_RAD)))
        f = np.arange(1, n + 1) / n
        out.extend(np.column_stack((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * dyaw)))
        s_out.extend(sa + f * (sb - sa))
    return np.asarray(out), np.asarray(s_out)


def release_lengths(snapshot, cells, d0, samples, arc, shape, *, upto_m: float | None = None) -> np.ndarray:
    """Per start-band cell, the arc length of the first dense pose whose outline is nearer to it than at the start
    (inf: never within the walk, or within ``upto_m``)."""
    out = np.full(len(cells), np.inf)
    if not len(cells):
        return out
    poses, s = dense_poses(samples, arc)
    pending = np.ones(len(cells), dtype=bool)
    for pose, s_k in zip(poses, s):
        if upto_m is not None and s_k > upto_m + 1e-9:
            break
        idx = np.nonzero(pending)[0]
        if not len(idx):
            break
        d = cell_distances(snapshot, cells[idx], pose, shape)
        near = d < d0[idx] - 1e-9
        out[idx[near]] = s_k
        pending[idx[near]] = False
    return out


def released(snapshot, cells):
    """The snapshot with the start-band cells read as FREE (seen at the snapshot's own stamp, so they never make the
    evidence older than it is)."""
    if not len(cells):
        return snapshot
    state = np.array(snapshot.state, copy=True)
    stamp = np.array(snapshot.free_stamp, copy=True)
    state[cells[:, 0], cells[:, 1]] = FREE
    stamp[cells[:, 0], cells[:, 1]] = snapshot.stamp_s
    return replace(snapshot, state=state, free_stamp=stamp)


__all__ = ["DENSE_STEP_M", "DENSE_YAW_RAD", "cell_distances", "dense_error_m", "dense_poses", "footprint_rect_distance",
           "release_lengths", "released", "shape_radius", "start_band_cells", "sweep_keeps_away"]
