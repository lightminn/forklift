"""Exit-prefix exemption (docs/plans/2026-10-28-exit-prefix-extract-rules.md D1, D2).

A loaded truck just pulled out of a narrow bay stands inside the painted halo of the posts or walls beside it: the
grid marks a tall prop's hits grown by the odometry age, the start of the transport's exit prefix (a straight reverse)
is held at zero though the faces are 0.16-0.22 m away. While the mission's one-shot latch is open, the raw watch checks
every scan against the *region* -- the loaded outline along the remaining exempt path and along the stop arcs at the
measured and commanded curvature -- and the scan's non-own hits; a scan with none inside the region plus the margin
*verifies* that region, kept as map-frame poses (the grid's origin follows the truck, so cell indices would not
stay put). The one snapshot transform then finds the cells the verified region overlaps on the snapshot at hand and
reads only the OCCUPIED ones FREE, stamped with the verifying scan, so every reader of the snapshot sees the same cells and
the freed ones age like any other free evidence. Unknown cells, cells outside the corridor, and a corridor that is
missing, of another correction or older than the evidence age are left alone.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from forklift_core.perception.obstacle_grid import FREE, OCCUPIED


@dataclass(frozen=True)
class VerifiedCorridor:
    """A region a scan verified: no non-own hit of that scan lay inside it. ``poses`` in the map frame the grid
    marks that scan in (its correction, version ``correction_version``)."""

    stamp_s: float
    correction_version: int
    poses: np.ndarray  # (n, 3) rear-axle poses


def region_poses(path_ahead, current_pose, *, measured_kappa: float, commanded_kappa: float, direction: int,
                 stop_m: float, scan_m: float, step_m: float = 0.02) -> np.ndarray:
    """The rear-axle poses the watch sweeps the outline over: ``path_ahead`` (the remaining exempt path and its stop
    length, already cut by the caller) and, from ``current_pose``, the stop arcs at the measured and the commanded
    curvature, each the stop length plus one scan's travel long."""
    from forklift_core.control.drive_permission import arc_poses

    out = [np.asarray(path_ahead, dtype=float).reshape(-1, 3)]
    for kappa in (measured_kappa, commanded_kappa):
        out.append(arc_poses(tuple(float(v) for v in current_pose), float(kappa), int(direction) or -1,
                             float(stop_m) + float(scan_m), step_m)[0])
    return np.vstack(out)


def path_window(poses, start_m: float, end_m: float, step_m: float = 0.02) -> np.ndarray:
    """The polyline ``poses`` between arc lengths ``start_m`` and ``end_m`` (clamped to it), every ``step_m``."""
    p = np.asarray(poses, dtype=float).reshape(-1, 3)
    if len(p) < 2:
        return p.copy()
    s = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(p[:, :2], axis=0).T))))
    a, b = max(0.0, float(start_m)), min(float(end_m), float(s[-1]))
    if b <= a:
        return np.array([[np.interp(a, s, p[:, 0]), np.interp(a, s, p[:, 1]), np.interp(a, s, np.unwrap(p[:, 2]))]])
    q = np.arange(a, b + 1e-9, step_m)
    if q[-1] < b - 1e-9:
        q = np.append(q, b)
    return np.column_stack((np.interp(q, s, p[:, 0]), np.interp(q, s, p[:, 1]), np.interp(q, s, np.unwrap(p[:, 2]))))


def relative_to(pose, poses) -> np.ndarray:
    """``poses`` in the frame of ``pose`` (x forward, y left)."""
    x0, y0, a0 = (float(v) for v in pose)
    p = np.asarray(poses, dtype=float).reshape(-1, 3)
    c, s = math.cos(a0), math.sin(a0)
    dx, dy = p[:, 0] - x0, p[:, 1] - y0
    return np.column_stack((dx * c + dy * s, -dx * s + dy * c, p[:, 2] - a0))


def watch_hits(points_xy, own, poses, footprint, margin_m: float) -> np.ndarray:
    """Indices of the non-own points inside the footprint, grown by ``margin_m``, at any of ``poses``."""
    pts = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    keep = ~np.asarray(own, dtype=bool).reshape(-1)
    if not len(pts) or not keep.any():
        return np.zeros(0, dtype=int)
    idx = np.flatnonzero(keep)
    p = np.asarray(poses, dtype=float).reshape(-1, 3)
    dx = pts[idx, 0][None, :] - p[:, 0][:, None]
    dy = pts[idx, 1][None, :] - p[:, 1][:, None]
    c, s = np.cos(p[:, 2])[:, None], np.sin(p[:, 2])[:, None]
    x, y = dx * c + dy * s, -dx * s + dy * c
    m = float(margin_m)
    inside = (x >= -footprint.rear_m - m) & (x <= footprint.front_m + m) & (np.abs(y) <= footprint.half_width_m + m)
    return idx[inside.any(axis=0)]


def corridor_cells(snapshot, poses, footprint, margin_m: float) -> np.ndarray:
    """Every grid cell the footprint grown by ``margin_m`` overlaps at any of ``poses`` (both ends grown), each once."""
    from forklift_core.control.drive_permission import footprint_cells

    found = [footprint_cells(snapshot, tuple(float(v) for v in pose), footprint, margin_m, direction=0)[0]
             for pose in np.asarray(poses, dtype=float).reshape(-1, 3)]
    if not found:
        return np.zeros((0, 2), dtype=np.int64)
    return np.unique(np.vstack(found).astype(np.int64), axis=0)


def apply_verified(snapshot, verified: VerifiedCorridor | None, footprint, margin_m: float, *, now_s: float,
                   max_age_s: float):
    """(snapshot, cells freed, why skipped): the OCCUPIED cells the verified region (the footprint grown by
    ``margin_m`` at its poses) overlaps on this snapshot read FREE with its stamp. Skipped -- the snapshot returned as
    it is -- without a verification ("unverified"), for another correction than the snapshot's ("version"), with a
    stamp or a now that is not finite or a stamp after now ("invalid"), or one older than ``max_age_s`` ("stale")."""
    if verified is None:
        return snapshot, 0, "unverified"
    if int(verified.correction_version) != int(snapshot.correction_version):
        return snapshot, 0, "version"
    stamp_s, now = float(verified.stamp_s), float(now_s)
    if not (math.isfinite(stamp_s) and math.isfinite(now)) or now - stamp_s < 0.0:
        return snapshot, 0, "invalid"  # never written as evidence: NaN or a future stamp would pass the age test
    if now - stamp_s > float(max_age_s):
        return snapshot, 0, "stale"
    cells = corridor_cells(snapshot, verified.poses, footprint, margin_m)
    nx, ny = snapshot.state.shape
    cells = cells[(cells[:, 0] >= 0) & (cells[:, 0] < nx) & (cells[:, 1] >= 0) & (cells[:, 1] < ny)]
    cells = cells[snapshot.state[cells[:, 0], cells[:, 1]] == OCCUPIED] if len(cells) else cells
    if not len(cells):
        return snapshot, 0, None
    state = snapshot.state.copy()
    stamp = snapshot.free_stamp.copy()
    state[cells[:, 0], cells[:, 1]] = FREE
    stamp[cells[:, 0], cells[:, 1]] = float(verified.stamp_s)
    return replace(snapshot, state=state, free_stamp=stamp), int(len(cells)), None


__all__ = ["VerifiedCorridor", "apply_verified", "corridor_cells", "path_window", "region_poses", "relative_to",
           "watch_hits"]
