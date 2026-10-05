"""Drive permission from the obstacle grid (priority-5 plan D4).

The global plan may cross cells nobody has seen; the truck may not. Two
checks bound the speed, and the smaller wins:

* the emergency-stop arc (every control tick): an emergency stop zeroes the
  wheels and holds the steering, so the truck stops along the arc of its
  current curvature, not along the plan (Codex L0 P1). Every cell the
  footprint newly covers along that arc -- inflated by the measured stopping
  envelope, which grows from zero at the present pose -- must be FREE, and
  the speed must stop inside the verified part of the arc;
* the path ahead (each grid snapshot): how far along the active path the
  same holds, so the truck slows before a blocked stretch instead of only at
  the last moment.

Cells the truck's own body covers now are excluded (its body hides them);
nothing around the body is (Codex L0 P1: an envelope-grown exclusion let an
obstacle cell in). FREE evidence expires per cell: the oldest FREE cell a
check relied on must be no older than evidence_max_age_s at the tick, which
the grid sized its error radius for. No snapshot, an expired one, or any
obstacle sensor silent for sensor_timeout_s gives 0.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, cos, floor, sin, sqrt

import numpy as np

from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, GridSnapshot

RETAINED = 3  # shadow-band memory (control/shadow_memory.py)
from forklift_core.planning.geometry import Bounds, Footprint, FootprintCollisionChecker


@dataclass(frozen=True)
class StoppingModel:
    latency_s: float
    decel_mps2: float
    margin_m: float

    def distance_m(self, speed_mps: float) -> float:
        v = abs(speed_mps)
        return v * self.latency_s + v * v / (2 * self.decel_mps2) + self.margin_m

    def speed_for(self, distance_m: float) -> float:
        """Largest speed whose stopping distance fits in distance_m (0 if none)."""
        d = distance_m - self.margin_m
        if d <= 0:
            return 0.0
        a, t = self.decel_mps2, self.latency_s
        return -a * t + sqrt((a * t) ** 2 + 2 * a * d)


@dataclass(frozen=True)
class PermissionConfig:
    stopping: StoppingModel
    envelope_offset_m: float
    evidence_max_age_s: float
    envelope_ramp_m: float = 0.2  # the envelope reaches its full width this far along the stop
    sensor_timeout_s: float = 0.25
    step_m: float = 0.02  # sample spacing; the sweep pad between samples is at most about this
    lookahead_m: float = 3.0
    end_creep_mps: float = 0.02
    shadow_band_m: float = 0.0  # RETAINED passes only inside the outline grown by this (D4 delta)


@dataclass
class Check:
    verified_m: float
    blocked: str | None  # "occupied" / "unknown" / "edge" / None
    oldest_free_s: float  # oldest FREE evidence the verified part relied on (inf: none needed)
    reached_end: bool
    blocked_cells: tuple = ()  # diagnostics: up to 20 (i, j) grid cells that blocked


def parts_of(shape) -> list:
    """A checked shape as (Footprint, lateral, longitudinal offset) parts: a
    Footprint is one part at the rear axle; a sequence of (Footprint, lateral_m,
    longitudinal_m) is the truck's union -- e.g. the body and the two fork
    blades, whose gap is not the truck (D4)."""
    if isinstance(shape, Footprint):
        return [(shape, 0.0, 0.0)]
    return [(fp, float(lat), float(lon)) for fp, lat, lon in shape]


def in_shape(px, py, pose, shape) -> np.ndarray:
    """Whether world points lie inside any part of a shape at pose (closed)."""
    x, y, yaw = pose
    c, s = cos(yaw), sin(yaw)
    u = (np.asarray(px) - x) * c + (np.asarray(py) - y) * s
    v = -(np.asarray(px) - x) * s + (np.asarray(py) - y) * c
    out = np.zeros(np.shape(u), dtype=bool)
    for fp, lat, lon in parts_of(shape):
        w = u - lon
        out |= (w >= -fp.rear_m) & (w <= fp.front_m) & (np.abs(v - lat) <= fp.half_width_m)
    return out


def shape_meets(rect, shape, pose, margin_m: float = 0.0) -> bool:
    """Whether any part of a shape at pose, inflated by margin_m, overlaps a
    Rectangle (geometry only, no hall bounds) -- the truth side of shape_cells."""
    x, y, yaw = pose
    c, s = cos(yaw), sin(yaw)
    open_hall = Bounds(-1e9, 1e9, -1e9, 1e9)
    for fp, lat, lon in parts_of(shape):
        part_pose = (x + lon * c - lat * s, y + lon * s + lat * c, yaw)
        if not FootprintCollisionChecker([rect], fp, open_hall).free(part_pose, margin_m):
            return True
    return False


def shape_cells(snapshot: GridSnapshot, pose, shape, margin_m: float, *, direction: int = 0):
    """footprint_cells over every part of a shape (union, each cell once)."""
    found, outside = [], False
    for fp, lat, lon in parts_of(shape):
        cells, out = footprint_cells(snapshot, pose, fp, margin_m, direction=direction, lateral_m=lat,
                                     longitudinal_m=lon)
        found.append(cells)
        outside |= out
    cells = np.unique(np.concatenate(found), axis=0) if found else np.zeros((0, 2), dtype=int)
    return cells, outside


def footprint_cells(snapshot: GridSnapshot, pose, footprint: Footprint, margin_m: float, *, direction: int = 0,
                    lateral_m: float = 0.0, longitudinal_m: float = 0.0):
    """(cells (k, 2), any outside the grid) whose square the inflated footprint at pose overlaps.

    direction +1 (forward) leaves the rear edge uninflated, -1 the front edge:
    no point of the body moves against the direction of travel while the
    curvature stays below 1 / half width (the corner's along-track speed is
    v (1 - kappa w) > 0), so the margin is never needed there.
    """
    x, y, yaw = pose
    c, s = cos(yaw), sin(yaw)
    front = footprint.front_m + (margin_m if direction >= 0 else 0.0)
    rear = footprint.rear_m + (margin_m if direction <= 0 else 0.0)
    offset = (front - rear) / 2 + longitudinal_m
    cx, cy = x + offset * c - lateral_m * s, y + offset * s + lateral_m * c
    hl = (front + rear) / 2
    hw = footprint.half_width_m + margin_m
    ex = abs(c) * hl + abs(s) * hw
    ey = abs(s) * hl + abs(c) * hw
    res = snapshot.resolution_m
    nx, ny = snapshot.state.shape
    i0 = int(floor((cx - ex - snapshot.origin_x_m) / res)) - 1
    i1 = int(ceil((cx + ex - snapshot.origin_x_m) / res)) + 1
    j0 = int(floor((cy - ey - snapshot.origin_y_m) / res)) - 1
    j1 = int(ceil((cy + ey - snapshot.origin_y_m) / res)) + 1
    ii, jj = np.meshgrid(np.arange(i0, i1), np.arange(j0, j1), indexing="ij")
    ii, jj = ii.ravel(), jj.ravel()
    sx = snapshot.origin_x_m + (ii + 0.5) * res
    sy = snapshot.origin_y_m + (jj + 0.5) * res
    half = res / 2
    dx, dy = sx - cx, sy - cy
    proj = half * (abs(c) + abs(s))
    hit = (
        (np.abs(dx) <= ex + half)
        & (np.abs(dy) <= ey + half)
        & (np.abs(dx * c + dy * s) <= hl + proj)
        & (np.abs(-dx * s + dy * c) <= hw + proj)
    )
    out = np.column_stack((ii[hit], jj[hit]))
    outside = (out[:, 0] < 0) | (out[:, 0] >= nx) | (out[:, 1] < 0) | (out[:, 1] >= ny)
    return out, bool(outside.any())


def resample_path(poses: np.ndarray, step_m: float, limit_m: float):
    """Poses every step_m along a polyline up to limit_m, their arc length, and the full length."""
    xy = poses[:, :2]
    seg = np.hypot(*np.diff(xy, axis=0).T) if len(poses) > 1 else np.zeros(0)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    total = min(float(s[-1]), limit_m)
    targets = np.arange(0.0, total + 1e-9, step_m)
    if not len(targets) or targets[-1] < total - 1e-9:
        targets = np.append(targets, total)
    x = np.interp(targets, s, xy[:, 0])
    y = np.interp(targets, s, xy[:, 1])
    yaw = np.interp(targets, s, np.unwrap(poses[:, 2]))
    return np.column_stack((x, y, yaw)), targets, float(s[-1])


def arc_poses(pose, curvature_inv_m: float, direction: int, length_m: float, step_m: float):
    """Rear-axle poses along the arc of fixed curvature (forward or reverse), with arc length."""
    x0, y0, yaw0 = pose
    s = np.arange(0.0, length_m + 1e-9, step_m)
    if not len(s) or s[-1] < length_m - 1e-9:
        s = np.append(s, length_m)
    d = direction * s
    k = curvature_inv_m
    if abs(k) < 1e-9:
        xs = x0 + d * cos(yaw0)
        ys = y0 + d * sin(yaw0)
        yaws = np.full(len(s), float(yaw0))
    else:
        yaws = yaw0 + d * k
        xs = x0 + (np.sin(yaws) - sin(yaw0)) / k
        ys = y0 + (cos(yaw0) - np.cos(yaws)) / k
    return np.column_stack((xs, ys, yaws)), s


class DrivePermission:
    def __init__(self, config: PermissionConfig):
        self.config = config
        self.snapshot: GridSnapshot | None = None
        self.path_check: Check | None = None
        self._driven_since_m = 0.0

    @staticmethod
    def _enters(snapshot, cell, own_pose, own_footprint, pose, footprint, margin, direction) -> bool:
        """Whether the part of a cell outside the truck's present outline lies in
        the inflated footprint at pose (5 mm sub-samples)."""
        res = snapshot.resolution_m
        x0 = snapshot.origin_x_m + cell[0] * res
        y0 = snapshot.origin_y_m + cell[1] * res
        g = (np.arange(11) + 0.0) / 10 * res
        px, py = np.meshgrid(x0 + g, y0 + g, indexing="ij")
        px, py = px.ravel(), py.ravel()
        outside = ~in_shape(px, py, own_pose, own_footprint)
        if not outside.any():
            return False
        x, y, yaw = pose
        c, s_ = cos(yaw), sin(yaw)
        u = (px[outside] - x) * c + (py[outside] - y) * s_
        v = -(px[outside] - x) * s_ + (py[outside] - y) * c
        for fp, lat, lon in parts_of(footprint):
            front = fp.front_m + (margin if direction >= 0 else 0.0)
            rear = fp.rear_m + (margin if direction <= 0 else 0.0)
            w = u - lon
            if ((w >= -rear) & (w <= front) & (np.abs(v - lat) <= fp.half_width_m + margin)).any():
                return True
        return False

    def _walk(self, snapshot, samples, arc, footprint, own_cells, *, full_path_m=None, direction=0,
              own_pose=None, own_footprint=None) -> Check:
        """Walk the samples; between two samples the midpoint is checked with half
        the interval's motion added (translation + farthest corner x rotation), so
        the continuous sweep is covered, not just the samples (Codex L0b P1)."""
        cfg = self.config
        verified, blocked, oldest = 0.0, None, np.inf
        blocked_cells = ()
        radius = max(float(np.hypot(abs(lon) + max(fp.front_m, fp.rear_m), abs(lat) + fp.half_width_m))
                     for fp, lat, lon in parts_of(footprint))
        band = None  # cells near the present outline where RETAINED may pass, built on demand
        own_lookup = None  # (whole mask, partial index grid, outside sub-samples), built on demand
        checks = [(samples[0], float(arc[0]), 0.0)]
        for i in range(1, len(samples)):
            a, b = samples[i - 1], samples[i]
            dyaw = float(np.arctan2(np.sin(b[2] - a[2]), np.cos(b[2] - a[2])))
            motion = float(np.hypot(b[0] - a[0], b[1] - a[1])) + radius * abs(dyaw)
            mid = np.array([(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, a[2] + dyaw / 2])
            checks.append((mid, float(arc[i]), motion / 2))
        for pose, s, pad in checks:
            ramp = min(1.0, s / cfg.envelope_ramp_m) if cfg.envelope_ramp_m > 0 else 1.0
            margin = cfg.envelope_offset_m * ramp + pad
            cells, outside = shape_cells(snapshot, pose, footprint, margin, direction=direction)
            if outside:
                blocked = "edge"
                break
            whole, partial = own_cells
            if whole or partial:
                if own_lookup is None:
                    own_lookup = self._own_lookup(snapshot, own_cells, own_pose, own_footprint)
                whole_mask, partial_id, part_pts = own_lookup
                keep = ~whole_mask[cells[:, 0], cells[:, 1]]
                ids = partial_id[cells[:, 0], cells[:, 1]]
                touched = keep & (ids >= 0)
                if touched.any() and own_pose is not None:
                    # Partly under the truck now: checked only if this sample's
                    # footprint reaches into its part outside the present
                    # outline (Codex checkpoint P1) -- all such cells at once.
                    keep[touched] = self._parts_reach(part_pts, ids[touched], pose, footprint, margin, direction)
                cells = cells[keep]
            states = snapshot.state[cells[:, 0], cells[:, 1]]
            if (states == OCCUPIED).any():
                blocked = "occupied"
                blocked_cells = tuple(map(tuple, cells[states == OCCUPIED][:20].tolist()))
                break
            # Only cells wholly under the truck are exempt; every other cell the
            # stop sweeps must be FREE (Codex checkpoint P1: exempting UNKNOWN in
            # a band around the body let the stop enter unseen space).
            # RETAINED (shadow-band memory, D4 delta 2026-10-05) passes; its
            # validity was judged at the snapshot, and it never counts as
            # fresh evidence.
            retained = states == RETAINED
            if retained.any():
                # Only inside the band around where the truck is now: the truck
                # moves between snapshots and the memory was granted for the
                # band alone (Codex P1).
                if band is None:
                    band = np.zeros(snapshot.state.shape, dtype=bool)
                    if own_pose is not None and own_footprint is not None and cfg.shadow_band_m > 0:
                        near, _ = shape_cells(snapshot, own_pose, own_footprint, cfg.shadow_band_m)
                        nx_, ny_ = band.shape
                        near = near[(near[:, 0] >= 0) & (near[:, 0] < nx_) & (near[:, 1] >= 0) & (near[:, 1] < ny_)]
                        band[near[:, 0], near[:, 1]] = True
                outside = retained & ~band[cells[:, 0], cells[:, 1]]
                if outside.any():
                    states = states.copy()
                    states[outside] = 0
            bad = (states != FREE) & (states != RETAINED)
            if bad.any():
                blocked = "unknown"
                blocked_cells = tuple(map(tuple, cells[bad][:20].tolist()))
                break
            # A RETAINED cell's stamp is the oldest fresh evidence its support
            # used, so it ages out under the same limit.
            seen = (states == FREE) | (states == RETAINED)
            if seen.any():
                oldest = min(oldest, float(np.min(snapshot.free_stamp[cells[seen, 0], cells[seen, 1]])))
            verified = float(s)
        reached_end = blocked is None and full_path_m is not None and arc[-1] >= full_path_m - 1e-9
        return Check(verified, blocked, oldest, reached_end, blocked_cells)

    @staticmethod
    def _own_lookup(snapshot, own_cells, own_pose, own_footprint):
        """Grid lookups for one walk: wholly-own mask, partial-cell index grid,
        and each partial cell's 5 mm sub-samples outside the present outline
        (an array per cell of points, in world coordinates)."""
        whole, partial = own_cells
        shape = snapshot.state.shape
        whole_mask = np.zeros(shape, dtype=bool)
        if whole:
            w = np.array(list(whole), dtype=np.int64)
            w = w[(w[:, 0] >= 0) & (w[:, 0] < shape[0]) & (w[:, 1] >= 0) & (w[:, 1] < shape[1])]
            whole_mask[w[:, 0], w[:, 1]] = True
        partial_id = np.full(shape, -1, dtype=np.int64)
        pts = []
        if partial and own_pose is not None:
            pa = np.array(list(partial), dtype=np.int64)
            pa = pa[(pa[:, 0] >= 0) & (pa[:, 0] < shape[0]) & (pa[:, 1] >= 0) & (pa[:, 1] < shape[1])]
            partial_id[pa[:, 0], pa[:, 1]] = np.arange(len(pa))
            res = snapshot.resolution_m
            g = np.arange(11) / 10 * res
            gx, gy = np.meshgrid(g, g, indexing="ij")
            px = snapshot.origin_x_m + pa[:, 0, None] * res + gx.ravel()[None, :]
            py = snapshot.origin_y_m + pa[:, 1, None] * res + gy.ravel()[None, :]
            out = ~in_shape(px, py, own_pose, own_footprint)
            pts = [np.column_stack((px[k][out[k]], py[k][out[k]])) for k in range(len(pa))]
        return whole_mask, partial_id, pts

    @staticmethod
    def _parts_reach(part_pts, ids, pose, footprint, margin, direction) -> np.ndarray:
        """For each partial cell id, whether any of its outside sub-samples lies
        in the inflated shape at pose (the vectorised _enters)."""
        sizes = np.array([len(part_pts[i]) for i in ids])
        if not sizes.sum():
            return np.zeros(len(ids), dtype=bool)
        allp = np.concatenate([part_pts[i] for i in ids])
        owner = np.repeat(np.arange(len(ids)), sizes)
        x, y, yaw = pose
        c, s_ = cos(yaw), sin(yaw)
        u = (allp[:, 0] - x) * c + (allp[:, 1] - y) * s_
        v = -(allp[:, 0] - x) * s_ + (allp[:, 1] - y) * c
        hit = np.zeros(len(allp), dtype=bool)
        for fp, lat, lon in parts_of(footprint):
            front = fp.front_m + (margin if direction >= 0 else 0.0)
            rear = fp.rear_m + (margin if direction <= 0 else 0.0)
            w = u - lon
            hit |= (w >= -rear) & (w <= front) & (np.abs(v - lat) <= fp.half_width_m + margin)
        return np.bincount(owner[hit], minlength=len(ids)) > 0

    @staticmethod
    def _own_cells(snapshot, pose, own_footprint, band_m: float = 0.0):
        """(wholly inside, partly covered) cells of the truck's own outline.

        A wholly covered cell is exempt. A partly covered cell is mostly under the
        body, where no beam reaches, so UNKNOWN is accepted there -- but an
        OCCUPIED mark still blocks (Codex L0b P1: an obstacle in the part the body
        does not cover). The partly covered set is reported but no longer
        exempt from anything (Codex checkpoint P1): a blind strip against the
        body is the sensors' problem, not the check's.
        """
        cells, _ = shape_cells(snapshot, pose, own_footprint, band_m)
        if not len(cells):
            return set(), set()
        res = snapshot.resolution_m
        x0 = snapshot.origin_x_m + cells[:, 0] * res
        y0 = snapshot.origin_y_m + cells[:, 1] * res
        # Wholly inside means inside one part: the union of body and blades is
        # not convex, so four corners in it do not put the square in it (Codex
        # checkpoint P1: a bar between a blade and the body).
        inside = np.zeros(len(cells), dtype=bool)
        for part in parts_of(own_footprint):
            in_part = np.ones(len(cells), dtype=bool)
            for dx, dy in ((0, 0), (res, 0), (0, res), (res, res)):
                in_part &= in_shape(x0 + dx, y0 + dy, pose, [part])
            inside |= in_part
        out = {(int(a), int(b)) for a, b in cells[inside]}
        partial = {(int(a), int(b)) for a, b in cells[~inside]}
        return out, partial

    def update(self, snapshot: GridSnapshot, path_ahead, footprint: Footprint, own_footprint: Footprint, *,
               current_pose, direction: int = 0) -> Check:
        """New snapshot: the verified distance along path_ahead (rear-axle poses from the truck on)."""
        poses = np.asarray(path_ahead, dtype=float)
        if poses.ndim != 2 or poses.shape[1] != 3 or not len(poses):
            raise ValueError("path_ahead must be a non-empty (N, 3) array")
        samples, arc, total = resample_path(poses, self.config.step_m, self.config.lookahead_m)
        own = self._own_cells(snapshot, current_pose, own_footprint)
        self.snapshot = snapshot
        self.path_check = self._walk(snapshot, samples, arc, footprint, own, full_path_m=total,
                                     direction=direction, own_pose=current_pose, own_footprint=own_footprint)
        self._driven_since_m = 0.0
        return self.path_check

    def advance(self, distance_m: float) -> None:
        """Path length driven since the last snapshot (both directions count)."""
        self._driven_since_m += abs(distance_m)

    def allowed_speed(
        self,
        now_s: float,
        sensors_last_s: dict,
        *,
        current_pose,
        curvature_inv_m: float,
        direction: int,
        footprint: Footprint,
        own_footprint: Footprint,
        speed_cap_mps: float,
    ) -> tuple[float, str]:
        """(speed limit, reason) for this control tick."""
        cfg = self.config
        snap, path = self.snapshot, self.path_check
        if snap is None or path is None:
            return 0.0, "no_snapshot"
        for name, last in sensors_last_s.items():
            if now_s - last > cfg.sensor_timeout_s:
                return 0.0, f"sensor_silent:{name}"
        # Emergency-stop arc at the present curvature and direction.
        length = cfg.stopping.distance_m(speed_cap_mps) + cfg.step_m
        samples, arc = arc_poses(current_pose, curvature_inv_m, 1 if direction >= 0 else -1, length, cfg.step_m)
        own = self._own_cells(snap, current_pose, own_footprint)
        estop = self._walk(snap, samples, arc, footprint, own, direction=1 if direction >= 0 else -1,
                           own_pose=current_pose, own_footprint=own_footprint)
        self.last_estop = estop
        path_left = path.verified_m - self._driven_since_m
        if now_s - estop.oldest_free_s > cfg.evidence_max_age_s or now_s - path.oldest_free_s > cfg.evidence_max_age_s:
            return 0.0, "evidence_stale"
        arc_limit = cfg.stopping.speed_for(estop.verified_m)
        if path.reached_end:
            # Stopping at the goal needs no margin past it; the arc check
            # still covers the actual stopping volume beyond the end.
            path_limit = max(
                cfg.stopping.speed_for(path_left + cfg.stopping.margin_m),
                cfg.end_creep_mps if path_left > 0 else 0.0,
            )
        else:
            path_limit = cfg.stopping.speed_for(path_left)
        if arc_limit <= path_limit:
            return arc_limit, estop.blocked or ("ok" if arc_limit > 0 else "limit")
        return path_limit, path.blocked or ("ok" if path_limit > 0 else "limit")


__all__ = ["Check", "DrivePermission", "PermissionConfig", "StoppingModel", "arc_poses", "footprint_cells",
           "in_shape", "parts_of", "shape_cells", "shape_meets"]
