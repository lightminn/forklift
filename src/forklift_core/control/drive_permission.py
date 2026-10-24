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

import math
from dataclasses import dataclass
from math import ceil, cos, floor, sin, sqrt

import numpy as np

from forklift_core import kernels
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, GridSnapshot
from forklift_core.perf import bucket, dual, dumped, shape_kind

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
    # Start-band release (docs/plans/2026-10-24-start-band-release.md): an occupied cell outside the outline at the
    # start but within this distance of it does not block while the swept outline stays as far from it; 0: off.
    start_band_m: float = 0.0


@dataclass
class Check:
    verified_m: float
    blocked: str | None  # "occupied" / "unknown" / "edge" / None
    oldest_free_s: float  # oldest FREE evidence the verified part relied on (inf: none needed)
    reached_end: bool
    blocked_cells: tuple = ()  # diagnostics: up to 20 (i, j) grid cells that blocked
    first_unknown_m: float | None = None  # unknown_passes walks: where the first unknown/edge cell was


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
    if not found:
        return np.zeros((0, 2), dtype=int), outside
    if len(found) == 1:
        return found[0], outside  # one part: already unique, in (i, j) order
    # The union, in the same (i, j) order as a row-wise unique, keyed in one
    # integer (the 2-D unique was a tenth of a mission's wall time).
    cells = np.concatenate(found).astype(np.int64)
    off, span = 1 << 20, 1 << 21
    if np.abs(cells).max() >= off:  # outside the key's range: the row-wise unique (Codex checkpoint 10 P2)
        return np.unique(cells, axis=0), outside
    keys = np.unique((cells[:, 0] + off) * span + (cells[:, 1] + off))
    return np.column_stack((keys // span - off, keys % span - off)), outside


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


def _shape_cells_batch(snapshot: GridSnapshot, poses, parts, margins, direction: int):
    """shape_cells for several poses at once (real-time plan S1): the cells of each pose in shape_cells' order
    (one part: footprint_cells' order; several: the sorted union), each pose's own margin, all found in one
    vectorised pass by footprint_cells' own expressions. Returns (cells (n, 2), owner pose index (n,),
    outside per pose) with the rows grouped by pose, or (None, None, None) when a cell leaves the union's key
    range (shape_cells then sorts row-wise)."""
    res = snapshot.resolution_m
    ox, oy = snapshot.origin_x_m, snapshot.origin_y_m
    half = res / 2
    rows = []  # per (pose, part): pose index, i0, ni, j0, nj, c, s, cx, cy, ex + half, ey + half, hl + p, hw + p
    for k, (pose, margin) in enumerate(zip(poses, margins, strict=True)):
        x, y, yaw = pose
        c, s = cos(yaw), sin(yaw)
        for fp, lateral_m, longitudinal_m in parts:
            front = fp.front_m + (margin if direction >= 0 else 0.0)
            rear = fp.rear_m + (margin if direction <= 0 else 0.0)
            offset = (front - rear) / 2 + longitudinal_m
            cx, cy = x + offset * c - lateral_m * s, y + offset * s + lateral_m * c
            hl = (front + rear) / 2
            hw = fp.half_width_m + margin
            ex = abs(c) * hl + abs(s) * hw
            ey = abs(s) * hl + abs(c) * hw
            i0 = int(floor((cx - ex - ox) / res)) - 1
            i1 = int(ceil((cx + ex - ox) / res)) + 1
            j0 = int(floor((cy - ey - oy) / res)) - 1
            j1 = int(ceil((cy + ey - oy) / res)) + 1
            proj = half * (abs(c) + abs(s))
            rows.append((k, i0, i1 - i0, j0, j1 - j0, c, s, cx, cy, ex + half, ey + half, hl + proj, hw + proj))
    k_, i0_, ni_, j0_, nj_ = (np.array([r[n] for r in rows], dtype=np.int64) for n in range(5))
    c_, s_, cx_, cy_, exh_, eyh_, hlp_, hwp_ = (np.array([r[n] for r in rows], dtype=float) for n in range(5, 13))
    size = ni_ * nj_
    entry = np.repeat(np.arange(len(rows)), size)
    local = np.arange(int(size.sum())) - np.repeat(np.cumsum(size) - size, size)
    nj_e = nj_[entry]
    ii = i0_[entry] + local // nj_e
    jj = j0_[entry] + local % nj_e
    sx = ox + (ii + 0.5) * res
    sy = oy + (jj + 0.5) * res
    dx, dy = sx - cx_[entry], sy - cy_[entry]
    c_e, s_e = c_[entry], s_[entry]
    hit = (
        (np.abs(dx) <= exh_[entry])
        & (np.abs(dy) <= eyh_[entry])
        & (np.abs(dx * c_e + dy * s_e) <= hlp_[entry])
        & (np.abs(-dx * s_e + dy * c_e) <= hwp_[entry])
    )
    ii, jj, owner = ii[hit], jj[hit], k_[entry[hit]]
    nx, ny = snapshot.state.shape
    out = (ii < 0) | (ii >= nx) | (jj < 0) | (jj >= ny)
    outside = np.bincount(owner[out], minlength=len(poses)) > 0
    if len(parts) > 1 and len(ii):
        off, span = 1 << 20, 1 << 21
        if max(int(np.abs(ii).max()), int(np.abs(jj).max())) >= off:
            return None, None, None
        keys = np.unique(owner * (span * span) + (ii + off) * span + (jj + off))
        owner = keys // (span * span)
        rest = keys % (span * span)
        ii, jj = rest // span - off, rest % span - off
    return np.column_stack((ii, jj)), owner, outside


def _own_lookup_flat(snapshot, own_cells, own_pose, own_footprint):
    """DrivePermission._own_lookup with the partial cells' outside sub-samples in one flat array: (whole mask,
    partial index grid, (points (m, 2), start per partial cell, count per partial cell)), the same points in the
    same order."""
    whole, partial = own_cells
    shape = snapshot.state.shape
    whole_mask = np.zeros(shape, dtype=bool)
    if whole:
        w = np.array(list(whole), dtype=np.int64)
        w = w[(w[:, 0] >= 0) & (w[:, 0] < shape[0]) & (w[:, 1] >= 0) & (w[:, 1] < shape[1])]
        whole_mask[w[:, 0], w[:, 1]] = True
    partial_id = np.full(shape, -1, dtype=np.int64)
    points, start, count = np.zeros((0, 2)), np.zeros(0, dtype=np.int64), np.zeros(0, dtype=np.int64)
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
        points = np.column_stack((px[out], py[out]))
        count = out.sum(axis=1).astype(np.int64)
        start = np.cumsum(count) - count
    return whole_mask, partial_id, (points, start, count)


def _parts_reach_batch(flat, ids, owner, poses, parts, margins, direction) -> np.ndarray:
    """_parts_reach for cells of several poses (each cell its own pose and margin), by the same expressions."""
    points, start, count = flat
    sizes = count[ids]
    if not sizes.sum():
        return np.zeros(len(ids), dtype=bool)
    cell = np.repeat(np.arange(len(ids)), sizes)
    allp = points[np.repeat(start[ids], sizes) + (np.arange(int(sizes.sum())) - np.repeat(np.cumsum(sizes) - sizes,
                                                                                            sizes))]
    pose_of = owner[cell]
    xs = np.array([float(p[0]) for p in poses])[pose_of]
    ys = np.array([float(p[1]) for p in poses])[pose_of]
    cs = np.array([cos(p[2]) for p in poses])[pose_of]
    ss = np.array([sin(p[2]) for p in poses])[pose_of]
    u = (allp[:, 0] - xs) * cs + (allp[:, 1] - ys) * ss
    v = -(allp[:, 0] - xs) * ss + (allp[:, 1] - ys) * cs
    hit = np.zeros(len(allp), dtype=bool)
    for fp, lat, lon in parts:
        front = np.array([fp.front_m + (m if direction >= 0 else 0.0) for m in margins])[pose_of]
        rear = np.array([fp.rear_m + (m if direction <= 0 else 0.0) for m in margins])[pose_of]
        width = np.array([fp.half_width_m + m for m in margins])[pose_of]
        w = u - lon
        hit |= (w >= -rear) & (w <= front) & (np.abs(v - lat) <= width)
    return np.bincount(cell[hit], minlength=len(ids)) > 0


class DrivePermission:
    def __init__(self, config: PermissionConfig):
        self.config = config
        self.snapshot: GridSnapshot | None = None
        self.path_check: Check | None = None
        self._driven_since_m = 0.0
        # Start-band release (docs/plans/2026-10-24-start-band-release.md): the pose latched at the first standing
        # tick, its correction version and each start-band cell's best (largest) part distances since.
        self.release_anchor: dict | None = None
        self._standing = False

    def note_standing(self, standing: bool, pose, correction_version: int) -> None:
        """The truck stands (or moves) at pose: the first standing tick latches the start-band release's anchor."""
        if self.config.start_band_m <= 0.0:
            return
        if standing and (not self._standing or self.release_anchor is None):
            # a new standing, or still standing after the anchor was dropped (a correction re-projected the grid
            # under it -- the scene replan's release raises the version before its escape is checked, Codex v2 P2-3)
            self.release_anchor = {"pose": tuple(float(v) for v in pose), "version": int(correction_version),
                                   "best": {}}
        self._standing = bool(standing)

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
              own_pose=None, own_footprint=None, extra_margin_m=0.0, extra_ramp_m=None,
              unknown_passes=False) -> Check:
        """Walk the samples; between two samples the midpoint is checked with half
        the interval's motion added (translation + farthest corner x rotation), so
        the continuous sweep is covered, not just the samples (Codex L0b P1).
        extra_margin_m (obstacle recovery v4) is added to the margin, growing from 0
        over extra_ramp_m along the walk (None: in full from the start). unknown_passes (cusp
        position stall, plan design v7): walk on past unknown and edge cells, noting where the
        first was, so that only an occupied cell blocks -- the part of a sample inside the grid is
        still checked, every FREE cell met counts in the evidence age, and verified_m stops at the
        first unknown or edge."""
        checks = self._checks(samples, arc, footprint)
        mode = kernels.mode()
        walk = self._walk_reference if mode == "reference" else self._walk_numba if mode == "numba" else \
            self._walk_batched
        return walk(snapshot, checks, arc, footprint, own_cells, full_path_m=full_path_m, direction=direction,
                    own_pose=own_pose, own_footprint=own_footprint, extra_margin_m=extra_margin_m,
                    extra_ramp_m=extra_ramp_m, unknown_passes=unknown_passes)

    @staticmethod
    def _checks(samples, arc, footprint) -> list:
        """(pose, arc length, pad) of every check: the first sample, then each interval's midpoint."""
        radius = max(float(np.hypot(abs(lon) + max(fp.front_m, fp.rear_m), abs(lat) + fp.half_width_m))
                     for fp, lat, lon in parts_of(footprint))
        checks = [(samples[0], float(arc[0]), 0.0)]
        for i in range(1, len(samples)):
            a, b = samples[i - 1], samples[i]
            dyaw = float(np.arctan2(np.sin(b[2] - a[2]), np.cos(b[2] - a[2])))
            motion = float(np.hypot(b[0] - a[0], b[1] - a[1])) + radius * abs(dyaw)
            mid = np.array([(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, a[2] + dyaw / 2])
            checks.append((mid, float(arc[i]), motion / 2))
        return checks

    def _margin(self, s: float, pad: float, extra_margin_m: float, extra_ramp_m) -> float:
        cfg = self.config
        ramp = min(1.0, s / cfg.envelope_ramp_m) if cfg.envelope_ramp_m > 0 else 1.0
        margin = cfg.envelope_offset_m * ramp + pad
        if extra_margin_m:
            margin += extra_margin_m * (min(1.0, s / extra_ramp_m) if extra_ramp_m else 1.0)
        return margin

    def _walk_reference(self, snapshot, checks, arc, footprint, own_cells, *, full_path_m=None, direction=0,
                        own_pose=None, own_footprint=None, extra_margin_m=0.0, extra_ramp_m=None,
                        unknown_passes=False) -> Check:
        """The original walk, one check at a time (the oracle of real-time plan D2)."""
        cfg = self.config
        verified, blocked, oldest = 0.0, None, np.inf
        blocked_cells = ()
        first_unknown = None
        band = None  # cells near the present outline where RETAINED may pass, built on demand
        own_lookup = None  # (whole mask, partial index grid, outside sub-samples), built on demand
        for pose, s, pad in checks:
            margin = self._margin(s, pad, extra_margin_m, extra_ramp_m)
            cells, outside = shape_cells(snapshot, pose, footprint, margin, direction=direction)
            if outside:
                if not unknown_passes:
                    blocked = "edge"
                    break
                first_unknown = float(s) if first_unknown is None else first_unknown
                nx_, ny_ = snapshot.state.shape
                cells = cells[(cells[:, 0] >= 0) & (cells[:, 0] < nx_) & (cells[:, 1] >= 0) & (cells[:, 1] < ny_)]
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
                if not unknown_passes:
                    blocked = "unknown"
                    blocked_cells = tuple(map(tuple, cells[bad][:20].tolist()))
                    break
                first_unknown = float(s) if first_unknown is None else first_unknown
            # A RETAINED cell's stamp is the oldest fresh evidence its support
            # used, so it ages out under the same limit.
            seen = (states == FREE) | (states == RETAINED)
            if seen.any():
                oldest = min(oldest, float(np.min(snapshot.free_stamp[cells[seen, 0], cells[seen, 1]])))
            if first_unknown is None:
                verified = float(s)
        reached_end = blocked is None and full_path_m is not None and arc[-1] >= full_path_m - 1e-9
        return Check(verified, blocked, oldest, reached_end, blocked_cells, first_unknown)

    def _walk_numba(self, snapshot, checks, arc, footprint, own_cells, *, full_path_m=None, direction=0,
                    own_pose=None, own_footprint=None, extra_margin_m=0.0, extra_ramp_m=None,
                    unknown_passes=False) -> Check:
        """The reference walk compiled (real-time plan S3, ``permission_numba.walk``), bit for bit its result."""
        from forklift_core.control import permission_numba as nb

        cfg = self.config
        parts = parts_of(footprint)
        p_front, p_rear, p_hw, p_lat, p_lon = (np.array([v[n] for v in [(fp.front_m, fp.rear_m, fp.half_width_m,
                                                                           lat, lon) for fp, lat, lon in parts]],
                                                        dtype=float) for n in range(5))
        xs = np.array([float(pose[0]) for pose, _, _ in checks])
        ys = np.array([float(pose[1]) for pose, _, _ in checks])
        cs = np.array([cos(pose[2]) for pose, _, _ in checks])
        ss = np.array([sin(pose[2]) for pose, _, _ in checks])
        svals = np.array([s for _, s, _ in checks], dtype=float)
        margins = np.array([self._margin(s, pad, extra_margin_m, extra_ramp_m) for _, s, pad in checks])
        nx, ny = snapshot.state.shape
        if own_cells is None:  # the own outline found here (check_arc and check_path leave it to the kernel)
            o_parts = parts_of(own_footprint)
            o = [np.array([v[n] for v in [(fp.front_m, fp.rear_m, fp.half_width_m, lat, lon)
                                         for fp, lat, lon in o_parts]], dtype=float) for n in range(5)]
            oi0, oj0, whole_box, pid_box, pts, pstart, pcount, has_own = nb.own_lookup(
                float(snapshot.origin_x_m), float(snapshot.origin_y_m), float(snapshot.resolution_m), nx, ny,
                float(own_pose[0]), float(own_pose[1]), cos(own_pose[2]), sin(own_pose[2]), *o)
        else:
            whole, partial = own_cells
            has_own = bool(whole or partial)
            oi0 = oj0 = 0
            if has_own:
                whole_box, pid_box, (pts, pstart, pcount) = _own_lookup_flat(snapshot, own_cells, own_pose,
                                                                            own_footprint)
            else:
                whole_box = np.zeros((1, 1), dtype=bool)
                pid_box = np.full((1, 1), -1, dtype=np.int64)
                pts, pstart, pcount = np.zeros((0, 2)), np.zeros(0, dtype=np.int64), np.zeros(0, dtype=np.int64)
        band = np.zeros(snapshot.state.shape, dtype=bool)
        if (own_pose is not None and own_footprint is not None and cfg.shadow_band_m > 0
                and (snapshot.state == RETAINED).any()):
            near, _ = shape_cells(snapshot, own_pose, own_footprint, cfg.shadow_band_m)
            near = near[(near[:, 0] >= 0) & (near[:, 0] < nx) & (near[:, 1] >= 0) & (near[:, 1] < ny)]
            band[near[:, 0], near[:, 1]] = True
        verified, code, oldest, first_unknown, cells = nb.walk(
            snapshot.state, snapshot.free_stamp, float(snapshot.origin_x_m), float(snapshot.origin_y_m),
            float(snapshot.resolution_m), xs, ys, cs, ss, svals, margins, int(direction), bool(unknown_passes),
            p_front, p_rear, p_hw, p_lat, p_lon, has_own, oi0, oj0, whole_box, pid_box, pts, pstart, pcount,
            own_pose is not None, band)
        if code == nb.NEEDS_REFERENCE:
            if own_cells is None:
                own_cells = self._own_cells(snapshot, own_pose, own_footprint)
            return self._walk_reference(snapshot, checks, arc, footprint, own_cells, full_path_m=full_path_m,
                                        direction=direction, own_pose=own_pose, own_footprint=own_footprint,
                                        extra_margin_m=extra_margin_m, extra_ramp_m=extra_ramp_m,
                                        unknown_passes=unknown_passes)
        blocked = {0: None, nb.EDGE: "edge", nb.BLOCK_OCCUPIED: "occupied", nb.BLOCK_UNKNOWN: "unknown"}[int(code)]
        blocked_cells = tuple(map(tuple, cells.tolist())) if blocked in ("occupied", "unknown") else ()
        reached_end = blocked is None and full_path_m is not None and arc[-1] >= full_path_m - 1e-9
        return Check(float(verified), blocked, float(oldest), reached_end, blocked_cells,
                     None if math.isnan(first_unknown) else float(first_unknown))

    def _walk_batched(self, snapshot, checks, arc, footprint, own_cells, *, full_path_m=None, direction=0,
                      own_pose=None, own_footprint=None, extra_margin_m=0.0, extra_ramp_m=None,
                      unknown_passes=False, first_chunk: int = 2, chunk: int = 16) -> Check:
        """The walk with each chunk of checks' cells found in one pass (real-time plan S1), then judged in the
        reference's order: the same cells in the same order, the same samples counted, bit for bit."""
        cfg = self.config
        verified, blocked, oldest = 0.0, None, np.inf
        blocked_cells = ()
        first_unknown = None
        parts = parts_of(footprint)
        nx, ny = snapshot.state.shape
        whole, partial = own_cells
        own_lookup = _own_lookup_flat(snapshot, own_cells, own_pose, own_footprint) if (whole or partial) else None
        band = None
        start, size = 0, first_chunk
        while start < len(checks):
            # Growing chunks: a walk blocked at its first checks (most stop arcs that block) pays for few.
            part_checks = checks[start:start + size]
            start, size = start + size, min(chunk, 2 * size)
            margins = [self._margin(s, pad, extra_margin_m, extra_ramp_m) for _, s, pad in part_checks]
            cells, owner, outside = _shape_cells_batch(snapshot, [c[0] for c in part_checks], parts, margins,
                                                       direction)
            if cells is None:  # outside the key range: the reference walk decides (never on a real grid)
                return self._walk_reference(snapshot, checks, arc, footprint, own_cells, full_path_m=full_path_m,
                                            direction=direction, own_pose=own_pose, own_footprint=own_footprint,
                                            extra_margin_m=extra_margin_m, extra_ramp_m=extra_ramp_m,
                                            unknown_passes=unknown_passes)
            inside = (cells[:, 0] >= 0) & (cells[:, 0] < nx) & (cells[:, 1] >= 0) & (cells[:, 1] < ny)
            cells, owner = cells[inside], owner[inside]
            if own_lookup is not None:
                whole_mask, partial_id, flat = own_lookup
                keep = ~whole_mask[cells[:, 0], cells[:, 1]]
                ids = partial_id[cells[:, 0], cells[:, 1]]
                touched = keep & (ids >= 0)
                if touched.any() and own_pose is not None:
                    keep[touched] = _parts_reach_batch(flat, ids[touched], owner[touched],
                                                       [c[0] for c in part_checks], parts, margins, direction)
                cells, owner = cells[keep], owner[keep]
            states = snapshot.state[cells[:, 0], cells[:, 1]]
            retained = states == RETAINED
            if retained.any():
                if band is None:
                    band = np.zeros(snapshot.state.shape, dtype=bool)
                    if own_pose is not None and own_footprint is not None and cfg.shadow_band_m > 0:
                        near, _ = shape_cells(snapshot, own_pose, own_footprint, cfg.shadow_band_m)
                        near = near[(near[:, 0] >= 0) & (near[:, 0] < nx) & (near[:, 1] >= 0) & (near[:, 1] < ny)]
                        band[near[:, 0], near[:, 1]] = True
                judged = states.copy()
                judged[retained & ~band[cells[:, 0], cells[:, 1]]] = 0
            else:
                judged = states
            n = len(part_checks)
            occupied = states == OCCUPIED
            bad = (judged != FREE) & (judged != RETAINED)
            seen = (judged == FREE) | (judged == RETAINED)
            any_occupied = np.bincount(owner[occupied], minlength=n) > 0
            any_bad = np.bincount(owner[bad], minlength=n) > 0
            # The oldest FREE stamp per check: a minimum of nonzero finite times, the same whatever the order.
            seen_owner = owner[seen]
            seen_stamp = snapshot.free_stamp[cells[seen, 0], cells[seen, 1]]
            if np.any((seen_stamp == 0.0) | np.isnan(seen_stamp)):
                # Which zero, or NaN, np.min keeps depends on its reduction order: the reference decides.
                return self._walk_reference(snapshot, checks, arc, footprint, own_cells, full_path_m=full_path_m,
                                            direction=direction, own_pose=own_pose, own_footprint=own_footprint,
                                            extra_margin_m=extra_margin_m, extra_ramp_m=extra_ramp_m,
                                            unknown_passes=unknown_passes)
            present, first = np.unique(seen_owner, return_index=True)
            lowest = dict(zip(present.tolist(), np.minimum.reduceat(seen_stamp, first).tolist(), strict=True)) \
                if len(seen_owner) else {}
            for k, (_, s, _) in enumerate(part_checks):
                if outside[k]:
                    if not unknown_passes:
                        blocked = "edge"
                        break
                    first_unknown = float(s) if first_unknown is None else first_unknown
                if any_occupied[k]:
                    blocked = "occupied"
                    blocked_cells = tuple(map(tuple, cells[(owner == k) & occupied][:20].tolist()))
                    break
                if any_bad[k]:
                    if not unknown_passes:
                        blocked = "unknown"
                        blocked_cells = tuple(map(tuple, cells[(owner == k) & bad][:20].tolist()))
                        break
                    first_unknown = float(s) if first_unknown is None else first_unknown
                if k in lowest:
                    oldest = min(oldest, float(lowest[k]))
                if first_unknown is None:
                    verified = float(s)
            if blocked is not None:
                break
        reached_end = blocked is None and full_path_m is not None and arc[-1] >= full_path_m - 1e-9
        return Check(verified, blocked, oldest, reached_end, blocked_cells, first_unknown)

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

    @dumped("check_path", pure=True, strata=lambda self_, a, k, r: (
        k.get("direction", 0), bool(k.get("unknown_passes")), bool(k.get("extra_margin_m")),
        k.get("extra_ramp_m") is not None, k.get("lookahead_m") is not None, shape_kind(a[2]),
        self_.config.step_m, r.blocked, r.reached_end))
    @dual("check_path")
    def check_path(self, snapshot: GridSnapshot, path_ahead, footprint: Footprint, own_footprint: Footprint, *,
                   current_pose, direction: int = 0, extra_margin_m: float = 0.0,
                   extra_ramp_m: float | None = None, unknown_passes: bool = False,
                   lookahead_m: float | None = None) -> Check:
        """The verified distance along path_ahead on snapshot, leaving this permission as it is
        (obstacle recovery v4 D1: a candidate is checked before it is installed). lookahead_m
        replaces the config's walk length for this check (the return turnaround checks a whole
        reverse arc longer than it); None keeps it."""
        poses = np.asarray(path_ahead, dtype=float)
        if poses.ndim != 2 or poses.shape[1] != 3 or not len(poses):
            raise ValueError("path_ahead must be a non-empty (N, 3) array")
        limit = self.config.lookahead_m if lookahead_m is None else float(lookahead_m)
        samples, arc, total = resample_path(poses, self.config.step_m, limit)
        own = None if kernels.mode() == "numba" else self._own_cells(snapshot, current_pose, own_footprint)
        walked, samples, arc, cap = self._start_release(snapshot, samples, arc, footprint, current_pose)
        return self._capped(self._walk(walked, samples, arc, footprint, own, full_path_m=None if cap else total,
                                       direction=direction, own_pose=current_pose, own_footprint=own_footprint,
                                       extra_margin_m=extra_margin_m, extra_ramp_m=extra_ramp_m,
                                       unknown_passes=unknown_passes), cap)

    @dumped("check_arc", pure=True, strata=lambda self_, a, k, r: (
        a[3] >= 0, bucket(a[2]), shape_kind(a[4]), self_.config.step_m, r.blocked, r.verified_m > 0))
    @dual("check_arc")
    def check_arc(self, snapshot: GridSnapshot, current_pose, curvature_inv_m: float, direction: int,
                  footprint: Footprint, own_footprint: Footprint, speed_cap_mps: float) -> Check:
        """The emergency-stop arc allowed_speed walks, on snapshot, leaving this permission as it is."""
        cfg = self.config
        length = cfg.stopping.distance_m(speed_cap_mps) + cfg.step_m
        samples, arc = arc_poses(current_pose, curvature_inv_m, 1 if direction >= 0 else -1, length, cfg.step_m)
        own = None if kernels.mode() == "numba" else self._own_cells(snapshot, current_pose, own_footprint)
        walked, samples, arc, cap = self._start_release(snapshot, samples, arc, footprint, current_pose)
        return self._capped(self._walk(walked, samples, arc, footprint, own, direction=1 if direction >= 0 else -1,
                                       own_pose=current_pose, own_footprint=own_footprint), cap)

    def _start_release(self, snapshot, samples, arc, footprint, current_pose):
        """(the snapshot the walk reads, its samples, their arc lengths, the cap) of the start-band release.

        Off (no band, no anchor, a correction since it, nothing in the band): the walk's own, cap None. On: the
        start-band cells read as free (never as evidence) and the samples cut at the last pose before the swept outline
        first comes nearer to one of them than allowed; the cap is (that arc length, those cells) or None when the
        whole walk keeps away. Each call raises the cells' best distances to the current pose's (never lowers them)."""
        band = float(self.config.start_band_m)
        anchor = self.release_anchor
        if band <= 0.0 or anchor is None or current_pose is None or snapshot is None:
            return snapshot, samples, arc, None
        if int(snapshot.correction_version) != anchor["version"]:
            self.release_anchor = None  # the grid moved under the anchor: no release until the truck stands again
            return snapshot, samples, arc, None
        from forklift_core.control import start_release as sr

        cells, d0 = sr.start_band_cells(snapshot, anchor["pose"], footprint, band)
        if not len(cells):
            return snapshot, samples, arc, None
        pose = tuple(float(v) for v in current_pose)
        now = sr.part_distances(snapshot, cells, [pose], footprint)[0]  # (m, P)
        # each cell's history under a key that survives the rolling grid's origin moving (its world cell index) and
        # apart for each checked shape (hull forward, body and blades reversing -- Codex v2 P2-1, P2-2)
        keys = sr.history_keys(snapshot, cells, footprint)
        best = np.array([np.maximum(anchor["best"].get(k, d), d0_c) for k, d, d0_c in zip(keys, now, d0)])
        if np.all(best.min(axis=1) > band):
            self.release_anchor = None  # moved away from every start-band cell: the walk's own rule from here
            return snapshot, samples, arc, None
        # a part that started within the band is held to the band, one that started farther to the band plus the
        # stopping margin (the walk's own lateral and stopping inflation, never more than its best so far)
        cap = np.where(d0 > band, band + float(self.config.stopping.margin_m), band)
        first, hit = sr.release_length(snapshot, cells, np.minimum(best, cap), cap, samples, arc, footprint)
        for k, d in zip(keys, np.maximum(best, now)):
            anchor["best"][k] = d
        walked = sr.released(snapshot, cells)
        if not np.isfinite(first):
            return walked, samples, arc, None
        keep = np.asarray(arc) <= first + 1e-12
        keep[0] = True
        return walked, np.asarray(samples)[keep], np.asarray(arc)[keep], (first, tuple(map(tuple, hit[:20].tolist())))

    @staticmethod
    def _capped(check: Check, cap) -> Check:
        """The walk cut at a start-band cap: blocked there by the cells that end the release, unless it stopped
        before on its own (its own reason, cells and evidence of the part it walked)."""
        if cap is None or check.blocked is not None:
            return check
        first, hit = cap
        return Check(min(check.verified_m, first), "occupied", check.oldest_free_s, False, hit, check.first_unknown_m)

    def update(self, snapshot: GridSnapshot, path_ahead, footprint: Footprint, own_footprint: Footprint, *,
               current_pose, direction: int = 0, lookahead_m: float | None = None) -> Check:
        """New snapshot: the verified distance along path_ahead (rear-axle poses from the truck on). lookahead_m
        shortens the walk as in check_path (the teleoperated arcs walk only what their speed limit can use)."""
        check = self.check_path(snapshot, path_ahead, footprint, own_footprint, current_pose=current_pose,
                                direction=direction, lookahead_m=lookahead_m)
        self.snapshot = snapshot
        self.path_check = check
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
        estop = self.check_arc(snap, current_pose, curvature_inv_m, direction, footprint, own_footprint, speed_cap_mps)
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
