"""Low-clearance guard (docs/plans/2026-10-25-low-clearance-guard.md).

Props below the LiDAR plane are not in the obstacle layer, so the drive permission never sees them: only the planning
clearance keeps the truck off them, and a tracking deviation (a heading error on the loaded outline's long front) can
use it up. The guard watches those props along the permission's stop arc from the estimated pose. Each check is held
to the planned clearance the same distance ahead on the leg -- ``min(margin, max(0, c - deviation))`` -- so a plan that
passes close on purpose is not stopped while it is followed, and contact is always a violation. The speed is capped
so the truck stops before the first violation; past the leg end the arc runs only as far as the truck can overrun it
(the end allowance or the physical braking distance) with the planned clearance carried on less the overrun, and
interval midpoints with half the motion between samples taken off keep the sweep between
samples (as the permission's walk does)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from forklift_core.control.drive_permission import StoppingModel, arc_poses, parts_of
from forklift_core.control.start_release import footprint_rect_distance
from forklift_core.planning.pallet_mission import LIDAR_PLANE_M, SEARCH_TAG, SLOT_TAG, TransportScenario


RECORD_CUTOFF_M = 0.2  # the leg's planned clearance is recorded exactly below this
RECORD_STRIDE = 4  # ... at every this many path poses (a record, not a check: the guard computes c where it needs it)


@dataclass(frozen=True)
class LowClearanceConfig:
    margin_m: float = 0.05  # the share of localisation error and pallet slip on the forks the guard keeps
    deviation_m: float = 0.02  # how much nearer than planned a tight plan may be followed
    step_m: float = 0.01  # arc sample spacing; half the motion between samples comes off each midpoint

    def __post_init__(self) -> None:
        for name in ("margin_m", "deviation_m", "step_m"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0) or (name == "step_m" and value <= 0.0):
                raise ValueError(f"{name} must be finite and non-negative (step positive)")


def low_props(scenario: TransportScenario, stage: str, plane_m: float = LIDAR_PLANE_M) -> list:
    """The rectangles, at their size, of ``scenario``'s props no taller than ``plane_m`` that a check at ``stage`` sees
    (pallet_mission.scenario_props' stage rule: "search" leaves out the slot copies, "slot" the search ones)."""
    if stage not in ("search", "slot"):
        raise ValueError(f"unknown stage {stage!r}")
    drop = SLOT_TAG if stage == "search" else SEARCH_TAG
    return [p.rectangle for p in scenario.props if not p.asset.uri.endswith(drop) and p.asset.height_m <= plane_m]


def shape_radius(shape) -> float:
    """The farthest any point of the shape lies from the rear axle."""
    return max(math.hypot(abs(lon) + max(fp.front_m, fp.rear_m), abs(lat) + fp.half_width_m)
               for fp, lat, lon in parts_of(shape))


def _lower_bound(pose, parts, rect) -> float:
    """A cheap lower bound on the distance between the shape's parts at ``pose`` and ``rect``: for each part, the
    largest gap between the two rectangles' projections on the four edge directions (a separating-axis bound; never
    above the exact distance, exact when they are separated along one of those axes)."""
    c, s_ = math.cos(pose[2]), math.sin(pose[2])
    rc, rs = math.cos(rect.yaw_rad), math.sin(rect.yaw_rad)
    hl, hw = rect.length_m / 2, rect.width_m / 2
    corners = [(rect.x_m + a * rc - b * rs, rect.y_m + a * rs + b * rc) for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]
    best = math.inf
    for fp, lat, lon in parts:
        ox, oy = pose[0] + lon * c - lat * s_, pose[1] + lon * s_ + lat * c  # the part's frame origin
        pc = [(ox + a * c - b * s_, oy + a * s_ + b * c) for a, b in
              ((fp.front_m, fp.half_width_m), (fp.front_m, -fp.half_width_m), (-fp.rear_m, -fp.half_width_m),
               (-fp.rear_m, fp.half_width_m))]
        gap = 0.0
        for ax, ay in ((c, s_), (-s_, c), (rc, rs), (-rs, rc)):
            p1 = [x * ax + y * ay for x, y in pc]
            p2 = [x * ax + y * ay for x, y in corners]
            gap = max(gap, min(p2) - max(p1), min(p1) - max(p2))
        best = min(best, gap)
    return best


def _rect_box(rect):
    """(xmin, ymin, xmax, ymax) of a rectangle."""
    ex = abs(rect.length_m / 2 * math.cos(rect.yaw_rad)) + abs(rect.width_m / 2 * math.sin(rect.yaw_rad))
    ey = abs(rect.length_m / 2 * math.sin(rect.yaw_rad)) + abs(rect.width_m / 2 * math.cos(rect.yaw_rad))
    return rect.x_m - ex, rect.y_m - ey, rect.x_m + ex, rect.y_m + ey


def _outline_box(poses, shape):
    """(xmin, ymin, xmax, ymax) of the shape's part corners at all the poses."""
    xs, ys = [], []
    for pose in poses:
        c, s_ = math.cos(pose[2]), math.sin(pose[2])
        for fp, lat, lon in parts_of(shape):
            for a, b in ((lon + fp.front_m, lat + fp.half_width_m), (lon + fp.front_m, lat - fp.half_width_m),
                         (lon - fp.rear_m, lat - fp.half_width_m), (lon - fp.rear_m, lat + fp.half_width_m)):
                xs.append(pose[0] + a * c - b * s_)
                ys.append(pose[1] + a * s_ + b * c)
    return min(xs), min(ys), max(xs), max(ys)


def _box_gap(a, b) -> float:
    """Distance between two axis-aligned boxes (0 where they overlap): a lower bound for what they hold."""
    return math.hypot(max(0.0, b[0] - a[2], a[0] - b[2]), max(0.0, b[1] - a[3], a[1] - b[3]))


def shape_clearance(pose, shape, rects, cutoff: float = math.inf) -> float:
    """Exact distance between the shape at ``pose`` and the nearest rectangle (0 where they meet, inf with none).
    With ``cutoff``, a rectangle whose cheap lower bound exceeds ``cutoff`` is not measured: the result is then exact
    where at or below ``cutoff`` (a contact included, whatever the cutoff) and above it otherwise."""
    if not rects:
        return math.inf
    parts = parts_of(shape)
    best = math.inf
    pose_ = tuple(float(v) for v in pose)
    for r in rects:
        lb = _lower_bound(pose_, parts, r)
        if lb >= best or lb > cutoff:
            continue  # no nearer than the nearest already found, or beyond the cutoff (a contact is always measured)
        best = min(best, footprint_rect_distance(pose_, shape, r))
        if best <= 0.0:
            return 0.0
    return best


@dataclass
class LegClearance:
    """The planned clearance ``c(s)`` along one tracker path: at every path pose, the shape's distance to the low props
    of the stage that planned that part of the path (``slot_from_m``: path length from which the slot props count --
    a transport's delivery straight; ``slot_to_m``: path length up to which they count -- a return's start backoff,
    docs/plans/2026-10-27-return-start-withdraw-extension.md R5; None: none). Built once when the tracker is
    installed."""

    poses: np.ndarray
    shape: object
    search_props: list
    slot_props: list
    slot_from_m: float | None = None
    slot_to_m: float | None = None
    s: np.ndarray = field(init=False)
    c: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        poses = np.asarray(self.poses, dtype=float)
        if poses.ndim != 2 or poses.shape[1] != 3 or len(poses) < 1:
            raise ValueError("poses must be an (N, 3) array")
        self.poses = poses
        self.s = np.r_[0.0, np.cumsum(np.hypot(np.diff(poses[:, 0]), np.diff(poses[:, 1])))]
        # for the record only (the guard computes c where it needs it): exact below RECORD_CUTOFF_M
        boxes = {id(props): [(r, _rect_box(r)) for r in props] for props in (self.search_props, self.slot_props)}
        c = []
        for p_, si in zip(poses[::RECORD_STRIDE], self.s[::RECORD_STRIDE]):
            box = _outline_box([p_], self.shape)
            near = [r for r, rb in boxes[id(self.props_at(si))] if _box_gap(box, rb) < RECORD_CUTOFF_M]
            c.append(shape_clearance(p_, self.shape, near, cutoff=RECORD_CUTOFF_M) if near else math.inf)
        self.c = np.array(c)

    @property
    def length_m(self) -> float:
        return float(self.s[-1])

    def props_at(self, s: float) -> list:
        slot = ((self.slot_from_m is not None and s >= self.slot_from_m)
                or (self.slot_to_m is not None and s < self.slot_to_m))
        return self.slot_props if slot else self.search_props

    def pose_at(self, s: float) -> np.ndarray:
        """The planned pose at path length ``s`` (clamped), between path poses along their chord and turn."""
        s = min(max(float(s), 0.0), self.length_m)
        if len(self.s) == 1:
            return self.poses[0].copy()
        k = int(np.clip(np.searchsorted(self.s, s, side="right") - 1, 0, len(self.s) - 2))
        span = float(self.s[k + 1] - self.s[k])
        t = 0.0 if span <= 0.0 else (s - float(self.s[k])) / span
        a, b = self.poses[k], self.poses[k + 1]
        dyaw = float(np.arctan2(np.sin(b[2] - a[2]), np.cos(b[2] - a[2])))
        return np.array([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * dyaw])

    def at(self, s: float, cutoff: float = math.inf) -> float:
        """c at path length ``s`` (clamped to the path): the shape at the planned pose there against the props of the
        stage that planned it -- computed, never blended between poses or across the stage boundary (Codex rounds 8,
        9); inf where no prop is near. Exact below ``cutoff``, at least ``cutoff`` otherwise."""
        s = min(max(float(s), 0.0), self.length_m)
        return shape_clearance(self.pose_at(s), self.shape, self.props_at(s), cutoff=cutoff)


@dataclass(frozen=True)
class GuardCap:
    allowed_mps: float  # inf: no cap
    s_star_m: float  # arc length of the first violation (inf: none)
    lower_bound_m: float  # the violating check's lower bound (inf: none)
    threshold_m: float  # its m(s)
    arc_m: float  # the arc length looked at


def guard_cap(pose, curvature_inv_m: float, direction: int, speed_cap_mps: float, shape, leg: LegClearance, *,
              progress_m: float, leg_remaining_m: float, end_allowance_m: float, stopping: StoppingModel,
              config: LowClearanceConfig) -> GuardCap:
    """The speed cap of one control tick (plan D3): the stop arc at the held curvature; the first sample, then each
    interval midpoint with half the motion between samples taken off, against
    ``min(margin, max(0, c(progress + s) - deviation))`` -- a lower bound at or below 0 always violates.

    Past the leg end (the tracker stops there) the arc runs to the farther of the end plus ``end_allowance_m`` and the
    physical braking distance at this speed (latency and deceleration, without the permission's fixed stopping margin):
    the truck can overrun the end that far, never the margin. There the planned clearance is carried on from the end,
    less the distance past it (as if the end were driven straight on into the prop)."""
    step = config.step_m
    remaining = max(0.0, float(leg_remaining_m))
    v = abs(float(speed_cap_mps))
    braking = v * stopping.latency_s + v * v / (2.0 * stopping.decel_mps2)
    length = min(stopping.distance_m(speed_cap_mps) + step,
                 max(remaining + max(0.0, end_allowance_m), braking + step))
    samples, arc = arc_poses(tuple(float(v) for v in pose), float(curvature_inv_m), 1 if direction >= 0 else -1,
                             length, step)
    radius = shape_radius(shape)
    checks = [(samples[0], float(arc[0]), 0.0, float(arc[0]))]
    for i in range(1, len(samples)):
        a, b = samples[i - 1], samples[i]
        dyaw = float(np.arctan2(np.sin(b[2] - a[2]), np.cos(b[2] - a[2])))
        motion = float(np.hypot(b[0] - a[0], b[1] - a[1])) + radius * abs(dyaw)
        mid = np.array([(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, a[2] + dyaw / 2])
        checks.append((mid, (float(arc[i - 1]) + float(arc[i])) / 2, motion / 2, float(arc[i - 1])))
    end_along = progress_m + remaining
    # One sweep-wide filter per stage: a prop whose axis-aligned box stays farther than the margin plus the largest pad
    # from the axis-aligned box of every check's outline cannot violate anywhere on this arc.
    reach_cut = config.margin_m + max(pad for _, _, pad, _ in checks)
    sweep_box = _outline_box([c_[0] for c_ in checks], shape)
    near_cache: dict = {}

    def near_props(props):
        key = id(props)
        if key not in near_cache:
            near_cache[key] = [r for r in props if _box_gap(sweep_box, _rect_box(r)) <= reach_cut]
        return near_cache[key]

    for check_pose, s_check, pad, s_from in checks:
        along = min(progress_m + s_check, end_along)
        props = near_props(leg.props_at(along))
        if not props:
            continue
        lower = shape_clearance(check_pose, shape, props, cutoff=config.margin_m + pad) - pad
        if lower > 0.0 and lower >= config.margin_m:
            continue  # no threshold exceeds the margin and no contact: no violation, whatever the plan left here
        overrun = max(0.0, s_check - remaining)
        # exact where it can matter: a c at or above margin + deviation + overrun gives the margin itself
        planned = leg.at(along, cutoff=config.margin_m + config.deviation_m + overrun) - overrun
        threshold = min(config.margin_m, max(0.0, planned - config.deviation_m))
        if lower <= 0.0 or lower < threshold:
            return GuardCap(stopping.speed_for(s_from) if s_from > 0.0 else 0.0, s_from, lower, threshold, length)
    return GuardCap(math.inf, math.inf, math.inf, math.inf, length)
