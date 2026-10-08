"""Deterministic Hybrid A* with forward/reverse bicycle arcs and Dubins shots.

Continuous poses are retained inside discretized x/y/yaw/control search keys,
following the search principle in Dolgov et al., Practical Search Techniques
in Path Planning for Autonomous Driving (2008):
https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf

This implementation uses weighted Euclidean/heading heuristics, bounded work,
and forward-only or reverse-only Dubins analytic connections. It does not
implement the paper's smoothing stage or Reeds-Shepp solver, and makes no
completeness, shortest-path, or minimum-time guarantee. Steering may change
instantaneously; an adapter must account for steering rate and stop at cusps.
"""

import heapq
import time
from collections.abc import Sequence
from dataclasses import dataclass, replace
from itertools import count
from math import acos, atan2, ceil, cos, hypot, isfinite, pi, sin, sqrt

import numpy as np
from numpy.typing import NDArray

from forklift_core._validation import _finite_scalar

from .geometry import Bounds, Footprint, FootprintCollisionChecker, Pose2D, Rectangle
from .grid_collision import make_checker
from .reeds_shepp import reeds_shepp_words


@dataclass(frozen=True)
class PlannerConfig:
    """Algorithm settings, not measured chassis parameters; SI units.

    Curvature is yaw change per signed axle travel. Override its synthetic
    default using the chosen vehicle's verified steering/wheelbase envelope.
    """

    curvature_limit_inv_m: float = 0.5
    xy_resolution_m: float = 0.2
    yaw_resolution_rad: float = pi / 18
    primitive_length_m: float = 0.5
    collision_step_m: float = 0.06
    max_expansions: int = 12000
    analytic_expansion_interval: int = 8
    heuristic_weight: float = 1.8
    reverse_penalty: float = 1.3
    gear_change_penalty_m: float = 1.0
    steering_penalty: float = 0.15
    steering_change_penalty_m: float = 0.15
    clearance_m: float = 0.0
    # None keeps the distance/heading heuristic alone. A positive cell size adds
    # the paper's holonomic-with-obstacles term: a goal-rooted grid distance.
    obstacle_heuristic_resolution_m: float | None = None
    # Goal connection (priority-5 plan D7a): "dubins" -- forward-only or reverse-only
    # Dubins words, the original; "reeds_shepp" -- every Reeds-Shepp word (gear
    # changes inside the shot) together with the Dubins ones, tried in cost order.
    # A Reeds-Shepp word with a segment shorter than rs_min_segment_m is not tried:
    # the tracker cannot drive a cusp every few centimetres (its cusp tolerance is 0.03 m).
    goal_connection: str = "dubins"
    rs_min_segment_m: float = 0.10
    # Shot cap (priority-5 plan D7a, D7d): an analytic connection longer than the
    # straight-line distance to the goal plus shot_cap_m is not taken at once -- the
    # search keeps expanding (a short reverse then a forward shot replaces the 21.7 m
    # forward loop a 0.02 rad heading fix at the goal gave). The shortest one passed
    # over is kept and returned if the search ends without a path, so no plan the
    # uncapped search finds is lost. None keeps the original behaviour.
    shot_cap_m: float | None = None
    # Expansions the search may spend after its first capped-out shot before it
    # returns the cheapest one (bounds the extra time; a deadline also returns it).
    shot_cap_patience: int = 2000
    # D7b (priority-5 plan): besides the arcs, expand a turn cycle -- full steering one
    # way for turn_cycle_m, then the other gear at full opposite steering for
    # turn_cycle_m (four variants). Its first half is never a search state.
    turn_cycles: bool = False
    turn_cycle_m: float = 0.5

    def __post_init__(self) -> None:
        positive = (
            self.curvature_limit_inv_m,
            self.xy_resolution_m,
            self.yaw_resolution_rad,
            self.primitive_length_m,
            self.collision_step_m,
            self.heuristic_weight,
            self.reverse_penalty,
        )
        nonnegative = (
            self.gear_change_penalty_m,
            self.steering_penalty,
            self.steering_change_penalty_m,
            self.clearance_m,
        )
        for value in (*positive, *nonnegative):
            _finite_scalar(value, "planner setting")
        if min(positive) <= 0 or min(nonnegative) < 0:
            raise ValueError("planner scales must be positive; penalties nonnegative")
        if self.yaw_resolution_rad > pi or self.reverse_penalty < 1:
            raise ValueError("yaw resolution must be <= pi and reverse penalty >= 1")
        for value in (self.max_expansions, self.analytic_expansion_interval):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError("expansion limits must be positive integers")
        resolution = self.obstacle_heuristic_resolution_m
        if (
            resolution is not None
            and _finite_scalar(resolution, "obstacle heuristic resolution") <= 0
        ):
            raise ValueError("obstacle heuristic resolution must be positive")
        if self.goal_connection not in ("dubins", "reeds_shepp"):
            raise ValueError("goal_connection must be dubins or reeds_shepp")
        if _finite_scalar(self.rs_min_segment_m, "rs_min_segment_m") < 0:
            raise ValueError("rs_min_segment_m must be nonnegative")
        if self.shot_cap_m is not None and _finite_scalar(self.shot_cap_m, "shot_cap_m") < 0:
            raise ValueError("shot_cap_m must be nonnegative")
        if _finite_scalar(self.turn_cycle_m, "turn_cycle_m") <= 0:
            raise ValueError("turn_cycle_m must be positive")
        if isinstance(self.shot_cap_patience, bool) or not isinstance(self.shot_cap_patience, int) \
                or self.shot_cap_patience <= 0:
            raise ValueError("shot_cap_patience must be a positive integer")


@dataclass(frozen=True)
class PlanResult:
    """Path samples in the world frame, all positions at the rear axle.

    directions[i] and curvatures_inv_m[i] control the segment arriving at i.
    Sample zero repeats the first segment's controls. A direction change at i
    means a stop/reversal at poses[i-1]. The exact continuous goal is retained
    (heading modulo 2*pi). Failure arrays are empty, never partial drive paths.
    """

    success: bool
    status: str
    poses: NDArray[np.float64]
    directions: NDArray[np.int8]
    curvatures_inv_m: NDArray[np.float64]
    length_m: float
    expanded_nodes: int
    # Set by pallet_mission._search, which may retry on a finer lattice and at
    # denser analytic intervals: the interval of the search that produced this
    # result, and one (interval, status, expanded_nodes, xy_resolution_m,
    # yaw_resolution_rad, collision_step_m, primitive_length_m, max_expansions,
    # pruned_children, superseded_pops) entry per search it ran, kept free of timing so
    # equal inputs give equal results. None and () for a direct plan_hybrid_astar call.
    analytic_expansion_interval: int | None = None
    search_attempts: tuple = ()
    # Search diagnostics (priority-5 plan D7d): children dropped because their
    # state key already had a cheaper or equal cost, and queue entries skipped
    # because a cheaper node replaced them. They never change the result.
    pruned_children: int = 0
    superseded_pops: int = 0
    # D7b: turn cycles queued during the search, and cycles on the returned path.
    cycles_queued: int = 0
    cycles_in_path: int = 0
    # Shot cap: connections passed over for their length, and whether the result is
    # the shortest of them, returned because the search found nothing else.
    shots_capped: int = 0
    shot_fallback: bool = False
    # The root's goal shot: "taken", "capped" (passed over for its length) or "none";
    # D7b cycles generated (both halves free) before the key check.
    root_shot: str = "none"
    cycles_generated: int = 0


def _wrap(angle):
    return (angle + pi) % (2 * pi) - pi


class _ObstacleDistance:
    """Goal-rooted 8-connected grid distance for the rear axle around obstacles.

    The rear axle always carries a disc of the footprint's shortest semi-axis
    plus clearance, so it never comes closer than that to an obstacle or the
    bounds. A cell is blocked only if its centre is closer than that radius
    minus half a cell diagonal: every cell holding a usable axle position stays
    open, so a narrow gap is never sealed. Returned distances are shortened by
    one cell diagonal for the cell-centre offset. The result guides the search
    and, like the planner's weighted heuristic, carries no optimality claim.
    Cells the grid cannot reach return 0 and leave the other heuristic terms
    in charge.
    """

    def __init__(
        self,
        obstacles: Sequence[Rectangle],
        footprint: Footprint,
        bounds: Bounds,
        goal_xy: tuple[float, float],
        *,
        resolution_m: float,
        clearance_m: float,
        occupancy=None,
    ) -> None:
        self.bounds, self.resolution_m = bounds, resolution_m
        self.shape = (
            max(1, ceil((bounds.x_max_m - bounds.x_min_m) / resolution_m)),
            max(1, ceil((bounds.y_max_m - bounds.y_min_m) / resolution_m)),
        )
        x = bounds.x_min_m + (np.arange(self.shape[0]) + 0.5) * resolution_m
        y = bounds.y_min_m + (np.arange(self.shape[1]) + 0.5) * resolution_m
        cx, cy = np.meshgrid(x, y, indexing="ij")
        radius = min(footprint.front_m, footprint.rear_m, footprint.half_width_m)
        reach = radius + clearance_m - resolution_m * sqrt(2) / 2
        blocked = (
            np.minimum.reduce(
                [
                    cx - bounds.x_min_m,
                    bounds.x_max_m - cx,
                    cy - bounds.y_min_m,
                    bounds.y_max_m - cy,
                ]
            )
            < reach
        )
        for o in obstacles:
            c, s = cos(o.yaw_rad), sin(o.yaw_rad)
            dx, dy = cx - o.x_m, cy - o.y_m
            along = np.maximum(np.abs(dx * c + dy * s) - o.length_m / 2, 0)
            across = np.maximum(np.abs(dy * c - dx * s) - o.width_m / 2, 0)
            blocked |= np.hypot(along, across) < reach
        if occupancy is not None and hasattr(occupancy, "body"):
            # A split grid (docking, plan D5): the heuristic takes the part
            # with fewer occupied cells (the band-cleared one) -- blocking less
            # can only underestimate, never seal a passage the checker allows.
            occupancy = occupancy.forks
        if occupancy is not None and occupancy.occupied.any():
            # Occupied grid cells (plan D3): mark the heuristic cells holding
            # an occupied centre, then block only where the nearest such
            # centre is certainly within reach -- a cell centre is at most a
            # half diagonal from its occupied centre, so a gap is never sealed.
            ii, jj = np.nonzero(occupancy.occupied)
            ox = occupancy.origin_x_m + (ii + 0.5) * occupancy.resolution_m
            oy = occupancy.origin_y_m + (jj + 0.5) * occupancy.resolution_m
            hi = np.floor((ox - bounds.x_min_m) / resolution_m).astype(int)
            hj = np.floor((oy - bounds.y_min_m) / resolution_m).astype(int)
            inside = (hi >= 0) & (hi < self.shape[0]) & (hj >= 0) & (hj < self.shape[1])
            seeds = np.zeros(self.shape, dtype=bool)
            seeds[hi[inside], hj[inside]] = True
            # Heuristic cells whose centre is within reach - half diagonal of
            # a seed cell's centre (the occupied centre lies inside that cell).
            span = int(ceil(reach / resolution_m))
            offsets = [
                (di, dj)
                for di in range(-span, span + 1)
                for dj in range(-span, span + 1)
                if hypot(di, dj) * resolution_m + resolution_m * sqrt(2) / 2 < reach
            ]
            si, sj = np.nonzero(seeds)
            for di, dj in offsets:
                ti, tj = si + di, sj + dj
                ok = (ti >= 0) & (ti < self.shape[0]) & (tj >= 0) & (tj < self.shape[1])
                blocked[ti[ok], tj[ok]] = True
        self.distance_m = np.full(self.shape, np.inf)
        goal = self._cell(*goal_xy)
        self.distance_m[goal] = 0.0
        queue = [(0.0, goal)]
        diagonal = resolution_m * sqrt(2)
        steps = [
            (di, dj, diagonal if di and dj else resolution_m)
            for di in (-1, 0, 1)
            for dj in (-1, 0, 1)
            if di or dj
        ]
        while queue:
            distance, (i, j) = heapq.heappop(queue)
            if distance > self.distance_m[i, j]:
                continue
            for di, dj, step in steps:
                k, m = i + di, j + dj
                if not (0 <= k < self.shape[0] and 0 <= m < self.shape[1]):
                    continue
                if blocked[k, m] or distance + step >= self.distance_m[k, m]:
                    continue
                self.distance_m[k, m] = distance + step
                heapq.heappush(queue, (distance + step, (k, m)))
        self._slack_m = diagonal

    def _cell(self, x_m: float, y_m: float) -> tuple[int, int]:
        i = int((x_m - self.bounds.x_min_m) / self.resolution_m)
        j = int((y_m - self.bounds.y_min_m) / self.resolution_m)
        return (
            min(max(i, 0), self.shape[0] - 1),
            min(max(j, 0), self.shape[1] - 1),
        )

    def distance(self, x_m: float, y_m: float) -> float:
        value = self.distance_m[self._cell(x_m, y_m)]
        return max(0.0, value - self._slack_m) if isfinite(value) else 0.0


def _advance(pose, distance_m, curvature):
    x, y, yaw = pose
    turn = distance_m * curvature
    if abs(curvature) < 1e-12:
        return x + distance_m * cos(yaw), y + distance_m * sin(yaw), yaw
    return (
        x + (sin(yaw + turn) - sin(yaw)) / curvature,
        y + (cos(yaw) - cos(yaw + turn)) / curvature,
        _wrap(yaw + turn),
    )


def _sample_arc(pose, length, direction, curvature, checker, config):
    count_samples = max(1, ceil(length / config.collision_step_m))
    ds = length / count_samples
    # A body point travels at most ds * (1 + radius * |curvature|).
    # Midpoint inflation encloses the swept footprint for each half interval.
    margin = config.clearance_m + ds / 2 * (1 + checker.radius_m * abs(curvature))
    samples = []
    for index in range(count_samples):
        midpoint = _advance(pose, direction * ds * (index + 0.5), curvature)
        if not checker.free(midpoint, margin):
            return None
        samples.append(_advance(pose, direction * ds * (index + 1), curvature))
    if not checker.free(samples[-1], config.clearance_m):
        return None
    return samples


def _dubins_words(start, goal, curvature):
    """Six unit-radius Dubins families as (signed-turn word, lengths)."""
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    distance = hypot(dx, dy) * curvature
    theta = atan2(dy, dx)
    alpha = (start[2] - theta) % (2 * pi)
    beta = (goal[2] - theta) % (2 * pi)
    sa, sb, ca, cb = sin(alpha), sin(beta), cos(alpha), cos(beta)
    cab = cos(alpha - beta)
    tau = 2 * pi
    candidates = []
    p2 = 2 + distance**2 - 2 * cab + 2 * distance * (sa - sb)
    if p2 >= -1e-12:
        t = atan2(cb - ca, distance + sa - sb)
        candidates.append(
            ((1, 0, 1), ((t - alpha) % tau, sqrt(max(0, p2)), (beta - t) % tau))
        )
    p2 = 2 + distance**2 - 2 * cab + 2 * distance * (sb - sa)
    if p2 >= -1e-12:
        t = atan2(ca - cb, distance - sa + sb)
        candidates.append(
            ((-1, 0, -1), ((alpha - t) % tau, sqrt(max(0, p2)), (t - beta) % tau))
        )
    p2 = -2 + distance**2 + 2 * cab + 2 * distance * (sa + sb)
    if p2 >= -1e-12:
        p = sqrt(max(0, p2))
        t = atan2(-ca - cb, distance + sa + sb) - atan2(-2.0, p)
        candidates.append(((1, 0, -1), ((t - alpha) % tau, p, (t - beta) % tau)))
    p2 = distance**2 - 2 + 2 * cab - 2 * distance * (sa + sb)
    if p2 >= -1e-12:
        p = sqrt(max(0, p2))
        t = atan2(ca + cb, distance - sa - sb) - atan2(2.0, p)
        candidates.append(((-1, 0, 1), ((alpha - t) % tau, p, (beta - t) % tau)))
    value = (6 - distance**2 + 2 * cab + 2 * distance * (sa - sb)) / 8
    if abs(value) <= 1:
        p = (tau - acos(value)) % tau
        t = (alpha - atan2(ca - cb, distance - sa + sb) + p / 2) % tau
        candidates.append(((-1, 1, -1), (t, p, (alpha - beta - t + p) % tau)))
    value = (6 - distance**2 + 2 * cab + 2 * distance * (-sa + sb)) / 8
    if abs(value) <= 1:
        p = (tau - acos(value)) % tau
        t = (-alpha - atan2(ca - cb, distance + sa - sb) + p / 2) % tau
        candidates.append(((1, -1, 1), (t, p, (beta - alpha - t + p) % tau)))
    return candidates


def _connection(pose, goal, previous_direction, checker, config, cap_length=None):
    """(connector, its cost) for the cheapest collision-free goal shot no longer than
    cap_length (None: any length), and (over, its cost) for the cheapest collision-free
    one longer than the cap -- None where there is none. Cost is the candidates' own:
    length x reverse ratio + gear-change penalties (incl. one against the node's gear)."""
    if (
        hypot(pose[0] - goal[0], pose[1] - goal[1]) < 1e-12
        and abs(_wrap(pose[2] - goal[2])) < 1e-12
    ):
        return ([], [], [], 0.0), 0.0, None, None
    k = config.curvature_limit_inv_m
    candidates = []
    for direction in (1, -1):
        offset = pi if direction < 0 else 0
        words = _dubins_words(
            (pose[0], pose[1], pose[2] + offset),
            (goal[0], goal[1], goal[2] + offset),
            k,
        )
        for word, lengths in words:
            lengths = tuple(value / k for value in lengths)
            cost = sum(lengths) * (config.reverse_penalty if direction < 0 else 1)
            if previous_direction and direction != previous_direction:
                cost += config.gear_change_penalty_m
            # One segment list for both kinds: (gear, curvature, length).
            segments = tuple((direction, turn * direction * k, length) for turn, length in zip(word, lengths, strict=True))
            candidates.append((cost, segments))
    if config.goal_connection == "reeds_shepp":
        for word in reeds_shepp_words(pose, goal, k):
            used = [seg for seg in word if seg[2] >= 1e-10]
            if not used or any(seg[2] < config.rs_min_segment_m for seg in used):
                continue
            cost = sum(length * (config.reverse_penalty if gear < 0 else 1) for _, gear, length in used)
            gears = [gear for _, gear, _ in used]
            cost += config.gear_change_penalty_m * sum(a != b for a, b in zip(gears, gears[1:]))
            if previous_direction and gears[0] != previous_direction:
                cost += config.gear_change_penalty_m
            candidates.append((cost, tuple((gear, turn * k, length) for turn, gear, length in used)))
    candidates.sort(key=lambda candidate: candidate[0])
    over = over_cost = None
    for cost, segments in candidates:
        length_total = sum(seg[2] for seg in segments)
        capped = cap_length is not None and length_total > cap_length
        if capped and over is not None:
            continue  # only the cheapest over-cap shot is ever needed
        current = pose
        samples, directions, curvatures = [], [], []
        for direction, curvature, length in segments:
            if length < 1e-10:
                continue
            arc = _sample_arc(current, length, direction, curvature, checker, config)
            if arc is None:
                break
            samples.extend(arc)
            directions.extend([direction] * len(arc))
            curvatures.extend([curvature] * len(arc))
            current = arc[-1]
        else:
            # Numerical guards verify the analytic construction, never snap a
            # nearby lattice node to the goal via a nonholonomic interpolation.
            if hypot(current[0] - goal[0], current[1] - goal[1]) > 1e-7:
                continue
            if abs(_wrap(current[2] - goal[2])) > 1e-7:
                continue
            connector = (samples, directions, curvatures, length_total)
            if capped:
                over, over_cost = connector, cost
                continue
            return connector, cost, over, over_cost
    return None, None, over, over_cost


@dataclass
class _Node:
    pose: tuple
    cost: float
    direction: int
    steering: int
    parent: object
    samples: list
    cycle_half: bool = False  # the first half of a D7b turn cycle (never a search state)
    length_m: float = 0.0  # path length this node adds (its primitive or cycle half)


def _result(node, connector, expanded, config, counts=(0, 0, 0)):
    chain = []
    current = node
    while current.parent is not None:
        chain.append(current)
        current = current.parent
    cycles_in_path = sum(1 for item in chain if item.cycle_half)
    poses, directions, curvatures = [current.pose], [1], [0.0]
    for item in reversed(chain):
        poses.extend(item.samples)
        directions.extend([item.direction] * len(item.samples))
        curvatures.extend(
            [item.steering * config.curvature_limit_inv_m] * len(item.samples)
        )
    samples, gear, curvature, length = connector
    poses.extend(samples)
    directions.extend(gear)
    curvatures.extend(curvature)
    if len(poses) > 1:
        directions[0], curvatures[0] = directions[1], curvatures[1]
    return PlanResult(
        True,
        "success",
        np.asarray(poses, dtype=float),
        np.asarray(directions, dtype=np.int8),
        np.asarray(curvatures),
        sum(item.length_m for item in chain) + length,
        expanded,
        pruned_children=counts[0],
        superseded_pops=counts[1],
        cycles_queued=counts[2] if len(counts) > 2 else 0,
        cycles_in_path=cycles_in_path,
        shots_capped=counts[3] if len(counts) > 3 else 0,
        cycles_generated=counts[4] if len(counts) > 4 else 0,
    )


def _failure(status, expanded=0, counts=(0, 0, 0)):
    return PlanResult(
        False,
        status,
        np.empty((0, 3)),
        np.empty(0, dtype=np.int8),
        np.empty(0),
        0.0,
        expanded,
        pruned_children=counts[0],
        superseded_pops=counts[1],
        cycles_queued=counts[2] if len(counts) > 2 else 0,
        shots_capped=counts[3] if len(counts) > 3 else 0,
        cycles_generated=counts[4] if len(counts) > 4 else 0,
    )


def plan_hybrid_astar(
    start: Pose2D,
    goal: Pose2D,
    obstacles: Sequence[Rectangle],
    footprint: Footprint,
    bounds: Bounds,
    config: PlannerConfig | None = None,
    *,
    occupancy=None,
    deadline=None,
) -> PlanResult:
    """Plan bounded, collision-checked forward/reverse motion in a static map.

    occupancy optionally adds an OccupancyGrid (forklift_core.planning.
    grid_collision) checked cell by cell alongside the rectangles; unknown
    cells are free to the planner (priority-5 plan D3). deadline optionally
    stops the search at that time.monotonic() value with status "timeout".

    Invalid colliding endpoints return invalid_start/invalid_goal. Exhaustion
    returns no_path or expansion_limit; neither proves physical infeasibility.
    Malformed numerical inputs raise ValueError at their dataclass boundary.
    """
    config = config if config is not None else PlannerConfig()
    start_pose = (start.x_m, start.y_m, _wrap(start.yaw_rad))
    goal_pose = (goal.x_m, goal.y_m, _wrap(goal.yaw_rad))
    checker = make_checker(obstacles, footprint, bounds, occupancy)
    if not checker.free(start_pose, config.clearance_m):
        return _failure("invalid_start")
    if not checker.free(goal_pose, config.clearance_m):
        return _failure("invalid_goal")
    yaw_bins = ceil(2 * pi / config.yaw_resolution_rad)

    def key(pose, direction, steering):
        return (
            round((pose[0] - bounds.x_min_m) / config.xy_resolution_m),
            round((pose[1] - bounds.y_min_m) / config.xy_resolution_m),
            round(pose[2] / (2 * pi) * yaw_bins) % yaw_bins,
            direction,
            steering,
        )

    field = (
        None
        if config.obstacle_heuristic_resolution_m is None
        else _ObstacleDistance(
            obstacles,
            footprint,
            bounds,
            (goal.x_m, goal.y_m),
            resolution_m=config.obstacle_heuristic_resolution_m,
            clearance_m=config.clearance_m,
            occupancy=occupancy,
        )
    )

    def heuristic(pose):
        value = max(
            hypot(pose[0] - goal.x_m, pose[1] - goal.y_m),
            abs(_wrap(pose[2] - goal.yaw_rad)) / config.curvature_limit_inv_m,
        )
        return value if field is None else max(value, field.distance(*pose[:2]))

    root = _Node(start_pose, 0.0, 0, 0, None, [])
    serial = count()
    queue = [(heuristic(start_pose), next(serial), root)]
    best = {key(start_pose, 0, 0): 0.0}
    expanded = 0
    # pruned children, superseded pops, cycles queued, shots capped, cycles generated
    counts = [0, 0, 0, 0, 0]
    passed_over = None  # (search cost + shot cost, node, connector, expansion): the cheapest capped-out shot
    root_shot = "none"

    def fallback(status):
        if passed_over is not None:
            result = _result(passed_over[1], passed_over[2], expanded, config, tuple(counts))
            return replace(result, shot_fallback=True, root_shot=root_shot)
        return replace(_failure(status, expanded, tuple(counts)), root_shot=root_shot)

    while queue and expanded < config.max_expansions:
        if deadline is not None and expanded % 256 == 0 and time.monotonic() > deadline:
            return fallback("timeout")
        if passed_over is not None and expanded - passed_over[3] >= config.shot_cap_patience:
            return fallback("expansion_limit")
        _, _, node = heapq.heappop(queue)
        if (
            node.cost
            > best.get(key(node.pose, node.direction, node.steering), float("inf"))
            + 1e-10
        ):
            counts[1] += 1
            continue
        expanded += 1
        if expanded == 1 or expanded % config.analytic_expansion_interval == 0:
            cap_length = (
                None if config.shot_cap_m is None
                else hypot(node.pose[0] - goal_pose[0], node.pose[1] - goal_pose[1]) + config.shot_cap_m
            )
            connector, _, over, over_cost = _connection(
                node.pose, goal_pose, node.direction, checker, config, cap_length
            )
            if connector is not None:
                if expanded == 1:
                    root_shot = "taken"
                return replace(_result(node, connector, expanded, config, tuple(counts)), root_shot=root_shot)
            if over is not None:
                counts[3] += 1
                if expanded == 1:
                    root_shot = "capped"
                total = node.cost + over_cost
                if passed_over is None or total < passed_over[0]:
                    passed_over = (total, node, over, expanded if passed_over is None else passed_over[3])
        for direction in (1, -1):
            for steering in (0, -1, 1):
                curvature = steering * config.curvature_limit_inv_m
                samples = _sample_arc(
                    node.pose,
                    config.primitive_length_m,
                    direction,
                    curvature,
                    checker,
                    config,
                )
                if samples is None:
                    continue
                cost = node.cost + config.primitive_length_m * (
                    (config.reverse_penalty if direction < 0 else 1)
                    + config.steering_penalty * abs(steering)
                )
                cost += config.steering_change_penalty_m * abs(steering - node.steering)
                if node.direction and node.direction != direction:
                    cost += config.gear_change_penalty_m
                state_key = key(samples[-1], direction, steering)
                if cost >= best.get(state_key, float("inf")) - 1e-10:
                    counts[0] += 1
                    continue
                best[state_key] = cost
                child = _Node(samples[-1], cost, direction, steering, node, samples)
                child.length_m = config.primitive_length_m
                heapq.heappush(
                    queue,
                    (
                        cost + config.heuristic_weight * heuristic(child.pose),
                        next(serial),
                        child,
                    ),
                )
        if config.turn_cycles:
            # D7b: (gear, steering) of the two halves; the second half reverses both.
            length = config.turn_cycle_m
            k = config.curvature_limit_inv_m
            for first_dir, first_steer in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
                second_dir, second_steer = -first_dir, -first_steer
                first = _sample_arc(node.pose, length, first_dir, first_steer * k, checker, config)
                if first is None:
                    continue
                cost1 = node.cost + length * (
                    (config.reverse_penalty if first_dir < 0 else 1) + config.steering_penalty
                )
                cost1 += config.steering_change_penalty_m * abs(first_steer - node.steering)
                if node.direction and node.direction != first_dir:
                    cost1 += config.gear_change_penalty_m
                second = _sample_arc(first[-1], length, second_dir, second_steer * k, checker, config)
                if second is None:
                    continue
                counts[4] += 1
                cost2 = cost1 + length * (
                    (config.reverse_penalty if second_dir < 0 else 1) + config.steering_penalty
                )
                cost2 += config.steering_change_penalty_m * abs(second_steer - first_steer)
                cost2 += config.gear_change_penalty_m
                state_key = key(second[-1], second_dir, second_steer)
                if cost2 >= best.get(state_key, float("inf")) - 1e-10:
                    counts[0] += 1
                    continue
                best[state_key] = cost2
                half = _Node(first[-1], cost1, first_dir, first_steer, node, first, cycle_half=True)
                half.length_m = length
                child = _Node(second[-1], cost2, second_dir, second_steer, half, second)
                child.length_m = length
                counts[2] += 1
                heapq.heappush(
                    queue,
                    (cost2 + config.heuristic_weight * heuristic(child.pose), next(serial), child),
                )
    return fallback("expansion_limit" if queue else "no_path")
