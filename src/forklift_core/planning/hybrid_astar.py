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
from collections.abc import Sequence
from dataclasses import dataclass
from itertools import count
from math import acos, atan2, ceil, cos, hypot, isfinite, pi, sin, sqrt

import numpy as np
from numpy.typing import NDArray

from forklift_core._validation import _finite_scalar

from .geometry import Bounds, Footprint, FootprintCollisionChecker, Pose2D, Rectangle


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


def _connection(pose, goal, previous_direction, checker, config):
    if (
        hypot(pose[0] - goal[0], pose[1] - goal[1]) < 1e-12
        and abs(_wrap(pose[2] - goal[2])) < 1e-12
    ):
        return [], [], [], 0.0
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
            candidates.append((cost, direction, word, lengths))
    candidates.sort(key=lambda candidate: candidate[0])
    for _, direction, word, lengths in candidates:
        current = pose
        samples, directions, curvatures = [], [], []
        for turn, length in zip(word, lengths, strict=True):
            if length < 1e-10:
                continue
            curvature = turn * direction * k
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
            return samples, directions, curvatures, sum(lengths)
    return None


@dataclass
class _Node:
    pose: tuple
    cost: float
    direction: int
    steering: int
    parent: object
    samples: list


def _result(node, connector, expanded, config):
    chain = []
    current = node
    while current.parent is not None:
        chain.append(current)
        current = current.parent
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
        len(chain) * config.primitive_length_m + length,
        expanded,
    )


def _failure(status, expanded=0):
    return PlanResult(
        False,
        status,
        np.empty((0, 3)),
        np.empty(0, dtype=np.int8),
        np.empty(0),
        0.0,
        expanded,
    )


def plan_hybrid_astar(
    start: Pose2D,
    goal: Pose2D,
    obstacles: Sequence[Rectangle],
    footprint: Footprint,
    bounds: Bounds,
    config: PlannerConfig | None = None,
) -> PlanResult:
    """Plan bounded, collision-checked forward/reverse motion in a static map.

    Invalid colliding endpoints return invalid_start/invalid_goal. Exhaustion
    returns no_path or expansion_limit; neither proves physical infeasibility.
    Malformed numerical inputs raise ValueError at their dataclass boundary.
    """
    config = config if config is not None else PlannerConfig()
    start_pose = (start.x_m, start.y_m, _wrap(start.yaw_rad))
    goal_pose = (goal.x_m, goal.y_m, _wrap(goal.yaw_rad))
    checker = FootprintCollisionChecker(obstacles, footprint, bounds)
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
    while queue and expanded < config.max_expansions:
        _, _, node = heapq.heappop(queue)
        if (
            node.cost
            > best.get(key(node.pose, node.direction, node.steering), float("inf"))
            + 1e-10
        ):
            continue
        expanded += 1
        if expanded == 1 or expanded % config.analytic_expansion_interval == 0:
            connector = _connection(
                node.pose, goal_pose, node.direction, checker, config
            )
            if connector is not None:
                return _result(node, connector, expanded, config)
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
                    continue
                best[state_key] = cost
                child = _Node(samples[-1], cost, direction, steering, node, samples)
                heapq.heappush(
                    queue,
                    (
                        cost + config.heuristic_weight * heuristic(child.pose),
                        next(serial),
                        child,
                    ),
                )
    return _failure("expansion_limit" if queue else "no_path", expanded)
