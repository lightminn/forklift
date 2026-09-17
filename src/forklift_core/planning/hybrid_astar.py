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
from math import acos, atan2, ceil, cos, hypot, pi, sin, sqrt

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

    def heuristic(pose):
        return max(
            hypot(pose[0] - goal.x_m, pose[1] - goal.y_m),
            abs(_wrap(pose[2] - goal.yaw_rad)) / config.curvature_limit_inv_m,
        )

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
