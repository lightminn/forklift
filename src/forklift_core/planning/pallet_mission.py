"""Synthetic ground-truth pallet transport geometry and path orchestration.

The sites are pallet centres, while every path pose references the rear axle.
This module neither detects pallets nor validates pocket clearance or physical
load handling. The simulation adapter supplies real asset bounds and executes
the distinct insertion, loading, transport, unloading, and withdrawal stages.
"""

import time
from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from math import ceil, cos, pi, sin

import numpy as np

from forklift_core._validation import _finite_scalar
from forklift_core.perception.pallet_geometry import (
    CARRIAGE_INSERTION_LIMIT_M,
    INSERTION_RESERVE_M,
    target_insertion_depth_m,
)

from .geometry import (
    Bounds,
    Footprint,
    Pose2D,
    Rectangle,
    collision_free_path,
    collision_free_pose,
)
from .hybrid_astar import PlannerConfig, PlanResult, plan_hybrid_astar

DEFAULT_TRANSPORT_CLEARANCE_M = 0.10
# Denser analytic-connection intervals tried, in order, when a mission search
# exhausts its expansions or finds no path. The search only reaches its goal
# through an exact analytic connection tried every N-th expansion, so whether a
# connectable node comes up on an N-th turn can flip with millimetres of goal
# change (G5 1020/1029, G2' seed 1, G2a seeds 4/7; 2026-10-02 planner/tracker
# plan). A search that succeeds at its own interval is never repeated.
FALLBACK_ANALYTIC_INTERVALS = (4, 2, 1)
# Tried first, at the original interval: a 0.2 m / 10 deg lattice can close its
# queue from a start it actually drove to while 0.1 m / 5 deg plans in about a
# second (second-evaluation seeds 2007, 2025; dev 1025). Ahead of the denser
# intervals it rescues the most bay and perturbed searches without losing any
# the intervals found (docs/plans/2026-10-02-second-eval-failure-fixes.md, P4).
# (xy_resolution_m, yaw_resolution_rad), or None for no lattice retry.
FALLBACK_FINE_LATTICE = (0.1, pi / 36)
# The extended ladder, tried only after every retry above has run out, so a
# search the earlier ladder solved keeps its path exactly
# (docs/plans/2026-10-03-transport-stage-fixes.md, T2). First the fine
# lattice again at this multiple of the caller's budget -- it has eight times
# the cells, and 30,000 ran out where 56,376 plans fourth-evaluation seed
# 4013's loaded transport; a factor, so a deliberately small budget stays
# small -- then the fine lattice with this primitive length: 0.25 m cannot
# leave seed 4020's start between two props, 0.10 m can at the same 0.10 m
# clearance. None/1 drops a step.
FALLBACK_FINE_BUDGET_FACTOR = 4
FALLBACK_FINE_PRIMITIVE_M = 0.10
# A start free at the clearance but inside the extra margin that encloses each
# primitive's swept footprint rejects every primitive, so the search ends after
# its root (fine-lattice dev seed 1006, 48 mrad off at a prop). Every retry at
# the same collision step is boxed in the same way; the whole ladder runs again
# at this step instead, whose smaller margin still encloses the swept footprint
# (docs/plans/2026-10-02-second-eval-failure-fixes.md, P6).
FALLBACK_BOXED_COLLISION_STEP_M = 0.01
_RETRIED_STATUSES = ("expansion_limit", "no_path")
DEFAULT_TRANSPORT_PRIMITIVE_LENGTH_M = 0.25


@dataclass
class SearchBudget:
    """A wall-clock budget for every search ladder on its own (priority-5 plan D7 CPU
    baseline: "재계획 시간 상한 20 s 는 사다리 전체에 적용한다"). Passed where a deadline
    goes, each _search turns it into a deadline from its own start, so one slow leg does
    not eat the next leg's time, and a ladder that ends after it -- even with a goal
    connection found past the last 256-expansion check -- fails as "timeout" (Codex D7
    3rd P2). ``spent_s`` collects every ladder's own wall time, in call order."""

    seconds: float
    spent_s: list = field(default_factory=list)


def _retries(config, extended=True):
    """The configs _search tries, in order, after config itself runs out."""
    fine = None
    if FALLBACK_FINE_LATTICE is not None:
        xy, yaw = FALLBACK_FINE_LATTICE
        fine = replace(
            config,
            xy_resolution_m=min(xy, config.xy_resolution_m),
            yaw_resolution_rad=min(yaw, config.yaw_resolution_rad),
        )
        # Not searched again when the config is already as fine; the extended
        # ladder below still uses it (Codex review, 2026-10-03).
        if fine != config:
            yield fine
    for denser in FALLBACK_ANALYTIC_INTERVALS:
        if denser < config.analytic_expansion_interval:
            yield replace(config, analytic_expansion_interval=denser)
    if not extended or fine is None:
        return
    if FALLBACK_FINE_BUDGET_FACTOR and FALLBACK_FINE_BUDGET_FACTOR > 1:
        fine = replace(
            fine, max_expansions=fine.max_expansions * FALLBACK_FINE_BUDGET_FACTOR
        )
        yield fine
    if (
        FALLBACK_FINE_PRIMITIVE_M is not None
        and FALLBACK_FINE_PRIMITIVE_M < fine.primitive_length_m
    ):
        yield replace(fine, primitive_length_m=FALLBACK_FINE_PRIMITIVE_M)


def _search(
    start, goal, obstacles, footprint, bounds, config, extended=True, occupancy=None, deadline=None
) -> PlanResult:
    """plan_hybrid_astar, retried on a finer lattice, then at denser analytic
    intervals, then (``extended``) on the fine lattice with more budget and a
    shorter primitive, if it runs out."""
    budget = deadline if isinstance(deadline, SearchBudget) else None
    if budget is not None:
        started = time.monotonic()
        deadline = started + float(budget.seconds)
    attempts = []
    closed = []  # configs whose queue closed (no_path), budget aside

    attempts_config = []

    def attempt(attempt_config):
        attempts_config.append(attempt_config)
        # Without a grid the call is exactly the pre-grid one.
        extra = {} if occupancy is None else {"occupancy": occupancy}
        if deadline is not None:
            extra["deadline"] = deadline
        result = plan_hybrid_astar(
            start, goal, obstacles, footprint, bounds, attempt_config, **extra
        )
        attempts.append(
            (
                attempt_config.analytic_expansion_interval,
                result.status,
                int(result.expanded_nodes),
                attempt_config.xy_resolution_m,
                attempt_config.yaw_resolution_rad,
                attempt_config.collision_step_m,
                attempt_config.primitive_length_m,
                attempt_config.max_expansions,
                int(getattr(result, "pruned_children", 0)),
                int(getattr(result, "superseded_pops", 0)),
            )
        )
        return result

    result = attempt(config)
    interval = config.analytic_expansion_interval
    if (
        result.status == "no_path"
        and result.expanded_nodes == 1
        and FALLBACK_BOXED_COLLISION_STEP_M is not None
        and FALLBACK_BOXED_COLLISION_STEP_M < config.collision_step_m
    ):
        config = replace(config, collision_step_m=FALLBACK_BOXED_COLLISION_STEP_M)
        result = attempt(config)
    for retry in _retries(config, extended):
        if result.status == "timeout":
            break
        if result.status == "no_path":
            closed.append(replace(attempts_config[-1], max_expansions=1))
        if result.success or result.status not in _RETRIED_STATUSES:
            break
        # A queue that closed closes again with more budget: skip it.
        if replace(retry, max_expansions=1) in closed:
            continue
        result = attempt(retry)
        interval = retry.analytic_expansion_interval
    if budget is not None:
        ended = time.monotonic()
        budget.spent_s.append(ended - started)
        if ended > deadline and result.status != "timeout":
            result = _timed_out(result)
    return replace(
        result, analytic_expansion_interval=interval, search_attempts=tuple(attempts)
    )


def _timed_out(result: PlanResult) -> PlanResult:
    """A result that came back after its budget: a timeout, its path dropped."""
    return replace(
        result, success=False, status="timeout", poses=np.empty((0, 3)),
        directions=np.empty(0, dtype=np.int8), curvatures_inv_m=np.empty(0), length_m=0.0,
    )


# Plan D7 CPU judgement (2026-10-08, judge_rerun.json, combination 3): the goal connection
# tries Reeds-Shepp words as well as Dubins, and a connection longer than the straight-line
# distance + 4 m waits for the search instead of being taken at once. Penalties and D7b
# stay at their defaults. Applied by the runner's --d7-planner (Isaac D7 off/on).
D7_PLANNER_OPTIONS = {"goal_connection": "reeds_shepp", "shot_cap_m": 4.0}


def make_transport_planner_config(**overrides: float | int) -> PlannerConfig:
    """Build mission/observation defaults while allowing explicit replay settings.

    Generic PlannerConfig defaults stay independent. Callers may override any
    planner field, including primitive length and expansion budget.
    """
    return replace(
        PlannerConfig(
            primitive_length_m=DEFAULT_TRANSPORT_PRIMITIVE_LENGTH_M,
            clearance_m=DEFAULT_TRANSPORT_CLEARANCE_M,
        ),
        **overrides,
    )


@dataclass(frozen=True)
class AssetSpec:
    """Actual asset URI and adapter-measured axis-aligned nominal dimensions."""

    uri: str
    length_m: float
    width_m: float
    height_m: float

    def __post_init__(self) -> None:
        if not isinstance(self.uri, str) or not self.uri.strip():
            raise ValueError("asset URI must be nonempty")
        for value in (self.length_m, self.width_m, self.height_m):
            if _finite_scalar(value, "asset dimension") <= 0:
                raise ValueError("asset dimensions must be positive")


@dataclass(frozen=True)
class PalletSite:
    """Pallet centre and fork insertion heading in the world frame."""

    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self) -> None:
        for value in (self.x_m, self.y_m, self.yaw_rad):
            _finite_scalar(value, "pallet site")


@dataclass(frozen=True)
class PlacedProp:
    asset: AssetSpec
    rectangle: Rectangle


@dataclass(frozen=True)
class TransportScenario:
    seed: int
    start_rear: Pose2D
    pickup: PalletSite
    destination: PalletSite
    props: tuple[PlacedProp, ...]
    bounds: Bounds


@dataclass(frozen=True)
class SyntheticMissionGeometry:
    """Explicit provisional/synthetic geometry, never measured robot calibration.

    Axle x=-0.34 m and fork tip x=0.95 m in the provisional base frame imply
    unloaded front=1.29 m. Unloaded half-width 0.36 m encloses the steered
    tire corners: 0.255 + 0.135*sin(0.45) + 0.05*cos(0.45) = 0.358743 m.
    The 0.60 m pallet and 0.36 m penetration imply
    inserted axle 1.23 m behind pallet centre and loaded front=1.53 m.
    """

    unloaded_footprint: Footprint = Footprint(1.29, 0.17, 0.36)
    pallet_depth_m: float = 0.60
    pallet_width_m: float = 0.80
    axle_to_fork_tip_m: float = 1.29  # Independent of the unloaded front envelope.
    approach_gap_m: float = 0.10  # Standoff, not planner clearance_m.
    alignment_straight_m: float = 0.80
    delivery_straight_m: float = 0.70
    extraction_m: float = 0.65
    withdrawal_m: float = 0.55
    spawn_clearance_m: float = 0.12
    carriage_limit_m: float = CARRIAGE_INSERTION_LIMIT_M  # Pass the run's model limit.
    insertion_reserve_m: float = INSERTION_RESERVE_M  # Policy; diagnostic sweeps only.

    inserted_offset_m: float = field(init=False)
    approach_offset_m: float = field(init=False)
    prealign_offset_m: float = field(init=False)
    predelivery_offset_m: float = field(init=False)
    loaded_footprint: Footprint = field(init=False)

    def __post_init__(self) -> None:
        d = target_insertion_depth_m(
            self.pallet_depth_m, self.carriage_limit_m, self.insertion_reserve_m
        )
        inserted = self.axle_to_fork_tip_m + self.pallet_depth_m / 2 - d
        approach = (
            self.axle_to_fork_tip_m + self.pallet_depth_m / 2 + self.approach_gap_m
        )
        prealign = approach + self.alignment_straight_m
        predelivery = inserted + self.delivery_straight_m
        loaded = Footprint(
            max(self.unloaded_footprint.front_m, inserted + self.pallet_depth_m / 2),
            self.unloaded_footprint.rear_m,
            max(self.unloaded_footprint.half_width_m, self.pallet_width_m / 2),
        )
        object.__setattr__(self, "inserted_offset_m", inserted)
        object.__setattr__(self, "approach_offset_m", approach)
        object.__setattr__(self, "prealign_offset_m", prealign)
        object.__setattr__(self, "predelivery_offset_m", predelivery)
        object.__setattr__(self, "loaded_footprint", loaded)

        for value in (
            self.pallet_depth_m,
            self.pallet_width_m,
            self.axle_to_fork_tip_m,
            self.approach_gap_m,
            self.alignment_straight_m,
            self.delivery_straight_m,
            self.inserted_offset_m,
            self.approach_offset_m,
            self.prealign_offset_m,
            self.extraction_m,
            self.predelivery_offset_m,
            self.withdrawal_m,
            self.spawn_clearance_m,
        ):
            if _finite_scalar(value, "mission geometry") <= 0:
                raise ValueError("mission geometry distances must be positive")
        if not self.prealign_offset_m > self.approach_offset_m > self.inserted_offset_m:
            raise ValueError("prealign, approach, inserted offsets must be decreasing")
        if self.predelivery_offset_m <= self.inserted_offset_m:
            raise ValueError("predelivery must precede the inserted delivery pose")


@dataclass(frozen=True)
class MissionPlan:
    """Every runnable path, or no paths if any stage failed.

    Load after insert, unload after transport. Stop at every stage boundary
    and every gear-change cusp. Pocket collision checks are adapter-owned.

    `return_home` is planned only when the caller asks for it, so a mission
    that does not return keeps exactly the five stages it had before.
    """

    success: bool
    status: str
    approach: PlanResult | None = None
    insert: PlanResult | None = None
    extract: PlanResult | None = None
    transport: PlanResult | None = None
    withdraw: PlanResult | None = None
    return_home: PlanResult | None = None


def site_poses(
    site: PalletSite,
    geometry: SyntheticMissionGeometry | None = None,
) -> dict[str, Pose2D]:
    """Convert a pallet centre into rear-axle poses along its insertion axis."""
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    offsets = {
        "inserted": geometry.inserted_offset_m,
        "approach": geometry.approach_offset_m,
        "prealign": geometry.prealign_offset_m,
        "extracted": geometry.inserted_offset_m + geometry.extraction_m,
        "delivery": geometry.inserted_offset_m,
        "predelivery": geometry.predelivery_offset_m,
        "withdrawn": geometry.inserted_offset_m + geometry.withdrawal_m,
    }
    return {
        name: Pose2D(
            site.x_m - distance * cos(site.yaw_rad),
            site.y_m - distance * sin(site.yaw_rad),
            site.yaw_rad,
        )
        for name, distance in offsets.items()
    }


def _swept_reservation(start: Pose2D, end: Pose2D, footprint: Footprint):
    distance = float(np.hypot(end.x_m - start.x_m, end.y_m - start.y_m))
    centre = Pose2D((start.x_m + end.x_m) / 2, (start.y_m + end.y_m) / 2, start.yaw_rad)
    swept = Footprint(
        footprint.front_m + distance / 2,
        footprint.rear_m + distance / 2,
        footprint.half_width_m,
    )
    return centre, swept


def destination_reservations(
    destination: PalletSite, geometry: SyntheticMissionGeometry | None = None
) -> list[tuple[Pose2D, Footprint]]:
    """Body rectangles swept by the loaded delivery and the unloaded withdrawal.

    Each entry is a centre pose and a footprint around it. Anything placed near
    a destination must leave these free; they do not guarantee a route there.
    """
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    poses = site_poses(destination, geometry)
    return [
        _swept_reservation(
            poses["predelivery"], poses["delivery"], geometry.loaded_footprint
        ),
        _swept_reservation(
            poses["delivery"], poses["withdrawn"], geometry.unloaded_footprint
        ),
    ]


def _reservations(start, pickup, destination, geometry):
    pickup_poses = site_poses(pickup, geometry)
    return [
        (start, geometry.unloaded_footprint),
        _swept_reservation(
            pickup_poses["prealign"],
            pickup_poses["inserted"],
            geometry.unloaded_footprint,
        ),
        _swept_reservation(
            pickup_poses["inserted"],
            pickup_poses["extracted"],
            geometry.loaded_footprint,
        ),
        *destination_reservations(destination, geometry),
    ]


def make_scenario(
    seed: int,
    assets: Sequence[AssetSpec],
    obstacle_count: int = 4,
    *,
    geometry: SyntheticMissionGeometry | None = None,
    max_attempts: int = 2000,
) -> TransportScenario:
    """Seeded placement in the synthetic clear factory bay, with bounded work.

    Props reserve their measured horizontal asset bounds, and the whole swept
    docking/extraction/withdrawal body rectangles are reserved analytically.
    Failed placement raises ValueError without silently replacing the seed.
    Geometric reservations do not guarantee that Hybrid A* finds a route.
    """
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("seed must be a nonnegative integer")
    if (
        isinstance(obstacle_count, bool)
        or not isinstance(obstacle_count, int)
        or obstacle_count < 0
    ):
        raise ValueError("obstacle_count must be a nonnegative integer")
    if (
        isinstance(max_attempts, bool)
        or not isinstance(max_attempts, int)
        or max_attempts <= 0
    ):
        raise ValueError("max_attempts must be a positive integer")
    if obstacle_count and not assets:
        raise ValueError("assets are required to place obstacles")
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    rng = np.random.default_rng(seed)
    bounds = Bounds(-3.0, 4.7, -1.75, 3.05)
    start = Pose2D(-2.34, 0, 0)
    for _ in range(max_attempts):
        pickup = PalletSite(
            float(rng.uniform(2.6, 3.6)),
            float(rng.uniform(-0.2, 1.5)),
            float(rng.uniform(-12, 12) * pi / 180),
        )
        destination = PalletSite(
            float(rng.uniform(-0.2, 0.9)),
            float(rng.uniform(-0.65, 1.9)),
            float(rng.uniform(-10, 10) * pi / 180),
        )
        reservations = _reservations(start, pickup, destination, geometry)
        if all(
            collision_free_pose(
                pose, [], footprint, bounds, margin_m=geometry.spawn_clearance_m
            )
            for pose, footprint in reservations
        ):
            break
    else:
        raise ValueError(f"could not place pallet sites for seed {seed}")
    props = []
    for _ in range(max_attempts):
        if len(props) == obstacle_count:
            return TransportScenario(
                seed, start, pickup, destination, tuple(props), bounds
            )
        asset = assets[int(rng.integers(len(assets)))]
        rectangle = Rectangle(
            float(rng.uniform(-0.8, 3.4)),
            float(rng.uniform(-1.1, 2.4)),
            asset.length_m,
            asset.width_m,
            float(rng.uniform(-pi, pi)),
        )
        prop_pose = Pose2D(rectangle.x_m, rectangle.y_m, rectangle.yaw_rad)
        prop_footprint = Footprint(
            asset.length_m / 2, asset.length_m / 2, asset.width_m / 2
        )
        if not collision_free_pose(
            prop_pose,
            [p.rectangle for p in props],
            prop_footprint,
            bounds,
            margin_m=geometry.spawn_clearance_m,
        ):
            continue
        if any(
            not collision_free_pose(
                pose,
                [rectangle],
                footprint,
                bounds,
                margin_m=geometry.spawn_clearance_m,
            )
            for pose, footprint in reservations
        ):
            continue
        props.append(PlacedProp(asset, rectangle))
    if len(props) == obstacle_count:
        return TransportScenario(seed, start, pickup, destination, tuple(props), bounds)
    raise ValueError(
        f"could not place {obstacle_count} props for seed {seed} after {max_attempts} attempts"
    )


def _straight_plan(start, goal, direction, obstacles, footprint, bounds, clearance_m, occupancy=None):
    distance = float(np.hypot(goal.x_m - start.x_m, goal.y_m - start.y_m))
    count = max(1, ceil(distance / 0.04))
    fractions = np.linspace(0, 1, count + 1)
    poses = np.column_stack(
        (
            start.x_m + fractions * (goal.x_m - start.x_m),
            start.y_m + fractions * (goal.y_m - start.y_m),
            np.full(count + 1, start.yaw_rad),
        )
    )
    if not collision_free_path(
        poses, obstacles, footprint, bounds, margin_m=clearance_m, max_step_m=0.04,
        occupancy=occupancy,
    ):
        return PlanResult(
            False,
            "straight_collision",
            np.empty((0, 3)),
            np.empty(0, dtype=np.int8),
            np.empty(0),
            0.0,
            0,
        )
    return PlanResult(
        True,
        "success",
        poses,
        np.full(count + 1, direction, dtype=np.int8),
        np.zeros(count + 1),
        distance,
        0,
    )


def _append_straight(first, second):
    poses = np.concatenate((first.poses, second.poses[1:]))
    directions = np.concatenate((first.directions, second.directions[1:]))
    curvatures = np.concatenate((first.curvatures_inv_m, second.curvatures_inv_m[1:]))
    if len(poses) > 1:
        directions[0], curvatures[0] = directions[1], curvatures[1]
    return PlanResult(
        True,
        "success",
        poses,
        directions,
        curvatures,
        first.length_m + second.length_m,
        first.expanded_nodes + second.expanded_nodes,
        analytic_expansion_interval=first.analytic_expansion_interval,
        search_attempts=first.search_attempts,
        pruned_children=first.pruned_children + second.pruned_children,
        superseded_pops=first.superseded_pops + second.superseded_pops,
        cycles_queued=first.cycles_queued + second.cycles_queued,
        cycles_in_path=first.cycles_in_path + second.cycles_in_path,
        shots_capped=first.shots_capped + second.shots_capped,
        shot_fallback=first.shot_fallback or second.shot_fallback,
        root_shot=first.root_shot,
        cycles_generated=first.cycles_generated + second.cycles_generated,
    )


def plan_observation_leg(
    scenario: TransportScenario,
    waypoint: Pose2D,
    config: PlannerConfig | None = None,
    *,
    geometry: SyntheticMissionGeometry | None = None,
    start_rear: Pose2D | None = None,
    pickup_bounds: Bounds | None = None,
    extended: bool = True,
    occupancy=None,
    pickup_obstacle: Rectangle | None = None,
    deadline=None,
) -> PlanResult:
    """Plan a separate leg to the observation waypoint at full clearance.

    occupancy / pickup_obstacle (priority-5 plan): a LiDAR occupancy grid and
    the rectangle that stands for the unrecognised pallet (the pickup zone
    prior) instead of scenario.pickup; pass a scenario without props then.

    Like plan_transport's target/obstacle split, scenario.pickup is an obstacle,
    not the goal of this leg.
    start_rear optionally replaces scenario.start_rear with the measured rear pose.
    pickup_bounds optionally confines the leg as in plan_transport.
    extended=False keeps the search to the earlier ladder: the runner tries
    every candidate that way first, so a candidate that only the extended
    ladder reaches never jumps ahead of one the earlier ladder planned.
    """
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    config = config if config is not None else make_transport_planner_config()
    props = [prop.rectangle for prop in scenario.props]
    pallet = pickup_obstacle if pickup_obstacle is not None else Rectangle(
        scenario.pickup.x_m,
        scenario.pickup.y_m,
        geometry.pallet_depth_m,
        geometry.pallet_width_m,
        scenario.pickup.yaw_rad,
    )
    return _search(
        start_rear if start_rear is not None else scenario.start_rear,
        waypoint,
        props + [pallet],
        geometry.unloaded_footprint,
        pickup_bounds if pickup_bounds is not None else scenario.bounds,
        config,
        extended,
        occupancy,
        deadline,
    )


def final_straight_prefix(path: PlanResult, keep_m: float) -> PlanResult | None:
    """The approach up to where its final forward straight still has keep_m left.

    Online SLAM plan v3.6: the truck stops there to see the pallet again
    before docking, so only that last straight runs on odometry. The final
    straight is found by walking back over forward, zero-curvature segments
    (so a 0.7999999 m straight still counts -- Codex v3.6 P1); the cut is at
    its start, or further along when it is longer than keep_m. None when the
    path does not end in such a straight at least keep_m long.
    """
    poses = np.asarray(path.poses, dtype=float)
    directions = np.asarray(path.directions)
    curvatures = np.asarray(path.curvatures_inv_m)
    if len(poses) < 3:
        return None
    steps = np.hypot(*np.diff(poses[:, :2], axis=0).T)
    start = len(poses) - 1
    while start > 0 and directions[start] > 0 and abs(curvatures[start]) < 1e-9:
        start -= 1
    remaining = np.concatenate((np.cumsum(steps[::-1])[::-1], [0.0]))
    tolerance = 1e-6
    if remaining[start] < keep_m - tolerance:
        return None
    cut = start
    while cut + 1 < len(poses) and remaining[cut + 1] >= keep_m - tolerance:
        cut += 1
    if cut < 1:
        # A path that is all one forward straight still needs somewhere to
        # drive before the capture (Codex v3.6 re-review P2); none here.
        return None
    return replace(
        path,
        poses=poses[: cut + 1],
        directions=directions[: cut + 1],
        curvatures_inv_m=curvatures[: cut + 1],
        length_m=float(steps[:cut].sum()),
    )


def straight_from_pose(
    current: Pose2D,
    line_start: Pose2D,
    line_end: Pose2D,
    *,
    max_lateral_m: float,
    max_yaw_rad: float,
    min_length_m: float,
    step_m: float = 0.04,
) -> tuple[PlanResult | None, dict]:
    """The final straight re-drawn from where the truck stands (plan v3.6).

    After the near capture the new pallet estimate moves the straight by a few
    centimetres; a Hybrid A* search from an offset start makes a manoeuvre
    with gear changes (Codex v3.6 P2), which would then run held. Instead the
    truck's pose is projected on the new line and the path is the rest of
    that line; the tracker takes out the offset. None (with the measured
    offsets) when the truck is too far off the line, turned too far, or has
    less than min_length_m of it left.
    """
    heading = np.array([cos(line_start.yaw_rad), sin(line_start.yaw_rad)])
    normal = np.array([-heading[1], heading[0]])
    offset = np.array([current.x_m - line_start.x_m, current.y_m - line_start.y_m])
    along = float(offset @ heading)
    lateral = float(offset @ normal)
    yaw = float(np.arctan2(np.sin(current.yaw_rad - line_start.yaw_rad), np.cos(current.yaw_rad - line_start.yaw_rad)))
    total = float(np.hypot(line_end.x_m - line_start.x_m, line_end.y_m - line_start.y_m))
    # Behind the line start the straight begins at the truck's own projection,
    # not at the start: cutting a negative along to 0 began the path ahead of
    # the truck (plan v10 D7c, Codex v10 2nd P1-4).
    left = total - along
    record = {"along_m": along, "lateral_m": lateral, "yaw_rad": yaw, "length_left_m": left}
    if abs(lateral) > max_lateral_m or abs(yaw) > max_yaw_rad or left < min_length_m:
        return None, record
    begin = np.array([line_start.x_m, line_start.y_m]) + along * heading
    count = max(1, ceil(left / step_m))
    fractions = np.linspace(0.0, 1.0, count + 1)
    xy = begin + fractions[:, None] * (np.array([line_end.x_m, line_end.y_m]) - begin)
    poses = np.column_stack((xy, np.full(count + 1, line_start.yaw_rad)))
    return (
        PlanResult(
            True,
            "success",
            poses,
            np.ones(count + 1, dtype=np.int8),
            np.zeros(count + 1),
            left,
            0,
        ),
        record,
    )


def plan_transport_leg(
    scenario: TransportScenario,
    start_rear: Pose2D,
    config: PlannerConfig | None = None,
    *,
    geometry: SyntheticMissionGeometry | None = None,
    travel_config: PlannerConfig | None = None,
    occupancy=None,
    deadline=None,
) -> PlanResult:
    """The loaded transport leg alone, from ``start_rear`` to the delivery pose.

    The same search and final straight as plan_transport's transport stage, for
    replanning from the measured pose when a gear cusp stops out of heading
    (docs/plans/2026-10-03-transport-stage-fixes.md). The carried pallet is
    part of the loaded footprint; the props and bounds stay obstacles.
    """
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    config = config if config is not None else make_transport_planner_config()
    travel_config = travel_config if travel_config is not None else config
    props = [prop.rectangle for prop in scenario.props]
    destination = site_poses(scenario.destination, geometry)
    search = _search(
        start_rear,
        destination["predelivery"],
        props,
        geometry.loaded_footprint,
        scenario.bounds,
        travel_config,
        occupancy=occupancy,
        deadline=deadline,
    )
    if not search.success:
        return search
    tail = _straight_plan(
        destination["predelivery"],
        destination["delivery"],
        1,
        props,
        geometry.loaded_footprint,
        scenario.bounds,
        config.clearance_m,
        occupancy,
    )
    if not tail.success:
        return tail
    return _append_straight(search, tail)


def plan_docking_reapproach(
    scenario: TransportScenario,
    start_rear: Pose2D,
    line_start: Pose2D,
    config: PlannerConfig | None = None,
    *,
    geometry: SyntheticMissionGeometry | None = None,
    keep_m: float,
    occupancy=None,
    deadline=None,
    behind_m: float = 4.0,
    lateral_m: float = 1.5,
    ahead_margin_m: float = 0.2,
    max_length_m: float = 6.0,
) -> tuple[PlanResult | None, dict]:
    """Back to the docking line's start from where the truck stands (plan D7c ③).

    A loaded search to ``line_start``, accepted only when every rear-axle pose
    stays in the line-start frame box -- along [-behind_m, keep_m + ahead_margin_m]
    (the docking goal is keep_m ahead; the margin is for a start at the goal),
    lateral within +-lateral_m -- and the path is at most max_length_m. None with
    the reason in the record ("no_path", "leaves_box", "too_long") otherwise.
    """
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    config = config if config is not None else make_transport_planner_config()
    props = [prop.rectangle for prop in scenario.props]
    result = _search(
        start_rear,
        line_start,
        props,
        geometry.loaded_footprint,
        scenario.bounds,
        config,
        occupancy=occupancy,
        deadline=deadline,
    )
    directions = np.asarray(result.directions)
    record = {"status": result.status, "refused": None, "length_m": None, "along_min_m": None,
              "along_max_m": None, "lateral_max_m": None, "expansions": int(result.expanded_nodes),
              "search_attempts": [list(a) for a in (result.search_attempts or ())],
              "gear_changes": int((directions[1:] != directions[:-1]).sum()) if len(directions) > 1 else 0}
    if not result.success:
        record["refused"] = "no_path"
        return None, record
    heading = np.array([cos(line_start.yaw_rad), sin(line_start.yaw_rad)])
    offsets = np.asarray(result.poses, dtype=float)[:, :2] - np.array([line_start.x_m, line_start.y_m])
    along = offsets @ heading
    lateral = offsets @ np.array([-heading[1], heading[0]])
    record.update(length_m=float(result.length_m), along_min_m=float(along.min()), along_max_m=float(along.max()),
                  lateral_max_m=float(np.abs(lateral).max()))
    if along.min() < -behind_m or along.max() > keep_m + ahead_margin_m or np.abs(lateral).max() > lateral_m:
        record["refused"] = "leaves_box"
    elif result.length_m > max_length_m:
        record["refused"] = "too_long"
    return (None if record["refused"] else result), record


def plan_return_leg(
    scenario: TransportScenario,
    start_rear: Pose2D,
    return_to: Pose2D,
    config: PlannerConfig | None = None,
    *,
    geometry: SyntheticMissionGeometry | None = None,
    travel_config: PlannerConfig | None = None,
    occupancy=None,
    deadline=None,
) -> PlanResult:
    """The unloaded return leg alone, as plan_transport's return_home stage.

    For replanning from the measured pose after a SLAM correction is applied
    at the destination (docs/plans/2026-10-04-online-slam-closed-loop.md); the
    delivered pallet is an obstacle, as in plan_transport.
    """
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    config = config if config is not None else make_transport_planner_config()
    travel_config = travel_config if travel_config is not None else config
    props = [prop.rectangle for prop in scenario.props]
    delivered = Rectangle(
        scenario.destination.x_m,
        scenario.destination.y_m,
        geometry.pallet_depth_m,
        geometry.pallet_width_m,
        scenario.destination.yaw_rad,
    )
    return _search(
        start_rear,
        return_to,
        props + [delivered],
        geometry.unloaded_footprint,
        scenario.bounds,
        travel_config,
        occupancy=occupancy,
        deadline=deadline,
    )


def plan_transport(
    scenario: TransportScenario,
    config: PlannerConfig | None = None,
    *,
    geometry: SyntheticMissionGeometry | None = None,
    target_pickup: PalletSite | None = None,
    start_rear: Pose2D | None = None,
    return_to: Pose2D | None = None,
    pickup_bounds: Bounds | None = None,
    travel_config: PlannerConfig | None = None,
    trace: list | None = None,
    occupancy=None,
    pickup_obstacle: Rectangle | None = None,
    docking_occupancy=None,
    deadline=None,
) -> MissionPlan:
    """Plan all stages with exact final straight approaches and loaded geometry.

    The initial pallet is an obstacle only for approach. During insertion and
    withdrawal, pocket geometry/contact must be checked by the adapter; other
    props and world bounds always remain obstacles. Approach clearance is
    capped at half of geometry.approach_gap_m, the fork-to-pallet stopping gap
    (0.05 m for the default 0.10 m gap).
    Other stages use the supplied clearance (default 0.10 m).
    target_pickup optionally supplies an estimated goal while the pallet
    obstacle stays at scenario.pickup (ground truth). start_rear optionally
    replaces scenario.start_rear with the rear-axle pose after observation.
    return_to optionally adds a final unloaded leg from the withdrawn pose
    back to that rear-axle pose; pass scenario.start_rear to return to where
    the mission began. The delivered pallet becomes an obstacle for that leg,
    because the forks are clear of it once withdrawal has finished.
    pickup_bounds optionally confines approach, insertion and extraction to a
    smaller region than scenario.bounds, which the remaining stages keep. On a
    large floor this keeps the pickup search as tight as in the original bay.
    travel_config optionally replaces config for the two long Hybrid A* legs,
    transport and return_home, e.g. to add the obstacle heuristic there only.
    trace optionally receives one dict per planning step, in order and up to
    the step that failed -- search and appended straight parts separately --
    because a failed MissionPlan keeps no paths. Anything with ``append``
    works; the plan returned is the same with or without it.
    occupancy / pickup_obstacle (priority-5 plan): a LiDAR occupancy grid
    checked with every stage, and the perceived pallet rectangle in place of
    scenario.pickup as the approach obstacle. docking_occupancy, when given,
    replaces occupancy from the approach straight on -- typically a
    grid_collision.SplitOccupancy: the body on the full grid, the forks (or
    the carried pallet) on the grid with the perceived pallet's band cleared,
    since the forks must enter what the LiDAR sees as the pallet (plan D5).
    The runner replans the travel legs on the live grid when they start.
    """
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    config = config if config is not None else make_transport_planner_config()
    travel_config = travel_config if travel_config is not None else config

    def note(stage: str, result: PlanResult) -> PlanResult:
        if trace is not None:
            directions = np.asarray(result.directions)
            trace.append(
                {
                    "stage": stage,
                    "status": result.status,
                    "length_m": float(result.length_m),
                    "expansions": int(result.expanded_nodes),
                    "analytic_expansion_interval": result.analytic_expansion_interval,
                    # Every search behind this stage; "expansions" is the last one's.
                    "search_attempts": [
                        list(entry) for entry in result.search_attempts
                    ],
                    "gear_changes": int(np.count_nonzero(np.diff(directions)))
                    if directions.size
                    else 0,
                }
            )
        return result

    props = [prop.rectangle for prop in scenario.props]
    pickup = site_poses(
        target_pickup if target_pickup is not None else scenario.pickup, geometry
    )
    destination = site_poses(scenario.destination, geometry)
    pallet = pickup_obstacle if pickup_obstacle is not None else Rectangle(
        scenario.pickup.x_m,
        scenario.pickup.y_m,
        geometry.pallet_depth_m,
        geometry.pallet_width_m,
        scenario.pickup.yaw_rad,
    )
    near_bounds = pickup_bounds if pickup_bounds is not None else scenario.bounds
    approach_config = replace(
        config, clearance_m=min(config.clearance_m, geometry.approach_gap_m / 2)
    )
    approach = note(
        "approach_search",
        _search(
            start_rear if start_rear is not None else scenario.start_rear,
            pickup["prealign"],
            props + [pallet],
            geometry.unloaded_footprint,
            near_bounds,
            approach_config,
            occupancy=occupancy,
            deadline=deadline,
        ),
    )
    if not approach.success:
        return MissionPlan(False, f"approach:{approach.status}")
    approach_tail = note(
        "approach_straight",
        _straight_plan(
            pickup["prealign"],
            pickup["approach"],
            1,
            props + [pallet],
            geometry.unloaded_footprint,
            near_bounds,
            approach_config.clearance_m,
            docking_occupancy if docking_occupancy is not None else occupancy,
        ),
    )
    if not approach_tail.success:
        return MissionPlan(False, f"approach:{approach_tail.status}")
    approach = _append_straight(approach, approach_tail)
    # On a grid (priority-5 D3) the marks are already swollen by the placement
    # error, so the docking straights keep the approach's clearance instead of
    # adding the full one on top (L3c seed 4: a prop 0.5 m beside the insert
    # line failed only at 0.10 m). Without a grid, as before.
    docking_clearance = approach_config.clearance_m if occupancy is not None else config.clearance_m
    insert = note(
        "insert",
        _straight_plan(
            pickup["approach"],
            pickup["inserted"],
            1,
            props,
            geometry.unloaded_footprint,
            near_bounds,
            docking_clearance,
            docking_occupancy if docking_occupancy is not None else occupancy,
        ),
    )
    if not insert.success:
        return MissionPlan(False, f"insert:{insert.status}")
    extract = note(
        "extract",
        _straight_plan(
            pickup["inserted"],
            pickup["extracted"],
            -1,
            props,
            geometry.loaded_footprint,
            near_bounds,
            # The lifted pallet backs out along the footprint it stood on, so its
            # gap to a neighbour exists by construction; on a grid no clearance
            # is added to the swelling (L3c seed 4: a prop beside the pallet).
            0.0 if occupancy is not None else config.clearance_m,
            docking_occupancy if docking_occupancy is not None else occupancy,
        ),
    )
    if not extract.success:
        return MissionPlan(False, f"extract:{extract.status}")
    transport = note(
        "transport_search",
        _search(
            pickup["extracted"],
            destination["predelivery"],
            props,
            geometry.loaded_footprint,
            scenario.bounds,
            travel_config,
            occupancy=docking_occupancy if docking_occupancy is not None else occupancy,
            deadline=deadline,
        ),
    )
    if transport.status == "invalid_start" and occupancy is not None:
        # The extracted pose sits where the pallet was: within the travel
        # clearance of a grid mark beside it (L3c seed 4). As the runner's
        # replans do, once more with no clearance -- the swelling holds the
        # placement error and the permission guards every tick.
        transport = note(
            "transport_search_tight",
            _search(
                pickup["extracted"],
                destination["predelivery"],
                props,
                geometry.loaded_footprint,
                scenario.bounds,
                replace(travel_config, clearance_m=0.0),
                occupancy=docking_occupancy if docking_occupancy is not None else occupancy,
                deadline=deadline,
            ),
        )
    if not transport.success:
        return MissionPlan(False, f"transport:{transport.status}")
    transport_tail = note(
        "transport_straight",
        _straight_plan(
            destination["predelivery"],
            destination["delivery"],
            1,
            props,
            geometry.loaded_footprint,
            scenario.bounds,
            config.clearance_m,
            docking_occupancy if docking_occupancy is not None else occupancy,
        ),
    )
    if not transport_tail.success:
        return MissionPlan(False, f"transport:{transport_tail.status}")
    transport = _append_straight(transport, transport_tail)
    withdraw = note(
        "withdraw",
        _straight_plan(
            destination["delivery"],
            destination["withdrawn"],
            -1,
            props,
            geometry.unloaded_footprint,
            scenario.bounds,
            config.clearance_m,
            docking_occupancy if docking_occupancy is not None else occupancy,
        ),
    )
    if not withdraw.success:
        return MissionPlan(False, f"withdraw:{withdraw.status}")
    if return_to is None:
        return MissionPlan(
            True, "success", approach, insert, extract, transport, withdraw
        )
    delivered = Rectangle(
        scenario.destination.x_m,
        scenario.destination.y_m,
        geometry.pallet_depth_m,
        geometry.pallet_width_m,
        scenario.destination.yaw_rad,
    )
    return_home = note(
        "return_home",
        _search(
            destination["withdrawn"],
            return_to,
            props + [delivered],
            geometry.unloaded_footprint,
            scenario.bounds,
            travel_config,
            occupancy=docking_occupancy if docking_occupancy is not None else occupancy,
            deadline=deadline,
        ),
    )
    if not return_home.success:
        return MissionPlan(False, f"return_home:{return_home.status}")
    return MissionPlan(
        True,
        "success",
        approach,
        insert,
        extract,
        transport,
        withdraw,
        return_home,
    )
