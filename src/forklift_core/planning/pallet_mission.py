"""Synthetic ground-truth pallet transport geometry and path orchestration.

The sites are pallet centres, while every path pose references the rear axle.
This module neither detects pallets nor validates pocket clearance or physical
load handling. The simulation adapter supplies real asset bounds and executes
the distinct insertion, loading, transport, unloading, and withdrawal stages.
"""

import time
from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from math import ceil, cos, floor, inf, pi, sin

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
from .grid_collision import make_checker
from .hybrid_astar import PlannerConfig, PlanResult, _sample_arc, _wrap, obstacle_heuristic_field, plan_hybrid_astar

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


# Stage tags on a prop's asset uri (docs/plans/2026-10-22-slot-anchored-neighbours.md 2.4): a session's delivered
# neighbour is two props -- SEARCH_TAG at its estimate for every search and straight, SLOT_TAG at its slot in the goal's
# frame for the two straights inside the slot (the delivery straight and the withdraw). Untagged props are in both.
SEARCH_TAG, SLOT_TAG = "@search", "@slot"


def scenario_props(scenario: TransportScenario, stage: str = "search") -> list[Rectangle]:
    """The obstacle rectangles of ``scenario`` a check at ``stage`` sees: "search" leaves out the slot-stage props,
    "slot" (the delivery straight and the withdraw) the search-stage ones. A scene without tags gives one list."""
    if stage not in ("search", "slot"):
        raise ValueError(f"unknown stage {stage!r}")
    drop = SLOT_TAG if stage == "search" else SEARCH_TAG
    return [p.rectangle for p in scenario.props if not p.asset.uri.endswith(drop)]


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
        exit_prefix=first.exit_prefix,  # the transport's exit prefix (ExitPrefix) through its delivery straight
    )


@dataclass(frozen=True)
class ExitPrefix:
    """The straight reverse a loaded transport leg starts with (docs/plans/2026-10-22-riibotics-trailer-exit.md D1):
    until the loaded footprint keeps ``clearance_m`` from every rectangle of ``obstacles`` -- the walls the obstacle
    layer's LiDAR sees, whose band (about 0.2 m) the planner's clearance (0.10 m) does not keep -- in ``step_m`` steps up to
    ``cap_m``. The Riibotics trailer stopped when the transport's first turn swung the pallet, still in the trailer
    mouth, into that band (t1_trailer, 2026-10-21)."""

    obstacles: tuple
    clearance_m: float
    step_m: float = 0.05
    cap_m: float = 1.0

    def __post_init__(self) -> None:
        obstacles = tuple(self.obstacles)
        if not all(isinstance(r, Rectangle) for r in obstacles):
            raise TypeError("exit obstacles must be Rectangles")
        object.__setattr__(self, "obstacles", obstacles)
        for name in ("clearance_m", "step_m", "cap_m"):
            if not np.isfinite(getattr(self, name)):
                raise ValueError(f"exit prefix {name} must be finite")
        if self.clearance_m <= 0 or self.step_m <= 0 or self.cap_m < 0:
            raise ValueError("exit prefix clearance and step must be positive, its cap non-negative")


def _footprint_corners(pose: Pose2D, footprint: Footprint) -> np.ndarray:
    c, s = cos(pose.yaw_rad), sin(pose.yaw_rad)
    local = ((footprint.front_m, footprint.half_width_m), (-footprint.rear_m, footprint.half_width_m),
             (-footprint.rear_m, -footprint.half_width_m), (footprint.front_m, -footprint.half_width_m))
    return np.array([(pose.x_m + c * u - s * v, pose.y_m + s * u + c * v) for u, v in local])


def _rectangle_corners(r: Rectangle) -> np.ndarray:
    c, s = cos(r.yaw_rad), sin(r.yaw_rad)
    hl, hw = r.length_m / 2, r.width_m / 2
    return np.array([(r.x_m + c * u - s * v, r.y_m + s * u + c * v) for u, v in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))])


def _separated(a: np.ndarray, b: np.ndarray) -> bool:
    """Whether two convex quadrilaterals are apart (a separating axis among their edge normals)."""
    for poly in (a, b):
        for i in range(4):
            edge = poly[(i + 1) % 4] - poly[i]
            axis = np.array((-edge[1], edge[0]))
            pa, pb = a @ axis, b @ axis
            if pa.max() < pb.min() or pb.max() < pa.min():
                return True
    return False


def _point_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    t = 0.0 if not ab.any() else float(np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0.0, 1.0))
    return float(np.hypot(*(p - (a + t * ab))))


def footprint_clearance_m(pose: Pose2D, footprint: Footprint, rectangles) -> float:
    """The smallest distance between the footprint at ``pose`` and any of ``rectangles`` (0 when one overlaps, inf for
    none)."""
    corners = _footprint_corners(pose, footprint)
    best = inf
    for rect in rectangles:
        other = _rectangle_corners(rect)
        if not _separated(corners, other):
            return 0.0
        for p, q in ((corners, other), (other, corners)):
            for point in p:
                for i in range(4):
                    best = min(best, _point_segment(point, q[i], q[(i + 1) % 4]))
    return best


def _reversed_by(pose: Pose2D, distance_m: float) -> Pose2D:
    return Pose2D(pose.x_m - distance_m * cos(pose.yaw_rad), pose.y_m - distance_m * sin(pose.yaw_rad), pose.yaw_rad)


def exit_prefix_length_m(start: Pose2D, footprint: Footprint, exit_prefix: ExitPrefix) -> float | None:
    """The shortest straight reverse from ``start`` (0, step, 2 step, ... cap) after which ``footprint`` keeps the exit
    clearance from the exit obstacles; None when none within the cap does."""
    steps = int(floor(exit_prefix.cap_m / exit_prefix.step_m + 1e-9))
    for k in range(steps + 1):
        distance = k * exit_prefix.step_m
        if footprint_clearance_m(_reversed_by(start, distance), footprint, exit_prefix.obstacles) >= exit_prefix.clearance_m:
            return distance
    return None


def _join_prefix(prefix: PlanResult, search: PlanResult, record: dict) -> PlanResult:
    """The straight reverse then the search from its end, the search's own records kept."""
    poses = np.concatenate((prefix.poses, search.poses[1:]))
    directions = np.concatenate((prefix.directions, search.directions[1:]))
    curvatures = np.concatenate((prefix.curvatures_inv_m, search.curvatures_inv_m[1:]))
    directions[0], curvatures[0] = directions[1], curvatures[1]
    return replace(search, poses=poses, directions=directions, curvatures_inv_m=curvatures,
                   length_m=prefix.length_m + search.length_m, exit_prefix=record)


def _transport_search(start, goal, props, footprint, bounds, config, *, occupancy=None, deadline=None,
                      exit_prefix: ExitPrefix | None = None) -> PlanResult:
    """The loaded transport search; with ``exit_prefix`` it starts with the straight reverse ExitPrefix asks for, checked
    against the props, bounds and grid at the search's clearance. Without a clear pose within the cap, a straight the
    props refuse, or no search from the prefix's end, the plain search from ``start``, its record saying why."""
    def plain() -> PlanResult:
        return _search(start, goal, props, footprint, bounds, config, occupancy=occupancy, deadline=deadline)

    if exit_prefix is None:
        return plain()
    record = {"start": [float(start.x_m), float(start.y_m), float(start.yaw_rad)], "clearance_m": exit_prefix.clearance_m,
              "step_m": exit_prefix.step_m, "cap_m": exit_prefix.cap_m, "length_m": 0.0}
    distance = exit_prefix_length_m(start, footprint, exit_prefix)
    if distance is None:
        return replace(plain(), exit_prefix={**record, "status": "unmet:clearance"})
    if distance == 0:
        return replace(plain(), exit_prefix={**record, "status": "clear",
                                             "min_clearance_m": footprint_clearance_m(start, footprint, exit_prefix.obstacles)})
    end = _reversed_by(start, distance)
    straight = _straight_plan(start, end, -1, props, footprint, bounds, config.clearance_m, occupancy)
    if not straight.success:
        return replace(plain(), exit_prefix={**record, "status": "unmet:straight_collision"})
    search = _search(end, goal, props, footprint, bounds, config, occupancy=occupancy, deadline=deadline)
    if not search.success:
        return replace(plain(), exit_prefix={**record, "status": f"unmet:search_{search.status}"})
    return _join_prefix(straight, search, {**record, "status": "prefix", "length_m": distance,
                                            "min_clearance_m": footprint_clearance_m(end, footprint, exit_prefix.obstacles)})


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
    obstacles, footprint, bounds = leg_obstacles(
        "observe", scenario, geometry, pickup_obstacle=pickup_obstacle, pickup_bounds=pickup_bounds
    )
    return _search(
        start_rear if start_rear is not None else scenario.start_rear,
        waypoint,
        obstacles,
        footprint,
        bounds,
        config,
        extended,
        occupancy,
        deadline,
    )


def observation_start_checker(
    scenario: TransportScenario,
    config: PlannerConfig | None = None,
    *,
    geometry: SyntheticMissionGeometry | None = None,
    pickup_bounds: Bounds | None = None,
    occupancy=None,
    pickup_obstacle: Rectangle | None = None,
    **_unused,
):
    """The start check plan_observation_leg's search makes (hybrid_astar's ``invalid_start``), from the same inputs
    and the same construction: ``(free, clearance_m)`` with ``free(pose, clearance_m) -> bool`` (persistent console
    plan D13 item 6). Extra keyword arguments (the leg's ``deadline``) are accepted and ignored."""
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    config = config if config is not None else make_transport_planner_config()
    obstacles, footprint, bounds = leg_obstacles(
        "observe", scenario, geometry, pickup_obstacle=pickup_obstacle, pickup_bounds=pickup_bounds
    )
    checker = make_checker(obstacles, footprint, bounds, occupancy)
    return checker.free, float(config.clearance_m)


def backoff_advice(free, clearance_m: float, start: Pose2D, *, step_m: float = 0.05, limit_m: float = 1.0,
                   deadline: float | None = None, clock=time.monotonic) -> dict:
    """How far straight back from ``start`` the start check passes (persistent console plan D13 item 6).

    The poses at ``step_m`` steps up to ``limit_m``: before the first one the start check passes, each must be free
    at clearance 0 (the body passes there -- a wall behind blocks the way); then ``backoff_m`` is that first
    distance and ``backoff_max_m`` the last one passing without a break (a distance not checked is never advised).
    Nothing within the limit: ``backoff_m`` None with ``within_m``; the way blocked first: ``blocked_at_m``. A
    ``deadline`` (``clock`` time) checked before and at every pose: spent, ``{"skipped": "deadline"}`` -- never a
    partial search reported as nothing found."""
    if deadline is not None and clock() >= deadline:
        return {"skipped": "deadline"}
    bx, by = -cos(start.yaw_rad), -sin(start.yaw_rad)
    first = last = None
    for k in range(1, int(round(limit_m / step_m)) + 1):
        if deadline is not None and clock() >= deadline:
            return {"skipped": "deadline"}
        d = round(k * step_m, 6)
        pose = (start.x_m + d * bx, start.y_m + d * by, start.yaw_rad)
        if free(pose, clearance_m):
            if first is None:
                first = d
            last = d
            continue
        if first is not None:
            break
        if not free(pose, 0.0):
            return {"backoff_m": None, "blocked_at_m": d, "from_rear": [start.x_m, start.y_m, start.yaw_rad]}
    if first is None:
        return {"backoff_m": None, "within_m": float(limit_m), "from_rear": [start.x_m, start.y_m, start.yaw_rad]}
    return {"backoff_m": first, "backoff_max_m": last, "from_rear": [start.x_m, start.y_m, start.yaw_rad]}


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


def correction_point_offsets(
    trajectory,
    line_start: Pose2D,
    *,
    face_distance,
    points_m: Sequence[float],
) -> list[dict]:
    """Where a rolled-out approach crosses the near-field correction points (plan D8 S4c-3).

    face_distance(pose) is the estimated fork tip to pallet face at a trajectory pose (rear
    axle x, y, yaw) as the runner's correction trigger measures it. For each point d, the
    first pose whose distance is within d, the pose before it (None when the first pose
    already is) and the offsets of the crossing pose to the line from line_start (along,
    lateral, heading -- as straight_from_pose records them); pose and offsets are None when
    the trajectory never comes that close.
    """
    heading = np.array([cos(line_start.yaw_rad), sin(line_start.yaw_rad)])
    normal = np.array([-heading[1], heading[0]])
    origin = np.array([line_start.x_m, line_start.y_m])
    poses = np.asarray(trajectory, dtype=float).reshape(-1, 3)
    distances = [float(face_distance(pose)) for pose in poses]
    out = []
    for point in points_m:
        entry = {"point_m": float(point), "pose": None, "offsets": None}
        for i, (pose, distance) in enumerate(zip(poses, distances, strict=True)):
            if distance <= point + 1e-12:
                offset = pose[:2] - origin
                entry.update(
                    pose=[float(v) for v in pose],
                    face_distance_m=distance,
                    previous_pose=[float(v) for v in poses[i - 1]] if i else None,
                    previous_face_distance_m=distances[i - 1] if i else None,
                    offsets={
                        "along_m": float(offset @ heading),
                        "lateral_m": float(offset @ normal),
                        "yaw_rad": float(np.arctan2(np.sin(pose[2] - line_start.yaw_rad), np.cos(pose[2] - line_start.yaw_rad))),
                    },
                )
                break
        out.append(entry)
    return out


def leg_obstacles(
    kind: str,
    scenario: TransportScenario,
    geometry: SyntheticMissionGeometry | None = None,
    *,
    pickup_obstacle: Rectangle | None = None,
    pickup_bounds: Bounds | None = None,
) -> tuple[list[Rectangle], Footprint, Bounds]:
    """(rectangles, footprint, bounds) the transport, return or observation leg's search checks
    besides its grid: the props, plus the delivered pallet on the return and the pallet (or the
    rectangle standing for it) on the observation leg; the loaded outline in transport. One source
    for plan_transport_leg, plan_return_leg, plan_observation_leg and the recoveries that check a
    sweep against them (plan 2026-10-04-lidar-obstacle-map, obstacle recovery v4 E2 ①, cusp
    position stall v7)."""
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    props = scenario_props(scenario)
    if kind == "observe":
        pallet = pickup_obstacle if pickup_obstacle is not None else Rectangle(
            scenario.pickup.x_m,
            scenario.pickup.y_m,
            geometry.pallet_depth_m,
            geometry.pallet_width_m,
            scenario.pickup.yaw_rad,
        )
        return props + [pallet], geometry.unloaded_footprint, (
            pickup_bounds if pickup_bounds is not None else scenario.bounds
        )
    if kind == "transport":
        return props, geometry.loaded_footprint, scenario.bounds
    if kind == "return":
        delivered = Rectangle(
            scenario.destination.x_m,
            scenario.destination.y_m,
            geometry.pallet_depth_m,
            geometry.pallet_width_m,
            scenario.destination.yaw_rad,
        )
        return props + [delivered], geometry.unloaded_footprint, scenario.bounds
    raise ValueError(f"unknown leg {kind!r}")


def plan_transport_leg(
    scenario: TransportScenario,
    start_rear: Pose2D,
    config: PlannerConfig | None = None,
    *,
    geometry: SyntheticMissionGeometry | None = None,
    travel_config: PlannerConfig | None = None,
    occupancy=None,
    deadline=None,
    exit_prefix: ExitPrefix | None = None,
) -> PlanResult:
    """The loaded transport leg alone, from ``start_rear`` to the delivery pose (with ``exit_prefix``, starting with
    its straight reverse -- ExitPrefix).

    The same search and final straight as plan_transport's transport stage, for
    replanning from the measured pose when a gear cusp stops out of heading
    (docs/plans/2026-10-03-transport-stage-fixes.md). The carried pallet is
    part of the loaded footprint; the props and bounds stay obstacles.
    """
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    config = config if config is not None else make_transport_planner_config()
    travel_config = travel_config if travel_config is not None else config
    props, footprint, bounds = leg_obstacles("transport", scenario, geometry)
    destination = site_poses(scenario.destination, geometry)
    search = _transport_search(
        start_rear,
        destination["predelivery"],
        props,
        footprint,
        bounds,
        travel_config,
        occupancy=occupancy,
        deadline=deadline,
        exit_prefix=exit_prefix,
    )
    if not search.success:
        return search
    tail = _straight_plan(
        destination["predelivery"],
        destination["delivery"],
        1,
        scenario_props(scenario, "slot"),
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
    props = scenario_props(scenario)
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


def redraw_docking(
    scenario: TransportScenario,
    current_rear: Pose2D,
    target_pickup: PalletSite,
    config: PlannerConfig | None = None,
    *,
    geometry: SyntheticMissionGeometry | None = None,
    pickup_bounds: Bounds | None = None,
    occupancy=None,
    pickup_obstacle: Rectangle | None = None,
    docking_occupancy=None,
    dry_runs: Sequence = (),
    max_lateral_m: float = 0.05,
    max_yaw_rad: float = 0.05,
    min_length_m: float = 0.3,
    dry_ahead_m: float = 0.006,
    dry_lateral_m: float = 0.006,
    dry_yaw_rad: float = 0.015,
) -> tuple[dict | None, dict]:
    """The docking straights re-drawn for a new pallet estimate from where the truck
    stands (plan D8 S4b, a correction point).

    The approach is the rest of the new prealign-approach line (straight_from_pose,
    the near capture's box); the insertion and the extraction are plan_transport's,
    with its checks: the approach straight against the props and the perceived pallet
    at the approach clearance, the insertion and extraction straights against the props
    -- on docking_occupancy when given, else occupancy. Each of dry_runs (callables
    taking the approach straight and returning a RolloutResult) must arrive with its
    last pose within dry_ahead_m past the goal, dry_lateral_m across the new line and
    dry_yaw_rad of its heading (the tracker's own stop leaves it up to its position
    tolerance short, which "arrived" bounds), and its trajectory must pass the
    approach straight's check. Returns ({"approach", "insert", "extract"}, record) or
    (None, record) with the first refusal in record["refused"].
    """
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    config = config if config is not None else make_transport_planner_config()
    props = scenario_props(scenario)
    pickup = site_poses(target_pickup, geometry)
    pallet = (
        pickup_obstacle
        if pickup_obstacle is not None
        else Rectangle(
            target_pickup.x_m,
            target_pickup.y_m,
            geometry.pallet_depth_m,
            geometry.pallet_width_m,
            target_pickup.yaw_rad,
        )
    )
    near_bounds = pickup_bounds if pickup_bounds is not None else scenario.bounds
    approach_clearance = min(config.clearance_m, geometry.approach_gap_m / 2)
    docking_clearance = (
        approach_clearance if occupancy is not None else config.clearance_m
    )
    grid = docking_occupancy if docking_occupancy is not None else occupancy
    straight, offsets = straight_from_pose(
        current_rear,
        pickup["prealign"],
        pickup["approach"],
        max_lateral_m=max_lateral_m,
        max_yaw_rad=max_yaw_rad,
        min_length_m=min_length_m,
    )
    record = {
        "offsets": offsets,
        "refused": None,
        "dry_runs": [],
        "target": [target_pickup.x_m, target_pickup.y_m, target_pickup.yaw_rad],
    }

    def approach_clear(poses) -> bool:
        return collision_free_path(
            poses,
            props + [pallet],
            geometry.unloaded_footprint,
            near_bounds,
            margin_m=approach_clearance,
            max_step_m=0.04,
            occupancy=grid,
        )

    if straight is None:
        record["refused"] = "box"
        return None, record
    if not approach_clear(straight.poses):
        record["refused"] = "approach_collision"
        return None, record
    end = pickup["approach"]
    heading = np.array([cos(end.yaw_rad), sin(end.yaw_rad)])
    for dry_run in dry_runs:
        dry = dry_run(straight)
        last = np.asarray(
            dry.trajectory[-1] if dry.trajectory else (np.nan,) * 3, dtype=float
        )
        offset = last[:2] - np.array([end.x_m, end.y_m])
        entry = {
            "status": dry.status,
            "position_error_m": float(dry.position_error_m),
            "yaw_error_rad": float(dry.yaw_error_rad),
            "time_s": float(dry.time_s),
            "end_ahead_m": float(offset @ heading),
            "end_lateral_m": float(offset @ np.array([-heading[1], heading[0]])),
            "end_yaw_rad": float(
                np.arctan2(np.sin(last[2] - end.yaw_rad), np.cos(last[2] - end.yaw_rad))
            ),
            "trajectory_samples": len(dry.trajectory),
        }
        record["dry_runs"].append(entry)
        if not (
            dry.status == "arrived"
            and entry["end_ahead_m"] <= dry_ahead_m
            and abs(entry["end_lateral_m"]) <= dry_lateral_m
            and abs(entry["end_yaw_rad"]) <= dry_yaw_rad
        ):
            record["refused"] = "dry_run"
            return None, record
        entry["swept_clear"] = approach_clear(np.asarray(dry.trajectory, dtype=float))
        if not entry["swept_clear"]:
            record["refused"] = "dry_run_collision"
            return None, record
    insert = _straight_plan(
        pickup["approach"],
        pickup["inserted"],
        1,
        props,
        geometry.unloaded_footprint,
        near_bounds,
        docking_clearance,
        grid,
    )
    if not insert.success:
        record["refused"] = f"insert:{insert.status}"
        return None, record
    extract = _straight_plan(
        pickup["inserted"],
        pickup["extracted"],
        -1,
        props,
        geometry.loaded_footprint,
        near_bounds,
        0.0 if occupancy is not None else config.clearance_m,
        grid,
    )
    if not extract.success:
        record["refused"] = f"extract:{extract.status}"
        return None, record
    return {"approach": straight, "insert": insert, "extract": extract}, record


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
    forward_turnaround: bool = False,
    turnaround_check=None,
) -> PlanResult:
    """The unloaded return leg alone, as plan_transport's return_home stage.

    For replanning from the measured pose after a SLAM correction is applied
    at the destination (docs/plans/2026-10-04-online-slam-closed-loop.md); the
    delivered pallet is an obstacle, as in plan_transport.

    forward_turnaround (docs/plans/2026-10-12-return-forward-turnaround.md): the plan
    the truck sets off home with, from the withdrawal end, also weighs turning round
    first -- see _return_turnaround. turnaround_check, given poses (N, 3) of a
    turnaround arc from start_rear, returns None when the arc may be driven now or the
    reason it may not (the runner: the drive permission's snapshot). Off, the result is
    the direct plan exactly as before.
    """
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    config = config if config is not None else make_transport_planner_config()
    travel_config = travel_config if travel_config is not None else config
    obstacles, footprint, bounds = leg_obstacles("return", scenario, geometry)
    started = time.monotonic()
    direct = _search(
        start_rear,
        return_to,
        obstacles,
        footprint,
        bounds,
        travel_config,
        occupancy=occupancy,
        deadline=deadline,
    )
    if not forward_turnaround:
        return direct
    return _return_turnaround(
        start_rear, return_to, obstacles, footprint, bounds, travel_config, direct,
        occupancy=occupancy, deadline=deadline, check=turnaround_check, started=started,
    )


# Return-leg forward preference (2026-10-12, user: prefer driving forward; plan
# docs/plans/2026-10-12-return-forward-turnaround.md). The unloaded return starts at the
# withdrawal end facing the delivered pallet, home usually behind the truck, and the search's
# root goal shot is taken whenever it is collision-free and within the length cap: seed 5 of
# the recorded mission reversed all 22.8 m home. Raising reverse_penalty does not reach it (the
# root shot is taken before any comparison) and a capped root shot leaves the search to turn
# round on a heuristic that ignores heading -- 41,536 expansions and 28 s at reverse weight 2.0,
# a timeout within the runner's 20 s. Instead the plan the truck sets off with compares the
# direct plan with full-lock reverse arcs to either side, each followed by one search home.
RETURN_TURNAROUND_TRIGGER_M = 3.0  # direct plans reversing at most this are kept as they are
# Policy value, not a measurement: reverse length counts this many times forward length when
# whole return paths are compared (the search itself keeps the travel config's own weights). With 2.0
# the live-grid run of the recorded seed 5 (Isaac v6) kept the direct, all-reverse return (best
# turnaround 47.9 against 47.3); after seeing that, 3.0 was adopted provisionally to weigh the user's
# preference for driving forward more strongly. It was tuned on that seed's result and does not
# guarantee a forward return; whether it generalises is checked on independent runs.
RETURN_TURNAROUND_REVERSE_WEIGHT = 3.0
# Each side: 5 arcs, 10 candidates. 15-degree steps from 45 (20 candidates) chose 60-150 degrees on the
# 45 recorded and planned starts of tools/return_turnaround_check.py, neighbours within about one cost
# unit, and needed up to 14 s on ws1 (2026-10-12, artifacts/20261012_return_forward/check.json).
RETURN_TURNAROUND_ANGLES_DEG = (60, 90, 120, 150, 180)
# Provisional tuning value, not a measurement or a stability guarantee: the turnaround's own moves (the
# reverse arc and the search home after it) plan at this curvature, below the travel config's 0.29 1/m, to
# ease the tracker's load (its 0.36 1/m limit minus the plan's curvature: 0.07 -> 0.11 1/m, a difference of
# limits, not a measured error budget). Tried after Isaac v4 of the recorded seed 5 (ws1
# artifacts/20261012_return_forward/seed_5_fwd4): it chose the turnaround, but on the 92-degree full-lock turn
# into the parking cusp near home the command sat at the tracker's 0.36 limit for about 6 s and the truck
# stopped 0.115 m off the cusp (cusp_position_stall), while the ideal bicycle rollout of the same path
# arrived within 2.5 mm. Whether the steering response or SLAM pose changes made that demand is not
# separated (45a0259 found the 0.29 / 0.36 margin not short by itself), and the lower curvature also changes
# the chosen path's shape. The direct plan is unchanged; min() keeps a gentler caller curvature.
RETURN_TURNAROUND_CURVATURE_INV_M = 0.25
RETURN_TURNAROUND_EXPANSIONS = 8000  # one candidate's search (a single attempt, no ladder)
RETURN_TURNAROUND_TOTAL_EXPANSIONS = 60000  # every candidate's search together
# Time safety only: the candidates stop at the first of the caller's deadline (an absolute time,
# or a SearchBudget counted from the call's start, so the direct plan and the candidates share
# it) and this many seconds after the direct plan; a candidate finished after that is dropped.
# The record says so. Results are deterministic as long as it does not fire.
# 18 s: the runner gives the live return plan 20 s; the simulator waits for it (Isaac v5 ran out of 10 s).
RETURN_TURNAROUND_MAX_S = 18.0


def path_terms(path: PlanResult, config: PlannerConfig, reverse_weight: float) -> dict:
    """A whole path's forward and reverse length, gear changes and cost for comparing whole
    return paths -- its own objective, built from the planner's terms: length, reverse length
    weighted reverse_weight, gear changes, steering and steering change (from straight at the
    start) at config's penalties. The search itself does not charge steering on its analytic
    shots, so this is not the search's cost. Lengths are summed over the path samples."""
    poses = np.asarray(path.poses, dtype=float)
    if len(poses) < 2:
        return {"forward_m": 0.0, "reverse_m": 0.0, "gear_changes": 0, "cost": 0.0}
    seg = np.hypot(np.diff(poses[:, 0]), np.diff(poses[:, 1]))
    gear = np.asarray(path.directions, dtype=int)[1:]
    steer = np.asarray(path.curvatures_inv_m, dtype=float)[1:] / config.curvature_limit_inv_m
    forward = float(seg[gear > 0].sum())
    reverse = float(seg[gear < 0].sum())
    changes = int(np.count_nonzero(np.diff(gear)))
    cost = (
        forward
        + reverse_weight * reverse
        + config.gear_change_penalty_m * changes
        + config.steering_penalty * float((seg * np.abs(steer)).sum())
        + config.steering_change_penalty_m * (abs(float(steer[0])) + float(np.abs(np.diff(steer)).sum()))
    )
    return {"forward_m": forward, "reverse_m": reverse, "gear_changes": changes, "cost": float(cost)}


def _join_turnaround(start_pose, arc, curvature, arc_m: float, sub: PlanResult, sub_config: PlannerConfig,
                     direct: PlanResult) -> PlanResult:
    """The reverse arc (start_pose + its samples) followed by the search from its end, one
    PlanResult in the array contract: sample zero and the arc samples carry the arc's
    controls, the search's first pose (the arc end) is not repeated."""
    arc_poses = np.asarray([start_pose, *arc], dtype=float)
    attempt = (
        sub_config.analytic_expansion_interval, sub.status, int(sub.expanded_nodes),
        sub_config.xy_resolution_m, sub_config.yaw_resolution_rad, sub_config.collision_step_m,
        sub_config.primitive_length_m, sub_config.max_expansions,
        int(sub.pruned_children), int(sub.superseded_pops),
    )
    return replace(
        sub,
        poses=np.vstack([arc_poses, sub.poses[1:]]),
        directions=np.concatenate([np.full(len(arc_poses), -1, dtype=np.int8), sub.directions[1:]]),
        curvatures_inv_m=np.concatenate([np.full(len(arc_poses), float(curvature)), sub.curvatures_inv_m[1:]]),
        length_m=float(arc_m) + float(sub.length_m),  # both continuous lengths
        analytic_expansion_interval=sub_config.analytic_expansion_interval,
        search_attempts=tuple(direct.search_attempts) + (attempt,),
    )


def _return_turnaround(start_rear, return_to, obstacles, footprint, bounds, config, direct, *,
                       occupancy=None, deadline=None, check=None, started=None) -> PlanResult:
    """The direct return plan, or a turnaround home when that is cheaper.

    When the direct plan reverses more than RETURN_TURNAROUND_TRIGGER_M, each candidate is a
    full-lock reverse arc from the start (left and right, RETURN_TURNAROUND_ANGLES_DEG, at
    RETURN_TURNAROUND_CURVATURE_INV_M), swept against the same rectangles and grid at the search's
    clearance and passed by ``check`` when given, followed by one Hybrid A* search (no retry
    ladder) from the arc end home; the searches run cheapest lower bound first and are skipped
    when the bound cannot beat the best so far. Every successful candidate and the direct plan are compared on path_terms at
    RETURN_TURNAROUND_REVERSE_WEIGHT; the cheapest is kept, the direct plan on a tie. This is
    the cheapest of what was evaluated, not an optimum of the weighted cost. The result's
    ``turnaround`` records the comparison."""
    weight = RETURN_TURNAROUND_REVERSE_WEIGHT
    record = {
        "trigger_m": RETURN_TURNAROUND_TRIGGER_M,
        "reverse_weight": weight,
        # The candidates' arc and search curvature, and the one path_terms normalises steering by.
        "candidate_curvature_inv_m": min(config.curvature_limit_inv_m, RETURN_TURNAROUND_CURVATURE_INV_M),
        "cost_curvature_inv_m": config.curvature_limit_inv_m,
        "direct": {"status": direct.status},
        "chosen": "direct",
        "evaluated": False,
        "candidates": [],
        "expansions": 0,
        "timed_out": False,
    }
    if not direct.success:
        return replace(direct, turnaround=record)
    record["direct"].update(path_terms(direct, config, weight))
    if record["direct"]["reverse_m"] <= RETURN_TURNAROUND_TRIGGER_M:
        return replace(direct, turnaround=record)
    record["evaluated"] = True
    limit = time.monotonic() + RETURN_TURNAROUND_MAX_S
    if isinstance(deadline, SearchBudget):
        limit = min(limit, (started if started is not None else time.monotonic()) + float(deadline.seconds))
    elif deadline is not None:
        limit = min(limit, float(deadline))
    checker = make_checker(obstacles, footprint, bounds, occupancy)
    field = None  # the candidates' searches share one obstacle heuristic (same goal and grid)
    move_config = replace(
        config, curvature_limit_inv_m=min(config.curvature_limit_inv_m, RETURN_TURNAROUND_CURVATURE_INV_M)
    )
    k = move_config.curvature_limit_inv_m
    start_pose = (start_rear.x_m, start_rear.y_m, _wrap(start_rear.yaw_rad))
    best_cost, best, best_name = record["direct"]["cost"], direct, "direct"
    spent = 0
    # 1. Every arc, in the fixed order: swept against the rectangles and grid, then the check.
    ready = []
    for side in (1, -1):
        for degrees in RETURN_TURNAROUND_ANGLES_DEG:
            entry = {"name": f"reverse_arc:{side:+d}:{degrees}"}
            record["candidates"].append(entry)
            if time.monotonic() > limit:  # the arcs' own checks share the limit too
                entry["status"] = "skipped_time"
                record["timed_out"] = True
                continue
            curvature = side * k
            arc_m = degrees * pi / 180 / k
            arc = _sample_arc(start_pose, arc_m, -1, curvature, checker, move_config)
            if arc is None:
                entry["status"] = "arc_blocked"
                continue
            if check is not None:
                why = check(np.asarray([start_pose, *arc], dtype=float))
                if why is not None:
                    entry["status"] = f"arc_not_permitted:{why}"
                    continue
            # A lower bound on the candidate's whole path_terms cost: the arc's own terms over its samples
            # (path_terms sums chords, not the analytic length -- Codex 2026-10-12 6th P2) plus the
            # straight-line distance from its end home (the search home costs at least its length;
            # the cusp's gear change and any further steering change cost at least 0).
            arc_xy = np.asarray([start_pose, *arc], dtype=float)[:, :2]
            chord_m = float(np.hypot(*np.diff(arc_xy, axis=0).T).sum())
            steer = abs(curvature) / config.curvature_limit_inv_m
            bound = (weight + config.steering_penalty * steer) * chord_m + config.steering_change_penalty_m * steer
            bound += float(np.hypot(arc[-1][0] - return_to.x_m, arc[-1][1] - return_to.y_m))
            entry["lower_bound"] = bound
            ready.append((bound, len(ready), entry, curvature, arc_m, arc))
    # 2. Searches cheapest bound first (Isaac v5: under the simulator's load the fixed order spent the
    # time safety on the far side's arcs before reaching the near side's), none whose bound cannot
    # beat the best so far.
    for bound, _, entry, curvature, arc_m, arc in sorted(ready, key=lambda item: item[:2]):
        if time.monotonic() > limit:
            entry["status"] = "skipped_time"
            record["timed_out"] = True
            continue
        if bound >= best_cost - 1e-9:
            entry["status"] = "pruned"
            continue
        if spent >= RETURN_TURNAROUND_TOTAL_EXPANSIONS:
            entry["status"] = "skipped_budget"
            continue
        sub_config = replace(
            move_config,
            max_expansions=min(RETURN_TURNAROUND_EXPANSIONS, RETURN_TURNAROUND_TOTAL_EXPANSIONS - spent),
        )
        if field is None:
            field = obstacle_heuristic_field(return_to, obstacles, footprint, bounds, config, occupancy)
        sub = plan_hybrid_astar(
            Pose2D(*arc[-1]), return_to, obstacles, footprint, bounds, sub_config,
            occupancy=occupancy, deadline=limit, obstacle_field=field,
        )
        spent += int(sub.expanded_nodes)
        entry.update(status=sub.status, expansions=int(sub.expanded_nodes))
        if sub.status == "timeout" or time.monotonic() > limit:
            record["timed_out"] = True
            if sub.success:
                entry["status"] = "late"  # finished after the limit: not compared
            continue
        if not sub.success:
            continue
        joined = _join_turnaround(start_pose, arc, curvature, arc_m, sub, sub_config, direct)
        terms = path_terms(joined, config, weight)
        entry.update(terms)
        if terms["cost"] < best_cost - 1e-9:
            best_cost, best, best_name = terms["cost"], joined, entry["name"]
    record.update(chosen=best_name, expansions=spent)
    return replace(best, turnaround=record)


def plan_loaded_delivery(
    scenario: TransportScenario,
    start_rear: Pose2D,
    config: PlannerConfig | None = None,
    *,
    geometry: SyntheticMissionGeometry | None = None,
    return_to: Pose2D | None = None,
    travel_config: PlannerConfig | None = None,
    trace: list | None = None,
    occupancy=None,
    docking_occupancy=None,
    deadline=None,
    exit_prefix: ExitPrefix | None = None,
) -> MissionPlan:
    """The delivery of a pallet already on the forks (persistent console plan D5 6c): from ``start_rear`` the same
    steps plan_transport takes from its extracted pose on -- the transport search with the loaded footprint (on a
    grid, once more with no clearance when the start is invalid), the delivery straight, the withdraw straight and,
    with ``return_to``, the return with the delivered pallet as an obstacle. plan_transport is left as it is; the
    tests hold the two equal from the extracted pose. approach, insert and extract are None."""
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    config = config if config is not None else make_transport_planner_config()
    travel_config = travel_config if travel_config is not None else config

    def note(stage: str, result: PlanResult) -> PlanResult:
        if trace is not None:
            directions = np.asarray(result.directions)
            trace.append({"stage": stage, "status": result.status, "length_m": float(result.length_m),
                          "expansions": int(result.expanded_nodes),
                          "analytic_expansion_interval": result.analytic_expansion_interval,
                          "search_attempts": [list(entry) for entry in result.search_attempts],
                          "gear_changes": int(np.count_nonzero(np.diff(directions))) if directions.size else 0})
        return result

    props = scenario_props(scenario)
    slot_props = scenario_props(scenario, "slot")  # the delivery straight and the withdraw (plan 2026-10-22 2.4)
    destination = site_poses(scenario.destination, geometry)
    grid = docking_occupancy if docking_occupancy is not None else occupancy
    transport = note("transport_search", _transport_search(start_rear, destination["predelivery"], props,
                                                           geometry.loaded_footprint, scenario.bounds, travel_config,
                                                           occupancy=grid, deadline=deadline, exit_prefix=exit_prefix))
    if transport.status == "invalid_start" and occupancy is not None:
        transport = note("transport_search_tight", _transport_search(start_rear, destination["predelivery"], props,
                                                                     geometry.loaded_footprint, scenario.bounds,
                                                                     replace(travel_config, clearance_m=0.0),
                                                                     occupancy=grid, deadline=deadline,
                                                                     exit_prefix=exit_prefix))
    if not transport.success:
        return MissionPlan(False, f"transport:{transport.status}")
    tail = note("transport_straight", _straight_plan(destination["predelivery"], destination["delivery"], 1,
                                                     slot_props, geometry.loaded_footprint, scenario.bounds,
                                                     config.clearance_m, grid))
    if not tail.success:
        return MissionPlan(False, f"transport:{tail.status}")
    transport = _append_straight(transport, tail)
    withdraw = note("withdraw", _straight_plan(destination["delivery"], destination["withdrawn"], -1, slot_props,
                                               geometry.unloaded_footprint, scenario.bounds, config.clearance_m, grid))
    if not withdraw.success:
        return MissionPlan(False, f"withdraw:{withdraw.status}")
    if return_to is None:
        return MissionPlan(True, "success", None, None, None, transport, withdraw)
    delivered = Rectangle(scenario.destination.x_m, scenario.destination.y_m, geometry.pallet_depth_m,
                          geometry.pallet_width_m, scenario.destination.yaw_rad)
    return_home = note("return_home", _search(destination["withdrawn"], return_to, props + [delivered],
                                              geometry.unloaded_footprint, scenario.bounds, travel_config,
                                              occupancy=grid, deadline=deadline))
    if not return_home.success:
        return MissionPlan(False, f"return_home:{return_home.status}")
    return MissionPlan(True, "success", None, None, None, transport, withdraw, return_home)


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
    exit_prefix: ExitPrefix | None = None,
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

    props = scenario_props(scenario)
    slot_props = scenario_props(scenario, "slot")  # the delivery straight and the withdraw (plan 2026-10-22 2.4)
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
        _transport_search(
            pickup["extracted"],
            destination["predelivery"],
            props,
            geometry.loaded_footprint,
            scenario.bounds,
            travel_config,
            occupancy=docking_occupancy if docking_occupancy is not None else occupancy,
            deadline=deadline,
            exit_prefix=exit_prefix,
        ),
    )
    if transport.status == "invalid_start" and occupancy is not None:
        # The extracted pose sits where the pallet was: within the travel
        # clearance of a grid mark beside it (L3c seed 4). As the runner's
        # replans do, once more with no clearance -- the swelling holds the
        # placement error and the permission guards every tick.
        transport = note(
            "transport_search_tight",
            _transport_search(
                pickup["extracted"],
                destination["predelivery"],
                props,
                geometry.loaded_footprint,
                scenario.bounds,
                replace(travel_config, clearance_m=0.0),
                occupancy=docking_occupancy if docking_occupancy is not None else occupancy,
                deadline=deadline,
                exit_prefix=exit_prefix,
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
            slot_props,
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
            slot_props,
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
