"""Synthetic ground-truth pallet transport geometry and path orchestration.

The sites are pallet centres, while every path pose references the rear axle.
This module neither detects pallets nor validates pocket clearance or physical
load handling. The simulation adapter supplies real asset bounds and executes
the distinct insertion, loading, transport, unloading, and withdrawal stages.
"""

from collections.abc import Sequence
from dataclasses import dataclass, replace
from math import ceil, cos, pi, sin

import numpy as np

from forklift_core._validation import _finite_scalar

from .geometry import (
    Bounds,
    Footprint,
    Pose2D,
    Rectangle,
    collision_free_path,
    collision_free_pose,
)
from .hybrid_astar import PlannerConfig, PlanResult, plan_hybrid_astar


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
    loaded_footprint: Footprint = Footprint(1.53, 0.17, 0.4)
    pallet_depth_m: float = 0.60
    pallet_width_m: float = 0.80
    inserted_offset_m: float = 1.23
    approach_offset_m: float = 1.69
    prealign_offset_m: float = 2.49
    extraction_m: float = 0.65
    predelivery_offset_m: float = 1.93
    withdrawal_m: float = 0.55
    spawn_clearance_m: float = 0.12

    def __post_init__(self) -> None:
        for value in (
            self.pallet_depth_m,
            self.pallet_width_m,
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
    """All five runnable paths, or no paths if any stage failed.

    Load after insert, unload after transport. Stop at every stage boundary
    and every gear-change cusp. Pocket collision checks are adapter-owned.
    """

    success: bool
    status: str
    approach: PlanResult | None = None
    insert: PlanResult | None = None
    extract: PlanResult | None = None
    transport: PlanResult | None = None
    withdraw: PlanResult | None = None


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


def _reservations(start, pickup, destination, geometry):
    pickup_poses = site_poses(pickup, geometry)
    destination_poses = site_poses(destination, geometry)
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
        _swept_reservation(
            destination_poses["predelivery"],
            destination_poses["delivery"],
            geometry.loaded_footprint,
        ),
        _swept_reservation(
            destination_poses["delivery"],
            destination_poses["withdrawn"],
            geometry.unloaded_footprint,
        ),
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


def _straight_plan(start, goal, direction, obstacles, footprint, bounds, clearance_m):
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
        poses, obstacles, footprint, bounds, margin_m=clearance_m, max_step_m=0.04
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
    )


def plan_transport(
    scenario: TransportScenario,
    config: PlannerConfig | None = None,
    *,
    geometry: SyntheticMissionGeometry | None = None,
) -> MissionPlan:
    """Plan all stages with exact final straight approaches and loaded geometry.

    The initial pallet is an obstacle only for approach. During insertion and
    withdrawal, pocket geometry/contact must be checked by the adapter; other
    props and world bounds always remain obstacles. Approach clearance is
    capped at 0.05 m because the nominal fork-to-pallet stopping gap is 0.10 m.
    Other stages use the supplied clearance (default 0.10 m).
    """
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    config = config if config is not None else PlannerConfig(clearance_m=0.10)
    props = [prop.rectangle for prop in scenario.props]
    pickup = site_poses(scenario.pickup, geometry)
    destination = site_poses(scenario.destination, geometry)
    pallet = Rectangle(
        scenario.pickup.x_m,
        scenario.pickup.y_m,
        geometry.pallet_depth_m,
        geometry.pallet_width_m,
        scenario.pickup.yaw_rad,
    )
    approach_config = replace(config, clearance_m=min(config.clearance_m, 0.05))
    approach = plan_hybrid_astar(
        scenario.start_rear,
        pickup["prealign"],
        props + [pallet],
        geometry.unloaded_footprint,
        scenario.bounds,
        approach_config,
    )
    if not approach.success:
        return MissionPlan(False, f"approach:{approach.status}")
    approach_tail = _straight_plan(
        pickup["prealign"],
        pickup["approach"],
        1,
        props + [pallet],
        geometry.unloaded_footprint,
        scenario.bounds,
        approach_config.clearance_m,
    )
    if not approach_tail.success:
        return MissionPlan(False, f"approach:{approach_tail.status}")
    approach = _append_straight(approach, approach_tail)
    insert = _straight_plan(
        pickup["approach"],
        pickup["inserted"],
        1,
        props,
        geometry.unloaded_footprint,
        scenario.bounds,
        config.clearance_m,
    )
    if not insert.success:
        return MissionPlan(False, f"insert:{insert.status}")
    extract = _straight_plan(
        pickup["inserted"],
        pickup["extracted"],
        -1,
        props,
        geometry.loaded_footprint,
        scenario.bounds,
        config.clearance_m,
    )
    if not extract.success:
        return MissionPlan(False, f"extract:{extract.status}")
    transport = plan_hybrid_astar(
        pickup["extracted"],
        destination["predelivery"],
        props,
        geometry.loaded_footprint,
        scenario.bounds,
        config,
    )
    if not transport.success:
        return MissionPlan(False, f"transport:{transport.status}")
    transport_tail = _straight_plan(
        destination["predelivery"],
        destination["delivery"],
        1,
        props,
        geometry.loaded_footprint,
        scenario.bounds,
        config.clearance_m,
    )
    if not transport_tail.success:
        return MissionPlan(False, f"transport:{transport_tail.status}")
    transport = _append_straight(transport, transport_tail)
    withdraw = _straight_plan(
        destination["delivery"],
        destination["withdrawn"],
        -1,
        props,
        geometry.unloaded_footprint,
        scenario.bounds,
        config.clearance_m,
    )
    if not withdraw.success:
        return MissionPlan(False, f"withdraw:{withdraw.status}")
    return MissionPlan(True, "success", approach, insert, extract, transport, withdraw)
