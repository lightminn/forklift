"""One fleet truck's mission as runnable stages, and replanning a travel stage.

Plan: docs/plans/2026-10-08-fleet-five-forklifts.md. The stages are those of
plan_transport with two changes for the warehouse: a travel leg ``observe``
from the parking place to the mouth of the truck's aisle first, and an
``aisle_exit`` after extraction -- a straight reverse back down the aisle to
its mouth, because a measured-chassis truck cannot turn round in a 3.5 m
aisle. The Hybrid A* searches (observe, approach, transport, return_home) then
never have to find a 16 m reverse and a turn in one go.

The parking place is left and entered straight (``parking_exit`` ahead,
``parking_entry`` behind): observe starts with the straight exit and
return_home ends with the straight entry.

Order: observe, approach, insert, [lift], extract, aisle_exit, transport,
[lower], withdraw, return_home. Loaded from lift to lower. The truck's own
intake pallet is an obstacle until it is lifted; the pallet it delivers is an
obstacle for return_home.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from .fleet_layout import TruckMission
from .geometry import Pose2D, Rectangle
from .hybrid_astar import PlannerConfig, PlanResult
from .pallet_mission import (
    SyntheticMissionGeometry,
    _append_straight,
    _search,
    _straight_plan,
    site_poses,
)

STAGES = (
    "observe",
    "approach",
    "insert",
    "extract",
    "aisle_exit",
    "transport",
    "withdraw",
    "return_home",
)
LOADED_STAGES = frozenset({"extract", "aisle_exit", "transport"})
REPLANNABLE = frozenset({"observe", "transport", "return_home"})


@dataclass(frozen=True)
class FleetPlan:
    success: bool
    status: str
    stages: dict


def _rect(site, geometry):
    return Rectangle(
        site.x_m,
        site.y_m,
        geometry.pallet_depth_m,
        geometry.pallet_width_m,
        site.yaw_rad,
    )


def aisle_exit_pose(
    mission: TruckMission, geometry: SyntheticMissionGeometry
) -> Pose2D:
    extracted = site_poses(mission.pickup, geometry)["extracted"]
    return Pose2D(extracted.x_m, mission.aisle_entry.y_m, extracted.yaw_rad)


def plan_fleet_mission(
    mission: TruckMission,
    config: PlannerConfig,
    travel_config: PlannerConfig,
    geometry: SyntheticMissionGeometry,
) -> FleetPlan:
    """Every stage, or the first failure as ``"<stage>:<status>"``."""
    scenario = mission.scenario
    props = [p.rectangle for p in scenario.props]
    pallet = _rect(mission.pickup, geometry)
    delivered = _rect(mission.destination, geometry)
    pickup = site_poses(mission.pickup, geometry)
    destination = site_poses(mission.destination, geometry)
    unloaded, loaded = geometry.unloaded_footprint, geometry.loaded_footprint
    near = mission.pickup_bounds
    approach_config = replace(
        config, clearance_m=min(config.clearance_m, geometry.approach_gap_m / 2)
    )
    exit_pose = aisle_exit_pose(mission, geometry)
    steps = [
        (
            "observe",
            lambda: _lead_in(
                _straight_plan(
                    mission.parking,
                    mission.parking_exit,
                    1,
                    props + [pallet],
                    unloaded,
                    scenario.bounds,
                    config.clearance_m,
                ),
                lambda: _search(
                    mission.parking_exit,
                    mission.aisle_entry,
                    props + [pallet],
                    unloaded,
                    scenario.bounds,
                    travel_config,
                ),
            ),
        ),
        (
            "approach",
            lambda: _chain(
                _search(
                    mission.aisle_entry,
                    pickup["prealign"],
                    props + [pallet],
                    unloaded,
                    near,
                    approach_config,
                ),
                lambda: _straight_plan(
                    pickup["prealign"],
                    pickup["approach"],
                    1,
                    props + [pallet],
                    unloaded,
                    near,
                    approach_config.clearance_m,
                ),
            ),
        ),
        (
            "insert",
            lambda: _straight_plan(
                pickup["approach"],
                pickup["inserted"],
                1,
                props,
                unloaded,
                near,
                config.clearance_m,
            ),
        ),
        (
            "extract",
            lambda: _straight_plan(
                pickup["inserted"],
                pickup["extracted"],
                -1,
                props,
                loaded,
                near,
                config.clearance_m,
            ),
        ),
        (
            "aisle_exit",
            lambda: _straight_plan(
                pickup["extracted"],
                exit_pose,
                -1,
                props,
                loaded,
                scenario.bounds,
                config.clearance_m,
            ),
        ),
        (
            "transport",
            lambda: _transport(
                exit_pose, props, mission, geometry, config, travel_config
            ),
        ),
        (
            "withdraw",
            lambda: _straight_plan(
                destination["delivery"],
                destination["withdrawn"],
                -1,
                props,
                unloaded,
                scenario.bounds,
                config.clearance_m,
            ),
        ),
        (
            "return_home",
            lambda: _home(
                destination["withdrawn"],
                props + [delivered],
                mission,
                geometry,
                config,
                travel_config,
            ),
        ),
    ]
    stages = {}
    for name, step in steps:
        result = step()
        if not result.success:
            return FleetPlan(False, f"{name}:{result.status}", stages)
        stages[name] = result
    return FleetPlan(True, "success", stages)


def _chain(first: PlanResult, second) -> PlanResult:
    if not first.success:
        return first
    tail = second()
    return tail if not tail.success else _append_straight(first, tail)


def _lead_in(straight: PlanResult, search) -> PlanResult:
    """A straight lead-in, then a search; the search's metadata is kept."""
    if not straight.success:
        return straight
    rest = search()
    if not rest.success:
        return rest
    return replace(
        _append_straight(straight, rest),
        expanded_nodes=rest.expanded_nodes,
        analytic_expansion_interval=rest.analytic_expansion_interval,
        search_attempts=rest.search_attempts,
    )


def _home(start, props, mission, geometry, config, travel_config) -> PlanResult:
    """Search to the parking place's entry pose, then straight in."""
    unloaded = geometry.unloaded_footprint
    bounds = mission.scenario.bounds
    return _chain(
        _search(start, mission.parking_entry, props, unloaded, bounds, travel_config),
        lambda: _straight_plan(
            mission.parking_entry,
            mission.parking,
            1,
            props,
            unloaded,
            bounds,
            config.clearance_m,
        ),
    )


def _transport(start, props, mission, geometry, config, travel_config) -> PlanResult:
    destination = site_poses(mission.destination, geometry)
    loaded = geometry.loaded_footprint
    bounds = mission.scenario.bounds
    return _chain(
        _search(
            start, destination["predelivery"], props, loaded, bounds, travel_config
        ),
        lambda: _straight_plan(
            destination["predelivery"],
            destination["delivery"],
            1,
            props,
            loaded,
            bounds,
            config.clearance_m,
        ),
    )


def replan_stage(
    mission: TruckMission,
    stage: str,
    start: Pose2D,
    extra_obstacles: list[Rectangle],
    config: PlannerConfig,
    travel_config: PlannerConfig,
    geometry: SyntheticMissionGeometry,
) -> PlanResult:
    """The rest of a travel stage from ``start``, with optional extra obstacles."""
    if stage not in REPLANNABLE:
        raise ValueError(f"{stage} is not replanned")
    props = [p.rectangle for p in mission.scenario.props] + list(extra_obstacles)
    bounds = mission.scenario.bounds
    if stage == "observe":
        return _search(
            start,
            mission.aisle_entry,
            props + [_rect(mission.pickup, geometry)],
            geometry.unloaded_footprint,
            bounds,
            travel_config,
        )
    if stage == "transport":
        return _transport(start, props, mission, geometry, config, travel_config)
    return _home(
        start,
        props + [_rect(mission.destination, geometry)],
        mission,
        geometry,
        config,
        travel_config,
    )
