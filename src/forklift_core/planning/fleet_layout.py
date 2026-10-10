"""Several trucks on one warehouse floor: who picks what, where it goes, what each avoids.

Plan: docs/plans/2026-10-08-fleet-five-forklifts.md. The south hall is filled
by the factory generator (make_factory_scenario) exactly as for one truck;
its kept-clear bay becomes the parking row. North of it the warehouse racks
are fixed obstacles with single-lane aisles between them, and an intake
pallet waits in front of each usable aisle in the north strip. A seeded
draw gives every truck one aisle (so one intake pallet) and one shipping dock.

Each truck gets its own TransportScenario whose props are everything it must
not touch: the static floor, the other trucks' intake pallets, the pallets
that will stand at the other trucks' docks, the other parking places, and
every aisle but its own (one truck per aisle is the traffic rule; a truck
cannot turn round or pass another in a 3.5 m aisle). Moving trucks are not in
here -- fleet_traffic keeps them apart at run time. A parked truck is grown by
``parked_gap_m``, the gap fleet_traffic keeps between two trucks: a route that
passes a parked truck closer than that would wait on it for good.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from math import cos, pi, sin
from pathlib import Path

import numpy as np

from .factory_layout import (
    FactoryAssets,
    FactoryLayout,
    Region,
    StackedProp,
    load_factory_layout,
    make_factory_scenario,
)
from .fleet_traffic import footprint_rectangle
from .geometry import Bounds, Footprint, Pose2D, Rectangle, collision_free_pose
from .pallet_mission import (
    AssetSpec,
    PalletSite,
    PlacedProp,
    SyntheticMissionGeometry,
    TransportScenario,
)

_FLEET_STREAM = 0x464C4545  # "FLEE", independent of the factory streams
_FIXED_ASSET = AssetSpec("fixed:warehouse", 1.0, 1.0, 2.0)
_RULE_ASSET = AssetSpec("rule:aisle_closed", 1.0, 1.0, 2.0)
_TRUCK_ASSET = AssetSpec("rule:parking", 1.0, 1.0, 2.0)


@dataclass(frozen=True)
class Aisle:
    name: str
    x_min_m: float
    x_max_m: float

    @property
    def centre_x_m(self) -> float:
        return (self.x_min_m + self.x_max_m) / 2


@dataclass(frozen=True)
class FleetLayout:
    layout_version: str
    south_hall: FactoryLayout
    bounds: Bounds
    fixed_obstacles: tuple[tuple[str, Region], ...]
    aisles: tuple[Aisle, ...]
    aisle_y_m: tuple[float, float]
    intake_y_m: float
    aisle_entry_y_m: float
    parking: tuple[Pose2D, ...]
    parking_exit_m: float = 3.5
    parking_entry_m: float = 3.5
    # Warehouse prims a simulator deactivates (paths under its root).
    removed_prims: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not self.aisles or not self.parking:
            raise ValueError("a fleet layout needs aisles and parking places")
        if not self.aisle_y_m[0] < self.aisle_y_m[1] < self.intake_y_m:
            raise ValueError("the intake must lie north of the aisles")
        if not self.aisle_entry_y_m < self.aisle_y_m[0]:
            raise ValueError("the aisle entry must lie south of the aisles")
        b = self.bounds
        for pose in self.parking:
            if not (
                b.x_min_m < pose.x_m < b.x_max_m and b.y_min_m < pose.y_m < b.y_max_m
            ):
                raise ValueError(f"parking {pose} lies outside the bounds")
        for aisle in self.aisles:
            if not b.x_min_m < aisle.x_min_m < aisle.x_max_m < b.x_max_m:
                raise ValueError(f"{aisle.name} lies outside the bounds")


@dataclass(frozen=True)
class TruckMission:
    """One truck's job and the scenario it plans in."""

    truck: int
    aisle: str
    parking: Pose2D
    parking_exit: Pose2D
    parking_entry: Pose2D
    aisle_entry: Pose2D
    pickup: PalletSite
    destination: PalletSite
    scenario: TransportScenario
    pickup_bounds: Bounds


@dataclass(frozen=True)
class FleetScenario:
    seed: int
    layout_version: str
    bounds: Bounds
    fixed: tuple[PlacedProp, ...]
    work_items: tuple[PlacedProp, ...]
    loads: tuple[StackedProp, ...]
    missions: tuple[TruckMission, ...]


def _region(data: dict, where: str) -> Region:
    keys = {"x_min_m", "x_max_m", "y_min_m", "y_max_m"}
    if set(data) - {"name"} != keys:
        raise ValueError(f"{where} needs exactly {sorted(keys)} (and a name)")
    return Region(
        *(float(data[k]) for k in ("x_min_m", "x_max_m", "y_min_m", "y_max_m"))
    )


def load_fleet_layout(path: Path) -> FleetLayout:
    import yaml

    path = Path(path)
    data = yaml.safe_load(path.read_text())
    expected = {
        "layout_version",
        "south_hall_layout",
        "bounds",
        "fixed_obstacles",
        "aisles",
        "aisle_y_m",
        "intake_y_m",
        "aisle_entry_y_m",
        "parking_exit_m",
        "parking_entry_m",
        "parking",
        "removed_prims",
    }
    if set(data) != expected:
        raise ValueError(f"{path} keys differ: {sorted(set(data) ^ expected)}")
    b = _region(data["bounds"], "bounds")
    return FleetLayout(
        layout_version=str(data["layout_version"]),
        south_hall=load_factory_layout(path.parent / data["south_hall_layout"]),
        bounds=Bounds(b.x_min_m, b.x_max_m, b.y_min_m, b.y_max_m),
        fixed_obstacles=tuple(
            (str(item["name"]), _region(item, "fixed obstacle"))
            for item in data["fixed_obstacles"]
        ),
        aisles=tuple(
            Aisle(str(a["name"]), float(a["x_min_m"]), float(a["x_max_m"]))
            for a in data["aisles"]
        ),
        aisle_y_m=tuple(float(v) for v in data["aisle_y_m"]),
        intake_y_m=float(data["intake_y_m"]),
        aisle_entry_y_m=float(data["aisle_entry_y_m"]),
        parking=tuple(Pose2D(*(float(v) for v in p)) for p in data["parking"]),
        parking_exit_m=float(data["parking_exit_m"]),
        parking_entry_m=float(data["parking_entry_m"]),
        removed_prims=tuple(str(name) for name in data["removed_prims"]),
    )


def _ahead(pose: Pose2D, distance_m: float) -> Pose2D:
    return Pose2D(
        pose.x_m + distance_m * cos(pose.yaw_rad),
        pose.y_m + distance_m * sin(pose.yaw_rad),
        pose.yaw_rad,
    )


def _pallet(site: PalletSite, geometry: SyntheticMissionGeometry) -> Rectangle:
    return Rectangle(
        site.x_m,
        site.y_m,
        geometry.pallet_depth_m,
        geometry.pallet_width_m,
        site.yaw_rad,
    )


def make_fleet_scenario(
    seed: int,
    layout: FleetLayout,
    bay_assets: Sequence[AssetSpec],
    factory_assets: FactoryAssets,
    *,
    geometry: SyntheticMissionGeometry,
    trucks: int = 5,
    parked_gap_m: float = 0.0,
) -> FleetScenario:
    """Fill the floor for this seed and give each of ``trucks`` trucks a mission.

    Raises ValueError when a parking, entry, intake or dock pose touches the
    static floor -- a layout defect, not a planning result.
    """
    docks = layout.south_hall.docks
    if not 0 < trucks <= min(len(layout.aisles), len(docks), len(layout.parking)):
        raise ValueError("more trucks than aisles, docks or parking places")
    factory = make_factory_scenario(
        seed, layout.south_hall, bay_assets, factory_assets, geometry=geometry
    )
    rng = np.random.default_rng([seed, _FLEET_STREAM])
    aisle_order = rng.permutation(len(layout.aisles))[:trucks]
    dock_order = rng.permutation(len(docks))[:trucks]
    hall = layout.south_hall
    destinations = []
    for index in dock_order:
        dock = docks[int(index)]
        lateral = float(rng.uniform(-hall.dock_jitter_m, hall.dock_jitter_m))
        destinations.append(
            PalletSite(
                dock.x_m - lateral * sin(dock.yaw_rad),
                dock.y_m + lateral * cos(dock.yaw_rad),
                dock.yaw_rad
                + float(
                    rng.uniform(-hall.dock_yaw_jitter_rad, hall.dock_yaw_jitter_rad)
                ),
            )
        )
    aisles = [layout.aisles[int(i)] for i in aisle_order]
    pickups = [PalletSite(a.centre_x_m, layout.intake_y_m, pi / 2) for a in aisles]
    entries = [Pose2D(a.centre_x_m, layout.aisle_entry_y_m, pi / 2) for a in aisles]
    exits = [_ahead(p, layout.parking_exit_m) for p in layout.parking[:trucks]]
    prepark = [_ahead(p, -layout.parking_entry_m) for p in layout.parking[:trucks]]
    fixed = tuple(
        PlacedProp(_FIXED_ASSET, region.rectangle())
        for _, region in layout.fixed_obstacles
    )
    floor = list(fixed) + list(factory.work_items)
    floor_rects = [p.rectangle for p in floor]
    y0, y1 = layout.aisle_y_m
    footprint = geometry.unloaded_footprint
    for name, pose, fp in (
        *(("parking", p, footprint) for p in layout.parking[:trucks]),
        *(("parking exit", p, footprint) for p in exits),
        *(("parking entry", p, footprint) for p in prepark),
        *(("aisle entry", p, footprint) for p in entries),
    ):
        if not collision_free_pose(pose, floor_rects, fp, layout.bounds):
            raise ValueError(f"{name} {pose} touches the floor for seed {seed}")
    for site in pickups + destinations:
        rect = _pallet(site, geometry)
        if not collision_free_pose(
            Pose2D(rect.x_m, rect.y_m, rect.yaw_rad),
            floor_rects,
            Footprint(rect.length_m / 2, rect.length_m / 2, rect.width_m / 2),
            layout.bounds,
        ):
            raise ValueError(f"pallet site {site} touches the floor for seed {seed}")
    missions = []
    for k in range(trucks):
        rules = []
        for j in range(trucks):
            if j == k:
                continue
            rules.append(PlacedProp(_RULE_ASSET, _pallet(pickups[j], geometry)))
            rules.append(PlacedProp(_RULE_ASSET, _pallet(destinations[j], geometry)))
            rules.append(
                PlacedProp(
                    _TRUCK_ASSET,
                    footprint_rectangle(layout.parking[j], footprint, parked_gap_m),
                )
            )
        for aisle in layout.aisles:
            if aisle is not aisles[k]:
                rules.append(
                    PlacedProp(
                        _RULE_ASSET,
                        Region(aisle.x_min_m, aisle.x_max_m, y0, y1).rectangle(),
                    )
                )
        scenario = TransportScenario(
            seed,
            layout.parking[k],
            pickups[k],
            destinations[k],
            tuple(floor) + tuple(rules),
            layout.bounds,
        )
        missions.append(
            TruckMission(
                truck=k,
                aisle=aisles[k].name,
                parking=layout.parking[k],
                parking_exit=exits[k],
                parking_entry=prepark[k],
                aisle_entry=entries[k],
                pickup=pickups[k],
                destination=destinations[k],
                scenario=scenario,
                pickup_bounds=Bounds(
                    aisles[k].x_min_m,
                    aisles[k].x_max_m,
                    layout.aisle_entry_y_m - 0.5,
                    layout.bounds.y_max_m,
                ),
            )
        )
    return FleetScenario(
        seed=seed,
        layout_version=layout.layout_version,
        bounds=layout.bounds,
        fixed=fixed,
        work_items=factory.work_items,
        loads=factory.loads,
        missions=tuple(missions),
    )
