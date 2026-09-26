"""Seeded factory floor built around the original synthetic transport bay.

The bay scenario of a seed is kept exactly as make_scenario draws it, so
perception and pickup conditions stay comparable. A second random stream then
chooses a shipping-yard destination and fills the rest of the hall with
block-stacked pallets, their loads and wall-side clutter. Dimensions come from
the adapter's measured asset bounds; the layout itself is a synthetic choice.
Placement guarantees geometric separation only, never that a route exists.
"""

from collections.abc import Sequence
from dataclasses import dataclass
from math import cos, floor, pi, sin
from pathlib import Path

import numpy as np

from forklift_core._validation import _finite_scalar

from .geometry import Bounds, Footprint, Pose2D, Rectangle, collision_free_pose
from .hybrid_astar import PlannerConfig, PlanResult, plan_hybrid_astar
from .pallet_mission import (
    AssetSpec,
    PalletSite,
    PlacedProp,
    SyntheticMissionGeometry,
    TransportScenario,
    destination_reservations,
    make_scenario,
)

# Independent of make_scenario's stream, so the bay draws stay untouched.
_FACTORY_STREAM = 0x46414354
_ITEM_GAP_M = 0.02
_CLUTTER_GAP_M = 0.10
_MAX_LOADS_PER_SIDE = 2


@dataclass(frozen=True)
class Region:
    """Axis-aligned world rectangle in metres."""

    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float

    def __post_init__(self) -> None:
        for value in (self.x_min_m, self.x_max_m, self.y_min_m, self.y_max_m):
            _finite_scalar(value, "region limit")
        if self.x_max_m <= self.x_min_m or self.y_max_m <= self.y_min_m:
            raise ValueError("region maxima must exceed minima")

    def contains(self, other: "Region") -> bool:
        return (
            self.x_min_m <= other.x_min_m
            and other.x_max_m <= self.x_max_m
            and self.y_min_m <= other.y_min_m
            and other.y_max_m <= self.y_max_m
        )

    def overlaps(self, other: "Region") -> bool:
        """Shared interior; touching edges do not overlap."""
        return (
            self.x_min_m < other.x_max_m
            and other.x_min_m < self.x_max_m
            and self.y_min_m < other.y_max_m
            and other.y_min_m < self.y_max_m
        )

    def rectangle(self) -> Rectangle:
        return Rectangle(
            (self.x_min_m + self.x_max_m) / 2,
            (self.y_min_m + self.y_max_m) / 2,
            self.x_max_m - self.x_min_m,
            self.y_max_m - self.y_min_m,
        )

    def bounds(self) -> Bounds:
        return Bounds(self.x_min_m, self.x_max_m, self.y_min_m, self.y_max_m)


@dataclass(frozen=True)
class ClutterZone:
    region: Region
    count: int

    def __post_init__(self) -> None:
        if isinstance(self.count, bool) or not isinstance(self.count, int):
            raise ValueError("clutter count must be an integer")
        if self.count < 0:
            raise ValueError("clutter count must be nonnegative")


@dataclass(frozen=True)
class FactoryLayout:
    """A hall, what must stay empty, and where items, docks and the survey go."""

    layout_version: str
    hall: Bounds
    keep_clear: tuple[Region, ...]
    slot_pitch_m: tuple[float, float]
    slot_occupancy: float
    slot_jitter_m: float
    slot_yaw_jitter_rad: float
    load_height_m: tuple[float, float]
    empty_pallet_fraction: float
    storage: tuple[Region, ...]
    docks: tuple[PalletSite, ...]
    dock_jitter_m: float
    dock_yaw_jitter_rad: float
    clutter: tuple[ClutterZone, ...]
    survey_route: tuple[Pose2D, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.layout_version, str) or not self.layout_version:
            raise ValueError("layout_version must be a nonempty string")
        hall = Region(
            self.hall.x_min_m, self.hall.x_max_m, self.hall.y_min_m, self.hall.y_max_m
        )
        if (
            len(self.slot_pitch_m) != 2
            or min(_finite_scalar(value, "slot pitch") for value in self.slot_pitch_m)
            <= 0
        ):
            raise ValueError("slot pitch needs two positive values")
        if not 0 <= _finite_scalar(self.slot_occupancy, "slot_occupancy") <= 1:
            raise ValueError("slot_occupancy must lie in [0, 1]")
        for name in (
            "slot_jitter_m",
            "slot_yaw_jitter_rad",
            "dock_jitter_m",
            "dock_yaw_jitter_rad",
        ):
            if _finite_scalar(getattr(self, name), name) < 0:
                raise ValueError(f"{name} must be nonnegative")
        if len(self.load_height_m) != 2:
            raise ValueError("load_height_m needs a lower and an upper height")
        low, high = (_finite_scalar(v, "load height") for v in self.load_height_m)
        if not 0 < low <= high:
            raise ValueError("load heights must satisfy 0 < low <= high")
        if not 0 <= _finite_scalar(self.empty_pallet_fraction, "empty fraction") <= 1:
            raise ValueError("empty_pallet_fraction must lie in [0, 1]")
        placed = list(self.storage) + [zone.region for zone in self.clutter]
        for region in list(self.keep_clear) + placed:
            if not hall.contains(region):
                raise ValueError(f"region {region} leaves the hall")
        for index, region in enumerate(placed):
            if any(region.overlaps(zone) for zone in self.keep_clear):
                raise ValueError(f"region {region} enters keep_clear")
            if any(region.overlaps(other) for other in placed[index + 1 :]):
                raise ValueError(f"region {region} overlaps another region")
        if not self.docks:
            raise ValueError("at least one dock is required")
        if len(self.survey_route) < 2:
            raise ValueError("the survey route needs at least two poses")
        for x, y in [(d.x_m, d.y_m) for d in self.docks] + [
            (p.x_m, p.y_m) for p in self.survey_route
        ]:
            if not (
                hall.x_min_m < x < hall.x_max_m and hall.y_min_m < y < hall.y_max_m
            ):
                raise ValueError(f"({x}, {y}) lies outside the hall")


@dataclass(frozen=True)
class FactoryAssets:
    """Measured work-item assets: the stacked pallet, boxes, loose clutter."""

    pallet: AssetSpec
    loads: tuple[AssetSpec, ...]
    clutter: tuple[AssetSpec, ...]


@dataclass(frozen=True)
class StackedProp:
    """An item resting on another; base_height_m is its underside above floor."""

    asset: AssetSpec
    rectangle: Rectangle
    base_height_m: float


@dataclass(frozen=True)
class FactoryScenario:
    """The bay scenario on the hall floor, plus what fills the hall.

    transport.props is the bay props followed by work_items, so every planning
    and runtime check sees the factory. Loads sit inside their pallet's
    rectangle and add no planning obstacle of their own. pickup_bounds is the
    original bay, for plan_transport's pickup-side stages.
    """

    transport: TransportScenario
    pickup_bounds: Bounds
    work_items: tuple[PlacedProp, ...]
    loads: tuple[StackedProp, ...]
    layout_version: str


_REGION_KEYS = {"x_min_m", "x_max_m", "y_min_m", "y_max_m"}
_LAYOUT_KEYS = {
    "layout_version",
    "hall",
    "keep_clear",
    "slot_pitch_m",
    "slot_occupancy",
    "slot_jitter_m",
    "slot_yaw_jitter_rad",
    "load_height_m",
    "empty_pallet_fraction",
    "storage",
    "docks",
    "dock_jitter_m",
    "dock_yaw_jitter_rad",
    "clutter",
    "survey_route",
}


def _exact_keys(data: object, keys: set[str], where: str) -> dict:
    if not isinstance(data, dict):
        raise ValueError(f"{where} must be a mapping")
    unknown, missing = set(data) - keys, keys - set(data)
    if unknown:
        raise ValueError(f"unknown keys in {where}: {sorted(unknown)}")
    if missing:
        raise ValueError(f"missing keys in {where}: {sorted(missing)}")
    return data


def _region(data: object, where: str) -> Region:
    data = _exact_keys(data, _REGION_KEYS, where)
    return Region(data["x_min_m"], data["x_max_m"], data["y_min_m"], data["y_max_m"])


def _triples(data: object, where: str) -> list[tuple[float, float, float]]:
    if not isinstance(data, list) or not all(
        isinstance(item, list) and len(item) == 3 for item in data
    ):
        raise ValueError(f"{where} must be a list of [x, y, yaw] triples")
    return [tuple(item) for item in data]


def load_factory_layout(path: Path) -> FactoryLayout:
    """Read a layout YAML strictly: unknown or missing keys raise ValueError."""
    import yaml

    data = _exact_keys(
        yaml.safe_load(Path(path).read_text(encoding="utf-8")), _LAYOUT_KEYS, "layout"
    )
    hall = _region(data["hall"], "hall")
    return FactoryLayout(
        layout_version=data["layout_version"],
        hall=hall.bounds(),
        keep_clear=tuple(_region(r, "keep_clear") for r in data["keep_clear"]),
        slot_pitch_m=tuple(data["slot_pitch_m"]),
        slot_occupancy=data["slot_occupancy"],
        slot_jitter_m=data["slot_jitter_m"],
        slot_yaw_jitter_rad=data["slot_yaw_jitter_rad"],
        load_height_m=tuple(data["load_height_m"]),
        empty_pallet_fraction=data["empty_pallet_fraction"],
        storage=tuple(_region(r, "storage") for r in data["storage"]),
        docks=tuple(PalletSite(*t) for t in _triples(data["docks"], "docks")),
        dock_jitter_m=data["dock_jitter_m"],
        dock_yaw_jitter_rad=data["dock_yaw_jitter_rad"],
        clutter=tuple(
            ClutterZone(
                _region(
                    _exact_keys(z, {"region", "count"}, "clutter")["region"], "clutter"
                ),
                z["count"],
            )
            for z in data["clutter"]
        ),
        survey_route=tuple(
            Pose2D(*t) for t in _triples(data["survey_route"], "survey_route")
        ),
    )


def _rect_footprint(rect: Rectangle, margin_m: float = 0.0):
    return (
        Pose2D(rect.x_m, rect.y_m, rect.yaw_rad),
        Footprint(
            rect.length_m / 2 + margin_m,
            rect.length_m / 2 + margin_m,
            rect.width_m / 2 + margin_m,
        ),
    )


def _free(rect, others, reservations, bounds, gap_m, reserve_margin_m) -> bool:
    pose, footprint = _rect_footprint(rect, gap_m)
    if not collision_free_pose(pose, others, footprint, bounds):
        return False
    return all(
        collision_free_pose(
            reserved_pose, [rect], reserved_footprint, bounds, margin_m=reserve_margin_m
        )
        for reserved_pose, reserved_footprint in reservations
    )


def _load_grid(pallet: AssetSpec, load: AssetSpec) -> tuple[int, int]:
    along = min(_MAX_LOADS_PER_SIDE, floor(pallet.length_m / load.length_m))
    across = min(_MAX_LOADS_PER_SIDE, floor(pallet.width_m / load.width_m))
    if along < 1 or across < 1:
        raise ValueError(f"load {load.uri} does not fit on pallet {pallet.uri}")
    return along, across


def _stack(pallet_rect, assets, layers, load) -> list[StackedProp]:
    along, across = _load_grid(assets.pallet, load)
    c, s = cos(pallet_rect.yaw_rad), sin(pallet_rect.yaw_rad)
    items = []
    for layer in range(layers):
        for i in range(along):
            for j in range(across):
                u = (i + 0.5 - along / 2) * load.length_m
                v = (j + 0.5 - across / 2) * load.width_m
                items.append(
                    StackedProp(
                        load,
                        Rectangle(
                            pallet_rect.x_m + c * u - s * v,
                            pallet_rect.y_m + s * u + c * v,
                            load.length_m,
                            load.width_m,
                            pallet_rect.yaw_rad,
                        ),
                        assets.pallet.height_m + layer * load.height_m,
                    )
                )
    return items


def make_factory_scenario(
    seed: int,
    layout: FactoryLayout,
    bay_assets: Sequence[AssetSpec],
    factory_assets: FactoryAssets,
    obstacle_count: int = 4,
    *,
    geometry: SyntheticMissionGeometry | None = None,
    max_attempts: int = 2000,
) -> FactoryScenario:
    """Keep the seed's bay scenario and fill the hall around it.

    Storage slots whose pallet would touch a kept-clear zone, the delivery
    corridor or a neighbour stay empty. Clutter that cannot be placed within
    max_attempts raises ValueError instead of silently shrinking the count.
    """
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    for load in factory_assets.loads:
        _load_grid(factory_assets.pallet, load)
    bay = make_scenario(seed, bay_assets, obstacle_count, geometry=geometry)
    rng = np.random.default_rng([seed, _FACTORY_STREAM])
    hall = layout.hall
    dock = layout.docks[int(rng.integers(len(layout.docks)))]
    lateral = float(rng.uniform(-layout.dock_jitter_m, layout.dock_jitter_m))
    destination = PalletSite(
        dock.x_m - lateral * sin(dock.yaw_rad),
        dock.y_m + lateral * cos(dock.yaw_rad),
        dock.yaw_rad
        + float(rng.uniform(-layout.dock_yaw_jitter_rad, layout.dock_yaw_jitter_rad)),
    )
    reservations = destination_reservations(destination, geometry)
    forbidden = [zone.rectangle() for zone in layout.keep_clear] + [
        Region(
            bay.bounds.x_min_m,
            bay.bounds.x_max_m,
            bay.bounds.y_min_m,
            bay.bounds.y_max_m,
        ).rectangle()
    ]
    placed: list[Rectangle] = [prop.rectangle for prop in bay.props]
    items: list[PlacedProp] = []
    loads: list[StackedProp] = []
    pitch_x, pitch_y = layout.slot_pitch_m
    pallet = factory_assets.pallet
    for region in layout.storage:
        columns = floor((region.x_max_m - region.x_min_m) / pitch_x)
        rows = floor((region.y_max_m - region.y_min_m) / pitch_y)
        x0 = (region.x_min_m + region.x_max_m - (columns - 1) * pitch_x) / 2
        y0 = (region.y_min_m + region.y_max_m - (rows - 1) * pitch_y) / 2
        for i in range(columns):
            for j in range(rows):
                # Draw every slot's values even when it stays empty, so one
                # slot's outcome never shifts the next slot's randomness.
                occupied = rng.random() < layout.slot_occupancy
                dx, dy = rng.uniform(-layout.slot_jitter_m, layout.slot_jitter_m, 2)
                dyaw = rng.uniform(
                    -layout.slot_yaw_jitter_rad, layout.slot_yaw_jitter_rad
                )
                empty = rng.random() < layout.empty_pallet_fraction
                target_m = rng.uniform(*layout.load_height_m)
                load = (
                    factory_assets.loads[int(rng.integers(len(factory_assets.loads)))]
                    if factory_assets.loads
                    else None
                )
                # Whole layers up to the drawn height, never an empty "load".
                layers = (
                    0
                    if empty or load is None
                    else max(1, floor(target_m / load.height_m))
                )
                if not occupied:
                    continue
                rect = Rectangle(
                    x0 + i * pitch_x + float(dx),
                    y0 + j * pitch_y + float(dy),
                    pallet.length_m,
                    pallet.width_m,
                    float(dyaw),
                )
                if not _free(
                    rect,
                    placed + forbidden,
                    reservations,
                    hall,
                    _ITEM_GAP_M,
                    geometry.spawn_clearance_m,
                ):
                    continue
                placed.append(rect)
                items.append(PlacedProp(pallet, rect))
                if load is not None and layers:
                    loads.extend(_stack(rect, factory_assets, layers, load))
    for zone in layout.clutter:
        if zone.count and not factory_assets.clutter:
            raise ValueError("clutter assets are required to fill clutter zones")
        region_bounds = zone.region.bounds()
        count = 0
        for _ in range(max_attempts):
            if count == zone.count:
                break
            asset = factory_assets.clutter[
                int(rng.integers(len(factory_assets.clutter)))
            ]
            rect = Rectangle(
                float(rng.uniform(zone.region.x_min_m, zone.region.x_max_m)),
                float(rng.uniform(zone.region.y_min_m, zone.region.y_max_m)),
                asset.length_m,
                asset.width_m,
                float(rng.uniform(-pi, pi)),
            )
            pose, footprint = _rect_footprint(rect)
            if not collision_free_pose(pose, [], footprint, region_bounds):
                continue
            if not _free(
                rect,
                placed + forbidden,
                reservations,
                hall,
                _CLUTTER_GAP_M,
                geometry.spawn_clearance_m,
            ):
                continue
            placed.append(rect)
            items.append(PlacedProp(asset, rect))
            count += 1
        if count != zone.count:
            raise ValueError(
                f"could not place {zone.count} clutter items in {zone.region} "
                f"for seed {seed} after {max_attempts} attempts"
            )
    transport = TransportScenario(
        seed,
        bay.start_rear,
        bay.pickup,
        destination,
        bay.props + tuple(items),
        hall,
    )
    return FactoryScenario(
        transport, bay.bounds, tuple(items), tuple(loads), layout.layout_version
    )


def plan_survey_route(
    scenario: FactoryScenario,
    route: Sequence[Pose2D],
    config: PlannerConfig,
    *,
    geometry: SyntheticMissionGeometry | None = None,
) -> PlanResult:
    """Join consecutive survey poses with unloaded Hybrid A* legs in the hall.

    Every factory item and the bay's pickup pallet are obstacles. The legs are
    concatenated into one rear-axle path whose samples pass through every
    waypoint exactly; a direction change between legs is an ordinary cusp. A
    failed leg fails the whole route with status ``leg<i>:<status>`` and no
    partial path.
    """
    geometry = geometry if geometry is not None else SyntheticMissionGeometry()
    transport = scenario.transport
    obstacles = [prop.rectangle for prop in transport.props] + [
        Rectangle(
            transport.pickup.x_m,
            transport.pickup.y_m,
            geometry.pallet_depth_m,
            geometry.pallet_width_m,
            transport.pickup.yaw_rad,
        )
    ]
    legs = []
    for index, (start, goal) in enumerate(zip(route[:-1], route[1:], strict=True)):
        leg = plan_hybrid_astar(
            start,
            goal,
            obstacles,
            geometry.unloaded_footprint,
            transport.bounds,
            config,
        )
        if not leg.success:
            return PlanResult(
                False,
                f"leg{index}:{leg.status}",
                np.empty((0, 3)),
                np.empty(0, dtype=np.int8),
                np.empty(0),
                0.0,
                sum(item.expanded_nodes for item in legs) + leg.expanded_nodes,
            )
        legs.append(leg)
    poses = np.concatenate([legs[0].poses] + [leg.poses[1:] for leg in legs[1:]])
    directions = np.concatenate(
        [legs[0].directions] + [leg.directions[1:] for leg in legs[1:]]
    )
    curvatures = np.concatenate(
        [legs[0].curvatures_inv_m] + [leg.curvatures_inv_m[1:] for leg in legs[1:]]
    )
    return PlanResult(
        True,
        "success",
        poses,
        directions,
        curvatures,
        sum(leg.length_m for leg in legs),
        sum(leg.expanded_nodes for leg in legs),
    )
