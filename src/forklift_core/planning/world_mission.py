"""Continuous pallet jobs on a random factory floor: dispatch, keep-outs, stages.

Plan: docs/plans/2026-10-09-fleet-random-worlds.md (W1, W2). A job moves the
pallet at a source slot to a free sink slot: the truck backs out of its
holding place, picks the pallet, delivers it, and drives into a holding place
reserved for it when the job was given. Stages:

    leave, to_pickup, insert, [lift], extract, to_drop, [lower], withdraw, to_holding

Every job is planned against the static floor plus **keep-outs**: every
pallet slot but the job's own two (pallet grown by PALLET_KEEP_M) and every
holding place but the job's own two (an unloaded truck grown by TRUCK_GAP_M).
A pallet that arrives later, or a truck that parks later, therefore never
lands on a route planned earlier, and a waiting truck at a holding place is
off every other route -- the precondition fleet_traffic's no-lock argument
needs. Ranks are the order jobs were given (the Dispatcher counts them).

Plan 2026-10-17 (large random floors): ``od="random"`` takes any pallet to any
free place, whatever the slots' roles; a **chained** job (``start`` -1) begins
where the truck's last drop left it -- no holding place to leave, stages from
``to_pickup`` on.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np

from .fleet_mission import FleetPlan, _chain
from .forked import Forked
from .geometry import Pose2D, Rectangle
from .hybrid_astar import PlannerConfig
from .pallet_mission import (
    SyntheticMissionGeometry,
    _search,
    _straight_plan,
    site_poses,
)
from .world_gen import (
    PALLET_KEEP_M,
    TRUCK_GAP_M,
    Slot,
    WorldMap,
    holding_box,
    holding_exit,
    pallet_box,
)

STAGES = (
    "leave",
    "to_pickup",
    "insert",
    "extract",
    "to_drop",
    "withdraw",
    "to_holding",
)
LOADED_STAGES = frozenset({"extract", "to_drop"})
TRAVEL_STAGES = frozenset({"to_pickup", "to_drop", "to_holding"})


@dataclass(frozen=True)
class Job:
    number: int  # also the truck's traffic rank while it runs
    truck: int
    source: Slot
    sink: Slot
    start: int  # holding place the truck leaves (-1: chained, none)
    home: int  # holding place it ends at
    after: Slot | None = None  # chained: the slot the truck has just lowered on


def keep_outs(
    world: WorldMap,
    geometry: SyntheticMissionGeometry,
    *,
    own_slots: tuple[str, ...],
    own_holding: tuple[int, ...],
) -> list[Rectangle]:
    """Planning obstacles of one job: the floor plus everyone else's places."""
    rects = world.obstacle_rects()
    rects += [
        pallet_box(s.site, geometry).grow(PALLET_KEEP_M).rect()
        for s in world.slots
        if s.name not in own_slots
    ]
    rects += [
        holding_box(p, geometry, TRUCK_GAP_M).rect()
        for k, p in enumerate(world.holding)
        if k not in own_holding
    ]
    return rects


def plan_stage(
    stage: str,
    start: Pose2D,
    job: Job,
    world: WorldMap,
    config: PlannerConfig,
    travel_config: PlannerConfig,
    geometry: SyntheticMissionGeometry,
    extra: list[Rectangle] = (),
):
    """One stage of a job from ``start`` (a travel stage may start anywhere)."""
    own = (job.source.name, job.sink.name)
    near = []
    if stage == "to_pickup" and job.after is not None:
        # A chained job leaves from in front of the pallet just put down: that
        # pallet, not its keep-out, as on the way to a holding place.
        own += (job.after.name,)
        near = [pallet_box(job.after.site, geometry).rect()]
    props = keep_outs(
        world,
        geometry,
        own_slots=own,
        own_holding=(job.start, job.home),
    ) + list(extra)
    bounds = world.bounds
    unloaded, loaded = geometry.unloaded_footprint, geometry.loaded_footprint
    pickup = site_poses(job.source.site, geometry)
    drop = site_poses(job.sink.site, geometry)
    source_pallet = pallet_box(job.source.site, geometry).rect()
    sink_pallet = pallet_box(job.sink.site, geometry).rect()
    home = world.holding[job.home]
    clearance = config.clearance_m
    approach_config = replace(
        config, clearance_m=min(config.clearance_m, geometry.approach_gap_m / 2)
    )
    if stage == "leave":
        return _straight_plan(
            start, holding_exit(start), -1, props, unloaded, bounds, clearance
        )
    if stage == "to_pickup":
        return _chain(
            _search(
                start,
                pickup["prealign"],
                props + [source_pallet] + near,
                unloaded,
                bounds,
                travel_config,
                extended=False,
            ),
            lambda: _straight_plan(
                pickup["prealign"],
                pickup["approach"],
                1,
                props + [source_pallet],
                unloaded,
                bounds,
                approach_config.clearance_m,
            ),
        )
    if stage == "insert":
        return _straight_plan(
            start, pickup["inserted"], 1, props, unloaded, bounds, clearance
        )
    if stage == "extract":
        return _straight_plan(
            start, pickup["extracted"], -1, props, loaded, bounds, clearance
        )
    if stage == "to_drop":
        return _chain(
            _search(
                start,
                drop["predelivery"],
                props,
                loaded,
                bounds,
                travel_config,
                extended=False,
            ),
            lambda: _straight_plan(
                drop["predelivery"],
                drop["delivery"],
                1,
                props,
                loaded,
                bounds,
                clearance,
            ),
        )
    if stage == "withdraw":
        return _straight_plan(
            start, drop["withdrawn"], -1, props, unloaded, bounds, clearance
        )
    if stage == "to_holding":
        # The source is restocked by now, or will be on the way.
        entry = holding_exit(home)
        done = props + [sink_pallet, source_pallet]
        return _chain(
            _search(
                start,
                entry,
                done,
                unloaded,
                bounds,
                travel_config,
                extended=False,
            ),
            lambda: _straight_plan(entry, home, 1, done, unloaded, bounds, clearance),
        )
    raise ValueError(f"unknown stage {stage}")


def stage_end(stage: str, job: Job, world: WorldMap, geometry) -> Pose2D:
    """Where a stage ends, so the next one can be planned before it is driven."""
    pickup = site_poses(job.source.site, geometry)
    drop = site_poses(job.sink.site, geometry)
    return {
        "leave": holding_exit(world.holding[job.start]),
        "to_pickup": pickup["approach"],
        "insert": pickup["inserted"],
        "extract": pickup["extracted"],
        "to_drop": drop["delivery"],
        "withdraw": drop["withdrawn"],
        "to_holding": world.holding[job.home],
    }[stage]


def plan_job(
    job: Job,
    world: WorldMap,
    config: PlannerConfig,
    travel_config: PlannerConfig,
    geometry: SyntheticMissionGeometry,
    *,
    first_stage: str = "leave",
    start: Pose2D | None = None,
    extra: list[Rectangle] = (),
    forked: bool = False,
) -> FleetPlan:
    """Stages from ``first_stage`` (default: all of them), or the first failure.

    Each stage starts where the one before ends by stage_end, not by its path,
    so with ``forked`` every stage is planned at once in its own forked
    process; the stages are still taken in order up to the first failure, so
    the plan is the sequential one."""
    order = STAGES[STAGES.index(first_stage) :]
    poses = [world.holding[job.start] if start is None else start]
    poses += [stage_end(stage, job, world, geometry) for stage in order[:-1]]
    args = [(stage, pose, job, world, config, travel_config, geometry, extra) for stage, pose in zip(order, poses, strict=True)]
    runs = [Forked(plan_stage, *a) for a in args] if forked else []
    try:
        stages = {}
        for k, stage in enumerate(order):
            result = runs[k].result() if forked else plan_stage(*args[k])
            if not result.success:
                return FleetPlan(False, f"{stage}:{result.status}", stages)
            stages[stage] = result
        return FleetPlan(True, "success", stages)
    finally:
        for run in runs:
            run.cancel()


@dataclass
class Dispatcher:
    """Pallets at slots, reservations and job numbers; seeded choices.

    Sources start filled with probability ``fill``; an emptied source fills
    again ``restock_s`` later, a filled sink empties ``ship_s`` later -- the
    flow never runs dry. ``od="random"``: every slot starts filled with
    probability ``fill``, a job takes any pallet to any free slot and nothing
    restocks or ships -- the pallets only move. Holding places are taken by
    idle trucks or reserved for a job's end.
    """

    world: WorldMap
    rng: np.random.Generator
    fill: float = 0.6
    restock_s: float = 60.0
    ship_s: float = 120.0
    od: str = "roles"  # "roles": sources to sinks; "random": any pallet to any free slot
    occupied: dict = field(default_factory=dict)
    reserved: set = field(default_factory=set)
    holding_taken: dict = field(default_factory=dict)  # place -> truck
    timers: list = field(default_factory=list)  # (time, slot, occupied)
    held_back: set = field(default_factory=set)  # (time, slot) of fills put off
    jobs: int = 0
    unreachable: set = field(default_factory=set)  # (source, sink) pairs that failed
    ranks: int = 0  # traffic ranks handed out, one per route given

    def __post_init__(self) -> None:
        if self.od not in ("roles", "random"):
            raise ValueError(f"od must be roles or random, not {self.od}")
        for slot in self.world.slots:
            self.occupied[slot.name] = (
                (self.od == "random" or slot.role == "source") and self.rng.random() < self.fill
            )

    def spawn(self, trucks: int) -> list[int]:
        places = self.rng.choice(len(self.world.holding), trucks, replace=False)
        for k, place in enumerate(places):
            self.holding_taken[int(place)] = k
        return [int(p) for p in places]

    def tick(self, now: float, held=None) -> None:
        """Apply the timers due. held(slot), if given, says a truck stands over
        the slot: its pallet comes back once it is clear, not under the truck."""
        keep = []
        for t in self.timers:
            if t[0] > now:
                keep.append(t)
            elif t[2] and held is not None and held(t[1]):
                self.held_back.add(t[:2])
                keep.append(t)
            else:
                self.occupied[t[1]] = t[2]
        self.timers = keep

    def next_job(self, truck: int, at: int, after: Slot | None = None) -> Job | None:
        """A job for the truck at holding place ``at``, or (``at`` -1) chained
        on from its drop at ``after``, whose pallet it does not pick again."""
        slots = {s.name: s for s in self.world.slots}
        anyrole = self.od == "random"
        sources = [
            n
            for n, s in slots.items()
            if (anyrole or s.role == "source")
            and self.occupied[n]
            and n not in self.reserved
            and (after is None or n != after.name)
        ]
        sinks = [
            n
            for n, s in slots.items()
            if (anyrole or s.role == "sink") and not self.occupied[n] and n not in self.reserved
        ]
        homes = [
            k
            for k in range(len(self.world.holding))
            if k not in self.holding_taken or self.holding_taken[k] == truck
        ]
        pairs = [(a, b) for a in sources for b in sinks if (a, b) not in self.unreachable]
        if not pairs or not homes:
            return None
        source, sink = pairs[int(self.rng.integers(len(pairs)))]
        home = homes[int(self.rng.integers(len(homes)))]
        self.reserved |= {source, sink}
        self.holding_taken[home] = truck  # ``at`` stays taken until it leaves
        self.jobs += 1
        return Job(self.jobs, truck, slots[source], slots[sink], at, home, after)

    def free_home(self, job: Job) -> None:
        """Chaining: give back the holding place job would have ended at."""
        if self.holding_taken.get(job.home) == job.truck:
            del self.holding_taken[job.home]

    def take_home(self, job: Job) -> None:
        """The chained job did not start: the truck heads for job's place again."""
        self.holding_taken[job.home] = job.truck

    def next_rank(self) -> int:
        """Traffic rank for a route given now -- a new job's or a re-planned one's:
        behind every route given before, so it adds no constraint to them."""
        self.ranks += 1
        return self.ranks

    def left(self, job: Job) -> None:
        if job.start != job.home and self.holding_taken.get(job.start) == job.truck:
            del self.holding_taken[job.start]

    def lifted(self, job: Job, now: float) -> None:
        self.occupied[job.source.name] = False
        self.reserved.discard(job.source.name)
        if self.od == "roles":
            self.timers.append((now + self.restock_s, job.source.name, True))

    def lowered(self, job: Job, now: float) -> None:
        self.occupied[job.sink.name] = True
        self.reserved.discard(job.sink.name)
        if self.od == "roles":
            self.timers.append((now + self.ship_s, job.sink.name, False))

    def abandon(self, job: Job, *, reached_pickup: bool, unreachable: bool) -> None:
        """A job that cannot go on: free what it held (the truck keeps its place)."""
        if not reached_pickup:
            self.reserved.discard(job.source.name)
        self.reserved.discard(job.sink.name)
        if job.home != job.start and self.holding_taken.get(job.home) == job.truck:
            del self.holding_taken[job.home]
        if unreachable:
            self.unreachable.add((job.source.name, job.sink.name))
