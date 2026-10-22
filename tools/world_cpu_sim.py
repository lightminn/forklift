"""Run a fleet with continuous jobs on a random factory floor (CPU, kinematic).

    python tools/world_cpu_sim.py --structure rack --seeds 100 --trucks 10 \\
        --condition C0 --output <dir>

Plan: docs/plans/2026-10-09-fleet-random-worlds.md (W1-W5). The floor comes
from world_gen, jobs from world_mission's Dispatcher, traffic from
fleet_traffic (ranks = the order jobs were given: a new job goes to the back, a
re-plan keeps its rank; safe stops) with TA/RA resolution on top
(docs/plans/2026-10-10-fleet-ta-ra.md, world_resolution); ``--traffic
windows`` swaps ranks and safe stops for time-window tokens at route crossings
(docs/plans/2026-10-11-fleet-time-windows.md): a new job whose route would
leave no lock-free order waits at its holding place (retried every 5 s, given
up after HOLD_MAX_S), and a TA or RA path that would is turned down. A truck
its protective field has held for ``--yield-s`` backs straight away from it by up
to ``--yield-back-m`` and comes back, so that a person it stopped for can get out
(plan 2026-10-11 (h); both traffic modes).
``--vehicle physics`` (docs/plans/2026-10-12-fleet-physical-time-windows.md)
gives each job a load (``--load-kg`` range) and drives under fleet_physics'
load-dependent limits (speed caps braking toward each stop, speeding up and
braking with the limit of the force's direction); ``--eta physics`` gives the
windows the same profile. Otherwise trucks follow
their paths perfectly (0.1 s steps, a stop at each gear change, 4.5 s to lift
and to lower); what they believe and where they are differ only under pose
noise. Conditions (the plan's table):

    C0 none · C1 faults · C2 communication · C3 pose noise · C4 people · C5 all

Large random floors (docs/plans/2026-10-17-fleet-large-random-chained.md):
``--world-scale`` stretches the halls, ``--od random`` takes any pallet to any
free slot, and ``--chain`` lets a truck that has just withdrawn from a drop take
the next job from there (rank kept, as for a re-plan) instead of driving to its
holding place first; with no job to take, or none the traffic mode admits, it
drives there as before.

Safety is judged on the true poses: truck-truck (stopped trucks included),
truck-floor (static floor, spills, pallets standing at slots) and truck-person
contact.
Writes <output>/<structure>_<seed>_<condition>_<trucks>/result.json,
trajectory.npz (1 Hz) and world.json; one summary line per run on stdout.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml

from forklift_core.planning.fleet_physics import Chassis, Load, limits, speed_caps
from forklift_core.planning.fleet_traffic import (
    MODES,
    TrafficManager,
    arc_lengths,
    oriented_rects,
    overlap_matrix,
    pose_at,
)
from forklift_core.planning.geometry import Pose2D
from forklift_core.planning.pallet_mission import (
    make_transport_planner_config,
    speculate,
)
from forklift_core.planning.world_gen import (
    PALLET_KEEP_M,
    Box,
    make_world,
    pallet_box,
    world_record,
)
from forklift_core.planning.world_mission import (
    LOADED_STAGES,
    STAGES,
    TRAVEL_STAGES,
    Dispatcher,
    Job,
    plan_job,
)
from forklift_core.planning.world_people import (
    PERSON_RADIUS,
    People,
    protective_block_m,
)
from forklift_core.planning.world_resolution import (
    make_room,
    own_pallets_keepout,
    rectangles,
    resolution_counts,
    resolve,
    wait_stats,
)

ROOT = Path(__file__).resolve().parents[1]
DT_S = 0.1
LIFT_S = 4.5
IDLE_RANK = 10**9
HOLD_MAX_S = 60.0  # windows: a job held back this long is given back
CONDITIONS = {
    "C0": {},
    "C1": {"faults": True},
    "C2": {"comm": True},
    "C3": {"noise": True},
    "C4": {"people": 10},
    "C5": {"faults": True, "comm": True, "noise": True, "people": 10},
}
COMM_DELAY_S = 0.5
COMM_LOSS = 0.2
NOISE_LIMIT_M = 0.17
NOISE_STEP_M = 0.01  # random-walk step per second
NOISE_YAW_LIMIT = 0.02
# Stage codes in trajectory.npz: index into LOG_STAGES.
LOG_STAGES = ("idle", *STAGES, "lift", "lower", "broken")
# Last trajectory.npz column: 0 clear, 1 waiting for a truck (TA), 2 in RA.
ALERT_CLEAR, ALERT_TA, ALERT_RA = 0, 1, 2


def _fleet_tool():
    path = ROOT / "tools/fleet_cpu_sim.py"
    spec = importlib.util.spec_from_file_location("fleet_cpu_sim", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _box_rects(boxes) -> np.ndarray:
    if not boxes:
        return np.zeros((0, 6))
    b = np.array([[x.x0, x.y0, x.x1, x.y1] for x in boxes])
    return np.column_stack(
        (
            (b[:, 0] + b[:, 2]) / 2,
            (b[:, 1] + b[:, 3]) / 2,
            (b[:, 2] - b[:, 0]) / 2,
            (b[:, 3] - b[:, 1]) / 2,
            np.ones(len(b)),
            np.zeros(len(b)),
        )
    )


class Truck:
    """A kinematic follower doing one job after another."""

    def __init__(self, k, place, world, geometry, settings):
        self.k, self.name = k, f"truck_{k}"
        self.world, self.geometry = world, geometry
        self.place = place  # holding place it stands at (idle) or heads for
        self.pose = np.array(
            [world.holding[place].x_m, world.holding[place].y_m, world.holding[place].yaw_rad]
        )
        self.speeds = {
            "leave": settings["withdraw_speed_mps"],
            "to_pickup": settings["approach_speed_mps"],
            "insert": settings["insert_speed_mps"],
            "extract": settings["extract_speed_mps"],
            "to_drop": settings["transport_speed_mps"],
            "withdraw": settings["withdraw_speed_mps"],
            "to_holding": settings["approach_speed_mps"],
        }
        self.accel = settings["drive_acceleration_mps2"]
        self.physics = None  # (chassis, tip share, accel cap, lift while loaded): physics vehicle or timing
        self.drive_physics = False  # --vehicle physics
        self.eta_physics = False
        self.load_kg = 0.0
        self.lims = None  # (empty, loaded) fleet_physics.Limits for this job's load
        self._caps_key, self._caps = None, None
        self.stops = 0
        self.state = "idle"
        self.idle_until = 0.0
        self.job: Job | None = None
        self.items: list = []
        self.item = -1
        self.loaded = False
        self.segments: list = []
        self.segment = 0
        self.progress = 0.0
        self.v = 0.0
        self.wait_left = 0.0
        self.offsets: dict[str, float] = {}
        self.route_end_m = 0.0
        self.granted_m = 0.0
        self.pace_mps = None  # queues mode with --pace: the permit's speed cap
        self.route_version = 0  # permits and reports for an older route are ignored
        self.route_start = self.pose.copy()
        self.error = np.zeros(3)  # truth - believed (pose noise)
        self.stopped_until = 0.0  # temporary breakdown
        self.pending_replan = False
        self.chain = False  # --chain: ask for the next job before to_holding
        self.held = None  # windows: (job, plan, since) waiting for a lock-free order
        self.jobs_done = 0
        self.job_started = 0.0
        self.waiting_s = 0.0
        self.person_stops_s = 0.0
        self.field_s = 0.0  # how long in a row its protective field has held it

    # -- poses -------------------------------------------------------------------
    def truth(self) -> np.ndarray:
        return self.pose + self.error

    def footprint(self):
        g = self.geometry
        return g.loaded_footprint if self.loaded else g.unloaded_footprint

    @property
    def stage(self) -> str:
        if self.state != "job":
            return self.state
        return self.items[self.item][1]

    def log_stage(self, broken: bool) -> int:
        return LOG_STAGES.index("broken" if broken else self.stage)

    # -- jobs --------------------------------------------------------------------
    @staticmethod
    def job_items(plan) -> list:
        items = []
        for stage in STAGES:
            if stage not in plan.stages:  # a chained job has no leave
                continue
            items.append(("path", stage, plan.stages[stage]))
            if stage == "insert":
                items.append(("wait", "lift", LIFT_S))
            if stage == "to_drop":
                items.append(("wait", "lower", LIFT_S))
        return items

    def set_load(self, kg):
        """This job's load and the limits it gives (physics vehicle or timing)."""
        self.load_kg = kg
        if self.physics is not None:
            chassis, share, cap, lift = self.physics
            self.lims = (limits(chassis, None, 0.0, share, cap), limits(chassis, Load(kg), lift, share, cap))

    def limits_for(self, stage):
        return self.lims[1 if stage in LOADED_STAGES else 0]

    def start_job(self, job, plan, now, manager, rank):
        self.job, self.state, self.job_started = job, "job", now
        self.items = self.job_items(plan)
        self.item = -1
        self.set_route(manager, 0, rank=rank)
        self.next_item(now, manager, None)

    def chunks(self, items, first_item) -> tuple[list, list]:
        """Traffic chunks (poses, footprint, speed, stop before) of items'
        paths from first_item, and those paths as (stage, result)."""
        g = self.geometry
        paths, out, pause = [], [], 0.0
        for kind, stage, payload in items[first_item:]:
            if kind == "wait":
                pause += payload
                continue
            footprint = g.loaded_footprint if stage in LOADED_STAGES else g.unloaded_footprint
            timing = (self.speeds[stage], pause) + ((self.limits_for(stage),) if self.eta_physics else ())
            out.append((np.asarray(payload.poses), footprint, *timing))
            paths.append((stage, payload))
            pause = 0.0
        return out, paths

    def set_route(self, manager, first_item, rank=None):
        chunks, paths = self.chunks(self.items, first_item)
        offsets = manager.set_route(self.name, chunks, rank=rank)
        self.offsets = dict(zip((s for s, _ in paths), offsets, strict=True))
        self.route_end_m = offsets[-1] + float(arc_lengths(paths[-1][1].poses)[-1])
        self.granted_m = 0.0
        self.pace_mps = None
        self.route_version += 1
        self.route_start = self.pose.copy()

    def go_idle(self, now, manager, rng):
        self.state, self.job, self.items, self.item = "idle", None, [], -1
        self.idle_until = now + float(rng.uniform(2.0, 10.0))
        manager.set_route(
            self.name, [(self.pose.reshape(1, 3), self.footprint())], rank=IDLE_RANK + self.k
        )
        self.granted_m = 0.0
        self.pace_mps = None
        self.route_version += 1
        self.route_start = self.pose.copy()

    def next_item(self, now, manager, dispatcher):
        if self.item >= 0 and dispatcher is not None:
            kind, stage, _ = self.items[self.item]
            if stage == "leave":
                dispatcher.left(self.job)
            if stage == "lift":
                self.loaded = True
                dispatcher.lifted(self.job, now)
            if stage == "lower":
                self.loaded = False
                dispatcher.lowered(self.job, now)
        self.item += 1
        if self.item >= len(self.items):
            return "done"
        if self.chain and self.items[self.item][1] == "to_holding":
            return "chain"  # the caller starts the next job or enter_item()s this one
        self.enter_item(manager)
        return "next"

    def enter_item(self, manager):
        kind, stage, payload = self.items[self.item]
        if kind == "wait":
            self.wait_left = payload
        else:
            self.loaded = stage in LOADED_STAGES
            self.segments = _segments(payload)
            self.segment, self.progress, self.v = 0, 0.0, 0.0
        manager.set_footprint(self.name, self.footprint())

    def route_progress(self) -> float:
        if self.state != "job":
            return 0.0
        kind, stage, _ = self.items[self.item]
        if kind == "wait":
            return self.offsets[self.items[self.item + 1][1]]
        return self.offsets[stage] + self.segments[self.segment][2] + self.progress

    def step(self, now, manager, dispatcher, person_block_m):
        """Advance by DT_S inside the grant; returns 'done' when the job ends."""
        if self.state != "job" or now < self.stopped_until:
            self.v, self.field_s = 0.0, 0.0
            return None
        kind, stage, payload = self.items[self.item]
        if kind == "wait":
            self.field_s = 0.0
            self.wait_left -= DT_S
            if self.wait_left <= 1e-9:
                return self.next_item(now, manager, dispatcher)
            return None
        poses = self.segments[self.segment][0]
        s = arc_lengths(poses)
        remaining = s[-1] - self.progress
        allowed = max(self.granted_m - self.route_progress(), 0.0)
        field = person_block_m is not None and max(person_block_m, 0.0) < allowed
        if person_block_m is not None:
            allowed = min(allowed, max(person_block_m, 0.0))
        room = min(remaining, allowed)
        end_limited = allowed >= remaining - 1e-9
        speed = self.speeds[stage]
        moving_before = self.v > 0.05
        if not self.drive_physics:
            target = min(speed, math.sqrt(2 * self.accel * max(room - 0.01, 0.0)))
            up = self.accel
        else:
            lim = self.limits_for(stage)
            direction = self.segments[self.segment][1]
            key = (self.segment, stage)  # with the segments list itself, so its id cannot be reused
            if self._caps_key is None or self._caps_key[0] is not self.segments or self._caps_key[1:] != key:
                self._caps_key, self._caps = (self.segments, *key), speed_caps(poses, speed, lim)
            up, down = lim.along(direction, True), lim.along(direction, False)
            cap = float(np.interp(self.progress, *self._caps))
            target = min(cap, math.sqrt(2 * down * max(room - 0.01, 0.0)))
        if self.pace_mps is not None:
            target = min(target, self.pace_mps)
        if end_limited:
            target = max(target, min(speed, 0.05))
        self.v = min(self.v + up * DT_S, target)
        advance = min(self.v * DT_S, room)
        if end_limited and remaining - advance < 0.002:
            advance = remaining
        if advance <= 1e-6 and room < remaining:
            self.stops += moving_before
            self.waiting_s += DT_S
            if person_block_m is not None:
                self.person_stops_s += DT_S
        self.field_s = self.field_s + DT_S if advance <= 1e-6 and field else 0.0
        self.progress += max(advance, 0.0)
        self.pose = pose_at(poses, s, self.progress)
        if self.progress >= s[-1] - 1e-6:
            self.v = 0.0
            if self.segment + 1 < len(self.segments):
                self.segment += 1
                self.progress = 0.0
            else:
                return self.next_item(now, manager, dispatcher)
        return None


def _segments(result):
    poses, directions = np.asarray(result.poses), np.asarray(result.directions)
    if len(poses) < 2:
        return [(poses, int(directions[0]) if len(directions) else 1, 0.0)]
    s = arc_lengths(poses)
    cuts = [0] + [
        i for i in range(1, len(poses) - 1) if directions[i + 1] != directions[i]
    ]
    cuts.append(len(poses) - 1)
    return [
        (poses[a : b + 1], int(directions[min(a + 1, b)]), s[a])
        for a, b in zip(cuts[:-1], cuts[1:], strict=True)
        if b > a
    ]


def truck_block_m(truck, people_pos) -> float | None:
    """The protective field on the truth, with the people's true positions."""
    if not len(people_pos) or truck.state != "job":
        return None
    kind, _, _ = truck.items[truck.item]
    if kind != "path":
        return None
    return protective_block_m(
        truck.truth(),
        truck.footprint(),
        truck.segments[truck.segment][1],
        truck.v,
        truck.accel,
        people_pos,
    )


def run(structure, seed, trucks_n, condition, args, settings, geometry, configs, out):
    config, travel = configs
    flags = CONDITIONS[condition]
    rng = np.random.default_rng([seed, trucks_n, list(CONDITIONS).index(condition), 7])
    world = make_world(structure, seed, geometry=geometry, trucks=trucks_n, scale=args.world_scale)
    dispatcher = Dispatcher(world, np.random.default_rng([seed, trucks_n, 11]), od=args.od)
    places = dispatcher.spawn(trucks_n)
    cap = args.accel_cap if args.accel_cap is not None else settings["drive_acceleration_mps2"]
    physics = None
    if args.vehicle == "physics" or args.eta == "physics":
        chassis = Chassis.from_urdf(
            args.forklift_urdf, wheel_torque_nm=settings["wheel_torque_nm"],
            wheel_speed_radps=settings["max_wheel_rate_rad_s"], friction=settings["static_friction"],
        )
        physics = (chassis, args.tip_share, cap, settings["lift_target_m"])
    load_lo, load_hi = map(float, args.load_kg.split(","))
    load_rng = np.random.default_rng([seed, trucks_n, list(CONDITIONS).index(condition), 23])
    manager = TrafficManager(
        margin_m=args.margin_m,
        lookahead_m=args.lookahead_m,
        stall_s=args.stall_s,
        ta_s=args.ta_s if args.ta_s > 0 else None,
        ra_s=args.ra_s if args.ra_s > 0 else None,
        cooldown_s=args.cooldown_s,
        mode=args.traffic,
        issue_s=args.issue_s,
        slack_s=args.slack_s,
        window_period_s=args.window_period_s,
        window_evals=args.window_evals,
        age_s=args.age_s,
        park_s=args.park_s,
        accel_mps2=cap,
        pace=args.pace,
        pace_min_mps=args.pace_min_mps,
        queue_opt=args.queue_opt,
        opt_period_s=args.opt_period_s,
        node_limit=args.node_limit,
        opt_horizon_s=args.opt_horizon_s,
        mapf_window_s=args.mapf_window_s,
        mapf_replan_s=args.mapf_replan_s,
        tile_m=args.tile_m,
        tile_horizon_m=args.tile_horizon_m,
    )
    trucks = [Truck(k, p, world, geometry, settings) for k, p in enumerate(places)]
    for t in trucks:
        t.accel = cap
        t.physics = physics
        t.drive_physics = args.vehicle == "physics"
        t.eta_physics = args.eta == "physics"
        t.chain = args.chain
        t.set_load(0.0)
    loads = []
    named = {t.name: t for t in trucks}
    for t in trucks:
        manager.add(t.name, IDLE_RANK + t.k, t.pose, t.footprint())
    for t in trucks:
        t.idle_until = float(rng.uniform(0.0, 5.0))
    static_boxes = [b for _, b in world.obstacles]
    static = _box_rects(static_boxes)
    slot_names = [s.name for s in world.slots]
    slot_rects = _box_rects([pallet_box(s.site, geometry) for s in world.slots])
    slot_keep = {s.name: _box_rects([pallet_box(s.site, geometry).grow(PALLET_KEEP_M)]) for s in world.slots}

    def under_truck(name):
        """A truck stands over the slot's keep-out: no pallet comes back there yet."""
        bodies = oriented_rects(np.array([t.truth() for t in trucks]), [t.footprint() for t in trucks], 0.0)
        return bool(overlap_matrix(slot_keep[name], bodies).any())

    extra_boxes: list[tuple[str, Box]] = []  # spills and broken trucks
    people = People(world, geometry, flags.get("people", 0), rng)
    # Faults: times drawn up front.
    faults = []
    if flags.get("faults"):
        faults = sorted(
            [
                (float(rng.uniform(120, 400)), "pause"),
                (float(rng.uniform(200, 600)), "break"),
                (float(rng.uniform(150, 650)), "spill"),
            ]
        )
    history = []  # (time, poses, progress) for delayed reports
    inbox = []  # (deliver_time, name, granted)
    events, contacts, plans, job_log = [], [], [], []
    trajectory = []
    reported = set()
    now, idle_s, last_total = 0.0, 0.0, None
    status = "time_up"
    dead: set[str] = set()

    def plan(job, first_stage="leave", start=None, avoid=()):
        t0 = time.perf_counter()
        result = plan_job(
            job,
            world,
            config,
            travel,
            geometry,
            first_stage=first_stage,
            start=start,
            extra=[b.rect() for _, b in extra_boxes] + list(avoid),
            forked=args.fork_plans,
        )
        plans.append(
            {
                "job": job.number,
                "truck": job.truck,
                "from": first_stage,
                "status": result.status,
                "wall_s": round(time.perf_counter() - t0, 2),
            }
        )
        return result

    def preconditions(t, after):
        """Record no-lock precondition breaks that involve t's new route
        (windows: no lock-free crossing order for it)."""
        if manager.windows is not None and not manager.windows.last_feasible:
            events.append({"t_s": round(now, 1), "kind": "precondition", "after": after, "truck": t.name, "why": "order_lock"})
        for c in manager.route_conflicts():
            if t.name in (c["truck"], c["route_of"]):
                events.append({"t_s": round(now, 1), "kind": "precondition", "after": after, **c})

    chains = {"started": 0, "no_job": 0, "plan_failed": 0, "refused": 0}

    def chain(t):
        """--chain: t has withdrawn from its drop -- its next job from there, its
        rank kept (as for a re-plan: it may stand on routes given after its own).
        False: no job, none planned or none admitted; it drives to its holding
        place as before."""
        old, old_kg = t.job, t.load_kg
        dispatcher.free_home(old)
        job = dispatcher.next_job(t.k, -1, after=old.sink)
        why = "no_job"
        if job is not None:
            result = plan(job, "to_pickup", Pose2D(*map(float, t.pose)))
            if not result.success:
                # A first stage that fails says something about where t stands, not the pair.
                dispatcher.abandon(job, reached_pickup=False, unreachable=not result.status.startswith("to_pickup:"))
                job_log.append({"job": job.number, "truck": t.k, "status": "plan_failed", "why": result.status, "t_s": round(now, 1), "chained": True})
                why = "plan_failed"
            else:
                load = float(load_rng.uniform(load_lo, load_hi))
                t.set_load(load)
                if manager.windows is not None and not manager.trial_route(t.name, t.chunks(t.job_items(result), 0)[0]):
                    dispatcher.abandon(job, reached_pickup=False, unreachable=False)
                    t.set_load(old_kg)
                    why = "refused"
                else:
                    why = None
        if why is not None:
            dispatcher.take_home(old)
            chains[why] += 1
            return False
        chains["started"] += 1
        t.jobs_done += 1
        job_log.append({"job": old.number, "truck": t.k, "status": "done", "t_s": round(now, 1), "took_s": round(now - t.job_started, 1)})
        loads.append(load)
        t.place = job.home
        t.start_job(job, result, now, manager, None)
        preconditions(t, "chain")
        return True

    def affected(t, box):
        if t.state != "job":
            return False
        mine = manager.trucks[t.name]
        ahead = mine.rects[mine.g >= t.route_progress() - 0.2]
        return bool(overlap_matrix(ahead, _box_rects([box.grow(args.margin_m)])).any())

    def replan(t):
        """Re-plan the rest of t's job round the new obstacles: from where it is
        on a travel stage, or from a lift/lower stop (both start a fresh route)."""
        kind, stage, _ = t.items[t.item]
        if kind == "path":
            first, start, keep = stage, Pose2D(*map(float, t.pose)), t.item
        else:
            keep = t.item + 1  # lift is followed by extract, lower by withdraw
            first = t.items[keep][1]
            start = Pose2D(*map(float, t.pose))
        # Round where the lower-ranked trucks may stop (plan 2026-10-10; windows:
        # round the other trucks); with no way round them, as before.
        stops = rectangles(manager.replan_keepout(t.name, config.clearance_m + 0.05))
        own = own_pallets_keepout(manager, t.name, t.job, stage, dispatcher.occupied, geometry, config.clearance_m + 0.05)
        result = plan(t.job, first, start, stops + own)
        kept = result.success or not stops
        if not result.success and stops:
            result = plan(t.job, first, start, own)
        events.append({"t_s": round(now, 1), "kind": "replan", "truck": t.name, "from": first, "status": result.status, "stops_kept": kept})
        if not result.success:
            return False
        for i in range(keep, len(t.items)):
            kind_i, stage_i, _ = t.items[i]
            if kind_i == "path":
                t.items[i] = (kind_i, stage_i, result.stages[stage_i])
        if keep == t.item:
            t.segments = _segments(t.items[t.item][2])
            t.segment, t.progress, t.v = 0, 0.0, 0.0
        # Rank kept (plan 2026-10-10): sending it to the back left it standing on
        # higher-ranked routes; a lock that the new route makes, RA clears.
        t.set_route(manager, keep)
        preconditions(t, "replan")
        return True

    def advise(advice):
        """Carry out a TA or RA: the truck's stage again, its rank kept."""
        t = named[advice.truck]
        record = {"t_s": round(now, 1), "kind": advice.kind, "truck": t.name, "other": advice.other, "why": advice.why}
        if t.name in dead or t.state != "job" or t.items[t.item][0] != "path":
            events.append({**record, "how": "skipped"})
            return
        _, stage, path = t.items[t.item]
        res = resolve(
            advice,
            manager,
            stage=stage,
            path=path,
            driven_m=t.segments[t.segment][2] + t.progress,
            stage_start_m=t.offsets[stage],
            pose=t.pose,
            job=t.job,
            world=world,
            config=config,
            travel_config=travel,
            geometry=geometry,
            extra=[b.rect() for _, b in extra_boxes]
            + own_pallets_keepout(manager, t.name, t.job, stage, dispatcher.occupied, geometry, config.clearance_m + 0.05),
            wall_m=args.wall_m,
            retreat_max_m=args.retreat_max_m,
            clock=time.perf_counter,
        )
        for what, status, wall_s in res.plans:
            plans.append({"job": t.job.number, "truck": t.k, "from": f"{what}:{stage}", "status": status, "wall_s": wall_s})
        if res.path is not None and manager.windows is not None:
            items = list(t.items)
            items[t.item] = ("path", stage, res.path)
            if not manager.trial_route(t.name, t.chunks(items, t.item)[0]):
                events.append({**record, "how": "failed", "note": "order_lock"})
                manager.failed(advice)
                return
        events.append({**record, **res.record()})
        if res.path is None:
            if res.how == "failed":
                manager.failed(advice)
            return
        t.items[t.item] = ("path", stage, res.path)
        t.segments = _segments(res.path)
        t.segment, t.progress, t.v = 0, 0.0, 0.0
        t.set_route(manager, t.item)
        preconditions(t, advice.kind)
        if advice.kind == "ra":
            manager.resolved(t.name, advice.other)

    def make_way(t):
        """(h) Back t off along its stage so that a person its protective field
        has stopped it for can get out; its rank kept."""
        kind, stage, path = t.items[t.item]
        record = {"t_s": round(now, 1), "kind": "yield", "truck": t.name, "stage": stage}
        if t.name in dead or kind != "path" or stage not in TRAVEL_STAGES:
            events.append({**record, "how": "skipped"})
            return
        floor = np.vstack((static, slot_rects, _box_rects([b for _, b in extra_boxes])))
        res = make_room(manager, t.name, path, t.segments[t.segment][2] + t.progress, t.segments[t.segment][1], args.yield_back_m, floor)
        if res.path is not None and manager.windows is not None:
            items = list(t.items)
            items[t.item] = ("path", stage, res.path)
            if not manager.trial_route(t.name, t.chunks(items, t.item)[0]):
                events.append({**record, "how": "failed", "note": "order_lock"})
                return
        events.append({**record, **res.record()})
        if res.path is None:
            return
        t.items[t.item] = ("path", stage, res.path)
        t.segments = _segments(res.path)
        t.segment, t.progress, t.v = 0, 0.0, 0.0
        t.set_route(manager, t.item)
        preconditions(t, "yield")

    def immobilise(t, event, box_kind):
        """t stops for good and becomes an obstacle; its job is given up and the
        trucks whose remaining route it touches re-plan."""
        dead.add(t.name)
        t.stopped_until = math.inf
        # Grown by both trucks' traffic margins: a route planned round it (with
        # the planner's own clearance) must leave the gap traffic control needs,
        # or every truck on it would wait for this one for good.
        box = _aabb(t, 2 * args.margin_m)
        extra_boxes.append((box_kind, box))
        manager.set_route(t.name, [(t.pose.reshape(1, 3), t.footprint())], rank=-1)
        t.route_version += 1
        t.route_start = t.pose.copy()
        if t.job is not None:
            dispatcher.abandon(t.job, reached_pickup=t.loaded, unreachable=False)
        events.append({"t_s": round(now, 1), "kind": event, "truck": t.name, "stage": t.stage})
        for other in trucks:
            if other is not t and other.name not in dead and affected(other, box):
                other.pending_replan = True

    while now < args.sim_s:
        dispatcher.tick(now, under_truck)
        # -- faults ----------------------------------------------------------
        while faults and faults[0][0] <= now:
            _, kind = faults.pop(0)
            moving = [t for t in trucks if t.state == "job" and t.v > 0.05 and t.name not in dead]
            if kind == "pause" and moving:
                t = moving[int(rng.integers(len(moving)))]
                t.stopped_until = now + float(rng.uniform(30, 120))
                events.append({"t_s": round(now, 1), "kind": "pause", "truck": t.name, "until": round(t.stopped_until, 1)})
            elif kind == "break" and moving:
                immobilise(moving[int(rng.integers(len(moving)))], "break", "broken")
            elif kind == "spill":
                box = _spill(trucks, manager, world, geometry, rng, extra_boxes, slot_rects)
                if box is not None:
                    extra_boxes.append(("spill", box))
                    events.append({"t_s": round(now, 1), "kind": "spill", "box": [round(v, 2) for v in (box.x0, box.y0, box.x1, box.y1)]})
                    for other in trucks:
                        if other.name not in dead and affected(other, box):
                            other.pending_replan = True
        # -- pending replans at a safe moment (travel stage or a stage boundary)
        for t in trucks:
            if t.pending_replan and t.state == "job" and t.name not in dead:
                kind, stage, _ = t.items[t.item]
                if (kind == "path" and stage in TRAVEL_STAGES) or kind == "wait":
                    t.pending_replan = False
                    if not replan(t):
                        # No way round (plan W3: the job fails). Trying again each
                        # tick would fail the same way, slowly; the truck stays put.
                        immobilise(t, "stranded", "stranded")
        # -- idle trucks ask for work ----------------------------------------
        for t in trucks:
            if t.state == "idle" and now >= t.idle_until and t.name not in dead:
                if t.held is not None:
                    job, result, since, load = t.held
                else:
                    job = dispatcher.next_job(t.k, t.place)
                    if job is None:
                        t.idle_until = now + 5.0
                        continue
                    result = plan(job)
                    if not result.success:
                        dispatcher.abandon(job, reached_pickup=False, unreachable=True)
                        t.idle_until = now + 5.0
                        job_log.append({"job": job.number, "truck": t.k, "status": "plan_failed", "why": result.status, "t_s": round(now, 1)})
                        continue
                    since = now
                    load = float(load_rng.uniform(load_lo, load_hi))
                t.set_load(load)  # the windows' timing of the trial needs it
                if manager.windows is not None and not manager.trial_route(t.name, t.chunks(t.job_items(result), 0)[0]):
                    t.idle_until = now + 5.0
                    if now - since >= HOLD_MAX_S:
                        dispatcher.abandon(job, reached_pickup=False, unreachable=False)
                        t.held = None
                        job_log.append({"job": job.number, "truck": t.k, "status": "held_out", "t_s": round(now, 1)})
                    else:
                        if t.held is None:
                            events.append({"t_s": round(now, 1), "kind": "job_held", "truck": t.name, "job": job.number})
                        t.held = (job, result, since, load)
                    continue
                t.held = None
                loads.append(load)
                t.place = job.home
                t.start_job(job, result, now, manager, dispatcher.next_rank())
                preconditions(t, "job")
        # -- people ----------------------------------------------------------
        truths = np.array([t.truth() for t in trucks])
        bodies = oriented_rects(truths, [t.footprint() for t in trucks], 0.0)
        people.step(DT_S, bodies)
        # -- traffic (reports possibly delayed, permits possibly late or lost)
        poses = {t.name: t.pose.copy() for t in trucks}
        progress = {t.name: t.route_progress() for t in trucks}
        versions = {t.name: t.route_version for t in trucks}
        history.append((now, poses, progress, versions))
        delay = COMM_DELAY_S if flags.get("comm") else 0.0
        while len(history) > 1 and history[1][0] <= now - delay:
            history.pop(0)
        seen_poses, seen_progress = dict(history[0][1]), dict(history[0][2])
        for t in trucks:
            # A report sent on the old route says nothing about the new one: the
            # truck stands at the new route's start until it gets a permit for it.
            if history[0][3][t.name] != t.route_version:
                seen_poses[t.name], seen_progress[t.name] = t.route_start.copy(), 0.0
        permits = manager.update(now, seen_poses, seen_progress)
        by_name = {t.name: t for t in trucks}
        for name, permit in permits.items():
            if flags.get("comm") and rng.random() < COMM_LOSS:
                continue
            inbox.append((now + delay, name, permit.granted_m, by_name[name].route_version, permit.pace_mps))
        due = [m for m in inbox if m[0] <= now + 1e-9]
        inbox = [m for m in inbox if m[0] > now + 1e-9]
        for _, name, granted, version, pace in due:
            t = by_name[name]
            if version == t.route_version:  # a permit for the current route only
                t.granted_m = max(t.granted_m, granted)
                t.pace_mps = pace
        for name, blocker in manager.deadlocks(now):
            episode = (name, blocker, manager.trucks[name].waiting_since_s)
            if episode not in reported:
                reported.add(episode)
                cycle = manager.trucks[blocker].waiting_on == name
                events.append({"t_s": round(now, 1), "kind": "cycle" if cycle else "stall", "truck": name, "blocker": blocker, "stage": by_name[name].stage})
        for advice in manager.advisories(now):
            if advice.kind == "cc":
                events.append({"t_s": round(now, 1), "kind": "cc", "truck": advice.truck, "other": advice.other})
            else:
                advise(advice)
        # -- motion ----------------------------------------------------------
        for t in trucks:
            block = truck_block_m(t, people.pos)
            stepped = t.step(now, manager, dispatcher, block)
            if stepped == "chain" and not chain(t):
                t.enter_item(manager)
            if stepped == "done":
                t.jobs_done += 1
                job_log.append({"job": t.job.number, "truck": t.k, "status": "done", "t_s": round(now, 1), "took_s": round(now - t.job_started, 1)})
                t.place = t.job.home
                t.go_idle(now, manager, rng)
        if args.yield_s > 0:
            for t in trucks:
                if t.field_s >= args.yield_s:
                    t.field_s = 0.0
                    make_way(t)
        # -- pose noise ------------------------------------------------------
        if flags.get("noise"):
            for t in trucks:
                step = rng.normal(0, NOISE_STEP_M * math.sqrt(DT_S), 2)
                t.error[:2] += step
                norm = float(np.hypot(*t.error[:2]))
                if norm > NOISE_LIMIT_M:
                    t.error[:2] *= NOISE_LIMIT_M / norm
                t.error[2] = float(np.clip(t.error[2] + rng.normal(0, 0.002), -NOISE_YAW_LIMIT, NOISE_YAW_LIMIT))
        # -- safety on the truth ---------------------------------------------
        truths = np.array([t.truth() for t in trucks])
        bodies = oriented_rects(truths, [t.footprint() for t in trucks], 0.0)
        touch = overlap_matrix(bodies, bodies)
        for i in range(len(trucks)):
            for j in range(i + 1, len(trucks)):
                if touch[i, j]:
                    contacts.append({"t_s": round(now, 1), "kind": "truck", "a": trucks[i].name, "b": trucks[j].name})
        floor = static
        spills = [b for name, b in extra_boxes if name == "spill"]
        if spills:  # a stopped truck's body is checked with the trucks, not its grown box
            floor = np.vstack((static, _box_rects(spills)))
        hit_floor = overlap_matrix(bodies, floor).any(axis=1)
        occupied = np.array([dispatcher.occupied[n] for n in slot_names])
        on_pallet = overlap_matrix(bodies, slot_rects) & occupied[None, :]
        for i, t in enumerate(trucks):
            if hit_floor[i] and t.name not in dead:
                contacts.append({"t_s": round(now, 1), "kind": "floor", "a": t.name, "stage": t.stage})
            own = () if t.job is None else (t.job.source.name, t.job.sink.name)
            if any(slot_names[j] not in own for j in np.nonzero(on_pallet[i])[0]):
                contacts.append({"t_s": round(now, 1), "kind": "pallet", "a": t.name, "stage": t.stage})
        if len(people.pos):
            discs = np.column_stack(
                (people.pos, np.full(len(people.pos), PERSON_RADIUS), np.full(len(people.pos), PERSON_RADIUS), np.ones(len(people.pos)), np.zeros(len(people.pos)))
            )
            hit = overlap_matrix(bodies, discs)
            for i, j in zip(*np.nonzero(hit), strict=True):
                contacts.append({"t_s": round(now, 1), "kind": "person", "a": trucks[i].name, "person": int(j), "truck_moving": bool(trucks[i].v > 0.02)})
        # -- log -------------------------------------------------------------
        if abs(now - round(now)) < DT_S / 2:
            trajectory.append(
                [
                    [*t.truth(), t.loaded, t.log_stage(t.name in dead), -1 if permits[t.name].blocked_by is None or permits[t.name].allowed_m >= 0.05 else int(permits[t.name].blocked_by[6:]), _alert(t, manager, permits)]
                    for t in trucks
                ]
            )
            people_log = people.pos.copy()
            trajectory[-1].append(people_log.tolist())
        total = sum(t.route_progress() + t.item * 1000 + t.jobs_done * 1e5 for t in trucks)
        busy = any(t.state == "job" and t.name not in dead for t in trucks)
        idle_s = idle_s + DT_S if busy and last_total is not None and total == last_total else 0.0
        last_total = total
        now += DT_S
        if idle_s > args.gridlock_s:
            status = "gridlock"
            break
    done = [j for j in job_log if j["status"] == "done"]
    kinds = {}
    for c in contacts:
        key = c["kind"] + ("_moving" if c.get("truck_moving") else "") if c["kind"] == "person" else c["kind"]
        kinds[key] = kinds.get(key, 0) + 1
    result = {
        "structure": structure,
        "seed": seed,
        "trucks": trucks_n,
        "condition": condition,
        "status": status,
        "sim_s": round(now, 1),
        "world": {
            "size_m": world.size_m,
            "sources": len(world.sources()),
            "sinks": len(world.sinks()),
            "holding": len(world.holding),
            "obstacles": len(world.obstacles),
            "dropped": world.dropped,
        },
        "jobs_done": len(done),
        "jobs_per_hour": round(len(done) / max(now, 1e-9) * 3600, 1),
        "job_time_s": _stats([j["took_s"] for j in done]),
        "plan_failed": sum(j["status"] == "plan_failed" for j in job_log),
        "plan_wall_s": _stats([p["wall_s"] for p in plans]),
        "contacts": kinds,
        "contact_events": contacts[:200],
        "cycles": sum(e["kind"] == "cycle" for e in events),
        "stalls": sum(e["kind"] == "stall" for e in events),
        "preconditions": sum(e["kind"] == "precondition" for e in events),
        "replans": sum(e["kind"] == "replan" for e in events),
        "replan_failed": sum(e["kind"] == "replan" and e["status"] != "success" for e in events),
        "resolution": resolution_counts(events),
        "yields": {h: sum(e["kind"] == "yield" and e["how"] == h for e in events) for h in ("back", "failed", "skipped")},
        "waits": wait_stats(np.array([[row[5] for row in step[:-1]] for step in trajectory], dtype=int).reshape(len(trajectory), len(trucks))),
        "stranded": sum(e["kind"] == "stranded" for e in events),
        "fills_held": len(dispatcher.held_back),
        "traffic": args.traffic,
        "vehicle": args.vehicle,
        "eta": args.eta,
        "pace": args.pace,
        "world_scale": args.world_scale,
        "od": args.od,
        "chain": args.chain,
        "chains": chains,
        "queue_opt": args.queue_opt if args.traffic == "queues" else None,
        "opt_horizon_s": args.opt_horizon_s if args.traffic == "queues" else None,
        "accel_cap": cap,
        "load_kg_mean": round(float(np.mean(loads)), 2) if loads else None,
        "stops": sum(t.stops for t in trucks),
        "windows": manager.windows.summary() if manager.windows is not None else None,
        "jobs_held": sum(e["kind"] == "job_held" for e in events),
        "jobs_held_out": sum(j["status"] == "held_out" for j in job_log),
        "waiting_share": round(sum(t.waiting_s for t in trucks) / max(now * len(trucks), 1e-9), 3),
        "person_stop_s": round(sum(t.person_stops_s for t in trucks), 1),
        "people_walked_m": round(people.walked_m, 1),
        "events": events[:400],
        "window_events": manager.windows.events[:300] if manager.windows is not None else [],
        "jobs": job_log,
        "plans": plans,
    }
    out.mkdir(parents=True, exist_ok=True)
    (out / "result.json").write_text(json.dumps(result, indent=1, ensure_ascii=False) + "\n")
    (out / "world.json").write_text(json.dumps(world_record(world, geometry, extra_boxes), ensure_ascii=False) + "\n")
    np.savez_compressed(
        out / "trajectory.npz",
        trucks=np.array([[row[:8] for row in step[:-1]] for step in trajectory], dtype=float),
        people=np.array([step[-1] for step in trajectory], dtype=float) if people.pos.size else np.zeros((len(trajectory), 0, 2)),
    )
    return result


def _alert(t, manager, permits) -> int:
    if t.name in manager.ra_against:
        return ALERT_RA
    p = permits[t.name]
    return ALERT_TA if p is not None and p.blocked_by is not None and p.allowed_m < 0.05 else ALERT_CLEAR


def _aabb(t, grow):
    x, y, yaw = t.truth()
    rect = oriented_rects(np.array([[x, y, yaw]]), t.footprint(), 0.0)[0]
    hx = abs(rect[4]) * rect[2] + abs(rect[5]) * rect[3]
    hy = abs(rect[5]) * rect[2] + abs(rect[4]) * rect[3]
    return Box(rect[0] - hx, rect[1] - hy, rect[0] + hx, rect[1] + hy).grow(grow)


def _spill(trucks, manager, world, geometry, rng, extra_boxes, slot_rects):
    """A 1-2 m box on some truck's route ahead, touching no truck, claim or slot."""
    candidates = [t for t in trucks if t.state == "job"]
    rng.shuffle(candidates)
    for t in candidates:
        mine = manager.trucks[t.name]
        ahead = mine.g > t.route_progress() + 8.0
        if not ahead.any():
            continue
        idx = np.nonzero(ahead)[0]
        r = mine.rects[idx[int(rng.integers(len(idx)))]]
        size = float(rng.uniform(1.0, 2.0))
        box = Box(r[0] - size / 2, r[1] - size / 2, r[0] + size / 2, r[1] + size / 2)
        rect = _box_rects([box.grow(0.3)])
        if any(overlap_matrix(rect, manager.trucks[o.name].claim).any() for o in trucks):
            continue
        if overlap_matrix(rect, slot_rects).any():
            continue
        return box
    return None


def _stats(values):
    if not values:
        return None
    v = np.asarray(values, float)
    return {"n": int(len(v)), "mean": round(float(v.mean()), 2), "p95": round(float(np.percentile(v, 95)), 2), "max": round(float(v.max()), 2)}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--structure", required=True, choices=("rack", "shop", "multi"))
    parser.add_argument("--seeds", required=True)
    parser.add_argument("--trucks", type=int, default=10)
    parser.add_argument("--condition", default="C0", choices=list(CONDITIONS))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--settings", type=Path, default=ROOT / "config/isaac_transport_measured.yaml")
    parser.add_argument("--forklift-urdf", type=Path, default=ROOT / "sim/models/dls08_measured/forklift.urdf")
    parser.add_argument("--pallet-geometry", type=Path, default=ROOT / "config/pallet_geometry_epal6.yaml")
    parser.add_argument("--insertion-reserve-m", type=float, default=0.016)
    parser.add_argument("--expansions", type=int, default=30000)
    parser.add_argument("--heuristic-resolution-m", type=float, default=0.25)
    parser.add_argument("--margin-m", type=float, default=0.3)
    parser.add_argument("--lookahead-m", type=float, default=3.0)
    parser.add_argument("--stall-s", type=float, default=20.0)
    parser.add_argument("--gridlock-s", type=float, default=300.0)
    # TA/RA (plan 2026-10-10); a non-positive ta/ra time turns that advice off.
    parser.add_argument("--ta-s", type=float, default=20.0)
    parser.add_argument("--ra-s", type=float, default=60.0)
    parser.add_argument("--cooldown-s", type=float, default=60.0)
    parser.add_argument("--wall-m", type=float, default=20.0)
    parser.add_argument("--retreat-max-m", type=float, default=20.0)
    # Making room for a person (plan 2026-10-11 (h)); a non-positive time turns it off.
    parser.add_argument("--yield-s", type=float, default=20.0)
    parser.add_argument("--yield-back-m", type=float, default=2.5)
    # Time-window tokens (plan 2026-10-11).
    parser.add_argument("--traffic", default="rank", choices=MODES)
    parser.add_argument("--issue-s", type=float, default=30.0)
    parser.add_argument("--slack-s", type=float, default=10.0)
    parser.add_argument("--window-period-s", type=float, default=1.0)
    parser.add_argument("--window-evals", type=int, default=30)
    parser.add_argument("--age-s", type=float, default=120.0)
    parser.add_argument("--park-s", type=float, default=10.0)
    # Crossing queues (plan 2026-10-16): --traffic queues, then these.
    parser.add_argument("--pace", action="store_true", help="speed cap to reach a held place as it clears")
    parser.add_argument("--pace-min-mps", type=float, default=0.1)
    parser.add_argument("--queue-opt", default="local", choices=("local", "global"))
    parser.add_argument("--opt-period-s", type=float, default=5.0)
    parser.add_argument("--node-limit", type=int, default=2000)
    parser.add_argument("--opt-horizon-s", type=float, help="global orders: only crossings reached within this (free flow); default all")
    # Literature baselines (plan 2026-10-17): --traffic adg | sections | mapf | tiles.
    parser.add_argument("--mapf-window-s", type=float, default=30.0)
    parser.add_argument("--mapf-replan-s", type=float, default=5.0)
    parser.add_argument("--tile-m", type=float, default=0.25)
    parser.add_argument("--tile-horizon-m", type=float, default=0.0, help="0: the whole route")
    # Large random floors (plan 2026-10-17-fleet-large-random-chained).
    parser.add_argument("--world-scale", type=float, default=1.0, help="stretch the halls (1: the floors of every earlier run)")
    parser.add_argument("--od", default="roles", choices=("roles", "random"), help="roles: sources to sinks; random: any pallet to any free slot")
    parser.add_argument("--chain", action="store_true", help="take the next job from the drop, not from a holding place")
    parser.add_argument("--fork-plans", action="store_true",
                        help="plan a job's stages at once and run a long search's retries ahead, in forked processes (same plans, more cores)")
    # Physical model (plan 2026-10-12).
    parser.add_argument("--vehicle", default="kinematic", choices=("kinematic", "physics"))
    parser.add_argument("--eta", default="simple", choices=("simple", "physics"))
    parser.add_argument("--accel-cap", type=float, default=None, help="tracker acceleration cap (default: the settings')")
    parser.add_argument("--load-kg", default="5,13", help="load range per job, kg (estimate)")
    parser.add_argument("--tip-share", type=float, default=0.5)
    parser.add_argument("--sim-s", type=float, default=1800.0)
    args = parser.parse_args(argv)
    if args.fork_plans:
        speculate(2.0)  # a search still going after 2 s starts its retries alongside
    fleet = _fleet_tool()
    settings = yaml.safe_load(args.settings.read_text())
    config = make_transport_planner_config(
        curvature_limit_inv_m=settings["planner_curvature_inv_m"],
        clearance_m=settings["planning_clearance_m"],
        max_expansions=args.expansions,
    )
    travel = replace(config, obstacle_heuristic_resolution_m=args.heuristic_resolution_m)
    geometry = fleet.mission_geometry(args)
    for seed in fleet._seeds(args.seeds):
        out = args.output / f"{args.structure}_{seed}_{args.condition}_{args.trucks}"
        start = time.perf_counter()
        result = run(args.structure, seed, args.trucks, args.condition, args, settings, geometry, (config, travel), out)
        line = {k: result[k] for k in ("structure", "seed", "trucks", "condition", "traffic", "vehicle", "eta", "pace", "queue_opt", "od", "chain", "chains", "status", "sim_s", "jobs_done", "jobs_per_hour", "plan_failed", "contacts", "cycles", "stalls", "preconditions", "replans", "waiting_share", "resolution", "waits")}
        line["wall_s"] = round(time.perf_counter() - start, 1)
        print(json.dumps(line, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
