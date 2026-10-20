"""The survey epoch's tour of the hall and the region it must cover (SLAM localization plan
docs/plans/2026-10-20-slam-localization-session.md L2 v2.5).

Everything is in the rear-axle frame of the planner (a node's corrected base pose moves back by the rear axle offset).
A cell of the ``CELL_M`` grid can hold the rear axle only when its centre is at least the footprint's rear overhang plus
the planner clearance from every obstacle the planner plans around (the run's scenario: props of 1.15 m and taller --
the lower ones are not in the scene) and from the hall bounds: the nearest point of the footprint to the axle is its
rear edge, so no pose nearer can be planned.

- The **judged region** (L2 v2.5) is the cells of that kind inside the aisle bands (``aisles`` of the tour config,
  ``AISLE_BAND_M`` either side), the work approaches (the rear axle from the inserted to the prealign pose along each
  pallet site's axis, ``site_poses``, ``APPROACH_BAND_M`` either side, for the pickup and the destination site alike),
  the observation poses of reference runs (``OBSERVATION_DISC_M``) and the home (``HOME_DISC_M``). Every cell of it
  must be within ``CELL_RADIUS_M`` of a node.
- The **placeable estimate** is every cell whose centre takes the whole unloaded footprint (and the clearance) at one
  of ``HEADINGS`` headings with the props alone: an estimate (the heading sample misses some poses, the pallet moves),
  so what it leaves uncovered is reported as unsurveyed, not judged.

``plan`` joins the tour's rear-axle poses (``config/slam_survey_tour.yaml``) with the run's own travel planner (the
measured curvature, the unloaded footprint) through ``plan_survey_route``, from the truck's home and back to it, and
writes the path (``poses``, ``directions``, ``curvatures_inv_m``) for the console probe's tour scenario, with the
coverage its samples would give. The survey's verdict is the saved graph's nodes, not this prediction
(tools/slam_survey_bundle.py).

    python tools/survey_tour.py plan --result RUN/result.json --tour config/slam_survey_tour.yaml --out TOUR.npz
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from forklift_core.planning.factory_layout import plan_survey_route  # noqa: E402
from forklift_core.planning.geometry import Bounds, Footprint, FootprintCollisionChecker, Pose2D, Rectangle  # noqa: E402
from forklift_core.planning.hybrid_astar import PlannerConfig  # noqa: E402

CELL_M = 0.25
HEADINGS = 16
AISLE_BAND_M = 1.5  # a 4 m aisle: 2 - half width 0.36 - clearance 0.10
APPROACH_BAND_M = 1.0
OBSERVATION_DISC_M = 1.0
HOME_DISC_M = 1.5
COVERAGE_RADIUS_M = 3.0
# a cell is covered when its centre is this close to a node: then every point of the cell is within the radius
CELL_RADIUS_M = COVERAGE_RADIUS_M - CELL_M * math.sqrt(2) / 2


def _rect(d: dict) -> Rectangle:
    return Rectangle(d["x_m"], d["y_m"], d["length_m"], d["width_m"], d.get("yaw_rad", 0.0))


def scenario_obstacles(result: dict, pallet: bool = False) -> list[Rectangle]:
    """The rectangles the run's planner plans around: its props, and with ``pallet`` the pallet at the pickup site."""
    s, g = result["scenario"], result["geometry"]
    pickup = s["pickup"]
    props = [_rect(p["rectangle"]) for p in s["props"]]
    return props + ([Rectangle(pickup["x_m"], pickup["y_m"], g["pallet_depth_m"], g["pallet_width_m"],
                               pickup["yaw_rad"])] if pallet else [])


def segment_distance(px: np.ndarray, py: np.ndarray, a, b) -> np.ndarray:
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    t = np.clip(((px - ax) * dx + (py - ay) * dy) / length2, 0.0, 1.0) if length2 > 0 else np.zeros_like(px)
    return np.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _grid(result: dict, cell_m: float):
    b, g = result["scenario"]["bounds"], result["geometry"]["unloaded_footprint"]
    clearance = float(result["travel_planner_config"].get("clearance_m", 0.0))
    near = float(g["rear_m"]) + clearance
    xs = np.arange(b["x_min_m"] + cell_m / 2, b["x_max_m"], cell_m)
    ys = np.arange(b["y_min_m"] + cell_m / 2, b["y_max_m"], cell_m)
    gx, gy = np.meshgrid(xs, ys)
    px, py = gx.ravel(), gy.ravel()
    nearest = np.full(len(px), np.inf)
    for r in scenario_obstacles(result):
        nearest = np.minimum(nearest, rect_distance(px, py, r))
    wall = np.minimum.reduce([px - b["x_min_m"], b["x_max_m"] - px, py - b["y_min_m"], b["y_max_m"] - py])
    return px, py, np.minimum(nearest, wall), near


def work_segments(result: dict) -> list[tuple]:
    """Each pallet site's rear-axle approach, inserted -> prealign pose (``site_poses``), for pickup and destination."""
    g = result["geometry"]
    out = []
    for site in (result["scenario"]["pickup"], result["scenario"]["destination"]):
        c, s = math.cos(site["yaw_rad"]), math.sin(site["yaw_rad"])
        out.append(tuple((site["x_m"] - d * c, site["y_m"] - d * s) for d in (g["inserted_offset_m"], g["prealign_offset_m"])))
    return out


def observation_poses(reference_runs: list) -> list[tuple]:
    """The observation poses a reference run's missions planned: its result.json and, per mission, the archived
    ``missions/<id>/records.json`` (its ``state``; the mission's result.json holds only the outcome)."""
    poses = []
    for run in reference_runs:
        for path in [Path(run) / "result.json"] + sorted((Path(run) / "missions").glob("*/records.json")):
            if not path.exists():
                continue
            data = json.loads(path.read_text())
            data = data.get("state", data) if path.name == "records.json" else data
            for c in data.get("observation_candidates") or []:
                if c.get("pose") is not None:
                    poses.append(tuple(c["pose"][:2]))
            for v in data.get("runtime_viewpoints") or []:
                pose = v.get("pose") if isinstance(v, dict) else v
                if pose is not None:
                    poses.append(tuple(pose[:2]))
    return sorted(set(poses))


def judged_region(result: dict, aisles: list, observations: list = (), cell_m: float = CELL_M) -> np.ndarray:
    """Centres (N, 2) of the judged region's cells (see the module doc)."""
    px, py, clear, near = _grid(result, cell_m)
    keep = np.zeros(len(px), bool)
    for a, b in aisles:
        keep |= segment_distance(px, py, a, b) <= AISLE_BAND_M
    for a, b in work_segments(result):
        keep |= segment_distance(px, py, a, b) <= APPROACH_BAND_M
    for x, y in observations:
        keep |= np.hypot(px - x, py - y) <= OBSERVATION_DISC_M
    home = result["scenario"]["start_rear"]
    keep |= np.hypot(px - home["x_m"], py - home["y_m"]) <= HOME_DISC_M
    keep &= clear >= near
    return np.column_stack((px[keep], py[keep]))


def scenario_sha256(result: dict) -> str:
    """The scene the tour was planned in (a survey run must be on the same one)."""
    import hashlib

    return hashlib.sha256(json.dumps(result["scenario"], sort_keys=True).encode()).hexdigest()


def base_to_rear(x, y, yaw, rear_axle_offset_m: float):
    """A base_link pose's rear axle point (the axle sits ``|offset|`` behind base_link)."""
    off = abs(float(rear_axle_offset_m))
    return np.asarray(x) - off * np.cos(yaw), np.asarray(y) - off * np.sin(yaw)


def rect_distance(px: np.ndarray, py: np.ndarray, r: Rectangle) -> np.ndarray:
    """Distance from points to a rectangle (0 inside)."""
    c, s = math.cos(r.yaw_rad), math.sin(r.yaw_rad)
    dx, dy = px - r.x_m, py - r.y_m
    u, v = c * dx + s * dy, -s * dx + c * dy
    eu = np.maximum(np.abs(u) - r.length_m / 2, 0.0)
    ev = np.maximum(np.abs(v) - r.width_m / 2, 0.0)
    return np.hypot(eu, ev)


def placeable_estimate(result: dict, cell_m: float = CELL_M, headings: int = HEADINGS) -> np.ndarray:
    """Centres (N, 2) of the placeable estimate's cells (see the module doc)."""
    b, g = result["scenario"]["bounds"], result["geometry"]["unloaded_footprint"]
    clearance = float(result["travel_planner_config"].get("clearance_m", 0.0))
    footprint = Footprint(**g)
    reach = math.hypot(max(g["front_m"], g["rear_m"]), g["half_width_m"]) + clearance  # farther: every heading fits
    px, py, clear, near = _grid(result, cell_m)
    obstacles = scenario_obstacles(result)
    keep = clear >= reach
    checker = FootprintCollisionChecker(obstacles, footprint, Bounds(**b))
    for i in np.flatnonzero((clear >= near) & ~keep):
        keep[i] = any(checker.free((px[i], py[i], 2 * math.pi * k / headings), clearance) for k in range(headings))
    return np.column_stack((px[keep], py[keep]))


def nearest_distance(points: np.ndarray, sites: np.ndarray, chunk: int = 4096) -> np.ndarray:
    out = np.empty(len(points))
    for start in range(0, len(points), chunk):
        part = points[start:start + chunk]
        out[start:start + chunk] = np.sqrt(((part[:, None, :] - sites[None, :, :]) ** 2).sum(axis=2)).min(axis=1)
    return out


def region_coverage(region: np.ndarray, sites: np.ndarray, radius_m: float = CELL_RADIUS_M) -> dict:
    """Every region cell within ``radius_m`` of a site (the cell's centre; ``CELL_RADIUS_M`` makes it the cell)."""
    d = nearest_distance(region, sites)
    outside = d > radius_m
    return {"ok": bool(len(d) and not outside.any()), "cells": int(len(d)), "cell_m": CELL_M,
            "radius_m": radius_m, "uncovered": int(outside.sum()),
            "uncovered_area_m2": float(outside.sum() * CELL_M**2),
            "farthest_m": float(d.max()) if len(d) else None,
            "farthest_cell": region[int(np.argmax(d))].tolist() if len(d) else None}


TOUR_ZONE_EXTRA_M = 0.05
STUCK_NEAR_END_M = 0.5  # standing this close to a leg's end, short of its tolerance, is the tracker's braking stall


def permit_pickup_zone(result: dict) -> Rectangle:
    """The keep-out zone the teleop pallet permit holds an unrecognised (prior) pallet as -- the tour never recognises
    it: the runner's pickup-zone prior (``BAY_PICKUP_ZONE`` grown by the pallet's half diagonal, run_transport) grown by
    ``PRIOR_POSE_MARGIN_M + ENVELOPE_M`` (``pallet_permit.zones_from``) and ``TOUR_ZONE_EXTRA_M``. The first tour (ws1
    job 6807) stood still on ``pallet:P-1`` beside the pallet itself, planned around with 0.10 m."""
    from forklift_core.planning.observation_viewpoints import BAY_PICKUP_ZONE as zone
    from forklift_core.safety.pallet_permit import ENVELOPE_M, PRIOR_POSE_MARGIN_M

    g = result["geometry"]
    grow = math.hypot(g["pallet_depth_m"], g["pallet_width_m"]) / 2 + PRIOR_POSE_MARGIN_M + ENVELOPE_M + TOUR_ZONE_EXTRA_M
    return Rectangle((zone.x_min_m + zone.x_max_m) / 2, (zone.y_min_m + zone.y_max_m) / 2,
                     zone.x_max_m - zone.x_min_m + 2 * grow, zone.y_max_m - zone.y_min_m + 2 * grow, 0.0)


PLAN_KEYS = ("scenario", "geometry", "travel_planner_config")  # what a replan needs of the result, kept in the tour file


SURVEY_CLEARANCE_M = 0.30  # the tour's driving clearance, initial plan and every replan (plan L2 v2.5, r6 below)


def plan_tour(result: dict, tour: list, start=None, clearance_m: float = SURVEY_CLEARANCE_M) -> SimpleNamespace:
    """The tour from ``start`` (home when None) through ``tour`` back home, around the props and the permit's pickup
    zone (in place of the pallet's own outline: ``plan_survey_route`` adds a pickup pallet of the geometry's size, here
    the zone's), with ``clearance_m`` (never below the run's planner clearance).

    The tour drives on the SLAM map pose while it is planned in world, so the map's error comes off the clearance: ws1
    job 6837 (r6) stood with the true rear axle 0.22 m from a prop where the estimate read 0.36, the teleop permit
    holding it (``occupied``), and every replan at 0.10 m gave the same blocked leg. A replan from a pose that does not
    keep ``clearance_m`` itself (stopped off the path) is tried once more at the run's planner clearance -- never at 0
    (r6's fallback to 0 would plan the very leg the permit holds). The
    judged region keeps the run's planner clearance (``judged_region``): only the driving clearance grows. Returns the
    planner's result fields and ``clearance_m``, the clearance the plan was made with."""
    s, g = result["scenario"], result["geometry"]
    zone = permit_pickup_zone(result)
    transport = SimpleNamespace(props=[SimpleNamespace(rectangle=_rect(p["rectangle"])) for p in s["props"]],
                                pickup=SimpleNamespace(x_m=zone.x_m, y_m=zone.y_m, yaw_rad=zone.yaw_rad),
                                bounds=Bounds(**s["bounds"]))
    geometry = SimpleNamespace(unloaded_footprint=Footprint(**g["unloaded_footprint"]),
                               pallet_depth_m=zone.length_m, pallet_width_m=zone.width_m)
    home = Pose2D(**s["start_rear"])
    route = [home if start is None else Pose2D(*start)] + [Pose2D(*p) for p in tour] + [home]
    from dataclasses import fields, replace

    config = PlannerConfig(**result["travel_planner_config"])
    ladder = [max(clearance_m, config.clearance_m)]
    if start is not None and ladder[0] > config.clearance_m:
        ladder.append(config.clearance_m)
    for clearance in ladder:
        plan = plan_survey_route(SimpleNamespace(transport=transport), route, replace(config, clearance_m=clearance),
                                 geometry=geometry)
        if plan.success:
            break
    return SimpleNamespace(**{f.name: getattr(plan, f.name) for f in fields(plan)}, clearance_m=clearance)


def uncovered_clusters(cells: np.ndarray, cell_m: float = CELL_M) -> list[np.ndarray]:
    """The 8-connected clusters of grid cell centres, largest first."""
    key = {tuple(np.round(p / cell_m).astype(int)): i for i, p in enumerate(cells)}
    seen, clusters = set(), []
    for start in key:
        if start in seen:
            continue
        stack, members = [start], []
        seen.add(start)
        while stack:
            c = stack.pop()
            members.append(key[c])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (c[0] + dx, c[1] + dy)
                    if n in key and n not in seen:
                        seen.add(n)
                        stack.append(n)
        clusters.append(cells[members])
    return sorted(clusters, key=len, reverse=True)


def refine_tour(result: dict, tour: list, sites: np.ndarray, rounds: int, judged: np.ndarray,
                log=print) -> tuple[list, SimpleNamespace]:
    """Greedy: plan the tour; for the largest cluster of ``judged`` cells it leaves uncovered (with ``sites``, other
    survey nodes in the rear-axle frame), add a waypoint at the cluster's cell nearest the path, heading as the path does there (or
    reversed, whichever the footprint takes), between the two waypoints the path passes it between; replan. Stops
    when covered, after ``rounds`` additions, or when no added waypoint plans (that cluster is then skipped)."""
    region = judged
    g = result["geometry"]["unloaded_footprint"]
    checker = FootprintCollisionChecker(scenario_obstacles(result) + [permit_pickup_zone(result)], Footprint(**g),
                                        Bounds(**result["scenario"]["bounds"]))
    clearance = float(result["travel_planner_config"].get("clearance_m", 0.0))
    plan = plan_tour(result, tour)
    if not plan.success:
        return tour, plan
    skipped: set = set()
    for _ in range(rounds):
        poses = np.asarray(plan.poses)
        covered = nearest_distance(region, np.vstack([sites, poses[:, :2]])) <= CELL_RADIUS_M
        clusters = [c for c in uncovered_clusters(region[~covered]) if tuple(np.round(c.mean(0), 1)) not in skipped]
        if not clusters:
            break
        cluster = clusters[0]
        d = nearest_distance(cluster, poses[:, :2])
        cell = cluster[int(np.argmin(d))]
        at = int(np.argmin(np.hypot(poses[:, 0] - cell[0], poses[:, 1] - cell[1])))
        yaw = float(poses[at, 2])
        heading = next((h for h in (yaw, yaw + math.pi, yaw + math.pi / 2, yaw - math.pi / 2)
                        if checker.free((cell[0], cell[1], h), clearance)), None)
        # the waypoint the path passes before this sample: the waypoints sit on the path exactly
        marks = [int(np.argmin(np.hypot(poses[:, 0] - x, poses[:, 1] - y))) for x, y, _ in tour]
        index = sum(1 for m in marks if m <= at)
        candidate = tour[:index] + [[float(cell[0]), float(cell[1]), float(math.atan2(math.sin(heading), math.cos(heading)))]] + tour[index:] \
            if heading is not None else None
        trial = plan_tour(result, candidate) if candidate is not None else None
        if trial is None or not trial.success:
            skipped.add(tuple(np.round(cluster.mean(0), 1)))
            log(json.dumps({"skipped_cluster": cluster.mean(0).round(2).tolist(), "cells": len(cluster)}))
            continue
        tour, plan = candidate, trial
        log(json.dumps({"added": tour[index], "at": index, "cluster_cells": len(cluster), "length_m": round(plan.length_m, 1)}))
    return tour, plan


def simulate_tour(poses, directions, curvatures, *, physics_hz=120, state_hz=10.0, poll_hz=20.0, factor=0.3,
                  curvature_rate=0.5, accel=0.8, kappa_limit=0.36, max_sim_s=4000.0, curvature_gain=1.0,
                  plant_curvature_rate=None, plan_input=None, **driver_args) -> dict:
    """The tour through a CPU model of the teleop chain (Codex v2.5 review P2-3): an ideal rear-axle bicycle at
    ``physics_hz``, the teleop's curvature slew (``TeleopLimits.curvature_rate_inv_m_s``) and speed slew, states at
    ``state_hz`` and polls at ``poll_hz`` of wall time (sim = wall x ``factor``), no permit, no SLAM error; the plant
    turns ``curvature_gain`` of the curvature commanded and, with ``plant_curvature_rate``, reaches it at that rate
    (the steering joints' lag behind the applied command). With
    ``plan_input`` (the tour file's) a replan request is planned at once. Returns the driver's end, sim time, progress,
    replans and events."""
    driver = TourDriver(poses, directions, curvatures, kappa=kappa_limit, **driver_args)
    x, y, yaw = (float(v) for v in poses[0])
    v = k = k_plant = 0.0
    sim, wall, seq, command = 0.0, 0.0, 0, (0.0, 0.0)
    driver.start(0.0)
    next_state, next_poll, state = 0.0, 0.0, None
    dt = 1.0 / physics_hz
    end = None
    while sim < max_sim_s and end is None:
        wall = sim / factor
        if wall >= next_state:
            seq += 1
            state = {"state_seq": seq, "mode": "TELEOP", "armed": True, "rear_axle_speed_mps": v,
                     "applied": {"speed_mps": v, "curvature_inv_m": k},
                     "steering_rad": [math.atan(k_plant * 0.66)] * 2,
                     "pose": {"x_m": x, "y_m": y, "yaw_rad": yaw}, "pacing": {"target_factor": factor, "ok": True}}
            next_state += 1.0 / state_hz
        if wall >= next_poll:
            new, end = driver.step(wall, state)
            command = new if new is not None else command
            if driver.replan_request is not None and plan_input is not None:
                start, left = driver.replan_request
                driver.adopt(wall, plan_tour(plan_input, [list(w) for w in left], start=start), left)
            next_poll += 1.0 / poll_hz
        speed_cap = driver.forward if command[0] >= 0 else driver.reverse
        target_v, target_k = command[0] * speed_cap, command[1] * kappa_limit
        k += max(-curvature_rate * dt, min(curvature_rate * dt, target_k - k))
        v += max(-accel * dt, min(accel * dt, target_v - v))
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        step_p = math.inf if plant_curvature_rate is None else plant_curvature_rate * dt
        k_plant += max(-step_p, min(step_p, k - k_plant))
        yaw += v * k_plant * curvature_gain * dt
        sim += dt
    return {"end": end, "sim_s": sim, "progress_m": driver.best, "replans": driver.replans, "events": driver.events}


class TourDriver:
    """The tour's teleop driver (the console probe's tour scenario; plan L2 v2.5): one call per poll with the latest
    state message, returns ``(command, end)`` -- the teleop ``(speed_ratio, curvature_ratio)`` to send now (None: keep
    the last) and, once the tour is over, why (``arrived`` or a failure). The runner's ``RearAxlePathTracker`` with its
    return-home settings, the tour's own tolerances (a repositioning drive, not docking), closed on the state's map pose
    (the rear axle's SLAM estimate) and measured rear-axle speed, on each *new* state only.

    - The tour starts at the first new state in TELEOP after ``start`` (an older PAUSED state is not a departure); none
      within ``enter_s`` fails it.
    - The tracker's dt is the wall time between states times the declared factor, held to a physics step .. 0.15 s
      (states carry no sim time).
    - A gear cusp the tracker holds short of its tolerance (the teleop curvature slew leaves the axle off it) is
      released after ``release_s`` standing at the leg's end; the last leg likewise ends ``arrived`` within
      ``goal_slack_m``. Both are recorded.
    - No progress for ``replan_stall_s`` of drivable time replans from where the truck is (while replans are left);
      no progress for ``stall_s`` fails the tour (time with teleop forbidden for pacing counts apart,
      up to ``pacing_wait_s`` in a row); no new state for ``stale_s`` fails it; so does a mode that leaves TELEOP. A
      failure is never skipped past.
    - Each leg starts by steering standing (departure, a cusp passed, a replan): speed 0 with the leg's curvature until the
      state's applied curvature is within ``steer_tolerance`` of it and the measured steering (``steering_rad``) has
      settled (moved under ``steer_settle_rad`` since the last state). The teleop's curvature slews 0.5 1/m/s, so a cusp
      that flips the steering takes 1.4 sim s; moving meanwhile left the Isaac truck (ws1 job 6826, truth and estimate
      alike) 0.28 m off a leg at the planner's curvature limit.
    - A tracker failure (off the path by ``max_cross_track_m``, or an endpoint heading) stops the truck and, with
      ``waypoints`` and up to ``max_replans`` times, asks for a new path from the stopped pose through the waypoints
      not yet passed (``replan_request``; the caller plans and ``adopt``s it) -- as the runner replans a stalled leg.
      The Isaac tour of ws1 job 6826 left its path by 0.36 m 5.8 m in: a cusp left the axle off the path and the
      correction saturated the curvature the plant could not quite give.
    """

    def __init__(self, poses, directions, curvatures, *, forward_mps=0.5, reverse_mps=0.25, kappa=0.36,
                 cruise_mps=0.30, stall_s=60.0, pacing_wait_s=900.0, stale_s=10.0, enter_s=20.0, release_s=3.0,
                 goal_slack_m=0.3, waypoints=None, max_replans=24, max_cross_track_m=0.5, steer_tolerance=0.05,
                 steer_wait_s=20.0, steer_settle_rad=0.01, stop_wait_s=30.0, replan_stall_s=20.0):
        self.forward, self.reverse, self.kappa = float(forward_mps), float(reverse_mps), float(kappa)
        self.cruise, self.max_cross_track_m = min(cruise_mps, self.forward), float(max_cross_track_m)
        self.can_replan = waypoints is not None
        self.waypoints = [tuple(map(float, w)) for w in (waypoints if waypoints is not None else [])]
        self.max_replans, self.replans = int(max_replans), 0
        self.replan_request = None  # (pose, waypoints left) once the truck stands after a tracker failure
        self.steer_tolerance, self.steer_wait_s = float(steer_tolerance), float(steer_wait_s)
        self._leg, self._steer_since = None, None
        self.steer_settle_rad, self._last_steer = float(steer_settle_rad), None
        self.stop_wait_s, self._stop_since = float(stop_wait_s), None
        self.replan_stall_s = float(replan_stall_s)
        self._failed = None
        self._blocked = False
        self._set_path(poses, directions, curvatures)
        self.stall_s, self.pacing_wait_s, self.stale_s = stall_s, pacing_wait_s, stale_s
        self.enter_s, self.release_s, self.goal_slack_m = enter_s, release_s, goal_slack_m
        self.events: list = []
        self.started_wall = self.last_seq = self.last_state_wall = self.last_feedback_wall = None
        self.departed = False
        self.best = 0.0
        self.drivable_idle = self.pacing_idle = 0.0
        self.stand_since = None
        self.command = None
        self.last_cmd = None

    def _set_path(self, poses, directions, curvatures) -> None:
        from forklift_core.control.path_tracking import RearAxlePathTracker, TrackerConfig

        poses = np.asarray(poses, float)
        self.tracker = RearAxlePathTracker(poses, directions, curvatures, TrackerConfig(
            cruise_speed_mps=self.cruise, max_curvature_inv_m=self.kappa,
            max_acceleration_mps2=0.30, lookahead_m=0.28, position_tolerance_m=0.10, yaw_tolerance_rad=0.15,
            cusp_position_tolerance_m=0.10, cusp_yaw_tolerance_rad=0.10, cusp_brake_window_m=0.008,
            cusp_heading_pending_rad=0.20, overshoot_tolerance_m=0.05, stop_speed_mps=0.012,
            max_cross_track_error_m=self.max_cross_track_m, max_reverse_speed_mps=self.reverse))
        self.last_index = len(poses) - 1
        # where each waypoint not yet passed lies on this path (the planner passes through them exactly)
        self.marks = [int(np.argmin(np.hypot(poses[:, 0] - w[0], poses[:, 1] - w[1]))) for w in self.waypoints]
        self.base_progress = getattr(self, "best", 0.0)

    def waypoints_left(self, segment_index: int, pose=None, passed_m: float = 0.5) -> list:
        """The waypoints the path has not passed yet; with ``pose``, one within ``passed_m`` of it counts as passed
        (a replan from beside a cusp waypoint must not lead back to it)."""
        left = [w for w, m in zip(self.waypoints, self.marks) if m > segment_index]
        if pose is not None:
            left = [w for w in left if math.hypot(w[0] - pose[0], w[1] - pose[1]) > passed_m]
        return left

    def adopt(self, now: float, plan, waypoints_left: list) -> bool:
        """A replanned path from the stopped pose (``replan_request``) through ``waypoints_left``; False fails the
        tour (the plan failed)."""
        self.replan_request = None
        if plan is None or not plan.success:
            self._event(now, "replan_failed", status=None if plan is None else plan.status)
            self._failed = "replan_failed"
            return False
        self.waypoints = [tuple(map(float, w)) for w in waypoints_left]
        self._set_path(plan.poses, plan.directions, plan.curvatures_inv_m)
        self._leg = None  # the new path's first leg steers standing too
        # the planning took wall time with no step: it is neither idle nor a stale state (Codex v2.5 round 3 P2-2)
        self.last_state_wall = self.last_feedback_wall = now
        self.drivable_idle = self.pacing_idle = 0.0
        self.stand_since = None
        self._event(now, "replanned", replans=self.replans, waypoints_left=len(self.waypoints),
                    length_m=float(plan.length_m), clearance_m=getattr(plan, "clearance_m", None))
        return True

    def start(self, now: float) -> None:
        self.started_wall = self.last_state_wall = self.last_feedback_wall = now

    def _event(self, now, name, **fields):
        self.events.append({"wall": now, "event": name, "progress_m": self.best, **fields})

    def step(self, now: float, s: dict | None):
        if self._failed is not None:
            return (0.0, 0.0), self._failed
        if s is None or s.get("state_seq") == self.last_seq:
            if now - self.last_state_wall > self.stale_s:
                self._event(now, "state_stale")
                return None, "state_stale"
            return None, None
        dt_wall, self.last_state_wall, self.last_seq = now - self.last_state_wall, now, s.get("state_seq")
        if not self.departed:
            if s.get("mode") != "TELEOP" or not s.get("armed", True):
                if now - self.started_wall > self.enter_s:
                    self._event(now, "teleop_not_entered", mode=s.get("mode"))
                    return None, "teleop_not_entered"
                return None, None
            self.departed = True
            self._event(now, "departed")
        if s.get("mode") != "TELEOP":
            self._event(now, "left_teleop", mode=s.get("mode"), stop_kind=s.get("stop_kind"))
            return None, "left_teleop"
        pose, speed = s.get("pose") or {}, s.get("rear_axle_speed_mps")
        if speed is None or pose.get("x_m") is None:
            # a state with no pose or speed is no feedback: the same limit as no state (Codex v2.5 round 2 P3)
            if now - self.last_feedback_wall > self.stale_s:
                self._event(now, "feedback_missing")
                return (0.0, 0.0), "feedback_missing"
            return None, None
        self.last_feedback_wall = now
        permit = s.get("permit") or {}
        blocked = permit.get("limit_mps") == 0.0 and permit.get("reason") in ("occupied", "unknown")
        if blocked and not self._blocked:
            # the console exposes no blocked arc or cells: the pose, command and steering the permit held it at
            # (Codex r6 diagnosis: what the next run needs to tell a real approach from arc margins)
            self._event(now, "permit_blocked", pose=pose, permit=permit, applied=s.get("applied"),
                        steering_rad=s.get("steering_rad"), leg=self._leg)
        self._blocked = blocked
        factor = float((s.get("pacing") or {}).get("target_factor") or 1.0)
        dt = min(max(dt_wall * factor, 1.0 / 120.0), 0.15)
        if self.replan_request is not None or getattr(self, "_replan_wait", None) is not None:
            # stopping after a tracker failure; the request goes out once the truck stands (planned from where it is),
            # within stop_wait_s (Codex v2.5 round 3 P2-3: a truck that never stands fails the tour)
            self._stop_since = self._stop_since if self._stop_since is not None else now
            if self.replan_request is None and abs(float(speed)) < 0.03:
                self.replan_request = ((pose["x_m"], pose["y_m"], pose["yaw_rad"]), self._replan_wait)
                self._replan_wait, self._stop_since = None, None
            elif self.replan_request is None and now - self._stop_since > self.stop_wait_s:
                self._event(now, "stop_timeout", speed=float(speed))
                self._failed = "stop_timeout"
                return (0.0, 0.0), "stop_timeout"
            return (0.0, 0.0), None
        tracker = self.tracker
        cmd = tracker.update((pose["x_m"], pose["y_m"], pose["yaw_rad"]), float(speed), dt)
        self.last_cmd = cmd
        progress = self.base_progress + cmd.progress_m
        if cmd.status == "failed":
            self._event(now, "tracker_failed", failure=cmd.failure, pose=pose, position_error_m=cmd.position_error_m)
            if self.can_replan and self.replans < self.max_replans:
                self.replans += 1
                self._replan_wait = self.waypoints_left(cmd.segment_index, (pose["x_m"], pose["y_m"]))
                return (0.0, 0.0), None
            return (0.0, 0.0), f"tracker_failed:{cmd.failure}"
        if cmd.status == "arrived":
            self._event(now, "arrived", position_error_m=cmd.position_error_m)
            return (0.0, 0.0), "arrived"
        pacing = ((s.get("teleop_forbidden_reason") or "").startswith("pacing")
                  or not (s.get("pacing") or {}).get("ok", True))
        # not moving, whatever the tracker asks -- but a stand the pacing forces is no stall (Codex v2.5 round 5 P2)
        standing = abs(float(speed)) < 0.03 and not pacing
        self.stand_since = (self.stand_since or now) if standing else None
        at_leg_end = tracker.leg_remaining_m() <= tracker.leg_end_window_m()  # release_cusp's own condition
        handled = False
        if standing and at_leg_end and now - self.stand_since >= self.release_s:
            if tracker.leg_end_index == self.last_index:
                if cmd.position_error_m <= self.goal_slack_m:
                    self._event(now, "arrived_slack", position_error_m=cmd.position_error_m, yaw_error_rad=cmd.yaw_error_rad)
                    return (0.0, 0.0), "arrived"
                # short of the goal by more than the slack: the stall below replans (Codex v2.5 round 4 P2-1)
            elif tracker.release_cusp(True):
                self._event(now, "cusp_released", position_error_m=cmd.position_error_m, yaw_error_rad=cmd.yaw_error_rad)
                self.stand_since = None
                handled = True
        if (not handled and standing and self.stand_since is not None and now - self.stand_since >= self.release_s
              and self._steer_since is None and tracker.leg_remaining_m() <= STUCK_NEAR_END_M
              and self.can_replan and self.replans < self.max_replans):
            # stood short of a leg end outside its tolerance, the tracker asking ~nothing (ws1 job 6828: 0.24 m beside
            # a cusp waypoint, the braking profile at zero): a new path from here, the waypoint beside counted passed
            self._event(now, "stuck", leg_remaining_m=tracker.leg_remaining_m(), position_error_m=cmd.position_error_m)
            self.replans += 1
            self._replan_wait = self.waypoints_left(cmd.segment_index, (pose["x_m"], pose["y_m"]))
            self.stand_since = None
            return (0.0, 0.0), None
        steer = s.get("steering_rad")
        steer_now = None if not steer else float(np.mean(steer))
        settling = (steer_now is not None and self._last_steer is not None
                    and abs(steer_now - self._last_steer) > self.steer_settle_rad)
        self._last_steer = steer_now
        # steer standing at a leg's start (see the class doc); the tick the leg changes on carries the old leg's last
        # command, so the new leg's is judged from the next state on (Codex v2.5 round 3 P2-1)
        if tracker.leg_index != self._leg:
            self._leg, self._steer_since = tracker.leg_index, now
            tracker.restart_speed_slew(0.0)
            self.command = (0.0, self.command[1] if self.command else 0.0)
            return self.command, None
        if self._steer_since is not None:
            applied = (s.get("applied") or {}).get("curvature_inv_m")
            off = applied is not None and abs(float(applied) - cmd.curvature_inv_m) > self.steer_tolerance
            if off or settling:
                if now - self._steer_since > self.steer_wait_s:
                    # the steering never settled where the leg starts: not driven anyway -- a new path from here
                    self._event(now, "steer_timeout", applied=applied, target=cmd.curvature_inv_m, steering=steer_now)
                    self._steer_since = None
                    if self.can_replan and self.replans < self.max_replans:
                        self.replans += 1
                        self._replan_wait = self.waypoints_left(cmd.segment_index, (pose["x_m"], pose["y_m"]))
                        return (0.0, 0.0), None
                    self._failed = "steer_timeout"
                    return (0.0, 0.0), "steer_timeout"
                tracker.restart_speed_slew(0.0)
                self.stand_since = None  # standing to steer is not standing stuck
                self.command = (0.0, max(-1.0, min(1.0, cmd.curvature_inv_m / self.kappa)))
                return self.command, None
            self._steer_since = None
        cap = self.forward if cmd.speed_mps >= 0 else self.reverse
        self.command = (max(-1.0, min(1.0, cmd.speed_mps / cap)), max(-1.0, min(1.0, cmd.curvature_inv_m / self.kappa)))
        if progress > self.best + 0.05:
            self.best, self.drivable_idle, self.pacing_idle = progress, 0.0, 0.0
        else:
            if pacing:
                self.pacing_idle += dt_wall
            else:
                self.drivable_idle += dt_wall
                self.pacing_idle = 0.0
            if self.drivable_idle > self.replan_stall_s and self.can_replan and self.replans < self.max_replans:
                # no progress anywhere on a leg (ws1 job 6831: the pallet permit held a truck turned toward the pickup
                # zone): a new path from here -- it may start backing away -- before the stall fails the tour
                self._event(now, "stall_replan", pose=pose, permit=s.get("permit"), drivable_idle_s=self.drivable_idle)
                self.replans += 1
                self._replan_wait = self.waypoints_left(cmd.segment_index, (pose["x_m"], pose["y_m"]))
                self.drivable_idle = 0.0
                return (0.0, 0.0), None
            if self.drivable_idle > self.stall_s or self.pacing_idle > self.pacing_wait_s:
                self._event(now, "stalled", pose=pose, permit=s.get("permit"), drive_allowed=s.get("drive_allowed"),
                            teleop_forbidden_reason=s.get("teleop_forbidden_reason"), pacing=s.get("pacing"),
                            status=cmd.status, drivable_idle_s=self.drivable_idle, pacing_idle_s=self.pacing_idle)
                return (0.0, 0.0), "stalled"
        return self.command, None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="command", required=True)
    p = sub.add_parser("plan")
    p.add_argument("--result", type=Path, required=True, help="a run's result.json on this scene (its scenario)")
    p.add_argument("--tour", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--nodes", type=Path, default=None,
                   help="a survey's nodes.csv (corrected base poses) whose coverage counts too")
    p.add_argument("--reference-run", type=Path, action="append", default=[],
                   help="a run of operational missions: its observation poses join the judged region")
    p.add_argument("--refine", type=int, default=0, help="add up to this many waypoints for uncovered clusters")
    args = ap.parse_args(argv)
    result = json.loads(args.result.read_text())
    config = yaml.safe_load(args.tour.read_text())
    tour = [list(map(float, p)) for p in config["tour"]]
    aisles = [tuple(map(tuple, a)) for a in config["aisles"]]
    judged = judged_region(result, aisles, observation_poses(args.reference_run))
    sites = np.empty((0, 2))
    if args.nodes is not None:
        import csv

        with open(args.nodes, newline="") as handle:
            rows = list(csv.DictReader(handle))
        rx, ry = base_to_rear(np.array([float(r["x"]) for r in rows]), np.array([float(r["y"]) for r in rows]),
                              np.array([float(r["yaw"]) for r in rows]),
                              result["arguments"]["rear_axle_offset_m"])
        sites = np.column_stack((rx, ry))
    if args.refine:
        tour, plan = refine_tour(result, tour, sites, args.refine, judged)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.with_suffix(".yaml").write_text(yaml.safe_dump({"tour": tour}))
    else:
        plan = plan_tour(result, tour)
    report = {"status": plan.status, "length_m": float(plan.length_m) if plan.success else None,
              "waypoints": len(tour), "samples": int(len(plan.poses)), "scenario_sha256": scenario_sha256(result),
              "result": str(args.result)}
    if plan.success:
        directions = np.asarray(plan.directions)
        report["cusps"] = int((np.diff(directions) != 0).sum())
        report["reverse_m"] = float(np.sum(np.hypot(*np.diff(plan.poses[:, :2], axis=0).T)[directions[1:] < 0]))
        path = np.asarray(plan.poses)[:, :2]
        both = np.vstack([sites, path])
        report["judged"] = region_coverage(judged, both)
        report["unsurveyed_estimate"] = region_coverage(placeable_estimate(result), both)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        np.savez(args.out, poses=plan.poses, directions=plan.directions, curvatures_inv_m=plan.curvatures_inv_m,
                 tour=np.asarray(tour, float), scenario_sha256=np.asarray(scenario_sha256(result)),
                 plan_input=np.asarray(json.dumps({k: result[k] for k in PLAN_KEYS})))
    print(json.dumps(report))
    (args.out.with_suffix(".json")).write_text(json.dumps(report, indent=1))
    return 0 if plan.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
