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
    """The observation poses a reference run's missions planned (its result.json and missions/*/result.json)."""
    poses = []
    for run in reference_runs:
        for path in [Path(run) / "result.json"] + sorted((Path(run) / "missions").glob("*/result.json")):
            if not path.exists():
                continue
            data = json.loads(path.read_text())
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


def plan_tour(result: dict, tour: list) -> SimpleNamespace:
    s, g = result["scenario"], result["geometry"]
    pickup = s["pickup"]
    transport = SimpleNamespace(props=[SimpleNamespace(rectangle=_rect(p["rectangle"])) for p in s["props"]],
                                pickup=SimpleNamespace(x_m=pickup["x_m"], y_m=pickup["y_m"], yaw_rad=pickup["yaw_rad"]),
                                bounds=Bounds(**s["bounds"]))
    geometry = SimpleNamespace(unloaded_footprint=Footprint(**g["unloaded_footprint"]),
                               pallet_depth_m=g["pallet_depth_m"], pallet_width_m=g["pallet_width_m"])
    home = Pose2D(**s["start_rear"])
    route = [home] + [Pose2D(*p) for p in tour] + [home]
    config = PlannerConfig(**result["travel_planner_config"])
    return plan_survey_route(SimpleNamespace(transport=transport), route, config, geometry=geometry)


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
    checker = FootprintCollisionChecker(scenario_obstacles(result, pallet=True), Footprint(**g),
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
                 tour=np.asarray(tour, float), scenario_sha256=np.asarray(scenario_sha256(result)))
    print(json.dumps(report))
    (args.out.with_suffix(".json")).write_text(json.dumps(report, indent=1))
    return 0 if plan.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
