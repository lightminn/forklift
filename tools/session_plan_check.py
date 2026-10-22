"""The planned envelope of a session's missions against its pallets' true outlines (CPU; docs/plans/
2026-10-22-session-scene-pallet-props.md 2.7).

A session scene (tools/riibotics_isaac_scenes.py --session) is planned mission after mission in its file's order, as an
Isaac run would plan it with ideal estimates and deliveries: each mission starts from the session's start and returns
there; the pallets carried before stand delivered at their slots (uncertainty 0.21 m); the others are prior pallets the
planner holds at their drawn slots + D1 (0.25 m, session_pallet_props). The observation leg plans on the mission
pallet's zone, the rest on its truth (as tools/riibotics_isaac_scenes.plan_scene). Then every planned pose's outline
(the planner's footprint, empty or loaded by stage) is measured against the pallets' truth outlines, between poses too:

    stage       outline   mission pallet                  others
    observe     empty     its truth outline               truth (prior: truth, delivered: the slot)
    approach    empty     its truth outline
    insert      empty     left out (the forks enter it)
    extract     loaded    carried (left out)
    transport   loaded    carried
    withdraw    empty     left out (the forks leave its pockets)
    return_home empty     its slot

A mission is contact_observed when a sampled pose overlaps a truth outline, clearance_unproven when only the
continuous lower bound (sampled minimum - half the 5 mm sampling step - the arc's sagitta) reaches 0 even at 0.5 mm.
A mission runs only if every earlier one planned and passed (else prior_failed): "the file's order, full returns,
ideal deliveries" -- not a run's outcome.

    python tools/session_plan_check.py SESSION.json --out RESULT.json [--prior-grow-m 0.25]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT / "tools", ROOT / "sim/isaac", ROOT):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from bay_planning_sweep import mission_geometry  # noqa: E402
from riibotics_scenario_paths import PALLETS, STAGES  # noqa: E402
from scene_file import load_session, register_session, zone_rect  # noqa: E402

from forklift_core.mission.targets import Pose, TargetsBook  # noqa: E402
from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.planning import Pose2D  # noqa: E402
from forklift_core.planning.geometry import Rectangle  # noqa: E402
from forklift_core.planning.pallet_mission import (  # noqa: E402
    D7_PLANNER_OPTIONS,
    TransportScenario,
    make_transport_planner_config,
    plan_observation_leg,
    plan_transport,
)
from forklift_core.safety.pallet_permit import RECOGNISED_MARGIN_M, session_pallet_props  # noqa: E402

URDF = ROOT / "sim/models/dls08_measured/forklift.urdf"
STRAIGHTS = {"alignment_straight_m": 2.1, "delivery_straight_m": 1.5, "withdrawal_m": 0.55}
RESERVE_M, KAPPA, CLEARANCE_M, MAX_EXPANSIONS = 0.016, 0.29, 0.10, 30000
STEP_M, FINE_STEP_M = 0.005, 0.0005
LOADED = {"extract", "transport"}
OWN_TRUTH = {"observe", "approach"}  # the mission pallet where it stands
OWN_SLOT = {"return_home"}  # the mission pallet at its slot


# --- distances between oriented boxes -----------------------------------------------------------------------------
def footprint_corners(pose, fp) -> np.ndarray:
    x, y, yaw = (float(v) for v in pose)
    c, s = math.cos(yaw), math.sin(yaw)
    local = [(fp.front_m, fp.half_width_m), (-fp.rear_m, fp.half_width_m), (-fp.rear_m, -fp.half_width_m),
             (fp.front_m, -fp.half_width_m)]
    return np.array([(x + c * u - s * v, y + s * u + c * v) for u, v in local])


def rect_corners(r: Rectangle) -> np.ndarray:
    c, s = math.cos(r.yaw_rad), math.sin(r.yaw_rad)
    hx, hy = r.length_m / 2, r.width_m / 2
    return np.array([(r.x_m + c * u - s * v, r.y_m + s * u + c * v) for u, v in ((hx, hy), (-hx, hy), (-hx, -hy),
                                                                               (hx, -hy))])


def _overlap(a: np.ndarray, b: np.ndarray) -> bool:
    for poly in (a, b):
        for i in range(len(poly)):
            edge = poly[(i + 1) % len(poly)] - poly[i]
            axis = np.array([-edge[1], edge[0]])
            pa, pb = a @ axis, b @ axis
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


def _point_segment(p, a, b) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / max(float(np.dot(ab, ab)), 1e-18), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def distance(a: np.ndarray, b: np.ndarray) -> float:
    """The gap between two convex polygons; 0 when they touch or overlap."""
    if _overlap(a, b):
        return 0.0
    best = math.inf
    for poly, other in ((a, b), (b, a)):
        for p in poly:
            for i in range(len(other)):
                best = min(best, _point_segment(p, other[i], other[(i + 1) % len(other)]))
    return best


def _reach(fp) -> float:
    return math.hypot(max(fp.front_m, fp.rear_m), fp.half_width_m)


def leg_clearance(poses, curvatures, fp, outlines: dict) -> dict:
    """The leg's smallest gap to each outline: sampled at the planned poses and between them (each step moving no
    outline point more than STEP_M), with the continuous lower bound; a segment whose bound reaches 0 is sampled again
    at FINE_STEP_M. ``outlines`` id -> Rectangle. Returns per id {min_m, bound_m, status}."""
    poses = np.asarray(poses, dtype=float)
    curvatures = np.zeros(len(poses)) if curvatures is None else np.abs(np.asarray(curvatures, dtype=float))
    out = {}
    reach = _reach(fp)
    for oid, rect in outlines.items():
        corners = rect_corners(rect)
        sampled, bound = math.inf, math.inf
        for i in range(len(poses)):
            sampled = min(sampled, distance(footprint_corners(poses[i], fp), corners))
        for i in range(1, len(poses)):
            a, b = poses[i - 1], poses[i]
            ds = float(math.hypot(b[0] - a[0], b[1] - a[1]))
            dyaw = abs(math.remainder(float(b[2] - a[2]), math.tau))
            sag = float(curvatures[i]) * ds * ds / 8
            seg = _segment(a, b, ds, dyaw, reach, corners, fp, STEP_M)
            lower = seg[0] - seg[1] / 2 - sag
            if lower <= 0 < seg[0]:
                seg = _segment(a, b, ds, dyaw, reach, corners, fp, FINE_STEP_M)
                lower = seg[0] - seg[1] / 2 - sag
            sampled, bound = min(sampled, seg[0]), min(bound, lower)
        if len(poses) == 1:
            bound = sampled
        status = "contact_observed" if sampled <= 0 else ("clearance_unproven" if bound <= 0 else "clear")
        out[oid] = {"min_m": sampled, "bound_m": bound, "status": status}
    return out


def _segment(a, b, ds, dyaw, reach, corners, fp, step):
    """(the smallest sampled gap along a -> b, the largest outline-point move between samples)."""
    move = ds + dyaw * reach
    n = max(1, int(math.ceil(move / step)))
    best = math.inf
    for k in range(n + 1):
        t = k / n
        pose = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]),
                a[2] + t * math.remainder(float(b[2] - a[2]), math.tau))
        best = min(best, distance(footprint_corners(pose, fp), corners))
    return best, move / n


# --- the session ----------------------------------------------------------------------------------------------------
def _rect(site, pallet) -> Rectangle:
    return Rectangle(float(site.x_m), float(site.y_m), float(pallet.overall_depth_m), float(pallet.overall_width_m),
                     float(site.yaw_rad))


def check_session(path: Path, *, prior_grow_m: float = 0.25, planner=None) -> dict:
    """``planner``: (plan_observation_leg, plan_transport) stand-ins for tests."""
    observe_planner, transport_planner = planner or (plan_observation_leg, plan_transport)
    session = load_session(path)
    pallet = load_pallet_geometry(PALLETS[session.pallet])
    geometry = mission_geometry(URDF, pallet, RESERVE_M, **STRAIGHTS)
    config = make_transport_planner_config(curvature_limit_inv_m=KAPPA, clearance_m=CLEARANCE_M,
                                           max_expansions=MAX_EXPANSIONS, **D7_PLANNER_OPTIONS)
    targets = TargetsBook()
    register_session(targets, session, kind=session.pallet, depth_m=pallet.overall_depth_m,
                     width_m=pallet.overall_width_m, live=False)
    nominal = {p.pallet_id: (p.nominal.x_m, p.nominal.y_m, p.nominal.yaw_rad) for p in session.pallets}
    shape = (pallet.overall_depth_m, pallet.overall_width_m, pallet.overall_height_m)
    result = {"session": session.name, "session_sha256": session.sha256, "prior_grow_m": prior_grow_m,
              "settings": {"urdf": str(URDF.relative_to(ROOT)), "reserve_m": RESERVE_M, **STRAIGHTS, "kappa": KAPPA,
                           "clearance_m": CLEARANCE_M, "max_expansions": MAX_EXPANSIONS,
                           "d7_options": {k: v for k, v in D7_PLANNER_OPTIONS.items()},
                           "step_m": STEP_M, "fine_step_m": FINE_STEP_M},
              "missions": []}
    slots: dict = {}  # pallet id -> the slot it was delivered to
    failed = False
    for k, (pid, did) in enumerate(session.sequence):
        entry = {"index": k, "pallet_id": pid, "destination_id": did}
        result["missions"].append(entry)
        if failed:
            entry["outcome"] = "prior_failed"
            continue
        started = time.monotonic()
        own, dest = session.pallet_by_id(pid), session.destination(did)
        props = tuple(session.static_props) + session_pallet_props(
            targets, exclude_id=pid, nominal=nominal, shape=shape, prior_grow_m=prior_grow_m)
        scenario = TransportScenario(seed=0, start_rear=session.start_rear, pickup=own.truth, destination=dest,
                                     props=props, bounds=session.bounds)
        observe = None
        zone = zone_rect(own.zone, pallet.overall_depth_m, pallet.overall_width_m)
        entry["candidates"] = []
        for extended in (False, True):
            for index, w in enumerate(own.observation_waypoints):
                leg = observe_planner(scenario, Pose2D(*w), config, geometry=geometry, extended=extended,
                                      pickup_obstacle=zone)
                entry["candidates"].append({"index": index, "extended": extended, "status": leg.status})
                if leg.success:
                    observe = leg
                    break
            if observe is not None:
                break
        if observe is None:
            entry["outcome"], failed = "observe_failed", True
            entry["wall_s"] = round(time.monotonic() - started, 1)
            continue
        end = observe.poses[-1]
        plan = transport_planner(scenario, config, geometry=geometry, target_pickup=own.truth,
                                 start_rear=Pose2D(float(end[0]), float(end[1]), float(end[2])),
                                 return_to=session.start_rear, pickup_obstacle=_rect(own.truth, pallet))
        entry["plan_status"] = plan.status
        if not plan.success:
            entry["outcome"], failed = "plan_failed", True
            entry["wall_s"] = round(time.monotonic() - started, 1)
            continue
        # the truth outlines by stage
        others = {}
        for q in session.pallets:
            if q.pallet_id == pid:
                continue
            others[q.pallet_id] = _rect(slots[q.pallet_id], pallet) if q.pallet_id in slots else _rect(q.truth, pallet)
        legs = [("observe", observe)] + [(name, getattr(plan, name, None)) for name in STAGES]
        entry["stages"] = {}
        worst = "clear"
        for name, leg in legs:
            if leg is None or len(leg.poses) == 0:
                continue
            outlines = dict(others)
            if name in OWN_TRUTH:
                outlines[pid] = _rect(own.truth, pallet)
            elif name in OWN_SLOT:
                outlines[pid] = _rect(dest, pallet)
            fp = geometry.loaded_footprint if name in LOADED else geometry.unloaded_footprint
            clear = leg_clearance(leg.poses, getattr(leg, "curvatures_inv_m", None), fp, outlines)
            nearest = min(clear, key=lambda oid: clear[oid]["bound_m"]) if clear else None
            entry["stages"][name] = {"nearest": nearest, **({} if nearest is None else clear[nearest]),
                                     "all": clear}
            for c in clear.values():
                if c["status"] == "contact_observed" or (c["status"] == "clearance_unproven" and worst == "clear"):
                    worst = c["status"]
        entry["outcome"] = "passed" if worst == "clear" else worst
        entry["wall_s"] = round(time.monotonic() - started, 1)
        if worst != "clear":
            failed = True
            continue
        # an ideal delivery: the pallet stands at its slot, a delivered estimate
        targets.carried(pid)
        targets.put_down(pid, Pose(float(dest.x_m), float(dest.y_m), float(dest.yaw_rad)),
                         uncertainty_m=RECOGNISED_MARGIN_M)
        slots[pid] = dest
    return result


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("session", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--prior-grow-m", type=float, default=0.25)
    args = ap.parse_args(argv)
    result = check_session(args.session, prior_grow_m=args.prior_grow_m)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=1, default=float) + "\n")
    for m in result["missions"]:
        print(m["index"], m["pallet_id"], m["destination_id"], m["outcome"], flush=True)
    return 0 if all(m["outcome"] == "passed" for m in result["missions"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
