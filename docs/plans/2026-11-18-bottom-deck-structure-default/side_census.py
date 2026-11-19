"""Side-stand census: for each Riibotics scene's pickup, where the bottom-deck side stands (bottom_deck_legs.stand,
x hypothesis, both faces) fall -- free, footprint blocked (by a pallet or by something else), outside the hall -- and
what lies on the camera's sight line to the face centre; with --plan also the three side-stage paths the core
plans (far observation point -> first stand -> second stand -> back), with the scene tool's planner settings.

    python side_census.py <repo> <out.json> [--plan]
"""

import json
import math
import sys
from pathlib import Path

ROOT = Path(sys.argv[1]).resolve()
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "tools"), str(ROOT / "sim/isaac")]

import riibotics_isaac_scenes as R  # noqa: E402
from scene_file import scenario_from_record  # noqa: E402

from forklift_core.mission import bottom_deck_legs as L  # noqa: E402
from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.planning.geometry import (  # noqa: E402
    FootprintCollisionChecker,
    Rectangle,
)

MOUNT_X, REAR_OFF = 0.619, 0.34
CLEAR = R.CLEARANCE_M
pallet = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
geo = R.mission_geometry(R.URDF, pallet, R.RESERVE_M, **R.STRAIGHTS)
fp = geo.unloaded_footprint


def seg_hits_rect(p, q, r: Rectangle) -> bool:
    c, s = math.cos(r.yaw_rad), math.sin(r.yaw_rad)

    def local(pt):
        dx, dy = pt[0] - r.x_m, pt[1] - r.y_m
        return (c * dx + s * dy, -s * dx + c * dy)

    a, b = local(p), local(q)
    hx, hy = r.length_m / 2, r.width_m / 2
    t0, t1 = 0.0, 1.0
    for i, h in ((0, hx), (1, hy)):
        d = b[i] - a[i]
        if abs(d) < 1e-12:
            if abs(a[i]) > h:
                return False
            continue
        u0, u1 = (-h - a[i]) / d, (h - a[i]) / d
        if u0 > u1:
            u0, u1 = u1, u0
        t0, t1 = max(t0, u0), min(t1, u1)
        if t0 > t1:
            return False
    return True


PLAN = "--plan" in sys.argv


def side_plans(rec, sc, own):
    """As the core plans the side stage (core._bottom_deck_begin / _bottom_deck_plan): from the first far observation
    waypoint the scene tool's own planning reaches, to the first stand, to the second, back to that waypoint -- at the
    planning clearance, then at none. The zone is the record's pickup zone (a batch run's observe bounds)."""
    from dataclasses import replace

    from forklift_core.planning import Bounds, Pose2D
    from forklift_core.planning.pallet_mission import plan_observation_leg

    config = R.make_transport_planner_config(curvature_limit_inv_m=R.KAPPA, clearance_m=R.CLEARANCE_M,
                                             max_expansions=30000, **R.D7_PLANNER_OPTIONS)
    z = rec["pickup_zone"]
    zone = Bounds(z["x_min_m"], z["x_max_m"], z["y_min_m"], z["y_max_m"])
    far = None
    for w in rec["observation_waypoints"]:
        leg = plan_observation_leg(sc, Pose2D(*w), config, geometry=geo, extended=False, pickup_obstacle=own)
        if leg.success:
            far = tuple(float(v) for v in leg.poses[-1])
            break
    if far is None:
        return {"far": "observe_failed"}
    pk = sc.pickup
    order = L.ordered_faces("x", (pk.x_m, pk.y_m), pk.yaw_rad, rear_xy=far[:2], mount_x_m=MOUNT_X,
                            rear_axle_offset_m=REAR_OFF)
    st = {f: L.stand((pk.x_m, pk.y_m), pk.yaw_rad, "x", f, mount_x_m=MOUNT_X, rear_axle_offset_m=REAR_OFF)
          for f in order}
    bounds = L.side_bounds(zone, sc.bounds, [st[f].rear for f in order] + [L.retreat_pose(st[f].rear) for f in order]
                           + [far], fp, min(R.CLEARANCE_M, geo.approach_gap_m / 2))

    def plan(a, b):
        out = None
        for cfg in (config, replace(config, clearance_m=0.0)):
            out = plan_observation_leg(sc, Pose2D(*b), cfg, geometry=geo, start_rear=Pose2D(*a),
                                       pickup_bounds=bounds, extended=False, pickup_obstacle=own)
            if out.success:
                break
        return out.status

    return {"far": "ok", "order": list(order),
            "first": plan(far, st[order[0]].rear), "second": plan(st[order[0]].rear, st[order[1]].rear),
            "return": plan(st[order[1]].rear, far)}


built = R.scenes()
for kind in R.MARKED_KINDS:
    for name in [n for n in list(built) if n.startswith(kind + "_")]:
        built[f"{name}_m"] = R.with_slot_markers(kind, built[name])
for name in ("aisle", "aisle_out"):
    built[f"{name}_rack"] = R.rack_scene()
rows = []
for name in sorted(built):
    rec = R.scene_record(name, built[name])
    sc = scenario_from_record(rec["scenario"])
    pk = sc.pickup
    own = Rectangle(pk.x_m, pk.y_m, pallet.overall_depth_m, pallet.overall_width_m, pk.yaw_rad)
    props = [(p.asset.uri, p.rectangle) for p in sc.props]
    checker = FootprintCollisionChecker([r for _, r in props] + [own], fp, R.HALL)
    faces = {}
    for face in (1, -1):
        st = L.stand((pk.x_m, pk.y_m), pk.yaw_rad, "x", face, mount_x_m=MOUNT_X, rear_axle_offset_m=REAR_OFF)
        rear = st.rear
        free = checker.free(rear, CLEAR)
        hit = []
        if not free:
            for uri, r in props:
                if not FootprintCollisionChecker([r], fp, R.HALL).free(rear, CLEAR):
                    hit.append(uri)
            if not FootprintCollisionChecker([own], fp, R.HALL).free(rear, CLEAR):
                hit.append("own_pallet")
            if not hit:
                hit.append("outside_hall")
        my = L.model_yaw(pk.yaw_rad, "x")
        n = (-math.sin(my) * face, math.cos(my) * face)
        fc = (pk.x_m + n[0] * pallet.overall_width_m / 2, pk.y_m + n[1] * pallet.overall_width_m / 2)
        sight = [uri for uri, r in props if seg_hits_rect(st.camera_xy, fc, r)]
        faces[face] = {"stand_free": free, "stand_blockers": sorted(set(hit)), "sight_blockers": sight}
    plans = side_plans(rec, sc, own) if PLAN else None
    rows.append({"scene": name, "pickup": [round(pk.x_m, 3), round(pk.y_m, 3), round(math.degrees(pk.yaw_rad), 1)],
                 "faces": faces, "plans": plans})
for r in rows:
    f = r["faces"]

    def tag(d):
        if d["stand_free"] and not d["sight_blockers"]:
            return "free"
        b = d["stand_blockers"] + [f"sight:{s}" for s in d["sight_blockers"]]
        kinds = {"pallet" if "pallet" in x else x if x in ("outside_hall", "own_pallet") else "other" for x in b}
        return "+".join(sorted(kinds)) + " " + ",".join(sorted(set(x.replace("riibotics:", "") for x in b)))[:110]

    print(f"{r['scene']:16s} +1: {tag(f[1])}\n{'':16s} -1: {tag(f[-1])}\n{'':16s} plans: {r['plans']}")
json.dump(rows, open(sys.argv[2], "w"), indent=1)
