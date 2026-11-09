"""Recompute a live session's observe-replan start check from its records (docs/validation/2026-11-10-l4c-repro-
results.md 4): the static check the observation leg's search makes at its start -- the mission's scene props (the
search stage, grown by the LiDAR keepout above the plane), the target pallet's prior (its session zone grown by the
pallet's half diagonal, scene_file.zone_rect, as safety.pallet_permit.preparation_obstacle gives a prior pallet) and
the mission's bounds, against the unloaded footprint -- with no occupancy (a scene-layout run without
--grid-planning plans its observation on its layout, run_transport grid_kwargs).

    python tools/observe_start_recheck.py RUN_DIR --mission m2 --scene-file SCENE.json [--json OUT]

RUN_DIR is a session's ``live/epoch_1/run`` (result.json, missions/<id>/records.json). The pose checked is the last
observe replan's ``rear_pose``. Reports whether the start is free at zero clearance and at the planner's, the largest
clearance it is free at, which rectangles (or the bounds) block it at the planner's clearance, and the straight start
advice backwards and forwards (pallet_mission.straight_start_advice, 0.01 m steps, 0.30 m)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "sim" / "isaac"))

from forklift_core.planning.geometry import Bounds, Footprint, Rectangle  # noqa: E402
from forklift_core.planning.grid_collision import make_checker  # noqa: E402
from forklift_core.planning.hybrid_astar import Pose2D  # noqa: E402
from forklift_core.planning.pallet_mission import LIDAR_PLANE_M, SEARCH_TAG, SLOT_TAG, straight_start_advice  # noqa: E402,F401

FAR = Bounds(-1e4, 1e4, -1e4, 1e4)


def props_of(scenario: dict, keepout_m: float = 0.0, plane_m: float = LIDAR_PLANE_M) -> list:
    """(label, Rectangle) of the recorded scenario's props a search sees (scenario_props(stage="search"))."""
    out = []
    for p in scenario.get("props") or []:
        uri = str(p["asset"]["uri"])
        if uri.endswith(SLOT_TAG):
            continue
        r = p["rectangle"]
        grow = keepout_m if keepout_m > 0.0 and float(p["asset"]["height_m"]) > plane_m else 0.0
        out.append((uri, Rectangle(float(r["x_m"]), float(r["y_m"]), float(r["length_m"]) + 2 * grow,
                                   float(r["width_m"]) + 2 * grow, float(r.get("yaw_rad", 0.0)))))
    return out


def max_free_clearance(free, pose, hi: float = 1.0, tol: float = 1e-5, cap: float = 64.0):
    """(clearance, capped): the largest clearance the pose is free at (bisection after doubling ``hi``), (None, False)
    when it is not free at zero; capped True when it is still free at ``cap`` (the value is then a lower bound)."""
    if not free(pose, 0.0):
        return None, False
    while free(pose, hi):
        if hi >= cap:
            return hi, True
        hi = min(2 * hi, cap)
    lo = 0.0
    while hi - lo > tol:
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if free(pose, mid) else (lo, mid)
    return lo, False


def recheck(rects: list, footprint: Footprint, bounds: Bounds, pose, clearance_m: float) -> dict:
    """The start check of ``pose`` (x, y, yaw) against ``rects`` [(label, Rectangle)] inside ``bounds``."""
    checker = make_checker([r for _, r in rects], footprint, bounds, None)
    blockers = [label for label, r in rects
                if not make_checker([r], footprint, FAR, None).free(pose, clearance_m)]
    if not make_checker([], footprint, bounds, None).free(pose, clearance_m):
        blockers.append("bounds")
    start = Pose2D(float(pose[0]), float(pose[1]), float(pose[2]))
    clear_, capped_ = max_free_clearance(checker.free, pose)
    return {"pose": [float(v) for v in pose], "clearance_m": clearance_m,
            "free_at_zero": bool(checker.free(pose, 0.0)), "free_at_clearance": bool(checker.free(pose, clearance_m)),
            "max_free_clearance_m": clear_, "max_free_clearance_is_lower_bound": capped_,
            "blockers": blockers,
            "advice_back": straight_start_advice(checker.free, clearance_m, start, -1),
            "advice_forward": straight_start_advice(checker.free, clearance_m, start, +1)}


class Unsupported(ValueError):
    """A record this reconstruction does not cover (it would rebuild a different check)."""


def supported(result: dict, missions: dict, mission_id: str, replan_t: float) -> None:
    """Raise Unsupported unless the check is the one rebuilt here: a scene layout planning without --grid-planning
    (no occupancy, the props kept), the replan before the mission's approach (the observation still holds the pallet
    as its prior, not a perceived outline -- run_transport grid_kwargs), and no earlier mission in the session for the
    same pallet (a prior at its first place: its session zone, safety.pallet_permit preparation_obstacle; an earlier
    mission may have recognised it even when cancelled)."""
    args = result.get("arguments") or {}
    if args.get("layout") != "scene" or args.get("grid_planning") is not False:
        raise Unsupported("only a --layout scene run without --grid-planning (its observation plans on the layout)")
    me = missions[mission_id]
    t0, t1 = float(me["started_s"]), float(me["ended_s"])
    trs = sorted((float(t["time_s"]), t.get("to")) for t in result.get("transitions") or [])
    if any(to == "approach" and t0 <= t <= replan_t for t, to in trs):
        raise Unsupported("the replan came after the mission's approach (the pallet held as its perceived outline)")
    for mid, m in missions.items():
        # any earlier mission for the same pallet may have recognised it (a cancel in the approach keeps it
        # recognised, core's targets book): its next preparation holds the perceived outline, not the zone
        if mid != mission_id and m.get("pallet_id") == me.get("pallet_id") and float(m["started_s"]) < t0:
            raise Unsupported(f"{me.get('pallet_id')} had an earlier mission {mid} (it may be recognised: no longer "
                              "a prior at its first place)")


def inputs_of(run_dir: Path, mission_id: str, scene_file: Path) -> dict:
    """Everything the check reads, from the session's records and its scene file (Unsupported when the record is
    not the case rebuilt here)."""
    from scene_file import load_session, zone_rect

    result = json.loads((run_dir / "result.json").read_text())
    rec = json.loads((run_dir / "missions" / mission_id / "records.json").read_text())
    state = rec.get("state", rec)
    missions = {d.name: json.loads((d / "result.json").read_text()).get("result", {})
                for d in sorted((run_dir / "missions").iterdir()) if (d / "result.json").exists()}
    mres = missions[mission_id]
    replans = state.get("observe_replans") or []
    if replans:
        supported(result, missions, mission_id, float(replans[-1]["time_s"]))
    geom = result["geometry"]
    fp = geom["unloaded_footprint"]
    footprint = Footprint(float(fp["front_m"]), float(fp["rear_m"]), float(fp["half_width_m"]))
    keepout = (result.get("arguments") or {}).get("lidar_keepout_m") or 0.0  # unrecorded: the runner's default 0
    scenario = state["mission_scenario"]
    b = scenario["bounds"]
    bounds = Bounds(float(b["x_min_m"]), float(b["x_max_m"]), float(b["y_min_m"]), float(b["y_max_m"]))
    pallet_id = mres["pallet_id"]
    session = load_session(scene_file)
    pallet = next(p for p in session.pallets if p.pallet_id == pallet_id)
    zone = zone_rect(pallet.zone, float(geom["pallet_depth_m"]), float(geom["pallet_width_m"]))
    rects = props_of(scenario, float(keepout)) + [(f"prior:{pallet_id}", zone)]
    return {"rects": rects, "footprint": footprint, "bounds": bounds, "pose": replans[-1]["rear_pose"] if replans else None,
            "status": replans[-1]["status"] if replans else None,
            "clearance_m": float(result["planner_config"]["clearance_m"]), "pallet_id": pallet_id,
            "zone": [zone.x_m, zone.y_m, zone.length_m, zone.width_m, zone.yaw_rad]}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--mission", required=True)
    ap.add_argument("--scene-file", type=Path, required=True)
    ap.add_argument("--json", type=Path)
    a = ap.parse_args(argv)
    try:
        inp = inputs_of(a.run_dir, a.mission, a.scene_file)
    except Unsupported as exc:
        print(f"unsupported record: {exc}")
        return 2
    if inp["pose"] is None:
        print("no observe replan in this mission")
        return 1
    out = {"recorded_status": inp["status"], "pallet_id": inp["pallet_id"], "prior_zone": inp["zone"],
           **recheck(inp["rects"], inp["footprint"], inp["bounds"], inp["pose"], inp["clearance_m"])}
    text = json.dumps(out, indent=1)
    if a.json:
        a.json.write_text(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
