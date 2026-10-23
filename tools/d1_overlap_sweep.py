"""D1 + planning clearance overlap (CT-1 follow-up, line_m seed 0 m1 extract:straight_collision): for each mission of a
session, the recognition error along the mission pallet's own axis at which its transport plan stops planning --
bisected on the real planner from tools/session_plan_check.py's setup, the earlier missions delivered ideally to
their slots. Variants: the current rule (prior neighbours = nominal + D1, every fixed straight keeps the planning
clearance on top), the fixed straights at zero clearance (an upper bound of "no clearance on top of D1"), and smaller
D1. The thresholds are read against the recorded error distribution separately.

    python tools/d1_overlap_sweep.py SESSION.json --out RESULT.json [--variants current,straights0,d1_020,d1_015]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tools"))

import session_plan_check as spc  # noqa: E402
from forklift_core.planning import Pose2D  # noqa: E402
from forklift_core.planning import pallet_mission as pm  # noqa: E402
from forklift_core.planning.pallet_mission import PalletSite, TransportScenario  # noqa: E402

VARIANTS = {"current": (0.25, False), "straights0": (0.25, True), "d1_020": (0.20, False), "d1_015": (0.15, False)}


def _shifted(site, along_m: float) -> PalletSite:
    return PalletSite(site.x_m + along_m * math.cos(site.yaw_rad), site.y_m + along_m * math.sin(site.yaw_rad),
                      site.yaw_rad)


def sweep(path: Path, variants, *, limit_m: float = 0.15, tol_m: float = 0.0025) -> dict:
    session = spc.load_session(path)
    pallet = spc.load_pallet_geometry(spc.PALLETS[session.pallet])
    geometry = spc.mission_geometry(spc.URDF, pallet, spc.RESERVE_M, **spc.STRAIGHTS)
    config = spc.make_transport_planner_config(curvature_limit_inv_m=spc.KAPPA, clearance_m=spc.CLEARANCE_M,
                                               max_expansions=spc.MAX_EXPANSIONS, **spc.D7_PLANNER_OPTIONS)
    nominal = {p.pallet_id: (p.nominal.x_m, p.nominal.y_m, p.nominal.yaw_rad) for p in session.pallets}
    shape = (pallet.overall_depth_m, pallet.overall_width_m, pallet.overall_height_m)
    out = {"session": session.name, "settings": {"clearance_m": spc.CLEARANCE_M, "kappa": spc.KAPPA,
                                                 "limit_m": limit_m, "tol_m": tol_m}, "variants": {}}
    real_straight = pm._straight_plan
    for name in variants:
        grow, straights0 = VARIANTS[name]
        if straights0:
            pm._straight_plan = lambda *a, **k: real_straight(*a[:6], 0.0, *a[7:], **k)
        else:
            pm._straight_plan = real_straight
        try:
            targets = spc.TargetsBook()
            spc.register_session(targets, session, kind=session.pallet, depth_m=pallet.overall_depth_m,
                                 width_m=pallet.overall_width_m, live=False)
            rows = []
            for pid, did in session.sequence:
                own, dest = session.pallet_by_id(pid), session.destination(did)
                props = tuple(session.static_props) + spc.session_pallet_props(
                    targets, exclude_id=pid, nominal=nominal, shape=shape, prior_grow_m=grow)
                started = time.monotonic()

                def plans(along):
                    est = _shifted(own.truth, along)
                    scenario = TransportScenario(seed=0, start_rear=session.start_rear, pickup=est, destination=dest,
                                                 props=props, bounds=session.bounds)
                    # from the observation waypoint the session check reaches (its first candidate)
                    w = own.observation_waypoints[0]
                    plan = spc.plan_transport(scenario, config, geometry=geometry, target_pickup=est,
                                              start_rear=Pose2D(*w), return_to=session.start_rear,
                                              pickup_obstacle=spc._rect(est, pallet))
                    return plan.success, plan.status

                ok0, st0 = plans(0.0)
                row = {"pallet_id": pid, "destination_id": did, "at_truth": st0}
                for sign, key in ((1, "deeper_m"), (-1, "shallower_m")):
                    if not ok0:
                        row[key] = None
                        continue
                    ok_far, st_far = plans(sign * limit_m)
                    if ok_far:
                        row[key], row[key + "_status"] = f">{limit_m}", None
                        continue
                    lo, hi = 0.0, limit_m
                    while hi - lo > tol_m:
                        mid = (lo + hi) / 2
                        ok, st = plans(sign * mid)
                        lo, hi = (mid, hi) if ok else (lo, mid)
                    row[key], row[key + "_status"] = round(lo, 4), plans(sign * hi)[1]
                row["wall_s"] = round(time.monotonic() - started, 1)
                rows.append(row)
                # an ideal delivery, as the session check does
                targets.carried(pid)
                targets.put_down(pid, spc.Pose(float(dest.x_m), float(dest.y_m), float(dest.yaw_rad)),
                                 uncertainty_m=spc.RECOGNISED_MARGIN_M)
            out["variants"][name] = rows
        finally:
            pm._straight_plan = real_straight
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("session", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--variants", default="current,straights0,d1_020,d1_015")
    ap.add_argument("--limit-m", type=float, default=0.15)
    args = ap.parse_args(argv)
    result = sweep(args.session, args.variants.split(","), limit_m=args.limit_m)
    args.out.write_text(json.dumps(result, indent=1))
    for name, rows in result["variants"].items():
        print(name, [(r["pallet_id"], r["at_truth"], r.get("deeper_m"), r.get("shallower_m")) for r in rows])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
