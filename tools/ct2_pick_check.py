"""CT-2 V2 (docs/plans/2026-10-24-ct2-next-pick.md 2): the next-pick rule against the planner, on CPU.

For each session file: every order of min(pallets, slots) missions (line 3! x 3! = 36, random P(5,3) x 3! = 360) is
planned and measured mission by mission as session_plan_check does, in the state its earlier missions leave under
ideal deliveries -- each distinct (state, pair) once, in parallel, and without check_session's "prior_failed" skip, so
every step of every order is judged. In every state the rule's open pairs (next_pick.recommend(open_pairs=True)) are
set against the planner: criterion 4 is that no pair the rule calls open fails at one of the planner's fixed straights.
The rule's own sequence (recommend, deliver ideally, again) is then read off the same table.

    python tools/ct2_pick_check.py SESSION.json... --out RESULT.json [--jobs N]
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from multiprocessing import Pool
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for p_ in (ROOT / "src", ROOT / "tools", ROOT / "sim/isaac", ROOT):
    if str(p_) not in sys.path:
        sys.path.insert(0, str(p_))

from session_plan_check import (  # noqa: E402
    CLEARANCE_M, fresh_targets, ideal_delivery, plan_mission, session_context)

from forklift_core.mission.next_pick import PickSnapshot, recommend  # noqa: E402
from forklift_core.planning.pallet_mission import (  # noqa: E402
    PalletSite, TransportScenario, plan_observation_leg, plan_transport, site_poses)
from forklift_core.safety.pallet_permit import SESSION_PRIOR_GROW_M  # noqa: E402

FIXED_STRAIGHT = ("insert:", "extract:", "withdraw:")
_CTX: dict = {}


def straight_failure(status: str | None) -> bool:
    """A planner status that names one of the five fixed straights (the rule's own judgement)."""
    return bool(status) and (status.startswith(FIXED_STRAIGHT) or status.endswith("straight_collision"))


def state_of(ctx, moves):
    targets, slots = fresh_targets(ctx), {}
    for pid, did in moves:
        ideal_delivery(targets, slots, ctx["session"], pid, did)
    return targets, slots


def snapshot(ctx, targets) -> PickSnapshot:
    s = ctx["session"]
    home = (float(s.start_rear.x_m), float(s.start_rear.y_m), float(s.start_rear.yaw_rad))
    scene = TransportScenario(seed=0, start_rear=s.start_rear, pickup=PalletSite(0.0, 0.0, 0.0),
                              destination=PalletSite(0.0, 0.0, 0.0), props=tuple(s.static_props), bounds=s.bounds)
    return PickSnapshot(targets, home, home, scene, ctx["geometry"], CLEARANCE_M, ctx["nominal"], ctx["shape"],
                        SESSION_PRIOR_GROW_M)


def _init(paths):
    for path in paths:
        _CTX[path] = session_context(Path(path))


def _plan(task):
    path, moves, pid, did = task
    ctx = _CTX[path]
    targets, slots = state_of(ctx, moves)
    entry = plan_mission(ctx["session"], ctx["pallet"], ctx["geometry"], ctx["config"], targets, slots, pid, did,
                         nominal=ctx["nominal"], shape=ctx["shape"], prior_grow_m=SESSION_PRIOR_GROW_M,
                         observe_planner=plan_observation_leg, transport_planner=plan_transport)
    entry.pop("stages", None)  # the per-outline detail stays in session_plan_check's own runs
    return (path, tuple(sorted(moves)), pid, did), entry


def estimate_m(ctx, order) -> float:
    """next_pick's straight-line estimate of a whole order (home = the session start, the truck there too)."""
    s, g = ctx["session"], ctx["geometry"]
    home = (float(s.start_rear.x_m), float(s.start_rear.y_m))
    total = 0
    for pid, did in order:
        own, dest = s.pallet_by_id(pid), s.destination(did)
        nom = ctx["nominal"].get(pid, (own.nominal.x_m, own.nominal.y_m, own.nominal.yaw_rad))
        a, b = site_poses(PalletSite(*nom), g), site_poses(PalletSite(dest.x_m, dest.y_m, dest.yaw_rad), g)
        legs = (math.hypot(home[0] - a["prealign"].x_m, home[1] - a["prealign"].y_m)
                + math.hypot(a["extracted"].x_m - b["predelivery"].x_m, a["extracted"].y_m - b["predelivery"].y_m)
                + math.hypot(b["withdrawn"].x_m - home[0], b["withdrawn"].y_m - home[1]))
        total += int(round(1000.0 * legs))
    return total / 1000.0


def spearman(xs, ys) -> float | None:
    if len(xs) < 3:
        return None

    def ranks(v):
        order = sorted(range(len(v)), key=lambda i: v[i])
        r = [0.0] * len(v)
        i = 0
        while i < len(order):
            j = i
            while j + 1 < len(order) and v[order[j + 1]] == v[order[i]]:
                j += 1
            for k in range(i, j + 1):
                r[order[k]] = (i + j) / 2.0
            i = j + 1
        return r

    rx, ry = ranks(xs), ranks(ys)
    mx, my = sum(rx) / len(rx), sum(ry) / len(ry)
    num = sum((a - mx) * (b - my) for a, b in zip(rx, ry))
    den = math.sqrt(sum((a - mx) ** 2 for a in rx) * sum((b - my) ** 2 for b in ry))
    return None if den == 0 else num / den


def run(paths, jobs: int) -> dict:
    ctxs = {p: session_context(Path(p)) for p in paths}
    orders, tasks = {}, {}
    for path, ctx in ctxs.items():
        s = ctx["session"]
        pallets, dests = [p.pallet_id for p in s.pallets], sorted(fresh_targets(ctx).destinations)
        n = min(len(pallets), len(dests))
        orders[path] = [tuple(zip(ps, ds)) for ps in itertools.permutations(pallets, n)
                        for ds in itertools.permutations(dests, n)]
        for order in orders[path]:
            for k, (pid, did) in enumerate(order):
                moves = tuple(sorted(order[:k]))
                tasks[(path, moves, pid, did)] = (path, moves, pid, did)
    with Pool(jobs, initializer=_init, initargs=(list(paths),)) as pool:
        planned = dict(pool.imap_unordered(_plan, list(tasks.values()), chunksize=1))
    out = {"sessions": []}
    for path, ctx in ctxs.items():
        s = ctx["session"]
        per = {"session": s.name, "file": str(path), "orders": len(orders[path]), "plans": 0}
        # the rule's open pairs in every state met by an order
        opened, agree = {}, {"open_pass": 0, "open_straight_fail": 0, "open_other_fail": 0, "closed_pass": 0,
                             "closed_fail": 0}
        straight_fails = []
        for (p, moves, pid, did), entry in planned.items():
            if p != path:
                continue
            per["plans"] += 1
            if moves not in opened:
                targets, _ = state_of(ctx, moves)
                opened[moves] = {tuple(x) for x in recommend(snapshot(ctx, targets), open_pairs=True)["open"]}
            passed = entry["outcome"] == "passed" and entry["submit_check"] is None
            if (pid, did) in opened[moves]:
                if passed:
                    agree["open_pass"] += 1
                elif straight_failure(entry.get("plan_status")):
                    agree["open_straight_fail"] += 1
                    straight_fails.append({"moves": [list(m) for m in moves], "pair": [pid, did],
                                           "status": entry.get("plan_status")})
                else:
                    agree["open_other_fail"] += 1
            else:
                agree["closed_pass" if passed else "closed_fail"] += 1
        per["agreement"] = agree
        per["open_straight_failures"] = straight_fails
        # every order: passes when all its missions pass the submit check, the plan and the truth gaps
        rows = []
        for order in orders[path]:
            entries = [planned[(path, tuple(sorted(order[:k])), pid, did)] for k, (pid, did) in enumerate(order)]
            ok = all(e["outcome"] == "passed" and e["submit_check"] is None for e in entries)
            rows.append({"order": [list(x) for x in order], "passed": ok,
                         "length_m": sum(e.get("length_m", 0.0) for e in entries) if ok else None,
                         "estimate_m": estimate_m(ctx, order),
                         "first_failure": next(({"index": k, "outcome": e["outcome"], "status": e.get("plan_status"),
                                                 "submit_check": e["submit_check"]}
                                                for k, e in enumerate(entries)
                                                if not (e["outcome"] == "passed" and e["submit_check"] is None)), None)})
        passing = [r for r in rows if r["passed"]]
        per["passing_orders"] = len(passing)
        per["shortest_passing_m"] = min((r["length_m"] for r in passing), default=None)
        per["rank_correlation_estimate_vs_length"] = spearman([r["estimate_m"] for r in passing],
                                                              [r["length_m"] for r in passing])
        # the rule's own sequence
        targets, slots, seq, picks = fresh_targets(ctx), {}, [], []
        while True:
            r = recommend(snapshot(ctx, targets))
            picks.append({"reason": r["reason"], "pick": r["pick"], "missions": r["missions"],
                          "estimate_total_m": r["estimate_total_m"]})
            if r["pick"] is None:
                break
            seq.append((r["pick"]["pallet_id"], r["pick"]["destination_id"]))
            ideal_delivery(targets, slots, s, *seq[-1])
        rule = next((x for x in rows if [tuple(o) for o in x["order"]] == seq), None)
        per["rule"] = {"sequence": [list(x) for x in seq], "picks": picks, "final_reason": picks[-1]["reason"],
                       "passed": None if rule is None else rule["passed"],
                       "length_m": None if rule is None else rule["length_m"],
                       "first_failure": None if rule is None else rule["first_failure"]}
        per["hand"] = {"sequence": [list(x) for x in s.sequence],
                       "same_as_rule": [tuple(x) for x in s.sequence] == seq}
        hand = next((x for x in rows if [tuple(o) for o in x["order"]] == [tuple(x) for x in s.sequence]), None)
        per["hand"].update({"passed": None if hand is None else hand["passed"],
                            "length_m": None if hand is None else hand["length_m"]})
        best = per["shortest_passing_m"]
        per["criteria"] = {
            "1_rule_order_passes": bool(per["rule"]["passed"]),
            "2_final_reason": per["rule"]["final_reason"],
            "3_rule_within_110pct": (per["rule"]["length_m"] is not None and best is not None
                                     and per["rule"]["length_m"] <= 1.10 * best),
            "3_ratio": None if per["rule"]["length_m"] is None or not best else per["rule"]["length_m"] / best,
            "4_no_open_pair_fails_a_fixed_straight": agree["open_straight_fail"] == 0,
        }
        per["orders_table"] = rows
        out["sessions"].append(per)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--jobs", type=int, default=4)
    args = ap.parse_args(argv)
    result = run(args.sessions, args.jobs)
    args.out.write_text(json.dumps(result, indent=1, default=float) + "\n")
    for per in result["sessions"]:
        print(per["session"], per["plans"], "plans", per["passing_orders"], "/", per["orders"], "orders pass;",
              "rule", per["rule"]["sequence"], per["rule"]["final_reason"], per["criteria"], per["agreement"],
              flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
