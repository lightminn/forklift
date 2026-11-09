"""The cancel-then-other-target repro batch's verdict (docs/plans/2026-11-09-l4c-repro-cancel-other-target.md 2).

    python tools/l4c_repro_verdict.py RUN_DIR --json OUT.json

RUN_DIR holds ``plan_l4r.json`` ({"pairs": [{"seed": k, "H1": name, "H0": name}], "planned_pairs": 24, ...}) and the
session folders ``s/<name>``. Per session: the required evidence, the arm's validity (H1: the STOP-cancel-recovery chain;
H0: the pose match and the relocalization), and over all its P-5:D-2 missions F_p / F_c (2.1). Per pair: b / c. Then the
one-sided exact McNemar, the conclusion row (2.3) robust to every assignment of the undecided sessions and the pairs not
run (a compressed enumeration over (b, c, H0 F_p seeds, H0 F_c seeds, any F)), the mechanical early stop and the
feasibility check (2.4)."""

from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import riib_v2_verdict as V  # noqa: E402

from forklift_core.live.spool import load_list  # noqa: E402

P4, P5 = ("P-4", "D-1"), ("P-5", "D-2")
D2_XY = (3.4, -8.469)  # random_m's D-2 (the scene file's destination)
ENTRY_M, RUN_MIN_S, SCAN_GAP_S, END_GAP_S = 3.0, 5.0, 0.15, 0.25
H0_TARGET_REAR = (-1.2691, 0.8528, 0.1706)  # 1.3: the confirmation run's median truth rear axle at P-5's submit
POSE_TOL_M, POSE_TOL_RAD = 0.02, 0.02
ALPHA = 0.05
CANDIDATE_WAIT_S = 3.0  # the scene replan's candidate re-check before a refusal
OUTCOMES = ("none", "p", "c", "pc")  # a session's possible F result: none, F_p only, F_c only, both


# --- the one-sided exact McNemar and the conclusion row (2.3) --------------------------------------------------------
def mcnemar_p(b: int, c: int) -> float:
    """P(X >= b) for X ~ Bin(b + c, 1/2): the one-sided exact McNemar for H1's excess."""
    n = b + c
    if n == 0:
        return 1.0
    return sum(math.comb(n, k) for k in range(b, n + 1)) / 2 ** n


def conclusion(b: int, c: int, h0_fp_seeds: int, h0_fc_seeds: int, any_f: bool) -> str:
    excess = mcnemar_p(b, c) <= ALPHA
    reproduced = h0_fp_seeds >= 2 or h0_fc_seeds >= 2
    if excess and reproduced:
        return "both"
    if excess:
        return "h1_excess"
    if reproduced:
        return "h0_same_type"
    if not any_f:
        return "not_reproduced"
    return "undecided_or_other"


def _flags(outcome: str) -> tuple[bool, bool]:
    return "p" in outcome, "c" in outcome


def _choices(o) -> tuple:
    """A session's possible outcomes: None (undecided) -> all of OUTCOMES, a string -> itself, a set -> its members."""
    if o is None:
        return OUTCOMES
    if isinstance(o, str):
        return (o,)
    return tuple(sorted(o))


def rows_possible(pairs: list, planned: int) -> set:
    """Every conclusion row reachable when each session takes each of its possible outcomes (an undecided one and every
    pair not run: all of OUTCOMES; a partly known one: its set). ``pairs``: [(h1, h0)]. Compressed: the state is (b, c,
    H0 F_p seeds capped at 2, H0 F_c seeds capped at 2, any F)."""
    states = {(0, 0, 0, 0, False)}
    full = list(pairs) + [(None, None)] * max(0, planned - len(pairs))
    for h1, h0 in full:
        nxt = set()
        for o1 in _choices(h1):
            for o0 in _choices(h0):
                f1, f0 = o1 != "none", o0 != "none"
                p0, c0 = _flags(o0)
                for b, c, hp, hc, af in states:
                    nxt.add((b + (f1 and not f0), c + (f0 and not f1), min(2, hp + p0), min(2, hc + c0),
                             af or f1 or f0))
        states = nxt
    return {conclusion(*s) for s in states}


def rows_brute(pairs: list, planned: int) -> set:
    """The same by full enumeration (tests only: 16 ** unknown pairs)."""
    full = list(pairs) + [(None, None)] * max(0, planned - len(pairs))
    slots = [(i, k) for i, pr in enumerate(full) for k in (0, 1) if not isinstance(pr[k], str)]
    out = set()
    for combo in itertools.product(*[_choices(full[i][k]) for i, k in slots]):
        filled = [list(pr) for pr in full]
        for (i, k), o in zip(slots, combo):
            filled[i][k] = o
        b = sum(1 for h1, h0 in filled if h1 != "none" and h0 == "none")
        c = sum(1 for h1, h0 in filled if h0 != "none" and h1 == "none")
        hp = sum(1 for _, h0 in filled if "p" in h0)
        hc = sum(1 for _, h0 in filled if "c" in h0)
        af = any(h1 != "none" or h0 != "none" for h1, h0 in filled)
        out.add(conclusion(b, c, hp, hc, af))
    return out


# --- F per mission (2.1) ---------------------------------------------------------------------------------------------
def occupied_run_at_end(scans: list, ended_s: float) -> dict:
    """The transport scans before ``ended_s``: the occupied run (gap <= SCAN_GAP_S, unknown breaks it) the last scan
    belongs to. ``status``: "run" (that scan occupied), "released" (the last scan not occupied), "no_scans"."""
    before = sorted((s for s in scans if s.get("phase") == "transport" and float(s["stamp_s"]) <= ended_s),
                    key=lambda s: float(s["stamp_s"]))
    if not before:
        return {"status": "no_scans"}
    last = before[-1]
    if last.get("blocked") != "occupied":
        return {"status": "released", "last_s": float(last["stamp_s"])}
    start = float(last["stamp_s"])
    for prev, cur in zip(reversed(before[:-1]), reversed(before[1:])):
        if prev.get("blocked") != "occupied" or float(cur["stamp_s"]) - float(prev["stamp_s"]) > SCAN_GAP_S:
            break
        start = float(prev["stamp_s"])
    return {"status": "run", "start_s": start, "last_s": float(last["stamp_s"]),
            "length_s": float(last["stamp_s"]) - start, "end_gap_s": ended_s - float(last["stamp_s"])}


def truth_at(track, t: float):
    """The truth rear axle at or just before ``t`` (track: [(t, x, y, yaw)] sorted), None without one."""
    best = None
    for row in track or ():
        if row[0] <= t:
            best = row
        else:
            break
    return best


def entry(xy, dest=D2_XY) -> bool:
    return math.hypot(float(xy[0]) - dest[0], float(xy[1]) - dest[1]) <= ENTRY_M


def f_p(mission: dict, scans: list, track) -> str:
    """"yes", "no" or "undecided" (2.1 F_p) for one mission that failed in transport. No transport scan within END_GAP_S
    of the end -- the observation stopped before it -- is undecided whatever the last scan was."""
    ended = float(mission["ended_s"])
    run = occupied_run_at_end(scans, ended)
    if run["status"] == "no_scans" or ended - float(run.get("last_s", -math.inf)) > END_GAP_S:
        return "undecided"
    if run["status"] != "run" or run["length_s"] < RUN_MIN_S:
        return "no"
    at = truth_at(track, run["last_s"])
    if at is None:
        return "undecided"
    return "yes" if entry(at[1:3]) else "no"


def f_c(state: dict) -> str:
    """"yes"/"no" (2.1 F_c): a cusp_pose scene replan refused after its candidate waits, blocked:occupied, at the
    entry by its truth rear, ending the mission."""
    for r in state.get("scene_replans") or []:  # an absent key in a valid record: no replan
        waits = r.get("permit_waits") or []
        last = waits[-1] if waits else {}
        final_why = (r.get("candidate") or {}).get("why", last.get("why"))
        waited = (float(last["until_s"]) - float(last["since_s"])) if last.get("since_s") is not None else 0.0
        if (r.get("trigger") == "cusp_pose" and r.get("outcome") == "refused" and r.get("mission_end")
                and last.get("why") == "blocked:occupied" and final_why == "blocked:occupied"
                and waited >= CANDIDATE_WAIT_S - 0.05
                and r.get("truth_rear") is not None and entry(r["truth_rear"][:2])):
            return "yes"
    return "no"


# --- a session (2.2) -------------------------------------------------------------------------------------------------
def _missions(sess: dict) -> tuple[list, list]:
    """The P-5:D-2 missions with their results, and what is missing: a mission folder without a readable result or
    records, or a mission the session rows name (an acked submit) without a folder."""
    out, missing = [], []
    run = sess.get("run")
    seen = set()
    for md in sorted(run.glob("missions/*")) if run is not None else []:
        seen.add(md.name)
        try:
            r = json.loads((md / "result.json").read_text()).get("result") or {}
        except (OSError, ValueError):
            missing.append(f"result:{md.name}")
            continue
        if not (md / "records.json").exists():
            missing.append(f"records:{md.name}")
        if (r.get("pallet_id"), r.get("destination_id")) == P5:
            out.append({"mission_id": md.name, **r})
    for row in sess.get("rows") or []:
        mid = row.get("mission_id")
        if mid is not None and mid not in seen:
            missing.append(f"mission_folder:{mid}")
    return out, missing


def _reached_transport(sess: dict, mission: dict) -> bool:
    """The mission entered transport: a transition to it within the mission's time (result.json transitions)."""
    t0, t1 = float(mission.get("started_s") or 0.0), float(mission.get("ended_s") or math.inf)
    return any(x.get("to") == "transport" and t0 <= float(x.get("time_s", -1.0)) <= t1
               for x in (sess.get("result") or {}).get("transitions") or [])


def _stage(sess: dict, mission_id: str):
    for row in sess.get("rows") or []:
        if row.get("mission_id") == mission_id:
            return row.get("stage")
    return None


def _required(sess: dict) -> list:
    """The missing required evidence (2.2): a result, a complete spool with the scans and the control rows, readable
    mission records."""
    missing = []
    run = sess.get("run")
    if run is None or sess.get("result") is None:
        return ["result"]
    try:
        manifest = json.loads((run / "spool" / "manifest.json").read_text())
    except (OSError, ValueError):
        return ["spool_manifest"]
    for name in ("obstacle.scans", "slam.control"):
        m = manifest.get(name) or {}
        if not m or m.get("dropped_items") or m.get("unsettled_items"):
            missing.append(f"spool:{name}")
    for mid, rec in (sess.get("records") or {}).items():
        if rec is None:
            missing.append(f"records:{mid}")
    return missing


def _track(sess: dict):
    try:
        rows = load_list(sess["run"] / "spool", "slam.control")
    except (KeyError, OSError, ValueError, TypeError):
        return None
    return [(float(r[0]), float(r[4]), float(r[5]), float(r[6])) for r in rows]


def h1_valid(sess: dict) -> dict:
    """The confirmation run's STOP validity, STOP hold, target and recovery chain up to the P-5 submit."""
    stop = V.stop_valid(sess, P4)
    hold = V.stop_hold(sess)
    rows = sess.get("rows") or []
    m1 = next((r for r in rows if (r.get("pallet_id"), r.get("destination_id")) == P4), None)
    results = V._mission_results(sess)
    p5_ok = any(results.get(r.get("mission_id")) == P5 for r in rows if (r.get("pallet_id"),
                                                                           r.get("destination_id")) == P5)
    p = sess.get("probe") or {}
    runs = p.get("missions_run") or []
    w = (runs[0] if runs else {}).get("stop_wait") or {}
    needs_recovery = "recovering" in (w.get("after_cancel_statuses") or []) or bool(w.get("recovery_resume"))
    steps = [("approach", V._label("stop_wait:approach")), ("stop", V._reply("stop")),
             ("stopped", V._label("stopped_for_wait")), ("rearm", V._reply("rearm")),
             ("cancel", V._reply("mission_cancel"))]
    if needs_recovery:
        steps.append(("recovery_resume", V._reply("mission_resume")))
    steps.append(("p5_submit", V._reply("mission_submit")))
    order_ok, _ = V.in_order(p.get("events") or [], steps)
    ok = (stop["pass"] and hold.get("held") is True and m1 is not None and m1.get("outcome") == "cancelled"
          and results.get(m1.get("mission_id")) == P4 and p5_ok and order_ok)
    return {"ok": ok, "stop": stop["pass"], "hold": hold.get("held"), "order": order_ok, "p5_target": p5_ok}


def h0_valid(sess: dict, track, p5: list) -> dict:
    """The pose match at the P-5 mission's start (truth rear vs the target) and the relocalization confirmed by the
    matcher (its estimate's error reported)."""
    rec = (sess.get("result") or {}).get("start_reloc") or {}
    confirmed = rec.get("status") == "chosen" and rec.get("source") == "matcher" and rec.get("verdict") is not None
    err = None
    if rec.get("verdict"):
        s_, t_ = rec["verdict"].get("slam_base"), rec["verdict"].get("truth_base")
        if s_ and t_:
            err = (math.hypot(s_[0] - t_[0], s_[1] - t_[1]),
                   abs(math.atan2(math.sin(s_[2] - t_[2]), math.cos(s_[2] - t_[2]))))
    first = min(p5, key=lambda m: float(m.get("started_s") or 0.0)) if p5 else None
    at = None if first is None else truth_at(track, float(first.get("started_s") or 0.0))
    pose_ok = None
    if at is not None:
        dx = math.hypot(at[1] - H0_TARGET_REAR[0], at[2] - H0_TARGET_REAR[1])
        dyaw = abs(math.atan2(math.sin(at[3] - H0_TARGET_REAR[2]), math.cos(at[3] - H0_TARGET_REAR[2])))
        pose_ok = dx <= POSE_TOL_M and dyaw <= POSE_TOL_RAD
    return {"ok": bool(confirmed and pose_ok), "relocalized": confirmed, "pose_ok": pose_ok,
            "reloc_error": err, "start_truth": None if at is None else list(at[1:])}


def session_outcome(sess: dict, arm: str) -> dict:
    """The session's F result over all its P-5:D-2 missions: any F -> its subtypes; else one done -> "none"; else
    undecided (None). Missing required evidence or an invalid arm -> undecided."""
    out = {"arm": arm}
    missing = _required(sess)
    out["missing"] = missing
    track = _track(sess)
    p5, missing_m = _missions(sess)
    missing += missing_m
    out["missing"] = missing
    out["p5"] = [{"mission_id": m["mission_id"], "outcome": m.get("outcome"), "reason": m.get("reason"),
                  "reached_transport": _reached_transport(sess, m)} for m in p5]
    valid = h1_valid(sess) if arm == "H1" else h0_valid(sess, track, p5)
    out["valid"] = valid
    if missing or not valid["ok"] or track is None:
        out["outcome"] = None
        return out
    try:
        scans = load_list(sess["run"] / "spool", "obstacle.scans")
    except (OSError, ValueError):
        out["outcome"], out["missing"] = None, missing + ["spool:obstacle.scans"]
        return out
    p_vals, c_vals = {False}, {False}  # what F_p / F_c can be over the session's missions (any mission F -> F)
    for m in p5:
        if m.get("outcome") != "failed" or _stage(sess, m["mission_id"]) != "transport":
            continue
        fp = f_p(m, scans, track)
        fc = f_c((sess.get("records") or {}).get(m["mission_id"]) or {})
        next(x for x in out["p5"] if x["mission_id"] == m["mission_id"]).update(f_p=fp, f_c=fc)
        if fp == "yes":
            p_vals = {True}
        elif fp == "undecided" and p_vals != {True}:
            p_vals = {False, True}
        if fc == "yes":
            c_vals = {True}
    possible = {("p" if p else "") + ("c" if c else "") or "none" for p in p_vals for c in c_vals}
    if possible == {"none"} and not any(m.get("outcome") == "done" for m in p5):
        out["outcome"] = None  # no F and no P-5 done: undecided (2.2)
    elif len(possible) == 1:
        out["outcome"] = possible.pop()
    else:
        out["outcome"] = sorted(possible)  # partly known (e.g. F_c yes, F_p undecided): kept for the enumeration
    return out


# --- the batch (2.3, 2.4) --------------------------------------------------------------------------------------------
def batch_verdict(pairs: list, planned: int) -> dict:
    """``pairs``: [(h1_outcome or None, h0_outcome or None)] for the pairs run so far, in order."""
    rows = rows_possible(pairs, planned)
    decided = [(h1, h0) for h1, h0 in pairs if isinstance(h1, str) and isinstance(h0, str)]
    b = sum(1 for h1, h0 in decided if h1 != "none" and h0 == "none")
    c = sum(1 for h1, h0 in decided if h0 != "none" and h1 == "none")
    return {"rows_possible": sorted(rows), "settled": len(rows) == 1,
            "conclusion": next(iter(rows)) if len(rows) == 1 else "undecided", "b_decided": b, "c_decided": c,
            "mcnemar_p_decided": mcnemar_p(b, c), "pairs_run": len(pairs), "planned_pairs": planned}


def feasibility(chunk: list) -> dict:
    """After the first chunk: valid H1 (STOP valid and P-5 reaching transport) >= 4 of 6 and P-5 reaching transport
    >= 8 of 12 sessions, else stop (2.4). ``chunk``: [(h1_session_result, h0_session_result)]."""
    def reached(r):
        return any(m.get("reached_transport") for m in r.get("p5") or [])

    valid_h1 = sum(1 for h1, _ in chunk if h1.get("valid", {}).get("stop") and reached(h1))
    reach = sum(1 for pr in chunk for r in pr if reached(r))
    return {"valid_h1": valid_h1, "reached": reach, "ok": valid_h1 >= 4 and reach >= 8}


PLANNED_PAIRS, SEEDS = 24, tuple(range(2, 26))


def validate_plan(plan: dict) -> list:
    """The registered shape (1.4): 24 planned pairs on seeds 2-25 in order, each seed once, every attempt folder name
    used once. An arm's attempts: [{"name": .., "base_failure": bool}] in run order."""
    errors = []
    if int(plan.get("planned_pairs", -1)) != PLANNED_PAIRS:
        errors.append("planned_pairs")
    seeds = [int(p_["seed"]) for p_ in plan.get("pairs", [])]
    if seeds != list(SEEDS[:len(seeds)]):
        errors.append("seeds")
    names = [a["name"] for p_ in plan.get("pairs", []) for arm in ("H1", "H0") for a in p_[arm]]
    if len(names) != len(set(names)):
        errors.append("duplicate_session")
    for p_ in plan.get("pairs", []):
        for arm in ("H1", "H0"):
            if not 1 <= len(p_[arm]) <= 2:
                errors.append(f"attempts:{p_['seed']}:{arm}")
    return errors


def args_mismatch(args: dict, seed: int, arm: str) -> list:
    """The session's recorded arguments (args.txt) against its pair: the seed, and the arm's construction -- H1 the
    cycle without a start pose, H0 the queue with the registered START_POSE."""
    out = []
    if str(args.get("SEED")) != str(seed):
        out.append("args:seed")
    want = ("cycle", False) if arm == "H1" else ("queue", True)
    if args.get("SCENARIO") != want[0] or bool(args.get("START_POSE")) != want[1]:
        out.append("args:arm")
    return out


def chosen_attempt(attempts: list):
    """1.4: the first attempt without a base failure, or None (both failed at the base: undecided)."""
    return next((a["name"] for a in attempts if not a.get("base_failure")), None)


def evaluate(run_dir: Path) -> dict:
    plan = json.loads((run_dir / "plan_l4r.json").read_text())
    errors = validate_plan(plan)
    if errors:
        return {"plan_errors": errors, "conclusion": "invalid_plan"}
    sessions, pairs = {}, []
    for pr in plan["pairs"]:
        res = {}
        for arm in ("H1", "H0"):
            name = chosen_attempt(pr[arm])
            d = None if name is None else run_dir / "s" / name
            if d is None or not d.exists():
                res[arm] = {"arm": arm, "outcome": None, "missing": ["no_attempt" if name is None else "not_run"]}
            else:
                sess_ = V.load_session(d)
                res[arm] = session_outcome(sess_, arm)
                mismatch = args_mismatch(V._session_args(sess_), int(pr["seed"]), arm)
                if mismatch:
                    res[arm]["missing"] = res[arm].get("missing", []) + mismatch
                    res[arm]["outcome"] = None
            res[arm]["attempt"] = name
            sessions[f"{pr['seed']}:{arm}"] = res[arm]
        pairs.append((res["H1"]["outcome"], res["H0"]["outcome"]))
    out = {"sessions": sessions, **batch_verdict(pairs, int(plan["planned_pairs"]))}
    if len(plan["pairs"]) >= 6:
        out["feasibility"] = feasibility([(sessions[f"{p_['seed']}:H1"], sessions[f"{p_['seed']}:H0"])
                                          for p_ in plan["pairs"][:6]])
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--json", type=Path, required=True)
    a = ap.parse_args(argv)
    a.json.write_text(json.dumps(evaluate(a.run_dir), indent=1, default=str) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
