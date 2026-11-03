"""Near-leg cusp miss diagnosis (docs/plans/2026-11-03-near-leg-cusp-miss-diagnosis.md, edition 5): from a batch's
run/result.json records only, every observe-phase cusp stall classified -- near-confirmed, far-confirmed or
unconfirmed -- with the existing K3's reason for not releasing it (the last scan), the replan it was joined to, and the
plan's robust choice between a cause diagnosis and the C1 family.

    python3 tools/near_leg_cusp_miss.py OUT.json BATCH_DIR [BATCH_DIR ...]

Standard library only (ws1's plain python3 runs it)."""

from __future__ import annotations

import glob
import json
import math
import os
import sys

TICK_S = 1.0 / 120.0
EPS_S = 1e-6  # the raw times are compared, not rounded ones (Codex plan review 5 P3)
RECUR_M = 0.30  # Q1: a stall after a replan within this of the previous stall's last-scan rear


def last_scan_reason(stall: dict) -> str:
    """Plan 2: why the existing K3 did not release, from the last scan (the top-level errors are the latch's)."""
    scans = stall.get("scans") or []
    if not scans:
        return "k3_absent"
    last = scans[-1]
    if not last.get("bounds_ok"):
        return "bounds"
    if "converged" not in last:
        return "k3_absent"  # within the bounds but no rollout recorded: the reason lies outside the scan
    if not last.get("converged"):
        return "converge"
    if not last.get("sweep_ok"):
        return "sweep"
    return "sensor_or_other"  # the pose passed; the sensor check (not recorded as a flag) or the release refused


def closed_event(stall: dict) -> dict | None:
    return next((e for e in stall.get("events") or [] if e.get("event") == "closed"), None)


def join_replan(stall: dict, replans: list, switches: list) -> tuple[dict | None, str]:
    """Plan 2 (edition 5): the replan a stall's own closed key_changed event marks -- not later than the close (one
    tick), no leg switch within a tick of it, the candidate unique. Returns (replan, how)."""
    closed = closed_event(stall)
    if closed is None:
        return None, "open"
    if closed.get("why") != "key_changed":
        return None, "no_replan"
    tc = float(closed["time_s"])
    if any(abs(float(w["time_s"]) - tc) <= TICK_S + EPS_S for w in switches):
        return None, "leg_switch"
    cands = [r for r in replans if tc - TICK_S - EPS_S <= float(r["time_s"]) <= tc + EPS_S]
    if len(cands) != 1:
        return None, "unlinked" if not cands else "ambiguous"
    return cands[0], "linked"


def excluded(s: dict) -> str | None:
    """Plan 2: runs whose stalls are all unconfirmed -- a realign or retry back to the far observation, a bottom-deck
    leg, an observe-phase scene candidate."""
    if s.get("near_capture_refused") or s.get("near_capture_history"):
        return "realign_or_retry"
    if any(k.startswith("bottom_deck") and s[k] for k in s):
        return "bottom_deck"
    if any(r.get("phase") == "observe" for r in s.get("scene_replans") or []):
        return "scene_candidate"
    return None


def analyse(s: dict, name: str) -> dict:
    armed = ((s.get("near_end_injections") or {}).get("windows") or {}).get("injection")
    armed_s = None if not armed else float(armed[0])
    replans = [r for r in s.get("observe_replans") or []]
    switches = [w for w in s.get("leg_switches") or [] if w.get("phase") == "observe"
                and w.get("via") in ("arrival", "release")]
    near_stands = [e for e in s.get("observe_end_stands") or [] if e.get("kind") == "near"]
    why_out = excluded(s)
    near_serials = {e.get("leg_serial") for e in near_stands}
    if (s.get("near_capture") or {}).get("waypoint") and why_out is None:
        near_serials.add(1)  # without realign, bottom-deck or scene traces only the near leg's install raises it
    near_events = [float(r["time_s"]) for r in replans if r.get("leg_serial") in near_serials] + [
        float(e["time_s"]) for e in near_stands]
    first_near = min(near_events) if near_events else None
    rows = []
    for st in s.get("cusp_stalls") or []:
        if st.get("phase") != "observe":
            continue
        t0 = float(st["time_s"])
        if armed_s is not None and t0 >= armed_s:
            continue  # after the injection's arming
        replan, how = join_replan(st, replans, switches)
        if why_out is not None:
            role, basis = "unconfirmed", why_out
        elif replan is not None and replan.get("leg_serial") in near_serials:
            role, basis = "near", "joined_near_replan"
        elif replan is not None and replan.get("leg_serial") == 0:
            role, basis = "far", "joined_far_replan"
        elif first_near is not None and t0 > first_near:
            role, basis = "near", "after_first_near_event"
        else:
            role, basis = "unconfirmed", "no_near_evidence"
        scans = st.get("scans") or []
        last = scans[-1] if scans else {}
        rows.append({
            "run": name, "time_s": t0, "role": role, "basis": basis, "disposition": st.get("disposition"),
            "reason": "released" if st.get("disposition") == "released" else last_scan_reason(st),
            "join": how, "replan_time_s": None if replan is None else float(replan["time_s"]),
            "replan_serial": None if replan is None else replan.get("leg_serial"),
            "replan_trigger": None if replan is None else (replan.get("trigger") or "observe_miss"),
            "last_rear": last.get("rear"), "last_position_error_m": last.get("position_error_m"),
            "last_yaw_error_rad": last.get("yaw_error_rad"), "last_bounds_ok": last.get("bounds_ok"),
            "last_prefix_failure": (last.get("prefix") or {}).get("failure"),
        })
    linked_times = {r["replan_time_s"] for r in rows if r["replan_time_s"] is not None}
    unlinked_miss = [float(r["time_s"]) for r in replans if r.get("leg_serial") in near_serials
                     and not r.get("trigger") and float(r["time_s"]) not in linked_times
                     and (armed_s is None or float(r["time_s"]) < armed_s)]
    near_miss = [r for r in replans if r.get("leg_serial") in near_serials and not r.get("trigger")]
    return {"run": name, "success": bool(s.get("success")), "failure": (s.get("failure_reason") or "")[:80],
            "excluded": why_out, "near_serials": sorted(x for x in near_serials if x is not None),
            "near_misses": len(near_miss), "unlinked_near_miss_replans": unlinked_miss, "stalls": rows}


def recurrences(run: dict) -> list:
    """Q1: near stalls after a joined near replan (before the next), and whether within RECUR_M of the previous
    stall's last-scan rear."""
    near = [r for r in run["stalls"] if r["role"] == "near"]
    out = []
    for prev, nxt in zip(near, near[1:]):
        if prev["replan_time_s"] is None or nxt["time_s"] <= prev["replan_time_s"]:
            continue
        d = (math.dist(prev["last_rear"][:2], nxt["last_rear"][:2])
             if prev["last_rear"] and nxt["last_rear"] else None)
        out.append({"run": run["run"], "after_s": prev["replan_time_s"], "time_s": nxt["time_s"],
                    "distance_m": d, "within": d is not None and d <= RECUR_M})
    return out


def choose(population: list, on_s2: dict | None) -> dict:
    """Plan 4: each choice must hold with every unconfirmed stall against it; on s2 vetoes."""
    judged = [r for r in population if r["reason"] != "released" and r["role"] != "far"]  # near + unconfirmed
    confirmed = [r for r in judged if r["role"] == "near" and r["reason"] != "k3_absent"]
    unconfirmed = [r for r in judged if r not in confirmed]
    n = len(confirmed) + len(unconfirmed)
    counts = {}
    for r in confirmed:
        counts[r["reason"]] = counts.get(r["reason"], 0) + 1
    out = {"n": n, "confirmed": len(confirmed), "unconfirmed": len(unconfirmed), "reasons": counts}
    choice = None
    if n and 3 * counts.get("bounds", 0) >= 2 * n:
        choice = ("cause_diagnosis", "bounds")
    else:
        for reason in ("converge", "sweep", "sensor_or_other"):
            if n and 3 * counts.get(reason, 0) >= 2 * n:
                choice = ("C1", reason)
    veto = None
    if choice is not None:
        if on_s2 is None or on_s2["role"] != "near" or on_s2["reason"] in ("k3_absent", "released"):
            veto = "on_s2_unconfirmed"
        elif on_s2["reason"] != choice[1]:
            veto = f"on_s2_reason_{on_s2['reason']}"
    out.update(choice=None if choice is None else list(choice), veto=veto,
               decision=("report_only" if choice is None or veto else choice[0]))
    return out


def main(argv: list) -> None:
    out_path, batches = argv[1], argv[2:]
    runs = []
    for b in batches:
        for f in sorted(glob.glob(os.path.join(b, "*random_1*", "run", "result.json"))):
            name = f.split(os.sep)[-3]
            runs.append(analyse(json.load(open(f)), f"{os.path.basename(os.path.normpath(b))}/{name}"))
    stalls = [r for run in runs for r in run["stalls"]]
    on_s2_run = next((r for r in runs if r["run"].endswith("s1c_on_random_1_s2")), None)
    on_s2 = None
    if on_s2_run is not None:
        near_ = [r for r in on_s2_run["stalls"] if r["role"] == "near"]
        on_s2 = near_[-1] if near_ else (on_s2_run["stalls"][-1] if on_s2_run["stalls"] else None)
    spent_ok = [r for r in runs if r["near_misses"] >= 2 and r["success"]]
    q3 = []
    for r in spent_ok + ([on_s2_run] if on_s2_run else []):
        miss_t = sorted(x["replan_time_s"] for x in r["stalls"] if x["replan_trigger"] == "observe_miss"
                        and x["role"] == "near")
        after = [x for x in r["stalls"] if miss_t and len(miss_t) >= 2 and x["time_s"] > miss_t[1]]
        q3.append({"run": r["run"], "success": r["success"], "stalls_after_second_miss":
                   [(x["role"], x["reason"], x["disposition"]) for x in after]})
    summary = {
        "runs": len(runs), "excluded": {k: sum(r["excluded"] == k for r in runs)
                                        for k in ("realign_or_retry", "bottom_deck", "scene_candidate")},
        "stalls": len(stalls),
        "by_role": {k: sum(r["role"] == k for r in stalls) for k in ("near", "far", "unconfirmed")},
        "near_reasons": {}, "joins": {},
        "q1": [x for run in runs for x in recurrences(run)],
        "q3": q3, "on_s2_last": on_s2,
        "unlinked_near_miss_replans": sum(len(r["unlinked_near_miss_replans"]) for r in runs),
    }
    for r in stalls:
        summary["joins"][r["join"]] = summary["joins"].get(r["join"], 0) + 1
        if r["role"] == "near":
            summary["near_reasons"][r["reason"]] = summary["near_reasons"].get(r["reason"], 0) + 1
    summary["choice"] = choose(stalls, on_s2)
    json.dump({"summary": summary, "runs": runs}, open(out_path, "w"), indent=1)
    print(json.dumps({k: v for k, v in summary.items() if k not in ("q1", "q3")}, indent=1))
    print("q1 recurrences", len(summary["q1"]), "within", sum(x["within"] for x in summary["q1"]))
    for x in q3:
        print("q3", x)


if __name__ == "__main__":
    main(sys.argv)
