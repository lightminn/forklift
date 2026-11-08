"""The v2 main batch's verdict tables (docs/plans/2026-11-08-live-riib-defaults-v2.md 3.3): pace over the integrated
A/B pairs on each pair's common interval (pooled F, R, the arm-wide longest ok=false run, all stalls with the C2 replan
steps apart), pair eligibility and exclusions, the C2 2.0 s cap over every valid session, safety and record
completeness, 4's and 5's minimum valid firing, the lifecycles.

    python tools/riib_v2_verdict.py OUT --json VERDICT.json

OUT holds plan_main.json and s/<run>. A description of the registered rules' inputs and their outcomes; the promotion
recommendation stays with the reader and the lead."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "isaac"))

import ct_survey  # noqa: E402
from riib_v2_baseline import COMMON_MIN_S, interval_stats, read_log  # noqa: E402
from session_pallets import RESTING_M, RESTING_RAD  # noqa: E402  (no Isaac imports)

from forklift_core.live.spool import load_list  # noqa: E402

STALL_S, C2_CAP_S = 0.25, 2.0
F_RATIO, R_ADD, LONG_MUL, LONG_ADD_S, STALL_ADD = 0.97, 0.01, 1.2, 2.0, 2  # the lead's policy budgets (3.3)
MAX_DROPPED_PAIRS = 2
SAFETY_REASONS = ("Pallet dropped during transport", "Pallet slipped off forks")


# --- pace pieces ----------------------------------------------------------------------------------------------------
def longest_false(steps: dict, a: float, b: float) -> float:
    """The longest run of pacing_ok false inside [a, b] by the registered end rules (wall seconds)."""
    keys = sorted(k for k, v in steps.items() if a <= v[0] <= b)
    best, start, last_false = 0.0, None, None
    for i, k in enumerate(keys):
        mono, _, ok = steps[k]
        if start is not None and keys[i - 1] != k - 1:  # a missing step ends the run at the last observed false step
            best = max(best, last_false - start)
            start = None
        if not ok:
            start = mono if start is None else start
            last_false = mono
        elif start is not None:  # the next step is a true step inside the window
            best = max(best, mono - start)
            start = None
    if start is not None:  # false to the window's end: k1's mono
        best = max(best, steps[keys[-1]][0] - start)
    return best


def stall_rows(trace: dict, a: float, b: float, exclude: set) -> dict:
    """Stalls (> 0.25 s, S[k] - S[k-1]) with S[k] in [a, b], C2 replan steps apart, split by the previous row's flags."""
    S, step, flags, names = trace["S"], trace["live_step"], trace["flags"], trace["flag_names"]
    out = {"total": 0, "rendered": 0, "capture": 0, "other": 0, "c2_excluded": 0, "gaps": []}
    for i in range(1, len(S)):
        if not (a <= S[i] <= b) or math.isnan(S[i]) or math.isnan(S[i - 1]):
            continue
        g = S[i] - S[i - 1]
        if g <= STALL_S:
            continue
        if int(step[i]) in exclude:
            out["c2_excluded"] += 1
            continue
        prev = 0 if math.isnan(flags[i - 1]) else int(flags[i - 1])
        kind = ("capture" if prev >> names.index("capture") & 1 else
                "rendered" if prev >> names.index("rendered") & 1 else "other")
        out["total"] += 1
        out[kind] += 1
        out["gaps"].append(round(float(g), 4))
    return out


def trace_missing(trace: dict, a: float, b: float) -> bool:
    """A live_step break among the trace rows with S in [a, b]."""
    sel = [int(s) for s, x in zip(trace["live_step"], trace["S"], strict=True) if not math.isnan(x) and a <= x <= b]
    return any(n != p + 1 for p, n in zip(sel, sel[1:], strict=False))


def arm_rules(a: dict, b: dict) -> dict:
    """The four pooled comparisons (3.3) from per-arm totals {sim, wall, false_wall, longest, stalls}."""
    fa, fb = a["sim"] / a["wall"], b["sim"] / b["wall"]
    ra, rb = a["false_wall"] / a["valid_wall"], b["false_wall"] / b["valid_wall"]
    out = {"F": {"A": fa, "B": fb, "pass": fb >= F_RATIO * fa},
           "R": {"A": ra, "B": rb, "pass": rb <= ra + R_ADD},
           "longest": {"A": a["longest"], "B": b["longest"], "pass": b["longest"] <= LONG_MUL * a["longest"] + LONG_ADD_S},
           "stalls": {"A": a["stalls"], "B": b["stalls"], "pass": b["stalls"] <= a["stalls"] + STALL_ADD}}
    out["pass"] = all(v["pass"] for v in out.values())
    return out


# --- records --------------------------------------------------------------------------------------------------------
def load_session(d: Path) -> dict:
    out = {"name": d.name, "dir": d}
    runs = sorted(d.glob("live/epoch_*/run"))
    out["run"] = runs[0] if len(runs) == 1 else None
    out["probe"] = ct_survey._json(d / "probe.json") or {}
    if out["run"] is None:
        return out
    run = out["run"]
    try:
        out["result"] = json.loads((run / "result.json").read_text())
    except (OSError, ValueError):
        out["result"] = None
    rows = ct_survey.session_rows(d, 3)["rows"]
    out["rows"] = rows
    entries, complete, err = read_log(run)
    out["log_complete"], out["log_error"], out["entries"] = complete, err, entries
    out["steps"] = {int(e["step"]): (float(e["mono"]), float(e["tick_t"]), bool(e["pacing_ok"]))
                    for e in entries if "pacing_ok" in e and "step" in e}
    submits = [float(e["mono"]) for e in entries if e.get("t") == "mission_submit" and "mono" in e]
    ends = [float(e["mono"]) for e in entries if "mission_end" in e and "mono" in e]
    out["window"] = [min(submits), max(ends)] if submits and ends else None
    try:
        z = np.load(run / "tick_trace.npz")
        cols = list(z["columns"])
        data = z["data"]
        out["trace"] = {"S": data[:, cols.index("S")], "live_step": data[:, cols.index("live_step")],
                        "flags": data[:, cols.index("flags")], "flag_names": [str(f) for f in z["flags"]]}
    except (OSError, KeyError, ValueError):
        out["trace"] = None
    out["records"] = {}
    for md in sorted(run.glob("missions/*")):
        try:
            out["records"][md.name] = json.loads((md / "records.json").read_text()).get("state") or {}
        except (OSError, ValueError):
            out["records"][md.name] = None
    return out


def c2_steps(sess: dict) -> tuple[set, list]:
    """The C2 planning-call steps (kind near, planning_wall_s present) mapped by tick_t equality; unmapped listed."""
    by_tick = {}
    for k, (_, t, _) in sess.get("steps", {}).items():
        by_tick.setdefault(round(t, 9), k)
    found, unmapped = set(), []
    for mid, st in (sess.get("records") or {}).items():
        for r in (st or {}).get("observe_end_stands") or []:
            if r.get("kind") == "near" and r.get("planning_wall_s") is not None:
                k = by_tick.get(round(float(r["time_s"]), 9))
                (found.add(k) if k is not None else unmapped.append({"mission": mid, "time_s": r["time_s"]}))
    return found, unmapped


def c2_gaps(sess: dict, ks: set) -> list:
    tr = sess.get("trace")
    if tr is None:
        return [{"step": k, "gap_s": None} for k in ks]
    idx = {int(s): i for i, s in enumerate(tr["live_step"]) if not math.isnan(s)}
    out = []
    for k in sorted(ks):
        i = idx.get(k)
        g = None if i is None or i == 0 else float(tr["S"][i] - tr["S"][i - 1])
        out.append({"step": k, "gap_s": g, "over_cap": None if g is None else g > C2_CAP_S})
    return out


def eligible(sess: dict) -> bool:
    rows = sess.get("rows") or []
    return len(rows) == 3 and all(r.get("outcome") == "done" for r in rows)


def safety(sess: dict) -> dict:
    res = sess.get("result")
    if res is None:
        return {"undecidable": "no run result.json"}
    absent = [k for k in ("forbidden_pocket_contacts", "session_neighbour_gap", "session_rest_largest", "mission_results",
                          "obstacle_layer") if k not in res]
    unparsed = [m for m, st in (sess.get("records") or {}).items() if st is None]
    if absent or unparsed or not (sess["run"] / "spool" / "manifest.json").exists():
        return {"undecidable": {"absent": absent, "unparsed_records": unparsed,
                                "spool_manifest": (sess["run"] / "spool" / "manifest.json").exists()}}
    gaps = res.get("session_neighbour_gap") or {}
    rest = res.get("session_rest_largest") or {}
    reasons = [r.get("reason") for r in sess.get("rows") or [] if r.get("reason")]
    classes = [ct_survey.classify(x) for x in reasons]
    manifest = ct_survey._json(sess["run"] / "spool" / "manifest.json") or {}
    unsettled = [m for m in res.get("mission_results") or [] if m.get("record_error") or m.get("unsettled")]
    missing_files = [md.name for md in sess["run"].glob("missions/*")
                     if not ((md / "result.json").exists() and (md / "records.json").exists())]
    out = {"forbidden_run": len(res.get("forbidden_pocket_contacts") or []),
           "forbidden_missions": sum(len((st or {}).get("forbidden_pocket_contacts") or [])
                                     for st in (sess.get("records") or {}).values()),
           "neighbour_overlap": sorted(k for k, g in gaps.items() if float(g["m"]) <= 0.0),
           "min_gap_m": min((float(g["m"]) for g in gaps.values()), default=None),
           "rest_past_bound": sorted(k for k, (m, r) in rest.items() if m > RESTING_M or r > RESTING_RAD),
           "unpermitted_dir_entries": (res.get("obstacle_layer") or {}).get("unpermitted_dir_entries"),
           "scene_damaged": res.get("scene_damaged"),
           "reasons": reasons,
           "safety_reasons": [x for x, c in zip(reasons, classes, strict=True)
                              if c == "load_safety" or any(x.startswith(s) for s in SAFETY_REASONS)],
           "other_reasons": [x for x, c in zip(reasons, classes, strict=True) if c == "other"],
           "spool_complete": manifest.get("complete"),
           "spool_dropped": {k: v.get("dropped_items") for k, v in manifest.items()
                             if isinstance(v, dict) and v.get("dropped_items")},
           "spool_unsettled": {k: v.get("unsettled_items") for k, v in manifest.items()
                               if isinstance(v, dict) and v.get("unsettled_items")},
           "results_unsettled": unsettled, "mission_files_missing": missing_files}
    out["pass"] = (out["forbidden_run"] == 0 and out["forbidden_missions"] == 0 and not out["neighbour_overlap"]
                   and not out["rest_past_bound"] and out["unpermitted_dir_entries"] == 0 and not out["scene_damaged"]
                   and not out["safety_reasons"] and out["spool_complete"] is True and not out["spool_dropped"]
                   and not out["spool_unsettled"] and not unsettled and not missing_files)
    return out


def backoff_firing(sess: dict) -> list:
    """4 per mission: installed (release_replan adopted), the installed path's prefix, the truth reverse >= 0.8 L,
    complete."""
    res = sess.get("result") or {}
    try:
        control = np.asarray(load_list(sess["run"] / "spool", "slam.control"), dtype=float).reshape(-1, 7)
    except (OSError, ValueError, KeyError):
        control = None
    trans = res.get("transitions") or []
    out = []
    for mid, st in sorted((sess.get("records") or {}).items()):
        st = st or {}
        rows = [r for r in sess.get("rows") or [] if r.get("mission_id") == mid]
        e = {"mission": mid, "installed": any(b.get("where") == "release_replan" and b.get("adopted")
                                              for b in st.get("return_start_backoffs") or []),
             "boxed_trigger": any(b.get("trigger") == "boxed" for b in st.get("return_start_backoffs") or [])}
        rh = (st.get("paths") or {}).get("return_home") or {}
        sb = rh.get("start_backoff") or {}
        L = sb.get("length_m")
        e["path_prefix"] = bool(sb.get("status") == "prefix" and L and (rh.get("directions") or [0])[0] == -1)
        e["complete"] = bool(rows and rows[0].get("outcome") == "done")
        e["truth_reverse"] = None
        r0 = rows[0] if rows else {}
        t_lo, t_hi = r0.get("started_s"), r0.get("ended_s")
        t0 = next((float(x["time_s"]) for x in trans if x.get("to") == "return_home"
                   and t_lo is not None and t_lo <= float(x["time_s"]) <= (t_hi or math.inf)), None)
        if e["path_prefix"] and control is not None and t0 is not None:
            sel = control[control[:, 0] >= t0]
            if len(sel):
                x0, y0, yaw0 = sel[0, 4], sel[0, 5], sel[0, 6]
                dist, along_min = 0.0, 0.0
                for j in range(1, len(sel)):
                    dist += math.hypot(sel[j, 4] - sel[j - 1, 4], sel[j, 5] - sel[j - 1, 5])
                    along = (sel[j, 4] - x0) * math.cos(yaw0) + (sel[j, 5] - y0) * math.sin(yaw0)
                    along_min = min(along_min, along)
                    if dist >= L or sel[j, 0] - t0 > 10.0:
                        break
                e["truth_reverse"] = along_min <= -0.8 * L
                e["truth_along_min_m"], e["L_m"] = along_min, L
        e["firing"] = bool(e["installed"] and e["path_prefix"] and e["truth_reverse"] and e["complete"])
        out.append(e)
    return out


def c2_firing(sess: dict) -> list:
    out = []
    for mid, st in sorted((sess.get("records") or {}).items()):
        st = st or {}
        rows = [r for r in sess.get("rows") or [] if r.get("mission_id") == mid]
        inst = [r for r in st.get("observe_end_stands") or [] if r.get("kind") == "near" and r.get("outcome") == "installed"
                and r.get("planning_wall_s") is not None]
        near = (st.get("near_capture") or {}).get("status")
        done = bool(rows and rows[0].get("outcome") == "done")
        out.append({"mission": mid, "installed": len(inst), "near_capture": near, "complete": done,
                    "firing": bool(inst and near == "done" and done)})
    return out


def _epoch_exit(sess: dict):
    sup = sess["dir"] / "live" / "supervisor.jsonl"
    codes = []
    try:
        for line in sup.read_text().splitlines():
            e = json.loads(line)
            if e.get("event") == "exit":
                codes.append(e.get("code"))
    except (OSError, ValueError):
        pass
    return codes


def in_order(events: list, steps: list) -> tuple[bool, list]:
    """Each step matched by a later event than the previous one (a step = (name, predicate on an event)); returns
    (all matched, [(name, event index or None)])."""
    found, i = [], 0
    for name, pred in steps:
        j = next((k for k in range(i, len(events)) if pred(events[k])), None)
        found.append((name, j))
        if j is None:
            return False, found + [(n, None) for n, _ in steps[len(found):]]
        i = j + 1
    return True, found


def _reply(kind, t="ack", reason=None):
    return lambda e: (e.get("event") == "reply" and e.get("kind") == kind and e.get("t") == t
                      and (reason is None or e.get("reason") == reason))


def _label(label):
    return lambda e: e.get("event") == "reached" and e.get("label") == label


def _replies(probe: dict, kind: str) -> list:
    return [e for e in probe.get("events") or [] if e.get("event") == "reply" and e.get("kind") == kind]


def _reached(probe: dict, label: str) -> bool:
    return any(e.get("event") == "reached" and e.get("label") == label for e in probe.get("events") or [])


def lifecycle(sess: dict, which: int) -> dict:
    p, rows, codes = sess.get("probe") or {}, sess.get("rows") or [], _epoch_exit(sess)
    ends = _replies(p, "session_end")
    common = {"epoch_exit": codes, "exit_ok": codes == [0], "session_end_ack": any(e.get("t") == "ack" for e in ends),
              "result_json": sess.get("result") is not None,
              "tick_trace": sess.get("trace") is not None}
    if which == 1:
        c = {"stopped_in_capture": _reached(p, "stopped_in_capture"),
             "rearm_ack": any(e.get("t") == "ack" for e in _replies(p, "rearm")),
             "resume_acks": sum(e.get("t") == "ack" for e in _replies(p, "mission_resume")),
             "auto_again": _reached(p, "auto_again"),
             "teleop_entry_ack": (p.get("teleop_entry") or {}).get("t") == "ack",
             "arm_ack": any(e.get("t") == "ack" for e in _replies(p, "arm")),
             "expired_disarmed": _reached(p, "expired_disarmed"),
             "new_lease": bool((p.get("reconnect") or {}).get("new_lease")),
             "old_lease_refused": (p.get("old_lease_after_reconnect") or {}).get("reason") == "stale_lease",
             "mission_done": bool(rows) and rows[0].get("outcome") == "done"}
        teleop_id = (p.get("teleop_entry") or {}).get("request_id")
        ok, order = in_order(p.get("events") or [], [
            ("stopped_in_capture", _label("stopped_in_capture")), ("rearm", _reply("rearm")),
            ("resume", _reply("mission_resume")), ("auto_again", _label("auto_again")),
            ("teleop_entry", lambda e: _reply("mode_request")(e) and e.get("request_id") == teleop_id),
            ("arm", _reply("arm")), ("expired_disarmed", _label("expired_disarmed")),
            ("new_lease", _reply("lease_request")), ("old_lease_refused", _reply("mode_request", "reject", "stale_lease")),
            ("final_resume", _reply("mission_resume")), ("session_end", _reply("session_end"))])
        c["order"] = order
        c["pass"] = all([ok, c["teleop_entry_ack"], c["new_lease"], c["old_lease_refused"], c["mission_done"],
                         common["exit_ok"], common["result_json"], common["tick_trace"]])
    elif which == 2:
        runs = p.get("missions_run") or []
        first = runs[0] if runs else {}
        rejects = [e.get("reason") for e in _replies(p, "mission_submit") if e.get("t") == "reject"]
        cancel = (first.get("cancel_reply") or {})
        h3 = h3_check(sess, (cancel or {}).get("request_id"))
        c = {"transport_reached": bool(first.get("cancel_phase_reached")), "cancel_ack": cancel.get("t") == "ack",
             "cancelled_operator": bool(rows) and rows[0].get("outcome") == "cancelled"
                                   and rows[0].get("reason") == "operator",
             "cargo_held_rejects": rejects.count("cargo_held"), "h3": h3}
        c["pass"] = all([c["transport_reached"], c["cancel_ack"], c["cancelled_operator"],
                         c["cargo_held_rejects"] >= 2, h3.get("ordered"), common["session_end_ack"], common["exit_ok"]])
    elif which == 3:
        reloc_needs_op = any("reloc_needs_operator" in str(e.get("reason") or "")
                             for e in _supervisor(sess) if e.get("event") == "exit")
        c = {"reloc_needs_operator": reloc_needs_op,
             "mission_done": bool(rows) and rows[0].get("outcome") == "done"}
        c["outcome"] = ("ended_per_record" if reloc_needs_op else
                        "pass" if c["mission_done"] and common["session_end_ack"] and common["exit_ok"] else "fail")
        c["pass"] = c["outcome"] == "pass"
    else:
        runs = p.get("missions_run") or []
        w = (runs[0] if runs else {}).get("stop_wait") or {}
        stop = w.get("stop_reply") or {}
        c = {"phase_reached": bool(w.get("phase_reached")), "stop_ack": stop.get("t") == "ack",
             "rearm_ack": (w.get("rearm") or {}).get("t") == "ack",
             "cancel_ack": (w.get("cancel_reply") or {}).get("t") == "ack",
             "m1_cancelled": bool(rows) and rows[0].get("outcome") == "cancelled",
             "m2_done": len(rows) > 1 and rows[1].get("outcome") == "done",
             "after_cancel_statuses": w.get("after_cancel_statuses"), "recovery_resume": w.get("recovery_resume")}
        stop_ev = next((e for e in _replies(p, "stop") if e.get("sent_mono") is not None), None)
        c["stop_reply_s"] = None if stop_ev is None else stop_ev["reply_mono"] - stop_ev["sent_mono"]
        needs_recovery = "recovering" in (w.get("after_cancel_statuses") or []) or bool(w.get("recovery_resume"))
        steps_ = [("approach", _label("stop_wait:approach")), ("stop", _reply("stop")),
                  ("stopped", _label("stopped_for_wait")), ("rearm", _reply("rearm")),
                  ("cancel", _reply("mission_cancel"))]
        if needs_recovery:
            steps_.append(("recovery_resume", _reply("mission_resume")))
        steps_ += [("m2_submit", _reply("mission_submit")), ("session_end", _reply("session_end"))]
        ok, order = in_order(p.get("events") or [], steps_)
        c["order"], c["needs_recovery"] = order, needs_recovery
        c["pass"] = all([ok, c["phase_reached"], c["m1_cancelled"], c["m2_done"], common["exit_ok"]])
    return {**common, **c}


def _supervisor(sess: dict) -> list:
    out = []
    try:
        for line in (sess["dir"] / "live" / "supervisor.jsonl").read_text().splitlines():
            out.append(json.loads(line))
    except (OSError, ValueError):
        pass
    return out


def h3_check(sess: dict, request_id) -> dict:
    """The applied cancel's record in the live log (H3) and the order probe send <= runtime receipt <= applied <= reply,
    the applied core time mapped to the runtime clock by the per-step records."""
    if not request_id:
        return {"found": False}
    entries = sess.get("entries") or []
    applied = [(e["tick_t"], c) for e in entries if "mission_commands" in e
               for c in e["mission_commands"] if c.get("request_id") == request_id and c.get("command") == "cancel"]
    receipt = next((e["mono"] for e in entries if e.get("t") == "mission_cancel" and e.get("request_id") == request_id), None)
    rep = next((e for e in _replies(sess.get("probe") or {}, "mission_cancel") if e.get("request_id") == request_id), None)
    if not applied or rep is None or receipt is None:
        return {"found": bool(applied), "receipt": receipt, "reply": rep is not None}
    t_applied = applied[0][0]
    steps = sess.get("steps") or {}
    mono_applied = min((v[0] for v in steps.values() if v[1] >= t_applied), default=None)
    sent = rep.get("sent_mono", rep["mono"] - float(rep.get("rtt_s") or 0.0))
    reply = rep.get("reply_mono", rep["mono"])
    return {"found": True, "applied_t": t_applied, "applied_mono": mono_applied, "sent": sent, "receipt": receipt,
            "reply": reply,
            "ordered": mono_applied is not None and sent <= receipt <= mono_applied <= reply}


# --- the batch ----------------------------------------------------------------------------------------------------
def evaluate(out_dir: Path) -> dict:
    plan = json.loads((out_dir / "plan_main.json").read_text())
    sess = {}
    names = [n for p in plan["pairs"] for n in p["runs"]] + plan["rest"]
    for n in names:
        sess[n] = load_session(out_dir / "s" / n)
    pairs, dropped = [], []
    totals = {arm: {"sim": 0.0, "wall": 0.0, "false_wall": 0.0, "valid_wall": 0.0, "longest": 0.0, "stalls": 0}
              for arm in ("A", "B")}
    hyp = {"A": {"rendered": 0, "capture": 0, "other": 0}, "B": {"rendered": 0, "capture": 0, "other": 0}}
    for p in plan["pairs"]:
        ss = [sess[n] for n in p["runs"]]
        why = []
        if not all(eligible(s) for s in ss):
            why.append("eligibility")
        ws = [s.get("window") for s in ss]
        a = b = None
        if all(ws):
            a, b = max(w[0] for w in ws), min(w[1] for w in ws)
            if b - a < COMMON_MIN_S:
                why.append("common interval under 15 min")
        else:
            why.append("no window")
        row = {"pair": p["pair"], "scene": p["scene"], "seed": p["seed"], "first": p["first"], "interval": [a, b],
               "sessions": {}}
        for s in ss:
            arm = s["name"].rsplit("_", 1)[1]
            info = {"eligible": eligible(s), "missions": [(r.get("mission_id"), r.get("outcome"), r.get("reason"))
                                                          for r in s.get("rows") or []]}
            if a is not None and (s.get("trace") is None or not s.get("steps")):
                why.append(f"missing {s['name']} (no tick trace or no step records)")
            if a is not None and s.get("trace") is not None and s.get("steps"):
                st = interval_stats(s["steps"], a, b)
                ex, unm = c2_steps(s)
                stl = stall_rows(s["trace"], a, b, ex)
                miss = (((s.get("result") or {}).get("tick_trace") or {}).get("dropped") != 0
                        or trace_missing(s["trace"], a, b) or not st["decidable"] or not s.get("log_complete"))
                if miss:
                    why.append(f"missing {s['name']}")
                info.update({k: st.get(k) for k in ("F", "R", "wall_s", "missing_steps", "missing_wall_s", "max_gap_s")})
                info.update({"longest_false_s": longest_false(s["steps"], a, b), "stalls": stl,
                             "c2_unmapped": unm})
            row["sessions"][arm] = info
        row["dropped"] = sorted(set(why))
        if row["dropped"]:
            dropped.append(row)
        else:
            for arm, info in row["sessions"].items():
                s = sess[f"ab_{p['scene']}_s{p['seed']}_{arm}"]
                st = interval_stats(s["steps"], a, b)
                totals[arm]["sim"] += st["F"] * st["wall_s"]  # = tick_t[k1] - tick_t[k0]
                totals[arm]["wall"] += st["wall_s"]
                totals[arm]["false_wall"] += st["false_wall_s"]  # the registered R: valid adjacent intervals only
                totals[arm]["valid_wall"] += st["valid_wall_s"]
                totals[arm]["longest"] = max(totals[arm]["longest"], info["longest_false_s"])
                totals[arm]["stalls"] += info["stalls"]["total"]
                for kind in ("rendered", "capture", "other"):
                    hyp[arm][kind] += info["stalls"][kind]
        pairs.append(row)
    pace = {"pairs_dropped": len(dropped)}
    if len(dropped) > MAX_DROPPED_PAIRS or totals["A"]["wall"] == 0 or totals["B"]["wall"] == 0:
        pace["verdict"] = "undecidable"
    else:
        pace.update(arm_rules(totals["A"], totals["B"]))
        pace["verdict"] = "pass" if pace["pass"] else "hold"
    pace["totals"], pace["hypothesis_split"] = totals, hyp
    # the C2 cap over every valid session that runs 5 (arm B, the C2 fixtures, the backoff targets, the lifecycles)
    cap = {}
    for n, s in sess.items():
        if n.endswith("_A") or s.get("run") is None:
            continue
        ks, unm = c2_steps(s)
        if ks or unm:
            cap[n] = {"gaps": c2_gaps(s, ks), "unmapped": unm}
    cap_ok = all(not v["unmapped"] and all(g["gap_s"] is not None and not g["over_cap"] for g in v["gaps"])
                 for v in cap.values())
    safety_tab = {n: safety(s) for n, s in sess.items() if not n.endswith("_A")}
    l3 = lifecycle(sess["life3_random_m_reloc"], 3)
    if l3.get("outcome") == "ended_per_record":
        safety_tab["life3_random_m_reloc"] = {"not_applied": "reloc_needs_operator before any mission"}
    b_only = [r for r in pairs if r["sessions"].get("A", {}).get("eligible") and not r["sessions"].get("B", {}).get("eligible")]
    return {"pace": pace, "pairs": pairs, "c2_cap": {"sessions": cap, "pass": cap_ok},
            "safety": safety_tab,
            "backoff": {n: backoff_firing(sess[n]) for n in ("boxed_line_m", "boxed_random_m")},
            "c2_function": {"c2_line_m_on": c2_firing(sess["c2_line_m_on"]),
                            "c2_line_m_shadow": c2_firing(sess["c2_line_m_shadow"])},
            "lifecycle": {"1": lifecycle(sess["life1_line_m_main"], 1), "2": lifecycle(sess["life2_random_m_held"], 2),
                          "3": l3, "4": lifecycle(sess["life4_line_m_stop"], 4)},
            "b_only_failures": [{"pair": r["pair"], "B": r["sessions"]["B"]["missions"]} for r in b_only],
            "rules": {"stall_s": STALL_S, "c2_cap_s": C2_CAP_S, "F_ratio": F_RATIO, "R_add": R_ADD,
                      "longest": [LONG_MUL, LONG_ADD_S], "stall_add": STALL_ADD, "max_dropped_pairs": MAX_DROPPED_PAIRS}}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out", type=Path)
    ap.add_argument("--json", type=Path, required=True)
    a = ap.parse_args(argv)
    a.json.write_text(json.dumps(evaluate(a.out), indent=1, default=str) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
