"""The live riib-defaults validation's verdict tables (docs/plans/2026-11-08-live-riib-defaults-validation.md 3.3, 3.4):
one JSON for batch 16782's sessions (OUT/s/<name>), read-only over the records.

    python tools/riib_live_verdict.py OUT --json VERDICT.json [--sessions NAME ...]

Per session: its arm, fixture and scenario (args.txt); the missions (tools/ct_survey.py session_rows: outcome, reason,
failure class, forbidden contacts); the safety values (forbidden contacts, low-object overlap through
the runner's session-pallet gap and rest records, unpermitted_dir_entries, load-safety failures); the features (the
low-clearance guard's caps and holds by phase, empty and loaded apart; the return start backoffs; the C2 stands and
injections); the probe's stall recoveries with the registered false-positive check; and the pace (3.4): the tick trace's
S where it has rows, the live log's per-step records after it, checked on the overlap first; stalls (> 0.25 s) with the
substitute's 0.20-0.30 s gaps uncertain; the C2 replan steps counted apart (their gap from the trace only); the longest
run of pacing_ok false; a trace-only (<= 300 s) column beside the full window. A description: the 3.3 rules are applied
by the reader (the arm-wise comparisons) from these tables."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import shlex
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "isaac"))
import ct_survey  # noqa: E402
from session_pallets import RESTING_M, RESTING_RAD  # noqa: E402  (no Isaac imports)

from forklift_core.live.spool import read_live_log  # noqa: E402

STALL_S = 0.25  # the pacer's stall (registration 3.3)
BAND_S = 0.05  # the substitute's uncertain band around it (3.4)
OVERLAP_MAX_S = 0.05  # the overlap check's largest difference (3.4)
C2_CAP_S = 2.0  # the C2 replan step's policy cap (3.3)
LOADED_PHASES = ("lift", "extract", "transport", "settle", "lower")  # tools/low_clearance_report.py's


def session_meta(d: Path) -> dict:
    """The run's env as the wrapper wrote it (args.txt)."""
    out = {}
    for line in (d / "args.txt").read_text().splitlines():
        if "=" in line and not line.startswith("PROBE="):
            k, v = line.split("=", 1)
            out[k] = v
        elif line.startswith("PROBE="):
            out["PROBE"] = shlex.split(line[len("PROBE="):])
    return out


def run_dir(d: Path) -> Path | None:
    runs = sorted(d.glob("live/epoch_*/run"))
    return runs[0] if len(runs) == 1 else None


# --- pace (3.4) ---------------------------------------------------------------------------------------------------------
def step_records(entries: list) -> dict:
    """step -> (mono, tick_t, pacing_ok) from the per-step publish records."""
    return {int(e["step"]): (float(e["mono"]), float(e["tick_t"]), bool(e["pacing_ok"]))
            for e in entries if "pacing_ok" in e and "step" in e}


def trace_gaps(run: Path) -> tuple[dict, dict]:
    """step -> S[k] - S[k-1] where both rows exist, and the trace's own record (rows, dropped)."""
    z = np.load(run / "tick_trace.npz")
    cols = list(z["columns"])
    data = z["data"]
    S, ls = data[:, cols.index("S")], data[:, cols.index("live_step")]
    by_step = {int(k): float(s) for k, s in zip(ls, S, strict=True) if not (math.isnan(k) or math.isnan(s))}
    gaps = {k: by_step[k] - by_step[k - 1] for k in by_step if k - 1 in by_step}
    return gaps, {"rows": int(len(data))}


def log_gaps(steps: dict) -> dict:
    return {k: steps[k][0] - steps[k - 1][0] for k in steps if k - 1 in steps}


def overlap_check(tg: dict, lg: dict) -> dict:
    both = sorted(set(tg) & set(lg))
    if not both:
        return {"steps": 0, "passed": False}
    d = np.abs(np.array([tg[k] - lg[k] for k in both]))
    mism = sum((tg[k] > STALL_S) != (lg[k] > STALL_S) for k in both)
    return {"steps": len(both), "class_mismatches": int(mism), "max_diff_s": float(d.max()),
            "p99_diff_s": float(np.percentile(d, 99)), "passed": bool(mism == 0 and d.max() <= OVERLAP_MAX_S)}


def window(entries: list) -> tuple[float | None, float | None]:
    """First mission submit to the last mission end, on the runtime clock (the live log's own marks)."""
    submits = [float(e["mono"]) for e in entries if e.get("t") == "mission_submit" and "mono" in e]
    ends = [float(e["mono"]) for e in entries if "mission_end" in e and "mono" in e]
    return (min(submits) if submits else None), (max(ends) if ends else None)


def step_at_tick(steps: dict, t: float) -> int | None:
    """The step whose publish record carries this core time (the control tick that ran at t)."""
    best = min(steps.items(), key=lambda kv: abs(kv[1][1] - t), default=None)
    return None if best is None or abs(best[1][1] - t) > 1e-6 else best[0]


def pace(steps: dict, tg: dict, lg: dict, ok: bool, w0, w1, c2_steps: set, trace_last: int | None) -> dict:
    """The window's step gaps: the trace where it has the step, else the substitute (when the overlap check passed).
    C2 replan steps are listed apart. Returns the full-window and the trace-only (steps <= trace_last) columns."""
    in_w = sorted(k for k in steps if w0 is not None and w1 is not None and w0 <= steps[k][0] <= w1)

    def column(keys: list) -> dict:
        confirmed = uncertain = missing = from_log = 0
        biggest, c2 = 0.0, []
        undecidable_span = 0
        for k in keys:
            if k in c2_steps:
                c2.append({"step": k, "gap_s": tg.get(k), "source": "trace" if k in tg else "none",
                           "log_gap_s": lg.get(k)})
                continue
            if k in tg:
                g = tg[k]
            elif k in lg:
                if not ok:
                    undecidable_span += 1
                    continue
                g = lg[k]
                from_log += 1
                if STALL_S - BAND_S <= g <= STALL_S + BAND_S:
                    uncertain += 1
                    biggest = max(biggest, g)
                    continue
            else:
                missing += 1
                continue
            biggest = max(biggest, g)
            confirmed += g > STALL_S
        return {"steps": len(keys), "stalls_confirmed": confirmed, "stalls_uncertain": uncertain,
                "max_gap_s": biggest, "gaps_from_log": from_log, "missing_gaps": missing,
                "undecidable_steps": undecidable_span, "c2_steps": c2}

    full = column(in_w)
    trace_only = column([k for k in in_w if trace_last is not None and k <= trace_last])
    # the longest run of pacing_ok false inside the window (a missing step breaks it), in wall seconds
    longest, run_start, prev = 0.0, None, None
    for k in in_w:
        if prev is not None and k != prev + 1:
            run_start = None
        if not steps[k][2]:
            run_start = steps[k][0] if run_start is None else run_start
            longest = max(longest, steps[k][0] - run_start)
        else:
            run_start = None
        prev = k
    full["longest_pacing_false_s"] = longest
    return {"full": full, "trace_only": trace_only}


def realtime_factors(entries: list, w0, w1) -> dict:
    vals = [e["pacing"]["realtime_factor"] for e in entries
            if e.get("status") and (e.get("pacing") or {}).get("realtime_factor") is not None
            and w0 is not None and w1 is not None and w0 <= e["mono"] <= w1]
    return {"median": float(np.median(vals)) if vals else None, "n": len(vals)}


# --- features -----------------------------------------------------------------------------------------------------------
def guard_summary(result: dict) -> dict | None:
    g = result.get("low_clearance")
    if g is None:
        return None
    split = {"empty": {"capped": 0, "zero": 0}, "loaded": {"capped": 0, "zero": 0}}
    for key, out in (("capped_ticks", "capped"), ("zero_ticks", "zero")):
        for ph, n in (g.get(key) or {}).items():
            split["loaded" if ph in LOADED_PHASES else "empty"][out] += int(n)
    holds = g.get("holds") or []
    return {"config": g.get("config"), "ticks": split, "holds": holds, "holds_saturated": len(holds) >= 50,
            "min_clearance_m": g.get("min_clearance_m")}


def mission_features(run: Path) -> list:
    out = []
    for md in sorted(run.glob("missions/*"), key=lambda p: ct_survey._mission_key(p.name)):
        try:
            st = (json.loads((md / "records.json").read_text()).get("state") or {})
        except (OSError, ValueError):
            out.append({"mission_id": md.name, "records": False})
            continue
        out.append({"mission_id": md.name, "records": True, "phase": st.get("phase"),
                    "return_start_backoffs": st.get("return_start_backoffs") or [],
                    "observe_end_stands": st.get("observe_end_stands") or [],
                    "near_end_injections": st.get("near_end_injections") or [],
                    "near_capture_status": (st.get("near_capture") or {}).get("status"),
                    "log": [r for r in st.get("log") or [] if r.get("command") in ("cancel", "submit")]})
    return out


# --- safety -------------------------------------------------------------------------------------------------------------
def low_objects(result: dict) -> dict:
    """The low objects of a session scene are the other session pallets (the static props are 1.3 m slot markers, above
    the LiDAR plane): the runner's own truth record per tick -- the smallest gap between the truck's planner outline and
    each other pallet's true outline (``session_neighbour_gap``, 0 = touching or overlapping) and each resting pallet's
    largest move (``session_rest_largest``, scene damage past RESTING_M / RESTING_RAD)."""
    gaps = result.get("session_neighbour_gap") or {}
    rest = result.get("session_rest_largest") or {}
    return {"neighbour_gap": gaps, "overlaps": sorted(k for k, g in gaps.items() if float(g["m"]) <= 0.0),
            "min_gap_m": min((float(g["m"]) for g in gaps.values()), default=None),
            "rest_moves": rest, "rest_bound": [RESTING_M, RESTING_RAD],
            "moved_past_bound": sorted(k for k, (m, r) in rest.items() if m > RESTING_M or r > RESTING_RAD),
            "recorded": bool(gaps) or bool(rest)}


# --- the false-positive check (3.2, 3.4) --------------------------------------------------------------------------------
def to_sim(steps: dict, mono: float) -> float | None:
    """The core time at a runtime-clock instant, by the per-step (mono, tick_t) pairs (linear between neighbours)."""
    pts = sorted((v[0], v[1]) for v in steps.values())
    xs = [p[0] for p in pts]
    i = bisect.bisect_left(xs, mono)
    if i == 0 or i == len(xs):
        return None
    (m0, t0), (m1, t1) = pts[i - 1], pts[i]
    return t0 + (t1 - t0) * (mono - m0) / (m1 - m0) if m1 > m0 else t0


def false_positive(entry: dict, steps: dict, guard: dict | None, mission: dict | None, probe_events: list) -> dict:
    rec = entry.get("stall_recovery") or {}
    span = rec.get("stall_span_mono")
    if not span or guard is None:
        return {"verdict": "undecidable", "why": "no span or no guard record"}
    a, b = to_sim(steps, span["since"]), to_sim(steps, span["decided"])
    if a is None or b is None:
        return {"verdict": "undecidable", "why": "the span is outside the per-step records"}
    # the independent check: the probe's cancel request vs the mission log's cancel, within one control tick
    # the recovery's cancel: the first cancel reply after the stall decision; its send = the reply's mono - rtt_s
    cancel_req = [e for e in probe_events if e.get("event") == "reply" and e.get("kind") == "mission_cancel"
                  and e.get("mono", -math.inf) >= span["decided"]]
    logged = [r for r in (mission or {}).get("log", []) if r.get("command") == "cancel" and r.get("accepted")]
    check = None
    if cancel_req and logged:
        first = min(cancel_req, key=lambda e: e["mono"])
        t_req = to_sim(steps, first["mono"] - float(first.get("rtt_s") or 0.0))
        check = None if t_req is None else {"probe_sim_s": t_req, "log_sim_s": logged[-1]["t"],
                                            "diff_s": abs(t_req - logged[-1]["t"])}
    dt = 1.0 / 120.0
    if check is None or check["diff_s"] > dt:
        return {"verdict": "undecidable", "why": "cancel check", "check": check, "span_sim_s": [a, b]}
    hit = [h for h in guard["holds"] if h["since_s"] <= b and h["last_s"] >= a]
    if hit:
        return {"verdict": "guard_stop", "holds": hit, "span_sim_s": [a, b], "check": check}
    if guard["holds_saturated"]:
        return {"verdict": "undecidable", "why": "holds saturated", "span_sim_s": [a, b], "check": check}
    return {"verdict": "false_positive", "span_sim_s": [a, b], "check": check}


# --- one session ----------------------------------------------------------------------------------------------------------
def session(d: Path) -> dict:
    meta = session_meta(d)
    out = {"name": d.name, "arm": meta.get("ARM"), "fix": meta.get("FIX"), "scenario": meta.get("SCENARIO"),
           "scene": Path(meta.get("SCENE_FILE", "")).stem, "seed": meta.get("SEED"), "probe": meta.get("PROBE")}
    run = run_dir(d)
    probe = ct_survey._json(d / "probe.json")
    out["probe_found"] = probe is not None
    if run is None:
        out["undecidable"] = "not exactly one epoch"
        return out
    rows = ct_survey.session_rows(d, 3)
    out["missions"] = [{k: r.get(k) for k in ("order", "pallet_id", "destination_id", "mission_id", "outcome", "reason",
                                              "class", "forbidden_contacts", "reject_reason", "start")}
                       for r in rows["rows"]]
    try:
        result = json.loads((run / "result.json").read_text())
    except (OSError, ValueError):
        result = None  # the session did not end through its own end path (e.g. TERMed): no run record
    out["result_json"] = result is not None
    if result is None:
        out["safety"] = {"undecidable": "no run result.json",
                         "forbidden_contacts_missions": sum(int(m.get("forbidden_contacts") or 0)
                                                            for m in out["missions"]),
                         "load_safety_failures": sum(m.get("class") == "load_safety" for m in out["missions"])}
        result = {}
    else:
        out["safety"] = None
    ol = result.get("obstacle_layer") or {}
    out["safety"] = out["safety"] or {"forbidden_contacts_run": len(result.get("forbidden_pocket_contacts") or []),
                     "forbidden_contacts_missions": sum(int(m.get("forbidden_contacts") or 0) for m in out["missions"]),
                     "unpermitted_dir_entries": ol.get("unpermitted_dir_entries"),
                     "unpermitted_entries": ol.get("unpermitted_entries"),
                     "load_safety_failures": sum(m.get("class") == "load_safety" for m in out["missions"]),
                     "low_objects": low_objects(result)}
    out["guard"] = guard_summary(result)
    feats = mission_features(run)
    out["features"] = feats
    out["tick_trace"] = result.get("tick_trace")
    # pace
    try:
        entries = read_live_log(run)
    except ValueError as e:  # 3.4: a log read_live_log refuses leaves the substitute span undecidable
        entries = read_live_log(run, allow_incomplete=True)
        out["live_log_incomplete"] = str(e)
    steps = step_records(entries)
    tg = trace_gaps(run)[0] if (run / "tick_trace.npz").exists() else {}
    lg = log_gaps(steps)
    chk = overlap_check(tg, lg)
    if out.get("live_log_incomplete"):
        chk["passed"] = False  # the substitute is not used on a log read_live_log refuses
    out["overlap"] = chk
    w0, w1 = window(entries)
    c2_steps = set()
    for f in feats:
        for r in f.get("observe_end_stands", []):
            if "planning_wall_s" in r:
                k = step_at_tick(steps, r["time_s"])
                if k is not None:
                    c2_steps.add(k)
    trace_last = max(tg) if tg else None
    out["pace"] = pace(steps, tg, lg, chk["passed"], w0, w1, c2_steps, trace_last)
    out["pace"]["window_mono"] = [w0, w1]
    out["pace"]["realtime_factor"] = realtime_factors(entries, w0, w1)
    for c in out["pace"]["full"]["c2_steps"]:
        c["over_cap"] = None if c["gap_s"] is None else c["gap_s"] > C2_CAP_S
    # stall recoveries
    events = (probe or {}).get("events") or []
    stalls = []
    for e in (probe or {}).get("missions_run") or []:
        if "stall_recovery" in e:
            mid = (e.get("submit") or {}).get("mission_id")
            mission = next((f for f in feats if f["mission_id"] == mid), None)
            stalls.append({"mission_id": mid, "outcome": e["stall_recovery"].get("outcome"),
                           "false_positive_check": false_positive(e, steps, out["guard"], mission, events)})
    out["stall_recoveries"] = stalls
    out["manual_recovery"] = (probe or {}).get("manual_recovery")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out", type=Path)
    ap.add_argument("--json", type=Path, required=True)
    ap.add_argument("--sessions", nargs="*")
    a = ap.parse_args(argv)
    names = a.sessions or sorted(p.name for p in (a.out / "s").iterdir() if p.is_dir())
    res = {"sessions": [session(a.out / "s" / n) for n in names],
           "rules": {"stall_s": STALL_S, "band_s": BAND_S, "overlap_max_s": OVERLAP_MAX_S, "c2_cap_s": C2_CAP_S}}
    a.json.write_text(json.dumps(res, indent=1, default=str) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
