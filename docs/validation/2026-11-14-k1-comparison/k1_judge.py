"""K1 comparison, judged by the rules fixed before the results (docs/plans/2026-11-13-k1-near-loose-arrival.md 5).

    python k1_judge.py RAW_ROOT --out K1.json [--random1-seeds 0-29]
    python k1_judge.py --raw-fields k1_raw_fields.json.gz --out K1.json

RAW_ROOT holds k1c_<B0|K1>_<scene>_s<seed>/run/result.json; --raw-fields reads the same results reduced to the keys
judged (keyed by run name). The cells are random_1 seeds 0-29 (or the pre-registered extension 0-59, plan 4), line_2
and aisle_rack seeds 0-4; a run outside them stops the judgement instead of being ignored. Needs forklift_core on the
path (the scene-damage class is mission.failures'). Reads only. Every run is first checked: exactly one run per arm and cell, deterministic.enabled true and
deterministic.broken empty. A run is then put in one class: complete, insert (reached the insert phase, not complete),
refused (ended at the near capture: near_capture_failed / near_capture_misaligned), accepted (near capture accepted,
insert never reached) or pre_capture (anything else). The cycle time is the home_settle transition's simulated time
(a riib run's one mission starts at the run's start).
"""

from __future__ import annotations

import argparse
import gzip
import json
import math
import re
import statistics
from pathlib import Path

from forklift_core.mission.failures import SCENE, classify

ARMS = ("B0", "K1")


def cells_for(random1_seeds: int = 30) -> list:
    return ([("random_1", s) for s in range(random1_seeds)] + [("line_2", s) for s in range(5)]
            + [("aisle_rack", s) for s in range(5)])


CELLS = cells_for(30)
NAME = re.compile(r"^k1c_(B0|K1)_(.+)_s(\d+)$")
NEAR_FAILS = ("near_capture_failed", "near_capture_misaligned")
OFFSET_LIMIT_M = 0.08
MOVE_LIMIT_M = 0.005


def failure_of(r: dict) -> str | None:
    return r.get("failure_reason") or (r.get("tracking_failure") or {}).get("reason") or None


def outcome_of(r: dict) -> str:
    if r.get("success"):
        return "complete"
    if any(t.get("to") == "insert" for t in r.get("transitions") or ()):
        return "insert"
    if any(str(failure_of(r) or "").startswith(k) for k in NEAR_FAILS):
        return "refused"
    if (r.get("near_capture") or {}).get("status") == "done":
        return "accepted"
    return "pre_capture"


def first_near_offset(r: dict) -> float | None:
    """The first near capture's lateral offset to the new line (a capture refused into a realignment comes first)."""
    for nc in list(r.get("near_capture_refused") or ()) + [r.get("near_capture") or {}]:
        off = (nc or {}).get("offsets_to_new_line")
        if off and off.get("lateral_m") is not None:
            return float(off["lateral_m"])
    return None


def cycle_of(r: dict) -> float | None:
    for t in r.get("transitions") or ():
        if t.get("to") == "home_settle":
            return float(t["time_s"])
    return None


def observe_timeout(r: dict) -> bool:
    f = str(failure_of(r) or "")
    return f in ("Tracking timeout in observe", "observe_timeout")


def event_success(r: dict, ev: dict) -> str:
    """'success' (C2 installed, the arrival inside the narrow bounds, the near capture accepted after it),
    'outside_narrow' (an arrival outside them), or what stopped short."""
    stands = r.get("observe_end_stands") or []
    idx = ev.get("c2_record_index")
    installed = idx is not None and idx < len(stands) and stands[idx].get("outcome") == "installed"
    arrival, capture = ev.get("arrival"), ev.get("capture") or {}
    if not installed:
        return "no_c2_install"
    if arrival is None:
        return "no_arrival"
    if not arrival.get("inside_narrow"):
        return "outside_narrow"
    return "success" if capture.get("status") == "done" else f"capture_{capture.get('status')}"


def scene_damage(failure) -> bool:
    return bool(failure) and classify(str(failure)) == SCENE


def summarize(run: dict) -> dict:
    r = run["result"]
    det = r.get("deterministic") or {}
    events = r.get("near_loose_refusals") or []
    return {
        "name": run["name"], "arm": run["arm"], "scene": run["scene"], "seed": run["seed"],
        "valid": bool(det.get("enabled")) and not det.get("broken"),
        "outcome": outcome_of(r), "success": bool(r.get("success")), "failure": failure_of(r),
        "phase": r.get("phase"), "cycle_s": cycle_of(r), "near_lateral_m": first_near_offset(r),
        "observe_timeout": observe_timeout(r),
        "forbidden_contacts": len(r.get("forbidden_pocket_contacts") or []),
        "events": [{"time_s": e.get("time_s"), "kinds": e.get("kinds"), "lateral_m": e.get("lateral_m"),
                    "along_m": e.get("along_m"), "position_error_m": e.get("position_error_m"),
                    "yaw_error_rad": e.get("yaw_error_rad"), "verdict": event_success(r, e),
                    "arrival": e.get("arrival"), "capture": e.get("capture"),
                    "superseded_by": e.get("superseded_by")} for e in events],
        "fallbacks": r.get("near_loose_fallbacks") or [],
        "watch": [{k: w.get(k) for k in ("start_s", "end_s", "steps", "max_move_m", "contact_steps", "first_contact",
                                         "near_capture_status")} for w in r.get("near_loose_watch") or []],
    }


def median(xs):
    return statistics.median(xs) if xs else None


def finite(v) -> bool:
    return isinstance(v, (int, float)) and math.isfinite(v)


def record_gaps(pairs: list) -> list:
    """What the judgement needs and a run does not carry: every intervention's watch window (closed, with steps and
    finite measurements) and every joint completion's cycle time. Any gap stops the judgement."""
    gaps = []
    for b, k in pairs:
        if k["events"]:
            if len(k["watch"]) < 1:
                gaps.append(f"{k['name']}: interventions without a watch window")
            for w in k["watch"]:
                if w["end_s"] is None or not (w["steps"] or 0) > 0 or not finite(w["max_move_m"]) \
                        or not isinstance(w["contact_steps"], int):
                    gaps.append(f"{k['name']}: watch window open, empty or not finite")
        for x in (b, k):
            if x["events"] and x["arm"] == "B0":
                gaps.append(f"{x['name']}: a B0 run with K1 interventions")
        if b["success"] and k["success"] and not (finite(b["cycle_s"]) and finite(k["cycle_s"])):
            gaps.append(f"{b['name']}/{k['name']}: joint completion without a home_settle time")
    return gaps


def judge(rows: list[dict], cells: list | None = None) -> dict:
    cells = CELLS if cells is None else cells
    by = {(x["arm"], x["scene"], x["seed"]): x for x in rows}
    out: dict = {"checks": {}, "criteria": {}, "records": {}}
    counts = {}
    for x in rows:
        counts[(x["arm"], x["scene"], x["seed"])] = counts.get((x["arm"], x["scene"], x["seed"]), 0) + 1
    missing = [f"{a}_{c}_s{s}" for c, s in cells for a in ARMS if counts.get((a, c, s), 0) != 1]
    selected = {(a, c, s) for c, s in cells for a in ARMS}
    outside = sorted(x["name"] for x in rows if (x["arm"], x["scene"], x["seed"]) not in selected)
    invalid = [x["name"] for x in rows if not x["valid"]]
    out["checks"] = {"pairs_complete": not missing, "missing_or_duplicate": missing, "outside_cells": outside,
                     "invalid": invalid, "cells": len(cells)}
    if missing or invalid or outside:
        out["verdict"] = "not_judged: incomplete, invalid or unselected runs"
        return out
    pairs = [(by[("B0", c, s)], by[("K1", c, s)]) for c, s in cells]
    gaps = record_gaps(pairs)
    out["checks"]["record_gaps"] = gaps
    if gaps:
        out["verdict"] = "not_judged: missing records"
        return out
    r1 = [(b, k) for b, k in pairs if b["scene"] == "random_1"]
    others = [(b, k) for b, k in pairs if b["scene"] != "random_1"]
    k1_runs = [k for _, k in pairs]

    # 1 safety
    intervened = [k for k in k1_runs if k["events"]]
    watch_contacts = sum(w["contact_steps"] or 0 for k in intervened for w in k["watch"])
    watch_move = max((w["max_move_m"] or 0.0 for k in intervened for w in k["watch"]), default=0.0)
    open_windows = [k["name"] for k in intervened for w in k["watch"] if w["end_s"] is None]
    damage = [k["name"] for k in intervened if scene_damage(k["failure"])]  # registered scope: the intervened runs
    after_accept = [k["name"] for k in intervened
                    if k["outcome"] in ("accepted", "insert") and k["phase"] in ("approach", "insert")]
    forbidden = sum(x["forbidden_contacts"] for x in rows)
    c1 = {"intervened_runs": len(intervened), "watch_windows": sum(len(k["watch"]) for k in intervened),
          "watch_steps": sum(w["steps"] for k in intervened for w in k["watch"]),
          "watch_contact_steps": watch_contacts, "watch_max_move_m": watch_move,
          "watch_windows_open_at_end": open_windows, "damage_failures": damage,
          "damage_failures_all_runs_record": [x["name"] for x in rows if scene_damage(x["failure"])],
          "approach_insert_failures_after_accepted_capture": after_accept, "forbidden_contacts_all": forbidden}
    c1["pass"] = (watch_contacts == 0 and watch_move <= MOVE_LIMIT_M and not damage and not after_accept
                  and forbidden == 0)
    out["criteria"]["1_safety"] = c1

    # 2 completion
    comp = lambda ps, arm: sum(1 for b, k in ps if (b if arm == "B0" else k)["success"])  # noqa: E731
    c2 = {"all_B0": comp(pairs, "B0"), "all_K1": comp(pairs, "K1"), "r1_B0": comp(r1, "B0"), "r1_K1": comp(r1, "K1")}
    c2["pass"] = c2["all_K1"] >= c2["all_B0"] - 1 and c2["r1_K1"] >= c2["r1_B0"]
    out["criteria"]["2_completion"] = c2

    # 3 near refusals (random_1)
    refused = {a: sum(1 for b, k in r1 if (b if a == "B0" else k)["outcome"] == "refused") for a in ARMS}
    big = {a: sum(1 for b, k in r1 if (lat := (b if a == "B0" else k)["near_lateral_m"]) is not None
                  and abs(lat) > OFFSET_LIMIT_M) for a in ARMS}
    worse = [k["name"] for b, k in r1 if b["outcome"] in ("accepted", "insert", "complete")
             and k["outcome"] in ("pre_capture", "refused")]
    c3 = {"refused": refused, "lateral_over_0_08": big, "new_worse_pairs": worse,
          "underexposed": big["B0"] < 4}
    # an underexposed lateral term (B0 under 4) is left out of the pass and named by "underexposed"
    c3["pass"] = (refused["K1"] <= refused["B0"] and len(worse) <= 1
                  and (c3["underexposed"] or big["K1"] <= big["B0"] / 2))
    out["criteria"]["3_near_refusals"] = c3

    # 4 recovery
    events = [(k["name"], e) for k in k1_runs for e in k["events"]]
    verdicts: dict = {}
    for _, e in events:
        verdicts[e["verdict"]] = verdicts.get(e["verdict"], 0) + 1
    n_ok = verdicts.get("success", 0)
    c4 = {"events": len(events), "verdicts": verdicts, "success_rate": n_ok / len(events) if events else None,
          "underexposed": len(events) < 5}
    c4["pass"] = c4["underexposed"] or n_ok / len(events) >= 0.80
    out["criteria"]["4_recovery"] = c4

    # 5 time cost (random_1)
    joint = [(b, k) for b, k in r1 if b["success"] and k["success"]]  # every one has its time (record_gaps)
    ratio = (median([k["cycle_s"] for _, k in joint]) / median([b["cycle_s"] for b, _ in joint])) if joint else None
    inter = [k["cycle_s"] - b["cycle_s"] for b, k in joint if k["events"]]
    new_timeouts = [k["name"] for b, k in pairs if k["observe_timeout"] and not b["observe_timeout"]]
    c5 = {"joint_complete_pairs": len(joint), "median_cycle_ratio": ratio, "intervened_joint_pairs": len(inter),
          "intervened_increase_median_s": median(inter), "intervened_increase_max_s": max(inter, default=None),
          "new_observe_timeouts": new_timeouts, "underexposed": len(inter) < 3}
    c5["pass"] = (ratio is not None and ratio <= 1.10 and not new_timeouts
                  and (c5["underexposed"] or (median(inter) <= 15.0 and max(inter) <= 45.0)))
    out["criteria"]["5_time"] = c5

    # 6 non-regression (line_2, aisle_rack)
    diff = [k["name"] for b, k in others if (b["success"], b["failure"]) != (k["success"], k["failure"])]
    other_events = [{"name": k["name"], "verdict": e["verdict"]} for _, k in others for e in k["events"]]
    # an intervention there is no regression only when it meets criterion 4's success definition
    failed_events = [e for e in other_events if e["verdict"] != "success"]
    c6 = {"pairs": len(others), "different_outcome": diff, "k1_events": other_events,
          "k1_events_not_successful": failed_events}
    c6["pass"] = not diff and not failed_events
    out["criteria"]["6_non_regression"] = c6

    cr = out["criteria"]
    if not (cr["1_safety"]["pass"] and cr["2_completion"]["pass"] and cr["6_non_regression"]["pass"]):
        verdict = "worse"
    elif c3["underexposed"] or c4["underexposed"] or c5["underexposed"]:
        # plan 5: any underexposed second term holds promotion whatever else 3-5 say; ask the lead for the
        # pre-registered extension (plan 4). The other 3-5 failures, if any, are reported with it.
        verdict = "effect_unconfirmed_promotion_held"
    elif not (c3["pass"] and c4["pass"] and c5["pass"]):
        verdict = "no_difference_confirmed"
    else:
        verdict = "k1_better"
    out["verdict"] = verdict

    # records (not judgements)
    all_joint = [(b, k) for b, k in pairs if b["success"] and k["success"]]
    out["records"] = {
        "cycle_delta_s_all_joint": sorted(round(k["cycle_s"] - b["cycle_s"], 3) for b, k in all_joint),
        "cycle_delta_s_intervened": sorted(round(k["cycle_s"] - b["cycle_s"], 3) for b, k in all_joint if k["events"]),
        "cycle_delta_s_not_intervened": sorted(round(k["cycle_s"] - b["cycle_s"], 3) for b, k in all_joint
                                               if not k["events"]),
        "observe_timeouts": {a: sum(1 for b, k in pairs if (b if a == "B0" else k)["observe_timeout"]) for a in ARMS},
        "outcomes": {a: {c: sum(1 for b, k in pairs if (b if a == "B0" else k)["outcome"] == c)
                         for c in ("pre_capture", "refused", "accepted", "insert", "complete")} for a in ARMS},
        "events": [{"name": n, **e} for n, e in events],
        "fallbacks": [{"name": k["name"], **f} for k in k1_runs for f in k["fallbacks"]],
        "pairs": [{"cell": f"{b['scene']}_s{b['seed']}", "B0": [b["outcome"], b["failure"], b["cycle_s"],
                                                                b["near_lateral_m"]],
                   "K1": [k["outcome"], k["failure"], k["cycle_s"], k["near_lateral_m"], len(k["events"])]}
                  for b, k in pairs],
    }
    return out


def row(name: str, result: dict) -> dict:
    m = NAME.match(name)
    return summarize({"name": name, "arm": m[1], "scene": m[2], "seed": int(m[3]), "result": result})


def load(root: Path) -> list[dict]:
    rows = []
    for d in sorted(root.iterdir()):
        p = d / "run" / "result.json"
        if NAME.match(d.name) and p.exists():
            rows.append(row(d.name, json.loads(p.read_text())))
    return rows


def load_fields(path: Path) -> list[dict]:
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt") as f:
        return [row(name, result) for name, result in sorted(json.load(f).items())]


def parse_seeds(text: str) -> int:
    lo, hi = (int(v) for v in text.split("-"))
    if lo != 0 or hi not in (29, 59):
        raise SystemExit("--random1-seeds is 0-29 (registered) or 0-59 (the pre-registered extension)")
    return hi + 1


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path, nargs="?")
    ap.add_argument("--raw-fields", type=Path)
    ap.add_argument("--random1-seeds", default="0-29")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    if (a.root is None) == (a.raw_fields is None):
        raise SystemExit("give RAW_ROOT or --raw-fields")
    rows = load(a.root) if a.root is not None else load_fields(a.raw_fields)
    res = judge(rows, cells_for(parse_seeds(a.random1_seeds)))
    a.out.write_text(json.dumps(res, indent=1, allow_nan=False) + "\n")
    print(res.get("verdict"))
    for k, v in res.get("criteria", {}).items():
        print(k, "pass" if v.get("pass") else "FAIL", "(underexposed)" if v.get("underexposed") else "")


if __name__ == "__main__":
    main()
