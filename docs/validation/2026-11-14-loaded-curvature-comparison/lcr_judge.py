"""The loaded curvature rule comparison, judged by the rules fixed before the results
(docs/plans/2026-11-14-loaded-curvature-rule.md 5).

    python lcr_judge.py RAW_ROOT --out LCR.json
    python lcr_judge.py --raw-fields FIELDS.json.gz --out LCR.json

RAW_ROOT holds lcr_<B0|R>_<scene>_s<seed>/run/result.json for random_1 seeds 0-59, line_2 and aisle_rack seeds 0-4
(--raw-fields: the same results reduced to the keys judged, keyed by run name). A run outside those cells, a missing
or duplicate run, or an invalid deterministic run stops the judgement. Needs forklift_core on the path (the scene-damage
class is mission.failures'). Reads only.

Run states (plan 5.1, one per run): complete (success); transport_planning (the loaded transport path could not be
made: the mission plan's transport stage -- a reason "transport:..." or "Mission planning failed: transport:..." --
or a transport-phase replan's final failure: a reason with replan_failed / invalid_start / search_no_path / no_path /
deadline / no_plan while the run ended in transport, the obstacle layer's "obstacle_replan_failed in transport" and
"obstacle_live_plan_failed in transport", and the D7 re-approach's "docking_retry_no_plan" -- a loaded transport path);
transport_cross_track (tracking_failure in transport, failure cross_track); other. The phase a run ended in is the
tracking failure's phase when there is one, else the last transition's target (the final state phase is not relied
on).
"""

from __future__ import annotations

import argparse
import gzip
import json
import re
import statistics
from pathlib import Path

from forklift_core.mission.failures import SCENE, classify

ARMS = ("B0", "R")
NAME = re.compile(r"^lcr_(B0|R)_(.+)_s(\d+)$")
PLANNING_WORDS = ("replan_failed", "invalid_start", "search_no_path", "no_path", "deadline", "no_plan")
TRANSPORT_PLANNING_PREFIXES = ("transport:", "Mission planning failed: transport:",
                               "obstacle_replan_failed in transport", "obstacle_live_plan_failed in transport",
                               "docking_retry_no_plan")


def cells() -> list:
    return ([("random_1", s) for s in range(60)] + [("line_2", s) for s in range(5)]
            + [("aisle_rack", s) for s in range(5)])


def failure_of(r: dict) -> str:
    return str(r.get("failure_reason") or (r.get("tracking_failure") or {}).get("reason") or "")


def ended_phase(r: dict):
    tf = r.get("tracking_failure") or {}
    if tf.get("phase"):
        return tf["phase"]
    trans = r.get("transitions") or []
    return trans[-1].get("to") if trans else r.get("phase")


def state_of(r: dict) -> str:
    if r.get("success"):
        return "complete"
    f = failure_of(r)
    tf = r.get("tracking_failure") or {}
    if tf.get("phase") == "transport" and (tf.get("tracking") or {}).get("failure") == "cross_track":
        return "transport_cross_track"
    if f.startswith(TRANSPORT_PLANNING_PREFIXES):
        return "transport_planning"
    if ended_phase(r) == "transport" and any(w in f for w in PLANNING_WORDS):
        return "transport_planning"
    return "other"


def failure_class(r: dict) -> str | None:
    f = failure_of(r)
    if not f:
        return None
    return f"{classify(f)}:{f.split(':')[0][:60]}"


def t_of(r: dict, to: str):
    for t in r.get("transitions") or ():
        if t.get("to") == to:
            return float(t["time_s"])
    return None


def transport_path(r: dict):
    replaced = [e["path"] for e in r.get("paths_replaced") or [] if e.get("leg") == "transport"]
    return replaced[0] if replaced else (r.get("paths") or {}).get("transport")


def summarize(name: str, r: dict) -> dict:
    m = NAME.match(name)
    det = r.get("deterministic") or {}
    path = transport_path(r)
    dirs = list((path or {}).get("directions") or [])
    t0, t1, t2 = t_of(r, "transport"), t_of(r, "lower"), t_of(r, "home_settle")
    return {
        "name": name, "arm": m[1], "scene": m[2], "seed": int(m[3]),
        "valid": bool(det.get("enabled")) and not det.get("broken"),
        "state": state_of(r), "success": bool(r.get("success")), "failure": failure_of(r)[:120] or None,
        "failure_class": failure_class(r), "phase": ended_phase(r),
        "transport_s": (t1 - t0) if t0 is not None and t1 is not None else None,
        "cycle_s": t2,
        "since_transport_s": (float(r.get("simulated_time_s") or 0.0) - t0) if t0 is not None else None,
        "forbidden_contacts": len(r.get("forbidden_pocket_contacts") or []),
        "scene_damage": bool(failure_of(r)) and classify(failure_of(r)) == SCENE,
        "rule": r.get("loaded_curvature_rule"),
        "transport_path_m": (path or {}).get("length_m"),
        "transport_cusps": sum(1 for a, b in zip(dirs, dirs[1:]) if a * b < 0),
        "transport_kappa_max": (max(abs(float(k)) for k in path["curvatures_inv_m"]) if path and path.get(
            "curvatures_inv_m") else None),
        "cusp_replans": len(r.get("cusp_replans") or []),
        "endpoint_heading": ((r.get("tracking_failure") or {}).get("tracking") or {}).get("failure") == "endpoint_heading"
        and (r.get("tracking_failure") or {}).get("phase") == "transport",
    }


def median(xs):
    return statistics.median(xs) if xs else None


def judge(rows: list[dict]) -> dict:
    out: dict = {"checks": {}, "criteria": {}, "records": {}}
    cs = cells()
    counts: dict = {}
    for x in rows:
        counts[(x["arm"], x["scene"], x["seed"])] = counts.get((x["arm"], x["scene"], x["seed"]), 0) + 1
    selected = {(a, c, s) for c, s in cs for a in ARMS}
    missing = [f"{a}_{c}_s{s}" for c, s in cs for a in ARMS if counts.get((a, c, s), 0) != 1]
    outside = sorted(x["name"] for x in rows if (x["arm"], x["scene"], x["seed"]) not in selected)
    invalid = [x["name"] for x in rows if not x["valid"]]
    # the arms' own identity: R carries the rule's record, B0 none
    wrong_arm = [x["name"] for x in rows if (x["arm"] == "R") != (x["rule"] is not None)]
    out["checks"] = {"missing_or_duplicate": missing, "outside_cells": outside, "invalid": invalid,
                     "wrong_arm": wrong_arm}
    if missing or outside or invalid or wrong_arm:
        out["verdict"] = "not_judged: incomplete, invalid, unselected or mislabelled runs"
        return out
    by = {(x["arm"], x["scene"], x["seed"]): x for x in rows}
    pairs = [(by[("B0", c, s)], by[("R", c, s)]) for c, s in cs]
    r1 = [p for p in pairs if p[0]["scene"] == "random_1"]
    others = [p for p in pairs if p[0]["scene"] != "random_1"]
    count = lambda ps, arm, st: sum(1 for b, r in ps if (b if arm == "B0" else r)["state"] == st)  # noqa: E731

    # 1 safety
    unsafe = lambda x: x["forbidden_contacts"] > 0 or x["scene_damage"]  # noqa: E731
    new_unsafe = [r["name"] for b, r in pairs if unsafe(r) and not unsafe(b)]
    c1 = {"B0_events": [b["name"] for b, _ in pairs if unsafe(b)], "R_events": [r["name"] for _, r in pairs if unsafe(r)],
          "new_in_R": new_unsafe}
    c1["pass"] = not new_unsafe
    out["criteria"]["1_safety"] = c1

    # 2 transport cross_track (random_1)
    ct_b, ct_r = count(r1, "B0", "transport_cross_track"), count(r1, "R", "transport_cross_track")
    c2 = {"B0": ct_b, "R": ct_r, "underexposed": ct_b < 4}
    c2["pass"] = c2["underexposed"] or ct_r <= ct_b // 2
    out["criteria"]["2_cross_track"] = c2

    # 3 completion
    c3 = {"all_B0": count(pairs, "B0", "complete"), "all_R": count(pairs, "R", "complete"),
          "r1_B0": count(r1, "B0", "complete"), "r1_R": count(r1, "R", "complete")}
    c3["pass"] = c3["all_R"] >= c3["all_B0"] - 1 and c3["r1_R"] >= c3["r1_B0"] - 1
    out["criteria"]["3_completion"] = c3

    # 4 transport planning failures
    by_scene = {}
    for scene in ("random_1", "line_2", "aisle_rack"):
        ps = [p for p in pairs if p[0]["scene"] == scene]
        by_scene[scene] = {"B0": count(ps, "B0", "transport_planning"), "R": count(ps, "R", "transport_planning")}
    new_other = [r["name"] for b, r in others if r["state"] == "transport_planning" and b["state"] != "transport_planning"]
    c4 = {"all_B0": count(pairs, "B0", "transport_planning"), "all_R": count(pairs, "R", "transport_planning"),
          "by_scene": by_scene, "new_in_line2_aisle_rack": new_other,
          "R_reasons": [r["failure"] for _, r in pairs if r["state"] == "transport_planning"]}
    c4["pass"] = c4["all_R"] <= c4["all_B0"] + 1 and not new_other
    out["criteria"]["4_planning"] = c4

    # 5 time (both completed)
    joint = [(b, r) for b, r in pairs if b["success"] and r["success"]
             and b["transport_s"] is not None and r["transport_s"] is not None and b["cycle_s"] and r["cycle_s"]]
    tr_ratio = (median([r["transport_s"] for _, r in joint]) / median([b["transport_s"] for b, _ in joint])) if joint else None
    cy_ratio = (median([r["cycle_s"] for _, r in joint]) / median([b["cycle_s"] for b, _ in joint])) if joint else None
    one_sided = [{"cell": f"{b['scene']}_s{b['seed']}", "B0": [b["state"], b["phase"], b["since_transport_s"]],
                  "R": [r["state"], r["phase"], r["since_transport_s"]]}
                 for b, r in pairs if b["success"] != r["success"]]
    c5 = {"joint_pairs": len(joint), "transport_time_ratio": tr_ratio, "cycle_time_ratio": cy_ratio,
          "one_sided_pairs": one_sided}
    c5["pass"] = tr_ratio is not None and tr_ratio <= 1.15 and cy_ratio <= 1.10
    out["criteria"]["5_time"] = c5

    # 6 non-regression (line_2, aisle_rack)
    diff = [r["name"] for b, r in others if (b["state"], b["failure_class"]) != (r["state"], r["failure_class"])]
    c6 = {"pairs": len(others), "different": diff}
    c6["pass"] = not diff
    out["criteria"]["6_non_regression"] = c6

    if not (c1["pass"] and c3["pass"] and c6["pass"]):
        verdict = "worse"
    elif c2["underexposed"]:
        verdict = "effect_unconfirmed_promotion_held"
    elif not (c2["pass"] and c4["pass"] and c5["pass"]):
        verdict = "no_difference_confirmed"
    else:
        verdict = "r_better"
    out["verdict"] = verdict

    out["records"] = {
        "states": {a: {st: count(pairs, a, st) for st in ("complete", "transport_planning", "transport_cross_track",
                                                         "other")} for a in ARMS},
        "transport_path_m_median": {a: median([(b if a == "B0" else r)["transport_path_m"] for b, r in pairs
                                               if (b if a == "B0" else r)["transport_path_m"] is not None]) for a in ARMS},
        "transport_cusps_total": {a: sum((b if a == "B0" else r)["transport_cusps"] for b, r in pairs) for a in ARMS},
        "cusp_replans_total": {a: sum((b if a == "B0" else r)["cusp_replans"] for b, r in pairs) for a in ARMS},
        "endpoint_heading": {a: sum(1 for b, r in pairs if (b if a == "B0" else r)["endpoint_heading"]) for a in ARMS},
        "R_transport_kappa_max": max((r["transport_kappa_max"] or 0.0) for _, r in pairs),
        "pairs": [{"cell": f"{b['scene']}_s{b['seed']}", "B0": [b["state"], b["failure"], b["cycle_s"]],
                   "R": [r["state"], r["failure"], r["cycle_s"]]} for b, r in pairs],
    }
    return out


def load(root: Path) -> list[dict]:
    rows = []
    for d in sorted(root.iterdir()):
        p = d / "run" / "result.json"
        if NAME.match(d.name) and p.is_file():
            rows.append(summarize(d.name, json.loads(p.read_text())))
    return rows


def load_fields(path: Path) -> list[dict]:
    with gzip.open(path, "rt") as f:
        return [summarize(name, r) for name, r in sorted(json.load(f).items())]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path, nargs="?")
    ap.add_argument("--raw-fields", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    if (a.root is None) == (a.raw_fields is None):
        raise SystemExit("give RAW_ROOT or --raw-fields")
    res = judge(load(a.root) if a.root is not None else load_fields(a.raw_fields))
    a.out.write_text(json.dumps(res, indent=1, allow_nan=False) + "\n")
    print(res.get("verdict"))
    for k, v in res.get("criteria", {}).items():
        print(k, "pass" if v.get("pass") else "FAIL", "(underexposed)" if v.get("underexposed") else "")


if __name__ == "__main__":
    main()
