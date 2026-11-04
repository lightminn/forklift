#!/usr/bin/env python3
"""The F1 v2 operating regression batch read by the rules fixed before the results (docs/plans/
2026-11-04-f1-v2-registration.md section 4): per scene (line_2, aisle_rack; seeds 3-17), v1's rules (complete F1 >= O - 1,
safety F1 <= O, slip F1 <= O, at least 12 valid pairs of 15), the two mechanism failures counted apart (line_2 round-1
delivery match rejections, aisle_rack transport cusp replan budget exhausted -- an endpoint_heading miss at a cusp
on the path with the budget spent; each F1 <= O + 1), at least 10 pairs lifted in both arms, the new F1 ends, and
the physical settle runs read both ways (left out as invalid, and counted as that arm's failure: verdicts that differ,
or two or more of them in one arm of a scene, make the scene unjudgeable).

  PYTHONPATH=<snapshot>/src python3 tools/f1v2_regression_judge.py BATCH --json OUT.json

Reads BATCH/f1_<arm>_<scene>_s<seed>/run/result.json (and run/delivery_match_1.npz for the round-1 stop distance and the
SLAM along-track error) with the bundle records BATCH/b_*/results.jsonl; the run reading, validity, the _r2 rerun pair,
the arm identity and the start-failure rules are v1's (tools/f1_tip_mass_judge.py, amendment 3).
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

import f1_tip_mass_judge as V1

ARMS = V1.ARMS
SCENES = ("line_2", "aisle_rack")
SEEDS = tuple(range(3, 18))
EXPECTED_KEYS = [f"{sc}_s{s}" for sc in SCENES for s in SEEDS]
MIN_VALID = 12
MIN_LIFTED = 10  # amendment 2: v1's both-lifted minimum (8 of 12) carried to 15 pairs
MECHANISM_SLACK = 1
MECHANISM = {"line_2": "round1_rejected", "aisle_rack": "cusp_budget_exhausted"}
SETTLE_REASONS = ("Truck spawn displaced", "Invalid pallet spawn after settling")
SETTLE_ONE_ARM_HOLD = 2


def registered(name: str) -> bool:
    try:
        arm, key, _ = V1.split_name(name)
    except ValueError:
        return False
    return arm in ARMS and key in EXPECTED_KEYS


def scene_of(key: str) -> str:
    return key.rsplit("_s", 1)[0]


def _compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def _invert(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def round1_stop(run_dir: Path) -> dict | None:
    """The round-1 match's truth stop in the delivery reference frame: the stop distance (the reference rear ahead of
    the truth rear) and the SLAM estimate's along/lateral error (the line_2 information diagnosis' measures)."""
    path = run_dir / "delivery_match_1.npz"
    if not path.exists():
        return None
    z = np.load(path, allow_pickle=True)
    ref = tuple(float(v) for v in z["reference_rear"])
    truth = _compose(_invert(ref), tuple(float(v) for v in z["truth_rear"]))
    est = _compose(_invert(ref), tuple(float(v) for v in z["estimate"]))
    return {"stop_distance_m": -truth[0], "slam_along_error_m": est[0] - truth[0],
            "slam_lateral_error_m": est[1] - truth[1]}


def mechanism_fields(d: dict | None, run_dir: Path) -> dict:
    if d is None:
        return {}
    rounds = (d.get("delivery_docking") or {}).get("rounds") or []
    first = rounds[0] if rounds else None
    phases = {s.get("phase") for s in d.get("samples") or []} | {d.get("phase")}
    budget = int((d.get("arguments") or {}).get("transport_cusp_replans", 2))
    tf = d.get("tracking_failure") or {}
    tracking = tf.get("tracking") or {}
    reason = str(d.get("failure_reason") or "")
    return {"round1_reached": first is not None,
            "round1_rejected": first is not None and not first.get("accepted"),
            "round1_reason": None if first is None else first.get("reason"),
            "round1_min_information": None if first is None else min(
                (s.get("min_information") for s in first.get("start_details") or [] if s.get("min_information")
                 is not None), default=None),
            "transport_entered": "transport" in phases,
            "cusp_replans": len(d.get("cusp_replans") or []),
            # the transport's cusp heading miss the core would have replanned had the budget remained (core.py
            # cusp_and_observe_recovery: endpoint_heading at a cusp, on the path), with the budget spent
            "cusp_budget_exhausted": (d.get("phase") == "transport" and tf.get("phase") == "transport"
                                      and tracking.get("failure") == "endpoint_heading"
                                      and bool(tracking.get("at_cusp")) and not tracking.get("off_path")
                                      and len(d.get("cusp_replans") or []) >= budget
                                      # the budget's last replan found a path and the miss came after it -- not a
                                      # last replan that failed to plan (Codex review 2)
                                      and (d.get("cusp_replans") or [{}])[-1].get("status") == "success"
                                      and int(tf.get("loop_step", -1))
                                      > int((d.get("cusp_replans") or [{}])[-1].get("loop_step", 1 << 62))),
            "settle_invalid": reason.startswith(SETTLE_REASONS),
            "round1_stop": round1_stop(run_dir)}


def read_run(root: Path, name: str, bundle: dict) -> dict:
    r = V1.batch_run(root, name, bundle)
    r.pop("_samples", None)
    r.update(mechanism_fields(V1.load(root / "run" / "result.json"), root / "run"))
    if r.get("settle_invalid"):
        # v1 listed the two settle ends as infrastructure; here they are read both ways (_as), so the run's own
        # infrastructure state leaves their prefix out
        r["infra_without_settle"] = bool((bundle.get(name) or {}).get("status") in ("infra_failed", "stalled")
                                         or r.get("overscan_end"))
    return r


def pair_table(runs: list) -> dict:
    """v1's: the first pair, or its rerun pair where the first is broken or missing; a pair whose run never started
    after its retries is out (amendment 3)."""
    first, rerun = {}, {}
    for r in runs:
        _, key, is_rerun = V1.split_name(r["name"])
        (rerun if is_rerun else first).setdefault(key, {})[r["arm"]] = r
    out = {}
    for key in EXPECTED_KEYS:
        pair = first.get(key, {})
        never = any(r.get("start_failure") for r in pair.values())
        out[key] = pair if V1.valid({a: _as(r, "fail") for a, r in pair.items()}) or never or key not in rerun \
            else rerun[key]
    return out


def _as(run: dict, mode: str) -> dict:
    """A run read under one settle mode: 'exclude' makes a settle run invalid, 'fail' keeps it a failed valid run."""
    if run.get("settle_invalid"):
        return {**run, "infra": True if mode == "exclude" else run["infra_without_settle"]}
    return run


def scene_verdict(scene: str, pairs: dict, runs: list, mode: str) -> dict:
    pairs = {k: {a: _as(r, mode) for a, r in v.items()} for k, v in pairs.items()}
    good = {k: v for k, v in pairs.items() if V1.valid(v)}
    every = [_as(r, mode) for r in runs if scene_of(r["key"]) == scene]
    safety = {a: sum(bool(r.get("safety")) for r in every if r["arm"] == a) for a in ARMS}
    lifted = {k: v for k, v in good.items() if v["O"].get("lifted") and v["F1"].get("lifted")}
    complete = {a: sum(bool(v[a].get("complete")) for v in good.values()) for a in ARMS}
    slip = {a: sum(bool(v[a].get("slip_failure")) for v in lifted.values()) for a in ARMS}
    field = MECHANISM[scene]
    mech = {a: sum(bool(v[a].get(field)) for v in good.values()) for a in ARMS}
    reasons = {a: sorted({str(v[a].get("reason")) for v in good.values() if not v[a].get("complete")}) for a in ARMS}
    out = {"mode": mode, "valid_pairs": len(good), "both_lifted_pairs": len(lifted), "complete": complete,
           "safety_failures": safety, "slip_failures": slip, "mechanism": {"field": field, **mech},
           "failure_reasons": reasons, "new_reasons_F1": sorted(set(reasons["F1"]) - set(reasons["O"]))}
    if len(good) < MIN_VALID or len(lifted) < MIN_LIFTED:
        out["verdict"] = "inconclusive"
    elif not (complete["F1"] >= complete["O"] - 1 and safety["F1"] <= safety["O"] and slip["F1"] <= slip["O"]
              and mech["F1"] <= mech["O"] + MECHANISM_SLACK):
        out["verdict"] = "fail"
    else:
        out["verdict"] = "needs_cause" if out["new_reasons_F1"] else "pass"
    return out


def reach_table(pairs: dict) -> dict:
    out = {}
    for field in ("round1_reached", "transport_entered", "complete"):
        cells = {"both": 0, "O_only": 0, "F1_only": 0, "neither": 0}
        for v in pairs.values():
            if not V1.valid(v):
                continue
            o, f = bool(v["O"].get(field)), bool(v["F1"].get(field))
            cells["both" if o and f else "O_only" if o else "F1_only" if f else "neither"] += 1
        out[field] = cells
    return out


def stop_report(pairs: dict) -> dict:
    out = {}
    for a in ARMS:
        rows = [v[a]["round1_stop"] for v in pairs.values() if V1.valid(v) and v[a].get("round1_stop")]
        out[a] = {k: (None if not rows else {"n": len(rows), "mean": float(np.mean([r[k] for r in rows])),
                                              "min": float(min(r[k] for r in rows)),
                                              "max": float(max(r[k] for r in rows))})
                  for k in ("stop_distance_m", "slam_along_error_m", "slam_lateral_error_m")}
    return out


def judge(batch: Path) -> dict:
    bundle = V1.bundle_rows(batch)
    folders = [p for p in sorted(batch.iterdir()) if p.is_dir() and p.name.startswith("f1_") and ".attempt" not in p.name]
    runs = [read_run(p, p.name, bundle) for p in folders if registered(p.name)]
    unregistered = [p.name for p in folders if not registered(p.name)]
    pairs = pair_table(runs)
    used = [r for v in pairs.values() for r in v.values()]
    scenes = {}
    for scene in SCENES:
        sp = {k: v for k, v in pairs.items() if scene_of(k) == scene}
        exclude, fail = (scene_verdict(scene, sp, runs, m) for m in ("exclude", "fail"))
        settle = {a: sum(bool(r.get("settle_invalid")) for r in runs if scene_of(r["key"]) == scene and r["arm"] == a)
                  for a in ARMS}
        held = [f"{a}: {settle[a]} settle runs" for a in ARMS
                if settle[a] >= SETTLE_ONE_ARM_HOLD and settle[ARMS[1 - ARMS.index(a)]] == 0]
        verdict = ("unjudgeable" if exclude["verdict"] != fail["verdict"] or held else exclude["verdict"])
        sp_exclude = {k: {a: _as(r, "exclude") for a, r in v.items()} for k, v in sp.items()}
        scenes[scene] = {"verdict": verdict, "exclude": exclude, "fail": fail, "settle_runs": settle,
                         "settle_hold": held, "reach": reach_table(sp_exclude), "round1_stop": stop_report(sp_exclude)}
    # every run either reading counts (a settle end is counted in the 'fail' reading) shows its arm and scene
    counted = [r for r in used if not _as(r, "fail")["infra"]]
    identity_bad = {r["name"]: r["identity_problems"] for r in counted if r["identity_problems"]}
    hashes = {}
    for r in counted:
        hashes.setdefault(r["arm"], set()).add(json.dumps(r["identity"].get("scene_sha256"), sort_keys=True))
    scene_problems = [f"{a}: {len(v)} scene hashes" for a, v in sorted(hashes.items()) if len(v) != 1]
    if hashes.get("O") and hashes.get("O") == hashes.get("F1"):
        scene_problems.append("the two arms ran the same scene")
    V1_keys, V1.EXPECTED_KEYS = V1.EXPECTED_KEYS, EXPECTED_KEYS  # v1's start-retry report reads its registered list
    try:
        retries = V1.start_retries(batch, runs)
    finally:
        V1.EXPECTED_KEYS = V1_keys
    ran_before = [n for a in ARMS for n in retries[a]["other_earlier_attempts"]]
    hold = any(retries[a]["hold_never_started"] for a in ARMS)
    parts = [s["verdict"] for s in scenes.values()]
    overall = ("inconclusive" if identity_bad or scene_problems or ran_before or hold
               else "unjudgeable" if "unjudgeable" in parts else "fail" if "fail" in parts
               else "inconclusive" if "inconclusive" in parts else "needs_cause" if "needs_cause" in parts
               else "pass")
    return {"overall": overall, "scenes": scenes, "identity_mismatch": identity_bad, "scene_problems": scene_problems,
            "unregistered_folders": unregistered, "start_retries": retries, "earlier_attempts_that_ran": ran_before,
            "runs": runs}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("batch", type=Path)
    p.add_argument("--json", type=Path, required=True)
    args = p.parse_args()
    out = judge(args.batch)
    args.json.write_text(json.dumps(out, indent=1, default=str) + "\n")
    print(json.dumps({"overall": out["overall"], **{s: v["verdict"] for s, v in out["scenes"].items()}}))


if __name__ == "__main__":
    main()
