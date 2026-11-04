#!/usr/bin/env python3
"""F1 v2 controlled probe judge (docs/plans/2026-11-04-f1-v2-registration.md sections 2-3, amendments 1-2), fixed before
any full probe result.

  python tools/f1v2_probe_judge.py delivery --stage0 S0.json --v1-stops v1_stops.json \
      --o A.json B.json --o-reverse R.json --f1 A.json B.json --f1-reverse R.json --json OUT.json
  python tools/f1v2_probe_judge.py cusp --o A.json B.json --o-off OFF.json --f1 A.json B.json --f1-off OFF.json \
      --json OUT.json

The inputs are the probe records (f1v2_delivery_probe.json, f1v2_cusp_probe.json). The verdict is "unjudgeable" when any
precondition the plan names fails (it then reports what it could compare as well), else "different" when any pair
breaks a margin, else "same". Every pair over a margin is listed with what the lead asked to see (plan section 0).
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

# (i') -- plan section 2 and amendment 1
DELTA_I = 0.5
GOAL_ERROR_M = 0.001
GOAL_ERROR_RAD = 0.001
JUDGED_CASTS = ("a",) + tuple(f"b{k}" for k in range(8))
PATH_CASTS = ("c", "d")
ESTIMATES = ("e0", "e1")
OP_LIFT_M, OP_BASE_Z_M, OP_TILT_RAD = 1.0e-3, 0.5e-3, 2e-6
# the five representative points the reversed run must reproduce (fixed before the results: both ends and the centre
# of round 1, both corners of round 2)
REPRESENTATIVE = ("r1_d1.38_y-9.000_w0.00", "r1_d1.50_y-8.955_w0.02", "r1_d1.60_y-8.910_w0.04", "r2_d0.50_l-0.08",
                  "r2_d0.64_l+0.08")
# (ii') -- plan section 3 and amendment 2
DELTA_E = 0.005


def _q(values, qs=(0.5, 0.9, 0.99, 1.0)):
    v = np.asarray([x for x in values if x is not None], dtype=float)
    if not len(v):
        return None
    return {"n": int(len(v)), **{f"q{int(q * 100)}": float(np.quantile(v, q)) for q in qs}, "mean": float(v.mean())}


def _tilt(q) -> float:
    w, x, y, z = (float(v) for v in q)
    return math.acos(float(np.clip(1 - 2 * (x * x + y * y), -1, 1)))


def _min_start(ev: dict) -> dict:
    return min(ev["starts"], key=lambda s: s["information"])


# --- (i') delivery ---

def delivery(stage0: dict, v1_stops: list, o_runs: list, o_reverse: dict, f1_runs: list, f1_reverse: dict) -> dict:
    unjudgeable, notes = [], {}
    if not stage0.get("all_bit_identical"):
        unjudgeable.append("stage 0: dock() did not reproduce the recorded round 1 bit for bit")
    for arm, runs, rev in (("O", o_runs, o_reverse), ("F1", f1_runs, f1_reverse)):
        if len(runs) != 2 or runs[0]["points"] != runs[1]["points"]:
            unjudgeable.append(f"determinism: the {arm} arm's two fresh runs differ")
        rev_points = {p["label"]: p for p in rev["points"]}
        fwd_points = {p["label"]: p for p in runs[0]["points"]}
        bad = [lab for lab in REPRESENTATIVE
               if lab not in rev_points or rev_points[lab]["evals"] != fwd_points.get(lab, {}).get("evals")
               or rev_points[lab]["state"] != fwd_points.get(lab, {}).get("state")]
        notes[f"reverse_{arm}"] = {"representative": list(REPRESENTATIVE), "differ": bad}
        if bad:
            unjudgeable.append(f"determinism: the {arm} arm's reversed run differs at {bad}")
    o, f1 = ({p["label"]: p for p in runs[0]["points"]} for runs in (o_runs, f1_runs))
    # the operating comparison: each v1 stop against the same arm's probe state at that laser pose
    op_rows = []
    for stop in v1_stops:
        arm_points = o if stop["arm"] == "O" else f1
        p = arm_points.get(stop["label"])
        if p is None:
            op_rows.append({"label": stop["label"], "ok": False, "why": "missing"})
            continue
        rec = stop["recorded"]
        d_lift = p["state"]["lift_m"] - rec["lift_m"]
        d_z = p["state"]["base"][2] - rec["base_z_m"]
        d_tilt = _tilt(p["state"]["q"]) - rec["base_tilt_rad"]
        ok = (abs(d_lift) <= OP_LIFT_M and abs(d_z) <= OP_BASE_Z_M and abs(d_tilt) <= OP_TILT_RAD
              and p["pose_ok"] and p["settle"]["converged"])
        op_rows.append({"label": stop["label"], "arm": stop["arm"], "d_lift_m": d_lift, "d_base_z_m": d_z,
                        "d_tilt_rad": d_tilt, "pose_ok": p["pose_ok"], "settled": p["settle"]["converged"], "ok": ok})
    if not all(r["ok"] for r in op_rows):
        unjudgeable.append("operating comparison: a v1 stop is not reproduced within the tolerances")
    grid = sorted(set(o) & set(f1) - {s["label"] for s in v1_stops})
    missing = sorted((set(o) ^ set(f1)) - {s["label"] for s in v1_stops})
    if missing:
        unjudgeable.append(f"grid points in one arm only: {missing}")
    pose_bad = [lab for lab in grid if not (o[lab]["pose_ok"] and f1[lab]["pose_ok"])]
    if pose_bad:
        unjudgeable.append(f"laser pose out of tolerance (amendment 1) at {pose_bad}")
    settle_one = [lab for lab in grid if o[lab]["settle"]["converged"] != f1[lab]["settle"]["converged"]]
    settle_both = [lab for lab in grid if not (o[lab]["settle"]["converged"] or f1[lab]["settle"]["converged"])]
    if settle_both:
        unjudgeable.append(f"neither arm settled at {settle_both}")
    over, pairs = [], 0
    di_all = []
    for lab in grid:
        for cast in JUDGED_CASTS:
            for est in ESTIMATES:
                key = f"{cast}/{est}"
                eo, ef = o[lab]["evals"][key], f1[lab]["evals"][key]
                pairs += 1
                di = ef["min_information"] - eo["min_information"]
                di_all.append(abs(di))
                why = []
                if abs(di) > DELTA_I:
                    why.append("delta_I")
                if eo["accepted"] != ef["accepted"]:
                    why.append("accept_crossing_O_only" if eo["accepted"] else "accept_crossing_F1_only")
                ge = None
                if eo["accepted"] and ef["accepted"]:
                    ge = [ef["goal_error_vs_truth"][i] - eo["goal_error_vs_truth"][i] for i in range(3)]
                    if abs(ge[0]) > GOAL_ERROR_M or abs(ge[1]) > GOAL_ERROR_M or abs(ge[2]) > GOAL_ERROR_RAD:
                        why.append("correction_error")
                if why:
                    over.append({"label": lab, "d_m": o[lab]["d_m"], "laser_target": o[lab]["target_laser"],
                                 "cast": cast, "estimate": est, "why": why, "delta_I": di,
                                 "I_O": eo["min_information"], "I_F1": ef["min_information"],
                                 "accepted_O": eo["accepted"], "accepted_F1": ef["accepted"],
                                 "beams_O": eo["live_beams"], "beams_F1": ef["live_beams"],
                                 "beams_diff": ef["live_beams"] - eo["live_beams"],
                                 "inliers_diff": _min_start(ef)["inliers"] - _min_start(eo)["inliers"],
                                 "correction_error_diff": ge})
    # path report: (c) the orientation path, (d) the height path, and the body state differences
    path = {}
    for cast in ("a",) + PATH_CASTS:
        path[cast] = _q([abs(f1[lab]["evals"][f"{cast}/e0"]["min_information"]
                             - o[lab]["evals"][f"{cast}/e0"]["min_information"]) for lab in grid])
    dpitch = [f1[lab]["state"]["pitch_rad"] - o[lab]["state"]["pitch_rad"] for lab in grid]
    dz = [f1[lab]["state"]["base"][2] - o[lab]["state"]["base"][2] for lab in grid]
    dia = [f1[lab]["evals"]["a/e0"]["min_information"] - o[lab]["evals"]["a/e0"]["min_information"] for lab in grid]
    corr = {}
    for name, x in (("pitch", dpitch), ("base_z", dz)):
        corr[name] = (float(np.corrcoef(x, dia)[0, 1]) if len(grid) > 2 and np.std(x) > 0 and np.std(dia) > 0
                      else None)
    curve = {}
    for arm, pts in (("O", o), ("F1", f1)):
        by_d = {}
        for lab in grid:
            if lab.startswith("r1_"):
                by_d.setdefault(pts[lab]["d_m"], []).append(pts[lab]["evals"]["a/e0"]["min_information"])
        curve[arm] = {f"{d:.2f}": float(np.mean(v)) for d, v in sorted(by_d.items())}
    different = bool(over) or bool(settle_one)
    verdict = "unjudgeable" if unjudgeable else ("different" if different else "same")
    return {"probe": "delivery", "verdict": verdict, "unjudgeable": unjudgeable, "grid_points": len(grid),
            "pairs": pairs, "settled_one_arm_only": settle_one,
            "over": over, "over_count": len(over), "abs_delta_I": _q(di_all),
            "over_abs_delta_I": _q([abs(r["delta_I"]) for r in over]),
            "over_beams_diff": _q([abs(r["beams_diff"]) for r in over]),
            "operating_comparison": op_rows, "path_abs_delta_I_e0": path,
            "delta_I_correlation": corr, "information_by_distance_a_e0": curve, "determinism": notes,
            "body_state_diff": {"pitch_rad": _q([abs(v) for v in dpitch]), "base_z_m": _q([abs(v) for v in dz])}}


# --- (ii') cusp ---

def _exceeds(t: dict) -> bool:
    return (t.get("events") or {}).get("first_exceed_after_endpoint") is not None


def cusp(o_runs: list, o_off: dict, f1_runs: list, f1_off: dict) -> dict:
    unjudgeable = []
    for arm, runs, off in (("O", o_runs, o_off), ("F1", f1_runs, f1_off)):
        if len(runs) != 2 or runs[0]["trials"] != runs[1]["trials"]:
            unjudgeable.append(f"determinism: the {arm} arm's two fresh runs differ")
        on = {t["id"]: t for t in runs[0]["trials"]}
        offd = {t["id"]: t for t in off["trials"]}
        bad = [i for i in on if i not in offd or any(on[i][k] != offd[i][k] for k in
                                                       ("trajectory_sha256", "outcome", "e_stop_rad", "stop", "ticks"))]
        if bad or set(offd) - set(on):
            unjudgeable.append(f"instrumentation: the {arm} arm's record on/off runs differ at {len(bad)} trials")
    o, f1 = ({t["id"]: t for t in runs[0]["trials"]} for runs in (o_runs, f1_runs))
    ids = sorted(set(o) & set(f1))
    if set(o) ^ set(f1):
        unjudgeable.append(f"trials in one arm only: {sorted(set(o) ^ set(f1))[:10]}")
    load_bad = [i for i in ids if not (o[i]["wheel_load_ok"] and f1[i]["wheel_load_ok"])]
    if load_bad:
        unjudgeable.append(f"wheel load check failed at {len(load_bad)} trials")
    rows, over = [], []
    both_unreached, both_no_stop, settle_both, contact_both = [], [], [], []
    for i in ids:
        a, b = o[i], f1[i]
        why = []
        if a["settle"]["converged"] != b["settle"]["converged"]:
            why.append("settled_one_arm_only")
        elif not a["settle"]["converged"]:
            settle_both.append(i)
        ca, cb = a["foreign_contacts"] > 0, b["foreign_contacts"] > 0
        if ca and cb:
            contact_both.append(i)
        elif ca or cb:
            why.append("foreign_contact_one_arm_only")
        ra = (a["events"] or {}).get("first_at_endpoint") is not None
        rb = (b["events"] or {}).get("first_at_endpoint") is not None
        if not (ra or rb):
            both_unreached.append(i)
        elif ra != rb:
            why.append("endpoint_reached_one_arm_only")
        sa, sb = a["e_stop_rad"] is not None, b["e_stop_rad"] is not None
        de = None
        if not (sa or sb):
            both_no_stop.append(i)
        elif sa != sb:
            why.append("stop_one_arm_only")
        else:
            de = abs(b["e_stop_rad"]) - abs(a["e_stop_rad"])
            if abs(de) > DELTA_E:
                why.append("delta_e_stop")
        if a["outcome"] != b["outcome"]:
            why.append("outcome")
        xa, xb = _exceeds(a), _exceeds(b)
        if xa != xb:
            why.append("exceed_crossing_F1_only" if xb else "exceed_crossing_O_only")
        row = {"id": i, "version": a["version"], "leg": a["leg"], "config": a["config"], "load": a["load"],
               "e_stop_O": a["e_stop_rad"], "e_stop_F1": b["e_stop_rad"], "delta_abs_e": de,
               "outcome_O": a["outcome"], "outcome_F1": b["outcome"],
               "stop_t_O": (a["stop"] or {}).get("t_s"), "stop_t_F1": (b["stop"] or {}).get("t_s"),
               "exceed_O": xa, "exceed_F1": xb, "why": why}
        rows.append(row)
        if why:
            over.append(row)
    for name, lst in (("neither arm reached the endpoint judgement", both_unreached),
                      ("neither arm confirmed a stop", both_no_stop), ("neither arm settled", settle_both),
                      ("both arms touched something besides the floor", contact_both)):
        if lst:
            unjudgeable.append(f"{name} at {len(lst)} trials: {lst[:10]}")
    report = {}
    for config in ("transport", "observe"):
        for load in ("loaded", "empty"):
            sel = [r for r in rows if r["config"] == config and r["load"] == load]
            n = len(sel)
            entry = {"legs": n, "delta_abs_e": _q([abs(r["delta_abs_e"]) for r in sel if r["delta_abs_e"] is not None])}
            for arm, key, xkey in (("O", "e_stop_O", "exceed_O"), ("F1", "e_stop_F1", "exceed_F1")):
                entry[arm] = {"abs_e_stop": _q([abs(r[key]) for r in sel if r[key] is not None]),
                              "exceed_rate": (sum(r[xkey] for r in sel) / n) if n else None,
                              "outcomes": {k: sum(r[f"outcome_{arm}"] == k for r in sel)
                                           for k in sorted({r[f"outcome_{arm}"] for r in sel})}}
            report[f"{config}/{load}"] = entry
    entry_vs_stop = {}
    for arm, d in (("O", o), ("F1", f1)):
        diffs = []
        for i in ids:
            ev = (d[i]["events"] or {}).get("first_at_endpoint")
            if ev is not None and d[i]["e_stop_rad"] is not None:
                diffs.append(abs(d[i]["e_stop_rad"] - ev["yaw_error_rad"]))
        entry_vs_stop[arm] = _q(diffs)
    verdict = "unjudgeable" if unjudgeable else ("different" if over else "same")
    return {"probe": "cusp", "verdict": verdict, "unjudgeable": unjudgeable, "trials": len(ids), "over": over,
            "over_count": len(over), "over_delta_abs_e": _q([abs(r["delta_abs_e"]) for r in over
                                                             if r["delta_abs_e"] is not None]),
            "by_combination": report, "entry_vs_stop_abs": entry_vs_stop,
            "start_offset_m": {arm: _q([d[i]["start_offset_m"] for i in ids]) for arm, d in (("O", o), ("F1", f1))}}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = p.add_subparsers(dest="probe", required=True)
    d = sub.add_parser("delivery")
    d.add_argument("--stage0", type=Path, required=True)
    d.add_argument("--v1-stops", type=Path, required=True)
    d.add_argument("--o", type=Path, nargs=2, required=True)
    d.add_argument("--o-reverse", type=Path, required=True)
    d.add_argument("--f1", type=Path, nargs=2, required=True)
    d.add_argument("--f1-reverse", type=Path, required=True)
    d.add_argument("--json", type=Path, required=True)
    c = sub.add_parser("cusp")
    c.add_argument("--o", type=Path, nargs=2, required=True)
    c.add_argument("--o-off", type=Path, required=True)
    c.add_argument("--f1", type=Path, nargs=2, required=True)
    c.add_argument("--f1-off", type=Path, required=True)
    c.add_argument("--json", type=Path, required=True)
    args = p.parse_args()

    def load(path):
        return json.loads(Path(path).read_text())

    if args.probe == "delivery":
        out = delivery(load(args.stage0), load(args.v1_stops), [load(x) for x in args.o], load(args.o_reverse),
                       [load(x) for x in args.f1], load(args.f1_reverse))
    else:
        out = cusp([load(x) for x in args.o], load(args.o_off), [load(x) for x in args.f1], load(args.f1_off))
    args.json.write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({k: out[k] for k in ("probe", "verdict", "unjudgeable", "over_count")}, indent=1))


if __name__ == "__main__":
    main()
