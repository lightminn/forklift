#!/usr/bin/env python3
"""F2 2x2 probe (docs/plans/2026-11-05-f2-tyre-2x2.md sections 4.3, 4.4 and 6), fixed before any 2x2 result: the
effects and the interaction per state (descriptive), the validity of each measure (determinism, the same-arm reversed
run), the dt condition, and the candidate rule.

  python tools/f2_2x2.py MANIFEST.json --json OUT.json

MANIFEST (paths to the probe records; arms O, F1, O+F2, F1+F2):
  {"static":   {"120": {arm: STATIC_A, ...}, "240": {...}},                                   # M1, plus "<arm>#b"
   "steer":    {"120": {arm: RESULT_A, ...}, "240": {...}},                                   # M2, plus "<arm>#b"
   "cusp":     {"120": {arm: {"a": CUSP, "b": CUSP, "r": CUSP}}, "240": {arm: {"a": ..., "b": ...}}},   # M3
   "delivery": {"120": {arm: {"a": DELIVERY, "b": DELIVERY, "r": DELIVERY}}},                 # M4
   "calibration": CALIBRATION_VERDICT}
STATIC is run/f1v2_static_probe.json, RESULT the steering probe's run/result.json (its "steering_probe"), CUSP and
DELIVERY the F1 v2 probe records. "#b" keys are the second fresh process of M1/M2 (determinism).
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

ARMS = ("O", "F1", "O+F2", "F1+F2")
WHEELS = ("front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel")
LOADS = ("empty", "loaded")
T_N, T_E = 2.0, 1e-3  # the distinguishing floors (N, rad), the steering asymmetry plan's
DT_MIN = {"M1": 4, "M2": 10}
DT_SIGN, DT_SIZE, DT_VANISH = 0.8, 0.5, 0.2  # plan 4.4
CONTRASTS = {"F2|O": ("O+F2", "O"), "F2|F1": ("F1+F2", "F1")}
CANDIDATE_CONTRAST = {"F2": ("O+F2", "O"), "F1+F2": ("F1+F2", "O")}
REV_E, REV_I, REV_GE_M, REV_GE_RAD = 0.0005, 0.05, 1e-4, 1e-4  # the lead's tenth of the margin (F1 v2)
E_MARGIN, GE_M, GE_RAD, LR_MRAD = 0.005, 1e-3, 1e-3, 1.0  # plan 6
JUDGED_CASTS = ("a",) + tuple(f"b{k}" for k in range(8))
BAD_OUTCOMES = ("timeout", "stopped_not_released", "pending")
M3_MIN_SHARE, M4_MIN_POINTS, M2_SEGMENTS = 0.9, 18, 20


def _q(xs):
    a = np.asarray([x for x in xs if x is not None], dtype=float)
    if not len(a):
        return None
    return {"n": int(len(a)), "median": float(np.median(np.abs(a))), "q90": float(np.quantile(np.abs(a), 0.9)),
            "max": float(np.max(np.abs(a))), "positive_share": float(np.mean(a > 0))}


# --- state values per measure ---

def m1_values(static: dict) -> dict:
    """M1: each wheel's floor force per load (N)."""
    return {f"{ld}/{w}": static["loads"][ld]["wheels"][w]["fz"]["mean"] for ld in LOADS for w in WHEELS}


def m2_values(result: dict) -> dict:
    """M2: |e_L - e_R| (rad) per judged steering segment (curvature not 0), from the segment means; e = (command -
    actual) x sign(command)."""
    out = {}
    for seg in result["steering_probe"]["segments"]:
        if seg["kappa_cmd"] == 0.0 or seg["label"] in ("settle", "stop"):
            continue
        e = {}
        for s_ in ("FL", "FR"):
            cmd, q = seg[f"target_{s_}"], seg[f"q_{s_}"]
            e[s_] = (cmd - q) * math.copysign(1.0, cmd)
        out[f"{seg['label']}/{seg['kappa_cmd']:+.2f}/{seg['segment']}"] = abs(e["FL"] - e["FR"])
    return out


def effects(values: dict) -> dict:
    """Per state: F1 = F1 - O, F2 = (O+F2) - O, F1|F2 = (F1+F2) - (O+F2), I = ((F1+F2) - F1) - ((O+F2) - O)."""
    states = sorted(set.intersection(*(set(values[a]) for a in ARMS)))
    out = {"F1": {}, "F2": {}, "F1|F2": {}, "I": {}}
    for s in states:
        o, f1, of2, f1f2 = (values[a][s] for a in ARMS)
        if None in (o, f1, of2, f1f2):
            continue
        out["F1"][s], out["F2"][s], out["F1|F2"][s] = f1 - o, of2 - o, f1f2 - of2
        out["I"][s] = (f1f2 - f1) - (of2 - o)
    return out


# --- validity ---

def cusp_order_sensitive(fwd: dict, rev: dict) -> list:
    f = {t["id"]: t for t in fwd["trials"]}
    r = {t["id"]: t for t in rev["trials"]}
    bad = []
    for i in f:
        a, b = f[i], r.get(i)
        if b is None:
            bad.append(i)
            continue
        reach = lambda t: (t["events"] or {}).get("first_at_endpoint") is not None  # noqa: E731
        exceed = lambda t: (t["events"] or {}).get("first_exceed_after_endpoint") is not None  # noqa: E731
        if (a["outcome"] != b["outcome"] or reach(a) != reach(b) or exceed(a) != exceed(b)
                or (a["e_stop_rad"] is None) != (b["e_stop_rad"] is None)
                or (a["e_stop_rad"] is not None and abs(a["e_stop_rad"] - b["e_stop_rad"]) > REV_E)):
            bad.append(i)
    return bad


def delivery_order_sensitive(fwd: dict, rev: dict) -> list:
    f = {p["label"]: p for p in fwd["points"]}
    r = {p["label"]: p for p in rev["points"]}
    bad = []
    for lab, p in f.items():
        q = r.get(lab)
        if q is None:
            bad.append(lab)
            continue
        for key, ef in p["evals"].items():
            er = q["evals"].get(key)
            if er is None or abs(ef["min_information"] - er["min_information"]) > REV_I or ef["accepted"] != er["accepted"]:
                bad.append(lab)
                break
            if ef["accepted"]:
                ge = [ef["goal_error_vs_truth"][i] - er["goal_error_vs_truth"][i] for i in range(3)]
                if abs(ge[0]) > REV_GE_M or abs(ge[1]) > REV_GE_M or abs(ge[2]) > REV_GE_RAD:
                    bad.append(lab)
                    break
    return bad


def validity(manifest: dict, load) -> dict:
    """Per measure and arm: determinism (two fresh processes equal) and, for M3/M4, the order-sensitive tag."""
    out = {}
    for m, key, field in (("M1", "static", None), ("M2", "steer", "steering_probe")):
        for dt, arms in manifest.get(key, {}).items():
            for arm in ARMS:
                a, b = arms.get(arm), arms.get(f"{arm}#b")
                # the physics record only (a result.json also holds wall-clock times)
                pick = (lambda r: r) if field is None else (lambda r: r[field])  # noqa: E731
                ok = a is not None and b is not None and pick(load(a)) == pick(load(b))
                out.setdefault(m, {}).setdefault(dt, {})[arm] = {"deterministic": ok, "order_sensitive": None}
    for m, key, rec, sens in (("M3", "cusp", "trials", cusp_order_sensitive),
                              ("M4", "delivery", "points", delivery_order_sensitive)):
        for dt, arms in manifest.get(key, {}).items():
            for arm in ARMS:
                runs = arms.get(arm) or {}
                a, b = runs.get("a"), runs.get("b")
                det = a is not None and b is not None and load(a)[rec] == load(b)[rec]
                tag = None
                if runs.get("r") is not None and a is not None:
                    tag = sens(load(a), load(runs["r"]))
                out.setdefault(m, {}).setdefault(dt, {})[arm] = {
                    "deterministic": det, "order_sensitive": None if tag is None else bool(tag),
                    "order_sensitive_states": None if tag is None else tag[:20]}
    return out


def usable(valid: dict, m: str, dt: str, arms) -> bool:
    rows = valid.get(m, {}).get(dt, {})
    return all(a in rows and rows[a]["deterministic"] and rows[a]["order_sensitive"] is not True for a in arms)


# --- dt condition (plan 4.4) ---

def dt_check(v120: dict, v240: dict, pair, floor: float, minimum: int) -> dict:
    a, b = pair
    states = sorted(set(v120[a]) & set(v120[b]) & set(v240[a]) & set(v240[b]))
    c120 = {s: v120[a][s] - v120[b][s] for s in states}
    c240 = {s: v240[a][s] - v240[b][s] for s in states}
    both = [s for s in states if abs(c120[s]) >= floor and abs(c240[s]) >= floor]
    d120 = [s for s in states if abs(c120[s]) >= floor]
    one = [s for s in states if (abs(c120[s]) >= floor) != (abs(c240[s]) >= floor)]
    out = {"states": len(states), "distinguishable_both": len(both), "distinguishable_120": len(d120),
           "one_dt_only": len(one)}
    if len(both) < minimum:
        out["verdict"] = "undecided"
        return out
    same = sum(math.copysign(1, c120[s]) == math.copysign(1, c240[s]) for s in both) / len(both)
    size = sum(abs(c240[s] - c120[s]) <= DT_SIZE * abs(c120[s]) for s in both) / len(both)
    vanish = len(one) / len(d120) if d120 else 0.0
    out.update(sign_share=same, size_share=size, vanish_share=vanish)
    out["verdict"] = "pass" if same >= DT_SIGN and size >= DT_SIGN and vanish <= DT_VANISH else "dt-dependent"
    return out


def dt_condition(m1: dict, m2: dict) -> dict:
    """m1/m2 = {dt: {arm: values}}; the three contrasts (F2 under O, F2 under F1, each candidate vs O)."""
    out = {}
    pairs = dict(CONTRASTS)
    pairs.update({f"cand:{k}": v for k, v in CANDIDATE_CONTRAST.items()})
    for name, pair in pairs.items():
        out[name] = {"M1": dt_check(m1["120"], m1["240"], pair, T_N, DT_MIN["M1"]) if "240" in m1 else
                     {"verdict": "undecided"},
                     "M2": dt_check(m2["120"], m2["240"], pair, T_E, DT_MIN["M2"]) if "240" in m2 else
                     {"verdict": "undecided"}}
    return out


def candidate_dt_ok(dt: dict, candidate: str) -> bool:
    needed = ["F2|O", f"cand:{candidate}"] + (["F2|F1"] if candidate == "F1+F2" else [])
    return all(dt[n][m]["verdict"] == "pass" for n in needed for m in ("M1", "M2"))


# --- candidate rule (plan 6) ---

def cusp_worse(a: dict, b: dict) -> int:
    """+1 when trial a ended worse than trial b, -1 when better, 0 otherwise (plan 6 M3)."""
    def bad(t):
        ev = t["events"] or {}
        return (ev.get("first_at_endpoint") is None or t["outcome"] in BAD_OUTCOMES
                or str(t["outcome"]).startswith("failed_") or ev.get("first_exceed_after_endpoint") is not None)
    ba, bb = bad(a), bad(b)
    if ba != bb:
        return 1 if ba else -1
    if a["outcome"] == b["outcome"] and a["e_stop_rad"] is not None and b["e_stop_rad"] is not None:
        d = abs(a["e_stop_rad"]) - abs(b["e_stop_rad"])
        if abs(d) > E_MARGIN:
            return 1 if d > 0 else -1
    return 0


def m3_counts(x: dict, y: dict) -> dict:
    xs = {t["id"]: t for t in x["trials"]}
    ys = {t["id"]: t for t in y["trials"]}
    ids = sorted(set(xs) & set(ys))
    valid = [i for i in ids if xs[i]["settle"]["converged"] and ys[i]["settle"]["converged"]
             and xs[i]["wheel_load_ok"] and ys[i]["wheel_load_ok"]
             and xs[i]["foreign_contacts"] == 0 and ys[i]["foreign_contacts"] == 0]
    w = [cusp_worse(xs[i], ys[i]) for i in valid]
    return {"trials": len(ids), "valid": len(valid), "worse": w.count(1), "better": w.count(-1)}


def m4_counts(x: dict, y: dict) -> dict:
    xs = {p["label"]: p for p in x["points"]}
    ys = {p["label"]: p for p in y["points"]}
    labels = sorted(set(xs) & set(ys))
    valid = [lab for lab in labels if xs[lab]["pose_ok"] and ys[lab]["pose_ok"]
             and xs[lab]["settle"]["converged"] and ys[lab]["settle"]["converged"]]
    worse = better = 0
    for lab in valid:
        for key, ex in xs[lab]["evals"].items():
            if key.split("/")[0] not in JUDGED_CASTS:
                continue
            ey = ys[lab]["evals"][key]
            if ex["accepted"] != ey["accepted"]:
                worse += not ex["accepted"]
                better += not ey["accepted"]
            elif ex["accepted"]:
                d = [abs(ex["goal_error_vs_truth"][i]) - abs(ey["goal_error_vs_truth"][i]) for i in range(3)]
                if d[0] > GE_M or d[1] > GE_M or d[2] > GE_RAD:
                    worse += 1
                elif d[0] < -GE_M or d[1] < -GE_M or d[2] < -GE_RAD:
                    better += 1
    return {"points": len(labels), "valid": len(valid), "worse": worse, "better": better}


def m2_counts(vx: dict, vy: dict) -> dict:
    states = sorted(set(vx) & set(vy))
    d = [(vx[s] - vy[s]) * 1e3 for s in states]
    return {"segments": len(states), "worse": sum(x > LR_MRAD for x in d), "better": sum(x < -LR_MRAD for x in d)}


def candidate(manifest: dict, load, calib: dict, valid: dict, dt: dict, m2: dict) -> dict:
    out = {"preconditions": [], "self_degradation": {}, "f1_under_f2": {}}
    if calib.get("chosen_k_mat") is None:
        out["preconditions"].append("stage C chose no material value")
    cusp = manifest["cusp"]["120"]
    dlv = manifest["delivery"]["120"]
    for m, dts in (("M1", ("120", "240")), ("M2", ("120", "240")), ("M3", ("120",)), ("M4", ("120",))):
        for d in dts:
            if not usable(valid, m, d, ARMS):
                out["preconditions"].append(f"{m} at {d} Hz not usable (determinism or order-sensitive)")
    pairs = {"f1_under_f2": ("F1+F2", "O+F2"), "self_degradation": ("O+F2", "O")}
    for name, (x, y) in pairs.items():
        c3 = m3_counts(load(cusp[x]["a"]), load(cusp[y]["a"]))
        c4 = m4_counts(load(dlv[x]["a"]), load(dlv[y]["a"]))
        c2 = m2_counts(m2["120"][x], m2["120"][y])
        out[name] = {"M3": c3, "M4": c4, "M2": c2,
                     "worse_any": c3["worse"] > c3["better"] or c4["worse"] > c4["better"] or c2["worse"] > c2["better"]}
        if name == "f1_under_f2":
            if c3["valid"] < M3_MIN_SHARE * c3["trials"]:
                out["preconditions"].append(f"M3 valid {c3['valid']} of {c3['trials']}")
            if c4["valid"] < M4_MIN_POINTS:
                out["preconditions"].append(f"M4 valid {c4['valid']} of {c4['points']}")
            if c2["segments"] != M2_SEGMENTS:
                out["preconditions"].append(f"M2 segments {c2['segments']} of {M2_SEGMENTS}")
    pick = "F2" if out["f1_under_f2"]["worse_any"] else "F1+F2"
    if not candidate_dt_ok(dt, pick):
        out["preconditions"].append(f"dt condition not passed for {pick}")
    out["escalate_self_degradation"] = out["self_degradation"]["worse_any"]
    out["candidate"] = None if out["preconditions"] else pick
    out["verdict"] = ("no selection (to the lead)" if out["preconditions"] else
                      f"{pick}" + (" -- F2 itself worsens a measure: to the lead before (나)"
                                   if out["escalate_self_degradation"] else ""))
    return out


def run(manifest: dict, load=lambda p: json.loads(Path(p).read_text())) -> dict:
    valid = validity(manifest, load)
    m1 = {dt: {a: m1_values(load(p)) for a, p in arms.items() if a in ARMS} for dt, arms in manifest["static"].items()}
    m2 = {dt: {a: m2_values(load(p)) for a, p in arms.items() if a in ARMS} for dt, arms in manifest["steer"].items()}
    desc = {"M1": {dt: {k: _q(list(v.values())) for k, v in effects(m1[dt]).items()} for dt in m1},
            "M2": {dt: {k: _q(list(v.values())) for k, v in effects(m2[dt]).items()} for dt in m2}}
    for m, key, getter in (("M3", "cusp", lambda r: {t["id"]: t["e_stop_rad"] for t in r["trials"]}),
                           ("M4", "delivery", lambda r: {f"{p['label']}/{k}": e["min_information"]
                                                         for p in r["points"] for k, e in p["evals"].items()})):
        desc[m] = {}
        for dt, arms in manifest.get(key, {}).items():
            vals = {a: getter(load(arms[a]["a"])) for a in ARMS if a in arms}
            if len(vals) == len(ARMS):
                abs_vals = {a: {s: (None if x is None else abs(x)) for s, x in v.items()} for a, v in vals.items()}
                desc[m][dt] = {k: _q(list(v.values())) for k, v in effects(abs_vals).items()}
    dt = dt_condition(m1, m2)
    calib = load(manifest["calibration"]) if manifest.get("calibration") else {}
    cand = candidate(manifest, load, calib, valid, dt, m2)
    return {"validity": valid, "effects": desc, "dt_condition": dt, "candidate": cand}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("manifest", type=Path)
    p.add_argument("--json", type=Path, required=True)
    args = p.parse_args()
    out = run(json.loads(args.manifest.read_text()))
    args.json.write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({"candidate": out["candidate"]["candidate"], "verdict": out["candidate"]["verdict"],
                      "preconditions": out["candidate"]["preconditions"]}, indent=1))


if __name__ == "__main__":
    main()
