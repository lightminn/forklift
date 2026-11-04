#!/usr/bin/env python3
"""F2 2x2 probe (docs/plans/2026-11-05-f2-tyre-2x2.md sections 4.3, 4.4 and 6, with amendment 1), fixed before any 2x2
result: the effects and the interaction per state (descriptive), the validity of each measure, the dt condition, and
the candidate rule.

  PYTHONPATH=<snapshot>/src python tools/f2_2x2.py MANIFEST.json --json OUT.json

MANIFEST (arms O, F1, O+F2, F1+F2; "#b" = the second fresh process):
  {"static":   {"120": {arm: STATIC, "<arm>#b": STATIC, "<arm>#r": STATIC_REVERSED}, "240": {arm, "<arm>#b"}},   # M1
   "steer":    {"120": {arm: RUN_DIR, "<arm>#b": RUN_DIR}, "240": {...}},                                        # M2
   "cusp":     {"120": {arm: {"a": CUSP, "b": CUSP, "r": CUSP}}, "240": {arm: {"a": CUSP, "b": CUSP}}},          # M3
   "delivery": {"120": {arm: {"a": DELIVERY, "b": DELIVERY, "r": DELIVERY}}},                                    # M4
   "registry": CUSP_REGISTRY, "calibration": CALIBRATION_VERDICT,
   "plant_gates": {"F2": V4_REPORT, "F1+F2": V4_REPORT}}
STATIC is run/f1v2_static_probe.json, RUN_DIR a steering probe run folder (result.json, steering_probe.npz,
steering_probe_arms.npz -- read with tools/steer_asym_judge.Run, the same windows), CUSP and DELIVERY the probe
records, V4_REPORT {"v4_v2_preserved": bool, ...} from the October plant gates recomputed on M2 (plan 4.4).
"""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "isaac"))

ARMS = ("O", "F1", "O+F2", "F1+F2")
WHEELS = ("front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel")
LOADS = ("empty", "loaded")
T_N, T_E = 2.0, 1e-3  # the distinguishing floors (N, rad), the steering asymmetry plan's
REV_N = T_N / 10  # amendment 1: M1's reversed run, a tenth of the floor
DT_MIN = {"M1": 4, "M2": 10}
DT_SIGN, DT_SIZE, DT_VANISH = 0.8, 0.5, 0.2  # plan 4.4
CONTRASTS = {"F2|O": ("O+F2", "O"), "F2|F1": ("F1+F2", "F1")}
CANDIDATE_CONTRAST = {"F2": ("O+F2", "O"), "F1+F2": ("F1+F2", "O")}
REV_E, REV_I, REV_GE_M, REV_GE_RAD = 0.0005, 0.05, 1e-4, 1e-4  # the lead's tenth of the margin (F1 v2)
E_MARGIN, GE_M, GE_RAD, LR_MRAD = 0.005, 1e-3, 1e-3, 1.0  # plan 6
JUDGED_CASTS = ("a",) + tuple(f"b{k}" for k in range(8))
ALL_CASTS = JUDGED_CASTS + ("c", "d", "e")
ESTIMATES = ("e0", "e1")
BAD_OUTCOMES = ("timeout", "stopped_not_released", "pending")
M3_MIN_SHARE, M4_MIN_POINTS, M2_SEGMENTS = 0.9, 18, 20
M2_LABELS = ("still", "fwd", "rev", "fwd_fast", "rev_fast")
TOP = 5


def _summary(effect: dict) -> dict:
    """Distribution of one effect over states, plus its largest states (plan 4.3)."""
    vals = {s: v for s, v in effect.items() if v is not None}
    if not vals:
        return {"n": 0}
    a = np.asarray(list(vals.values()), dtype=float)
    top = sorted(vals, key=lambda s: -abs(vals[s]))[:TOP]
    return {"n": int(len(a)), "median_abs": float(np.median(np.abs(a))), "q90_abs": float(np.quantile(np.abs(a), 0.9)),
            "max_abs": float(np.max(np.abs(a))), "positive_share": float(np.mean(a > 0)),
            "largest": [[s, float(vals[s])] for s in top]}


# --- state values per measure ---

def m1_values(static: dict) -> dict:
    """M1: each wheel's floor force per load (N)."""
    return {f"{ld}/{w}": static["loads"][ld]["wheels"][w]["fz"]["mean"] for ld in LOADS for w in WHEELS}


def m1_valid(static: dict) -> bool:
    return all(static["loads"][ld]["start"]["converged"] and static["loads"][ld]["foreign_contacts"] == 0
               and abs(static["loads"][ld]["wheel_load_ratio"] - 1.0) <= 0.01 for ld in LOADS)


def read_steer(folder) -> dict:
    """M2 from a steering probe run folder: |e_L - e_R| (rad) per judged segment from steer_asym_judge.Run's windows
    (the last half of the segment, at most 1 s, the command constant, a clean window), None when the window is not
    clean; the digest of the tick record and tyre contacts (determinism)."""
    import steer_asym_judge as S

    run = S.Run(Path(folder))
    values = {}
    for seg in run.segments:
        if seg["kappa_cmd"] == 0.0 or seg["label"] not in M2_LABELS:
            continue
        v = run.values(seg["segment"])
        values[f"{seg['label']}/{seg['kappa_cmd']:+.2f}/{seg['segment']}"] = None if v is None else abs(v["e_LR"])
    h = hashlib.sha256(np.ascontiguousarray(run.x).tobytes())
    h.update(np.ascontiguousarray(run.tyre).tobytes())
    return {"values": values, "digest": h.hexdigest(), "physics_hz": 1.0 / run.dt,
            "arm": arm_identity(run.result)}


ENGINE_KG = {"O": 26.0, "F1": 24.0002}
TYRE_FRICTION = (0.8, 0.7)


def arm_identity(result: dict) -> dict:
    """What a run's result.json shows of its arm: the engine mass and the tyres' compliance and friction."""
    pb = result.get("plant_backend") or {}
    tyres = result.get("plant_tyre_contact") or []
    return {"engine_kg": float(sum(pb.get("masses_kg") or [])) if pb.get("masses_kg") else None,
            "tyres": [(t.get("compliant_stiffness"), t.get("compliant_damping"), t.get("static_friction"),
                       t.get("dynamic_friction")) for t in tyres]}


def arm_problems(arm: str, ident: dict, k_mat) -> list:
    """Why a record is not the registered arm (amendment 1 / Codex review: the dt, order and arm are bound to the
    record, not to the manifest's slot)."""
    out = []
    base = "F1" if arm.startswith("F1") else "O"
    if ident.get("engine_kg") is None or not math.isclose(ident["engine_kg"], ENGINE_KG[base], abs_tol=1e-3):
        out.append(f"engine mass {ident.get('engine_kg')}")
    tyres = ident.get("tyres") or []
    if len(tyres) != 4:
        out.append(f"{len(tyres)} tyres")
    for k, c, fs, fd in tyres:
        if fs is None or fd is None or not (math.isclose(fs, TYRE_FRICTION[0], rel_tol=1e-6)
                                            and math.isclose(fd, TYRE_FRICTION[1], rel_tol=1e-6)):
            out.append(f"friction {fs}/{fd}")
        if arm.endswith("+F2"):
            if k_mat is None or k is None or not math.isclose(k, k_mat, rel_tol=1e-6) or c is None \
                    or not math.isclose(c, k_mat * 310.0 / 400_000.0, rel_tol=1e-6):
                out.append(f"tyre k {k} c {c}, calibrated {k_mat}")
        elif k is not None:
            out.append(f"rigid arm with compliance {k}")
    return sorted(set(out))


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


# --- population (Codex review of P2: bound to the registry and the grid) ---

def expected_cusp(registry: dict, dt: str) -> dict:
    """trial id -> path hash (None: synthetic) for the registered population at a dt (240 Hz: the every-8 subset)."""
    import f1v2_probe_judge as PJ

    want = PJ.expected_trials(registry)
    if dt == "120":
        return want
    running, keep = {}, set()
    for v in registry["versions"]:
        for leg in range(v["cusp_legs"]):
            if len(running) % 8 == 0:
                keep.add(f"{v['id']}/leg{leg}")
            running[(v["id"], leg)] = len(running)
    return {i: h for i, h in want.items() if h is None or i.rsplit("/", 2)[0] in keep}


def cusp_population(rec: dict, want: dict) -> list:
    got = {t["id"]: t for t in rec["trials"]}
    why = []
    if sorted(got) != sorted(want):
        why.append(f"{len(got)} trials, not the {len(want)} registered")
    for i, h in want.items():
        if i in got and h is not None and got[i]["path_sha256"] != h:
            why.append(f"{i}: path not the registered one")
    return why


def delivery_population(rec: dict) -> list:
    import f1v2_probe_core as core

    labels = [p["label"] for p in core.f2_grid((0.0, 0.0, 0.0), 0.4, (0.0, 0.0), 0.0)]
    got = {p["label"]: p for p in rec["points"]}
    why = []
    if sorted(got) != sorted(labels):
        why.append(f"{sorted(set(got) ^ set(labels))[:5]} not the registered 22-point grid")
    for lab, p in got.items():
        missing = [f"{c}/{e}" for c in ALL_CASTS for e in ESTIMATES if f"{c}/{e}" not in p["evals"]]
        if missing:
            why.append(f"{lab}: casts {missing[:3]} missing")
    return why


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


def static_order_sensitive(fwd: dict, rev: dict) -> list:
    vf, vr = m1_values(fwd), m1_values(rev)
    return [s for s in vf if vr.get(s) is None or abs(vf[s] - vr[s]) > REV_N]


def _trial_valid(t: dict) -> bool:
    return t["settle"]["converged"] and t["wheel_load_ok"] and t["foreign_contacts"] == 0


def _point_valid(p: dict) -> bool:
    return p["pose_ok"] and p["settle"]["converged"]


def _result_of(path) -> str:
    """The run's result.json next to a probe record (run/f1v2_*.json)."""
    return str(Path(str(path)).parent / "result.json")


def validity(manifest: dict, load, steer_load) -> dict:
    """Per measure, dt and arm: determinism, physical validity of every record, population, and the reversed run
    (required where amendment 1 registers one: M1, M3 and M4 at 120 Hz) -- a missing reversed run is not a pass."""
    out = {}
    registry = load(manifest["registry"]) if manifest.get("registry") else None
    k_mat = (load(manifest["calibration"]) if manifest.get("calibration") else {}).get("chosen_k_mat")
    hz = {"120": 120.0, "240": 240.0}
    for dt, arms in manifest.get("static", {}).items():
        for arm in ARMS:
            recs = {k: arms.get(f"{arm}{k}") for k in ("", "#b", "#r")}
            a, b = (None if recs[k] is None else load(recs[k]) for k in ("", "#b"))
            r = None if recs["#r"] is None else load(recs["#r"])
            bound = []
            for k, x in (("", a), ("#b", b), ("#r", r)):
                if x is None:
                    continue
                if x.get("physics_hz") != hz[dt]:
                    bound.append(f"{k or 'a'}: physics_hz {x.get('physics_hz')}")
                if x.get("order", "forward") != ("reverse" if k == "#r" else "forward"):
                    bound.append(f"{k or 'a'}: order {x.get('order')}")
                ident = {"engine_kg": x.get("robot_mass_kg"),
                         "tyres": [(t.get("compliant_stiffness"), t.get("compliant_damping"), t.get("static_friction"),
                                    t.get("dynamic_friction")) for t in x.get("tyre_readback") or []]}
                bound += [f"{k or 'a'}: {w}" for w in arm_problems(arm, ident, k_mat)]
            row = {"deterministic": a is not None and b is not None and a == b,
                   "physically_valid": all(x is not None and m1_valid(x) for x in (a, b))
                   and (r is None or m1_valid(r)),
                   "population": bound[:10], "reverse_required": dt == "120"}
            tag = None if (a is None or r is None) else static_order_sensitive(a, r)
            row["order_sensitive"] = None if tag is None else bool(tag)
            out.setdefault("M1", {}).setdefault(dt, {})[arm] = row
    for dt, arms in manifest.get("steer", {}).items():
        for arm in ARMS:
            a, b = (None if arms.get(k) is None else steer_load(arms[k]) for k in (arm, f"{arm}#b"))

            def full(x):
                return (x is not None and len(x["values"]) == M2_SEGMENTS
                        and all(v is not None and math.isfinite(v) for v in x["values"].values()))

            bound = []
            for k, x in (("a", a), ("b", b)):
                if x is None:
                    continue
                if not math.isclose(x.get("physics_hz") or 0.0, hz[dt], rel_tol=1e-6):
                    bound.append(f"{k}: physics_hz {x.get('physics_hz')}")
                bound += [f"{k}: {w}" for w in arm_problems(arm, x.get("arm") or {}, k_mat)]
            out.setdefault("M2", {}).setdefault(dt, {})[arm] = {
                "deterministic": a is not None and b is not None and a["digest"] == b["digest"],
                "physically_valid": full(a) and full(b), "population": bound[:10],
                "reverse_required": False, "order_sensitive": None}
    for m, key, rec, sens, valid_one in (("M3", "cusp", "trials", cusp_order_sensitive, _trial_valid),
                                         ("M4", "delivery", "points", delivery_order_sensitive, _point_valid)):
        for dt, arms in manifest.get(key, {}).items():
            for arm in ARMS:
                runs = arms.get(arm) or {}
                loaded = {k: load(runs[k]) for k in ("a", "b", "r") if runs.get(k) is not None}
                a, b = loaded.get("a"), loaded.get("b")
                pop = []
                for k, x in loaded.items():
                    want_order = "reverse" if k == "r" else "forward"
                    if x.get("order") != want_order:
                        pop.append(f"{k}: order {x.get('order')}")
                    rec_hz = (1.0 / x["dt_s"]) if m == "M3" and x.get("dt_s") else None
                    if m == "M3" and (rec_hz is None or not math.isclose(rec_hz, hz[dt], rel_tol=1e-6)):
                        pop.append(f"{k}: dt {x.get('dt_s')}")
                    if m == "M4" and x.get("grid") != "f2":
                        pop.append(f"{k}: grid {x.get('grid')}")
                    if m == "M4" and x.get("physics_hz") != hz[dt]:
                        pop.append(f"{k}: physics_hz {x.get('physics_hz')}")
                    res = load(_result_of(runs[k]))
                    pop += [f"{k}: {w}" for w in arm_problems(arm, arm_identity(res), k_mat)]
                for k, x in loaded.items():
                    pop += [f"{k}: {w}" for w in (cusp_population(x, expected_cusp(registry, dt)) if m == "M3" and
                                                  registry else ([] if m == "M3" else delivery_population(x)))]
                if m == "M3" and registry is None:
                    pop.append("no registry")
                shares = [sum(valid_one(e) for e in x[rec]) / max(len(x[rec]), 1) for x in loaded.values()]
                row = {"deterministic": a is not None and b is not None and a[rec] == b[rec],
                       "physically_valid": bool(shares) and min(shares) >= (M3_MIN_SHARE if m == "M3" else
                                                                             M4_MIN_POINTS / 22),
                       "population": pop[:10], "reverse_required": dt == "120"}
                tag = sens(a, loaded["r"]) if (a is not None and "r" in loaded) else None
                row["order_sensitive"] = None if tag is None else bool(tag)
                row["order_sensitive_states"] = None if tag is None else tag[:20]
                out.setdefault(m, {}).setdefault(dt, {})[arm] = row
    return out


def usable(valid: dict, m: str, dt: str, arms) -> bool:
    rows = valid.get(m, {}).get(dt, {})
    for a in arms:
        r = rows.get(a)
        if r is None or not r["deterministic"] or not r["physically_valid"] or r.get("population"):
            return False
        if r["reverse_required"] and r["order_sensitive"] is not False:
            return False  # not run, or order-sensitive
        if not r["reverse_required"] and r["order_sensitive"] is True:
            return False
    return True


# --- dt condition (plan 4.4) ---

def dt_check(v120: dict, v240: dict, pair, floor: float, minimum: int) -> dict:
    a, b = pair
    states = sorted(set(v120[a]) & set(v120[b]) & set(v240[a]) & set(v240[b]))
    states = [s for s in states if None not in (v120[a][s], v120[b][s], v240[a][s], v240[b][s])]
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


def m3_counts(x: dict, y: dict, want: dict) -> dict:
    xs = {t["id"]: t for t in x["trials"]}
    ys = {t["id"]: t for t in y["trials"]}
    valid = [i for i in want if i in xs and i in ys and _trial_valid(xs[i]) and _trial_valid(ys[i])]
    w = [cusp_worse(xs[i], ys[i]) for i in valid]
    return {"registered": len(want), "valid": len(valid), "worse": w.count(1), "better": w.count(-1)}


def m4_counts(x: dict, y: dict) -> dict:
    """Plan 6 M4 (cast a/b, both estimates): accept lost/gained, and when both accept, a correction error larger or
    smaller by more than 1 mm / 0.001 rad on any axis -- counted apart, so a state larger on one axis and smaller on
    another counts in both (amendment 1)."""
    xs = {p["label"]: p for p in x["points"]}
    ys = {p["label"]: p for p in y["points"]}
    labels = sorted(set(xs) & set(ys))
    valid = [lab for lab in labels if _point_valid(xs[lab]) and _point_valid(ys[lab])]
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
                lim = (GE_M, GE_M, GE_RAD)
                worse += any(d[i] > lim[i] for i in range(3))
                better += any(d[i] < -lim[i] for i in range(3))
    return {"points": len(labels), "valid": len(valid), "worse": worse, "better": better}


def m2_counts(vx: dict, vy: dict) -> dict:
    states = sorted(s for s in set(vx) & set(vy) if vx[s] is not None and vy[s] is not None)
    d = [(vx[s] - vy[s]) * 1e3 for s in states]
    return {"segments": len(states), "worse": sum(x > LR_MRAD for x in d), "better": sum(x < -LR_MRAD for x in d)}


def candidate(manifest: dict, load, calib: dict, valid: dict, dt: dict, m2: dict) -> dict:
    out = {"preconditions": [], "self_degradation": {}, "f1_under_f2": {}}
    if calib.get("chosen_k_mat") is None:
        out["preconditions"].append("stage C chose no material value")
    for m, dts in (("M1", ("120", "240")), ("M2", ("120", "240")), ("M3", ("120",)), ("M4", ("120",))):
        for d in dts:
            if not usable(valid, m, d, ARMS):
                out["preconditions"].append(f"{m} at {d} Hz not usable (determinism, validity, population or the "
                                            "reversed run)")
    registry = load(manifest["registry"]) if manifest.get("registry") else {"versions": []}
    want = expected_cusp(registry, "120") if manifest.get("registry") else {}
    cusp, dlv = manifest["cusp"]["120"], manifest["delivery"]["120"]
    pairs = {"f1_under_f2": ("F1+F2", "O+F2"), "self_degradation": ("O+F2", "O")}
    for name, (x, y) in pairs.items():
        c3 = m3_counts(load(cusp[x]["a"]), load(cusp[y]["a"]), want)
        c4 = m4_counts(load(dlv[x]["a"]), load(dlv[y]["a"]))
        c2 = m2_counts(m2["120"][x], m2["120"][y])
        out[name] = {"M3": c3, "M4": c4, "M2": c2,
                     "worse_any": c3["worse"] > c3["better"] or c4["worse"] > c4["better"] or c2["worse"] > c2["better"]}
        if name == "f1_under_f2":
            if not want or c3["valid"] < M3_MIN_SHARE * c3["registered"]:
                out["preconditions"].append(f"M3 valid {c3['valid']} of {c3['registered']} registered")
            if c4["valid"] < M4_MIN_POINTS:
                out["preconditions"].append(f"M4 valid {c4['valid']} of {c4['points']}")
            if c2["segments"] != M2_SEGMENTS:
                out["preconditions"].append(f"M2 segments {c2['segments']} of {M2_SEGMENTS}")
    pick = "F2" if out["f1_under_f2"]["worse_any"] else "F1+F2"
    if not candidate_dt_ok(dt, pick):
        out["preconditions"].append(f"dt condition not passed for {pick}")
    v4 = (manifest.get("plant_gates") or {}).get(pick)
    v4rec = None if v4 is None else load(v4)
    if v4rec is None or not isinstance(v4rec.get("v4_v2_preserved"), bool):
        out["preconditions"].append(f"October V2/V4 not reported for {pick} (plan 4.4; a missing or unverified "
                                    "value is not a report)")
    out["escalate_v4"] = bool(v4rec is not None and v4rec.get("v4_v2_preserved") is False)
    out["escalate_self_degradation"] = out["self_degradation"]["worse_any"]
    out["candidate"] = None if out["preconditions"] else pick
    notes = [n for n, f in (("F2 itself worsens a measure", out["escalate_self_degradation"]),
                            ("the candidate's V4 is broken", out["escalate_v4"])) if f]
    out["verdict"] = ("no selection (to the lead)" if out["preconditions"] else
                      pick + (f" -- {', '.join(notes)}: to the lead before (나)" if notes else ""))
    return out


def run(manifest: dict, load=lambda p: json.loads(Path(p).read_text()), steer_load=read_steer) -> dict:
    valid = validity(manifest, load, steer_load)
    m1 = {dt: {a: m1_values(load(arms[a])) for a in ARMS if a in arms} for dt, arms in manifest["static"].items()}
    m2 = {dt: {a: steer_load(arms[a])["values"] for a in ARMS if a in arms} for dt, arms in manifest["steer"].items()}
    per_state, desc = {}, {}
    for m, vals in (("M1", m1), ("M2", m2)):
        for dt, v in vals.items():
            if len(v) == len(ARMS):
                e = effects(v)
                per_state.setdefault(m, {})[dt] = e
                desc.setdefault(m, {})[dt] = {k: _summary(x) for k, x in e.items()}
    getters = {"M3 e_stop (signed)": ("cusp", lambda r: {t["id"]: t["e_stop_rad"] for t in r["trials"]}),
               "M3 |e_stop|": ("cusp", lambda r: {t["id"]: None if t["e_stop_rad"] is None else abs(t["e_stop_rad"])
                                                  for t in r["trials"]}),
               "M4 information": ("delivery", lambda r: {f"{p['label']}/{k}": e["min_information"]
                                                         for p in r["points"] for k, e in p["evals"].items()})}
    for name, (key, getter) in getters.items():
        for dt, arms in manifest.get(key, {}).items():
            if all(a in arms for a in ARMS):
                e = effects({a: getter(load(arms[a]["a"])) for a in ARMS})
                per_state.setdefault(name, {})[dt] = e
                desc.setdefault(name, {})[dt] = {k: _summary(x) for k, x in e.items()}
    dt = dt_condition(m1, m2)
    calib = load(manifest["calibration"]) if manifest.get("calibration") else {}
    cand = candidate(manifest, load, calib, valid, dt, m2)
    return {"validity": valid, "effects": desc, "effects_per_state": per_state, "dt_condition": dt, "candidate": cand}


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
