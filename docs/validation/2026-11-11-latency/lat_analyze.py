"""The latency-tolerance test A's judgement (docs/plans/2026-11-11-latency-tolerance.md 4, fixed before the results).

    python lat_analyze.py OUT_ROOT --snap SNAP [--smoke SMOKE_OUT] [--json OUT.json]

OUT_ROOT holds RUNS_A.jsonl (and RUNS_rerun.jsonl), the run folders it names and the bundles' b*/results.jsonl. For
every run: environment validity (4.7), safety (4.1, the baseline's stopping model for every arm), the stop-vs-end split
(4.2), the mission and the operating metrics against the scene-seed baseline (4.3), the exposure (4.5) and the cell
class (4.4, first match of: dangerous, observed_end, safe_failure, underexposed, operating_degraded, tolerated); then
the equivalence (S0/L0 vs B0, ``deterministic_compare --ignore-injection``; aisle_rack's from the smoke) and the
determinism repeats (strict), and the conclusion rules (defects by priority). Report-only values ride along.
"""

from __future__ import annotations

import argparse
import glob
import importlib.util
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]

NOMINAL_TICKS = {"B0": 0, "S0": 0, "L0": 0, "S3": 3, "S6": 6, "S12": 12, "U3": 3, "U6": 6, "U12": 12, "D12": 12,
                 "C6": 6, "J": 12}
SLAM_ARMS = {"S0", "S3", "S6", "S12", "C6", "J"}
LAYER_ARMS = {"L0", "U3", "U6", "U12", "D12", "C6", "J"}
MUST_TOLERATE = {"S3", "S6", "U3", "U6", "C6"}  # 4.4: 50 ms and under
EXPOSURE_MIN = 200
ON_SCHEDULE_MIN = 0.8
STAND_S = 0.5
TIME_FACTOR, TIME_ADD_S, DELIVERY_ADD_CM, ROUNDS_ADD = 1.15, 30.0, 1.0, 1
SAME_EVENT_WORSE_M = 0.01


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


# ---- per run ------------------------------------------------------------------------------------------------------

def stop_or_end(result: dict) -> str | None:
    """4.2 for a run that failed its mission: safe_failure when the stand that runs up to the end is >= 0.5 s (the
    requested and applied speeds zero, truth under 0.05 m/s), else observed_end. None for a completed mission."""
    if result.get("success"):
        return None
    stand = ((result.get("latency_injection") or {}).get("final_stand") or {})
    length = stand.get("length_s")
    return "safe_failure" if length is not None and length >= STAND_S - 1e-9 else "observed_end"


def safety_of(result: dict, low: dict | None) -> dict:
    """4.1, absolute. None values are unreadable (the run is then the environment's)."""
    ol = result.get("obstacle_layer") or {}
    events = []
    for key, kind in (("unpermitted", "unpermitted"), ("unpermitted_dir", "unpermitted_dir")):
        for e in ol.get(key) or []:
            events.append({"kind": kind, "object": e.get("object"), "phase": e.get("phase")})
    phys_min, overlaps, arc = None, [], []
    if low is not None and "error" not in low:
        rows = low.get("rows") or {}
        mins = [(k, r.get("min_m"), r.get("phase"), r.get("object")) for k, r in rows.items() if k.endswith("physical")]
        vals = [m for _, m, _, _ in mins if m is not None]
        phys_min = min(vals) if vals else math.inf
        overlaps = [{"kind": "physical_overlap", "row": k, "min_m": m, "phase": ph, "object": o} for k, m, ph, o in mins
                    if m is not None and m <= 0.0]
        arc = [{"kind": "physical_arc", "object": tuple(h.get("objects") or ()), "phase": h.get("phase")}
               for h in low.get("physical_arc_violations") or []]
    contacts = result.get("forbidden_pocket_contacts")
    out = {
        "forbidden_contacts": None if contacts is None else len(contacts),
        "unpermitted_entries": ol.get("unpermitted_entries"),
        "unpermitted_dir_entries": ol.get("unpermitted_dir_entries"),
        "low_clearance_read": low is not None and "error" not in low,
        "physical_min_m": phys_min,
        "events": events + overlaps + arc,
    }
    out["violations"] = (None if (out["forbidden_contacts"] is None or out["unpermitted_entries"] is None
                                  or out["unpermitted_dir_entries"] is None or not out["low_clearance_read"])
                         else out["forbidden_contacts"] + out["unpermitted_entries"] + out["unpermitted_dir_entries"]
                         + len(overlaps) + len(arc))
    return out


def mission_of(result: dict) -> dict:
    tr = result.get("transitions") or []
    done = bool(result.get("success"))
    sim_s = (float(tr[-1]["time_s"]) - float(tr[0]["time_s"])) if done and len(tr) >= 2 else None
    de = result.get("delivery_error_m")
    rounds = (result.get("delivery_docking") or {}).get("rounds")
    ol = result.get("obstacle_layer") or {}
    slowed = ol.get("slowed_ticks") or {}
    return {"done": done, "reason": result.get("failure_reason"), "mission_sim_s": sim_s,
            "delivery_error_cm": None if de is None else 100.0 * float(de),
            "docking_rounds": None if rounds is None else len(rounds),
            # report only
            "slowed_ticks": int(sum(slowed.values())) if isinstance(slowed, dict) else slowed,
            "obstacle_replans": len(ol.get("replans") or []), "mission_replans": len(ol.get("mission_replans") or [])}


def exposure_of(arm: str, result: dict, layer_records: list | None) -> dict:
    """4.5: SLAM -- delayed replies delivered while moving outside holding (direct pose effect) >= 200, holding ones
    reported; layer -- jobs submitted while moving >= 200 and >= 0.8 of them installed on schedule (scheduled or, for
    k = 0, immediate), cut ones left out of both."""
    inj = result.get("latency_injection") or {}
    out: dict = {"ok": True}
    if arm in SLAM_ARMS:
        recs = (inj.get("slam") or {}).get("records") or []
        delivered = [r for r in recs if r.get("delivered_tick") is not None]
        direct = sum(1 for r in delivered if r.get("moving_at_delivery") and not r.get("holding_at_delivery"))
        out["slam"] = {"direct": direct, "holding": sum(1 for r in delivered if r.get("holding_at_delivery")),
                       "cut": sum(1 for r in recs if r.get("truncated")),
                       "delay_ticks": _spread([r["delivered_tick"] - r["scan_tick"] for r in delivered])}
        out["ok"] &= direct >= EXPOSURE_MIN
    if arm in LAYER_ARMS:
        recs = layer_records or []
        moving = [r for r in recs if r.get("moving")]
        on = sum(1 for r in moving if r.get("reason") in ("scheduled", "immediate"))
        share = on / len(moving) if moving else 0.0
        out["layer"] = {"moving_jobs": len(moving), "on_schedule_share": share,
                        "holding_jobs": sum(1 for r in recs if r.get("holding")),
                        "reasons": _count(r.get("reason") for r in recs),
                        "delay_ticks": _spread([r["install_tick"] - r["scan_tick"] for r in recs
                                                if r.get("install_tick") is not None])}
        out["ok"] &= len(moving) >= EXPOSURE_MIN and share >= ON_SCHEDULE_MIN
    return out


def _spread(xs: list) -> dict | None:
    if not xs:
        return None
    s = sorted(xs)
    return {"min": s[0], "median": s[len(s) // 2], "max": s[-1], "n": len(s)}


def _count(xs) -> dict:
    out: dict = {}
    for x in xs:
        out[x] = out.get(x, 0) + 1
    return out


def operating_within(m: dict, base: dict) -> dict:
    """4.3 against the scene-seed baseline (a completed one)."""
    lim_t = max(base["mission_sim_s"] * TIME_FACTOR, base["mission_sim_s"] + TIME_ADD_S)
    checks = {"mission_sim_s": (m["mission_sim_s"], lim_t),
              "delivery_error_cm": (m["delivery_error_cm"], (base["delivery_error_cm"] or 0.0) + DELIVERY_ADD_CM),
              "docking_rounds": (m["docking_rounds"], (base["docking_rounds"] or 0) + ROUNDS_ADD)}
    return {k: {"value": v, "limit": lim, "within": v is not None and v <= lim + 1e-9} for k, (v, lim) in checks.items()}


def classify(row: dict, base: dict | None, *, aisle: bool) -> dict:
    """4.4 (first match), with aisle_rack's baseline exceptions ahead of the conclusion rules (3.1)."""
    s, m = row["safety"], row["mission"]
    out = {"baseline_failed": bool(base and not base["mission"]["done"]),
           "baseline_violation": bool(base and (base["safety"]["violations"] or 0) > 0)}
    if s["violations"]:
        out["class"] = "dangerous"
        if base is not None:
            out["b0_repeat"] = b0_repeat(s, base["safety"])
        return out
    if not m["done"]:
        out["class"] = row["stop"]  # observed_end or safe_failure
        if aisle and out["baseline_failed"]:
            out["attribution"] = "baseline_failed: not a latency defect, safety only"
        return out
    if not row["exposure"]["ok"] and row["arm"] not in ("B0", "S0", "L0"):
        out["class"] = "underexposed"
        return out
    if base is None or not base["mission"]["done"]:
        out["class"] = "tolerated_no_operating_comparison"
        return out
    op = operating_within(m, base["mission"])
    out["operating"] = op
    out["class"] = "tolerated" if all(c["within"] for c in op.values()) else "operating_degraded"
    return out


def b0_repeat(s: dict, bs: dict) -> list:
    """The 'B0 repeat' mark per event: the same kind, object and phase in the baseline (and, for a physical overlap,
    not more than 1 cm worse); the mark never changes the judgement."""
    out = []
    for e in s["events"]:
        same = [b for b in bs["events"] if b["kind"] == e["kind"] and b.get("object") == e.get("object")
                and b.get("phase") == e.get("phase")]
        if e["kind"] == "physical_overlap":
            same = [b for b in same if e["min_m"] >= b["min_m"] - SAME_EVENT_WORSE_M]
        out.append({**e, "b0_repeat": bool(same)})
    return out


def conclusions(rows: dict, equivalence: dict, determinism: dict) -> dict:
    """4.4 conclusion rules: defects by priority (1 dangerous, 2 observed_end, 3 failures or degradation at 50 ms and
    under, 4 the rest), the arms an equivalence failure leaves unread, the D12/U12 reading."""
    unread = set()
    for scene, eq in equivalence.items():
        if eq.get("S0") is False:
            unread |= {(scene, a) for a in SLAM_ARMS - {"S0"}}
            if scene == "aisle_rack":
                unread |= {(scene, "U12")}
        if eq.get("L0") is False:
            unread |= {(scene, a) for a in LAYER_ARMS - {"L0"}}
    stopped = {scene for scene, ok in determinism.items() if ok is False}
    defects = []
    for name, r in sorted(rows.items()):
        if r.get("invalid") or r["arm"] in ("B0", "S0", "L0") or r.get("repeat"):
            continue
        c = r["cell"]["class"]
        key = (r["scene"], r["arm"])
        if r["scene"] in stopped:
            continue
        if key in unread and c != "dangerous":
            continue  # only the absolute safety is read for an unread arm
        if c == "dangerous":
            prio = 1
        elif c == "observed_end":
            prio = 2
        elif r["cell"].get("attribution"):
            continue
        elif c in ("safe_failure", "operating_degraded", "underexposed", "tolerated_no_operating_comparison"):
            prio = 3 if r["arm"] in MUST_TOLERATE else 4
            if c == "operating_degraded":
                prio += 0.5  # after the failures of its band
        else:
            continue
        defects.append({"run": name, "scene": r["scene"], "seed": r["seed"], "arm": r["arm"], "class": c,
                        "priority": prio})
    reading = {}
    for scene in {r["scene"] for r in rows.values()}:
        for seed in {r["seed"] for r in rows.values()}:
            d = next((r for r in rows.values() if (r["scene"], r["seed"], r["arm"]) == (scene, seed, "D12")
                      and not r.get("invalid")), None)
            u = next((r for r in rows.values() if (r["scene"], r["seed"], r["arm"]) == (scene, seed, "U12")
                      and not r.get("invalid")), None)
            if d and u:
                dc, uc = d["cell"]["class"], u["cell"]["class"]
                reading[f"{scene}_s{seed}"] = (
                    "knows it: holds; unaware: stops" if dc == "tolerated" and uc == "safe_failure"
                    else "stop safety unknown" if uc == "observed_end"
                    else "safety violation" if uc == "dangerous" else f"D12 {dc} / U12 {uc}")
    return {"defects": sorted(defects, key=lambda x: (x["priority"], x["run"])), "unread": sorted(map(list, unread)),
            "determinism_stopped": sorted(stopped), "d12_u12": reading}


# ---- I/O ----------------------------------------------------------------------------------------------------------

def bundle_records(out_root: Path) -> dict:
    recs: dict = {}
    for f in glob.glob(str(out_root / "b*" / "results.jsonl")):
        for line in Path(f).read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                recs.setdefault(r.get("name"), []).append(r)
    return recs


def environment_problems(attempts: list, run: Path, arm: str, result: dict | None) -> list:
    out = []
    if len(attempts) != 1 or attempts[0].get("retried") or attempts[0].get("status") != "ended":
        out.append(f"bundle attempts {[(a.get('attempt'), a.get('status')) for a in attempts]}")
    if result is None:
        out.append("no result.json")
        return out
    broken = (result.get("deterministic") or {}).get("broken")
    if broken is None:
        out.append("not a deterministic run")
    elif broken:
        out.append(f"deterministic broken {broken}")
    if not (run / "slam_control.npy").exists():
        out.append("no slam_control.npy")
    if arm in LAYER_ARMS and not (run / "layer_worker.json").exists():
        out.append("no layer_worker.json")
    if arm in SLAM_ARMS | LAYER_ARMS and "latency_injection" not in result:
        out.append("no latency_injection record")
    return out


def stopping_from(snap: Path, result: dict, low_mod):
    layer = (result.get("arguments") or {}).get("obstacle_layer")
    import yaml

    p = Path(layer) if Path(layer).is_absolute() else snap / layer
    perm = (yaml.safe_load(p.read_text()) or {})["permission"]
    return low_mod.StoppingModel(float(perm["latency_s"]), float(perm["decel_mps2"]), float(perm.get("margin_m", 0.05)))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out_root", type=Path)
    ap.add_argument("--snap", type=Path, required=True)
    ap.add_argument("--smoke", type=Path, default=None, help="the smoke's out root (aisle_rack's B0/S0 equivalence)")
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args(argv)
    lists = _load("lat_lists", HERE / "lat_lists.py")
    low_mod = _load("low_clearance_report", ROOT / "tools/low_clearance_report.py")
    sys.path.insert(0, str(ROOT))
    from tools import deterministic_compare as dc

    rows_in = [json.loads(x) for x in (a.out_root / "RUNS_A.jsonl").read_text().splitlines() if x.strip()]
    registered = {r["name"] for r in lists.a_rows(str(a.snap), str(a.out_root))}
    if {r["name"] for r in rows_in} != registered:
        print("list_not_registered")
        return 2
    rerun_file = a.out_root / "RUNS_rerun.jsonl"
    reruns = {json.loads(x)["name"]: json.loads(x) for x in rerun_file.read_text().splitlines()} if rerun_file.exists() else {}
    attempts = bundle_records(a.out_root)
    rows: dict = {}
    for r in rows_in:
        name = r["name"]
        use = reruns.get(f"lat_r_{name}", r)  # a rerun stands in for the first, which stays on record
        d = Path(use["argv"][-1])
        run = d / "run"
        result = _json(run / "result.json")
        env = use["env"]
        arm, scene, seed = env["LAT_ARM"], env["SCENE"], int(env["SEED"])
        layer_recs = _json(run / "layer_worker.json")
        problems = environment_problems(attempts.get(use["name"], []), run, arm, result)
        row = {"name": use["name"], "first": name if use is not r else None, "arm": arm, "scene": scene, "seed": seed,
               "repeat": name.endswith("_rep"), "dir": str(d), "invalid": problems}
        if result is not None:
            row["mission"] = mission_of(result)
            row["stop"] = stop_or_end(result)
            row["exposure"] = exposure_of(arm, result, layer_recs)
            row["_result"] = result
        rows[name] = row
    # the baselines' stopping model and the safety, for every run that has a result (read even when invalid)
    for name, row in rows.items():
        res = row.pop("_result", None)
        if res is None:
            continue
        base_name = (f"lat_S0_aisle_rack_s{row['seed']}" if row["scene"] == "aisle_rack"
                     else f"lat_B0_{row['scene']}_s{row['seed']}")
        base_res = _json(Path(rows[base_name]["dir"]) / "run" / "result.json") if base_name in rows else None
        try:
            stopping = stopping_from(a.snap, base_res or res, low_mod)
            low = low_mod.run_report(str(Path(row["dir"]) / "run"), stopping, 0.01)
            sens = (low_mod.run_report(str(Path(row["dir"]) / "run"), low_mod.StoppingModel(
                stopping.latency_s + NOMINAL_TICKS[row["arm"]] / 120.0, stopping.decel_mps2, stopping.margin_m), 0.01)
                if NOMINAL_TICKS[row["arm"]] else None)
        except Exception as exc:  # noqa: BLE001 -- unreadable: the run is the environment's
            low, sens = {"error": repr(exc)[:300]}, None
        row["safety"] = safety_of(res, low)
        row["sensitivity_low_objects"] = None if sens is None else {
            "physical_arc_violations": len(sens.get("physical_arc_violations") or []),
            "reopen": sens.get("reopen")}
        if row["safety"]["violations"] is None:
            row["invalid"].append("safety record unreadable")
    for name, row in rows.items():
        if "safety" not in row:
            continue
        base_name = (f"lat_S0_aisle_rack_s{row['seed']}" if row["scene"] == "aisle_rack"
                     else f"lat_B0_{row['scene']}_s{row['seed']}")
        base = rows.get(base_name)
        base = base if base is not None and not base["invalid"] and "safety" in base else None
        row["cell"] = classify(row, base if name != base_name else None, aisle=row["scene"] == "aisle_rack")
    # equivalence and determinism (4.7): whole runs compared, the injection records left out for S0/L0
    def cmp(x, y, ignore):
        if x not in rows or y not in rows or rows[x]["invalid"] or rows[y]["invalid"]:
            return None
        dc.IGNORE_INJECTION = ignore
        try:
            return bool(dc.compare(Path(rows[x]["dir"]), Path(rows[y]["dir"]))["equal"])
        finally:
            dc.IGNORE_INJECTION = False
    equivalence, determinism = {}, {}
    for scene in ("line_2", "random_1"):
        equivalence[scene] = {"S0": cmp(f"lat_B0_{scene}_s0", f"lat_S0_{scene}_s0", True),
                              "L0": cmp(f"lat_B0_{scene}_s0", f"lat_L0_{scene}_s0", True)}
        b, c = cmp(f"lat_B0_{scene}_s0", f"lat_B0_{scene}_s0_rep", False), cmp(f"lat_C6_{scene}_s0",
                                                                               f"lat_C6_{scene}_s0_rep", False)
        determinism[scene] = None if None in (b, c) else (b and c)
    if a.smoke is not None:
        dc.IGNORE_INJECTION = True
        try:
            eq = dc.compare(a.smoke / "lat_sm_B0_aisle_rack_s0", a.smoke / "lat_sm_S0_aisle_rack_s0")["equal"]
        finally:
            dc.IGNORE_INJECTION = False
        equivalence["aisle_rack"] = {"S0": bool(eq)}
    rerun_needed = sorted(n for n, r in rows.items() if r["invalid"] and not r["first"])
    out = {"rows": rows, "equivalence": equivalence, "determinism": determinism,
           "rerun_needed": rerun_needed, "to_lead": sorted(n for n, r in rows.items() if r["invalid"] and r["first"]),
           "conclusions": conclusions(rows, equivalence, determinism)}
    if a.json is not None:
        a.json.write_text(json.dumps(out, indent=1, default=str) + "\n")
    print(json.dumps({k: out[k] for k in ("equivalence", "determinism", "rerun_needed", "to_lead")}, indent=1))
    print(json.dumps(out["conclusions"], indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
