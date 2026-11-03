#!/usr/bin/env python3
"""The F1 fork-tip-mass registration batch read by the rules fixed before the results (docs/plans/
2026-11-04-f1-fork-tip-mass.md section 3; validity, sources and the live rule from docs/plans/2026-11-01-pallet-mass.md
section 4 with its amendment 1).

Reads BATCH/f1_<arm>_<key>/ (arms O and F1; riib and cycle runs in run/result.json, live runs in probe.json and
live/epoch_1/run/result.json) and the bundle records BATCH/b_*/results.jsonl, and writes the verdict per group, the
steering residual non-inferiority per cell, and the reported-only quantities.

  PYTHONPATH=<snapshot>/src python3 tools/f1_tip_mass_judge.py BATCH --json OUT.json \
      [--age-table config/obstacle_odometry_age_team_spec.json --age O=O_AGE.json --age F1=F1_AGE.json]

The population is the registered list (EXPECTED_KEYS); a missing pair counts as not valid, folders outside it are
listed and not read. Each run must show its arm (tip link masses, run URDF and the scene manifest's URDF hashes, the
shape's operating pallet mass) and each arm/shape one scene hash, different between the arms; otherwise the batch is
not read (inconclusive). The odometry age comparison (reported only) reads tools/p0a_measure.py age outputs, one per
arm, against the operating table.

A pair whose two arms are not both valid is run again once as <key>_r2 (both arms); the rerun pair then stands for
the key. The steering residual: e = (command - actual) * sign(command) per front wheel from result.json samples
(10 Hz), |command| >= 0.20 rad, both commands within 1 mrad of the previous sample's, that sample 0.1 s earlier
(0.08-0.12 s: the 10 Hz record's spacing);
cells (standstill |v| <= 0.02 / forward v > 0.02) x (empty / loaded lift_m > 0.02) x (left / right turn, sign of
that wheel's command) x (L / R wheel); reverse is reported only. Per cell an arm's value is the median of the pooled
samples of every valid pair; d = median(F1) - median(O); the 95 % interval resamples the valid pairs whole (2,000 draws,
numpy default_rng(0) per cell). A cell is sufficient with >= 8 valid pairs that have it in both arms and >= 200
samples per arm. A sample with no speed or lift_m record is missing, not standstill or empty.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

ARMS = ("O", "F1")
INFRA_PREFIXES = ("slam_link_failed", "slam_prewarm_timeout", "plan_worker_dead", "Camera did not produce RGB",
                  "Perception camera did not produce depth", "Cannot open base scene",
                  "Invalid pallet spawn after settling", "Truck spawn displaced")  # riib batch 2 section 4 list
# Registration amendment 2 (2026-11-04, the lead's decision before the verdict; the F1 plan's section 3): a run that
# ended before its first perception frame on the omni.replicator overscan TypeError of the perception camera's
# get_rgba, with no contact and no movement, is the 'no sensor output' infrastructure end, in both arms alike.
OVERSCAN_REASON = "unsupported operand type(s) for -: 'NoneType' and 'NoneType'"
OVERSCAN_TRACE = "_resize_data_for_overscan"
STILL_M = 0.01  # the truth rear axle's largest move from its first control sample
OVERSCAN_HOLD_SHARE = 0.25  # amendment 2's stop rule: above this share of an arm's runs the batch is held
MIN_PAIRS = {"riib": 10, "cycle": 3, "live": 2}
MIN_LIFT_PAIRS = {"riib": 8, "cycle": 2}
CMD_MIN, STEADY, V_STILL, LOADED = 0.20, 1e-3, 0.02, 0.02
MARGIN = 1e-3  # rad, the non-inferiority bound
CELL_MIN_PAIRS, CELL_MIN_SAMPLES, DRAWS = 8, 200, 2000
TIP_MASS = {"O": 1.0, "F1": 1e-4}  # kg per fork tip link, the plan's section 1
# The run URDF and the scene manifest's URDF (sha256 prefixes) per arm and shape: O = the operating scenes
# (20261013_fork_spacing base_scene / base_scene_0p29), F1 = artifacts/f1_scenes (plan section 2).
URDF_SHA = {("O", "epal6"): "517e8260", ("O", "t11_06_d600"): "f3068a89",
            ("F1", "epal6"): "f5a348d3", ("F1", "t11_06_d600"): "d04e1237"}
PALLET_KG = {"epal6": 4.93, "t11_06_d600": 2.49}  # the operating pallet masses (plan section 3 run list)
EXPECTED_KEYS = ([f"{sc}_s{s}" for sc in ("line_2", "random_1", "aisle_rack", "trailer") for s in range(3)]
                 + [f"cy_{p}_s{s}" for p in ("epal6", "t11_06_d600") for s in range(3)]
                 + [f"live_s{s}" for s in range(2)])
# Fork blade top above the base_link origin at lift 0: the measured model's fork_carriage blade box (origin z 0.04,
# height 0.025; fork_lift origin 0). Unchanged by F1, which edits only the two tip inertials.
FORK_TOP_M = 0.04 + 0.025 / 2
OPENING_TOP_M = {"epal6": 0.022 + 0.078, "t11_06_d600": 0.015 + 0.045}  # deck_bottom + block_height, the configs


def load(p: Path):
    try:
        return json.loads(p.read_text())
    except (OSError, ValueError):
        return None


def bundle_rows(batch: Path) -> dict:
    rows = {}
    for f in sorted(batch.glob("b_*/results.jsonl")):
        for line in f.read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                rows[r["name"]] = r  # the last attempt
    return rows


def split_name(name: str):
    """f1_<arm>_<key> -> (arm, key, rerun)."""
    _, arm, key = name.split("_", 2)
    rerun = key.endswith("_r2")
    return arm, key[:-3] if rerun else key, rerun


def group_of(key: str) -> str:
    if key.startswith("cy_"):
        return "cycle_" + key[3:].rsplit("_s", 1)[0]
    if key.startswith("live_"):
        return "live"
    return "riib"


def shape_of(key: str) -> str:
    return key[3:].rsplit("_s", 1)[0] if key.startswith("cy_") else "epal6"


def arm_identity(d: dict | None) -> dict:
    if d is None:
        return {}
    pb = d.get("plant_backend") or {}
    tips = [m for n, m in zip(pb.get("body_names") or [], pb.get("masses_kg") or []) if "fork_tip" in str(n)]
    return {"scene_sha256": d.get("scene_sha256"), "scene_built_from_urdf_sha256": d.get("scene_built_from_urdf_sha256"),
            "forklift_urdf_sha256": d.get("forklift_urdf_sha256"), "tip_masses_kg": tips,
            "pallet_mass_kg": (d.get("pallet_mass") or {}).get("kg")}


def identity_problems(arm: str, shape: str, ident: dict) -> list:
    """Why the run does not show its registered arm (empty: it does)."""
    if not ident:
        return ["no result"]
    out = []
    tips = ident.get("tip_masses_kg") or []
    if not (len(tips) == 2 and all(math.isclose(m, TIP_MASS[arm], rel_tol=1e-3) for m in tips)):
        out.append(f"tip masses {tips}")
    want = URDF_SHA.get((arm, shape))
    for field in ("forklift_urdf_sha256", "scene_built_from_urdf_sha256"):
        if want is None or not str(ident.get(field) or "").startswith(want):
            out.append(f"{field} {ident.get(field)}")
    scene = ident.get("scene_sha256")
    if not ((isinstance(scene, str) and scene)
            or (isinstance(scene, dict) and scene and all(isinstance(v, str) and v for v in scene.values()))):
        out.append(f"scene hash {scene}")
    kg = ident.get("pallet_mass_kg")
    if kg is None or not math.isclose(kg, PALLET_KG[shape], rel_tol=1e-6):
        out.append(f"pallet mass {kg}")
    return out


def insert_stops(samples: list) -> list:
    """Stops >= 0.5 s inside the insert phase, between its first and last moving sample."""
    ins = [x for x in samples if x.get("phase") == "insert"]
    ins = [x for x in ins if x.get("signed_speed_mps") is not None]
    moving = [i for i, x in enumerate(ins) if abs(x["signed_speed_mps"]) > V_STILL]
    if not moving:
        return []
    stops, start = [], None
    for x in ins[moving[0]:moving[-1] + 1]:
        still = abs(x.get("signed_speed_mps") or 0.0) <= V_STILL
        if still and start is None:
            start = x["time_s"]
        elif not still and start is not None:
            if x["time_s"] - start >= 0.5:
                stops.append({"start_s": start, "duration_s": x["time_s"] - start})
            start = None
    return stops


def upper_gap(samples: list, shape: str):
    """Smallest pocket upper gap over insert samples: pallet origin z + opening top - (base z + blade top + lift).
    Base and pallet tilt are ignored (a proxy; F1 changes the truck's attitude too, so the bias need not be the same
    in both arms)."""
    vals = [x["pallet_position_m"][2] + OPENING_TOP_M[shape] - (x["base_position_m"][2] + FORK_TOP_M + x["lift_m"])
            for x in samples if x.get("phase") == "insert" and x.get("lift_m") is not None
            and x.get("pallet_position_m") and x.get("base_position_m")]
    return min(vals) if vals else None


def median_of(samples: list, phase: str, field: str):
    vals = [x[field] for x in samples if x.get("phase") == phase and x.get(field) is not None]
    return float(np.median(vals)) if vals else None


def overscan_end(run_dir: Path, d: dict | None) -> bool:
    """Amendment 2: the overscan TypeError before the first perception frame, no contact, no movement. Every
    condition needs its own record; a missing record does not meet it."""
    if d is None or d.get("failure_reason") != OVERSCAN_REASON:
        return False
    for field in ("samples", "transitions", "forbidden_pocket_contacts"):  # recorded and empty, not absent
        if not isinstance(d.get(field), list) or d[field]:
            return False
    command = d.get("applied_command")
    if not isinstance(command, list) or not command or not isinstance(command[0], (int, float)) or command[0] != 0:
        return False
    try:
        trace = (run_dir / "failure.txt").read_text()
        control = np.load(run_dir / "slam_control.npy").reshape(-1, 7)
    except (OSError, ValueError):
        return False
    if OVERSCAN_TRACE not in trace or "get_rgba" not in trace or len(control) == 0:
        return False
    moved = float(np.max(np.hypot(control[:, 4] - control[0, 4], control[:, 5] - control[0, 5])))
    return not d.get("scene_damaged") and moved <= STILL_M


def batch_run(root: Path, name: str, bundle: dict, live: bool = False) -> dict:
    b = bundle.get(name) or {}
    arm, key, _ = split_name(name)
    probe = load(root / "probe.json") if live else None
    run_dir = root / "live" / "epoch_1" / "run" if live else root / "run"
    d = load(run_dir / "result.json")
    reason_ = None if d is None else d.get("failure_reason")
    overscan = overscan_end(run_dir, d)
    # a session that stopped after 'app ready' (no log, GPU idle) leaves no result.json: the existing list's case
    infra = (d is None or (live and probe is None) or b.get("status") in ("infra_failed", "stalled")
             or (reason_ is not None and str(reason_).startswith(INFRA_PREFIXES)) or overscan)
    ident = arm_identity(d)
    out = {"name": name, "arm": arm, "key": key, "infra": infra, "overscan_end": overscan,
           "overscan_reason": reason_ == OVERSCAN_REASON, "bundle_status": b.get("status"),
           "identity": ident, "identity_problems": identity_problems(arm, shape_of(key), ident)}
    if d is None:  # with a result and no probe the result's safety evidence is still read, not the live rule
        return out
    from forklift_core.mission.failures import SCENE, classify  # the snapshot's classifier

    reason = d.get("failure_reason")
    contacts = d.get("forbidden_pocket_contacts") or []
    lift = d.get("lift_tilt_peak")
    ins = d.get("insertion_measured") or {}
    lat = (d.get("lateral_clearance_min_m") or {}).get("insert") or {}
    sw = d.get("slip_watch") or {}
    samples = d.get("samples") or []
    pushed = None
    if lift and d.get("initial_pallet_m"):
        a, b_ = lift["pallet_position_m"], d["initial_pallet_m"]
        pushed = math.hypot(a[0] - b_[0], a[1] - b_[1])
    tilts = [x["base_tilt_rad"] for x in samples if x.get("base_tilt_rad") is not None]
    out.update(
        pallet_mass_kg=(d.get("pallet_mass") or {}).get("kg"),
        complete=bool(d.get("success")) and d.get("phase") == "complete", phase=d.get("phase"), reason=reason,
        safety=bool(contacts) or (reason is not None and classify(reason) == SCENE),
        slip_failure=sw.get("failure"), lifted=lift is not None,
        tilt_rad=None if lift is None else lift.get("pallet_tilt_rad"), pushed_m=pushed,
        insertion_m=[(ins.get(s) or {}).get("insertion_m") for s in ("left", "right")],
        lateral_min_m=min([v for v in lat.values() if v is not None], default=None),
        outline_motion_max_m=sw.get("outline_motion_max_m"), pocket_gap_min_m=sw.get("pocket_gap_min_m"),
        insert_contact=reason is not None and str(reason).startswith("Forbidden fork/pallet contact in insert"),
        ended_in_insert=not (bool(d.get("success")) and d.get("phase") == "complete") and d.get("phase") == "insert",
        insert_stops=insert_stops(samples),
        lift_m_insert_median=median_of(samples, "insert", "lift_m"),
        lift_m_observe_median=median_of(samples, "observe", "lift_m"),
        lift_m_transport_median=median_of(samples, "transport", "lift_m"),  # where the sag can show
        upper_gap_insert_min_m=upper_gap(samples, shape_of(key)),
        base_tilt_max_rad=max(tilts) if tilts else None)
    if live and probe is not None:
        first = (probe.get("missions_run") or [{}])[0]
        w = first.get("stop_wait") or {}
        span = (None if w.get("end_sim_s") is None or w.get("start_sim_s") is None
                else w["end_sim_s"] - w["start_sim_s"])
        result = first.get("result") or {}
        events = [e for e in probe.get("events") or [] if isinstance(e, dict)]
        out["live_pass"] = bool(
            w.get("phase_reached") and (w.get("stop_reply") or {}).get("t") == "ack"
            and w.get("end_reason") == "sim_time" and span is not None and span >= 30
            and w.get("mode_at_end") == "STOPPED" and (w.get("rearm") or {}).get("t") == "ack"
            and any(e.get("event") == "reached" and e.get("label") == "paused_after_rearm" for e in events)
            and result.get("mission_id") is not None and result.get("mission_id") == w.get("mission_id")
            and result.get("outcome") in ("cancelled", "recovery_required") and not d.get("scene_damaged"))
        out.update(live_outcome=result.get("outcome"), stand_sim_s=span, scene_damaged=d.get("scene_damaged"))
    out["_samples"] = samples
    return out


def pair_table(runs: list) -> dict:
    """registered key -> {arm: run}: the first pair, or its rerun pair where the first is broken or missing
    (pallet-mass section 4 validity); a key with neither has an empty pair."""
    first, rerun = {}, {}
    for r in runs:
        _, key, is_rerun = split_name(r["name"])
        (rerun if is_rerun else first).setdefault(key, {})[r["arm"]] = r
    out = {}
    for key in EXPECTED_KEYS:
        pair = first.get(key, {})
        out[key] = pair if valid(pair) or key not in rerun else rerun[key]
    return out


def valid(pair: dict) -> bool:
    return all(a in pair and not pair[a]["infra"] for a in ARMS)


def group_verdict(group: str, pairs: dict, runs: list) -> dict:
    kind = "cycle" if group.startswith("cycle") else group
    good = {k: v for k, v in pairs.items() if valid(v)}
    every = [r for r in runs if group_of(r["key"]) == group]
    safety = {a: sum(bool(r.get("safety")) for r in every if r["arm"] == a) for a in ARMS}  # invalid runs too
    infra = {a: sum(r["infra"] for r in every if r["arm"] == a) for a in ARMS}
    out = {"pairs": len(pairs), "valid_pairs": len(good), "safety_failures": safety, "infra": infra}
    if kind == "live":
        out["pass"] = {v[a]["name"]: v[a].get("live_pass") for v in good.values() for a in ARMS}
        out["outcome"] = {v[a]["name"]: v[a].get("live_outcome") for v in good.values() for a in ARMS}
        # different ends of the two arms need a cause (pallet-mass section 4 live rule)
        out["outcome_differs"] = sorted(k for k, v in good.items() if v["O"].get("live_outcome") != v["F1"].get("live_outcome"))
        if len(good) < MIN_PAIRS["live"]:
            out["verdict"] = "inconclusive"
        elif not (all(out["pass"].values()) and safety["F1"] <= safety["O"]):
            out["verdict"] = "fail"
        else:
            out["verdict"] = "needs_cause" if out["outcome_differs"] else "pass"
        return out
    lifted = {k: v for k, v in good.items() if v["O"].get("lifted") and v["F1"].get("lifted")}
    complete = {a: sum(bool(v[a].get("complete")) for v in good.values()) for a in ARMS}
    slip = {a: sum(bool(v[a].get("slip_failure")) for v in lifted.values()) for a in ARMS}
    reasons = {a: sorted({str(v[a].get("reason")) for v in good.values() if not v[a].get("complete")}) for a in ARMS}
    out.update(both_lifted_pairs=len(lifted), complete=complete, slip_failures=slip, failure_reasons=reasons,
               new_reasons_F1=sorted(set(reasons["F1"]) - set(reasons["O"])))
    if len(good) < MIN_PAIRS[kind] or len(lifted) < MIN_LIFT_PAIRS[kind]:
        out["verdict"] = "inconclusive"
    else:
        ok = complete["F1"] >= complete["O"] - 1 and safety["F1"] <= safety["O"] and slip["F1"] <= slip["O"]
        out["verdict"] = "fail" if not ok else ("needs_cause" if out["new_reasons_F1"] else "pass")
    return out


def residual_samples(samples: list) -> dict:
    """cell -> shortfalls; cell = (speed, load, turn, wheel) with speed in still/forward/reverse."""
    cells = {}
    for x0, x in zip(samples, samples[1:]):
        c0, c, a = x0.get("steering_command_rad"), x.get("steering_command_rad"), x.get("steering_actual_rad")
        if not c or not c0 or not a or None in c or None in c0 or None in a:
            continue
        if not 0.08 <= x["time_s"] - x0["time_s"] <= 0.12 or max(abs(c[0] - c0[0]), abs(c[1] - c0[1])) > STEADY:
            continue
        v, lift = x.get("signed_speed_mps"), x.get("lift_m")
        if v is None or lift is None:  # missing, not standstill or empty
            continue
        speed = "still" if abs(v) <= V_STILL else ("forward" if v > 0 else "reverse")
        load = "loaded" if lift > LOADED else "empty"
        for w, wheel in ((0, "L"), (1, "R")):
            if abs(c[w]) < CMD_MIN:
                continue
            sign = 1.0 if c[w] > 0 else -1.0
            cells.setdefault((speed, load, "left" if sign > 0 else "right", wheel), []).append((c[w] - a[w]) * sign)
    return cells


def cell_estimate(pairs: list) -> dict:
    """pairs: [(O samples, F1 samples)] of every valid pair (either list may be empty)."""
    arr = [(np.asarray(o, dtype=float), np.asarray(f, dtype=float)) for o, f in pairs]
    n_o, n_f = sum(len(o) for o, _ in arr), sum(len(f) for _, f in arr)
    both = sum(len(o) > 0 and len(f) > 0 for o, f in arr)
    out = {"valid_pairs": len(arr), "pairs_with_both": both, "samples": {"O": n_o, "F1": n_f}}
    if n_o == 0 or n_f == 0:
        out["status"] = "insufficient"
        return out
    out["median_O"] = float(np.median(np.concatenate([o for o, _ in arr])))
    out["median_F1"] = float(np.median(np.concatenate([f for _, f in arr])))
    out["d"] = out["median_F1"] - out["median_O"]
    if both < CELL_MIN_PAIRS or n_o < CELL_MIN_SAMPLES or n_f < CELL_MIN_SAMPLES:
        out["status"] = "insufficient"
        return out
    rng = np.random.default_rng(0)
    ds = np.full(DRAWS, np.nan)
    for i in range(DRAWS):
        idx = rng.integers(0, len(arr), len(arr))
        o = np.concatenate([arr[j][0] for j in idx])
        f = np.concatenate([arr[j][1] for j in idx])
        if len(o) and len(f):  # a draw with no sample of one arm has no d
            ds[i] = np.median(f) - np.median(o)
    out["draws_without_d"] = int(np.isnan(ds).sum())
    lo, hi = np.nanpercentile(ds, [2.5, 97.5])
    out.update(ci95=[float(lo), float(hi)],
               status=("increase" if lo > MARGIN else "non_inferior" if hi <= MARGIN else "inconclusive"),
               decrease=bool(hi < -MARGIN))
    return out


def residual_verdict(pairs: dict) -> dict:
    per_pair = {}
    for key, pair in pairs.items():
        if valid(pair):
            per_pair[key] = {a: residual_samples(pair[a].get("_samples") or []) for a in ARMS}
    cells = sorted({c for v in per_pair.values() for a in ARMS for c in v[a]})
    table = {}
    for cell in cells:
        table["/".join(cell)] = cell_estimate([(v["O"].get(cell, []), v["F1"].get(cell, [])) for v in per_pair.values()])
    primary = {k: v for k, v in table.items()
               if k.split("/")[0] in ("still", "forward") and k.split("/")[1] == "empty" and k.endswith("/L")}
    others = {k: v for k, v in table.items() if k.split("/")[0] in ("still", "forward") and k not in primary}
    sufficient = [k for k, v in primary.items() if v["status"] != "insufficient"]
    have = {s: any(k.startswith(s + "/") for k in sufficient) for s in ("still", "forward")}
    if any(v["status"] == "increase" for v in list(primary.values()) + list(others.values())):
        verdict = "fail"
    elif not (have["still"] and have["forward"]):
        verdict = "inconclusive"
    elif any(primary[k]["status"] == "inconclusive" for k in sufficient):
        verdict = "inconclusive"
    else:
        verdict = "pass"
    return {"verdict": verdict, "pairs_read": len(per_pair), "sufficient_primary_cells": sufficient,
            "decrease_cells": sorted(k for k, v in table.items() if v.get("decrease")), "cells": table}


def reported(pairs: dict) -> dict:
    """Per group and arm over the valid pairs; the insertion and lift measures over the pairs both arms lifted in
    (pallet-mass section 4)."""
    out = {}
    for group in sorted({group_of(k) for k in pairs}):
        good = [v for k, v in pairs.items() if group_of(k) == group and valid(v)]
        g = {}
        for a in ARMS:
            mine = [v[a] for v in good]
            lifted = [v[a] for v in good if v["O"].get("lifted") and v["F1"].get("lifted")]

            def vals(field, rs=mine):
                return [r[field] for r in rs if r.get(field) is not None]

            g[a] = {
                "insert_contact_runs": [r["name"] for r in mine if r.get("insert_contact")],
                "ended_in_insert": {r["name"]: r.get("reason") for r in mine if r.get("ended_in_insert")},
                "insert_stops": {r["name"]: r["insert_stops"] for r in mine if r.get("insert_stops")},
                "lift_m_insert_median": float(np.median(vals("lift_m_insert_median"))) if vals("lift_m_insert_median") else None,
                "lift_m_observe_median": float(np.median(vals("lift_m_observe_median"))) if vals("lift_m_observe_median") else None,
                "lift_m_transport_median": float(np.median(vals("lift_m_transport_median"))) if vals("lift_m_transport_median") else None,
                "upper_gap_insert_min_m": min(vals("upper_gap_insert_min_m")) if vals("upper_gap_insert_min_m") else None,
                "lift_tilt_max_rad": max(vals("tilt_rad", lifted)) if vals("tilt_rad", lifted) else None,
                "lateral_min_m": min(vals("lateral_min_m", lifted)) if vals("lateral_min_m", lifted) else None,
                "insertion_median_m": (float(np.median([x for r in lifted for x in r["insertion_m"] if x is not None]))
                                       if any(x is not None for r in lifted for x in r["insertion_m"]) else None),
                "pushed_max_m": max(vals("pushed_m", lifted)) if vals("pushed_m", lifted) else None,
                "outline_motion_max_m": max(vals("outline_motion_max_m", lifted)) if vals("outline_motion_max_m", lifted) else None,
                "base_tilt_max_rad": max(vals("base_tilt_max_rad")) if vals("base_tilt_max_rad") else None,
            }
        out[group] = g
    return out


def age_comparison(table: dict, measured: dict) -> dict:
    """Per age: the arm's maximum odometry error against the operating table's bound (reported only)."""
    if list(measured.get("ages_s") or []) != list(table["ages_s"]):
        return {"error": f"ages differ: {measured.get('ages_s')} vs {table['ages_s']}"}
    rows = []
    for i, age in enumerate(table["ages_s"]):
        p, y = measured["position_max_m"][i], measured["yaw_max_rad"][i]
        rows.append({"age_s": age, "position_max_m": p, "table_position_m": table["position_max_m"][i],
                     "yaw_max_rad": y, "table_yaw_rad": table["yaw_max_rad"][i],
                     "over": p > table["position_max_m"][i] or y > table["yaw_max_rad"][i]})
    return {"rows": rows, "ages_over": [r["age_s"] for r in rows if r["over"]]}


def overscan_share(runs: list) -> dict:
    """Amendment 2's stop rule, per arm over every run folder of the batch (first runs and reruns, a hung session
    without a result included): the share that ended on the overscan TypeError (amendment-2 ends and later ones)."""
    out = {}
    for a in ARMS:
        mine = [r for r in runs if r["arm"] == a]
        hit = [r["name"] for r in mine if r.get("overscan_reason")]
        out[a] = {"runs": len(mine), "overscan": hit,
                  "over_hold_share": bool(mine) and len(hit) / len(mine) > OVERSCAN_HOLD_SHARE}
    return out


def registered(name: str) -> bool:
    try:
        arm, key, _ = split_name(name)
    except ValueError:
        return False
    return arm in ARMS and key in EXPECTED_KEYS


def judge(batch: Path, age_table: dict | None = None, ages: dict | None = None) -> dict:
    bundle = bundle_rows(batch)
    folders = [p for p in sorted(batch.iterdir()) if p.is_dir() and p.name.startswith("f1_")]
    runs = [batch_run(p, p.name, bundle, live="_live_" in p.name) for p in folders if registered(p.name)]
    unregistered = [p.name for p in folders if not registered(p.name)]
    pairs = pair_table(runs)
    used = [r for v in pairs.values() for r in v.values()]
    groups = {}
    for key, pair in pairs.items():
        groups.setdefault(group_of(key), {})[key] = pair
    # safety and infrastructure counts take every run, a replaced first pair's too (pallet-mass section 4 validity)
    verdict = {g: group_verdict(g, p, runs) for g, p in sorted(groups.items())}
    residual = residual_verdict(pairs)
    identity_bad = {r["name"]: r["identity_problems"] for r in used if not r["infra"] and r["identity_problems"]}
    scenes = {}
    for r in used:
        if not r["infra"]:
            scenes.setdefault((r["arm"], shape_of(r["key"])), set()).add(
                json.dumps(r["identity"].get("scene_sha256"), sort_keys=True))
    scene_problems = [f"{a}/{sh}: {len(v)} scene hashes" for (a, sh), v in sorted(scenes.items()) if len(v) != 1]
    for sh in PALLET_KG:
        if scenes.get(("O", sh)) and scenes.get(("O", sh)) == scenes.get(("F1", sh)):
            scene_problems.append(f"{sh}: the two arms ran the same scene")
    parts = [v["verdict"] for v in verdict.values()] + [residual["verdict"]]
    overall = ("inconclusive" if identity_bad or scene_problems else "fail" if "fail" in parts
               else "inconclusive" if "inconclusive" in parts else "needs_cause" if "needs_cause" in parts else "pass")
    age = ({a: age_comparison(age_table, m) for a, m in (ages or {}).items()} if age_table is not None else None)
    for r in runs:
        r.pop("_samples", None)
    return {"overall": overall, "groups": verdict, "residual": residual, "identity_mismatch": identity_bad,
            "scene_problems": scene_problems, "scenes": {f"{a}/{sh}": sorted(v) for (a, sh), v in scenes.items()},
            "unregistered_folders": unregistered, "reported": reported(pairs), "odometry_age": age,
            "overscan": overscan_share(runs), "runs": runs}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("batch", type=Path)
    p.add_argument("--json", type=Path, required=True)
    p.add_argument("--age-table", type=Path, help="the operating odometry age table")
    p.add_argument("--age", action="append", default=[], metavar="ARM=PATH",
                   help="tools/p0a_measure.py age output for that arm's runs")
    args = p.parse_args()
    ages = {k: json.loads(Path(v).read_text()) for k, v in (a.split("=", 1) for a in args.age)}
    table = json.loads(args.age_table.read_text()) if args.age_table else None
    out = judge(args.batch, table, ages)
    args.json.write_text(json.dumps(out, indent=1, default=str) + "\n")
    print(json.dumps({"overall": out["overall"], **{g: v["verdict"] for g, v in out["groups"].items()},
                      "residual": out["residual"]["verdict"]}))


if __name__ == "__main__":
    main()
