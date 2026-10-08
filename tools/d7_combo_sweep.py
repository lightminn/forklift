"""Priority-5 plan D7 CPU judgement: 48 planner combinations on B_M and the near-goal set R.

docs/plans/2026-10-04-lidar-obstacle-map.md, D7 "판정 개정" and "D7a 후보 보완". For each
combination and each seed the B_M baseline planned successfully: the whole mission again
(B_M non-regression) and the 24 near-goal replans of R -- from the transport leg goal
(pre-delivery) and the return goal, start = goal composed with each fixed residual,
planned as the runner's stall replan does (plan_transport_leg / plan_return_leg, truth
rectangles, the travel config, 20 s per ladder). One JSON per (combination, seed), resumed
when its inputs hash the same; ``judge`` applies the adoption rule.

    python tools/d7_combo_sweep.py run --bm-dir artifacts/.../BM_v2 --combos 0-47 --output OUT
    python tools/d7_combo_sweep.py judge OUT
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import statistics
import sys
import time
from dataclasses import replace
from itertools import product
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools import bay_planning_sweep as BAY  # noqa: E402

# Fixed combination order (plan D7 "고정 조합 순서"): penalty high first, reverse ratio high
# first, D7b off first, Dubins first, no shot cap first. Index 0 is the current setting.
COMBOS = [
    {"gear_change_penalty_m": g, "reverse_penalty": r, "turn_cycles": c, "goal_connection": k, "shot_cap_m": cap}
    for g, r, c, k, cap in product((1.0, 0.5, 0.2), (1.3, 1.1), (False, True), ("dubins", "reeds_shepp"), (None, 4.0))
]
LOOP_M = 4.0  # plan D7: a start within 0.1 m of its goal and more than this to the goal


def residuals(goal: str) -> list[tuple[str, float, float, float]]:
    """(kind, along, lateral, yaw) residuals of R at this goal, in the goal's frame."""
    mixed = [("mixed", -0.02, lat, yaw) for lat in (-0.06, -0.04, 0.04, 0.06) for yaw in (-0.05, 0.05)]
    lateral = [("lateral", 0.0, lat, 0.0) for lat in (-0.04, 0.04)]
    heading = 0.06 if goal == "transport" else 0.04
    yaw_only = [("yaw", 0.0, 0.0, sign * heading) for sign in (-1.0, 1.0)]
    return mixed + lateral + yaw_only


def compose(goal, along, lateral, yaw):
    c, s = math.cos(goal[2]), math.sin(goal[2])
    return (goal[0] + along * c - lateral * s, goal[1] + along * s + lateral * c, goal[2] + yaw)


def _entry(leg, extra_m=0.0) -> dict:
    record = BAY._plan_entry(leg)
    record["length_to_goal_m"] = None if not leg.success else float(leg.length_m) - extra_m
    for name in ("cycles_queued", "cycles_in_path", "cycles_generated", "shots_capped", "shot_fallback",
                 "root_shot", "pruned_children"):
        record[name] = getattr(leg, name, None)
    return record


def plan_seed(job: dict) -> dict:
    """B_M mission and the 24 R replans for one seed under one combination."""
    from forklift_core.planning import Pose2D
    from forklift_core.planning.pallet_mission import (
        SearchBudget,
        make_transport_planner_config,
        plan_return_leg,
        plan_transport_leg,
        site_poses,
    )

    combo = COMBOS[job["combo"]]
    scenario, factory, geometry, _ = BAY.factory_scene(job)
    config = replace(make_transport_planner_config(
        curvature_limit_inv_m=job["curvature"], clearance_m=job["clearance"], max_expansions=job["max_expansions"],
    ), **combo)
    travel = replace(config, obstacle_heuristic_resolution_m=0.25)
    record = {"combo": job["combo"], "settings": combo, "seed": job["seed"], "input_sha256": job["input_sha256"],
              "scenario_sha256": BAY.scene_sha256(scenario, geometry, job["min_top_m"])}
    budget = SearchBudget(job["search_budget_s"])
    trace = []
    started = time.monotonic()
    plan, chosen = BAY.plan_factory_mission(scenario, factory, geometry, config, travel, budget, trace, True)
    record["mission"] = {
        "outcome": "success" if plan is not None and plan.success else "planning_failure",
        "status": "observe_no_candidate" if plan is None else plan.status, "observation": chosen,
        "timeouts": sum(1 for t in trace if t.get("status") == "timeout"),
        "wall_s": round(time.monotonic() - started, 3),
        "gear_changes": sum(t.get("gear_changes", 0) for t in trace if t.get("stage") in ("transport_search", "return_home")),
    }
    goals = {
        "transport": tuple(float(v) for v in (lambda p: (p.x_m, p.y_m, p.yaw_rad))(site_poses(scenario.destination, geometry)["predelivery"])),
        "return": (scenario.start_rear.x_m, scenario.start_rear.y_m, scenario.start_rear.yaw_rad),
    }
    rows = []
    for goal_name, goal in goals.items():
        for kind, along, lateral, yaw in residuals(goal_name):
            start = Pose2D(*compose(goal, along, lateral, yaw))
            ladder = SearchBudget(job["search_budget_s"])
            t0 = time.monotonic()
            if goal_name == "transport":
                leg = plan_transport_leg(scenario, start, config, geometry=geometry, travel_config=travel, deadline=ladder)
                entry = _entry(leg, geometry.delivery_straight_m if leg.success else 0.0)
            else:
                leg = plan_return_leg(scenario, start, scenario.start_rear, config, geometry=geometry,
                                      travel_config=travel, deadline=ladder)
                entry = _entry(leg)
            entry.update(goal=goal_name, kind=kind, residual=[along, lateral, yaw],
                         wall_s=round(time.monotonic() - t0, 3), ladder_s=[round(v, 3) for v in ladder.spent_s])
            entry.pop("search_attempts", None)
            rows.append(entry)
    record["R"] = rows
    return record


def bm_records_sha256(bm_dir: Path) -> str:
    """One hash over every B_M seed record (path name and bytes), in seed-file order."""
    digest = hashlib.sha256()
    for path in sorted(Path(bm_dir).glob("part*/seed_*.json")):
        digest.update(f"{path.parent.name}/{path.name}".encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def freeze_bm(args) -> dict:
    """Write the B_M reference manifest beside a run that predates the hash in its inputs."""
    out = Path(args.output)
    inputs = json.loads((out / "inputs.json").read_text())
    manifest = {"bm_dir": inputs["bm_dir"], "bm_records_sha256": bm_records_sha256(Path(inputs["bm_dir"])),
                "run_sha256": inputs["run_sha256"]}
    path = out / "bm_manifest.json"
    if path.exists():
        raise SystemExit(f"{path} exists: a frozen reference is never rewritten")
    path.write_text(json.dumps(manifest, indent=2))
    return manifest


def _ranges(text: str) -> list[int]:
    out = []
    for part in text.split(","):
        first, _, last = part.partition("-")
        out.extend(range(int(first), int(last or first) + 1))
    return out


def run(args) -> int:
    settings = yaml.safe_load(args.settings.read_text(encoding="utf-8"))
    bm = {}
    for path in sorted(Path(args.bm_dir).glob("part*/seed_*.json")):
        rec = json.loads(path.read_text())
        bm[rec["seed"]] = rec
    seeds = sorted(s for s, r in bm.items() if r.get("outcome") == "success")
    bm_successes = list(seeds)
    if args.seeds:
        seeds = [s for s in seeds if s in set(_ranges(args.seeds))]
    inputs = {
        "settings_sha256": hashlib.sha256(args.settings.read_bytes()).hexdigest(),
        "forklift_urdf_sha256": hashlib.sha256(args.forklift_urdf.read_bytes()).hexdigest(),
        "pallet_geometry_sha256": hashlib.sha256(args.pallet_geometry.read_bytes()).hexdigest(),
        "factory_layout_sha256": hashlib.sha256(args.factory_layout.read_bytes()).hexdigest(),
        # Everything the scene and the plans are computed from (Codex D7 5th P1-2).
        "sources_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in [
            *sorted((ROOT / "src/forklift_core/planning").glob("*.py")),
            ROOT / "src/forklift_core/perception/pallet_geometry.py", ROOT / "sim/isaac/factory_assets.py",
            ROOT / "sim/isaac/insertion_geometry.py", ROOT / "tools/bay_planning_sweep.py",
            Path(__file__).resolve()]},
        "bm_successes": None,  # filled below
        "curvature": settings["planner_curvature_inv_m"], "clearance": settings["planning_clearance_m"],
        "max_expansions": 30000, "search_budget_s": args.search_budget_s, "reserve_m": args.insertion_reserve_m,
        "min_top_m": 1.15, "alignment_straight_m": 2.1, "delivery_straight_m": 1.5, "withdrawal_m": 0.55,
        "obstacles": 4,
    }
    inputs["bm_successes"] = bm_successes
    inputs["bm_records_sha256"] = bm_records_sha256(Path(args.bm_dir))
    run_hash = hashlib.sha256(json.dumps(inputs, sort_keys=True).encode()).hexdigest()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "inputs.json").write_text(json.dumps({**inputs, "run_sha256": run_hash, "seeds": seeds,
                                                         "bm_dir": str(args.bm_dir)}, indent=2))
    for combo in _ranges(args.combos):
        out = args.output / f"combo_{combo:02d}"
        out.mkdir(exist_ok=True)
        for seed in seeds:
            path = out / f"seed_{seed}.json"
            if path.exists() and json.loads(path.read_text()).get("input_sha256") == run_hash:
                continue
            job = {**inputs, "combo": combo, "seed": seed, "input_sha256": run_hash,
                   "forklift_urdf": str(args.forklift_urdf), "pallet_geometry": str(args.pallet_geometry),
                   "factory_layout": str(args.factory_layout)}
            record = plan_seed(job)
            tmp = path.with_suffix(".tmp")
            tmp.write_text(json.dumps(record))
            os.replace(tmp, path)
        print(f"combo {combo} done", flush=True)
    return 0


def _stats(values):
    if not values:
        return None
    v = sorted(values)
    return {"median": statistics.median(v), "p95": v[min(len(v) - 1, int(0.95 * len(v)))], "max": v[-1]}


def _timed_out_pairs(combos: dict) -> list[tuple[int, int]]:
    """Plan D7 측정 복구 ①, recomputed from the run's own records: every pair with a mission or
    R timeout, plus combination 0 on every such seed (the control)."""
    timed = {(c, r["seed"]) for c, recs in combos.items() for r in recs
             if r["mission"].get("timeouts", 0) > 0 or any(x["status"] == "timeout" for x in r["R"])}
    return sorted(timed | {(0, s) for _, s in timed})


def _apply_overlay(overlay: Path, combos: dict, run_hash: str) -> dict:
    """Replace the registered pairs with their re-run records (plan D7 측정 복구 ④).

    The registered list must be the one the run's records define (Codex overlay review P2-1)
    and name this run; every record sits at its canonical path, once, and nothing else is in
    the combination folders (P2-2); a missing record is an error, never filled from the
    original run. A re-run that failed again is used as it is.
    """
    registered = json.loads((overlay / "pairs.json").read_text())
    listed = [(int(c), int(s)) for c, s in registered["pairs"]]
    if registered.get("run_sha256") != run_hash:
        raise SystemExit("pairs.json was registered for another run")
    if len(set(listed)) != len(listed):
        raise SystemExit("pairs.json lists a pair twice")
    expected = _timed_out_pairs(combos)
    if sorted(listed) != expected:
        raise SystemExit(f"pairs.json is not the registered selection: missing {sorted(set(expected) - set(listed))}, "
                         f"extra {sorted(set(listed) - set(expected))}")
    allowed = set()
    for c, s in listed:
        allowed |= {f"combo_{c:02d}/seed_{s}.json", f"combo_{c:02d}/seed_{s}.instrument.json",
                    f"combo_{c:02d}/seed_{s}.claim"}
    present = {str(p.relative_to(overlay)) for p in overlay.rglob("*") if p.is_file() and p.parent != overlay}
    if present - allowed:
        raise SystemExit(f"overlay holds files outside the registered pairs: {sorted(present - allowed)[:5]}")
    missing = [(c, s) for c, s in listed if f"combo_{c:02d}/seed_{s}.json" not in present]
    if missing:
        raise SystemExit(f"overlay records missing for {missing}")
    for c, s in listed:
        rec = json.loads((overlay / f"combo_{c:02d}" / f"seed_{s}.json").read_text())
        if rec["combo"] != c or rec["seed"] != s or rec["settings"] != COMBOS[c]:
            raise SystemExit(f"overlay combo_{c:02d}/seed_{s}.json holds another pair")
        if c not in combos or sum(1 for r in combos[c] if r["seed"] == s) != 1:
            raise SystemExit(f"overlay pair ({c}, {s}) is not one record of the run")
        combos[c] = [rec if r["seed"] == s else r for r in combos[c]]
    return {"pairs": listed, "pairs_sha256": hashlib.sha256((overlay / "pairs.json").read_bytes()).hexdigest()}


def _sidecar(overlay: Path, rec: dict) -> dict:
    """The control's instrument file, tied to its pair and to the record beside it (Codex
    overlay review P2-3): same pair, and its trace gives the record's timeouts, chosen
    observation candidate and gear changes."""
    c, s = rec["combo"], rec["seed"]
    inst = json.loads((overlay / f"combo_{c:02d}" / f"seed_{s}.instrument.json").read_text())
    trace = inst["trace"]
    # Tied to the very execution (Codex overlay review 2nd P2): by the record's hash when the
    # driver wrote one; otherwise (the 2026-10-08 rerun's driver did not) by its timing -- the
    # mission's wall time is its ladders plus non-search work, so a sidecar from another
    # execution of the same pair has to land within max(0.5 s, 5 %) of it to pass.
    record_bytes = (overlay / f"combo_{c:02d}" / f"seed_{s}.json").read_bytes()
    if "record_sha256" in inst:
        bound = inst["record_sha256"] == hashlib.sha256(record_bytes).hexdigest()
    else:
        wall, ladders = rec["mission"].get("wall_s"), sum(inst["mission_ladder_s"])
        bound = wall is not None and -0.002 <= wall - ladders <= max(0.5, 0.05 * wall)
    chosen = [{"candidate_index": t.get("candidate_index"), "extended": t.get("extended")}
              for t in trace if t.get("stage") == "observe_candidate" and t.get("status") == "success"]
    mission = rec["mission"]
    consistent = (
        bound and inst.get("combo") == c and inst.get("seed") == s
        and sum(1 for t in trace if t.get("status") == "timeout") == mission.get("timeouts", 0)
        and chosen == ([mission["observation"]] if mission.get("observation") is not None else [])
        and sum(t.get("gear_changes", 0) for t in trace if t.get("stage") in ("transport_search", "return_home"))
        == mission.get("gear_changes", 0)
    )
    if not consistent:
        raise SystemExit(f"combo_{c:02d}/seed_{s}.instrument.json does not belong to its record")
    return inst


def _control(overlay: Path, pairs, base_recs: list, base_row: dict, bm_records: dict) -> dict:
    """Plan D7 측정 복구 ③: combination 0 keeps every B_M success with no new timeout, and its
    ladder wall times over B_M's (B_M ladders of 1 s or more) have a median of at most 1.10.
    Expansions per stage against the B_M trace are reported only: B_M and this run differ in
    code, so a pass shows the operating criterion met at the lowered load -- it does not
    separate load from a code change in the cost per expansion."""
    seeds = sorted(s for c, s in pairs if c == 0)
    by_seed = {r["seed"]: r for r in base_recs}
    ratios, ladder_mismatch, expansions_differ = [], [], []
    for s in seeds:
        inst = _sidecar(overlay, by_seed[s])
        mine, theirs = inst["mission_ladder_s"], bm_records[s].get("ladder_wall_s") or []
        if len(mine) != len(theirs):
            ladder_mismatch.append(s)
        else:
            ratios += [a / b for a, b in zip(mine, theirs) if b >= 1.0]
        stages = lambda trace: [(t.get("stage"), t.get("status"), t.get("expansions")) for t in trace]  # noqa: E731
        if stages(inst["trace"]) != stages(bm_records[s].get("trace", [])):
            expansions_differ.append(s)
    lost = list(base_row["mission_lost"])
    median = statistics.median(ratios) if ratios else None
    passed = (not lost and base_row["new_mission_timeouts"] == 0 and not ladder_mismatch
              and median is not None and median <= 1.10)
    return {"seeds": seeds, "mission_lost": lost, "new_mission_timeouts": base_row["new_mission_timeouts"],
            "ladder_ratio_median": median, "ladder_ratio_max": max(ratios) if ratios else None,
            "ladder_ratios": len(ratios), "ladder_count_mismatch": ladder_mismatch,
            "stage_expansions_differ": expansions_differ, "passed": passed}


def judge(args) -> dict:
    """The plan's D7 adoption rule over one run directory (every part from one input hash)."""
    out = Path(args.output)
    inputs = json.loads((out / "inputs.json").read_text())
    run_hash, bm_seeds = inputs["run_sha256"], list(inputs["bm_successes"])
    # The original B_M records are the reference for mission timeouts too, not combination
    # 0's rerun (Codex D7 6th P2-1): read, hashed, and checked against the run's list.
    bm_dir = Path(getattr(args, "bm_dir", None) or inputs["bm_dir"])
    # The B_M records must be the frozen reference (Codex D7 7th P2): the hash recorded in
    # the run's inputs, or in bm_manifest.json for a run that predates it.
    expected = inputs.get("bm_records_sha256")
    if expected is None and (out / "bm_manifest.json").exists():
        manifest = json.loads((out / "bm_manifest.json").read_text())
        if manifest["run_sha256"] != run_hash:
            raise SystemExit("bm_manifest.json belongs to another run")
        expected = manifest["bm_records_sha256"]
    if expected is None:
        raise SystemExit("no frozen B_M hash: run `freeze-bm` before judging this run")
    bm_sha = bm_records_sha256(bm_dir)
    if bm_sha != expected:
        raise SystemExit(f"the B_M records in {bm_dir} are not the frozen reference")
    bm_paths = sorted(bm_dir.glob("part*/seed_*.json"))
    bm_records = {json.loads(p.read_text())["seed"]: json.loads(p.read_text()) for p in bm_paths}
    if sorted(s for s, r in bm_records.items() if r.get("outcome") == "success") != sorted(bm_seeds):
        raise SystemExit("the B_M records do not match the run's B_M successes")
    bm_timeouts = {s: sum(1 for t in bm_records[s].get("trace", []) if t.get("status") == "timeout") for s in bm_seeds}
    combos = {}
    for d in sorted(out.glob("combo_*")):
        c = int(d.name.split("_")[1])
        recs = [json.loads(p.read_text()) for p in sorted(d.glob("seed_*.json"))]
        # Folder, record and settings must agree (Codex D7 6th P2-2).
        if any(r["combo"] != c or r["settings"] != COMBOS[c] for r in recs):
            raise SystemExit(f"{d.name} holds a record of another combination")
        if recs:
            combos[c] = recs
    overlay = None
    if getattr(args, "overlay", None) is not None:
        overlay = _apply_overlay(Path(args.overlay), combos, run_hash)
    if 0 not in combos:
        raise SystemExit("the baseline combination 0 is missing")
    scenes = {}
    for c, recs in combos.items():
        if sorted(r["seed"] for r in recs) != sorted(bm_seeds):
            raise SystemExit(f"combination {c} does not cover the B_M successes")
        for r in recs:
            # One experiment only (Codex D7 5th P1-2): every record from this run's inputs,
            # every seed's scene the same in every combination.
            if r["input_sha256"] != run_hash or r["settings"] != COMBOS[r["combo"]]:
                raise SystemExit(f"combination {c} seed {r['seed']} is from other inputs")
            if scenes.setdefault(r["seed"], r["scenario_sha256"]) != r["scenario_sha256"]:
                raise SystemExit(f"seed {r['seed']} has two scenes")
            if r["scenario_sha256"] != bm_records[r["seed"]].get("scenario_sha256"):
                raise SystemExit(f"seed {r['seed']}: not the scene B_M planned")
    base = {r["seed"]: r for r in combos[0]}
    base_r_ok = {(s, i) for s in bm_seeds for i, row in enumerate(base[s]["R"]) if row["status"] == "success"}

    def is_loop(row):
        return row["status"] == "success" and row["length_to_goal_m"] is not None and row["length_to_goal_m"] > LOOP_M

    base_loops = sum(1 for s in bm_seeds for row in base[s]["R"] if is_loop(row))
    rows = {}
    for c, recs in sorted(combos.items()):
        by_seed = {r["seed"]: r for r in recs}
        # The B_M successes are the reference, not combination 0's rerun (Codex D7 5th P1-1).
        mission_lost = [s for s in bm_seeds if by_seed[s]["mission"]["outcome"] != "success"]
        new_timeouts = sum(max(0, by_seed[s]["mission"]["timeouts"] - bm_timeouts[s]) for s in bm_seeds)
        r_lost = [(s, i) for (s, i) in base_r_ok if by_seed[s]["R"][i]["status"] != "success"]
        new_r_timeouts = sum(1 for s in bm_seeds for i, row in enumerate(by_seed[s]["R"])
                             if row["status"] == "timeout" and base[s]["R"][i]["status"] != "timeout")
        ok_rows = [(s, i, row) for s in bm_seeds for i, row in enumerate(by_seed[s]["R"])
                   if (s, i) in base_r_ok and row["status"] == "success"]
        per_goal = {}
        for g in ("transport", "return"):
            rows_g = [row for s in bm_seeds for row in by_seed[s]["R"] if row["goal"] == g]
            walls = sorted(x for row in rows_g for x in row["ladder_s"])
            per_goal[g] = {
                "loops": sum(1 for row in rows_g if is_loop(row)),
                "length_to_goal": _stats([row["length_to_goal_m"] for _, _, row in ok_rows if row["goal"] == g]),
                "gear_changes": _stats([row["gear_changes"] for _, _, row in ok_rows if row["goal"] == g]),
                "ladder_wall_p99_s": walls[min(len(walls) - 1, int(0.99 * len(walls)))] if walls else None,
                "root_shot": {k: sum(1 for row in rows_g if row.get("root_shot") == k) for k in ("taken", "capped", "none")},
            }
        pooled = [row["length_to_goal_m"] for _, _, row in ok_rows]
        rows[c] = {
            "settings": recs[0]["settings"], "mission_lost": mission_lost, "new_mission_timeouts": new_timeouts,
            "r_lost": len(r_lost), "new_r_timeouts": new_r_timeouts,
            "loops": sum(1 for s in bm_seeds for row in by_seed[s]["R"] if is_loop(row)),
            "per_goal": per_goal,
            "length_median_pooled": statistics.median(pooled) if pooled else None,
            "gear_changes_median": statistics.median([row["gear_changes"] for _, _, row in ok_rows]) if ok_rows else None,
            "cycles_generated": sum(row.get("cycles_generated") or 0 for s in bm_seeds for row in by_seed[s]["R"]),
            "cycles_queued": sum(row.get("cycles_queued") or 0 for s in bm_seeds for row in by_seed[s]["R"]),
            "cycles_in_path": sum(row.get("cycles_in_path") or 0 for s in bm_seeds for row in by_seed[s]["R"]),
            "shots_capped": sum(row.get("shots_capped") or 0 for s in bm_seeds for row in by_seed[s]["R"]),
            "shot_fallbacks": sum(1 for s in bm_seeds for row in by_seed[s]["R"] if row.get("shot_fallback")),
        }
        rows[c]["passes"] = (not mission_lost and new_timeouts == 0 and not r_lost and new_r_timeouts == 0
                             and base_loops >= 1 and rows[c]["loops"] <= base_loops // 2)

    # D7b with no recorded effect gives way to its off twin before ranking (Codex D7 5th P2-4).
    candidates = []
    for c in rows:
        if not rows[c]["passes"]:
            continue
        if COMBOS[c]["turn_cycles"] and rows[c]["cycles_in_path"] == 0:
            twin = COMBOS.index({**COMBOS[c], "turn_cycles": False})
            if twin in rows and rows[twin]["passes"]:
                continue
        candidates.append(c)

    def key(c):
        r = rows[c]
        changes = sum(1 for k, v in r["settings"].items() if v != COMBOS[0][k])
        median = r["length_median_pooled"] if r["length_median_pooled"] is not None else math.inf
        return (r["loops"], median, r["gear_changes_median"] or 0, changes, c)

    adopted = min(candidates, key=key) if candidates else None
    control = None
    if overlay is not None:
        # Plan D7 측정 복구 ③: a failed control means no adoption at all.
        control = _control(Path(args.overlay), overlay["pairs"], combos[0], rows[0], bm_records)
        if not control["passed"]:
            adopted = None
    return {"run_sha256": run_hash, "bm_records_sha256": bm_sha, "bm_timeouts": sum(bm_timeouts.values()),
            "overlay": overlay, "control": control,
            "baseline_loops": base_loops, "baseline_r_successes": len(base_r_ok),
            "seeds": len(bm_seeds), "baseline_rerun_mission_lost": rows[0]["mission_lost"],
            "adopted": adopted, "adopted_settings": COMBOS[adopted] if adopted is not None else None,
            "passing": [c for c in rows if rows[c]["passes"]], "ranked_candidates": sorted(candidates, key=key),
            "combos": rows}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    r = sub.add_parser("run")
    r.add_argument("--bm-dir", type=Path, required=True)
    r.add_argument("--combos", default="0-47")
    r.add_argument("--seeds", default=None)
    r.add_argument("--output", type=Path, required=True)
    r.add_argument("--settings", type=Path, default=ROOT / "config/isaac_transport_measured.yaml")
    r.add_argument("--forklift-urdf", type=Path, default=ROOT / "sim/models/dls08_measured/forklift.urdf")
    r.add_argument("--pallet-geometry", type=Path, default=ROOT / "config/pallet_geometry_epal6.yaml")
    r.add_argument("--factory-layout", type=Path, default=ROOT / "config/factory_south_hall.yaml")
    r.add_argument("--insertion-reserve-m", type=float, default=0.016)
    r.add_argument("--search-budget-s", type=float, default=20.0)
    f = sub.add_parser("freeze-bm")
    f.add_argument("output", type=Path)
    j = sub.add_parser("judge")
    j.add_argument("output", type=Path)
    j.add_argument("--bm-dir", type=Path, default=None, help="default: the run's recorded B_M directory")
    j.add_argument("--overlay", type=Path, default=None,
                   help="re-run records of the registered pairs (pairs.json) that replace the run's")
    args = parser.parse_args(argv)
    if args.command == "run":
        return run(args)
    if args.command == "freeze-bm":
        print(json.dumps(freeze_bm(args), indent=2))
        return 0
    print(json.dumps(judge(args), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
