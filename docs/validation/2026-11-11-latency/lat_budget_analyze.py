"""The layer latency budget check's judgement (docs/plans/2026-11-11-layer-latency-budget.md 3, fixed before the results).

    python lat_budget_analyze.py OUT_ROOT --snap SNAP [--json OUT.json]

OUT_ROOT holds RUNS_budget.jsonl (and RUNS_rerun.jsonl), the run folders it names and the bundles' b*/results.jsonl.
Latency test A's analyzer (lat_analyze.py) is reused as registered: environment validity, safety from the B0 file's
stopping model, the stop-vs-end split, the mission and operating metrics against the same scene-seed B0, the exposure,
the cell class with aisle_rack's baseline exceptions, reruns beside their first runs, and the determinism repeats
(here B0 and Ub6 on line_2 / random_1 seed 0) compared without the layer worker's timing fields (exception 1). Then:

- every arm: safety violations 0 (a violation is a defect, priority 1);
- Ub3 / Ub6 / Ub12 (inside the budget): tolerated, the A 4.4 priorities with all three in the must-tolerate band;
- Bb10 (the budget's cost): operating metrics within B0's bounds, else 'operating_degraded' reported to the lead;
- Uo9 (over the test budget 0.05): passes when all hold -- safety 0, watchdog events >= 1, invariant violations 0
  (no watchdog tick drove), and a run whose watchdog stop passed overdue_end_s ended on the first watchdog tick past it
  as layer_latency_exceeded with a safe stop (A 4.2); anything unreadable fails; a run that never reached the limit
  records the end function as unexposed.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]

ARMS = ("B0", "Bb10", "Ub3", "Ub6", "Ub12", "Uo9")
NOMINAL_TICKS = {"B0": 0, "Bb10": 0, "Ub3": 3, "Ub6": 6, "Ub12": 12, "Uo9": 9}
INJECTED = {"Ub3", "Ub6", "Ub12", "Uo9"}
TOLERATE = {"Ub3", "Ub6", "Ub12"}
END_REASON = "layer_latency_exceeded"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


LA = _load("lat_analyze", HERE / "lat_analyze.py")


# Exception 2 (a lead's post-result exception, 2026-11-13, with conditions): the slot's CPU set the layer worker's
# record writes (result.json layer_worker.process_affinity, and process_cores made from it), compared by its size
# only -- the CPU numbers left out, the count kept, so two repeats on CPU sets of different sizes still differ (a
# library's thread pool may follow the count: OpenBLAS, Ceres; Codex's review) -- and only while the worker does not
# pin itself (mode not thread, cpus and affinity None). A thread-mode record keeps both fields as they are.
AFFINITY_FIELDS = ("process_affinity", "process_cores")


def affinity_exempt(layer_worker) -> bool:
    """Exception 2's condition: the worker never set its CPUs, so the slot's CPU set is a record only."""
    return (isinstance(layer_worker, dict) and layer_worker.get("mode") != "thread"
            and layer_worker.get("cpus") is None and layer_worker.get("affinity") is None)


def strip_affinity(dc) -> None:
    """Exception 2 on top of whatever ``dc._items`` does already (exception 1 first)."""
    inner = dc._items

    def items(run):
        it = inner(run)
        for doc in it.get("result.json", ("json", []))[1]:
            lw = doc.get("layer_worker") if isinstance(doc, dict) else None
            if affinity_exempt(lw):
                for key in AFFINITY_FIELDS:
                    if key in lw:
                        lw[key] = {"count": len(lw[key]) if isinstance(lw[key], list) else None}
        return it

    dc._items = items


def budget_problems(arm: str, result: dict | None, run: Path) -> list:
    """What the budget check needs beyond A's validity: B0 runs without a budget, the other arms with theirs (0.05 for
    Uo9, 0.10 otherwise) and their watchdog record; the injected arms with their injection record."""
    if result is None:
        return []
    out = []
    lb = result.get("layer_budget")
    want = None if arm == "B0" else (0.05 if arm == "Uo9" else 0.10)
    got = None if not isinstance(lb, dict) else lb.get("budget_s")
    if (want is None) != (got is None) or (want is not None and abs(float(got) - want) > 1e-9):
        out.append(f"layer budget {got}, not {want}")
    if want is not None and (not isinstance(lb, dict) or not isinstance(lb.get("watchdog"), dict)):
        out.append("no watchdog record")
    if want is not None and not (run / "layer_latency.json").exists():
        out.append("no layer_latency.json")
    if arm in INJECTED:
        if "latency_injection" not in result:
            out.append("no latency_injection record")
        if not (run / "layer_worker.json").exists():
            out.append("no layer_worker.json")
    return out


def watchdog_of(result: dict, log: dict | None) -> dict:
    """The watchdog's record: events, ticks, the longest stop, the online invariant and how the run ended."""
    lb = result.get("layer_budget") or {}
    wd = lb.get("watchdog") or {}
    ticks = (log or {}).get("overdue_ticks")
    def num(x):
        return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)

    end = lb.get("overdue_end_s") if isinstance(lb, dict) else None
    # every row's time and held stop finite numbers, its reason a name, the limit finite (Codex round 3 P2: a NaN
    # compares false and would read as below the limit)
    readable = (isinstance(ticks, list) and num(end) and num(wd.get("longest_s"))
                and all(isinstance(r, list) and len(r) >= 7 and num(r[0]) and num(r[6])
                        and r[5] in ("layer_overdue", END_REASON) for r in ticks))
    exceeded = [r for r in ticks if r[5] == END_REASON] if readable else None
    # the first row past the limit (held > overdue_end_s) must be the end row and the last row (Codex round 2 P2)
    past = [i for i, r in enumerate(ticks) if float(r[6]) > float(end) + 1e-9] if readable and end is not None else None
    return {"events": wd.get("events"), "ticks": wd.get("ticks"), "longest_s": wd.get("longest_s"),
            "overdue_end_s": lb.get("overdue_end_s"), "invariant_violations": lb.get("invariant_violations", 0)
            if isinstance(lb, dict) and "watchdog" in lb else None,
            "d_max_s": lb.get("d_max_s"), "scans_used": lb.get("scans_used"),
            "log_readable": readable,
            "exceeded_rows": None if exceeded is None else len(exceeded),
            "last_row_exceeded": None if not readable or not ticks else ticks[-1][5] == END_REASON,
            "first_past_is_end": None if past is None else (bool(past) and past[0] == len(ticks) - 1
                                                             and ticks[past[0]][5] == END_REASON),
            "rows_past_limit": None if past is None else len(past),
            "failure_reason": result.get("failure_reason")}


def uo9_verdict(row: dict) -> dict:
    """Plan 3, Uo9 (all must hold; unreadable fails)."""
    s, w = row.get("safety") or {}, row.get("watchdog") or {}
    checks = {"safety_0": s.get("violations") == 0,
              "events_ge_1": (w.get("events") or 0) >= 1,
              "invariant_0": w.get("invariant_violations") == 0,
              "log_readable": w.get("log_readable") is True}
    longest, end = w.get("longest_s"), w.get("overdue_end_s")
    if longest is None or end is None or not w.get("log_readable"):
        checks["end_readable"] = False
        exposed = None
    else:
        exposed = longest > end + 1e-9
    if exposed:
        checks["ended_as_exceeded"] = w.get("failure_reason") == END_REASON
        checks["ended_on_first_tick_past"] = (w.get("exceeded_rows") == 1 and w.get("last_row_exceeded") is True
                                              and w.get("first_past_is_end") is True)
        checks["safe_stop"] = row.get("stop") == "safe_failure"
    elif exposed is False:
        checks["no_row_past_limit"] = w.get("rows_past_limit") == 0  # the log agrees with the summary
    return {"pass": all(checks.values()), "checks": checks,
            "end_function": "exposed" if exposed else ("unexposed" if exposed is False else "unreadable")}


def conclusions(rows: dict, determinism: dict) -> dict:
    """Safety from every run that left a result; the other effects only from an effective valid run whose scene's
    determinism is confirmed (aisle_rack has no repeat, as in A)."""
    def determined(scene):
        return True if scene == "aisle_rack" else determinism.get(scene) is True

    defects, degraded, uo9, unread = [], [], {}, set()
    for name, r in sorted(rows.items()):
        if (r.get("safety") or {}).get("violations"):
            defects.append({"run": name, "arm": r["arm"], "scene": r["scene"], "seed": r["seed"], "class": "dangerous",
                            "priority": 1, **({"invalid": r["invalid"]} if r.get("invalid") else {})})
    for name, r in sorted(rows.items()):
        if not r.get("effective") or r.get("invalid") or r["arm"] == "B0" or r.get("repeat"):
            continue
        if not determined(r["scene"]):
            unread.add((r["scene"], r["arm"]))
            continue
        cell = r.get("cell") or {}
        c = cell.get("class")
        if r["arm"] == "Uo9":
            uo9[name] = r["uo9"]
            if not r["uo9"]["pass"]:
                defects.append({"run": name, "arm": "Uo9", "scene": r["scene"], "seed": r["seed"],
                                "class": "uo9_failed", "priority": 2, "checks": r["uo9"]["checks"]})
            continue
        if c == "dangerous" or cell.get("attribution"):
            continue
        if r["arm"] == "Bb10":
            if c not in ("tolerated", "tolerated_no_operating_comparison"):
                degraded.append({"run": name, "scene": r["scene"], "seed": r["seed"], "class": c})
            if c not in ("observed_end", "safe_failure", "underexposed"):
                continue  # a completed run's degradation is the budget's cost, reported to the lead (plan 3)
            # a mission failure stays a defect at A's priorities as well (post-result fix, Codex results review P2-4:
            # the registration lowers only Bb10's operating degradation)
        prio = {"observed_end": 2, "safe_failure": 3, "underexposed": 3, "operating_degraded": 3.5}.get(c)
        if prio is not None:
            defects.append({"run": name, "arm": r["arm"], "scene": r["scene"], "seed": r["seed"], "class": c,
                            "priority": prio})
    return {"defects": sorted(defects, key=lambda x: (x["priority"], x["run"])),
            "bb10_to_lead": degraded, "uo9": uo9, "unread": sorted(map(list, unread)),
            "determinism_not_confirmed": sorted(sc for sc in ("line_2", "random_1") if determinism.get(sc) is not True)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out_root", type=Path)
    ap.add_argument("--snap", type=Path, required=True)
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--determinism-without-affinity", action="store_true",
                    help="exception 2 (the lead's post-result exception): the repeats compared without result.json "
                         "layer_worker.process_affinity / process_cores while the worker is not in thread mode; "
                         "off: as registered")
    a = ap.parse_args(argv)
    lists = _load("lat_lists", HERE / "lat_lists.py")
    low_mod = _load("low_clearance_report", ROOT / "tools/low_clearance_report.py")
    sys.path.insert(0, str(ROOT))
    from tools import deterministic_compare as dc

    rows_in = [json.loads(x) for x in (a.out_root / "RUNS_budget.jsonl").read_text().splitlines() if x.strip()]
    if {r["name"] for r in rows_in} != {r["name"] for r in lists.budget_rows(str(a.snap), str(a.out_root))}:
        print("list_not_registered")
        return 2
    rerun_file = a.out_root / "RUNS_rerun.jsonl"
    reruns = ({json.loads(x)["name"]: json.loads(x) for x in rerun_file.read_text().splitlines() if x.strip()}
              if rerun_file.exists() else {})
    attempts = LA.bundle_records(a.out_root)
    rows: dict = {}

    def make_row(use: dict, repeat: bool) -> dict:
        d = Path(use["argv"][-1])
        run = d / "run"
        result = LA._json(run / "result.json")
        env = use["env"]
        arm = env["LAT_ARM"]
        problems = LA.environment_problems(attempts.get(use["name"], []), run, "B0", result)
        problems += budget_problems(arm, result, run)
        row = {"name": use["name"], "arm": arm, "scene": env["SCENE"], "seed": int(env["SEED"]), "repeat": repeat,
               "dir": str(d), "invalid": problems}
        if result is not None:
            row["mission"] = LA.mission_of(result)
            row["stop"] = LA.stop_or_end(result)
            # A's layer exposure (moving jobs, on-schedule share) for the arms inside the budget; Uo9 is judged by its
            # own conditions and its exposure is reported only
            row["exposure"] = (LA.exposure_of("U6", result, LA._json(run / "layer_worker.json"))
                               if arm in INJECTED else {"ok": True})
            if arm == "Uo9":
                row["exposure"]["ok"] = True
            if arm != "B0":
                row["watchdog"] = watchdog_of(result, LA._json(run / "layer_latency.json"))
            row["_result"] = result
        return row

    for r in rows_in:
        name = r["name"]
        rows[name] = make_row(r, name.endswith("_rep"))
        rr = reruns.get(f"lat_r_{name}")
        rows[name]["effective"] = rr is None
        if rr is not None:
            rows[rr["name"]] = {**make_row(rr, name.endswith("_rep")), "rerun_of": name, "effective": True}

    def eff(name: str) -> str:
        return f"lat_r_{name}" if f"lat_r_{name}" in rows else name

    def base_of(row):
        return eff(f"lbg_B0_{row['scene']}_s{row['seed']}")

    for name, row in rows.items():
        res = row.pop("_result", None)
        if res is None:
            continue
        bn = base_of(row)
        base_res = LA._json(Path(rows[bn]["dir"]) / "run" / "result.json") if bn in rows else None
        try:
            stopping = LA.stopping_from(a.snap, base_res or res, low_mod)
            low = low_mod.run_report(str(Path(row["dir"]) / "run"), stopping, 0.01)
            sens = (low_mod.run_report(str(Path(row["dir"]) / "run"), low_mod.StoppingModel(
                stopping.latency_s + NOMINAL_TICKS[row["arm"]] / 120.0, stopping.decel_mps2, stopping.margin_m), 0.01)
                if NOMINAL_TICKS[row["arm"]] else None)
        except Exception as exc:  # noqa: BLE001 -- unreadable: the run is the environment's
            low, sens = {"error": repr(exc)[:300]}, None
        row["safety"] = LA.safety_of(res, low)
        row["sensitivity_low_objects"] = None if sens is None else {
            "physical_arc_violations": len(sens.get("physical_arc_violations") or []), "reopen": sens.get("reopen")}
        if row["safety"]["violations"] is None:
            row["invalid"].append("safety record unreadable")
    for name, row in rows.items():
        if "safety" not in row:
            continue
        bn = base_of(row)
        base = rows.get(bn)
        base = base if base is not None and not base["invalid"] and "safety" in base else None
        row["cell"] = LA.classify(row, base if name != bn else None, aisle=row["scene"] == "aisle_rack")
        if row["arm"] == "Uo9":
            row["uo9"] = uo9_verdict(row)
    LA.strip_layer_timing(dc)
    if a.determinism_without_affinity:
        strip_affinity(dc)

    def cmp(x, y):
        x, y = eff(x), eff(y)
        if x not in rows or y not in rows or rows[x]["invalid"] or rows[y]["invalid"]:
            return None
        return bool(dc.compare(Path(rows[x]["dir"]) / "run", Path(rows[y]["dir"]) / "run")["equal"])
    determinism = {}
    for scene in ("line_2", "random_1"):
        b = cmp(f"lbg_B0_{scene}_s0", f"lbg_B0_{scene}_s0_rep")
        u = cmp(f"lbg_Ub6_{scene}_s0", f"lbg_Ub6_{scene}_s0_rep")
        determinism[scene] = None if None in (b, u) else (b and u)
    rerun_needed = sorted(n for n, r in rows.items() if r["invalid"] and r.get("effective") and not r.get("rerun_of"))
    out = {"determinism_without_affinity": bool(a.determinism_without_affinity),
           "rows": rows, "determinism": determinism, "rerun_needed": rerun_needed,
           "to_lead": sorted(n for n, r in rows.items() if r["invalid"] and r.get("rerun_of")),
           "conclusions": conclusions(rows, determinism)}
    if a.json is not None:
        a.json.write_text(json.dumps(out, indent=1, default=str) + "\n")
    print(json.dumps({k: out[k] for k in ("determinism", "rerun_needed", "to_lead")}, indent=1))
    print(json.dumps(out["conclusions"], indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
