"""1e (c) core-only direct comparison (docs/plans/2026-11-11-g3-1e-core-only.md, fixed before the result): four
single-session jobs A B B A (A: one physical core for the session, B: two), 1e off in both arms.

    PYTHONPATH=SNAP/src python g3_1e_c_analyze.py --lists OUT/g3cc_1_A.jsonl ... (in job order) [--json out]

Each job's directory is the list's directory / its name (bundle/, the run, <name>_ps.jsonl, <name>_threads.jsonl).

Validity per session, split by kind:
- environment / setup (the pair is run again once): the 1e-0 off reading's checks (g3_1e0_analyze.OffSession: monitor,
  the CPU sampler, dumps, INSTR, the registered conditions, no --layer-worker, scan switches); the bundle's single
  slot and its session CPUs -- one whole core for A, two for B (ws1: CPU n and n + 12 are one core's siblings,
  lscpu 2026-11-10); one attempt, ended, not retried.
- mission: not done, or a safety event (or a safety record missing).
- the launcher's RETURN_START / NEAR_END_STAND (env.sh) the same in all four sessions; otherwise the set is to the
  lead.
Judgement: per pair, saving = W(A) - W(B) (W: AUTO rows' work over their sim time, the 1e-0 population rule);
"(c) passes" when both pairs save >= 0.03707 s/sim s, else "(c) fails". Reported: each session's AUTO rate, W,
evidence_stale ticks, mission, the thread sampler's per-name CPU and run-queue delay.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve()
_spec = importlib.util.spec_from_file_location("g3_1e0_analyze", HERE.parents[1] / "2026-11-10-g3-1e0" / "g3_1e0_analyze.py")
e0 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(e0)
_spec2 = importlib.util.spec_from_file_location("g3_1e_core_analyze",
                                                HERE.parents[1] / "2026-11-10-g3-1e-core" / "g3_1e_core_analyze.py")
core = importlib.util.module_from_spec(_spec2)
_spec2.loader.exec_module(core)
base, g2m = e0.base, e0.g2m
TARGET = e0.TARGET
SIBLING_OFFSET = 12  # ws1: lscpu -e, CPU n and n + 12 on one core (2026-11-10)
ORDER = [("A", 1), ("B", 1), ("B", 2), ("A", 2)]
CORES = {"A": 1, "B": 2}


def whole_cores(cpus, offset: int = SIBLING_OFFSET):
    """How many whole cores the CPUs make (each CPU with its sibling), or None when a core is incomplete."""
    s = set(int(c) for c in cpus)
    if any(((c + offset) not in s and (c - offset) not in s) for c in s):
        return None
    return len(s) // 2


def allocation_problem(summary: dict, arm: str) -> str | None:
    if summary.get("slots") != 1:
        return f"allocation: bundle slots {summary.get('slots')}, registered 1"
    cpus = summary.get("session_cpus")
    if not cpus or whole_cores(cpus) != CORES[arm]:
        return f"allocation: session_cpus {cpus}, registered {CORES[arm]} whole core(s)"
    return None


def attempt_problem(records: list) -> str | None:
    if len(records) != 1 or records[0].get("attempt") != 1 or records[0].get("retried") \
            or records[0].get("status") != "ended":
        return f"bundle: attempts {[(r.get('attempt'), r.get('status')) for r in records]}"
    return None


def launcher_values(run: Path) -> dict:
    out = {}
    f = run / "env.sh"
    for line in (f.read_text().splitlines() if f.exists() else []):
        key, _, value = line.partition("=")
        if key in ("RETURN_START", "NEAR_END_STAND"):
            out[key] = value.strip("'\"")
    return out


def launcher_problem(values: list) -> str | None:
    keys = ("RETURN_START", "NEAR_END_STAND")
    if any(any(k not in v for k in keys) for v in values):
        return f"launcher values missing: {values}"
    if len({tuple(v[k] for k in keys) for v in values}) != 1:
        return f"launcher values differ between sessions: {values}"
    return None


def session_w(s) -> float | None:
    ok, w, sim, _ = s.auto_population()
    return float(np.nansum(w[ok])) / sim if sim > 0 else None


def judge(pairs: dict) -> dict:
    """pairs: {p: (W of A, W of B)}."""
    per = {p: {"W_A": a, "W_B": b, "saving": a - b, "passes": (a - b) >= TARGET} for p, (a, b) in sorted(pairs.items())}
    return {"pairs": per, "decision": "(c) passes" if all(v["passes"] for v in per.values()) else "(c) fails",
            "target_s_per_sim_s": TARGET}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--lists", type=Path, nargs="+", required=True)
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args()
    entries = [json.loads(f.read_text().splitlines()[0]) for f in a.lists]
    order = [(e["env"]["ARM_NAME"], int(e["env"]["PAIR"])) for e in entries]
    if order != ORDER or len({e["name"] for e in entries}) != 4 or any("LAYER_WORKER" in e["env"] for e in entries):
        print(json.dumps({"list_not_registered": order}))
        return 2
    rows, ws, launch = {}, {}, []
    for f, e in zip(a.lists, entries, strict=True):
        jobdir = f.parent / e["name"]
        run = jobdir / e["name"]
        ps_f = f.parent / f"{e['name']}_ps.jsonl"
        ps = g2m.read_ps(ps_f.read_text()) if ps_f.exists() else None
        summary_f = jobdir / "bundle" / "summary.json"
        summary = json.loads(summary_f.read_text()) if summary_f.exists() else {}
        res_f = jobdir / "bundle" / "results.jsonl"
        recs = [json.loads(x) for x in res_f.read_text().splitlines() if x.strip()] if res_f.exists() else []
        s = e0.OffSession(run, e, ps=ps)
        arm = e["env"]["ARM_NAME"]
        for prob in (allocation_problem(summary, arm), attempt_problem([r for r in recs if r.get("name") == e["name"]])):
            if prob:
                s.invalid.append(prob)
        row = base.analyse_session(s) if not s.invalid else {
            "name": s.name, "arm": arm, "pair": s.pair, "invalid": s.invalid, "ps": s.ps,
            "mission_outcomes": s.mission_outcomes, "mission_done": s.mission_done}
        row["arm"], row["pair"] = arm, int(e["env"]["PAIR"])
        s.worker_failed = None
        row["kinds"] = core.kinds(s, row)
        row["evidence_stale_ticks"] = e0.stale_ticks(s.result)
        row["session_cpus"] = summary.get("session_cpus")
        th_f = f.parent / f"{e['name']}_threads.jsonl"
        th = [json.loads(x) for x in th_f.read_text().splitlines() if x.strip()] if th_f.exists() else []
        walls = s._walls() if hasattr(s, "_walls") else []
        tck = int(th[0].get("tck", 100)) if th and "sampler" in th[0] else 100
        row["threads"] = core.threads_report([x for x in th if "threads" in x],
                                             (min(walls), max(walls)) if walls else None, tck)
        if not any(row["kinds"].values()):
            ws[e["name"]] = session_w(s)
        row["W"] = ws.get(e["name"])
        rows[e["name"]] = row
        launch.append(launcher_values(run))
    out = {"sessions": rows, "launcher": launch}
    rerun = sorted({r["pair"] for r in rows.values() if r["kinds"]["environment"]})
    failed = sorted({r["pair"] for r in rows.values() if not r["kinds"]["environment"] and r["kinds"]["mission"]})
    lp = launcher_problem(launch)
    if failed:
        out["result"] = {"decision": "to the lead", "mission_failed_pairs": failed, "rerun_pairs": rerun}
    elif rerun:
        out["result"] = {"rerun_pairs": rerun, "why": "an environment or setup invalidity (once per pair)"}
    elif lp:
        out["result"] = {"decision": "to the lead", "why": lp}
    else:
        pairs = {}
        for r in rows.values():
            pairs.setdefault(r["pair"], {})[r["arm"]] = r["W"]
        out["result"] = judge({p: (v["A"], v["B"]) for p, v in pairs.items()})
    text = json.dumps(out, indent=1, default=str)
    if a.json:
        a.json.write_text(text)
    print(json.dumps(out["result"], indent=1, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
