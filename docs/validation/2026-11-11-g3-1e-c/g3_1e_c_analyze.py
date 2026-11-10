"""1e (c) core-only direct comparison (docs/plans/2026-11-11-g3-1e-core-only.md, fixed before the result): four
single-session jobs A B B A (A: one physical core for the session, B: two), 1e off in both arms.

    PYTHONPATH=SNAP/src python g3_1e_c_analyze.py --lists OUT/g3cc_1_A.jsonl ... (in job order)
        [--rerun P OUT/g3cc_rP_<n>_<arm>.jsonl OUT/g3cc_rP_<m>_<arm>.jsonl]... [--json out]

Each job's directory is the list's directory / its name (bundle/, the run, <name>_ps.jsonl, <name>_threads.jsonl).

Validity per session, split by kind (Codex review: the mission reading is kept apart from the environment's):
- environment / setup (the pair is run again once): the 1e-0 off reading's checks (g3_1e0_analyze.OffSession: monitor,
  dumps, INSTR, the registered conditions, no --layer-worker, scan switches) and the CPU sampler, whose record must
  exist; the bundle's single slot and its session CPUs -- one whole core for A, two for B (ws1: CPU n and n + 12 are
  one core's siblings, lscpu 2026-11-10; Slurm gives whole sibling pairs: CPU 4 -> [0, 1, 12, 13], checked
  2026-11-11); one attempt, not retried, its status "ended" or a mission status.
- mission: not done, a mission status of the bundle (early mission failure, tour incomplete, no mission report), or
  a safety event (or a safety record missing) -- read whatever the environment says, and never erased by a rerun. A
  probe exit goes with a mission failure when there is one, else with the environment.
- a rerun pair (--rerun) replaces that pair's sessions, allowed only for a pair whose first run had an environment
  invalidity; the first run's rows stay in the output; a rerun pair invalid again goes to the lead.
- the launcher's RETURN_START / NEAR_END_STAND (<run>/live/env.sh) the same in all sessions in use; otherwise the set
  goes to the lead.
Judgement: per pair, saving = W(A) - W(B) (W: AUTO rows' work over their sim time, the 1e-0 population rule);
"(c) passes" when both pairs save >= 0.03707 s/sim s, else "(c) fails". Reported: each session's AUTO rate, W,
evidence_stale ticks, mission, GPU index, start and end, the thread sampler's per-name CPU and run-queue delay.
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
MISSION_STATUSES = ("early_mission_failure", "tour_incomplete", "no_mission_report")


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
    """The bundle's record of the session: one attempt, not retried, ended or ended by a mission status."""
    if len(records) != 1 or records[0].get("attempt") != 1 or records[0].get("retried") \
            or records[0].get("status") not in ("ended", *MISSION_STATUSES):
        return f"bundle: attempts {[(r.get('attempt'), r.get('status')) for r in records]}"
    return None


def mission_status(records: list) -> str | None:
    s = records[0].get("status") if len(records) == 1 else None
    return f"bundle mission status {s}" if s in MISSION_STATUSES else None


def launcher_values(run: Path) -> dict:
    """RETURN_START / NEAR_END_STAND as the launcher wrote them: r1c_session runs it in <run>/live (env.sh there)."""
    out = {}
    f = run / "live" / "env.sh"
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


def split_kinds(invalid: list, row: dict, records: list) -> dict:
    """environment / mission, the mission reading independent of the environment's (Codex review P2-2)."""
    env, mission, probe = [], [], []
    for x in invalid:
        (probe if x.startswith("probe exit") else env).append(x)
    if not row.get("mission_done"):
        mission.append(f"mission not done: {row.get('mission_outcomes')}")
    ms = mission_status(records)
    if ms:
        mission.append(ms)
    safety = row.get("safety")
    keys = ("forbidden_contacts", "unpermitted_entries", "unpermitted_dir_entries")
    if not env:  # the safety counters exist only when the session was read in full
        if safety is None or any(safety.get(k) is None for k in keys):
            mission.append(f"safety record missing: {safety}")
        elif any(safety.get(k) for k in keys):
            mission.append(f"safety {safety}")
    (mission if mission else env).extend(probe)
    return {"environment": env, "worker": [], "mission": mission}


def session_w(s) -> float | None:
    ok, w, sim, _ = s.auto_population()
    return float(np.nansum(w[ok])) / sim if sim > 0 else None


def judge(pairs: dict) -> dict:
    """pairs: {p: (W of A, W of B)}."""
    per = {p: {"W_A": a, "W_B": b, "saving": a - b, "passes": (a - b) >= TARGET} for p, (a, b) in sorted(pairs.items())}
    return {"pairs": per, "decision": "(c) passes" if all(v["passes"] for v in per.values()) else "(c) fails",
            "target_s_per_sim_s": TARGET}


def read_session(f: Path, e: dict) -> tuple[dict, dict]:
    """One job: (its row, its launcher values)."""
    jobdir = f.parent / e["name"]
    run = jobdir / e["name"]
    ps_f = f.parent / f"{e['name']}_ps.jsonl"
    ps = g2m.read_ps(ps_f.read_text()) if ps_f.exists() else None
    summary_f = jobdir / "bundle" / "summary.json"
    summary = json.loads(summary_f.read_text()) if summary_f.exists() else {}
    res_f = jobdir / "bundle" / "results.jsonl"
    recs = [json.loads(x) for x in res_f.read_text().splitlines() if x.strip()] if res_f.exists() else []
    recs = [r for r in recs if r.get("name") == e["name"]]
    s = e0.OffSession(run, e, ps=ps)
    arm = e["env"]["ARM_NAME"]
    s.invalid.extend(p for p in (allocation_problem(summary, arm), attempt_problem(recs),
                                 None if ps is not None else "no CPU sampler record") if p)
    row = base.analyse_session(s) if not s.invalid else {
        "name": s.name, "invalid": s.invalid, "ps": s.ps, "mission_outcomes": s.mission_outcomes,
        "mission_done": s.mission_done}
    row["arm"], row["pair"] = arm, int(e["env"]["PAIR"])
    row["kinds"] = split_kinds(s.invalid, row, recs)
    row["evidence_stale_ticks"] = e0.stale_ticks(s.result)
    row["session_cpus"], row["node_gpu"] = summary.get("session_cpus"), summary.get("node_gpu")
    row["start_end"] = [recs[0].get("start"), recs[0].get("end")] if recs else None
    th_f = f.parent / f"{e['name']}_threads.jsonl"
    th = [json.loads(x) for x in th_f.read_text().splitlines() if x.strip()] if th_f.exists() else []
    walls = s._walls() if hasattr(s, "_walls") else []
    tck = int(th[0].get("tck", 100)) if th and "sampler" in th[0] else 100
    row["threads"] = core.threads_report([x for x in th if "threads" in x],
                                         (min(walls), max(walls)) if walls else None, tck)
    row["W"] = session_w(s) if not any(row["kinds"].values()) else None
    return row, launcher_values(run)


def decide(rows: dict, launch: list, reruns=()) -> dict:
    """The registered decision over the sessions in use (rows: name -> row; each pair's latest)."""
    failed = sorted({r["pair"] for r in rows.values() if r["kinds"]["mission"]})
    env = sorted({r["pair"] for r in rows.values() if r["kinds"]["environment"]})
    if failed:
        return {"decision": "to the lead", "mission_failed_pairs": failed, "environment_pairs": env}
    again = sorted(set(env) & set(reruns))
    if again:
        return {"decision": "to the lead", "why": f"pair(s) {again} invalid again after their one rerun",
                "environment_pairs": env}
    if env:
        return {"rerun_pairs": env, "why": "an environment or setup invalidity (once per pair)"}
    lp = launcher_problem(launch)
    if lp:
        return {"decision": "to the lead", "why": lp}
    pairs: dict = {}
    for r in rows.values():
        pairs.setdefault(r["pair"], {})[r["arm"]] = r["W"]
    return judge({p: (v["A"], v["B"]) for p, v in pairs.items()})


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--lists", type=Path, nargs="+", required=True)
    ap.add_argument("--rerun", nargs=3, action="append", default=[], metavar=("PAIR", "LIST_1", "LIST_2"))
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args()
    entries = [json.loads(f.read_text().splitlines()[0]) for f in a.lists]
    order = [(e["env"]["ARM_NAME"], int(e["env"]["PAIR"])) for e in entries]
    if order != ORDER or len({e["name"] for e in entries}) != 4 or any("LAYER_WORKER" in e["env"] for e in entries):
        print(json.dumps({"list_not_registered": order}))
        return 2
    reruns: dict = {}
    for p, l1, l2 in a.rerun:
        p = int(p)
        es = [(Path(x), json.loads(Path(x).read_text().splitlines()[0])) for x in (l1, l2)]
        want = [x for x in ORDER if x[1] == p]
        if p in reruns or [(e["env"]["ARM_NAME"], int(e["env"]["PAIR"])) for _, e in es] != want \
                or any(not e["name"].startswith(f"g3cc_r{p}_") for _, e in es):
            print(json.dumps({"rerun_not_registered": [p, l1, l2]}))
            return 2
        reruns[p] = es
    first, current, launch = {}, {}, []
    for f, e in zip(a.lists, entries, strict=True):
        row, lv = read_session(f, e)
        first[e["name"]] = row
        if int(e["env"]["PAIR"]) not in reruns:
            current[e["name"]] = row
            launch.append(lv)
    first_env = {r["pair"] for r in first.values() if r["kinds"]["environment"]}
    first_mission = {r["pair"] for r in first.values() if r["kinds"]["mission"]}
    bad = sorted(p for p in reruns if p not in first_env or p in first_mission)
    if bad:
        print(json.dumps({"rerun_not_allowed": bad, "why": "a rerun replaces only an environment-invalid pair "
                                                           "without a mission failure"}))
        return 2
    for _p, es in sorted(reruns.items()):
        for f, e in es:
            row, lv = read_session(f, e)
            current[e["name"]] = row
            launch.append(lv)
    out = {"first_run": first, "sessions": current, "launcher": launch,
           "result": decide(current, launch, reruns=tuple(reruns))}
    text = json.dumps(out, indent=1, default=str)
    if a.json:
        a.json.write_text(text)
    print(json.dumps(out["result"], indent=1, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
