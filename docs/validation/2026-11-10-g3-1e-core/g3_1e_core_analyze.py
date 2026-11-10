"""1e stage 1 (docs/plans/2026-11-10-g3-1e-process-worker.md 3, fixed before the result): the reading of D T T D --
``delay`` (D) and ``thread`` pinned to the session's second core (T), both on two physical cores. Each session is read
with the 1e-0 analyzer's session reading and measures (g3_1e0_analyze: validity, AUTO rows, main_layer, main_other,
barrier, paired waits); the registered formula is unchanged.

    PYTHONPATH=SNAP/src python g3_1e_core_analyze.py OUT_DIR --list OUT_DIR.jsonl [--ps PS.jsonl] [--threads TH.jsonl]
        [--json out]

Validity (per session), split by kind (plan 3; Codex 2nd review P2):
- environment / setup: the 1e-0 checks (monitor, the CPU sampler, dumps, INSTR, registered conditions), plus the
  allocation: result.json layer_worker.process_affinity holds four CPUs and equals the bundle's session_cpus; T's worker
  is pinned (cpus of two CPUs inside the process affinity, its own affinity equal to them), D's is not; the arm's
  EXTRA_ARGS. A pair with such a session is to be run again once (the analyzer says which).
- worker: the layer worker failed or timed out.
- mission: the mission not done, or a safety event (forbidden contacts, unpermitted entries).
A worker or mission failure of T is not erased by a rerun: without an environment invalidity it counts as T failing.

Judgement:
- the waits are read only for k at or below the executed k (the 1e-0 analyzer's k above it is structurally 0);
- a k = 3 run: k = the smallest k <= 3 whose mean scheduled wait is at most 0.5 ms in both pairs; none: the
  supplementary k = 4 pair is due; the supplementary k = 4 run is judged at k = 4 (its measured wait in the net);
- per pair: net_T(k) = moved - mean wait(k) x scans per AUTO sim s - barrier wait[T] - max(0, main_other[T] -
  main_other[D]); direct = W[D] - W[T];
- (a) "thread plus a core" when both pairs have net_T >= 0.03707 and direct >= net_T / 2; else "stage 2 to the lead".
A chosen k = 2 is a candidate prediction (the session ran at k = 3); the chosen k is confirmed in 1e-2.

Reported only: each session's evidence_stale ticks and mission outcome; the worker's per-job scheduler counters (run
time, run-queue delay, CPU) as CPU / wall and delay / wall; the thread sampler's per-thread CPU and run-queue delay
inside each session's monitor window (threads seen in at least two samples; missed threads unknown); k = 4's corrected
wait estimate (the install wait and time at scan + 3 taken off scan + 4's time); the core alone: the k = 3 D's realised
AUTO rate and W beside 17604's delay (one core, k = 3: rate 0.941, W 0.8739).
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve()
_spec = importlib.util.spec_from_file_location("g3_1e0_analyze", HERE.parents[1] / "2026-11-10-g3-1e0" / "g3_1e0_analyze.py")
e0 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(e0)
base, g2m = e0.base, e0.g2m
TARGET = e0.TARGET
WAIT_OK_S = e0.WAIT_OK_S
REF_17604_DELAY = {"rate": 0.9406279402002807, "W": 0.8738943428706015}
ENV_MARKS = ("other users'", "monitor", "GPU process", "another job on the node", "probe exit", "live log", "tick trace",
             "long calls", "no result", "allocation", "EXTRA_ARGS", "registered", "set (dumps", "INSTR", "scan_switches",
             "bundle")


class CoreSession(e0.Session):
    def __init__(self, out: Path, entry: dict, ps=None, session_cpus=None, attempts=None):
        env = entry["env"]
        self.arm = env["ARM_NAME"]
        self.mode = env["LAYER_WORKER"]
        self.k = int(env["LAYER_WORKER_K"])
        self.pair = int(env["PAIR"])
        g2m.Session.__init__(self, out, entry, ps=ps)
        res = self.result or {}
        lw = res.get("layer_worker") or {}
        if lw.get("mode") != self.mode or lw.get("k") != self.k:
            self.invalid.append(f"registered: result.json layer_worker {lw.get('mode')}/{lw.get('k')}, "
                                f"registered {self.mode}/{self.k}")
        self.worker_failed = lw.get("failed")
        sw = res.get("scan_switches") or {}
        if not (sw.get("grid_clear_kernel") and sw.get("shadow_kernel") and sw.get("lidar_cast") == "fast"):
            self.invalid.append(f"scan_switches {sw}")
        aff = lw.get("process_affinity")
        cores = lw.get("process_cores")  # [(siblings inside the affinity, all siblings)] (the runner's record)
        if session_cpus is None:
            self.invalid.append("allocation: no bundle summary (session_cpus)")
        elif not aff or sorted(session_cpus) != sorted(aff):
            self.invalid.append(f"allocation: process affinity {aff}, bundle session_cpus {session_cpus}")
        if not cores or len(cores) != 2 or any(sorted(c[0]) != sorted(c[1]) for c in cores):
            self.invalid.append(f"allocation: process cores {cores} (two whole cores registered)")
        recs = attempts or []
        if len(recs) != 1 or recs[0].get("attempt") != 1 or recs[0].get("retried") or recs[0].get("status") != "ended":
            self.invalid.append(f"bundle: attempts {[(r.get('attempt'), r.get('status')) for r in recs]}")
        cpus, waff = lw.get("cpus"), lw.get("affinity")
        if self.arm == "T":
            last = sorted(cores[-1][0]) if cores else None
            if not cpus or sorted(cpus) != last or waff != sorted(cpus):
                self.invalid.append(f"allocation: T worker cpus {cpus}, its affinity {waff}")
        elif cpus is not None:
            self.invalid.append(f"allocation: D worker pinned to {cpus}")
        f = self.run / "layer_worker.json"
        self.installs = json.loads(f.read_text()) if f.exists() else None
        if self.installs is None:
            self.invalid.append("registered: no layer_worker.json")

    def _check_conditions(self):
        c = self.cond
        for key in c:
            if key.startswith(g2m.FORBIDDEN):
                self.invalid.append(f"{key} set (dumps and the dual run stay out of the check)")
        want = {"SET": "far", "ARM": "K", "FACTOR": "1.0", "FORKLIFT_LIVE_CATCHUP_STEPS": "6", "SEED": "0",
                "FORKLIFT_GRID_CLEAR_KERNEL": "1", "FORKLIFT_SHADOW_KERNEL": "1"}
        for key, v in want.items():
            if c.get(key) != v:
                self.invalid.append(f"registered: {key}={c.get(key)}, registered {v}")
        extra = c.get("EXTRA_ARGS", "").replace("\\ ", " ")
        need = ["--lidar-cast fast", f"--layer-worker {self.mode} --layer-worker-k {self.k}"]
        if self.arm == "T":
            need.append("--layer-worker-cpus auto")
        for n in need:
            if n not in extra:
                self.invalid.append(f"EXTRA_ARGS without {n}")
        if self.arm == "D" and "--layer-worker-cpus" in extra:
            self.invalid.append("EXTRA_ARGS with --layer-worker-cpus in D")
        for key in g2m.INSTR_ON:
            if c.get(key) is not None:
                self.invalid.append(f"{key} set in an INSTR off session")


def kinds(s: CoreSession, row: dict) -> dict:
    """The session's invalidity split: environment / setup (rerun the pair), worker, mission. A worker failure ends
    the session, so the probe's exit then belongs to the worker, not to the environment (Codex review P2-1)."""
    env = [x for x in s.invalid if any(m in x for m in ENV_MARKS)
           and not (s.worker_failed and x.startswith("probe exit"))]
    worker = [f"layer worker failed: {s.worker_failed}"] if s.worker_failed else []
    worker += [x for x in s.invalid if x not in env and ("layer worker" in x or x.startswith("probe exit"))]
    other = [x for x in s.invalid if x not in env and x not in worker]
    mission = list(other)
    if not row.get("mission_done"):
        mission.append(f"mission not done: {row.get('mission_outcomes')}")
    safety = row.get("safety")
    keys = ("forbidden_contacts", "unpermitted_entries", "unpermitted_dir_entries")
    if not env and not worker:
        if safety is None or any(safety.get(k) is None for k in keys):
            mission.append(f"safety record missing: {safety}")  # unknown is not zero (Codex review P2-6)
        elif any(safety.get(k) for k in keys):
            mission.append(f"safety {safety}")
    return {"environment": env, "worker": worker, "mission": mission}


def scheduler_report(installs: list) -> dict:
    rows = [r for r in installs if r.get("run_ns") is not None and r.get("compute_s")]
    if not rows:
        return {"jobs": 0}
    wall = sum(float(r["compute_s"]) for r in rows)
    return {"jobs": len(rows), "cpu_over_wall": sum(r["run_ns"] for r in rows) / 1e9 / wall,
            "delay_over_wall": sum(r["run_delay_ns"] for r in rows) / 1e9 / wall,
            "cpus": sorted({r["cpu"] for r in rows if r.get("cpu") is not None})}


def k4_corrected_ms(installs: list) -> dict | None:
    """Report only: k = 4's wait with the install wait and time at scan + 3 taken off scan + 4's start."""
    out = []
    for r in installs:
        tt = {int(a): float(b) for a, b in (r.get("tick_t") or {}).items()}
        if 4 not in tt or r.get("install_tick") != r.get("scan_tick", -9) + 3 or r.get("reason") != "scheduled":
            continue
        s4 = tt[4] - float(r["wait_s"]) - float(r["install_s"]) - float(r["submit_t"])
        out.append(1e3 * max(0.0, (float(r["done_t"]) - float(r["submit_t"])) - s4))
    return e0._q(out)


def threads_report(samples: list, walls: tuple | None, tck: int) -> dict | None:
    """Per (process, thread) name: CPU s and run-queue delay s over consecutive samples inside the window."""
    if not walls or not samples:
        return None
    a, b = walls
    inside = [s for s in samples if a <= s["wall"] <= b]
    agg: dict = {}
    for s0, s1 in zip(inside, inside[1:], strict=False):
        for key, v1 in s1["threads"].items():
            v0 = s0["threads"].get(key)
            if v0 is None:
                continue
            name = f"{v1[0]}/{v1[1]}"
            g = agg.setdefault(name, {"cpu_s": 0.0, "delay_s": 0.0, "threads": set(), "affinity": set()})
            g["cpu_s"] += (v1[2] - v0[2]) / tck
            g["delay_s"] += (v1[4] - v0[4]) / 1e9
            g["threads"].add(key)
            if len(v1) > 6 and v1[6] is not None:
                g["affinity"].add(v1[6])
    span = inside[-1]["wall"] - inside[0]["wall"] if len(inside) >= 2 else 0.0
    return {"window_s": span, "samples": len(inside), "missed_threads": "unknown",
            "by_name": {n: {"cpu_s": g["cpu_s"], "delay_s": g["delay_s"], "threads": len(g["threads"]),
                            "affinity": sorted(g["affinity"])}
                        for n, g in sorted(agg.items(), key=lambda kv: -kv[1]["cpu_s"])}}


def choose_k(t_measures: list, k_exec: int):
    """A k = 3 run: the smallest k <= 3 whose mean scheduled wait is at most 0.5 ms in every pair, else None (the
    supplementary pair is due). The supplementary k = 4 run is judged at k = 4 (plan 3; Codex review P2-2)."""
    if k_exec == 4:
        return 4
    for k in range(2, k_exec + 1):
        means = [(m["paired_wait_ms"].get(k) or {}).get("mean") for m in t_measures]
        if all(x is not None and x / 1e3 <= WAIT_OK_S for x in means):
            return k
    return None


def pair_net(d: dict, t: dict, k: int) -> dict:
    q = t["paired_wait_ms"][k]
    increase = max(0.0, t["main_other"] - d["main_other"])
    moved = d["main_layer"] - t["main_layer"]
    net = moved - (q["mean"] / 1e3) * t["scans_per_sim_s"] - t["barrier"]["wait"] - increase
    direct = d["W"] - t["W"]
    return {"k": k, "moved": moved, "main_other_increase": increase, "barrier_wait": t["barrier"]["wait"],
            "net": net, "direct": direct, "residual_direct_minus_net": direct - net,
            "passes": net >= TARGET and direct >= net / 2}


def judge(pairs: dict, k_exec: int) -> dict:
    """pairs: {p: {"D": measures, "T": measures}} for the valid pairs (both registered pairs present)."""
    t_list = [pairs[p]["T"] for p in sorted(pairs)]
    k = choose_k(t_list, k_exec)
    if k is None:
        return {"k": None, "decision": "supplementary k = 4 pair due",
                "why": "no k <= 3 with a mean scheduled wait <= 0.5 ms in both pairs"}
    per = {p: pair_net(pairs[p]["D"], pairs[p]["T"], k) for p in sorted(pairs)}
    ok = all(v["passes"] for v in per.values())
    return {"k": k, "k_is_prediction": k < k_exec, "pairs": per,
            "decision": "(a) thread plus a core" if ok else "stage 2 to the lead", "target_s_per_sim_s": TARGET}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=Path)
    ap.add_argument("--list", type=Path, required=True)
    ap.add_argument("--ps", type=Path, default=None)
    ap.add_argument("--threads", type=Path, default=None)
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args()
    entries = [json.loads(x) for x in a.list.read_text().splitlines() if x.strip()]
    order = [(e["env"]["ARM_NAME"], int(e["env"]["PAIR"])) for e in entries]
    ks = {int(e["env"]["LAYER_WORKER_K"]) for e in entries}
    names = [e["name"] for e in entries]
    if order != [("D", 1), ("T", 1), ("T", 2), ("D", 2)] or len(ks) != 1 or not ks <= {3, 4} \
            or len(set(names)) != len(names):
        print(json.dumps({"list_not_registered": order, "k": sorted(ks), "names": names}))
        return 2
    k_exec = ks.pop()
    ps = g2m.read_ps(a.ps.read_text()) if a.ps else None
    summary_f = a.out / "bundle" / "summary.json"
    summary = json.loads(summary_f.read_text()) if summary_f.exists() else {}
    session_cpus = summary.get("session_cpus") if summary.get("slots") == 1 else None  # single slot registered
    attempts: dict = {}
    res_f = a.out / "bundle" / "results.jsonl"
    for line in (res_f.read_text().splitlines() if res_f.exists() else []):
        if line.strip():
            r = json.loads(line)
            attempts.setdefault(r.get("name"), []).append(r)
    th, tck = [], 100
    if a.threads and a.threads.exists():
        lines = [json.loads(x) for x in a.threads.read_text().splitlines() if x.strip()]
        tck = int(lines[0].get("tck", 100)) if lines and "sampler" in lines[0] else 100
        th = [x for x in lines if "threads" in x]
    rows, sessions = {}, {}
    for e in entries:
        s = CoreSession(a.out / e["name"], e, ps=ps, session_cpus=session_cpus, attempts=attempts.get(e["name"]))
        row = base.analyse_session(s) if not s.invalid else {
            "name": s.name, "arm": s.arm, "pair": s.pair, "invalid": s.invalid, "ps": s.ps,
            "mission_outcomes": s.mission_outcomes, "mission_done": s.mission_done}
        row["kinds"] = kinds(s, row)
        row["evidence_stale_ticks"] = e0.stale_ticks(s.result)
        row["scheduler"] = scheduler_report(s.installs or [])
        row["k4_corrected_wait_ms"] = k4_corrected_ms(s.installs or []) if k_exec == 3 and s.arm == "T" else None
        walls = s._walls() if hasattr(s, "_walls") else []
        row["threads"] = threads_report(th, (min(walls), max(walls)) if walls else None, tck)
        rows[e["name"]], sessions[e["name"]] = row, s
    out = {"sessions": rows, "k_executed": k_exec, "summary_slots": summary.get("slots")}
    env_bad = {r["pair"] for r in rows.values() if r["kinds"]["environment"]}
    # a T failure in a pair without an environment invalidity of its own T decides, whatever the other pair needs
    t_failed = sorted({r["pair"] for r in rows.values() if r["arm"] == "T" and not r["kinds"]["environment"]
                       and (r["kinds"]["worker"] or r["kinds"]["mission"])})
    d_failed = sorted({r["pair"] for r in rows.values() if r["arm"] == "D" and not r["kinds"]["environment"]
                       and (r["kinds"]["worker"] or r["kinds"]["mission"])})
    rerun = sorted(env_bad)
    if t_failed:
        out["result"] = {"decision": "stage 2 to the lead", "t_failed_pairs": t_failed, "rerun_pairs": rerun,
                         "why": "a worker or mission failure of T, not erased by a rerun"}
    elif rerun:
        out["result"] = {"rerun_pairs": rerun, "why": "an environment or setup invalidity (plan 3: once)"}
    elif d_failed:
        out["result"] = {"decision": "to the lead", "d_failed_pairs": d_failed, "why": "the reference D failed"}
    else:
        m = {name: e0.measure(s) for name, s in sessions.items()}
        out["measures"] = m
        incomplete = {n: {"scans": x["scans"], "scans_without_record": x["scans_without_record"]}
                      for n, x in m.items() if x["scans"] == 0 or x["scans_without_record"]}
        if incomplete:
            out["result"] = {"incomplete": incomplete}
        else:
            pairs: dict = {}
            for name, r in rows.items():
                pairs.setdefault(r["pair"], {})[r["arm"]] = m[name]
            out["result"] = judge(pairs, k_exec)
    d_rows = [r for r in rows.values() if r["arm"] == "D" and not any(r["kinds"].values())]
    core_alone = []
    for r in d_rows:  # each valid D on its own, whatever the judgement path (Codex review P3-10)
        try:
            w = e0.measure(sessions[r["name"]])["W"]
        except Exception as exc:  # noqa: BLE001 -- report only
            w = f"unreadable: {type(exc).__name__}"
        core_alone.append({"name": r["name"], "rate": (r.get("auto") or {}).get("rate"), "W": w})
    out["core_alone"] = None if k_exec != 3 else {"reference_17604_delay": REF_17604_DELAY, "D": core_alone}
    text = json.dumps(out, indent=1, default=str)
    if a.json:
        a.json.write_text(text)
    print(json.dumps(out["result"], indent=1, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
