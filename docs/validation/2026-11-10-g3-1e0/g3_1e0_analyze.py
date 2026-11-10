"""Real-time plan 5.28 1e-0: the prototype measurement's reading, fixed before its result. Two sessions of
g3_1e0_lists.py (delay, thread), each read with g3_1b_loop_analyze's session reading (validity, AUTO rows and windows,
missions, safety) and its own records: result.json ``layer_worker`` and ``scan_switches``, run/layer_worker.json (one
record per install: scan_tick, due_tick, install_tick, reason, submit_t, done_t, compute_s, compute_cpu_s, wait_s,
install_s, tick_t {1..4: the control start of scan_tick + n}), run/obstacle_scans.json or the spool's scans (each
scan's cast_wall_s and total_wall_s, the main thread's part), the tick trace and the status-line timers.

    PYTHONPATH=SNAP/src python g3_1e0_analyze.py OUT_DIR --list OUT_DIR.jsonl [--ps PS.jsonl] [--json out]

Population: the AUTO rows -- rows i whose control tick and the next row's are both AUTO (W_i holds row i's step and row
i+1's control), with a finite W_i; their sim time is (rows) / 120 s (plan 5.28, Codex P3-8: NaN rows leave numerator and
denominator alike). A scan belongs to the population when its scan tick's row does.

Per session (s per AUTO sim s unless named):
- W: the sum of W_i over the AUTO rows / their sim time;
- main_layer: the main thread's layer refresh work per scan -- (total_wall_s - cast_wall_s) + install_s -- summed over
  the AUTO scans / the AUTO sim time (delay: the computation is inside total_wall_s; thread: only prepare and submit);
- compute: the computation's own wall (compute_s) per AUTO sim s, and its CPU / wall ratio (GIL waiting shows as < 1);
- barrier: the installs that were not scheduled (by reason), their count per AUTO sim s and their wait per AUTO sim s;
- main_other: the status-line timers' isaac.step plus every core.* region per AUTO sim s (the AUTO windows of
  g2m_analyze's auto_windows), less the barrier installs' wait and install time (a barrier inside a core region is
  timed by it; Codex 1e-0 review P2) -- the main thread's other work; the scheduled installs run at the control tick's
  start, outside every core region;
- reported, not used: the SLAM replies' apply lag (applied_tick_s - stamp_s of the async link's records, sim s) and the
  far recognition's job wall times (the missions' recognition_mission.live.jobs).

Thread session, per AUTO scan with tick_t[k]: T_i = done_t - submit_t, S_i(k) = tick_t[k] - submit_t, the scheduled wait
max(T_i - S_i(k), 0); its mean and p99 (ms a scan) for k = 2, 3, 4; scans per AUTO sim s.

Prediction (5.28 1e-0 registration):
- chosen k = the smallest of 2, 3, 4 whose mean scheduled wait is at most 0.5 ms a scan; none: k = 4;
- net(k) = (main_layer[delay] - main_layer[thread]) - mean wait(k) x scans per AUTO sim s - barrier wait[thread]
  - max(0, main_other[thread] - main_other[delay]);
  (the moved work is measured as the main thread's difference, so the thread's submit and install cost is already in
  it; the waits and the barrier are the thread's; main_other excludes them -- no double counting, Codex round 2 P3);
- the direct estimate W[delay] - W[thread] beside it, and the residual direct - net(chosen k);
- decision: "go to 1e-1" when net(chosen k) >= 0.03707 s/sim s, else "process design to the lead".
The session ran at k = 3: the waits for 2 and 4 are read off its records; the main thread's pattern at another k is not
measured (stated with the result). Neither session is a judgement (1e-2 judges).

Completeness (Codex 1e-0 review P2): every AUTO install must have its scan record (scans_without_record = 0) and both
sessions AUTO scans; otherwise no prediction is made.

Validity: as g3_1b_loop_analyze (dumps, INSTR), plus the registered conditions (far K f 1.0 seed 0, grid 1, shadow 1,
EXTRA_ARGS with --lidar-cast fast and --layer-worker <arm> --layer-worker-k 3), result.json layer_worker mode = the
arm with no failure, scan_switches grid / shadow True and lidar fast, layer_worker.json present, the mission done.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve()
_spec = importlib.util.spec_from_file_location(
    "g3_1b_loop_analyze", HERE.parents[1] / "2026-11-10-g3-1b-loop" / "g3_1b_loop_analyze.py")
base = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(base)
g2m = base.g2m
C = g2m.C
TARGET = 0.03707  # s per sim s: the 1e target in W terms (plan 5.28)
WAIT_OK_S = 0.0005  # the mean scheduled wait a scan for a k to be chosen
KS = (2, 3, 4)


class Session(base.Session):
    def __init__(self, out: Path, entry: dict, ps=None):
        self.arm = entry["env"]["LAYER_WORKER"]
        self.pair = 1
        g2m.Session.__init__(self, out, entry, ps=ps)
        res = self.result or {}
        lw = res.get("layer_worker") or {}
        if lw.get("mode") != self.arm or lw.get("k") != 3:
            self.invalid.append(f"result.json layer_worker {lw.get('mode')}/{lw.get('k')}, registered {self.arm}/3")
        if lw.get("failed"):
            self.invalid.append(f"layer worker failed: {lw.get('failed')}")
        sw = res.get("scan_switches") or {}
        if not (sw.get("grid_clear_kernel") and sw.get("shadow_kernel") and sw.get("lidar_cast") == "fast"):
            self.invalid.append(f"scan_switches {sw}")
        f = self.run / "layer_worker.json"
        self.installs = json.loads(f.read_text()) if f.exists() else None
        if self.installs is None:
            self.invalid.append("no layer_worker.json")

    def _check_conditions(self):
        c = self.cond
        for k in c:
            if k.startswith(g2m.FORBIDDEN):
                self.invalid.append(f"{k} set (dumps and the dual run stay out of the check)")
        want = {"SET": "far", "ARM": "K", "FACTOR": "1.0", "FORKLIFT_LIVE_CATCHUP_STEPS": "6", "SEED": "0",
                "FORKLIFT_GRID_CLEAR_KERNEL": "1", "FORKLIFT_SHADOW_KERNEL": "1"}
        for k, v in want.items():
            if c.get(k) != v:
                self.invalid.append(f"{k}={c.get(k)}, registered {v}")
        extra = c.get("EXTRA_ARGS", "").replace("\\ ", " ")
        for need in ("--lidar-cast fast", f"--layer-worker {self.arm} --layer-worker-k 3"):
            if need not in extra:
                self.invalid.append(f"EXTRA_ARGS without {need}")
        for k in g2m.INSTR_ON:
            if c.get(k) is not None:
                self.invalid.append(f"{k} set in an INSTR off session")

    # ---- the population ----------------------------------------------------------------------------------------
    def auto_population(self):
        """(row mask over the trace, AUTO sim s, set of AUTO ticks): rows i and i+1 AUTO and W_i finite."""
        D = self.trace
        w = g2m.work(D)
        auto = self.auto_rows()
        ok = np.zeros(len(D), bool)
        ok[:-1] = auto[:-1] & auto[1:] & np.isfinite(w[:-1])
        ticks = {int(t) for t in D[ok, C["tick"]] if np.isfinite(t)}
        return ok, w, float(ok.sum()) / 120.0, ticks

    def scans(self) -> list[dict]:
        f = self.run / "obstacle_scans.json"
        if f.exists():
            return json.loads(f.read_text())
        out = []  # a live run spools them (spool/obstacle.scans/*.jsonl), as g2m_analyze.m5_corrections reads them
        for f in sorted((self.run / "spool" / "obstacle.scans").glob("*.jsonl")):
            out += [json.loads(line) for line in f.read_text().splitlines() if line.strip()]
        return out

    def slam_records(self) -> list[dict]:
        out = []
        for f in sorted((self.run / "spool" / "slam.records").glob("*.jsonl")):
            out += [json.loads(line) for line in f.read_text().splitlines() if line.strip()]
        return out


def _q(v) -> dict | None:
    if not v:
        return None
    a = np.asarray(v, float)
    return {"n": int(a.size), "mean": float(a.mean()), "p50": float(np.median(a)), "p99": float(np.percentile(a, 99)),
            "max": float(a.max())}


def measure(s: Session) -> dict:
    ok, w, sim, ticks = s.auto_population()
    out = {"auto_sim_s": sim, "W": float(np.nansum(w[ok])) / sim if sim > 0 else None}
    inst = [r for r in s.installs if r.get("scan_tick") in ticks]
    by_stamp = {round(float(r["stamp_s"]), 6): r for r in s.scans() if r.get("stamp_s") is not None}
    main_layer, missing = 0.0, 0
    for r in inst:
        sc = by_stamp.get(round(float(r["stamp_s"]), 6))
        if sc is None or sc.get("total_wall_s") is None:
            missing += 1
            continue
        main_layer += (float(sc["total_wall_s"]) - float(sc.get("cast_wall_s") or 0.0)) + float(r["install_s"])
    out["scans"] = len(inst)
    out["scans_without_record"] = missing
    out["scans_per_sim_s"] = len(inst) / sim if sim > 0 else None
    out["main_layer"] = main_layer / sim if sim > 0 else None
    out["compute"] = sum(float(r["compute_s"]) for r in inst) / sim if sim > 0 else None
    cw = [float(r["compute_cpu_s"]) / float(r["compute_s"]) for r in inst if float(r["compute_s"]) > 0]
    out["compute_cpu_over_wall"] = _q(cw)
    barrier = [r for r in inst if r["reason"] != "scheduled"]
    reasons: dict = {}
    for r in barrier:
        reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
    out["barrier"] = {"reasons": reasons, "per_sim_s": len(barrier) / sim if sim > 0 else None,
                      "wait": sum(float(r["wait_s"]) for r in barrier) / sim if sim > 0 else None,
                      "install": sum(float(r["install_s"]) for r in barrier) / sim if sim > 0 else None}
    out["scheduled_wait_ms"] = _q([1e3 * float(r["wait_s"]) for r in inst if r["reason"] == "scheduled"])
    totals: dict = {}
    for _, _, _, timers in s.auto_windows():
        for k, v in timers.items():
            if g2m.head(k) == "isaac.step" or g2m.head(k).startswith("core."):
                totals[g2m.head(k)] = totals.get(g2m.head(k), 0.0) + v[0]
    wsim = sum(x[2] for x in s.auto_windows())
    out["main_other_raw"] = sum(totals.values()) / wsim if wsim > 0 else None
    out["main_other"] = None if out["main_other_raw"] is None or sim <= 0 else \
        out["main_other_raw"] - out["barrier"]["wait"] - out["barrier"]["install"]
    T = {}
    for k in KS:
        waits = []
        for r in inst:
            tt = {int(a): float(b) for a, b in (r.get("tick_t") or {}).items()}  # JSON keeps the keys as strings
            if k in tt:
                waits.append(max(0.0, (float(r["done_t"]) - float(r["submit_t"])) - (tt[k] - float(r["submit_t"]))))
        T[k] = _q([1e3 * x for x in waits])
    out["paired_wait_ms"] = T
    lags = [float(r["applied_tick_s"]) - float(r["stamp_s"]) for r in s.slam_records()
            if r.get("applied_tick_s") is not None and r.get("stamp_s") is not None]
    jobs = []
    for f in sorted((s.run / "missions").glob("*/records.json")):
        st = json.loads(f.read_text()).get("state") or {}
        jobs += [float(j["wall_s"]) for j in ((st.get("recognition_mission") or {}).get("live") or {}).get("jobs", [])
                 if j.get("wall_s") is not None]
    out["reported"] = {"slam_apply_lag_s": _q(lags), "far_job_wall_s": _q(jobs)}
    return out


def predict(delay: dict, thread: dict) -> dict:
    out = {"moved": delay["main_layer"] - thread["main_layer"],
           "main_other_increase": max(0.0, thread["main_other"] - delay["main_other"]),
           "barrier_wait": thread["barrier"]["wait"], "direct": delay["W"] - thread["W"], "net": {}}
    chosen = None
    for k in KS:
        q = thread["paired_wait_ms"][k]
        mean_s = None if q is None else q["mean"] / 1e3
        if mean_s is None:
            continue
        out["net"][k] = (out["moved"] - mean_s * thread["scans_per_sim_s"] - out["barrier_wait"]
                         - out["main_other_increase"])
        if chosen is None and mean_s <= WAIT_OK_S:
            chosen = k
    out["chosen_k"] = chosen if chosen is not None else 4
    out["chosen_by_rule"] = chosen is not None
    net = out["net"].get(out["chosen_k"])
    out["net_chosen"] = net
    out["residual_direct_minus_net"] = None if net is None else out["direct"] - net
    out["decision"] = None if net is None else ("go to 1e-1" if net >= TARGET else "process design to the lead")
    out["target_s_per_sim_s"] = TARGET
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=Path)
    ap.add_argument("--list", type=Path, required=True)
    ap.add_argument("--ps", type=Path, default=None)
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args()
    ps = g2m.read_ps(a.ps.read_text()) if a.ps else None
    entries = [json.loads(x) for x in a.list.read_text().splitlines() if x.strip()]
    arms = [e["env"].get("LAYER_WORKER") for e in entries]
    if arms != ["delay", "thread"]:
        print(json.dumps({"list_not_registered": arms}))
        return 2
    sessions = {e["env"]["LAYER_WORKER"]: Session(a.out / e["name"], e, ps=ps) for e in entries}
    rows = {arm: base.analyse_session(s) for arm, s in sessions.items()}
    out = {"sessions": rows}
    bad = [arm for arm, s in sessions.items() if s.invalid or not rows[arm].get("mission_done")]
    if bad:
        out["result"] = {"invalid_or_not_done": {arm: sessions[arm].invalid for arm in bad}}
    else:
        m = {arm: measure(s) for arm, s in sessions.items()}
        out["measures"] = m
        incomplete = {arm: {"scans": x["scans"], "scans_without_record": x["scans_without_record"]}
                      for arm, x in m.items() if x["scans"] == 0 or x["scans_without_record"]}
        out["result"] = {"incomplete": incomplete} if incomplete else predict(m["delay"], m["thread"])
    text = json.dumps(out, indent=1, default=str)
    if a.json:
        a.json.write_text(text)
    print(json.dumps(out["result"], indent=1, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
