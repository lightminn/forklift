"""Real-time plan 5.13: the baseline re-measure's analysis (descriptive; the rules are the plan's, fixed before).

    PYTHONPATH=SNAP/src python rtb_analyze.py SNAP RTB_DIR [--json out.json]

SNAP is the snapshot root (its sim/isaac/tick_trace.py and forklift_core are used, so the trace columns match the
runner that wrote them). RTB_DIR holds the job folders j*_*/ with one folder per run (rt_session.sh's OUT) and the
bundle's results.jsonl under j*/bundle/.

Per run (plan 5.13 "분석 절차" and "지표"):
  - the driving period: from the first `tracking` SLAM record (the spool's slam.records, read with the manifest's
    completeness and item counts checked) to the sim time of the probe's mission result;
  - W_i, the wait d, the wake-up overshoot eps and the interval S[i+1]-S[i], all from adjacent rows of the whole
    trace, then masked; the ratio |K| dt / sum of the intervals of the masked set K (never first-to-last S);
  - the identity check sum(interval) vs sum(max(W, h)) + sum(eps);
  - W's parts (as tools/tick_trace_report.py), their finiteness and sum against W (the rest: unclassified), the raw
    cost and the removal gain G_p = sum[max(W, h) - max(W - part, h)]; the "1x near" set (ratio >= 1/1.05): parts
    (eps one of them) in gain order, removed together, until sum max(W - h, 0) + sum eps <= 0.05 |K| h;
  - render counts by source, the render stalls (rendered and world.step > 0.15 s; secondary > 0.10 s, and non-render
    steps > 0.10 s) per sim minute;
  - the monitor over the driving window: GPU utilisation median/p95, SM clock, throttle reasons seen, the GPU
    processes outside the job (R1), the sample gaps, the bridge CPU and the other CPUs' use;
  - the bundle's cpu_s / wall_s and vram_max_mib.
Bundles of 3 slots: each step's count of the job's other sessions in their driving period at that step's S
(CLOCK_MONOTONIC, one host), the ratio and parts per count 3/2/1, and the sim-time window of the 3-common segment.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
from pathlib import Path

import numpy as np

SNAP = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(SNAP / "sim" / "isaac"))
sys.path.insert(0, str(SNAP / "src"))
from tick_trace import COLUMNS, FLAGS, work  # noqa: E402
from forklift_core.live.spool import load_list  # noqa: E402

DT = 1 / 120
C = {name: i for i, name in enumerate(COLUMNS)}
BIT = {name: 1 << i for i, name in enumerate(FLAGS)}
PARTS = ("gate", "world", "step_rest", "after", "between", "control", "to_wait")


def last_json(path: Path, pred):
    out = None
    if path.exists():
        for line in path.read_text().splitlines():
            try:
                row = json.loads(line)
            except ValueError:
                continue
            if pred(row):
                out = row
    return out


def mission_end_sim(out: Path):
    """(sim seconds the mission ended, the result, a problem or None) from the probe's report: the result must be
    this mission's (its mission_id the submit ack's, no result_not_this_mission mark) and carry ended_s
    (forklift_core.mission.results)."""
    probe = out / "probe.json"
    if not probe.exists():
        return None, None, "no probe report"
    runs = json.loads(probe.read_text()).get("missions_run") or []
    if len(runs) != 1:
        return None, None, f"{len(runs)} missions in the report (1 expected)"
    entry = runs[0]
    res = entry.get("result") or {}
    mid = (entry.get("submit") or {}).get("mission_id")
    if entry.get("result_not_this_mission") or mid is None or res.get("mission_id") != mid:
        return None, res, f"result not this mission's (ack {mid}, result {res.get('mission_id')})"
    if not isinstance(res.get("ended_s"), (int, float)):
        return None, res, "result without ended_s"
    return float(res["ended_s"]), res, None


def analyse_run(out: Path, factor: float = 1.0) -> dict:
    run = out / "live" / "epoch_1" / "run"
    r: dict = {"name": out.name, "valid": False, "why": []}
    end = last_json(run / "live_log.jsonl", lambda x: x.get("event") == "end")
    saved = (end or {}).get("tick_trace") or {}
    if (run / "result.json").exists():  # the run's result carries the same save record
        saved = json.loads((run / "result.json").read_text()).get("tick_trace") or saved
    r["tick_trace"] = saved
    if not (run / "tick_trace.npz").exists() or saved.get("error") or saved.get("dropped", 1) != 0:
        r["why"].append("trace missing, errored or dropped")
        return r
    with np.load(run / "tick_trace.npz") as z:
        if list(z["columns"]) != list(COLUMNS) or list(z["flags"]) != list(FLAGS):
            r["why"].append("trace columns differ")
            return r
        data = z["data"]
    try:
        records = load_list(run / "spool", "slam.records")
        r["spool_complete"] = True
    except ValueError as exc:
        r["spool_complete"] = False
        r["why"].append(f"spool: {exc}")
        records = load_list(run / "spool", "slam.records", allow_incomplete=True)
    stamps = [x["stamp_s"] for x in records if x.get("mode") == "tracking"]
    if not stamps:
        r["why"].append("no tracking record")
        return r
    t0 = float(min(stamps))
    t1, result, problem = mission_end_sim(out)
    r["mission_result"] = result
    if problem:
        r["why"].append(problem)
        return r
    r["driving_sim_s"] = [t0, t1]
    h = DT / factor
    tick = data[:, C["tick"]]
    sim = tick * DT
    S, q, d = data[:, C["S"]], data[:, C["q"]], data[:, C["d"]]
    w = work(data)  # W_i = q[i+1] - S[i]
    n = len(data)
    nxt = np.r_[np.arange(1, n), n - 1]
    interval = np.r_[S[1:] - S[:-1], np.nan]
    eps = np.r_[S[1:] - (q[1:] + d[1:]), np.nan]
    wait = np.r_[d[1:], np.nan]
    sim_next = np.r_[sim[1:], np.inf]
    inside = (sim >= t0) & (sim_next <= t1)  # the step and the interval to the next step both in the driving period
    K = inside & np.isfinite(w) & np.isfinite(interval)
    k = int(K.sum())
    if k * DT < 60:
        r["why"].append(f"driving period {k * DT:.1f} sim s < 60")
        return r
    r["valid"] = r["spool_complete"]
    nx = data[nxt]
    parts = {
        "gate": data[:, C["world_start"]] - S,
        "world": data[:, C["world_end"]] - data[:, C["world_start"]],
        "step_rest": data[:, C["step_end"]] - data[:, C["world_end"]],
        "after": data[:, C["after_end"]] - data[:, C["step_end"]],
        "between": nx[:, C["ctrl_start"]] - data[:, C["after_end"]],
        "control": nx[:, C["ctrl_end"]] - nx[:, C["ctrl_start"]],
        "to_wait": nx[:, C["q"]] - nx[:, C["ctrl_end"]],
    }
    replies = np.nan_to_num(nx[:, C["replies_s"]])
    r.update(block(K, w, interval, eps, wait, parts, replies, h, data))
    # renders and stalls over the driving rows (the last one included)
    D = inside
    bits = np.nan_to_num(data[:, C["flags"]]).astype(int)
    rendered = (bits & BIT["rendered"]) != 0
    world = parts["world"]
    stepall = data[:, C["after_end"]] - S
    minutes = D.sum() * DT / 60
    r["renders"] = {f: int(np.sum(D & ((bits & BIT[f]) != 0))) for f in FLAGS if f != "scan"}
    r["stalls"] = {
        "render_gt_0.15": int(np.sum(D & rendered & (world > 0.15))),
        "render_gt_0.10": int(np.sum(D & rendered & (world > 0.10))),
        "nonrender_gt_0.10": int(np.sum(D & ~rendered & (stepall > 0.10))),
        "per_sim_min_render_gt_0.15": float(np.sum(D & rendered & (world > 0.15)) / minutes) if minutes else None,
        "list": [{"sim_s": float(sim[i]), "world_s": float(world[i]), "step_s": float(stepall[i]),
                  "rendered": bool(rendered[i]), "phase": int(tick[i]) % 12, "S_mono": float(S[i]),
                  "flags": [f for f in FLAGS if bits[i] & BIT[f]]}
                 for i in np.flatnonzero(D & (((world > 0.10) & rendered) | (stepall > 0.10)))],
    }
    di = np.flatnonzero(D)
    r["window_mono"] = [float(S[di[0]]), float(S[di[-1] + 1])]
    r["monitor"] = monitor(out / "monitor", *r["window_mono"])
    # for the bundle concurrency
    r["_S"], r["_K"], r["_sim"] = S, K, sim
    r["_arr"] = (w, interval, eps, wait, parts, replies, h, data)
    r["_D"], r["_rendered"], r["_world"], r["_stepall"] = D, rendered, world, stepall
    return r


def block(K, w, interval, eps, wait, parts, replies, h, data) -> dict:
    """The masked set's ratio, identity, overrun, parts and removal gains (plan 5.13 지표)."""
    k = int(K.sum())
    if k == 0:
        return {"rows": 0}
    W, I, E = w[K], interval[K], eps[K]
    P = {p: v[K] for p, v in parts.items()}
    finite = np.all(np.vstack([np.isfinite(v) for v in P.values()]), axis=0)
    psum = np.sum(np.vstack([np.nan_to_num(v) for v in P.values()]), axis=0)
    resid = W - psum
    T = float(np.sum(I))
    over_work = float(np.sum(np.maximum(W - h, 0)))
    eps_sum = float(np.nansum(E))
    base = np.maximum(W, h)
    gains = {p: float(np.sum(base - np.maximum(W - np.nan_to_num(v), h))) for p, v in P.items()}
    gains["eps"] = eps_sum
    # a part missing on a row may hold anything from 0 to that row's whole unclassified rest (Codex round 3 P2)
    rest_pos = np.maximum(resid, 0)
    gains_hi = {p: float(np.sum(base - np.maximum(W - np.where(np.isfinite(v), v, rest_pos), h)))
                for p, v in P.items()}
    gains_hi["eps"] = eps_sum
    raw = {p: float(np.nansum(v)) for p, v in P.items()}
    raw["control_replies"] = float(np.sum(replies[K]))
    raw["eps"] = eps_sum
    raw["unclassified"] = float(np.sum(resid))
    # the 1x-near set: parts in gain order removed together until total excess <= 0.05 k h; empty when already
    # there; computed with the low and the high part values (a part missing on a row), uncertain when they differ
    def near_set(gain, fill_hi):
        chosen = []
        for p in [None] + sorted(gain, key=lambda x: -gain[x]):
            if p is not None:
                chosen.append(p)
            rest = W - sum(np.where(np.isfinite(P[x]), P[x], rest_pos if fill_hi else 0.0)
                           for x in chosen if x != "eps")
            excess = float(np.sum(np.maximum(rest - h, 0))) + (0.0 if "eps" in chosen else eps_sum)
            if excess <= 0.05 * k * h:
                return chosen
        return None

    order = sorted(gains, key=lambda p: -gains[p])
    near_lo, near_hi = near_set(gains, False), near_set(gains_hi, True)
    # confirmed only when both ends agree and no part inside the set has a gain range overlapping one outside it
    # (any allocation of a missing part could reorder them; Codex round 5 P2)
    overlap = near_lo is not None and any(gains[x] <= gains_hi[y] and gains[y] <= gains_hi[x]
                                          for x in near_lo for y in gains if y not in near_lo)
    near = near_lo if near_lo == near_hi and not overlap else {"uncertain": [near_lo, near_hi]}
    kinds = kind_of(data[K, C["flags"]])
    phase = data[K, C["tick"]].astype(int) % 12
    over = np.maximum(W - h, 0)
    sim_s = k * DT
    return {
        "rows": k,
        "sim_s": sim_s,
        "ratio": sim_s / T,
        "wall_per_sim_s": T / sim_s,
        "identity_resid_per_sim_s": (T - float(np.sum(base)) - eps_sum) / sim_s,
        "excess_per_sim_s": (T - k * h) / sim_s,
        "overrun_work_per_sim_s": over_work / sim_s,
        "eps_per_sim_s": eps_sum / sim_s,
        "wait_per_sim_s": float(np.nansum(wait[K])) / sim_s,
        "raw_per_sim_s": {p: v / sim_s for p, v in raw.items()},
        "gain_per_sim_s": {p: v / sim_s for p, v in gains.items()},
        "gain_hi_per_sim_s": {p: v / sim_s for p, v in gains_hi.items()},
        "gain_order": order,
        # a rank is confirmed when its gain range [low, high] overlaps no other part's range
        "gain_rank_confirmed": [not any(gains[o] <= gains_hi[x] and gains[x] <= gains_hi[o]
                                        for x in order if x != o) for o in order],
        "near_1x_set": near,
        "rows_part_missing": int(np.sum(~finite)),
        "rows_part_missing_frac": float(np.mean(~finite)),
        "overrun_by_kind_per_sim_s": {kd: float(over[kinds == kd].sum() / sim_s) for kd in sorted(set(kinds))},
        "rows_by_kind": {kd: int(np.sum(kinds == kd)) for kd in sorted(set(kinds))},
        "mean_ms_by_kind": {kd: {p: float(np.nanmean(P[p][kinds == kd]) * 1000) for p in P}
                            for kd in sorted(set(kinds))},
        "overrun_by_phase_per_sim_s": [float(over[phase == j].sum() / sim_s) for j in range(12)],
    }


def kind_of(flags):
    bits = np.nan_to_num(flags).astype(int)
    render = (bits & BIT["rendered"]) != 0
    scan = (bits & BIT["scan"]) != 0
    return np.where(render & scan, "render+scan", np.where(render, "render", np.where(scan, "scan", "plain")))


def monitor(folder: Path, m0: float, m1: float) -> dict:
    out: dict = {}
    host = folder / "host.jsonl"
    rows = [json.loads(x) for x in host.read_text().splitlines() if x.strip()] if host.exists() else []
    win = [x for x in rows if m0 <= x["mono"] <= m1]
    out["host_samples"] = len(win)
    out["host_expected"] = int(m1 - m0)
    monos = [x["mono"] for x in win]
    out["host_gap_max_s"] = float(np.max(np.diff(monos))) if len(monos) > 1 else None
    foreign, unknown = 0, 0
    names = set()
    for x in win:
        if x.get("gpu_status") != "ok" or x.get("gpu_procs") is None:
            unknown += 1
            continue
        if any(p[3] is None for p in x["gpu_procs"]):
            unknown += 1
        bad = [p for p in x["gpu_procs"] if p[3] is False]
        if bad:
            foreign += 1
            names.update(f"{p[0]}:{p[2]}" for p in bad)
    out["gpu_foreign_samples"] = foreign
    out["gpu_unknown_samples"] = unknown
    out["gpu_foreign_procs"] = sorted(names)
    out["r1"] = ("contaminated" if foreign else
                 "unverified" if (not win or unknown > 0.10 * len(win) or len(win) < 0.9 * out["host_expected"])
                 else "clean")
    b = [x["bridge"] for x in win if x.get("bridge")]
    if len(b) >= 2:
        out["bridge_cores"] = (int(b[-1]["usage_usec"]) - int(b[0]["usage_usec"])) / 1e6 / (m1 - m0)
    if len(win) >= 2:  # per-CPU busy share over the window
        a, z = win[0]["cpus"], win[-1]["cpus"]
        busy = {}
        for ra, rz in zip(a, z):
            va, vz = np.array(ra[1:], float), np.array(rz[1:], float)
            dv = vz - va
            tot = dv.sum()
            idle = dv[3] + dv[4]
            busy[ra[0]] = float(1 - idle / tot) if tot > 0 else None
        out["cpu_busy"] = busy
        out["loadavg"] = [win[0]["loadavg"], win[-1]["loadavg"]]
        out["squeue"] = [x["squeue"] for x in win if "squeue" in x][:1]
    gpu = folder / "gpu.csv"
    if gpu.exists():
        lines = gpu.read_text().splitlines()
        head = [x for x in lines if x.startswith("# start")]
        if head:
            parts_ = head[0].split()
            offset = float(parts_[5]) - float(parts_[3])  # wall - mono
            util, clk, thr = [], [], set()
            for line in lines:
                if line.startswith("#"):
                    continue
                c = [x.strip() for x in line.split(",")]
                try:
                    wall = dt.datetime.strptime(c[0], "%Y/%m/%d %H:%M:%S.%f").timestamp()
                except ValueError:
                    continue
                mono = wall - offset
                if m0 <= mono <= m1:
                    try:
                        util.append(float(c[2]))
                        clk.append(float(c[4]))
                    except ValueError:
                        pass
                    thr.add(c[7] if len(c) > 7 else "?")
            if util:
                out["gpu_util_median"] = float(np.median(util))
                out["gpu_util_p95"] = float(np.percentile(util, 95))
                out["sm_clock_median"] = float(np.median(clk))
                out["gpu_samples"] = len(util)
                out["throttle_reasons_seen"] = sorted(thr)
    return out


def stalls_in(r, mask) -> dict:
    D, rendered, world, stepall = r["_D"] & mask, r["_rendered"], r["_world"], r["_stepall"]
    minutes = D.sum() * DT / 60
    n15 = int(np.sum(D & rendered & (world > 0.15)))
    return {"render_gt_0.15": n15, "render_gt_0.10": int(np.sum(D & rendered & (world > 0.10))),
            "nonrender_gt_0.10": int(np.sum(D & ~rendered & (stepall > 0.10))),
            "per_sim_min_render_gt_0.15": n15 / minutes if minutes else None}


def window_block(r, mask) -> dict:
    w, interval, eps, wait, parts, replies, h, data = r["_arr"]
    blk = block(r["_K"] & mask, w, interval, eps, wait, parts, replies, h, data)
    blk["stalls"] = stalls_in(r, mask)
    return blk


def concurrency(runs: list[dict]) -> None:
    """A 3-slot job: per step, the number of the job's sessions in their driving period at that step's S; the
    measures per count 3/2/1, the 3-common segment's sim window since t0, and whether it reaches 60 sim s."""
    ok = [r for r in runs if "_S" in r]
    for r in ok:
        S = r["_S"]
        count = np.ones(len(S), int)
        for o in ok:
            if o is not r:
                a, b = o["window_mono"]
                count += (S >= a) & (S <= b)
        r["by_concurrency"] = {}
        for c in (3, 2, 1):
            m = count == c
            if (r["_K"] & m).any():
                blk = window_block(r, m)
                sims = r["_sim"][r["_K"] & m]
                blk["sim_window_since_t0"] = [float(sims.min() - r["driving_sim_s"][0]),
                                              float(sims.max() - r["driving_sim_s"][0])]
                r["by_concurrency"][str(c)] = blk
        three = r["by_concurrency"].get("3", {})
        r["c3_common_sim_s"] = three.get("sim_s", 0.0)
    enough = len(ok) == 3 and all(r["c3_common_sim_s"] >= 60 for r in ok)
    for r in runs:
        r["c3_job_scope"] = "3-common" if enough else "max-3-slot bundle"


def phase_matched(i_runs: list[dict], c3_runs: list[dict]) -> None:
    """Each alone run of a level measured again over every same-level 3-slot session's 3-common window in elapsed
    sim seconds since its own t0 (key "elapsed_matched"). This matches elapsed time, not mission phases: where the
    phase transitions fall at other times the windows can still hold different phases (Codex round 5 P2)."""
    for r in i_runs:
        if "_S" not in r:
            continue
        r["elapsed_matched"] = {}
        for c in c3_runs:
            win = ((c.get("by_concurrency") or {}).get("3") or {}).get("sim_window_since_t0")
            if c.get("level") != r.get("level") or not win:
                continue
            t0 = r["driving_sim_s"][0]
            m = (r["_sim"] >= t0 + win[0]) & (r["_sim"] <= t0 + win[1])
            if (r["_K"] & m).any():
                blk = window_block(r, m)
                r["elapsed_matched"][c["name"]] = {k: blk[k] for k in ("sim_s", "ratio", "excess_per_sim_s",
                                                                       "gain_per_sim_s", "stalls")}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("snap")
    ap.add_argument("rtb", type=Path)
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    out = []
    for job in sorted(p for p in a.rtb.glob("j*_*") if p.is_dir()):
        res = {}
        bres = job / "bundle" / "results.jsonl"
        if bres.exists():
            for line in bres.read_text().splitlines():
                x = json.loads(line)
                res[x["name"]] = x  # the last attempt wins
        runs = []
        for run in sorted(p for p in job.iterdir() if p.is_dir() and p.name != "bundle"):
            r = analyse_run(run)
            b = res.get(run.name) or {}
            r["bundle"] = {k: b.get(k) for k in ("status", "attempt", "rc", "cpu_s", "wall_s", "vram_max_mib",
                                                  "stalled", "infra", "bridge_cpu")}
            if b.get("cpu_s") and b.get("wall_s"):
                r["session_cores"] = b["cpu_s"] / b["wall_s"]
            r["job"] = job.name
            r["level"] = run.name.rsplit("_", 1)[-1]
            runs.append(r)
        if "c3" in job.name:
            concurrency(runs)
        out.extend(runs)
    phase_matched([r for r in out if "c3" not in r["job"] and "warm" not in r["name"]],
                  [r for r in out if "c3" in r["job"]])
    for r in out:
        for key in [k for k in r if k.startswith("_")]:
            del r[key]
        print(f"{r['job']:14s} {r['name']:26s} valid {r['valid']!s:5s} ratio {r.get('ratio', float('nan')):.4f} "
              f"excess/s {r.get('excess_per_sim_s', float('nan')):.4f} near {r.get('near_1x_set')} "
              f"stalls15 {(r.get('stalls') or {}).get('render_gt_0.15')} r1 {(r.get('monitor') or {}).get('r1')} "
              f"why {r['why']}")
    if a.json:
        a.json.write_text(json.dumps(out, indent=1, default=float))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
