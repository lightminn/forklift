"""Real-time plan 5.22 results: the tables of docs/validation/2026-11-09-g2-measure-results.md, from g2m_analyze.py's
JSON and the sessions' own records (tick trace, live log's status-line timers). Descriptive only -- the registered
verdicts are g2m_analyze.py's; these are the readings the review asked for (Codex round 1 on the results):

- gaps: per session, the start gaps over 0.25 s, how many have world.step alone over 0.25 s, the largest control tick
  inside them, and the 0.20-0.25 s gaps with their control share;
- periodic: the long world.steps' wall spacing and each session's phase on one fitted period (a per-process timer or a
  node-wide one);
- phases: the INSTR on - off difference of each AUTO phase (ms/sim s) and of each region's per-call time, from the
  status-line timers both arms keep (the on arm's ``name|parent`` keys folded onto their name);
- refresh: the on arm's subtree under the obstacle scan's refresh (by_parent), mean over the on sessions -- in the AUTO
  windows (the registered population) and, apart, over the whole session; the per-call comparison is AUTO too.

    PYTHONPATH=SNAP/src python g2m_tables.py ANALYSIS.json RUN_DIR [RUN_DIR ...] [--set H|F]
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))
sys.path.insert(0, str(ROOT / "src"))
from tick_trace import COLUMNS  # noqa: E402
from forklift_core.live.spool import read_live_log  # noqa: E402

C = {n: i for i, n in enumerate(COLUMNS)}
REFRESH = "layer.refresh<obstacle_scan"
START_TICKS = 200  # set F's two start stalls sit at physics ticks 84 and 96; the periodic reading leaves them out


def find(run_dirs, name):
    for d in run_dirs:
        if (d / name).exists():
            return d / name
    return None


def timers(run: Path, auto_only: bool = True) -> tuple[dict, float]:
    """{name: [wall s, calls]} and the sim s they cover, from the status-line windows -- the timers a line carries are
    those since the line before, so a window is (previous line, line] and the first line opens none (Codex round 2
    P2-3: the old sum took the first line's timers over the last-minus-first span). ``auto_only``: only the windows
    whose every command line is AUTO, g2m_analyze.py's ``auto_windows`` (the registered M3 population)."""
    log = read_live_log(run)
    cmd = [r for r in log if "pacing_ok" in r and "speed_mps" in r and r.get("mono") is not None]
    lines = [r for r in log if r.get("status") and isinstance(r.get("timers"), dict) and r.get("mono") is not None
             and r.get("tick_t") is not None]
    cmd_mono = np.array([r["mono"] for r in cmd])
    cmd_auto = np.array([r.get("mode") == "AUTO" for r in cmd])
    out = defaultdict(lambda: [0.0, 0])
    span = 0.0
    for a, b in zip(lines, lines[1:]):
        if auto_only:
            sel = (cmd_mono > a["mono"]) & (cmd_mono <= b["mono"])
            if not (sel.any() and cmd_auto[sel].all()):
                continue
        span += b["tick_t"] - a["tick_t"]
        for k, v in b["timers"].items():
            out[k][0] += v[0]
            out[k][1] += v[1]
    return dict(out), span if span > 0 else float("nan")


def folded(t: dict) -> dict:
    """The on arm's ``name|parent`` keys added onto ``name`` (the off arm keeps only ``name``); a name kept both plain
    and by parent in one run would count twice -- listed so the reader can drop it."""
    out = defaultdict(lambda: [0.0, 0])
    for k, (s, c) in t.items():
        out[k.split("|", 1)[0]][0] += s
        out[k.split("|", 1)[0]][1] += c
    return dict(out)


def gaps(row: dict) -> dict:
    m1 = row["M1"]
    over = [g for g in m1 if g["gap_s"] > 0.25]
    near = [g for g in m1 if g["gap_s"] <= 0.25]
    return {"over_0_25": len(over), "world_alone_over_0_25": sum(g["parts"]["world"] > 0.25 for g in over),
            "max_control_inside_over": max([g["parts"]["control_next"] for g in over] or [0.0]),
            "near": [{"gap_s": round(g["gap_s"], 3), "control_s": round(g["parts"]["control_next"], 3),
                      "world_s": round(g["parts"]["world"], 3),
                      "top": [c["name"] for c in g.get("top_calls", [])[:2]]} for g in near]}


def long_steps(sess: Path) -> np.ndarray:
    d = np.load(sess / "live" / "epoch_1" / "run" / "tick_trace.npz")["data"]
    w = d[:, C["world_end"]] - d[:, C["world_start"]]
    keep = (w > 0.25) & (d[:, C["tick"]] >= START_TICKS)
    return d[keep, C["world_start"]]


def periodic(starts: dict) -> dict:
    t = np.concatenate([v for v in starts.values() if len(v)]) if starts else np.array([])
    if len(t) < 3:
        return {}
    spacing = np.concatenate([np.diff(v) for v in starts.values() if len(v) > 1])
    base = float(np.median(spacing[spacing < 25]))
    grid = np.arange(base - 0.5, base + 0.5, 0.0005)
    within = []
    for P in grid:  # one period for every session, each session free in phase
        within.append(np.mean([abs(np.mean(np.exp(2j * np.pi * v / P))) for v in starts.values() if len(v) > 2]))
    P = float(grid[int(np.argmax(within))])
    glob = abs(np.mean(np.exp(2j * np.pi * t / P)))
    per = {}
    for n, v in starts.items():
        if len(v) > 2:
            z = np.mean(np.exp(2j * np.pi * v / P))
            per[n] = {"n": len(v), "coherence": round(float(abs(z)), 2),
                      "phase_s": round(float((np.angle(z) / (2 * np.pi) * P) % P), 2)}
    return {"median_spacing_s": round(base, 3), "period_s": round(P, 4), "pooled_coherence": round(float(glob), 3),
            "sessions": per}


def phases(rows, on, off) -> dict:
    keys = ("W", "world", "step_rest", "after", "control")
    get = {n: r["M7"]["auto_s_per_sim_s"] for n, r in rows.items()}
    out = {}
    for k in keys:
        a, b = np.mean([get[n][k] for n in on]), np.mean([get[n][k] for n in off])
        out[k] = {"on": round(a * 1e3, 2), "off": round(b * 1e3, 2), "diff_ms": round((a - b) * 1e3, 2),
                  "rel": round((a - b) / b, 4)}
    return out


def per_call(tm: dict, on, off, names) -> dict:
    def mean_pc(ns, k):
        vals = [tm[n][0].get(k) for n in ns]
        vals = [v for v in vals if v and v[1]]
        return float(np.mean([v[0] / v[1] for v in vals])) if vals else float("nan")
    out = {}
    for k in names:
        a, b = mean_pc(on, k), mean_pc(off, k)
        out[k] = {"on_us": round(a * 1e6, 1), "off_us": round(b * 1e6, 1), "rel": round(a / b - 1, 4) if b else None}
    return out


def subtree(raw: dict, on) -> list:
    acc = defaultdict(lambda: [0.0, 0.0])
    for n in on:
        t, span = raw[n]
        for k, (s, c) in t.items():
            acc[k][0] += s / span / len(on)
            acc[k][1] += c / span / len(on)
    rows = []

    def walk(parent, depth):
        kids = sorted((k for k in acc if k.split("|", 1)[1:] == [parent]), key=lambda k: -acc[k][0])
        for k in kids:
            sub = sum(acc[c][0] for c in acc if c.split("|", 1)[1:] == [k])
            rows.append({"depth": depth, "name": k.split("|", 1)[0], "ms_per_sim_s": round(acc[k][0] * 1e3, 2),
                         "calls_per_sim_s": round(acc[k][1], 1), "self_ms_per_sim_s": round((acc[k][0] - sub) * 1e3, 2)})
            walk(k, depth + 1)
    rows.append({"depth": 0, "name": REFRESH, "ms_per_sim_s": round(acc[REFRESH][0] * 1e3, 2)})
    walk(REFRESH, 1)
    return rows


REGIONS = ("step.plain", "step.scan", "step.render", "isaac.step", REFRESH, "grid.snapshot<refresh", "shadow.apply",
           "lidar.cast", "core.permission", "core.follow", "core.assemble_command", "core.cusp_tick",
           "core.cusp_and_observe_recovery", "core.capture_and_detect", "core.slam_replies")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("analysis", type=Path)
    ap.add_argument("runs", type=Path, nargs="+")
    ap.add_argument("--set", choices=("H", "F"), required=True)
    a = ap.parse_args()
    j = json.loads(a.analysis.read_text())
    rows = {r["name"]: r for r in j["sessions"] if r.get("set") == a.set and not r.get("invalid") and "M7" in r}
    judged = {n: r for n, r in rows.items() if not r.get("warm")}
    on = [n for n, r in judged.items() if r["instr"] == "on"]
    off = [n for n, r in judged.items() if r["instr"] == "off"]
    sess = {n: find(a.runs, n) for n in rows}
    raw = {n: timers(sess[n] / "live" / "epoch_1" / "run") for n in judged}
    raw_all = {n: timers(sess[n] / "live" / "epoch_1" / "run", auto_only=False) for n in judged}
    tm = {n: (folded(t), s) for n, (t, s) in raw.items()}
    out = {"set": a.set, "on": on, "off": off,
           "gaps": {n: gaps(r) for n, r in rows.items()},
           "periodic_long_world_steps": periodic({n: long_steps(sess[n]) for n in rows}),
           "phases_on_minus_off": phases(judged, on, off) if on and off else None,
           "per_call_on_vs_off": per_call(tm, on, off, REGIONS) if on and off else None,
           "refresh_subtree_on_auto": subtree(raw, on) if on else None,
           "refresh_subtree_on_whole_session": subtree(raw_all, on) if on else None}
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
