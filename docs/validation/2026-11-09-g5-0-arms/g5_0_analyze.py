"""Real-time plan 5.24: the G5-0 arms' measures and the registered reading.

Per session: validity (g2m_analyze.Session: ① another GPU process, ② other users' CPU from the sampler with --ps; and
the prep's physics time unchanged), the first two render steps after the first control tick (world.step s and their
start gaps), the prep renders' and the wait's wall times (the live log's ``live_test_prep`` event), the first ``ok``
from the runner start and from the first tick (M6), the mission outcome, and the 0.25 s gaps in the first 10 s.

Per arm (the warm-up left out): a stall k (k = 1, 2) is "gone" when both sessions' render step k is at most 0.25 s, and
"reduced" when the arm's median is at most half of P0's. The reading follows the registration's rules; when more than
one holds, every one is listed.

    PYTHONPATH=SNAP/src python g5_0_analyze.py OUT_DIR --list OUT_DIR.jsonl --ps PS.jsonl [--json out.json]
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve()
_spec = importlib.util.spec_from_file_location("g2m_analyze", HERE.parents[1] / "2026-11-09-g2-measure" / "g2m_analyze.py")
g2m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(g2m)
C, BIT = g2m.C, g2m.BIT
EDGE_S = 0.25


def render_steps(trace: np.ndarray, first_row: int, k: int = 2) -> list[dict]:
    """The first k rendered steps at or after the first control tick: world.step s and the start gap holding it."""
    flags = np.nan_to_num(trace[:, C["flags"]]).astype(int)
    rows = [i for i in np.where(flags & BIT["rendered"])[0] if i >= first_row][:k]
    out = []
    for i in rows:
        world = float(trace[i, C["world_end"]] - trace[i, C["world_start"]])
        gap = float(trace[i + 1, C["S"]] - trace[i, C["S"]]) if i + 1 < len(trace) else float("nan")
        out.append({"row": int(i), "tick": float(trace[i, C["tick"]]), "world_s": world, "gap_s": gap})
    return out


def session_row(sess, arm: str) -> dict:
    row = {"name": sess.name, "arm": arm, "invalid": list(sess.invalid)}
    prep = next((r for r in sess.log if r.get("event") == "live_test_prep"), None)
    if arm in ("P1", "P2", "W"):
        if prep is None:
            row["invalid"].append("no live_test_prep event for a prep arm")
        elif not prep.get("physics_unchanged"):
            row["invalid"].append("the prep moved the physics time")
    elif prep is not None:
        row["invalid"].append(f"a live_test_prep event in arm {arm}")
    if prep is not None:
        row["prep"] = {"renders_s": [b - a for a, b in prep.get("renders", [])],
                       "wait_s": (prep["wait"][1] - prep["wait"][0]) if prep.get("wait") else None,
                       "physics_unchanged": prep.get("physics_unchanged")}
    if sess.trace is None:
        return row
    first = sess.first_ctrl_row()
    row["render_steps"] = render_steps(sess.trace, first)
    S = sess.trace[:, C["S"]]
    t0 = S[first]
    gaps = np.diff(S[first:])
    within = (S[first:-1] - t0) <= 10.0
    row["gaps_over_0_25_first_10s"] = int(((gaps > EDGE_S) & within).sum())
    m6 = sess.m6_start()
    row["first_ok_from_first_tick_s"] = m6.get("first_ok_from_first_tick_s")
    row["first_ok_from_runner_s"] = m6.get("first_ok_from_runner_s")
    row["mission_outcomes"] = sess.mission_outcomes
    return row


def arm_summary(rows: list[dict]) -> dict:
    by = {}
    for r in rows:
        if r["arm"] == "warm" or r["invalid"] or len(r.get("render_steps", [])) < 2:
            continue
        by.setdefault(r["arm"], []).append(r)
    out = {}
    base = {k: float(np.median([r["render_steps"][k]["world_s"] for r in by.get("P0", [])])) if by.get("P0") else None
            for k in (0, 1)}
    for arm, rs in sorted(by.items()):
        s = {"sessions": len(rs)}
        for k in (0, 1):
            vals = [r["render_steps"][k]["world_s"] for r in rs]
            med = float(np.median(vals))
            s[f"stall{k + 1}"] = {"world_s": vals, "median_s": med, "gone": len(rs) == 2 and all(v <= EDGE_S for v in vals),
                                  "reduced": base[k] is not None and arm != "P0" and med <= base[k] / 2}
        out[arm] = s
    return out


def reading(summary: dict) -> list[str]:
    """The registration's rules (plan 5.24), each one that holds."""
    def gone(arm, k):
        return summary.get(arm, {}).get(f"stall{k}", {}).get("gone", False)

    def eased(arm, k):
        st = summary.get(arm, {}).get(f"stall{k}", {})
        return st.get("gone", False) or st.get("reduced", False)
    out = []
    if eased("P1", 1) and gone("P2", 1) and gone("P2", 2):
        out.append("first render-graph evaluation/preparation (P1 eases stall 1, P2 removes both)")
    if eased("W", 1) or eased("W", 2):
        out.append("time-based background work (the wait alone eases a stall)")
    if (gone("O", 1) or gone("O", 2)) and not (gone("P2", 1) or gone("W", 1)):
        out.append("the learned worker / IPC (only the oracle arm removes a stall)")
    if not out:
        out.append("none of the arms removes a stall: a Kit profiler arm is next (registered separately)")
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--list", type=Path, required=True)
    ap.add_argument("--ps", type=Path, required=True, help="g2m_cpu_sampler.py's record (rule 2)")
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    ps = g2m.read_ps(a.ps.read_text())
    rows = []
    for line in a.list.read_text().splitlines():
        if not line.strip():
            continue
        e = json.loads(line)
        arm = e["name"].rsplit("_", 1)[1]
        d = a.out_dir / e["name"]
        if not d.exists():
            rows.append({"name": e["name"], "arm": arm, "invalid": ["missing"]})
            continue
        rows.append(session_row(g2m.Session(d, e, None, ps), arm))
    summary = arm_summary(rows)
    out = {"sessions": rows, "arms": summary, "reading": reading(summary)}
    for r in rows:
        print(json.dumps({k: r.get(k) for k in ("name", "arm", "invalid", "render_steps")}, default=float))
    print(json.dumps({"arms": summary, "reading": out["reading"]}, indent=1, default=float))
    if a.json:
        a.json.write_text(json.dumps(out, indent=1, default=float))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
