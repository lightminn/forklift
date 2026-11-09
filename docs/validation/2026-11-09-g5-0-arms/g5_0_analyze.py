"""Real-time plan 5.24: the G5-0 arms' measures and the registered reading.

Per session: validity (g2m_analyze.Session: ① another GPU process, ② other users' CPU from the sampler with --ps; and
the prep's physics time unchanged), the first two render steps after the first control tick (world.step s and their
start gaps), the prep renders' and the wait's wall times (the live log's ``live_test_prep`` event), the first ``ok``
from the runner start and from the first tick (M6), the mission outcome, and the 0.25 s gaps in the first 10 s.

Per arm (the warm-up left out), each stall k (k = 1, 2) on its own: "gone" when every session's render step k is at
most 0.25 s, "reduced" when the arm's median is at most half of P0's; "split" when the sessions disagree (one at most
0.25 s, one above: a third session is due, plan 5.24); "shifted" when a later render step in the first 10 s takes over
0.25 s (the stall moved, not gone). A prep arm whose renders each took at most 0.25 s while the stalls stayed did not
do the live renders' work: the render reading is then undecided, not rejected. The oracle arm counts only with the
same semantic labels as P0. The reading follows the registration's rules; when more than one holds, each is listed.

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


def render_steps(trace: np.ndarray, first_row: int, k: int | None = 2) -> list[dict]:
    """The first k rendered steps (all with k None) at or after the first control tick: world.step s and the start
    gap holding it."""
    flags = np.nan_to_num(trace[:, C["flags"]]).astype(int)
    rows = [i for i in np.where(flags & BIT["rendered"])[0] if i >= first_row]
    rows = rows if k is None else rows[:k]
    out = []
    for i in rows:
        world = float(trace[i, C["world_end"]] - trace[i, C["world_start"]])
        gap = float(trace[i + 1, C["S"]] - trace[i, C["S"]]) if i + 1 < len(trace) else float("nan")
        out.append({"row": int(i), "tick": float(trace[i, C["tick"]]), "world_s": world, "gap_s": gap})
    return out


def session_row(sess, arm: str) -> dict:
    row = {"name": sess.name, "arm": arm, "invalid": list(sess.invalid)}
    prep = next((r for r in sess.log if r.get("event") == "live_test_prep"), None)
    want_renders = {"P1": 1, "P2": 2}.get(arm, 0)
    want_wait = arm == "W"
    if arm in ("P1", "P2", "W"):
        if prep is None:
            row["invalid"].append("no live_test_prep event for a prep arm")
        else:
            if not prep.get("physics_unchanged"):
                row["invalid"].append("the prep moved the physics (time, step index or the runner's tick)")
            if len(prep.get("renders", [])) != want_renders or bool(prep.get("wait")) != want_wait:
                row["invalid"].append(f"the prep given is not arm {arm}'s: {len(prep.get('renders', []))} renders, "
                                      f"wait {bool(prep.get('wait'))}")
    elif prep is not None:
        row["invalid"].append(f"a live_test_prep event in arm {arm}")
    extra = sess.cond.get("EXTRA_ARGS", "")
    oracle = "--recognition-proposals oracle" in extra
    if (arm == "O") != oracle or ("--recognition far" not in extra):
        row["invalid"].append(f"arm {arm} ran with the proposals {'oracle' if oracle else 'learned'} (EXTRA_ARGS)")
    if prep is not None:
        row["prep"] = {"before": prep.get("before"), "after": prep.get("after"),
                       "renders_s": [b - a for a, b in prep.get("renders", [])], "renders_cpu_s": prep.get("renders_cpu_s"),
                       "wait_s": (prep["wait"][1] - prep["wait"][0]) if prep.get("wait") else None,
                       "wait_cpu_s": prep.get("wait_cpu_s"), "physics_unchanged": prep.get("physics_unchanged")}
    rec = (sess.result or {}).get("recognition") or {}
    row["semantic_labels"] = rec.get("labels")
    if sess.trace is None:
        return row
    first = sess.first_ctrl_row()
    row["render_steps"] = render_steps(sess.trace, first)
    S = sess.trace[:, C["S"]]
    t0 = S[first]
    later = [r for r in render_steps(sess.trace, first, None)[2:] if r["gap_s"] == r["gap_s"]
             and S[r["row"]] - t0 <= 10.0]
    row["later_render_over_0_25"] = [r for r in later if r["world_s"] > EDGE_S]
    gaps = np.diff(S[first:])
    within = (S[first:-1] - t0) <= 10.0
    row["gaps_over_0_25_first_10s"] = int(((gaps > EDGE_S) & within).sum())
    m6 = sess.m6_start()
    row["first_ok_from_first_tick_s"] = m6.get("first_ok_from_first_tick_s")
    row["first_ok_from_runner_s"] = m6.get("first_ok_from_runner_s")
    row["mission_outcomes"] = sess.mission_outcomes
    return row


MIN_SESSIONS = 2


def stall_state(vals: list[float]) -> str:
    """One arm's stall k over its valid sessions: "short" (too few), "split" (two disagreeing: a third is due),
    "gone" (every session, or with three or more the median, at most 0.25 s) or "present"."""
    if len(vals) < MIN_SESSIONS:
        return "short"
    below = [v <= EDGE_S for v in vals]
    if len(vals) == 2 and any(below) and not all(below):
        return "split"
    if len(vals) >= 3:
        return "gone" if float(np.median(vals)) <= EDGE_S else "present"
    return "gone" if all(below) else "present"


def arm_summary(rows: list[dict]) -> dict:
    by = {}
    for r in rows:
        if r["arm"] == "warm" or r["invalid"] or len(r.get("render_steps", [])) < 2:
            continue
        by.setdefault(r["arm"], []).append(r)
    out = {}
    p0 = by.get("P0", [])
    base = {k: (float(np.median([r["render_steps"][k]["world_s"] for r in p0])) if len(p0) >= MIN_SESSIONS else None)
            for k in (0, 1)}
    p0_labels = [r.get("semantic_labels") for r in p0]
    for arm, rs in sorted(by.items()):
        s = {"sessions": len(rs), "shifted": any(r.get("later_render_over_0_25") for r in rs)}
        for k in (0, 1):
            vals = [r["render_steps"][k]["world_s"] for r in rs]
            med = float(np.median(vals))
            state = stall_state(vals)
            s[f"stall{k + 1}"] = {"world_s": vals, "median_s": med, "state": state,
                                  # a moved stall is never credited (Codex 5.24 review P2-1)
                                  "gone": state == "gone" and not s["shifted"],
                                  "split": state == "split",
                                  "reduced": (base[k] is not None and arm != "P0" and state in ("gone", "present")
                                              and not s["shifted"] and med <= base[k] / 2)}
        if arm in ("P1", "P2"):
            preps = [v for r in rs for v in (r.get("prep") or {}).get("renders_s", [])]
            s["prep_renders_s"] = preps
            s["prep_did_the_work"] = bool(preps) and max(preps) > EDGE_S
        if arm == "O":
            ok = (len(p0_labels) >= MIN_SESSIONS and p0_labels[0] is not None
                  and all(lab == p0_labels[0] for lab in p0_labels)
                  and all(r.get("semantic_labels") == p0_labels[0] for r in rs))
            s["same_labels_as_P0"] = ok
        out[arm] = s
    out["_base"] = {"P0_sessions": len(p0), "P0_median_s": [base[0], base[1]],
                    "P0_reproduces": [b is not None and b > EDGE_S for b in (base[0], base[1])]}
    return out


def reading(summary: dict) -> list[str]:
    """The registration's rules (plan 5.24), each one that holds; undecided where the evidence is short."""
    arms = {k: v for k, v in summary.items() if not k.startswith("_")}
    base = summary.get("_base", {})
    reproduces = base.get("P0_reproduces", [False, False])

    def st(arm, k):
        return arms.get(arm, {}).get(f"stall{k}", {})

    def gone(arm, k):
        return st(arm, k).get("gone", False)

    def eased(arm, k):
        return gone(arm, k) or st(arm, k).get("reduced", False)
    out = []
    for arm, s in arms.items():
        for k in (1, 2):
            if st(arm, k).get("state") == "split":
                out.append(f"{arm} stall {k} split between its sessions: a third {arm} session is due")
            if st(arm, k).get("state") == "short" and arm != "P0":
                out.append(f"{arm} stall {k}: fewer than {MIN_SESSIONS} valid sessions -- undecided")
        if s.get("shifted") and arm != "P0":
            out.append(f"{arm}: a later render step in the first 10 s took over 0.25 s -- a stall moved, not gone")
    for k in (1, 2):  # each stall on its own (independent review of 5.24, item 2)
        if not reproduces[k - 1]:
            out.append(f"stall {k}: P0 does not reproduce it ({base.get('P0_sessions', 0)} valid P0 sessions) -- "
                       "undecided")
            continue
        if any(st(a, k).get("state") == "split" for a in arms):
            out.append(f"stall {k}: an arm is split -- undecided until its third session")
            continue
        prep = gone("P2", k) or (k == 1 and eased("P1", 1))
        wait = eased("W", k)
        if prep:
            out.append(f"stall {k}: first render-graph evaluation/preparation (a prep render removes or eases it)")
        if wait:
            out.append(f"stall {k}: time-based background work (the wait alone eases it)")
        if gone("O", k) and not (prep or wait or eased("P1", k) or eased("P2", k)):  # O alone (P2-3)
            if arms.get("O", {}).get("same_labels_as_P0"):
                out.append(f"stall {k}: the learned worker / IPC (only the oracle arm removes it)")
            else:
                out.append(f"stall {k}: the oracle arm removes it but its semantic labels are not P0's (or missing): "
                           "undecided")
    for arm in ("P1", "P2"):
        s = arms.get(arm)
        if s is not None and not s.get("prep_did_the_work") and not (eased(arm, 1) or eased(arm, 2)):
            out.append(f"{arm}: its prep renders each took at most 0.25 s and the stalls stayed -- world.render() did "
                       "not do the live renders' work; the render reading is undecided, not rejected")
    if not any(o.startswith("stall") for o in out):
        out.append("no arm removes or eases a stall: a Kit profiler arm is next (registered separately)")
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
