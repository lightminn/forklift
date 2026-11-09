"""Real-time plan 5.26 G3-1b stage check (5.20 pairing and stage pass), fixed before the results.

    PYTHONPATH=SNAP/src python g3_1b_loop_analyze.py OUT_DIR --list OUT_DIR.jsonl [--ps PS.jsonl] [--json out.json]

Sessions: g3_1b_loop_lists.py's (line_m far K f 1.0 alone, the grid switch off/on, pairs by the list's PAIR). The arm
is the list's FORKLIFT_GRID_CLEAR_KERNEL (the bundle runner hands the list's env to the session; the run records no
switch of its own). Validity is g2m_analyze.py's session reading (live log, tick trace, probe exit, the monitor's GPU
processes; with --ps the other-user CPU rule of the 5.22 F supplement) with this run's registered conditions.

Per session (AUTO: the status-line windows whose every command line is AUTO, g2m_analyze.py's auto_windows):
- the AUTO scan steps' after-step processing per sim s (step_end - world_end on scan-flagged rows; M3, 5.20's
  denominator) and the scan steps' W p50/p99/max;
- the AUTO rate: the windows' sim s over their wall s (mono), and ``ok``: the share of AUTO command lines with
  pacing_ok;
- the missions' outcomes.

Pairs (5.26: the timing of a pair whose run failed is not counted): a pair is comparable when both sessions are valid
and both missions are done. A pair with a mission not done in one arm is reported (mission or safety, kept in the
degradation line) and not counted for timing.

Stage pass (5.20; prediction from 1b-0: -14 ms/sim s, half = 0.007 s/sim s), with the registered 3 pairs:
- pass: every comparable pair's scan-step share falls (off - on > 0) and its AUTO rate rises (on - off > 0), every
  pair's fall is at least 0.007 s/sim s, three comparable pairs, and no pair where the off arm's missions were done and
  the on arm's were not;
- extend to 6 pairs (once): the pairs disagree in direction (some fall and rise, some not) -- 5.20;
- fail: otherwise (no pair improves, or an improving set misses half the prediction, or a mission degradation).
With 6 pairs: pass when the comparable pairs' mean fall is at least 0.007 s/sim s, their mean rate rise is positive
and no mission degradation; a fail otherwise (5.20's six pairs, read like 5.26's 1e rule: an operating decision, not
statistical certainty).
A comparable pair count below the registered number is reported as such (the check is then not passed).
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
HALF_PREDICTION = 0.007  # s per sim s: half of 1b-0's -14 ms/sim s


class Session(g2m.Session):
    def __init__(self, out: Path, entry: dict, ps=None):
        self.arm = "on" if entry["env"].get("FORKLIFT_GRID_CLEAR_KERNEL") == "1" else "off"
        self.pair = int(entry["env"]["PAIR"])
        super().__init__(out, entry, ps=ps)

    def _check_conditions(self):
        c = self.cond
        for k in c:
            if k.startswith(g2m.FORBIDDEN):
                self.invalid.append(f"{k} set (dumps and the dual run stay out of the check)")
        want = {"SET": "far", "ARM": "K", "FACTOR": "1.0", "FORKLIFT_LIVE_CATCHUP_STEPS": "6"}
        for k, v in want.items():
            if c.get(k) != v:
                self.invalid.append(f"{k}={c.get(k)}, registered {v}")

    def auto_rate(self) -> dict:
        win = self.auto_windows()
        mono = [(a, b) for a, b, _, _ in win]
        sim = sum(x[2] for x in win)
        wall = sum(b - a for a, b in mono)
        auto_cmd = [r for r in self.cmd if r.get("mode") == "AUTO"]
        return {"windows": len(win), "sim_s": sim, "wall_s": wall, "rate": sim / wall if wall > 0 else None,
                "ok": float(np.mean([bool(r.get("pacing_ok")) for r in auto_cmd])) if auto_cmd else None}


def analyse_session(s: Session) -> dict:
    row = {"name": s.name, "arm": s.arm, "pair": s.pair, "invalid": s.invalid, "ps": s.ps,
           "mission_outcomes": s.mission_outcomes, "mission_done": s.mission_done}
    if s.trace is None or s.invalid:
        return row
    m3 = s.m3_scan()
    row.update({"scan_step_rest_s_per_sim_s": m3.get("scan_step_rest_s_per_sim_s"),
                "scan_step_W_ms": m3.get("scan_step_W_ms"), "auto": s.auto_rate(), "M8": s.m8_report_only()})
    return row


def verdict(rows: list[dict], registered_pairs: int = 3) -> dict:
    by_pair: dict[int, dict] = {}
    for r in rows:
        by_pair.setdefault(r["pair"], {})[r["arm"]] = r
    pairs, degraded, left_out = [], [], []
    for p in sorted(by_pair):
        off, on = by_pair[p].get("off"), by_pair[p].get("on")
        if off is None or on is None or off["invalid"] or on["invalid"]:
            left_out.append({"pair": p, "why": "a session missing or invalid"})
            continue
        if off["mission_done"] and not on["mission_done"]:
            degraded.append(p)
        if not (off["mission_done"] and on["mission_done"]):
            left_out.append({"pair": p, "why": "a mission not done: timing not counted"})
            continue
        fall = off["scan_step_rest_s_per_sim_s"] - on["scan_step_rest_s_per_sim_s"]
        ro, rn = off["auto"]["rate"], on["auto"]["rate"]
        rise = None if ro is None or rn is None else rn - ro
        pairs.append({"pair": p, "fall_s_per_sim_s": fall, "rate_rise": rise,
                      "improves": fall > 0 and rise is not None and rise > 0,
                      "half_prediction": fall >= HALF_PREDICTION})
    n = len(pairs)
    out = {"pairs": pairs, "left_out": left_out, "mission_degraded_pairs": degraded, "registered_pairs":
           registered_pairs, "half_prediction_s_per_sim_s": HALF_PREDICTION}
    if registered_pairs == 3:
        improving = [x["improves"] for x in pairs]
        if degraded:
            out["result"] = "fail"
        elif n < 3:
            out["result"] = "not enough comparable pairs"
        elif all(improving) and all(x["half_prediction"] for x in pairs):
            out["result"] = "pass"
        elif any(improving) and not all(improving):
            out["result"] = "extend to 6 pairs"
        else:
            out["result"] = "fail"
    else:
        if degraded:
            out["result"] = "fail"
        elif n < registered_pairs:
            out["result"] = "not enough comparable pairs"
        else:
            mean_fall = float(np.mean([x["fall_s_per_sim_s"] for x in pairs]))
            rises = [x["rate_rise"] for x in pairs]
            mean_rise = float(np.mean(rises)) if None not in rises else None
            out.update({"mean_fall_s_per_sim_s": mean_fall, "mean_rate_rise": mean_rise})
            out["result"] = "pass" if mean_fall >= HALF_PREDICTION and mean_rise is not None and mean_rise > 0 \
                else "fail"
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=Path)
    ap.add_argument("--list", type=Path, required=True, help="the run list (6 or, extended, 12 lines)")
    ap.add_argument("--ps", type=Path, default=None)
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args()
    ps = g2m.read_ps(a.ps.read_text()) if a.ps else None
    entries = [json.loads(x) for x in a.list.read_text().splitlines() if x.strip()]
    rows = [analyse_session(Session(a.out / e["name"], e, ps=ps)) for e in entries]
    registered = len({e["env"]["PAIR"] for e in entries})
    out = {"sessions": rows, "verdict": verdict(rows, registered)}
    text = json.dumps(out, indent=1, default=str)
    if a.json:
        a.json.write_text(text)
    for r in rows:
        print(json.dumps({k: r.get(k) for k in ("name", "invalid", "mission_done", "scan_step_rest_s_per_sim_s")},
                         default=str))
    print(json.dumps(out["verdict"], indent=1, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
