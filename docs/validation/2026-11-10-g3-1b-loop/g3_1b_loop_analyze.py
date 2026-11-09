"""Real-time plan 5.26 G3-1b stage check (5.20 pairing and stage pass), fixed before the results.

    PYTHONPATH=SNAP/src python g3_1b_loop_analyze.py OUT_DIR --list OUT_DIR.jsonl [--ps PS.jsonl] [--json out.json]

Sessions: g3_1b_loop_lists.py's (line_m far K f 1.0 alone, the grid switch off/on, pairs by the list's PAIR). The list
must hold exactly pairs 1..3 (or, extended, 1..6), each with one off and one on session. The arm is the list's
FORKLIFT_GRID_CLEAR_KERNEL, and the session's args.txt (r1c_session.sh writes the FORKLIFT_* environment there) must
show the same value, FORKLIFT_SHADOW_KERNEL=0, SEED=0 and no profiler variable; result.json's kernel mode is recorded
and must be the same in both arms of a pair. Validity is otherwise g2m_analyze.py's session reading (live log, tick
trace, probe exit, the monitor's GPU processes; with --ps the other-user CPU rule of the 5.22 F supplement).

Per session (AUTO: the status-line windows whose every command line is AUTO, g2m_analyze.py's auto_windows):
- the AUTO scan steps' after-step processing per sim s (step_end - world_end on scan-flagged rows; M3, 5.20's
  denominator) and the scan steps' W p50/p99/max;
- the AUTO rate: the windows' sim s over their wall s (mono), and ``ok``: the share of AUTO command lines with
  pacing_ok;
- the missions' outcomes and safety: the forbidden pocket contacts of every finished mission's record
  (missions/<id>/records.json, where the mission's state goes at its end) plus result.json's, and the obstacle layer's
  unpermitted entries (and direction entries), as r1c_analyze.py reads them; a missing key makes the count unknown.

Pairs (5.26: the timing of a pair whose run failed is not counted): a pair is comparable when both sessions are valid
and both missions are done. Degradation: the off arm's missions done and the on arm's not, or more forbidden contacts
or unpermitted entries with on than with off; a safety record missing in either arm leaves the pair undecided, which
does not pass. A pair with a mission not done is not counted for timing.

Stage pass (5.20; prediction from 1b-0: -14 ms/sim s, half = 0.007 s/sim s), with the registered 3 pairs:
- pass: every comparable pair's scan-step share falls (off - on > 0) and its AUTO rate rises (on - off > 0), every
  pair's fall is at least 0.007 s/sim s, three comparable pairs, and no pair where the off arm's missions were done and
  the on arm's were not;
- extend to 6 pairs (once): the pairs disagree in direction -- 5.20; a pair's direction is the joint one (its share
  falls and its rate rises, or not), so pairs improving on one measure each do not count as disagreeing;
- fail: otherwise (no pair improves, or an improving set misses half the prediction, or a mission degradation).
With 6 pairs: pass when the comparable pairs' mean fall is at least 0.007 s/sim s, their mean rate rise is positive
and no mission degradation; a fail otherwise (5.20's six pairs, read like 5.26's 1e rule: an operating decision, not
statistical certainty).
A comparable pair count below the registered number is reported as such (the check is then not passed).
Also reported, not judged: whether the mean fall reaches 0.025 s/sim s, the low end of -25 to -35 ms/sim s, which is
the G3 budget's target and not part of the stage pass (lead decision 2026-11-10, plan 5.26 amendment).
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
STAGE_TARGET_LOW = 0.025  # s per sim s: the stage's -25 ms/sim s, reported only


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
        want = {"SET": "far", "ARM": "K", "FACTOR": "1.0", "FORKLIFT_LIVE_CATCHUP_STEPS": "6", "SEED": "0",
                "FORKLIFT_GRID_CLEAR_KERNEL": "1" if self.arm == "on" else "0", "FORKLIFT_SHADOW_KERNEL": "0"}
        for k, v in want.items():
            if c.get(k) != v:
                self.invalid.append(f"{k}={c.get(k)}, registered {v}")
        for k in g2m.INSTR_ON:
            if c.get(k) is not None:
                self.invalid.append(f"{k} set in an INSTR off session")

    def auto_rate(self) -> dict:
        win = self.auto_windows()
        mono = [(a, b) for a, b, _, _ in win]
        sim = sum(x[2] for x in win)
        wall = sum(b - a for a, b in mono)
        auto_cmd = [r for r in self.cmd if r.get("mode") == "AUTO"]
        return {"windows": len(win), "sim_s": sim, "wall_s": wall, "rate": sim / wall if wall > 0 else None,
                "ok": float(np.mean([bool(r.get("pacing_ok")) for r in auto_cmd])) if auto_cmd else None}


def mission_ids(out: Path) -> list[str] | None:
    """The missions the probe reports (the confirm scenario's one, the cycle scenario's missions_run); None when the
    probe report is missing or a reported result has no mission id."""
    f = out / "probe.json"
    if not f.exists():
        return None
    j = json.loads(f.read_text())
    results = [(j.get("confirm") or {}).get("mission_result")] + [m.get("result") for m in j.get("missions_run") or []]
    results = [r for r in results if r is not None]
    if any(not r.get("mission_id") for r in results):
        return None  # a reported mission without its id: its record cannot be found (Codex round 4)
    return [r["mission_id"] for r in results]


def forbidden_contacts(out: Path, run: Path, result: dict | None) -> int | None:
    """Forbidden pocket contacts of the session: every mission the probe reports must have its record
    (missions/<id>/records.json, where the mission's state goes at its end) with a list there, and result.json a list
    for a mission still on; anything missing or null makes the count unknown (Codex review rounds 2-3)."""
    ids = mission_ids(out)
    if ids is None or not result or not isinstance(result.get("forbidden_pocket_contacts"), list):
        return None
    total = len(result["forbidden_pocket_contacts"])
    for mid in ids:
        f = run / "missions" / mid / "records.json"
        if not f.exists():
            return None
        state = (json.loads(f.read_text()).get("state")) or {}
        if not isinstance(state.get("forbidden_pocket_contacts"), list):
            return None
        total += len(state["forbidden_pocket_contacts"])
    return total


def analyse_session(s: Session) -> dict:
    row = {"name": s.name, "arm": s.arm, "pair": s.pair, "invalid": s.invalid, "ps": s.ps,
           "mission_outcomes": s.mission_outcomes, "mission_done": s.mission_done}
    if s.trace is None or s.invalid:
        return row
    m3 = s.m3_scan()
    res = s.result or {}
    ol = res.get("obstacle_layer") or {}
    row.update({"scan_step_rest_s_per_sim_s": m3.get("scan_step_rest_s_per_sim_s"),
                "scan_step_W_ms": m3.get("scan_step_W_ms"), "auto": s.auto_rate(), "M8": s.m8_report_only(),
                "kernels_mode": (res.get("kernels") or {}).get("mode"),
                "safety": {"forbidden_contacts": forbidden_contacts(s.out, s.run, res),
                           "unpermitted_entries": ol.get("unpermitted_entries"),
                           "unpermitted_dir_entries": ol.get("unpermitted_dir_entries")}})
    return row


def list_problems(entries: list[dict]) -> list[str]:
    """The list holds pairs 1..3 or 1..6, each with one off and one on session (Codex review P2-3)."""
    arms: dict[int, list[str]] = {}
    for e in entries:
        arms.setdefault(int(e["env"]["PAIR"]), []).append("on" if e["env"].get("FORKLIFT_GRID_CLEAR_KERNEL") == "1"
                                                           else "off")
    out = []
    if sorted(arms) not in (list(range(1, 4)), list(range(1, 7))):
        out.append(f"pairs {sorted(arms)}, registered 1..3 or 1..6")
    out += [f"pair {p}: arms {sorted(a)}" for p, a in arms.items() if sorted(a) != ["off", "on"]]
    return out


def verdict(rows: list[dict], registered_pairs: int = 3) -> dict:
    by_pair: dict[int, dict] = {}
    for r in rows:
        by_pair.setdefault(r["pair"], {})[r["arm"]] = r
    pairs, degraded, left_out, undecided = [], [], [], []
    for p in sorted(by_pair):
        off, on = by_pair[p].get("off"), by_pair[p].get("on")
        if off is None or on is None or off["invalid"] or on["invalid"]:
            left_out.append({"pair": p, "why": "a session missing or invalid"})
            continue
        if off.get("kernels_mode") is None or off.get("kernels_mode") != on.get("kernels_mode"):
            left_out.append({"pair": p, "why": f"kernel modes differ: {off.get('kernels_mode')} / {on.get('kernels_mode')}"})
            continue
        so, sn = off.get("safety") or {}, on.get("safety") or {}
        keys = ("forbidden_contacts", "unpermitted_entries", "unpermitted_dir_entries")
        if any(so.get(k) is None or sn.get(k) is None for k in keys):
            undecided.append(p)
        elif any(sn[k] > so[k] for k in keys):
            degraded.append(p)
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
    out = {"pairs": pairs, "left_out": left_out, "mission_degraded_pairs": sorted(set(degraded)),
           "safety_undecided_pairs": undecided, "registered_pairs": registered_pairs,
           "half_prediction_s_per_sim_s": HALF_PREDICTION}
    if pairs:
        mean_fall_ = float(np.mean([x["fall_s_per_sim_s"] for x in pairs]))
        out["reported_mean_fall_s_per_sim_s"] = mean_fall_
        out["reported_reaches_stage_target_low"] = mean_fall_ >= STAGE_TARGET_LOW
    if registered_pairs == 3:
        improving = [x["improves"] for x in pairs]
        if degraded or undecided:
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
        if degraded or undecided:
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
    problems = list_problems(entries)
    if problems:
        print(json.dumps({"list_not_registered": problems}))
        return 2
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
