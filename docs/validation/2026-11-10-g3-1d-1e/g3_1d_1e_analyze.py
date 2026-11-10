"""Real-time plan 5.26: the G3-1d stage check and the 1e judgement run, fixed before their results. Both reuse
g3_1b_loop_analyze.py's session reading (validity, AUTO scan-step share, AUTO rate, missions and safety, the kernel
mode) with the arms of g3_1d_1e_lists.py (the list's ARM_NAME), checked against the session's records: args.txt's
FORKLIFT_GRID_CLEAR_KERNEL / FORKLIFT_SHADOW_KERNEL and EXTRA_ARGS' --lidar-cast, and result.json's lidar_cast mode
(written only for a mode other than loop).

    PYTHONPATH=SNAP/src python g3_1d_1e_analyze.py {1d,1e} OUT_DIR --list OUT_DIR.jsonl [--ps PS.jsonl] [--json out]

1d (stage check, plan 5.26 amendment): loop is "off", fast "on"; g3_1b_loop_analyze.verdict with the per-pair
threshold half of 1d's registered prediction, -11.5 ms/sim s from the bench (#388): 0.00575 s/sim s. Pass / extend to
6 pairs / fail / not enough comparable pairs as for 1b.

1e (judgement run, plan 5.26 "1e 판단 규칙"): baseline "off", candidate "on". Pairs as g3_1b_loop_analyze builds them
(both valid, kernel modes recorded and equal, both missions done for the timing; degradation = the baseline's missions
done and the candidate's not, or more forbidden contacts or unpermitted entries with the candidate; unknown safety
records leave the pair undecided). Each comparable pair's saving = baseline share - candidate share (s/sim s):
- 3 pairs: every saving >= 0.100 -> "no 1e"; every saving < 0.100 -> "plan 1e"; on both sides -> "extend to 6
  pairs" (once);
- 6 pairs: the mean saving decides (>= 0.100 "no 1e", else "plan 1e"), the straddle recorded;
- the 1e target (remaining gap) = 0.100 - the mean saving of the pairs used, unrounded;
- a pair whose timing cannot be compared (a mission not done) or a degradation -> "hold: to the lead" (5.26: mission and
  safety failures are kept as results, not supplemented, and a pair that cannot be compared holds the 1e branch), even
  when other sessions need a supplement; otherwise infra failures (an invalid session) -> "supplement" (rerun those
  sessions, listed in supplement_sessions either way).
The list must hold each arm's registered switches (1d: grid 1, shadow 1, the arm's LiDAR mode; 1e: the baseline all
off with loop, the candidate grid 1, shadow 1 and one LiDAR mode across the list).
Also reported (not judged): each arm's AUTO ok and rate, the scan-step W p99 and max, the AUTO sim s (null for a
session without AUTO scan steps). Not produced here, as registered in plan 5.26: the recovery after the W tail and the
AUTO windows' mission-phase composition (reported as not measured), and the per-stage predicted-against-realized table
(1b -14 / 14.0 and 1c -30 / 36.1 from their stage checks, 1d from its check, the combined run's saving apart).
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np

_spec = importlib.util.spec_from_file_location(
    "g3_1b_loop_analyze", Path(__file__).resolve().parents[1] / "2026-11-10-g3-1b-loop" / "g3_1b_loop_analyze.py")
base = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(base)
g2m = base.g2m
D_HALF = 0.00575  # s per sim s: half of 1d's -11.5 ms/sim s (bench #388)
BUDGET = 0.100  # s per sim s: the 1e saving budget (5.26)
OFF_ON = {"1d": ("loop", "fast"), "1e": ("base", "cand")}


class Session(base.Session):
    def __init__(self, out: Path, entry: dict, kind: str, ps=None):
        self.kind = kind
        self.want = {k: entry["env"][k] for k in ("FORKLIFT_GRID_CLEAR_KERNEL", "FORKLIFT_SHADOW_KERNEL", "LIDAR_CAST")}
        name = entry["env"]["ARM_NAME"]
        self.arm = "off" if name == OFF_ON[kind][0] else "on"
        self.pair = int(entry["env"]["PAIR"])
        g2m.Session.__init__(self, out, entry, ps=ps)
        mode = ((self.result or {}).get("lidar_cast") or {}).get("mode", "loop") if self.result else None
        if mode != self.want["LIDAR_CAST"]:
            self.invalid.append(f"result.json lidar_cast mode {mode}, registered {self.want['LIDAR_CAST']}")

    def _check_conditions(self):
        c = self.cond
        for k in c:
            if k.startswith(g2m.FORBIDDEN):
                self.invalid.append(f"{k} set (dumps and the dual run stay out of the check)")
        want = {"SET": "far", "ARM": "K", "FACTOR": "1.0", "FORKLIFT_LIVE_CATCHUP_STEPS": "6", "SEED": "0",
                "FORKLIFT_GRID_CLEAR_KERNEL": self.want["FORKLIFT_GRID_CLEAR_KERNEL"],
                "FORKLIFT_SHADOW_KERNEL": self.want["FORKLIFT_SHADOW_KERNEL"]}
        for k, v in want.items():
            if c.get(k) != v:
                self.invalid.append(f"{k}={c.get(k)}, registered {v}")
        extra = c.get("EXTRA_ARGS", "").replace("\\ ", " ")
        if f"--lidar-cast {self.want['LIDAR_CAST']}" not in extra:
            self.invalid.append(f"EXTRA_ARGS without --lidar-cast {self.want['LIDAR_CAST']}")
        for k in g2m.INSTR_ON:
            if c.get(k) is not None:
                self.invalid.append(f"{k} set in an INSTR off session")


def canonical(kind: str, arm_name: str, lidar: str | None) -> dict:
    """Each arm's registered switches (plan 5.26 registration of the 1d check and the 1e run)."""
    if kind == "1d":
        return {"FORKLIFT_GRID_CLEAR_KERNEL": "1", "FORKLIFT_SHADOW_KERNEL": "1", "LIDAR_CAST": arm_name}
    if arm_name == "base":
        return {"FORKLIFT_GRID_CLEAR_KERNEL": "0", "FORKLIFT_SHADOW_KERNEL": "0", "LIDAR_CAST": "loop"}
    return {"FORKLIFT_GRID_CLEAR_KERNEL": "1", "FORKLIFT_SHADOW_KERNEL": "1", "LIDAR_CAST": lidar}


def list_problems(kind: str, entries: list[dict]) -> list[str]:
    names = OFF_ON[kind]
    bad = [e.get("name") for e in entries if e["env"].get("ARM_NAME") not in names]
    if bad:
        return [f"arm names not {names} in {bad}"]
    out = []
    lidar = None
    if kind == "1e":  # one candidate LiDAR mode across the list (Codex review P2-2: no fast/loop mix)
        modes = {e["env"].get("LIDAR_CAST") for e in entries if e["env"]["ARM_NAME"] == "cand"}
        if len(modes) != 1 or not modes <= {"fast", "loop"}:
            out.append(f"candidate LiDAR modes {sorted(map(str, modes))}, registered one of fast or loop")
        lidar = next(iter(modes)) if len(modes) == 1 else None
    for e in entries:
        want = canonical(kind, e["env"]["ARM_NAME"], lidar)
        got = {k: e["env"].get(k) for k in want}
        if got != want:
            out.append(f"{e.get('name')}: switches {got}, registered {want}")
    return out + base.list_problems([{"env": {"PAIR": e["env"]["PAIR"], "FORKLIFT_GRID_CLEAR_KERNEL":
                                        "1" if e["env"]["ARM_NAME"] == names[1] else "0"}} for e in entries])


def verdict_1d(rows: list[dict], registered_pairs: int) -> dict:
    keep = base.HALF_PREDICTION
    base.HALF_PREDICTION = D_HALF
    try:
        return base.verdict(rows, registered_pairs)
    finally:
        base.HALF_PREDICTION = keep


def verdict_1e(rows: list[dict], registered_pairs: int) -> dict:
    v = base.verdict(rows, registered_pairs)  # the pairs, exclusions and degradations as 1b reads them
    pairs = [{"pair": x["pair"], "saving_s_per_sim_s": x["fall_s_per_sim_s"], "rate_rise": x["rate_rise"]}
             for x in v["pairs"]]
    invalid = [r["name"] for r in rows if r["invalid"]]
    not_comparable = [x["pair"] for x in v["left_out"] if not x["why"].startswith("a session missing or invalid")]
    out = {"pairs": pairs, "left_out": v["left_out"], "degraded_pairs": v["mission_degraded_pairs"],
           "safety_undecided_pairs": v["safety_undecided_pairs"], "not_comparable_pairs": not_comparable,
           "supplement_sessions": invalid, "registered_pairs": registered_pairs, "budget_s_per_sim_s": BUDGET}
    # A pair that cannot be compared or that degraded holds the branch whatever else needs a supplement (Codex P2-3);
    # infra-invalid sessions are listed for their supplement either way.
    if v["mission_degraded_pairs"] or v["safety_undecided_pairs"] or not_comparable:
        out["result"] = "hold: to the lead"
        return out
    if invalid:
        out["result"] = "supplement"
        return out
    if len(pairs) < registered_pairs:
        out["result"] = "hold: to the lead"
        return out
    s = [x["saving_s_per_sim_s"] for x in pairs]
    mean = float(np.mean(s))
    out.update({"mean_saving_s_per_sim_s": mean, "straddle": min(s) < BUDGET <= max(s)})
    if registered_pairs == 3:
        if all(x >= BUDGET for x in s):
            out["result"] = "no 1e"
        elif all(x < BUDGET for x in s):
            out["result"] = "plan 1e"
        else:
            out["result"] = "extend to 6 pairs"
    else:
        out["result"] = "no 1e" if mean >= BUDGET else "plan 1e"
    if out["result"] == "plan 1e":
        out["1e_target_s_per_sim_s"] = BUDGET - mean
    return out


def arm_summary(rows: list[dict]) -> dict:
    out = {}
    for arm in ("off", "on"):
        rr = [r for r in rows if r["arm"] == arm and not r["invalid"] and "auto" in r]
        if rr:  # a session that ended before any AUTO scan has no W: kept as null (Codex review P2-1)
            out[arm] = {"ok": [r["auto"].get("ok") for r in rr], "rate": [r["auto"].get("rate") for r in rr],
                        "W_p99_ms": [(r.get("scan_step_W_ms") or {}).get("p99") for r in rr],
                        "W_max_ms": [(r.get("scan_step_W_ms") or {}).get("max") for r in rr],
                        "auto_sim_s": [r["auto"].get("sim_s") for r in rr]}
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("kind", choices=("1d", "1e"))
    ap.add_argument("out", type=Path)
    ap.add_argument("--list", type=Path, required=True)
    ap.add_argument("--ps", type=Path, default=None)
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args()
    ps = g2m.read_ps(a.ps.read_text()) if a.ps else None
    entries = [json.loads(x) for x in a.list.read_text().splitlines() if x.strip()]
    problems = list_problems(a.kind, entries)
    if problems:
        print(json.dumps({"list_not_registered": problems}))
        return 2
    rows = [base.analyse_session(Session(a.out / e["name"], e, a.kind, ps=ps)) for e in entries]
    n = len({e["env"]["PAIR"] for e in entries})
    verdict = verdict_1d(rows, n) if a.kind == "1d" else verdict_1e(rows, n)
    out = {"kind": a.kind, "sessions": rows, "verdict": verdict, "arms": arm_summary(rows)}
    text = json.dumps(out, indent=1, default=str)
    if a.json:
        a.json.write_text(text)
    for r in rows:
        print(json.dumps({k: r.get(k) for k in ("name", "arm", "invalid", "mission_done",
                                                  "scan_step_rest_s_per_sim_s")}, default=str))
    print(json.dumps(verdict, indent=1, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
