"""T2 judge of the insertion lateral drift plan (docs/plans/2026-10-27-insertion-lateral-drift.md, "T2 사전 등록").

Reads ``<batch>/t2d_<arm>_<scene>_s<seed>/run/result.json`` (arms b0, c3, c1) and applies the rules fixed before the
results: outcome normalisation, per-metric sets, the sign test, the verdicts 1-5. C3 is the primary test, C1
exploratory. Records beside it (not judged): C3's second-capture outcome distribution and, when ``frames.jsonl`` is
there, the fork tip's lateral offset and relative heading to the pallet at the insertion start.

    python -m tools.t2_drift_judge <batch dir> --out t2_judge.json
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from collections import Counter
from pathlib import Path

NOMINAL_GAP_MM = 86.25  # EPAL 6, fork spacing 0.3725 m: the blade-to-block gap each side
TIP_X_M = 0.95  # fork tip ahead of base_link (measured chassis)
ARMS = ("b0", "c3", "c1")
REPLACEABLE = ("Camera did not produce RGB", "plan_worker_dead", "slam_prewarm_timeout", "slam_map_")
EXPECTED_REFUSAL = ("not_pickable:", "bottom_deck_refused")
NAME = re.compile(r"^t2d_(b0|c3|c1)_(.+)_s(\d+)$")


def outcome(result: dict | None) -> str:
    """complete / expected_refusal / mission_failure / infrastructure (replaceable)."""
    if result is None:
        return "infrastructure"  # no result: the run did not get to write one (crash, Slurm)
    if result.get("success") and result.get("phase") == "complete":
        return "complete"
    # A bottom-deck refusal returns without an exception: no failure_reason, the status in mission_result / phase.
    if "bottom_deck_refused" in (result.get("mission_result"), result.get("phase")):
        return "expected_refusal"
    reason = str(result.get("failure_reason") or "")
    if any(reason.startswith(p) for p in REPLACEABLE):
        return "infrastructure"
    if any(reason.startswith(p) for p in EXPECTED_REFUSAL):
        return "expected_refusal"
    return "mission_failure"


def loss_mm(result: dict | None) -> float | None:
    """86.25 - min(left, right) over the insert phase, mm; None when the run did not insert."""
    lat = ((result or {}).get("lateral_clearance_min_m") or {}).get("insert") or {}
    gaps = [g for g in (lat.get("left"), lat.get("right")) if g is not None]
    return None if not gaps else NOMINAL_GAP_MM - 1000.0 * min(gaps)


def cycle_s(result: dict | None) -> float | None:
    for t in (result or {}).get("transitions") or []:
        if t.get("to") == "home_settle":
            return float(t["time_s"])
    return None


def forbidden(result: dict | None) -> int:
    return 1 if (result or {}).get("forbidden_pocket_contacts") else 0


def p95_higher(values) -> float | None:
    if not values:
        return None
    v = sorted(values)
    return v[min(len(v) - 1, math.ceil(0.95 * len(v)) - 1)]


def sign_test_p(better: int, worse: int) -> float:
    """Two-sided exact sign test, ties excluded."""
    n = better + worse
    if n == 0:
        return 1.0
    k = max(better, worse)
    tail = sum(math.comb(n, i) for i in range(k, n + 1)) / 2 ** n
    return min(1.0, 2 * tail)


def _yaw(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def insert_start(run_dir: Path, result: dict | None) -> dict | None:
    """Truth fork-tip lateral (mm) and relative heading (deg) in the pallet frame at the approach -> insert switch."""
    t0 = next((t["time_s"] for t in (result or {}).get("transitions") or [] if t.get("to") == "insert"), None)
    frames = run_dir / "frames.jsonl"
    if t0 is None or not frames.exists():
        return None
    with frames.open() as f:
        for line in f:
            fr = json.loads(line)
            if fr["simulation_time_s"] >= t0:
                py, by = _yaw(fr["pallet_orientation_wxyz"]), _yaw(fr["base_orientation_wxyz"])
                bx, byy = fr["base_position_m"][:2]
                px, pyy = fr["pallet_position_m"][:2]
                tx, ty = bx + TIP_X_M * math.cos(by) - px, byy + TIP_X_M * math.sin(by) - pyy
                lat = -math.sin(py) * tx + math.cos(py) * ty
                d = math.atan2(math.sin(by - py), math.cos(by - py))
                return {"tip_lateral_mm": 1000 * lat, "heading_deg": math.degrees(d)}
    return None


def load(batch: Path) -> dict:
    runs = {}
    for d in sorted(batch.iterdir()):
        m = NAME.match(d.name)
        if not m or not d.is_dir():
            continue
        arm, scene, seed = m.group(1), m.group(2), int(m.group(3))
        path = d / "run" / "result.json"
        try:
            result = json.loads(path.read_text()) if path.exists() else None
        except (OSError, ValueError):
            result = None
        runs[(arm, scene, seed)] = {"result": result, "dir": d / "run"}
    return runs


def _median(values):
    v = sorted(values)
    n = len(v)
    return v[n // 2] if n % 2 else (v[n // 2 - 1] + v[n // 2]) / 2


def judge_arm(runs: dict, arm: str) -> dict:
    keys = sorted({(s, n) for a, s, n in runs if a == arm} & {(s, n) for a, s, n in runs if a == "b0"})
    # A pair with an infrastructure failure on either side is replaced as a pair: neither run enters any metric.
    kept = [(s, n) for s, n in keys
            if outcome(runs[(arm, s, n)]["result"]) != "infrastructure"
            and outcome(runs[("b0", s, n)]["result"]) != "infrastructure"]
    better = worse = ties = both_missing = 0
    joint_times = []
    for scene, seed in kept:
        ra, rb = runs[(arm, scene, seed)]["result"], runs[("b0", scene, seed)]["result"]
        la, lb = loss_mm(ra), loss_mm(rb)
        if la is None and lb is None:
            both_missing += 1
        elif la is None:
            worse += 1
        elif lb is None:
            better += 1
        elif la < lb:
            better += 1
        elif la > lb:
            worse += 1
        else:
            ties += 1
        ca, cb = cycle_s(ra) if outcome(ra) == "complete" else None, cycle_s(rb) if outcome(rb) == "complete" else None
        if ca is not None and cb is not None:
            joint_times.append((ca, cb))

    def arm_stats(a):
        rs = [runs[(a, s, n)]["result"] for s, n in kept]
        losses = [l for l in (loss_mm(r) for r in rs) if l is not None]
        return {"runs": len(rs), "complete": sum(outcome(r) == "complete" for r in rs),
                "forbidden": sum(forbidden(r) for r in rs), "inserted": len(losses),
                "not_inserted": len(rs) - len(losses), "p95_mm": p95_higher(losses),
                "max_mm": max(losses) if losses else None,
                "outcomes": dict(Counter(outcome(r) for r in rs))}

    a, b = arm_stats(arm), arm_stats("b0")
    valid_pairs = better + worse + ties
    p = sign_test_p(better, worse)
    out = {"arm": a, "b0": b, "pairs": {"better": better, "worse": worse, "ties": ties,
                                         "both_not_inserted": both_missing, "valid": valid_pairs, "sign_p": p},
           "joint_complete_pairs": len(joint_times)}
    if valid_pairs < 12:
        out["verdict"] = "판정 불가(유효 짝 12 미만)"
        return out
    c1 = a["forbidden"] == 0
    c2 = a["complete"] >= b["complete"] - 1
    c3 = better > worse and p < 0.05
    c4 = (a["p95_mm"] is not None and b["p95_mm"] is not None and a["p95_mm"] <= b["p95_mm"] - 10.0
          and a["max_mm"] <= b["max_mm"] - 10.0)
    if len(joint_times) >= 8:
        # The registered statistic: the arm's median over the jointly completed pairs over B0's median there.
        ratio = _median([x for x, _ in joint_times]) / _median([y for _, y in joint_times])
        out["cycle_median_ratio"] = ratio
        c5 = ratio <= 1.10
    else:
        c5 = None
    out["criteria"] = {"1_no_contact": c1, "2_completions": c2, "3_sign_test": c3, "4_tail_10mm": c4, "5_cycle": c5}
    if not (c1 and c2):
        out["verdict"] = "더 나쁨"
    elif not (c3 and c4):
        out["verdict"] = "차이 확인 안 됨"
    elif c5 is None:
        out["verdict"] = "나음(시간 지표 판정 불가 — 공동 완주 짝 8 미만)"
    elif not c5:
        out["verdict"] = "나아졌으나 시간 비용 초과"
    else:
        out["verdict"] = "나음"
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("batch", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    runs = load(args.batch)
    infra = sorted(f"{a}_{s}_s{n}" for (a, s, n), v in runs.items() if outcome(v["result"]) == "infrastructure")
    second = Counter(((v["result"] or {}).get("near_capture") or {}).get("second", {}).get("status", "absent")
                     for (a, s, n), v in runs.items() if a == "c3" and v["result"] is not None)
    starts = {f"{a}_{s}_s{n}": insert_start(v["dir"], v["result"]) for (a, s, n), v in runs.items()}
    report = {"runs": len(runs), "infrastructure_to_replace": infra,
              "c3": judge_arm(runs, "c3"), "c1_exploratory": judge_arm(runs, "c1"),
              "c3_second_capture": dict(second), "insert_start": starts}
    args.out.write_text(json.dumps(report, indent=1, allow_nan=False))
    print(json.dumps({k: report[k] for k in ("runs", "infrastructure_to_replace", "c3_second_capture")}))
    for k in ("c3", "c1_exploratory"):
        print(k, report[k].get("verdict"), json.dumps(report[k].get("pairs")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
