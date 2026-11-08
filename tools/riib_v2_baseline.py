"""The v2 concurrency baseline's verdict (docs/plans/2026-11-08-live-riib-defaults-v2.md 2, 2.1): per session F and R on
the registered interval with the missing-data rules, the catch-up setting check, the workload; per arm the frozen
stage-1 F1/R1 and held set; the mechanical stage-2 targets or the reasons to go to the lead.

    python tools/riib_v2_baseline.py OUT --json STAGE1.json

OUT holds plan.json and s/<run> (args.txt, probe.json, live/epoch_1/run/{live_log.jsonl,tick_trace.npz,result.json})."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import ct_survey  # noqa: E402

from forklift_core.live.spool import read_live_log  # noqa: E402

F_RATIO, R_ADD, F1_STOP = 0.97, 0.03, 0.27  # registration 2 (lead's policy budgets) and its stop line
MISSING_SHARE, MISSING_GAP_S, COMMON_MIN_S = 0.01, 5.0, 15 * 60.0


def interval_stats(steps: dict, a: float, b: float) -> dict:
    """F, R and the missing rules on [a, b] from the per-step records {step: (mono, tick_t, pacing_ok)}."""
    keys = sorted(k for k, v in steps.items() if a <= v[0] <= b)
    if len(keys) < 2:
        return {"decidable": False, "why": "fewer than two steps in the interval"}
    k0, k1 = keys[0], keys[-1]
    (m0, t0, _), (m1, t1, _) = steps[k0], steps[k1]
    false_wall = wall = missing_wall = max_gap = 0.0
    for k, nxt in zip(keys, keys[1:], strict=False):
        dm = steps[nxt][0] - steps[k][0]
        if nxt == k + 1:
            wall += dm
            false_wall += dm * (not steps[k][2])
        else:
            missing_wall += dm
            max_gap = max(max_gap, dm)
    expected = k1 - k0 + 1
    missing = expected - len(keys)
    out = {"F": (t1 - t0) / (m1 - m0) if m1 > m0 else math.nan, "R": false_wall / wall if wall > 0 else math.nan,
           "wall_s": m1 - m0, "missing_steps": missing, "missing_share": missing / expected,
           "missing_wall_s": missing_wall, "max_gap_s": max_gap}
    why = []
    if out["missing_share"] > MISSING_SHARE:
        why.append("missing steps over 1 %")
    if (m1 - m0) > 0 and missing_wall / (m1 - m0) > MISSING_SHARE:
        why.append("missing wall over 1 %")
    if max_gap > MISSING_GAP_S:
        why.append("a missing gap over 5 s")
    out["decidable"], out["why"] = not why, "; ".join(why)
    return out


def held(sessions: list, f1: float, r1: float):
    """True / False by the registered rule (every session F >= 0.97 F1 and R <= R1 + 0.03); None when a session is
    not eligible or not decidable (the concurrency is then undecidable for the arm)."""
    if any(not s.get("eligible") or not s.get("decidable", True) for s in sessions):
        return None
    return all(s["F"] >= F_RATIO * f1 and s["R"] <= r1 + R_ADD for s in sessions)


def setting_ok(catchup: int, gaps, spacing: float) -> bool:
    """Catch-up 0 never starts a step more than 1 ms short of the spacing; catch-up 6 does after late steps."""
    short = sum(1 for g in gaps if g < spacing - 0.001)
    return short == 0 if catchup == 0 else short > 0


def stage2_targets(held6: dict):
    """The catch-up 6 arm's largest held N and its A/B N (2 if 2 held, else 1), once if equal; None if nothing held."""
    held_n = [n for n, h in held6.items() if h]
    if not held_n:
        return None
    big = max(held_n)
    ab = 2 if held6.get(2) else 1
    return [big] if big == ab else [big, ab]


def stage2_blockers(arms: dict) -> list:
    """Reasons to send stage 1 to the lead without a stage 2."""
    out = []
    for k, arm in sorted(arms.items()):
        if any(h is None for h in arm["held"].values()):
            out.append(f"catch-up {k}: an undecidable concurrency")
        if not any(arm["held"].values()):
            out.append(f"catch-up {k}: no held N")
        if not (arm["F1"] >= F1_STOP):
            out.append(f"catch-up {k}: F1 {arm['F1']} < {F1_STOP}")
    return out


# --- records ------------------------------------------------------------------------------------------------------------
def session_record(d: Path, factor: float) -> dict:
    out = {"name": d.name}
    args = dict(line.split("=", 1) for line in (d / "args.txt").read_text().splitlines()
                if "=" in line and not line.startswith(("PROBE=", "EXTRA_ARGS=")))
    out["catchup_args"] = args.get("FORKLIFT_LIVE_CATCHUP_STEPS")
    out["phase_args"] = args.get("FORKLIFT_LIVE_CAMERA_PHASE")
    runs = sorted(d.glob("live/epoch_*/run"))
    if len(runs) != 1:
        return {**out, "eligible": False, "decidable": False, "why": "not exactly one epoch"}
    run = runs[0]
    rows = ct_survey.session_rows(d, 3)["rows"]
    out["missions"] = [(r.get("mission_id"), r.get("outcome")) for r in rows]
    out["eligible"] = len(rows) == 3 and all(r.get("outcome") == "done" for r in rows)
    try:
        result = json.loads((run / "result.json").read_text())
    except (OSError, ValueError):
        result = {}
    out["dropped"] = (result.get("tick_trace") or {}).get("dropped")
    try:
        entries = read_live_log(run)
        out["log_complete"] = True
    except ValueError:
        entries, out["log_complete"] = read_live_log(run, allow_incomplete=True), False
    out["steps"] = {int(e["step"]): (float(e["mono"]), float(e["tick_t"]), bool(e["pacing_ok"]))
                    for e in entries if "pacing_ok" in e and "step" in e}
    submits = [float(e["mono"]) for e in entries if e.get("t") == "mission_submit" and "mono" in e]
    ends = [float(e["mono"]) for e in entries if "mission_end" in e and "mono" in e]
    out["window"] = [min(submits) if submits else None, max(ends) if ends else None]
    spacing = (1 / 120) / factor
    try:
        z = np.load(run / "tick_trace.npz")
        S = z["data"][:, list(z["columns"]).index("S")]
        gaps = np.diff(S)
        gaps = gaps[~np.isnan(gaps)]
        out["short_gaps"] = int((gaps < spacing - 0.001).sum())
        out["setting_ok"] = (setting_ok(int(out["catchup_args"] or -1), gaps, spacing)
                             if out["catchup_args"] in ("0", "6") else False)
    except (OSError, KeyError, ValueError):
        out["setting_ok"], out["short_gaps"] = False, None
    return out


def evaluate(out_dir: Path) -> dict:
    plan = json.loads((out_dir / "plan.json").read_text())
    factor = float(plan["factor"])
    bundles = []
    for b in plan["bundles"]:
        recs = [session_record(out_dir / "s" / n, factor) for n in b["runs"]]
        windows = [r.get("window") for r in recs]
        if any(not w or None in w for w in windows):
            interval, why = None, "a session without a measurement window"
        elif b["concurrency"] == 1:
            interval, why = "own", ""
        else:
            a, z = max(w[0] for w in windows), min(w[1] for w in windows)
            interval, why = ([a, z], "") if z - a >= COMMON_MIN_S else (None, "common interval under 15 min")
        for r, w in zip(recs, windows, strict=True):
            iv = w if interval == "own" else interval
            st = interval_stats(r["steps"], iv[0], iv[1]) if iv else {"decidable": False, "why": why}
            r.update({k: v for k, v in st.items()})
            ok_checks = (r.get("dropped") == 0, r.get("log_complete"), r.get("setting_ok"),
                         r.get("phase_args") == "unset", r.get("catchup_args") == str(b["catchup"]))
            if not all(ok_checks):
                r["decidable"] = False
                r["why"] = (r.get("why") or "") + f" checks {ok_checks}"
            r.pop("steps", None)
        bundles.append({**b, "interval": interval, "sessions": recs})
    arms = {}
    for k in (0, 6):
        ones = [s for b in bundles if b["catchup"] == k and b["concurrency"] == 1 for s in b["sessions"]]
        usable = [s for s in ones if s.get("eligible") and s.get("decidable")]
        f1 = float(np.mean([s["F"] for s in usable])) if len(usable) == len(ones) and ones else math.nan
        r1 = float(np.mean([s["R"] for s in usable])) if len(usable) == len(ones) and ones else math.nan
        held_by_n = {}
        for b in bundles:
            if b["catchup"] == k:
                held_by_n[b["concurrency"]] = None if math.isnan(f1) else held(b["sessions"], f1, r1)
        arms[k] = {"F1": f1, "R1": r1, "held": held_by_n}
    blockers = stage2_blockers(arms)
    targets = None if blockers else stage2_targets(arms[6]["held"])
    return {"bundles": bundles, "arms": {str(k): v for k, v in arms.items()}, "stage2_blockers": blockers,
            "stage2_targets": targets,
            "rules": {"F_ratio": F_RATIO, "R_add": R_ADD, "F1_stop": F1_STOP, "missing_share": MISSING_SHARE,
                      "missing_gap_s": MISSING_GAP_S, "common_min_s": COMMON_MIN_S}}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out", type=Path)
    ap.add_argument("--json", type=Path, required=True)
    a = ap.parse_args(argv)
    a.json.write_text(json.dumps(evaluate(a.out), indent=1, default=str) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
