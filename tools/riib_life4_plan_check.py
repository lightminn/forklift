"""Lifecycle 4 confirmation's CPU pre-check (docs/plans/2026-11-09-live-riib-v2-lifecycle4-confirmation.md 3).

m2 (P-5 -> D-2 on random_m) is planned as tools/session_plan_check.plan_mission plans a mission, with P-4 left where it
stands (nothing delivered -- m1 was cancelled before pickup), leaving from each start pose and returning to the
session's start:

- the session's start;
- for each of the two registered m1 records.json (the main batch's random_m arm B, seeds 0 and 1, named s0B and s1B):
  its planned approach's first, middle and last pose, and the last pose backed 0.31 m (the original lifecycle 4's
  clear_out) and 1.0 m (clear_out's budget) against its heading.

These are sample points for a planning gate, not the range of where a STOP and the recovery after a cancel can leave the
truck (clear_out retraces the recent forward path and may extend it), and the fresh prior P-4 is not the P-4 m1
recognised. The formal check needs every registered start checked once; it passes when each gating start (the session
start, each approach's first pose and backed-off ends) plans and passes (outcome ``passed``, no submit refusal) -- the
approach's middle and last pose are reported only, since clear_out runs when such a pose fails the start check. It shows that plans exist; whether m2 observes and completes is the Isaac run's to show.

    python tools/riib_life4_plan_check.py SESSION.json --records s0B=REC.json --records s1B=REC.json --out RESULT.json
    python tools/riib_life4_plan_check.py SESSION.json --smoke --out RESULT.json     # the session start only, no gate
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))

PALLET, DESTINATION = "P-5", "D-2"
RECORD_NAMES = ("s0B", "s1B")
BACKS_M = (0.31, 1.0)
LABELS = ("approach_start", "approach_mid", "approach_end")


def _back_label(b: float) -> str:
    return f"approach_end_back_{b:.2f}"


def approach_starts(record: dict, name: str, *, backs=BACKS_M) -> list[dict]:
    """The start poses one m1 record gives: its planned approach's first, middle and last pose, and the last backed
    each of ``backs`` against its heading."""
    poses = record["state"]["paths"]["approach"]["poses"]
    end = [float(v) for v in poses[-1]]
    picked = list(zip(LABELS, (poses[0], poses[len(poses) // 2], end)))
    for b in backs:
        picked.append((_back_label(b), [end[0] - b * math.cos(end[2]), end[1] - b * math.sin(end[2]), end[2]]))
    return [{"name": f"{name}:{label}", "pose": [float(v) for v in pose]} for label, pose in picked]


def expected_names(records=RECORD_NAMES, backs=BACKS_M) -> list[str]:
    return ["session_start"] + [f"{r}:{label}" for r in records
                                for label in (*LABELS, *(_back_label(b) for b in backs))]


def gate_names(records=RECORD_NAMES, backs=BACKS_M) -> list[str]:
    """The starts that gate: the session start and, per record, the approach's first pose and the backed-off ends. The
    approach's middle and last pose are reported only: clear_out runs exactly when such a pose fails the next mission's
    start check (independent review P2-3)."""
    return ["session_start"] + [f"{r}:{label}" for r in records
                                for label in ("approach_start", *(_back_label(b) for b in backs))]


def verdict(entries: list[dict], expected: list[str], gate: list[str] | None = None) -> dict:
    gate = expected if gate is None else gate
    names = [e["name"] for e in entries]
    bad = [e["name"] for e in entries if e.get("outcome") != "passed" or e.get("submit_check") is not None]
    failed = [n for n in bad if n in gate]
    missing = [n for n in expected if n not in names]
    extra = sorted({n for n in names if n not in expected or names.count(n) > 1})
    return {"pass": bool(entries) and not failed and not missing and not extra, "failed": failed,
            "report_failed": [n for n in bad if n not in gate], "missing": missing,
            "unexpected_or_duplicate": extra, "starts": len(entries)}


def check(session_path: Path, starts: list[dict]) -> list[dict]:
    import session_plan_check as S
    from forklift_core.planning import Pose2D

    ctx = S.session_context(session_path)
    out = []
    for start in starts:
        entry = S.plan_mission(ctx["session"], ctx["pallet"], ctx["geometry"], ctx["config"], S.fresh_targets(ctx), {},
                               PALLET, DESTINATION, nominal=ctx["nominal"], shape=ctx["shape"],
                               prior_grow_m=S.SESSION_PRIOR_GROW_M, observe_planner=S.plan_observation_leg,
                               transport_planner=S.plan_transport, start=Pose2D(*start["pose"]))
        out.append({"name": start["name"], "start_rear": start["pose"], **entry})
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("session", type=Path)
    ap.add_argument("--records", action="append", default=[], help="s0B=path and s1B=path (m1 records.json)")
    ap.add_argument("--smoke", action="store_true", help="the session start only; never a pass of the gate")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    recs = dict(item.split("=", 1) for item in a.records)
    if not a.smoke and (sorted(recs) != sorted(RECORD_NAMES) or len(a.records) != len(RECORD_NAMES)):
        ap.error(f"the formal check needs exactly --records {' and '.join(RECORD_NAMES)}")
    session = json.loads(a.session.read_text())
    s0 = session["start_rear"]
    starts = [{"name": "session_start", "pose": [float(s0["x_m"]), float(s0["y_m"]), float(s0["yaw_rad"])]}]
    if not a.smoke:
        for name in RECORD_NAMES:
            starts += approach_starts(json.loads(Path(recs[name]).read_text()), name)
    entries = check(a.session, starts)
    expected = ["session_start"] if a.smoke else expected_names()
    v = verdict(entries, expected, None if a.smoke else gate_names())
    if a.smoke:
        v["pass"] = False
        v["smoke"] = True
    result = {"session": str(a.session), "pallet": PALLET, "destination": DESTINATION, "backs_m": list(BACKS_M),
              "records": recs, "verdict": v, "entries": entries}
    a.out.write_text(json.dumps(result, indent=1, default=str))
    print(json.dumps({"verdict": v, "outcomes": {e["name"]: e.get("outcome") for e in entries}}, indent=1))
    return 0 if v["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
