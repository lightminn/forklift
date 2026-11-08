"""The vehicle-side F1 block's smoke verdict from the vehicle's own records (docs/plans/2026-11-08-f1-vehicle-block.md 5):
the probe's block check passed, the runner's record says the start was operator_placed and how it decided, its recorded
argument is the expected setting, and -- block -- no AUTO tick and no mission in the run, or -- allow -- a mission made.

    python tools/f1_block_judge.py SESSION_DIR --expect block|allow

SESSION_DIR is reloc_session.sh's OUT (probe.json, epoch_*/run: the launcher hands it to the supervisor as is). Writes
SESSION_DIR/f1_block_verdict.json and exits 0 on a pass, 1 on a fail (a missing record is a fail, never a pass)."""

from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from forklift_core.live.spool import read_live_log  # noqa: E402


def _json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def _ticks(run: Path) -> tuple[int, int] | None:
    """(ticks, AUTO ticks) of the whole live log, read as complete (spool.read_live_log: no rolled part gone, an end
    summary with no lost line); None when it is not complete or holds no end summary."""
    try:
        entries = read_live_log(run)
    except (OSError, ValueError):
        return None
    if not any("log_end" in e for e in entries):
        return None  # a session that never closed its log is not judged whole
    ticks = [e for e in entries if "tick_t" in e and "mode" in e]
    return len(ticks), sum(e["mode"] == "AUTO" for e in ticks)


def judge(session: Path, expect: str) -> dict:
    checks: dict = {}
    probe = _json(session / "probe.json") or {}
    checks["probe_pass"] = (probe.get("block_check") or {}).get("verdict") == "pass"
    runs = sorted(glob.glob(str(session / "epoch_*" / "run")))
    checks["one_epoch"] = len(runs) == 1
    run = Path(runs[0]) if runs else session / "missing"
    result = _json(run / "result.json") or {}
    reloc = result.get("start_reloc") or {}
    checks["source_operator_placed"] = reloc.get("source") == "operator_placed"
    checks["argument_applied"] = (result.get("arguments") or {}).get("operator_placed_auto") == expect
    checks["decision_recorded"] = reloc.get("autonomy") == ("blocked" if expect == "block" else "allowed")
    missions = sorted(glob.glob(str(run / "missions" / "*"))) + list(result.get("mission_results") or [])
    read = _ticks(run)
    checks["log_whole"] = read is not None
    checks["ticks_logged"] = bool(read and read[0] > 0)
    auto = None if read is None else read[1]
    if expect == "block":
        checks["no_auto_tick"] = auto == 0
        checks["no_mission"] = not missions
    else:
        checks["mission_made"] = bool(missions)
    return {"expect": expect, "run": str(run), "checks": checks, "auto_ticks": auto,
            "verdict": "pass" if all(checks.values()) else "fail",
            "failed": sorted(k for k, v in checks.items() if not v)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("session", type=Path)
    ap.add_argument("--expect", choices=("block", "allow"), required=True)
    args = ap.parse_args(argv)
    out = judge(args.session, args.expect)
    (args.session / "f1_block_verdict.json").write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out))
    return 0 if out["verdict"] == "pass" else 1


if __name__ == "__main__":
    sys.exit(main())
