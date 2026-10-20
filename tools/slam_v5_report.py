"""SLAM localization plan V5 (docs/plans/2026-10-20-slam-localization-session.md): a live session's continuous-operation
verdict from its epoch's run folder.

    python tools/slam_v5_report.py RUN_DIR [--json OUT] [--min-done 7]

Reads ``result.json`` (the runner's end: failure, slam_map, slam_summary), ``missions/*/result.json`` (each mission's
outcome and delivery error) and the spool (``slam.records``: each processed keyframe's bridge time and when it was
queued; ``slam.control``: the estimate against the truth every control tick). V5's rows:

- at least ``--min-done`` missions done (U16's seven), no ``slam_link_failed`` / ``localization_stale`` end;
- keyframe processing (the bridge's ``slam_wall_s`` of processed replies) at most 0.3 s;
- every 30 min of wall time (by the runner's monotonic queue clock) 50 or more samples, the last window's p99 at most
  1.5 x the first's;
- the estimate against the truth over every ``slam.control`` row: position max <= 0.15 m, yaw max <= 0.05 rad;
- each mission's delivery error, listed (the console's U-case bound is the reader's).
"""

from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "isaac"))
from forklift_core.live.spool import iter_chunks, load_list  # noqa: E402
from live_records import slam_summary  # noqa: E402

KEYFRAME_MAX_S = 0.3
WINDOW_S = 1800.0
WINDOW_MIN_SAMPLES = 50
P99_GROWTH = 1.5
POSITION_MAX_M, YAW_MAX_RAD = 0.15, 0.05
BAD_ENDS = ("slam_link_failed", "localization_stale")


def keyframe_windows(records: list, window_s: float = WINDOW_S) -> list[dict]:
    """Processed keyframes' bridge times by wall window (queued_mono from the first record's)."""
    rows = [(float(r["queued_mono"]), float(r["slam_wall_s"])) for r in records
            if r.get("status") == "processed" and r.get("queued_mono") is not None and r.get("slam_wall_s") is not None]
    if not rows:
        return []
    start = rows[0][0]
    out: dict[int, list] = {}
    for mono, wall in rows:
        out.setdefault(int((mono - start) // window_s), []).append(wall)
    return [{"window": k, "samples": len(v), "p99_s": float(np.percentile(v, 99)), "max_s": float(max(v)),
             "mean_s": float(np.mean(v))} for k, v in sorted(out.items())]


def report(run: Path, min_done: int = 7) -> dict:
    result = json.loads((run / "result.json").read_text())
    missions = []
    for path in sorted(glob.glob(str(run / "missions" / "*" / "result.json"))):
        m = json.loads(Path(path).read_text())
        entry = m.get("result") or {}
        missions.append({"mission_id": entry.get("mission_id"), "outcome": entry.get("outcome"),
                         "reason": entry.get("reason"), "map_id": entry.get("map_id"),
                         "delivery_error_m": m.get("delivery_error_m")})
    done = sum(1 for m in missions if m["outcome"] == "done")
    failure = result.get("failure_reason") or ""
    records = load_list(run / "spool", "slam.records")
    processed = [float(r["slam_wall_s"]) for r in records if r.get("status") == "processed" and
                 r.get("slam_wall_s") is not None]
    windows = keyframe_windows(records)
    summary = slam_summary(iter_chunks(run / "spool", "slam.control"), [records])
    checks = {
        "missions": {"ok": done >= min_done, "done": done, "of": len(missions), "min": min_done},
        "ending": {"ok": not any(failure.startswith(b) for b in BAD_ENDS), "failure_reason": failure or None},
        "keyframe": {"ok": bool(processed) and max(processed) <= KEYFRAME_MAX_S, "processed": len(processed),
                     "max_s": max(processed) if processed else None,
                     "p99_s": float(np.percentile(processed, 99)) if processed else None},
        "windows": {"ok": bool(windows) and all(w["samples"] >= WINDOW_MIN_SAMPLES for w in windows)
                    and windows[-1]["p99_s"] <= P99_GROWTH * windows[0]["p99_s"], "windows": windows},
        "estimate": {"ok": summary["raw_position_max_m"] is not None and summary["raw_position_max_m"] <= POSITION_MAX_M
                     and summary["raw_yaw_max_rad"] <= YAW_MAX_RAD, **summary},
        "deliveries": {"missions": missions},
    }
    return {"run": str(run), "slam_map": result.get("slam_map"), "checks": checks,
            "ok": all(c["ok"] for k, c in checks.items() if k != "deliveries")}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run", type=Path)
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--min-done", type=int, default=7)
    a = ap.parse_args(argv)
    out = report(a.run, a.min_done)
    text = json.dumps(out, indent=1)
    if a.json:
        a.json.write_text(text)
    print(text)
    return 0 if out["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
