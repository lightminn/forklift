"""SLAM localization plan V5 (docs/plans/2026-10-20-slam-localization-session.md): a live session's continuous-operation
verdict from its epoch's run folder.

    python tools/slam_v5_report.py RUN_DIR [--json OUT] [--min-done 7] [--min-hours 2]

Reads ``result.json`` (the runner's end: failure, slam_map), the live log's ``mission_end`` entries (every mission the
session ended) with each ``missions/<id>/result.json`` (its delivery error) and the spool (``slam.records``: each processed keyframe's bridge time and when it was
queued; ``slam.control``: the estimate against the truth every control tick). V5's rows:

- at least ``--min-done`` missions done (U16's seven), every ended mission's result file there, no
  ``slam_link_failed`` / ``localization_stale`` end (the session's or a mission's);
- keyframe processing (the bridge's ``slam_wall_s`` of processed replies) at most 0.3 s;
- ``--min-hours`` (2 h) of whole 30 min wall windows (the runner's monotonic queue clock, first to last processed
  keyframe), every one -- empty ones included -- with 50 or more samples, the last window's p99 at most 1.5 x the first's;
- the estimate against the truth over every ``slam.control`` row (the spool's count checked, every value finite):
  position max <= 0.15 m, yaw max <= 0.05 rad;
- each mission's delivery within the console's U-case bound (forklift_core.mission.delivery: under 0.08 m, grounded,
  forks out) -- a mission that failed it fails V5.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "isaac"))
from forklift_core.live.spool import load_list, read_live_log  # noqa: E402
from forklift_core.mission.delivery import FORK_INSIDE, MISSED, NOT_GROUNDED  # noqa: E402
from live_records import slam_summary  # noqa: E402

KEYFRAME_MAX_S = 0.3
WINDOW_S = 1800.0
WINDOW_MIN_SAMPLES = 50
P99_GROWTH = 1.5
POSITION_MAX_M, YAW_MAX_RAD = 0.15, 0.05
BAD_ENDS = ("slam_link_failed", "localization_stale")


def keyframe_windows(records: list, window_s: float = WINDOW_S) -> tuple[list[dict], float]:
    """Processed keyframes' bridge times by wall window (queued_mono from the first processed record's), every whole
    window from the first to the last processed record -- an empty one included (Codex V5 review P2) -- and the span."""
    rows = [(float(r["queued_mono"]), float(r["slam_wall_s"])) for r in records
            if r.get("status") == "processed" and r.get("queued_mono") is not None and r.get("slam_wall_s") is not None]
    if not rows:
        return [], 0.0
    start = rows[0][0]
    span = rows[-1][0] - start
    out: dict[int, list] = {k: [] for k in range(int(span // window_s))}  # whole windows only; a trailing part is not one
    for mono, wall in rows:
        k = int((mono - start) // window_s)
        if k in out:
            out[k].append(wall)
    return [{"window": k, "samples": len(v), "p99_s": float(np.percentile(v, 99)) if v else None,
             "max_s": float(max(v)) if v else None, "mean_s": float(np.mean(v)) if v else None}
            for k, v in sorted(out.items())], span


def control_errors(control: list) -> dict:
    """Position and yaw error over every ``slam.control`` row [t, rear x, y, yaw, truth x, y, yaw]; a non-finite row
    fails the check rather than hiding in a chunk maximum (Codex V5 review P2)."""
    rows = np.asarray(control, dtype=float).reshape(-1, 7)
    finite = bool(len(rows)) and bool(np.isfinite(rows).all())
    if not finite:
        return {"ok": False, "rows": int(len(rows)), "finite": False}
    error = np.hypot(rows[:, 1] - rows[:, 4], rows[:, 2] - rows[:, 5])
    yaw = np.abs(np.angle(np.exp(1j * (rows[:, 3] - rows[:, 6]))))
    return {"ok": float(error.max()) <= POSITION_MAX_M and float(yaw.max()) <= YAW_MAX_RAD, "rows": int(len(rows)),
            "finite": True, "position_max_m": float(error.max()), "position_rmse_m": float(np.sqrt(np.mean(error**2))),
            "yaw_max_rad": float(yaw.max()), "yaw_rmse_rad": float(np.sqrt(np.mean(yaw**2)))}


def correction_jumps(records: list) -> dict:
    """Each processed keyframe's correction against the previous one, both composed with its odometry -- the pose change
    the correction alone makes (reported, not judged: Codex V5 diagnosis P2, loop closure can jump the pose)."""
    def compose(a, b):
        c, s = math.cos(a[2]), math.sin(a[2])
        return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])

    out, previous = [], None
    for r in records:
        if r.get("status") != "processed" or r.get("map_from_odom") is None:
            continue
        if previous is not None:
            before, after = compose(previous, r["odom_base"]), compose(r["map_from_odom"], r["odom_base"])
            out.append((math.hypot(after[0] - before[0], after[1] - before[1]),
                        abs(math.remainder(after[2] - before[2], 2 * math.pi))))
        previous = r["map_from_odom"]
    j = np.asarray(out, float).reshape(-1, 2)
    return {"n": int(len(j)), "position_max_m": float(j[:, 0].max()) if len(j) else None,
            "yaw_max_rad": float(j[:, 1].max()) if len(j) else None, "position_over_0_05_m": int((j[:, 0] > 0.05).sum())}


def mission_list(run: Path) -> list[dict]:
    """Every mission the session ended, from the live log's ``mission_end`` entries (the authoritative list), each with
    its ``missions/<id>/result.json`` delivery error; a mission whose result file is missing says so (Codex V5 P2)."""
    out = []
    for entry in read_live_log(run):
        end = entry.get("mission_end")
        if end is None:
            continue
        path = run / "missions" / str(end["mission_id"]) / "result.json"
        record = json.loads(path.read_text()) if path.exists() else None
        out.append({"mission_id": end["mission_id"], "outcome": end.get("outcome"), "reason": end.get("reason"),
                    "map_id": end.get("map_id"), "result_file": record is not None,
                    "delivery_error_m": None if record is None else record.get("delivery_error_m")})
    return out


def report(run: Path, min_done: int = 7, min_hours: float = 2.0) -> dict:
    result = json.loads((run / "result.json").read_text())
    missions = mission_list(run)
    done = sum(1 for m in missions if m["outcome"] == "done")
    failure = result.get("failure_reason") or ""
    records = load_list(run / "spool", "slam.records")
    control = load_list(run / "spool", "slam.control")  # its manifest count checked, as the records' are
    # every processed keyframe's bridge time, present and finite (a missing or NaN one would drop out of max/p99)
    processed = [r.get("slam_wall_s") for r in records if r.get("status") == "processed"]
    timed = all(v is not None and np.isfinite(float(v)) for v in processed)
    processed = [float(v) for v in processed if v is not None]
    windows, span = keyframe_windows(records)
    need = int(math.ceil(min_hours * 3600 / WINDOW_S - 1e-9))  # whole windows covering at least min_hours
    # The plan's delivery bound: the console's U-case check (forklift_core.mission.delivery: the pallet centre under
    # 0.08 m from the destination, grounded, the forks out). A done mission passed it; a mission that failed it, or
    # whose result file is missing, fails V5 here.
    delivery_failed = [m["mission_id"] for m in missions
                       if m["reason"] in (MISSED, NOT_GROUNDED, FORK_INSIDE)
                       or (m["delivery_error_m"] is not None and not m["delivery_error_m"] < 0.08)
                       or (m["outcome"] == "done" and m["result_file"] and m["delivery_error_m"] is None)]  # a done one
    # has its error
    missing = [m["mission_id"] for m in missions if not m["result_file"]]
    checks = {
        "missions": {"ok": done >= min_done and not missing, "done": done, "of": len(missions), "min": min_done,
                     "result_file_missing": missing},
        "ending": {"ok": not any(failure.startswith(b) for b in BAD_ENDS)
                   and not any((m["reason"] or "").startswith(BAD_ENDS) for m in missions),
                   "failure_reason": failure or None},
        "keyframe": {"ok": bool(processed) and timed and max(processed) <= KEYFRAME_MAX_S, "processed": len(processed),
                     "all_timed": timed,
                     "max_s": max(processed) if processed else None,
                     "p99_s": float(np.percentile(processed, 99)) if processed and timed else None},
        "windows": {"ok": len(windows) >= need and all(w["samples"] >= WINDOW_MIN_SAMPLES for w in windows)
                    and windows[-1]["p99_s"] <= P99_GROWTH * windows[0]["p99_s"],
                    "span_s": span, "need": need, "windows": windows},
        "estimate": {**control_errors(control), "summary": slam_summary([control], [records])},
        "deliveries": {"ok": not delivery_failed, "failed": delivery_failed,
                       "errors_m": {m["mission_id"]: m["delivery_error_m"] for m in missions}},
    }
    return {"run": str(run), "slam_map": result.get("slam_map"), "checks": checks, "jumps": correction_jumps(records),
            "missions": missions, "ok": all(c["ok"] for c in checks.values())}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run", type=Path)
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--min-done", type=int, default=7)
    ap.add_argument("--min-hours", type=float, default=2.0)
    a = ap.parse_args(argv)
    out = report(a.run, a.min_done, a.min_hours)
    text = json.dumps(out, indent=1)
    if a.json:
        a.json.write_text(text)
    print(text)
    return 0 if out["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
