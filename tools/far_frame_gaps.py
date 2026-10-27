"""The learned proposer's timing contract, as revised on 2026-10-28 (design 2026-10-15 §6): read live far-recognition
records and report the primary condition and the watch metrics.

Primary condition: the gap between consecutive applied driving frames (capture time, within one leg) stays at or under
the candidate expiry 0.5 s. Watch metrics, an alert when nonzero: runs of two or more consecutive dropped driving frames,
and gaps over 0.4 s (at 5 Hz one missing frame gives 0.4 s; two in a row give 0.6 s, past the expiry). Recorded only:
the worker round trip (model_ms; p50/p95/p99/max, first call apart) and its share over 200 ms, and whether every stop
frame was applied.

    python -m tools.far_frame_gaps <dir> [<dir> ...] --out gaps.json

Every ``**/recognition_missions/*/result.json`` under the given directories is read (live runs write one per mission).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

EXPIRY_S = 0.5
WATCH_GAP_S = 0.4
LATENCY_MS = 200.0
_EPS = 1e-6  # capture times carry ~2e-8 s of float error (0.40000002 for one missing frame at 5 Hz)


class RecordError(ValueError):
    """A record without the live far-recognition structure the contract reads."""


def _finite(value, what):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise RecordError(f"{what} is not a finite number: {value!r}")
    return float(value)


def _validated(record) -> tuple[list, list, dict]:
    if not isinstance(record, dict):
        raise RecordError("the record is not an object")
    rec = record.get("recognition")
    live = rec.get("live") if isinstance(rec, dict) else None
    if not isinstance(live, dict):
        raise RecordError("no recognition.live")
    jobs, frames = live.get("jobs"), rec.get("frames")
    if not isinstance(jobs, list) or not isinstance(frames, list):
        raise RecordError("recognition.live.jobs / recognition.frames are not lists")
    for j in jobs:
        if not isinstance(j, dict) or not isinstance(j.get("seq"), int) or not isinstance(j.get("status"), str) \
                or j.get("kind") not in ("drive", "stop"):
            raise RecordError(f"a malformed job: {j!r}")
        if j.get("model_ms") is not None:
            _finite(j["model_ms"], "model_ms")
    for f in frames:
        if not isinstance(f, dict):
            raise RecordError(f"a malformed frame: {f!r}")
        if f.get("kind") == "drive":
            _finite(f.get("pixel_t_s"), "pixel_t_s")
    return jobs, frames, live


def mission_metrics(record: dict) -> dict:
    jobs, all_frames, live = _validated(record)
    drive_frames = [f for f in all_frames if f.get("kind") == "drive"]
    # A skipped frame (no pose, no depth or no segmentation yet) updates nothing: it does not split a gap.
    frames = [f for f in drive_frames if not f.get("skipped")]
    gaps = []
    legs: dict = {}
    for f in frames:
        legs.setdefault(f.get("leg"), []).append(float(f["pixel_t_s"]))
    for times in legs.values():
        times.sort()
        gaps += [b - a for a, b in zip(times, times[1:])]
    drives = sorted((j for j in jobs if j["kind"] == "drive"), key=lambda j: j["seq"])
    runs_of_two, run = 0, 0
    for j in drives:
        if j["status"] == "dropped":
            run += 1
            if run == 2:
                runs_of_two += 1
        else:
            run = 0
    stops = [j for j in jobs if j["kind"] == "stop"]
    ordered = sorted(jobs, key=lambda j: j["seq"])
    return {
        "mission_id": live.get("mission_id"),
        "max_gap_s": max(gaps) if gaps else None,
        "gaps_over_watch": sum(g > WATCH_GAP_S + _EPS for g in gaps),
        "gaps_over_expiry": sum(g > EXPIRY_S + _EPS for g in gaps),
        "consecutive_drop_runs": runs_of_two,
        "dropped": sum(j["status"] == "dropped" for j in jobs),
        "skipped_frames": len(drive_frames) - len(frames),
        "stop_not_applied": sum(j["status"] != "applied" for j in stops),
        # the calls the worker made: with a time, or in flight at the mission's end (time from the late calls)
        "calls": [(j["seq"], float(j["model_ms"])) for j in ordered if j.get("model_ms") is not None],
        "in_flight": [j["seq"] for j in ordered if j["status"] == "in_flight" and j.get("model_ms") is None],
        "leg_lost_frames": sum(1 for f in frames if f.get("leg_lost")),
    }


def late_calls(records: list) -> dict:
    """(mission, seq) -> model_ms of one run's calls that finished after their mission ended, from its mission
    records' live.late_calls and the run's far_late_calls. Mission ids restart in every run: merge per run."""
    out = {}
    for r in records:
        live = ((r.get("recognition") or {}).get("live") or {}) if isinstance(r, dict) else {}
        for c in list(live.get("late_calls") or []) + list((r.get("far_late_calls") or []) if isinstance(r, dict) else []):
            if isinstance(c, dict) and c.get("model_ms") is not None and isinstance(c.get("seq"), int):
                out[(c.get("mission"), c["seq"])] = _finite(c["model_ms"], "late model_ms")
    return out


def resolve_calls(m: dict, late: dict) -> dict:
    """The mission's call times with its in-flight calls filled from its own run's late calls; the first model call
    (the smallest seq that called the worker) apart."""
    calls = dict(m["calls"])
    unresolved = 0
    for seq in m["in_flight"]:
        if (m["mission_id"], seq) in late:
            calls[seq] = late[(m["mission_id"], seq)]
        else:
            unresolved += 1
    made = sorted(set(calls) | set(m["in_flight"]))
    first_seq = made[0] if made else None
    return {"first": calls.get(first_seq) if first_seq is not None else None,
            "first_unresolved": first_seq is not None and first_seq not in calls,
            "later": [ms for seq, ms in sorted(calls.items()) if seq != first_seq],
            "unresolved": unresolved}


def _quantile(values, p):
    v = sorted(values)
    return v[min(len(v) - 1, math.ceil(p * len(v)) - 1)] if v else None


def summarize(missions: list, resolved: list) -> dict:
    later = [ms for r in resolved for ms in r["later"]]
    first = [r["first"] for r in resolved if r["first"] is not None]
    out = {
        "missions": len(missions),
        "primary_ok": all(m["gaps_over_expiry"] == 0 for m in missions),
        "max_gap_s": max((m["max_gap_s"] for m in missions if m["max_gap_s"] is not None), default=None),
        "gaps_over_expiry": sum(m["gaps_over_expiry"] for m in missions),
        "gaps_over_watch": sum(m["gaps_over_watch"] for m in missions),
        "consecutive_drop_runs": sum(m["consecutive_drop_runs"] for m in missions),
        "dropped": sum(m["dropped"] for m in missions),
        "skipped_frames": sum(m["skipped_frames"] for m in missions),
        "stop_not_applied": sum(m["stop_not_applied"] for m in missions),
        "leg_lost_frames": sum(m["leg_lost_frames"] for m in missions),
        "round_trip_ms": {"n": len(later), "p50": _quantile(later, .5), "p95": _quantile(later, .95),
                          "p99": _quantile(later, .99), "max": max(later) if later else None,
                          "over_200_share": (sum(x > LATENCY_MS for x in later) / len(later)) if later else None,
                          "first_call_max": max(first) if first else None,
                          "in_flight_without_time": sum(r["unresolved"] for r in resolved)},
    }
    out["alert"] = bool(out["gaps_over_watch"] or out["consecutive_drop_runs"] or out["stop_not_applied"])
    return out


def evaluate_dir(d: Path) -> dict:
    """Every mission record under ``d``, its late calls merged within its own run (the run directory holding its
    recognition_missions folder, whose result.json carries the run's far_late_calls)."""
    if not d.is_dir():
        raise RecordError(f"{d}: not a directory")
    paths = sorted(d.glob("**/recognition_missions/*/result.json"))
    if not paths:
        raise RecordError(f"{d}: no recognition_missions/*/result.json -- undetermined, not a pass")
    by_run: dict = {}
    for p in paths:
        by_run.setdefault(p.parent.parent.parent, []).append(p)
    missions, resolved = [], []
    for run_dir, run_paths in sorted(by_run.items()):
        try:
            records = [json.loads(p.read_text()) for p in run_paths]
            run_result = run_dir / "result.json"
            extra = [json.loads(run_result.read_text())] if run_result.exists() else []
        except ValueError as exc:  # json.JSONDecodeError is a ValueError
            raise RecordError(f"{run_dir}: unreadable JSON ({exc})") from exc
        late = late_calls(records + extra)
        for r in records:
            m = mission_metrics(r)
            missions.append(m)
            resolved.append(resolve_calls(m, late))
    return summarize(missions, resolved)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dirs", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    report = {}
    for d in args.dirs:
        try:
            report[str(d)] = evaluate_dir(d)
        except RecordError as exc:
            print(str(exc), file=sys.stderr)
            return 2
    args.out.write_text(json.dumps(report, indent=1, allow_nan=False))
    for d, s in report.items():
        print(d, json.dumps({k: s[k] for k in ("missions", "primary_ok", "max_gap_s", "gaps_over_watch",
                                                  "consecutive_drop_runs", "alert")}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
