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


def mission_metrics(record: dict) -> dict:
    rec = record.get("recognition")
    live = (rec or {}).get("live")
    if not isinstance(rec, dict) or not isinstance(live, dict) or "jobs" not in live or "frames" not in rec:
        raise RecordError("no recognition.live.jobs / recognition.frames")
    jobs = live["jobs"] or []
    drive_frames = [f for f in rec["frames"] or [] if f.get("kind") == "drive"]
    # A skipped frame (no pose, no depth or no segmentation yet) updates nothing: it does not split a gap.
    frames = [f for f in drive_frames if not f.get("skipped")]
    gaps = []
    legs: dict = {}
    for f in frames:
        legs.setdefault(f.get("leg"), []).append(float(f["pixel_t_s"]))
    for times in legs.values():
        times.sort()
        gaps += [b - a for a, b in zip(times, times[1:])]
    drives = sorted((j for j in jobs if j.get("kind") == "drive"), key=lambda j: j["seq"])
    runs_of_two, run = 0, 0
    for j in drives:
        if j.get("status") == "dropped":
            run += 1
            if run == 2:
                runs_of_two += 1
        else:
            run = 0
    stops = [j for j in jobs if j.get("kind") == "stop"]
    ordered = sorted(jobs, key=lambda j: j["seq"])
    return {
        "mission_id": live.get("mission_id"),
        "first_seq": ordered[0]["seq"] if ordered else None,
        "max_gap_s": max(gaps) if gaps else None,
        "gaps_over_watch": sum(g > WATCH_GAP_S + _EPS for g in gaps),
        "gaps_over_expiry": sum(g > EXPIRY_S + _EPS for g in gaps),
        "consecutive_drop_runs": runs_of_two,
        "dropped": sum(j.get("status") == "dropped" for j in jobs),
        "skipped_frames": len(drive_frames) - len(frames),
        "stop_not_applied": sum(j.get("status") != "applied" for j in stops),
        # (seq, model_ms) of the calls this mission took in; in_flight ones get their time from the late calls
        "calls": [(j["seq"], j["model_ms"]) for j in ordered if j.get("model_ms") is not None],
        "in_flight": [j["seq"] for j in ordered if j.get("status") == "in_flight" and j.get("model_ms") is None],
        "leg_lost_frames": sum(1 for f in frames if f.get("leg_lost")),
    }


def late_calls(records: list) -> dict:
    """(mission, seq) -> model_ms of the calls that finished after their mission ended, from every mission record's
    live.late_calls and the runs' far_late_calls, without duplicates."""
    out = {}
    for r in records:
        for c in ((r.get("recognition") or {}).get("live") or {}).get("late_calls") or []:
            if c.get("model_ms") is not None:
                out[(c.get("mission"), c["seq"])] = c["model_ms"]
        for c in r.get("far_late_calls") or []:
            if c.get("model_ms") is not None:
                out[(c.get("mission"), c["seq"])] = c["model_ms"]
    return out


def _quantile(values, p):
    v = sorted(values)
    return v[min(len(v) - 1, math.ceil(p * len(v)) - 1)] if v else None


def summarize(missions: list, late: dict | None = None) -> dict:
    late = late or {}
    later, first, unresolved = [], [], 0
    for m in missions:
        calls = dict(m["calls"])
        for seq in m["in_flight"]:
            if (m["mission_id"], seq) in late:
                calls[seq] = late[(m["mission_id"], seq)]
            else:
                unresolved += 1
        for seq, ms in sorted(calls.items()):
            (first if seq == m["first_seq"] else later).append(ms)  # each mission's first call apart
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
                          "in_flight_without_time": unresolved},
    }
    out["alert"] = bool(out["gaps_over_watch"] or out["consecutive_drop_runs"] or out["stop_not_applied"])
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dirs", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    report = {}
    for d in args.dirs:
        if not d.is_dir():
            print(f"{d}: not a directory", file=sys.stderr)
            return 2
        paths = sorted(d.glob("**/recognition_missions/*/result.json"))
        if not paths:
            print(f"{d}: no recognition_missions/*/result.json -- undetermined, not a pass", file=sys.stderr)
            return 2
        records = [json.loads(p.read_text()) for p in paths]
        runs = [json.loads(p.read_text()) for p in sorted(d.glob("**/run/result.json"))
                if "recognition_missions" not in p.parts]
        try:
            missions = [mission_metrics(r) for r in records]
        except RecordError as exc:
            print(f"{d}: {exc}", file=sys.stderr)
            return 2
        report[str(d)] = summarize(missions, late_calls(records + runs))
    args.out.write_text(json.dumps(report, indent=1, allow_nan=False))
    for d, s in report.items():
        print(d, json.dumps({k: s[k] for k in ("missions", "primary_ok", "max_gap_s", "gaps_over_watch",
                                                  "consecutive_drop_runs", "alert")}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
