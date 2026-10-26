#!/usr/bin/env python3
"""The live-default plan's Isaac verdict (docs/plans/2026-10-26-slam-localization-default.md step 3): a localization run
L on the accepted survey bundle against a mapping run R (``MAP_BUNDLE=none``) of the same snapshot, scene, seed and
order, from their epochs' run folders.

L must hold, on its own:
- every processed keyframe under 1 s, no SLAM end (slam_link_failed, localization_stale, slam_not_localized, a map
  refusal);
- an observed length (first to last processed keyframe, sim time) of at least 3,600 s and, in each of the six fixed
  sim-600 s windows from its first keyframe, a mission done and 50 processed keyframes.

On the common sim span (the intersection of the two runs' observed spans; "not comparable" when there is none) L's
response pose (each processed keyframe's map <- odom composed with its odometry) and applied pose (``slam.control``)
against the truth, position and heading, median and maximum, are no worse than R's, and L completes no fewer missions.
Past the end of R (R failed or stopped first), L is held to PR #62 V5's bounds -- applied position max <= 0.15 m, heading
max <= 0.05 rad -- and its second 30 min applied-position median is no higher than its first. Correction jumps and
deliveries are reported beside.

    python tools/slam_live_compare.py L_RUN R_RUN [--json OUT]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

KEYFRAME_MAX_S = 1.0
OBSERVED_MIN_S = 3600.0
WINDOW_S, WINDOWS, WINDOW_MIN_KEYFRAMES = 600.0, 6, 50
POSITION_MAX_M, YAW_MAX_RAD = 0.15, 0.05  # PR #62 V5
HALF_S = 1800.0
BAD_ENDS = ("slam_link_failed", "localization_stale", "slam_not_localized", "slam_map_mismatch", "slam_map_not_loaded")


@dataclass
class RunData:
    records: list = field(default_factory=list)  # slam.records rows
    control: list = field(default_factory=list)  # slam.control rows [t, rear x, y, yaw, truth x, y, yaw]
    missions: list = field(default_factory=list)  # mission_end entries with ended_s and delivery_error_m
    failure: str | None = None
    failures: list = field(default_factory=list)  # result.json live_failures: the mission failures of the session
    holds: list = field(default_factory=list)  # result.json slam_summary.holds: the session's hold / release timeline


def failures_of(result: dict) -> list[dict]:
    """A session's mission failures (``live_failures``: time, phase, reason, class), the traceback left out -- the
    session's own ``failure_reason`` is its end, and a mission failure does not end a session (S3 result review)."""
    return [{k: f.get(k) for k in ("time_s", "phase", "reason", "class")} for f in result.get("live_failures") or []]


def _compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def _yaw_err(a, b):
    return abs(math.remainder(a - b, 2 * math.pi))


def _finite(values, n) -> bool:
    try:
        return values is not None and len(values) == n and all(math.isfinite(float(v)) for v in values)
    except (TypeError, ValueError):
        return False


def record_problems(run: RunData) -> list[str]:
    """What is missing or not finite in the raw samples, before any filter could drop it (Codex round 2): every
    processed keyframe's stamp, bridge time, correction, odometry and truth, and every control row."""
    out = []
    for r in run.records:
        if r.get("status") != "processed":
            continue
        bad = [k for k, ok in (("stamp_s", _finite([r.get("stamp_s")], 1)),
                               ("slam_wall_s", _finite([r.get("slam_wall_s")], 1)),
                               ("map_from_odom", _finite(r.get("map_from_odom"), 3)),
                               ("odom_base", _finite(r.get("odom_base"), 3)),
                               ("truth_base", _finite(r.get("truth_base"), 3))) if not ok]
        if bad:
            out.append(f"keyframe {r.get('scan_id')}: {', '.join(bad)} missing or not finite")
    bad_rows = [i for i, row in enumerate(run.control) if not _finite(row, 7)]
    if bad_rows:
        out.append(f"{len(bad_rows)} control rows missing a value or not finite (first {bad_rows[0]})")
    return out


def _processed(run: RunData, t0=None, t1=None) -> list:
    return [r for r in run.records if r.get("status") == "processed" and r.get("stamp_s") is not None
            and (t0 is None or r["stamp_s"] >= t0) and (t1 is None or r["stamp_s"] <= t1)]


def _span(run: RunData):
    p = _processed(run)
    return (float(p[0]["stamp_s"]), float(p[-1]["stamp_s"])) if p else None


def _stats(position, yaw) -> dict:
    """Median and max of the errors; ``finite`` False (no figures) when a sample is missing a value."""
    position, yaw = np.asarray(position, float), np.asarray(yaw, float)
    if not len(position):
        return {"n": 0, "finite": False}
    if not (np.isfinite(position).all() and np.isfinite(yaw).all()):
        return {"n": int(len(position)), "finite": False}
    return {"n": int(len(position)), "finite": True, "position_median_m": float(np.median(position)),
            "position_max_m": float(position.max()), "yaw_median_rad": float(np.median(yaw)),
            "yaw_max_rad": float(yaw.max())}


def _applied(run: RunData, t0=None, t1=None) -> tuple:
    rows = np.asarray(run.control, float).reshape(-1, 7)
    if t0 is not None:
        rows = rows[rows[:, 0] >= t0]
    if t1 is not None:
        rows = rows[rows[:, 0] <= t1]
    position = np.hypot(rows[:, 1] - rows[:, 4], rows[:, 2] - rows[:, 5])
    yaw = np.abs(np.angle(np.exp(1j * (rows[:, 3] - rows[:, 6]))))
    return rows[:, 0], position, yaw


def correction_jumps(records: list) -> dict:
    """The pose change each processed keyframe's correction alone makes (against the previous one, both composed with
    its odometry) -- as tools/slam_v5_report.py reports it."""
    out, previous = [], None
    for r in records:
        if r.get("map_from_odom") is None or r.get("odom_base") is None:
            continue
        if previous is not None:
            before, after = _compose(previous, r["odom_base"]), _compose(r["map_from_odom"], r["odom_base"])
            out.append((math.hypot(after[0] - before[0], after[1] - before[1]), _yaw_err(after[2], before[2])))
        previous = r["map_from_odom"]
    j = np.asarray(out, float).reshape(-1, 2)
    return {"n": int(len(j)), "position_max_m": float(j[:, 0].max()) if len(j) else None,
            "yaw_max_rad": float(j[:, 1].max()) if len(j) else None, "position_over_0_05_m": int((j[:, 0] > 0.05).sum())}


def metrics(run: RunData, t0=None, t1=None) -> dict:
    processed = _processed(run, t0, t1)
    rp, ry = [], []
    for r in processed:
        if r.get("map_from_odom") is None or r.get("truth_base") is None:
            continue
        est, truth = _compose(r["map_from_odom"], r["odom_base"]), r["truth_base"]
        rp.append(math.hypot(est[0] - truth[0], est[1] - truth[1]))
        ry.append(_yaw_err(est[2], truth[2]))
    _, ap, ay = _applied(run, t0, t1)
    missions = [m for m in run.missions if (t0 is None or m["ended_s"] >= t0) and (t1 is None or m["ended_s"] <= t1)]
    done = [m for m in missions if m.get("outcome") == "done"]
    errors = [m["delivery_error_m"] for m in done if m.get("delivery_error_m") is not None]
    walls = [r.get("slam_wall_s") for r in processed]
    timed = bool(walls) and all(w is not None and math.isfinite(float(w)) for w in walls)
    delays = [float(r["applied_tick_s"]) - float(r["stamp_s"]) for r in processed if r.get("applied_tick_s") is not None]
    return {"keyframes": len(processed), "keyframes_timed": timed,
            "keyframe_max_s": max(float(w) for w in walls if w is not None) if timed else None,
            "applied_delay_s": ({"n": len(delays), "median": float(np.median(delays)),
                                 "p99": float(np.percentile(delays, 99)), "max": float(max(delays))}
                                if delays else {"n": 0}),
            "phases": {p: sum(1 for r in processed if r.get("phase") == p)
                       for p in sorted({r.get("phase") for r in processed if r.get("phase")})},
            "response": _stats(rp, ry), "applied": _stats(ap, ay), "correction_jumps": correction_jumps(processed),
            "missions_done": len(done), "missions_ended": len(missions),
            "delivery_error_max_m": max(errors) if errors else None}


def progress(run: RunData) -> dict:
    span = _span(run)
    if span is None:
        return {"ok": False, "observed_s": 0.0, "windows": []}
    start, end = span
    windows = []
    for k in range(WINDOWS):
        a, b = start + k * WINDOW_S, start + (k + 1) * WINDOW_S
        keyframes = len([r for r in _processed(run, a, b) if r["stamp_s"] < b])
        done = len([m for m in run.missions if a <= m["ended_s"] < b and m.get("outcome") == "done"])
        windows.append({"window": k, "from_s": a, "keyframes": keyframes, "done": done,
                        "ok": keyframes >= WINDOW_MIN_KEYFRAMES and done >= 1})
    observed = end - start
    return {"ok": observed >= OBSERVED_MIN_S and all(w["ok"] for w in windows), "observed_s": observed,
            "windows": windows}


COMPARED = (("applied", "position_median_m", "applied position median"), ("applied", "position_max_m", "applied position max"),
            ("applied", "yaw_median_rad", "applied heading median"), ("applied", "yaw_max_rad", "applied heading max"),
            ("response", "position_median_m", "response position median"),
            ("response", "position_max_m", "response position max"),
            ("response", "yaw_median_rad", "response heading median"), ("response", "yaw_max_rad", "response heading max"))


def verdict(l_run: RunData, r_run: RunData) -> dict:
    failed = [f"L record: {p}" for p in record_problems(l_run)[:5]] + [f"R record: {p}" for p in record_problems(r_run)[:5]]
    if failed:  # a damaged record is the verdict; the figures need whole samples (Codex round 3 P3)
        return {"ok": False, "failed": failed, "l": None, "progress": None, "r_failure": r_run.failure,
                "common_span_s": None, "l_after_r": None, "comparison": "records damaged"}
    whole = metrics(l_run)
    if not whole["keyframes_timed"] or not whole["keyframe_max_s"] < KEYFRAME_MAX_S:
        failed.append(f"L keyframe max {whole['keyframe_max_s']} s (< {KEYFRAME_MAX_S} s; every keyframe timed: "
                      f"{whole['keyframes_timed']})")
    for group in ("response", "applied"):
        if not whole[group]["finite"]:
            failed.append(f"L {group} pose samples missing or not finite")
    if l_run.failure and any(str(l_run.failure).startswith(b) for b in BAD_ENDS):
        failed.append(f"L end: {l_run.failure}")
    prog = progress(l_run)
    if not prog["ok"]:
        failed.append(f"L progress: observed {prog['observed_s']:.0f} s, windows "
                      f"{[w['window'] for w in prog['windows'] if not w['ok']]} short")
    out = {"l": whole, "progress": prog, "r_failure": r_run.failure, "common_span_s": None, "l_after_r": None,
           "l_failures": l_run.failures, "r_failures": r_run.failures}
    ls, rs = _span(l_run), _span(r_run)
    if ls is None or rs is None or min(ls[1], rs[1]) <= max(ls[0], rs[0]):
        out["comparison"] = "not comparable (no common sim span)"
        failed.append("L/R not comparable: no common sim span")
    else:
        c0, c1 = max(ls[0], rs[0]), min(ls[1], rs[1])
        out["common_span_s"] = [c0, c1]
        ml, mr = metrics(l_run, c0, c1), metrics(r_run, c0, c1)
        out["comparison"] = {"l": ml, "r": mr}
        for group in ("response", "applied"):
            if not (ml[group]["finite"] and mr[group]["finite"]):
                failed.append(f"{group} pose samples missing or not finite on the common span (L {ml[group]['n']}, "
                              f"R {mr[group]['n']})")
        for group, key, label in COMPARED:
            a, b = ml[group].get(key), mr[group].get(key)
            if a is not None and b is not None and a > b:
                failed.append(f"{label}: L {a} > R {b}")
        if ml["missions_done"] < mr["missions_done"]:
            failed.append(f"missions done on the common span: L {ml['missions_done']} < R {mr['missions_done']}")
    c_end = out["common_span_s"][1] if out["common_span_s"] is not None else None
    t_after, _, _ = _applied(l_run, c_end, None) if c_end is not None else _applied(l_run)
    if ls is not None and (c_end is None or (t_after > c_end).any()):
        t0 = c_end if c_end is not None else ls[0]
        t_a, ap, ay = _applied(l_run, t0, None)
        if c_end is not None:
            keep = t_a > c_end
            ap, ay = ap[keep], ay[keep]
        t_all, p_all, _ = _applied(l_run)
        # the first hour's two half-hours from the first response (not the run's last 30 min: S3 result review)
        first = p_all[(t_all >= ls[0]) & (t_all < ls[0] + HALF_S)]
        second = p_all[(t_all >= ls[0] + HALF_S) & (t_all < ls[0] + 2 * HALF_S)]
        after = {"from_s": t0, "position_max_m": float(ap.max()) if len(ap) else None,
                 "yaw_max_rad": float(ay.max()) if len(ay) else None,
                 "first_half_median_m": float(np.median(first)) if len(first) else None,
                 "second_half_median_m": float(np.median(second)) if len(second) else None}
        problems = []
        if after["position_max_m"] is None or not after["position_max_m"] <= POSITION_MAX_M:
            problems.append(f"L after R: applied position max {after['position_max_m']} > {POSITION_MAX_M} m")
        if after["yaw_max_rad"] is None or not after["yaw_max_rad"] <= YAW_MAX_RAD:
            problems.append(f"L after R: applied heading max {after['yaw_max_rad']} > {YAW_MAX_RAD} rad")
        r_cut = rs is None or bool(r_run.failure) or (rs[1] - rs[0]) < OBSERVED_MIN_S
        if r_cut and (after["first_half_median_m"] is None or after["second_half_median_m"] is None
                      or after["second_half_median_m"] > after["first_half_median_m"]):
            problems.append("L after R: the second half's applied-position median is above the first's")
        after["ok"] = not problems
        out["l_after_r"] = after
        failed += problems
    out["failed"] = failed
    out["ok"] = not failed
    return out


# --- the second confirmation's criteria (plan 5c, pre-registered after S3; ``verdict`` above stays S3's) --------------
DELIVERY_REASONS = ("Pallet missed the green destination center", "Delivered pallet is not grounded",
                    "Fork still inside delivered pallet")  # forklift_core.mission.delivery MISSED / NOT_GROUNDED / FORK_INSIDE
DRIFT_HALF_HOURS = 15  # V5's 0.15 m spread over an 8 h session's remaining fifteen half-hours
COMMON_MIN_S, R_MISSIONS_MIN = 600.0, 2
HOLD_GATE, HOLD_MIN_R, HOLD_TRAVEL_FLOOR_M, HOLD_PER_M_MIN_TRAVEL_M = 1.5, 10, 0.1, 0.3
SLAM_FAILURE = ("slam_", "localization_")  # plan 5c item 8 (a): SLAM-attributable mission failures
LAST_WINDOW_S = 600.0  # item 8 (d): the first hour's last ten minutes


def _timeline_intervals(run: RunData) -> list:
    """Closed [t_h, t_r] of the timeline: each release with the last unmatched hold event; an unclosed hold runs on."""
    out, open_ = [], []
    for e in sorted(run.holds, key=lambda e: e["time_s"]):
        if e.get("event") == "hold":
            open_.append(float(e["time_s"]))
        elif str(e.get("event", "")).startswith("release") and open_:
            out.append((open_.pop(), float(e["time_s"])))
            open_.clear()
    out += [(t, math.inf) for t in open_]
    return out


def tracking_mask(run: RunData) -> np.ndarray:
    """Plan 5c item 1: a control row is tracking only outside every closed hold-to-release interval of the timeline and
    between two records (by applied tick) that were both processed in tracking; everything else is holding."""
    rows = np.asarray(run.control, float).reshape(-1, 7)
    t = rows[:, 0]
    mask = np.ones(len(t), bool)
    for a, b in _timeline_intervals(run):
        mask &= ~((t >= a) & (t <= b))
    recs = sorted((r for r in run.records if r.get("applied_tick_s") is not None), key=lambda r: r["applied_tick_s"])
    ticks = np.asarray([float(r["applied_tick_s"]) for r in recs], float)
    tracking = np.asarray([r.get("mode") == "tracking" for r in recs], bool)
    if len(ticks):
        before = np.searchsorted(ticks, t, side="right") - 1
        after = before + 1
        ok_before = (before >= 0) & tracking[np.clip(before, 0, len(ticks) - 1)]
        ok_after = (after < len(ticks)) & tracking[np.clip(after, 0, len(ticks) - 1)]
        mask &= ok_before & ok_after
    else:
        mask[:] = False
    return mask


def hold_values(run: RunData) -> tuple[list, bool]:
    """Plan 5c item 7: the per-metre values of the holds that travelled >= 0.3 m (item 8 (c)), and the completeness."""
    holds, incomplete = hold_records(run)
    return [h["per_m"] for h in holds if h["per_m"] is not None], incomplete


def hold_records(run: RunData) -> tuple[list, bool]:
    """Plan 5c items 7 and 8: each closed hold's |release jump_rad| and |jump_m|, its odometry travel and the per-metre
    value |jump_rad| / max(travel, 0.1 m) when it travelled >= 0.3 m (else None); and whether the data is incomplete
    (an event-less hold with no holding record to rebuild it from, or a recorded holding span no release closes, the
    trailing unclosed one excepted)."""
    recs = sorted((r for r in run.records if r.get("applied_tick_s") is not None), key=lambda r: r["applied_tick_s"])
    ticks = [float(r["applied_tick_s"]) for r in recs]

    def travel(a, b):
        pts = [r["odom_base"] for r in recs if a <= float(r["applied_tick_s"]) <= b and r.get("odom_base")]
        return sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(pts, pts[1:]))

    values, incomplete, open_, prev_release = [], False, [], -math.inf
    closed = []
    for e in sorted(run.holds, key=lambda e: e["time_s"]):
        t = float(e["time_s"])
        if e.get("event") == "hold":
            open_.append(t)
            continue
        if not str(e.get("event", "")).startswith("release"):
            continue
        if open_:
            t_h = open_.pop()
            open_.clear()
        else:  # an event-less hold (start_clear_out): its start from the first holding record after the last release
            starts = [ti for ti, r in zip(ticks, recs) if prev_release < ti <= t and r.get("mode") == "holding"]
            if not starts:
                incomplete = True
                prev_release = t
                continue
            t_h = starts[0]
        closed.append((t_h, t))
        moved = travel(t_h, t)
        jump_rad, jump_m = abs(float(e.get("jump_rad") or 0.0)), abs(float(e.get("jump_m") or 0.0))
        values.append({"t_h": t_h, "t_r": t, "travel_m": moved, "jump_rad": jump_rad, "jump_m": jump_m,
                       "per_m": (jump_rad / max(moved, HOLD_TRAVEL_FLOOR_M)
                                 if moved >= HOLD_PER_M_MIN_TRAVEL_M else None)})
        prev_release = t
    # every recorded holding span must meet a closed interval, the trailing unclosed one excepted
    span_start = None
    for ti, r in zip(ticks + [math.inf], recs + [None]):
        holding = r is not None and r.get("mode") == "holding"
        if holding and span_start is None:
            span_start = ti
        elif not holding and span_start is not None:
            end = ti
            trailing = r is None
            if not trailing and not any(a <= end and b >= span_start for a, b in closed):
                incomplete = True
            span_start = None
    return values, incomplete


def _drift(run: RunData, ls) -> dict:
    t_all, p_all, _ = _applied(run)
    a = p_all[(t_all >= ls[0]) & (t_all < ls[0] + HALF_S)]
    b = p_all[(t_all >= ls[0] + HALF_S) & (t_all < ls[0] + 2 * HALF_S)]
    if not len(a) or not len(b):
        return {"ok": False, "reason": "a half-hour without samples"}
    last = p_all[(t_all >= ls[0] + 2 * HALF_S - LAST_WINDOW_S) & (t_all < ls[0] + 2 * HALF_S)]
    ma, mb = float(np.median(a)), float(np.median(b))
    mc = float(np.median(last)) if len(last) else math.inf
    allowance = (POSITION_MAX_M - ma) / DRIFT_HALF_HOURS
    return {"ok": mb - ma <= allowance and mc - ma <= allowance, "first_median_m": ma, "second_median_m": mb,
            "last_ten_median_m": mc, "allowance_m": allowance}


def _deliveries(run: RunData) -> list:
    """Plan 5c item 3: every attempt that reached the delivery check, under 0.08 m, no delivery-check failure."""
    problems = []
    for m in run.missions:
        reason, err = str(m.get("reason") or ""), m.get("delivery_error_m")
        reached = m.get("outcome") == "done" or err is not None or reason.startswith(DELIVERY_REASONS)
        if not reached:
            continue
        if reason.startswith(DELIVERY_REASONS):
            problems.append(f"delivery check failed in {m.get('mission_id')}: {reason}")
        if err is None or not math.isfinite(float(err)):
            problems.append(f"delivery error missing in {m.get('mission_id')}")
        elif not float(err) < 0.08:
            problems.append(f"delivery {float(err):.3f} m in {m.get('mission_id')} (< 0.08 m)")
    return problems


def pair_verdict(l_run: RunData, r_run: RunData) -> dict:
    """One scene/seed pair under plan 5c: L's absolute bounds always; the L <= R comparison only on a comparable pair
    (common span >= 600 s and R done >= 2 missions in it); 'not comparable' is not a failure."""
    failed = [f"L record: {p}" for p in record_problems(l_run)[:5]] + [f"R record: {p}" for p in record_problems(r_run)[:5]]
    if failed:
        return {"ok": False, "l_absolute_ok": False, "comparable": False, "failed": failed}
    whole = metrics(l_run)
    absolute = []
    if not whole["keyframes_timed"] or not whole["keyframe_max_s"] < KEYFRAME_MAX_S:
        absolute.append(f"L keyframe max {whole['keyframe_max_s']} s (< {KEYFRAME_MAX_S} s)")
    if l_run.failure and any(str(l_run.failure).startswith(b) for b in BAD_ENDS):
        absolute.append(f"L end: {l_run.failure}")
    prog = progress(l_run)
    if not prog["ok"]:
        absolute.append(f"L progress: observed {prog['observed_s']:.0f} s, windows "
                        f"{[w['window'] for w in prog['windows'] if not w['ok']]} short")
    absolute += [f"L {p}" for p in _deliveries(l_run)]
    for f in l_run.failures:  # item 8 (a): a SLAM-attributable mission failure is no pass; others are reported
        reason = str(f.get("reason") or "")
        if reason.startswith(SLAM_FAILURE) or "correction_out_of_bounds" in reason:
            absolute.append(f"L SLAM mission failure at {f.get('time_s')}: {reason}")
    mask = tracking_mask(l_run)
    _, lp, ly = _applied(l_run)
    if (~mask).any() and not float(ly[~mask].max()) <= YAW_MAX_RAD:
        absolute.append(f"L holding-row applied heading max {float(ly[~mask].max()):.4f} > {YAW_MAX_RAD} rad")
    ls, rs = _span(l_run), _span(r_run)
    drift = _drift(l_run, ls) if ls is not None else {"ok": False, "reason": "no span"}
    if not drift["ok"]:
        absolute.append(f"L drift: {drift}")
    out = {"l": whole, "progress": prog, "drift": drift, "l_failures": l_run.failures, "r_failures": r_run.failures,
           "r_failure": r_run.failure, "common_span_s": None, "comparable": False}
    comparison = []
    c_end = None
    if ls is not None and rs is not None and min(ls[1], rs[1]) > max(ls[0], rs[0]):
        c0, c1 = max(ls[0], rs[0]), min(ls[1], rs[1])
        out["common_span_s"] = [c0, c1]
        r_done = {m["mission_id"] for m in r_run.missions if m.get("outcome") == "done" and c0 <= m["ended_s"] <= c1}
        out["comparable"] = (c1 - c0) >= COMMON_MIN_S and len(r_done) >= R_MISSIONS_MIN
        if out["comparable"]:
            c_end = c1
            ml, mr = metrics(l_run, c0, c1), metrics(r_run, c0, c1)
            for label, a, b in (("applied position median", ml["applied"]["position_median_m"], mr["applied"]["position_median_m"]),
                                ("applied position max", ml["applied"]["position_max_m"], mr["applied"]["position_max_m"]),
                                ("response position median", ml["response"]["position_median_m"], mr["response"]["position_median_m"]),
                                ("response position max", ml["response"]["position_max_m"], mr["response"]["position_max_m"]),
                                ("response heading median", ml["response"]["yaw_median_rad"], mr["response"]["yaw_median_rad"]),
                                ("response heading max", ml["response"]["yaw_max_rad"], mr["response"]["yaw_max_rad"])):
                if a > b:
                    comparison.append(f"{label}: L {a} > R {b}")
            rmask = tracking_mask(r_run)
            tl, _, yl = _applied(l_run)
            tr, _, yr = _applied(r_run)
            sl = mask & (tl >= c0) & (tl <= c1)
            sr = rmask & (tr >= c0) & (tr <= c1)
            if not sl.any() or not sr.any():
                comparison.append("tracking rows missing on the common span")
            else:
                for label, a, b in (("tracking applied heading median", float(np.median(yl[sl])), float(np.median(yr[sr]))),
                                    ("tracking applied heading max", float(yl[sl].max()), float(yr[sr].max()))):
                    if a > b:
                        comparison.append(f"{label}: L {a} > R {b}")
            if ml["missions_done"] < mr["missions_done"]:
                comparison.append(f"missions done on the common span: L {ml['missions_done']} < R {mr['missions_done']}")
            out["comparison"] = {"l": ml, "r": mr}
    # V5 on L where no comparison covers it: past a comparable span's end, or the whole run when not comparable
    t_l = _applied(l_run)[0]
    beyond = (t_l > c_end) if c_end is not None else np.ones(len(t_l), bool)
    if beyond.any():
        if not float(lp[beyond].max()) <= POSITION_MAX_M:
            absolute.append(f"L applied position max {float(lp[beyond].max()):.3f} > {POSITION_MAX_M} m (V5)")
        if not float(ly[beyond].max()) <= YAW_MAX_RAD:
            absolute.append(f"L applied heading max {float(ly[beyond].max()):.4f} > {YAW_MAX_RAD} rad (V5)")
    out["l_absolute_ok"] = not absolute
    out["comparison_ok"] = not comparison if out["comparable"] else None
    out["failed"] = absolute + comparison
    out["ok"] = not absolute and (not out["comparable"] or not comparison)
    return out


def scene_gate(l_runs: list, r_runs: list) -> dict:
    """Plan 5c item 7: pooled per-hold values of a scene's L and R runs; 'fail' when L's median or p90 (method
    'higher') passes 1.5 x R's, 'incomplete' / 'insufficient' (no pass) when the data cannot say."""
    pooled = {"l": {}, "r": {}}
    incomplete = False
    for side, runs in (("l", l_runs), ("r", r_runs)):
        for run in runs:
            holds, inc = hold_records(run)
            incomplete |= inc
            pooled[side].setdefault("per_m", []).extend(h["per_m"] for h in holds if h["per_m"] is not None)
            pooled[side].setdefault("jump_rad", []).extend(h["jump_rad"] for h in holds)
            pooled[side].setdefault("jump_m", []).extend(h["jump_m"] for h in holds)
            pos, yaw = _hold_responses(run)  # item 8 (b): responses processed during holds
            pooled[side].setdefault("hold_response_position", []).extend(pos)
            pooled[side].setdefault("hold_response_yaw", []).extend(yaw)
    out = {"counts": {k: [len(pooled["l"].get(k, [])), len(pooled["r"].get(k, []))] for k in pooled["r"]}}
    if incomplete:
        return {**out, "state": "incomplete"}
    stats, short = {}, []
    for key in ("per_m", "jump_rad", "jump_m", "hold_response_position", "hold_response_yaw"):
        lv, rv = pooled["l"].get(key, []), pooled["r"].get(key, [])
        if not lv:  # no L sample of this kind (e.g. no hold travelled 0.3 m): nothing of L's to judge
            continue
        if len(rv) < HOLD_MIN_R:
            short.append(key)
            continue
        for name, fn in (("median", np.median), ("p90", lambda v: np.percentile(v, 90, method="higher"))):
            a, b = float(fn(lv)), float(fn(rv))
            if b == 0.0:
                short.append(f"{key} {name}")
                continue
            stats.setdefault(key, {})[name] = {"l": a, "r": b, "ratio": a / b}
    out["stats"] = stats
    if short:
        return {**out, "state": "insufficient", "short": short}
    out["state"] = ("fail" if any(s["ratio"] > HOLD_GATE for v in stats.values() for s in v.values()) else "ok")
    return out


def _hold_responses(run: RunData) -> tuple[list, list]:
    """Position and heading error of every processed response handled while holding (plan 5c item 8 (b))."""
    pos, yaw = [], []
    for r in run.records:
        if r.get("status") != "processed" or r.get("mode") != "holding" or not r.get("map_from_odom"):
            continue
        est, truth = _compose(r["map_from_odom"], r["odom_base"]), r["truth_base"]
        pos.append(math.hypot(est[0] - truth[0], est[1] - truth[1]))
        yaw.append(_yaw_err(est[2], truth[2]))
    return pos, yaw


def overall(scenes: dict) -> dict:
    """Plan 5c item 6: every pair's L on its absolute bounds, a comparable pair in every scene, every comparable pair
    passing, every scene's hold gate 'ok'."""
    failed = []
    for name, (pairs, gate) in scenes.items():
        if not all(p.get("l_absolute_ok") for p in pairs):
            failed.append(f"{name}: an L outside its absolute bounds")
        comparable = [p for p in pairs if p.get("comparable")]
        if not comparable:
            failed.append(f"{name}: no comparable pair (absolute bounds only)")
        if not all(p.get("ok") for p in comparable):
            failed.append(f"{name}: a comparable pair fails")
        if gate.get("state") != "ok":
            failed.append(f"{name}: hold-drift gate {gate.get('state')}")
    return {"ok": not failed, "failed": failed}


def load(run: Path) -> RunData:
    """An epoch's run folder: result.json's failure, the spooled slam.records / slam.control, the live log's missions."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from forklift_core.live.spool import load_list, read_live_log  # noqa: PLC0415

    result = json.loads((run / "result.json").read_text()) if (run / "result.json").exists() else {}
    missions = []
    for entry in read_live_log(run):
        end = entry.get("mission_end")
        if end is not None:
            missions.append({**end, "delivery_error_m": entry.get("delivery_error_m")})
    return RunData(records=load_list(run / "spool", "slam.records"), control=load_list(run / "spool", "slam.control"),
                   missions=missions, failure=result.get("failure_reason"), failures=failures_of(result),
                   holds=list((result.get("slam_summary") or {}).get("holds") or []))


def judge(spec: dict, loader=None) -> dict:
    """The second confirmation (plan 5c): ``spec`` = {"scenes": {name: {"pairs": [[L_RUN, R_RUN], ...],
    "extra_r": [R_RUN, ...]}}} -- the pairs as judged (a re-run R in place of a not comparable one), the replaced
    original R runs in ``extra_r`` (their holds still count in the gate)."""
    loader = loader or (lambda p: load(Path(p)))
    scenes, out = {}, {"scenes": {}}
    for name, s in spec["scenes"].items():
        pairs = [(loader(lp), loader(rp)) for lp, rp in s["pairs"]]
        extra = [loader(p) for p in s.get("extra_r", [])]
        verdicts = [pair_verdict(lr, rr) for lr, rr in pairs]
        gate = scene_gate([lr for lr, _ in pairs], [rr for _, rr in pairs] + extra)
        scenes[name] = (verdicts, gate)
        out["scenes"][name] = {"pairs": verdicts, "gate": gate}
    out["overall"] = overall(scenes)
    return out


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv[:1] == ["judge"]:  # the second confirmation, plan 5c
        jp = argparse.ArgumentParser(prog="slam_live_compare.py judge")
        jp.add_argument("spec", type=Path)
        jp.add_argument("--json", type=Path, default=None)
        ja = jp.parse_args(argv[1:])
        out = judge(json.loads(ja.spec.read_text()))
        if ja.json is not None:
            ja.json.write_text(json.dumps(out, indent=1, default=str) + "\n")
        print("PASS" if out["overall"]["ok"] else "FAIL", "; ".join(out["overall"]["failed"]))
        for name, s in out["scenes"].items():
            for i, p in enumerate(s["pairs"]):
                print(f"  {name} pair {i}: ok {p['ok']} comparable {p['comparable']} | {'; '.join(p['failed'])[:300]}")
            print(f"  {name} gate: {s['gate'].get('state')} {s['gate'].get('short', '')}")
        return 0 if out["overall"]["ok"] else 1
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0] + " (two runs: S3's verdict; 'judge SPEC': 5c)")
    ap.add_argument("l_run", type=Path)
    ap.add_argument("r_run", type=Path)
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args(argv)
    out = verdict(load(a.l_run), load(a.r_run))
    if a.json is not None:
        a.json.write_text(json.dumps(out, indent=1) + "\n")
    print("PASS" if out["ok"] else "FAIL", "; ".join(out["failed"]),
          "| common span", out["common_span_s"], "| R end", out["r_failure"],
          "| R mission failures", [f.get("reason") for f in out.get("r_failures") or []])
    return 0 if out["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
