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
            "applied_delay_s": ({"n": len(delays), "median": float(np.median(delays)), "max": float(max(delays))}
                                if delays else {"n": 0}),
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
    out = {"l": whole, "progress": prog, "r_failure": r_run.failure, "common_span_s": None, "l_after_r": None}
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
        first = p_all[t_all < ls[0] + HALF_S]
        second = p_all[t_all >= ls[1] - HALF_S]
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
                   missions=missions, failure=result.get("failure_reason"))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("l_run", type=Path)
    ap.add_argument("r_run", type=Path)
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args(argv)
    out = verdict(load(a.l_run), load(a.r_run))
    if a.json is not None:
        a.json.write_text(json.dumps(out, indent=1) + "\n")
    print("PASS" if out["ok"] else "FAIL", "; ".join(out["failed"]),
          "| common span", out["common_span_s"], "| R end", out["r_failure"])
    return 0 if out["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
