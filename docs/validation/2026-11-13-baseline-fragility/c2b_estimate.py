"""C2(b) stage 1 (docs/plans/2026-11-13-c2b-hold-release.md 2): a records-only estimate of the seat offset at lift had
the truck accepted SLAM corrections after the near capture until the end-of-leg hold, with the target fixed in the map.

    python c2b_estimate.py DECOMP.json --raw-root DIR --out C2B.json [--summary C2B.md]

DECOMP.json is seat_decompose.py's output; under DIR, <root>/<name>/run/ holds result.json, slam_control.npy,
slam_log.npz and slam_records.json (the synchronous record: per scan stamp_s, status processed/skipped, map_from_odom = the tracker's
latest received correction, applied_map_from_odom).

Per run (the decomposition's judged / not-selected runs, i.e. reached lift and not held):
- M_hold: the applied correction from the capture to lift (must be constant, else not judgeable).
- t_k: the first control tick after the capture where the approach tracker's remaining arc is at most
  0.5 + v^2 / (2 * 0.3) (the existing end-of-leg hold, plan premise 3). The remaining arc is the approach path's
  (result.paths.approach) length past the estimate's projection, progress never going back; v is the runner's
  tracker_speed: the odometry interval speed (slam_log wheel increments x radius / dt) averaged over 0.1 s, kept just
  above 0.012 m/s while it is below. The estimate's own travel to the insert entry gives a second t_k, reported as a
  sensitivity (whether it selects another correction). An exploratory t at 1.0 m is also reported.
- M(t_k): the last record's map_from_odom with stamp <= t_k; none within 0.25 s -> not judgeable.
- Delta = M(t_k) * M_hold^-1; W' = Delta * W_lift (W_lift = E_lift * A_lift^-1); S_C2b = lateral(W'(V), E_lift), and
  the pallet's yaw relative to the truck at lift the same way. Delta = identity gives S_C2b = S (checked).
- The jumps as the code returns them (the base position under the two corrections at the same odometry pose, and the
  wrapped yaw difference), at every change of the applied correction from the release at the capture to t_k; the
  conservative variant uses the largest of them, not only the end-to-end one.
- Also: replies received from the capture to t_k and their spread, the age of the held correction when the hold
  began, and the received correction's error against the truth at t_k.
The plan's 2.2 rule is applied as written: the mapping exploratory pass needs all of its five conditions; not
judgeable runs count as unchanged. Results are a conditional end-point estimate on mapping records, not a bound on the
localization effect. Reads only.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

ACCEL_MPS2 = 0.3
END_WINDOW_M = 0.5
EXPLORE_M = 1.0
MAX_AGE_S = 0.25
JUMP_M, JUMP_RAD = 0.02, 0.02
SELECT_M = 0.010
TIME_TOL_S = 1e-6


def se2(p) -> np.ndarray:
    x, y, yaw = (float(v) for v in p[:3])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def pose_of(m: np.ndarray) -> tuple[float, float, float]:
    return float(m[0, 2]), float(m[1, 2]), math.atan2(float(m[1, 0]), float(m[0, 0]))


def lateral(point, pose) -> float:
    x, y, yaw = (float(v) for v in pose[:3])
    return -math.sin(yaw) * (float(point[0]) - x) + math.cos(yaw) * (float(point[1]) - y)


def wrap(a: float) -> float:
    return math.remainder(a, math.tau)


STOP_SPEED_MPS = 0.012
SPEED_WINDOW_TICKS = 12


def tracker_speeds(log, wheel_radius_m: float) -> tuple[np.ndarray, np.ndarray]:
    """(stamps, tracker_speed): the odometry interval speed averaged over 0.1 s, kept just above 0.012 m/s while
    below (StopDetector.tracker_speed, the stopped state not modelled: the approach does not stop)."""
    stamps = np.asarray(log["joint_stamps_s"], dtype=float)
    inc = np.asarray(log["wheel_increments_rad"], dtype=float).mean(axis=1) * wheel_radius_m
    speed = np.concatenate([[0.0], inc[1:] / np.diff(stamps)])
    kernel = np.ones(SPEED_WINDOW_TICKS) / SPEED_WINDOW_TICKS
    mean = np.convolve(speed, kernel, mode="full")[: len(speed)]
    mean[: SPEED_WINDOW_TICKS - 1] = [speed[: k + 1].mean() for k in range(SPEED_WINDOW_TICKS - 1)]
    floor = np.abs(mean) <= STOP_SPEED_MPS
    mean[floor] = STOP_SPEED_MPS * 1.01
    return stamps, mean


def path_remaining(path_xy: np.ndarray, points_xy: np.ndarray) -> np.ndarray:
    """The remaining arc of a polyline past each point's projection, progress monotone (the tracker never goes
    back)."""
    seg = np.diff(path_xy, axis=0)
    seg_len = np.hypot(seg[:, 0], seg[:, 1])
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    total, progress, out, i0 = cum[-1], 0.0, [], 0
    for p in points_xy:
        best, best_d = progress, None
        for i in range(i0, len(seg)):
            if seg_len[i] == 0:
                continue
            u = min(1.0, max(0.0, float(np.dot(p - path_xy[i], seg[i]) / seg_len[i] ** 2)))
            d = float(np.hypot(*(path_xy[i] + u * seg[i] - p)))
            if best_d is None or d < best_d:
                best_d, best = d, cum[i] + u * seg_len[i]
        if best >= progress:
            progress = best
            i0 = int(np.searchsorted(cum, progress, side="right")) - 1
            i0 = max(0, min(i0, len(seg) - 1))
        out.append(total - progress)
    return np.array(out)


def end_hold_time_tracked(sc: np.ndarray, t_cap: float, t_ins: float, path_xy: np.ndarray, speed_t: np.ndarray,
                          speed_v: np.ndarray, window_m: float | None = None) -> float | None:
    """The first control tick after the capture whose approach remaining arc is within the end window."""
    rows = sc[(sc[:, 0] > t_cap + TIME_TOL_S) & (sc[:, 0] <= t_ins + TIME_TOL_S)]
    if len(rows) < 2:
        return None
    remaining = path_remaining(path_xy, rows[:, 1:3])
    v = np.interp(rows[:, 0], speed_t, speed_v)
    limit = np.full(len(rows), window_m) if window_m is not None else END_WINDOW_M + v ** 2 / (2 * ACCEL_MPS2)
    hit = np.nonzero(remaining <= limit)[0]
    return float(rows[hit[0], 0]) if len(hit) else None


def applied_jumps(records: list, t_cap: float, t_k: float, M_hold, sc: np.ndarray, offset_m: float) -> list[dict]:
    """Every change of the applied correction from the release at the capture to t_k: (time, position jump, yaw
    jump) at that time's odometry pose."""
    current, out = list(M_hold), []
    first, _ = correction_at(records, t_cap)
    changes = [(t_cap, first)] if first is not None else []
    changes += [(r["stamp_s"], r["map_from_odom"]) for r in records
                if t_cap < r["stamp_s"] <= t_k + TIME_TOL_S and r["status"] == "processed"]
    for t, M in changes:
        if list(M) == current:
            continue
        pos, yaw = jump(M_hold, current, M, rear_row(sc, t)[1:4], offset_m)
        out.append({"t": t, "jump_m": pos, "jump_rad": yaw})
        current = list(M)
    return out


def end_hold_time(sc: np.ndarray, t_cap: float, t_ins: float, window_m: float | None = None) -> float | None:
    """The first row after the capture whose remaining estimate travel to the insert entry is within the end window
    (fixed window_m, or 0.5 + v^2/(2a) with the estimate's speed)."""
    rows = sc[(sc[:, 0] > t_cap + TIME_TOL_S) & (sc[:, 0] <= t_ins + TIME_TOL_S)]
    if len(rows) < 2:
        return None
    step = np.hypot(np.diff(rows[:, 1]), np.diff(rows[:, 2]))
    remaining = np.concatenate([np.cumsum(step[::-1])[::-1], [0.0]])
    speed = np.concatenate([[0.0], step / np.diff(rows[:, 0])])
    limit = (np.full(len(rows), window_m) if window_m is not None
             else END_WINDOW_M + speed ** 2 / (2 * ACCEL_MPS2))
    hit = np.nonzero(remaining <= limit)[0]
    return float(rows[hit[0], 0]) if len(hit) else None


def correction_at(records: list, t: float) -> tuple[list | None, float | None]:
    """The tracker's latest received correction at t and the stamp of the record carrying it (None if no record within
    MAX_AGE_S before t)."""
    last = None
    for r in records:
        if r["stamp_s"] <= t + TIME_TOL_S:
            last = r
        else:
            break
    if last is None or t - last["stamp_s"] > MAX_AGE_S:
        return None, None
    return last["map_from_odom"], last["stamp_s"]


def held_correction(records: list, t0: float, t1: float) -> list | None:
    """The applied correction over [t0, t1] if constant, else None."""
    applied = {tuple(r["applied_map_from_odom"]) for r in records if t0 - TIME_TOL_S <= r["stamp_s"] <= t1}
    return list(next(iter(applied))) if len(applied) == 1 else None


def rear_row(sc: np.ndarray, t: float) -> np.ndarray:
    return sc[int(np.argmin(np.abs(sc[:, 0] - t)))]


def counterfactual(V, V_yaw, E_lift, A_lift, M_hold, M_k) -> dict:
    """S and the relative pallet yaw at lift with the correction M_k instead of M_hold (target fixed in the map)."""
    delta = se2(M_k) @ np.linalg.inv(se2(M_hold))
    W = se2(E_lift) @ np.linalg.inv(se2(A_lift))
    Wp = delta @ W
    Vp = Wp @ np.array([float(V[0]), float(V[1]), 1.0])
    A_cf = pose_of(np.linalg.inv(Wp) @ se2(E_lift))
    return {"S_m": lateral(Vp[:2], E_lift), "yaw_rad": wrap(V_yaw - A_cf[2]),
            "S_actual_m": lateral(V, A_lift), "yaw_actual_rad": wrap(V_yaw - float(A_lift[2]))}


def jump(M_hold, M_prev, M_new, E_k, offset_m: float) -> tuple[float, float]:
    """The base position jump and the yaw jump the code would report when the applied correction goes from M_prev to
    M_new at the recorded estimate pose E_k (rear). The record's estimate is always M_hold * odom, so the odometry
    pose is recovered with M_hold, then both corrections are applied to that same pose."""
    odom = np.linalg.inv(se2(M_hold)) @ se2(E_k)
    base = se2([abs(offset_m), 0.0, 0.0])
    a = pose_of(se2(M_new) @ odom @ base)
    b = pose_of(se2(M_prev) @ odom @ base)
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(wrap(a[2] - b[2]))


def run_estimate(decomp: dict, result: dict, sc: np.ndarray, records: list, log=None) -> dict:
    out: dict = {"name": decomp["name"], "scene": decomp["scene"], "arm": decomp["arm"],
                 "S_m": decomp["terms_m"]["S"], "judgeable": False}
    t_cap, t_ins, t_lift = decomp["capture_t"], decomp["t_insert"], decomp["t_lift"]
    M_hold = held_correction(records, t_cap, t_lift)
    if M_hold is None:
        out["why"] = "held correction not constant from capture to lift"
        return out
    lift = next(s for s in result["samples"] if s.get("phase") == "lift" and s["time_s"] >= t_lift - TIME_TOL_S)
    V, V_yaw = decomp["V"], float(lift["pallet_yaw_rad"])
    offset = result["arguments"]["rear_axle_offset_m"]
    path = (result.get("paths") or {}).get("approach") or {}
    if log is None or not path.get("poses"):
        out["why"] = "no slam log or approach path for the end-hold rule"
        return out
    path_xy = np.asarray(path["poses"], dtype=float)[:, :2]
    radius = result["chassis_model"]["drive_geometry"]["wheel_radius_m"]
    speed_t, speed_v = tracker_speeds(log, radius)
    t_travel = end_hold_time(sc, t_cap, t_ins)
    for key, window in (("end_hold", None), ("explore_1m", EXPLORE_M)):
        t_k = end_hold_time_tracked(sc, t_cap, t_ins, path_xy, speed_t, speed_v, window)
        entry: dict = {"t_k": t_k}
        if t_k is None:
            entry["why"] = "no row inside the end window"
        else:
            M_k, stamp = correction_at(records, t_k)
            if M_k is None:
                entry["why"] = "no fresh correction at t_k"
            else:
                E_k = rear_row(sc, t_k)
                cf = counterfactual(V, V_yaw, decomp["E"], decomp["A"], M_hold, M_k)
                pos_jump, yaw_jump = jump(M_hold, M_hold, M_k, E_k[1:4], offset)
                odom_k = np.linalg.inv(se2(M_hold)) @ se2(E_k[1:4])
                rec = pose_of(se2(M_k) @ odom_k)
                truth = E_k[4:7]
                seq = applied_jumps(records, t_cap, t_k, M_hold, sc, offset)
                entry.update(
                    jumps=seq, max_jump_m=max((j["jump_m"] for j in seq), default=0.0),
                    max_jump_rad=max((j["jump_rad"] for j in seq), default=0.0),
                    correction_stamp_s=stamp, S_C2b_m=cf["S_m"], yaw_C2b_rad=cf["yaw_rad"],
                    yaw_actual_rad=cf["yaw_actual_rad"], check_identity_m=counterfactual(
                        V, V_yaw, decomp["E"], decomp["A"], M_hold, M_hold)["S_m"] - cf["S_actual_m"],
                    jump_m=pos_jump, jump_rad=yaw_jump,
                    over_jump=bool(any(j["jump_m"] > JUMP_M or j["jump_rad"] > JUMP_RAD for j in seq)),
                    received_error_lateral_m=lateral(rec, truth),
                    received_error_yaw_rad=wrap(rec[2] - float(truth[2])),
                    replies_to_t_k=sum(1 for r in records if t_cap < r["stamp_s"] <= t_k),
                    processed_to_t_k=sum(1 for r in records if t_cap < r["stamp_s"] <= t_k and r["status"] == "processed"),
                    last_received_age_s=t_k - stamp,
                    last_processed_age_s=next((t_k - r["stamp_s"] for r in reversed(records)
                                               if r["stamp_s"] <= t_k + TIME_TOL_S and r["status"] == "processed"),
                                              None))
                window_rows = [r["map_from_odom"] for r in records
                               if t_cap < r["stamp_s"] <= t_k and r["status"] == "processed"]
                if len(window_rows) >= 2:
                    arr = np.array(window_rows)
                    entry["reply_spread"] = {"x_m": float(np.ptp(arr[:, 0])), "y_m": float(np.ptp(arr[:, 1])),
                                             "yaw_rad": float(np.ptp(arr[:, 2]))}
        out[key] = entry
    out["end_hold"]["t_k_travel_sum"] = t_travel
    if t_travel is not None and out["end_hold"].get("t_k") is not None:
        a_, _ = correction_at(records, t_travel)
        b_, _ = correction_at(records, out["end_hold"]["t_k"])
        out["end_hold"]["travel_sum_selects_other_correction"] = a_ != b_
    holds = [h for h in (result.get("slam_summary") or {}).get("holds", [])
             if h.get("event") == "hold" and h["time_s"] <= t_cap + TIME_TOL_S]
    if holds:
        t_hold = holds[-1]["time_s"]
        prior = [r for r in records if r["stamp_s"] <= t_hold + TIME_TOL_S and r["status"] == "processed"]
        # the held correction's age when the hold began: since the last new (processed) correction
        out["held_correction_processed_age_s"] = (t_hold - prior[-1]["stamp_s"]) if prior else None
    out["judgeable"] = "S_C2b_m" in out["end_hold"]
    return out


def judge(runs: list[dict], key: str = "end_hold") -> dict:
    """Plan 2.2. A not judgeable run counts as unchanged; the conservative variant also treats over-jump runs as
    unchanged."""
    def s_after(r, conservative):
        e = r.get(key) or {}
        if "S_C2b_m" not in e or (conservative and e["over_jump"]):
            return r["S_m"]
        return e["S_C2b_m"]

    def rule(conservative):
        sel = [r for r in runs if abs(r["S_m"]) > SELECT_M]
        better = sum(abs(s_after(r, conservative)) < abs(r["S_m"]) for r in sel)
        med_before = float(np.median([abs(r["S_m"]) for r in sel])) if sel else None
        med_after = float(np.median([abs(s_after(r, conservative)) for r in sel])) if sel else None
        bad_after = sum(abs(s_after(r, conservative)) > SELECT_M for r in runs)
        worse = [abs(s_after(r, conservative)) - abs(r["S_m"]) for r in runs]
        worse10 = sum(w > 0.010 for w in worse)
        c = {"1_better": better >= 22, "2_median_drop": bool(sel) and med_before - med_after >= 0.005,
             "3_fewer_over_10mm": bad_after < len(sel), "4_worse_by_10mm_at_most_4": worse10 <= 4}
        return {"selected": len(sel), "better": better, "median_before_m": med_before, "median_after_m": med_after,
                "over_10mm_after": bad_after, "worse_by_10mm": worse10, "largest_worsening_m": max(worse) if worse else None,
                "conditions": c, "all": all(c.values())}

    plain, conservative = rule(False), rule(True)
    return {"plain": plain, "conservative": conservative, "pass": plain["all"] and conservative["all"],
            "not_judgeable": sum("S_C2b_m" not in (r.get(key) or {}) for r in runs),
            "over_jump": sum(bool((r.get(key) or {}).get("over_jump")) for r in runs)}


def by_group(runs: list[dict], key: str) -> dict:
    groups: dict = {}
    for r in runs:
        for g in (f"scene:{r['scene']}", "arm:B0" if r["arm"] == "B0" else "arm:perturbed"):
            groups.setdefault(g, []).append(r)
    return {g: judge(rs, key) for g, rs in sorted(groups.items())}


def analyse(decomp_runs: list[dict], raw_root: Path) -> dict:
    runs = []
    for d in decomp_runs:
        run = raw_root / d["root"] / d["name"] / "run"
        result = json.loads((run / "result.json").read_text())
        with np.load(run / "slam_log.npz") as log:
            runs.append(run_estimate(d, result, np.load(run / "slam_control.npy"),
                                     json.loads((run / "slam_records.json").read_text()),
                                     {k: log[k] for k in ("joint_stamps_s", "wheel_increments_rad")}))
    return {"n": len(runs), "end_hold": judge(runs, "end_hold"), "explore_1m": judge(runs, "explore_1m"),
            "groups_end_hold": by_group(runs, "end_hold"), "runs": runs}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("decomp", type=Path)
    ap.add_argument("--raw-root", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    runs = [r for r in json.loads(args.decomp.read_text())["runs"]
            if r["judgement"]["status"] in ("judged", "not_selected")]
    args.out.write_text(json.dumps(analyse(runs, args.raw_root), indent=1, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
