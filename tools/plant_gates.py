"""New plant candidate gates V2 and V3 measured from joint angles (EPAL 6 fork spacing plan, 13th draft
gates as revised by the user on 2026-10-15: the engine-reported joint velocity under TGS is not the
derivative of the joint angle, so the steady window and the rolling ratio are read from angles).

V2 steady window: in a segment's second half, the longest run of consecutive samples in which every
adjacent pair has, for both steering joints, |dq| / dt < 0.02 rad/s and |d(q - target)| / dt < 0.005
rad/s (adjacent differences -- a central difference hides a tick-to-tick alternation); used when it
lasts >= 0.3 s, otherwise the segment is unverified. In the window: the body curvature from the truth
rear pose, the curvature implied by the two steering joints' mean actual angles, their residual, and the
left/right steering error.

V3 rolling ratio: in the same kind of window on a straight, the rear axle's longitudinal travel (truth)
over the two rear wheels' angle advance x wheel radius.

    python tools/plant_gates.py probe_dir [probe_dir ...] --output gates.json

The 16th draft (16.1, fixed before its results; plan block "16판") replaces both definitions: V2 on
0.1 s blocks with a trailing window, per-block curvature residuals and per-sample left/right checks
(v16_window, v16_judge); V3 as a calibrated rolling gate on valid straight windows (v16_straight_window,
v16_rolling) -- calibrated on the long set's unloaded 0.25 m/s forward straight, reproduced on the
standard set, validated on the other straights and on the loaded transport of V5 missions:

    python tools/plant_gates.py v16 --long LONG_RUN --standard STANDARD_RUN [--missions RUN ...]
        [--r-cal R] --output gates16.json
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

QDOT_MAX_RAD_S = 0.02
SLOPE_MAX_RAD_S = 0.005
MIN_STEADY_S = 0.3


def steady_span(q: np.ndarray, target: np.ndarray, dt: float, *, qdot_max: float = QDOT_MAX_RAD_S,
                slope_max: float = SLOPE_MAX_RAD_S, min_s: float = MIN_STEADY_S) -> tuple[int, int] | None:
    """(start, stop) sample indices of the longest steady run in rows q (N, 2) / target (N, 2), or None.
    A run of k samples holds k - 1 adjacent pairs, all of which must pass."""
    q, target = np.asarray(q, float), np.asarray(target, float)
    if len(q) < 2:
        return None
    rate = np.abs(np.diff(q, axis=0)) / dt
    slope = np.abs(np.diff(q - target, axis=0)) / dt
    ok = np.all((rate < qdot_max) & (slope < slope_max), axis=1)  # one flag per adjacent pair
    best, start, best_span = 0, None, None
    for i, flag in enumerate(np.r_[ok, False]):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            if i - start > best:
                best, best_span = i - start, (start, i + 1)  # pairs start..i-1 -> samples start..i
            start = None
    if best_span is None or (best_span[1] - best_span[0] - 1) * dt < min_s:
        return None
    return best_span


def wheel_curvatures(q_left: float, q_right: float, wheelbase_m: float, track_m: float) -> tuple[float, float]:
    tl, tr = math.tan(q_left), math.tan(q_right)
    half = track_m / 2
    return tl / (wheelbase_m + tl * half), tr / (wheelbase_m - tr * half)


def window_metrics(rows: np.ndarray, c: dict, dt: float, *, wheelbase_m: float, track_m: float, radius_m: float,
                   direction: float) -> dict:
    yaw = np.unwrap(rows[:, c["rear_yaw"]])
    dx, dy = np.diff(rows[:, c["rear_x"]]), np.diff(rows[:, c["rear_y"]])
    travel = float(np.sum(np.hypot(dx, dy)))
    k_true = direction * float(yaw[-1] - yaw[0]) / travel if travel > 0.02 else None
    q_l, q_r = float(np.mean(rows[:, c["q_FL"]])), float(np.mean(rows[:, c["q_FR"]]))
    k_l, k_r = wheel_curvatures(q_l, q_r, wheelbase_m, track_m)
    out = {
        "steady_s": (len(rows) - 1) * dt,
        "kappa_true": k_true,
        "kappa_FL": k_l, "kappa_FR": k_r, "kappa_avg": (k_l + k_r) / 2,
        "residual": None if k_true is None else k_true - (k_l + k_r) / 2,
        "err_FL_mrad": float(np.mean(rows[:, c["q_FL"]] - rows[:, c["target_FL"]])) * 1e3,
        "err_FR_mrad": float(np.mean(rows[:, c["q_FR"]] - rows[:, c["target_FR"]])) * 1e3,
    }
    if "p_RL" in c:
        lon = float(np.sum(dx * np.cos(yaw[1:]) + dy * np.sin(yaw[1:])))
        advance = float(np.mean([np.unwrap(rows[:, c[k]])[-1] - np.unwrap(rows[:, c[k]])[0] for k in ("p_RL", "p_RR")]))
        out["rolling_ratio"] = lon / (advance * radius_m) if abs(advance) > 1e-6 else None
    return out


def evaluate(run: Path) -> dict:
    run = Path(run)
    if (run / "run").is_dir():
        run = run / "run"
    result = json.loads((run / "result.json").read_text())
    z = np.load(run / "steering_probe.npz")
    rec = z["record"]
    c = {str(k): i for i, k in enumerate(z["columns"])}
    dt = float(result["steering_probe"]["dt_s"])
    geom = (result.get("chassis_model") or {}).get("drive_geometry") or {}
    wheelbase, track, radius = float(geom["wheelbase_m"]), float(geom["track_m"]), float(geom["wheel_radius_m"])
    segments = []
    for s in result["steering_probe"]["segments"]:
        if s["speed_mps"] == 0:
            continue
        rows = rec[rec[:, 1] == s["segment"]]
        late = rows[len(rows) // 2:]
        span = steady_span(late[:, [c["q_FL"], c["q_FR"]]], late[:, [c["target_FL"], c["target_FR"]]], dt)
        entry = {"segment": s["segment"], "label": s["label"], "speed_mps": s["speed_mps"], "kappa_cmd": s["kappa_cmd"],
                 "steady": None}
        if span is not None:
            entry["steady"] = window_metrics(late[span[0]:span[1]], c, dt, wheelbase_m=wheelbase, track_m=track,
                                             radius_m=radius, direction=1.0 if s["speed_mps"] > 0 else -1.0)
        segments.append(entry)
    return {"run": str(run), "settings": result.get("settings_synthetic"), "has_wheel_angles": "p_RL" in c,
            "segments": segments}


# --- 16th draft (docs/plans/2026-10-13-epal6-fork-spacing.md, "16판", fixed before results) -------------
BLOCK_S = 0.1
MIN_BLOCKS = 4
NEIGHBOUR_MAX_RAD = 0.0005
SPREAD_MAX_RAD = 0.0015
SLOPE_MAX_RAD_PER_S = 0.001
DEVIATION_MAX_RAD = 0.0031
RESIDUAL_MAX = 0.005
LR_DIFF_MAX_RAD = 0.010
KAPPA_MIN = 0.33
STRAIGHT_Q_MAX_RAD = 0.0031
STRAIGHT_YAW_MAX_RAD = 0.005
STRAIGHT_SPEED_RATIO_MAX = 1.05
STRAIGHT_TRAVEL_MIN_M = 0.20


def block_ticks(dt: float) -> int:
    return int(round(BLOCK_S / dt))


def v16_window(q: np.ndarray, dt: float) -> tuple[int, int] | None:
    """(start, stop) sample indices of the V2 window in rows q (N, 2) that start at the segment's
    second half: the longest run of whole 0.1 s blocks that ends at the last whole block, >= 4
    blocks, with every neighbouring block-mean change <= 0.5 mrad, block-mean spread <= 1.5 mrad,
    block-mean least-squares slope <= 1 mrad/s and every sample within 3.1 mrad of the window mean --
    each for both joints, bounds inclusive. None when no such run exists."""
    q = np.asarray(q, float)
    ticks = block_ticks(dt)
    nb = len(q) // ticks
    if nb < MIN_BLOCKS:
        return None
    q = q[: nb * ticks]
    means = q.reshape(nb, ticks, 2).mean(axis=1)
    centres = (np.arange(nb) + 0.5) * ticks * dt
    tol = 1e-12
    for k in range(nb, MIN_BLOCKS - 1, -1):
        j0 = nb - k
        m = means[j0:]
        if np.any(np.abs(np.diff(m, axis=0)) > NEIGHBOUR_MAX_RAD + tol):
            continue
        if np.any(m.max(axis=0) - m.min(axis=0) > SPREAD_MAX_RAD + tol):
            continue
        slopes = np.polyfit(centres[j0:], m, 1)[0]
        if np.any(np.abs(slopes) > SLOPE_MAX_RAD_PER_S + tol):
            continue
        rows = q[j0 * ticks:]
        if np.any(np.abs(rows - rows.mean(axis=0)) > DEVIATION_MAX_RAD + tol):
            continue
        return j0 * ticks, nb * ticks
    return None


def v16_judge(rows: np.ndarray, c: dict, dt: float, *, kappa_cmd: float, direction: float, wheelbase_m: float,
              track_m: float) -> dict:
    """The 16th-draft V2 criteria. rows = the row just before the window followed by the window's rows
    (whole blocks of 0.1 s). Every tick (adjacent-row interval) belongs to exactly one block: block j
    takes the ticks that end on its own rows, so its body curvature spans 12 ticks (120 Hz) and starts
    from the last row of the block before (the row before the window for the first block); its steering
    curvature is from the mean of its own rows. The window's body curvature spans all its ticks
    (blocks x 0.1 s) and the steering, error and left/right checks read the window's own rows."""
    rows = np.asarray(rows, float)
    ticks = block_ticks(dt)
    own = rows[1:]
    yaw = np.unwrap(rows[:, c["rear_yaw"]])

    def body_kappa(i0, i1):  # ticks from row i0 to row i1 (indices into rows, i1 inclusive)
        r = rows[i0:i1 + 1]
        travel = float(np.sum(np.hypot(np.diff(r[:, c["rear_x"]]), np.diff(r[:, c["rear_y"]]))))
        return direction * float(yaw[i1] - yaw[i0]) / travel if travel > 1e-6 else None

    def angle_kappa(r):
        k_l, k_r = wheel_curvatures(float(np.mean(r[:, c["q_FL"]])), float(np.mean(r[:, c["q_FR"]])), wheelbase_m, track_m)
        return (k_l + k_r) / 2

    k_true, k_angle = body_kappa(0, len(own)), angle_kappa(own)
    block_residuals = []
    for j in range(len(own) // ticks):
        kb = body_kappa(j * ticks, (j + 1) * ticks)
        block_residuals.append(None if kb is None else kb - angle_kappa(own[j * ticks:(j + 1) * ticks]))
    err_l = own[:, c["q_FL"]] - own[:, c["target_FL"]]
    err_r = own[:, c["q_FR"]] - own[:, c["target_FR"]]
    lr = np.abs(err_l - err_r)
    residual = None if k_true is None else k_true - k_angle
    checks = {
        "same_sign": k_true is not None and k_true * kappa_cmd > 0,
        "kappa_min": k_true is not None and abs(k_true) >= KAPPA_MIN,
        "residual_mean": residual is not None and abs(residual) <= RESIDUAL_MAX,
        "residual_blocks": all(r is not None and abs(r) <= RESIDUAL_MAX for r in block_residuals),
        "lr_mean": abs(float(np.mean(err_l - err_r))) <= LR_DIFF_MAX_RAD,
        "lr_samples": bool(np.all(lr <= LR_DIFF_MAX_RAD)),
    }
    return {"window_s": len(own) * dt, "blocks": len(own) // ticks, "kappa_true": k_true, "kappa_angles": k_angle,
            "residual": residual, "block_residuals": block_residuals,
            "block_residual_max": max((abs(r) for r in block_residuals if r is not None), default=None),
            "err_FL_mrad": float(np.mean(err_l)) * 1e3, "err_FR_mrad": float(np.mean(err_r)) * 1e3,
            "lr_sample_max_mrad": float(lr.max()) * 1e3, "checks": {k: bool(v) for k, v in checks.items()},
            "pass": bool(all(checks.values()))}


def v16_straight_window(rows: np.ndarray, c: dict, dt: float, *, angle_cols=("p_RL", "p_RR"),
                        steer_cols=("q_FL", "q_FR"), valid: np.ndarray | None = None,
                        direction: int | None = None) -> tuple[int, int] | None:
    """(start, stop) rows of the valid V3 straight window with the most truth travel, or None: both
    steering angles within 3.1 mrad of straight, both rear wheel angles moving the same one way on every
    tick, body yaw max - min <= 5 mrad, truth speed max / min <= 1.05 over the window's ticks and
    travel >= 0.20 m. Each valid window ending at a row is the longest one ending there (the yaw and
    speed limits only get easier as a window shrinks), so the best is found in one pass. valid: rows
    allowed (e.g. loaded); direction: +1 / -1 keeps ticks whose wheel angles grow / shrink only."""
    rows = np.asarray(rows, float)
    n = len(rows)
    if n < 2:
        return None
    straight = np.all(np.abs(rows[:, [c[k] for k in steer_cols]]) <= STRAIGHT_Q_MAX_RAD + 1e-12, axis=1)
    if valid is not None:
        straight &= np.asarray(valid, bool)
    steps = np.diff(np.column_stack([np.unwrap(rows[:, c[k]]) for k in angle_cols]), axis=0)
    sign = np.where(np.all(steps > 0, axis=1), 1, np.where(np.all(steps < 0, axis=1), -1, 0))
    if direction is not None:
        sign = np.where(sign == direction, sign, 0)
    yaw = np.unwrap(rows[:, c["rear_yaw"]])
    hop = np.hypot(np.diff(rows[:, c["rear_x"]]), np.diff(rows[:, c["rear_y"]]))
    speed = hop / dt
    best, best_travel = None, 0.0
    a = 0  # first row of the current window; the window is rows a..b (ticks a..b-1)
    for b in range(1, n):
        t = b - 1  # the tick ending at row b
        if not (straight[b] and straight[t] and sign[t] != 0 and speed[t] > 0):
            a = b
            continue
        if a < t and sign[t - 1] != sign[t]:
            a = t  # ticks a..t-1 share one sign; a reversal starts a new window at row t
        while a < b and (np.ptp(yaw[a:b + 1]) > STRAIGHT_YAW_MAX_RAD
                         or speed[a:b].max() / speed[a:b].min() > STRAIGHT_SPEED_RATIO_MAX):
            a += 1  # down to an empty window: a single tick is checked too (Codex 16.1, P3)
        if a == b:
            continue
        travel = float(hop[a:b].sum())
        if travel >= STRAIGHT_TRAVEL_MIN_M and travel > best_travel:
            best, best_travel = (a, b + 1), travel
    return best


def v16_rolling(rows: np.ndarray, c: dict, radius_m: float, *, angle_cols=("p_RL", "p_RR")) -> dict:
    """Ratios of the body's longitudinal truth travel -- each tick's displacement projected on that
    tick's mid heading, summed (a sideslip does not turn the wheels; Codex 16.1, P2) -- over each rear
    wheel's angle advance (end rows, not summed increments) x radius, and over their mean. lateral_m:
    the same sum across the heading, reported."""
    yaw = np.unwrap(rows[:, c["rear_yaw"]])
    mid = (yaw[1:] + yaw[:-1]) / 2
    dx, dy = np.diff(rows[:, c["rear_x"]]), np.diff(rows[:, c["rear_y"]])
    lon = float(np.sum(dx * np.cos(mid) + dy * np.sin(mid)))
    lat = float(np.sum(-dx * np.sin(mid) + dy * np.cos(mid)))
    adv = [float(np.unwrap(rows[:, c[k]])[-1] - np.unwrap(rows[:, c[k]])[0]) for k in angle_cols]
    return {"travel_m": lon, "lateral_m": lat, "ratio": float(lon / (np.mean(adv) * radius_m)),
            "ratio_per_wheel": [lon / (a * radius_m) for a in adv]}


V2_KAPPA = 0.36
V3_REPRO_TOL = 0.001
V3_VALID_TOL = 0.005
LOADED_LIFT_MIN_M = 0.15
LOADED_PALLET_RISE_MIN_M = 0.10  # the pallet's own lift above its start: off the floor, on the blades
LOADED_MARGIN_S = 0.1  # a row is loaded when every 0.1 s sample within this of it is


def load_probe(run: Path):
    run = Path(run)
    if (run / "run").is_dir():
        run = run / "run"
    result = json.loads((run / "result.json").read_text())
    z = np.load(run / "steering_probe.npz")
    c = {str(k): i for i, k in enumerate(z["columns"])}
    geom = (result.get("chassis_model") or {}).get("drive_geometry") or {}
    return run, result, z["record"], c, float(result["steering_probe"]["dt_s"]), geom


def v16_probe(run: Path) -> dict:
    """V2 on every +-0.36 arc and the V3 straight windows (raw ratio at the geometric radius and the
    effective radius lon / advance) on every moving straight of one probe run."""
    run, result, rec, c, dt, geom = load_probe(run)
    wheelbase, track, radius = float(geom["wheelbase_m"]), float(geom["track_m"]), float(geom["wheel_radius_m"])
    v2, v3 = [], []
    for s in result["steering_probe"]["segments"]:
        speed, kappa = float(s["speed_mps"]), float(s["kappa_cmd"])
        if speed == 0:
            continue
        seg = rec[rec[:, 1] == s["segment"]]
        head = {"segment": s["segment"], "label": s["label"], "speed_mps": speed, "kappa_cmd": kappa}
        if math.isclose(abs(kappa), V2_KAPPA, abs_tol=1e-9):
            off = len(seg) // 2
            span = v16_window(seg[off:, [c["q_FL"], c["q_FR"]]], dt)
            entry = dict(head, window=None)
            if span is not None:
                entry["window"] = [off + span[0], off + span[1]]
                entry.update(v16_judge(seg[off + span[0] - 1: off + span[1]], c, dt, kappa_cmd=kappa,
                                       direction=1.0 if speed > 0 else -1.0, wheelbase_m=wheelbase, track_m=track))
            v2.append(entry)
        elif kappa == 0:
            span = v16_straight_window(seg, c, dt)
            entry = dict(head, window=None)
            if span is not None:
                w = seg[span[0]:span[1]]
                eff = v16_rolling(w, c, 1.0)
                entry.update(window=list(span), window_s=(span[1] - span[0] - 1) * dt, travel_m=eff["travel_m"],
                             lateral_m=eff["lateral_m"],
                             r_eff_m=eff["ratio"], r_eff_per_wheel_m=eff["ratio_per_wheel"],
                             raw_ratio=eff["ratio"] / radius)
            v3.append(entry)
    return {"run": str(run), "dt_s": dt, "radius_m": radius, "settings": result.get("settings_synthetic"),
            "v2": v2, "v3": v3}


def mission_rows(run: Path):
    """Rows (rear_x, rear_y, rear_yaw, q_FL, q_FR, p_RL, p_RR) on every physics tick of a mission's
    slam_log, and each row's loaded flag: transport phase, lift >= 0.15 m and the pallet >= 0.10 m
    above its start, in every 0.1 s sample within 0.1 s of the row."""
    run = Path(run)
    if (run / "run").is_dir():
        run = run / "run"
    result = json.loads((run / "result.json").read_text())
    meta = json.loads((run / "meta.json").read_text())
    z = np.load(run / "slam_log.npz")
    geo = meta["odometry_geometry"]
    pose = np.asarray(z["base_pose_world"], float)
    w, x, y, zq = pose[:, 3], pose[:, 4], pose[:, 5], pose[:, 6]
    yaw = np.arctan2(2 * (w * zq + x * y), 1 - 2 * (y * y + zq * zq))
    ax = float(geo["rear_axle_x_in_base_m"])
    steer = np.asarray(z["steering_rad"], float)
    assert meta["steering_order"] == ["front_left", "front_right"], meta["steering_order"]
    order = meta["wheel_order"]
    angles = np.asarray(z["wheel_angles_rad"], float)
    rows = np.column_stack([pose[:, 0] + ax * np.cos(yaw), pose[:, 1] + ax * np.sin(yaw), yaw, steer[:, 0], steer[:, 1],
                            angles[:, order.index("rear_left")], angles[:, order.index("rear_right")]])
    c = {k: i for i, k in enumerate(("rear_x", "rear_y", "rear_yaw", "q_FL", "q_FR", "p_RL", "p_RR"))}
    samples = result["samples"]
    st = np.array([s_["time_s"] for s_ in samples])
    z0 = samples[0]["pallet_position_m"][2]
    ok = np.array([s_["phase"] == "transport" and s_["lift_m"] >= LOADED_LIFT_MIN_M
                   and s_["pallet_position_m"][2] - z0 >= LOADED_PALLET_RISE_MIN_M for s_ in samples])
    stamps = np.asarray(z["joint_stamps_s"], float)
    lo = np.searchsorted(st, stamps - LOADED_MARGIN_S, side="left")
    hi = np.searchsorted(st, stamps + LOADED_MARGIN_S, side="right")
    bad = np.r_[0, np.cumsum(~ok)]
    loaded = (hi > lo) & (bad[hi] - bad[lo] == 0)
    dt = float(np.median(np.diff(stamps)))
    # The geometric radius (raw ratio): a record made with --odometry-wheel-radius-m keeps it beside
    # the odometry's own; older records have only the geometric one.
    return run, rows, c, loaded, dt, float(geo.get("geometric_wheel_radius_m", geo["wheel_radius_m"]))


def v16_loaded(run: Path) -> dict:
    """The loaded V3 windows of one mission: per direction, the valid straight window with the most travel."""
    run, rows, c, loaded, dt, radius = mission_rows(run)
    out = {"run": str(run), "dt_s": dt, "loaded_s": float(loaded.sum()) * dt}
    for name, d in (("forward", 1), ("reverse", -1)):
        span = v16_straight_window(rows, c, dt, valid=loaded, direction=d)
        entry = None
        if span is not None:
            eff = v16_rolling(rows[span[0]:span[1]], c, 1.0)
            entry = {"window": list(span), "window_s": (span[1] - span[0] - 1) * dt, "travel_m": eff["travel_m"],
                     "lateral_m": eff["lateral_m"],
                     "r_eff_m": eff["ratio"], "r_eff_per_wheel_m": eff["ratio_per_wheel"], "raw_ratio": eff["ratio"] / radius}
        out[name] = entry
    return out


def v16_report(long_run: Path, standard_run: Path, missions=(), r_cal: float | None = None) -> dict:
    """16th-draft V2 and V3 for one plant. r_cal: the long set's unloaded 0.25 m/s forward straight
    unless given (V4 solver/dt with the base candidate's r_cal)."""
    long_, std = v16_probe(long_run), v16_probe(standard_run)
    radius = long_["radius_m"]
    cal = [e for e in long_["v3"] if e["label"] == "fwd" and e["window"] is not None]
    own = cal[0]["r_eff_m"] if cal else None
    ref = r_cal if r_cal is not None else own
    rep = {"v2": {"long": long_["v2"], "standard": std["v2"]}, "v3": {"radius_m": radius, "r_cal_own_m": own,
           "r_cal_used_m": ref, "r_cal_over_radius": None if ref is None else ref / radius}}

    def ratio(e, tol):
        if e is None or e.get("window") is None or ref is None:
            return {"verified": False}
        r = float(e["r_eff_m"]) / ref
        return {"verified": True, "ratio": r, "per_wheel": [float(w) / ref for w in e["r_eff_per_wheel_m"]],
                "raw_ratio": float(e["raw_ratio"]), "window_s": float(e["window_s"]), "travel_m": float(e["travel_m"]),
                "lateral_m": float(e["lateral_m"]),
                "pass": bool(abs(r - 1) <= tol)}  # plain types: the report goes through json.dumps
    rep["v3"]["reproducibility"] = [dict(segment=e["segment"], **ratio(e, V3_REPRO_TOL))
                                    for e in std["v3"] if e["label"] == "fwd"]
    rep["v3"]["validation"] = [dict(segment=e["segment"], label=e["label"], **ratio(e, V3_VALID_TOL))
                               for e in long_["v3"] if e["label"] != "fwd" or r_cal is not None]
    rep["v3"]["loaded"] = []
    for m in missions:
        lm = v16_loaded(m)
        rep["v3"]["loaded"].append({"run": lm["run"], "loaded_s": lm["loaded_s"],
                                    **{k: ratio(lm[k], V3_VALID_TOL) for k in ("forward", "reverse")}})
    return rep


def main16(argv) -> int:
    parser = argparse.ArgumentParser(description="16th-draft V2/V3 (fork spacing plan)")
    parser.add_argument("--long", type=Path, required=True)
    parser.add_argument("--standard", type=Path, required=True)
    parser.add_argument("--missions", type=Path, nargs="*", default=())
    parser.add_argument("--r-cal", type=float)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    rep = v16_report(args.long, args.standard, args.missions, args.r_cal)
    args.output.write_text(json.dumps(rep, indent=1) + "\n")
    return 0


def main(argv=None) -> int:
    import sys
    argv = sys.argv[1:] if argv is None else list(argv)
    if argv[:1] == ["v16"]:
        return main16(argv[1:])
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("runs", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    out = [evaluate(r) for r in args.runs]
    args.output.write_text(json.dumps(out, indent=1) + "\n")
    for e in out:
        print("==", e["run"], "wheel angles" if e["has_wheel_angles"] else "no wheel angles")
        for s in e["segments"]:
            st = s["steady"]
            print("  %2d %-9s v %+.2f k %+.2f  %s" % (
                s["segment"], s["label"], s["speed_mps"], s["kappa_cmd"],
                "unverified" if st is None else "steady %.2fs k_true %s res %s roll %s errL/R %.1f/%.1f" % (
                    st["steady_s"], None if st["kappa_true"] is None else round(st["kappa_true"], 4),
                    None if st["residual"] is None else round(st["residual"], 4),
                    None if st.get("rolling_ratio") is None else round(st["rolling_ratio"], 4),
                    st["err_FL_mrad"], st["err_FR_mrad"])))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
