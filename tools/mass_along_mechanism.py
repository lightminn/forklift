"""Records-only mechanism diagnosis after the pallet mass probe
(docs/plans/2026-11-06-mass-along-mechanism-diagnosis.md, edition 3): D1-D8 per run and per pair, from the batch the
probe judged (no Isaac). An exploration after the results were read: it describes when and where the error appears and
what the records rule out, without thresholds.

    python tools/mass_along_mechanism.py OUT VERDICT_JSON --json DIAG [--csv-events EVENTS] [--csv-pairs PAIRS]

OUT holds the run folders ``mae_{A|B}_{scene}_s{seed}`` (``run/`` and ``bridge/``); VERDICT_JSON is the registered
tool's verdict (tools/mass_along_judge.py, #319) for the pair list, the primary pairs and their split."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mass_along_judge as J  # noqa: E402
from forklift_core.localization.slam_pose import compose, invert  # noqa: E402

WINDOW_S = 0.1  # D3 windows
TURN_CURVATURE = 0.05  # D3: a window turns when its mean kinematic curvature is at least this (1/m)
MIN_INTERVAL_TRAVEL_M = 0.05  # D2
DEFAULT_LASER_HEIGHT_M = 1.05


# ---------------------------------------------------------------- shared geometry


def err3(truth, est) -> tuple[float, float, float]:
    """Estimate in the truth frame: along, lateral, yaw."""
    x, y, yaw = compose(invert(tuple(float(v) for v in truth)), tuple(float(v) for v in est))
    return float(x), float(y), J.wrap(float(yaw))


def phase_windows(result: dict, t_m: float | None) -> dict[str, tuple[float, float] | None]:
    insert = next((float(x["time_s"]) for x in result.get("transitions") or [] if x.get("to") == "insert"), None)
    return {"approach_empty": (0.0, insert) if insert is not None else None,
            "extract": J.phase_window(result, "extract", "transport"),
            "transport": J.phase_window(result, "transport", None, t_end=t_m) if t_m is not None else None}


def window_of(t: float, windows: dict) -> str | None:
    for name, w in windows.items():
        if w is not None and w[0] <= t <= w[1]:
            return name
    return None


# ---------------------------------------------------------------- D1


def correction_effects(events: list[tuple], records: list[dict], control: np.ndarray, offset: float) -> list[dict]:
    """D1: per correction event (the first application left out), the error under the applied correction just before
    (C-) and just after (C+). A processed scan: its odom_base under C-/C+ against its truth_base, both moved to the rear
    axle. A release: the first control row after it (only before the next event) holds the corrected rear estimate E and
    the truth T; E+ = E, E- = C- o invert(C+) o E, both against T, with the row's delay recorded."""
    out = []
    times = [e[0] for e in events]
    for i, (t, kind, before, after, _jump) in enumerate(events):
        if before is None:
            continue
        nxt = times[i + 1] if i + 1 < len(times) else math.inf
        if kind == "processed":
            rec = next((r for r in records if r.get("status") == "processed" and abs(float(r["stamp_s"]) - t) < 1e-9
                        and r.get("odom_base") is not None and r.get("truth_base") is not None), None)
            if rec is None:
                continue
            odom = tuple(float(v) for v in rec["odom_base"])
            truth = J.rear_of(rec["truth_base"], offset)
            pre = err3(truth, J.rear_of(compose(tuple(before), odom), offset))
            post = err3(truth, J.rear_of(compose(tuple(after), odom), offset))
            delay = 0.0
        else:
            k = J.control_row_at_or_after(control, t)
            if k is None or control[k, 0] >= nxt:
                continue
            e_plus = tuple(control[k, 1:4])
            truth = tuple(control[k, 4:7])
            e_minus = compose(compose(tuple(before), invert(tuple(after))), e_plus)
            pre, post = err3(truth, e_minus), err3(truth, e_plus)
            delay = float(control[k, 0] - t)
        out.append({"t": t, "kind": kind, "pre": pre, "post": post, "delay_s": delay})
    return out


# ---------------------------------------------------------------- D2 and D8


def keyframe_intervals(records: list[dict], control: np.ndarray, offset: float) -> list[dict]:
    """D2: consecutive processed scans i, j: dO = Oi^-1 o Oj, dT = Ti^-1 o Tj (rear axle); the x, y, yaw differences, the
    truth rear path length between them (control rows), direction and turn."""
    proc = [r for r in records if r.get("status") == "processed" and r.get("odom_base") is not None
            and r.get("truth_base") is not None]
    out = []
    for a, b in zip(proc, proc[1:]):
        oa, ob = J.rear_of(a["odom_base"], offset), J.rear_of(b["odom_base"], offset)
        ta, tb = J.rear_of(a["truth_base"], offset), J.rear_of(b["truth_base"], offset)
        do, dt = compose(invert(oa), ob), compose(invert(ta), tb)
        t0, t1 = float(a["stamp_s"]), float(b["stamp_s"])
        sel = (control[:, 0] >= t0) & (control[:, 0] <= t1)
        seg = control[sel, 4:6]
        path = float(np.sum(np.hypot(*np.diff(seg, axis=0).T))) if len(seg) > 1 else 0.0
        out.append({"t0": t0, "t1": t1, "dx": float(do[0] - dt[0]), "dy": float(do[1] - dt[1]),
                    "dyaw": J.wrap(float(do[2] - dt[2])), "path_m": path, "along_true_m": float(dt[0]),
                    "turn_rad": J.wrap(float(dt[2])), "speed_mps": path / (t1 - t0) if t1 > t0 else None})
    return out


def encoder_split(log: dict, intervals: list[dict], radius_m: float) -> list[dict]:
    """D8: per keyframe interval a -> b (joint samples at or before the stamps): the encoder model's error (summed
    increments minus the rear joints' rotation, mean of the two wheels, rad) and the joint rotation against the truth
    rear's signed along motion (joint rotation x wheel_radius_m - along, m)."""
    t = np.asarray(log["joint_stamps_s"], float)
    angles = np.asarray(log["wheel_angles_rad"], float)
    inc = np.asarray(log["wheel_increments_rad"], float)
    out = []
    for iv in intervals:
        a = int(np.searchsorted(t, iv["t0"], side="right")) - 1
        b = int(np.searchsorted(t, iv["t1"], side="right")) - 1
        if a < 0 or b <= a:
            continue
        rot = angles[b, 2:4] - angles[a, 2:4]
        model = inc[a + 1:b + 1].sum(axis=0) - rot
        out.append({"t0": iv["t0"], "t1": iv["t1"], "model_err_rad": float(model.mean()),
                    "joint_vs_truth_m": float(rot.mean() * radius_m - iv["along_true_m"])})
    return out


# ---------------------------------------------------------------- D3


def kinematic_curvature(delta_l: np.ndarray, delta_r: np.ndarray, wheelbase: float, track: float) -> np.ndarray:
    """wheel_odometry._curvature_inv_m: the mean of the two front wheels' Ackermann curvatures."""
    tl, tr = np.tan(delta_l), np.tan(delta_r)
    return 0.5 * (tl / (wheelbase + track * tl / 2.0) + tr / (wheelbase - track * tr / 2.0))


def steering_residuals(log: dict, control: np.ndarray, window, wheelbase: float, track: float) -> dict:
    """D3: per physics sample the kinematic curvature from the actual (and the measured) steering angles and the truth
    rear's signed along step; per 0.1 s window the residual r = dpsi_true - sum(0.5 (k_{k-1} + k_k) ds_k); windows whose
    mean kinematic curvature (actual) is at least TURN_CURVATURE count. Sums over the counted windows."""
    if window is None:
        return {}
    t = np.asarray(log["joint_stamps_s"], float)
    sel = np.nonzero((t >= window[0]) & (t <= window[1]))[0]
    if len(sel) < 3:
        return {}
    ts = t[sel]
    x = np.interp(ts, control[:, 0], control[:, 4])
    y = np.interp(ts, control[:, 0], control[:, 5])
    yaw = np.interp(ts, control[:, 0], np.unwrap(control[:, 6]))
    mid = 0.5 * (yaw[1:] + yaw[:-1])
    ds = np.diff(x) * np.cos(mid) + np.diff(y) * np.sin(mid)
    dpsi = np.diff(yaw)
    out = {}
    for name, key in (("actual", "steering_rad"), ("measured", "steering_measured_rad")):
        if key not in log:
            continue
        st = np.asarray(log[key], float)[sel]
        k = kinematic_curvature(st[:, 0], st[:, 1], wheelbase, track)
        pred = 0.5 * (k[1:] + k[:-1]) * ds
        win = np.floor((ts[1:] - window[0]) / WINDOW_S).astype(int)
        k_mid = 0.5 * (k[1:] + k[:-1])
        if name == "actual":
            turning = {w for w in np.unique(win) if abs(np.mean(k_mid[win == w])) >= TURN_CURVATURE}
        mask = np.isin(win, sorted(turning))
        out[f"{name}.residual_rad"] = float(np.sum(dpsi[mask] - pred[mask]))
        out[f"{name}.windows"] = int(len(turning))
        out[f"{name}.abs_ds_m"] = float(np.sum(np.abs(ds[mask])))
    return out


# ---------------------------------------------------------------- D4, D5, D7


def attitude(log: dict, stamps: list[float], laser_height: float) -> dict:
    """D4: signed pitch and roll at the processed-scan stamps (nearest joint sample), their summaries, and the scales
    they imply (the LiDAR origin's horizontal shift height x tilt, the beam height change 5 m ahead)."""
    t = np.asarray(log["joint_stamps_s"], float)
    q = np.asarray(log["base_pose_world"], float)[:, 3:7]
    if not len(stamps) or not len(t):
        return {}
    idx = np.clip(np.searchsorted(t, stamps), 0, len(t) - 1)
    pr = np.array([J.pitch_roll(q[i]) for i in idx])
    out = {}
    for j, name in enumerate(("pitch", "roll")):
        v = pr[:, j]
        out[f"{name}_mean_rad"] = float(v.mean())
        out[f"{name}_std_rad"] = float(v.std())
        out[f"{name}_maxabs_rad"] = float(np.abs(v).max())
    tilt = max(out["pitch_maxabs_rad"], out["roll_maxabs_rad"])
    out["origin_shift_max_m"] = laser_height * tilt
    out["beam_height_5m_max_m"] = 5.0 * tilt
    return out


def inherited_parts(e_c, t_c, t_p) -> dict:
    """D5: e_inh = along of T_p^-1 o E_c o (T_c^-1 o T_p), split exactly: the along part cos(Rpsi) ex, the lateral part
    sin(Rpsi) ey and the rest (the yaw error's finite-rotation part), with err = T_c^-1 o E_c and R = T_c^-1 o T_p."""
    ex, ey, eyaw = compose(invert(tuple(t_c)), tuple(e_c))
    rx, ry, rpsi = compose(invert(tuple(t_c)), tuple(t_p))
    along = math.cos(rpsi) * ex
    lateral = math.sin(rpsi) * ey
    yaw = (math.cos(rpsi) * ((math.cos(eyaw) - 1) * rx - math.sin(eyaw) * ry)
           + math.sin(rpsi) * (math.sin(eyaw) * rx + (math.cos(eyaw) - 1) * ry))
    total = J.along(tuple(t_p), compose(tuple(e_c), compose(invert(tuple(t_c)), tuple(t_p))))
    return {"along": along, "lateral": lateral, "yaw": yaw, "e_inh": total, "residual": total - (along + lateral + yaw)}


def map_publications(maps: list[dict], windows: dict) -> dict:
    """D7: map publications per window (a count of what the bridge received, not of content changes) and the extent
    change across it."""
    out = {}
    for name, w in windows.items():
        if w is None:
            continue
        ms = [m for m in maps if w[0] <= float(m["stamp_ns"]) * 1e-9 <= w[1]]
        out[f"{name}.publications"] = len(ms)
        if ms:
            out[f"{name}.area_change_cells"] = int(ms[-1]["width"] * ms[-1]["height"] - ms[0]["width"] * ms[0]["height"])
    return out


# ---------------------------------------------------------------- per run


def diagnose_run(folder: Path, t_m: float | None, split: dict | None) -> dict:
    run = folder / "run"
    result = json.loads((run / "result.json").read_text())
    control = np.load(run / "slam_control.npy")
    records = json.loads((run / "slam_records.json").read_text())
    z = np.load(run / "slam_log.npz")
    log = {k: z[k] for k in z.files}
    meta = json.loads((run / "meta.json").read_text())
    geom = meta["odometry_geometry"]
    offset = float((result.get("arguments") or {}).get("rear_axle_offset_m", geom.get("rear_axle_x_in_base_m")))
    laser_h = float(((meta.get("laser") or {}).get("mount_xyz_m") or [0, 0, DEFAULT_LASER_HEIGHT_M])[2])
    if t_m is None:
        r1 = J.first_round1(result)
        t_m = float(r1["time_s"]) if r1 else None
    windows = phase_windows(result, t_m)
    if t_m is None:
        # a run that never reached round 1: its transport up to its end
        tr = J.phase_window(result, "transport", None, t_end=float(control[-1, 0]))
        windows["transport"] = tr
    holds = (result.get("slam_summary") or {}).get("holds") or []
    events, bad = J.correction_events(records, holds)
    out = {"run": folder.name, "failure": result.get("failure_reason"), "windows": windows, "replay_error": bad}
    d1 = [] if bad else correction_effects(events, records, control, offset)
    d2 = keyframe_intervals(records, control, offset)
    d8 = encoder_split(log, d2, float(geom["wheel_radius_m"]))
    for name, w in windows.items():
        if w is None:
            continue
        e = [x for x in d1 if w[0] <= x["t"] <= w[1]]
        s = out.setdefault(name, {})
        s["d1_events"] = len(e)
        for kind in ("processed", "release", "all"):
            ek = [x for x in e if kind == "all" or x["kind"] == kind]
            if ek:
                s[f"d1_{kind}_pre_along_mean_m"] = float(np.mean([x["pre"][0] for x in ek]))
                s[f"d1_{kind}_post_along_mean_m"] = float(np.mean([x["post"][0] for x in ek]))
                s[f"d1_{kind}_reduction_along_mean_m"] = float(np.mean([abs(x["pre"][0]) - abs(x["post"][0]) for x in ek]))
                s[f"d1_{kind}_post_yaw_mean_rad"] = float(np.mean([x["post"][2] for x in ek]))
        iv = [x for x in d2 if w[0] <= x["t0"] and x["t1"] <= w[1] and x["path_m"] >= MIN_INTERVAL_TRAVEL_M]
        path = sum(x["path_m"] for x in iv)
        s["d2_intervals"] = len(iv)
        s["d2_path_m"] = path
        if path > 0:
            s["d2_dx_per_m"] = sum(x["dx"] for x in iv) / path
            s["d2_dy_per_m"] = sum(x["dy"] for x in iv) / path
            s["d2_dyaw_per_m"] = sum(x["dyaw"] for x in iv) / path
            for direction, sign in (("fwd", 1), ("rev", -1)):
                ivd = [x for x in iv if math.copysign(1, x["along_true_m"]) == sign]
                pd = sum(x["path_m"] for x in ivd)
                if pd > 0:
                    s[f"d2_{direction}_dyaw_per_m"] = sum(x["dyaw"] for x in ivd) / pd
                    s[f"d2_{direction}_dx_per_m"] = sum(x["dx"] for x in ivd) / pd
                    s[f"d2_{direction}_path_m"] = pd
        d8w = [x for x in d8 if w[0] <= x["t0"] and x["t1"] <= w[1]]
        if d8w:
            s["d8_model_err_rad"] = float(sum(x["model_err_rad"] for x in d8w))
            s["d8_joint_vs_truth_m"] = float(sum(x["joint_vs_truth_m"] for x in d8w))
        for k, v in steering_residuals(log, control, w, float(geom["wheelbase_m"]), float(geom["track_m"])).items():
            s[f"d3_{k}"] = v
        stamps = [float(r["stamp_s"]) for r in records if r.get("status") == "processed" and w[0] <= float(r["stamp_s"]) <= w[1]]
        for k, v in attitude(log, stamps, laser_h).items():
            s[f"d4_{k}"] = v
    maps_path = folder / "bridge" / "maps.json"
    if maps_path.exists():
        for k, v in map_publications(json.loads(maps_path.read_text()), windows).items():
            name, key = k.split(".", 1)
            out.setdefault(name, {})[f"d7_{key}"] = v
    if split and "t_c" in split and "t_p" in split:
        ic = J.control_row_at_or_after(control, split["t_c"])
        ip = J.control_row_before(control, split["t_m"])
        if ic is not None and ip is not None:
            out["d5"] = inherited_parts(control[ic, 1:4], control[ic, 4:7], control[ip, 4:7])
    out["_events"] = d1
    return out


# ---------------------------------------------------------------- pairs


def spearman(x: list[float], y: list[float]) -> float | None:
    if len(x) < 3:
        return None
    from scipy import stats

    rho = stats.spearmanr(x, y).correlation
    return None if rho is None or math.isnan(rho) else float(rho)


def diagnose(out_dir: Path, verdict: dict) -> dict:
    runs, pairs = {}, []
    for p in verdict["pairs"]:
        sp = p.get("split") or {}
        for arm in J.ARMS:
            name = f"mae_{arm}_{p['scene']}_s{p['seed']}"
            folder = out_dir / name
            if not (folder / "run" / "result.json").exists():
                continue
            a_sp = sp.get(arm) or {}
            runs[name] = diagnose_run(folder, a_sp.get("t_m"), a_sp if "t_c" in a_sp else None)
        pairs.append(p)
    rows = []
    for p in pairs:
        a, b = runs.get(f"mae_A_{p['scene']}_s{p['seed']}"), runs.get(f"mae_B_{p['scene']}_s{p['seed']}")
        if a is None or b is None:
            continue
        row = {"scene": p["scene"], "seed": p["seed"], "primary": bool(p.get("primary"))}
        for w in ("approach_empty", "extract", "transport"):
            for k in sorted(set(a.get(w, {})) | set(b.get(w, {}))):
                va, vb = a.get(w, {}).get(k), b.get(w, {}).get(k)
                row[f"{w}.{k}.A"] = va
                row[f"{w}.{k}.B"] = vb
                if isinstance(va, (int, float)) and isinstance(vb, (int, float)):
                    row[f"{w}.{k}.d"] = vb - va
        for part in ("along", "lateral", "yaw", "e_inh", "residual"):
            if "d5" in a and "d5" in b:
                row[f"d5.{part}.d"] = b["d5"][part] - a["d5"][part]
        if p.get("primary"):
            row["d6.delta_e_inh"] = (p.get("delta") or {}).get("e_inh")
            row["d6.path_hausdorff_m"] = p.get("path_hausdorff_m")
            spl = p.get("split") or {}
            for k in ("d_c", "t_m_minus_t_c"):
                va, vb = spl.get("A", {}).get(k), spl.get("B", {}).get(k)
                row[f"d6.{k}.d"] = vb - va if va is not None and vb is not None else None
        rows.append(row)
    summary = {}
    for scene in J.SCENES:
        prim = [r for r in rows if r["scene"] == scene and r["primary"]]
        keys = sorted({k for r in prim for k in r if k.endswith(".d")})
        summary[scene] = {"n": len(prim), "mean_d": {k: float(np.mean([r[k] for r in prim if r.get(k) is not None]))
                                                        for k in keys if any(r.get(k) is not None for r in prim)}}
        d6 = [r for r in prim if r.get("d6.delta_e_inh") is not None]
        summary[scene]["d6_spearman"] = {k: spearman([r["d6.delta_e_inh"] for r in d6 if r.get(k) is not None],
                                                     [r[k] for r in d6 if r.get(k) is not None])
                                         for k in ("d6.path_hausdorff_m", "d6.d_c.d", "d6.t_m_minus_t_c.d")}
    return {"runs": {k: {kk: vv for kk, vv in v.items() if kk != "_events"} for k, v in runs.items()},
            "pairs": rows, "summary": summary, "_events": {k: v["_events"] for k, v in runs.items()}}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out", type=Path)
    ap.add_argument("verdict", type=Path)
    ap.add_argument("--json", type=Path, required=True)
    ap.add_argument("--csv-events", type=Path)
    ap.add_argument("--csv-pairs", type=Path)
    args = ap.parse_args(argv)
    res = diagnose(args.out, json.loads(args.verdict.read_text()))
    events = res.pop("_events")
    args.json.write_text(json.dumps(res, indent=1, default=str) + "\n")
    if args.csv_events:
        with args.csv_events.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["run", "t", "kind", "delay_s", "pre_along", "pre_lat", "pre_yaw", "post_along", "post_lat", "post_yaw"])
            for run, ev in sorted(events.items()):
                for e in ev:
                    w.writerow([run, e["t"], e["kind"], e["delay_s"], *e["pre"], *e["post"]])
    if args.csv_pairs and res["pairs"]:
        keys = sorted({k for r in res["pairs"] for k in r})
        with args.csv_pairs.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(res["pairs"])
    print(json.dumps(res["summary"], indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
