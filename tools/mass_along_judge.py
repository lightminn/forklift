"""Pallet mass -> delivery round-1 SLAM along-track error: the registered verdict
(docs/plans/2026-11-05-pallet-mass-along-error-probe.md, edition 4 and amendment 1). Written and registered before any
measured value of the batch is read; the plan's definitions win where this file differs.

    python tools/mass_along_judge.py OUT [--json VERDICT] [--csv PAIRS] [--compare COMPARE_JSON]

OUT holds the run folders ``mae_{A|B}_{scene}_s{seed}`` (``_rep`` for the determinism repeats), each with ``run/``.
``--compare`` is the determinism comparisons' outcome, ``{"<scene>/<arm>": true|false}`` (tools/deterministic_compare.py
on ``s0`` against ``s0_rep``); a missing entry counts as unconfirmed.

Measures (plan section 3): e_m (the round's own ``estimate_error_truck_frame[0]``), the pre-match control pose t_p,
e_p / e_phase, the tracker replay's correction events, the counterfactual split e_inh / e_new, and the descriptive
mechanism measures. Verdicts J0-J3 (section 5)."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

from forklift_core.localization.slam_pose import compose, invert

ARMS = ("A", "B")
ARM_KG = {"A": 10.0, "B": 4.93}
SCENES = ("line_2", "random_1")
NAME = re.compile(r"^mae_(?P<arm>[AB])_(?P<scene>line_2|random_1)_s(?P<seed>\d+)(?P<rep>_rep)?$")
TRANSFORM_TOL = 1e-12  # plan 3.2: a correction event is a change of the applied transform beyond this
MIN_PAIRS_SCENE, MIN_BLOCKS = 7, 7  # plan 4.1
REPORT_THRESHOLD_M = -0.010  # plan J1
SAME_PATH_HAUSDORFF_M = 0.10  # plan 3.4
CUSP_MIN_SEGMENT_M = 0.05  # a direction change counts once the truck has gone this far each way
K_MIN_TRAVEL_M = 0.3  # plan 3.3


# ---------------------------------------------------------------- geometry


def along(truth, pose) -> float:
    """The along-track component of ``pose`` in the truck frame of ``truth`` (plan 3, coordinates)."""
    return float(compose(invert(tuple(float(v) for v in truth)), tuple(float(v) for v in pose))[0])


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def pitch_roll(q_wxyz) -> tuple[float, float]:
    """Signed pitch and roll (rad) of a w, x, y, z quaternion (ZYX convention)."""
    w, x, y, z = (float(v) for v in q_wxyz)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return pitch, roll


# ---------------------------------------------------------------- the round and the endpoints


def first_round1(result: dict) -> dict | None:
    rounds = (result.get("delivery_docking") or {}).get("rounds") or []
    return next((r for r in rounds if r.get("round") == 1), None)


def control_row_before(control: np.ndarray, t: float) -> int | None:
    """Index of the last control row strictly before ``t`` (plan 3.1: the row at t can be overwritten)."""
    idx = int(np.searchsorted(control[:, 0], t, side="left")) - 1
    return idx if idx >= 0 else None


def control_row_at_or_after(control: np.ndarray, t: float) -> int | None:
    idx = int(np.searchsorted(control[:, 0], t, side="left"))
    return idx if idx < len(control) else None


# ---------------------------------------------------------------- the tracker replay (plan 3.2)


def _transform(v) -> tuple[float, float, float]:
    return tuple(float(x) for x in v)


def _differs(a, b) -> bool:
    if a is None or b is None:
        return a is not b
    return any(abs(x - y) > TRANSFORM_TOL for x, y in zip(a, b))


def correction_events(records: list[dict], holds: list[dict]) -> tuple[list[tuple[float, str]], str | None]:
    """The applied correction's real changes, by replaying the tracker: processed scans set the received R and, unless
    the record's mode is holding, the applied A; skipped scans change no transform; a release (a holds entry carrying
    jump_m / jump_rad) applies R. Records come before releases at the same time. Returns the events (time, kind) and
    the first inconsistency (None when every record's applied value and every release's jump agree)."""
    stream = [(float(r["stamp_s"]), 0, "record", r) for r in records]
    stream += [(float(h["time_s"]), 1, "release", h) for h in holds if "jump_m" in h or "jump_rad" in h]
    stream.sort(key=lambda e: (e[0], e[1]))
    received = applied = None
    events: list[tuple[float, str]] = []
    for t, _, kind, item in stream:
        before = applied
        if kind == "record":
            if item.get("status") == "processed":
                received = _transform(item["map_from_odom"])
                if item.get("mode") != "holding":
                    applied = received
            recorded = item.get("applied_map_from_odom")
            if recorded is not None and _differs(applied, _transform(recorded)):
                return events, f"record {item.get('scan_id')} at {t:.4f}: replayed applied differs from the record"
        else:
            applied = received
            jumped = max(abs(float(item.get("jump_m") or 0.0)), abs(float(item.get("jump_rad") or 0.0))) > TRANSFORM_TOL
            if jumped != _differs(before, applied):
                return events, f"release {item.get('event')} at {t:.4f}: jump and replayed change disagree"
        if before is not None and _differs(before, applied):
            events.append((t, kind if kind == "release" else "processed"))
        elif before is None and applied is not None:
            events.append((t, "first"))
    return events, None


# ---------------------------------------------------------------- the split (plan 3.1, 3.2)


def split(result: dict, control: np.ndarray, records: list[dict]) -> dict:
    """e_m, e_p, e_phase, e_inh, e_new and their diagnostics; ``missing`` names why a part is undefined."""
    out: dict = {}
    r1 = first_round1(result)
    if r1 is None:
        return {"missing": "no round 1"}
    out["e_m"] = float(r1["estimate_error_truck_frame"][0])
    out["e_m_world_x"] = float(r1["estimate_vs_truth_at_match"][0])
    t_m = float(r1["time_s"])
    out["t_m"] = t_m
    ip = control_row_before(control, t_m)
    if ip is None:
        return {**out, "missing": "no control row before the match"}
    t_p = float(control[ip, 0])
    e_pose, t_pose = control[ip, 1:4], control[ip, 4:7]
    out.update(t_p=t_p, e_p=along(t_pose, e_pose))
    out["e_phase"] = out["e_m"] - out["e_p"]
    holds = (result.get("slam_summary") or {}).get("holds") or []
    events, inconsistency = correction_events(records, holds)
    if inconsistency:
        return {**out, "missing": f"replay: {inconsistency}"}
    before = [e for e in events if e[0] <= t_p]
    if not before:
        return {**out, "missing": "no correction event before t_p"}
    t_c, kind_c = before[-1]
    ic = control_row_at_or_after(control, t_c)
    if ic is None or control[ic, 0] > t_p:
        return {**out, "missing": "no control row in [t_c, t_p]"}
    e_c, tr_c = control[ic, 1:4], control[ic, 4:7]
    counterfactual = compose(tuple(e_c), compose(invert(tuple(tr_c)), tuple(t_pose)))
    out["e_inh"] = along(t_pose, counterfactual)
    out["e_new"] = out["e_p"] - out["e_inh"]
    seg = control[ic:ip + 1, 4:6]
    out.update(t_c=float(t_c), t_c_kind=kind_c, t_m_minus_t_c=t_m - float(t_c),
               d_c=float(np.sum(np.hypot(*np.diff(seg, axis=0).T))) if len(seg) > 1 else 0.0,
               dyaw_c=wrap(float(t_pose[2]) - float(tr_c[2])),
               events_before_tp=len(before))
    return out


# ---------------------------------------------------------------- mechanism measures (plan 3.3)


def phase_window(result: dict, start_to: str, end_to: str | None, t_end: float | None = None) -> tuple[float, float] | None:
    """[first transition into start_to, first transition into end_to after it) or up to t_end."""
    tr = result.get("transitions") or []
    t0 = next((float(x["time_s"]) for x in tr if x.get("to") == start_to), None)
    if t0 is None:
        return None
    if t_end is not None:
        return (t0, t_end)
    t1 = next((float(x["time_s"]) for x in tr if x.get("to") == end_to and float(x["time_s"]) >= t0), None)
    return (t0, t1) if t1 is not None else None


def scale_errors(log: dict, control: np.ndarray, window, radius_m: float) -> dict:
    """k_fwd, k_rev: signed per-direction scale error of the rear encoders against the truth rear displacement projected
    on the truth heading; odometry yaw minus truth yaw is left to the caller (plan 3.3)."""
    if window is None:
        return {}
    t = np.asarray(log["joint_stamps_s"], float)
    inc = np.asarray(log["wheel_increments_rad"], float)
    sel = (t > window[0]) & (t <= window[1])
    if sel.sum() < 2:
        return {}
    ts = t[sel]
    s_odo = inc[sel].mean(axis=1) * radius_m
    xs = np.interp(ts, control[:, 0], control[:, 4])
    ys = np.interp(ts, control[:, 0], control[:, 5])
    yaw = np.interp(ts, control[:, 0], np.unwrap(control[:, 6]))
    # the interval ending at each sample: the previous sample's position
    x0 = np.interp(np.r_[window[0], ts[:-1]], control[:, 0], control[:, 4])
    y0 = np.interp(np.r_[window[0], ts[:-1]], control[:, 0], control[:, 5])
    s_true = (xs - x0) * np.cos(yaw) + (ys - y0) * np.sin(yaw)
    out = {}
    for name, mask in (("fwd", s_true > 0), ("rev", s_true < 0)):
        travel = float(np.abs(s_true[mask]).sum())
        out[f"travel_{name}_m"] = travel
        out[f"k_{name}"] = float((s_odo[mask].sum() - s_true[mask].sum()) / travel) if travel >= K_MIN_TRAVEL_M else None
    return out


def window_mean(t: np.ndarray, v: np.ndarray, window) -> float | None:
    if window is None:
        return None
    sel = (t >= window[0]) & (t <= window[1])
    return float(np.mean(v[sel])) if sel.any() else None


def mechanisms(result: dict, control: np.ndarray, log: dict | None, tyres: np.ndarray | None, sp: dict) -> dict:
    out: dict = {}
    t_m = sp.get("t_m")
    windows = {"extract": phase_window(result, "extract", "transport"),
               "transport": phase_window(result, "transport", None, t_end=t_m) if t_m is not None else None,
               "tc_tm": (sp["t_c"], t_m) if "t_c" in sp else None,
               "approach_empty": (0.0, next((float(x["time_s"]) for x in result.get("transitions") or []
                                              if x.get("to") == "insert"), 0.0))}
    radius = float((result.get("slam_feedback") or {}).get("odometry_wheel_radius_m") or 0.125)
    samples = result.get("samples") or []
    st = np.array([s["time_s"] for s in samples], float) if samples else np.zeros(0)
    for wname, w in windows.items():
        if log is not None:
            for k, v in scale_errors(log, control, w, radius).items():
                out[f"{wname}.{k}"] = v
            jt = np.asarray(log["joint_stamps_s"], float)
            q = np.asarray(log["base_pose_world"], float)[:, 3:7]
            pr = np.array([pitch_roll(x) for x in q]) if len(q) else np.zeros((0, 2))
            out[f"{wname}.pitch_rad"] = window_mean(jt, pr[:, 0], w)
            out[f"{wname}.roll_rad"] = window_mean(jt, pr[:, 1], w)
            if "steering_measured_rad" in log:
                read_err = np.abs(np.asarray(log["steering_measured_rad"], float) - np.asarray(log["steering_rad"], float)).mean(axis=1)
                out[f"{wname}.steer_read_err_rad"] = window_mean(jt, read_err, w)
        if len(st):
            track = np.array([np.mean(np.abs(np.asarray(s["steering_actual_rad"], float) - float(s["steering_command_rad"])))
                              if s.get("steering_command_rad") is not None and s.get("steering_actual_rad") is not None
                              else np.nan for s in samples])
            ok = ~np.isnan(track)
            out[f"{wname}.steer_track_err_rad"] = window_mean(st[ok], track[ok], w) if ok.any() else None
            out[f"{wname}.lift_m"] = window_mean(st, np.array([float(s.get("lift_m") or 0.0) for s in samples]), w)
        if tyres is not None and len(tyres) and w is not None:
            # per tyre FL FR RL RR, TYRE_FIELDS: the floor impulse z is field 2; force = -Jz / dt (plan 3.3)
            dt = 1.0 / 120.0
            fz = np.stack([-tyres[:, 1 + 10 * i + 2] / dt for i in range(4)], axis=1)
            sel = (tyres[:, 0] >= w[0]) & (tyres[:, 0] <= w[1]) & np.all(np.isfinite(fz), axis=1)
            if sel.any():
                m = fz[sel].mean(axis=0)
                out[f"{wname}.tyre_fz_n"] = [float(v) for v in m]
                out[f"{wname}.rear_load_share"] = float((m[2] + m[3]) / m.sum()) if m.sum() > 0 else None
        if w is not None:
            i0, i1 = control_row_at_or_after(control, w[0]), control_row_before(control, w[1])
            if i0 is not None and i1 is not None and i1 > i0:
                e_yaw = control[i1, 3] - control[i0, 3]
                t_yaw = control[i1, 6] - control[i0, 6]
                out[f"{wname}.odom_yaw_err_rad"] = wrap(float(e_yaw - t_yaw))
    pal = samples[0] if samples else {}
    out["initial_pallet_m"] = result.get("initial_pallet_m")
    out["initial_pallet_yaw_rad"] = pal.get("pallet_yaw_rad")
    return out


# ---------------------------------------------------------------- pairs (plan 3.4)


def truth_path(control: np.ndarray, window) -> np.ndarray:
    sel = (control[:, 0] >= window[0]) & (control[:, 0] <= window[1])
    return control[sel, 4:6]


def hausdorff(a: np.ndarray, b: np.ndarray) -> float:
    if not len(a) or not len(b):
        return math.inf
    a, b = a[:: max(1, len(a) // 400)], b[:: max(1, len(b) // 400)]
    d = np.hypot(a[:, None, 0] - b[None, :, 0], a[:, None, 1] - b[None, :, 1])
    return float(max(d.min(axis=1).max(), d.min(axis=0).max()))


def cusps(control: np.ndarray, window) -> int:
    path = control[(control[:, 0] >= window[0]) & (control[:, 0] <= window[1])]
    if len(path) < 3:
        return 0
    step = np.diff(path[:, 4:6], axis=0)
    s = step[:, 0] * np.cos(path[:-1, 6]) + step[:, 1] * np.sin(path[:-1, 6])
    count, sign, run = 0, 0, 0.0
    for v in s:
        sg = 1 if v > 0 else -1 if v < 0 else 0
        if sg == 0:
            continue
        if sg == sign or sign == 0:
            run += abs(v)
            sign = sg
        else:
            if run >= CUSP_MIN_SEGMENT_M:
                count += 1
            sign, run = sg, abs(v)
    return count


def replans_before(result: dict, t: float) -> int:
    n = 0
    for key in ("stall_replans", "scene_replans", "observe_replans", "cusp_replans"):
        for e in result.get(key) or []:
            ts = e.get("time_s") if isinstance(e, dict) else None
            if ts is not None and float(ts) < t:
                n += 1
    return n


def first_divergence(ca: np.ndarray, cb: np.ndarray) -> float | None:
    n = min(len(ca), len(cb))
    diff = np.nonzero(np.any(ca[:n] != cb[:n], axis=1))[0]
    if len(diff):
        return float(ca[diff[0], 0])
    return None if len(ca) == len(cb) else float(ca[n - 1, 0]) if n else 0.0


def phase_at(result: dict, t: float) -> str | None:
    phase = None
    for x in result.get("transitions") or []:
        if float(x["time_s"]) <= t:
            phase = x.get("to")
    return phase


# ---------------------------------------------------------------- statistics


def t_quantile(p: float, df: int) -> float:
    from scipy import stats

    return float(stats.t.ppf(p, df))


def one_sided(values: list[float]) -> dict:
    """Mean with the one-sided 95 % t bounds (upper for the 'negative' reading, lower for the 'positive' one)."""
    v = np.asarray([x for x in values if x is not None and math.isfinite(x)], float)
    n = len(v)
    if n < 2:
        return {"n": int(n), "mean": float(v.mean()) if n else None, "upper": None, "lower": None}
    m, se = float(v.mean()), float(v.std(ddof=1) / math.sqrt(n))
    q = t_quantile(0.95, n - 1)
    return {"n": n, "mean": m, "upper": m + q * se, "lower": m - q * se, "sd": float(v.std(ddof=1))}


# ---------------------------------------------------------------- loading


def load_run(folder: Path) -> dict:
    run = folder / "run"
    out = {"folder": str(folder)}
    try:
        out["result"] = json.loads((run / "result.json").read_text())
    except (OSError, ValueError) as exc:
        return {**out, "invalid": f"result.json: {exc}"}
    try:
        out["control"] = np.load(run / "slam_control.npy")
        out["records"] = json.loads((run / "slam_records.json").read_text())
    except (OSError, ValueError) as exc:
        return {**out, "invalid": f"slam records: {exc}"}
    try:
        z = np.load(run / "slam_log.npz")
        out["log"] = {k: z[k] for k in z.files}
    except (OSError, ValueError):
        out["log"] = None
    try:
        out["tyres"] = np.load(run / "tyre_contacts.npy")
    except (OSError, ValueError):
        out["tyres"] = None
    return out


def identity_problem(run: dict, arm: str) -> str | None:
    r = run["result"]
    pm = r.get("pallet_mass") or {}
    if pm.get("shape") != "epal6" or abs(float(pm.get("kg") or 0) - ARM_KG[arm]) > 1e-9:
        return f"pallet_mass {pm.get('kg')} {pm.get('shape')} is not arm {arm}"
    det = r.get("deterministic") or {}
    if not det.get("enabled"):
        return "not a deterministic run"
    if det.get("broken"):
        return f"deterministic broken: {det['broken']}"
    return None


def same_fields(ra: dict, rb: dict) -> list[str]:
    """Fields that must agree between the arms (plan 3.4: only the epal6 value differs)."""
    bad = []
    for k in ("source_sha256", "script_sha256", "forklift_urdf_sha256", "pallet_geometry_sha256", "seed"):
        if ra.get(k) != rb.get(k):
            bad.append(k)
    return bad


# ---------------------------------------------------------------- the verdict


def judge(out_dir: Path, compare: dict | None = None) -> dict:
    runs: dict = defaultdict(dict)  # (scene, seed) -> arm -> run
    for folder in sorted(out_dir.iterdir()):
        m = NAME.match(folder.name)
        if not m or m["rep"]:
            continue
        runs[(m["scene"], int(m["seed"]))][m["arm"]] = folder
    compare = compare or {}
    unconfirmed = {f"{s}/{a}" for s in SCENES for a in ARMS if compare.get(f"{s}/{a}") is not True}
    pairs = []
    for (scene, seed), arms in sorted(runs.items()):
        row = {"scene": scene, "seed": seed}
        if set(arms) != set(ARMS):
            pairs.append({**row, "valid": False, "why": "an arm is missing"})
            continue
        loaded = {a: load_run(arms[a]) for a in ARMS}
        why = next((f"{a}: {loaded[a]['invalid']}" for a in ARMS if "invalid" in loaded[a]), None)
        why = why or next((f"{a}: {p}" for a in ARMS if (p := identity_problem(loaded[a], a))), None)
        why = why or (lambda b: f"fields differ: {b}" if b else None)(same_fields(loaded["A"]["result"], loaded["B"]["result"]))
        if why:
            pairs.append({**row, "valid": False, "why": why})
            continue
        if any(f"{scene}/{a}" in unconfirmed for a in ARMS):
            row["determinism_unconfirmed"] = True
        sp = {a: split(loaded[a]["result"], loaded[a]["control"], loaded[a]["records"]) for a in ARMS}
        row["failure"] = {a: loaded[a]["result"].get("failure_reason") for a in ARMS}
        if any("e_m" not in sp[a] for a in ARMS):
            pairs.append({**row, "valid": True, "primary": False, "why": "a run did not reach round 1",
                          "split": sp})
            continue
        mech = {a: mechanisms(loaded[a]["result"], loaded[a]["control"], loaded[a]["log"], loaded[a]["tyres"], sp[a])
                for a in ARMS}
        row.update(valid=True, primary=True, split=sp, mech=mech, delta_e_m=sp["B"]["e_m"] - sp["A"]["e_m"])
        if all("e_inh" in sp[a] for a in ARMS):
            row["delta"] = {k: sp["B"][k] - sp["A"][k] for k in ("e_inh", "e_new", "e_phase")}
        ca, cb = loaded["A"]["control"], loaded["B"]["control"]
        t_div = first_divergence(ca, cb)
        row["t_div"] = t_div
        row["t_div_phase"] = phase_at(loaded["A"]["result"], t_div) if t_div is not None else None
        wa = phase_window(loaded["A"]["result"], "transport", None, t_end=sp["A"]["t_m"])
        wb = phase_window(loaded["B"]["result"], "transport", None, t_end=sp["B"]["t_m"])
        if wa and wb:
            hd = hausdorff(truth_path(ca, wa), truth_path(cb, wb))
            same = (hd <= SAME_PATH_HAUSDORFF_M and cusps(ca, wa) == cusps(cb, wb)
                    and replans_before(loaded["A"]["result"], sp["A"]["t_m"]) == replans_before(loaded["B"]["result"], sp["B"]["t_m"]))
            row.update(path_hausdorff_m=hd, same_path=bool(same))
        pairs.append(row)

    return verdict(pairs, unconfirmed)


def blocks(pairs: list[dict], key, subset=lambda p: True) -> dict[int, float]:
    by = defaultdict(dict)
    for p in pairs:
        if p.get("primary") and subset(p):
            v = key(p)
            if v is not None:
                by[p["seed"]][p["scene"]] = v
    return {s: (d["line_2"] + d["random_1"]) / 2 for s, d in by.items() if set(d) == set(SCENES)}


def verdict(pairs: list[dict], unconfirmed: set) -> dict:
    out = {"pairs": pairs, "determinism_unconfirmed": sorted(unconfirmed)}
    if unconfirmed:
        out["J"] = "stopped: determinism unconfirmed for " + ", ".join(sorted(unconfirmed))
        return out
    primary = [p for p in pairs if p.get("primary")]
    per_scene = {s: one_sided([p["delta_e_m"] for p in primary if p["scene"] == s]) for s in SCENES}
    out["per_scene"] = per_scene
    out["sign_negative"] = {s: sum(1 for p in primary if p["scene"] == s and p["delta_e_m"] < 0) for s in SCENES}
    d1 = blocks(pairs, lambda p: p["delta_e_m"])
    out["J1_blocks"] = d1
    j1 = one_sided(list(d1.values()))
    out["J1_stats"] = j1
    out["J1_same_path"] = one_sided(list(blocks(pairs, lambda p: p["delta_e_m"], lambda p: p.get("same_path")).values()))
    if any(per_scene[s]["n"] < MIN_PAIRS_SCENE for s in SCENES) or j1["n"] < MIN_BLOCKS:
        out["J1"] = "undecidable: too few valid pairs or blocks"
        return out
    if j1["upper"] < 0 and j1["mean"] <= REPORT_THRESHOLD_M:
        out["J1"] = "confirmed (this condition)"
    elif j1["lower"] > 0:
        out["J1"] = "opposite direction"
    else:
        out["J1"] = "not confirmed"
    if out["J1"] != "confirmed (this condition)":
        return out

    def j2(subset) -> dict:
        dec = blocks(pairs, lambda p: p["delta_e_m"] if "delta" in p else None, subset)
        res = {"blocks": len(dec), "m_dec": one_sided(list(dec.values()))}
        comps = {k: one_sided(list(blocks(pairs, lambda p, k=k: p["delta"][k] if "delta" in p else None, subset).values()))
                 for k in ("e_inh", "e_new", "e_phase")}
        res["components"] = comps
        m_dec, m_j1 = res["m_dec"]["mean"], j1["mean"]
        rep_ok = (len(dec) >= 0.75 * len(d1) and res["m_dec"]["upper"] is not None and res["m_dec"]["upper"] < 0
                  and 0.75 <= m_dec / m_j1 <= 1.33 and abs(comps["e_phase"]["mean"]) <= 0.2 * abs(m_dec))
        res["representative"] = bool(rep_ok)
        if not rep_ok:
            res["verdict"] = "undecidable: the decomposed subset does not represent J1"
            return res
        share = {k: comps[k]["mean"] / m_dec for k in ("e_inh", "e_new")}
        res["share"] = share
        if share["e_new"] >= 0.67 and comps["e_new"]["upper"] < 0:
            res["verdict"] = "new odometry drift leads"
        elif share["e_inh"] >= 0.67 and comps["e_inh"]["upper"] < 0:
            res["verdict"] = "carried correction-time error leads"
        else:
            res["verdict"] = "mixed"
        return res

    out["J2"] = j2(lambda p: True)
    out["J2_same_path"] = j2(lambda p: p.get("same_path"))
    keys = sorted({k for p in pairs if p.get("primary") for k in p["mech"]["A"]
                   if isinstance(p["mech"]["A"][k], (int, float)) and k in p["mech"]["B"]})
    out["J3"] = {k: one_sided(list(blocks(pairs, lambda p, k=k: (p["mech"]["B"].get(k) - p["mech"]["A"].get(k))
                                          if isinstance(p["mech"]["B"].get(k), (int, float))
                                          and isinstance(p["mech"]["A"].get(k), (int, float)) else None).values()))
                 for k in keys}
    return out


def pair_rows(pairs: list[dict]) -> list[dict]:
    rows = []
    for p in pairs:
        row = {"scene": p["scene"], "seed": p["seed"], "valid": p.get("valid"), "primary": p.get("primary"),
               "why": p.get("why"), "same_path": p.get("same_path"), "t_div": p.get("t_div"),
               "t_div_phase": p.get("t_div_phase")}
        for a in ARMS:
            for k in ("e_m", "e_m_world_x", "e_p", "e_phase", "e_inh", "e_new", "t_m_minus_t_c", "d_c", "t_c_kind"):
                row[f"{k}_{a}"] = (p.get("split") or {}).get(a, {}).get(k)
            row[f"failure_{a}"] = (p.get("failure") or {}).get(a)
        row["delta_e_m"] = p.get("delta_e_m")
        rows.append(row)
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out", type=Path)
    ap.add_argument("--json", type=Path)
    ap.add_argument("--csv", type=Path)
    ap.add_argument("--compare", type=Path)
    args = ap.parse_args(argv)
    compare = json.loads(args.compare.read_text()) if args.compare else None
    res = judge(args.out, compare)
    summary = {k: v for k, v in res.items() if k != "pairs"}
    print(json.dumps(summary, indent=1, default=str))
    if args.json:
        args.json.write_text(json.dumps(res, indent=1, default=str) + "\n")
    if args.csv:
        rows = pair_rows(res["pairs"])
        with args.csv.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else ["scene"])
            w.writeheader()
            w.writerows(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
