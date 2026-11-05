"""Pallet mass -> delivery round-1 SLAM along-track error: the registered verdict
(docs/plans/2026-11-05-pallet-mass-along-error-probe.md, edition 4 with amendments 1-2). Written and registered before
any measured value of the batch is read; the plan's definitions win where this file differs.

    python tools/mass_along_judge.py OUT [--json VERDICT] [--csv PAIRS] [--compare COMPARE_JSON]

OUT holds the run folders ``mae_{A|B}_{scene}_s{seed}`` (``_rep`` for the determinism repeats), each with ``run/``, and
the bundle's ``b*/results.jsonl``. ``--compare`` is the determinism comparisons' outcome, ``{"<scene>/<arm>": true|false}``
(tools/deterministic_compare.py on ``s0`` against ``s0_rep``); a missing entry counts as unconfirmed.

Measures (plan section 3): e_m (the round's own ``estimate_error_truck_frame[0]``, checked against
``delivery_match_1.npz``), the pre-match control pose t_p, e_p / e_phase, the tracker replay's correction events, the
counterfactual split e_inh / e_new, and the descriptive mechanism measures. Verdicts J0-J3 (section 5)."""

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
MATCH_TOL_M = 1e-9  # plan 3.1: e_m against delivery_match_1.npz
MIN_PAIRS_SCENE, MIN_BLOCKS = 7, 7  # plan 4.1
REPORT_THRESHOLD_M = -0.010  # plan J1
SAME_PATH_HAUSDORFF_M = 0.10  # plan 3.4
CUSP_MIN_SEGMENT_M = 0.05  # a direction change counts once the truck has gone this far each way
K_MIN_TRAVEL_M = 0.3  # plan 3.3
GATE_DISTANCE_M, GATE_TURN_RAD, GATE_INTERVAL_S = math.sqrt(0.8) * 0.5, 0.5, 0.5  # slam_link.KeyframeGate's defaults
# plan 4.1 -> docs/plans/2026-10-31-riib-default-batch2.md section 4: the only infrastructure invalidity
INFRA_BUNDLE_STATUSES = {"infra_failed", "stalled"}
INFRA_FAILURE_PREFIXES = ("slam_link_failed", "slam_prewarm_timeout", "plan_worker_dead", "Camera did not produce RGB",
                          "Perception camera did not produce depth", "Cannot open base scene",
                          "Invalid pallet spawn after settling", "Truck spawn displaced")
# fields that must agree between the arms (plan 3.4: only the epal6 mass differs); the run's own paths are normalised
IDENTITY_FIELDS = ("source_sha256", "script_sha256", "forklift_urdf_sha256", "pallet_geometry_sha256", "pallet_urdf_sha256",
                   "scene_sha256", "seed", "arguments", "plant", "plant_tyre_contact", "encoder_model", "chassis_model")
SNAP_ARM = re.compile(r"(mae_[0-9a-f]+)_[AB]\b")
# the same across the whole batch (plan 2.2 one snapshot, amendment 2): everything but the seed and the scene
BATCH_FIELDS = ("source_sha256", "script_sha256", "forklift_urdf_sha256", "pallet_geometry_sha256", "pallet_urdf_sha256",
                "scene_sha256", "plant_tyre_contact", "chassis_model")
# plan amendment 3: the plant record lists the scene's own prop materials (roots below) beside the truck's, the pallet's
# and the environment's; the encoder model carries the seed's encoder phases beside its fixed specification. Only those
# parts may differ -- the props within a scene, the phases within a seed; the rest of both is one value across the batch.
SCENE_PROP_ROOTS = ("Scene", "ScenePallets")


def plant_common(result: dict):
    """The plant record without the scene's props; None when the record or its materials list is missing (an empty list
    that was recorded stays an empty list)."""
    plant = result.get("plant")
    if not isinstance(plant, dict) or not isinstance(plant.get("materials"), list):
        return None
    return {**plant, "materials": [m for m in plant["materials"] if m.get("root") not in SCENE_PROP_ROOTS]}


def plant_props(result: dict):
    plant = result.get("plant")
    if not isinstance(plant, dict) or not isinstance(plant.get("materials"), list):
        return None
    return [m for m in plant["materials"] if m.get("root") in SCENE_PROP_ROOTS]


def encoder_common(result: dict):
    enc = result.get("encoder_model")
    return {k: v for k, v in enc.items() if k != "phases_rad"} if isinstance(enc, dict) else enc


def encoder_phases(result: dict):
    enc = result.get("encoder_model")
    return enc.get("phases_rad") if isinstance(enc, dict) else None


BATCH_PARTS = {"plant(common)": plant_common, "encoder_model(spec)": encoder_common}
PER_SCENE_PARTS = {"plant(props)": plant_props}
PER_SEED_PARTS = {"encoder_model(phases)": encoder_phases}
RUN_ARM = re.compile(r"\bmae_[AB]_")


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


def correction_events(records: list[dict], holds: list[dict]) -> tuple[list[tuple], str | None]:
    """The applied correction's real changes, by replaying the tracker: processed scans set the received R and, unless
    the record's mode is holding, the applied A; skipped scans change no transform; a release (a holds entry carrying
    jump_m / jump_rad) applies R. Records come before releases at the same time. Returns the events
    (time, kind, A before, A after, release jump m or None) and the first inconsistency (None when every record's
    applied value and every release's jump agree with the replay)."""
    stream = [(float(r["stamp_s"]), 0, "record", r) for r in records]
    stream += [(float(h["time_s"]), 1, "release", h) for h in holds if "jump_m" in h or "jump_rad" in h]
    stream.sort(key=lambda e: (e[0], e[1]))
    received = applied = None
    events: list[tuple] = []
    for t, _, kind, item in stream:
        before = applied
        jump = None
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
            jump = float(item.get("jump_m") or 0.0)
            jumped = max(abs(jump), abs(float(item.get("jump_rad") or 0.0))) > TRANSFORM_TOL
            if jumped != _differs(before, applied):
                return events, f"release {item.get('event')} at {t:.4f}: jump and replayed change disagree"
        if before is not None and _differs(before, applied):
            events.append((t, "release" if kind == "release" else "processed", before, applied, jump))
        elif before is None and applied is not None:
            events.append((t, "first", None, applied, jump))
    return events, None


def gate_condition(records: list[dict], t_event: float) -> str | None:
    """Which KeyframeGate condition the processed scan at ``t_event`` met, as the gate reckons it: travel and absolute
    turn summed scan to scan (every record's ``odom_base``) since the previous processed scan, and the interval since
    it (``KeyframeGate.classify`` / ``processed``). 'distance', 'turn', 'both', or 'neither' (a reply the gate did not
    ask for, a start-up scan); 'short' when the interval was under GATE_INTERVAL_S."""
    recs = [r for r in records if r.get("odom_base") is not None]
    idx = next((i for i, r in enumerate(recs) if r.get("status") == "processed"
                and abs(float(r["stamp_s"]) - t_event) < 1e-9), None)
    if idx is None:
        return None
    start = next((i for i in range(idx - 1, -1, -1) if recs[i].get("status") == "processed"), None)
    if start is None:
        return None
    travel = turn = 0.0
    for a, b in zip(recs[start:idx], recs[start + 1:idx + 1]):
        pa, pb = a["odom_base"], b["odom_base"]
        travel += math.hypot(pb[0] - pa[0], pb[1] - pa[1])
        turn += abs(wrap(pb[2] - pa[2]))
    if float(recs[idx]["stamp_s"]) - float(recs[start]["stamp_s"]) < GATE_INTERVAL_S - 1e-9:
        return "short"
    dist, rot = travel >= GATE_DISTANCE_M, turn >= GATE_TURN_RAD
    return "both" if dist and rot else "distance" if dist else "turn" if rot else "neither"


# ---------------------------------------------------------------- the split (plan 3.1, 3.2)


def split(result: dict, control: np.ndarray, records: list[dict], match: dict | None = None) -> dict:
    """e_m, e_p, e_phase, e_inh, e_new and their diagnostics; ``missing`` names why a part is undefined, ``invalid``
    why the primary measure itself is not used (a match file that disagrees with the round)."""
    out: dict = {}
    r1 = first_round1(result)
    if r1 is None:
        return {"missing": "no round 1"}
    out["e_m"] = float(r1["estimate_error_truck_frame"][0])
    out["e_m_world_x"] = float(r1["estimate_vs_truth_at_match"][0])
    t_m = float(r1["time_s"])
    out["t_m"] = t_m
    if match is not None:
        check = along(match["truth_rear"], match["estimate"])
        out["e_m_match_file"] = check
        if abs(check - out["e_m"]) > MATCH_TOL_M:
            return {**out, "invalid": f"delivery_match_1.npz gives {check:.9f}, the round {out['e_m']:.9f}"}
    else:
        return {**out, "invalid": "no delivery_match_1.npz"}
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
    out["_events"] = events
    before = [e for e in events if e[0] <= t_p]
    if not before:
        return {**out, "missing": "no correction event before t_p"}
    t_c, kind_c = before[-1][0], before[-1][1]
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
               dyaw_c=wrap(float(t_pose[2]) - float(tr_c[2])), events_before_tp=len(before),
               t_c_gate=gate_condition(records, float(t_c)) if kind_c == "processed" else None)
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
    on the truth heading (plan 3.3)."""
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


def window_stat(t: np.ndarray, v: np.ndarray, window, how=np.mean) -> float | None:
    if window is None or not len(t):
        return None
    sel = (t >= window[0]) & (t <= window[1])
    return float(how(v[sel])) if sel.any() else None


def value_at(t: np.ndarray, v: np.ndarray, at: float | None) -> float | None:
    if at is None or not len(t):
        return None
    return float(v[min(int(np.searchsorted(t, at)), len(t) - 1)])


def odometry_yaw_error(control: np.ndarray, events: list[tuple], window) -> float | None:
    """The odometry's own yaw increment minus the truth's over the window: the corrected estimate's yaw increment less the
    applied correction's yaw changes inside the window (plan 3.3; the estimate is the correction composed with the
    odometry, so its yaw is their sum)."""
    if window is None:
        return None
    i0, i1 = control_row_at_or_after(control, window[0]), control_row_before(control, window[1])
    if i0 is None or i1 is None or i1 <= i0:
        return None
    t0, t1 = control[i0, 0], control[i1, 0]
    corr = sum(wrap(e[3][2] - e[2][2]) for e in events if e[2] is not None and t0 < e[0] <= t1)
    return wrap(float(control[i1, 3] - control[i0, 3]) - corr - float(control[i1, 6] - control[i0, 6]))


def rear_of(base, rear_axle_offset_m: float) -> tuple[float, float, float]:
    """The rear-axle pose of a base_link pose, as the runner makes it (core.py: back by |rear_axle_offset_m| along yaw)."""
    off = abs(float(rear_axle_offset_m))
    x, y, yaw = (float(v) for v in base)
    return (x - off * math.cos(yaw), y - off * math.sin(yaw), yaw)


def post_event_error(event: tuple, events: list[tuple], records: list[dict], control: np.ndarray,
                     rear_axle_offset_m: float | None = None) -> float | None:
    """The along-track rear-axle error right after one correction event, from that event alone: a processed scan's own
    record (its ``odom_base`` under the event's applied transform against its ``truth_base``, both moved to the rear
    axle, plan 3 coordinates); a release from the first control row after it (rear-axle poses already), only when no
    later event comes before that row."""
    t, kind, _, applied = event[0], event[1], event[2], event[3]
    if kind == "processed":
        rec = next((r for r in records if r.get("status") == "processed" and abs(float(r["stamp_s"]) - t) < 1e-9
                    and r.get("odom_base") is not None and r.get("truth_base") is not None), None)
        if rec is None or rear_axle_offset_m is None:
            return None
        est = compose(tuple(applied), tuple(float(v) for v in rec["odom_base"]))
        return along(rear_of(rec["truth_base"], rear_axle_offset_m), rear_of(est, rear_axle_offset_m))
    i = control_row_at_or_after(control, t)
    if i is None:
        return None
    nxt = next((e[0] for e in events if e[0] > t), math.inf)
    if control[i, 0] >= nxt:
        return None
    return along(control[i, 4:7], control[i, 1:4])


def mechanisms(result: dict, control: np.ndarray, records: list[dict], log: dict | None, tyres: np.ndarray | None,
               sp: dict, truck_mass_kg: float | None = None) -> dict:
    out: dict = {}
    t_m = sp.get("t_m")
    events = sp.get("_events")  # None when the replay failed: every value that needs the events is then missing
    windows = {"extract": phase_window(result, "extract", "transport"),
               "transport": phase_window(result, "transport", None, t_end=t_m) if t_m is not None else None,
               "tc_tm": (sp["t_c"], t_m) if "t_c" in sp else None,
               "approach_empty": (0.0, next((float(x["time_s"]) for x in result.get("transitions") or []
                                              if x.get("to") == "insert"), 0.0))}
    radius = float((result.get("slam_feedback") or {}).get("odometry_wheel_radius_m") or 0.125)
    samples = result.get("samples") or []
    st = np.array([s["time_s"] for s in samples], float) if samples else np.zeros(0)
    track = np.array([float(np.mean(np.abs(np.asarray(s["steering_actual_rad"], float)
                                         - np.asarray(s["steering_command_rad"], float))))
                      if s.get("steering_command_rad") is not None and s.get("steering_actual_rad") is not None
                      else np.nan for s in samples]) if samples else np.zeros(0)
    lift = np.array([float(s["lift_m"]) if s.get("lift_m") is not None else np.nan for s in samples]) if samples else np.zeros(0)
    if log is not None:
        jt = np.asarray(log["joint_stamps_s"], float)
        q = np.asarray(log["base_pose_world"], float)[:, 3:7]
        pr = np.array([pitch_roll(x) for x in q]) if len(q) else np.zeros((0, 2))
        read_err = (np.abs(np.asarray(log["steering_measured_rad"], float) - np.asarray(log["steering_rad"], float)).mean(axis=1)
                    if "steering_measured_rad" in log else None)
        out["pitch_at_t_c_rad"] = value_at(jt, pr[:, 0], sp.get("t_c"))
        out["roll_at_t_c_rad"] = value_at(jt, pr[:, 1], sp.get("t_c"))
        out["pitch_at_t_m_rad"] = value_at(jt, pr[:, 0], t_m)
        out["roll_at_t_m_rad"] = value_at(jt, pr[:, 1], t_m)
    rec_t = np.array([float(r["stamp_s"]) for r in records], float)
    rec_proc = np.array([r.get("status") == "processed" for r in records], bool)
    for wname, w in windows.items():
        if log is not None:
            for k, v in scale_errors(log, control, w, radius).items():
                out[f"{wname}.{k}"] = v
            out[f"{wname}.pitch_rad"] = window_stat(jt, pr[:, 0], w)
            out[f"{wname}.roll_rad"] = window_stat(jt, pr[:, 1], w)
            if read_err is not None:
                out[f"{wname}.steer_read_err_rad"] = window_stat(jt, read_err, w)
        if len(st):
            ok = ~np.isnan(track)
            out[f"{wname}.steer_track_err_rad"] = window_stat(st[ok], track[ok], w)
            out[f"{wname}.steer_track_err_max_rad"] = window_stat(st[ok], track[ok], w, np.max)
            okl = ~np.isnan(lift)
            out[f"{wname}.lift_m"] = window_stat(st[okl], lift[okl], w)
        if tyres is not None and len(tyres) and w is not None:
            # per tyre FL FR RL RR, TYRE_FIELDS: the floor impulse z is field 2; force = -Jz / dt (plan 3.3)
            dt = 1.0 / 120.0
            fz = np.stack([-tyres[:, 1 + 10 * i + 2] / dt for i in range(4)], axis=1)
            sel = (tyres[:, 0] >= w[0]) & (tyres[:, 0] <= w[1]) & np.all(np.isfinite(fz), axis=1)
            if sel.any():
                m = fz[sel].mean(axis=0)
                for i, name in enumerate(("fl", "fr", "rl", "rr")):
                    out[f"{wname}.tyre_fz_{name}_n"] = float(m[i])
                out[f"{wname}.rear_load_share"] = float((m[2] + m[3]) / m.sum()) if m.sum() > 0 else None
        if w is not None:
            inw = (rec_t >= w[0]) & (rec_t <= w[1])
            out[f"{wname}.processed_scans"] = int((inw & rec_proc).sum())
            if events is None:
                for k in ("odom_yaw_err_rad", "correction_events", "release_jump_max_m", "post_event_err_mean_m",
                          "post_event_err_last3_m"):
                    out[f"{wname}.{k}"] = None
                continue
            out[f"{wname}.odom_yaw_err_rad"] = odometry_yaw_error(control, events, w)
            ev = [e for e in events if w[0] <= e[0] <= w[1] and e[2] is not None]
            out[f"{wname}.correction_events"] = len(ev)
            jumps = [e[4] for e in ev if e[4] is not None]
            out[f"{wname}.release_jump_max_m"] = max(jumps) if jumps else None
            offset = (result.get("arguments") or {}).get("rear_axle_offset_m")
            post = [x for x in (post_event_error(e, events, records, control, offset) for e in ev) if x is not None]
            out[f"{wname}.post_event_err_mean_m"] = float(np.mean(post)) if post else None
            out[f"{wname}.post_event_err_last3_m"] = float(np.mean(post[-3:])) if post else None
    for k in ("t_m_minus_t_c", "d_c", "dyaw_c"):
        out[k] = sp.get(k)
    out["t_c_gate"] = sp.get("t_c_gate")
    if tyres is not None and len(tyres):
        # the static weight check (plan 3.3): the four floor forces' sum over the first 0.5 s, before the truck moves
        dt = 1.0 / 120.0
        fz = np.stack([-tyres[:, 1 + 10 * i + 2] / dt for i in range(4)], axis=1)
        sel = (tyres[:, 0] <= 0.5) & np.all(np.isfinite(fz), axis=1)
        out["static_fz_sum_n"] = float(fz[sel].sum(axis=1).mean()) if sel.any() else None
        if out["static_fz_sum_n"] is not None and truck_mass_kg:
            out["static_weight_ratio"] = out["static_fz_sum_n"] / (truck_mass_kg * 9.81)
    out["initial_pallet_m"] = result.get("initial_pallet_m")
    out["initial_pallet_yaw_rad"] = samples[0].get("pallet_yaw_rad") if samples else None
    return out


# ---------------------------------------------------------------- pairs (plan 3.4)


def truth_path(control: np.ndarray, window) -> np.ndarray:
    sel = (control[:, 0] >= window[0]) & (control[:, 0] <= window[1])
    return control[sel, 4:6]


def hausdorff(a: np.ndarray, b: np.ndarray) -> float:
    """The exact symmetric Hausdorff distance of two point sets (every point; no subsampling)."""
    if not len(a) or not len(b):
        return math.inf
    from scipy.spatial.distance import directed_hausdorff

    return float(max(directed_hausdorff(a, b)[0], directed_hausdorff(b, a)[0]))


def cusps(control: np.ndarray, window) -> int:
    """Direction changes of the executed truth motion, each counted once both sides have run CUSP_MIN_SEGMENT_M."""
    path = control[(control[:, 0] >= window[0]) & (control[:, 0] <= window[1])]
    if len(path) < 3:
        return 0
    step = np.diff(path[:, 4:6], axis=0)
    s = step[:, 0] * np.cos(path[:-1, 6]) + step[:, 1] * np.sin(path[:-1, 6])
    segments: list[list] = []  # [sign, length]
    for v in s:
        sg = 1 if v > 0 else -1 if v < 0 else 0
        if sg == 0:
            continue
        if segments and segments[-1][0] == sg:
            segments[-1][1] += abs(v)
        else:
            segments.append([sg, abs(v)])
    long_ = [g for g in segments if g[1] >= CUSP_MIN_SEGMENT_M]
    merged: list[int] = []
    for g in long_:
        if not merged or merged[-1] != g[0]:
            merged.append(g[0])
    return max(0, len(merged) - 1)


def replans_before(result: dict, t: float) -> int:
    n = 0
    lists = [result.get(k) or [] for k in ("stall_replans", "scene_replans", "observe_replans", "cusp_replans")]
    lists.append((result.get("obstacle_layer") or {}).get("replans") or [])
    for entries in lists:
        for e in entries:
            ts = e.get("time_s") if isinstance(e, dict) else None
            if ts is not None and float(ts) < t:
                n += 1
    return n


def first_divergence(ca: np.ndarray, cb: np.ndarray) -> float | None:
    n = min(len(ca), len(cb))
    diff = np.nonzero(np.any(ca[:n] != cb[:n], axis=1))[0]
    if len(diff):
        return float(ca[diff[0], 0])
    if len(ca) == len(cb):
        return None
    longer = ca if len(ca) > len(cb) else cb
    return float(longer[n, 0])


def phase_at(result: dict, t: float) -> str | None:
    tr = result.get("transitions") or []
    phase = tr[0].get("from") if tr else None
    for x in tr:
        if float(x["time_s"]) <= t:
            phase = x.get("to")
    return phase


def adoption(result: dict) -> dict:
    """The near capture's far and near pallet estimates (x, y, yaw) for the plan 3.4 diagnostic."""
    nc = result.get("near_capture")
    if not isinstance(nc, dict):
        return {}
    return {k: nc.get(k) for k in ("far_estimate_m", "near_estimate_m") if nc.get(k) is not None}


def pose_diff(a, b) -> dict | None:
    """Position (m) and yaw (rad) differences of two x, y, yaw poses."""
    try:
        x, y = [float(v) for v in a][:3], [float(v) for v in b][:3]
    except (TypeError, ValueError):
        return None
    if len(x) < 3 or len(y) < 3:
        return None
    return {"position_m": math.hypot(x[0] - y[0], x[1] - y[1]), "yaw_rad": abs(wrap(x[2] - y[2]))}


def max_abs_diff(a, b) -> float | None:
    try:
        x, y = np.asarray(a, float).ravel(), np.asarray(b, float).ravel()
    except (TypeError, ValueError):
        return None
    if x.shape != y.shape or not len(x):
        return None
    return float(np.max(np.abs(x - y)))


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


def bundle_status(out_dir: Path) -> dict[str, str]:
    last: dict = {}
    for f in sorted(out_dir.glob("b*/results.jsonl")):
        for line in f.read_text().splitlines():
            if not line.strip():
                continue
            x = json.loads(line)
            if x["name"] not in last or x.get("attempt", 0) >= last[x["name"]].get("attempt", 0):
                last[x["name"]] = x
    return {k: v.get("status") for k, v in last.items()}


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
        z = np.load(run / "delivery_match_1.npz")
        out["match"] = {"estimate": z["estimate"], "truth_rear": z["truth_rear"]}
    except (OSError, ValueError, KeyError):
        out["match"] = None
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


def infra_reason(run: dict, status: str | None) -> str | None:
    if status in INFRA_BUNDLE_STATUSES:
        return f"bundle {status}"
    if "invalid" in run and run["invalid"].startswith("result.json"):
        return run["invalid"]
    reason = str(run.get("result", {}).get("failure_reason") or "")
    return reason if reason.startswith(INFRA_FAILURE_PREFIXES) else None


def identity_problem(run: dict, arm: str, seed: int) -> str | None:
    r = run["result"]
    pm = r.get("pallet_mass") or {}
    if pm.get("shape") != "epal6" or abs(float(pm.get("kg") or 0) - ARM_KG[arm]) > 1e-9:
        return f"pallet_mass {pm.get('kg')} {pm.get('shape')} is not arm {arm}"
    if r.get("seed") != seed:
        return f"seed {r.get('seed')} is not the folder's {seed}"
    det = r.get("deterministic") or {}
    if not det.get("enabled"):
        return "not a deterministic run"
    if det.get("broken"):
        return f"deterministic broken: {det['broken']}"
    return None


def _normal(value) -> str:
    return RUN_ARM.sub("mae_X_", SNAP_ARM.sub(r"\1_X", json.dumps(value, sort_keys=True, default=str)))


def identity_differences(ra: dict, rb: dict) -> list[str]:
    """Fields outside the allowed difference (plan 3.4): every IDENTITY_FIELDS entry must be present and equal after the
    run's own snapshot and folder names are normalised; the synthetic settings must agree but for pallet_mass_kg; the
    scene layout hash must agree."""
    bad = [k for k in IDENTITY_FIELDS if ra.get(k) is None or rb.get(k) is None or _normal(ra[k]) != _normal(rb[k])]
    sa, sb = dict(ra.get("settings_synthetic") or {}), dict(rb.get("settings_synthetic") or {})
    if not sa or not sb:
        bad.append("settings_synthetic missing")
    sa.pop("pallet_mass_kg", None)
    sb.pop("pallet_mass_kg", None)
    if _normal(sa) != _normal(sb):
        bad.append("settings_synthetic")
    la, lb = (ra.get("scene_layout") or {}).get("sha256"), (rb.get("scene_layout") or {}).get("sha256")
    if la is None or la != lb:
        bad.append("scene_layout.sha256")
    return bad


# ---------------------------------------------------------------- the verdict


TIED = "<tied>"


def _mode(values: list[str]) -> str | None:
    """The single most common value; TIED when two values share the top count (no reference then)."""
    if not values:
        return None
    counts = {v: values.count(v) for v in set(values)}
    top = max(counts.values())
    winners = [v for v, c in counts.items() if c == top]
    return winners[0] if len(winners) == 1 else TIED


def _settings_without_mass(result: dict) -> str:
    st = dict(result.get("settings_synthetic") or {})
    st.pop("pallet_mass_kg", None)
    return _normal(st)


def batch_reference(loaded: dict) -> dict:
    """The batch's common value of each BATCH_FIELDS entry, of the synthetic settings without the pallet mass, and of
    each scene's layout hash (the same scene across seeds and arms): the single most common one (plan 2.2, amendment
    2). A run that differs is outside the allowed difference."""
    runs = [(key, r) for key, r in loaded.items() if "result" in r]
    ref = {k: _mode([_normal(r["result"].get(k)) for _, r in runs]) for k in BATCH_FIELDS}
    ref["settings_synthetic"] = _mode([_settings_without_mass(r["result"]) for _, r in runs])
    for k, part in BATCH_PARTS.items():
        ref[k] = _mode([_normal(part(r["result"])) for _, r in runs])
    for scene in SCENES:
        ref[f"scene_layout:{scene}"] = _mode([_normal((r["result"].get("scene_layout") or {}).get("sha256"))
                                             for key, r in runs if key[0] == scene])
        for k, part in PER_SCENE_PARTS.items():
            ref[f"{k}:{scene}"] = _mode([_normal(part(r["result"])) for key, r in runs if key[0] == scene])
    for seed in sorted({key[1] for key, _ in runs}):
        for k, part in PER_SEED_PARTS.items():
            ref[f"{k}:s{seed}"] = _mode([_normal(part(r["result"])) for key, r in runs if key[1] == seed])
    return ref


def batch_differences(result: dict, ref: dict, scene: str, seed: int | None = None) -> list[str]:
    checks = {k: _normal(result.get(k)) for k in BATCH_FIELDS}
    checks["settings_synthetic"] = _settings_without_mass(result)
    checks[f"scene_layout:{scene}"] = _normal((result.get("scene_layout") or {}).get("sha256"))
    missing = []
    for name, parts, key in ((None, BATCH_PARTS, lambda k: k), (scene, PER_SCENE_PARTS, lambda k: f"{k}:{scene}"),
                             (seed, PER_SEED_PARTS, lambda k: f"{k}:s{seed}")):
        for k, part in parts.items():
            value = part(result)
            if value is None:  # a missing part is outside the allowed difference, even when every run lacks it
                missing.append(f"{key(k)} missing")
            checks[key(k)] = _normal(value)
    return missing + [k for k, v in checks.items() if ref.get(k) in (None, TIED) or v != ref[k]]


def judge(out_dir: Path, compare: dict | None = None, truck_mass_kg: float | None = None) -> dict:
    runs: dict = defaultdict(dict)  # (scene, seed) -> arm -> folder
    for folder in sorted(out_dir.iterdir()):
        m = NAME.match(folder.name)
        if not m or m["rep"]:
            continue
        runs[(m["scene"], int(m["seed"]))][m["arm"]] = folder
    status = bundle_status(out_dir)
    compare = compare or {}
    unconfirmed = {f"{s}/{a}" for s in SCENES for a in ARMS if compare.get(f"{s}/{a}") is not True}
    pairs = []
    infra = {a: 0 for a in ARMS}
    cache = {(sc, sd, a): load_run(f) for (sc, sd), arms in runs.items() for a, f in arms.items()}
    ref = batch_reference(cache)
    for (scene, seed), arms in sorted(runs.items()):
        row = {"scene": scene, "seed": seed}
        if set(arms) != set(ARMS):
            pairs.append({**row, "valid": False, "why": "an arm is missing"})
            continue
        loaded = {a: cache[(scene, seed, a)] for a in ARMS}
        inf = {a: infra_reason(loaded[a], status.get(arms[a].name)) for a in ARMS}
        for a in ARMS:
            infra[a] += inf[a] is not None
        why = next((f"{a}: infrastructure ({inf[a]})" for a in ARMS if inf[a]), None)
        why = why or next((f"{a}: {loaded[a]['invalid']}" for a in ARMS if "invalid" in loaded[a]), None)
        why = why or next((f"{a}: {p}" for a in ARMS if (p := identity_problem(loaded[a], a, seed))), None)
        why = why or (lambda b: f"fields differ: {b}" if b else None)(identity_differences(loaded["A"]["result"], loaded["B"]["result"]))
        why = why or next((f"{a}: batch snapshot differs: {b}" for a in ARMS
                           if (b := batch_differences(loaded[a]["result"], ref, scene, seed))), None)
        if why:
            pairs.append({**row, "valid": False, "why": why})
            continue
        if any(f"{scene}/{a}" in unconfirmed for a in ARMS):
            row["determinism_unconfirmed"] = True
        sp = {a: split(loaded[a]["result"], loaded[a]["control"], loaded[a]["records"], loaded[a]["match"]) for a in ARMS}
        row["failure"] = {a: loaded[a]["result"].get("failure_reason") for a in ARMS}
        ca, cb = loaded["A"]["control"], loaded["B"]["control"]
        t_div = first_divergence(ca, cb)
        row.update(t_div=t_div, t_div_phase=phase_at(loaded["A"]["result"], t_div) if t_div is not None else None,
                   initial_pallet_diff_m=max_abs_diff(loaded["A"]["result"].get("initial_pallet_m"),
                                                      loaded["B"]["result"].get("initial_pallet_m")))
        ad = {a: adoption(loaded[a]["result"]) for a in ARMS}
        row["adoption_diff"] = {k: pose_diff(ad["A"].get(k), ad["B"].get(k)) for k in set(ad["A"]) | set(ad["B"])}
        if any("e_m" not in sp[a] for a in ARMS):
            pairs.append({**row, "valid": True, "primary": False, "why": "a run did not reach round 1",
                          "split": _public(sp)})
            continue
        if any("invalid" in sp[a] for a in ARMS):
            pairs.append({**row, "valid": False, "primary": False,
                          "why": "; ".join(f"{a}: {sp[a]['invalid']}" for a in ARMS if "invalid" in sp[a]),
                          "split": _public(sp)})
            continue
        mech = {a: mechanisms(loaded[a]["result"], loaded[a]["control"], loaded[a]["records"], loaded[a]["log"],
                              loaded[a]["tyres"], sp[a], truck_mass_kg) for a in ARMS}
        row.update(valid=True, primary=True, split=_public(sp), mech=mech, delta_e_m=sp["B"]["e_m"] - sp["A"]["e_m"])
        if all("e_inh" in sp[a] for a in ARMS):
            row["delta"] = {k: sp["B"][k] - sp["A"][k] for k in ("e_inh", "e_new", "e_phase")}
        wa = phase_window(loaded["A"]["result"], "transport", None, t_end=sp["A"]["t_m"])
        wb = phase_window(loaded["B"]["result"], "transport", None, t_end=sp["B"]["t_m"])
        if wa and wb:
            hd = hausdorff(truth_path(ca, wa), truth_path(cb, wb))
            same = (hd <= SAME_PATH_HAUSDORFF_M and cusps(ca, wa) == cusps(cb, wb)
                    and replans_before(loaded["A"]["result"], sp["A"]["t_m"]) == replans_before(loaded["B"]["result"], sp["B"]["t_m"]))
            row.update(path_hausdorff_m=hd, same_path=bool(same))
        pairs.append(row)
    out = verdict(pairs, unconfirmed)
    out["infra_invalid_by_arm"] = infra
    return out


def _public(sp: dict) -> dict:
    return {a: {k: v for k, v in d.items() if not k.startswith("_")} for a, d in sp.items()}


def blocks(pairs: list[dict], key, subset=lambda p: True) -> dict[int, float]:
    by = defaultdict(dict)
    for p in pairs:
        if p.get("primary") and subset(p):
            v = key(p)
            if v is not None:
                by[p["seed"]][p["scene"]] = v
    return {s: (d["line_2"] + d["random_1"]) / 2 for s, d in by.items() if set(d) == set(SCENES)}


def j3(pairs: list[dict], subset=lambda p: True) -> dict:
    keys = sorted({k for p in pairs if p.get("primary") for k in p["mech"]["A"]
                   if isinstance(p["mech"]["A"].get(k), (int, float)) and not isinstance(p["mech"]["A"].get(k), bool)})

    def diff(p, k):
        a, b = p["mech"]["A"].get(k), p["mech"]["B"].get(k)
        ok = isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool)
        return b - a if ok else None

    return {k: one_sided(list(blocks(pairs, lambda p, k=k: diff(p, k), subset).values())) for k in keys}


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
    # J3 is described whatever J1 says (plan J3); J2 waits for J1
    out["J3"] = j3(pairs)
    out["J3_same_path"] = j3(pairs, lambda p: p.get("same_path"))
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
    return out


def pair_rows(pairs: list[dict]) -> list[dict]:
    rows = []
    for p in pairs:
        row = {"scene": p["scene"], "seed": p["seed"], "valid": p.get("valid"), "primary": p.get("primary"),
               "why": p.get("why"), "same_path": p.get("same_path"), "t_div": p.get("t_div"),
               "t_div_phase": p.get("t_div_phase")}
        for a in ARMS:
            for k in ("e_m", "e_m_world_x", "e_p", "e_phase", "e_inh", "e_new", "t_m_minus_t_c", "d_c", "t_c_kind", "t_c_gate"):
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
    ap.add_argument("--truck-mass-kg", type=float, default=26.0,
                    help="the static weight check's truck mass (the operating scene, F2 plan section 2)")
    args = ap.parse_args(argv)
    compare = json.loads(args.compare.read_text()) if args.compare else None
    res = judge(args.out, compare, args.truck_mass_kg)
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
