"""Baseline priority 3, records only: transport path-tracking failures split by the rules of
docs/plans/2026-11-14-transport-departure-classification.md (fixed before the results).

    python transport_departure.py --raw-root DIR [--raw-root DIR ...] --out OUT.json

Each DIR holds <name>/run/{result.json, slam_control.npy}; <name> is <prefix>_<arm>_<scene>_s<seed>. slam_control
columns: time, estimated rear x, y, yaw, true rear x, y, yaw (the tracker is rear-axle). Reads only.

Per transport failure (phase transport):
- cross_track: the failing leg of tracking_failure.path, measured from its start to the failing tick: dE on the
  tracker's recorded segment, dT within +-1 m path distance of the tracker's projection, the decision flag D, the onset
  marker E0, saturation S, plant shortfall P, plan margin M, inherited offset O, the ordered primary-cause table.
- endpoint_heading: the first cusp heading judgement (the first cusp_replans record) on its replaced path, D on the
  truth's yaw error against the active tolerance, S_end, Short, P and M over the leg's last metre.
- Tracking timeout in transport: the endpoint-stall pre-rule; anything else in transport: tabled as a separate ending.
Distinct trajectories are merged when the failing path, the tracker config, the failing tick and the window's
estimate/truth rows agree.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path

import numpy as np

LIMIT_M = 0.35  # recorded in tracking_failure.config; checked per run
SAT = 0.355  # |commanded curvature| at the 0.36 /m limit
MOVING = 0.05  # m/s
ONSET_M = 0.05
E0_POS, E0_YAW, E0_JUMP = 0.05, 0.03, 0.03
S_SHARE = 0.5
P_RATIO = 0.95
M_MARGIN = 0.03
O_LAT, O_YAW = 0.02, 0.02
WINDOW_AROUND_M = 1.0
SHORT_LEG_M = 3 * 0.28
END_WINDOW_M = 1.0
MIN_STEP_M = 0.02
AMBIGUOUS_M = 0.01
AMBIGUOUS_SEPARATION_M = 0.3
PAIR_TOL_S = 1e-6
DROP_LIMIT = 0.10
WHEELBASE, TRACK = 0.66, 0.53
NAME = re.compile(r"^(?:lat_r_)?(.+?)_(.+)_(random_\d+|line_\d+|aisle_rack|aisle|trailer)_s(\d+)$")


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---- path geometry -------------------------------------------------------------------------------------------------


class Path2:
    def __init__(self, path: dict):
        self.poses = np.asarray(path["poses"], dtype=float)
        self.directions = list(path["directions"])
        self.kappa = np.asarray(path["curvatures_inv_m"], dtype=float)
        self.vec = np.diff(self.poses[:, :2], axis=0)
        self.len = np.linalg.norm(self.vec, axis=1)
        self.dist = np.concatenate([[0.0], np.cumsum(self.len)])
        d = self.directions
        self.cusps = [i - 1 for i in range(1, len(d)) if d[i] * d[i - 1] < 0]  # directions[i] arrives at i
        self.ends = self.cusps + [len(self.poses) - 1]

    def leg_range(self, leg: int) -> tuple[int, int]:
        return (0 if leg == 0 else self.ends[leg - 1]), self.ends[leg]

    def on_segment(self, p, i: int) -> tuple[float, float]:
        """(distance, path distance of the clipped point) of p to segment i."""
        f = float(np.dot(np.asarray(p[:2]) - self.poses[i, :2], self.vec[i]) / max(self.len[i] ** 2, 1e-12))
        f = min(1.0, max(0.0, f))
        q = self.poses[i, :2] + f * self.vec[i]
        return float(np.hypot(*(np.asarray(p[:2]) - q))), float(self.dist[i] + f * self.len[i])

    def near_window(self, p, s0: float, lo: int, hi: int) -> tuple[float, float, int] | None:
        """Nearest clipped point among the leg's segments within +-1 m path distance of s0; None when another
        candidate within 0.01 m of it lies more than 0.3 m of path away (another part of the path: ambiguous)."""
        cands = []
        for j in range(lo, hi):
            if self.dist[j + 1] < s0 - WINDOW_AROUND_M or self.dist[j] > s0 + WINDOW_AROUND_M:
                continue
            d, s = self.on_segment(p, j)
            cands.append((d, s, j))
        if not cands:
            return None
        best = min(cands)
        if any(abs(s - best[1]) > AMBIGUOUS_SEPARATION_M and d - best[0] < AMBIGUOUS_M for d, s, _ in cands):
            return None
        return best

    def forward(self, poses, lo: int, hi: int) -> list[tuple[float, float, int]]:
        """The tracker's walk (path_tracking._project, without the advance limit): forward only, next segment while the
        fraction along the current one is >= 1."""
        i, out = lo, []
        for p in poses:
            while i + 1 < hi:
                f = float(np.dot(np.asarray(p[:2]) - self.poses[i, :2], self.vec[i]) / max(self.len[i] ** 2, 1e-12))
                if f < 1:
                    break
                i += 1
            d, s = self.on_segment(p, i)
            out.append((d, s, i))
        return out

    def frame(self, p, i: int) -> tuple[float, float]:
        """(left offset, yaw error) of pose p against segment i's direction."""
        heading = math.atan2(self.vec[i][1], self.vec[i][0])
        if self.directions[i + 1] < 0:
            heading = wrap(heading + math.pi)
        dx, dy = p[0] - self.poses[i, 0], p[1] - self.poses[i, 1]
        return -math.sin(heading) * dx + math.cos(heading) * dy, wrap(p[2] - self.poses[i + 1, 2])

    def digest(self) -> str:
        blob = json.dumps([np.round(self.poses, 9).tolist(), self.directions, np.round(self.kappa, 9).tolist()])
        return hashlib.sha256(blob.encode()).hexdigest()[:16]


# ---- records -------------------------------------------------------------------------------------------------------


def failure_of(r: dict) -> str:
    return str(r.get("failure_reason") or (r.get("tracking_failure") or {}).get("reason") or "")


def install_events(r: dict) -> list[dict]:
    """Path installs in transport: time, kind, whether the new path is recorded."""
    ev = [{"time_s": float(t["time_s"]), "kind": "transport_start", "recorded": True}
          for t in r.get("transitions") or [] if t.get("to") == "transport"]
    ev += [{"time_s": float(c["time_s"]), "kind": "cusp_replan", "recorded": True} for c in r.get("cusp_replans") or []]
    ev += [{"time_s": float(c["time_s"]), "kind": "stall_replan", "recorded": "path" in c}
           for c in r.get("stall_replans") or []]
    ev += [{"time_s": float(c["time_s"]), "kind": "paths_replaced", "recorded": True}
           for c in r.get("paths_replaced") or [] if c.get("leg") == "transport"]
    ev += [{"time_s": float(c["time_s"]), "kind": "scene_replan", "recorded": False}
           for c in r.get("scene_replans") or [] if c.get("phase") == "transport" and c.get("outcome") == "installed"]
    ob = r.get("obstacle_layer") or {}
    for key in ("replans", "mission_replans"):
        ev += [{"time_s": float(c["time_s"]), "kind": f"obstacle_{key}", "recorded": False}
               for c in ob.get(key) or [] if isinstance(c, dict) and c.get("phase") == "transport" and "time_s" in c]
    return sorted(ev, key=lambda e: e["time_s"])


def leg_start(r: dict, leg: int, installed_s: float) -> float:
    """When the tracker began leg `leg` of the path installed at installed_s (a leg switch names the leg that ended)."""
    if leg == 0:
        return installed_s
    for s in r.get("leg_switches") or []:
        if s.get("phase") == "transport" and float(s["time_s"]) > installed_s and int(s["leg"]) == leg - 1:
            return float(s["time_s"])
    raise ValueError(f"no switch into leg {leg}")


def row_at(sc: np.ndarray, t: float) -> np.ndarray:
    k = int(np.searchsorted(sc[:, 0], t + 1e-9, side="right") - 1)
    return sc[max(k, 0)]


def row_exact(sc: np.ndarray, t: float):
    """The slam_control row at t (within 1e-6), or None."""
    k = int(np.searchsorted(sc[:, 0], t - PAIR_TOL_S, side="left"))
    if k < len(sc) and abs(sc[k, 0] - t) <= PAIR_TOL_S:
        return sc[k]
    return None


def wheel_curvatures(delta) -> tuple[float, float]:
    """Signed curvature each wheel's measured steering angle implies (Ackermann, [left, right]; left turn positive)."""
    out = []
    for side, d in zip((+1, -1), delta):
        t = math.tan(float(d))
        if abs(t) < 1e-9:
            out.append(0.0)
            continue
        inner = (t > 0) == (side > 0)  # the left wheel is inner in a left turn
        # inner: tan d = L / (R - w/2) -> 1/R = tan d / (L + w/2 tan d); outer: tan d = L / (R + w/2)
        out.append(t / (WHEELBASE + (TRACK / 2) * abs(t)) if inner else t / (WHEELBASE - (TRACK / 2) * abs(t)))
    return out[0], out[1]


def saturation(samples: list[dict], sc: np.ndarray, path: Path2, t_end: float, select) -> dict:
    """Time-weighted saturation share over moving samples that `select` keeps, and P, M over direction-agreeing
    saturated samples."""
    rows = [s for s in samples if select(s) and s.get("_row") is not None]
    moving_w = sat_w = agree_w = 0.0
    ratios, margins, deficits, wheel = [], [], [], []
    for k, s in enumerate(rows):
        t = float(s["time_s"])
        t_next = float(rows[k + 1]["time_s"]) if k + 1 < len(rows) else t_end
        w = max(0.0, t_next - t)
        if abs(float(s.get("signed_speed_mps", 0.0))) <= MOVING:
            continue
        moving_w += w
        kc = float(s["curvature_inv_m"])
        if abs(kc) < SAT:
            continue
        sat_w += w
        a = s["_row"]
        b = rows[k + 1]["_row"] if k + 1 < len(rows) else row_exact(sc, t_end)
        if b is None:
            continue
        step = math.hypot(b[4] - a[4], b[5] - a[5])
        if step < MIN_STEP_M:
            continue
        kach = wrap(b[6] - a[6]) / step * (1 if float(s["signed_speed_mps"]) > 0 else -1)
        seg = s.get("_seg")
        kplan = float(path.kappa[seg + 1]) if seg is not None else None
        if kplan is None or not (np.sign(kc) == np.sign(kach) == np.sign(kplan) != 0):
            continue
        agree_w += w
        ratios.append((w, kach / kc))
        margins.append((w, np.sign(kplan) * (kach - kplan)))
        cmd, act = s.get("steering_command_rad"), s.get("steering_actual_rad")
        if cmd and act:
            deficits.append((w, [float(c) - float(x) for c, x in zip(cmd, act)]))
            wheel.append((w, wheel_curvatures(act)))

    def mean(pairs):
        tw = sum(w for w, _ in pairs)
        return None if tw <= 0 else sum(w * v for w, v in pairs) / tw

    def mean_vec(pairs):
        tw = sum(w for w, _ in pairs)
        return None if tw <= 0 else [sum(w * v[i] for w, v in pairs) / tw for i in range(2)]

    known = sat_w > 0 and agree_w >= 0.5 * sat_w
    ratio, margin = mean(ratios), mean(margins)
    return {"moving_s": round(moving_w, 3), "saturated_s": round(sat_w, 3), "agreeing_s": round(agree_w, 3),
            "S": None if moving_w <= 0 else bool(sat_w / moving_w >= S_SHARE),
            "share": None if moving_w <= 0 else round(sat_w / moving_w, 3),
            "ach_over_cmd": None if ratio is None else round(ratio, 4),
            "ach_minus_plan": None if margin is None else round(margin, 4),
            "P": bool(ratio < P_RATIO) if known and ratio is not None else None,
            "M": bool(margin < M_MARGIN) if known and margin is not None else None,
            "steer_deficit": None if not deficits else [round(v, 4) for v in mean_vec(deficits)],
            "wheel_curvature": None if not wheel else [round(v, 4) for v in mean_vec(wheel)]}


def primary_cross(S, E0, P, M, unknown_path: bool) -> str:
    if unknown_path or S is None:
        return "unresolved"
    if not S:
        return {True: "unsaturated_estimate_onset_marker", False: "tracking_unsaturated"}.get(E0, "unresolved")
    if P is None or M is None:
        return "unresolved_saturated_no_curvature"
    return {(True, True): "mixed_plan_margin_and_plant", (True, False): "plant_shortfall",
            (False, True): "plan_margin", (False, False): "unresolved_saturated_margins_ok"}[(P, M)]


# ---- cross_track ---------------------------------------------------------------------------------------------------


def cross_track(r: dict, sc: np.ndarray) -> dict:
    tf = r["tracking_failure"]
    path = Path2(tf["path"])
    t_fail = float(tf["time_s"])
    leg = int(tf["leg"])
    lo, hi = path.leg_range(leg)
    events = install_events(r)
    before = [e for e in events if e["time_s"] <= t_fail + 1e-9]
    installed = before[-1]["time_s"] if before else float(tf["phase_started_s"])
    start = leg_start(r, leg, installed)
    inside = [e for e in events if start + 1e-9 < e["time_s"] < t_fail - 1e-9]
    out = {"kind": "cross_track", "leg": leg, "leg_range": [lo, hi], "leg_start_s": round(start, 4),
           "t_fail": round(t_fail, 4), "path_sha": path.digest(), "installs_inside_window": [e["kind"] for e in inside],
           "limit_m": tf["config"]["max_cross_track_error_m"]}
    # the failing tick: the judged pose, the tracker's segment
    seg = int(tf["tracking"]["segment_index"])
    est = tf["rear_pose"]
    tru = row_at(sc, t_fail)[4:7]
    dE, sE = path.on_segment(est, seg)
    nw = path.near_window(tru, sE, lo, hi)
    if nw is None:
        raise ValueError("ambiguous truth projection at the failing tick")
    dT, sT, segT = nw
    row = row_at(sc, t_fail)
    out.update({"dE": round(dE, 4), "dT": round(dT, 4), "L": round(dE - dT, 4), "progress_gap_m": round(sT - sE, 4),
                "D": bool(dE > out["limit_m"] and dT <= out["limit_m"]),
                "row_overwritten": bool(np.max(np.abs(np.asarray(row[1:4]) - np.asarray(est))) > 1e-6),
                "E_minus_T_m": round(math.hypot(est[0] - tru[0], est[1] - tru[1]), 4),
                "yaw_E_minus_T": round(wrap(est[2] - tru[2]), 4)})
    # the window's series at the samples (the tracker's segment), and the truth's onset
    samples = [s for s in r.get("samples") or [] if s.get("phase") == "transport" and start - 1e-9 <= float(s["time_s"]) <= t_fail]
    series, dropped = [], 0
    for s in samples:
        t = float(s["time_s"])
        trk = s.get("tracking") or {}
        row = row_exact(sc, t)
        sg = trk.get("segment_index")
        s["_seg"], s["_row"] = None, None
        if row is None or sg is None or not (lo <= int(sg) < hi):
            dropped += 1
            continue
        de, se = path.on_segment(row[1:4], int(sg))
        nw = path.near_window(row[4:7], se, lo, hi)
        if nw is None:
            dropped += 1
            continue
        s["_seg"], s["_row"] = int(sg), row
        series.append({"t": t, "dE": de, "dT": nw[0], "ET": math.hypot(row[1] - row[4], row[2] - row[5]),
                       "yawET": wrap(row[3] - row[6])})
    out["samples_window"], out["samples_dropped"] = len(samples), dropped
    series.append({"t": t_fail, "dE": dE, "dT": dT, "ET": out["E_minus_T_m"], "yawET": out["yaw_E_minus_T"]})
    onset = None
    for x in series:
        if x["dT"] <= ONSET_M:
            onset = x
    if onset is None:
        out.update({"onset": None, "E0": None})
    else:
        prior = [x for x in series if onset["t"] - 0.5 - 1e-9 <= x["t"] <= onset["t"]]
        jump = max((onset["ET"] - x["ET"] for x in prior), default=0.0)
        out["onset"] = {"t": round(onset["t"], 3), "ET": round(onset["ET"], 4), "yawET": round(onset["yawET"], 4),
                        "jump_0_5s": round(jump, 4)}
        out["E0"] = bool(onset["ET"] >= E0_POS or abs(onset["yawET"]) >= E0_YAW or jump > E0_JUMP)
    t0 = onset["t"] if onset else start
    out["sat"] = saturation(samples, sc, path, t_fail, lambda s: s.get("_seg") is not None and float(s["time_s"]) >= t0 - 1e-9)
    # reference slopes of dT (not used in the judgement)
    out["dT_slope"] = {}
    for w in (0.5, 1.0, 2.0):
        pts = [x for x in series if x["t"] >= t_fail - w - 1e-9]
        out["dT_slope"][str(w)] = (round((pts[-1]["dT"] - pts[0]["dT"]) / (pts[-1]["t"] - pts[0]["t"]), 4)
                                  if len(pts) >= 2 and pts[-1]["t"] > pts[0]["t"] else None)
    # the inherited offset at the leg start
    a = row_at(sc, start)
    first = path.forward([a[4:7]], lo, hi)[0]
    lat, yaw = path.frame(a[4:7], first[2])
    out["leg_start_truth"] = {"lateral_m": round(lat, 4), "yaw_rad": round(yaw, 4)}
    e_lat, e_yaw = path.frame(a[1:4], path.forward([a[1:4]], lo, hi)[0][2])
    out["leg_start_estimate"] = {"lateral_m": round(e_lat, 4), "yaw_rad": round(e_yaw, 4)}
    out["O"] = bool(abs(lat) > O_LAT or abs(yaw) > O_YAW)
    out["max_dT_leg"] = round(max(x["dT"] for x in series), 4)
    out["unknown_path"] = bool(inside) or (len(samples) > 0 and dropped / len(samples) > DROP_LIMIT)
    out["primary"] = primary_cross(out["sat"]["S"], out["E0"], out["sat"]["P"], out["sat"]["M"], out["unknown_path"])
    out["shape"] = leg_shape(path, leg)
    out["whole_leg"] = measure_leg(r, sc, path, leg, start, t_fail)
    # the window's paired sample rows and the failing tick's row (what a --fields file carries too)
    paired = [s["_row"] for s in samples if s.get("_row") is not None] + [row_at(sc, t_fail)]
    out["window_digest"] = hashlib.sha256(json.dumps(np.round(np.asarray(paired), 6).tolist()).encode()).hexdigest()[:16]
    return out


# ---- endpoint_heading ----------------------------------------------------------------------------------------------


def endpoint_heading(r: dict, sc: np.ndarray) -> dict:
    tf = r["tracking_failure"]
    first = (r.get("cusp_replans") or [None])[0]
    out = {"kind": "endpoint_heading", "final": {"leg": tf["leg"], "yaw_error_rad": tf["tracking"]["yaw_error_rad"],
                                                 "at_cusp": tf["tracking"]["at_cusp"]},
           "cusp_replans": len(r.get("cusp_replans") or [])}
    if first is None:
        judged, path_d, t = tf, tf["path"], float(tf["time_s"])
        e_E = float(tf["tracking"]["yaw_error_rad"])
    else:
        judged, path_d, t = first, first["replaced_path"], float(first["time_s"])
        e_E = float(first["yaw_error_rad"])
    path = Path2(path_d)
    rear = judged["rear_pose"]
    # the leg whose end the truck stood at: the leg end nearest the judged pose
    end = min(path.ends, key=lambda i: math.hypot(path.poses[i, 0] - rear[0], path.poses[i, 1] - rear[1]))
    leg = path.ends.index(end)
    lo, hi = path.leg_range(leg)
    tau = 0.05 if end != len(path.poses) - 1 else 0.02
    tru = row_at(sc, t)[4:7]
    e_T = wrap(path.poses[end, 2] - tru[2])
    events = install_events(r)
    before = [e for e in events if e["time_s"] < t - 1e-9]
    installed = before[-1]["time_s"] if before else float(tf["phase_started_s"])
    start = leg_start(r, leg, installed)
    inside = [e for e in events if start + 1e-9 < e["time_s"] < t - 1e-9]
    samples = [s for s in r.get("samples") or [] if s.get("phase") == "transport" and start - 1e-9 <= float(s["time_s"]) <= t]
    end_dist = path.dist[end]
    for s in samples:
        trk = s.get("tracking") or {}
        sg = trk.get("segment_index")
        s["_row"] = row_exact(sc, float(s["time_s"]))
        s["_seg"] = int(sg) if sg is not None and lo <= int(sg) < hi and s["_row"] is not None else None
    sat = saturation(samples, sc, path, t, lambda s: s.get("_seg") is not None
                     and end_dist - path.dist[s["_seg"]] <= END_WINDOW_M)
    a = row_at(sc, start)
    fw = path.forward([a[4:7]], lo, hi)[0]
    lat, yaw = path.frame(a[4:7], fw[2])
    leg_len = float(path.dist[hi] - path.dist[lo])
    D = abs(e_E) > tau and abs(e_T) <= tau
    out.update({"t_judged": round(t, 4), "leg": leg, "leg_length_m": round(leg_len, 3), "tau": tau,
                "e_E": round(e_E, 4), "e_T": round(e_T, 4), "D": bool(D), "sat_end": sat,
                "Short": leg_len < SHORT_LEG_M, "leg_start_truth": {"lateral_m": round(lat, 4), "yaw_rad": round(yaw, 4)},
                "O": bool(abs(lat) > O_LAT or abs(yaw) > O_YAW), "installs_inside_window": [e["kind"] for e in inside],
                "path_sha": path.digest()})
    if inside or abs(e_E) <= tau:
        out["primary"] = "unresolved"
    elif D:
        out["primary"] = "localisation_decision"
    elif sat["S"]:
        out["primary"] = primary_cross(True, None, sat["P"], sat["M"], False)
    elif out["Short"]:
        out["primary"] = "plan_short_leg"
    else:
        out["primary"] = "tracking_unsaturated"
    return out


# ---- reference range (completed runs, plan 3.3) --------------------------------------------------------------------


def leg_shape(path: Path2, leg: int) -> dict:
    lo, hi = path.leg_range(leg)
    signs = []
    for k in path.kappa[lo + 1: hi + 1]:
        sg = int(np.sign(k)) if abs(k) > 1e-9 else 0
        if sg and (not signs or signs[-1] != sg):
            signs.append(sg)
    return {"direction": int(path.directions[lo + 1]), "signs": signs, "length_m": float(path.dist[hi] - path.dist[lo]),
            "max_kappa": float(np.max(np.abs(path.kappa[lo + 1: hi + 1]))) if hi > lo else 0.0}


def same_shape(a: dict, b: dict) -> bool:
    return a["direction"] == b["direction"] and a["signs"] == b["signs"] and abs(a["length_m"] - b["length_m"]) < 0.2


def installed_paths(r: dict) -> list[tuple[float, dict | None]]:
    """(install time, path or None when not recorded) over transport, from the recorded replacements."""
    t0 = next((float(t["time_s"]) for t in r.get("transitions") or [] if t.get("to") == "transport"), None)
    if t0 is None:
        return []
    ev = [(float(c["time_s"]), c.get("replaced_path"), c.get("path")) for c in r.get("cusp_replans") or []]
    ev += [(float(c["time_s"]), c.get("path"), None) for c in r.get("paths_replaced") or [] if c.get("leg") == "transport"]
    ev += [(e["time_s"], None, None) for e in install_events(r) if not e["recorded"]]
    ev.sort(key=lambda e: e[0])
    final = (r.get("paths") or {}).get("transport")
    first = ev[0][1] if ev else final
    out = [(t0, first)]
    for k, (t, _old, new) in enumerate(ev):
        nxt = ev[k + 1][1] if k + 1 < len(ev) else final
        out.append((t, new if new is not None else nxt))
    return out


def measure_leg(r: dict, sc: np.ndarray, path: Path2, leg: int, t_start: float, t_end: float) -> dict | None:
    """The whole-leg measures a reference leg and a failing leg share: max dT, saturation, P/M values, the inherited
    offset, wheel curvature. None when the tracker's samples do not cover the leg."""
    lo, hi = path.leg_range(leg)
    samples = [s for s in r.get("samples") or [] if s.get("phase") == "transport"
               and t_start - 1e-9 <= float(s["time_s"]) <= t_end]
    dts, kept = [], 0
    for s in samples:
        trk = s.get("tracking") or {}
        sg = trk.get("segment_index")
        row = row_exact(sc, float(s["time_s"]))
        s["_seg"], s["_row"] = None, None
        if row is None or sg is None or not (lo <= int(sg) < hi):
            continue
        _, se = path.on_segment(row[1:4], int(sg))
        nw = path.near_window(row[4:7], se, lo, hi)
        if nw is None:
            continue
        s["_seg"], s["_row"] = int(sg), row
        kept += 1
        dts.append(nw[0])
    if not samples or kept < 0.9 * len(samples) or kept < 5:
        return None
    sat = saturation(samples, sc, path, t_end, lambda s: s.get("_seg") is not None)
    a = row_at(sc, t_start)
    lat, yaw = path.frame(a[4:7], path.forward([a[4:7]], lo, hi)[0][2])
    e_lat, e_yaw = path.frame(a[1:4], path.forward([a[1:4]], lo, hi)[0][2])
    return {"max_dT": round(max(dts), 4), "leg_start_est_lateral_m": round(e_lat, 4),
            "leg_start_est_yaw_rad": round(e_yaw, 4), "sat_share": sat["share"], "ach_over_cmd": sat["ach_over_cmd"],
            "ach_minus_plan": sat["ach_minus_plan"], "wheel_curvature": sat["wheel_curvature"],
            "steer_deficit": sat["steer_deficit"], "leg_start_lateral_m": round(lat, 4), "leg_start_yaw_rad": round(yaw, 4)}


def reference_legs(name: str, r: dict, sc: np.ndarray, shapes: list[dict]) -> list[dict]:
    """Every leg of a completed run's transport that has the shape of one of the failing legs."""
    out = []
    installs = installed_paths(r)
    replans = len(installs) - 1
    for k, (t_in, pd) in enumerate(installs):
        if pd is None:
            continue
        t_out = installs[k + 1][0] if k + 1 < len(installs) else math.inf
        path = Path2(pd)
        for leg in range(len(path.ends)):
            shape = leg_shape(path, leg)
            hits = [i for i, f in enumerate(shapes) if same_shape(shape, f)]
            if not hits:
                continue
            try:
                start = leg_start(r, leg, t_in)
            except ValueError:
                continue
            end = next((float(s["time_s"]) for s in r.get("leg_switches") or [] if s.get("phase") == "transport"
                        and float(s["time_s"]) > start and int(s["leg"]) == leg), None)
            if end is None or end > t_out:
                continue
            m = measure_leg(r, sc, path, leg, start, end)
            if m is not None:
                out.append({"name": name, "leg": leg, "shapes": hits, "replans_in_transport": replans, **m})
    return out


# ---- timeouts and the rest -----------------------------------------------------------------------------------------


def timeout(r: dict) -> dict:
    samples = [s for s in r.get("samples") or [] if s.get("phase") == "transport" and s.get("tracking")]
    if not samples:
        return {"kind": "timeout", "primary": "unresolved", "why": "no tracking samples"}
    s = samples[-1]
    trk = s["tracking"]
    stall = (trk.get("status") == "tracking" and abs(float(s.get("speed_command_mps", 0.0))) < 1e-9
             and float(trk.get("position_error_m", 1.0)) <= 0.03 + 1e-9)
    return {"kind": "timeout", "last_sample": {"t": s["time_s"], **{k: trk.get(k) for k in
            ("status", "position_error_m", "yaw_error_rad", "at_cusp", "segment_index")},
            "speed_command_mps": s.get("speed_command_mps")},
            "endpoint_stall": bool(stall), "primary": "endpoint_stall" if stall else "separate_ending",
            "stops": {k: len(r.get(k) or []) for k in ("cusp_replans", "stall_replans", "cusp_stalls")}}


def classify_run(name: str, r: dict, sc: np.ndarray | None) -> dict | None:
    if r.get("phase") != "transport" and "transport" not in failure_of(r):
        return None
    base = {"name": name, "failure": failure_of(r)[:80]}
    kind = ((r.get("tracking_failure") or {}).get("tracking") or {}).get("failure")
    try:
        if kind == "cross_track":
            return {**base, **cross_track(r, sc)}
        if kind == "endpoint_heading":
            return {**base, **endpoint_heading(r, sc)}
        if failure_of(r).startswith("Tracking timeout in transport"):
            return {**base, **timeout(r)}
    except (ValueError, KeyError, IndexError, TypeError) as exc:
        return {**base, "kind": kind or "other", "primary": "unresolved", "error": repr(exc)[:200]}
    return {**base, "kind": "separate_ending", "primary": "separate_ending"}


def merge(rows: list[dict]) -> list[dict]:
    """Distinct trajectories: same path digest, failing tick, segment-level values and window rows."""
    cases: dict = {}
    for x in rows:
        key = (x.get("kind"), x.get("path_sha"), x.get("t_fail") or x.get("t_judged"), x.get("window_digest"),
               x.get("dE"), x.get("e_E"))
        if x.get("kind") in ("separate_ending", "timeout") or x.get("path_sha") is None:
            key = ("single", x["name"])
        cases.setdefault(key, []).append(x)
    out = []
    for members in cases.values():
        head = dict(min(members, key=lambda m: m["name"]))  # the same head whatever order the runs came in
        head["runs"] = sorted(m["name"] for m in members)
        out.append(head)
    return out


class _Run:
    """One run's folder or its entry in a --fields file: name, root, result, slam_control (or None)."""

    def __init__(self, root: str, name: str, result: dict, sc):
        self.root, self.name, self.result, self.sc = root, name, result, sc


def runs_from(raw_roots: list[Path], fields: Path | None) -> list[_Run]:
    out = []
    if fields is not None:
        import gzip
        with gzip.open(fields, "rt") as f:
            blob = json.load(f)
        for key, v in sorted(blob.items()):
            root, name = key.split("/", 1)
            rows = np.asarray(v["slam_control_rows"], dtype=float).reshape(-1, 7)
            out.append(_Run(root, name, v["result"], rows if len(rows) else None))
        return out
    for root in raw_roots:
        for d in sorted(root.iterdir()):
            p = d / "run" / "result.json"
            if not p.is_file():
                continue
            scp = d / "run" / "slam_control.npy"
            out.append(_Run(root.name, d.name, json.loads(p.read_text()), np.load(scp) if scp.is_file() else None))
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw-root", action="append", type=Path, default=[])
    ap.add_argument("--fields", type=Path, help="the reduced fields file (docs/validation/assets/transport-departure)")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    if bool(a.raw_root) == bool(a.fields):
        raise SystemExit("give --raw-root DIR... or --fields FILE")
    runs = runs_from(a.raw_root, a.fields)
    rows = []
    for d in runs:
        r, sc = d.result, d.sc
        if sc is None and r.get("phase") == "transport":
            rows.append({"name": d.name, "failure": failure_of(r)[:80], "kind": "no_slam_control",
                         "primary": "unresolved", "root": d.root})
            continue
        x = classify_run(d.name, r, sc)
        if x is not None:
            x["root"] = d.root
            rows.append(x)
    cases = merge(rows)
    # the reference range: completed runs of the same batch and scene with a leg of a failing leg's shape
    for root in sorted({run.root for run in runs}):
        mine = [c for c in cases if c.get("kind") == "cross_track" and c.get("shape")
                and any(rw["root"] == root for rw in rows if rw["name"] in c["runs"])]
        if not mine:
            continue
        for d in runs:
            m = NAME.match(d.name)
            if d.root != root or d.sc is None or not m:
                continue
            r = d.result
            if not r.get("success"):
                continue
            scene = m[3]
            shapes = [c["shape"] for c in mine if NAME.match(c["runs"][0]) and NAME.match(c["runs"][0])[3] == scene]
            if not shapes:
                continue
            for hit in reference_legs(d.name, r, d.sc, shapes):
                hit["root"] = root
                for i in hit["shapes"]:
                    mine_i = [c for c in mine if NAME.match(c["runs"][0])[3] == scene][i]
                    mine_i.setdefault("reference", []).append({k: v for k, v in hit.items() if k != "shapes"})
    tally: dict = {}
    for c in cases:
        tally.setdefault(c.get("kind"), {}).setdefault(c.get("primary"), 0)
        tally[c["kind"]][c["primary"]] += 1
    out = {"runs": rows, "cases": cases, "tally": tally}
    a.out.write_text(json.dumps(out, indent=1, default=float, allow_nan=False) + "\n")
    for c in cases:
        print(c.get("kind"), c.get("primary"), c["runs"])
    print(json.dumps(tally))


if __name__ == "__main__":
    main()
