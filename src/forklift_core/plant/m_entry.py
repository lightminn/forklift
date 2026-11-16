"""G8 entry: bridge record + IMU record + truth -> per-trial M values (docs/design/2026-11-15-m-analyze-entry.md).

``analyze`` checks identity (section 2), voids what 3.4 voids, and computes M1 (3.5) with the report-only IMU block,
M2 (3.6, PR 2) and M3 (3.7 merge points and moving-point IMU heading, 3.8 q-step delay; PR 3a), and the in-trial
bias-slope check judged afterwards over every record of the IMU stretch (``block_rows``; registration revision 3,
10.1; entry 3.10, PR 3b). No threshold of the registered reading is applied beyond the truth budget and the IMU
limits: the output (section 5) is per-trial values plus whether each can be judged.
"""

from __future__ import annotations

import math

import numpy as np

from forklift_core.live import drive_profile as dprof
from forklift_core.live.synthetic import SyntheticRefused, refuse_or_mark
from forklift_core.mcu_link.messages import FaultFlag
from forklift_core.plant import m_analysis as ma
from forklift_core.plant import m_rev4 as r4
from forklift_core.plant.clock_map import clock_fits_from_rows, fit_clock
from forklift_core.plant.record_rows import sample_from_row
from forklift_core.sensors.imu_mount import _unit, _vec3
from forklift_core.sensors.imu_record import RESTART_BACK_US, continuous_segments, imu_stream

REGISTRATION_DOC = "docs/plans/2026-11-15-m1-m3-registration.md"
REGISTERED = {
    "revision": "초안 개정 2", "quiet_max_s": 60.0, "quiet_min_s": 50.0, "quiet_gyro_tol_rad_s": 0.025,
    "quiet_accel_tol_mps2": 0.15, "bias_jump_rad_s": 1.5e-4, "bias_slope_rad_s2": 3.5e-7,
    "desk_n_rad_sqrt_s": 2.054e-4, "budget_dyaw_rad": 1.0e-3, "budget_lat_m": 1.7e-3,
    "k_limit_merge_end_l09": 1.48, "k_limit_run_042_l09": 1.14, "free_rotation_rad": 0.04,
    "accel_ramp_s": 0.125, "smooth_window_s": 0.025, "pre_s": 0.5, "frac": 0.63,
    # the k-limit inputs (registration 6.3: N, W, the warm-up's residual bias drift x D = W + T, the bias instability)
    "bias_window_s": 60.0, "warm_drift_rad_s2": 7.8e-8, "bias_instability_rad_s": 3.2e-5,
    "q_threshold_rad": 0.02,  # M3 q step onset (lead decision, D4 provisional values)
}
IMU_GAP_NS = 10_000_000  # an IMU quiet run breaks past this (entry 9)
MAX_GAP_S = 0.05  # STATE gap that ends a standing stretch / breaks odometry (entry 9)
STAND_SPEED_MPS = 0.005  # standing thresholds: proposed, to be confirmed (entry 7.3)
STAND_DQ_RAD = 5e-4
MEASUREMENTS = (("m1_", "m1"), ("m2_", "m2"), ("m3_merge_", "m3_merge"), ("m3_qstep_", "m3_qstep"))
SCHEMAS = dict(record="teleop_bridge_record/1", imu="imu_record/1", truth="truth/1", field="field_log/1")
TRUTH_METHODS = ("A_floor_marks", "B_camera", "synthetic", "tape")  # tape: registration revision 4 (11.2)
REHEARSAL_TRUTH_SCHEMA = "rehearsal_truth/1"
POINT_KINDS = dict(start="stop_pose", end="stop_pose", merge_end="moving_point", run_042="moving_point")
FAULT_VOIDS = tuple(FaultFlag[n] for n in ("ENCODER_GAP", "ENCODER_JUMP_L", "ENCODER_JUMP_R", "ENCODER_STUCK_L",
                                           "ENCODER_STUCK_R", "ENCODER_POLARITY_L", "ENCODER_POLARITY_R",
                                           "STEER_SENSOR_INVALID"))
ODO_VOIDS = ("gap", "boot_changed", "encoder_continuity", "steering_invalid", "time_not_increasing")


class Refused(Exception):
    def __init__(self, why: str) -> None:
        super().__init__(why)
        self.why = why


def _need(cond, why: str) -> None:
    if not cond:
        raise Refused(why)


def _header(rows, name: str) -> dict:
    _need(bool(rows) and isinstance(rows[0], dict) and rows[0].get("t") == "header",
          f"{name}: the first row is not a header")
    schema = rows[0].get("schema")
    _need(schema == SCHEMAS[name], f"{name}: unknown schema {schema!r}")
    return rows[0]


def _finite(*v) -> bool:
    return all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v)


def _check_tape(rows, trial_ids, hdr) -> dict:
    """The tape truth of revision 4 (11.2): per trial the left rear axle end's distance along the start tape line
    (``s_cm``) and its offset to the left of it (``y_cm``), at the end stop."""
    h = rows[0]
    _need(h.get("body_point") == "left_rear_axle_end", "truth tape: body_point is not left_rear_axle_end")
    _need(h.get("frame") == "start_tape_line", "truth tape: frame is not start_tape_line")
    rec = h.get("record") or {}
    _need(rec.get("host_boot_id") == hdr.get("host_boot_id"), "truth: host_boot_id differs from the record")
    _need(rec.get("start_mono_ns") == hdr.get("start_mono_ns"), "truth: record.start_mono_ns differs from the record")
    _need(_finite((h.get("sigma") or {}).get("pos_m")), "truth tape: header sigma needs pos_m")
    out: dict = {}
    for r in rows[1:]:
        kind = r.get("t")
        if kind != "tape":
            _need(kind == "void", f"truth tape: unknown row type {kind!r}")
            continue
        n = r.get("trial")
        _need(n in trial_ids, f"truth tape: row for trial {n!r}, which the record does not have")
        _need(n not in out, f"truth tape: two rows for trial {n}")
        _need(_finite(r.get("s_cm"), r.get("y_cm")) and r["s_cm"] > 0, f"truth tape trial {n}: s_cm, y_cm")
        out[n] = {"s_cm": float(r["s_cm"]), "y_cm": float(r["y_cm"])}
    return out


def _check_truth(rows, trial_ids, hdr) -> dict:
    h = _header(rows, "truth")
    _need(h.get("method") in TRUTH_METHODS, "truth: unknown method")
    # a synthetic method must carry the synthetic mark, or synthetic truth would pass unmarked into judgeable values
    # (Codex D4 impl review P1)
    _need(h.get("method") != "synthetic" or h.get("synthetic") is True,
          "truth: method synthetic without the synthetic mark")
    _need(h.get("body_point") == "rear_axle_center", "truth: body_point is not rear_axle_center")
    _need(h.get("frame") == "floor", "truth: frame is not floor")
    rec = h.get("record") or {}
    _need(rec.get("host_boot_id") == hdr.get("host_boot_id"), "truth: host_boot_id differs from the record")
    _need(rec.get("start_mono_ns") == hdr.get("start_mono_ns"), "truth: record.start_mono_ns differs from the record")
    sg = h.get("sigma") or {}
    _need(_finite(sg.get("pose_xy_m"), sg.get("pose_yaw_rad")), "truth: header sigma needs pose_xy_m, pose_yaw_rad")
    out: dict = {}
    for r in rows[1:]:
        kind = r.get("t")
        if kind != "truth":
            _need(kind == "void", f"truth: unknown row type {kind!r}")
            continue
        n = r.get("trial")
        _need(n in trial_ids, f"truth: row for trial {n!r}, which the record does not have")
        _need(n not in out, f"truth: two rows for trial {n}")
        pts = r.get("points")
        _need(isinstance(pts, dict), f"truth trial {n}: no points")
        for pid, p in pts.items():
            _need(pid in POINT_KINDS and isinstance(p, dict) and p.get("kind") == POINT_KINDS[pid],
                  f"truth trial {n}: point {pid!r} unknown or of the wrong kind")
            _need(_finite(p.get("x_m"), p.get("y_m"), p.get("yaw_rad")), f"truth trial {n}: point {pid!r} fields")
            if p["kind"] == "moving_point":
                _need(isinstance(p.get("host_ns"), int) and p.get("cov") is not None,
                      f"truth trial {n}: moving point {pid!r} needs host_ns and cov")
            if p.get("cov") is not None:
                try:
                    ma._cov3(p["cov"], "cov")
                except ValueError as e:
                    raise Refused(f"truth trial {n}: point {pid!r} {e}") from e
        out[n] = pts
    return out


def _field_voids(rows_list) -> dict:
    voids: dict = {}
    for rows in rows_list:
        for r in rows or ():
            if r.get("t") == "void":
                _need(r.get("closed_before_results") is True, "a field void row is not closed_before_results")
                voids[int(r["trial"])] = str(r.get("reason"))
    return voids


def _check_field(rows, hdr, starts) -> None:
    h = _header(rows, "field")
    rec = h.get("record") or {}
    _need(rec.get("host_boot_id") == hdr.get("host_boot_id") and rec.get("start_mono_ns") == hdr.get("start_mono_ns"),
          "field_log: record identity differs from the record")
    last = None
    starts = sorted(starts)
    for r in rows[1:]:
        if r.get("t") != "bias_recheck":
            continue
        x, a, b = r.get("exceeded_host_ns"), r.get("recheck_start_ns"), r.get("recheck_end_ns")
        _need(_finite(x, a, b) and a <= b, "field_log: bad bias_recheck times")
        _need(last is None or x >= last, "field_log: rows are not in time order")
        last = x
        nxt = next((s for s in starts if s > x), None)
        _need(nxt is None or b <= nxt, "field_log: a recheck ends after the next trial started")


def _identity_profile(rec_rows, profile, trs) -> None:
    starts = [r for r in rec_rows if r.get("t") == "profile_start" and not r.get("refused")]
    _need(bool(starts), "the record has no started profile (profile_start)")  # Codex D4 impl review P3
    for r in starts:
        _need(r.get("profile") == profile["name"], "profile_start names another profile than the given one")
    segs = profile.get("segments") or []
    labels = {s["label"] for s in segs}
    speeds = {s["label"]: float(s["speed_mps"]) for s in segs if "speed_mps" in s}
    seen: set = set()
    for tr in trs:
        seen |= set(tr.spans)
    _need(seen <= labels, "profile: a record label is not in the profile")
    _need(not any(t.valid for t in trs) or seen == labels, "profile: the record's labels differ from the profile's")
    prof_rows = [r for r in rec_rows if r.get("t") == "profile" and r.get("state") == "running"]
    for r in prof_rows:
        if r.get("label") in speeds:
            v, req = speeds[r["label"]], float(r.get("req_mps") or 0.0)
            _need(req * v >= 0 and abs(req) <= abs(v) * (1 + 1e-6), f"profile: req_mps {req} against speed {v}")
    for tr in trs:
        if tr.valid:
            for label, v in speeds.items():
                top = max((abs(float(r.get("req_mps") or 0.0)) for r in prof_rows
                           if r.get("trial") == tr.trial and r.get("label") == label), default=0.0)
                _need(abs(top - abs(v)) <= 1e-6, f"profile: trial {tr.trial} {label!r} speed differs from the profile")


# --- time ----------------------------------------------------------------------------------------------------------

def _state_times(rec_rows):
    """STATE rows -> (rows, host ns on the fits' clock, fit index per row, fits)."""
    fits = clock_fits_from_rows(rec_rows)
    rows, times, fi, extra = [], [], [], []
    key, g, hb = None, -1, None
    for r in rec_rows:
        if r.get("t") == "header":
            hb = r.get("host_boot_id")
        if r.get("t") != "state":
            continue
        h = r.get("read_ns", r.get("host_ns"))
        if h is None:
            continue
        if (hb, r.get("boot_id")) != key:
            key, g = (hb, r.get("boot_id")), g + 1
        f = fits[g]
        rows.append(r)
        fi.append(g)
        if f.valid:
            t_host, extrapolated = f.to_host_ns(r["mcu_time_us"])[:2]
            times.append(int(round(t_host)))
            extra.append(bool(extrapolated))
        else:
            times.append(int(h))
            extra.append(False)
    return rows, np.asarray(times, dtype=np.int64), fi, fits, extra


def _imu_series(rows, stream):
    """One stream on the host clock. A sample whose host time does not advance, or whose device time repeats or
    steps back by no more than ``RESTART_BACK_US`` (a late sample), is dropped; a larger step back is a device
    restart (a D435i reconnect) and is kept -- ``continuous_segments`` starts a new stretch there by the same rule,
    so the samples after it are not lost until they pass the old maximum (entry 3.10; lead 2026-11-16)."""
    st = imu_stream(rows, stream=stream)
    keep, dropped, ld, lh = [], 0, None, None
    for i, (d, h) in enumerate(zip(st.dev_us.tolist(), st.host_ns.tolist())):
        if ld is not None and (h <= lh or (d <= ld and ld - d <= RESTART_BACK_US)):
            dropped += 1
            continue
        keep.append(i)
        ld, lh = d, h
    dev, host, xyz = st.dev_us[keep], st.host_ns[keep], st.xyz[keep]
    segs = continuous_segments(dev, host)
    fits = [fit_clock(dev[a:b].tolist(), host[a:b].tolist(), host_boot=st.host_boot) for a, b in segs]
    t = None
    if fits and all(f.valid for f in fits):
        t = np.concatenate([np.round(f.offset_ns + f.slope * dev[a:b]).astype(np.int64)
                            for f, (a, b) in zip(fits, segs, strict=True)])
    return dict(t=t, xyz=xyz, fits=fits, dropped=dropped, domains=st.domains, clock=st.host_clock,
                boundaries=[int(host[a]) for a, _ in segs[1:]], boot=st.host_boot)


def _standing(times, states, cfg):
    """Host intervals where both wheels are slow, the steering still and no STATE missing (entry 3.3 5)."""
    ok = []
    for s in states:
        v = max(abs(s.wheel_rate_left_rad_s) * cfg.wheel_radius_m[0],
                abs(s.wheel_rate_right_rad_s) * cfg.wheel_radius_m[1])
        ok.append(bool(v < STAND_SPEED_MPS) and math.isfinite(s.steering_rad))
    out = []
    for i in range(1, len(states)):
        dt = int(times[i] - times[i - 1])
        if ok[i] and ok[i - 1] and abs(states[i].steering_rad - states[i - 1].steering_rad) <= STAND_DQ_RAD \
                and 0 < dt <= MAX_GAP_S * 1e9:
            if out and out[-1][1] == int(times[i - 1]):
                out[-1] = (out[-1][0], int(times[i]))
            else:
                out.append((int(times[i - 1]), int(times[i])))
    return out


class _Imu:
    """The IMU file put on the host clock once; ``why`` lists the file-level reasons no trial gets an IMU value."""

    def __init__(self, rows, up, standing) -> None:
        self.up, self.standing, self.why, self.t = up, standing, [], None
        try:
            g, a = _imu_series(rows, "g"), _imu_series(rows, "a")
        except ValueError as e:
            raise Refused(f"imu: {e}") from e
        prof = next((r for r in rows if r.get("t") == "profile"), {})
        foot = rows[-1] if rows[-1].get("t") == "footer" else {}
        mc = ((prof.get("options") or {}).get("enable_motion_correction") or {}).get("read_back")
        self.profile_ok = mc == 0 and not prof.get("rate_mismatch", True)
        self.footer_ok = foot.get("stop") in ("seconds", "interrupt") and not foot.get("error") and \
            foot.get("dropped") == 0 and foot.get("write_errors") == 0
        if not (self.profile_ok and self.footer_ok):
            self.why.append("imu_record_unclean")
        if len(g["domains"] | a["domains"]) > 1:
            self.why.append("imu_domain_mixed")
        if g["clock"] != "monotonic" or g["t"] is None or a["t"] is None or len(g["t"]) < 2:
            self.why.append("imu_clock_invalid")
        else:
            self.t, self.g, self.bounds = g["t"], g["xyz"], g["boundaries"]
            self.a = np.stack([np.interp(self.t, a["t"], a["xyz"][:, k]) for k in range(3)], axis=1)
        self.fits = [f.summary() for f in g["fits"]]
        self.dropped = dict(gyro=g["dropped"], accel=a["dropped"])

    def windows(self, t0, t1, drive0, drive1, rec_span=None) -> dict:
        """The bias windows of one trial: ``{"why": [...]}`` when there are none, else the windows (sample index
        ranges), their means ``b0``/``b1``, the up axis ``u``, the yaw-axis change ``db``, the per-window body N and
        the shorter window's length. ``rec_span``: the record's first and last STATE time -- the bias-window search
        runs ``quiet_max_s`` before the first drive and after the last, as far as the record goes, and the IMU must
        cover that whole search (entry design 2; Codex impl review P2), not only the trial window."""
        if self.why or self.t is None:
            return dict(why=list(self.why))
        reach = int(REGISTERED["quiet_max_s"] * 1e9)
        lo, hi = min(t0, drive0 - reach), max(t1, drive1 + reach)
        if rec_span is not None:
            lo, hi = max(lo, rec_span[0]), min(hi, rec_span[1])
        if self.t[0] > lo or self.t[-1] < hi:
            return dict(why=["imu_coverage"])
        if any(t0 < b < t1 for b in self.bounds):
            return dict(why=["imu_clock_break"])
        c = REGISTERED
        kw = dict(max_s=c["quiet_max_s"], min_s=c["quiet_min_s"], standing=self.standing,
                  gyro_tol=c["quiet_gyro_tol_rad_s"], accel_tol=c["quiet_accel_tol_mps2"], gap_max_ns=IMU_GAP_NS)
        w0 = ma.quiet_window(self.t, self.g, self.a, boundary_ns=drive0, side="before", **kw)
        w1 = ma.quiet_window(self.t, self.g, self.a, boundary_ns=drive1, side="after", **kw)
        if w0 is None or w1 is None or w0[1] > w1[0]:
            return dict(why=["no_bias_window"])
        u = _unit(_vec3(self.up))
        b0, b1 = self.g[w0[0]:w0[1]].mean(axis=0), self.g[w1[0]:w1[1]].mean(axis=0)
        n = {}
        for name, (s, e), b in (("before", w0, b0), ("after", w1, b1)):
            try:
                n[name] = ma.allan_dev_tau(self.t[s:e], (self.g[s:e] - b) @ u, 1.0)
            except ValueError:
                n[name] = None
        win_s = min(float(self.t[w[1] - 1] - self.t[w[0]]) / 1e9 for w in (w0, w1))
        return dict(why=[], w0=w0, w1=w1, b0=b0, b1=b1, u=u, db=float((b1 - b0) @ u), n=n, win_s=win_s)

    def heading_between(self, t_a: int, t_b: int, bias) -> float | None:
        """The bias-free up-axis heading change from host time ``t_a`` to ``t_b`` (interpolated at both ends);
        None when the IMU does not cover the stretch."""
        if self.t is None or self.t[0] > t_a or self.t[-1] < t_b or t_b <= t_a:
            return None
        a = max(int(np.searchsorted(self.t, t_a, side="right")) - 1, 0)
        b = min(int(np.searchsorted(self.t, t_b, side="left")) + 1, len(self.t))
        h = ma.imu_heading(self.t[a:b], self.g[a:b], self.up, bias)
        ends = np.interp([t_a, t_b], self.t[a:b], h)
        return float(ends[1] - ends[0])

    def trial(self, t0, t1, drive0, drive1, rec_span=None) -> dict:
        """The report-only IMU block of one trial: bias windows before and after, N per window, heading between."""
        bw = self.windows(t0, t1, drive0, drive1, rec_span)
        if bw["why"]:
            return dict(report_only=True, why=bw["why"])
        c = REGISTERED
        w0, w1, b0, b1, u, db, n = (bw[k] for k in ("w0", "w1", "b0", "b1", "u", "db", "n"))
        mid = slice(w0[1] - 1, w1[0] + 1)
        heading = ma.imu_heading(self.t[mid], self.g[mid], u, (b0 + b1) / 2)
        known = [v for v in n.values() if v is not None]
        drive_s = float(self.t[mid][-1] - self.t[mid][0]) / 1e9
        win_s = bw["win_s"]
        arws = dict(desk=c["desk_n_rad_sqrt_s"])
        if known:
            arws["body"] = max(known)
        lower = {k: ma.heading_sigma_lower(arw_rad_per_sqrt_s=v, drive_s=drive_s, bias_window_s=win_s,
                                           bias_change_rad_s=db) for k, v in arws.items()}
        return dict(report_only=True, why=["bias_jump"] if abs(db) > c["bias_jump_rad_s"] else [],
                    dyaw_rad=float(heading[-1]), bias_change_yaw_rad_s=db, drive_s=drive_s,
                    windows=dict(before=[int(self.t[w0[0]]), int(self.t[w0[1] - 1])],
                                 after=[int(self.t[w1[0]]), int(self.t[w1[1] - 1])]),
                    n_body={**n, "pooled_info": max(known) if known else None}, sigma_lower=lower)


# --- trials --------------------------------------------------------------------------------------------------------

def _value(v, judgeable=True, why=None, **extra) -> dict:
    out = dict(value=v, judgeable=bool(judgeable), **extra)
    if not judgeable:
        out["why"] = why if isinstance(why, list) else [why]
    return out


def _cov(p, sigma):
    if p.get("cov") is not None:
        return np.asarray(p["cov"], float)
    return np.diag([sigma["pose_xy_m"] ** 2, sigma["pose_xy_m"] ** 2, sigma["pose_yaw_rad"] ** 2])


def _unjudgeable(why) -> dict:
    return dict(d_rad_per_m=_value(None, False, why), end_lateral_m=_value(None, False, why, report_only=True),
                radius_scale=_value(None, False, why, report_only=True))


def _m1(pts, thdr, odo, i0, i1, s_m):
    """Distance, values and truth budget of M1 (entry 3.5) between odometry samples ``i0`` and ``i1``."""
    pe = np.stack([odo["x"], odo["y"], odo["yaw"]], axis=1)
    rel_o = ma.relative_pose(pe[i0], pe[i1])
    path = float(np.hypot(np.diff(odo["x"][i0:i1 + 1]), np.diff(odo["y"][i0:i1 + 1])).sum())
    dist = dict(source="encoder" if pts is None else "truth", s_enc_path_m=path, disp_enc_m=rel_o["dist_m"],
                s_m_profile=s_m)
    if pts is None:
        return dist, _unjudgeable("no_truth"), None
    sg = thdr["sigma"]
    p0, p1 = pts["start"], pts["end"]
    rel_t = ma.relative_pose((p0["x_m"], p0["y_m"], p0["yaw_rad"]), (p1["x_m"], p1["y_m"], p1["yaw_rad"]),
                             _cov(p0, sg), _cov(p1, sg))
    c = REGISTERED
    budget = dict(sigma_dyaw=rel_t["sigma_dyaw"], sigma_lat=rel_t["sigma_lat"],
                  ok=rel_t["sigma_dyaw"] <= c["budget_dyaw_rad"] and rel_t["sigma_lat"] <= c["budget_lat_m"])
    s_true = rel_t["dist_m"]
    dist.update(s_true_m=s_true, dyaw_true_rad=rel_t["dyaw"], dyaw_odo_rad=rel_o["dyaw"],
                radius_scale=s_true / path if path > 0 else None)
    if s_true <= 0 or path <= 0:
        return dist, _unjudgeable("zero_distance"), budget
    if not budget["ok"]:
        return dist, _unjudgeable("truth_over_budget"), budget
    d = ma.m1_drift(odo_dyaw=rel_o["dyaw"], heading_ref_dyaw=rel_t["dyaw"], ref_source="truth", distance_m=s_true,
                    distance_source="truth")
    return dist, dict(d_rad_per_m=_value(d["rad_per_m"], odo_minus_ref_rad=d["odo_minus_ref_rad"]),
                      end_lateral_m=_value(rel_o["dy_lat"] - rel_t["dy_lat"], report_only=True),
                      radius_scale=_value(dist["radius_scale"], report_only=True)), budget


THETA_MIN_RAD = 0.01  # M2: a smaller true rotation gives no gain (proposed, entry 3.6; to be confirmed)
SINC_SERIES_BELOW = 1e-4


def arc_length(chord_m: float, dyaw_rad: float) -> float:
    """Arc length of the circular arc with chord ``chord_m`` and heading change ``dyaw_rad``: c / sinc(dyaw / 2)."""
    x = abs(dyaw_rad) / 2.0
    sinc = 1.0 - x * x / 6.0 + x ** 4 / 120.0 if x < SINC_SERIES_BELOW else math.sin(x) / x
    return chord_m / sinc


def under_reached(rows, cfg, move_start_ns: int) -> dict:
    """``under_reached`` of an ``at_max`` arc, recomputed from the record (entry 3.6): over the arc's ``running`` rows
    after the movement started only, never met |meas_q - q(req_kappa)| <= UNDER_REACH_TOL_RAD. ``none_fraction``:
    share of those rows whose meas_q is None; all None (or no rows) -> not judgeable ``meas_q_missing``."""
    sel = [r for r in rows if r["host_ns"] >= move_start_ns and r.get("req_kappa")]
    n_none = sum(r.get("meas_q") is None for r in sel)
    frac = n_none / len(sel) if sel else None
    seen = [r for r in sel if r.get("meas_q") is not None]
    if not seen:
        return dict(value=None, judgeable=False, why=["meas_q_missing"], report_only=True, none_fraction=frac,
                    n_rows=len(sel))
    reached = any(abs(r["meas_q"] - cfg.steering.angle(r["req_kappa"])) <= dprof.UNDER_REACH_TOL_RAD for r in seen
                  if cfg.steering.angle(r["req_kappa"]) is not None)
    return dict(value=not reached, judgeable=True, report_only=True, none_fraction=frac, n_rows=len(sel))


def _m2(tr, pts, rec_rows, ctx, i0, i1, move_ns):
    """Distance, values and truth budget of M2 (entry 3.6) between odometry samples ``i0`` and ``i1``."""
    cfg, profile = ctx["cfg"], ctx["profile"]
    arc = ctx["drive_labels"][0]
    seg = next(s for s in profile["segments"] if s.get("label") == arc)
    direction = 1 if float(seg["speed_mps"]) > 0 else -1
    odo = ctx["odo"]
    pe = np.stack([odo["x"], odo["y"], odo["yaw"]], axis=1)
    rel_o = ma.relative_pose(pe[i0], pe[i1])
    path = float(np.hypot(np.diff(odo["x"][i0:i1 + 1]), np.diff(odo["y"][i0:i1 + 1])).sum())
    dist = dict(source="encoder" if pts is None else "truth", s_enc_path_m=path, disp_enc_m=rel_o["dist_m"],
                dyaw_odo_rad=rel_o["dyaw"])
    rows = [r for r in rec_rows if r.get("t") == "profile" and r.get("trial") == tr.trial and r.get("state") == "running"]
    arc_rows = [r for r in rows if r.get("label") == arc and r["host_ns"] >= move_ns]
    after = [r for r in arc_rows if r["host_ns"] >= move_ns + int(REGISTERED["accel_ramp_s"] * 1e9)]
    mq = [float(r["meas_q"]) for r in after if r.get("meas_q") is not None]
    pairs = [float(r["drive_q"]) - float(r["meas_q"]) for r in after
             if r.get("meas_q") is not None and r.get("drive_q") is not None]
    pre = [float(r["meas_q"]) for r in rows if r.get("label") == "presteer" and r.get("meas_q") is not None]
    report = dict(
        meas_q_spread_rad=_value(float(max(mq) - min(mq)) if mq else None, bool(mq), "meas_q_missing",
                                 report_only=True, n=len(mq)),
        drive_minus_meas_q_median_rad=_value(float(np.median(pairs)) if pairs else None, bool(pairs),
                                             "meas_q_missing", report_only=True),
        presteer_approach=_value(None if len(pre) < 2 else dict(sign=int(np.sign(pre[-1] - pre[0])),
                                                                from_zero=abs(pre[0]) < abs(pre[-1])),
                                 len(pre) >= 2, "meas_q_missing", report_only=True))
    if "kappa_inv_m" in seg and seg["kappa_inv_m"] in ("max", "-max"):
        report["under_reached"] = under_reached(arc_rows, cfg, move_ns)
    why: list = []
    kmq = kcq = None
    if mq:
        kmq = cfg.steering.curvature(float(np.mean(mq)))
        dq = [float(r["drive_q"]) for r in after if r.get("meas_q") is not None and r.get("drive_q") is not None]
        kcq = cfg.steering.curvature(float(np.mean(dq))) if dq else None
    if kmq is None or kcq is None:
        why.append("meas_q_missing")
    budget = None
    gains = dict(kappa_true=None, gain_measured_q=None, gain_commanded_q=None)
    if pts is None:
        why.insert(0, "no_truth")
    else:
        sg = ctx["truth_header"]["sigma"]
        p0, p1 = pts["start"], pts["end"]
        rel_t = ma.relative_pose((p0["x_m"], p0["y_m"], p0["yaw_rad"]), (p1["x_m"], p1["y_m"], p1["yaw_rad"]),
                                 _cov(p0, sg), _cov(p1, sg))
        c = REGISTERED
        budget = dict(sigma_dyaw=rel_t["sigma_dyaw"], sigma_lat=rel_t["sigma_lat"],
                      ok=rel_t["sigma_dyaw"] <= c["budget_dyaw_rad"] and rel_t["sigma_lat"] <= c["budget_lat_m"])
        dist.update(dyaw_true_rad=rel_t["dyaw"], chord_true_m=rel_t["dist_m"])
        if not budget["ok"]:
            why.append("truth_over_budget")
        if abs(rel_t["dyaw"]) < THETA_MIN_RAD:
            why.append("rotation_too_small")
        else:
            s_true = arc_length(rel_t["dist_m"], rel_t["dyaw"])
            dist["s_true_m"] = s_true
            if not why:
                try:
                    gains = ma.m2_gains(heading_ref_dyaw=rel_t["dyaw"], ref_source="truth", distance_m=s_true,
                                        direction=direction, kappa_measured_q=kmq, kappa_commanded_q=kcq,
                                        distance_source="truth")
                except ValueError:
                    why.append("kappa_zero")
    ok = not why
    vals = dict(
        gain_measured_q=_value(gains["gain_measured_q"] if ok else None, ok, why),
        gain_commanded_q=_value(gains["gain_commanded_q"] if ok else None, ok, why),
        kappa_true_inv_m=_value(gains["kappa_true"] if ok else None, ok, why, report_only=True),
        kappa_measured_q_inv_m=_value(kmq, kmq is not None, "meas_q_missing", report_only=True),
        kappa_commanded_q_inv_m=_value(kcq, kcq is not None, "meas_q_missing", report_only=True),
        direction=direction, **report)
    return dist, vals, budget


def _imu_kappa(imu: dict, dist: dict, direction: int) -> dict:
    """M2's IMU curvature (registration 4.3): a report value only -- it needs the G3 gyro scale (2.6), so it is never
    judgeable here (no_gyro_scale), with the IMU block's own reasons (entry 3.4; Codex PR 2 review P2). The distance
    is the true arc length when there is one, else the encoder path (labelled)."""
    s, source = (dist.get("s_true_m"), "truth") if dist.get("s_true_m") else (dist.get("s_enc_path_m"), "encoder")
    why = ["no_gyro_scale", *imu.get("why", [])]
    dyaw = imu.get("dyaw_rad")
    if dyaw is None or not s:
        return _value(None, False, why if dyaw is not None else [*why, "no_imu_heading"], report_only=True)
    return _value(float(dyaw) / (float(s) * direction), False, why, report_only=True, distance_source=source)


# --- bias slope over the IMU stretch (registration revision 3, 10.1; entry 3.10, PR 3b) ------------------------------

SLOPE_WINDOWS = 6  # the moving fit's last n intervals
SLOPE_MIN = 3  # fewer: no slope, bias_slope_unconfirmed
RECHECK_PIECE_S = 60.0
INTERVAL_KINDS = ("trial", "standing_record", "recheck_piece")  # kept in this order where windows overlap


def _drive_bounds(rows, trial):
    """The first and last host time a trial's profile asked for motion (a block record's profile file is not at
    hand: its rows say where it drove)."""
    drive = [r["host_ns"] for r in rows if r.get("t") == "profile" and r.get("trial") == trial
             and abs(float(r.get("req_mps") or 0.0)) > 0.0]
    return (min(drive), max(drive)) if drive else None


def _bias_intervals(imu, recs, field_rows) -> list:
    """Every bias interval of the IMU stretch (10.1): trial before/after windows of every record, one window per
    standstill of a record without trials (a G3 stop), and the 60 s pieces of the field log's rechecks; windows that
    share samples count once (trial windows over standstill windows over recheck pieces, the earlier first)."""
    if imu.why or imu.t is None:
        return []
    c = REGISTERED
    kw = dict(max_s=c["quiet_max_s"], min_s=c["quiet_min_s"], gyro_tol=c["quiet_gyro_tol_rad_s"],
              accel_tol=c["quiet_accel_tol_mps2"], gap_max_ns=IMU_GAP_NS)
    found = []
    for rec in recs:
        trs = [t for t in ma.trials(rec["rows"], valid_only=False) if t.spans]
        span = (int(rec["times"][0]), int(rec["times"][-1]))
        if trs:
            for tr in trs:
                d = _drive_bounds(rec["rows"], tr.trial)
                if d is None:
                    continue
                t0, t1 = min(s[0] for s in tr.spans.values()), max(s[1] for s in tr.spans.values())
                bw = imu.windows(t0, t1, d[0], d[1], rec_span=span)
                if not bw["why"]:
                    found += [("trial", bw["w0"]), ("trial", bw["w1"])]
        else:
            for lo, hi in rec["standing"]:
                if hi - lo >= c["quiet_min_s"] * 1e9:
                    w = ma.quiet_window(imu.t, imu.g, imu.a, boundary_ns=hi, side="before", standing=[(lo, hi)], **kw)
                    if w is not None:
                        found.append(("standing_record", w))
    covered = [(int(r["times"][0]), int(r["times"][-1])) for r in recs]
    standing = [iv for r in recs for iv in r["standing"]]
    piece = int(RECHECK_PIECE_S * 1e9)
    for r in field_rows or ():
        if r.get("t") != "bias_recheck":
            continue
        k = int(r["recheck_start_ns"])
        while k + piece <= int(r["recheck_end_ns"]):
            # a piece a record covers must also stand by its STATEs; elsewhere the IMU tolerances alone (10.1)
            # pieces are half-open [k, k + 60 s): adjacent pieces never share their boundary sample
            end = k + piece - 1
            on_rec = any(a <= k and end <= b for a, b in covered)
            within = [(max(lo, k), min(hi, end)) for lo, hi in standing if lo <= end and hi >= k] if on_rec \
                else [(k, end)]
            w = ma.quiet_window(imu.t, imu.g, imu.a, boundary_ns=end, side="before", standing=within, **kw) \
                if within else None
            if w is not None and imu.t[w[0]] >= k:
                found.append(("recheck_piece", w))
            k += piece
    found.sort(key=lambda x: (INTERVAL_KINDS.index(x[0]), x[1][0]))
    kept: list = []
    for kind, (s_, e_) in found:
        if all(e_ <= a or s_ >= b for _k, (a, b) in kept):
            kept.append((kind, (s_, e_)))
    u = _unit(_vec3(imu.up))
    out = [dict(kind=kind, start_ns=int(imu.t[s_]), end_ns=int(imu.t[e_ - 1]),
                center_ns=(int(imu.t[s_]) + int(imu.t[e_ - 1])) // 2, value=float(imu.g[s_:e_].mean(axis=0) @ u))
           for kind, (s_, e_) in kept]
    return sorted(out, key=lambda x: x["center_ns"])


def _segment(imu, t_ns: int) -> int:
    """Which continuous IMU clock segment a host time falls in (a device restart or a clock gap starts a new one)."""
    return int(np.searchsorted(np.asarray(imu.bounds, dtype=np.int64), t_ns, side="right"))


def _bias_slope(intervals, imu, bw) -> dict:
    """The trial's moving fit (10.1): intervals of the trial's own continuous IMU stretch (a clock break ends the
    stretch: earlier windows, block records or rechecks before it never enter; Codex PR 3b review P2) centred no later
    than its after window's end, the last n = min(6, available); n >= 3 gives the least-squares slope of the yaw-axis
    bias against time."""
    thr = REGISTERED["bias_slope_rad_s2"]
    if bw["why"]:
        return dict(value=None, judgeable=False, why=list(bw["why"]), n_windows=0, threshold=thr)
    end = int(imu.t[bw["w1"][1] - 1])
    seg = _segment(imu, end)
    use = [iv for iv in intervals if iv["center_ns"] <= end and _segment(imu, iv["start_ns"]) == seg
           and _segment(imu, iv["end_ns"]) == seg][-SLOPE_WINDOWS:]
    base = dict(n_windows=len(use), centers_ns=[iv["center_ns"] for iv in use], kinds=[iv["kind"] for iv in use],
                threshold=thr)
    if len(use) < SLOPE_MIN:
        return dict(value=None, judgeable=False, why=["bias_slope_unconfirmed"], **base)
    x = np.array([(iv["center_ns"] - use[0]["center_ns"]) / 1e9 for iv in use])
    y = np.array([iv["value"] for iv in use])
    r = float(np.polyfit(x, y, 1)[0])
    ok = abs(r) <= thr
    return dict(value=r, judgeable=ok, **({} if ok else {"why": ["bias_slope"]}), **base)


# --- M3 ------------------------------------------------------------------------------------------------------------

M3_RULES = dict(A_floor_marks="A_substitute_cont_end_stop", synthetic="A_substitute_cont_end_stop",
                B_camera="B_run_042_moving_point")


def k_limit(drive_s: float) -> float:
    """The registration 6.3 limit on body N over desk N for a moving-point gyro heading of duration ``drive_s``: the
    factor k on the two N-proportional terms (white, bias-mean over two W windows) at which the root sum with the
    bias-change (warm drift x (W + T) x T) and bias-instability (x T) terms reaches the 1 mrad budget. Gives the
    registered 1.48 (0.9 m merge end, T 9.125 s) and 1.14 (0.42 m point, 13.325 s); under 1 for the 2.1 m merge end
    -- that point's being unjudgeable comes out of this, it is not written in (entry 3.7)."""
    c = REGISTERED
    n, w, t = c["desk_n_rad_sqrt_s"], c["bias_window_s"], float(drive_s)
    prop = math.hypot(n * math.sqrt(t), n / math.sqrt(2 * w) * t)
    rest = c["budget_dyaw_rad"] ** 2 - (c["warm_drift_rad_s2"] * (w + t) * t) ** 2 - \
        (c["bias_instability_rad_s"] * t) ** 2
    return math.sqrt(rest) / prop if rest > 0 else 0.0


def _odo_at(ctx, host_ns: int, i0: int, i1: int):
    """The odometry pose interpolated at ``host_ns`` between the two STATEs around it inside [i0, i1]: ``(pose,
    bracket_ns)`` or ``(None, why)``. A bracket wider than MAX_GAP_S, or a time outside the trial's STATEs, is
    ``odo_interp_gap`` (entry 3.3 10)."""
    times = ctx["times"]
    i = int(np.searchsorted(times, host_ns, side="left"))
    if i <= i0 or i > i1:
        if i <= i1 and i == i0 and int(times[i0]) == host_ns:
            i = i0 + 1
        else:
            return None, "odo_interp_gap"
    a, b = int(times[i - 1]), int(times[i])
    if b - a > MAX_GAP_S * 1e9 or b <= a:
        return None, "odo_interp_gap"
    f = (host_ns - a) / (b - a)
    odo = ctx["odo"]
    pose = tuple(float(odo[k][i - 1] + f * (odo[k][i] - odo[k][i - 1])) for k in ("x", "y", "yaw"))
    return pose, b - a


def _errors(rel_o, rel_t, ok, why, report) -> dict:
    """e_theta, e_y (odometry minus truth in the start frame) and e_x (always report-only)."""
    if rel_t is None or not ok:
        none = dict(value=None, judgeable=False, why=why if isinstance(why, list) else [why])
        r = dict(report_only=True) if report else {}
        return dict(e_theta_rad={**none, **r}, e_y_m={**none, **r}, e_x_m={**none, "report_only": True})
    extra = dict(report_only=True) if report else {}
    return dict(e_theta_rad=_value(rel_o["dyaw"] - rel_t["dyaw"], **extra),
                e_y_m=_value(rel_o["dy_lat"] - rel_t["dy_lat"], **extra),
                e_x_m=_value(rel_o["dx_fwd"] - rel_t["dx_fwd"], report_only=True))


def _m3_merge(tr, pts, ctx, i_start, i1):
    """The merge's points (entry 3.7): A (and synthetic) judge the ``cont`` end stop pose under the substitute rule
    awaiting the lead; B judges ``run_042``, report-only until its 0.42 m is defined. Every other point is a report."""
    odo = ctx["odo"]
    pe = np.stack([odo["x"], odo["y"], odo["yaw"]], axis=1)
    rel_end = ma.relative_pose(pe[i_start], pe[i1])
    path = float(np.hypot(np.diff(odo["x"][i_start:i1 + 1]), np.diff(odo["y"][i_start:i1 + 1])).sum())
    dist = dict(source="encoder" if pts is None else "truth", s_enc_path_m=path, disp_enc_m=rel_end["dist_m"],
                dyaw_odo_rad=rel_end["dyaw"])
    c = REGISTERED
    if pts is None:
        return dist, dict(end=dict(role="judging_point", **_errors(rel_end, None, False, "no_truth", False))), \
            None, None
    method = ctx["truth_header"]["method"]
    rule = dict(method=method, rule=M3_RULES[method], substitute_pending_lead=method != "B_camera")
    judging = "end" if method != "B_camera" else "run_042"
    sg = ctx["truth_header"]["sigma"]
    p0 = pts["start"]
    t0 = (p0["x_m"], p0["y_m"], p0["yaw_rad"])
    vals, budget = {}, None
    for pid in ("end", "merge_end", "run_042"):
        if pid not in pts:
            if pid == judging:
                vals[pid] = dict(role="judging_point", **_errors(None, None, False, "truth_missing_point", False))
            continue
        p = pts[pid]
        role = "judging_point" if pid == judging else "report"
        entry: dict = dict(role=role)
        if p["kind"] == "stop_pose":
            o_pose, why = tuple(pe[i1]), None
        else:
            o_pose, why = _odo_at(ctx, int(p["host_ns"]), i_start, i1)
            entry["host_ns"] = int(p["host_ns"])
            if o_pose is not None:
                entry["odo_dt_ns"], why = int(why), None
        rel_t = ma.relative_pose(t0, (p["x_m"], p["y_m"], p["yaw_rad"]), _cov(p0, sg), _cov(p, sg))
        b = dict(sigma_dyaw=rel_t["sigma_dyaw"], sigma_lat=rel_t["sigma_lat"],
                 ok=rel_t["sigma_dyaw"] <= c["budget_dyaw_rad"] and rel_t["sigma_lat"] <= c["budget_lat_m"])
        entry["truth_budget"] = b
        reasons = [] if why is None else [why]
        if not b["ok"]:
            reasons.append("truth_over_budget")
        if pid == "run_042":
            reasons.append("run_042_definition_pending")  # until G9 and the lead say what the 0.42 m is (entry 9)
        if o_pose is None:
            entry.update(_errors(None, None, False, reasons, role == "report"))
        else:
            rel_o = ma.relative_pose(pe[i_start], o_pose)
            entry.update(_errors(rel_o, rel_t, True, None, role == "report"))
            if reasons:
                for k in ("e_theta_rad", "e_y_m"):
                    entry[k] = _value(entry[k]["value"], False, reasons, **({"report_only": True}
                                                                              if role == "report" else {}))
        if pid == "end":
            dist.update(s_true_m=rel_t["dist_m"], dyaw_true_rad=rel_t["dyaw"])
        if pid == judging:
            budget = b
        vals[pid] = entry
    return dist, vals, budget, rule


def _m3_imu_heading(imu, bw, ctx, i_start, move_ns, points, slope=None) -> dict:
    """The gyro heading at the moving points (A, registration 5.4, 6.3): odometry minus IMU heading change from the
    standstill before the movement to the point, bias = the two windows' mean. A value only when the body N of each
    window is within k(T) x desk N, its lower bound within 1 mrad, the net rotation within the free rotation and
    the bias slope within its limit (``slope``: the trial's moving fit, 10.1; none given: unconfirmed)."""
    c = REGISTERED
    out = {}
    odo = ctx["odo"]
    t_stand = int(ctx["times"][i_start])
    for pid, t_pt in points.items():
        drive_s = (t_pt - move_ns) / 1e9
        k = k_limit(drive_s)
        base = dict(report_only=True, point_ns=int(t_pt), drive_s=drive_s, k_limit=k)
        if bw["why"]:
            out[pid] = dict(value=None, judgeable=False, why=list(bw["why"]), **base)
            continue
        bias = (bw["b0"] + bw["b1"]) / 2
        h = imu.heading_between(t_stand, t_pt, bias)
        o_pose, why = _odo_at(ctx, t_pt, i_start, len(ctx["times"]) - 1)
        if h is None or o_pose is None:
            out[pid] = dict(value=None, judgeable=False, why=["imu_coverage" if h is None else why], **base)
            continue
        odo_dyaw = o_pose[2] - float(odo["yaw"][i_start])
        n = bw["n"]
        ratio = {kk: (None if v is None else v / c["desk_n_rad_sqrt_s"]) for kk, v in n.items()}
        known = [v for v in n.values() if v is not None]
        arws = dict(desk=c["desk_n_rad_sqrt_s"], **({"body": max(known)} if known else {}))
        lower = {kk: ma.heading_sigma_lower(arw_rad_per_sqrt_s=v, drive_s=drive_s, bias_window_s=bw["win_s"],
                                            bias_change_rad_s=bw["db"]) for kk, v in arws.items()}
        why_l = []
        if abs(bw["db"]) > c["bias_jump_rad_s"]:
            why_l.append("bias_jump")
        if len(known) < 2:
            why_l.append("no_n_body")
        elif any(r > k for r in ratio.values()):
            why_l.append("imu_noise_over_limit")
        if "body" in lower and lower["body"]["lower_bound"] > c["budget_dyaw_rad"]:
            why_l.append("lower_bound_over_budget")
        if abs(h) > c["free_rotation_rad"]:
            why_l.append("no_gyro_scale")
        if slope is None or slope["value"] is None:
            why_l.append("bias_slope_unconfirmed")
        elif not slope["judgeable"]:
            why_l.append("bias_slope")
        out[pid] = dict(value=odo_dyaw - h, judgeable=False, why=why_l, imu_dyaw_rad=h, odo_dyaw_rad=odo_dyaw,
                        n_body_over_desk=ratio, sigma_lower=lower, bias_change_yaw_rad_s=bw["db"], **base)
    return out


def smooth_centered(t_ns, y, window_s: float) -> tuple[np.ndarray, np.ndarray]:
    """Centred moving average over +-window/2 of real time (zero phase; registration 5.2, entry 3.8): ``(times,
    values)`` of the samples whose whole window lies inside the series -- the edge samples, averaged over one side
    only, are dropped (Codex PR 3a review P3)."""
    t = np.asarray(t_ns, dtype=np.int64)
    v = np.asarray(y, float)
    half = int(round(window_s * 1e9 / 2))
    lo = np.searchsorted(t, t - half, side="left")
    hi = np.searchsorted(t, t + half, side="right")
    cs = np.concatenate([[0.0], np.cumsum(v)])
    full = (t - half >= t[0]) & (t + half <= t[-1])
    return t[full], ((cs[hi] - cs[lo]) / (hi - lo))[full]


def _delay(q_t, q, y_t, y, post) -> dict:
    """t63(y) - t63(q) from the measured q's onset (entry 3.8): the two calls share the onset, so it cancels."""
    c = REGISTERED
    kw = dict(q_threshold=c["q_threshold_rad"], post_window_ns=post, frac=c["frac"], pre_s=c["pre_s"])
    dq = ma.step_response_delay(q_t, q, q_t, q, **kw)
    dy = ma.step_response_delay(q_t, q, y_t, y, **kw)
    if dq["delay_s"] is None:
        return dict(value=None, judgeable=False, why=["q_" + dq["reason"]], y=dy, q=dq)
    if dy["delay_s"] is None:
        return dict(value=None, judgeable=False, why=[dy["reason"]], y=dy, q=dq)
    return dict(value=dy["delay_s"] - dq["delay_s"], judgeable=True, y=dy, q=dq)


def _m3_qstep(tr, rec_rows, ctx, i0, i1) -> dict:
    """The q-step delay (entry 3.8, registration 5.2): encoder-differential yaw rate (25 ms centred) against the
    measured q, from the q onset; the post window from the record's own rows -- the first ``step`` row at the final
    curvature (the change row + the ramp) + ``Q_STEP_SETTLE_S`` to the segment's last running row. The IMU delay is
    reported with the unknown MCU-IMU clock offset in it."""
    cfg, c = ctx["cfg"], REGISTERED
    seg = next(s for s in ctx["profile"]["segments"] if s.get("label") == "step")
    rows = [r for r in rec_rows if r.get("t") == "profile" and r.get("trial") == tr.trial
            and r.get("state") == "running" and r.get("label") == "step"]
    direction = 1 if float(seg["q_step"]["to_rad"]) > 0 else -1
    k_first = float(rows[0].get("req_kappa") or 0.0) if rows else 0.0
    k_last = float(rows[-1].get("req_kappa") or 0.0) if rows else 0.0
    change = next((r for r in rows if abs(float(r.get("req_kappa") or 0.0) - k_first) > 1e-12), None)
    reached = next((r for r in rows if change is not None and r["host_ns"] >= change["host_ns"]
                    and abs(float(r.get("req_kappa") or 0.0) - k_last) <= 1e-12), None)
    vals: dict = dict(direction=direction)
    if change is None or reached is None:
        vals["encoder_delay_s"] = _value(None, False, "no_step")
        vals["imu_delay_s"] = _value(None, False, "no_step", report_only=True, unknown_clock_offset=True)
        return vals
    post = (int(reached["host_ns"] + dprof.Q_STEP_SETTLE_S * 1e9), int(rows[-1]["host_ns"]))
    t_a = int(tr.spans["step"][0])
    times = ctx["times"]
    sel = [i for i in range(i0, i1 + 1) if t_a <= times[i] <= post[1]]
    st = [ctx["states"][i] for i in sel]
    qt = times[sel]
    q = np.array([s.steering_rad for s in st])
    y = ma.encoder_yaw_rate([s.wheel_rate_left_rad_s for s in st], [s.wheel_rate_right_rad_s for s in st],
                            radius_m=cfg.wheel_radius_m, track_m=cfg.track_m)
    w = c["smooth_window_s"]
    common = dict(post_window_ns=list(post), change_ns=int(change["host_ns"]), reached_ns=int(reached["host_ns"]),
                  smoothing_s=w)
    vals["encoder_delay_s"] = {**_delay(qt, q, *smooth_centered(qt, y, w), post), **common}
    imu = ctx["imu"]
    if imu.why or imu.t is None:
        vals["imu_delay_s"] = _value(None, False, list(imu.why) or ["imu_clock_invalid"], report_only=True,
                                     unknown_clock_offset=True)
    elif imu.t[0] > t_a - w * 1e9 or imu.t[-1] < post[1] + w * 1e9:
        vals["imu_delay_s"] = _value(None, False, "imu_coverage", report_only=True, unknown_clock_offset=True)
    else:
        m = (imu.t >= t_a - w * 1e9) & (imu.t <= post[1] + w * 1e9)  # padded: the window is whole at both ends
        yi = imu.g[m] @ _unit(_vec3(imu.up))
        d = _delay(qt, q, *smooth_centered(imu.t[m], yi, w), post)
        vals["imu_delay_s"] = {**d, "report_only": True, "unknown_clock_offset": True, **common}
    return vals


def _trial(tr, kind, rec_rows, ctx) -> dict:
    out: dict = dict(trial=tr.trial, result=tr.result)

    def void(why):
        return {**out, "status": "void", "why": why}

    fv = ctx["field_void"].get(tr.trial)
    if fv is not None:
        return void(f"field:{fv}")
    if tr.result != "done":
        return void(tr.result or "not_finished")
    t0, t1 = min(s[0] for s in tr.spans.values()), max(s[1] for s in tr.spans.values())
    times = ctx["times"]
    idx = np.flatnonzero((times >= t0) & (times <= t1))
    if len(idx) < 2:
        return void("no_states_in_window")
    i0, i1 = int(idx[0]), int(idx[-1])
    sts = ctx["states"][i0:i1 + 1]
    for f in FAULT_VOIDS:
        if any(int(s.fault_flags) & int(f) for s in sts):
            return void(f"fault:{f.name}")
    if len({s.boot_id for s in sts}) > 1:
        return void("mcu_boot_changed")
    for i, _t, reason in ctx["odo"]["reasons"]:
        if i0 <= i <= i1 and reason in ODO_VOIDS:
            return void(f"odometry:{reason}")
    pts = None
    if ctx["truth"] is not None:
        pts = ctx["truth"].get(tr.trial)
        if pts is None or "start" not in pts or "end" not in pts:
            return void("truth_missing")
    for g in {ctx["fi"][i] for i in range(i0, i1 + 1)}:
        _need(ctx["fits"][g].valid, f"trial {tr.trial}: the MCU clock fit over the trial is invalid")
    if any(ctx["extrapolated"][i] for i in range(i0, i1 + 1)):
        return void("mcu_clock_extrapolated")  # the fit must cover the trial (entry design 2; Codex impl review P2)
    drives = [lb for lb in ctx["drive_labels"] if lb in tr.spans]
    d0, d1 = tr.spans[drives[0]][0], tr.spans[drives[-1]][1]
    counts = [(s.wheel_count_left, s.wheel_count_right) for s in ctx["states"]]
    moved = [i for i in range(max(i0, 1), i1 + 1) if times[i] >= d0 and counts[i] != counts[i - 1]]
    if not moved:
        return void("no_motion")
    out.update(status="valid", window_ns=[int(t0), int(t1)], move_start_ns=int(times[moved[0]]),
               start_index=(moved[0] - 1, i1))
    rec_span = (int(times[0]), int(times[-1]))
    slope = _bias_slope(ctx["intervals"], ctx["imu"], ctx["imu"].windows(t0, t1, d0, d1, rec_span=rec_span))
    if kind == "m2":
        out["distance"], out["values"], budget = _m2(tr, pts, rec_rows, ctx, moved[0] - 1, i1, int(times[moved[0]]))
        if budget is not None:
            out["truth_budget"] = budget
        out["imu"] = {**ctx["imu"].trial(t0, t1, d0, d1, rec_span=rec_span), "bias_slope": slope}
        out["values"]["imu_kappa_inv_m"] = _imu_kappa(out["imu"], out["distance"], out["values"]["direction"])
        return out
    if kind == "m3_merge":
        i_start = moved[0] - 1
        out["distance"], out["values"], budget, out["truth_rule"] = _m3_merge(tr, pts, ctx, i_start, i1)
        if budget is not None:
            out["truth_budget"] = budget
        imu = ctx["imu"]
        bw = imu.windows(t0, t1, d0, d1, rec_span=rec_span)
        points = {"merge_end": int(tr.spans["cont"][0]), "run_042": int(tr.spans["still_post"][0])}
        out["values"]["imu_heading"] = _m3_imu_heading(imu, bw, ctx, i_start, int(times[moved[0]]), points, slope)
        out["imu"] = {**imu.trial(t0, t1, d0, d1, rec_span=rec_span), "bias_slope": slope}
        return out
    if kind == "m3_qstep":
        out["values"] = _m3_qstep(tr, rec_rows, ctx, i0, i1)
        out["imu"] = {**ctx["imu"].trial(t0, t1, d0, d1, rec_span=rec_span), "bias_slope": slope}
        return out
    s_m = [float(r["s_m"]) for r in rec_rows if r.get("t") == "profile" and r.get("trial") == tr.trial
           and r.get("s_m") is not None]
    out["distance"], out["values"], budget = _m1(pts, ctx["truth_header"], ctx["odo"], moved[0] - 1, i1,
                                                 max(s_m, default=None))
    heading = (ctx.get("truth_heading") or {}).get(tr.trial)
    if pts is None and heading is not None:
        # heading-only truth (the rehearsal's stand-in): the drift on the encoder path, nothing that needs positions
        odo = ctx["odo"]
        pe = np.stack([odo["x"], odo["y"], odo["yaw"]], axis=1)
        rel_o = ma.relative_pose(pe[moved[0] - 1], pe[i1])
        path = out["distance"]["s_enc_path_m"]
        if path > 0:
            d = ma.m1_drift(odo_dyaw=rel_o["dyaw"], heading_ref_dyaw=heading, ref_source="truth", distance_m=path,
                            distance_source="encoder")
            out["values"] = dict(
                d_rad_per_m=_value(d["rad_per_m"], odo_minus_ref_rad=d["odo_minus_ref_rad"], distance_source="encoder",
                                   truth="heading_only"),
                end_lateral_m=_value(None, False, "heading_only_truth", report_only=True),
                radius_scale=_value(None, False, "heading_only_truth", report_only=True))
    if budget is not None:
        out["truth_budget"] = budget
    out["imu"] = {**ctx["imu"].trial(t0, t1, d0, d1, rec_span=rec_span), "bias_slope": slope}
    return out


def _rev4(out: dict, kind: str, ctx: dict) -> dict:
    """Revision 4 values of one valid trial (11.3) from its start and end stop odometry, gyro and tape row."""
    i_start, i1 = out["start_index"]
    odo = ctx["odo"]
    pe = np.stack([odo["x"], odo["y"], odo["yaw"]], axis=1)
    rel = ma.relative_pose(pe[i_start], pe[i1])
    path = float(np.hypot(np.diff(odo["x"][i_start:i1 + 1]), np.diff(odo["y"][i_start:i1 + 1])).sum())
    tape_all = ctx["tape"]
    row = None if tape_all is None else tape_all.get(out["trial"])
    why = "no_tape" if tape_all is None else "tape_missing"
    kappa = ((out.get("values") or {}).get("kappa_measured_q_inv_m") or {}).get("value")
    imu = out.get("imu") or {}
    gyro, gyro_why = imu.get("dyaw_rad"), "no_gyro"
    times, states = ctx["times"], ctx["states"]
    moving = [int(times[i]) for i in range(i_start + 1, i1 + 1)
              if (states[i].wheel_count_left, states[i].wheel_count_right) !=
              (states[i - 1].wheel_count_left, states[i - 1].wheel_count_right)]
    if gyro is not None and not (moving and r4.gyro_covers(imu.get("windows"), moving[0], moving[-1])):
        gyro, gyro_why = None, "gyro_window_not_covering_motion"
    return r4.trial_values(kind, rel_odo=rel, s_enc_m=path, gyro_dyaw_rad=gyro, gyro_why=gyro_why,
                           tape=row, tape_why=why, half_track_m=ctx["cfg"].track_m / 2, kappa_q=kappa)


def _check_block(rows, rec, marks, allow_synthetic) -> None:
    """A block record (another teleop record of the same IMU stretch, 10.1): a clean teleop record of the same host
    boot; it adds standing windows only."""
    try:
        hdr = _header(rows, "record")
        marks.update(refuse_or_mark([hdr], allow=allow_synthetic))
    except (Refused, SyntheticRefused) as e:
        raise Refused(f"block record: {getattr(e, 'why', e)}") from e
    _need(hdr.get("host_boot_id") == rec.get("host_boot_id"), "block record: host_boot_id differs from the record")
    foot = rows[-1]
    _need(foot.get("t") == "footer" and foot.get("dropped") == 0 and foot.get("write_errors") == 0,
          "block record: no clean footer as the last row")


def analyze(rec_rows, imu_rows, truth_rows, field_rows, profile, cfg, up, *, allow_synthetic=False, no_truth=False,
            trial=None, block_rows=None) -> dict:
    """The m_analyze/1 result for one bridge record (entry design 5); raises ``Refused`` on any identity failure."""
    name = profile.get("name") if isinstance(profile, dict) else None
    kind = next((k for p, k in MEASUREMENTS if isinstance(name, str) and name.startswith(p)), None)
    _need(kind is not None, f"profile name {name!r} is not an m1_/m2_/m3_merge_/m3_qstep_ schedule")
    _need(truth_rows is None or not no_truth, "give a truth file or no_truth, not both")
    _need(truth_rows is not None or no_truth, "a truth file or no_truth is required")
    marks: dict = {}
    # the rehearsal's truth stand-in (rehearsal_truth/1: per trial the heading change only) is read under
    # --allow-synthetic alone, as heading-only truth (entry design 2; rehearsal 2 R2-2)
    rehearsal_truth = truth_rows is not None and bool(truth_rows) and truth_rows[0].get("t") == "header" \
        and truth_rows[0].get("schema") == REHEARSAL_TRUTH_SCHEMA
    if rehearsal_truth:
        _need(allow_synthetic, "truth: rehearsal_truth/1 is the rehearsal's stand-in: give --allow-synthetic")
    for fname, rows in (("record", rec_rows), ("imu", imu_rows), ("truth", truth_rows), ("field", field_rows)):
        if rows is None:
            continue
        hdr = rows[0] if fname == "truth" and rehearsal_truth else _header(rows, fname)
        try:
            marks.update(refuse_or_mark([hdr], allow=allow_synthetic))
        except SyntheticRefused as e:
            raise Refused(f"{fname}: {e}") from e
    rec = rec_rows[0]
    boot = rec.get("host_boot_id")
    _need(imu_rows[0].get("host_boot_id") == boot, "imu: host_boot_id differs from the record")
    foot = rec_rows[-1]
    _need(foot.get("t") == "footer" and foot.get("dropped") == 0 and foot.get("write_errors") == 0,
          "record: no clean footer as the last row")
    trs = ma.trials(rec_rows, valid_only=False)
    _identity_profile(rec_rows, profile, trs)
    starts = [min(s[0] for s in t.spans.values()) for t in trs if t.spans]
    tape = None
    truth_heading = None
    if rehearsal_truth:
        truth_heading = {}
        for r in truth_rows[1:]:
            if r.get("t") != "truth":
                continue
            n = r.get("trial")
            # the same identity rules as truth/1 (Codex review P3): a row for a trial the record has, once, finite
            _need(n in {t.trial for t in trs}, f"truth: row for trial {n!r}, which the record does not have")
            _need(n not in truth_heading, f"truth: two rows for trial {n}")
            _need(_finite(r.get("dyaw_rad")), f"truth trial {n}: dyaw_rad is not a finite number")
            truth_heading[n] = float(r["dyaw_rad"])
        truth = None
    elif truth_rows is not None and truth_rows[0].get("method") == "tape":
        tape, truth = _check_tape(truth_rows, {t.trial for t in trs}, rec), None
    else:
        truth = _check_truth(truth_rows, {t.trial for t in trs}, rec) if truth_rows is not None else None
    if field_rows is not None:
        _check_field(field_rows, rec, starts)
    srows, times, fi, fits, extrapolated = _state_times(rec_rows)
    states = [sample_from_row(r).state for r in srows]
    odo = ma.odometry_pose(states, cfg, max_gap_s=MAX_GAP_S)
    upv = _unit(_vec3(up))
    recs = [dict(rows=rec_rows, times=times, standing=_standing(times, states, cfg))]
    for rows in block_rows or ():
        _check_block(rows, rec, marks, allow_synthetic)
        _b_rows, b_times, _fi, _fits, _x = _state_times(rows)
        _need(len(b_times) >= 2, "block record: fewer than two STATEs")
        recs.append(dict(rows=rows, times=b_times,
                         standing=_standing(b_times, [sample_from_row(r).state for r in _b_rows], cfg)))
    spans = sorted((int(r["times"][0]), int(r["times"][-1])) for r in recs)
    for (a0, a1), (b0, b1) in zip(spans, spans[1:]):
        _need(b0 > a1, "block record: records overlap in time (one STATE source at a time)")
    imu = _Imu(imu_rows, upv, [iv for r in recs for iv in r["standing"]])
    if imu.t is not None:
        for r in recs[1:]:  # a block record must lie inside the IMU file (Codex PR 3b review P2)
            _need(imu.t[0] <= r["times"][0] and r["times"][-1] <= imu.t[-1], "block record: outside the IMU file")
    ctx = dict(cfg=cfg, profile=profile, truth=truth, tape=tape, truth_heading=truth_heading,
               truth_header=truth_rows[0] if truth_rows is not None and not rehearsal_truth else None, times=times,
               extrapolated=extrapolated,
               states=states, odo=odo, fi=fi, fits=fits,
               field_void=_field_voids((None if rehearsal_truth else truth_rows, field_rows)),
               drive_labels=[s["label"] for s in profile["segments"] if "speed_mps" in s], imu=imu,
               intervals=_bias_intervals(imu, recs, field_rows))
    results = [_trial(t, kind, rec_rows, ctx) for t in trs if trial is None or t.trial == trial]
    for r in results:
        if r["status"] == "valid" and "start_index" in r:
            r["rev4"] = _rev4(r, kind, ctx)
        r.pop("start_index", None)
    voids: dict = {}
    for r in results:
        if r["status"] == "void":
            key = r["why"] if r["why"].startswith("void:") else "void:" + r["why"]
            voids[key] = voids.get(key, 0) + 1
    return {**marks, "schema": "m_analyze/1", "result": "ok", "measurement": kind, "profile_name": name,
            # revision 4 (11.1, 11.4): the cm/degree values under each trial's "rev4" are the verdict; the gates
            # listed here stay in the older values as information only
            "verdict": dict(revision=r4.REVISION, values="trials[].rev4",
                            gates_not_applied=list(r4.GATES_NOT_APPLIED)),
            "registration": dict(doc=REGISTRATION_DOC, revision=REGISTERED["revision"], status="draft",
                                 constants=dict(REGISTERED)),
            "inputs": dict(record=dict(host_boot_id=boot, footer=foot),
                           imu=dict(serial=imu_rows[0].get("serial"), profile_ok=imu.profile_ok,
                                    footer_ok=imu.footer_ok),
                           truth=dict(schema=REHEARSAL_TRUTH_SCHEMA, method="heading_only") if rehearsal_truth
                           else None if truth_rows is None else dict(schema=truth_rows[0]["schema"],
                                                                method=truth_rows[0]["method"]),
                           field_log=None if field_rows is None else dict(rows=len(field_rows) - 1),
                           profile=dict(name=name), up=dict(vector=[float(x) for x in upv])),
            "clock": dict(domain="host_monotonic_ns", mcu_fits=[f.summary() for f in fits], imu_fits=imu.fits,
                          imu_dropped=imu.dropped),
            "counts": dict(trials=len(results), valid=sum(r["status"] == "valid" for r in results), void=voids),
            "trials": results}
