"""G8 entry: bridge record + IMU record + truth -> per-trial M values (docs/design/2026-11-15-m-analyze-entry.md, PR 1).

``analyze`` checks identity (section 2), voids what 3.4 voids, and computes M1 (3.5) with the report-only IMU block;
M2/M3 get their identity and void handling here and their values in later PRs. No threshold of the registered
reading is applied beyond the truth budget: the output (section 5) is per-trial values plus whether each can be judged.
"""

from __future__ import annotations

import math

import numpy as np

from forklift_core.live import drive_profile as dprof
from forklift_core.live.synthetic import SyntheticRefused, refuse_or_mark
from forklift_core.mcu_link.messages import FaultFlag
from forklift_core.plant import m_analysis as ma
from forklift_core.plant.clock_map import clock_fits_from_rows, fit_clock
from forklift_core.plant.record_rows import sample_from_row
from forklift_core.sensors.imu_mount import _unit, _vec3
from forklift_core.sensors.imu_record import continuous_segments, imu_stream

REGISTRATION_DOC = "docs/plans/2026-11-15-m1-m3-registration.md"
REGISTERED = {
    "revision": "초안 개정 2", "quiet_max_s": 60.0, "quiet_min_s": 50.0, "quiet_gyro_tol_rad_s": 0.025,
    "quiet_accel_tol_mps2": 0.15, "bias_jump_rad_s": 1.5e-4, "bias_slope_rad_s2": 3.5e-7,
    "desk_n_rad_sqrt_s": 2.054e-4, "budget_dyaw_rad": 1.0e-3, "budget_lat_m": 1.7e-3,
    "k_limit_merge_end_l09": 1.48, "k_limit_run_042_l09": 1.14, "free_rotation_rad": 0.04,
    "accel_ramp_s": 0.125, "smooth_window_s": 0.025, "pre_s": 0.5, "frac": 0.63,
}
IMU_GAP_NS = 10_000_000  # an IMU quiet run breaks past this (entry 9)
MAX_GAP_S = 0.05  # STATE gap that ends a standing stretch / breaks odometry (entry 9)
STAND_SPEED_MPS = 0.005  # standing thresholds: proposed, to be confirmed (entry 7.3)
STAND_DQ_RAD = 5e-4
MEASUREMENTS = (("m1_", "m1"), ("m2_", "m2"), ("m3_merge_", "m3_merge"), ("m3_qstep_", "m3_qstep"))
SCHEMAS = dict(record="teleop_bridge_record/1", imu="imu_record/1", truth="truth/1", field="field_log/1")
TRUTH_METHODS = ("A_floor_marks", "B_camera", "synthetic")
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
    st = imu_stream(rows, stream=stream)
    keep, dropped, ld, lh = [], 0, None, None
    for i, (d, h) in enumerate(zip(st.dev_us.tolist(), st.host_ns.tolist())):
        if ld is not None and (d <= ld or h <= lh):
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

    def trial(self, t0, t1, drive0, drive1, rec_span=None) -> dict:
        """The report-only IMU block of one trial: bias windows before and after, N per window, heading between.
        ``rec_span``: the record's first and last STATE time -- the bias-window search runs ``quiet_max_s`` before
        the first drive and after the last, as far as the record goes, and the IMU must cover that whole search
        (entry design 2; Codex impl review P2), not only the trial window."""
        if self.why or self.t is None:
            return dict(report_only=True, why=list(self.why))
        reach = int(REGISTERED["quiet_max_s"] * 1e9)
        lo, hi = min(t0, drive0 - reach), max(t1, drive1 + reach)
        if rec_span is not None:
            lo, hi = max(lo, rec_span[0]), min(hi, rec_span[1])
        if self.t[0] > lo or self.t[-1] < hi:
            return dict(report_only=True, why=["imu_coverage"])
        if any(t0 < b < t1 for b in self.bounds):
            return dict(report_only=True, why=["imu_clock_break"])
        c = REGISTERED
        kw = dict(max_s=c["quiet_max_s"], min_s=c["quiet_min_s"], standing=self.standing,
                  gyro_tol=c["quiet_gyro_tol_rad_s"], accel_tol=c["quiet_accel_tol_mps2"], gap_max_ns=IMU_GAP_NS)
        w0 = ma.quiet_window(self.t, self.g, self.a, boundary_ns=drive0, side="before", **kw)
        w1 = ma.quiet_window(self.t, self.g, self.a, boundary_ns=drive1, side="after", **kw)
        if w0 is None or w1 is None or w0[1] > w1[0]:
            return dict(report_only=True, why=["no_bias_window"])
        u = _unit(_vec3(self.up))
        b0, b1 = self.g[w0[0]:w0[1]].mean(axis=0), self.g[w1[0]:w1[1]].mean(axis=0)
        db = float((b1 - b0) @ u)
        n = {}
        for name, (s, e), b in (("before", w0, b0), ("after", w1, b1)):
            try:
                n[name] = ma.allan_dev_tau(self.t[s:e], (self.g[s:e] - b) @ u, 1.0)
            except ValueError:
                n[name] = None
        mid = slice(w0[1] - 1, w1[0] + 1)
        heading = ma.imu_heading(self.t[mid], self.g[mid], u, (b0 + b1) / 2)
        known = [v for v in n.values() if v is not None]
        drive_s = float(self.t[mid][-1] - self.t[mid][0]) / 1e9
        win_s = min(float(self.t[w[1] - 1] - self.t[w[0]]) / 1e9 for w in (w0, w1))
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
    out.update(status="valid", window_ns=[int(t0), int(t1)], move_start_ns=int(times[moved[0]]))
    if kind == "m2":
        out["distance"], out["values"], budget = _m2(tr, pts, rec_rows, ctx, moved[0] - 1, i1, int(times[moved[0]]))
        if budget is not None:
            out["truth_budget"] = budget
        out["imu"] = ctx["imu"].trial(t0, t1, d0, d1, rec_span=(int(times[0]), int(times[-1])))
        return out
    if kind != "m1":
        out["values"] = dict(all=_value(None, False, "not_implemented_in_pr1"))
        return out
    s_m = [float(r["s_m"]) for r in rec_rows if r.get("t") == "profile" and r.get("trial") == tr.trial
           and r.get("s_m") is not None]
    out["distance"], out["values"], budget = _m1(pts, ctx["truth_header"], ctx["odo"], moved[0] - 1, i1,
                                                 max(s_m, default=None))
    if budget is not None:
        out["truth_budget"] = budget
    out["imu"] = ctx["imu"].trial(t0, t1, d0, d1, rec_span=(int(times[0]), int(times[-1])))
    return out


def analyze(rec_rows, imu_rows, truth_rows, field_rows, profile, cfg, up, *, allow_synthetic=False, no_truth=False,
            trial=None) -> dict:
    """The m_analyze/1 result for one bridge record (entry design 5); raises ``Refused`` on any identity failure."""
    name = profile.get("name") if isinstance(profile, dict) else None
    kind = next((k for p, k in MEASUREMENTS if isinstance(name, str) and name.startswith(p)), None)
    _need(kind is not None, f"profile name {name!r} is not an m1_/m2_/m3_merge_/m3_qstep_ schedule")
    _need(truth_rows is None or not no_truth, "give a truth file or no_truth, not both")
    _need(truth_rows is not None or no_truth, "a truth file or no_truth is required")
    marks: dict = {}
    for fname, rows in (("record", rec_rows), ("imu", imu_rows), ("truth", truth_rows), ("field", field_rows)):
        if rows is None:
            continue
        hdr = _header(rows, fname)
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
    truth = _check_truth(truth_rows, {t.trial for t in trs}, rec) if truth_rows is not None else None
    if field_rows is not None:
        _check_field(field_rows, rec, starts)
    srows, times, fi, fits, extrapolated = _state_times(rec_rows)
    states = [sample_from_row(r).state for r in srows]
    odo = ma.odometry_pose(states, cfg, max_gap_s=MAX_GAP_S)
    upv = _unit(_vec3(up))
    imu = _Imu(imu_rows, upv, _standing(times, states, cfg))
    ctx = dict(cfg=cfg, profile=profile, truth=truth, truth_header=truth_rows[0] if truth_rows is not None else None, times=times,
               extrapolated=extrapolated,
               states=states, odo=odo, fi=fi, fits=fits, field_void=_field_voids((truth_rows, field_rows)),
               drive_labels=[s["label"] for s in profile["segments"] if "speed_mps" in s], imu=imu)
    results = [_trial(t, kind, rec_rows, ctx) for t in trs if trial is None or t.trial == trial]
    voids: dict = {}
    for r in results:
        if r["status"] == "void":
            key = r["why"] if r["why"].startswith("void:") else "void:" + r["why"]
            voids[key] = voids.get(key, 0) + 1
    return {**marks, "schema": "m_analyze/1", "result": "ok", "measurement": kind,
            "registration": dict(doc=REGISTRATION_DOC, revision=REGISTERED["revision"], status="draft",
                                 constants=dict(REGISTERED)),
            "inputs": dict(record=dict(host_boot_id=boot, footer=foot),
                           imu=dict(serial=imu_rows[0].get("serial"), profile_ok=imu.profile_ok,
                                    footer_ok=imu.footer_ok),
                           truth=None if truth is None else dict(schema=truth_rows[0]["schema"],
                                                                 method=truth_rows[0]["method"]),
                           field_log=None if field_rows is None else dict(rows=len(field_rows) - 1),
                           profile=dict(name=name), up=dict(vector=[float(x) for x in upv])),
            "clock": dict(domain="host_monotonic_ns", mcu_fits=[f.summary() for f in fits], imu_fits=imu.fits,
                          imu_dropped=imu.dropped),
            "counts": dict(trials=len(results), valid=sum(r["status"] == "valid" for r in results), void=voids),
            "trials": results}
