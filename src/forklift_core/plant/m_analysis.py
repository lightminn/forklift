"""M1-M3 floor-measurement analysis (docs/design/2026-11-14-profile-drive-and-m-analysis.md 3, G8).

Inputs come from the teleop bridge record (STATE rows, profile rows, markers; G1/G5/G6), the IMU record (G2) put on the
host clock (G7) and the mount axis (G3). Position and true distance come from G9, whose method is not chosen yet, so
they are arguments. These functions give values and their limits; the registered reading (C3 6.5: thresholds,
medians, 95 % intervals) is applied elsewhere and never baked in here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from forklift_core.mcu_link.messages import FaultFlag
from forklift_core.plant.real_vehicle import RealOdometry
from forklift_core.sensors.imu_mount import _unit, _vec3

DISTANCE_SOURCES = ("truth", "encoder")
REF_SOURCES = ("truth", "imu")  # what the M1/M2 heading change is read from (entry design 4 (가))
QUIET_GAP_MAX_NS = 10_000_000  # neighbouring samples further apart end a quiet run (400 Hz gyro: 4 periods; entry 9)
ALLAN_GRID_PER_TAU = 10  # the Allan deviation's angle grid is tau / 10
QUIET_GYRO_RAD_S = 0.01  # a sample's gyro within this of the window's median ...
QUIET_ACCEL_MPS2 = 0.15  # ... and its acceleration within this: quiet (proposed; registered before measuring)
QUIET_CHUNK_S = 1.0  # the reference is the median of the stillest chunk this long (not of the whole stretch)
_ENCODER_CUT = FaultFlag.ENCODER_GAP | FaultFlag.ENCODER_JUMP_L | FaultFlag.ENCODER_JUMP_R


# --- trials --------------------------------------------------------------------------------------------------------

@dataclass
class Trial:
    trial: int
    result: str | None = None
    spans: dict = field(default_factory=dict)  # segment label -> (first host_ns, last host_ns) of its rows
    _seg_of: dict = field(default_factory=dict, repr=False)

    @property
    def valid(self) -> bool:
        return self.result == "done"


def trials(rows, *, valid_only: bool = False) -> list[Trial]:
    """The trials of a bridge record from its ``profile`` rows: each segment's host-time span and the result
    (``done`` or ``void:<why>``). A void trial is kept but marked; ``valid_only`` drops it."""
    out: dict[int, Trial] = {}
    for r in rows:
        if r.get("t") != "profile":
            continue
        tr = out.setdefault(int(r["trial"]), Trial(int(r["trial"])))
        if r.get("result"):
            tr.result = r["result"]
        label = r.get("label")
        if label is not None and r.get("state") == "running":
            seg = r.get("seg")
            if tr._seg_of.setdefault(label, seg) != seg:
                raise ValueError(f"trial {tr.trial}: label {label!r} names two segments; labels must be unique")
            s = tr.spans.get(label)
            tr.spans[label] = (int(r["host_ns"]), int(r["host_ns"])) if s is None else (s[0], int(r["host_ns"]))
    found = sorted(out.values(), key=lambda t: t.trial)
    return [t for t in found if t.valid] if valid_only else found


# --- IMU -----------------------------------------------------------------------------------------------------------

def quiet_window(t_ns, gyro, accel, *, boundary_ns: int, side: str, max_s: float, min_s: float,
                 standing, gyro_tol: float = QUIET_GYRO_RAD_S, accel_tol: float = QUIET_ACCEL_MPS2,
                 gap_max_ns: int = QUIET_GAP_MAX_NS) -> tuple[int, int] | None:
    """``[start, end)`` indices of the bias window next to a trial: within ``max_s`` before (or after) the boundary,
    the contiguous run of quiet samples (gyro and acceleration near the median of the stillest 1 s chunk) that is
    closest to the boundary and at least ``min_s`` long -- motion at the trial's edge (a creep, the end of a drive)
    is cut off. ``standing``: host-time intervals where the encoders say the truck stands (design 3: encoder speed
    zero and IMU quiet) -- a steady straight drive looks quiet to an IMU, so samples outside them never count. It
    is required; ``None`` (IMU alone) must be passed on purpose and is for diagnostics, never for a measurement.
    A run is contiguous in the record itself: it ends where the next kept sample is not the next sample of the
    array (a sample outside ``standing`` lies between) or comes more than ``gap_max_ns`` later (samples missing),
    so the returned ``[start, end)`` never spans a stretch that was not looked at (entry design 3.3 5, 4 (나), 9).
    Returns None when there is no such run."""
    if isinstance(gap_max_ns, bool) or int(gap_max_ns) != gap_max_ns or gap_max_ns <= 0:
        raise ValueError("gap_max_ns must be a positive whole number of nanoseconds")
    t = np.asarray(t_ns, dtype=np.int64)
    if side == "before":
        sel = np.flatnonzero((t <= boundary_ns) & (t >= boundary_ns - int(max_s * 1e9)))
    elif side == "after":
        sel = np.flatnonzero((t >= boundary_ns) & (t <= boundary_ns + int(max_s * 1e9)))
    else:
        raise ValueError("side is 'before' or 'after'")
    if len(sel) == 0:
        return None
    if standing is not None:
        ok = np.zeros(len(t), dtype=bool)
        for lo, hi in standing:
            ok |= (t >= int(lo)) & (t <= int(hi))
        sel = sel[ok[sel]]
        if len(sel) == 0:
            return None
    g, a = np.asarray(gyro, float)[sel], np.asarray(accel, float)[sel]
    # the reference standstill: the chunk with the smallest spread, so a stretch that mostly moves does not drag
    # the reference toward the motion (Codex G8 review P2)
    ts = t[sel]
    edges = np.arange(ts[0], ts[-1] + 1, int(QUIET_CHUNK_S * 1e9))
    best, best_spread = None, math.inf
    for lo in edges:
        m = (ts >= lo) & (ts < lo + int(QUIET_CHUNK_S * 1e9))
        if m.sum() < 5:
            continue
        # a steady turn has a small spread too: the chunk's mean rotation counts against it (the bias is small)
        spread = float(np.linalg.norm(g[m].std(axis=0)) / gyro_tol + np.linalg.norm(a[m].std(axis=0)) / accel_tol
                       + np.linalg.norm(g[m].mean(axis=0)) / gyro_tol)
        if spread < best_spread:
            best, best_spread = m, spread
    if best is None:
        return None
    g_ref, a_ref = np.median(g[best], axis=0), np.median(a[best], axis=0)
    quiet = (np.linalg.norm(g - g_ref, axis=1) <= gyro_tol) & (np.linalg.norm(a - a_ref, axis=1) <= accel_tol)
    # linked[i]: kept sample i follows kept sample i - 1 directly, in the array and in time
    linked = np.ones(len(sel), dtype=bool)
    linked[1:] = (np.diff(sel) == 1) & (np.diff(ts) <= int(gap_max_ns))
    runs, start = [], None
    for i, ok in enumerate(quiet):
        if start is not None and not (ok and linked[i]):
            runs.append((start, i))
            start = None
        if ok and start is None:
            start = i
    if start is not None:
        runs.append((start, len(quiet)))
    long_enough = [(s, e) for s, e in runs if (t[sel[e - 1]] - t[sel[s]]) / 1e9 >= min_s]
    if not long_enough:
        return None
    s, e = long_enough[-1] if side == "before" else long_enough[0]
    return int(sel[s]), int(sel[e - 1]) + 1


def imu_heading(t_ns, gyro, up, bias) -> np.ndarray:
    """The body heading (rad) from the bias-free yaw rate, trapezoid-integrated, 0 at the first sample."""
    t_ns = np.asarray(t_ns, dtype=np.int64)
    if t_ns.ndim != 1 or len(t_ns) < 2 or np.any(np.diff(t_ns) <= 0):
        raise ValueError("imu_heading needs at least two samples on a strictly increasing clock")
    g = np.asarray(gyro, float)
    if g.shape != (len(t_ns), 3) or not np.isfinite(g).all():
        raise ValueError("gyro must be N x 3 finite samples, one per time")
    t = (t_ns - int(t_ns[0])) / 1e9
    w = (g - _vec3(bias)) @ _unit(_vec3(up))  # yaw_rate's projection, without its minimum sample count
    out = np.zeros(len(t))
    out[1:] = np.cumsum((w[1:] + w[:-1]) / 2 * np.diff(t))
    return out


def heading_sigma_lower(*, arw_rad_per_sqrt_s: float, drive_s: float, bias_window_s: float,
                        bias_change_rad_s: float) -> dict:
    """A lower bound on the 1-sigma heading error over a drive (design 3): white noise over the drive and the bias
    estimate's noise (the mean of two windows of ``bias_window_s``) carried over the drive, root-sum-squared.
    The bias change between the windows is not in the bound: corrected by the windows' mean it adds nothing if the
    bias drifted linearly and up to ``|change| x drive`` if it stepped at one end -- reported as that range
    (Codex G8 review P2). Not included either: gyro scale (measured separately), mount axis error and body
    roll/pitch (diagnostics only). So a bound under 1 mrad does not by itself make a heading judgeable."""
    white = arw_rad_per_sqrt_s * math.sqrt(drive_s)
    bias_mean = arw_rad_per_sqrt_s / math.sqrt(2 * bias_window_s) * drive_s
    return {"white": white, "bias_mean": bias_mean, "lower_bound": math.sqrt(white ** 2 + bias_mean ** 2),
            "bias_change_range": (0.0, abs(bias_change_rad_s) * drive_s), "is_lower_bound": True,
            "not_included": ["bias_change", "scale", "mount_axis", "roll_pitch"]}


def gyro_scale(*, imu_dyaw: float, turns: int, heading_error_rad: float = 0.0) -> dict:
    """The gyro's scale from G3 arcs: the bias-free IMU rotation over ``turns`` full turns that start and end on the
    same floor-marked heading, over 2 pi turns. ``heading_error_rad`` is the floor marks' 1-sigma heading error at
    each end; it sets the scale's uncertainty (sqrt 2 x error over 2 pi turns)."""
    if isinstance(turns, bool) or int(turns) != turns or turns < 1:
        raise ValueError("whole turns only (at least one): the arcs end on the heading they started from")
    turns = int(turns)
    nominal = 2 * math.pi * turns
    return {"scale": abs(imu_dyaw) / nominal, "sigma": math.sqrt(2) * heading_error_rad / nominal, "turns": turns}


def allan_dev_tau(t_ns, rate, tau_s: float) -> float:
    """The overlapping Allan deviation of one rate series (rad/s, e.g. the up-projected, bias-free yaw rate of a
    standing window) at ``tau_s``, on the samples' actual times (entry design 3.3 8, 4 (나)): the rate is
    trapezoid-integrated to an angle on the real timestamps, the angle linearly interpolated onto a uniform grid of
    ``tau / 10`` and the overlapping estimator taken there, sigma^2 = mean((th[k+2m] - 2 th[k+m] + th[k])^2) /
    (2 tau^2) with m = 10. No uniform sample period is assumed (the D435i gyro runs at a measured 400.8 Hz, with
    jitter). For white rate noise of density N (rad/sqrt(s)) it reads N / sqrt(tau). The caller hands over a window
    without gaps (``quiet_window``); a record shorter than 2 tau is refused."""
    t = np.asarray(t_ns, dtype=np.int64)
    w = np.asarray(rate, float)
    if t.ndim != 1 or len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError("allan_dev_tau needs at least two samples on a strictly increasing clock")
    if w.shape != t.shape or not np.isfinite(w).all():
        raise ValueError("rate must be one finite value per time (project a 3-axis gyro onto the axis first)")
    tau = _positive(tau_s, "tau_s")
    ts = (t - int(t[0])) / 1e9
    angle = np.zeros(len(ts))
    angle[1:] = np.cumsum((w[1:] + w[:-1]) / 2 * np.diff(ts))
    m = ALLAN_GRID_PER_TAU
    step = tau / m
    n = int(math.floor(ts[-1] / step + 1e-9)) + 1
    if n < 2 * m + 1:
        raise ValueError(f"the record ({ts[-1]:.3f} s) is shorter than 2 tau ({2 * tau:.3f} s)")
    th = np.interp(np.arange(n) * step, ts, angle)
    d = th[2 * m:] - 2 * th[m:n - m] + th[:n - 2 * m]
    return float(math.sqrt(float(np.mean(d * d)) / (2 * tau * tau)))


# --- odometry ------------------------------------------------------------------------------------------------------

def odometry_pose(states, cfg, *, max_gap_s: float) -> dict:
    """The real truck's odometry re-run over recorded STATEs (each with mcu_time_us, boot_id, wheel counts,
    steering_rad, fault_flags): the rear-axle pose ``x``, ``y``, ``yaw`` after each sample (``RealOdometry``'s frame,
    starting at the origin) and where continuity broke (entry design 3.3 2, 4 (나)). The pose keeps accumulating
    across a break; never compare two samples on either side of one as if continuous.

    ``broken``: ``(mcu_time_us, reason)`` of every sample after the first that broke continuity (as before).
    ``reasons``: ``(index, mcu_time_us, reason)`` of every sample whose step gave a reason, the first sample
    included -- ``time_not_increasing`` (a repeated or late STATE: not integrated, but not a continuity break either,
    ``real_vehicle.py`` 328-329) and ``rereference`` only show here; the entry voids a trial on them (3.4)."""
    odo = RealOdometry(cfg, max_gap_s=max_gap_s)
    t, x, y, yaw, broken, reasons = [], [], [], [], [], []
    for i, s in enumerate(states):
        cut = bool(int(s.fault_flags) & int(_ENCODER_CUT))
        step = odo.update(s.mcu_time_us, s.boot_id, s.wheel_count_left, s.wheel_count_right, s.steering_rad,
                          encoder_cut=cut)
        t.append(int(s.mcu_time_us))
        x.append(step.pose[0])
        y.append(step.pose[1])
        yaw.append(step.pose[2])
        if step.reason is not None:
            reasons.append((i, int(s.mcu_time_us), step.reason))
        if step.continuity_broken and i > 0:
            broken.append((int(s.mcu_time_us), step.reason))
    return {"mcu_time_us": np.asarray(t, dtype=np.int64), "x": np.asarray(x), "y": np.asarray(y),
            "yaw": np.asarray(yaw), "broken": broken, "reasons": reasons}


def odometry_heading(states, cfg, *, max_gap_s: float) -> dict:
    """``odometry_pose``'s yaw alone, with the keys it always had (``mcu_time_us``, ``yaw``, ``broken``)."""
    p = odometry_pose(states, cfg, max_gap_s=max_gap_s)
    return {"mcu_time_us": p["mcu_time_us"], "yaw": p["yaw"], "broken": p["broken"]}


def _pose3(p, what: str) -> np.ndarray:
    v = np.asarray(p, float)
    if v.shape != (3,) or not np.isfinite(v).all():
        raise ValueError(f"{what} must be a finite (x, y, yaw)")
    return v


def _cov3(c, what: str) -> np.ndarray:
    m = np.asarray(c, float)
    if m.shape != (3, 3) or not np.isfinite(m).all():
        raise ValueError(f"{what} must be a finite 3 x 3 covariance of (x, y, yaw)")
    scale = float(np.abs(m).max())
    if not np.allclose(m, m.T, rtol=0.0, atol=1e-12 * scale) or np.any(np.diag(m) < 0):
        raise ValueError(f"{what} must be symmetric with non-negative variances")
    # positive semi-definite: a matrix with non-negative variances can still have a negative eigenvalue, and the
    # propagated variances would then hide it (Codex D4 impl review P2)
    if float(np.linalg.eigvalsh(0.5 * (m + m.T)).min()) < -1e-12 * max(scale, 1e-300):
        raise ValueError(f"{what} must be positive semi-definite")
    return m


def relative_pose(p0, p1, cov0=None, cov1=None) -> dict:
    """The pose ``p1`` seen from ``p0`` (both ``(x, y, yaw)`` in one floor frame): ``dx_fwd``, ``dy_lat`` = R(-yaw0)
    (p1 - p0) (forward along p0's heading, + to its left) and ``dyaw`` = yaw1 - yaw0 wrapped by atan2 to (-pi, pi]
    (entry design 3.3 10). Used alike for the truth and the odometry; ``dist_m`` is |p1 - p0|.

    With both covariances (3 x 3 over x, y, yaw; the two poses independent of each other) it adds ``cov``, the
    3 x 3 covariance of (dx_fwd, dy_lat, dyaw) carried by the Jacobians, J0 C0 J0^T + J1 C1 J1^T, and its
    ``sigma_fwd``, ``sigma_lat``, ``sigma_dyaw``. The registration's sqrt(s_th0^2 + s_th1^2) and
    sqrt(s_y0^2 + s_y1^2 + (|dp| s_th0)^2) (6.1-6.3) are its special case for diagonal covariances and motion along
    p0's heading; correlated errors are carried as they are (entry 9)."""
    a, b = _pose3(p0, "p0"), _pose3(p1, "p1")
    c, s = math.cos(a[2]), math.sin(a[2])
    ex, ey = float(b[0] - a[0]), float(b[1] - a[1])
    dx, dy = c * ex + s * ey, -s * ex + c * ey
    d = float(b[2] - a[2])
    out = {"dx_fwd": dx, "dy_lat": dy, "dyaw": math.atan2(math.sin(d), math.cos(d)), "dist_m": math.hypot(ex, ey)}
    if cov0 is None and cov1 is None:
        return out
    if cov0 is None or cov1 is None:
        raise ValueError("give both poses' covariances or neither")
    c0, c1 = _cov3(cov0, "cov0"), _cov3(cov1, "cov1")
    j0 = np.array([[-c, -s, dy], [s, -c, -dx], [0.0, 0.0, -1.0]])
    j1 = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
    cov = j0 @ c0 @ j0.T + j1 @ c1 @ j1.T
    cov = (cov + cov.T) / 2
    sig = np.sqrt(np.clip(np.diag(cov), 0.0, None))
    out.update(cov=cov, sigma_fwd=float(sig[0]), sigma_lat=float(sig[1]), sigma_dyaw=float(sig[2]))
    return out


def encoder_yaw_rate(rate_left_rad_s, rate_right_rad_s, *, radius_m, track_m: float) -> np.ndarray:
    """Body yaw rate from the rear wheels' speed difference: (v_R - v_L) / track. Assumes no wheel slip and the
    radii given (unverified on the floor) -- reported as the encoder-differential yaw response, not body yaw."""
    vl = np.asarray(rate_left_rad_s, float) * radius_m[0]
    vr = np.asarray(rate_right_rad_s, float) * radius_m[1]
    return (vr - vl) / track_m


# --- M1 / M2 -------------------------------------------------------------------------------------------------------

def _source(distance_source: str) -> str:
    if distance_source not in DISTANCE_SOURCES:
        raise ValueError(f"distance source must be one of {DISTANCE_SOURCES}")
    return distance_source


def _positive(value: float, what: str) -> float:
    v = float(value)
    if not (math.isfinite(v) and v > 0):
        raise ValueError(f"{what} must be positive and finite")
    return v


def _nonzero(value: float, what: str) -> float:
    v = float(value)
    if not math.isfinite(v) or v == 0.0:
        raise ValueError(f"{what} must be finite and nonzero")
    return v


def _heading_ref(heading_ref_dyaw, ref_source, imu_dyaw) -> tuple[float, str, bool]:
    """The heading change M1/M2 are measured against (entry design 4 (가)): ``heading_ref_dyaw`` with its
    ``ref_source`` (truth or imu), or the old ``imu_dyaw`` alone (the alias: source imu, old result keys). Exactly
    one of the two; the alias takes no ``ref_source``. Returns (value, source, alias)."""
    if imu_dyaw is not None:
        if heading_ref_dyaw is not None:
            raise ValueError("give heading_ref_dyaw or the imu_dyaw alias, not both")
        if ref_source is not None:
            raise ValueError("imu_dyaw is the imu alias and takes no ref_source; use heading_ref_dyaw with it")
        return imu_dyaw, "imu", True
    if heading_ref_dyaw is None:
        raise ValueError("a heading reference is needed: heading_ref_dyaw with ref_source, or the imu_dyaw alias")
    if ref_source not in REF_SOURCES:
        raise ValueError(f"ref_source must be one of {REF_SOURCES}")
    return heading_ref_dyaw, ref_source, False


def m1_drift(*, odo_dyaw: float, heading_ref_dyaw: float | None = None, ref_source: str | None = None,
             imu_dyaw: float | None = None, distance_m: float, distance_source: str) -> dict:
    """Odometry heading minus the reference heading per metre (registration 3.3; entry design 3.5, 4 (가)): the
    reference is ``heading_ref_dyaw`` from ``ref_source`` ("truth": the floor truth's stop poses; "imu"), giving
    ``odo_minus_ref_rad`` and ``ref_source``. The old ``imu_dyaw`` keyword stays as an alias: source imu and the old
    key ``odo_minus_imu_rad`` in place of ``odo_minus_ref_rad`` (only on that path). With ``encoder`` distance the
    drive's own encoder distance is the denominator (it also ended the segment), so a radius error enters it -- say
    so in the result."""
    ref, source, alias = _heading_ref(heading_ref_dyaw, ref_source, imu_dyaw)
    _positive(distance_m, "distance_m")
    diff = odo_dyaw - ref
    note = ("encoder distance: the drive's own encoder distance is the denominator (it also ended the segment), so a "
            "wheel radius error enters the rate") if _source(distance_source) == "encoder" else ""
    return {"rad_per_m": diff / distance_m, ("odo_minus_imu_rad" if alias else "odo_minus_ref_rad"): diff,
            "ref_source": source, "distance_m": distance_m, "distance_source": distance_source, "note": note}


def m2_gains(*, heading_ref_dyaw: float | None = None, ref_source: str | None = None, imu_dyaw: float | None = None,
             distance_m: float, direction: int, kappa_measured_q: float, kappa_commanded_q: float,
             distance_source: str) -> dict:
    """True curvature = heading change over the segment's distance (not a ratio of means), and two gains:
    ``gain_measured_q`` (the C3 true/kinematic gain's counterpart: the curvature of the steering that was there) and
    ``gain_commanded_q`` (steering tracking included). ``distance_m`` is the travelled length (positive, as the
    profile's ``s_m``) and ``direction`` +1 forward / -1 reverse: reversing with a left curvature turns the heading
    the other way, so the curvature is heading change over the signed distance. The heading change is
    ``heading_ref_dyaw`` from ``ref_source`` or the old ``imu_dyaw`` alias, as in ``m1_drift`` (entry design 3.6,
    4 (가)); the result names its ``ref_source``, and an IMU heading carries the gyro's unmeasured scale into every
    gain -- the note says so; whether such a gain is judgeable is the entry's call (``no_gyro_scale``)."""
    ref, source, _alias = _heading_ref(heading_ref_dyaw, ref_source, imu_dyaw)
    if direction not in (1, -1):
        raise ValueError("direction is +1 (forward) or -1 (reverse)")
    k = ref / (_positive(distance_m, "distance_m") * direction)
    _nonzero(kappa_measured_q, "kappa_measured_q")
    _nonzero(kappa_commanded_q, "kappa_commanded_q")
    notes = []
    if _source(distance_source) == "encoder":
        notes.append("encoder distance: a wheel radius error scales every condition's gain by the same factor; the M1 "
                     "true/encoder distance ratio separates it")
    if source == "imu":
        notes.append("imu heading: the gyro's scale error scales every gain; gyro scale needed")
    return {"kappa_true": k, "gain_measured_q": k / kappa_measured_q, "gain_commanded_q": k / kappa_commanded_q,
            "distance_m": distance_m, "direction": direction, "distance_source": distance_source,
            "ref_source": source, "note": "; ".join(notes)}


# --- M3 ------------------------------------------------------------------------------------------------------------

STEADY_FRACTION = 0.1  # a pre or post window whose spread exceeds this share of the step is not steady


def step_response_delay(q_t_ns, q, y_t_ns, y, *, q_threshold: float, post_window_ns: tuple[int, int],
                        frac: float = 0.63, pre_s: float = 0.5) -> dict:
    """Pass the q-step segment (it starts standing on the old q for at least ``pre_s``). From the q step's onset (the
    first sample ``q_threshold`` away from the mean of the first ``pre_s``) to ``y`` reaching
    ``frac`` of its change, from its mean over ``pre_s`` before the onset to its mean over ``post_window_ns`` -- the
    steady stretch after the step the caller names from the profile (the q-step segment at constant speed), never
    the record's end (Codex G8 review P1). A pre or post window that is not steady (spread over 10 % of the step)
    or a crossing after the post window starts is refused with its reason. The two series may be on different
    clocks only if both were put on the host clock (G7); the IMU's minimum delay against the MCU's is then an
    unknown constant in the result."""
    qt, q = np.asarray(q_t_ns, dtype=np.int64), np.asarray(q, float)
    yt, y = np.asarray(y_t_ns, dtype=np.int64), np.asarray(y, float)
    q0 = q[: max(1, np.searchsorted(qt, qt[0] + int(pre_s * 1e9)))].mean()
    moved = np.flatnonzero(np.abs(q - q0) > q_threshold)
    if len(moved) == 0:
        return {"delay_s": None, "reason": "no_step"}
    onset = int(qt[moved[0]])
    p0, p1 = int(post_window_ns[0]), int(post_window_ns[1])
    if not onset < p0 < p1:
        return {"delay_s": None, "reason": "bad_post_window", "onset_ns": onset}
    pre = y[(yt < onset) & (yt >= onset - int(pre_s * 1e9))]
    post = y[(yt >= p0) & (yt <= p1)]
    if len(pre) < 2 or len(post) < 2:
        return {"delay_s": None, "reason": "no_window", "onset_ns": onset}
    y0, y1 = float(pre.mean()), float(post.mean())
    out = {"onset_ns": onset, "y_pre": y0, "y_post": y1, "frac": frac}
    step = abs(y1 - y0)
    if step < 1e-9:
        return {"delay_s": None, "reason": "no_response", **out}
    if max(float(pre.std()), float(post.std())) > STEADY_FRACTION * step:
        return {"delay_s": None, "reason": "not_steady", **out}
    level = y0 + frac * (y1 - y0)
    after = np.flatnonzero((yt >= onset) & (yt <= p1) & ((y - level) * np.sign(y1 - y0) >= 0))
    if len(after) == 0:
        return {"delay_s": None, "reason": "not_reached", **out}
    i = after[0]
    t_cross = float(yt[i])
    if i > 0 and yt[i - 1] >= onset and y[i] != y[i - 1]:
        t_cross = yt[i - 1] + (level - y[i - 1]) / (y[i] - y[i - 1]) * (yt[i] - yt[i - 1])
    if t_cross > p0:
        return {"delay_s": None, "reason": "crossed_after_post_window_start", **out}
    return {"delay_s": (t_cross - onset) / 1e9, **out}
