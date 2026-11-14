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
from forklift_core.sensors.imu_mount import yaw_rate

DISTANCE_SOURCES = ("truth", "encoder")
QUIET_GYRO_RAD_S = 0.01  # a sample's gyro within this of the window's median ...
QUIET_ACCEL_MPS2 = 0.15  # ... and its acceleration within this: quiet (proposed; registered before measuring)
_ENCODER_CUT = FaultFlag.ENCODER_GAP | FaultFlag.ENCODER_JUMP_L | FaultFlag.ENCODER_JUMP_R


# --- trials --------------------------------------------------------------------------------------------------------

@dataclass
class Trial:
    trial: int
    result: str | None = None
    spans: dict = field(default_factory=dict)  # segment label -> (first host_ns, last host_ns) of its rows

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
            s = tr.spans.get(label)
            tr.spans[label] = (int(r["host_ns"]), int(r["host_ns"])) if s is None else (s[0], int(r["host_ns"]))
    found = sorted(out.values(), key=lambda t: t.trial)
    return [t for t in found if t.valid] if valid_only else found


# --- IMU -----------------------------------------------------------------------------------------------------------

def quiet_window(t_ns, gyro, accel, *, boundary_ns: int, side: str, max_s: float, min_s: float,
                 gyro_tol: float = QUIET_GYRO_RAD_S, accel_tol: float = QUIET_ACCEL_MPS2) -> tuple[int, int] | None:
    """``[start, end)`` indices of the bias window next to a trial: within ``max_s`` before (or after) the boundary,
    the contiguous run of quiet samples (gyro and acceleration near the stretch's median) that is closest to the
    boundary and at least ``min_s`` long -- motion at the trial's edge (a creep, the end of a drive) is cut off.
    None when there is no such run."""
    t = np.asarray(t_ns, dtype=np.int64)
    if side == "before":
        sel = np.flatnonzero((t <= boundary_ns) & (t >= boundary_ns - int(max_s * 1e9)))
    elif side == "after":
        sel = np.flatnonzero((t >= boundary_ns) & (t <= boundary_ns + int(max_s * 1e9)))
    else:
        raise ValueError("side is 'before' or 'after'")
    if len(sel) == 0:
        return None
    g, a = np.asarray(gyro, float)[sel], np.asarray(accel, float)[sel]
    quiet = (np.linalg.norm(g - np.median(g, axis=0), axis=1) <= gyro_tol) & \
            (np.linalg.norm(a - np.median(a, axis=0), axis=1) <= accel_tol)
    runs, start = [], None
    for i, ok in enumerate(quiet):
        if ok and start is None:
            start = i
        elif not ok and start is not None:
            runs.append((start, i))
            start = None
    if start is not None:
        runs.append((start, len(quiet)))
    long_enough = [(s, e) for s, e in runs if (t[sel[e - 1]] - t[sel[s]]) / 1e9 >= min_s]
    if not long_enough:
        return None
    s, e = long_enough[-1] if side == "before" else long_enough[0]
    return int(sel[s]), int(sel[e - 1]) + 1


def imu_heading(t_ns, gyro, up, bias) -> np.ndarray:
    """The body heading (rad) from the bias-free yaw rate, trapezoid-integrated, 0 at the first sample."""
    t = (np.asarray(t_ns, dtype=np.int64) - int(t_ns[0])) / 1e9
    w = yaw_rate(gyro, up, bias=bias)
    out = np.zeros(len(t))
    out[1:] = np.cumsum((w[1:] + w[:-1]) / 2 * np.diff(t))
    return out


def heading_sigma_lower(*, arw_rad_per_sqrt_s: float, drive_s: float, bias_window_s: float,
                        bias_change_rad_s: float) -> dict:
    """A lower bound on the 1-sigma heading error over a drive (design 3): white noise over the drive, the bias
    estimate's noise (the mean of two windows of ``bias_window_s``) carried over the drive, and half the bias change
    between the windows over the drive. Not included: gyro scale (measured separately), mount axis error and body
    roll/pitch (diagnostics only), so a value under 1 mrad does not by itself make a heading judgeable."""
    white = arw_rad_per_sqrt_s * math.sqrt(drive_s)
    bias_mean = arw_rad_per_sqrt_s / math.sqrt(2 * bias_window_s) * drive_s
    change = abs(bias_change_rad_s) * drive_s / 2
    return {"white": white, "bias_mean": bias_mean, "bias_change": change,
            "lower_bound": math.sqrt(white ** 2 + bias_mean ** 2 + change ** 2), "is_lower_bound": True,
            "not_included": ["scale", "mount_axis", "roll_pitch"]}


def gyro_scale(*, imu_dyaw: float, turns: int, heading_error_rad: float = 0.0) -> dict:
    """The gyro's scale from G3 arcs: the bias-free IMU rotation over ``turns`` full turns that start and end on the
    same floor-marked heading, over 2 pi turns. ``heading_error_rad`` is the floor marks' 1-sigma heading error at
    each end; it sets the scale's uncertainty (sqrt 2 x error over 2 pi turns)."""
    if turns < 1:
        raise ValueError("at least one full turn")
    nominal = 2 * math.pi * turns
    return {"scale": abs(imu_dyaw) / nominal, "sigma": math.sqrt(2) * heading_error_rad / nominal, "turns": turns}


# --- odometry ------------------------------------------------------------------------------------------------------

def odometry_heading(states, cfg, *, max_gap_s: float) -> dict:
    """The real truck's odometry re-run over recorded STATEs (each with mcu_time_us, boot_id, wheel counts,
    steering_rad, fault_flags): the yaw after each sample and where continuity broke."""
    odo = RealOdometry(cfg, max_gap_s=max_gap_s)
    t, yaw, broken = [], [], []
    for s in states:
        cut = bool(int(s.fault_flags) & int(_ENCODER_CUT))
        step = odo.update(s.mcu_time_us, s.boot_id, s.wheel_count_left, s.wheel_count_right, s.steering_rad,
                          encoder_cut=cut)
        t.append(int(s.mcu_time_us))
        yaw.append(step.pose[2])
        if step.continuity_broken and len(t) > 1:
            broken.append((int(s.mcu_time_us), step.reason))
    return {"mcu_time_us": np.asarray(t, dtype=np.int64), "yaw": np.asarray(yaw), "broken": broken}


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


def m1_drift(*, odo_dyaw: float, imu_dyaw: float, distance_m: float, distance_source: str) -> dict:
    """Odometry heading minus IMU heading per metre. With ``encoder`` distance the drive's own encoder distance is
    the denominator (it also ended the segment), so a radius error enters it -- say so in the result."""
    diff = odo_dyaw - imu_dyaw
    return {"rad_per_m": diff / distance_m, "odo_minus_imu_rad": diff, "distance_m": distance_m,
            "distance_source": _source(distance_source)}


def m2_gains(*, imu_dyaw: float, distance_m: float, kappa_measured_q: float, kappa_commanded_q: float,
             distance_source: str) -> dict:
    """True curvature = heading change over the segment's distance (not a ratio of means), and two gains:
    ``gain_measured_q`` (the C3 true/kinematic gain's counterpart: the curvature of the steering that was there) and
    ``gain_commanded_q`` (steering tracking included)."""
    k = imu_dyaw / distance_m
    note = ("encoder distance: a wheel radius error scales every condition's gain by the same factor; the M1 "
            "true/encoder distance ratio separates it") if _source(distance_source) == "encoder" else ""
    return {"kappa_true": k, "gain_measured_q": k / kappa_measured_q, "gain_commanded_q": k / kappa_commanded_q,
            "distance_m": distance_m, "distance_source": distance_source, "note": note}


# --- M3 ------------------------------------------------------------------------------------------------------------

def step_response_delay(q_t_ns, q, y_t_ns, y, *, q_threshold: float, frac: float = 0.63, pre_s: float = 0.5,
                        post_s: float = 1.0) -> dict:
    """From the q step's onset (the first sample ``q_threshold`` away from the mean before it) to ``y`` reaching
    ``frac`` of its change (mean over ``pre_s`` before the onset to mean over the last ``post_s`` of the record).
    The two series may be on different clocks only if both were put on the host clock (G7); the IMU's minimum
    delay against the MCU's is then an unknown constant in the result."""
    qt, q = np.asarray(q_t_ns, dtype=np.int64), np.asarray(q, float)
    yt, y = np.asarray(y_t_ns, dtype=np.int64), np.asarray(y, float)
    q0 = q[: max(1, np.searchsorted(qt, qt[0] + int(pre_s * 1e9)))].mean()
    moved = np.flatnonzero(np.abs(q - q0) > q_threshold)
    if len(moved) == 0:
        return {"delay_s": None, "reason": "no_step"}
    onset = int(qt[moved[0]])
    pre = y[(yt < onset) & (yt >= onset - int(pre_s * 1e9))]
    post = y[yt >= yt[-1] - int(post_s * 1e9)]
    if len(pre) == 0 or len(post) == 0:
        return {"delay_s": None, "reason": "no_window", "onset_ns": onset}
    y0, y1 = float(pre.mean()), float(post.mean())
    if abs(y1 - y0) < 1e-9:
        return {"delay_s": None, "reason": "no_response", "onset_ns": onset, "y_pre": y0, "y_post": y1}
    level = y0 + frac * (y1 - y0)
    after = np.flatnonzero((yt >= onset) & ((y - level) * np.sign(y1 - y0) >= 0))
    if len(after) == 0:
        return {"delay_s": None, "reason": "not_reached", "onset_ns": onset, "y_pre": y0, "y_post": y1}
    i = after[0]
    # interpolate the crossing between the sample before and this one
    t_cross = float(yt[i])
    if i > 0 and yt[i - 1] >= onset and y[i] != y[i - 1]:
        t_cross = yt[i - 1] + (level - y[i - 1]) / (y[i] - y[i - 1]) * (yt[i] - yt[i - 1])
    return {"delay_s": (t_cross - onset) / 1e9, "onset_ns": onset, "y_pre": y0, "y_post": y1, "frac": frac}
