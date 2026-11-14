"""Synthetic records for the G8 analysis edge tests (docs/design/2026-11-14-profile-drive-and-m-analysis.md 3).

Pure numpy, no I/O:

- ``profile_rows``: one trial's bridge ``profile`` rows shaped as ``ProfileRunner._row`` writes them -- ``running``
  while a segment runs; the cycle that ends the trial is ``zeroing``, still names the segment it ended in and carries
  the result; later ``zeroing`` rows have no label and keep the result. Without a result the record stops mid-trial.
- ``imu_series``: host-time IMU samples (ns, gyro N x 3, accel N x 3) for a body yaw rate seen through the mount axis
  ``up``: constant or linearly drifting gyro bias, white noise, a gyro scale error, an optional creep before a boundary.
- ``drive_states``: STATE-like samples of a constant-curvature drive whose true wheel radii are the configured ones
  times ``1 + radius_error``, for ``RealOdometry`` re-runs; ``odometry_cfg`` is the matching configuration.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np

NS = 1_000_000_000
G = 9.80665
TRACK_M = 0.53


def ns(t_s: float) -> int:
    return round(t_s * NS)


def unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def horizontal(up) -> np.ndarray:
    """A unit vector perpendicular to ``up`` (the direction a creep's acceleration is put on)."""
    u = unit(up)
    ref = np.array([1.0, 0.0, 0.0]) if abs(u[0]) < 0.9 else np.array([0.0, 0.0, 1.0])
    h = ref - (ref @ u) * u
    return h / np.linalg.norm(h)


# --- profile rows --------------------------------------------------------------------------------------------------

def profile_row(trial, label, host_ns, *, state="running", result=None, seg=0) -> dict:
    """One row with every field ``ProfileRunner._row`` writes (only t, host_ns, trial, state, label and result matter
    to the analysis; the rest are placeholders)."""
    return {"t": "profile", "host_ns": int(host_ns), "trial": int(trial), "state": state, "seg": int(seg),
            "label": label, "s_m": 0.0, "req_mps": 0.0, "req_kappa": 0.0, "arb_mps": 0.0, "arb_limit": None,
            "arb_policy": None, "deadman": False, "drive_mps": 0.0, "drive_q": None, "port": None,
            "direction_cut": None, "meas_q": None, "result": result}


def profile_rows(trial, segments, *, result=None, dt_s=0.02, tail=2) -> list[dict]:
    """``segments``: ``[(label, start_s, end_s), ...]``; a ``running`` row every ``dt_s`` from start to end inclusive.
    With ``result`` the next cycle ends the trial (``zeroing``, the last segment's label) and ``tail`` unlabelled
    ``zeroing`` rows follow; without it the record stops after the last running row."""
    step = ns(dt_s)
    rows: list[dict] = []
    for i, (label, start_s, end_s) in enumerate(segments):
        t, end = ns(start_s), ns(end_s)
        if end < t:
            raise ValueError(f"segment {label!r} ends before it starts")
        while t <= end:
            rows.append(profile_row(trial, label, t, seg=i))
            t += step
    if result is None:
        return rows
    if not segments:
        raise ValueError("a trial with a result needs a segment to end in")
    last = len(segments) - 1
    t = rows[-1]["host_ns"] + step
    rows.append(profile_row(trial, segments[-1][0], t, state="zeroing", result=result, seg=last))
    for _ in range(tail):
        t += step
        rows.append(profile_row(trial, None, t, state="zeroing", result=result, seg=last + 1))
    return rows


# --- IMU -----------------------------------------------------------------------------------------------------------

def imu_series(t0_s, t1_s, rate_hz, yaw_rate_fn, *, up, bias=(0.0, 0.0, 0.0), bias_drift=(0.0, 0.0, 0.0),
               noise=0.0, accel_noise=0.0, scale=1.0, accel_fn=None, creep=None, seed=0):
    """Samples every ``1/rate_hz`` (an integer ns period) from ``t0_s`` to ``t1_s`` inclusive.

    gyro = ``scale`` x body yaw rate x up + bias + bias_drift x (t - t0) + N(0, noise); accel = g along up (the
    upward reaction a standing accelerometer reads) + ``accel_fn(t)`` along a horizontal axis + N(0, accel_noise).
    ``creep = (boundary_s, length_s, yaw_rate, accel)`` adds that yaw rate and acceleration over
    ``[boundary_s - length_s, boundary_s)``. Returns ``(t_ns int64, gyro N x 3, accel N x 3)``."""
    period = round(NS / rate_hz)
    first, last = ns(t0_s), ns(t1_s)
    n = (last - first) // period + 1
    t_ns = first + np.arange(n, dtype=np.int64) * period
    ts = t_ns / NS
    rate = np.array([float(yaw_rate_fn(x)) for x in ts])
    horiz = np.zeros(n) if accel_fn is None else np.array([float(accel_fn(x)) for x in ts])
    if creep is not None:
        boundary_s, length_s, creep_rate, creep_accel = creep
        m = (ts >= boundary_s - length_s) & (ts < boundary_s)
        rate[m] += creep_rate
        horiz[m] += creep_accel
    u, h = unit(up), horizontal(up)
    rng = np.random.default_rng(seed)
    b = np.asarray(bias, float) + np.outer(ts - ts[0], np.asarray(bias_drift, float))
    gyro = scale * np.outer(rate, u) + b + rng.normal(0.0, noise, (n, 3))
    accel = G * np.tile(u, (n, 1)) + np.outer(horiz, h) + rng.normal(0.0, accel_noise, (n, 3))
    return t_ns, gyro, accel


# --- STATE ---------------------------------------------------------------------------------------------------------

class LinearSteering:
    """q = kappa x ``q_per_kappa`` within |q| <= ``q_max``; outside (or non-finite) the map has no curvature."""

    def __init__(self, q_per_kappa: float = 0.65, q_max: float = 0.3) -> None:
        self.q_per_kappa, self.q_max = q_per_kappa, q_max

    def curvature(self, q: float) -> float | None:
        if not math.isfinite(q) or abs(q) > self.q_max:
            return None
        return q / self.q_per_kappa

    def angle(self, kappa: float) -> float:
        return kappa * self.q_per_kappa


def odometry_cfg(*, radius_m=(0.125, 0.125), counts_per_rev=(4096, 4096), q_per_kappa=0.65, q_max=0.3,
                 track_m=TRACK_M) -> SimpleNamespace:
    """What ``RealOdometry`` reads (steering, counts per wheel revolution, configured radii) plus the track."""
    return SimpleNamespace(steering=LinearSteering(q_per_kappa, q_max), counts_per_wheel_rev=tuple(counts_per_rev),
                           wheel_radius_m=tuple(radius_m), track_m=track_m)


def wrap_i32(count: int) -> int:
    """A count as the STATE's signed 32-bit field carries it."""
    return (int(count) + 2**31) % 2**32 - 2**31


def odometry_distance_factor(kappa: float, radius_error=(0.0, 0.0), track_m: float = TRACK_M) -> float:
    """Odometry distance / true rear-axle distance when the true radii are the configured ones x (1 + error)."""
    half = track_m / 2
    return ((1 - kappa * half) / (1 + radius_error[0]) + (1 + kappa * half) / (1 + radius_error[1])) / 2


def drive_states(n, *, speed_mps, kappa, cfg, radius_error=(0.0, 0.0), dt_us=10_000, t0_us=0, boot_id=1,
                 count0=(0, 0), s0_m=0.0, yaw0=0.0) -> list[SimpleNamespace]:
    """``n`` STATEs every ``dt_us`` of a drive at constant signed ``speed_mps`` and rear-axle curvature ``kappa``
    (+ = left when moving forward; q = the map's angle for it). Counts are rounded to the nearest integer, offset by
    ``count0`` and wrapped to int32. Extra fields: wheel rates (true, rad/s), ``true_s_m`` (signed rear-axle distance,
    from ``s0_m``) and ``true_yaw`` (from ``yaw0``)."""
    half = cfg.track_m / 2
    r_true = tuple(cfg.wheel_radius_m[i] * (1 + radius_error[i]) for i in (0, 1))
    per_wheel = (1 - kappa * half, 1 + kappa * half)
    q = cfg.steering.angle(kappa)
    out = []
    for k in range(n):
        s = speed_mps * dt_us * k / 1e6
        counts = [wrap_i32(int(count0[i]) + math.floor(
            s * per_wheel[i] / r_true[i] * cfg.counts_per_wheel_rev[i] / (2 * math.pi) + 0.5)) for i in (0, 1)]
        out.append(SimpleNamespace(
            mcu_time_us=int(t0_us + k * dt_us), boot_id=boot_id, wheel_count_left=counts[0],
            wheel_count_right=counts[1], steering_rad=q, fault_flags=0,
            wheel_rate_left_rad_s=speed_mps * per_wheel[0] / r_true[0],
            wheel_rate_right_rad_s=speed_mps * per_wheel[1] / r_true[1],
            true_s_m=s0_m + s, true_yaw=yaw0 + kappa * s))
    return out
