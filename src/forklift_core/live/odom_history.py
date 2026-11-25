"""The odometry history entries the real loop builds from its STATEs (real safety wiring design 5a, 1.1) -- one copy,
shared by the loop and the observe run's SLAM path (PR 7 design 6.1, 8: "the loop's own function, never a copy").

- ``odometry_step(odometry, state)``: the raw odometry advanced by one STATE, the encoder fault bits as its cut.
- ``odom_entry(...)``: the STATE's measurement interval ``[receipt - delay bound, receipt]`` (core seconds) with its raw
  odometry pose, ``included`` only with a bound within ``state_delay_s``, and the motion bounds a scan placement checks
  against ``v_scan``: each wheel's speed over the interval from the previous STATE (MCU time) plus its error width
  (the acceleration over half the interval and a count at each end -- ``EncoderSpeedParams.error``), so the largest
  speed inside the interval, not its mean (Codex 5a impl P1-4); the yaw rate from the wheel difference with both
  widths. An entry is ``(lo_s, hi_s, pose, continuous, included, speed_ok, yaw_ok)``.
"""

from __future__ import annotations

import math

from forklift_core.mcu_link.messages import FaultFlag
from forklift_core.plant.real_vehicle import wheel_count_delta

ENCODER_CUT = FaultFlag.ENCODER_GAP | FaultFlag.ENCODER_JUMP_L | FaultFlag.ENCODER_JUMP_R  # contract 3.3


def odometry_step(odometry, state):
    return odometry.update(int(state.mcu_time_us), int(state.boot_id), int(state.wheel_count_left),
                           int(state.wheel_count_right), float(state.steering_rad),
                           encoder_cut=bool(int(state.fault_flags) & int(ENCODER_CUT)))


def odom_entry(state, step, *, hi_s: float, delay_b, state_delay_s: float, prev, has_history: bool, params,
               v_scan_mps: float, kappa_max: float):
    """``(entry, prev)``: this STATE's entry and the reference the next one compares with (``prev`` is the previous
    STATE's ``(boot, MCU s, counts)``; ``has_history``: an entry exists before this one)."""
    included = delay_b is not None and delay_b <= state_delay_s
    lo = hi_s - (state_delay_s if delay_b is None else delay_b)
    pose = tuple(float(v) for v in step.pose)
    speed_ok = yaw_ok = True
    mcu_s = int(state.mcu_time_us) / 1e6
    counts = (int(state.wheel_count_left), int(state.wheel_count_right))
    nxt = (int(state.boot_id), mcu_s, counts)
    if has_history and prev is not None and prev[0] == int(state.boot_id):
        dt = mcu_s - prev[1]  # the MCU's own interval between the two measurements
        if dt <= 0.0:
            speed_ok = yaw_ok = False
        else:
            p = params
            v, e = [], []
            for w in (0, 1):
                dc = wheel_count_delta(counts[w], prev[2][w])
                v.append(2.0 * math.pi * p.radius_m[w] * dc / (p.counts_per_rev[w] * dt))
                e.append(p.error(w, dt, 0.0))
            if max(abs((v[0] + v[1]) / 2.0) + max(e), 0.0) > v_scan_mps + 1e-9:
                speed_ok = False
            if (abs(v[1] - v[0]) + e[0] + e[1]) / p.track_m > v_scan_mps * max(kappa_max, 1e-9) + 1e-9:
                yaw_ok = False
    return (lo, hi_s, pose, not step.continuity_broken, included, speed_ok, yaw_ok), nxt


__all__ = ["ENCODER_CUT", "odom_entry", "odometry_step"]
