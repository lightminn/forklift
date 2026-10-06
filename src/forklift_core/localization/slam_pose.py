"""Online pose estimate from wheel odometry and a SLAM map<-odom correction.

Plan: docs/plans/2026-10-04-online-slam-closed-loop.md. The estimate is
map<-base(t) = map<-odom(last SLAM reply) * odom<-base(t, odometry): odometry
carries the pose between scans, SLAM corrects its drift. A correction older
than ``max_age_s`` stops the estimate (``LocalizationStale``) instead of letting
the robot drive on dead reckoning (roadmap: never drive on stale information).
Poses are planar (x, y metres, yaw radians).
"""

from __future__ import annotations

from math import atan2, cos, sin

import math

import numpy as np

from forklift_core._validation import _finite_scalar
from forklift_core.localization.wheel_odometry import (
    AckermannOdometryGeometry,
    _curvature_inv_m,
)


class LocalizationStale(RuntimeError):
    """No SLAM correction recent enough to drive on."""


def compose(a, b) -> tuple[float, float, float]:
    """a * b for planar poses (b expressed in a's frame)."""
    ax, ay, ayaw = (float(v) for v in a)
    bx, by, byaw = (float(v) for v in b)
    c, s = cos(ayaw), sin(ayaw)
    return ax + c * bx - s * by, ay + s * bx + c * by, ayaw + byaw


def invert(a) -> tuple[float, float, float]:
    x, y, yaw = (float(v) for v in a)
    c, s = cos(yaw), sin(yaw)
    return -(c * x + s * y), s * x - c * y, -yaw


def wrap(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


class IncrementalWheelOdometry:
    """``integrate_wheel_odometry`` one sample at a time, bit for bit.

    Each interval uses the mean speed and curvature of its two end samples and
    advances along that exact arc; yaw is left unwrapped like the batch form.
    """

    def __init__(self, geometry: AckermannOdometryGeometry, *, initial_pose=(0.0, 0.0, 0.0)):
        self.geometry = geometry
        self.pose = [_finite_scalar(v, "initial pose") for v in initial_pose]
        self._last = None  # (stamp, speed, curvature)

    def update(self, stamp_s, rear_wheel_rates_rad_s, steering_rad) -> tuple[float, float, float]:
        stamp = _finite_scalar(stamp_s, "stamp_s")
        rates = np.asarray(rear_wheel_rates_rad_s, dtype=float)
        steering = np.asarray(steering_rad, dtype=float)
        if rates.shape != (2,) or steering.shape != (2,):
            raise ValueError("wheel rates and steering must each hold two values")
        if not (np.isfinite(rates).all() and np.isfinite(steering).all()):
            raise ValueError("wheel rates and steering must be finite")
        if np.any(np.abs(steering) >= np.pi / 2):
            raise ValueError("steering angles must be below pi/2")
        speed = rates.mean() * self.geometry.wheel_radius_m
        curvature = float(_curvature_inv_m(steering[None, :], self.geometry)[0])
        if self._last is not None:
            last_stamp, last_speed, last_curvature = self._last
            dt = stamp - last_stamp
            if dt <= 0:
                raise ValueError("stamps must be strictly increasing")
            x, y, yaw = self.pose
            distance = (last_speed + speed) / 2 * dt
            kappa = (last_curvature + curvature) / 2
            turn = distance * kappa
            if abs(turn) < 1e-12:
                x += distance * cos(yaw + turn / 2)
                y += distance * sin(yaw + turn / 2)
            else:
                x += (sin(yaw + turn) - sin(yaw)) / kappa
                y += (cos(yaw) - cos(yaw + turn)) / kappa
            yaw += turn
            self.pose = [x, y, yaw]
        self._last = (stamp, speed, curvature)
        return tuple(self.pose)


class SlamPoseEstimator:
    """Holds the latest map<-odom and composes it with the odometry pose."""

    def __init__(self, *, max_age_s: float):
        self.max_age_s = _finite_scalar(max_age_s, "max_age_s")
        if self.max_age_s <= 0:
            raise ValueError("max_age_s must be positive")
        self.map_from_odom = None
        self.correction_stamp_s = None

    def set_map_from_odom(self, stamp_s, map_from_odom) -> None:
        stamp = _finite_scalar(stamp_s, "stamp_s")
        if self.correction_stamp_s is not None and stamp < self.correction_stamp_s:
            raise ValueError("SLAM corrections must not go back in time")
        pose = tuple(_finite_scalar(v, "map_from_odom") for v in map_from_odom)
        self.map_from_odom, self.correction_stamp_s = pose, stamp

    def age_s(self, now_s) -> float | None:
        if self.correction_stamp_s is None:
            return None
        return _finite_scalar(now_s, "now_s") - self.correction_stamp_s

    def map_from_base(self, now_s, odom_from_base) -> tuple[float, float, float]:
        age = self.age_s(now_s)
        if age is None or age > self.max_age_s + 1e-12:
            raise LocalizationStale(
                "no SLAM correction yet" if age is None else f"correction {age:.3f} s old"
            )
        return compose(self.map_from_odom, odom_from_base)


# --- plan v3: received/applied corrections, stop detection, sensor noise ------


class SlamPoseTracker:
    """map<-base for control from SLAM replies, in three modes (plan v3.4).

    ``received`` is the last ``processed`` correction with the stamp of the
    last reply that confirmed it (``processed``, or ``skipped`` -- a scan the
    bridge did not send to slam_toolbox, or one slam_toolbox dropped at start-up;
    the bridge is alive and the correction stands); ``applied`` is the one
    control uses. warming_up: drive held until a ``processed`` reply for scan id
    >= 4 (the bridge sends the first five scans; slam_toolbox answers the 1st
    and 5th). tracking: every processed correction is
    applied; stale when the received stamp is older than ``max_age_s``. holding (docking): new corrections are received but not
    applied; freshness is checked on the received stamp, and dead reckoning
    since the hold must stay within ``hold_limit_m``.
    """

    def __init__(self, *, max_age_s: float, hold_limit_m: float):
        self.max_age_s = _finite_scalar(max_age_s, "max_age_s")
        self.hold_limit_m = _finite_scalar(hold_limit_m, "hold_limit_m")
        if self.max_age_s <= 0 or self.hold_limit_m <= 0:
            raise ValueError("limits must be positive")
        self.mode = "warming_up"
        self.received = None  # (map_from_odom, stamp_s, scan_id)
        self.applied = None
        self.version = 0
        self._hold_start = None
        self._hold_last = None
        self.hold_travel_m = 0.0
        self.skipped_replies = 0

    def receive(self, scan_id: int, stamp_s: float, status: str, map_from_odom) -> None:
        stamp = _finite_scalar(stamp_s, "stamp_s")
        if self.received is not None and stamp < self.received[1]:
            raise ValueError("SLAM replies must not go back in time")
        if status == "skipped":
            if self.received is None:
                raise ValueError("a skipped reply before any processed one")
            # Liveness only: the correction is the last processed one.
            self.received = (self.received[0], stamp, int(scan_id))
            if self.mode == "tracking":
                self.applied = self.received
            self.skipped_replies += 1
            return
        if status != "processed":
            raise ValueError(f"unexpected reply status {status!r}")
        pose = tuple(_finite_scalar(v, "map_from_odom") for v in map_from_odom)
        self.received = (pose, stamp, int(scan_id))
        self.version += 1
        if self.mode == "warming_up":
            self.applied = self.received
            if scan_id >= 4:
                self.mode = "tracking"
        elif self.mode == "tracking":
            self.applied = self.received

    def may_drive(self) -> bool:
        return self.mode != "warming_up"

    def hold(self, *, odom_from_base) -> None:
        if self.mode != "tracking":
            raise ValueError("can only hold while tracking")
        self.mode = "holding"
        self._hold_start = tuple(float(v) for v in odom_from_base)
        self._hold_last = self._hold_start
        self.hold_travel_m = 0.0

    def release(self, *, odom_from_base) -> tuple[float, float]:
        """Apply the received correction; return the position and yaw jump."""
        if self.mode != "holding":
            raise ValueError("not holding")
        before = compose(self.applied[0], odom_from_base)
        self.applied = self.received
        after = compose(self.applied[0], odom_from_base)
        self.mode, self._hold_start, self._hold_last = "tracking", None, None
        return (
            float(np.hypot(after[0] - before[0], after[1] - before[1])),
            abs(wrap(after[2] - before[2])),
        )

    def map_from_base(self, now_s, odom_from_base) -> tuple[float, float, float]:
        now = _finite_scalar(now_s, "now_s")
        if self.applied is None:
            raise LocalizationStale("no SLAM correction yet")
        if self.mode != "warming_up":
            age = now - self.received[1]
            if age > self.max_age_s + 1e-12:
                raise LocalizationStale(f"last processed correction {age:.3f} s old")
        if self.mode == "holding":
            # Distance travelled (sum of steps), not displacement: insert and
            # back out again is dead reckoning both ways.
            self.hold_travel_m += float(
                np.hypot(
                    odom_from_base[0] - self._hold_last[0],
                    odom_from_base[1] - self._hold_last[1],
                )
            )
            self._hold_last = (float(odom_from_base[0]), float(odom_from_base[1]))
            if self.hold_travel_m > self.hold_limit_m:
                raise LocalizationStale(
                    f"dead reckoning {self.hold_travel_m:.2f} m while holding"
                )
        return compose(self.applied[0], odom_from_base)


class StopDetector:
    """Stopped = zero command held, and windowed mean speed and yaw rate small.

    Noisy odometry speed (sigma about 0.019 m/s at rest with the plan's wheel
    noise) cannot pass a per-sample 0.012 m/s gate reliably; the 0.1 s mean
    cuts the sigma by sqrt(12).
    """

    def __init__(
        self,
        *,
        tick_s: float,
        command_hold_s: float = 0.2,
        window_s: float = 0.1,
        run_s: float = 0.1,
        speed_mps: float = 0.012,
        yaw_rate_radps: float = 0.02,
    ):
        self.window = max(1, int(round(window_s / tick_s)))
        self.command_ticks = max(1, int(round(command_hold_s / tick_s)))
        self.run_ticks = max(1, int(round(run_s / tick_s)))
        self.speed_mps, self.yaw_rate_radps = speed_mps, yaw_rate_radps
        self._speeds, self._yaw_rates = [], []
        self._zero_command = self._run = 0
        self.stopped = False
        self._direction = 1.0

    def update(self, *, commanded_speed: float, speed: float, yaw_rate: float) -> bool:
        self._speeds = (self._speeds + [float(speed)])[-self.window :]
        self._yaw_rates = (self._yaw_rates + [float(yaw_rate)])[-self.window :]
        self._zero_command = self._zero_command + 1 if commanded_speed == 0.0 else 0
        if commanded_speed != 0.0:
            self._direction = math.copysign(1.0, commanded_speed)
        quiet = (
            len(self._speeds) == self.window
            and abs(sum(self._speeds) / self.window) <= self.speed_mps
            and abs(sum(self._yaw_rates) / self.window) <= self.yaw_rate_radps
        )
        self._run = self._run + 1 if quiet else 0
        self.stopped = self._zero_command >= self.command_ticks and self._run >= self.run_ticks
        return self.stopped

    @property
    def mean_speed(self) -> float:
        return sum(self._speeds) / len(self._speeds) if self._speeds else 0.0

    def tracker_speed(self, stop_speed_mps: float) -> float:
        """The speed to hand a path tracker that judges a stop by one sample.

        The windowed mean, except that while this detector has not passed a
        stop its magnitude is kept just above ``stop_speed_mps`` -- so the
        tracker's own arrival and gear-change checks wait for this detector
        (noisy odometry would otherwise pass them on a lucky sample). That
        magnitude carries the sign of the last non-zero command, never the
        noise: starting a reverse leg from rest must not read as still rolling
        forward, which the tracker would brake against.
        """
        mean = self.mean_speed
        if self.stopped or abs(mean) > stop_speed_mps:
            return mean
        return self._direction * stop_speed_mps * 1.01


class OdometryNoise:
    """The 9/26 replay's assumed noise, drawn online per sample (plan v3).

    Rear wheel rates sigma 0.2 rad/s and steering sigma 0.005 rad per 120 Hz
    joint sample, ranges sigma 0.02 m on measured beams clipped to the sensor
    range; independent streams per sensor from one seed, like
    forklift_ros.slam_replay. Assumed values, not A2M12 or encoder specs.
    """

    def __init__(
        self,
        *,
        seed: int,
        enabled: bool = True,
        wheel_rate_std_rad_s: float = 0.2,
        steering_std_rad: float = 0.005,
        range_std_m: float = 0.02,
        range_model: str = "constant",
    ):
        self.enabled = bool(enabled)
        self.wheel_rate_std_rad_s = wheel_rate_std_rad_s
        self.steering_std_rad = steering_std_rad
        self.range_std_m = range_std_m
        # "constant": range_std_m on every beam. "a2m12": the RPLIDAR A2M12
        # datasheet maximum error as sigma -- 1 % of range up to 3 m, 2 % to
        # 5 m, 2.5 % beyond (week-6 spec-noise runs).
        if range_model not in ("constant", "a2m12"):
            raise ValueError("range_model must be 'constant' or 'a2m12'")
        self.range_model = range_model
        self._ranges = np.random.default_rng([seed, 0])
        self._wheels = np.random.default_rng([seed, 1])
        self._steering = np.random.default_rng([seed, 2])

    def wheel_rates(self, rates) -> tuple[float, float]:
        rates = np.asarray(rates, dtype=float)
        if self.enabled:
            rates = rates + self._wheels.normal(0, self.wheel_rate_std_rad_s, 2)
        return tuple(float(v) for v in rates)

    def steering(self, angles) -> tuple[float, float]:
        angles = np.asarray(angles, dtype=float)
        if self.enabled:
            angles = angles + self._steering.normal(0, self.steering_std_rad, 2)
        return tuple(float(v) for v in angles)

    def ranges(self, ranges, *, range_min_m: float, range_max_m: float) -> np.ndarray:
        out = np.asarray(ranges, dtype=float).copy()
        if not self.enabled:
            return out
        measured = np.isfinite(out)
        if self.range_model == "a2m12":
            r = out[measured]
            sigma = np.where(r <= 3.0, 0.01, np.where(r <= 5.0, 0.02, 0.025)) * r
            out[measured] += self._ranges.normal(0, 1.0, int(measured.sum())) * sigma
        else:
            out[measured] += self._ranges.normal(0, self.range_std_m, int(measured.sum()))
        out[measured] = np.clip(out[measured], range_min_m, range_max_m)
        return out


__all__ = [
    "OdometryNoise",
    "SlamPoseTracker",
    "StopDetector",
    "IncrementalWheelOdometry",
    "LocalizationStale",
    "SlamPoseEstimator",
    "compose",
    "invert",
    "wrap",
]
