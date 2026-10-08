"""Sensor degradations for the SLAM comparison, applied at replay or send time.

Plan: docs/plans/2026-10-07-visual-slam-and-fusion.md, D5. The Isaac record
stays clean; a condition decides, per frame, whether the LiDAR and the cameras
delivered anything, clips LiDAR ranges, and scales the wheel distance an
encoder reports. The same object serves the offline record player and the
online runner, so both see the identical degradation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from forklift_core._validation import _finite_scalar

# Three 20 s windows spread over the ~245 s survey loop.
_BLACKOUT_WINDOWS_S = ((60.0, 80.0), (130.0, 150.0), (200.0, 220.0))


@dataclass(frozen=True)
class SensorCondition:
    """What reaches the SLAM front ends at time t (simulation seconds).

    ``lidar_blackout_s`` / ``camera_blackout_s``: half-open windows [start,
    end) in which that sensor delivers nothing (a driver that stops
    publishing, not a frame of empty readings). ``lidar_max_range_m``: beams
    farther than this read +inf, nothing in range (REP-117). ``wheel_distance_scale``
    multiplies the rear wheel rates the odometry integrates (1.05: it believes
    5 % more travel than happened).
    """

    name: str
    lidar_blackout_s: tuple[tuple[float, float], ...] = ()
    camera_blackout_s: tuple[tuple[float, float], ...] = ()
    lidar_max_range_m: float | None = None
    wheel_distance_scale: float = 1.0

    def __post_init__(self) -> None:
        for windows in (self.lidar_blackout_s, self.camera_blackout_s):
            for start, end in windows:
                if not _finite_scalar(start, "window start") < _finite_scalar(
                    end, "window end"
                ):
                    raise ValueError("blackout windows need start < end")
        if self.lidar_max_range_m is not None and not (
            _finite_scalar(self.lidar_max_range_m, "lidar_max_range_m") > 0
        ):
            raise ValueError("lidar_max_range_m must be positive")
        if not _finite_scalar(self.wheel_distance_scale, "wheel_distance_scale") > 0:
            raise ValueError("wheel_distance_scale must be positive")

    @staticmethod
    def _inside(windows, stamp_s: float) -> bool:
        return any(start <= stamp_s < end for start, end in windows)

    def lidar_available(self, stamp_s: float) -> bool:
        return not self._inside(self.lidar_blackout_s, stamp_s)

    def cameras_available(self, stamp_s: float) -> bool:
        return not self._inside(self.camera_blackout_s, stamp_s)

    def ranges(self, ranges_m: np.ndarray) -> np.ndarray:
        """Ranges beyond the limit read +inf; others (and +-inf) are unchanged."""
        out = np.asarray(ranges_m, dtype=float).copy()
        if self.lidar_max_range_m is not None:
            far = np.isfinite(out) & (out > self.lidar_max_range_m)
            out[far] = math.inf
        return out

    def wheel_rates(self, rates_rad_s) -> np.ndarray:
        return np.asarray(rates_rad_s, dtype=float) * self.wheel_distance_scale


CONDITIONS = {
    "nominal": SensorCondition("nominal"),
    "lidar_blackout": SensorCondition(
        "lidar_blackout", lidar_blackout_s=_BLACKOUT_WINDOWS_S
    ),
    "lidar_short": SensorCondition("lidar_short", lidar_max_range_m=4.0),
    "camera_blackout": SensorCondition(
        "camera_blackout", camera_blackout_s=_BLACKOUT_WINDOWS_S
    ),
    "wheel_slip": SensorCondition("wheel_slip", wheel_distance_scale=1.05),
}


def condition(name: str) -> SensorCondition:
    try:
        return CONDITIONS[name]
    except KeyError:
        raise ValueError(
            f"unknown sensor condition {name!r}; known: {sorted(CONDITIONS)}"
        ) from None


__all__ = ["CONDITIONS", "SensorCondition", "condition"]
