"""Sensor input contracts for the plant port and the real truck's pure conversions (sim-to-real P0-4 4c-1).

``PlanarScan`` is one LiDAR revolution with every beam's status kept apart: a hit (a range), a verified no-return
(nothing within range, REP-117 +inf), too close (REP-117 -inf: a detection under range_min), unknown (NaN, or a
finite value outside the stated bounds) and own (the truck's body or load, from measured mount sectors). An
unknown beam is never a no-return: a consumer must not clear free space on it; an own beam neither occupies nor
clears. The scan is not deskewed; ``beam_times_s`` gives each beam's time for a consumer that aligns odometry.

``RgbdFrame`` is one colour image and the depth registered to the same rectified grid, in metres (NaN unknown),
with both acquisition stamps, the host receipt, the stream's own depth scale and where the calibration came from.

Clocks: ``stamp_s`` / ``color_stamp_s`` / ``depth_stamp_s`` are in the clock ``stamp_domain`` names ("ros": the
real driver's header; "sim": Isaac; "device"); ``received_host_ns`` is the host monotonic receipt. Different
clocks are never subtracted without a stated conversion (CONTRIBUTING 5); an RGB-D pair shares one domain. The Isaac side's extraction into these contracts is 4c-2; the consumers (obstacle layer, SLAM,
far recognition) on real data are 4e-2.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum

import numpy as np
from numpy.typing import ArrayLike

from forklift_core._validation import _finite_scalar, _frame_id, _real_array
from forklift_core.sensors.rgbd import PinholeIntrinsics


class BeamStatus(IntEnum):
    HIT = 0
    NO_RETURN = 1  # verified: nothing within range (REP-117 +inf)
    UNKNOWN = 2  # invalid (NaN), or a finite value outside the stream's stated bounds
    OWN = 3  # the truck's own body or load (measured mount sectors): neither occupies nor clears
    TOO_CLOSE = 4  # REP-117 -inf: something detected closer than range_min (the Isaac grid occupies near it)


STAMP_DOMAINS = ("ros", "sim", "device")  # the clock a sensor stamp is in; pairs must share one


@dataclass(frozen=True)
class PlanarScan:
    frame_id: str
    angles_rad: np.ndarray  # (N,) laser frame, zero along +x, positive toward +y
    ranges_m: np.ndarray  # (N,) the range of a HIT, NaN for every other status
    status: np.ndarray  # (N,) BeamStatus values
    range_min_m: float
    range_max_m: float
    stamp_s: float  # time of the first beam, in stamp_domain's clock
    stamp_domain: str  # "ros" (the real driver's header), "sim" (Isaac), "device"
    time_increment_s: float  # between beams
    scan_time_s: float  # between scans
    received_host_ns: int
    own_mask_source: str | None = None  # where the own sectors came from (S7 measurement, load state)
    deskewed: bool = False

    def beam_times_s(self) -> np.ndarray:
        return self.stamp_s + np.arange(len(self.ranges_m)) * self.time_increment_s

    def hit_points_xy(self) -> np.ndarray:
        """(M, 2) laser-frame points of the hits only."""
        hit = self.status == BeamStatus.HIT
        r, a = self.ranges_m[hit], self.angles_rad[hit]
        return np.column_stack([r * np.cos(a), r * np.sin(a)])


def _wrap(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


def planar_scan_from_laserscan(ranges: ArrayLike, *, angle_min_rad: float, angle_increment_rad: float,
                               range_min_m: float, range_max_m: float, time_increment_s: float, scan_time_s: float,
                               stamp_s: float, stamp_domain: str, frame_id: str, received_host_ns: int,
                               own_sectors_rad: tuple = (), own_mask_source: str | None = None) -> PlanarScan:
    """A sensor_msgs/LaserScan's fields -> PlanarScan. ``own_sectors_rad``: ((start, end), ...) in the laser
    frame, -pi <= start < end <= pi, both ends included (a sector across +-pi is given as two), measured in S7;
    ``own_mask_source`` says which measurement and load state they are."""
    _frame_id(frame_id)
    a0 = _finite_scalar(angle_min_rad, "angle_min_rad")
    inc = _finite_scalar(angle_increment_rad, "angle_increment_rad")
    lo = _finite_scalar(range_min_m, "range_min_m")
    hi = _finite_scalar(range_max_m, "range_max_m")
    dt = _finite_scalar(time_increment_s, "time_increment_s")
    period = _finite_scalar(scan_time_s, "scan_time_s")
    stamp = _finite_scalar(stamp_s, "stamp_s")
    if stamp_domain not in STAMP_DOMAINS:
        raise ValueError(f"stamp_domain must be one of {STAMP_DOMAINS}")
    if own_sectors_rad and not (isinstance(own_mask_source, str) and own_mask_source):
        raise ValueError("own sectors need their own_mask_source")
    if inc == 0 or lo < 0 or hi <= lo or dt < 0 or period < 0:
        raise ValueError("a scan needs a nonzero increment, 0 <= range_min < range_max and non-negative times")
    r = _real_array(ranges, "ranges")
    if r.ndim != 1 or len(r) == 0:
        raise ValueError("ranges must be one-dimensional and non-empty")
    angles = a0 + np.arange(len(r)) * inc
    status = np.full(len(r), BeamStatus.UNKNOWN, dtype=np.int8)
    with np.errstate(invalid="ignore"):
        hit = np.isfinite(r) & (r >= lo) & (r <= hi)
    status[hit] = BeamStatus.HIT
    status[np.isposinf(r)] = BeamStatus.NO_RETURN
    status[np.isneginf(r)] = BeamStatus.TOO_CLOSE
    wrapped = _wrap(angles)
    for sector in own_sectors_rad:
        if len(sector) != 2:
            raise ValueError("an own sector is (start, end)")
        s0, s1 = (_finite_scalar(v, "own sector") for v in sector)
        if not (-math.pi <= s0 < s1 <= math.pi):
            raise ValueError("an own sector needs -pi <= start < end <= pi")
        status[(wrapped >= s0) & (wrapped <= s1)] = BeamStatus.OWN
    ranges_m = np.where(status == BeamStatus.HIT, r, np.nan)
    if isinstance(received_host_ns, bool) or not isinstance(received_host_ns, int):
        raise ValueError("received_host_ns must be an integer")
    return PlanarScan(frame_id, angles, ranges_m, status, lo, hi, stamp, stamp_domain, dt, period, received_host_ns,
                      own_mask_source)


@dataclass(frozen=True)
class RgbdFrame:
    rgb: np.ndarray  # (H, W, 3) uint8
    depth_m: np.ndarray  # (H, W) float32, NaN unknown, registered to rgb's grid
    intrinsics: PinholeIntrinsics  # of that grid
    color_stamp_s: float
    depth_stamp_s: float
    stamp_domain: str  # the clock both stamps are in
    received_host_ns: int
    depth_scale_m_per_unit: float  # from the stream, never inferred
    calibration_source: str


def rgbd_frame_from_aligned(*, color: ArrayLike, depth_raw: ArrayLike, depth_scale_m_per_unit: float,
                            intrinsics: PinholeIntrinsics, aligned_to_frame: str, color_stamp_s: float,
                            depth_stamp_s: float, stamp_domain: str, received_host_ns: int, max_pair_skew_s: float,
                            calibration_source: str) -> RgbdFrame:
    """A colour image and depth already registered to it (realsense-ros aligned_depth_to_color) -> RgbdFrame.

    The depth must name the grid it is registered to (``aligned_to_frame`` equal to the intrinsics' frame): equal
    sizes alone do not prove registration. The scale comes from the stream; raw depth is integer units; 0 is
    unknown, and so is a negative raw value. The two stamps are in one ``stamp_domain`` (convert before pairing)
    and must lie within ``max_pair_skew_s``. The arrays are copied (the frame owns its buffers)."""
    if stamp_domain not in STAMP_DOMAINS:
        raise ValueError(f"stamp_domain must be one of {STAMP_DOMAINS}")
    if aligned_to_frame != intrinsics.frame_id:
        raise ValueError("the depth is not registered to the intrinsics' grid")
    if not isinstance(calibration_source, str) or not calibration_source:
        raise ValueError("calibration_source must say where the intrinsics came from")
    scale = _finite_scalar(depth_scale_m_per_unit, "depth_scale_m_per_unit")
    skew = _finite_scalar(max_pair_skew_s, "max_pair_skew_s")
    cs, ds = _finite_scalar(color_stamp_s, "color_stamp_s"), _finite_scalar(depth_stamp_s, "depth_stamp_s")
    if scale <= 0 or skew < 0:
        raise ValueError("the depth scale must be positive and the pair skew non-negative")
    if abs(cs - ds) > skew:
        raise ValueError(f"colour and depth are {abs(cs - ds):.4f} s apart, over {skew} s")
    rgb = np.array(color, copy=True)
    raw = np.asarray(depth_raw)
    shape = (intrinsics.height, intrinsics.width)
    if rgb.dtype != np.uint8 or rgb.shape != (*shape, 3):
        raise ValueError(f"colour must be uint8 {(*shape, 3)}")
    if raw.dtype.kind not in "ui" or raw.shape != shape:
        raise ValueError(f"raw depth must be integer units of shape {shape}")
    unknown = raw <= 0
    with np.errstate(over="ignore"):
        depth = raw.astype(np.float32) * np.float32(scale)
    depth[unknown] = np.nan
    if not np.isfinite(depth[~unknown]).all():
        raise ValueError("the depth scale overflows float32 metres")
    if isinstance(received_host_ns, bool) or not isinstance(received_host_ns, int):
        raise ValueError("received_host_ns must be an integer")
    return RgbdFrame(rgb, depth, intrinsics, cs, ds, stamp_domain, received_host_ns, scale, calibration_source)


__all__ = ["BeamStatus", "PlanarScan", "RgbdFrame", "planar_scan_from_laserscan", "rgbd_frame_from_aligned"]
