"""The real A2M12's one source for mount, self mask and timing bounds, and its scan -> layer conversion (PR 7 design
4, 7a-1; config/real/lidar.yaml).

``to_layer_scan`` turns a ``scan_feed.RealScan`` into the obstacle layer's ``real_layer.RealScan`` only when the
scan's acquisition can be bounded: the SDK's first-sample time (on the host monotonic clock, carried by the adapter
as ``first_beam_mono_ns``) with a measured error bound. Without both the scan stays diagnostic -- the driver stamp is
taken before a buffered scan is fetched and is not an acquisition time (2026-11-12 design 1.1). With them:

- first beam ``[Flo, Fhi]`` and last beam ``[Elo, Ehi]`` (the beam spacing ``time_increment``);
- the layer gets ``first_s = Flo`` (pose and FREE age, conservative), ``last_s = Ehi`` (the odometry history must
  reach it) and ``deadline_last_s = Elo`` (the layer deadline ``+ B``);
- a scan with ``Ehi - Flo`` over the nominal period is refused for the layer: the stop model counts
  ``scan_period_s + B`` from the first beam (Codex PR 7 design round 4 P2-1);
- an invalid range (NaN: the driver's zero, out of range) is never free (``valid`` False) -- the HIL rule that a
  no-return is a valid far reading does not apply here (Codex round 1 P2-6).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

__all__ = ["LidarConfig", "load_lidar_config", "self_hit_mask", "band_limit_m", "to_layer_scan", "LIDAR_YAML"]

LIDAR_YAML = Path(__file__).resolve().parents[3] / "config" / "real" / "lidar.yaml"


@dataclass(frozen=True)
class LidarConfig:
    mount: tuple[float, float, float, float]  # x, y, z (m, rear-axle frame), yaw (rad)
    mount_measured: bool
    mask_rad: tuple[tuple[float, float], ...]  # sensor-frame intervals the truck's body hides
    limit_m: float  # per-beam height-band range limit (band_half_m / sin(max_tilt))
    scan_period_s: float
    first_beam_err_s: float | None  # None: not measured yet -> every scan diagnostic
    timing_measured: bool


def band_limit_m(max_tilt_deg: float, band_half_m: float) -> float:
    """The range within which a beam tilted at most ``max_tilt_deg`` stays inside +-``band_half_m`` of the mount
    height (the teleop bridge's rule)."""
    tilt = math.radians(float(max_tilt_deg))
    return float(band_half_m) / math.sin(tilt) if tilt > 0 else math.inf


def self_hit_mask(angles: np.ndarray, mask_rad) -> np.ndarray:
    """Beams whose (wrapped) angle falls in any masked interval -- the truck's own returns."""
    a = np.asarray(angles, dtype=float)
    hit = np.zeros(a.shape, dtype=bool)
    wrapped = np.arctan2(np.sin(a), np.cos(a))
    for lo, hi in mask_rad:
        hit |= (wrapped >= lo) & (wrapped <= hi)
    return hit


def _finite(v, name):
    f = float(v)
    if not math.isfinite(f):
        raise ValueError(f"lidar config: {name} must be finite")
    return f


def load_lidar_config(path: str | Path = LIDAR_YAML) -> LidarConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    m = raw["mount"]
    xyz = [_finite(v, "mount.xyz") for v in m["xyz"]]
    if len(xyz) != 3:
        raise ValueError("lidar config: mount.xyz takes three numbers")
    mask = []
    for pair in raw.get("self_mask_deg") or []:
        a, b = (_finite(v, "self_mask_deg") for v in pair)
        if not -180.0 <= a <= b <= 180.0:
            raise ValueError("lidar config: self_mask_deg intervals are [a, b] with -180 <= a <= b <= 180")
        mask.append((math.radians(a), math.radians(b)))
    band = raw["height_band"]
    t = raw["timing"]
    err = t.get("first_beam_err_s")
    if err is not None:
        err = _finite(err, "timing.first_beam_err_s")
        if err < 0:
            raise ValueError("lidar config: timing.first_beam_err_s must be >= 0")
    period = _finite(t["scan_period_s"], "timing.scan_period_s")
    if period <= 0:
        raise ValueError("lidar config: timing.scan_period_s must be > 0")
    return LidarConfig(
        mount=(xyz[0], xyz[1], xyz[2], math.radians(_finite(m["yaw_deg"], "mount.yaw_deg"))),
        mount_measured=bool(m.get("measured", False)),
        mask_rad=tuple(mask),
        limit_m=band_limit_m(_finite(band["max_tilt_deg"], "max_tilt_deg"), _finite(band["band_half_m"], "band_half_m")),
        scan_period_s=period,
        first_beam_err_s=err,
        timing_measured=bool(t.get("measured", False)),
    )


def to_layer_scan(scan, cfg: LidarConfig, start_ns: int, *, sensor: str = "lidar"):
    """``(real_layer.RealScan, None)`` or ``(None, why)`` -- why one of ``no_acquisition_time``, ``no_time_bound``
    (no error bound, or one not measured), ``no_beam_timing``, ``scan_too_long``, ``after_receipt`` (the last beam's
    earliest time after the adapter received the scan: impossible, the time is wrong). Core seconds are
    ``(ns - start_ns) / 1e9``."""
    from forklift_core.live.real_layer import RealScan as LayerScan

    first = scan.meta.get("first_beam_mono_ns")
    if not isinstance(first, int) or isinstance(first, bool):
        return None, "no_acquisition_time"
    if cfg.first_beam_err_s is None or not cfg.timing_measured:
        return None, "no_time_bound"  # an unmeasured bound keeps the scan diagnostic (Codex 7a-1 P2-3)
    n = len(scan.ranges)
    step = float(scan.meta.get("time_increment") or 0.0)
    if n < 2 or not (math.isfinite(step) and step > 0):
        return None, "no_beam_timing"
    err_ns = int(round(cfg.first_beam_err_s * 1e9))
    end = first + int(round((n - 1) * step * 1e9))
    flo, elo, ehi = first - err_ns, end - err_ns, end + err_ns
    if (ehi - flo) > cfg.scan_period_s * 1e9:
        return None, "scan_too_long"
    if elo > int(scan.recv_mono_ns):
        return None, "after_receipt"
    ranges = np.asarray(scan.ranges, dtype=float)
    valid = np.isfinite(ranges)
    out = LayerScan(sensor=sensor, first_s=(flo - start_ns) / 1e9, last_s=(ehi - start_ns) / 1e9,
                    angles_rad=np.asarray(scan.angles, dtype=float), ranges_m=np.where(valid, ranges, np.inf),
                    valid=valid, self_hit=self_hit_mask(scan.angles, cfg.mask_rad),
                    limit_m=np.full(n, cfg.limit_m), deadline_last_s=(elo - start_ns) / 1e9,
                    origin="synthetic" if scan.meta.get("synthetic") else "real")
    return out, None
