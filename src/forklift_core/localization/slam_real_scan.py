"""The real SLAM path's scan rules (docs/design/2026-11-25-real-sources-pr7.md 6.1, 0-1), shared by the executor's
``RealSlamSource`` and the observe run's ``--slam`` (7c-1c) -- one copy, never two.

- ``SlamScanIn``: an accepted scan as the SLAM path sees it: the first-beam estimate (host monotonic ns, the order
  time) and, when the timing is measured, the first-beam interval ``[Flo, Fhi]``; without a measured interval the scan
  is diagnostic (``measured`` False, ``Flo = Fhi`` = the estimate).
- ``from_feed``: a ``scan_feed.RealScan`` with ``lidar.yaml``'s timing (the 7a rule of ``to_layer_scan``: the SDK's
  first sample +- the measured bound; before 7a-3 there is no bound, so every real scan is diagnostic). ``from_layer``:
  a layer scan (the HIL's synthetic LiDAR, whose cast instant is exact).
- ``pick_odom``: the odometry over the interval -- the whole of ``[Flo, Fhi]`` covered by a continuous, included
  history (an entry ending at or before ``Flo`` and one starting at or after ``Fhi``, every entry between continuous
  and included), the pose interpolated at the estimate between entry interval midpoints (the loop's ``place_scan``
  way); the SLAM path does not apply the layer's motion bounds.
- ``to_bridge_scan``: the frame's scan: invalid beams, self-hit beams and anything outside ``[range_min, range_max]``
  sent as NaN (slam_toolbox 2.8.5's matcher treats only NaN/Inf as no return: a finite range past ``range_max`` may
  enter the match as a false point -- round 2 P2-7); the height band is the layer's rule, not SLAM's.
- ``base_from_laser``: the bridge's static TF from ``lidar.yaml``'s mount, which is in the rear-axle frame:
  ``base_from_laser = rear_from_base^-1 o rear_from_laser`` (round 2 P2-6).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from forklift_core.localization import slam_link
from forklift_core.localization.slam_pose import compose, invert

ANGLE_TOL_RAD = 1e-4  # beam angles must be evenly spaced (slam_link's scan carries angle_min + increment)


@dataclass(frozen=True)
class SlamScanIn:
    key: tuple  # the scan's identity on its feed (feed_gen, producer_gen, scan_id) -- for the record
    stamp_ns: int  # the first-beam estimate, host monotonic (the order time and the ROS stamp)
    flo_ns: int  # the first-beam interval; = stamp_ns when not measured
    fhi_ns: int
    measured: bool
    angles: np.ndarray  # radians, evenly spaced
    ranges: np.ndarray  # metres, NaN where invalid
    self_hit: np.ndarray  # beams the truck's own body hides
    range_min: float
    range_max: float


def from_feed(scan, cfg) -> tuple[SlamScanIn | None, str | None]:
    """A ``scan_feed.RealScan`` -> ``(SlamScanIn, None)`` or ``(None, why)``: ``no_acquisition_time`` without the SDK's
    first sample; with no measured bound (``first_beam_err_s`` null or ``timing.measured`` false) or no measured mount
    the scan is diagnostic (design 6.1, 0-1)."""
    from forklift_core.sensors.lidar_cfg import self_hit_mask

    first = scan.meta.get("first_beam_mono_ns")
    if not isinstance(first, int) or isinstance(first, bool):
        return None, "no_acquisition_time"
    measured = cfg.first_beam_err_s is not None and cfg.timing_measured and cfg.mount_measured
    err = int(round(cfg.first_beam_err_s * 1e9)) if measured else 0
    return SlamScanIn(
        key=(int(scan.feed_gen), scan.producer_gen, int(scan.scan_id)), stamp_ns=int(first), flo_ns=int(first) - err,
        fhi_ns=int(first) + err, measured=bool(measured), angles=np.asarray(scan.angles, dtype=float),
        ranges=np.asarray(scan.ranges, dtype=float), self_hit=self_hit_mask(scan.angles, cfg.mask_rad),
        range_min=float(scan.meta.get("range_min", 0.0)), range_max=float(scan.meta.get("range_max", math.inf))), None


def from_layer(scan, start_ns: int, *, key=None, range_min: float = 0.0, range_max: float = 30.0) -> SlamScanIn:
    """A layer ``real_layer.RealScan`` whose first beam is exact (the HIL's synthetic cast, ``first_s == last_s``): its
    first-beam instant is the estimate and the interval. A valid no-return (+inf) stays a no-return (NaN to SLAM)."""
    ns = int(start_ns) + int(round(float(scan.first_s) * 1e9))
    ranges = np.where(np.asarray(scan.valid, bool), np.asarray(scan.ranges_m, dtype=float), np.nan)
    return SlamScanIn(key=key if key is not None else ("layer", ns), stamp_ns=ns, flo_ns=ns, fhi_ns=ns, measured=True,
                      angles=np.asarray(scan.angles_rad, dtype=float), ranges=ranges,
                      self_hit=np.asarray(scan.self_hit, bool), range_min=float(range_min),
                      range_max=float(range_max))


def pick_odom(history, lo_s: float, hi_s: float, t_s: float):
    """``(rear-axle pose, None)`` or ``(None, why)`` from the loop's odometry history entries ``(lo_s, hi_s, pose,
    continuous, included, speed_ok, yaw_ok)``: ``not_yet`` (no entry starting at or after ``hi_s`` yet -- the caller
    tries once more a tick later), ``no_cover`` (no entry ending at or before ``lo_s``), ``broken`` (an entry in the
    span not continuous or not included)."""
    h = list(history)
    before = [k for k, e in enumerate(h) if e[1] <= lo_s]
    after = [k for k, e in enumerate(h) if e[0] >= hi_s]
    if not after:
        return None, "not_yet"
    if not before:
        return None, "no_cover"
    k0, k1 = before[-1], after[0]
    span = h[k0:k1 + 1]
    if not all(e[3] and e[4] for e in span):
        return None, "broken"
    mids = [((e[0] + e[1]) / 2.0, e[2]) for e in span]
    for (ta, pa), (tb, pb) in zip(mids, mids[1:]):
        if ta <= t_s <= tb:
            a = 0.0 if tb <= ta else (t_s - ta) / (tb - ta)
            dyaw = math.atan2(math.sin(pb[2] - pa[2]), math.cos(pb[2] - pa[2]))
            return (pa[0] + a * (pb[0] - pa[0]), pa[1] + a * (pb[1] - pa[1]), pa[2] + a * dyaw), None
    return (tuple(mids[0][1]) if t_s < mids[0][0] else tuple(mids[-1][1])), None


def base_from_laser(mount_rear, rear_offset_m: float) -> tuple:
    """``(x, y, z, yaw)`` of the laser in base_link from the mount in the rear-axle frame (``lidar.yaml``: x, y, z,
    yaw) and the base_link -> rear axle offset (the rear axle at ``(-rear_offset_m, 0)`` in base_link, as
    ``start_reloc.base_from_rear``)."""
    x, y, z, yaw = (float(v) for v in mount_rear)
    rear_from_base = (float(rear_offset_m), 0.0, 0.0)  # base_link at +offset ahead of the rear axle
    bx, by, byaw = compose(invert(rear_from_base), (x, y, yaw))
    return (float(bx), float(by), z, float(byaw))


def to_bridge_scan(s: SlamScanIn, scan_id: int, odom_from_base) -> tuple[slam_link.Scan | None, str | None]:
    """The bridge frame's scan (``stamp_s = stamp_ns / 1e9``) or ``(None, 'uneven_angles')``."""
    a = np.asarray(s.angles, dtype=float)
    if a.size < 2:
        return None, "uneven_angles"
    inc = (a[-1] - a[0]) / (a.size - 1)
    if not np.all(np.abs(np.diff(a) - inc) <= ANGLE_TOL_RAD):
        return None, "uneven_angles"
    r = np.asarray(s.ranges, dtype=float)
    bad = ~np.isfinite(r) | np.asarray(s.self_hit, bool) | (r < s.range_min) | (r > s.range_max)
    out = np.where(bad, np.nan, r)
    return slam_link.Scan(int(scan_id), int(s.stamp_ns) / 1e9, tuple(float(v) for v in odom_from_base), out,
                          float(a[0]), float(inc), float(s.range_min), float(s.range_max)), None


__all__ = ["SlamScanIn", "base_from_laser", "from_feed", "from_layer", "pick_odom", "to_bridge_scan"]
