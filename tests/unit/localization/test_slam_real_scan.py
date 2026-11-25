"""PR 7c-1b-2: the real SLAM path's scan rules (design 6.1, 0-1) -- odometry over the first-beam interval, the beams
sent as NaN, the mount in base_link and the diagnostic rule."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.localization import slam_real_scan as srs
from forklift_core.localization.slam_pose import compose
from forklift_core.localization.start_reloc import base_from_rear


def entry(lo, hi, pose, cont=True, inc=True):
    return (lo, hi, pose, cont, inc, True, True)


H = [entry(0.00, 0.01, (0.0, 0.0, 0.0)), entry(0.02, 0.03, (0.1, 0.0, 0.0)), entry(0.04, 0.05, (0.2, 0.0, 0.2))]


def test_pick_odom_interpolates_at_the_estimate_over_a_covered_interval():
    pose, why = srs.pick_odom(H, 0.012, 0.039, 0.025)
    assert why is None and pose == pytest.approx((0.1, 0.0, 0.0))
    pose, _ = srs.pick_odom(H, 0.012, 0.039, 0.035)
    assert pose == pytest.approx((0.15, 0.0, 0.1))


def test_pick_odom_needs_both_ends_and_an_unbroken_span():
    assert srs.pick_odom(H, 0.012, 0.045, 0.02) == (None, "not_yet")  # nothing starts at or after Fhi yet
    assert srs.pick_odom(H, -0.01, 0.015, 0.0) == (None, "no_cover")
    broken = [H[0], entry(0.02, 0.03, (0.1, 0.0, 0.0), cont=False), H[2]]
    assert srs.pick_odom(broken, 0.012, 0.039, 0.025) == (None, "broken")
    excluded = [H[0], entry(0.02, 0.03, (0.1, 0.0, 0.0), inc=False), H[2]]
    assert srs.pick_odom(excluded, 0.012, 0.039, 0.025) == (None, "broken")


def test_the_layer_and_slam_place_the_laser_at_the_same_pose():
    """Design 6.1, 1: a non-zero offset and yaw -- the laser in the odometry frame is the same whether reached from
    the rear axle (the layer's mount) or from base_link (the bridge's static TF)."""
    mount, offset = (0.42, -0.07, 0.3, math.radians(12.0)), 0.55
    rear = (1.0, 2.0, 0.7)
    via_rear = compose(rear, (mount[0], mount[1], mount[3]))
    bx, by, _z, byaw = srs.base_from_laser(mount, offset)
    via_base = compose(base_from_rear(rear, offset), (bx, by, byaw))
    assert via_base == pytest.approx(via_rear, abs=1e-12)


def scan_in(ranges, *, mask=None, angles=None, measured=True):
    n = len(ranges)
    a = np.linspace(-math.pi, math.pi, n, endpoint=False) if angles is None else np.asarray(angles)
    return srs.SlamScanIn(key=(0, None, 1), gen=(0, None), stamp_ns=5_000_000_000, flo_ns=4_999_000_000, fhi_ns=5_001_000_000,
                          measured=measured, angles=a, ranges=np.asarray(ranges, float),
                          self_hit=np.zeros(n, bool) if mask is None else np.asarray(mask, bool), range_min=0.15,
                          range_max=12.0)


def test_invalid_masked_and_out_of_range_beams_go_as_nan():
    s = scan_in([1.0, np.nan, 2.0, 0.05, 20.0, 3.0], mask=[False, False, True, False, False, False])
    scan, why = srs.to_bridge_scan(s, 7, (0.0, 0.0, 0.0))
    assert why is None and scan.scan_id == 7 and scan.stamp_s == 5.0
    r = scan.ranges_m
    assert r[0] == pytest.approx(1.0) and r[5] == pytest.approx(3.0)
    assert all(np.isnan(r[k]) for k in (1, 2, 3, 4))
    assert scan.range_max_m == 12.0 and scan.angle_increment_rad == pytest.approx(2 * math.pi / 6)


def test_uneven_angles_are_refused():
    s = scan_in([1.0, 1.0, 1.0], angles=[0.0, 0.1, 0.25])
    assert srs.to_bridge_scan(s, 1, (0.0, 0.0, 0.0)) == (None, "uneven_angles")


def cfg(err=None, timing=False, mount=False):
    return SimpleNamespace(first_beam_err_s=err, timing_measured=timing, mount_measured=mount, mask_rad=())


def feed_scan(first=True):
    meta = {"range_min": 0.15, "range_max": 12.0}
    if first:
        meta["first_beam_mono_ns"] = 9_000_000_000
    return SimpleNamespace(meta=meta, feed_gen=2, producer_gen=11, scan_id=40, angles=np.array([0.0, 0.1]),
                           ranges=np.array([1.0, np.nan]))


def test_from_feed_is_diagnostic_until_the_bound_and_the_mount_are_measured():
    assert srs.from_feed(feed_scan(first=False), cfg()) == (None, "no_acquisition_time")
    s, _ = srs.from_feed(feed_scan(), cfg())
    assert not s.measured and s.flo_ns == s.fhi_ns == s.stamp_ns == 9_000_000_000 and s.key == (2, 11, 40)
    s, _ = srs.from_feed(feed_scan(), cfg(0.002, True, False))
    assert not s.measured  # the mount unmeasured keeps it diagnostic
    s, _ = srs.from_feed(feed_scan(), cfg(0.002, True, True))
    assert s.measured and (s.flo_ns, s.fhi_ns) == (8_998_000_000, 9_002_000_000)


def test_pick_odom_with_entries_sharing_their_bounds():
    """Codex 7c-1b-2a round 1 P2-8: a point scan on STATEs read together (same interval)."""
    h = [entry(0.00, 0.01, (0.0, 0.0, 0.0)), entry(0.02, 0.02, (0.1, 0.0, 0.0)), entry(0.02, 0.02, (0.2, 0.0, 0.0)),
         entry(0.04, 0.05, (0.3, 0.0, 0.0))]
    pose, why = srs.pick_odom(h, 0.02, 0.02, 0.02)
    assert why is None and pose[0] in (0.1, 0.2)
