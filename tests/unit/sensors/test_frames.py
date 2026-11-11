"""Sim-to-real 4c-1: the sensor input contracts and the real truck's pure conversions
(docs/plans/2026-11-11-sim-to-real-bringup.md 2.3a/2.3b; docs/design/2026-11-11-p04a-kinematics-and-command.md 6)."""

import math

import numpy as np
import pytest

from forklift_core.sensors.frames import (
    BeamStatus,
    planar_scan_from_laserscan,
    rgbd_frame_from_aligned,
)
from forklift_core.sensors.rgbd import PinholeIntrinsics

INF = float("inf")


def scan(ranges, **kw):
    args = dict(angle_min_rad=-math.pi, angle_increment_rad=2 * math.pi / len(ranges), range_min_m=0.15,
                range_max_m=12.0, time_increment_s=1e-4, scan_time_s=0.1, stamp_s=100.0, stamp_domain="ros",
                frame_id="laser",
                received_host_ns=5_000_000)
    args.update(kw)
    return planar_scan_from_laserscan(ranges, **args)


def test_every_beam_keeps_its_status_unknown_is_never_free():
    """REP-117: +inf no return within range, -inf too close, NaN invalid. Only a hit has a range; an unknown beam
    is not a no-return (an obstacle layer must not clear on it, Codex 4c P2-3)."""
    s = scan([1.0, INF, -INF, float("nan"), 0.05, 20.0, 3.0, 12.0])
    assert list(s.status) == [BeamStatus.HIT, BeamStatus.NO_RETURN, BeamStatus.TOO_CLOSE, BeamStatus.UNKNOWN,
                              BeamStatus.UNKNOWN, BeamStatus.UNKNOWN, BeamStatus.HIT, BeamStatus.HIT]
    assert s.ranges_m[0] == 1.0 and s.ranges_m[7] == 12.0
    assert all(math.isnan(r) for r in s.ranges_m[1:6])


def test_beam_angles_and_timing_are_kept():
    s = scan([1.0] * 4)
    assert s.angles_rad == pytest.approx([-math.pi, -math.pi / 2, 0.0, math.pi / 2])
    assert (s.stamp_s, s.time_increment_s, s.scan_time_s, s.received_host_ns) == (100.0, 1e-4, 0.1, 5_000_000)
    assert s.frame_id == "laser" and s.deskewed is False
    assert s.beam_times_s() == pytest.approx([100.0, 100.0001, 100.0002, 100.0003])


def test_own_sectors_mark_the_body_whatever_the_reading():
    s = scan([1.0, 0.3, INF, 2.0], own_sectors_rad=((-1.7, -1.4), (3.0, math.pi)), own_mask_source="S7 empty forks")
    assert s.own_mask_source == "S7 empty forks"
    assert s.status[1] == BeamStatus.OWN  # -pi/2 is inside (-1.7, -1.4)
    assert s.status[0] == BeamStatus.HIT and s.status[2] == BeamStatus.NO_RETURN
    assert math.isnan(s.ranges_m[1])


def test_hit_points_are_the_hits_only():
    s = scan([1.0, INF, 2.0, float("nan")])
    pts = s.hit_points_xy()
    assert pts.shape == (2, 2)
    assert pts[0] == pytest.approx([-1.0, 0.0], abs=1e-12) and pts[1] == pytest.approx([2.0, 0.0], abs=1e-12)


@pytest.mark.parametrize("bad", [
    dict(angle_increment_rad=0.0), dict(range_min_m=-0.1), dict(range_max_m=0.1), dict(frame_id=""),
    dict(own_sectors_rad=((0.5, 0.2),), own_mask_source="m"), dict(own_sectors_rad=((0.0, 4.0),), own_mask_source="m"),
    dict(time_increment_s=-1.0), dict(stamp_domain="wall"), dict(own_sectors_rad=((0.0, 0.1),)),
])
def test_malformed_scans_are_refused(bad):
    with pytest.raises(ValueError):
        scan([1.0, 2.0], **bad)


K = PinholeIntrinsics(4, 3, 600.0, 600.0, 2.0, 1.5, "camera_color_optical_frame")


def frame(**kw):
    args = dict(color=np.zeros((3, 4, 3), np.uint8), depth_raw=np.full((3, 4), 1000, np.uint16),
                depth_scale_m_per_unit=0.001, intrinsics=K, aligned_to_frame="camera_color_optical_frame",
                color_stamp_s=10.000, depth_stamp_s=10.010, stamp_domain="ros", received_host_ns=7,
                max_pair_skew_s=0.02,
                calibration_source="device_factory")
    args.update(kw)
    return rgbd_frame_from_aligned(**args)


def test_an_aligned_pair_becomes_metres_with_zero_as_unknown():
    raw = np.full((3, 4), 1500, np.uint16)
    raw[0, 0] = 0
    f = frame(depth_raw=raw)
    assert f.depth_m.dtype == np.float32 and f.depth_m[1, 1] == pytest.approx(1.5)
    assert math.isnan(f.depth_m[0, 0])
    assert (f.color_stamp_s, f.depth_stamp_s, f.received_host_ns) == (10.0, 10.01, 7)
    assert f.depth_scale_m_per_unit == 0.001 and f.calibration_source == "device_factory"


@pytest.mark.parametrize("bad", [
    dict(depth_stamp_s=10.05),  # paired too far apart
    dict(aligned_to_frame="camera_depth_optical_frame"),  # not registered to the intrinsics' grid
    dict(depth_raw=np.zeros((4, 4), np.uint16)),  # shape off the calibration
    dict(color=np.zeros((3, 4), np.uint8)),
    dict(depth_scale_m_per_unit=0.0),
    dict(depth_raw=np.zeros((3, 4), np.float32)),  # the scale is the stream's, raw units are integers
    dict(calibration_source=""),
    dict(stamp_domain="host"),
    dict(depth_scale_m_per_unit=1e100),  # overflows float32 metres
])
def test_bad_pairs_are_refused(bad):
    with pytest.raises(ValueError):
        frame(**bad)


def test_negative_raw_depth_is_unknown_and_the_frame_owns_its_buffers():
    raw = np.full((3, 4), 1000, np.int16)
    raw[1, 2] = -1000
    color = np.zeros((3, 4, 3), np.uint8)
    f = frame(depth_raw=raw, color=color)
    assert math.isnan(f.depth_m[1, 2]) and f.depth_m[0, 0] == pytest.approx(1.0)
    color[0, 0, 0] = 255
    assert f.rgb[0, 0, 0] == 0 and f.stamp_domain == "ros"
