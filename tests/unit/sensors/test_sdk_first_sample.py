"""PR 7a-2 (docs/design/2026-11-25-real-sources-pr7.md 4): the patched sllidar node's scan_sdk topic, stamped at the
SDK's first sample, and the repository-side buffer-delay injection: a scan that waited in the driver's buffer is never
called fresh -- the call-start stamp (the upstream topic) gives no acquisition time at all, the first-sample stamp
gives its true age."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

from forklift_core.sensors import lidar_cfg as lc
from forklift_core.sensors import scan_feed as sf

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_scan_feed import MS, OFF, frame  # noqa: E402

REPO = Path(__file__).resolve().parents[3]


def measured_cfg(err_s=0.005, period=0.12):
    return lc.LidarConfig(mount=(0.1, 0.0, 0.3, 0.0), mount_measured=True, mask_rad=(), limit_m=5.7,
                          scan_period_s=period, first_beam_err_s=err_s, timing_measured=True)


def _adapter():
    sys.path.insert(0, str(REPO / "ros2" / "src" / "forklift_ros" / "forklift_ros"))
    import real_scan_adapter as ra

    return ra


def sdk_frame(k, *, first, span_s, recv, n=8):
    """A scan_sdk frame exactly as the adapter makes it from the patched node's message: stamp = first sample
    (monotonic), scan_time = last - first sample as a float32, frame_id with the format's name."""
    from types import SimpleNamespace

    ra = _adapter()
    span32 = float(np.float32(span_s))
    msg = SimpleNamespace(ranges=list(np.full(n, 2.0)), intensities=[], angle_min=-3.14, angle_increment=6.28 / n,
                          time_increment=float(np.float32(span_s / (n - 1))), scan_time=span32, range_min=0.05,
                          range_max=12.0,
                          header=SimpleNamespace(stamp=SimpleNamespace(sec=first // 10**9, nanosec=first % 10**9),
                                                 frame_id="laser" + ra.SDK_FORMAT))
    f = ra.sdk_fields(first, msg.scan_time, recv, OFF, frame_id=msg.header.frame_id)
    assert f is not None
    return ra.make_frame(msg, scan_id=k, recv_mono_ns=recv, sys_minus_mono_ns=OFF, gen=7,
                         stamp_kind="sdk_first_sample", sdk=f)


def settled_sdk(span_s=0.098):
    feed = sf.ScanFeed()
    for k in range(sf.ESTABLISH_COUNT):
        t = 10_000 * MS + k * 100 * MS
        feed.judge(sdk_frame(k, first=t, span_s=span_s, recv=t + int(span_s * 1e9) + 10 * MS), 0)
    return feed


@pytest.mark.parametrize("buffered_ms", [0, 120, 300])
def test_a_buffered_scan_is_never_called_fresh(buffered_ms):
    """The first sample at t0, the last 98 ms later (one rotation at the nominal 10 Hz); the scan then waited
    ``buffered_ms`` in the SDK's buffer and was received 10 ms after the read. The upstream topic's call-start stamp
    gives no acquisition time at all; the patched topic's first and last sample put the wait into the age."""
    k = sf.ESTABLISH_COUNT
    t0 = 20_000 * MS
    span = 0.098
    fetch = t0 + int(span * 1e9) + buffered_ms * MS
    recv = fetch + 10 * MS
    cfg = measured_cfg()
    raw = sf.ScanFeed()
    for j in range(sf.ESTABLISH_COUNT):
        raw.judge(frame(j, stamp_kind="call_start"), 0)
    s_raw, why = raw.judge(frame(k, recv=recv, stamp=fetch + OFF, stamp_kind="call_start"), 0)
    assert why is None and lc.to_layer_scan(s_raw, cfg, 0) == (None, "no_acquisition_time")
    sdk = settled_sdk()
    s_sdk, why = sdk.judge(sdk_frame(k, first=t0, span_s=span, recv=recv), 0)
    assert why is None
    out, lwhy = lc.to_layer_scan(s_sdk, cfg, 0)
    assert lwhy is None
    assert recv / 1e9 - out.first_s >= (98 + buffered_ms + 10) / 1e3  # the wait is in the age, never hidden
    assert out.last_s == pytest.approx((t0 + int(span * 1e9) + 5 * MS) / 1e9)  # the buffer's own last sample + err


def test_a_nominal_rotation_needs_a_budget_that_covers_it():
    """Codex 7a-2 P2-1: one 10 Hz rotation (98 ms) plus the error bound on both ends is wider than 100 ms -- the
    acceptance bound (lidar.yaml timing.scan_period_s, the real run layer's first-beam budget) must cover it."""
    t0 = 20_000 * MS
    s, why = settled_sdk().judge(sdk_frame(sf.ESTABLISH_COUNT, first=t0, span_s=0.098, recv=t0 + 110 * MS), 0)
    assert why is None
    assert lc.to_layer_scan(s, measured_cfg(period=0.10), 0)[1] == "scan_too_long"
    assert lc.to_layer_scan(s, measured_cfg(period=0.12), 0)[1] is None


@pytest.mark.parametrize("stamp,span,recv,ok", [
    (5 * 10**9, 0.098, 5 * 10**9 + 200 * MS, True),
    (5 * 10**9, 0.002, 5 * 10**9 + 10 * MS, True),     # (the frame_id case below is what refuses a remap)
    (1_700_000_000 * 10**9, 0.098, 5 * 10**9, False),  # an epoch stamp: the upstream topic remapped here
    (6 * 10**9, 0.098, 5 * 10**9, False),               # in the future
    (1 * 10**9, 0.098, 7 * 10**9, False),               # older than 5 s
    (5 * 10**9, 0.6, 6 * 10**9, False),                 # not one rotation
    (5 * 10**9, 0.098, 5 * 10**9 + 50 * MS, False),     # the last sample after the receipt
])
def test_the_adapter_takes_only_what_looks_like_a_scan_sdk_message(stamp, span, recv, ok):
    """Codex 7a-2 P2-3: a wrong topic or a remap gives no scan, never a call-start stamp taken for a first sample."""
    ra = _adapter()
    assert (ra.sdk_fields(stamp, span, recv, OFF, frame_id="laser" + ra.SDK_FORMAT) is not None) is ok


def test_a_laserscan_without_the_format_name_is_never_a_scan_sdk_message():
    """Codex 7a-2 round 2 P2-3: a call-start scan whose times happen to fit (20.000 s, 2 ms, received 20.010 s) is
    refused by its frame_id; and the host refuses acquisition times on a call_start frame."""
    ra = _adapter()
    assert ra.sdk_fields(20 * 10**9, 0.002, 20 * 10**9 + 10 * MS, OFF, frame_id="laser") is None
    assert ra.sdk_fields(20 * 10**9, 0.002, 20 * 10**9 + 10 * MS, OFF, frame_id="laser" + ra.SDK_FORMAT) is not None
    feed = sf.ScanFeed()
    assert feed.judge(frame(0, stamp_kind="call_start", first_beam_mono_ns=1, last_beam_mono_ns=2), 0)[1] == "bad_meta"


def test_the_last_sample_comes_back_exactly_through_float32():
    """Codex 7a-2 round 2 P3: 98 000 us survives the float32 scan_time."""
    ra = _adapter()
    f = ra.sdk_fields(10**9, float(np.float32(0.098)), 10**9 + 200 * MS, OFF, frame_id="laser" + ra.SDK_FORMAT)
    assert f["last_beam_mono_ns"] - f["first_beam_mono_ns"] == 98_000_000


def test_the_frame_id_is_the_sensor_frame_again():
    h, _r, _i = sf.decode_frame(sdk_frame(1, first=10**9, span_s=0.098, recv=10**9 + 200 * MS))
    assert h["frame_id"] == "laser"


def test_the_adapter_source_decides_topic_and_stamp_kind():
    sys.path.insert(0, str(REPO / "ros2" / "src" / "forklift_ros" / "forklift_ros"))
    import real_scan_adapter as ra
    from types import SimpleNamespace

    assert ra.SOURCES["raw"] == ("/sensors/lidar/scan_raw", "call_start")
    assert ra.SOURCES["sdk"] == ("/sensors/lidar/scan_raw_sdk", "sdk_first_sample")
    msg = SimpleNamespace(ranges=[1.0, 2.0], intensities=[], angle_min=-1.0, angle_increment=0.5,
                          time_increment=0.01, scan_time=0.1, range_min=0.1, range_max=10.0,
                          header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=2), frame_id="laser"))
    h, _r, _i = sf.decode_frame(ra.make_frame(msg, scan_id=1, recv_mono_ns=5, sys_minus_mono_ns=7,
                                              stamp_kind="sdk_first_sample"))
    assert h["stamp_kind"] == "sdk_first_sample"


def test_the_patch_is_applied_by_the_scan_image_and_launched_with_its_topic():
    patch = REPO / "deploy" / "ros2" / "sllidar_patches" / "0001-sdk-first-sample-stamp.patch"
    text = patch.read_text(encoding="utf-8")
    assert "grabScanDataHqWithTimeStamps" in text and '"scan_sdk"' in text and "_scan_last_timestamp_uS" in text
    assert "publish_delay_ms" in text and "@sdk_mono_v1" in text
    assert "this->now() - rclcpp::Duration" not in text  # no ROS-clock round trip
    docker = (REPO / "deploy" / "ros2" / "Dockerfile.scan").read_text(encoding="utf-8")
    assert "0001-sdk-first-sample-stamp.patch" in docker and "34300099" in docker
    launcher = (REPO / "tools" / "ros2_dev.sh").read_text(encoding="utf-8")
    assert "scan_sdk:=/sensors/lidar/scan_raw_sdk" in launcher and 'FORKLIFT_SCAN_SOURCE must be raw or sdk' in launcher


def test_the_teleop_bridge_drops_a_first_sample_stamped_scan():
    """Codex 7a-2 P2-4: the bridge places a scan at (stamp - period) -- a call-start stamp only."""
    src = (REPO / "tools" / "bridge_scan.py").read_text(encoding="utf-8")
    assert 'scan.meta.get("stamp_kind") not in (None, "call_start")' in src and '_drop("stamp_kind")' in src
