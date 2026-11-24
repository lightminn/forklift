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


def measured_cfg(err_s=0.005):
    return lc.LidarConfig(mount=(0.1, 0.0, 0.3, 0.0), mount_measured=True, mask_rad=(), limit_m=5.7,
                          scan_period_s=0.1, first_beam_err_s=err_s, timing_measured=True)


def feed_settled(kind, n=8):
    feed = sf.ScanFeed()
    for k in range(sf.ESTABLISH_COUNT):
        feed.judge(frame(k, n=n, stamp_kind=kind, time_increment=0.09 / (n - 1)), 0)
    return feed


@pytest.mark.parametrize("buffered_ms", [0, 120, 300])
def test_a_buffered_scan_is_never_called_fresh(buffered_ms):
    """The first sample at t0; the rotation ends 90 ms later; the scan then waited ``buffered_ms`` in the driver's
    buffer and was received 10 ms after the fetch.
    The upstream topic's stamp is the fetch (t0 + buffered): diagnostic only. The patched topic's stamp is t0: the
    layer scan's first beam is t0 - err, so its age at receipt includes the wait."""
    k = sf.ESTABLISH_COUNT
    t0 = 20_000 * MS
    fetch = t0 + 90 * MS + buffered_ms * MS
    recv = fetch + 10 * MS
    cfg = measured_cfg()
    # upstream (call start)
    raw = feed_settled("call_start")
    s_raw, why = raw.judge(frame(k, recv=recv, stamp=fetch + OFF, stamp_kind="call_start",
                                 time_increment=0.09 / 7), 0)
    assert why is None and lc.to_layer_scan(s_raw, cfg, 0) == (None, "no_acquisition_time")
    # patched (first sample)
    sdk = feed_settled("sdk_first_sample")
    s_sdk, why = sdk.judge(frame(k, recv=recv, stamp=t0 + OFF, stamp_kind="sdk_first_sample",
                                 time_increment=0.09 / 7), 0)
    assert why is None and s_sdk.meta["first_beam_mono_ns"] == t0
    out, lwhy = lc.to_layer_scan(s_sdk, cfg, 0)
    assert lwhy is None
    age_at_receipt_s = recv / 1e9 - out.first_s
    assert age_at_receipt_s >= (90 + buffered_ms + 10) / 1e3  # the wait is in the age, never hidden


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
    assert "grabScanDataHqWithTimeStamp" in text and '"scan_sdk"' in text
    docker = (REPO / "deploy" / "ros2" / "Dockerfile.scan").read_text(encoding="utf-8")
    assert "0001-sdk-first-sample-stamp.patch" in docker and "34300099" in docker
    launcher = (REPO / "tools" / "ros2_dev.sh").read_text(encoding="utf-8")
    assert "scan_sdk:=/sensors/lidar/scan_raw_sdk" in launcher and "--source ${FORKLIFT_SCAN_SOURCE:-raw}" in launcher
