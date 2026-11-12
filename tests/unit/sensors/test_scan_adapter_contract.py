"""The ROS adapter's side of scan/1 (real-scan design 1.2) against the host's ScanFeed: what the adapter frames from a
LaserScan decodes to the same scan, and sending never blocks -- no receiver or a full receiver drops and counts. The
adapter module is loaded without ROS (its rclpy imports live inside main)."""

import importlib.util
import math
import os
import socket
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.sensors import scan_feed as sf

ADAPTER = Path(__file__).resolve().parents[3] / "ros2/src/forklift_ros/forklift_ros/real_scan_adapter.py"


def _adapter():
    spec = importlib.util.spec_from_file_location("real_scan_adapter_under_test", ADAPTER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def laser_scan(n=16, sec=1_700_000_000, nanosec=250_000_000, ranges=None, intensities=True):
    r = list(np.linspace(0.5, 3.0, n)) if ranges is None else list(ranges)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec), frame_id="laser"),
        angle_min=-math.pi, angle_max=math.pi, angle_increment=2 * math.pi / (n - 1), time_increment=0.1 / (n - 1),
        scan_time=0.1, range_min=0.05, range_max=12.0, ranges=r, intensities=[47.0] * n if intensities else [])


def test_the_adapter_does_not_import_ros_at_module_level():
    src = ADAPTER.read_text(encoding="utf-8")
    top = src.split("def main(", 1)[0]
    import re

    assert not re.search(r"^\s*(import|from)\s+(rclpy|sensor_msgs|builtin_interfaces)\b", top, re.M)
    _adapter()  # loads without rclpy installed


def test_an_adapter_frame_decodes_in_scan_feed_unchanged():
    ad = _adapter()
    msg = laser_scan(ranges=[0.0, math.inf, 0.04, 1.5] + [2.0] * 12)
    data = ad.make_frame(msg, scan_id=7, recv_mono_ns=5_000_000_000, sys_minus_mono_ns=1_699_999_990 * 10**9)
    head, ranges, inten = sf.decode_frame(data)
    assert head["scan_id"] == 7 and head["n"] == 16 and head["frame_id"] == "laser"
    assert head["ros_stamp_ns"] == 1_700_000_000_250_000_000
    assert head["angle_min"] == pytest.approx(-math.pi) and head["range_min"] == pytest.approx(0.05)
    assert math.isinf(ranges[1]) and ranges[3] == pytest.approx(1.5) and inten is not None and inten[0] == 47.0
    feed = sf.ScanFeed()
    for k in range(sf.ESTABLISH_COUNT):
        scan, why = feed.judge(ad.make_frame(laser_scan(sec=1_699_999_990 + 5 + k), scan_id=k,
                                             recv_mono_ns=(5 + k) * 10**9 + 280_000_000,
                                             sys_minus_mono_ns=1_699_999_990 * 10**9), 0)
    assert why is None and scan.stamp_to_recv_ns == 30_000_000  # stamped at +250 ms, received at +280 ms


def test_a_scan_without_intensities_carries_none():
    ad = _adapter()
    _h, _r, inten = sf.decode_frame(ad.make_frame(laser_scan(intensities=False), scan_id=1, recv_mono_ns=1,
                                                  sys_minus_mono_ns=0))
    assert inten is None


def test_sending_with_no_receiver_drops_without_blocking(tmp_path):
    ad = _adapter()
    sender = ad.DatagramSender(str(tmp_path / "nobody.sock"))
    t0 = time.monotonic()
    for k in range(50):
        assert sender.send(b"x" * 1000) is False
    assert time.monotonic() - t0 < 0.5 and sender.dropped == 50 and sender.sent == 0
    sender.close()


def test_a_full_receiver_drops_and_a_reader_gets_whole_frames(tmp_path):
    ad = _adapter()
    path = str(tmp_path / "scan.sock")
    rx = sf.bind_socket(path)
    try:
        sender = ad.DatagramSender(path)
        frame = ad.make_frame(laser_scan(n=1600), scan_id=1, recv_mono_ns=1, sys_minus_mono_ns=0)
        t0 = time.monotonic()
        for _ in range(2000):  # far more than any receive buffer holds
            sender.send(frame)
        assert time.monotonic() - t0 < 2.0
        assert sender.sent > 0 and sender.dropped > 0
        data = rx.recv(sf.RECV_BYTES)
        assert data == frame  # a datagram is whole or absent
        sender.close()
    finally:
        rx.close()


def test_bind_socket_replaces_a_stale_path_and_is_non_blocking(tmp_path):
    path = tmp_path / "scan.sock"
    first = sf.bind_socket(str(path))
    first.close()
    assert path.exists()  # a crash leaves the path behind
    rx = sf.bind_socket(str(path))
    try:
        with pytest.raises(BlockingIOError):
            rx.recv(16)
        assert os.stat(path).st_mode & 0o777 == 0o660
    finally:
        rx.close()
        assert socket  # (the module is used)
