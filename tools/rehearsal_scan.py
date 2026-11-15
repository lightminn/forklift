#!/usr/bin/env python3
"""The arrival-day rehearsal's synthetic scan/1 sender (docs/design/2026-11-15-arrival-day-rehearsal.md 2.6): frames
like the ROS adapter's (ros2/src/forklift_ros/forklift_ros/real_scan_adapter.py) for an empty room -- walls at a
fixed distance, A2M12-like beam count and rate -- sent to the socket ``tools/scan_check.py`` (ScanFeed) binds. The
head carries ``"synthetic": true``. It exercises the 7a seam only (adapter frame -> ScanFeed -> scan_check); it is
never connected to the teleop bridge's floor permission (7b, after the A2M12 is measured).

usage: rehearsal_scan.py --socket /tmp/forklift-scan/scan.sock [--seconds 10] [--hz 10] [--beams 1600]
                         [--wall-m 3.0]
"""

from __future__ import annotations

import argparse
import math
import socket
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from forklift_core.sensors.scan_feed import encode_frame  # noqa: E402


def room_ranges(beams: int, wall_m: float) -> np.ndarray:
    """A square room centred on the sensor: the range to the walls at half-width ``wall_m`` along each beam."""
    angles = -math.pi + 2 * math.pi * np.arange(beams) / beams
    c, s = np.abs(np.cos(angles)), np.abs(np.sin(angles))
    return wall_m / np.maximum(np.maximum(c, s), 1e-9)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--socket", required=True)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--beams", type=int, default=1600)
    ap.add_argument("--wall-m", type=float, default=3.0)
    args = ap.parse_args(argv)
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    ranges = room_ranges(args.beams, args.wall_m).astype(np.float32)
    period = 1.0 / args.hz
    sys_minus_mono = time.time_ns() - time.monotonic_ns()
    sent = dropped = 0
    end = time.monotonic() + args.seconds
    scan_id = 0
    while time.monotonic() < end:
        now = time.monotonic_ns()
        head = {"scan_id": scan_id, "ros_stamp_ns": now + sys_minus_mono - int(period * 1e9), "recv_mono_ns": now,
                "sys_minus_mono_ns": sys_minus_mono, "angle_min": -math.pi,
                "angle_increment": 2 * math.pi / args.beams, "time_increment": period / args.beams,
                "scan_time": period, "range_min": 0.15, "range_max": 12.0, "frame_id": "laser", "n": args.beams,
                "adapter_dropped": 0, "synthetic": True}
        try:
            sock.sendto(encode_frame(head, ranges), args.socket)
            sent += 1
        except (BlockingIOError, ConnectionRefusedError, FileNotFoundError):
            dropped += 1
        scan_id += 1
        time.sleep(max(0.0, period - (time.monotonic_ns() - now) / 1e9))
    print(f'{{"sent": {sent}, "dropped": {dropped}, "synthetic": true}}')
    return 0


if __name__ == "__main__":
    sys.exit(main())
