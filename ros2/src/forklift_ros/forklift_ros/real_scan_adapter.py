"""The real LiDAR's adapter (docs/design/2026-11-12-real-scan-obstacle-layer.md 1.2): subscribes to the driver's raw
scan and sends one scan/1 frame per message to the host over a Unix datagram socket. It changes nothing in the
values (the host's ScanFeed reads them) and never waits: no receiver, or a full one, drops the scan and counts it.

Each frame carries the driver stamp as given, the adapter's receipt on CLOCK_MONOTONIC and the (system - monotonic)
pair read at that moment, so the host can place the stamp on its monotonic clock (plan 2.5). The upstream node's
stamp is taken before it fetches a scan that may have waited in a buffer -- it is not the acquisition time (design
1.1); the host names the delay stamp-to-receipt.

The ROS imports live in ``main`` so the framing and the sender load and are tested without ROS. The frame layout
matches ``forklift_core.sensors.scan_feed.encode_frame`` (a root test decodes what this module makes)."""

from __future__ import annotations

import errno
import json
import socket
import struct
import time

import numpy as np

MAGIC = b"SCN1"
_LEN = struct.Struct("<I")


def make_frame(msg, *, scan_id: int, recv_mono_ns: int, sys_minus_mono_ns: int) -> bytes:
    """A sensor_msgs/LaserScan (or anything with its fields) as one scan/1 frame."""
    ranges = np.asarray(msg.ranges, dtype="<f4")
    inten = np.asarray(msg.intensities, dtype="<f4") if len(msg.intensities) == len(ranges) and len(ranges) else None
    head = {
        "scan_id": int(scan_id),
        "ros_stamp_ns": int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec),
        "recv_mono_ns": int(recv_mono_ns),
        "sys_minus_mono_ns": int(sys_minus_mono_ns),
        "angle_min": float(msg.angle_min),
        "angle_increment": float(msg.angle_increment),
        "time_increment": float(msg.time_increment),
        "scan_time": float(msg.scan_time),
        "range_min": float(msg.range_min),
        "range_max": float(msg.range_max),
        "frame_id": str(msg.header.frame_id),
        "n": int(len(ranges)),
        "has_intensities": inten is not None,
    }
    raw = json.dumps(head).encode("utf-8")
    body = ranges.tobytes() + (inten.tobytes() if inten is not None else b"")
    return MAGIC + _LEN.pack(len(raw)) + raw + body


def clock_pair() -> tuple[int, int]:
    """(monotonic, system - monotonic), the system clock read between two monotonic reads."""
    m0 = time.monotonic_ns()
    s = time.time_ns()
    m1 = time.monotonic_ns()
    m = (m0 + m1) // 2
    return m, s - m


class DatagramSender:
    """Sends whole frames to ``path``; anything that would wait or fail drops the frame (counted)."""

    def __init__(self, path: str) -> None:
        self.path = path
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sent = 0
        self.dropped = 0
        self.last_error: str | None = None

    def send(self, frame: bytes) -> bool:
        try:
            self.sock.sendto(frame, self.path)
        except (BlockingIOError, InterruptedError):
            self.dropped += 1
            self.last_error = "full"
            return False
        except OSError as exc:  # no receiver yet (ENOENT, ECONNREFUSED), or anything else
            self.dropped += 1
            self.last_error = errno.errorcode.get(exc.errno, str(exc.errno))
            return False
        self.sent += 1
        return True


def main(argv=None) -> int:
    import argparse

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    ap = argparse.ArgumentParser(description="raw LaserScan -> scan/1 over a Unix datagram socket")
    ap.add_argument("--socket", required=True, help="the host's bound socket (forklift_core.sensors.scan_feed)")
    ap.add_argument("--topic", default="/sensors/lidar/scan_raw")
    args, ros_args = ap.parse_known_args(argv)

    class Adapter(Node):
        def __init__(self) -> None:
            super().__init__("real_scan_adapter")
            self.sender = DatagramSender(args.socket)
            self.scan_id = 0
            self.create_subscription(LaserScan, args.topic, self.on_scan, qos_profile_sensor_data)
            self.create_timer(5.0, self.report)

        def on_scan(self, msg) -> None:
            recv, pair = clock_pair()
            self.scan_id += 1
            self.sender.send(make_frame(msg, scan_id=self.scan_id, recv_mono_ns=recv, sys_minus_mono_ns=pair))

        def report(self) -> None:
            s = self.sender
            self.get_logger().info(f"scans {self.scan_id} sent {s.sent} dropped {s.dropped} last_error {s.last_error}")

    rclpy.init(args=ros_args)
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
