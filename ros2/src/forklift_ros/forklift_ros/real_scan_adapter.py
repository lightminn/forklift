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
import secrets
import socket
import struct
import time

import numpy as np

MAGIC = b"SCN1"
_LEN = struct.Struct("<I")


SOURCES = {  # --source: the topic and what its stamp means (PR 7a-2: the topic decides, so the two never mismatch)
    "raw": ("/sensors/lidar/scan_raw", "call_start"),  # upstream node: stamped before it fetches a scan
    "sdk": ("/sensors/lidar/scan_raw_sdk", "sdk_first_sample"),  # forklift patch 0001: the SDK's first sample
}


def make_frame(msg, *, scan_id: int, recv_mono_ns: int, sys_minus_mono_ns: int, gen: int | None = None,
               first_beam_mono_ns: int | None = None, stamp_kind: str | None = None) -> bytes:
    """A sensor_msgs/LaserScan (or anything with its fields) as one scan/1 frame. ``gen``: this adapter's start
    nonce (PR 7 design 3.2: the host opens a new generation when it changes); ``first_beam_mono_ns``: the SDK
    first-sample time on the host monotonic clock, when the driver gives it (7a-2)."""
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
    if gen is not None:
        head["gen"] = int(gen)
    if first_beam_mono_ns is not None:
        head["first_beam_mono_ns"] = int(first_beam_mono_ns)
    if stamp_kind is not None:
        head["stamp_kind"] = str(stamp_kind)
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

    def close(self) -> None:
        self.sock.close()


def main(argv=None) -> int:
    import argparse

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    ap = argparse.ArgumentParser(description="raw LaserScan -> scan/1 over a Unix datagram socket")
    ap.add_argument("--socket", required=True, action="append",
                    help="a host's bound socket (forklift_core.sensors.scan_feed); give it twice for the layer and "
                         "the SLAM bridge (PR 7 design 4: the adapter is where one scan gets its identity)")
    ap.add_argument("--source", choices=sorted(SOURCES), default="raw",
                    help="raw: the upstream topic (call-start stamps); sdk: the patched node's scan_sdk topic, stamped "
                         "at the SDK's first sample (PR 7a-2)")
    args, ros_args = ap.parse_known_args(argv)

    class Adapter(Node):
        def __init__(self) -> None:
            super().__init__("real_scan_adapter")
            self.senders = [DatagramSender(p) for p in args.socket]
            self.gen = secrets.randbits(62) + 1  # this start's nonce: a restart is a new producer generation
            self.scan_id = 0
            self.topic, self.stamp_kind = SOURCES[args.source]
            self.create_subscription(LaserScan, self.topic, self.on_scan, qos_profile_sensor_data)
            self.create_timer(5.0, self.report)

        def on_scan(self, msg) -> None:
            recv, pair = clock_pair()
            self.scan_id += 1
            # one frame, the same bytes to every receiver: each path keeps its own sent/dropped count
            frame = make_frame(msg, scan_id=self.scan_id, recv_mono_ns=recv, sys_minus_mono_ns=pair, gen=self.gen,
                               stamp_kind=self.stamp_kind)
            for s in self.senders:
                s.send(frame)

        def report(self) -> None:
            parts = [f"{s.path}: sent {s.sent} dropped {s.dropped} last_error {s.last_error}" for s in self.senders]
            self.get_logger().info(f"scans {self.scan_id} gen {self.gen} | " + " | ".join(parts))

    rclpy.init(args=ros_args)
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for s in node.senders:
            s.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
