"""Collect what a mapper produced during a bag replay, for offline evaluation.

For every /scan stamp it looks up map -> base_link once the mapper has
published a transform covering that instant: the online estimate the robot
would have used then, not a trajectory re-optimised after the fact. It keeps
the latest /map. On shutdown it writes slam_trajectory.csv, map.pgm/map.yaml
(the nav2 map_server convention) and recorder_summary.json into output_dir.
Scans whose pose never became available are counted, not dropped silently.
With map_history:=true every /map update is also kept, in map_history.npz,
so the map can be shown as it grew.
"""

import csv
import json
import math
from pathlib import Path

import numpy as np

UNKNOWN, FREE, OCCUPIED = 205, 254, 0


def occupancy_to_pgm(data: np.ndarray, width: int, height: int) -> bytes:
    """nav2-style trinary PGM: -1 unknown, 0 free, >= 65 occupied, row 0 on top."""
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    image = np.full(grid.shape, UNKNOWN, dtype=np.uint8)
    image[(grid >= 0) & (grid <= 25)] = FREE
    image[grid >= 65] = OCCUPIED
    return f"P5\n{width} {height}\n255\n".encode() + np.flipud(image).tobytes()


def pack_map_history(entries: list[tuple]) -> dict[str, np.ndarray]:
    """(stamp_ns, origin x, origin y, origin yaw, resolution, width, height,
    data) tuples -> arrays for np.savez; grid i is map_<i:04d>, row 0 at the
    map origin as in nav_msgs/OccupancyGrid."""
    packed = {
        "stamps_ns": np.array([e[0] for e in entries], dtype=np.int64),
        "origins": np.array([e[1:4] for e in entries], dtype=float).reshape(-1, 3),
        "resolutions_m": np.array([e[4] for e in entries], dtype=float),
    }
    for index, (*_, width, height, data) in enumerate(entries):
        packed[f"map_{index:04d}"] = np.asarray(data, dtype=np.int8).reshape(
            height, width
        )
    return packed


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main(args=None) -> None:
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener

    class SlamRecorder(Node):
        def __init__(self):
            super().__init__("slam_recorder")
            self.output = Path(self.declare_parameter("output_dir", "").value)
            self.keep_history = bool(self.declare_parameter("map_history", False).value)
            self.history: list[tuple] = []
            if not str(self.output):
                raise ValueError("output_dir is required")
            self.output.mkdir(parents=True, exist_ok=True)
            self.buffer = Buffer(cache_time=Duration(seconds=600))
            self.listener = TransformListener(self.buffer, self)
            self.pending: list[Time] = []
            self.poses: list[tuple[float, float, float, float]] = []
            self.scans = 0
            self.map = None
            self.create_subscription(LaserScan, "/scan", self.on_scan, 50)
            latched = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
            self.create_timer(0.2, self.resolve)

        def on_scan(self, message):
            self.scans += 1
            self.pending.append(Time.from_msg(message.header.stamp))

        def on_map(self, message):
            self.map = message
            if self.keep_history:
                info, origin = message.info, message.info.origin
                self.history.append(
                    (
                        Time.from_msg(message.header.stamp).nanoseconds,
                        origin.position.x,
                        origin.position.y,
                        yaw_from_quaternion(
                            origin.orientation.x,
                            origin.orientation.y,
                            origin.orientation.z,
                            origin.orientation.w,
                        ),
                        info.resolution,
                        info.width,
                        info.height,
                        np.asarray(message.data, dtype=np.int8),
                    )
                )

        def resolve(self):
            waiting = []
            for stamp in self.pending:
                if self.buffer.can_transform("map", "base_link", stamp):
                    t = self.buffer.lookup_transform("map", "base_link", stamp)
                    q = t.transform.rotation
                    self.poses.append(
                        (
                            stamp.nanoseconds * 1e-9,
                            t.transform.translation.x,
                            t.transform.translation.y,
                            yaw_from_quaternion(q.x, q.y, q.z, q.w),
                        )
                    )
                else:
                    waiting.append(stamp)
            self.pending = waiting

        def write(self):
            self.resolve()
            self.poses.sort()
            with (self.output / "slam_trajectory.csv").open("w", newline="") as f:
                table = csv.writer(f)
                table.writerow(["ros_stamp_s", "x_m", "y_m", "yaw_rad"])
                table.writerows([f"{v:.9f}" for v in pose] for pose in self.poses)
            summary = {
                "scans_received": self.scans,
                "poses_resolved": len(self.poses),
                "poses_unresolved": len(self.pending),
                "estimate": "online map->base_link at each scan stamp",
                "map_received": self.map is not None,
            }
            if self.map is not None:
                info = self.map.info
                (self.output / "map.pgm").write_bytes(
                    occupancy_to_pgm(self.map.data, info.width, info.height)
                )
                origin = info.origin
                (self.output / "map.yaml").write_text(
                    "image: map.pgm\nmode: trinary\n"
                    f"resolution: {info.resolution}\n"
                    f"origin: [{origin.position.x}, {origin.position.y}, "
                    f"{yaw_from_quaternion(origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w)}]\n"
                    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
                    encoding="utf-8",
                )
                summary["map"] = {
                    "width": info.width,
                    "height": info.height,
                    "resolution_m": info.resolution,
                }
            if self.keep_history and self.history:
                np.savez_compressed(
                    self.output / "map_history.npz", **pack_map_history(self.history)
                )
                summary["map_history_updates"] = len(self.history)
            (self.output / "recorder_summary.json").write_text(
                json.dumps(summary, indent=2) + "\n", encoding="utf-8"
            )
            return summary

    rclpy.init(args=args)
    node = SlamRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # launch shutdown ends the spin; the record is written below
    finally:
        summary = node.write()
        print(json.dumps(summary), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
