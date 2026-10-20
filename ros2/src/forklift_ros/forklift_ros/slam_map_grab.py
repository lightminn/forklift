"""Keep the /map a slam_toolbox loaded from a saved pose graph publishes, with no scan sent (SLAM localization plan
docs/plans/2026-10-20-slam-localization-session.md L2: the survey bundle's grid is made from the saved graph alone).

Waits for /map and ends once ``repeats`` maps in a row are the same (cells, size, resolution, origin), or at
``timeout_s``. Writes ``<out>/map.npz`` (``data`` int8 as /map: -1 unknown, 0 free, 100 occupied, row-major from the
origin; ``origin`` [x, y, yaw]; ``resolution_m``) and ``<out>/map_grab.json`` (maps received, whether it settled).

    python3 -m forklift_ros.slam_map_grab --ros-args -p output_dir:=/out -p timeout_s:=120.0
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def settled(maps: list, repeats: int) -> bool:
    """The last ``repeats`` maps are one and the same grid (``maps``: (data, origin, resolution) tuples)."""
    if len(maps) < repeats:
        return False
    data, origin, resolution = maps[-1]
    return all(o == origin and r == resolution and d.shape == data.shape and np.array_equal(d, data)
               for d, o, r in maps[-repeats:-1])


class MapGrab(Node):
    def __init__(self):
        super().__init__("slam_map_grab")
        self.declare_parameter("output_dir", "/out")
        self.declare_parameter("timeout_s", 120.0)
        self.declare_parameter("repeats", 3)
        self.maps: list = []
        self.received = 0
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)

    def on_map(self, msg: OccupancyGrid) -> None:
        info, q = msg.info, msg.info.origin.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        origin = [info.origin.position.x, info.origin.position.y, yaw]
        self.maps.append((np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width), origin,
                          float(info.resolution)))
        self.received += 1
        del self.maps[:-8]  # the last few are all the check reads


def main(args=None) -> int:
    rclpy.init(args=args)
    node = MapGrab()
    out = Path(node.get_parameter("output_dir").value)
    timeout_s = float(node.get_parameter("timeout_s").value)
    repeats = int(node.get_parameter("repeats").value)
    start = time.monotonic()
    while time.monotonic() - start < timeout_s and not settled(node.maps, repeats):
        rclpy.spin_once(node, timeout_sec=0.2)
    ok = settled(node.maps, repeats)
    out.mkdir(parents=True, exist_ok=True)
    if node.maps:
        data, origin, resolution = node.maps[-1]
        np.savez_compressed(out / "map.npz", data=data, origin=np.asarray(origin, dtype=float),
                            resolution_m=resolution)
    (out / "map_grab.json").write_text(json.dumps({
        "settled": ok, "repeats": repeats, "maps_received": node.received, "wall_s": time.monotonic() - start,
        "shape": list(node.maps[-1][0].shape) if node.maps else None}))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
