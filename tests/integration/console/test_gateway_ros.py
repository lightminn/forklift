"""The gateway node against synthetic ROS publishers (runs in the console container).

    docker run --rm -v "$PWD":/workspace forklift/ros2-console:jazzy \
        python3 -m pytest tests/integration/console/test_gateway_ros.py -q -p no:cacheprovider
"""

import asyncio
import json
import math
import struct
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
websockets = pytest.importorskip("websockets")

from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import CompressedImage, LaserScan  # noqa: E402
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster  # noqa: E402

from console.gateway import server  # noqa: E402
from console.gateway.ros_node import MAP_QOS, GatewayNode  # noqa: E402
from console.gateway.store import Store  # noqa: E402


def _tf(stamp, parent, child, x, y, yaw):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.rotation.z = math.sin(yaw / 2)
    t.transform.rotation.w = math.cos(yaw / 2)
    return t


class FakeRobot(Node):
    """map -> odom (1, 0, 0) static, odom -> base_link (2, 1, +90 deg), laser 0.1 m ahead."""

    def __init__(self):
        super().__init__("fake_robot")
        self.static = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()
        self.static.sendTransform(
            [
                _tf(now, "map", "odom", 1.0, 0.0, 0.0),
                _tf(now, "base_link", "laser", 0.1, 0.0, 0.0),
            ]
        )
        self.tfb = TransformBroadcaster(self)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", MAP_QOS)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.cam_pub = self.create_publisher(CompressedImage, "/cam", 10)
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.width, grid.info.height, grid.info.resolution = 2, 2, 0.5
        grid.info.origin.position.x = -2.0
        grid.info.origin.position.y = -1.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0, 100, -1, 30]
        self.map_pub.publish(grid)
        self.tf_on = True
        self.t0 = self.get_clock().now().nanoseconds
        self.create_timer(0.05, self.tick)

    def tick(self):
        now = self.get_clock().now().to_msg()
        if self.tf_on:
            self.tfb.sendTransform(_tf(now, "odom", "base_link", 2.0, 1.0, math.pi / 2))
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = "laser"
        scan.angle_min, scan.angle_increment = -0.1, 0.1
        scan.range_min, scan.range_max = 0.1, 8.0
        scan.ranges = [1.0, float("inf"), 2.0]
        self.scan_pub.publish(scan)
        # like the Isaac bridge and the replay: pose only, the twist left empty;
        # heading +y at 0.3 m/s, the position following the stamp (timer jitter-proof)
        odom = Odometry()
        odom.header.stamp = now
        odom.pose.pose.position.y = 0.3 * (now.sec * 1e9 + now.nanosec - self.t0) * 1e-9
        odom.pose.pose.orientation.z = math.sin(math.pi / 4)
        odom.pose.pose.orientation.w = math.cos(math.pi / 4)
        self.odom_pub.publish(odom)
        cam = CompressedImage()
        cam.header.stamp = now
        cam.format = "bgr8; jpeg compressed bgr8"
        cam.data = b"\xff\xd8\xff\xe0fake"
        self.cam_pub.publish(cam)


def _frame(data):
    n = struct.unpack(">I", data[:4])[0]
    return json.loads(data[4 : 4 + n]), data[4 + n :]


def test_gateway_node_turns_ros_topics_into_console_messages(tmp_path, unused_tcp_port):
    rclpy.init(args=["gw", "--ros-args", "-p", "camera_topic:=/cam"])
    store = Store()
    gw_node = GatewayNode(store)
    robot = FakeRobot()
    ex = MultiThreadedExecutor()
    ex.add_node(gw_node)
    ex.add_node(robot)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    async def body():
        gw = await server.start(
            store, "127.0.0.1", unused_tcp_port, tmp_path, "fl-1", "isaac", 5, -0.34
        )
        try:
            c = await websockets.connect(
                f"ws://127.0.0.1:{unused_tcp_port}/api/v1/control"
            )
            await c.send(
                json.dumps({"t": "hello", "api": "console/1", "client_id": "a"})
            )
            deadline = time.monotonic() + 10
            st = None
            while time.monotonic() < deadline:
                msg = json.loads(await c.recv())
                if (
                    msg.get("t") == "state"
                    and msg["pose"]
                    and msg["pose"]["frame"] == "map"
                    and msg["rear_axle_speed_mps"] is not None  # needs two odom poses
                ):
                    st = msg
                    break
            assert st is not None, "no map pose with a speed within 10 s"
            # map -> base_link = (3, 1, +90 deg); the rear axle is 0.34 m behind: (3, 0.66)
            assert st["pose"]["x_m"] == pytest.approx(3.0, abs=1e-6)
            assert st["pose"]["y_m"] == pytest.approx(0.66, abs=1e-6)
            assert st["pose"]["yaw_rad"] == pytest.approx(math.pi / 2, abs=1e-6)
            assert st["pose"]["correction_version"] == 1  # the static map -> odom, once
            assert st["rear_axle_speed_mps"] == pytest.approx(
                0.3, abs=1e-4
            )  # float stamps
            assert st["pose"]["age_s"] < 0.5
            # the publisher stops: the buffer keeps answering with its last transform,
            # and that pose must age instead of looking current
            robot.tf_on = False
            stop_at = time.monotonic()
            aged = None
            while (
                time.monotonic() < stop_at + 3.0
            ):  # states queued meanwhile come first
                msg = json.loads(await c.recv())
                if msg.get("t") == "state" and msg["pose"]["age_s"] > 1.0:
                    aged = msg
                    break
            assert aged is not None, "the pose never aged after its TF stopped"
            # the last transform was stamped up to one 0.05 s tick before the stop
            assert time.monotonic() - stop_at >= 0.9
            await c.close()

            m = await websockets.connect(
                f"ws://127.0.0.1:{unused_tcp_port}/api/v1/media"
            )
            await m.send(
                json.dumps(
                    {
                        "t": "media_subscribe",
                        "client_id": "a",
                        "grid": {},
                        "scan": {},
                        "camera_rgb": {},
                    }
                )
            )
            seen = {}
            while len(seen) < 3 and time.monotonic() < deadline + 10:
                head, body_ = _frame(await asyncio.wait_for(m.recv(), 5))
                seen[head["t"]] = (head, body_)
            grid_head, cells = seen["grid"]
            assert grid_head["origin_m"] == [-2.0, -1.0]
            assert list(cells) == [1, 2, 0, 0]
            scan_head, ranges = seen["scan"]
            # laser = base_link + 0.1 m ahead (+y in map): (3, 1.1)
            assert scan_head["pose"]["x_m"] == pytest.approx(3.0, abs=1e-6)
            assert scan_head["pose"]["y_m"] == pytest.approx(1.1, abs=1e-6)
            assert math.isnan(struct.unpack("<3f", ranges)[1])
            assert seen["camera"][1].startswith(b"\xff\xd8")
            await m.close()
        finally:
            gw.close()
            await gw.wait_closed()

    try:
        asyncio.run(asyncio.wait_for(body(), 40))
    finally:
        ex.shutdown(timeout_sec=2.0)
        spin.join(timeout=2.0)
        gw_node.destroy_node()
        robot.destroy_node()
        rclpy.shutdown()
