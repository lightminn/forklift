"""rclpy side of the gateway: subscriptions and TF lookups into the store (container only).

Every TF lookup asks for the latest transform with a zero timeout, so a broken tree never
holds the ROS thread (design D4, D7). Received times are the gateway's monotonic clock;
message stamps (sensor or sim time) are only passed through for the camera.
"""

from __future__ import annotations

import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from console.gateway.convert import (
    GridMessage,
    PoseSample,
    ScanMessage,
    is_jpeg,
    yaw_from_quat,
)
from console.gateway.store import CameraFrame, Store

POSE_HZ = 10.0
MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class GatewayNode(Node):
    def __init__(self, store: Store, clock=time.monotonic):
        super().__init__("console_gateway")
        self.store = store
        self.clock = clock
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        camera_topic = self.declare_parameter("camera_topic", "").value
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, MAP_QOS)
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        if camera_topic:
            self.create_subscription(
                CompressedImage, camera_topic, self.on_camera, qos_profile_sensor_data
            )
        self.create_timer(1.0 / POSE_HZ, self.on_pose_timer)

    # -------------------------------------------------------------- TF
    def lookup(self, target: str, source: str):
        """(x, y, yaw) of source in target from the latest transform, or None."""
        try:
            t = self.tf.lookup_transform(target, source, Time(), timeout=Duration())
        except TransformException:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return tr.x, tr.y, yaw_from_quat(q.x, q.y, q.z, q.w)

    def pose_of(self, frame: str):
        """frame in map, else in odom, as a PoseSample; None if neither is known."""
        now = self.clock()
        for target in (self.map_frame, self.odom_frame):
            xyyaw = self.lookup(target, frame)
            if xyyaw is not None:
                kind = "map" if target == self.map_frame else "odom"
                return PoseSample(kind, *xyyaw, received_s=now)
        return None

    def on_pose_timer(self) -> None:
        self.store.set_base(self.pose_of(self.base_frame))
        m2o = self.lookup(self.map_frame, self.odom_frame)
        if m2o is not None:
            self.store.set_map_to_odom(m2o, self.clock())

    # ------------------------------------------------------------ topics
    def on_map(self, msg: OccupancyGrid) -> None:
        o = msg.info.origin
        self.store.set_grid(
            GridMessage(
                width=msg.info.width,
                height=msg.info.height,
                resolution_m=msg.info.resolution,
                origin_x_m=o.position.x,
                origin_y_m=o.position.y,
                origin_yaw_rad=yaw_from_quat(
                    o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w
                ),
                frame="map"
                if msg.header.frame_id in ("", self.map_frame)
                else msg.header.frame_id,
                data=list(msg.data),
                received_s=self.clock(),
            )
        )

    def on_scan(self, msg: LaserScan) -> None:
        self.store.set_scan(
            ScanMessage(
                angle_min_rad=msg.angle_min,
                angle_increment_rad=msg.angle_increment,
                range_min_m=msg.range_min,
                range_max_m=msg.range_max,
                ranges=list(msg.ranges),
                received_s=self.clock(),
            ),
            self.pose_of(msg.header.frame_id),
        )

    def on_odom(self, msg: Odometry) -> None:
        self.store.set_speed(msg.twist.twist.linear.x, self.clock())

    def on_camera(self, msg: CompressedImage) -> None:
        data = bytes(msg.data)
        if not is_jpeg(data):
            self.store.note("camera", f"JPEG 가 아닌 영상은 보내지 않음 ({msg.format})")
            return
        self.store.note("camera", None)
        stamp = msg.header.stamp
        self.store.set_camera(
            CameraFrame(data, stamp.sec * 1_000_000_000 + stamp.nanosec, self.clock())
        )


def spin_in_thread(store: Store, args=None):
    """Start rclpy and spin a GatewayNode on a daemon thread; returns (node, thread)."""
    import threading

    rclpy.init(args=args)
    node = GatewayNode(store)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    return node, thread
