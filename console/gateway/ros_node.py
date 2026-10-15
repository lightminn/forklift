"""rclpy side of the gateway: subscriptions and TF lookups into the store (container only).

TF lookups never wait (zero timeout), so a broken tree never holds the ROS thread
(design D4). The buffer runs on the node's clock, so with use_sim_time a bag that jumps
back clears it instead of keeping the end of the previous pass. A pose's age is the
larger of its stamp's age on the node clock and the wall time since that stamp last
advanced -- the buffer keeps answering with its last transform after the publisher has
stopped, and that answer must age.

A pose in map is the latest map -> odom correction composed with the latest odom ->
frame motion, not a lookup at their common time: slam_toolbox stamps map -> odom with
its last keyframe, which stops advancing while the truck waits, and a common-time lookup
froze the displayed pose for tens of seconds (Isaac online SLAM, 2026-10-15). The pose's
age is the motion's; the correction's age is reported separately (freshness).
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
    compose2d,
    is_jpeg,
    speed_from_poses,
    yaw_from_quat,
)
from console.gateway.store import CameraFrame, Store

POSE_HZ = 10.0
MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class GatewayNode(Node):
    def __init__(self, store: Store, clock=time.monotonic):
        super().__init__("console_gateway")
        self.store = store
        self.clock = clock
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        camera_topic = self.declare_parameter("camera_topic", "").value
        self.tf = Buffer(node=self)  # node clock: sim time and its jumps
        self.tf_listener = TransformListener(self.tf, self)
        self._stamp_seen: dict[tuple[str, str], tuple[int, float]] = {}
        self._last_odom = None
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

    @property
    def sim_time(self) -> bool:
        return bool(self.get_parameter("use_sim_time").value)

    # -------------------------------------------------------------- TF
    def lookup(self, target: str, source: str, at: Time | None = None):
        """(x, y, yaw, stamp_ns) of source in target, or None; latest when at is None."""
        try:
            t = self.tf.lookup_transform(
                target, source, at if at is not None else Time(), timeout=Duration()
            )
        except TransformException:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        stamp_ns = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec
        return tr.x, tr.y, yaw_from_quat(q.x, q.y, q.z, q.w), stamp_ns

    def _age(self, key, stamp_ns: int) -> float:
        """Seconds the transform behind a lookup is old (see the module docstring)."""
        now_wall = self.clock()
        seen = self._stamp_seen.get(key)
        if seen is None or seen[0] != stamp_ns:
            self._stamp_seen[key] = (stamp_ns, now_wall)
            seen = self._stamp_seen[key]
        stalled = now_wall - seen[1]
        ros_age = 0.0
        if stamp_ns:  # 0 = a static-only chain: no time of its own
            ros_age = max(0.0, (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9)
        return max(ros_age, stalled)

    def pose_of(self, frame: str, at: Time | None = None):
        """frame in map (latest correction after the odom motion), else in odom, else a
        direct map lookup (e.g. a localiser publishing map -> frame); None if unknown.
        With `at`, the motion part is taken at that time (a scan's stamp)."""
        motion = self.lookup(self.odom_frame, frame, at)
        if motion is not None:
            x, y, yaw, stamp_ns = motion
            age = (
                0.0 if at is not None else self._age((self.odom_frame, frame), stamp_ns)
            )
            received = self.clock() - age
            correction = self.lookup(self.map_frame, self.odom_frame)
            if correction is not None:
                return PoseSample(
                    "map", *compose2d(correction[:3], (x, y, yaw)), received_s=received
                )
            return PoseSample("odom", x, y, yaw, received_s=received)
        direct = self.lookup(self.map_frame, frame, at)
        if direct is not None:
            x, y, yaw, stamp_ns = direct
            age = (
                0.0 if at is not None else self._age((self.map_frame, frame), stamp_ns)
            )
            return PoseSample("map", x, y, yaw, received_s=self.clock() - age)
        return None

    def on_pose_timer(self) -> None:
        self.store.set_base(self.pose_of(self.base_frame))
        m2o = self.lookup(self.map_frame, self.odom_frame)
        if m2o is not None:
            self.store.set_map_to_odom(m2o[:3], self.clock())

    # ------------------------------------------------------------ topics
    def on_map(self, msg: OccupancyGrid) -> None:
        o = msg.info.origin
        frame = msg.header.frame_id
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
                frame="map" if frame in ("", self.map_frame) else frame,
                data=list(msg.data),
                received_s=self.clock(),
            )
        )

    def on_scan(self, msg: LaserScan) -> None:
        """Place the scan with the laser pose at its own stamp; else the latest, marked."""
        laser = msg.header.frame_id
        laser_pose = self.pose_of(laser, at=Time.from_msg(msg.header.stamp))
        pose_at, delay = "scan_stamp", 0.0
        if laser_pose is None:
            laser_pose = self.pose_of(laser)
            pose_at = "latest"
            latest = self.lookup(self.odom_frame, laser)
            if latest is not None and latest[3]:
                delay = max(0.0, latest[3] * 1e-9 - _seconds(msg.header.stamp))
        self.store.set_scan(
            ScanMessage(
                angle_min_rad=msg.angle_min,
                angle_increment_rad=msg.angle_increment,
                range_min_m=msg.range_min,
                range_max_m=msg.range_max,
                ranges=list(msg.ranges),
                received_s=self.clock(),
            ),
            laser_pose,
            pose_at,
            delay,
        )

    def on_odom(self, msg: Odometry) -> None:
        """Speed from successive odom poses: the producers leave the twist empty."""
        p = msg.pose.pose
        cur = (
            _seconds(msg.header.stamp),
            p.position.x,
            p.position.y,
            yaw_from_quat(
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
            ),
        )
        prev, self._last_odom = self._last_odom, cur
        if prev is None:
            return
        v = speed_from_poses(prev, cur)
        if v is not None:
            self.store.set_speed(v, self.clock())

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
