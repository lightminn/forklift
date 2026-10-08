"""Lockstep bridge between rig frames and RTAB-Map (visual, LiDAR or both).

Plan: docs/plans/2026-10-07-visual-slam-and-fusion.md, D2-D3. Frames (scan +
RGB-D images + wheel odometry, one per 10 Hz LiDAR period) come either from
the Isaac runner over a Unix socket (``source:=socket``, the closed loop) or
from a recorded run (``source:=record``, offline). Each frame is pushed through
the front ends one at a time and waited for, so nothing is dropped whatever
the processing speed:

1. ``/clock``, ``/tf`` odom->base_link and ``/odom`` (wheel odometry) at the
   frame stamp.
2. Vision front end (rgbd_odometry, ``/rig/rgbd_images``) -> wait for its
   OdomInfo at that stamp. LiDAR front end (icp_odometry, ``/scan``) -> wait
   likewise. Both predict from the wheel odometry and publish no TF.
3. The bridge fuses the frame-to-frame motion of the wheels and of every front
   end that tracked this frame (``odometry_fusion``, by each source's assumed
   uncertainty) into the ``fused`` frame and publishes fused->odom. A lost
   front end, or one whose sensor delivered nothing, simply drops out.
4. Keyframe (``slam_link.KeyframeGate`` with one start-up frame; all the back
   end's sensors present): republish on ``/rig/key/*`` and wait for rtabmap's
   ``mapGraph`` at that stamp; rtabmap processes every keyframe it gets and
   uses ``fused`` as its odometry frame.
5. Estimate base_link in the output frame, and for the runner the map<-odom
   correction that implies. The output frame equals the wheel odometry frame
   at the first frame (the known start pose).

Frames: map -> fused -> odom -> base_link. Without wheel odometry
(``mode:=vision_noodom``, offline only) fused -> base_link and the visual
odometry is the only source.

The pose arithmetic is planar (x, y, yaw); the rig drives on a flat floor and
every RTAB-Map registration here is forced to 3 DoF.
"""

from __future__ import annotations

import json
import math
import socket
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from forklift_core.localization import slam_link
from forklift_core.localization.odometry_fusion import FusedOdometry, IncrementNoise
from forklift_core.localization.slam_pose import compose, invert

CLOCK_OFFSET_S = 10.0  # tf2 reads stamp 0 as "latest"; keep every stamp clear of it
MODES = {
    # mode: (front ends, back end sensors)
    "lidar": (("icp",), ("scan",)),
    "vision": (("vo",), ("rgbd",)),
    "vision_noodom": (("vo",), ("rgbd",)),
    "fusion": (("vo", "icp"), ("rgbd", "scan")),
}
# Per-frame increment uncertainty of each source (odometry_fusion.IncrementNoise).
# Design values from the development record (seed 0, plan D3), not sensor
# specifications: wheels are good along the path but over-count turns (right
# turns 7.6 %, left 2.1 % on seed 0); visual odometry turns well (0.6 %) but
# jitters along the path; ICP against a 1600-beam scan is tight in both.
INCREMENT_NOISE = {
    "wheel": IncrementNoise(0.02, 0.0005, 0.08, 0.002, 0.0005),
    "vo": IncrementNoise(0.03, 0.002, 0.01, 0.001, 0.0005),
    "icp": IncrementNoise(0.003, 0.001, 0.002, 0.0002, 0.0002),
}


def stamp_ns(stamp_s: float) -> int:
    return int(round((stamp_s + CLOCK_OFFSET_S) * 1e9))


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


@dataclass
class Chain:
    """The fused odometry frame and the back end's correction on top of it."""

    mode: str
    fused: FusedOdometry = field(init=False)
    top_from_base: tuple | None = None
    map_from_top: tuple = (0.0, 0.0, 0.0)
    output_from_map: tuple | None = None  # fixed at the first frame

    def __post_init__(self) -> None:
        sources = (("wheel",) if self.wheel else ()) + MODES[self.mode][0]
        self.fused = FusedOdometry({name: INCREMENT_NOISE[name] for name in sources})

    @property
    def stages(self) -> tuple[str, ...]:
        return MODES[self.mode][0]

    @property
    def wheel(self) -> bool:
        return self.mode != "vision_noodom"

    def update(self, poses: dict, odom_from_base) -> tuple:
        """Fuse this frame's front end poses (None = not tracked); fused <- base."""
        if self.wheel:
            if self.top_from_base is None:
                # The fused frame coincides with odom at the first frame.
                self.fused.pose = tuple(odom_from_base)
            poses = {"wheel": tuple(odom_from_base), **poses}
        self.top_from_base = self.fused.update(poses)
        return self.top_from_base

    def top_from_below(self, odom_from_base) -> tuple:
        """fused <- odom (fused <- base_link without wheels): the TF to publish."""
        if not self.wheel:
            return self.top_from_base
        return compose(self.top_from_base, invert(odom_from_base))

    def output_from_base(self, odom_from_base) -> tuple:
        map_from_base = compose(self.map_from_top, self.top_from_base)
        if self.output_from_map is None:
            # Known start: the output frame is the wheel odometry frame at frame 0.
            start = tuple(odom_from_base) if self.wheel else (0.0, 0.0, 0.0)
            self.output_from_map = compose(start, invert(map_from_base))
        return compose(self.output_from_map, map_from_base)


class RecordSource:
    """Frames from a run_slam_drive.py record with --depth-rig (offline).

    Wheel odometry and LiDAR noise come from forklift_ros.slam_replay (the
    same arrays the slam_toolbox bag gets); depth noise from camera_rig's
    keyed DepthNoise (the same draw the online runner makes for that frame).
    """

    def __init__(self, record: Path, *, cameras, noise_seed: int | None, condition):
        import cv2

        from forklift_core.sensors.camera_rig import DepthNoise, rig_from_config
        from forklift_ros import slam_replay

        self.cv2 = cv2
        self.record = Path(record)
        log, meta = slam_replay.load_slam_log(self.record)
        self.meta = meta
        noise = (
            slam_replay.ReplayNoise(0.02, 0.2, 0.005, noise_seed)
            if noise_seed is not None
            else slam_replay.ReplayNoise()
        )
        scaled = dict(log)
        scaled["wheel_rates_rad_s"] = log["wheel_rates_rad_s"].astype(float).copy()
        scaled["wheel_rates_rad_s"][:, 2:4] = condition.wheel_rates(
            scaled["wheel_rates_rad_s"][:, 2:4]
        )
        odometry = slam_replay.odometry_base_poses(scaled, meta, noise)
        rows = {float(t): i for i, t in enumerate(log["joint_stamps_s"])}
        self.scan_stamps = log["scan_stamps_s"].astype(float)
        self.odom = np.array([odometry[rows[float(t)]] for t in log["scan_stamps_s"]])
        self.truth = slam_replay.ground_truth_base_poses(log)[
            [rows[float(t)] for t in log["scan_stamps_s"]]
        ]
        self.ranges = slam_replay.noisy_ranges(log["scan_ranges_m"], meta, noise)
        self.laser = meta["laser"]
        rig = rig_from_config(meta["depth_rig"]["config"])
        self.rig = rig
        self.cameras = list(cameras)
        self.camera_index = {name: rig.names.index(name) for name in self.cameras}
        self.depth_noise = DepthNoise(
            coefficient_per_m=rig.depth_noise_coefficient_per_m,
            range_m=rig.depth_range_m,
            seed=noise_seed or 0,
            enabled=noise_seed is not None,
        )
        self.condition = condition

    def __len__(self) -> int:
        return len(self.scan_stamps)

    def frame(self, index: int) -> slam_link.Frame:
        stamp = float(self.scan_stamps[index])
        images = ()
        if self.condition.cameras_available(stamp):
            out = []
            for name in self.cameras:
                base = self.record / "rig" / name / f"{index:06d}"
                bgr = self.cv2.imread(
                    str(base.with_suffix(".jpg")), self.cv2.IMREAD_COLOR
                )
                depth = self.cv2.imread(
                    str(base.with_suffix(".png")), self.cv2.IMREAD_UNCHANGED
                )
                if bgr is None or depth is None or depth.dtype != np.uint16:
                    raise RuntimeError(f"missing or bad rig frame {base}")
                clean_m = np.where(depth > 0, depth.astype(float) / 1000.0, np.inf)
                noisy = self.depth_noise.depth_mm(
                    clean_m, self.camera_index[name], index
                )
                out.append(slam_link.RigImage(name, bgr[:, :, ::-1].copy(), noisy))
            images = tuple(out)
        ranges = None
        if self.condition.lidar_available(stamp):
            ranges = self.condition.ranges(self.ranges[index]).astype(np.float32)
        angle_increment = 2 * math.pi / self.laser["beam_count"]
        return slam_link.Frame(
            index,
            stamp,
            tuple(float(v) for v in self.odom[index]),
            ranges,
            float(self.laser["angle_min_rad"]),
            float(self.laser.get("angle_increment_rad", angle_increment)),
            float(self.laser["range_min_m"]),
            float(self.laser["range_max_m"]),
            images,
        )


def main(args=None) -> int:  # noqa: C901 - one ROS node, kept in one place
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from rtabmap_msgs.msg import MapGraph, OdomInfo, RGBDImage, RGBDImages
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from tf2_msgs.msg import TFMessage

    from forklift_core.localization.sensor_conditions import (
        condition as named_condition,
    )
    from forklift_core.sensors.camera_rig import rig_from_config

    def ros_time(stamp_s):
        from builtin_interfaces.msg import Time

        ns = stamp_ns(stamp_s)
        return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

    def msg_ns(stamp) -> int:
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def planar_tf(stamp, parent, child, pose, z=0.0, rotation=None):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
        t.transform.translation.x, t.transform.translation.y = (
            float(pose[0]),
            float(pose[1]),
        )
        t.transform.translation.z = float(z)
        if rotation is None:
            t.transform.rotation.z = math.sin(pose[2] / 2)
            t.transform.rotation.w = math.cos(pose[2] / 2)
        else:
            x, y, zq, w = rotation
            t.transform.rotation.x, t.transform.rotation.y = x, y
            t.transform.rotation.z, t.transform.rotation.w = zq, w
        return t

    class Bridge(Node):
        def __init__(self):
            super().__init__("rig_slam_bridge")
            p = self.declare_parameter
            p("mode", "fusion")
            p("source", "record")
            p("record_dir", "")
            p("socket_path", "/run/slam/slam.sock")
            p("output_dir", "/tmp/rig_bridge")
            p("rig_config", "/workspace/config/isaac_depth_rig.yaml")
            p("laser_xyz_yaw", [-0.12, 0.0, 1.05, 0.0])
            p("cameras", ["front", "rear", "left", "right"])
            p("noise_seed", -1)
            p("condition", "nominal")
            p("reply_timeout_s", 120.0)
            p("map_every", 10)
            p("max_frames", 0)  # development: stop a record after this many (0 = all)
            get = lambda name: self.get_parameter(name).value  # noqa: E731
            self.mode = get("mode")
            if self.mode not in MODES:
                raise ValueError(f"mode must be one of {sorted(MODES)}")
            self.source_kind = get("source")
            self.output_dir = Path(get("output_dir"))
            self.cameras = list(get("cameras"))
            seed = int(get("noise_seed"))
            self.noise_seed = None if seed < 0 else seed
            self.condition = named_condition(get("condition"))
            self.reply_timeout_s = float(get("reply_timeout_s"))
            self.map_every = int(get("map_every"))
            self.max_frames = int(get("max_frames"))
            self.record = None
            if self.source_kind == "record":
                self.record = RecordSource(
                    Path(get("record_dir")),
                    cameras=self.cameras,
                    noise_seed=self.noise_seed,
                    condition=self.condition,
                )
                self.rig = self.record.rig
                laser = self.record.laser
                self.laser = [*laser["mount_xyz_m"], laser["mount_yaw_rad"]]
            elif self.source_kind == "socket":
                import yaml

                self.rig = rig_from_config(
                    yaml.safe_load(Path(get("rig_config")).read_text())
                )
                self.laser = [float(v) for v in get("laser_xyz_yaw")]
            else:
                raise ValueError("source must be record or socket")
            for name in self.cameras:
                self.rig.camera(name)
            self.stages, self.back = MODES[self.mode]
            self.uses_rgbd = "vo" in self.stages
            self.uses_scan = "icp" in self.stages
            self.chain = Chain(self.mode)
            self.front_pose: dict[str, tuple] = {}

            reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            latched = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.clock_pub = self.create_publisher(Clock, "/clock", 10)
            self.tf_pub = self.create_publisher(TFMessage, "/tf", 100)
            self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", latched)
            self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
            self.rgbd_pub = self.create_publisher(
                RGBDImages, "/rig/rgbd_images", reliable
            )
            self.scan_pub = self.create_publisher(LaserScan, "/scan", reliable)
            self.key_rgbd_pub = self.create_publisher(
                RGBDImages, "/rig/key/rgbd_images", reliable
            )
            self.key_scan_pub = self.create_publisher(
                LaserScan, "/rig/key/scan", reliable
            )
            self.cond = threading.Condition()
            self.odom_infos: dict[str, dict[int, OdomInfo]] = {"vo": {}, "icp": {}}
            self.odoms: dict[str, dict[int, Odometry]] = {"vo": {}, "icp": {}}
            self.graphs: dict[int, MapGraph] = {}
            for stage, ns in (("vo", "/rig/vo"), ("icp", "/rig/icp")):
                self.create_subscription(
                    OdomInfo,
                    f"{ns}/odom_info",
                    self._keeper(self.odom_infos[stage]),
                    100,
                )
                self.create_subscription(
                    Odometry, f"{ns}/odom", self._keeper(self.odoms[stage]), 100
                )
            self.create_subscription(
                MapGraph, "/rtabmap/mapGraph", self._keeper(self.graphs), 100
            )
            self.maps: list[dict] = []
            self.map_arrays: dict[str, np.ndarray] = {}
            self.map_lock = threading.Lock()
            self.last_frame_id = -1
            self.create_subscription(
                OccupancyGrid, "/rtabmap/map", self._on_map, latched
            )
            self.records: list[dict] = []
            self.gate = slam_link.KeyframeGate(startup_scans=1)
            self.camera_infos = {name: self._camera_info(name) for name in self.cameras}
            self.ok = False

        # --- ROS side -----------------------------------------------------
        def _keeper(self, store):
            def keep(msg):
                with self.cond:
                    store[msg_ns(msg.header.stamp)] = msg
                    self.cond.notify_all()

            return keep

        def _on_map(self, msg):
            with self.map_lock:
                self.map_counter = getattr(self, "map_counter", 0) + 1
                if (self.map_counter - 1) % self.map_every:
                    self.latest_map = msg
                    return
                self._keep_map(msg)
                self.latest_map = None

        def _keep_map(self, msg):
            index = len(self.maps)
            info = msg.info
            self.maps.append(
                {
                    "index": index,
                    "after_frame_id": self.last_frame_id,
                    "stamp_ns": msg_ns(msg.header.stamp),
                    "resolution_m": info.resolution,
                    "width": info.width,
                    "height": info.height,
                    "origin": [
                        info.origin.position.x,
                        info.origin.position.y,
                        yaw_of(info.origin.orientation),
                    ],
                }
            )
            self.map_arrays[f"map_{index:04d}"] = np.asarray(msg.data, np.int8).reshape(
                info.height, info.width
            )

        def _camera_info(self, name):
            camera = self.rig.camera(name)
            k = camera.intrinsics
            info = CameraInfo()
            info.header.frame_id = camera.frame_id
            info.width, info.height = k.width, k.height
            info.distortion_model = "plumb_bob"
            info.d = [0.0] * 5
            info.k = [k.fx, 0.0, k.cx, 0.0, k.fy, k.cy, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [k.fx, 0.0, k.cx, 0.0, 0.0, k.fy, k.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            return info

        def publish_static(self):
            from forklift_core.sensors.camera_rig import (
                base_from_optical_rotation,  # noqa: F401
            )

            stamp = ros_time(0.0)
            x, y, z, yaw = self.laser
            transforms = [planar_tf(stamp, "base_link", "laser", (x, y, yaw), z=z)]
            for name in self.cameras:
                camera = self.rig.camera(name)
                r = np.asarray(camera.rotation_base_from_optical)
                transforms.append(
                    planar_tf(
                        stamp,
                        "base_link",
                        camera.frame_id,
                        (*camera.translation_base_m[:2], 0.0),
                        z=camera.translation_base_m[2],
                        rotation=_quaternion_xyzw(r),
                    )
                )
            self.tf_static_pub.publish(TFMessage(transforms=transforms))

        def wait_ready(self):
            self.publish_static()
            needed = []
            if self.uses_rgbd:
                needed.append("/rig/rgbd_images")
            if self.uses_scan:
                needed.append("/scan")
            needed += ["/rig/key/rgbd_images"] if "rgbd" in self.back else []
            needed += ["/rig/key/scan"] if "scan" in self.back else []
            deadline = time.monotonic() + 180.0
            while any(self.count_subscribers(topic) == 0 for topic in needed):
                if time.monotonic() > deadline:
                    missing = [t for t in needed if self.count_subscribers(t) == 0]
                    raise RuntimeError(f"no subscriber on {missing}")
                time.sleep(0.2)
            time.sleep(2.0)  # let every tf buffer take the static transforms

        def _rgbd_msg(self, frame, stamp):
            images = RGBDImages()
            images.header.stamp, images.header.frame_id = stamp, "base_link"
            for image in frame.images:
                if image.camera not in self.camera_infos:
                    continue
                info = self.camera_infos[image.camera]
                info.header.stamp = stamp
                one = RGBDImage()
                one.header.stamp, one.header.frame_id = stamp, info.header.frame_id
                one.rgb_camera_info = info
                one.depth_camera_info = info
                rgb, depth = Image(), Image()
                for msg, array, encoding, step in (
                    (rgb, image.rgb, "rgb8", image.rgb.shape[1] * 3),
                    (depth, image.depth_mm, "16UC1", image.depth_mm.shape[1] * 2),
                ):
                    msg.header.stamp, msg.header.frame_id = stamp, info.header.frame_id
                    msg.height, msg.width = array.shape[:2]
                    msg.encoding, msg.is_bigendian, msg.step = encoding, 0, step
                    msg.data = np.ascontiguousarray(array).tobytes()
                one.rgb, one.depth = rgb, depth
                images.rgbd_images.append(one)
            return images

        def _scan_msg(self, frame, stamp):
            msg = LaserScan()
            msg.header.stamp, msg.header.frame_id = stamp, "laser"
            msg.angle_min = float(frame.angle_min_rad)
            msg.angle_increment = float(frame.angle_increment_rad)
            msg.angle_max = float(
                frame.angle_min_rad
                + frame.angle_increment_rad * (len(frame.ranges_m) - 1)
            )
            msg.range_min, msg.range_max = (
                float(frame.range_min_m),
                float(frame.range_max_m),
            )
            msg.ranges = frame.ranges_m.astype(float).tolist()
            return msg

        def _wait(self, store, key):
            deadline = time.monotonic() + self.reply_timeout_s
            with self.cond:
                while key not in store:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        return None
                    self.cond.wait(remaining)
                return store.pop(key)

        def _front(self, stage, key, frame, stamp):
            """Run one front end on this frame; returns its status."""
            info = self._wait(self.odom_infos[stage], key)
            if info is None:
                raise RuntimeError(f"{stage} front end gave no output for stamp {key}")
            odom = self._wait(self.odoms[stage], key) if not info.lost else None
            if info.lost or odom is None:
                return "lost"
            pose = odom.pose.pose
            self.front_pose[stage] = (
                pose.position.x,
                pose.position.y,
                yaw_of(pose.orientation),
            )
            return "ok"

        def process(self, frame: slam_link.Frame) -> slam_link.Reply:
            started = time.monotonic()
            self.last_frame_id = frame.scan_id
            stamp = ros_time(frame.stamp_s)
            key = msg_ns(stamp)
            self.clock_pub.publish(Clock(clock=stamp))
            if self.chain.wheel:
                self.tf_pub.publish(
                    TFMessage(
                        transforms=[
                            planar_tf(stamp, "odom", "base_link", frame.odom_from_base)
                        ]
                    )
                )
                odom = Odometry()
                odom.header.stamp, odom.header.frame_id = stamp, "odom"
                odom.child_frame_id = "base_link"
                x, y, yaw = frame.odom_from_base
                odom.pose.pose.position.x, odom.pose.pose.position.y = (
                    float(x),
                    float(y),
                )
                odom.pose.pose.orientation.z = math.sin(yaw / 2)
                odom.pose.pose.orientation.w = math.cos(yaw / 2)
                self.odom_pub.publish(odom)
            has_rgbd = any(image.camera in self.camera_infos for image in frame.images)
            has_scan = frame.ranges_m is not None
            statuses, self.front_pose = {}, {}
            rgbd_msg = (
                self._rgbd_msg(frame, stamp) if has_rgbd and self.uses_rgbd else None
            )
            scan_msg = (
                self._scan_msg(frame, stamp) if has_scan and self.uses_scan else None
            )
            for stage in self.stages:
                fed = rgbd_msg if stage == "vo" else scan_msg
                if fed is None:
                    statuses[stage] = "no_input"
                else:
                    (self.rgbd_pub if stage == "vo" else self.scan_pub).publish(fed)
                    statuses[stage] = self._front(stage, key, frame, stamp)
            top_from_base = self.chain.update(
                {stage: self.front_pose.get(stage) for stage in self.stages},
                frame.odom_from_base,
            )
            below = "odom" if self.chain.wheel else "base_link"
            fused_tf = self.chain.top_from_below(frame.odom_from_base)
            self.tf_pub.publish(
                TFMessage(transforms=[planar_tf(stamp, "fused", below, fused_tf)])
            )
            available = ("rgbd" not in self.back or rgbd_msg is not None) and (
                "scan" not in self.back or has_scan
            )
            kind = (
                self.gate.classify(frame.stamp_s, top_from_base)
                if available
                else "local"
            )
            graph_status = None
            if kind != "local":
                if "rgbd" in self.back:
                    self.key_rgbd_pub.publish(rgbd_msg)
                if "scan" in self.back:
                    self.key_scan_pub.publish(scan_msg or self._scan_msg(frame, stamp))
                graph = self._wait(self.graphs, key)
                if graph is None:
                    raise RuntimeError(f"rtabmap gave no mapGraph for stamp {key}")
                t = graph.map_to_odom
                self.chain.map_from_top = (
                    t.translation.x,
                    t.translation.y,
                    yaw_of(t.rotation),
                )
                self.gate.processed(frame.stamp_s, top_from_base)
                graph_status = {"nodes": len(graph.poses_id), "links": len(graph.links)}
            estimate = self.chain.output_from_base(frame.odom_from_base)
            map_from_odom = compose(estimate, invert(frame.odom_from_base))
            elapsed = time.monotonic() - started
            self.records.append(
                {
                    "frame_id": frame.scan_id,
                    "stamp_s": frame.stamp_s,
                    "front": statuses,
                    "fused_from": list(self.chain.fused.last_sources),
                    "front_poses": {k: list(v) for k, v in self.front_pose.items()},
                    "keyframe": kind != "local",
                    "graph": graph_status,
                    "estimate": list(estimate),
                    "odom_from_base": list(frame.odom_from_base),
                    "map_from_odom": list(map_from_odom),
                    "images": [image.camera for image in frame.images],
                    "scan": has_scan,
                    "wall_s": elapsed,
                }
            )
            return slam_link.Reply(
                frame.scan_id,
                frame.stamp_s,
                "processed",
                map_from_odom,
                len(self.records),
                elapsed,
            )

        # --- sources --------------------------------------------------------
        def run_record(self):
            count = len(self.record)
            if self.max_frames > 0:
                count = min(count, self.max_frames)
            for index in range(count):
                self.process(self.record.frame(index))

        def run_socket(self):
            path = Path(self.get_parameter("socket_path").value)
            path.unlink(missing_ok=True)
            server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            server.bind(str(path))
            server.listen(1)
            (self.output_dir / "bridge_ready").write_text("ready\n")
            conn, _ = server.accept()
            with conn:
                while True:
                    try:
                        frame = slam_link.recv_frame(conn)
                    except ConnectionError:
                        break
                    try:
                        reply = self.process(frame)
                    except RuntimeError as exc:
                        self.records.append(
                            {"frame_id": frame.scan_id, "failed": str(exc)}
                        )
                        reply = slam_link.Reply(
                            frame.scan_id,
                            frame.stamp_s,
                            "failed",
                            (0.0, 0.0, 0.0),
                            -1,
                            0.0,
                        )
                        conn.sendall(slam_link.encode_reply(reply))
                        break
                    conn.sendall(slam_link.encode_reply(reply))
            server.close()

        def save(self):
            self.output_dir.mkdir(parents=True, exist_ok=True)
            with self.map_lock:
                if getattr(self, "latest_map", None) is not None:
                    self._keep_map(self.latest_map)
                maps, arrays = list(self.maps), dict(self.map_arrays)
            failed = [r for r in self.records if "failed" in r]
            self.ok = bool(self.records) and not failed
            summary = {
                "ok": self.ok,
                "mode": self.mode,
                "source": self.source_kind,
                "cameras": self.cameras,
                "noise_seed": self.noise_seed,
                "condition": self.condition.name,
                "frames": len(self.records),
                "increment_noise": {k: vars(v) for k, v in INCREMENT_NOISE.items()},
                "keyframes": sum(bool(r.get("keyframe")) for r in self.records),
                "front_status": {
                    stage: {
                        s: sum(r.get("front", {}).get(stage) == s for r in self.records)
                        for s in ("ok", "lost", "no_input")
                    }
                    for stage in self.stages
                },
                "failed": failed,
                "wall_s_median": float(
                    np.median([r["wall_s"] for r in self.records if "wall_s" in r])
                )
                if self.records
                else None,
            }
            (self.output_dir / "bridge_status.json").write_text(
                json.dumps(summary, indent=1)
            )
            (self.output_dir / "bridge_records.json").write_text(
                json.dumps(self.records)
            )
            (self.output_dir / "maps.json").write_text(json.dumps(maps))
            if arrays:
                np.savez_compressed(self.output_dir / "maps.npz", **arrays)
            if self.record is not None:
                np.savez(
                    self.output_dir / "record_truth.npz",
                    stamps_s=self.record.scan_stamps,
                    truth=self.record.truth,
                    odometry=self.record.odom,
                )

    rclpy.init(args=args)
    node = Bridge()
    node.output_dir.mkdir(parents=True, exist_ok=True)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    status = 3
    try:
        node.wait_ready()
        if node.source_kind == "record":
            node.run_record()
        else:
            node.run_socket()
        time.sleep(2.0)  # the last map
    except Exception as exc:  # recorded, then the run fails
        node.records.append({"failed": f"{type(exc).__name__}: {exc}"})
        raise
    finally:
        executor.shutdown()
        spinner.join(timeout=5.0)
        node.save()
        status = 0 if node.ok else 3
        node.destroy_node()
        rclpy.shutdown()
    return status


def _quaternion_xyzw(m: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(m))
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = (
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
            s / 4,
        )
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = (
            s / 4,
            (m[0, 1] + m[1, 0]) / s,
            (m[0, 2] + m[2, 0]) / s,
            (m[2, 1] - m[1, 2]) / s,
        )
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = (
            (m[0, 1] + m[1, 0]) / s,
            s / 4,
            (m[1, 2] + m[2, 1]) / s,
            (m[0, 2] - m[2, 0]) / s,
        )
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = (
            (m[0, 2] + m[2, 0]) / s,
            (m[1, 2] + m[2, 1]) / s,
            s / 4,
            (m[1, 0] - m[0, 1]) / s,
        )
    q = np.asarray(q) / np.linalg.norm(q)
    return tuple(float(v) for v in q)


if __name__ == "__main__":
    raise SystemExit(main())
