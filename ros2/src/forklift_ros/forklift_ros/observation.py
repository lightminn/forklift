"""ROS-independent validation of this synthetic sensor reference experiment.

Message objects follow ROS fields; no measurements are generated here. Expected
planes are independently specified: x=3 m, y=2 m. The off-axis pixel (200,80)
therefore maps to (3,-.5625,1.0625) with the synthetic mounting and pinhole grid.
"""

import math

import numpy as np

TOPICS = {
    "rgb": "/camera/image",
    "depth": "/camera/depth_image",
    "camera_info": "/camera/camera_info",
    "scan": "/scan",
    "clock": "/clock",
    "tf_static": "/tf_static",
}


def stamp_ns(stamp) -> int:
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("Invalid simulation timestamp")
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def quaternion_rotation(q) -> np.ndarray:
    xyzw = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(xyzw).all() or not np.isclose(
        np.linalg.norm(xyzw), 1, atol=1e-6
    ):
        raise ValueError("Invalid transform quaternion")
    x, y, z, w = xyzw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class ObservationWindow:
    """A fresh instance holds one live, stored-bag or replay observation phase."""

    def __init__(self, duration_s: float):
        if not math.isfinite(duration_s) or duration_s < 30:
            raise ValueError("duration must be finite and >=30 simulated seconds")
        self.duration_s = duration_s
        self.stamps = {name: [] for name in TOPICS if name != "tf_static"}
        self.transforms = {}
        self.samples = {}
        self.observations = {}

    def accept(self, kind: str, msg) -> None:
        """Reject malformed metadata, geometry and time order immediately."""
        if kind == "tf_static":
            self._tf(msg)
            return
        if kind not in self.stamps:
            raise ValueError(f"Unknown stream {kind}")
        stamp = stamp_ns(msg.clock if kind == "clock" else msg.header.stamp)
        if self.stamps[kind] and stamp <= self.stamps[kind][-1]:
            raise ValueError(f"{kind} timestamps must be strictly monotonic")
        if kind != "clock":
            expected = "lidar_link" if kind == "scan" else "camera_optical_frame"
            if msg.header.frame_id != expected:
                raise ValueError(
                    f"{kind} frame: expected {expected}, got {msg.header.frame_id}"
                )
        if kind in {"rgb", "depth"}:
            self._image(kind, msg)
        elif kind == "camera_info":
            self._info(msg)
        elif kind == "scan":
            self._scan(msg)
        self.stamps[kind].append(stamp)

    def _image(self, kind, msg):
        encoding, stride = ("rgb8", 960) if kind == "rgb" else ("32FC1", 1280)
        if (msg.width, msg.height, msg.encoding, msg.step) != (
            320,
            240,
            encoding,
            stride,
        ):
            raise ValueError(f"{kind} image shape/encoding/step mismatch")
        if msg.is_bigendian not in (0, 1) or len(msg.data) != stride * 240:
            raise ValueError(f"{kind} image payload/endian mismatch")
        if kind == "rgb":
            data = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(240, 320, 3)
            if int(np.ptp(data)) < 2:
                raise ValueError("RGB render is blank/uniform")
        else:
            dtype = ">f4" if msg.is_bigendian else "<f4"
            data = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(240, 320)
            roi = data[78:83, 198:203]
            if not np.isfinite(roi).all() or not np.allclose(
                roi, 2.25, atol=0.025, rtol=0
            ):
                raise ValueError("Known front-plane depth ROI is not 2.25 m")
            if (
                not np.isfinite(data[120, 160])
                or abs(float(data[120, 160]) - 2.25) > 0.025
            ):
                raise ValueError("Known central depth is not 2.25 m")
            self.observations["depth_roi_m"] = float(np.median(roi))
        self.samples[kind] = data.copy()

    def _info(self, msg):
        k = np.asarray(msg.k).reshape(3, 3)
        expected = np.array([[160.0, 0.0, 160.0], [0.0, 160.0, 120.0], [0.0, 0.0, 1.0]])
        if (
            (msg.width, msg.height) != (320, 240)
            or not np.isfinite(k).all()
            or not np.allclose(k, expected, atol=0.6, rtol=0)
        ):
            raise ValueError("CameraInfo calibration does not match the synthetic grid")
        p = np.asarray(msg.p).reshape(3, 4)
        if (
            not np.allclose(p[:3, :3], k, atol=1e-5, rtol=0)
            or not np.allclose(p[:, 3], 0, atol=1e-8)
            or not np.isfinite(msg.d).all()
            or not np.allclose(msg.d, 0)
        ):
            raise ValueError("CameraInfo must describe an undistorted aligned grid")
        if msg.distortion_model not in {"plumb_bob", ""}:
            raise ValueError("Unsupported CameraInfo distortion model")
        self.samples["camera_info"] = k.copy()

    def _scan(self, msg):
        metadata = np.array(
            [
                msg.angle_min,
                msg.angle_max,
                msg.angle_increment,
                msg.range_min,
                msg.range_max,
                msg.time_increment,
                msg.scan_time,
            ]
        )
        if (
            not np.isfinite(metadata).all()
            or len(msg.ranges) != 360
            or not np.allclose(
                metadata[:5],
                [-math.pi, math.pi, 2 * math.pi / 359, 0.05, 10],
                atol=1e-5,
                rtol=0,
            )
        ):
            raise ValueError("Scan geometry does not match the 360-degree reference")
        ranges = np.asarray(msg.ranges, dtype=float)
        if (
            np.isnan(ranges).any()
            or np.isneginf(ranges).any()
            or (ranges[np.isfinite(ranges)] < msg.range_min).any()
            or (ranges[np.isfinite(ranges)] > msg.range_max + 1e-5).any()
        ):
            raise ValueError("Invalid scan range samples")
        angles = msg.angle_min + np.arange(360) * msg.angle_increment
        front = int(np.argmin(abs(angles)))
        left = int(np.argmin(abs(angles - math.pi / 2)))
        x = ranges[front] * math.cos(angles[front])
        y = ranges[left] * math.sin(angles[left])
        if not np.isfinite([x, y]).all() or not np.allclose(
            [x, y], [2.25, 2.0], atol=0.035, rtol=0
        ):
            raise ValueError("Known scan reference plane distances disagree")
        self.observations.update(
            scan_front_x_m=float(x),
            scan_left_y_m=float(y),
            scan_front_range_m=float(ranges[front]),
            scan_left_range_m=float(ranges[left]),
            scan_left_point_base_m=[
                float(0.75 + ranges[left] * math.cos(angles[left])),
                float(y),
                0.5,
            ],
        )
        self.samples["scan"] = (ranges.copy(), angles)

    def _tf(self, msg):
        expected_rotation = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        for item in msg.transforms:
            if item.child_frame_id not in {"camera_optical_frame", "lidar_link"}:
                continue
            rotation = quaternion_rotation(item.transform.rotation)
            t = item.transform.translation
            translation = np.array([t.x, t.y, t.z])
            target = (
                expected_rotation
                if item.child_frame_id == "camera_optical_frame"
                else np.eye(3)
            )
            if (
                item.header.frame_id != "base_link"
                or not np.allclose(rotation, target, atol=1e-6, rtol=0)
                or not np.allclose(translation, [0.75, 0, 0.5], atol=1e-6, rtol=0)
            ):
                raise ValueError(
                    "Static transform disagrees with independent synthetic mounting"
                )
            self.transforms[item.child_frame_id] = (rotation, translation)

    def finish(self) -> dict:
        """Require complete duration, message density, alignment and TF evidence."""
        missing = [name for name, stamps in self.stamps.items() if not stamps]
        if missing or len(self.transforms) != 2:
            raise ValueError(
                f"missing streams/transforms: {missing}, TF={list(self.transforms)}"
            )
        streams = {}
        for name, stamps in self.stamps.items():
            span = (stamps[-1] - stamps[0]) / 1e9
            if span + 1e-9 < self.duration_s:
                raise ValueError(f"{name} duration {span} < {self.duration_s}")
            minimum = math.ceil(self.duration_s * 5)
            if len(stamps) < minimum:
                raise ValueError(f"{name} count {len(stamps)} < {minimum}")
            if name != "clock" and max(np.diff(stamps)) / 1e9 > 0.61:
                raise ValueError(f"{name} has a truncated/dropped interval")
            streams[name] = {
                "count": len(stamps),
                "first_stamp_ns": stamps[0],
                "last_stamp_ns": stamps[-1],
                "span_s": span,
            }
        common = (
            set(self.stamps["rgb"])
            & set(self.stamps["depth"])
            & set(self.stamps["camera_info"])
        )
        if (
            len(common) < math.ceil(self.duration_s * 5)
            or (max(common) - min(common)) / 1e9 + 1e-9 < self.duration_s
        ):
            raise ValueError(
                "RGB, depth and CameraInfo are not timestamp-aligned for required duration"
            )
        clock = self.stamps["clock"]
        for name in ("rgb", "depth", "camera_info", "scan"):
            if (
                self.stamps[name][0] < clock[0] - 200_000_000
                or self.stamps[name][-1] > clock[-1] + 200_000_000
            ):
                raise ValueError(
                    "Sensor timestamps are outside observed simulation clock"
                )
        k = self.samples["camera_info"]
        z = float(self.samples["depth"][80, 200])
        point = np.array(
            [(200 - k[0, 2]) * z / k[0, 0], (80 - k[1, 2]) * z / k[1, 1], z]
        )
        rotation, translation = self.transforms["camera_optical_frame"]
        base = rotation @ point + translation
        if not np.allclose(base, [3.0, -0.5625, 1.0625], atol=0.025, rtol=0):
            raise ValueError("Off-axis optical to base coordinate check failed")
        self.observations["point_base_m"] = base.tolist()
        self.observations["point_optical_m"] = point.tolist()
        return {
            "passed": True,
            "source_provenance": "synthetic",
            "validation_kind": "sensor_rendering",
            "streams": streams,
            "aligned_camera_count": len(common),
            "transforms": sorted(self.transforms),
            "observations": self.observations.copy(),
        }

    def export_pngs(self, output) -> None:
        """Export received RGB, metric depth color map and base-axis scan view."""
        from PIL import Image, ImageDraw

        Image.fromarray(self.samples["rgb"]).save(output / "rgb.png")
        data = self.samples["depth"]
        valid = np.isfinite(data) & (data > 0)
        scaled = np.zeros(data.shape, dtype=np.uint8)
        scaled[valid] = np.clip(data[valid] / 5 * 255, 0, 255).astype(np.uint8)
        colors = np.stack([scaled, 255 - scaled, np.full_like(scaled, 90)], axis=-1)
        colors[~valid] = 0
        Image.fromarray(colors).save(output / "depth.png")
        image = Image.new("RGB", (640, 640), "white")
        draw = ImageDraw.Draw(image)
        draw.line((320, 0, 320, 640), fill="gray")
        draw.line((0, 320, 640, 320), fill="gray")
        draw.text((325, 10), "+x forward; +y left; 50 px/m", fill="black")
        ranges, angles = self.samples["scan"]
        for r, a in zip(ranges, angles, strict=True):
            if math.isfinite(r):
                u = 320 - r * math.sin(a) * 50
                v = 320 - r * math.cos(a) * 50
                draw.ellipse((u - 2, v - 2, u + 2, v + 2), fill="navy")
        draw.ellipse((316, 316, 324, 324), fill="red")
        image.save(output / "scan.png")


class ExperimentWindow(ObservationWindow):
    """Exclude only a fixed initial [0, 2) simulation-second renderer warmup.

    Gazebo's initial RGB frame can be all black before the first scene render.
    Warmup never slides with arrival time or extends in response to bad data.
    Excluded raw samples remain in the bag and are separately accounted for.
    """

    def __init__(self, duration_s: float):
        super().__init__(duration_s)
        self.warmup_excluded = {name: [] for name in self.stamps}
        self.raw_stamps = {name: [] for name in self.stamps}
        self.static_message_count = 0

    def accept(self, kind: str, msg) -> None:
        if kind == "tf_static":
            super().accept(kind, msg)
            self.static_message_count += 1
            return
        if kind not in self.raw_stamps:
            raise ValueError(f"Unknown stream {kind}")
        timestamp = stamp_ns(msg.clock if kind == "clock" else msg.header.stamp)
        raw = self.raw_stamps[kind]
        if raw and timestamp <= raw[-1]:
            raise ValueError(f"{kind} raw timestamps must be strictly monotonic")
        raw.append(timestamp)
        if timestamp < 2_000_000_000:
            self.warmup_excluded[kind].append(timestamp)
            return
        super().accept(kind, msg)

    def finish(self) -> dict:
        result = super().finish()
        result["warmup"] = {
            "exclude_before_sim_ns": 2_000_000_000,
            "reason": "Fixed initial Gazebo RGB render warmup; raw data retained in bag",
            "excluded": {
                name: {
                    "count": len(stamps),
                    "first_stamp_ns": stamps[0] if stamps else None,
                    "last_stamp_ns": stamps[-1] if stamps else None,
                }
                for name, stamps in self.warmup_excluded.items()
            },
        }
        result["raw_message_counts"] = {
            name: len(stamps) for name, stamps in self.raw_stamps.items()
        }
        result["raw_message_counts"]["tf_static"] = self.static_message_count
        return result


def validate_bag_inventory(
    declared_counts: dict[str, int],
    observed_counts: dict[str, int],
    declared_total: int,
) -> None:
    """Reject missing/extra stored rows even if enough duration remains to pass."""
    if (
        not declared_counts
        or any(count <= 0 for count in declared_counts.values())
        or declared_counts != observed_counts
        or sum(declared_counts.values()) != declared_total
    ):
        raise ValueError(
            "Bag inventory disagrees with declared metadata: "
            f"declared={declared_counts}, read={observed_counts}, total={declared_total}"
        )
