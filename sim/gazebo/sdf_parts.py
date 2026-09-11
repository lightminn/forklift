"""Shared SDF primitives and synthetic sensor settings; no ROS or core imports."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml


def element(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


def rotation_rpy(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def origin_matrix(node):
    result = np.eye(4)
    if node is not None:
        result[:3, 3] = np.fromstring(node.get("xyz", "0 0 0"), sep=" ")
        result[:3, :3] = rotation_rpy(np.fromstring(node.get("rpy", "0 0 0"), sep=" "))
    return result


def pose_text(matrix):
    r = matrix[:3, :3]
    pitch = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    roll = math.atan2(r[2, 1], r[2, 2])
    yaw = math.atan2(r[1, 0], r[0, 0])
    return " ".join(str(float(v)) for v in [*matrix[:3, 3], roll, pitch, yaw])


def add_urdf_visuals(world, path):
    robot = ET.parse(path).getroot()
    poses = {"base_link": np.eye(4)}
    pending = list(robot.findall("joint"))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find("parent").get("link")
            if parent in poses:
                poses[joint.find("child").get("link")] = poses[parent] @ origin_matrix(
                    joint.find("origin")
                )
                pending.remove(joint)
                progressed = True
        if not progressed:
            raise ValueError("URDF has disconnected joints")
    model = element(world, "model", name="provisional_forklift_visuals")
    element(model, "static", "true")
    link = element(model, "link", name="frozen_visuals")
    count = 0
    for source in robot.findall("link"):
        for visual in source.findall("visual"):
            out = element(link, "visual", name=f"visual_{count}")
            count += 1
            element(
                out,
                "pose",
                pose_text(
                    poses[source.get("name")] @ origin_matrix(visual.find("origin"))
                ),
            )
            geometry = element(out, "geometry")
            shape = list(visual.find("geometry"))[0]
            if shape.tag not in {"box", "cylinder", "sphere"}:
                raise ValueError("Only local primitive URDF visuals are supported")
            converted = element(geometry, shape.tag)
            for key, value in shape.attrib.items():
                element(converted, key, value)
            color = visual.find("material/color")
            if color is not None:
                material = element(out, "material")
                element(material, "ambient", color.get("rgba"))
                element(material, "diffuse", color.get("rgba"))
    return count


def box(link, name, center, size, color, yaw=0):
    for kind in ("visual", "collision"):
        node = element(link, kind, name=f"{name}_{kind}")
        element(node, "pose", " ".join(map(str, [*center, 0, 0, yaw])))
        element(
            element(element(node, "geometry"), "box"), "size", " ".join(map(str, size))
        )
        if kind == "visual":
            material = element(node, "material")
            element(material, "ambient", color)
            element(material, "diffuse", color)


def add_world_skeleton(
    root: ET.Element,
    name: str,
    ambient: str,
    background: str,
    light_direction: str,
    light_diffuse: str,
) -> ET.Element:
    world = element(root, "world", name=name)
    physics = element(world, "physics", name="fixed_step", type="ignored")
    element(physics, "max_step_size", 0.01)
    element(physics, "real_time_factor", 1.0)
    for filename, name in [
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
        ("gz-sim-sensors-system", "gz::sim::systems::Sensors"),
    ]:
        plugin = element(world, "plugin", filename=filename, name=name)
        if "Sensors" in name:
            element(plugin, "render_engine", "ogre2")
    scene = element(world, "scene")
    element(scene, "ambient", ambient)
    element(scene, "background", background)
    light = element(world, "light", name="sun", type="directional")
    element(light, "direction", light_direction)
    element(light, "diffuse", light_diffuse)
    return world


def add_rgbd_camera(
    model: ET.Element,
    translation_m: list,
    width: int,
    height: int,
    horizontal_fov_rad: float,
    rate_hz: float,
) -> None:
    link = element(model, "link", name="camera_link")
    element(link, "pose", " ".join(map(str, [*translation_m, 0, 0, 0])))
    sensor = element(
        link,
        "sensor",
        name="camera",
        type="rgbd_camera",
    )
    element(sensor, "always_on", "true")
    element(sensor, "update_rate", rate_hz)
    element(sensor, "topic", "/camera")
    element(sensor, "gz_frame_id", "camera_link")
    camera = element(sensor, "camera")
    element(camera, "optical_frame_id", "camera_optical_frame")
    element(camera, "horizontal_fov", horizontal_fov_rad)
    im = element(camera, "image")
    element(im, "width", width)
    element(im, "height", height)
    element(im, "format", "R8G8B8")
    clip = element(camera, "clip")
    element(clip, "near", 0.05)
    element(clip, "far", 10)


def add_gpu_lidar(
    model: ET.Element,
    translation_m: list,
    samples: int,
    rate_hz: float,
) -> None:
    link = element(model, "link", name="lidar_link")
    element(link, "pose", " ".join(map(str, [*translation_m, 0, 0, 0])))
    sensor = element(
        link,
        "sensor",
        name="lidar",
        type="gpu_lidar",
    )
    element(sensor, "always_on", "true")
    element(sensor, "update_rate", rate_hz)
    element(sensor, "topic", "/scan")
    element(sensor, "gz_frame_id", "lidar_link")
    lidar = element(sensor, "lidar")
    horizontal = element(element(lidar, "scan"), "horizontal")
    for key, value in [
        ("samples", samples),
        ("resolution", 1),
        ("min_angle", -math.pi),
        ("max_angle", math.pi),
    ]:
        element(horizontal, key, value)
    limits = element(lidar, "range")
    for key, value in [("min", 0.05), ("max", 10), ("resolution", 0.01)]:
        element(limits, key, value)


def write_bridge(path: Path, topics: list[tuple[str, str, str]]) -> None:
    mappings = []
    for topic, ros_type, gz_type in topics:
        mappings.append(
            dict(
                ros_topic_name=topic,
                gz_topic_name=topic,
                ros_type_name=ros_type,
                gz_type_name=gz_type,
                direction="GZ_TO_ROS",
            )
        )
    path.write_text(yaml.safe_dump(mappings))


def write_transforms(path: Path, transforms: list[dict]) -> None:
    path.write_text(
        yaml.safe_dump({"source_provenance": "synthetic", "transforms": transforms})
    )
