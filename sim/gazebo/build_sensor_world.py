"""Generate a static, asset-free synthetic sensor world from explicit settings."""

import argparse
import copy
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


def box(link, name, center, size, color):
    for kind in ("visual", "collision"):
        node = element(link, kind, name=f"{name}_{kind}")
        element(node, "pose", " ".join(map(str, [*center, 0, 0, 0])))
        element(
            element(element(node, "geometry"), "box"), "size", " ".join(map(str, size))
        )
        if kind == "visual":
            material = element(node, "material")
            element(material, "ambient", color)
            element(material, "diffuse", color)


def load_config(path: Path) -> dict:
    config = yaml.safe_load(path.read_text())
    schemas = {
        "camera": {
            "translation_m",
            "optical_quaternion_xyzw",
            "width",
            "height",
            "rate_hz",
            "horizontal_fov_rad",
        },
        "lidar": {"translation_m", "samples", "rate_hz"},
        "reference": {"front_surface_x_m", "left_surface_y_m"},
        "pallet": {"center_x_m"},
    }
    if not isinstance(config, dict) or set(config) != {
        *schemas,
        "source_provenance",
        "urdf",
    }:
        raise ValueError("Unknown or missing config fields")
    if config["source_provenance"] != "synthetic":
        raise ValueError("config must declare synthetic provenance")
    for section, keys in schemas.items():
        if set(config[section]) != keys:
            raise ValueError(f"Invalid config section {section}")
        for value in config[section].values():
            if not np.isfinite(np.asarray(value, dtype=float)).all():
                raise ValueError("config values must be finite")
    # This is one bounded reference experiment; changing its truth requires review.
    if (
        config["camera"]
        != {
            "translation_m": [0.75, 0.0, 0.5],
            "optical_quaternion_xyzw": [-0.5, 0.5, -0.5, 0.5],
            "width": 320,
            "height": 240,
            "rate_hz": 5,
            "horizontal_fov_rad": math.pi / 2,
        }
        or config["lidar"]
        != {"translation_m": [0.75, 0.0, 0.5], "samples": 360, "rate_hz": 5}
        or config["reference"] != {"front_surface_x_m": 3.0, "left_surface_y_m": 2.0}
        or config["pallet"] != {"center_x_m": 2.0}
    ):
        raise ValueError(
            "config differs from the independently validated reference experiment"
        )
    return config


def generate(config_path: Path, output: Path) -> dict:
    """Write SDF, bridge and static transforms; return the source configuration."""
    config = load_config(config_path)
    output.mkdir(parents=True, exist_ok=True)
    root = ET.Element("sdf", version="1.9")
    world = element(root, "world", name="sensor_baseline")
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
    element(scene, "ambient", ".7 .7 .7 1")
    element(scene, "background", ".15 .2 .25 1")
    light = element(world, "light", name="sun", type="directional")
    element(light, "direction", "-1 -.5 -1")
    element(light, "diffuse", ".8 .8 .8 1")
    count = add_urdf_visuals(world, (config_path.parent / config["urdf"]).resolve())
    for name in ("floor", "reference_targets", "synthetic_pallet"):
        model = element(world, "model", name=name)
        element(model, "static", "true")
        link = element(model, "link", name="geometry")
        if name == "floor":
            box(link, "floor", [0, 0, -0.05], [20, 20, 0.1], ".35 .4 .4 1")
        elif name == "reference_targets":
            box(
                link,
                "front_plane",
                [config["reference"]["front_surface_x_m"] + 0.05, 0, 1.5],
                [0.1, 6, 3],
                ".15 .45 .8 1",
            )
            box(
                link,
                "left_plane",
                [0.75, config["reference"]["left_surface_y_m"] + 0.05, 1.5],
                [6, 0.1, 3],
                ".2 .8 .3 1",
            )
        else:
            x = config["pallet"]["center_x_m"]
            box(link, "bottom", [x, 0, 0.025], [0.6, 0.8, 0.05], ".6 .35 .12 1")
            box(link, "top", [x, 0, 0.275], [0.6, 0.8, 0.05], ".7 .45 .18 1")
            for index, y in enumerate([-0.35, 0, 0.35]):
                box(
                    link, f"spacer_{index}", [x, y, 0.15], [0.6, 0.1, 0.2], ".5 .3 .1 1"
                )
    model = element(world, "model", name="synthetic_sensor_rig")
    element(model, "static", "true")
    for kind in ("camera", "lidar"):
        settings = config[kind]
        link = element(model, "link", name=f"{kind}_link")
        element(link, "pose", " ".join(map(str, [*settings["translation_m"], 0, 0, 0])))
        sensor = element(
            link,
            "sensor",
            name=kind,
            type="rgbd_camera" if kind == "camera" else "gpu_lidar",
        )
        element(sensor, "always_on", "true")
        element(sensor, "update_rate", settings["rate_hz"])
        element(sensor, "topic", "/camera" if kind == "camera" else "/scan")
        element(sensor, "gz_frame_id", f"{kind}_link")
        if kind == "camera":
            camera = element(sensor, "camera")
            element(camera, "optical_frame_id", "camera_optical_frame")
            element(camera, "horizontal_fov", settings["horizontal_fov_rad"])
            im = element(camera, "image")
            element(im, "width", settings["width"])
            element(im, "height", settings["height"])
            element(im, "format", "R8G8B8")
            clip = element(camera, "clip")
            element(clip, "near", 0.05)
            element(clip, "far", 10)
        else:
            lidar = element(sensor, "lidar")
            horizontal = element(element(lidar, "scan"), "horizontal")
            for key, value in [
                ("samples", settings["samples"]),
                ("resolution", 1),
                ("min_angle", -math.pi),
                ("max_angle", math.pi),
            ]:
                element(horizontal, key, value)
            limits = element(lidar, "range")
            for key, value in [("min", 0.05), ("max", 10), ("resolution", 0.01)]:
                element(limits, key, value)
    ET.indent(root)
    ET.ElementTree(root).write(
        output / "sensor_world.sdf", encoding="utf-8", xml_declaration=True
    )
    mappings = []
    for topic, ros_type, gz_type in [
        ("/camera/image", "sensor_msgs/msg/Image", "gz.msgs.Image"),
        ("/camera/depth_image", "sensor_msgs/msg/Image", "gz.msgs.Image"),
        ("/camera/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
        ("/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
        ("/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock"),
    ]:
        mappings.append(
            dict(
                ros_topic_name=topic,
                gz_topic_name=topic,
                ros_type_name=ros_type,
                gz_type_name=gz_type,
                direction="GZ_TO_ROS",
            )
        )
    (output / "bridge.yaml").write_text(yaml.safe_dump(mappings))
    transforms = {
        "source_provenance": "synthetic",
        "transforms": [
            {
                "parent": "base_link",
                "child": "camera_optical_frame",
                "translation_m": config["camera"]["translation_m"],
                "quaternion_xyzw": config["camera"]["optical_quaternion_xyzw"],
            },
            {
                "parent": "base_link",
                "child": "lidar_link",
                "translation_m": config["lidar"]["translation_m"],
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
        ],
    }
    (output / "transforms.yaml").write_text(yaml.safe_dump(transforms))
    (output / "scene_config.yaml").write_text(yaml.safe_dump(copy.deepcopy(config)))
    return {"visual_count": count, "source_provenance": "synthetic"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=Path(__file__).with_name("scene_config.yaml")
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    generate(args.config, args.output)
