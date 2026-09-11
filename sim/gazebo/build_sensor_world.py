"""Generate a static, asset-free synthetic sensor world from explicit settings."""

import argparse
import copy
import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml


def _load_parts():
    spec = importlib.util.spec_from_file_location(
        "forklift_sdf_parts", Path(__file__).with_name("sdf_parts.py")
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sdf_parts = _load_parts()


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
    world = sdf_parts.add_world_skeleton(
        root, "sensor_baseline", ".7 .7 .7 1", ".15 .2 .25 1", "-1 -.5 -1", ".8 .8 .8 1"
    )
    count = sdf_parts.add_urdf_visuals(
        world, (config_path.parent / config["urdf"]).resolve()
    )
    for name in ("floor", "reference_targets", "synthetic_pallet"):
        model = sdf_parts.element(world, "model", name=name)
        sdf_parts.element(model, "static", "true")
        link = sdf_parts.element(model, "link", name="geometry")
        if name == "floor":
            sdf_parts.box(link, "floor", [0, 0, -0.05], [20, 20, 0.1], ".35 .4 .4 1")
        elif name == "reference_targets":
            sdf_parts.box(
                link,
                "front_plane",
                [config["reference"]["front_surface_x_m"] + 0.05, 0, 1.5],
                [0.1, 6, 3],
                ".15 .45 .8 1",
            )
            sdf_parts.box(
                link,
                "left_plane",
                [0.75, config["reference"]["left_surface_y_m"] + 0.05, 1.5],
                [6, 0.1, 3],
                ".2 .8 .3 1",
            )
        else:
            x = config["pallet"]["center_x_m"]
            sdf_parts.box(
                link, "bottom", [x, 0, 0.025], [0.6, 0.8, 0.05], ".6 .35 .12 1"
            )
            sdf_parts.box(link, "top", [x, 0, 0.275], [0.6, 0.8, 0.05], ".7 .45 .18 1")
            for index, y in enumerate([-0.35, 0, 0.35]):
                sdf_parts.box(
                    link, f"spacer_{index}", [x, y, 0.15], [0.6, 0.1, 0.2], ".5 .3 .1 1"
                )
    model = sdf_parts.element(world, "model", name="synthetic_sensor_rig")
    sdf_parts.element(model, "static", "true")
    camera = config["camera"]
    sdf_parts.add_rgbd_camera(
        model,
        camera["translation_m"],
        camera["width"],
        camera["height"],
        camera["horizontal_fov_rad"],
        camera["rate_hz"],
    )
    lidar = config["lidar"]
    sdf_parts.add_gpu_lidar(
        model, lidar["translation_m"], lidar["samples"], lidar["rate_hz"]
    )
    ET.indent(root)
    ET.ElementTree(root).write(
        output / "sensor_world.sdf", encoding="utf-8", xml_declaration=True
    )
    sdf_parts.write_bridge(
        output / "bridge.yaml",
        [
            ("/camera/image", "sensor_msgs/msg/Image", "gz.msgs.Image"),
            ("/camera/depth_image", "sensor_msgs/msg/Image", "gz.msgs.Image"),
            ("/camera/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
            ("/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
            ("/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock"),
        ],
    )
    sdf_parts.write_transforms(
        output / "transforms.yaml",
        [
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
    )
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
