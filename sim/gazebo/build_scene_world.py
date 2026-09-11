"""Build one static RGB-D world from the fixed synthetic scene catalogue."""

import argparse
import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


def _load_parts():
    spec = importlib.util.spec_from_file_location(
        "forklift_sdf_parts", Path(__file__).with_name("sdf_parts.py")
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sdf_parts = _load_parts()


def _require_keys(obj, keys):
    if not isinstance(obj, dict) or set(obj) != set(keys):
        raise ValueError("unknown or missing catalogue fields")


def _finite(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("catalogue geometry must be finite numbers")
    return value


def _vector(value, length):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError("invalid catalogue vector")
    for v in value:
        _finite(v)


def _validate_presets(presets):
    """Distractor geometry travels in the catalogue header, the single source of truth."""
    if not isinstance(presets, dict) or not presets:
        raise ValueError("catalogue distractor_presets must be a nonempty mapping")
    for name, preset in presets.items():
        if not isinstance(name, str) or not name:
            raise ValueError("invalid distractor preset name")
        _require_keys(preset, {"center_m", "size_m", "color"})
        _vector(preset["center_m"], 3)
        _vector(preset["size_m"], 3)
        if any(v <= 0 for v in preset["size_m"]) or not isinstance(
            preset["color"], str
        ):
            raise ValueError("invalid distractor preset geometry")


def _validate_scene(scene, presets):
    _require_keys(
        scene,
        {
            "scene_id",
            "split",
            "category",
            "pallet",
            "lookalike",
            "occluder",
            "distractors",
            "lighting",
            "surfaces",
            "visibility",
            "ground_truth",
        },
    )
    if not isinstance(scene["scene_id"], str) or not scene["scene_id"]:
        raise ValueError("invalid scene ID")
    category = scene["category"]
    if category not in (
        "positive",
        "occluded",
        "negative_no_pallet",
        "negative_lookalike",
    ):
        raise ValueError("unknown scene category")
    if scene["split"] not in ("dev", "eval"):
        raise ValueError("unknown scene split")
    for field, present in (
        ("pallet", category in ("positive", "occluded")),
        ("lookalike", category == "negative_lookalike"),
    ):
        target = scene[field]
        if not present:
            if target is not None:
                raise ValueError(f"{field} is incompatible with category")
            continue
        keys = {"x_m", "y_m", "yaw_rad"}
        if field == "pallet":
            keys.add("opening_width_m")
        _require_keys(target, keys)
        for value in target.values():
            _finite(value)
        if not (
            2 <= target["x_m"] <= 4
            and -1 <= target["y_m"] <= 1
            and -0.52 <= target["yaw_rad"] <= 0.52
        ):
            raise ValueError("target pose outside approved range")
        if field == "pallet" and not 0.20 <= target["opening_width_m"] <= 0.28:
            raise ValueError("opening width outside approved range")
    occ = scene["occluder"]
    if category == "occluded":
        _require_keys(occ, {"side", "gap_m", "fraction", "center_m", "size_m", "color"})
        _vector(occ["center_m"], 3)
        _vector(occ["size_m"], 3)
        if (
            occ["side"] not in ("left", "right")
            or occ["center_m"][0] < 1.05
            or not 0.3 <= _finite(occ["gap_m"]) <= 0.6
            or not 0.2 <= _finite(occ["fraction"]) <= 0.6
            or any(v <= 0 for v in occ["size_m"])
        ):
            raise ValueError("invalid occluder geometry")
    elif occ is not None:
        raise ValueError("occluder is incompatible with category")
    names = scene["distractors"]
    if (
        not isinstance(names, list)
        or len(names) > 2
        or any(not isinstance(name, str) or name not in presets for name in names)
        or len(set(names)) != len(names)
    ):
        raise ValueError("invalid distractor presets")
    _require_keys(scene["lighting"], {"name", "direction", "diffuse"})
    _vector(scene["lighting"]["direction"], 3)
    if _finite(scene["lighting"]["diffuse"]) not in (0.5, 0.9):
        raise ValueError("invalid light diffuse intensity")
    _require_keys(scene["surfaces"], {"floor", "background"})


def load_catalogue(path: Path) -> dict:
    """Load the bounded v1 geometry contract without importing forklift_core."""
    catalogue = yaml.safe_load(path.read_text())
    _require_keys(
        catalogue,
        {
            "format_version",
            "catalogue_version",
            "seed",
            "generator",
            "source_provenance",
            "camera",
            "pallet",
            "ranges",
            "distractor_presets",
            "scenes",
        },
    )
    if (
        type(catalogue["format_version"]) is not int
        or catalogue["format_version"] != 1
        or catalogue["catalogue_version"] != "v1"
        or catalogue["source_provenance"] != "synthetic"
    ):
        raise ValueError("unsupported catalogue format, version or provenance")
    if catalogue["camera"] != {
        "width": 640,
        "height": 480,
        "horizontal_fov_rad": 1.204,
        "rate_hz": 5,
        "translation_m": [0.75, 0.0, 0.5],
        "optical_quaternion_xyzw": [-0.5, 0.5, -0.5, 0.5],
    }:
        raise ValueError("camera differs from the approved synthetic rig")
    if catalogue["pallet"] != {
        "depth_m": 0.6,
        "width_m": 0.8,
        "height_m": 0.30,
        "deck_m": 0.05,
        "center_spacer_m": 0.10,
        "opening_height_m": 0.20,
    }:
        raise ValueError("pallet dimensions differ from the approved geometry")
    if not isinstance(catalogue["scenes"], list) or not catalogue["scenes"]:
        raise ValueError("catalogue must contain scenes")
    _validate_presets(catalogue["distractor_presets"])
    seen = set()
    for scene in catalogue["scenes"]:
        _validate_scene(scene, catalogue["distractor_presets"])
        if scene["scene_id"] in seen:
            raise ValueError("duplicate scene ID")
        seen.add(scene["scene_id"])
    return catalogue


def scene_entry(catalogue: dict, scene_id: str) -> dict:
    """Return a uniquely identified scene; unknown or duplicate IDs raise ValueError."""
    matches = [scene for scene in catalogue["scenes"] if scene["scene_id"] == scene_id]
    if len(matches) != 1:
        raise ValueError(f"scene ID must identify exactly one entry: {scene_id}")
    return matches[0]


def _static_model(world, name, center, yaw=0):
    model = sdf_parts.element(world, "model", name=name)
    sdf_parts.element(model, "static", "true")
    sdf_parts.element(model, "pose", " ".join(map(str, [*center, 0, 0, yaw])))
    return sdf_parts.element(model, "link", name="geometry")


def generate_scene(catalogue_path: Path, scene_id: str, output: Path) -> dict:
    """Write a static SDF, camera bridge/TF and the selected catalogue snapshot."""
    catalogue = load_catalogue(catalogue_path)
    scene = scene_entry(catalogue, scene_id)
    output.mkdir(parents=True, exist_ok=True)
    root = ET.Element("sdf", version="1.9")
    diffuse = scene["lighting"]["diffuse"]
    world = sdf_parts.add_world_skeleton(
        root,
        "scene_world",
        ".7 .7 .7 1",
        scene["surfaces"]["background"],
        " ".join(map(str, scene["lighting"]["direction"])),
        f"{diffuse} {diffuse} {diffuse} 1",
    )
    floor = _static_model(world, "floor", [0, 0, 0])
    sdf_parts.box(
        floor, "floor", [0, 0, -0.05], [20, 20, 0.1], scene["surfaces"]["floor"]
    )
    count = sdf_parts.add_urdf_visuals(
        world,
        Path(__file__).resolve().parents[1] / "models/dls08_provisional/forklift.urdf",
    )
    pallet = scene["pallet"]
    if pallet is not None:
        link = _static_model(
            world,
            "synthetic_pallet",
            [pallet["x_m"], pallet["y_m"], 0],
            pallet["yaw_rad"],
        )
        sdf_parts.box(link, "bottom", [0, 0, 0.025], [0.6, 0.8, 0.05], ".6 .35 .12 1")
        sdf_parts.box(link, "top", [0, 0, 0.275], [0.6, 0.8, 0.05], ".7 .45 .18 1")
        outer_width = (0.8 - 2 * pallet["opening_width_m"] - 0.1) / 2
        for name, y, width in (
            ("spacer_center", 0, 0.1),
            ("spacer_outer_0", -(0.4 - outer_width / 2), outer_width),
            ("spacer_outer_1", 0.4 - outer_width / 2, outer_width),
        ):
            sdf_parts.box(link, name, [0, y, 0.15], [0.6, width, 0.2], ".5 .3 .1 1")
            # Scene consumers use exact spacer IDs; the legacy helper keeps its suffix.
            link.find(f"collision[@name='{name}_collision']").set("name", name)
    lookalike = scene["lookalike"]
    if lookalike is not None:
        link = _static_model(
            world,
            "lookalike",
            [lookalike["x_m"], lookalike["y_m"], 0],
            lookalike["yaw_rad"],
        )
        sdf_parts.box(link, "solid", [0, 0, 0.15], [0.6, 0.8, 0.30], ".6 .35 .12 1")
    occ = scene["occluder"]
    if occ is not None:
        link = _static_model(world, "occluder", occ["center_m"], pallet["yaw_rad"])
        sdf_parts.box(link, "occluder", [0, 0, 0], occ["size_m"], occ["color"])
    for name in scene["distractors"]:
        preset = catalogue["distractor_presets"][name]
        link = _static_model(world, f"distractor_{name}", preset["center_m"])
        sdf_parts.box(link, name, [0, 0, 0], preset["size_m"], preset["color"])
    rig = sdf_parts.element(world, "model", name="synthetic_sensor_rig")
    sdf_parts.element(rig, "static", "true")
    camera = catalogue["camera"]
    sdf_parts.add_rgbd_camera(
        rig,
        camera["translation_m"],
        camera["width"],
        camera["height"],
        camera["horizontal_fov_rad"],
        camera["rate_hz"],
    )
    ET.indent(root)
    ET.ElementTree(root).write(
        output / "scene_world.sdf", encoding="utf-8", xml_declaration=True
    )
    sdf_parts.write_bridge(
        output / "bridge.yaml",
        [
            ("/camera/image", "sensor_msgs/msg/Image", "gz.msgs.Image"),
            ("/camera/depth_image", "sensor_msgs/msg/Image", "gz.msgs.Image"),
            ("/camera/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
            ("/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock"),
        ],
    )
    sdf_parts.write_transforms(
        output / "transforms.yaml",
        [
            {
                "parent": "base_link",
                "child": "camera_optical_frame",
                "translation_m": camera["translation_m"],
                "quaternion_xyzw": camera["optical_quaternion_xyzw"],
            }
        ],
    )
    (output / "scene.yaml").write_text(
        yaml.safe_dump(
            {
                **scene,
                "catalogue_version": catalogue["catalogue_version"],
                "camera": camera,
            },
            sort_keys=False,
            allow_unicode=True,
        )
    )
    return {
        "scene_id": scene_id,
        "category": scene["category"],
        "visual_count": count,
        "source_provenance": "synthetic",
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalogue", type=Path, required=True)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    generate_scene(args.catalogue, args.scene, args.output)
