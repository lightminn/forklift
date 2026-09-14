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


# Camera mounts a catalogue may declare. The mount is NOT decided: the brief
# puts the camera on the mast, where it rises with the forks, and no position
# has been measured on a delivered chassis. It is listed rather than hardcoded
# so a candidate mount can be captured and compared, while an accidental drift
# in a catalogue header is still refused.
#
# What the mount decides, measured: the near limit is the camera's vertical
# field of view, not any detector gate. At the baseline mount nothing below
# 1.90 m is detectable at any pose, and no parameter recovers it. See C-19 of
# docs/plans/2026-09-13-pocket-evidence-restructure.md.
_SENSOR = {"width": 640, "height": 480, "horizontal_fov_rad": 1.204, "rate_hz": 5}
_LEVEL = [-0.5, 0.5, -0.5, 0.5]
APPROVED_CAMERAS = {
    # The mount every 2026-09 measurement was taken at. Provisional.
    "baseline_0p50": {
        **_SENSOR,
        "translation_m": [0.75, 0.0, 0.5],
        "optical_quaternion_xyzw": _LEVEL,
    },
    # Candidate: lowered. Measured to reach 1.6-1.8 m when range_min_m is also
    # lowered; below about 1.56 m the horizontal field of view cuts the pallet
    # out of frame and no mount at this focal length recovers it.
    "low_0p27": {
        **_SENSOR,
        "translation_m": [0.75, 0.0, 0.27],
        "optical_quaternion_xyzw": _LEVEL,
    },
    # Candidate: mast height, which is where the brief actually puts it.
    "mast_0p90": {
        **_SENSOR,
        "translation_m": [0.75, 0.0, 0.90],
        "optical_quaternion_xyzw": _LEVEL,
    },
}


# Header geometry per catalogue version. v1 keeps its symmetric `deck_m`; a real
# pallet cannot use it. EPAL 6 is 0.022 bottom + 0.078 opening + 0.044 above,
# and no single deck thickness closes that -- 0.022*2 + 0.078 is 0.122 and
# 0.044*2 + 0.078 is 0.166, neither of which is its 0.144 height. Splitting the
# two decks is a schema change, not a swapped dictionary.
APPROVED_PALLETS = {
    "v1": {
        "depth_m": 0.6,
        "width_m": 0.8,
        "height_m": 0.30,
        "deck_m": 0.05,
        "center_spacer_m": 0.10,
        "opening_height_m": 0.20,
    },
    "epal6": {
        "depth_m": 0.6,
        "width_m": 0.8,
        "height_m": 0.144,
        "deck_bottom_m": 0.022,
        "deck_top_m": 0.044,
        "center_spacer_m": 0.145,
        "opening_height_m": 0.078,
    },
}
# Catalogue versions whose pallet is emitted from the committed URDF rather than
# from the five boxes v1 hardcodes.
_URDF_PALLETS = {"epal6": "models/epal6_pallet/pallet.urdf"}


def _emit_urdf_pallet(link, relative_urdf: str) -> None:
    """Emit the committed pallet as boxes read from its URDF.

    The URDF is the one geometry source the model exporters and the clearance
    consumers already share, so reading it here keeps the rendered scene and
    the measured article from drifting apart. v1's five boxes stay hardcoded
    because the archived catalogue and its tests describe those exact names.
    """
    urdf = Path(__file__).resolve().parents[1] / relative_urdf
    root = ET.parse(urdf).getroot()
    for visual in root.findall(".//visual"):
        name = visual.get("name")
        origin = [float(v) for v in visual.find("origin").get("xyz").split()]
        size = [float(v) for v in visual.find("geometry/box").get("size").split()]
        # Decks and blocks in different colours: one colour across all 22 leaves
        # no deck-to-block contrast in the RGB image.
        if name.startswith("bottom_board"):
            colour = ".6 .35 .12 1"
        elif name.startswith("top_board"):
            colour = ".7 .45 .18 1"
        elif name.startswith("stringer"):
            colour = ".66 .42 .16 1"
        else:
            colour = ".5 .3 .1 1"
        sdf_parts.box(link, name, origin, size, colour)
        # Scene consumers address collisions by the plain name, as v1 does.
        link.find(f"collision[@name='{name}_collision']").set("name", name)


def _emit_lookalike(link, category: str, version: str) -> None:
    """Negative structures, told apart by category rather than by a new key.

    `_require_keys` compares the scene key set for equality, so adding a key
    would reject all 100 committed v1 scenes. The category carries the shape
    instead, and the placement fields stay the ones `lookalike` already has.
    """
    if category == "negative_block_row":
        # Nine bare blocks: two openings and no deck over either. This is the
        # negative the detector must refuse on upper-deck evidence alone.
        geometry = APPROVED_PALLETS[version]
        opening = geometry["opening_height_m"]
        bottom = geometry.get("deck_bottom_m", geometry.get("deck_m", 0.0))
        spacer = geometry["center_spacer_m"]
        outer = (geometry["width_m"] - spacer - 2 * _opening_width(geometry)) / 2
        depth = geometry["depth_m"]
        for row, x in enumerate((-depth / 3, 0.0, depth / 3)):
            for index, (y, width) in enumerate(
                (
                    (-(geometry["width_m"] - outer) / 2, outer),
                    (0.0, spacer),
                    ((geometry["width_m"] - outer) / 2, outer),
                )
            ):
                sdf_parts.box(
                    link,
                    f"block_x{row}_y{index}",
                    [x, y, bottom + opening / 2],
                    [depth / 3, width, opening],
                    ".5 .3 .1 1",
                )
        return
    # negative_lookalike keeps v1's solid slab: the same envelope with no
    # openings at all.
    sdf_parts.box(link, "solid", [0, 0, 0.15], [0.6, 0.8, 0.30], ".6 .35 .12 1")


def _opening_width(geometry) -> float:
    return (geometry["width_m"] - geometry["center_spacer_m"] - 2 * _outer_block(
        geometry
    )) / 2


def _outer_block(geometry) -> float:
    """Outer column width, from the approved header alone.

    EPAL 6's columns are 100/145/100 across the face; v1 has no columns of its
    own, so its lookalike never reaches here.
    """
    return 0.100 if geometry["opening_height_m"] < 0.15 else 0.0


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
        # Nine bare blocks: two openings and no deck over either. A separate
        # category rather than a variant field, because `_require_keys` compares
        # the scene key set for equality and a new key would reject all 100
        # committed v1 scenes.
        "negative_block_row",
    ):
        raise ValueError("unknown scene category")
    if scene["split"] not in ("dev", "eval"):
        raise ValueError("unknown scene split")
    for field, present in (
        ("pallet", category in ("positive", "occluded")),
        ("lookalike", category in ("negative_lookalike", "negative_block_row")),
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
        or catalogue["catalogue_version"] not in APPROVED_PALLETS
        or catalogue["source_provenance"] != "synthetic"
    ):
        raise ValueError("unsupported catalogue format, version or provenance")
    if catalogue["camera"] not in APPROVED_CAMERAS.values():
        raise ValueError("camera differs from every approved synthetic rig")
    if catalogue["pallet"] != APPROVED_PALLETS[catalogue["catalogue_version"]]:
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
    version = catalogue["catalogue_version"]
    if pallet is not None:
        link = _static_model(
            world,
            "synthetic_pallet",
            [pallet["x_m"], pallet["y_m"], 0],
            pallet["yaw_rad"],
        )
        if version in _URDF_PALLETS:
            _emit_urdf_pallet(link, _URDF_PALLETS[version])
        else:
            sdf_parts.box(
                link, "bottom", [0, 0, 0.025], [0.6, 0.8, 0.05], ".6 .35 .12 1"
            )
            sdf_parts.box(
                link, "top", [0, 0, 0.275], [0.6, 0.8, 0.05], ".7 .45 .18 1"
            )
            outer_width = (0.8 - 2 * pallet["opening_width_m"] - 0.1) / 2
            for name, y, width in (
                ("spacer_center", 0, 0.1),
                ("spacer_outer_0", -(0.4 - outer_width / 2), outer_width),
                ("spacer_outer_1", 0.4 - outer_width / 2, outer_width),
            ):
                sdf_parts.box(link, name, [0, y, 0.15], [0.6, width, 0.2], ".5 .3 .1 1")
                # Scene consumers use exact spacer IDs; the legacy helper keeps
                # its suffix.
                link.find(f"collision[@name='{name}_collision']").set("name", name)
    lookalike = scene["lookalike"]
    if lookalike is not None:
        link = _static_model(
            world,
            "lookalike",
            [lookalike["x_m"], lookalike["y_m"], 0],
            lookalike["yaw_rad"],
        )
        _emit_lookalike(link, scene["category"], version)
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
