import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
CATALOGUE = ROOT / "sim/gazebo/scenes/catalogue_v1.yaml"


@pytest.fixture
def builder():
    path = ROOT / "sim/gazebo/build_scene_world.py"
    assert path.exists(), "implementation is missing"
    spec = importlib.util.spec_from_file_location("scene_world", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def scene_of(category):
    data = yaml.safe_load(CATALOGUE.read_text())
    return next(s for s in data["scenes"] if s["category"] == category)


def test_positive_scene_places_a_rotated_pallet_with_two_openings(builder, tmp_path):
    scene = scene_of("positive")
    builder.generate_scene(CATALOGUE, scene["scene_id"], tmp_path)
    world = ET.parse(tmp_path / "scene_world.sdf").getroot()
    pallet = world.find(".//model[@name='synthetic_pallet']")
    pose = [float(v) for v in pallet.findtext("pose").split()]
    assert pose[:2] == pytest.approx([scene["pallet"]["x_m"], scene["pallet"]["y_m"]])
    assert pose[5] == pytest.approx(scene["pallet"]["yaw_rad"])
    assert len(pallet.findall(".//collision")) == 5
    w = scene["pallet"]["opening_width_m"]
    outer = [c for c in pallet.findall(".//collision") if "outer" in c.get("name")]
    assert len(outer) == 2
    s_o = (0.8 - 2 * w - 0.1) / 2
    for collision in outer:
        size = [float(v) for v in collision.findtext("geometry/box/size").split()]
        assert size == pytest.approx([0.6, s_o, 0.2], abs=1e-9)
    # Reconstruct both opening centres from the spacer poses (model-local frame)
    # and compare with the catalogue ground truth via the model pose.
    psi = pose[5]
    a = np.array([math.cos(psi), math.sin(psi), 0.0])
    left_axis = np.array([-math.sin(psi), math.cos(psi), 0.0])
    origin = np.array([pose[0], pose[1], 0.0])
    outer_y = sorted(float(c.findtext("pose").split()[1]) for c in outer)
    for side, sign, edge in (
        ("left", 1, outer_y[1] - s_o / 2),
        ("right", -1, outer_y[0] + s_o / 2),
    ):
        local_y = (edge + sign * 0.05) / 2  # between outer spacer and centre spacer
        expected = origin - 0.3 * a + local_y * left_axis + np.array([0.0, 0.0, 0.15])
        assert list(expected) == pytest.approx(
            scene["ground_truth"][side]["center_m"], abs=1e-6
        )
    camera = world.find(".//sensor[@type='rgbd_camera']")
    assert camera.findtext("camera/image/width") == "640"
    assert camera.findtext("camera/horizontal_fov") == "1.204"
    assert world.find(".//sensor[@type='gpu_lidar']") is None
    assert world.find(".//model[@name='occluder']") is None
    bridge = yaml.safe_load((tmp_path / "bridge.yaml").read_text())
    assert {e["ros_topic_name"] for e in bridge} == {
        "/camera/image",
        "/camera/depth_image",
        "/camera/camera_info",
        "/clock",
    }
    transforms = yaml.safe_load((tmp_path / "transforms.yaml").read_text())[
        "transforms"
    ]
    assert [t["child"] for t in transforms] == ["camera_optical_frame"]
    assert (
        yaml.safe_load((tmp_path / "scene.yaml").read_text())["scene_id"]
        == scene["scene_id"]
    )


def test_occluded_scene_adds_an_occluder_in_front_of_the_named_pocket(
    builder, tmp_path
):
    scene = scene_of("occluded")
    builder.generate_scene(CATALOGUE, scene["scene_id"], tmp_path)
    world = ET.parse(tmp_path / "scene_world.sdf").getroot()
    occluder = world.find(".//model[@name='occluder']")
    pose = [float(v) for v in occluder.findtext("pose").split()]
    assert pose[:3] == pytest.approx(scene["occluder"]["center_m"], abs=1e-4)
    assert pose[5] == pytest.approx(scene["pallet"]["yaw_rad"])


def test_negative_scenes_have_no_pallet_openings(builder, tmp_path):
    for category, model in (
        ("negative_no_pallet", None),
        ("negative_lookalike", "lookalike"),
    ):
        out = tmp_path / category
        builder.generate_scene(CATALOGUE, scene_of(category)["scene_id"], out)
        world = ET.parse(out / "scene_world.sdf").getroot()
        assert world.find(".//model[@name='synthetic_pallet']") is None
        if model:
            assert (
                len(world.find(f".//model[@name='{model}']").findall(".//collision"))
                == 1
            )


def test_lighting_and_surfaces_follow_the_catalogue(builder, tmp_path):
    scene = scene_of("positive")
    builder.generate_scene(CATALOGUE, scene["scene_id"], tmp_path)
    world = ET.parse(tmp_path / "scene_world.sdf").getroot()
    light = world.find(".//light[@name='sun']")
    d = scene["lighting"]["diffuse"]
    assert light.findtext("diffuse") == f"{d} {d} {d} 1"
    assert world.find(".//scene/background").text == scene["surfaces"]["background"]


def test_unknown_scene_id_is_rejected(builder, tmp_path):
    with pytest.raises(ValueError):
        builder.generate_scene(CATALOGUE, "s999", tmp_path)


def test_scene_snapshot_sensor_pose_spacers_and_visual_summary(builder, tmp_path):
    data = yaml.safe_load(CATALOGUE.read_text())
    scene = scene_of("positive")
    summary = builder.generate_scene(CATALOGUE, scene["scene_id"], tmp_path)
    root = ET.parse(tmp_path / "scene_world.sdf").getroot()
    original = ET.parse(ROOT / "sim/models/dls08_provisional/forklift.urdf")
    assert summary == {
        "scene_id": scene["scene_id"],
        "category": "positive",
        "visual_count": len(original.findall(".//visual")),
        "source_provenance": "synthetic",
    }
    assert yaml.safe_load((tmp_path / "scene.yaml").read_text()) == {
        **scene,
        "catalogue_version": data["catalogue_version"],
        "camera": data["camera"],
    }
    spacers = root.findall(".//model[@name='synthetic_pallet']//collision")
    assert {c.get("name") for c in spacers if c.get("name").startswith("spacer_")} == {
        "spacer_center",
        "spacer_outer_0",
        "spacer_outer_1",
    }
    sensor = root.find(".//sensor")
    assert len(root.findall(".//sensor")) == 1
    assert sensor.findtext("update_rate") == "5"
    assert sensor.findtext("camera/image/height") == "480"
    assert root.findtext(".//link[@name='camera_link']/pose") == "0.75 0.0 0.5 0 0 0"
    transforms = yaml.safe_load((tmp_path / "transforms.yaml").read_text())
    assert transforms == {
        "source_provenance": "synthetic",
        "transforms": [
            {
                "parent": "base_link",
                "child": "camera_optical_frame",
                "translation_m": [0.75, 0.0, 0.5],
                "quaternion_xyzw": [-0.5, 0.5, -0.5, 0.5],
            }
        ],
    }
    assert root.findtext(".//light/direction") == " ".join(
        map(str, scene["lighting"]["direction"])
    )
    assert (
        root.findtext(".//model[@name='floor']//visual/material/diffuse")
        == scene["surfaces"]["floor"]
    )
    assert not root.findall(".//uri")
    assert all(model.findtext("static") == "true" for model in root.findall(".//model"))


def test_distractor_geometry_matches_the_catalogue_presets(builder, tmp_path):
    # Explicit expectations keep the standalone builder independent of the host generator.
    presets = {
        "crate_a": ([1.5, 1.6, 0.2], [0.4, 0.4, 0.4], ".5 .5 .5 1"),
        "crate_b": ([4.5, -1.8, 0.3], [0.6, 0.6, 0.6], ".2 .3 .7 1"),
        "post": ([3.0, 2.2, 0.5], [0.3, 0.3, 1.0], ".8 .7 .2 1"),
        "wall_block": ([5.5, 0.0, 0.4], [1.0, 0.5, 0.8], ".7 .2 .2 1"),
        "bin": ([2.2, -2.0, 0.15], [0.5, 0.3, 0.3], ".2 .6 .3 1"),
        "cube": ([4.0, 1.9, 0.25], [0.5, 0.5, 0.5], ".9 .9 .9 1"),
    }
    data = yaml.safe_load(CATALOGUE.read_text())
    seen = set()
    for scene in data["scenes"]:
        if not set(scene["distractors"]) - seen:
            continue
        builder.generate_scene(CATALOGUE, scene["scene_id"], tmp_path)
        root = ET.parse(tmp_path / "scene_world.sdf").getroot()
        models = {
            m.get("name"): m
            for m in root.findall(".//model")
            if m.get("name").startswith("distractor_")
        }
        assert set(models) == {f"distractor_{name}" for name in scene["distractors"]}
        for name in scene["distractors"]:
            model = models[f"distractor_{name}"]
            center, size, color = presets[name]
            assert [float(v) for v in model.findtext("pose").split()][:3] == center
            assert [
                float(v)
                for v in model.findtext(".//collision/geometry/box/size").split()
            ] == size
            assert model.findtext(".//visual/material/diffuse") == color
            seen.add(name)
    assert seen == set(presets)


@pytest.mark.parametrize(
    "mutation",
    [
        "format",
        "provenance",
        "unknown_root",
        "camera",
        "pallet_dimensions",
        "duplicate_scene",
        "category",
        "missing_pallet",
        "width",
        "nan",
        "distractor",
    ],
)
def test_invalid_catalogue_is_rejected_before_outputs(builder, tmp_path, mutation):
    data = yaml.safe_load(CATALOGUE.read_text())
    scene = next(s for s in data["scenes"] if s["category"] == "positive")
    if mutation == "format":
        data["format_version"] = 2
    elif mutation == "provenance":
        data["source_provenance"] = "live"
    elif mutation == "unknown_root":
        data["typo"] = 1
    elif mutation == "camera":
        data["camera"]["optical_quaternion_xyzw"] = [0, 0, 0, 1]
    elif mutation == "pallet_dimensions":
        data["pallet"]["depth_m"] = 1.2
    elif mutation == "duplicate_scene":
        data["scenes"].append(data["scenes"][0])
    elif mutation == "category":
        scene["category"] = "unknown"
    elif mutation == "missing_pallet":
        scene["pallet"] = None
    elif mutation == "width":
        scene["pallet"]["opening_width_m"] = 0.5
    elif mutation == "nan":
        scene["pallet"]["x_m"] = float("nan")
    elif mutation == "distractor":
        scene["distractors"] = ["unknown"]
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(data))
    output = tmp_path / "output"
    with pytest.raises(ValueError):
        builder.generate_scene(path, scene["scene_id"], output)
    assert not output.exists()


# ---------------------------------------------------------------------------
# The second geometry. v1's five boxes cannot express a real pallet, so the
# generator reads the committed URDF instead; these pin that branch.
# ---------------------------------------------------------------------------

EPAL6_CATALOGUE = (
    Path(__file__).resolve().parents[2] / "tests/fixtures/catalogue_epal6_min.yaml"
)
# The pocket convention: the complement of the pallet's boxes, which appears
# nowhere in the URDF.
EPAL6_OPENING_OFFSET_M = 0.18625
EPAL6_OPENING_WIDTH_M = 0.2275
EPAL6_OPENING_Z = (0.022, 0.100)


def _epal6_world(builder, tmp_path, scene_id):
    builder.generate_scene(EPAL6_CATALOGUE, scene_id, tmp_path)
    return ET.parse(next(tmp_path.glob("*.sdf"))).getroot()


def _boxes(model):
    """(name, centre, size) for each collision in a model, in model frame."""
    out = []
    for node in model.findall(".//collision"):
        pose = [float(v) for v in node.find("pose").text.split()]
        size = [float(v) for v in node.find("geometry/box/size").text.split()]
        out.append((node.get("name"), pose[:3], size))
    return out


def _hits(boxes, point):
    return {
        name
        for name, centre, size in boxes
        if all(
            abs(point[axis] - centre[axis]) <= size[axis] / 2 + 1e-9
            for axis in range(3)
        )
    }


def test_the_epal6_pallet_is_emitted_from_its_urdf(builder, tmp_path):
    world = _epal6_world(builder, tmp_path, "e001")
    model = world.find(".//model[@name='synthetic_pallet']")
    assert model is not None
    boxes = _boxes(model)
    assert len(boxes) == 22, "the committed URDF has 22 boxes"
    names = {name for name, _, _ in boxes}
    assert sum(n.startswith("bottom_board") for n in names) == 3
    assert sum(n.startswith("block_") for n in names) == 9
    assert sum(n.startswith("stringer") for n in names) == 3
    assert sum(n.startswith("top_board") for n in names) == 7


def test_both_fork_openings_are_empty_across_their_whole_rectangle(builder, tmp_path):
    """Sweep the opening, not its centreline.

    A zero-width centreline ray passes for any y in (0.0725, 0.300) and any z
    in (0, 0.100) -- a 110 mm lateral slack -- and it also passes when fired at
    a v1 five-box world. It cannot tell the two geometries apart, which is the
    only thing this branch exists to do.
    """
    boxes = _boxes(_epal6_world(builder, tmp_path, "e001").find(".//model[@name='synthetic_pallet']"))
    steps = 9
    for sign in (1.0, -1.0):
        centre_y = sign * EPAL6_OPENING_OFFSET_M
        for i in range(steps):
            y = centre_y + EPAL6_OPENING_WIDTH_M * (i / (steps - 1) - 0.5) * 0.98
            for j in range(steps):
                low, high = EPAL6_OPENING_Z
                z = low + (high - low) * (0.01 + 0.98 * j / (steps - 1))
                for x in (-0.29, 0.0, 0.29):
                    hit = _hits(boxes, (x, y, z))
                    assert not hit, f"opening blocked at {(x, y, z)} by {hit}"


def test_the_columns_and_stringers_are_where_the_openings_are_not(builder, tmp_path):
    """The negative sweep alone would pass on an empty world."""
    boxes = _boxes(_epal6_world(builder, tmp_path, "e001").find(".//model[@name='synthetic_pallet']"))
    # Mid-opening height, on each column centre.
    for y in (-0.35, 0.0, 0.35):
        hit = _hits(boxes, (0.0, y, 0.061))
        assert any(n.startswith("block_") for n in hit), f"no column at y={y}: {hit}"
    # Above the opening, the stringers span the full width.
    hit = _hits(boxes, (0.0, 0.18625, 0.111))
    assert any(n.startswith("stringer") for n in hit), hit
    # Below it, the bottom boards sit under the columns and not under the gaps.
    assert any(n.startswith("bottom_board") for n in _hits(boxes, (0.0, 0.0, 0.011)))
    assert not _hits(boxes, (0.0, 0.18625, 0.011))


def test_the_block_row_negative_has_two_openings_and_no_deck(builder, tmp_path):
    """The negative the detector must refuse on upper-deck evidence alone."""
    model = _epal6_world(builder, tmp_path, "e002").find(".//model[@name='lookalike']")
    boxes = _boxes(model)
    assert len(boxes) == 9
    assert all(name.startswith("block_") for name, _, _ in boxes)
    # Openings present ...
    assert not _hits(boxes, (0.0, EPAL6_OPENING_OFFSET_M, 0.061))
    assert not _hits(boxes, (0.0, -EPAL6_OPENING_OFFSET_M, 0.061))
    # ... columns present ...
    assert _hits(boxes, (0.0, 0.0, 0.061))
    # ... and nothing above the openings at all.
    for z in (0.111, 0.133):
        assert not _hits(boxes, (0.0, EPAL6_OPENING_OFFSET_M, z))


def test_the_v1_lookalike_is_unchanged_by_the_new_branch(builder, tmp_path):
    model = _epal6_world(builder, tmp_path, "e003").find(".//model[@name='lookalike']")
    boxes = _boxes(model)
    # Unchanged, suffix included: only the pallet's own boxes are renamed, and
    # v1 scene consumers address this one as it has always been named.
    assert [name for name, _, _ in boxes] == ["solid_collision"]


def test_a_v1_header_on_an_epal6_catalogue_is_refused(builder, tmp_path):
    """The header gate is per version, so a mismatched pair cannot slip through."""
    import yaml

    data = yaml.safe_load(EPAL6_CATALOGUE.read_text())
    data["pallet"] = builder.APPROVED_PALLETS["v1"]
    path = tmp_path / "mismatched.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="pallet dimensions"):
        builder.load_catalogue(path)


def test_an_unknown_catalogue_version_is_still_refused(builder, tmp_path):
    import yaml

    data = yaml.safe_load(EPAL6_CATALOGUE.read_text())
    data["catalogue_version"] = "v99"
    path = tmp_path / "unknown.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="unsupported catalogue"):
        builder.load_catalogue(path)
