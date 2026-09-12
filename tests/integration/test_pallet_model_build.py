"""Read exported geometry with MuJoCo; no renderer or physics integration."""

import hashlib
import json
from datetime import datetime
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest
import yaml

from tools import build_pallet_model

mujoco = pytest.importorskip("mujoco", reason="Install the model extra")
pytestmark = pytest.mark.simulation
REPO_ROOT = Path(__file__).resolve().parents[2]
EPAL6 = REPO_ROOT / "config/pallet_geometry_epal6.yaml"
PARAMETERS = REPO_ROOT / "sim/models/dls08_provisional/parameters.yaml"


def test_the_generated_pallet_has_eleven_boxes_in_the_declared_places(tmp_path):
    out = tmp_path / "epal6"
    assert (
        build_pallet_model.main(["--geometry", str(EPAL6), "--output", str(out)]) == 0
    )
    model = mujoco.MjModel.from_xml_path(str(out / "pallet.xml"))
    sizes = {}
    for index in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index)
        sizes[name] = (model.geom_size[index].copy(), model.geom_pos[index].copy())
    assert len(sizes) == 11
    # MuJoCo box size is the half extent
    half, pos = sizes["deck_bottom"]
    assert half == pytest.approx([0.300, 0.400, 0.011])
    assert pos == pytest.approx([0.0, 0.0, 0.011])
    half, pos = sizes["block_x1_y2"]
    assert half == pytest.approx([0.0365, 0.040, 0.039])
    assert pos == pytest.approx([0.0, 0.360, 0.061])


def test_the_fork_openings_of_the_generated_model_are_actually_empty(tmp_path):
    # sample points along each fork's swept volume and assert none is inside a box
    out = tmp_path / "epal6"
    assert (
        build_pallet_model.main(["--geometry", str(EPAL6), "--output", str(out)]) == 0
    )
    model = mujoco.MjModel.from_xml_path(str(out / "pallet.xml"))
    parameters = yaml.safe_load(PARAMETERS.read_text())
    d = parameters["dimensions"]
    length = (
        d["rear_extent_x_m"]
        + parameters["catalogue"]["overall_length_m"]
        - d["fork_root_x_m"]
    )
    geometry = yaml.safe_load(EPAL6.read_text())
    front = -geometry["overall_depth_m"] / 2
    for sign in (-1, 1):
        y = sign * d["fork_spacing_m"] / 2
        points = (
            np.array(
                np.meshgrid(
                    np.linspace(front, front + length, 121),
                    np.linspace(
                        y - d["fork_width_m"] / 2, y + d["fork_width_m"] / 2, 7
                    ),
                    np.linspace(
                        d["fork_center_height_m"] - d["fork_thickness_m"] / 2,
                        d["fork_center_height_m"] + d["fork_thickness_m"] / 2,
                        7,
                    ),
                )
            )
            .reshape(3, -1)
            .T
        )
        for index in range(model.ngeom):
            inside = np.all(
                np.abs(points - model.geom_pos[index]) <= model.geom_size[index], axis=1
            )
            assert not inside.any(), model.geom(index).name


def test_the_manifest_records_the_geometry_hash(tmp_path):
    out = tmp_path / "epal6"
    assert (
        build_pallet_model.main(["--geometry", str(EPAL6), "--output", str(out)]) == 0
    )
    manifest = json.loads((out / "model_manifest.json").read_text())
    assert manifest["geometry_sha256"] == hashlib.sha256(EPAL6.read_bytes()).hexdigest()
    assert datetime.fromisoformat(manifest["generated_at"]).tzinfo is not None
    assert manifest["source_provenance"] == "epal6_published_standard"
    assert "deck_board_gaps_omitted" in manifest["simplifications"]
    assert "continuous_bottom_and_combined_top_decks" in manifest["simplifications"]


def test_the_urdf_visual_and_collision_boxes_match_the_mjcf(tmp_path):
    out = tmp_path / "epal6"
    build_pallet_model.main(["--geometry", str(EPAL6), "--output", str(out)])
    model = mujoco.MjModel.from_xml_path(str(out / "pallet.xml"))
    root = ET.parse(out / "pallet.urdf").getroot()
    assert len(root.findall("link")) == 1
    for kind in ("visual", "collision"):
        boxes = root.findall(f"link/{kind}")
        assert len(boxes) == 11
        for box in boxes:
            geom = model.geom(box.attrib["name"])
            assert np.fromstring(
                box.find("geometry/box").attrib["size"], sep=" "
            ) == pytest.approx(2 * geom.size)
            assert np.fromstring(
                box.find("origin").attrib["xyz"], sep=" "
            ) == pytest.approx(geom.pos)


def test_geometry_edits_reach_both_model_formats(tmp_path):
    data = yaml.safe_load(EPAL6.read_text())
    data["overall_width_m"] = 0.9
    geometry = tmp_path / "wide.yaml"
    geometry.write_text(yaml.safe_dump(data))
    out = tmp_path / "wide"
    build_pallet_model.main(["--geometry", str(geometry), "--output", str(out)])
    model = mujoco.MjModel.from_xml_path(str(out / "pallet.xml"))
    assert model.geom("deck_bottom").size[1] == pytest.approx(0.45)
    assert model.geom("block_x1_y2").pos[1] == pytest.approx(0.410)
    root = ET.parse(out / "pallet.urdf").getroot()
    size = root.find("link/visual[@name='deck_bottom']/geometry/box").attrib["size"]
    assert np.fromstring(size, sep=" ") == pytest.approx([0.6, 0.9, 0.022])


def test_the_committed_models_match_fresh_generation(tmp_path):
    out = tmp_path / "epal6"
    build_pallet_model.main(["--geometry", str(EPAL6), "--output", str(out)])
    committed = REPO_ROOT / "sim/models/epal6_pallet"
    for name in ("pallet.xml", "pallet.urdf"):
        assert (out / name).read_bytes() == (committed / name).read_bytes()
    old, fresh = [
        json.loads((directory / "model_manifest.json").read_text())
        for directory in (committed, out)
    ]
    old.pop("generated_at")
    fresh.pop("generated_at")
    assert old == fresh
