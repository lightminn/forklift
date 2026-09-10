"""Exercise generated geometry and joints, not copies of XML source strings."""

import json
import os
import subprocess
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco", reason="Install the model extra")
yaml = pytest.importorskip("yaml", reason="Install the model extra")
pytestmark = pytest.mark.simulation
ROOT = Path(__file__).resolve().parents[2]
PARAMETERS = ROOT / "sim/models/dls08_provisional/parameters.yaml"
BUILDER = ROOT / "tools/build_forklift_model.py"


def generate(output, parameters=PARAMETERS):
    result = subprocess.run(
        [
            sys.executable,
            str(BUILDER),
            "--parameters",
            str(parameters),
            "--output",
            str(output),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    return mujoco.MjModel.from_xml_path(str(output / "forklift.xml"))


def geometry_bounds(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lower, upper = [], []
    for index in range(model.ngeom):
        if model.geom_group[index] != 1:
            continue
        rotation = data.geom_xmat[index].reshape(3, 3)
        size = model.geom_size[index]
        if model.geom_type[index] == mujoco.mjtGeom.mjGEOM_BOX:
            extent = np.abs(rotation) @ size
        elif model.geom_type[index] == mujoco.mjtGeom.mjGEOM_CYLINDER:
            extent = size[0] * np.sqrt(np.sum(rotation[:, :2] ** 2, axis=1))
            extent += size[1] * np.abs(rotation[:, 2])
        elif model.geom_type[index] == mujoco.mjtGeom.mjGEOM_SPHERE:
            extent = np.full(3, size[0])
        else:
            raise AssertionError("AABB oracle needs a new primitive formula")
        lower.append(data.geom_xpos[index] - extent)
        upper.append(data.geom_xpos[index] + extent)
    return np.min(lower, axis=0), np.max(upper, axis=0)


def test_generated_geometry_matches_catalog_envelope(tmp_path):
    model = generate(tmp_path)
    lower, upper = geometry_bounds(model)
    np.testing.assert_allclose(upper - lower, [1.46, 0.63, 1.01], atol=1e-6)
    np.testing.assert_allclose(lower, [-0.51, -0.315, 0.0], atol=1e-6)
    assert np.sum(model.body_mass) == pytest.approx(24.0)


def test_fork_lift_moves_only_up_and_preserves_two_tips(tmp_path):
    model = generate(tmp_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ids = [model.site(name).id for name in ["left_fork_tip", "right_fork_tip"]]
    before = data.site_xpos[ids].copy()
    assert before[0, 1] - before[1, 1] == pytest.approx(0.29)
    data.joint("fork_lift").qpos[0] = 0.2
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(data.site_xpos[ids] - before, [[0, 0, 0.2]] * 2)


def test_catalogue_size_changes_reach_exported_geometry(tmp_path):
    parameters = yaml.safe_load(PARAMETERS.read_text())
    parameters["catalogue"].update(
        overall_length_m=1.58, overall_width_m=0.70, overall_height_m=1.08
    )
    path = tmp_path / "parameters.yaml"
    path.write_text(yaml.safe_dump(parameters))
    model = generate(tmp_path / "model", path)
    lower, upper = geometry_bounds(model)
    np.testing.assert_allclose(upper - lower, [1.58, 0.70, 1.08], atol=1e-6)


def test_collision_model_keeps_gap_between_forks(tmp_path):
    model = generate(tmp_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    group = np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8)
    geom_id = np.zeros(1, dtype=np.int32)
    distances = []
    for y in [0.145, -0.145, 0.0]:
        distances.append(
            mujoco.mj_ray(
                model,
                data,
                np.array([1.2, y, 0.04]),
                np.array([-1.0, 0.0, 0.0]),
                group,
                1,
                -1,
                geom_id,
            )
        )
    np.testing.assert_allclose(distances[:2], [0.25, 0.25], atol=0.002)
    assert distances[2] < 0 or distances[2] > 0.55


def test_visual_geometry_does_not_duplicate_contact_geometry(tmp_path):
    model = generate(tmp_path)
    visual = model.geom_group == 1
    collision = model.geom_group == 3
    assert np.sum(visual) > 30
    assert np.sum(collision) > 10
    assert np.all(model.geom_contype[visual] == 0)
    assert np.all(model.geom_conaffinity[visual] == 0)
    assert np.all(model.geom_contype[collision] == 1)
    assert np.all(model.geom_conaffinity[collision] == 2)


@pytest.mark.parametrize("axle,steered_x", [("front", 0.30), ("rear", -0.34)])
def test_changing_steered_axle_changes_actual_wheel_transforms(
    tmp_path, axle, steered_x
):
    parameters = yaml.safe_load(PARAMETERS.read_text())
    parameters["assumptions"]["steering_axle"] = axle
    path = tmp_path / "parameters.yaml"
    path.write_text(yaml.safe_dump(parameters))
    model = generate(tmp_path / "model", path)
    data = mujoco.MjData(model)
    data.joint("left_steer").qpos[0] = 0.3
    mujoco.mj_forward(model, data)
    body = model.body(f"{axle}_left_wheel").id
    assert data.xpos[body, 0] == pytest.approx(steered_x)
    np.testing.assert_allclose(
        data.xmat[body].reshape(3, 3)[:, 0], [np.cos(0.3), np.sin(0.3), 0.0], atol=1e-8
    )


def test_scene_settles_under_gravity_without_exploding(tmp_path):
    generate(tmp_path)
    model = mujoco.MjModel.from_xml_path(str(tmp_path / "scene.xml"))
    data = mujoco.MjData(model)
    for _ in range(1500):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos))
    assert np.all(np.isfinite(data.qvel))
    assert np.all(data.warning.number == 0)
    assert data.qpos[2] == pytest.approx(0.0, abs=0.01)
    assert np.linalg.norm(data.qvel[:6]) < 0.02
    assert data.ncon >= 4


def test_urdf_has_a_valid_tree_and_compiles_in_mujoco(tmp_path):
    generate(tmp_path)
    tree = ET.parse(tmp_path / "forklift.urdf").getroot()
    links = {link.attrib["name"] for link in tree.findall("link")}
    children = [joint.find("child").attrib["link"] for joint in tree.findall("joint")]
    assert len(children) == len(set(children))
    assert links - set(children) == {"base_link"}
    for joint in tree.findall("joint"):
        assert joint.find("parent").attrib["link"] in links
        assert joint.find("child").attrib["link"] in links
    model = mujoco.MjModel.from_xml_path(str(tmp_path / "forklift.urdf"))
    assert model.njnt == 7
    assert np.sum(model.body_mass) == pytest.approx(24.0)


def test_urdf_joint_limits_preserve_assumed_units_and_changed_torque(tmp_path):
    parameters = yaml.safe_load(PARAMETERS.read_text())
    parameters["assumptions"]["steering_torque_limit_nm"] = 2.5
    path = tmp_path / "parameters.yaml"
    path.write_text(yaml.safe_dump(parameters))
    model = generate(tmp_path / "model", path)
    tree = ET.parse(tmp_path / "model/forklift.urdf").getroot()
    expected = {
        "continuous": (3.0, 8.0),
        "revolute": (2.5, 1.0),
        "prismatic": (180.0, 1.0),
    }
    for joint in tree.findall("joint"):
        kind = joint.attrib["type"]
        if kind == "fixed":
            continue
        limit = joint.find("limit")
        assert limit is not None
        effort, velocity = expected[kind]
        assert float(limit.attrib["effort"]) == effort
        assert float(limit.attrib["velocity"]) == velocity
    for name in ["left_steer_position", "right_steer_position"]:
        np.testing.assert_allclose(model.actuator(name).forcerange, [-2.5, 2.5])


@pytest.mark.parametrize(
    "change", ["unknown", "nan", "fork_gap", "axle", "wide_mast", "large_wheels"]
)
def test_bad_parameters_fail_before_writing_model(tmp_path, change):
    parameters = yaml.safe_load(PARAMETERS.read_text())
    if change == "unknown":
        parameters["dimensions"]["typo_m"] = 0.4
    elif change == "nan":
        parameters["dimensions"]["wheel_radius_m"] = float("nan")
    elif change == "fork_gap":
        parameters["dimensions"]["fork_width_m"] = 0.4
    elif change == "wide_mast":
        parameters["dimensions"]["mast_width_m"] = 0.8
    elif change == "large_wheels":
        parameters["dimensions"]["wheel_radius_m"] = 0.6
    else:
        parameters["assumptions"]["steering_axle"] = "unknown"
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(parameters))
    result = subprocess.run(
        [
            sys.executable,
            str(BUILDER),
            "--parameters",
            str(path),
            "--output",
            str(tmp_path / "out"),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    assert "parameters:" in result.stderr
    assert not (tmp_path / "out/forklift.xml").exists()


@pytest.mark.rendering
@pytest.mark.skipif(
    os.environ.get("FORKLIFT_RENDER_TEST") != "1",
    reason="Set FORKLIFT_RENDER_TEST=1 to exercise the EGL renderer",
)
def test_preview_renders_distinct_views_and_records_evidence(tmp_path):
    image_module = pytest.importorskip("PIL.Image")
    generate(tmp_path / "model")
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "tools/preview_forklift_model.py"),
            "--model",
            str(tmp_path / "model/scene.xml"),
            "--output",
            str(tmp_path / "preview"),
            "--backend",
            "egl",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    report = json.loads((tmp_path / "preview/preview.json").read_text())
    assert report["mode"] == "kinematic_pose_preview"
    assert report["renderer"]
    with image_module.open(tmp_path / "preview/overview.png") as frame:
        assert frame.size == (1400, 1000)
        assert np.asarray(frame).std() > 20
    assert (tmp_path / "preview/views.png").exists()
