"""Scene output behavior; no Gazebo runtime required."""

import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def builder():
    class PendingModule:
        def __getattr__(self, name):
            path = ROOT / "sim/gazebo/build_sensor_world.py"
            assert path.exists(), "implementation is missing"
            spec = importlib.util.spec_from_file_location("tested_module", path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return getattr(module, name)

    return PendingModule()


def test_world_reuses_all_static_visuals_and_has_openings(builder, tmp_path):
    builder.generate(ROOT / "sim/gazebo/scene_config.yaml", tmp_path)
    world = ET.parse(tmp_path / "sensor_world.sdf").getroot()
    original = ET.parse(ROOT / "sim/models/dls08_provisional/forklift.urdf")
    visual_model = world.find(".//model[@name='provisional_forklift_visuals']")
    assert visual_model.findtext("static") == "true"
    assert len(visual_model.findall(".//visual")) == len(original.findall(".//visual"))
    assert not visual_model.findall(".//collision")
    assert not world.findall(".//uri")
    pallet = world.find(".//model[@name='synthetic_pallet']")
    assert len(pallet.findall(".//collision")) == 5
    camera = world.find(".//sensor[@type='rgbd_camera']")
    assert camera.findtext("camera/optical_frame_id") == "camera_optical_frame"
    assert camera.findtext("camera/image/width") == "320"
    assert camera.findtext("update_rate") == "5"
    lidar = world.find(".//sensor[@type='gpu_lidar']")
    assert lidar.findtext("lidar/scan/horizontal/samples") == "360"
    assert (
        yaml.safe_load((tmp_path / "transforms.yaml").read_text())["source_provenance"]
        == "synthetic"
    )
    bridge = yaml.safe_load((tmp_path / "bridge.yaml").read_text())
    assert {entry["ros_topic_name"] for entry in bridge} == {
        "/camera/image",
        "/camera/depth_image",
        "/camera/camera_info",
        "/scan",
        "/clock",
    }


def test_unknown_config_and_nonfinite_mount_are_rejected(builder, tmp_path):
    data = yaml.safe_load((ROOT / "sim/gazebo/scene_config.yaml").read_text())
    data["typo"] = 1
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="config"):
        builder.generate(path, tmp_path)
