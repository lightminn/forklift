# tests/unit/perception/test_recognition_dataset.py
import json
from dataclasses import replace

import numpy as np
import pytest

from forklift_core.geometry import RigidTransform
from forklift_core.perception import recognition_dataset as rd
from tools import instance_rig, scene_rig

META = {"scene_id": "s", "case": "S1", "shape_key": "epal6", "split": "dev", "render_source": "rig", "noise": None}


def _write(tmp_path, origin="rig_render_pose", meta=META, scene=None):
    out = instance_rig.render_instances([[scene_rig.Box((2.0, 0.0, 0.15), (0.3, 0.6, 0.3))]])
    rd.write_recognition_scene(tmp_path, scene or out.scene, out.instance_ids, origin=origin, meta=meta,
                               truth={"instances": [], "instruction_truth": {"kind": "none", "target": None}},
                               instruction={"present": False})
    return out


def test_round_trip(tmp_path):
    out = _write(tmp_path)
    sample = rd.load_recognition_sample(tmp_path)
    finite = np.isfinite(out.scene.depth_m)
    assert np.nanmax(np.abs(sample.input.scene.depth_m[finite] - out.scene.depth_m[finite])) <= 0.0005 + 1e-12
    assert np.array_equal(sample.instance_ids, out.instance_ids) and sample.origin == "rig_render_pose"
    assert json.loads((tmp_path / "scene.json").read_text())["schema_version"] == 2


def test_input_loader_reads_no_truth(tmp_path):
    _write(tmp_path)
    (tmp_path / "truth.json").unlink()
    (tmp_path / "instances.png").unlink()
    assert rd.load_recognition_input(tmp_path).instruction == {"present": False}


def test_origin_and_render_source_must_agree(tmp_path):
    with pytest.raises(ValueError, match="origin"):
        _write(tmp_path, origin="computed")
    with pytest.raises(ValueError, match="render_source"):
        _write(tmp_path / "b", meta={**META, "render_source": "isaac"})


def test_mismatched_camera_info_frame_is_rejected(tmp_path):
    _write(tmp_path)
    info = json.loads((tmp_path / "camera_info.json").read_text())
    info["frame_id"] = "other"
    (tmp_path / "camera_info.json").write_text(json.dumps(info))
    with pytest.raises(ValueError, match="frame"):
        rd.load_recognition_input(tmp_path)


@pytest.mark.parametrize("axis", [(1, -1, 0), (0, 0, 1), (1, 0, 0)])
def test_tf_rotation_round_trips_at_180_degrees(tmp_path, axis):
    out = instance_rig.render_instances([])
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    rot = 2 * np.outer(k, k) - np.eye(3)  # 180 degrees about k
    tf = RigidTransform("camera_optical_frame", "base_link", rot, out.scene.base_from_optical.translation_m)
    _write(tmp_path, scene=replace(out.scene, base_from_optical=tf))
    assert np.allclose(rd.load_recognition_input(tmp_path).scene.base_from_optical.rotation, rot, atol=1e-9)


@pytest.mark.parametrize(("field", "value"), [("clock_domain", "banana"), ("source_provenance", "ground_truth"),
                                              ("stamp_ns", True)])
def test_unsupported_scene_metadata_is_rejected(tmp_path, field, value):
    _write(tmp_path)
    meta = json.loads((tmp_path / "scene.json").read_text())
    meta[field] = value
    (tmp_path / "scene.json").write_text(json.dumps(meta))
    with pytest.raises(ValueError):
        rd.load_recognition_input(tmp_path)


def test_digest_covers_every_file(tmp_path):
    _write(tmp_path)
    before = rd.scene_digest(tmp_path)
    (tmp_path / "truth.json").write_text(json.dumps({"instances": [1]}))
    assert rd.scene_digest(tmp_path) != before
