# tests/unit/test_recognition_render_core.py
import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

from tools import recognition_scenes as rs
from tools import scene_rig

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def rc():
    name = "recognition_render_core"
    if name not in sys.modules:  # dataclasses need the module registered before execution
        spec = importlib.util.spec_from_file_location(name, ROOT / "sim/isaac/recognition_render_core.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


def _payload(data, labels):
    return {"data": np.asarray(data, dtype=np.uint32), "info": {"idToLabels": labels}}


def test_prim_specs_reproduce_the_recipe_boxes_and_the_rig_floor_and_wall(rc):
    recipe = next(r for r in rs.catalogue() if r.case == "S6")
    specs = rc.prim_specs(recipe)
    boxes = [b for k in range(len(recipe.instances)) for b in rs.recipe_boxes(recipe, k)]
    inst = [s for s in specs if s.base_label.startswith("inst_")]
    assert len(inst) == len(boxes)
    for spec, box in zip(inst, boxes):
        assert spec.centre_m == pytest.approx(tuple(box.centre_m)) and spec.size_m == pytest.approx(tuple(box.size_m))
        assert spec.yaw_rad == pytest.approx(box.yaw_rad)
    assert len([s for s in specs if s.base_label == "distractor"]) == len(recipe.distractors)
    floor = next(s for s in specs if s.base_label == "floor")
    wall = next(s for s in specs if s.base_label == "wall")
    assert floor.centre_m[2] + floor.size_m[2] / 2 == pytest.approx(0.0)
    assert wall.centre_m[0] - wall.size_m[0] / 2 == pytest.approx(scene_rig.BACK_WALL_X_M)


def test_labels_carry_the_scene_token(rc):
    assert rc.scene_label("inst_3", 12) == "inst_3_t12"
    assert rc.parse_label("inst_3_t12") == ("inst_3", 12)
    assert rc.parse_label("BACKGROUND") is None and rc.parse_label("UNLABELLED") is None


def test_semantics_decode_instances_and_readiness(rc):
    labels = {"0": {"class": "BACKGROUND"}, "1": {"class": "inst_0_t5"}, "2": {"class": ["inst_3_t5"]},
              "3": {"class": "distractor_t5"}, "4": {"class": "floor_t5"}}
    frame = rc.decode_semantic(_payload([[0, 1, 2], [3, 4, 4]], labels), 5, shape=(2, 3))
    assert frame.instance_ids.tolist() == [[-1, 0, 3], [-2, -1, -1]] and frame.ready


def test_a_frame_from_the_previous_scene_is_not_ready(rc):
    frame = rc.decode_semantic(_payload([[1, 4]], {"1": {"class": "inst_0_t4"}, "4": {"class": "floor_t5"}}), 5,
                               shape=(1, 2))
    assert not frame.ready and frame.stale_pixels == 1
    assert not rc.decode_semantic(_payload([[1]], {"1": {"class": "inst_0_t5"}}), 5, shape=(1, 1)).ready


@pytest.mark.parametrize("payload", [None, {}, {"data": None, "info": {}}, {"data": np.zeros((0,), np.uint32)},
                                     {"data": np.zeros((2, 2), np.uint32), "info": {"idToLabels": {}}}])
def test_an_unready_annotator_is_retried_not_failed(rc, payload):
    frame = rc.decode_semantic(payload, 1, shape=(1, 1))
    assert not frame.ready and frame.status != "ready"


def test_bad_nonempty_semantics_are_refused(rc):
    with pytest.raises(ValueError, match="label"):
        rc.decode_semantic(_payload([[1]], {"1": {"class": "inst_0_t1,inst_1_t1"}}), 1, shape=(1, 1))
    with pytest.raises(ValueError, match="integer"):
        rc.decode_semantic({"data": np.zeros((1, 1), np.float32), "info": {"idToLabels": {"0": {"class": "x"}}}}, 1,
                           shape=(1, 1))


def test_clean_depth(rc):
    out = rc.clean_depth(np.array([np.inf, 0.0, -1.0, 2.0]).reshape(2, 2, 1))
    assert out.shape == (2, 2) and np.isnan(out.ravel()[:3]).all() and out[1, 1] == 2.0


@pytest.mark.parametrize("status, code", [
    ({"rendered": [{"scene_id": "a", "captures": 1}], "failed": []}, 0),
    ({"rendered": [], "failed": [{"scene_id": "a", "error": "x"}]}, 1),
    ({"rendered": [], "failed": [], "fatal": {"error": "x"}}, 1),
    ({"rendered": [], "failed": []}, 1),      # nothing rendered
    (None, 1),                                # status.json never written
])
def test_the_job_exit_code_comes_from_status_json(rc, tmp_path, status, code):
    import json

    path = tmp_path / "status.json"
    if status is not None:
        path.write_text(json.dumps(status))
    assert rc.status_exit_code(path) == code


def test_each_scene_gets_its_own_prim_root(rc):
    recipe = next(r for r in rs.catalogue() if r.case == "S1")
    a, b = rc.prim_specs(recipe, root=rc.scene_root(3)), rc.prim_specs(recipe, root=rc.scene_root(4))
    assert all(s.path.startswith("/World/Scene_t3/") for s in a) and all(s.path.startswith("/World/Scene_t4/") for s in b)
    assert not {s.path for s in a} & {s.path for s in b}


def test_stale_tokens_are_reported(rc):
    frame = rc.decode_semantic(_payload([[1, 2]], {"1": {"class": "floor_t4"}, "2": {"class": "wall_t2"}}), 5,
                               shape=(1, 2))
    assert frame.status == "stale" and frame.stale_tokens == (2, 4)
