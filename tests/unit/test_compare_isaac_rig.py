# tests/unit/test_compare_isaac_rig.py
from dataclasses import replace

import numpy as np
import pytest

from forklift_core.perception.recognition_dataset import load_recognition_sample, write_recognition_scene
from tools import compare_isaac_rig as cmp
from tools import generate_recognition_scenes as gen
from tools import recognition_scenes as rs

SETTINGS = cmp.load_settings(cmp.DEFAULT_SETTINGS)


def _recipe():
    return next(r for r in rs.catalogue() if r.case == "S2" and r.split == "dev")


def _write(tmp_path, record, depth, ids, name):
    write_recognition_scene(tmp_path / name, replace(record.rendered.scene, depth_m=depth), ids,
                            origin="isaac_render_pose",
                            meta={**record.meta, "render_source": "isaac", "truth_render": "rig", "noise": None},
                            truth=record.truth, instruction=record.instruction)
    return load_recognition_sample(tmp_path / name)


def _as_isaac(tmp_path, *, edit=None):
    recipe = _recipe()
    rig_k = gen.recipe_record(recipe, gen.load_site()).rendered.scene.intrinsics
    record = gen.recipe_record(recipe, gen.load_site(), intrinsics=replace(rig_k, cx=319.5, cy=239.5))
    depth, ids = record.rendered.scene.depth_m.copy(), record.rendered.instance_ids.copy()
    if edit:
        depth, ids = edit(depth, ids)
    return cmp.compare_scene(_write(tmp_path, record, depth, ids, recipe.scene_id), recipe, SETTINGS)


def test_a_rig_render_at_the_isaac_k_passes(tmp_path):
    report = _as_isaac(tmp_path)
    assert report["pass"] and min(t["iou"] for t in report["targets"].values()) == 1.0


@pytest.mark.parametrize("edit", [
    lambda d, i: (np.where(i == 0, d + 0.1, d), i),        # whole pallet 100 mm too far
    lambda d, i: (np.where(i == 0, np.nan, d), i),         # whole pallet without depth
    lambda d, i: (d, np.roll(i, 3, axis=1)),               # mask shifted 3 px
    lambda d, i: (np.where(i == -1, d + 0.005, d), i),     # floor and wall 5 mm off
])
def test_wrong_renders_fail(tmp_path, edit):
    assert not _as_isaac(tmp_path, edit=edit)["pass"]


def test_the_rig_k_fails_the_k_contract(tmp_path):
    recipe = _recipe()
    record = gen.recipe_record(recipe, gen.load_site())
    sample = _write(tmp_path, record, record.rendered.scene.depth_m, record.rendered.instance_ids, "x")
    report = cmp.compare_scene(sample, recipe, SETTINGS)
    assert not report["k_contract"] and not report["pass"]


def test_a_missing_pallet_block_fails(tmp_path):
    """A part missing from the render shows through as background: those pixels change id, so the
    per-target mismatch fraction (id or depth) must catch what IoU and same-id depth miss."""
    from tools import instance_rig

    recipe = next(r for r in rs.catalogue() if r.scene_id == "s1_epal6_1000000")
    rig_k = gen.recipe_record(recipe, gen.load_site()).rendered.scene.intrinsics
    k = replace(rig_k, cx=319.5, cy=239.5)
    record = gen.recipe_record(recipe, gen.load_site(), intrinsics=k)
    boxes = rs.recipe_boxes(recipe, 0)
    broken = instance_rig.render_instances([boxes[:8] + boxes[9:]], distractors=recipe.distractors, intrinsics=k)
    write_recognition_scene(tmp_path / recipe.scene_id, broken.scene, broken.instance_ids, origin="isaac_render_pose",
                            meta={**record.meta, "render_source": "isaac", "truth_render": "rig", "noise": None},
                            truth=record.truth, instruction=record.instruction)
    report = cmp.compare_scene(load_recognition_sample(tmp_path / recipe.scene_id), recipe, SETTINGS)
    assert report["targets"]["0"]["mismatch"] > cmp.MISMATCH_MAX and not report["pass"]


def test_a_label_tie_at_equal_depth_is_counted_but_not_a_mismatch(tmp_path):
    """Touching pallets share a face: at equal depth either label may win the pixel (s2_epal6_1000127)."""
    def relabel_edge(d, i):
        i = i.copy()
        rows, cols = np.nonzero(i == 1)
        take = slice(0, 40)
        i[rows[take], cols[take]] = 0
        return d, i

    report = _as_isaac(tmp_path, edit=relabel_edge)
    target = report["targets"]["1"]
    assert target["mismatch"] == 0.0 and target["ties"] > 0 and report["pass"]
