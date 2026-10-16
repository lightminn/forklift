# tests/unit/test_retruth_recognition_dataset.py
import copy
import json
from dataclasses import replace

import pytest

from tools import recognition_scenes as rs
from tools import retruth_recognition_dataset as rr
from tools.generate_recognition_scenes import load_site, recipe_record


@pytest.fixture(scope="module")
def s7_pair():
    """The truth of s7 1000873 before (x faces only) and after (x and y faces) the contract change."""
    r = next(x for x in rs.catalogue() if x.scene_id == "s7_nonstandard_1000873")
    before = replace(r, instances=tuple(replace(i, spec=replace(i.spec, entry_axes=("x",))) for i in r.instances))
    site = load_site()
    return rr.canon(recipe_record(before, site).truth), rr.canon(recipe_record(r, site).truth)


def test_the_allowed_s7_change_passes_and_is_reported(s7_pair):
    old, new = s7_pair
    changes = rr.check_s7(old, new, "s7")
    assert changes[0]["y_faces"] == 2 and changes[0]["instance_id"] == 0


@pytest.mark.parametrize("tamper", [
    lambda t: t["instances"][0]["spec"].__setitem__("row_depth_m", t["instances"][0]["spec"]["row_depth_m"] * 0.8),
    lambda t: next(f for f in t["instances"][0]["faces"] if f["axis"] == "x").__setitem__("exposure", 0.5),
    lambda t: t["instruction_truth"].__setitem__("target", 1),
    lambda t: t["instances"][0].__setitem__("support_z_m", 0.1),
    lambda t: t["instances"][0]["faces"].append(dict(t["instances"][0]["faces"][0], axis="z")),
])
def test_any_other_s7_change_is_refused(s7_pair, tamper):
    """Codex 2026-10-17: identical x faces do not prove identical geometry (a row depth change kept them),
    so the spec, the instance fields and the instruction truth are compared too."""
    old, new = s7_pair
    new = copy.deepcopy(new)
    tamper(new)
    with pytest.raises(rr.RetruthError):
        rr.check_s7(old, new, "s7")


def test_a_scene_outside_the_change_must_be_identical(s7_pair):
    old, _ = s7_pair
    rr.check_unchanged(old, copy.deepcopy(old), "s1")
    changed = copy.deepcopy(old)
    changed["instances"][0]["eligible"] = not changed["instances"][0]["eligible"]
    with pytest.raises(rr.RetruthError):
        rr.check_unchanged(old, changed, "s1")


def test_noise_levels_must_share_the_truth_inputs(tmp_path):
    dirs = {}
    for lv in ("clean", "mid"):
        d = tmp_path / lv / "s1_x"
        d.mkdir(parents=True)
        (d / "scene.json").write_text(json.dumps({"scene_id": "s1_x", "split": "dev", "noise": lv}))
        for name in rr.SHARED_INPUTS:
            (d / name).write_text("{}")
        dirs[lv] = d
    rr.check_levels(dirs, "s1_x")
    (dirs["mid"] / "camera_info.json").write_text('{"k": [1]}')
    with pytest.raises(rr.RetruthError):
        rr.check_levels(dirs, "s1_x")


def _levels(tmp_path, scenes_by_level):
    roots = {}
    for lv, scenes in scenes_by_level.items():
        roots[lv] = tmp_path / f"isaac_{lv}"
        for sid in scenes:
            (roots[lv] / sid).mkdir(parents=True)
    return roots


def test_every_level_must_hold_the_same_scenes(tmp_path):
    """Codex review 2026-10-17: the scene list came from the first level only, so a scene missing there
    silently left every level."""
    roots = _levels(tmp_path, {"clean": ["a"], "mid": ["a", "b"]})
    with pytest.raises(rr.RetruthError, match="scene sets differ"):
        rr.scene_ids(roots, {"a", "b"})


def test_the_scenes_must_be_the_whole_catalogue(tmp_path):
    roots = _levels(tmp_path, {"clean": ["a"], "mid": ["a"]})
    with pytest.raises(rr.RetruthError, match="catalogue"):
        rr.scene_ids(roots, {"a", "b"})
    assert rr.scene_ids(_levels(tmp_path / "ok", {"clean": ["a", "b"], "mid": ["b", "a"]}), {"a", "b"}) == ["a", "b"]
