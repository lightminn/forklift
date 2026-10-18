# tests/unit/test_recognition_scenes.py
import itertools

import pytest

from forklift_core.perception.fork_fit import ForkSetup
from forklift_core.perception.recognition_stats import eval_count
from tools import recognition_scenes as rs
from tools import scene_rig
from tools.recognition_truth import scene_truths


@pytest.fixture(scope="module")
def catalogue():
    return rs.catalogue()


def test_catalogue_counts_and_instruction_quotas(catalogue):
    assert len(catalogue) == 1020
    for shape in ("epal6", "t11_06"):
        ev = [r for r in catalogue if r.shape_key == shape and r.split == "eval"]
        assert len(ev) == 6 * eval_count(61) == 114
        kinds = [r.instruction.kind for r in ev]
        assert kinds.count("unambiguous") >= 29 and kinds.count("ambiguous") >= 15
        assert kinds.count("unambiguous") + kinds.count("ambiguous") >= 59
        assert sum(r.touching for r in ev if r.case == "S2") >= 4
    assert sum(r.case == "S7" and r.split == "eval" for r in catalogue) == 59
    assert sum(r.case == "N" and r.split == "eval" for r in catalogue) == 29


def test_generation_is_deterministic():
    a = rs.generate("S2", "epal6", 5, band="eval_pool", first_seed=1_000_000)
    b = rs.generate("S2", "epal6", 5, band="eval_pool", first_seed=1_000_000)
    assert a == b and rs.catalogue_digest(a) == rs.catalogue_digest(b)


def test_seed_overlap_is_detected():
    a = rs.generate("S1", "epal6", 3, band="eval_pool", first_seed=1_000_000)
    rs.assert_disjoint(a, rs.generate("S1", "epal6", 3, band="train", first_seed=5_000_000))
    with pytest.raises(ValueError, match="overlap"):
        rs.assert_disjoint(a, rs.generate("S1", "epal6", 3, band="eval_pool", first_seed=1_000_001))


def test_first_seed_must_lie_in_its_band():
    with pytest.raises(ValueError, match="band"):
        rs.generate("S1", "epal6", 3, band="train", first_seed=1_000_000)


@pytest.mark.parametrize("case", ["S2", "S3", "S6", "S7"])
def test_objects_do_not_interpenetrate(case):
    shape = "nonstandard" if case == "S7" else "t11_06"
    for recipe in rs.generate(case, shape, 8, band="eval_pool", first_seed=1_000_000):
        groups = [rs.recipe_boxes(recipe, k) for k in range(len(recipe.instances))] + [list(recipe.distractors)]
        for a, b in itertools.combinations(groups, 2):
            assert not any(scene_rig.boxes_interpenetrate(x, y) for x in a for y in b)


def test_s4_instructions_carry_the_layer_only_when_unambiguous():
    for r in rs.generate("S4", "epal6", 10, band="eval_pool", first_seed=1_000_000):
        if r.instruction.kind == "unambiguous":
            assert r.instruction.layer == r.instances[r.instruction.target].layer
        if r.instruction.kind == "ambiguous":
            assert r.instruction.layer is None and r.instruction.radius_m == pytest.approx(0.3)


def test_s7_eval_is_stratified(catalogue):
    verdicts = []
    for r in (r for r in catalogue if r.case == "S7" and r.split == "eval"):
        truths, _ = scene_truths([(i.spec, i.placement, i.damage, r.instruction.load) for i in r.instances],
                                 ForkSetup(r.fork_spacing_m))
        verdicts.append(tuple(truths[0].faces[0].fit["reasons"]))
    assert sum(v == () for v in verdicts) >= 25
    for code in rs.S7_VIOLATIONS:
        assert sum(code in v for v in verdicts) >= 4, code


def test_s7_shapes_stay_inside_the_design_ranges(catalogue):
    for r in (r for r in catalogue if r.case == "S7"):
        spec = r.instances[0].spec
        assert all(0.06 - 1e-9 <= w <= 0.20 + 1e-9 for w in spec.column_widths_m), r.scene_id
        assert 0.045 - 1e-9 <= spec.opening_height_m <= 0.12 + 1e-9, r.scene_id
        assert 0.60 - 1e-9 <= spec.overall_width_m <= 1.20 + 1e-9, r.scene_id


def test_s3_adjacent_gaps_stay_inside_the_design_range(catalogue):
    import math

    for r in (r for r in catalogue if r.case == "S3" or (r.case == "S5" and r.rank % 3 == 2)):
        if len(r.instances) < 2:
            continue
        yaw = r.instances[0].placement.yaw_rad
        a = (math.cos(yaw), math.sin(yaw))
        depth = r.instances[0].spec.overall_depth_m
        along = [p.placement.x_m * a[0] + p.placement.y_m * a[1] for p in r.instances]
        for front, back in zip(along, along[1:]):
            assert 0.05 - 1e-9 <= back - front - depth <= 1.0 + 1e-9, r.scene_id


def test_the_training_catalogue_is_the_catalogue_composition_from_the_train_band():
    train = rs.training_catalogue(scale=1)
    cat = rs.catalogue()
    assert len(train) == len(cat) == 1020
    assert {r.split for r in train} == {"train"}
    assert not {r.seed for r in train} & {r.seed for r in cat}
    count = lambda rs_: sorted((r.case, r.shape_key) for r in rs_)
    assert count(train) == count(cat)
    assert [r.seed for r in rs.training_catalogue(scale=1)] == [r.seed for r in train]
