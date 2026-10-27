from collections import Counter

import pytest

from forklift_core.perception.recognition_stats import min_scenes
from tools import negative_structures as ns
from tools import recognition_scenes as rs
from tools import recognition_sequences as rseq
from tools.recognition_truth import instance_boxes


@pytest.fixture(scope="module")
def bottomless():
    return rs.catalogue_bottomless()


def test_the_old_catalogue_is_unchanged():
    # The old N set and every earlier case keep their recipes byte for byte (digest pinned before this change).
    assert rs.catalogue_digest(rs.catalogue()) == rs.CATALOGUE_DIGEST_2026_10_27


def test_n2_has_153_scenes_with_46_for_eval(bottomless):
    n2 = [r for r in bottomless if r.case == "N2"]
    assert len(n2) == 153
    assert Counter(r.split for r in n2) == {"dev": 107, "eval": 46}
    assert {r.seed for r in n2} == set(range(1_100_000, 1_100_153))
    assert all(not r.instances and r.distractors for r in n2)


def test_n2_is_mostly_hard_lookalikes_in_each_split(bottomless):
    for split in ("dev", "eval"):
        kinds = [rs.n2_kind(r.rank) for r in bottomless if r.case == "N2" and r.split == split]
        assert sum(k in ns.HARD for k in kinds) / len(kinds) >= 2 / 3, split


def test_s8_has_enough_eval_scenes_per_shape(bottomless):
    need = min_scenes(0, 0.05)
    for shape in ("grounded_epal6", "grounded_t11_06"):
        s8 = [r for r in bottomless if r.case == "S8" and r.shape_key == shape]
        assert sum(r.split == "eval" for r in s8) >= need, shape
        assert all(len(r.instances) == 1 and r.instances[0].bottom == "none" for r in s8)


def test_s8_t11_shows_both_faces(bottomless):
    yaws = [r.instances[0].placement.yaw_rad for r in bottomless if r.shape_key == "grounded_t11_06"]
    turned = sum(abs(y) > 1.0 for y in yaws)
    assert 0.3 < turned / len(yaws) < 0.7


def test_s8_instructions_are_like_s1(bottomless):
    kinds = Counter(r.instruction.kind for r in bottomless if r.case == "S8")
    assert set(kinds) <= {"unambiguous", "none"} and kinds["unambiguous"] > kinds["none"] > 0


def test_a_grounded_instance_has_no_bottom_boards_and_blocks_to_the_floor(bottomless):
    r = next(r for r in bottomless if r.shape_key == "grounded_epal6")
    boxes = instance_boxes(r.instances[0].spec, r.instances[0].damage, r.instances[0].bottom)
    assert not [b for b in boxes if b.name.startswith("bottom_board")]
    blocks = [b for b in boxes if b.name.startswith("block")]
    assert blocks and all(b.centre_m[2] - b.size_m[2] / 2 == pytest.approx(0.0) for b in blocks)


def test_bottomless_seeds_stay_clear_of_every_reserved_band(bottomless):
    old = {r.seed for r in rs.catalogue()}
    mine = {r.seed for r in bottomless}
    assert not mine & old
    lo, hi = min(mine), max(mine)
    assert rs.SEED_BANDS["eval_pool"] <= lo and hi < rs.SEED_BANDS["eval_pool"] + rs.BAND_WIDTH
    for name, start in rseq.SEQUENCE_BANDS.items():
        assert not (lo <= start <= hi), name
    assert hi < min(rseq.SEQUENCE_BANDS.values())


def test_the_training_counterpart_comes_from_the_train_band():
    train = rs.training_catalogue_bottomless(scale=1)
    start = rs.SEED_BANDS["train"]
    assert all(start <= r.seed < start + rs.BAND_WIDTH and r.split == "train" for r in train)
    assert not {r.seed for r in train} & {r.seed for r in rs.training_catalogue(scale=8)}
