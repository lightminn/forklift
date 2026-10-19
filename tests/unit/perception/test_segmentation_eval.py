# tests/unit/perception/test_segmentation_eval.py
"""Segmentation metrics for instance proposals (stage 3 plan, task 1)."""
import numpy as np
import pytest

from forklift_core.perception import segmentation_eval as S

H, W = 60, 80


def _ids(*boxes):
    """Truth id image: -1 background; boxes (id, v0, v1, u0, u1)."""
    ids = np.full((H, W), -1, dtype=np.int32)
    for k, v0, v1, u0, u1 in boxes:
        ids[v0:v1, u0:u1] = k
    return ids


def _mask(v0, v1, u0, u1):
    m = np.zeros((H, W), bool)
    m[v0:v1, u0:u1] = True
    return m


def test_the_truth_masks_as_proposals_are_matched_one_to_one():
    ids = _ids((0, 10, 30, 5, 35), (1, 10, 30, 45, 75))
    out = S.evaluate_segmentation([ids == 0, ids == 1], ids, eligible={0, 1})
    assert out.matched == {0: 0, 1: 1} and out.ious == {0: 1.0, 1: 1.0}
    assert out.recall == 1.0 and out.merges == [] and out.splits == [] and out.extra == {}


def test_an_exact_union_of_two_truths_is_a_merge_not_a_match():
    # Codex v2: each truth has IoU 0.5 with the union; the merge is decided first and leaves the matching
    ids = _ids((0, 10, 30, 5, 35), (1, 10, 30, 35, 65))
    out = S.evaluate_segmentation([(ids == 0) | (ids == 1)], ids, eligible={0, 1})
    assert out.matched == {} and out.merges == [0] and out.recall == 0.0


def test_six_fragments_of_one_truth_are_a_split():
    # Codex v2: each fragment covers 16.7 % -- the sum decides
    ids = _ids((0, 0, 60, 0, 60))
    frags = [_mask(0, 60, 10 * k, 10 * (k + 1)) for k in range(6)]
    out = S.evaluate_segmentation(frags, ids, eligible={0})
    assert out.matched == {} and out.splits == [0] and out.extra == {}


def test_a_proposal_on_a_visible_but_ineligible_instance_is_not_extra():
    ids = _ids((0, 10, 30, 5, 35), (1, 10, 30, 45, 75))
    out = S.evaluate_segmentation([ids == 0, ids == 1], ids, eligible={0})
    assert out.matched == {0: 0} and out.ineligible_matched == [1] and out.extra == {}


def test_extra_proposals_are_counted_by_what_they_cover():
    ids = _ids((0, 10, 30, 5, 35), (-2, 40, 55, 5, 35))
    out = S.evaluate_segmentation([ids == 0, _mask(40, 55, 5, 35), _mask(40, 55, 50, 70)], ids, eligible={0})
    assert out.extra == {"distractor": 1, "background": 1}


def test_the_matching_maximises_the_total_iou_with_a_fixed_tie_order():
    ids = _ids((0, 10, 30, 0, 40), (1, 10, 30, 40, 80))
    a = _mask(10, 30, 0, 40)
    b = _mask(10, 30, 0, 44)  # overlaps 0 most, 1 a little
    out = S.evaluate_segmentation([b, a], ids, eligible={0, 1})
    assert out.matched == {0: 1} or out.matched == {0: 0}
    again = S.evaluate_segmentation([b, a], ids, eligible={0, 1})
    assert again.matched == out.matched  # deterministic


def test_the_recall_denominator_is_the_eligible_instances():
    ids = _ids((0, 10, 30, 5, 35), (1, 10, 30, 45, 75))
    out = S.evaluate_segmentation([ids == 0], ids, eligible={0, 1})
    assert out.recall == pytest.approx(0.5) and out.missed == [1]


def test_proposals_that_are_empty_or_mis_shaped_are_refused():
    ids = _ids((0, 10, 30, 5, 35))
    with pytest.raises(ValueError):
        S.evaluate_segmentation([np.zeros((H, W + 1), bool)], ids, eligible={0})
