# tests/unit/perception/test_instance_proposals.py
import numpy as np
import pytest

from forklift_core.perception.instance_proposals import InstanceProposal


def _mask():
    m = np.zeros((4, 5), bool)
    m[1:3, 1:4] = True
    return m


def test_a_proposal_keeps_its_mask_score_and_source():
    p = InstanceProposal("p0", _mask(), 0.9, "learned")
    assert p.mask.sum() == 6 and p.score == 0.9 and p.source == "learned"


@pytest.mark.parametrize("kw, match", [
    ({"mask": np.zeros((4, 5), bool)}, "nonempty"),
    ({"mask": np.ones((4, 5), np.uint8)}, "bool"),
    ({"mask": np.ones(5, bool)}, "two-dimensional"),
    ({"score": 1.5}, r"\[0, 1\]"),
    ({"score": True}, "finite"),
    ({"source": "truth"}, "source"),
    ({"proposal_id": ""}, "proposal_id"),
])
def test_bad_proposals_are_refused(kw, match):
    args = {"proposal_id": "p0", "mask": _mask(), "score": 0.5, "source": "geometric", **kw}
    with pytest.raises(ValueError, match=match):
        InstanceProposal(**args)
