# src/forklift_core/perception/instance_proposals.py
"""Instance proposals: one mask per pallet-like object and where it came from (design §4, §6).

A proposal says which pixels belong to one object. Whether it is a pallet and
where its pockets are is decided by the geometric fit (instance_fit), not here.
"""

from dataclasses import dataclass

import numpy as np

from forklift_core._validation import _finite_scalar

PROPOSAL_SOURCES = ("learned", "geometric", "oracle")


@dataclass(frozen=True)
class InstanceProposal:
    proposal_id: str
    mask: np.ndarray
    score: float
    source: str

    def __post_init__(self) -> None:
        if not isinstance(self.proposal_id, str) or not self.proposal_id:
            raise ValueError("proposal_id must be nonempty text")
        mask = np.asarray(self.mask)
        if mask.ndim != 2:
            raise ValueError("mask must be a two-dimensional bool array")
        if mask.dtype != bool:
            raise ValueError("mask must be a bool array")
        if not mask.any():
            raise ValueError("mask must be nonempty")
        score = _finite_scalar(self.score, "score")
        if not 0.0 <= score <= 1.0:
            raise ValueError("score must lie in [0, 1]")
        if self.source not in PROPOSAL_SOURCES:
            raise ValueError(f"source must be one of {PROPOSAL_SOURCES}")
        object.__setattr__(self, "mask", mask)
        object.__setattr__(self, "score", score)
