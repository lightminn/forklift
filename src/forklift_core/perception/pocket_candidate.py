# src/forklift_core/perception/pocket_candidate.py
"""PocketCandidate: one fitted pallet instance (ADR 0003 1-a, design §8).

Geometry state and identity state are separate (§7.5): only a fitted and
confirmed candidate may be selected automatically. Unlike PocketObservation,
a non-fitted candidate keeps its pose so the console and the tracker can show it.
pickable None means the fork-fit decision waits for more evidence (an unseen
support, or the near-range re-observation of §9.1); it never means "insert".
"""

from dataclasses import dataclass

import numpy as np

from forklift_core._validation import _finite_scalar
from forklift_core.perception.instance_proposals import PROPOSAL_SOURCES
from forklift_core.perception.pallet_model import VALUE_SOURCES, ObservedPalletShape

GEOMETRY_STATES = ("fitted", "ambiguous", "unfit")
IDENTITY_STATES = ("confirmed", "unconfirmed")
SUPPORT_STATES = ("visible", "coplanar", "physical_edge", "uncertain")


def _vector(value, n, name):
    try:
        out = tuple(_finite_scalar(v, name) for v in value)
    except TypeError:
        raise ValueError(f"{name} must be a finite {n}-vector") from None
    if len(out) != n:
        raise ValueError(f"{name} must be a finite {n}-vector")
    return out


@dataclass(frozen=True)
class PocketEstimate:
    centre_m: tuple
    width_m: float
    height_m: float
    width_source: str
    height_source: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "centre_m", _vector(self.centre_m, 3, "pocket centre"))
        for name in ("width_m", "height_m"):
            if _finite_scalar(getattr(self, name), name) <= 0:
                raise ValueError(f"{name} must be positive")
        for name in ("width_source", "height_source"):
            if getattr(self, name) not in VALUE_SOURCES:
                raise ValueError(f"{name} must be one of {VALUE_SOURCES}")


@dataclass(frozen=True)
class PocketCandidate:
    candidate_id: str
    proposal_id: str
    proposal_source: str
    stamp_ns: int
    hypothesis: str
    axis: str
    alternatives: tuple
    insertion_yaw_rad: float
    face_centre_m: tuple
    pallet_centre_m: tuple
    left: PocketEstimate
    right: PocketEstimate
    support_z_m: float
    support_state: str
    on_floor: bool
    range_m: float
    score: float
    geometry: str
    identity: str
    reasons: tuple
    pickable: bool | None
    fit_reasons: tuple
    shape: ObservedPalletShape
    alternative_centres_m: tuple = ()   # pallet centres of readings that compete with this one
    observed_centre_xy: tuple | None = None  # median xy of the proposal's points: where the object was seen
    observed_extent_m: float = 0.0           # largest distance of those points from observed_centre_xy
    def __post_init__(self) -> None:
        if self.geometry not in GEOMETRY_STATES:
            raise ValueError(f"geometry must be one of {GEOMETRY_STATES}")
        if self.identity not in IDENTITY_STATES:
            raise ValueError(f"identity must be one of {IDENTITY_STATES}")
        if self.support_state not in SUPPORT_STATES:
            raise ValueError(f"support_state must be one of {SUPPORT_STATES}")
        if self.proposal_source not in PROPOSAL_SOURCES:
            raise ValueError(f"proposal_source must be one of {PROPOSAL_SOURCES}")
        if self.axis not in ("x", "y"):
            raise ValueError("axis must be x or y")
        if self.geometry != "fitted" and not self.reasons:
            raise ValueError("a candidate that is not fitted needs a reason")
        if self.pickable is not None and not isinstance(self.pickable, bool):
            raise ValueError("pickable must be a bool or None")
        for name in ("insertion_yaw_rad", "support_z_m", "range_m", "score"):
            _finite_scalar(getattr(self, name), name)
        object.__setattr__(self, "face_centre_m", _vector(self.face_centre_m, 3, "face centre"))
        object.__setattr__(self, "pallet_centre_m", _vector(self.pallet_centre_m, 3, "pallet centre"))
        object.__setattr__(self, "alternative_centres_m",
                           tuple(_vector(c, 3, "alternative centre") for c in self.alternative_centres_m))
        if self.observed_centre_xy is not None:
            object.__setattr__(self, "observed_centre_xy", _vector(self.observed_centre_xy, 2, "observed centre"))
        if _finite_scalar(self.observed_extent_m, "observed_extent_m") < 0:
            raise ValueError("observed_extent_m must be nonnegative")
        for name in ("reasons", "fit_reasons", "alternatives"):
            object.__setattr__(self, name, tuple(getattr(self, name)))
        if not isinstance(self.on_floor, (bool, np.bool_)):
            raise ValueError("on_floor must be a bool")
        object.__setattr__(self, "on_floor", bool(self.on_floor))

    @property
    def counted(self) -> bool:
        return self.geometry == "fitted" and self.identity == "confirmed"
