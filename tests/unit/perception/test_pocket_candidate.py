# tests/unit/perception/test_pocket_candidate.py
import math
from dataclasses import replace

import pytest

from forklift_core.perception.pallet_model import ObservedPalletShape, face_layout
from forklift_core.perception.pocket_candidate import PocketCandidate, PocketEstimate
from tools import recognition_scenes as rs


def _candidate(**kw):
    spec = rs.known_spec("epal6")
    shape = ObservedPalletShape(spec, face_layout(spec, "x"), (("face", "known_shape"), ("depth", "known_shape")))
    pocket = PocketEstimate((2.0, 0.2, 0.061), 0.2275, 0.078, "known_shape", "known_shape")
    base = dict(candidate_id="c0", proposal_id="p0", proposal_source="oracle", stamp_ns=1, hypothesis="epal6:x",
                axis="x", alternatives=(), insertion_yaw_rad=0.1, face_centre_m=(2.0, 0.0, 0.0),
                pallet_centre_m=(2.3, 0.03, 0.0), left=pocket, right=replace(pocket, centre_m=(2.0, -0.2, 0.061)),
                support_z_m=0.0, support_state="visible", on_floor=True, range_m=1.25, score=0.9,
                geometry="fitted", identity="confirmed", reasons=(), pickable=True, fit_reasons=(), shape=shape,
                alternative_centres_m=((2.3, 0.33, 0.0),), observed_centre_xy=(2.0, 0.0), observed_extent_m=0.4)
    return PocketCandidate(**{**base, **kw})


def test_only_fitted_and_confirmed_candidates_are_counted():
    assert _candidate().counted
    assert not _candidate(geometry="ambiguous", reasons=("lateral_unconstrained",)).counted
    assert not _candidate(identity="unconfirmed").counted


@pytest.mark.parametrize("kw, match", [
    ({"geometry": "valid"}, "geometry"),
    ({"identity": "maybe"}, "identity"),
    ({"support_state": "floor"}, "support_state"),
    ({"insertion_yaw_rad": math.nan}, "finite"),
    ({"pallet_centre_m": (1.0, 2.0)}, "pallet centre"),
    ({"geometry": "ambiguous"}, "reason"),
    ({"pickable": "yes"}, "pickable"),
    ({"alternative_centres_m": ((1.0, 2.0),)}, "alternative centre"),
    ({"observed_extent_m": -0.1}, "observed_extent_m"),
])
def test_bad_candidates_are_refused(kw, match):
    with pytest.raises(ValueError, match=match):
        _candidate(**kw)


def test_value_sources_are_checked():
    with pytest.raises(ValueError, match="source"):
        PocketEstimate((0.0, 0.0, 0.0), 0.2, 0.08, "guess", "known_shape")
    spec = rs.known_spec("epal6")
    with pytest.raises(ValueError, match="source"):
        ObservedPalletShape(spec, face_layout(spec, "x"), (("face", "measured"),))
