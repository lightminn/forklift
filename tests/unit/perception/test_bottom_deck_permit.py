"""Bottom-deck evidence bound to its target and the insert permission (plan 2026-10-19 task 3)."""

import json
import math

import pytest

from forklift_core.perception import bottom_deck_permit as P
from forklift_core.perception import fork_fit, instance_fit
from forklift_core.perception.bottom_deck_evidence import BottomDeckParams


def record(verdict="present", *, centre=(3.0, 1.0), yaw=0.3, shape="epal6", axis="x"):
    return P.BottomDeckRecord(
        verdict=verdict, faces=({"face": 1, "verdict": verdict}, {"face": -1, "verdict": verdict}),
        centre_xy=centre, model_yaw_rad=yaw, shape=shape, axis=axis, support_z_m=0.0, candidate_id="c1",
        track_id="t1", captured_s=(10.0, 20.0), correction_s=9.5, params_sha256=P.params_digest(BottomDeckParams()))


def test_only_present_permits_and_absent_and_unobserved_have_their_own_reasons():
    assert P.insert_refusal(record(), (3.0, 1.0), 0.3, "epal6") is None
    assert P.insert_refusal(record("absent"), (3.0, 1.0), 0.3, "epal6") == "no_bottom_deck"
    assert P.insert_refusal(record("unobserved"), (3.0, 1.0), 0.3, "epal6") == "bottom_deck_unobserved"
    assert P.insert_refusal(None, (3.0, 1.0), 0.3, "epal6") == "bottom_deck_unobserved"  # no record: never seen


def test_the_verdict_is_read_before_the_target():
    # an absent record far from the target is still "no bottom deck", not a mismatch
    assert P.insert_refusal(record("absent"), (9.0, 9.0), 2.0, "t11_06") == "no_bottom_deck"


def test_the_target_must_be_within_15_cm_of_the_recorded_centre():
    assert P.insert_refusal(record(), (3.0 + 0.149, 1.0), 0.3, "epal6") is None
    assert P.insert_refusal(record(), (3.0, 1.0 + 0.151), 0.3, "epal6") == "bottom_deck_target_mismatch"


def test_the_shape_must_match():
    assert P.insert_refusal(record(), (3.0, 1.0), 0.3, "t11_06") == "bottom_deck_target_mismatch"


def test_the_entry_axis_must_match_either_way_round():
    assert P.insert_refusal(record(), (3.0, 1.0), 0.3 + math.pi, "epal6") is None  # 180 deg symmetric
    assert P.insert_refusal(record(), (3.0, 1.0), 0.3 + math.pi / 2, "epal6") == "bottom_deck_target_mismatch"
    tol = P.AXIS_TOLERANCE_RAD
    assert P.insert_refusal(record(), (3.0, 1.0), 0.3 - 0.99 * tol, "epal6") is None
    assert P.insert_refusal(record(), (3.0, 1.0), 0.3 + 1.01 * tol, "epal6") == "bottom_deck_target_mismatch"


def test_a_t11_y_record_permits_only_the_y_entry():
    rec = record(shape="t11_06", axis="y", yaw=0.3)  # entry along model +y: the insertion yaw is model yaw + 90 deg
    assert P.insert_refusal(rec, (3.0, 1.0), 0.3 + math.pi / 2, "t11_06") is None
    assert P.insert_refusal(rec, (3.0, 1.0), 0.3 - math.pi / 2, "t11_06") is None
    assert P.insert_refusal(rec, (3.0, 1.0), 0.3, "t11_06") == "bottom_deck_target_mismatch"  # the x entry


def test_the_record_refuses_values_outside_its_contract():
    with pytest.raises(ValueError):
        record("maybe")
    with pytest.raises(ValueError):
        record(axis="z")
    with pytest.raises(ValueError):
        record(centre=(math.nan, 0.0))


def test_the_params_digest_changes_with_a_threshold():
    assert P.params_digest(BottomDeckParams()) == P.params_digest(BottomDeckParams())
    assert P.params_digest(BottomDeckParams()) != P.params_digest(BottomDeckParams(plane_tol_m=0.016))


def test_the_refusals_are_fork_fit_reason_codes():
    assert set(P.REFUSALS) <= set(fork_fit.REASON_CODES)


def test_known_shapes_wait_for_the_bottom_deck_and_parametric_ones_keep_their_contract():
    assert instance_fit.KNOWN_FAR_PENDING == instance_fit.FAR_PENDING + ("bottom_deck",)
    assert instance_fit.PARAMETRIC_PENDING == ("depth", "lane_bounds", "top_layout", "entry_axis")


def test_the_record_survives_the_json_the_mission_state_is_written_as():
    rec = record()
    kept = json.loads(json.dumps(rec.to_record()))
    assert P.BottomDeckRecord.from_record(kept) == rec
