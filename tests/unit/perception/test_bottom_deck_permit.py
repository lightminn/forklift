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


def test_known_shapes_no_longer_wait_for_a_bottom_deck_and_parametric_ones_keep_their_contract():
    # plan 2026-10-27 §2.1 a: a pallet without a bottom deck is a pick target; nothing resolved the item with the
    # check off. The value stays allowed so older candidate objects still load.
    assert instance_fit.KNOWN_FAR_PENDING == instance_fit.FAR_PENDING
    assert instance_fit.PARAMETRIC_PENDING == ("depth", "lane_bounds", "top_layout", "entry_axis")
    from forklift_core.perception.pocket_candidate import PENDING_ITEMS
    assert "bottom_deck" in PENDING_ITEMS


def test_the_record_survives_the_json_the_mission_state_is_written_as():
    rec = record()
    kept = json.loads(json.dumps(rec.to_record()))
    assert P.BottomDeckRecord.from_record(kept) == rec


# --- plan 2026-10-27 (bottomless pallets) §2.1 g: the structure mode -- both faces seen, each present or absent ---

def faces(*verdicts, ids=(1, -1)):
    return tuple({"face": f, "verdict": v} for f, v in zip(ids, verdicts, strict=True))


@pytest.mark.parametrize("seen, on, structure", [
    (("present", "present"), "present", "present"),
    (("absent", "absent"), "absent", "absent"),
    (("present", "absent"), "absent", "absent"),
    (("absent", "unobserved"), "absent", "unobserved"),
    (("unobserved", "absent"), "absent", "unobserved"),
    (("present", "unobserved"), "unobserved", "unobserved"),
    (("absent",), "absent", "unobserved"),
])
def test_the_two_face_verdict_per_mode(seen, on, structure):
    fs = faces(*seen, ids=(1, -1)[:len(seen)])
    assert P.two_face_verdict(fs, "on") == on
    assert P.two_face_verdict(fs, "structure") == structure


def test_one_face_seen_twice_is_not_two_faces_in_either_mode():
    assert P.two_face_verdict(faces("present", "present", ids=(1, 1)), "on") == "unobserved"
    assert P.two_face_verdict(faces("present", "present", ids=(1, 1)), "structure") == "unobserved"
    assert P.two_face_verdict(faces("absent", "absent", ids=(1, 1)), "structure") == "unobserved"


def structure_record(*seen, **kw):
    fs = faces(*seen, ids=(1, -1)[:len(seen)])
    return P.BottomDeckRecord(**{**record(**kw).__dict__, "faces": fs,
                                 "verdict": P.two_face_verdict(fs, "structure")})


def test_structure_permits_present_and_absent_and_refuses_unobserved():
    for seen in (("present", "present"), ("absent", "absent"), ("present", "absent")):
        assert P.insert_refusal(structure_record(*seen), (3.0, 1.0), 0.3, "epal6", mode="structure") is None
    rec = structure_record("absent", "unobserved")
    assert P.insert_refusal(rec, (3.0, 1.0), 0.3, "epal6", mode="structure") == "bottom_deck_unobserved"
    assert P.insert_refusal(None, (3.0, 1.0), 0.3, "epal6", mode="structure") == "bottom_deck_unobserved"


def test_structure_keeps_the_target_match():
    rec = structure_record("absent", "absent")
    assert P.insert_refusal(rec, (3.0, 1.0 + 0.151), 0.3, "epal6", mode="structure") == "bottom_deck_target_mismatch"
    assert P.insert_refusal(rec, (3.0, 1.0), 0.3, "t11_06", mode="structure") == "bottom_deck_target_mismatch"


def test_structure_does_not_take_a_record_whose_faces_do_not_show_both_sides():
    # an on-mode record ends at the first absent face; structure needs both faces whatever the verdict says
    rec = P.BottomDeckRecord(**{**record("absent").__dict__, "faces": faces("absent", ids=(1,))})
    assert P.insert_refusal(rec, (3.0, 1.0), 0.3, "epal6", mode="structure") == "bottom_deck_unobserved"


def test_on_is_unchanged_and_the_default():
    assert P.insert_refusal(record("absent"), (3.0, 1.0), 0.3, "epal6") == "no_bottom_deck"
    assert P.insert_refusal(record("absent"), (3.0, 1.0), 0.3, "epal6", mode="on") == "no_bottom_deck"
    with pytest.raises(ValueError):
        P.insert_refusal(record(), (3.0, 1.0), 0.3, "epal6", mode="off")
