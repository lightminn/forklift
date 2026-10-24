# tests/unit/perception/test_target_selection.py
from dataclasses import replace

import pytest

from forklift_core.perception.pallet_model import ObservedPalletShape, face_layout
from forklift_core.perception.pocket_candidate import PocketCandidate, PocketEstimate
from forklift_core.perception.target_selection import assign_layers, select_target
from tools import recognition_scenes as rs

EPAL = rs.known_spec("epal6")
H = EPAL.overall_height_m


def _c(cid, x, y, *, z=0.0, state=None, geometry="fitted", identity="confirmed", pickable=True, alt=(), seen=None):
    """A candidate whose pallet centre is (x, y); its face is 0.3 m nearer the camera (EPAL 6)."""
    state = state or ("visible" if z == 0.0 else "coplanar")
    shape = ObservedPalletShape(EPAL, face_layout(EPAL, "x"), (("face", "known_shape"),))
    p = PocketEstimate((x - 0.3, y + 0.18625, z + 0.061), 0.2275, 0.078, "known_shape", "known_shape")
    return PocketCandidate(cid, "p" + cid, "oracle", 1, "epal6:x", "x", (), 0.0, (x - 0.3, y, z), (x, y, z), p,
                           replace(p, centre_m=(x - 0.3, y - 0.18625, z + 0.061)), z, state,
                           state == "visible" and z == 0.0, x - 0.3 - 0.75, 0.9, geometry, identity,
                           () if geometry == "fitted" else ("lateral_unconstrained",), pickable,
                           () if pickable is not False else ("fork_spacing_adjust",), shape,
                           tuple((ax, ay, z) for ax, ay in alt), seen or (x - 0.3, y), 0.4)


def _instr(x, y, r=0.3, layer=None):
    return {"present": True, "x_m": x, "y_m": y, "radius_m": r, "layer": layer, "load": None}


NONE = {"present": False}


def test_a_stack_on_the_floor_gets_absolute_layers():
    cands = [_c("a", 3.0, 0.0), _c("b", 3.0, 0.01, z=H), _c("c", 3.01, 0.0, z=2 * H)]
    assert assign_layers(cands) == {"a": (0, 0), "b": (1, 1), "c": (2, 2)}


def test_a_stack_whose_bottom_is_not_on_the_floor_has_only_relative_layers():
    cands = [_c("b", 3.0, 0.0, z=H), _c("c", 3.0, 0.0, z=2 * H)]
    assert assign_layers(cands) == {"b": (0, None), "c": (1, None)}


def test_a_gap_in_the_stack_drops_the_absolute_numbers():
    cands = [_c("a", 3.0, 0.0), _c("c", 3.0, 0.0, z=2 * H)]   # the middle pallet was not seen
    assert assign_layers(cands)["c"] == (1, None)


def test_an_upper_layer_with_an_unseen_support_drops_the_absolute_numbers():
    """Codex 2026-10-16: a floor candidate and one at height H with an uncertain support got layer 1."""
    cands = [_c("a", 3.0, 0.0), _c("b", 3.0, 0.0, z=H, state="uncertain")]
    assert assign_layers(cands)["b"] == (1, None)


def test_one_identified_fit_alone_in_the_radius_is_selected():
    s = select_target([_c("a", 3.0, 0.0), _c("b", 3.0, 1.5)], _instr(3.0, 0.0))
    assert (s.status, s.candidate_id) == ("selected", "a")


def test_the_instruction_names_the_pallet_centre_not_the_face():
    """Codex 2026-10-16: instructions carry the pallet centre (tools/recognition_scenes.py _instruction);
    the face of a T11 is 0.33 m nearer, outside a 0.3 m radius."""
    s = select_target([_c("a", 3.0, 0.0)], _instr(3.0, 0.0, r=0.1))
    assert (s.status, s.candidate_id) == ("selected", "a")
    assert select_target([_c("a", 3.0, 0.0)], _instr(2.7, 0.0, r=0.1)).status == "none"


def test_two_identified_fits_in_the_radius_ask_the_operator():
    s = select_target([_c("a", 3.0, 0.0), _c("b", 3.0, 0.5)], _instr(3.0, 0.25, r=0.5))
    assert s.status == "ask_operator" and s.candidate_id is None


def test_an_unconfirmed_neighbour_in_the_radius_asks_for_another_look():
    s = select_target([_c("a", 3.0, 0.0), _c("b", 3.0, 0.4, identity="unconfirmed")], _instr(3.0, 0.2, r=0.5))
    assert s.status == "reobserve" and "uncertain_in_radius" in s.reasons


def test_nothing_in_the_radius_is_none():
    assert select_target([_c("a", 3.0, 1.5)], _instr(3.0, 0.0)).status == "none"


def test_an_unpickable_target_is_reported_with_its_reason():
    s = select_target([_c("a", 3.0, 0.0, pickable=False)], _instr(3.0, 0.0))
    assert (s.status, s.candidate_id, s.reasons) == ("not_pickable", "a", ("fork_spacing_adjust",))


def test_the_instructed_layer_is_picked_from_an_absolute_stack():
    s = select_target([_c("a", 3.0, 0.0), _c("b", 3.0, 0.0, z=H)], _instr(3.0, 0.0, layer=1))
    assert (s.status, s.candidate_id) == ("selected", "b")


def test_a_layer_instruction_on_a_relative_stack_asks_for_another_look():
    s = select_target([_c("b", 3.0, 0.0, z=H), _c("c", 3.0, 0.0, z=2 * H)], _instr(3.0, 0.0, layer=1))
    assert s.status == "reobserve" and "layer_relative" in s.reasons


def test_without_an_instruction_the_nearest_pickable_identified_fit_is_chosen():
    cands = [_c("far", 4.0, 0.0), _c("near", 2.5, 0.8), _c("nope", 2.0, 0.0, pickable=False),
             _c("amb", 1.9, 0.0, geometry="ambiguous")]
    assert select_target(cands, NONE).candidate_id == "near"


def test_without_an_instruction_an_unpickable_pallet_is_reported_not_hidden():
    """§10: a fork-fit failure is not_pickable, not none (Codex 2026-10-16)."""
    s = select_target([_c("a", 3.0, 0.0, pickable=False)], NONE)
    assert (s.status, s.candidate_id) == ("not_pickable", "a")


def test_a_pending_fork_fit_may_be_selected_but_stays_pending():
    s = select_target([_c("a", 3.0, 0.0, pickable=None)], NONE)
    assert (s.status, s.candidate_id) == ("selected", "a")


def test_ties_without_an_instruction_are_broken_by_lateral_position_not_by_asking():
    s = select_target([_c("l", 3.0, 0.5), _c("r", 3.0, -0.5)], NONE)
    assert (s.status, s.candidate_id) == ("selected", "r")


def test_unreachable_candidates_are_skipped():
    s = select_target([_c("a", 2.5, 0.0), _c("b", 3.0, 0.0)], NONE, reachable=lambda c: c.candidate_id != "a")
    assert s.candidate_id == "b"


def test_a_pallet_seen_but_not_fitted_inside_the_radius_asks_for_another_look():
    """Codex 2026-10-16: touching pallets at (3, -0.4) and (3, 0.4), radius 0.401; the failed one is
    seen around (2.7, 0.75), 0.808 m away -- outside a 0.4 m margin, inside the 0.85 m half-diagonal."""
    from forklift_core.perception.instance_fit import FitFailure
    cands = [_c("a", 3.0, -0.4)]
    instr = _instr(3.0, 0.0, r=0.401)
    assert select_target(cands, instr).candidate_id == "a"
    seen = FitFailure("p1", "exception:FloatingPointError", (2.7, 0.75), 0.0)
    s = select_target(cands, instr, unresolved=(seen,))
    assert s.status == "reobserve" and "unresolved_in_radius" in s.reasons
    assert select_target(cands, instr, unresolved=(FitFailure("p1", "no_face", None),)).status == "reobserve"


def test_a_failed_fit_is_judged_by_its_nearest_visible_point():
    """The median of the visible points is 1.5 m out, but the object spreads 0.3 m around it: its
    nearest point is 1.2 m away, within 0.401 + 0.85, so its centre may be in the radius."""
    from forklift_core.perception.instance_fit import FitFailure
    cands = [_c("a", 3.0, -0.4)]
    instr = _instr(3.0, 0.0, r=0.401)
    spread = FitFailure("p1", "no_face", (3.0, 1.5), 0.3)
    assert select_target(cands, instr, unresolved=(spread,)).status == "reobserve"
    point = FitFailure("p1", "no_face", (3.0, 1.5), 0.0)
    assert select_target(cands, instr, unresolved=(point,)).candidate_id == "a"


def test_an_ambiguous_candidate_whose_rival_centre_is_in_the_radius_asks_for_another_look():
    """Codex review A round 2: an ambiguous candidate's best centre was pushed out of the radius by a
    wrong placement, so the other pallet was selected although both centres were inside."""
    cands = [_c("a", 2.0, -0.29, geometry="ambiguous", alt=((2.0, 0.0),)), _c("b", 3.2, 0.0)]
    s = select_target(cands, _instr(2.6, 0.0, r=0.62))
    assert s.status == "reobserve" and "uncertain_in_radius" in s.reasons


def test_an_uncertain_candidate_is_placed_by_what_was_seen_not_by_its_rejected_fit():
    """Codex review A round 4: a non-standard pallet came back shape_mismatch with a wrong EPAL centre
    outside the radius and no rivals, so the other pallet was selected. An uncertain candidate is
    judged like an unfitted object: by its visible points and the largest half-diagonal."""
    rejected = _c("a", 1.78, -0.42, geometry="ambiguous", seen=(1.9, -0.4))
    s = select_target([rejected, _c("b", 3.2, 0.4)], _instr(2.6, 0.0, r=0.73))
    assert s.status == "reobserve" and "uncertain_in_radius" in s.reasons
    far = _c("a", 1.78, -3.0, geometry="ambiguous", seen=(1.9, -3.0))
    assert select_target([far, _c("b", 3.2, 0.4)], _instr(2.6, 0.0, r=0.73)).candidate_id == "b"


def test_the_layer_filter_never_removes_an_uncertain_candidate():
    """Codex review A round 4: an ambiguous candidate whose best placement was layer 2 but whose
    competing placement was layer 3 was dropped by the layer filter and another layer-3 pallet won."""
    cands = [_c("a", 3.0, 0.0), _c("b", 3.0, 0.0, z=H), _c("c", 3.0, 0.0, z=2 * H),
             _c("x", 3.0, 0.3, z=H, geometry="ambiguous")]
    s = select_target(cands, _instr(3.0, 0.0, r=0.5, layer=2))
    assert s.status == "reobserve" and "uncertain_in_radius" in s.reasons
    assert select_target(cands[:3], _instr(3.0, 0.0, r=0.5, layer=2)).candidate_id == "c"


def test_no_candidates_is_none():
    assert select_target([], NONE).status == "none"


# --- the floor contract's occupancy exclusion (docs/plans/2026-10-24-far-target-occupancy.md O1) -------------

def _neighbour_behind():
    """P-2 behind P-4 in the CT-1 queue scene: seen in part, never fitted -- an uncertain candidate whose
    visible points come within 0.85 m of the radius."""
    return _c("n", 4.6, 0.0, geometry="ambiguous", seen=(4.2, 0.0))


def test_an_uncertain_neighbour_behind_a_floor_pallet_is_excluded_by_occupancy():
    cands, instr = [_c("a", 3.0, 0.0), _neighbour_behind()], _instr(3.0, 0.0, r=0.354)
    assert select_target(cands, instr).status == "reobserve"
    s = select_target(cands, instr, occupancy_margin_m=0.05)
    assert (s.status, s.candidate_id) == ("selected", "a") and "occupancy_excluded" in s.reasons
    assert s.detail["excluded"] == ["n"] and s.detail["slack_m"] == pytest.approx(0.55 - 0.354 - 0.05)


def test_an_unfitted_object_behind_a_floor_pallet_is_excluded_by_occupancy():
    from forklift_core.perception.instance_fit import FitFailure
    behind = FitFailure("l2", "no_face", (4.2, 0.0), 0.3)
    cands, instr = [_c("a", 3.0, 0.0)], _instr(3.0, 0.0, r=0.354)
    assert select_target(cands, instr, unresolved=(behind,)).status == "reobserve"
    s = select_target(cands, instr, unresolved=(behind,), occupancy_margin_m=0.05)
    assert (s.status, s.candidate_id) == ("selected", "a") and s.detail["excluded"] == ["l2"]


def test_without_a_margin_the_occupancy_exclusion_is_off():
    s = select_target([_c("a", 3.0, 0.0), _neighbour_behind()], _instr(3.0, 0.0, r=0.354), occupancy_margin_m=None)
    assert s.status == "reobserve" and "uncertain_in_radius" in s.reasons


def test_an_unpickable_floor_pallet_stays_unpickable_after_the_exclusion():
    """Codex review 2026-10-24 (2nd edition): the exclusion went past _decide's refusal."""
    s = select_target([_c("a", 3.0, 0.0, pickable=False), _neighbour_behind()], _instr(3.0, 0.0, r=0.354),
                      occupancy_margin_m=0.05)
    assert (s.status, s.candidate_id) == ("not_pickable", "a")
    assert "fork_spacing_adjust" in s.reasons and "occupancy_excluded" in s.reasons


def test_a_pallet_not_seen_on_the_floor_excludes_nothing():
    s = select_target([_c("a", 3.0, 0.0, state="uncertain"), _neighbour_behind()], _instr(3.0, 0.0, r=0.354),
                      occupancy_margin_m=0.05)
    assert s.status == "reobserve"


def test_a_non_standard_fit_excludes_nothing():
    """Codex review 2026-10-24: a non-standard fit's depth is unseen, its centre moves up to 0.175 m."""
    a = _c("a", 3.0, 0.0)
    a = replace(a, shape=replace(a.shape, spec=None))
    s = select_target([a, _neighbour_behind()], _instr(3.0, 0.0, r=0.354), occupancy_margin_m=0.05)
    assert s.status == "reobserve"


def test_a_fit_with_competing_centres_or_a_depth_segment_excludes_nothing():
    alt = _c("a", 3.0, 0.0, alt=((3.0, 0.05),))
    seg = replace(_c("a", 3.0, 0.0), centre_segment_m=((2.9, 0.0, 0.0), (3.1, 0.0, 0.0)))
    for a in (alt, seg):
        s = select_target([a, _neighbour_behind()], _instr(3.0, 0.0, r=0.354), occupancy_margin_m=0.05)
        assert s.status == "reobserve"


@pytest.mark.parametrize("offset, status", [(0.199, "selected"), (0.201, "reobserve")])
def test_the_occupancy_inequality_has_a_sharp_boundary(offset, status):
    """|c_H - t| + r + m < h_H + h_min: 0.30 + 0.25 - 0.30 - 0.05 = 0.20 m from the instructed centre."""
    cands = [_c("a", 3.0, 0.0), _c("n", 4.6, 0.0, geometry="ambiguous", seen=(4.2, 0.0))]
    s = select_target(cands, _instr(3.0, offset, r=0.30), occupancy_margin_m=0.05)
    assert s.status == status


def test_a_non_standard_minimum_pallet_can_sit_beside_a_t11():
    """Codex review 2026-10-24: a T11 at x 3.000 and a 0.50 x 0.60 non-standard pallet at x 3.581, both centres
    within 0.354 m of 3.230 and 1 mm apart: 0.230 + 0.354 + 0.05 = 0.634 >= 0.33 + 0.25, no exclusion."""
    from forklift_core.perception.instance_fit import FitFailure
    t11 = rs.known_spec("t11_06")
    a = _c("a", 3.0, 0.0)
    a = replace(a, shape=ObservedPalletShape(t11, face_layout(t11, "x"), (("face", "known_shape"),)))
    beside = FitFailure("u", "no_face", (3.581, 0.0), 0.0)
    s = select_target([a], _instr(3.230, 0.0, r=0.354), unresolved=(beside,), occupancy_margin_m=0.05)
    assert s.status == "reobserve"


def test_two_identified_fits_still_ask_the_operator_with_the_exclusion_on():
    s = select_target([_c("a", 3.0, 0.0), _c("b", 3.0, 0.5), _neighbour_behind()], _instr(3.0, 0.25, r=0.5),
                      occupancy_margin_m=0.05)
    assert s.status == "ask_operator"


def test_a_layer_instruction_keeps_the_layer_path():
    """Codex review 2026-10-24 (3rd edition): a layer-1 instruction picked the floor pallet."""
    from forklift_core.perception.instance_fit import FitFailure
    s = select_target([_c("a", 3.0, 0.0)], _instr(3.0, 0.0, r=0.30, layer=1),
                      unresolved=(FitFailure("u", "no_face", (3.7, 0.0), 0.0),), occupancy_margin_m=0.05)
    assert s.status == "reobserve" and "unresolved_in_radius" in s.reasons


def test_two_identified_fits_behind_an_unfitted_object_are_not_resolved_by_occupancy():
    """The unfitted object sends the robot to look again before the count of identified fits is read; the
    exclusion must not then pick one of two fits in the radius -- here two overlapping readings 0.1 m apart, where
    the inequality alone (0.05 + 0.1 + 0.05 < 0.55) would hold."""
    from forklift_core.perception.instance_fit import FitFailure
    unfitted = (FitFailure("u", "no_face", (4.2, 0.0), 0.3),)
    s = select_target([_c("a", 3.0, 0.0), _c("b", 3.0, 0.1)], _instr(3.0, 0.05, r=0.1), unresolved=unfitted,
                      occupancy_margin_m=0.05)
    assert s.status == "reobserve" and "unresolved_in_radius" in s.reasons


@pytest.mark.parametrize("margin", [-0.2, float("nan"), float("inf")])
def test_an_occupancy_margin_must_be_finite_and_not_negative(margin):
    """Codex implementation review 2026-10-24 P2: a negative or NaN margin loosened the inequality (NaN passes
    slack <= 0) and selected an ambiguous instruction."""
    with pytest.raises(ValueError):
        select_target([_c("a", 3.0, 0.0), _neighbour_behind()], _instr(3.0, 0.0, r=0.354), occupancy_margin_m=margin)
