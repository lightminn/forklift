# tests/unit/perception/test_evaluation_multi.py
import math

import pytest

from forklift_core.perception.evaluation_multi import (
    CandidateView, TargetDecision, _match, candidate_danger, judge_scene, summarize,
)


def _face(left, right, yaw=0.0, accessible=True, sign=1, pickable=True, thr=0.0437):
    return {"axis": "x", "sign": sign, "insertion_yaw_rad": yaw, "left_m": left, "right_m": right,
            "pocket_width_m": [0.2, 0.2], "pocket_height_m": 0.078, "accessible": accessible, "exposure": 1.0,
            "fit": {"pickable": pickable, "reasons": [] if pickable else ["fork_spacing_adjust"]},
            "danger_threshold_m": thr}


def _inst(k, y, *, eligible=True, pickable=True, thr=0.0437, layer=0):
    return {"instance_id": k, "known": True, "layer": layer, "support_z_m": 0.0, "eligible": eligible,
            "faces": [_face([2.0, y + 0.186, 0.061], [2.0, y - 0.186, 0.061], pickable=pickable, thr=thr),
                      _face([2.6, y - 0.186, 0.061], [2.6, y + 0.186, 0.061], yaw=math.pi, accessible=False, sign=-1)]}


META = {"scene_id": "s", "case": "S2", "shape_key": "epal6", "split": "eval", "render_source": "rig", "noise": {"k": 1}}
NONE_IN = {"present": False}


def _truth(insts, kind="none", target=None):
    return {"instances": insts, "instruction_truth": {"kind": kind, "target": target}}


def _cand(cid, y, dy=0.0, yaw=0.0, pickable=True, layer=0, dx=0.0):
    return CandidateView(cid, yaw, (2.0 + dx, y + 0.186 + dy, 0.061), (2.0 + dx, y - 0.186 + dy, 0.061),
                         True, True, pickable, (), layer)


def test_perfect_two_pallets_pass():
    v = judge_scene(META, _truth([_inst(0, 0.5), _inst(1, -0.5)]), NONE_IN,
                    [_cand("a", 0.5), _cand("b", -0.5)], TargetDecision("selected", "a", ()))
    assert (v.miss, v.dangerous, v.extra) == (False, False, False)


def test_back_face_candidate_is_not_matched():
    c = CandidateView("a", math.pi, (2.6, -0.186, 0.061), (2.6, 0.186, 0.061), True, True, False, (), 0)
    v = judge_scene(META, _truth([_inst(0, 0.0)]), NONE_IN, [c], TargetDecision("none", None, ()))
    assert v.miss and v.extra


def test_unmatched_pickable_duplicate_far_along_the_axis_is_dangerous():
    v = judge_scene(META, _truth([_inst(0, 0.0)]), NONE_IN, [_cand("a", 0.0), _cand("b", 0.0, dx=0.25)],
                    TargetDecision("selected", "b", ()))
    assert v.extra and v.dangerous


def test_lateral_plus_yaw_projection_is_dangerous():
    c = _cand("a", 0.0, dy=0.035, yaw=math.radians(3.9))  # 35 + 0.33 tan 3.9 deg = 57.5 mm > 43.7
    assert judge_scene(META, _truth([_inst(0, 0.0)]), NONE_IN, [c], TargetDecision("selected", "a", ())).dangerous


def test_nonfitting_pallet_correct_stop_vs_allowed_pick():
    meta = {**META, "case": "S7", "shape_key": "nonstandard"}
    truth = _truth([_inst(0, 0.0, pickable=False, thr=-0.0475)], "unambiguous", 0)
    instr = {"present": True, "x_m": 2.3, "y_m": 0.0, "radius_m": 0.3, "layer": None}
    stop = judge_scene(meta, truth, instr, [_cand("a", 0.0, pickable=False)],
                       TargetDecision("not_pickable", "a", ("fork_spacing_adjust",)))
    assert not stop.dangerous and stop.fit_error is False and stop.target_error is False
    pick = judge_scene(meta, truth, instr, [_cand("a", 0.0)], TargetDecision("selected", "a", ()))
    assert pick.dangerous and pick.fit_error


def test_nan_candidates_are_rejected():
    with pytest.raises(ValueError, match="finite"):
        CandidateView("a", 0.0, (float("nan"), 0.0, 0.0), (2.0, 0.0, 0.0), True, True, True, (), 0)


def test_instruction_rules():
    truth_amb = _truth([_inst(0, 0.2), _inst(1, -0.2)], "ambiguous", 0)
    instr = {"present": True, "x_m": 2.3, "y_m": 0.0, "radius_m": 0.5, "layer": None}
    cands = [_cand("a", 0.2), _cand("b", -0.2)]
    assert judge_scene(META, truth_amb, instr, cands, TargetDecision("selected", "a", ())).target_error
    assert judge_scene(META, truth_amb, instr, cands, TargetDecision("ask_operator", None, ())).target_error is False
    truth_un = _truth([_inst(0, 0.5), _inst(1, -0.5)], "unambiguous", 1)
    for status in ("ask_operator", "reobserve"):
        v = judge_scene(META, truth_un, instr, [_cand("a", 0.5), _cand("b", -0.5)], TargetDecision(status, None, ()))
        assert v.deferred_unambiguous and v.target_error is False


def test_selecting_without_an_absolute_layer_is_an_error():
    meta = {**META, "case": "S4"}
    truth = _truth([_inst(0, 0.0, layer=1)], "unambiguous", 0)
    instr = {"present": True, "x_m": 2.3, "y_m": 0.0, "radius_m": 0.3, "layer": 1}
    v = judge_scene(meta, truth, instr, [_cand("a", 0.0, layer=None)], TargetDecision("selected", "a", ()))
    assert v.target_error


def test_pose_gate_fails_on_a_30_mm_lateral_offset():
    verdicts = [judge_scene({**META, "scene_id": str(i)}, _truth([_inst(0, 0.0, thr=0.2)]), NONE_IN,
                            [_cand("a", 0.0, dy=0.03)], TargetDecision("selected", "a", ())) for i in range(114)]
    gates = summarize(verdicts, split="eval", render_source="rig")["gates"]
    assert not gates["known:epal6:pose_lateral_p95"]["pass"]


def test_empty_population_never_passes_and_known_has_no_fit_gate():
    gates = summarize([], split="eval", render_source="isaac")["gates"]
    assert gates and all(not g["pass"] for g in gates.values())
    assert not any(k.startswith("known") and k.endswith(":fit_error") for k in gates)


def test_opposite_pocket_errors_do_not_cancel():
    # Left pocket +60 mm, right pocket -60 mm laterally: the pair's mean is exact.
    c = CandidateView("a", 0.0, (2.0, 0.186 + 0.06, 0.071), (2.0, -0.186 - 0.06, 0.051), True, True, True, (), 0)
    v = judge_scene(META, _truth([_inst(0, 0.0, thr=0.2)]), NONE_IN, [c], TargetDecision("selected", "a", ()))
    lateral, _, _, z = v.errors[0]
    assert lateral == pytest.approx(0.06) and z == pytest.approx(0.01)


@pytest.mark.parametrize("truth", [{}, {"instances": [], "instruction_truth": {"kind": "none", "target": None}},
                                   {"instances": [_inst(0, 0.0)]}])
def test_a_positive_scene_with_missing_truth_is_refused(truth):
    with pytest.raises(ValueError, match="truth"):
        judge_scene(META, truth, NONE_IN, [], TargetDecision("none", None, ()))


def test_instruction_input_must_agree_with_the_instruction_truth():
    instr = {"present": True, "x_m": 2.3, "y_m": 0.0, "radius_m": 0.3, "layer": None}
    with pytest.raises(ValueError, match="instruction"):
        judge_scene(META, _truth([_inst(0, 0.0)]), instr, [], TargetDecision("none", None, ()))
    with pytest.raises(ValueError, match="instruction"):
        judge_scene(META, _truth([_inst(0, 0.0)], "unambiguous", 3), instr, [], TargetDecision("none", None, ()))


def _broken(**changes):
    inst = _inst(0, 0.0)
    face = dict(inst["faces"][0])
    for key, value in changes.items():
        if key in face:
            face[key] = value
        else:
            inst[key] = value
    inst["faces"] = [face] + inst["faces"][1:]
    return inst


@pytest.mark.parametrize("inst", [
    _broken(danger_threshold_m=float("nan")),
    _broken(left_m=[2.0, float("nan"), 0.061]),
    _broken(insertion_yaw_rad=float("inf")),
    _broken(fit={"reasons": []}),
    _broken(accessible="yes"),
    _broken(danger_threshold_m=True),
    _broken(left_m=[2.0, "0.186", 0.061]),
    {"eligible": False, "faces": []},
    {"instance_id": 0, "layer": 0, "faces": [_face([2.0, 0.186, 0.061], [2.0, -0.186, 0.061])]},
])
def test_malformed_instance_truth_is_refused(inst):
    with pytest.raises(ValueError, match="truth"):
        judge_scene(META, _truth([inst]), NONE_IN, [], TargetDecision("none", None, ()))


@pytest.mark.parametrize("chosen", ["ghost", "u"])
def test_selecting_an_unidentified_or_missing_candidate_is_dangerous(chosen):
    good = _cand("a", 0.0)
    unidentified = CandidateView("u", 0.0, good.left_m, good.right_m, True, False, True, (), 0)
    v = judge_scene(META, _truth([_inst(0, 0.0)]), NONE_IN, [good, unidentified],
                    TargetDecision("selected", chosen, ()))
    assert v.dangerous


def test_not_pickable_is_judged_on_the_face_the_candidate_matched():
    inst = _inst(0, 0.0)
    y_face = _face([2.3, 0.3, 0.045], [2.3, 0.0, 0.045], yaw=math.pi / 2, pickable=False)
    y_face["axis"] = "y"
    inst["faces"].append(y_face)  # x face pickable, y face not (near tracking unavailable)
    instr = {"present": True, "x_m": 2.3, "y_m": 0.0, "radius_m": 0.3, "layer": None}
    on_y = CandidateView("y", math.pi / 2, (2.3, 0.3, 0.045), (2.3, 0.0, 0.045), True, True, False,
                         ("near_tracking_unavailable",), 0)
    v = judge_scene(META, _truth([inst], "unambiguous", 0), instr, [on_y],
                    TargetDecision("not_pickable", "y", ("near_tracking_unavailable",)))
    assert v.target_error is False


def test_any_selection_in_a_negative_scene_is_a_false_positive():
    meta = {**META, "case": "N", "shape_key": "negative"}
    v = judge_scene(meta, _truth([]), NONE_IN, [], TargetDecision("selected", "ghost", ()))
    assert v.negative_fp


def _verdict(noise, render_source="isaac"):
    meta = {**META, "split": "dev", "render_source": render_source, "noise": noise}
    return judge_scene(meta, _truth([_inst(0, 0.0)]), NONE_IN, [_cand("a", 0.0)], TargetDecision("selected", "a", ()))


def test_official_needs_isaac_and_the_official_noise_role():
    official = {"config_sha256": "h", "scale": 1.0, "role": "official", "level": "mid"}
    sensitivity = {"config_sha256": "h", "scale": 1.66, "role": "sensitivity", "level": "mid"}
    provisional = {"config_sha256": "h", "scale": 1.0, "role": "provisional", "level": "low"}
    assert not summarize([_verdict(provisional)], split="dev", render_source="isaac")["official"]
    assert _verdict(provisional).noise_signature == "h:1.0:provisional:low"
    assert summarize([_verdict(official)], split="dev", render_source="isaac")["official"]
    assert not summarize([_verdict(sensitivity)], split="dev", render_source="isaac")["official"]
    assert not summarize([_verdict({"k": 1})], split="dev", render_source="isaac")["official"]
    assert not summarize([_verdict(None)], split="dev", render_source="isaac")["official"]
    assert _verdict(official).noise_signature == "h:1.0:official:mid" and _verdict({"k": 1}).noise_signature == "legacy"


def test_a_duplicate_of_a_matched_candidate_is_extra_but_not_dangerous():
    """Codex 2026-10-16: an unmatched pick-permitted candidate that can match a face is judged on that
    face, not flagged dangerous; candidate_danger gives exactly the answer judge_scene uses."""
    truth = _truth([_inst(0, 0.0)])
    cands = [_cand("a", 0.0), _cand("b", 0.0)]
    v = judge_scene(META, truth, NONE_IN, cands, TargetDecision("selected", "a", ()))
    assert v.extra and not v.dangerous
    pairs, costs = _match(cands, truth["instances"])
    assert [candidate_danger(i, c, pairs, costs, truth["instances"]) for i, c in enumerate(cands)] == [False, False]


def test_candidate_danger_agrees_with_the_unmatched_far_duplicate():
    truth = _truth([_inst(0, 0.0)])
    cands = [_cand("a", 0.0), _cand("b", 0.0, dx=0.25)]
    pairs, costs = _match(cands, truth["instances"])
    assert [candidate_danger(i, c, pairs, costs, truth["instances"]) for i, c in enumerate(cands)] == [False, True]
    assert candidate_danger(0, _cand("a", 0.0, pickable=None), pairs, costs, truth["instances"]) is None
