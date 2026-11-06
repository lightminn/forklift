"""The registered comparison tool (docs/plans/2026-11-06-near-capture-plane-refit.md): the replay path restored from the
records, the waiver computed per arm, the truth and error geometry, and each judged rule on hand-made results."""

import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.perception.pocket_detector import DetectorParams
from tools import near_refit_compare as C

GEOMETRY = {"overall_depth_m": 0.6, "overall_width_m": 0.8, "block_widths_m": [0.1, 0.145, 0.1]}
POSE = [[1.0, 2.0, 0.0], [1.0, 0.0, 0.0, 0.0]]


def state(far=None, **near):
    attempt = {"detector_reference": {"status": "valid"}, "detector_site": [3.42, 6.08, 1.61]}
    attempt.update(far or {})
    return {"near_capture": {"far_attempt": 1, "far_estimate_m": {"x_m": 3.4, "y_m": 6.1, "yaw_rad": 1.6}, **near},
            "observation_attempts": [attempt, {}]}


@pytest.mark.parametrize("far, why", [
    ({}, None),
    ({"detector_reference": {"status": "invalid"}}, "far_detector:invalid"),
    ({"lower_evidence": False}, "far_lower_waived"),
    ({"far_expected": {}}, "far_restricted"),
    ({"detector_site": None}, "far_detector_site_missing"),
    ({"detector_site": [3.7, 6.1, 1.6]}, "far_link"),
    ({"detector_site": [3.4, 6.1, 2.0]}, "far_link"),
])
def test_the_far_condition_comes_back_from_the_records_as_the_core_judged_it(far, why):
    assert C.far_waiver_refusal(state(far)) == why


def test_a_missing_far_attempt_refuses():
    assert C.far_waiver_refusal({"near_capture": {}, "observation_attempts": []}) == "far_attempt_missing"


EXPECTED = {"site": [2.0, 0.1, 0.02], "radius_m": 0.20, "axis_tolerance_rad": 0.26}
WAIVER_ARGS = {"near_capture_lower": "waive_far_confirmed"}


def test_a_near_attempt_replays_its_first_call_and_the_waiver_stays_open_when_the_far_condition_held():
    path = C.replay_path({"expected": EXPECTED}, state(), WAIVER_ARGS)
    assert path == {"kind": "near_expected", "site": [2.0, 0.1, 0.02], "radius_m": 0.20, "axis_rad": 0.26,
                    "lower_evidence": True, "waiver": "allowed"}


def test_an_applied_waiver_is_undone_back_to_the_first_call():
    narrow = {"site": [2.0, 0.1, 0.02], "radius_m": 0.15, "axis_tolerance_rad": 0.26}
    attempt = {"expected": narrow, "lower_evidence": False, "lower_waiver": {"applied": True, "first": {"expected": EXPECTED}}}
    path = C.replay_path(attempt, state(), WAIVER_ARGS)
    assert path["radius_m"] == 0.20 and path["lower_evidence"] is True and path["waiver"] == "allowed"


def test_a_refused_far_condition_keeps_the_second_call_closed_whatever_the_arm():
    path = C.replay_path({"expected": EXPECTED}, state({"lower_evidence": False}), WAIVER_ARGS)
    assert path["waiver"] == "refused:far_lower_waived"


def test_the_attempts_own_waiver_record_wins_over_the_state():
    refused = {"expected": EXPECTED, "lower_waiver": {"applied": False, "why": "far_link"}}
    assert C.replay_path(refused, {}, WAIVER_ARGS)["waiver"] == "refused:far_link"
    second_failed = {"expected": EXPECTED, "lower_waiver": {"applied": False, "why": "second:no_pallet:x"}}
    assert C.replay_path(second_failed, {}, WAIVER_ARGS)["waiver"] == "allowed"


def test_a_last_mission_without_state_records_leaves_the_waiver_unknown_and_closed():
    path = C.replay_path({"expected": EXPECTED}, {"observation_attempts": []}, WAIVER_ARGS)
    assert path["waiver"] == "unknown"
    fake = FakeDetector("no_pallet")
    _, second = C.detect_path(None, None, DetectorParams(), {**path}, "A1", detector=fake)
    assert second is False and len(fake.calls) == 1


def test_a_live_run_keeps_its_last_mission_attempts_even_with_a_depth_seen_before(tmp_path):
    records = tmp_path / "missions/m1"
    records.mkdir(parents=True)
    done = {"depth_sha256": "a", "t_before_capture_s": 1.0, "loop_step": 5}
    (records / "records.json").write_text(json.dumps({"state": {"observation_attempts": [done]}}))
    result = {"observation_attempts": [done, {"depth_sha256": "b", "t_before_capture_s": 9.0, "loop_step": 90},
                                       {"depth_sha256": "a", "t_before_capture_s": 9.5, "loop_step": 95}]}
    found = [(m, i, a["depth_sha256"]) for m, i, a, _ in C.run_attempts(tmp_path, result)]
    assert found == [("m1", 0, "a"), ("last", 1, "b"), ("last", 2, "a")]
    assert [m for m, *_ in C.run_attempts(tmp_path / "none", result)] == [None, None, None]


@pytest.mark.parametrize("attempt, arguments", [
    ({"expected": EXPECTED}, {}),  # the gate default
    ({"expected": EXPECTED, "lower_evidence": False}, WAIVER_ARGS),  # waived from the first call: nothing to fall to
])
def test_no_waiver_without_the_option_or_with_the_lower_band_already_off(attempt, arguments):
    assert C.replay_path(attempt, state(), arguments)["waiver"] == "off"


def test_far_and_plain_paths():
    far = C.replay_path({"far_expected": {**EXPECTED, "outside_zone": {"inside": False}}}, state(), WAIVER_ARGS)
    assert far["kind"] == "far_expected" and far["outside_zone_recorded"] is True and far["waiver"] == "off"
    assert C.replay_path({"lower_evidence": False}, state(), {}) == {"kind": "plain", "lower_evidence": False,
                                                                    "waiver": "off"}


def test_roles():
    near = {"expected": EXPECTED}
    s = {"near_capture": {"far_attempt": 1}, "observation_attempts": [{}, near, near]}
    assert C.attempt_role(0, {}, s, {}) == "far"
    assert C.attempt_role(1, near, s, {}) == "first_near"
    assert C.attempt_role(2, near, s, {}) == "reobserve"
    assert C.attempt_role(2, near, s, {"near_recapture_gap_m": 0.2}) == "second"
    assert C.attempt_role(1, {}, {"near_capture": {"far_attempt": 1}, "observation_attempts": [{}, {}]}, {}) == "near_no_site"


# --- geometry ---------------------------------------------------------------------------------------------------------

def perfect(truth, shift=(0.0, 0.0), yaw=0.0):
    d = np.array((math.cos(truth["yaw"]), math.sin(truth["yaw"])))
    l = np.array((-d[1], d[0]))
    face = np.asarray(truth["centre"]) - truth["depth_m"] / 2 * d + shift[0] * d + shift[1] * l
    middle = (truth["outer_m"] + truth["inner_m"]) / 2
    width = truth["outer_m"] - truth["inner_m"]
    return {"left": (face + middle * l).tolist(), "right": (face - middle * l).tolist(), "widths": [width, width],
            "yaw": truth["yaw"] + yaw}


def test_a_pallet_straight_ahead_in_the_capture_base():
    truth = C.truth_in_base((3.0, 2.0), 0.0, POSE, GEOMETRY)
    assert truth["centre"] == pytest.approx([2.0, 0.0]) and truth["yaw"] == 0.0
    assert (truth["outer_m"], truth["inner_m"]) == (pytest.approx(0.3), pytest.approx(0.0725))


def test_a_pallet_yawed_the_other_way_round_gives_the_face_toward_the_camera():
    truth = C.truth_in_base((3.0, 2.0), math.pi - 0.1, POSE, GEOMETRY)
    assert truth["yaw"] == pytest.approx(-0.1)


def test_the_entry_face_is_the_one_that_looks_at_the_camera_not_the_positive_x_half():
    # the reviewer's case: camera at (0.619, 0), pallet at (3, 1) of a base at the origin, pallet yaw -80 deg
    pose = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]
    truth = C.truth_in_base((3.0, 1.0), math.radians(-80.0), pose, GEOMETRY, camera_xy=(0.619, 0.0))
    assert truth["yaw"] == pytest.approx(math.radians(100.0))


def test_the_base_pose_is_taken_out_of_the_truth():
    q = [math.cos(0.25), 0.0, 0.0, math.sin(0.25)]  # base yaw 0.5
    truth = C.truth_in_base((1.0 + 2 * math.cos(0.5), 2.0 + 2 * math.sin(0.5)), 0.6, [[1.0, 2.0, 0.0], q], GEOMETRY)
    assert truth["centre"] == pytest.approx([2.0, 0.0]) and truth["yaw"] == pytest.approx(0.1)


@pytest.mark.parametrize("yaw", [0.0, 0.3, -0.5])
def test_a_perfect_detection_has_no_error(yaw):
    truth = C.truth_in_base((3.0, 2.0), yaw, POSE, GEOMETRY)
    errors = C.pose_errors(perfect(truth), truth)
    assert all(errors[p] == pytest.approx(0.0, abs=1e-12) for p in C.POINTS) and errors["match_m"] == pytest.approx(0.0, abs=1e-12)


def test_a_shift_lands_on_the_right_axis_and_a_narrow_width_only_on_the_walls():
    truth = C.truth_in_base((3.0, 2.0), 0.2, POSE, GEOMETRY)
    along = C.pose_errors(perfect(truth, shift=(0.007, 0.0)), truth)
    assert along["along_left"] == pytest.approx(0.007) and along["centre_left"] == pytest.approx(0.0, abs=1e-12)
    lateral = C.pose_errors(perfect(truth, shift=(0.0, 0.004)), truth)
    assert lateral["centre_right"] == pytest.approx(0.004) and lateral["wall_left_outer"] == pytest.approx(0.004)
    narrow = perfect(truth)
    narrow["widths"][0] -= 0.02
    errors = C.pose_errors(narrow, truth)
    assert errors["centre_left"] == pytest.approx(0.0, abs=1e-12)
    assert errors["wall_left_outer"] == pytest.approx(0.01) and errors["wall_left_inner"] == pytest.approx(0.01)
    assert errors["wall_right_outer"] == pytest.approx(0.0, abs=1e-12)


def clean_errors(**over):
    return {**{p: 0.0 for p in C.POINTS}, "yaw": 0.0, "match_m": 0.0, **over}


@pytest.mark.parametrize("over, passed", [
    ({}, True), ({"along_left": 0.008}, True), ({"along_right": 0.009}, False),  # the plan's own 9 mm case
    ({"wall_right_inner": 0.0101}, False), ({"centre_left": 0.0101}, False), ({"yaw": 0.011}, False),
    ({"match_m": 0.31}, False),
])
def test_the_one_quality_function(over, passed):
    assert C.quality_pass(clean_errors(**over)) is passed


# --- the call ----------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("arm, kw, ransac", [("A0", {}, 200), ("A1", {"plane_refit_iterations": 5}, 200), ("A2", {}, 400),
                                             ("A3", {"plane_refit_iterations": 5}, 400)])
def test_each_arm_carries_its_effective_values(arm, kw, ransac):
    params, got = C.arm_call(DetectorParams(), arm)
    assert got == kw and params.ransac_iterations == ransac


class FakeDetector:
    def __init__(self, *statuses):
        self.statuses, self.calls = list(statuses), []

    def __call__(self, scene, prior, params, site, **kw):
        self.calls.append((params.ransac_iterations, site, kw))
        return SimpleNamespace(observation=SimpleNamespace(status=self.statuses.pop(0))), ()


NEAR = {"kind": "near_expected", "site": [2.0, 0.1, 0.0], "radius_m": 0.2, "axis_rad": 0.26, "lower_evidence": True}


def test_the_expected_none_path_carries_the_arm():
    fake = FakeDetector("valid")
    C.detect_path(None, None, DetectorParams(), {"kind": "plain", "lower_evidence": True, "waiver": "off"}, "A3", detector=fake)
    assert fake.calls == [(400, None, {"lower_evidence": True, "plane_refit_iterations": 5})]


def test_a_first_call_valid_under_the_arm_never_reaches_the_waiver():
    fake = FakeDetector("valid")
    _, second = C.detect_path(None, None, DetectorParams(), {**NEAR, "waiver": "allowed"}, "A1", detector=fake)
    assert second is False and len(fake.calls) == 1


def test_a_first_call_invalid_goes_to_the_narrow_waiver_with_the_same_arm_and_stands_only_if_valid():
    fake = FakeDetector("no_pallet", "valid")
    detection, second = C.detect_path(None, None, DetectorParams(), {**NEAR, "waiver": "allowed"}, "A3", detector=fake)
    assert second is True and detection.observation.status == "valid"
    ransac, site, kw = fake.calls[1]
    assert ransac == 400 and site.radius_m == 0.15 and kw == {"lower_evidence": False, "plane_refit_iterations": 5}
    fake = FakeDetector("no_pallet", "invalid")
    detection, second = C.detect_path(None, None, DetectorParams(), {**NEAR, "waiver": "allowed"}, "A0", detector=fake)
    assert second is True and detection.observation.status == "no_pallet"


def test_a_refused_waiver_stays_closed_for_every_arm():
    fake = FakeDetector("no_pallet")
    _, second = C.detect_path(None, None, DetectorParams(), {**NEAR, "waiver": "refused:far_link"}, "A1", detector=fake)
    assert second is False and len(fake.calls) == 1


# --- the rules ----------------------------------------------------------------------------------------------------------

TRUTH = C.truth_in_base((3.0, 2.0), 0.0, POSE, GEOMETRY)


def valid(**kw):
    return {"status": "valid", "reason": None, "second_called": False, "canonical_sha256": "a", "residual_p95_m": 0.005,
            "plane_inlier_count": 100, "wall_s": 0.1, **perfect(TRUTH, **kw)}


def bad(reason="no_opening_pattern"):
    return {"status": "no_pallet", "reason": reason, "second_called": False, "canonical_sha256": "b",
            "residual_p95_m": None, "plane_inlier_count": 0, "wall_s": 0.1}


def entry(name="f", kind="near_expected", role="first_near", shape="epal6", sha="s", **kw):
    return {"id": name, "path": {"kind": kind}, "truth": TRUTH, "role": role, "shape": shape, "depth_sha256": sha,
            "e1": False, **kw}


def test_a_flip_is_refused_and_named():
    assert "flip:f" in C.degradation([(entry(), {"A0": valid(), "A1": bad()})], "A1")["problems"]


def test_a_shift_of_one_wall_hidden_by_the_frame_maximum_is_refused():
    a0, cand = valid(), valid()
    a0["widths"] = [a0["widths"][0] - 0.02, a0["widths"][1]]  # both left walls 10 mm off in A0
    cand["widths"] = [cand["widths"][0], cand["widths"][1] - 0.02]  # both right walls 10 mm off in the arm
    rows = [(entry(), {"A0": a0, "A1": cand})]
    assert any(p.startswith("point_worsened:f") for p in C.degradation(rows, "A1")["problems"])


def test_a_worsening_within_five_millimetres_passes_the_frame_rule():
    rows = [(entry(), {"A0": valid(), "A1": valid(shift=(0.0, 0.0049))})]
    assert not any(p.startswith("point_worsened") for p in C.degradation(rows, "A1")["problems"])


def test_a_new_valid_must_pass_the_quality_function_a_good_one_passes():
    rows = [(entry(), {"A0": bad(), "A1": valid(shift=(0.009, 0.0))})]
    assert C.degradation(rows, "A1")["problems"] == ["new_valid_quality:f"]
    assert C.degradation([(entry(), {"A0": bad(), "A1": valid(shift=(0.007, 0.0))})], "A1")["problems"] == []


def test_the_p95_rule_watches_each_group():
    rows = [(entry(name=str(i)), {"A0": valid(), "A1": valid(shift=(0.0015, 0.0))}) for i in range(30)]
    result = C.degradation(rows, "A1")
    assert result["problems"] == ["p95_along"] and result["common_valid"] == 30


def test_a_stratum_below_twenty_is_not_applied_rather_than_passed():
    rows = [(entry(name=str(i)), {"A0": valid(), "A1": valid()}) for i in range(19)]
    rows += [(entry(name=f"t{i}", shape="t11"), {"A0": valid(), "A1": valid(shift=(0.002, 0.0))}) for i in range(20)]
    strata = C.stratum_rule(rows, "A1")
    assert strata["epal6|first_near"] == {"applied": False, "common_valid": 19}
    assert strata["t11|first_near"]["applied"] is True and strata["t11|first_near"]["problems"] == ["p95_along"]


def test_the_e1_rescue_needs_position_walls_and_yaw():
    assert C.rescue(entry(), valid()) == []
    assert C.rescue(entry(), valid(shift=(0.006, 0.0))) == ["along_left", "along_right"]
    assert C.rescue(entry(), valid(yaw=0.003)) == ["yaw"]
    wall = valid()
    wall["widths"] = [wall["widths"][0] - 0.016, wall["widths"][1]]  # 8 mm at each left wall: within one cell
    assert C.rescue(entry(), wall) == []
    wall["widths"] = [wall["widths"][0] - 0.008, wall["widths"][1]]  # 12 mm: past it
    assert C.rescue(entry(), wall) == ["wall_left_outer", "wall_left_inner"]
    assert C.rescue(entry(), bad()) == ["status:no_pallet:no_opening_pattern"]


def test_timing_drops_the_warmup_and_compares_p95_and_maximum():
    def row(a0, arm):
        return entry(), {"A0": {"wall_s": a0, "second_called": False}, "A1": {"wall_s": arm, "second_called": False}}

    rows = [row(9.0, 99.0)] * C.WARMUP_ATTEMPTS + [row(0.1, 0.14)] * 40
    assert C.timing(rows, "A1")["passed"] is True
    assert C.timing([row(0.1, 0.1)] * C.WARMUP_ATTEMPTS + [row(0.1, 0.16)] * 40, "A1")["passed"] is False
    assert C.timing([row(0.1, 0.1)] * 60 + [row(0.1, 0.21)], "A1")["passed"] is False


def seeds(frames, e1=(4, 3)):
    return {"e1": {"A0": {"failures": e1[0], "seeds": 200}, "A1": {"failures": e1[1], "seeds": 200}},
            "frames": {f"f{i}": {"A0": {"failures": a, "seeds": 100}, "A1": {"failures": b, "seeds": 100}}
                       for i, (a, b) in enumerate(frames)}}


def test_the_seed_rule_counts_in_integers():
    assert C.seed_rule(seeds([(0, 0), (6, 5)]), "A1")["problems"] == []
    assert C.seed_rule(seeds([(10, 15), (6, 1)]), "A1")["problems"] == []  # exactly +5 is allowed
    assert C.seed_rule(seeds([(10, 16), (6, 0)]), "A1")["problems"] == ["frame:f0"]
    assert C.seed_rule(seeds([(0, 1)]), "A1")["problems"] == ["mean_failure_rate"]
    assert C.seed_rule(seeds([(0, 0)], e1=(4, 5)), "A1")["problems"] == ["e1_failures"]


def many_entries(n):
    return [entry(name=f"f{i:02d}", sha=f"{i:02d}") for i in range(n)]


def test_the_seed_sample_is_uniform_then_highest_residual_with_duplicates_and_e1_out():
    entries = many_entries(40)
    entries.append(entry(name="dup", sha="03"))
    entries.append(entry(name="e1", sha="99", e1=True))
    entries.append(entry(name="e1again", sha="99"))
    entries.append(entry(name="far", sha="77", kind="far_expected"))
    a0 = {e["id"]: {"residual_p95_m": float(e["depth_sha256"]) / 1000} for e in entries}
    chosen = C.seed_sample(entries, a0)
    assert [e["id"] for e in chosen[:10]] == [f"f{i:02d}" for i in range(0, 40, 4)]
    assert [e["id"] for e in chosen[10:]] == ["f39", "f38", "f37", "f35", "f34", "f33", "f31", "f30", "f29", "f27"]


def test_the_seed_lists_are_the_registered_ones():
    assert C.E1_SEEDS[0] == 20260913 and len(C.E1_SEEDS) == 200 and C.E1_SEEDS[-1] == 20260913 + 199
    assert C.FRAME_SEEDS == tuple(range(100))


def neg_rows(valid_scenes=(), arms=C.ARMS, skip=()):
    """The whole registered population; ``valid_scenes`` holds (arm, scene, prior, lower) that came out valid."""
    rows = []
    for level in C.E3_LEVELS:
        for number in range(C.E3_SCENES_PER_LEVEL):
            for prior in ("epal6", "t11_06"):
                for lower in (True, False):
                    for arm in arms:
                        key = (arm, f"n{number}", prior, lower)
                        if (level, *key) not in skip:
                            rows.append({"level": level, "scene": f"n{number}", "prior": prior, "lower": lower,
                                         "arm": arm, "valid": (level, *key) in valid_scenes})
    return {"rows": rows, "errors": []}


def test_negatives_count_per_level_mode_prior_and_union_and_name_new_scenes():
    swapped = {("isaac_clean", "A0", "n0", "epal6", True), ("isaac_clean", "A1", "n1", "epal6", True)}
    result = C.negative_rule(neg_rows(swapped), "A1")
    assert result["problems"] == [] and result["counts"]["isaac_clean|lower=True|epal6"]["new"] == ["n1"]
    worse = C.negative_rule(neg_rows(swapped | {("isaac_clean", "A1", "n2", "epal6", True)}), "A1")
    assert worse["problems"] == ["isaac_clean|lower=True|epal6", "isaac_clean|lower=True|any"]


def test_the_lower_off_mode_is_judged_on_its_own():
    assert C.negative_rule(neg_rows({("isaac_mid", "A1", "n3", "t11_06", False)}), "A1")["problems"] == [
        "isaac_mid|lower=False|t11_06", "isaac_mid|lower=False|any"]


@pytest.mark.parametrize("e3", [
    {"rows": [], "errors": []},
    {"rows": neg_rows()["rows"], "errors": ["bad scene"]},
    neg_rows(arms=("A0",)),  # a missing arm is not zero false positives
    neg_rows(skip={("isaac_low", "A1", "n7", "epal6", True)}),
    {"rows": neg_rows()["rows"] + neg_rows()["rows"][:1], "errors": []},
    {"rows": [r for r in neg_rows()["rows"] if r["scene"] != "n40"], "errors": []},  # a smaller population
])
def test_a_negative_set_that_is_not_complete_is_undecidable(e3):
    assert "undecidable" in C.negative_rule(e3, "A1")


def test_grounded_is_read_off_the_rank_as_the_scene_builder_did():
    assert C.negative_kind({"rank": 0}) == "grounded" and C.negative_kind({"rank": 4}) == "grounded"
    assert [C.negative_kind({"rank": r}) for r in (1, 2, 3)] == ["shelf", "blocks", "crates"]


def recorded(**over):
    return {"status": "valid", "reason": None, "yaw": 0.1, "left": [1.0, 2.0], "right": [1.0, 1.5],
            "widths": [0.22, 0.23], **over}


def replayed(**over):
    return {**recorded(), **over}


@pytest.mark.parametrize("over", [dict(left=[1.0, 2.00002]), dict(right=[3.00002, 1.5]), dict(widths=[0.22, 0.2301]),
                                  dict(yaw=0.10001), dict(status="no_pallet", reason="x")])
def test_any_field_of_the_observation_counts_at_one_micrometre_absolute(over):
    entry_ = {"recorded": recorded(right=[3.0, 1.5]) if "right" in over else recorded(), "path": {}}
    assert C.classify_mismatch(entry_, replayed(**over)) == "unexplained"


def test_an_equal_observation_and_a_recorded_zone_refusal():
    assert C.classify_mismatch({"recorded": recorded(), "path": {}}, replayed()) is None
    far = {"recorded": recorded(status="no_pallet", reason="no_expected_candidate:outside_zone"),
           "path": {"outside_zone_recorded": True}}
    assert C.classify_mismatch(far, replayed()) == "far_outside_zone_not_replayed"


def gate_entry(name="f", **over):
    record = valid()
    fields = {k: record[k] for k in ("left", "right", "widths", "yaw")}
    return {**entry(name=name), "recorded": {"status": "valid", "reason": None, **fields}, "tilt_rad": 0.001,
            "target_id": "P-1", "sample_pallet_id": "P-1", "sample_age_s": 0.05, **over}


def gate_run(entries, record=None):
    return {"frames": [{"id": e["id"], "arms": {"A0": record or valid()}} for e in entries]}


def test_the_freeze_gate_passes_only_a_clean_set():
    e1 = gate_entry("e1", e1=True)
    assert C.freeze_gate({"entries": [e1, gate_entry()]}, gate_run([e1, gate_entry()]))["problems"] == []


@pytest.mark.parametrize("over, word", [(dict(tilt_rad=0.03), "tilt"), (dict(sample_pallet_id="P-2"), "target"),
                                        (dict(truth=None), "no_truth")])
def test_the_freeze_gate_refuses_a_broken_truth_assumption(over, word):
    e1, bad_entry = gate_entry("e1", e1=True), gate_entry(**over)
    problems = C.freeze_gate({"entries": [e1, bad_entry]}, gate_run([e1, bad_entry]))["problems"]
    assert len(problems) == 1 and problems[0].startswith(word)


def test_the_freeze_gate_refuses_a_truth_that_disagrees_with_a_valid_a0():
    e1 = gate_entry("e1", e1=True)
    far = {**gate_entry(), "recorded": gate_entry()["recorded"]}
    record = valid(shift=(0.0, 0.5))
    far["recorded"] = {"status": "valid", "reason": None, **{k: record[k] for k in ("left", "right", "widths", "yaw")}}
    run = {"frames": [{"id": "e1", "arms": {"A0": valid()}}, {"id": "f", "arms": {"A0": record}}]}
    assert any(p.startswith("truth_disagrees:f") for p in C.freeze_gate({"entries": [e1, far]}, run)["problems"])


def test_a_plain_far_detection_on_a_neighbour_is_counted_not_a_gate_failure():
    e1 = gate_entry("e1", e1=True)
    record = valid(shift=(0.0, 1.0))
    far = {**gate_entry(), "role": "far",
           "recorded": {"status": "valid", "reason": None, **{k: record[k] for k in ("left", "right", "widths", "yaw")}}}
    run = {"frames": [{"id": "e1", "arms": {"A0": valid()}}, {"id": "f", "arms": {"A0": record}}]}
    result = C.freeze_gate({"entries": [e1, far]}, run)
    assert result["problems"] == [] and result["far_on_another_pallet"] == 1


def test_a_last_mission_has_no_target_to_compare():
    assert C.mission_targets(Path("/nonexistent"), {"mission_results": [{"mission_id": "m1", "pallet_id": "P-1"}]}) == {
        "m1": "P-1"}


def test_the_freeze_gate_refuses_duplicates_and_a_missing_e1():
    problems = C.freeze_gate({"entries": [gate_entry(), gate_entry()]}, gate_run([gate_entry()]))["problems"]
    assert "duplicate_id:f" in problems and "e1_not_exactly_one" in problems


EVIDENCE = {"replay_gate": [29, 29], "full_pytest_failed": 0, "stored_eval_changes": 0,
            "near_contract_default_match": True, "v1_guard_baseline": "skipped: artifacts absent"}


def judged(arm_record, negatives=None, evidence=EVIDENCE, seed_frames=None, stamps=None, far_arm=None):
    e1 = entry(name="e1", sha="e1", e1=True)
    others = many_entries(C.WARMUP_ATTEMPTS + 15)
    far = entry(name="far", sha="far", kind="plain", role="far")
    entries = [e1, *others, far]
    manifest = {"entries": entries}
    arms = {"A0": bad(), **{a: arm_record for a in C.CANDIDATES}}
    frames = [{"id": "e1", "arms": arms}] + [{"id": e["id"], "arms": {a: valid() for a in C.ARMS}} for e in others]
    frames.append({"id": "far", "arms": {"A0": valid(), **{a: far_arm or valid() for a in C.CANDIDATES}}})
    replay = {"frames": frames}
    a0 = {f["id"]: f["arms"]["A0"] for f in frames}
    picked = [e["id"] for e in C.seed_sample(entries, a0)]
    seed = {"e1": {a: {"failures": 4, "seeds": 200} for a in C.ARMS}, "e1_seeds": list(C.E1_SEEDS),
            "frame_seeds": list(C.FRAME_SEEDS),
            "frames": seed_frames or {i: {a: {"failures": 0, "seeds": 100} for a in C.ARMS} for i in picked}}
    e3 = negatives or neg_rows()
    return C.judge(manifest, replay, replay, seed, e3, {"rows": [], "skipped": {"x": "mount_unknown"}}, evidence, stamps)


def test_the_judge_passes_an_arm_that_rescues_and_breaks_nothing():
    result = judged(valid())
    assert result["verdicts"]["A1"]["verdict"] == "adoptable"
    assert result["recommended_order"] == ["A1", "A3", "A2"]


def test_the_judge_refuses_an_arm_that_does_not_rescue():
    assert judged(bad())["verdicts"]["A1"]["failed"] == ["rescue"]


def test_an_unread_negative_set_leaves_a_passing_arm_undecidable():
    result = judged(valid(), negatives={"rows": [], "errors": ["x"]})
    assert result["verdicts"]["A1"]["verdict"] == "undecidable" and result["recommended_order"] == []


@pytest.mark.parametrize("gap", ["replay_gate", "full_pytest_failed", "stored_eval_changes",
                                 "near_contract_default_match", "v1_guard_baseline"])
def test_without_the_outside_evidence_no_arm_reads_adoptable(gap):
    evidence = {k: v for k, v in EVIDENCE.items() if k != gap}
    result = judged(valid(), evidence=evidence)
    assert result["verdicts"]["A1"]["verdict"] == "unverified" and result["recommended_order"] == []


def test_a_seed_set_that_is_not_the_registered_twenty_is_undecidable():
    one = {"f00": {a: {"failures": 0, "seeds": 100} for a in C.ARMS}}
    assert judged(valid(), seed_frames=one)["verdicts"]["A1"]["undecided"] == ["seed_frames"]


def test_outputs_of_another_manifest_or_source_are_undecidable():
    stamps = {"replay": {"m": 1}, "seeds": {"m": 2}}
    assert judged(valid(), stamps=stamps)["verdicts"]["A1"]["undecided"] == ["stamp:seeds"]


def test_an_arm_result_on_a_call_it_never_reaches_is_a_reference_not_a_failure():
    result = judged(valid(), far_arm=bad())["verdicts"]["A1"]
    assert result["verdict"] == "adoptable" and "flip:far" in result["reference"]["problems"]


def toy_meta(coeffs=(0.0,) * 5, notes="camera lens height 0.27 m on a stand, level", post=()):
    return {"device": {"depth_scale_m": 0.001, "post_processing": list(post),
                       "intrinsics": {"fx": 400.0, "fy": 400.0, "ppx": 2.0, "ppy": 1.0, "coeffs": list(coeffs)}},
            "setup": {"notes": notes}}


def test_a_level_camera_on_a_stand_sees_the_floor_below_and_the_axis_forward():
    from forklift_core.sensors.rgbd import deproject_depth_pixels  # noqa: F401 -- the grid contract under test

    z16 = np.array(((1000, 1000, 0, 1000), (1000, 1000, 1000, 1000), (1000, 1000, 1000, 1000)), np.uint16)
    scene, why = C.toy_input(toy_meta(), z16)
    assert why is None and np.isnan(scene.depth_m[0, 2]) and scene.depth_m[1, 2] == pytest.approx(1.0)
    point = scene.base_from_optical.rotation @ np.array((0.0, 0.0, 1.0)) + scene.base_from_optical.translation_m
    assert point == pytest.approx([1.0, 0.0, 0.27])
    right = scene.base_from_optical.rotation @ np.array((1.0, 0.0, 0.0))
    down = scene.base_from_optical.rotation @ np.array((0.0, 1.0, 0.0))
    assert right == pytest.approx([0.0, -1.0, 0.0]) and down == pytest.approx([0.0, 0.0, -1.0])


@pytest.mark.parametrize("meta, why", [
    (toy_meta(coeffs=(0.1, 0.0, 0.0, 0.0, 0.0)), "not_rectified"),
    (toy_meta(post=("spatial",)), "not_rectified"),
    (toy_meta(notes="on a tripod"), "mount_unknown"),
    (toy_meta(notes="camera lens height 0.27 m, tilted"), "mount_unknown"),
])
def test_a_capture_that_does_not_state_its_grid_or_mount_is_not_run(meta, why):
    assert C.toy_input(meta, np.ones((3, 4), np.uint16))[1] == why


def test_a_spread_that_cannot_be_compared_is_not_a_flag():
    rows = [{"capture": "toy_1", "prior": "toy", "arm": "A0", "kind": "toy", "valid": 1},
            {"capture": "toy_1", "prior": "toy", "arm": "A1", "kind": "toy", "valid": 3, "centre_std_m": 0.01}]
    assert C.toy_flags({"rows": rows}, "A1") == []
    assert "front_minus_tape_m" not in C.toy_row("c", "toy", "toy", "A1", 30, [valid()])


def test_toy_flags_are_flags():
    rows = [{"capture": "toy_1", "prior": "toy", "arm": "A0", "kind": "toy", "valid": 30, "centre_std_m": 0.001},
            {"capture": "toy_1", "prior": "toy", "arm": "A1", "kind": "toy", "valid": 29, "centre_std_m": 0.0035},
            {"capture": "wall_1", "prior": "toy", "arm": "A0", "kind": "wall", "valid": 0},
            {"capture": "wall_1", "prior": "toy", "arm": "A1", "kind": "wall", "valid": 1}]
    assert C.toy_flags({"rows": rows}, "A1") == ["valid_fell:toy_1", "spread_grew:toy_1", "wall_valid:wall_1:toy"]
