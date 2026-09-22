"""CPU tests of the final G1 decision, including height distance coverage."""

import copy
import runpy
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
FINISH = runpy.run_path(str(ROOT / "sim/isaac/verify_perception_camera.py"))[
    "finish_result"
]


PROVENANCE_KEYS = (
    "panel_placement_K_reference",
    "selection_fit_reference",
    "selection_coverage",
)
MISSING_PROVENANCE = "panel_provenance_missing"


def _fit_reference(index, *, role, gate_2a="PASS"):
    # The record save_height_selection and the panel write: one fitted K, named
    # once per role, carrying the gate that validated it.
    return {
        "collection": "intrinsics_fits",
        "index": index,
        "anchor_horizontal_m": (0.8, 1.0, 2.0, 3.0, 4.0, 5.0)[index],
        "repeat": 0,
        "repeat_kind": "measured",
        "warmup_capture_excluded": True,
        "gate_2a": gate_2a,
        "role": role,
    }


@pytest.fixture
def complete_result():
    # Each panel observes one distance; other distances are genuinely unobserved.
    # Distinct dictionaries ensure a missing bin cannot be hidden by aliasing.
    # A measured panel carries the provenance a real run records for it: both
    # fit roles and the selection coverage. A fixture without them would be an
    # invalid run, and would exercise the gate's fail-open path as if it were
    # the happy path.
    return {
        "perception_camera_intrinsics": {"status": "PASS"},
        "mount": {"status": "PASS"},
        "intrinsics_fits": [{"gate_2a": "PASS", "gate_2b": "PASS"} for _ in range(18)],
        "boards": [{"gate_4": "PASS", "status": "PASS"} for _ in range(162)],
        "height_panels": [
            {
                "status": "PASS",
                "panel_placement_K_reference": _fit_reference(
                    panel // 9, role="height_panel_physical_placement"
                ),
                "selection_fit_reference": _fit_reference(
                    panel // 9, role="height_grid_sample_selection"
                ),
                "selection_coverage": {"status": "COMPLETE", "lost_bins": []},
                "captures": [
                    {
                        "status": "PASS",
                        "per_distance": [
                            {
                                "distance_bin": distance,
                                "count": 20 if distance == panel // 9 else 0,
                                "status": "PASS"
                                if distance == panel // 9
                                else "UNOBSERVED",
                            }
                            for distance in range(6)
                        ],
                    }
                    for _ in range(3)
                ],
            }
            for panel in range(54)
        ],
        "gate_5": "PASS",
        "marker": [{"bright_centroid_outside_marker_bbox": True}],
    }


def test_complete_numerics_require_user_review_and_never_authorize_g2(complete_result):
    FINISH(complete_result)
    assert complete_result["gates"]["7a_prime"] == "PASS"
    assert complete_result["gates"]["7b"] == "RECORD_ONLY"
    assert complete_result["gates"]["6"] == "USER_JUDGMENT_REQUIRED"
    assert complete_result["numerical_status"] == "PASS"
    assert complete_result["status"] == "REVIEW_REQUIRED"
    assert complete_result["g2_allowed"] is False


@pytest.mark.parametrize("missing_bin", range(6))
def test_final_height_gate_refuses_each_unobserved_distance(
    complete_result, missing_bin
):
    for panel in complete_result["height_panels"]:
        for capture in panel["captures"]:
            group = capture["per_distance"][missing_bin]
            group.update(status="UNOBSERVED", count=0)
    FINISH(complete_result)
    assert complete_result["gates"]["7a_prime"] == "FAIL"
    assert complete_result["numerical_status"] == complete_result["status"] == "FAIL"
    assert complete_result["g2_allowed"] is False


@pytest.mark.parametrize("empty_capture_list", [False, True])
def test_final_height_gate_never_promotes_all_unobserved_panels(
    complete_result, empty_capture_list
):
    for panel in complete_result["height_panels"]:
        panel["status"] = "UNOBSERVED"
        for capture in panel["captures"]:
            capture["status"] = "UNOBSERVED"
            for group in capture["per_distance"]:
                group.update(status="UNOBSERVED", count=0)
        if empty_capture_list:
            panel["captures"] = []
    FINISH(complete_result)
    assert complete_result["gates"]["7a_prime"] == "FAIL"
    assert complete_result["status"] == "FAIL"


def test_local_unobserved_panel_is_allowed_when_all_distances_are_measured(
    complete_result,
):
    complete_result["height_panels"][0] = {"status": "UNOBSERVED", "captures": []}
    FINISH(complete_result)
    assert complete_result["gates"]["7a_prime"] == "PASS"
    assert complete_result["status"] == "REVIEW_REQUIRED"


def _unvalidated_panel(panel, *, index, anchor):
    # A fit that never passed gate 2a both placed and sampled this panel.
    panel.update(
        selection_fit_unvalidated=True,
        panel_placement_K_reference={
            "collection": "intrinsics_fits",
            "index": index,
            "anchor_horizontal_m": anchor,
            "repeat": 0,
            "gate_2a": "FAIL",
            "role": "height_panel_physical_placement",
        },
    )
    return panel["panel_placement_K_reference"]


def test_validated_height_gate_records_an_empty_reason_list(complete_result):
    FINISH(complete_result)
    assert complete_result["gates"]["7a_prime"] == "PASS"
    assert complete_result["gate_7a_prime_detail"] == {
        "status": "PASS",
        "reasons": [],
        "unvalidated_selection_fits": [],
    }


def test_unvalidated_selection_fit_forces_the_height_gate_without_aborting(
    complete_result,
):
    # Every other numeric is a PASS: only the fit's provenance fails the gate.
    reference = _unvalidated_panel(
        complete_result["height_panels"][0], index=0, anchor=0.8
    )
    FINISH(complete_result)
    assert complete_result["gates"]["7a_prime"] == "FAIL"
    assert complete_result["gate_7a_prime_detail"] == {
        "status": "FAIL",
        "reasons": ["selection_fit_unvalidated"],
        "unvalidated_selection_fits": [reference],
    }
    assert complete_result["status"] == "FAIL"
    assert complete_result["g2_allowed"] is False
    # The panel numbers stay on record as diagnostics, never deleted.
    assert complete_result["height_panels"][0]["captures"]


def test_one_unvalidated_fit_shared_by_several_panels_is_named_once(complete_result):
    for panel in complete_result["height_panels"][:9]:
        reference = _unvalidated_panel(panel, index=0, anchor=0.8)
    FINISH(complete_result)
    assert complete_result["gate_7a_prime_detail"]["unvalidated_selection_fits"] == [
        reference
    ]


def test_forced_and_genuine_height_failures_stay_separately_visible(complete_result):
    complete_result["height_panels"][0]["selection_coverage"] = {
        "status": "FAIL",
        "lost_bins": [[0, 1, 1]],
    }
    complete_result["height_panels"][1]["status"] = "FAIL"
    reference = _unvalidated_panel(
        complete_result["height_panels"][2], index=3, anchor=1.0
    )
    FINISH(complete_result)
    detail = complete_result["gate_7a_prime_detail"]
    assert detail["status"] == complete_result["gates"]["7a_prime"] == "FAIL"
    # Three independent defects; none may absorb another.
    assert detail["reasons"] == [
        "panel_measurement_failed",
        "selection_coverage_lost",
        "selection_fit_unvalidated",
    ]
    assert detail["unvalidated_selection_fits"] == [reference]


def test_panel_geometry_failure_fails_the_gate_without_aborting_the_run(
    complete_result,
):
    # A panel whose geometry or margin check failed is recorded and named, the
    # same way an unvalidated fit is. Aborting would leave no gate table at all.
    complete_result["height_panels"][0].update(
        status="FAIL",
        panel_geometry_failed=True,
        panel_geometry_failed_reason="panel_geometry_failed",
        panel_geometry_failed_detail="ValueError: margin does not clear erosion",
    )
    FINISH(complete_result)
    assert complete_result["gates"]["7a_prime"] == "FAIL"
    assert complete_result["gate_7a_prime_detail"]["reasons"] == [
        "panel_measurement_failed",
        "panel_geometry_failed",
    ]
    assert complete_result["status"] == "FAIL"
    assert complete_result["g2_allowed"] is False
    # The other 53 panels keep their measurements.
    assert len(complete_result["height_panels"]) == 54
    assert complete_result["height_panels"][1]["captures"]


def test_the_gate_checks_the_selection_role_not_only_the_placement_role(
    complete_result,
):
    # The two uses of one fitted K are recorded separately precisely so a later
    # change can move them apart. A selection fit that never passed gate 2a
    # must fail the gate even when the fit that placed the panel did pass.
    panel = complete_result["height_panels"][0]
    panel["panel_placement_K_reference"] = {
        "collection": "intrinsics_fits",
        "index": 0,
        "anchor_horizontal_m": 0.8,
        "repeat": 0,
        "gate_2a": "PASS",
        "role": "height_panel_physical_placement",
    }
    panel["selection_fit_reference"] = {
        "collection": "intrinsics_fits",
        "index": 4,
        "anchor_horizontal_m": 0.8,
        "repeat": 0,
        "gate_2a": "FAIL",
        "role": "height_grid_sample_selection",
    }
    FINISH(complete_result)
    assert complete_result["gates"]["7a_prime"] == "FAIL"
    detail = complete_result["gate_7a_prime_detail"]
    assert detail["reasons"] == ["selection_fit_unvalidated"]
    assert detail["unvalidated_selection_fits"] == [panel["selection_fit_reference"]]


def test_final_height_gate_refuses_margin_lost_screen_bin(complete_result):
    # Other panels still cover every distance. Losing a required screen cell
    # must remain a separate failure even if numerical statistics all pass.
    complete_result["height_panels"][0]["selection_coverage"] = {
        "status": "FAIL",
        "lost_bins": [[0, 1, 1]],
    }
    FINISH(complete_result)
    assert complete_result["gates"]["7a_prime"] == "FAIL"
    assert complete_result["status"] == "FAIL"


@pytest.mark.parametrize(
    "section,field,gate,blocking",
    [
        # Gate 2a keeps its measured value in the table but no longer feeds the
        # numerical aggregate (user decision, 2026-09-21), so a failed fit no
        # longer blocks by itself. Dropping a fit still blocks, through 2b.
        ("intrinsics_fits", "gate_2a", "2a", False),
        ("intrinsics_fits", "gate_2b", "2b", True),
        ("boards", "gate_4", "4", True),
        ("boards", "status", "7a", True),
        ("height_panels", "status", "7a_prime", True),
    ],
)
@pytest.mark.parametrize("defect", ["failed_record", "missing_record"])
def test_final_gate_refuses_one_failed_or_missing_record(
    complete_result, section, field, gate, defect, blocking
):
    if defect == "failed_record":
        complete_result[section][0][field] = "FAIL"
    else:
        complete_result[section].pop()
    FINISH(complete_result)
    assert complete_result["gates"][gate] == "FAIL"
    blocks = blocking or defect == "missing_record"
    assert complete_result["numerical_status"] == ("FAIL" if blocks else "PASS")
    assert complete_result["status"] == ("FAIL" if blocks else "REVIEW_REQUIRED")
    assert complete_result["g2_allowed"] is False


def test_a_dropped_fit_still_blocks_through_gate_2b(complete_result):
    # 2a is diagnostic now, so the count of fits must still be enforced by 2b.
    complete_result["intrinsics_fits"].pop()
    FINISH(complete_result)
    assert complete_result["gates"]["2a"] == complete_result["gates"]["2b"] == "FAIL"
    assert complete_result["numerical_status"] == "FAIL"


@pytest.mark.parametrize("key", PROVENANCE_KEYS)
@pytest.mark.parametrize("defect", ["absent", "none", "string", "empty"])
def test_a_measured_panel_without_its_provenance_fails_the_gate(
    complete_result, key, defect
):
    # An absent record is not a validated one. The gate reads a fit's gate_2a
    # and the selection's coverage status; a panel carrying neither would pass
    # both checks by omission, which is the fail-open shape this closes.
    panel = complete_result["height_panels"][0]
    if defect == "absent":
        panel.pop(key)
    else:
        panel[key] = {"none": None, "string": "intrinsics_fits[0]", "empty": {}}[defect]
    FINISH(complete_result)
    assert complete_result["gates"]["7a_prime"] == "FAIL"
    assert MISSING_PROVENANCE in complete_result["gate_7a_prime_detail"]["reasons"]
    assert complete_result["numerical_status"] == complete_result["status"] == "FAIL"
    assert complete_result["g2_allowed"] is False


def test_a_panel_that_supplies_coverage_must_carry_its_provenance(complete_result):
    # covered_ranges counts a capture's passing distance bins whatever the
    # panel's own status, so a panel recorded as UNOBSERVED can still supply
    # one of the six required distances. Deleting its provenance must not turn
    # that contribution into a validated measurement.
    for panel in complete_result["height_panels"][:9]:
        panel["status"] = "UNOBSERVED"
        for key in PROVENANCE_KEYS:
            panel.pop(key)
    FINISH(complete_result)
    assert complete_result["gates"]["7a_prime"] == "FAIL"
    assert complete_result["gate_7a_prime_detail"]["reasons"] == [MISSING_PROVENANCE]


def test_an_early_exit_unobserved_panel_needs_no_provenance(complete_result):
    # A panel that never reached save_height_selection supplies nothing: it has
    # no captures, contributes no distance bin, and must stay exempt.
    complete_result["height_panels"][0] = {"status": "UNOBSERVED", "captures": []}
    FINISH(complete_result)
    assert complete_result["gates"]["7a_prime"] == "PASS"
    assert complete_result["gate_7a_prime_detail"]["reasons"] == []


def test_gate_2a_is_recorded_but_no_longer_blocks_the_numerical_aggregate(
    complete_result,
):
    # User decision, 2026-09-21. The measurement is unchanged: 2a keeps its
    # value in the gate table and only leaves the numerical aggregate.
    complete_result["intrinsics_fits"][0]["gate_2a"] = "FAIL"
    FINISH(complete_result)
    assert complete_result["gates"]["2a"] == "FAIL"
    assert complete_result["numerical_status"] == "PASS"
    assert complete_result["status"] == "REVIEW_REQUIRED"
    assert complete_result["g2_allowed"] is False
    # A reader of result.json alone must see which gates were excluded and why.
    assert set(complete_result["diagnostic_only_gates"]) == {"2a", "7b"}
    assert "pocket_detector" in complete_result["diagnostic_only_gates"]["2a"]
    # The marker diagnosis still reads the measured 2a; it is not cascaded.
    assert (
        complete_result["marker_diagnosis"]["conclusion"]
        == "camera_model_or_transform_discrepancy"
    )


def test_demoting_gate_2a_does_not_excuse_an_unvalidated_selection_fit(complete_result):
    # Regression guard, not a driver: 7a-prime judges the gate 2a copied into
    # the fit references it actually reads, which is narrower than the global
    # gate and survives the demotion.
    reference = _unvalidated_panel(
        complete_result["height_panels"][0], index=0, anchor=0.8
    )
    FINISH(complete_result)
    assert complete_result["gates"]["2a"] == "PASS"
    assert complete_result["gates"]["7a_prime"] == "FAIL"
    assert complete_result["gate_7a_prime_detail"]["reasons"] == [
        "selection_fit_unvalidated"
    ]
    assert complete_result["gate_7a_prime_detail"]["unvalidated_selection_fits"] == [
        reference
    ]
    assert complete_result["numerical_status"] == "FAIL"


# The measured marker record run 8 wrote, as the user's judgement cites it.
MARKER_MEASURED = {
    "semantic_minus_projected_centre_px": [5.684341886080802e-14, 0.1392097168059081],
    "bright_minus_projected_centre_px": [29.48544866612781, -27.688754725562035],
    "semantic_count": 1052,
    "bright_count": 1237,
    "bright_outside_marker_count": 185,
    "bright_centroid_outside_marker_bbox": True,
    "sphere_silhouette_is_not_projected_centre": True,
}
ANCHOR_KEY = "0.8=0;1.0=0;2.0=0;3.0=0;4.0=0;5.0=0"


def test_gate_6_keeps_its_value_and_carries_the_recorded_user_judgment(complete_result):
    # USER decision, 2026-09-22. Gate 6 is USER_JUDGMENT_REQUIRED by design and
    # stays that way; what is added is the decision itself, in result.json.
    FINISH(complete_result)
    assert complete_result["gates"]["6"] == "USER_JUDGMENT_REQUIRED"
    assert complete_result["marker_diagnosis"]["status"] == "USER_JUDGMENT_REQUIRED"
    assert complete_result["numerical_status"] == "PASS"
    assert complete_result["status"] == "REVIEW_REQUIRED"
    assert complete_result["g2_allowed"] is False
    judgment = complete_result["marker_diagnosis"]["user_judgment"]
    assert judgment["decided"] == "2026-09-22"
    assert judgment["decided_by"] == "user"
    assert "measurement method" in judgment["judgment"]
    assert judgment["claim"] == "this discrepancy is the brightness method's"
    assert judgment["not_claimed"] == "that the camera is verified"
    # The basis the user judged on, named in the record rather than implied.
    basis = " ".join(judgment["basis"])
    assert "(5.7e-14, 0.139) px" in basis
    assert "(29.3, -27.5) px" in basis
    assert "1237" in basis and "185" in basis
    assert "bright_centroid_outside_marker_bbox" in basis
    assert "345.95" in basis and "347.10" in basis


def test_the_user_judgment_records_what_it_does_not_establish(complete_result):
    # The reservations must be IN the record, not only in a source comment.
    FINISH(complete_result)
    judgment = complete_result["marker_diagnosis"]["user_judgment"]
    limits = judgment["does_not_establish"]
    assert len(limits) == 3
    joined = " ".join(limits)
    assert "bloom is not proven" in joined
    assert "0.1 px" in joined and "camera-wide" in joined
    assert "sphere_silhouette_is_not_projected_centre" in joined
    assert "USER_JUDGMENT_REQUIRED" in judgment["gate_effect"]
    assert "g2_allowed" in judgment["gate_effect"]


def test_the_user_judgment_names_the_conclusion_it_was_made_on(complete_result):
    FINISH(complete_result)
    diagnosis = complete_result["marker_diagnosis"]
    assert diagnosis["conclusion"] == "global_brightness_method_discrepancy"
    judgment = diagnosis["user_judgment"]
    assert judgment["applies_to_conclusion"] == "global_brightness_method_discrepancy"
    assert judgment["current_conclusion"] == "global_brightness_method_discrepancy"
    assert judgment["current_conclusion_matches_judgment"] is True


def test_a_numeric_gate_moves_the_conclusion_without_retracting_the_judgment(
    complete_result,
):
    # Gates 1-4 decide the conclusion branch before any marker number is read,
    # so a mismatch records the run's evidence, never a change of the decision.
    complete_result["perception_camera_intrinsics"]["status"] = "FAIL"
    FINISH(complete_result)
    diagnosis = complete_result["marker_diagnosis"]
    assert diagnosis["conclusion"] == "camera_model_or_transform_discrepancy"
    judgment = diagnosis["user_judgment"]
    assert judgment["current_conclusion_matches_judgment"] is False
    assert "does not retract" in judgment["conclusion_match_note"]
    assert complete_result["gates"]["6"] == "USER_JUDGMENT_REQUIRED"
    assert complete_result["g2_allowed"] is False


def test_the_user_judgment_reports_the_basis_this_run_actually_measured(
    complete_result,
):
    complete_result["marker"] = [dict(MARKER_MEASURED) for _ in range(3)]
    FINISH(complete_result)
    observed = complete_result["marker_diagnosis"]["user_judgment"][
        "basis_observed_in_this_run"
    ]
    assert observed["marker_count"] == 3
    assert observed["fields_complete"] is True
    assert observed["all_bright_centroids_outside_marker_bbox"] is True
    assert observed["identified_minus_projected_max_abs_component_px"] == pytest.approx(
        0.1392097168059081
    )
    assert observed["bright_minus_projected_max_abs_component_px"] == pytest.approx(
        29.48544866612781
    )
    assert observed["bright_count"] == [1237, 1237, 1237]
    assert observed["bright_outside_marker_count"] == [185, 185, 185]


def test_the_observed_basis_is_left_incomplete_rather_than_invented(complete_result):
    # A number the run did not record must never be filled in, and an empty
    # marker list must not agree with the judgement through all([]).
    empty = copy.deepcopy(complete_result)
    empty["marker"] = []
    FINISH(complete_result)
    observed = complete_result["marker_diagnosis"]["user_judgment"][
        "basis_observed_in_this_run"
    ]
    assert observed["marker_count"] == 1
    assert observed["fields_complete"] is False
    assert observed["identified_minus_projected_max_abs_component_px"] is None
    assert observed["bright_minus_projected_max_abs_component_px"] is None
    assert observed["all_bright_centroids_outside_marker_bbox"] is True
    FINISH(empty)
    blank = empty["marker_diagnosis"]["user_judgment"]["basis_observed_in_this_run"]
    assert blank["marker_count"] == 0
    assert blank["fields_complete"] is False
    assert blank["all_bright_centroids_outside_marker_bbox"] is False


def test_the_result_names_the_repeat_that_placed_the_panels(complete_result):
    FINISH(complete_result)
    choice = complete_result["selection_fit_choice"]
    assert choice["status"] == "COMPLETE"
    assert choice["panel_placement_repeat_key"] == ANCHOR_KEY
    assert [entry["anchor_horizontal_m"] for entry in choice["per_anchor"]] == [
        0.8,
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
    ]
    assert all(entry["repeat"] == 0 for entry in choice["per_anchor"])
    assert all(entry["panels"] == 9 for entry in choice["per_anchor"])
    # These references predate the rule; the rule is not attributed to them.
    assert all(
        entry["selection_rule"] == "unrecorded" for entry in choice["per_anchor"]
    )
    assert all(entry["repeat_choice"] == "unrecorded" for entry in choice["per_anchor"])
    assert choice["issues"] == []
    assert choice["panels_without_placement_reference"] == 0
    assert "not directly comparable" in choice["comparability"]
    assert "does not prove" in choice["comparability_caveat"]


def test_a_different_chosen_repeat_changes_the_comparability_key(complete_result):
    for panel in complete_result["height_panels"][36:45]:
        panel["panel_placement_K_reference"]["repeat"] = 1
    FINISH(complete_result)
    choice = complete_result["selection_fit_choice"]
    assert choice["panel_placement_repeat_key"] == "0.8=0;1.0=0;2.0=0;3.0=0;4.0=1;5.0=0"
    assert choice["panel_placement_repeat_key"] != ANCHOR_KEY
    assert choice["status"] == "COMPLETE"


def test_a_panel_that_never_reached_placement_is_counted_not_guessed(complete_result):
    complete_result["height_panels"][0] = {"status": "UNOBSERVED", "captures": []}
    FINISH(complete_result)
    choice = complete_result["selection_fit_choice"]
    assert choice["panels_without_placement_reference"] == 1
    assert choice["per_anchor"][0]["panels"] == 8
    assert choice["panel_placement_repeat_key"] == ANCHOR_KEY
    assert choice["status"] == "COMPLETE"
    assert complete_result["gates"]["7a_prime"] == "PASS"


@pytest.mark.parametrize(
    "defect,gates_unchanged",
    [
        ("conflict", True),
        ("no_anchor", True),
        ("no_repeat", True),
        ("not_a_dict", False),
    ],
)
def test_the_key_is_withheld_when_the_placement_records_are_not_one_choice(
    complete_result, defect, gates_unchanged
):
    clean = copy.deepcopy(complete_result)
    FINISH(clean)
    panel = complete_result["height_panels"][0]
    if defect == "conflict":
        panel["panel_placement_K_reference"]["repeat"] = 2
    elif defect == "no_anchor":
        panel["panel_placement_K_reference"].pop("anchor_horizontal_m")
    elif defect == "no_repeat":
        panel["panel_placement_K_reference"].pop("repeat")
    else:
        panel["panel_placement_K_reference"] = "intrinsics_fits[0]"
    FINISH(complete_result)
    choice = complete_result["selection_fit_choice"]
    assert choice["panel_placement_repeat_key"] is None
    assert choice["status"] != "COMPLETE"
    assert choice["issues"]
    # The rollup reports; it never moves a gate by itself.
    if gates_unchanged:
        assert complete_result["gates"] == clean["gates"]
        assert complete_result["gate_7a_prime_detail"] == clean["gate_7a_prime_detail"]
        assert complete_result["numerical_status"] == clean["numerical_status"]
