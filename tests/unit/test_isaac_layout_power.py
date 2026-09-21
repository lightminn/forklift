"""CPU tests for the calibration-layout detection-power diagnostic.

The diagnostic answers one question - does an alternative board layout still
detect a known intrinsics error as well as the judging layout - and emits only
numbers. It must never move the judging measurement or any gate.
"""

import ast
import copy
import importlib.util
import json
import runpy
from pathlib import Path

import numpy as np
import pytest

from forklift_core.sensors.rgbd import PinholeIntrinsics

ROOT = Path(__file__).resolve().parents[2]
K = PinholeIntrinsics(640, 480, 465.741156, 465.741156, 320.0, 240.0, "optical")
MOUNT = np.array([[0, 0, 1, 0.75], [-1, 0, 0, 0], [0, -1, 0, 0.5], [0, 0, 0, 1.0]])


def _load(name: str, relative: str):
    path = ROOT / relative
    assert path.exists(), f"missing module: {relative}"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def power():
    return _load("layout_power_test", "sim/isaac/layout_power.py")


@pytest.fixture
def measurement():
    return _load("layout_power_measure_test", "sim/isaac/camera_calibration.py")


def _corner_uv(measurement, geometry):
    optical = measurement.transform_points(
        np.linalg.inv(MOUNT), geometry["vertices_base_m"][geometry["corner_indices"]]
    )
    return measurement.project(optical, K)


def test_production_arm_is_the_authored_layout_at_one_fixed_pixel_phase(
    power, measurement
):
    # The judging layout is the baseline arm: it must be reproduced exactly.
    # Section 12.3 of the run record says its boards are degenerate - every
    # checker boundary at one single phase - except the r=5.0 central boards,
    # which the authored rule tilts so their endpoint column has a plane.
    tilted = 0
    for distance in measurement.DISTANCES_M:
        for index, centre in enumerate(measurement.SCREEN_CENTRES_UV):
            authored = measurement.checkerboard_layout(distance, centre, K, MOUNT)
            variant = power.variant_layout(
                distance, centre, K, MOUNT, variant="production", position_index=index
            )
            np.testing.assert_array_equal(
                variant["vertices_base_m"], authored["vertices_base_m"]
            )
            np.testing.assert_array_equal(
                variant["corner_indices"], authored["corner_indices"]
            )
            phase = power.board_phase(_corner_uv(measurement, variant))
            if measurement.default_tilt_deg(distance, centre, K):
                tilted += 1
                assert phase["grid_offset_spread_uv_px"][0] > 0.2
                continue
            # One phase for the whole board, and the same phase for every
            # board: nothing in this layout ever samples another phase.
            assert phase["grid_offset_spread_uv_px"] == pytest.approx(
                [0.0, 0.0], abs=1e-9
            )
            assert phase["max_abs_grid_offset_px"] == pytest.approx(0.0, abs=1e-9)
    assert tilted == 3


def test_tilted_arm_sweeps_every_board_off_the_pixel_grid(power, measurement):
    # Tilting is the arm with measured evidence behind it (section 12.3): the
    # boundary walks across pixel phases instead of landing on the grid.
    for distance in measurement.DISTANCES_M:
        for index, centre in enumerate(measurement.SCREEN_CENTRES_UV):
            variant = power.variant_layout(
                distance, centre, K, MOUNT, variant="tilted", position_index=index
            )
            phase = power.board_phase(_corner_uv(measurement, variant))
            assert phase["grid_offset_spread_uv_px"][0] > 0.2
            assert phase["max_abs_grid_offset_px"] > 0.1


def test_phase_arm_moves_whole_boards_by_a_fraction_of_a_pixel(power, measurement):
    # Phase variation has no supporting evidence in the corpus at all, so it is
    # the untested arm: every board is translated by a declared sub-pixel
    # offset, and the board itself stays as rigid as the judging layout.
    offsets = set()
    for index, centre in enumerate(measurement.SCREEN_CENTRES_UV):
        authored = measurement.checkerboard_layout(2.0, centre, K, MOUNT)
        variant = power.variant_layout(
            2.0, centre, K, MOUNT, variant="phase_varied", position_index=index
        )
        shift = _corner_uv(measurement, variant) - _corner_uv(measurement, authored)
        # A pure sub-pixel translation: identical for every corner of a board.
        assert np.ptp(shift, axis=0) == pytest.approx([0.0, 0.0], abs=1e-6)
        phase = power.board_phase(_corner_uv(measurement, variant))
        assert phase["max_abs_grid_offset_px"] > 0.05
        assert phase["grid_offset_spread_uv_px"] == pytest.approx([0.0, 0.0], abs=1e-6)
        offsets.add(tuple(np.round(shift[0], 6)))
        assert 0 < abs(shift[0, 0]) < 1 and 0 < abs(shift[0, 1]) < 1
    assert len(offsets) == len(measurement.SCREEN_CENTRES_UV)


def test_layout_variants_are_declared_with_their_evidence(power):
    # "Untested" must be recorded, not inferred by a later reader.
    assert power.LAYOUT_VARIANTS["production"]["evidence"] == "measured"
    assert power.LAYOUT_VARIANTS["tilted"]["evidence"] == "measured"
    assert power.LAYOUT_VARIANTS["phase_varied"]["evidence"] == "untested"
    assert "no supporting evidence" in power.LAYOUT_VARIANTS["phase_varied"]["note"]


def _observation(
    measurement, *, distance, bias_px, noise_px, seed, variant="production"
):
    """One fit's worth of corners: nine boards, projected with a shifted K."""
    rng = np.random.default_rng(seed)
    holdout = np.array([(i + j) % 4 == 0 for i in range(7) for j in range(9)])
    points, uv, mask = [], [], []
    shifted = PinholeIntrinsics(
        K.width, K.height, K.fx, K.fy, K.cx + bias_px[0], K.cy + bias_px[1], K.frame_id
    )
    for index, centre in enumerate(measurement.SCREEN_CENTRES_UV):
        geometry = power_variant(measurement, distance, centre, index, variant)
        optical = measurement.transform_points(
            np.linalg.inv(MOUNT),
            geometry["vertices_base_m"][geometry["corner_indices"]],
        )
        points.append(optical)
        uv.append(
            measurement.project(optical, shifted)
            + rng.normal(0.0, noise_px, (len(optical), 2))
        )
        mask.append(holdout)
    return {
        "anchor_horizontal_m": distance,
        "repeat": seed,
        "position_count": len(measurement.SCREEN_CENTRES_UV),
        "points_optical": np.concatenate(points),
        "uv": np.concatenate(uv),
        "holdout": np.concatenate(mask),
    }


def power_variant(measurement, distance, centre, index, variant):
    module = _load("layout_power_geometry_test", "sim/isaac/layout_power.py")
    return module.variant_layout(
        distance, centre, K, MOUNT, variant=variant, position_index=index
    )


def test_injected_principal_point_error_is_recovered_and_its_scatter_recorded(power):
    measurement = _load(
        "layout_power_measure_inject", "sim/isaac/camera_calibration.py"
    )
    observations = [
        _observation(
            measurement, distance=d, bias_px=(0.0, -0.06), noise_px=0.05, seed=s
        )
        for d in (2.0, 3.0, 4.0)
        for s in range(3)
    ]
    record = power.arm_power(observations, K, layout="production")
    assert record["fit_count"] == 9
    for entry in record["injections"]:
        for axis in ("cx", "cy"):
            recovery = entry["recovery"][axis]
            # A principal-point injection enters the estimator linearly, so a
            # full-rank layout returns it exactly. This is the wiring control.
            assert recovery["mean_fraction"] == pytest.approx(1.0, abs=1e-9)
            assert recovery["max_abs_fraction_error"] < 1e-9
            spread = entry["post_injection_deviation_from_nominal_px"][axis]
            assert spread["sd"] == pytest.approx(
                record["baseline_deviation_from_nominal_px"][axis]["sd"], abs=1e-12
            )
            assert entry["standardized_effect_size"][axis]["value"] == pytest.approx(
                entry["delta_px"] / spread["sd"]
            )
        # An injection of the principal point must not move the focal length
        # or the held-out residuals; if it does, the estimator changed.
        assert entry["focal_max_abs_relative_shift"] < 1e-12
        assert entry["heldout_p95_max_abs_shift_px"] < 1e-9
    assert record["baseline_deviation_from_nominal_px"]["cy"]["mean"] == pytest.approx(
        -0.06, abs=0.01
    )


def test_an_arms_own_bias_can_mask_an_injected_error(power):
    # Gate 2a measures the distance from nominal, so a -0.06 px arm that is
    # given +0.05 px of real error looks BETTER than before the injection.
    # Recovering the delta exactly and detecting the error are different things.
    measurement = _load("layout_power_measure_mask", "sim/isaac/camera_calibration.py")
    observations = [
        _observation(
            measurement, distance=d, bias_px=(0.0, -0.06), noise_px=0.002, seed=s
        )
        for d in (2.0, 3.0, 4.0)
        for s in range(3)
    ]
    record = power.arm_power(observations, K, layout="production", deltas=(0.05, 0.5))
    masked, unmasked = record["injections"]
    assert masked["masking"]["cy"]["fraction"] == 1.0
    assert (
        masked["post_injection_deviation_from_nominal_px"]["cy"]["max_abs"]
        < record["baseline_deviation_from_nominal_px"]["cy"]["max_abs"]
    )
    assert unmasked["masking"]["cy"]["fraction"] == 0.0
    assert (
        unmasked["post_injection_deviation_from_nominal_px"]["cy"]["max_abs"]
        > record["baseline_deviation_from_nominal_px"]["cy"]["max_abs"]
    )


def test_one_fit_reports_an_unestimable_spread_instead_of_zero(power):
    measurement = _load("layout_power_measure_one", "sim/isaac/camera_calibration.py")
    observations = [
        _observation(
            measurement, distance=3.0, bias_px=(0.0, 0.0), noise_px=0.05, seed=0
        )
    ]
    record = power.arm_power(observations, K, layout="production", deltas=(0.1,))
    spread = record["baseline_deviation_from_nominal_px"]["cx"]
    assert spread["sd"] is None
    assert "fewer than two" in spread["sd_unavailable_reason"]
    effect = record["injections"][0]["standardized_effect_size"]["cx"]
    assert effect["value"] is None
    assert effect["unavailable_reason"] == spread["sd_unavailable_reason"]
    # The OLS standard error is a different uncertainty and stays available.
    assert record["mean_ols_standard_error_px"]["cx"] > 0


def test_injection_never_mutates_the_measured_corners(power):
    measurement = _load("layout_power_measure_copy", "sim/isaac/camera_calibration.py")
    observations = [
        _observation(
            measurement, distance=3.0, bias_px=(0.0, 0.0), noise_px=0.05, seed=s
        )
        for s in range(2)
    ]
    originals = [o["uv"].copy() for o in observations]
    power.arm_power(observations, K, layout="production")
    for observation, original in zip(observations, originals, strict=True):
        np.testing.assert_array_equal(observation["uv"], original)


def test_comparison_records_numbers_for_every_arm_and_judges_nothing(power):
    measurement = _load("layout_power_measure_cmp", "sim/isaac/camera_calibration.py")
    arms = {
        "production": {
            "observations": [
                _observation(
                    measurement, distance=d, bias_px=(0.0, -0.06), noise_px=0.05, seed=s
                )
                for d in (2.0, 3.0)
                for s in range(2)
            ],
            "provenance": {"source": "judging run"},
        },
        "tilted": {
            "observations": [
                _observation(
                    measurement,
                    distance=d,
                    bias_px=(0.0, 0.01),
                    noise_px=0.005,
                    seed=s,
                    variant="tilted",
                )
                for d in (2.0, 3.0)
                for s in range(2)
            ],
            "provenance": {"source": "diagnostic capture"},
        },
    }
    record = power.compare_layout_power(arms, K)
    assert set(record["arms"]) == {"production", "tilted"}
    assert record["arms"]["tilted"]["evidence"] == "measured"
    assert record["reference_layout"] == "production"
    assert [row["delta_px"] for row in record["by_delta"]] == [
        float(d) for d in power.INJECTION_LADDER_PX
    ]
    row = record["by_delta"][0]["cy"]
    assert set(row["standardized_effect_size"]) == {"production", "tilted"}
    assert row["relative_to_reference"]["production"] == pytest.approx(1.0)
    # A quieter arm separates the same injection further from its own spread.
    assert row["relative_to_reference"]["tilted"] > 1.0
    # No verdict anywhere, and the record must survive strict JSON encoding.
    # Prose may say the words; no field may ever CARRY one as its value.
    assert not _verdict_values(json.loads(json.dumps(record, allow_nan=False)))
    assert "No PASS/FAIL" in record["judgment"]


def _verdict_values(node, path="") -> list:
    if isinstance(node, dict):
        return [v for k, n in node.items() for v in _verdict_values(n, f"{path}.{k}")]
    if isinstance(node, list):
        return [
            v for i, n in enumerate(node) for v in _verdict_values(n, f"{path}[{i}]")
        ]
    return [path] if node in ("PASS", "FAIL", "REVIEW_REQUIRED") else []


VERIFY_SOURCE = ROOT / "sim/isaac/verify_perception_camera.py"


@pytest.fixture
def verify():
    return runpy.run_path(str(VERIFY_SOURCE))


def _fit_reference(index, role):
    # The provenance a measured panel carries in a real run: one fitted K,
    # recorded once per role, with the gate that validated it.
    return {
        "collection": "intrinsics_fits",
        "index": index,
        "anchor_horizontal_m": (0.8, 1.0, 2.0, 3.0, 4.0, 5.0)[index],
        "repeat": 0,
        "repeat_kind": "measured",
        "warmup_capture_excluded": True,
        "gate_2a": "PASS",
        "role": role,
    }


def _complete_result():
    """A fully passing G1 result at the judging counts: 162 / 18 / 54."""
    return {
        "perception_camera_intrinsics": {"status": "PASS"},
        "mount": {"status": "PASS"},
        "intrinsics_fits": [{"gate_2a": "PASS", "gate_2b": "PASS"} for _ in range(18)],
        "boards": [{"gate_4": "PASS", "status": "PASS"} for _ in range(162)],
        "height_panels": [
            {
                "status": "PASS",
                "panel_placement_K_reference": _fit_reference(
                    panel // 9, "height_panel_physical_placement"
                ),
                "selection_fit_reference": _fit_reference(
                    panel // 9, "height_grid_sample_selection"
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


def test_the_diagnostic_leaves_the_judged_collections_and_gates_untouched(verify):
    measurement = _load("layout_power_measure_gate", "sim/isaac/camera_calibration.py")
    observations = [
        _observation(
            measurement, distance=d, bias_px=(0.0, -0.06), noise_px=0.05, seed=s
        )
        for d in (2.0, 3.0)
        for s in range(2)
    ]
    without = _complete_result()
    verify["finish_result"](without)

    result = _complete_result()
    judged = copy.deepcopy(
        {key: result[key] for key in ("boards", "intrinsics_fits", "height_panels")}
    )
    result[verify["LAYOUT_POWER_KEY"]] = verify["layout_power_diagnostic"](
        observations, K, provenance={"source": "this run"}
    )
    verify["finish_result"](result)

    # Same counts, same records, same gates as a run without the diagnostic.
    assert len(result["boards"]) == 162
    assert len(result["intrinsics_fits"]) == 18
    assert len(result["height_panels"]) == 54
    for key, before in judged.items():
        assert result[key] == before
    assert result["gates"] == without["gates"]
    assert result["numerical_status"] == without["numerical_status"]
    assert result["status"] == without["status"]
    assert result["g2_allowed"] is False
    assert result["gate_7a_prime_detail"] == without["gate_7a_prime_detail"]
    # The diagnostic is still there, under its own key, with its numbers.
    diagnostic = result[verify["LAYOUT_POWER_KEY"]]
    assert set(diagnostic["arms"]) == {"production"}
    assert diagnostic["arms"]["production"]["fit_count"] == len(observations)
    assert not _verdict_values(diagnostic)


def test_a_broken_diagnostic_is_recorded_and_never_costs_the_run_its_gates(verify):
    # The gate table is the evidence the run exists to produce. A diagnostic
    # must never be able to destroy it by raising.
    record = verify["layout_power_diagnostic"](
        [
            {
                "points_optical": np.zeros((2, 3)),
                "uv": np.zeros((2, 2)),
                "holdout": [True],
            }
        ],
        K,
        provenance={"source": "broken"},
    )
    assert "error" in record
    assert record["arms"] == {}
    result = _complete_result()
    result[verify["LAYOUT_POWER_KEY"]] = record
    verify["finish_result"](result)
    assert result["gates"]["2a"] == "PASS"
    assert result["numerical_status"] == "PASS"

    # A degraded record keeps the shape of a healthy one, so a reader that
    # indexes by_delta or caveats does not break on it. Empty and None mean
    # "not computed"; no number is invented to fill the hole.
    measurement = _load(
        "layout_power_measure_broken", "sim/isaac/camera_calibration.py"
    )
    healthy = verify["layout_power_diagnostic"](
        [
            _observation(
                measurement, distance=d, bias_px=(0.0, 0.0), noise_px=0.01, seed=s
            )
            for d in (2.0, 3.0)
            for s in range(2)
        ],
        K,
        provenance={"source": "this run"},
    )
    assert "error" not in healthy
    assert set(record) == set(healthy) | {"provenance", "error"}
    assert record["by_delta"] == []
    assert record["caveats"] == []
    assert record["injection"] is None
    assert record["nominal_intrinsics"] is None
    assert record["reference_layout"] is None
    json.dumps(record, allow_nan=False)


def test_a_zero_anchor_is_recorded_like_any_other_anchor(power, measurement):
    # 0.0 m is a real anchor value, not an absence. The neighbouring repeat
    # filter already distinguishes the two; this one must as well.
    observations = [
        _observation(
            measurement, distance=3.0, bias_px=(0.0, 0.0), noise_px=0.01, seed=s
        )
        for s in range(2)
    ]
    for observation in observations:
        observation["anchor_horizontal_m"] = 0.0
    record = power.arm_power(observations, K, layout="production", deltas=(0.1,))
    assert record["fit_structure"]["anchors_horizontal_m"] == [0.0]
    # repeat 0 is equally falsy and is already kept; the two must agree.
    assert record["fit_structure"]["repeats"] == [0, 1]


def test_the_reference_arm_is_indexed_without_an_unreachable_default():
    # `if reference in records` already guarantees the arm and its injections
    # exist, so a default for them can never be taken.
    tree = ast.parse((ROOT / "sim/isaac/layout_power.py").read_text(encoding="utf-8"))
    dead = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "get"
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == "injections"
        and len(node.args) > 1
    ]
    assert not dead


def test_the_judging_run_takes_no_extra_captures_for_the_diagnostic(verify):
    # The baseline arm reuses corners the run already measured. Rendering any
    # other layout is a separate, opt-in script - never part of this run.
    tree = ast.parse(VERIFY_SOURCE.read_text(encoding="utf-8"))
    capture_loops = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.For)
        and isinstance(node.iter, ast.Call)
        and getattr(node.iter.func, "id", None) == "capture_plan"
    ]
    assert len(capture_loops) == 2, "the diagnostic must not add a capture loop"
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", None) == "layout_power_diagnostic"
    ]
    assert len(calls) == 1
    source = VERIFY_SOURCE.read_text(encoding="utf-8")
    assert "diagnose_layout_power" not in source
    assert "variant_layout" not in source


@pytest.fixture
def capture_script():
    return _load("layout_power_capture_test", "sim/isaac/diagnose_layout_power.py")


def test_extra_layout_captures_are_opt_in_and_plan_only_what_was_asked_for(
    capture_script, measurement
):
    # Nothing renders without an explicit invocation of this separate script.
    with pytest.raises(SystemExit):
        capture_script.arguments([])
    args = capture_script.arguments(
        ["--base-scene", "scene.usda", "--output", "out", "--layouts", "tilted"]
    )
    plan = capture_script.layout_plan(args.layouts, args.distances, args.repeats)
    assert {entry["layout"] for entry in plan} == {"tilted"}
    assert len(plan) == len(args.distances) * len(measurement.SCREEN_CENTRES_UV)
    for entry in plan:
        # Same warm-up policy as the judging run: one discarded capture after
        # authoring, then the measured repeats.
        assert [c["measured"] for c in entry["captures"]] == [False] + [True] * 3
    with pytest.raises(ValueError):
        capture_script.layout_plan(["no_such_layout"], (3.0,), 3)


def test_captured_arms_round_trip_into_the_cpu_analysis(
    capture_script, power, measurement, tmp_path
):
    # The capture writes exactly the corner files the analysis reads back, so
    # the power comparison never needs Isaac.
    observations = [
        _observation(
            measurement, distance=3.0, bias_px=(0.0, 0.0), noise_px=0.01, seed=s
        )
        for s in range(3)
    ]
    corners = len(observations[0]["uv"]) // len(measurement.SCREEN_CENTRES_UV)
    for entry in capture_script.layout_plan(["tilted"], (3.0,), 3):
        for capture in entry["captures"]:
            repeat = capture["repeat"] or 0
            start = entry["position_index"] * corners
            np.savez_compressed(
                tmp_path / f"{capture['name']}_samples.npz",
                truth_optical_m=observations[repeat]["points_optical"][
                    start : start + corners
                ],
                uv=observations[repeat]["uv"][start : start + corners],
                holdout=observations[repeat]["holdout"][start : start + corners],
            )
    arm = power.observations_from_samples(tmp_path)
    assert arm["provenance"]["skipped"]["warmup"] == len(measurement.SCREEN_CENTRES_UV)
    assert len(arm["observations"]) == 3
    for observation in arm["observations"]:
        assert observation["position_count"] == len(measurement.SCREEN_CENTRES_UV)
        assert observation["anchor_horizontal_m"] == 3.0
    record = power.compare_layout_power({"tilted": arm}, K, reference="tilted")
    assert record["arms"]["tilted"]["fit_count"] == 3
    assert record["arms"]["tilted"]["provenance"]["skipped"]["warmup"] == 9
