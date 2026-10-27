"""Plan D8 S4a: the runner side of near-field tracking (no Isaac)."""

import importlib.util
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "near_tracking", ROOT / "sim/isaac/near_tracking.py"
)
NT = importlib.util.module_from_spec(SPEC)
sys.modules["near_tracking"] = NT
SPEC.loader.exec_module(NT)

TICK = NT.TICK_S


def test_depth_is_rounded_to_one_millimetre_and_invalid_pixels_stay_nan():
    out = NT.quantize_depth_mm(np.array([1.23449, 1.2345001, np.nan, np.inf]))
    assert out[0] == pytest.approx(1.234) and out[1] == pytest.approx(1.235)
    assert np.isnan(out[2]) and np.isnan(out[3])


def straight_history(n=30, v=0.06, yaw=0.0, episode=1, offset=0.34):
    h = NT.RearHistory(offset)
    for i in range(n):
        s = v * i * TICK
        h.add(i * TICK, (s * math.cos(yaw), s * math.sin(yaw), yaw), episode)
    return h


def test_the_pose_is_the_interpolated_rear_axle_moved_to_base_link():
    h = straight_history(yaw=0.3)
    t = 10.5 * TICK
    s = 0.06 * t
    base = h.base_at(t, 1)
    assert base == pytest.approx(
        (
            s * math.cos(0.3) + 0.34 * math.cos(0.3),
            s * math.sin(0.3) + 0.34 * math.sin(0.3),
            0.3,
        )
    )


def test_yaw_is_interpolated_across_the_wrap():
    h = NT.RearHistory(0.0)
    h.add(0.0, (0.0, 0.0, math.pi - 0.01), 1)
    h.add(TICK, (0.0, 0.0, -math.pi + 0.01), 1)
    assert abs(abs(h.rear_at(TICK / 2, 1)[2]) - math.pi) < 1e-9


def test_a_gap_another_episode_or_a_time_outside_the_history_is_not_covered():
    h = straight_history()
    assert h.base_at(-TICK, 1) is None and h.base_at(40 * TICK, 1) is None
    assert h.base_at(5 * TICK, 2) is None  # another hold episode
    gap = NT.RearHistory(0.34)
    gap.add(0.0, (0, 0, 0), 1)
    gap.add(2 * TICK, (0.001, 0, 0), 1)  # a missing row
    assert gap.base_at(TICK, 1) is None
    assert gap.path_length_since(0.0, 1) is None
    h.clear()
    assert h.base_at(5 * TICK, 1) is None


def test_the_history_keeps_one_second_and_replaces_a_repeated_instant():
    h = straight_history(n=200)
    assert h.rows[0][0] >= 199 * TICK - 1.0 - 1e-9
    h.add(199 * TICK, (9.0, 0.0, 0.0), 1)
    assert (
        h.rows[-1][1][0] == 9.0 and len([r for r in h.rows if r[0] == 199 * TICK]) == 1
    )
    with pytest.raises(ValueError, match="time order"):
        h.add(100 * TICK, (0, 0, 0), 1)


def test_the_path_length_runs_from_an_interpolated_start_to_the_latest_row():
    h = straight_history(n=30, v=0.06)
    assert h.path_length_since(10.5 * TICK, 1) == pytest.approx(
        0.06 * (29 - 10.5) * TICK
    )
    assert h.path_length_since(29 * TICK, 1) == pytest.approx(0.0)


def test_the_wait_moves_on_past_a_frame_that_did_not_come():
    # A result is due at stamp + k * 0.1 + A; just after one arrived, the next is 0.1 away.
    assert NT.result_wait_s(1.0 + TICK, 1.0, 0.1, TICK) == pytest.approx(0.1)
    # Before the first frame is due.
    assert NT.result_wait_s(1.05, 1.0, 0.1, TICK) == pytest.approx(0.05 + TICK)
    # At the due tick with nothing delivered, the next frame is the one that can come.
    assert NT.result_wait_s(1.1 + TICK, 1.0, 0.1, TICK) == pytest.approx(0.1)
    # Two missed frames.
    assert NT.result_wait_s(1.25, 1.0, 0.1, TICK) == pytest.approx(0.05 + TICK)


LATENCY = {"max_s": 0.09893963240057052, "read_delay_s": TICK, "render_period_s": 0.1}


def terms_data(**kw):
    data = {
        "status": "confirmed",
        "cruise_mps": 0.05,
        "cruise_cap_mps": 0.08,
        "delta_len_m": [0.0] * 200,
        "delta_s_m": [0.0] * 200,
        "delta_v_mps": 0.0,
        "tick_s": TICK,
        "align_age_s": LATENCY["max_s"] + LATENCY["read_delay_s"],
        "render_period_s": 0.1,
        "condition": {"arguments": {}},
        "provenance": {"bounds": {"sha256": "a" * 64}},
    }
    data.update(kw)
    return data


def terms(**kw):
    return NT.DriveTerms.from_dict(terms_data(**kw), "a" * 64, LATENCY)


def test_drive_terms_come_only_from_a_confirmed_measurement_of_these_bounds():
    with pytest.raises(ValueError, match="confirmed"):
        NT.DriveTerms.from_dict(
            {"status": "diagnostic", "diagnostic_cruise_mps": 0.05}, "a" * 64, LATENCY
        )
    with pytest.raises(ValueError, match="running maxima"):
        terms(delta_len_m=[0.0, 0.002, 0.001])
    with pytest.raises(ValueError, match="delta_v"):
        terms(delta_v_mps=-0.001)
    # Codex S4a-1 review P2: tied to the bounds file, its latency and a finite cruise.
    with pytest.raises(ValueError, match="another bounds file"):
        NT.DriveTerms.from_dict(terms_data(), "b" * 64, LATENCY)
    # Codex 2nd review: missing or NaN must not pass the latency binding either.
    for key in ("align_age_s", "render_period_s"):
        for bad in (0.2, float("nan"), None, True):
            with pytest.raises(ValueError, match=key):
                terms(**{key: bad})
    for bad in (float("nan"), 0.0, 0.09, True):
        with pytest.raises(ValueError, match="cruise"):
            terms(cruise_mps=bad)
    with pytest.raises(ValueError, match="condition"):
        terms(condition=None)
    assert terms().cruise_mps == 0.05


def test_the_condition_names_every_argument_setting_and_input_that_differs(tmp_path):
    (tmp_path / "config").mkdir()
    (tmp_path / "config/prior.yaml").write_text("a: 1\n")
    (tmp_path / "config/layer.yaml").write_text("odometry_age: config/age.json\n")
    (tmp_path / "config/age.json").write_text("{}\n")
    result = {
        "arguments": {
            "pallet_prior": "config/prior.yaml",
            "obstacle_layer": "config/layer.yaml",
            "slam_noise_spec": None,
            "tracker_profile": "current",
            "seed": 3,
            "output": "/x",
        },
        "settings_synthetic": {"wheel_torque_nm": 3.0},
        "detector_params": {"cell_m": 0.01},
        "planner_config": {"p": 1},
        "travel_planner_config": {"p": 2},
        "lidar_synthetic": {"beam_count": 1600},
        "chassis_model": {
            "forklift_urdf": "f.urdf",
            "drive_geometry": {"wheelbase_m": 0.66},
        },
        "tracker_configs": {
            "approach": {"cruise_speed_mps": 0.055, "lookahead_m": 0.28},
            "insert": {"x": 1},
        },
    }
    expected = NT.run_condition(result, tmp_path)
    # The approach cruise is the speed under test; every other follower field is shared.
    assert expected["tracker_configs"]["approach"] == {"lookahead_m": 0.28}
    faster = {
        **result["tracker_configs"],
        "approach": {"cruise_speed_mps": 0.08, "lookahead_m": 0.28},
    }
    assert NT.tracker_config_problems(result["tracker_configs"], faster) == []
    longer = {
        **result["tracker_configs"],
        "approach": {"cruise_speed_mps": 0.055, "lookahead_m": 0.5},
    }
    assert NT.tracker_config_problems(result["tracker_configs"], longer) == [
        "tracker_configs approach differs from the measurement runs"
    ]
    assert NT.tracker_config_problems(None, longer) == ["tracker_configs missing"]
    # A follower field added since (fix A's hold) reads as its earlier value None on a record
    # without it; the observe tracker's 0.08 still differs (Codex fix A implementation review 1).
    measured = json.loads((ROOT / "config/near_field_drive_terms.json").read_text())[
        "condition"
    ]["tracker_configs"]
    # A record from before the field (the committed terms carry it since the S3 re-measure).
    recorded = {
        phase: {k: v for k, v in config.items() if k != "cusp_heading_pending_rad"}
        for phase, config in measured.items()
    }
    now = {
        phase: {**config, "cusp_heading_pending_rad": None}
        for phase, config in recorded.items()
    }
    assert NT.tracker_config_problems(recorded, now) == []
    assert NT.tracker_config_problems(now, recorded) == []
    now["observe"]["cusp_heading_pending_rad"] = 0.08
    assert NT.tracker_config_problems(recorded, now) == [
        "tracker_configs observe differs from the measurement runs"
    ]
    # The same file written as an absolute path is the same input (compared by content).
    absolute = {
        **result,
        "arguments": {
            **result["arguments"],
            "pallet_prior": str(tmp_path / "config/prior.yaml"),
        },
    }
    assert NT.condition_problems(expected, NT.run_condition(absolute, tmp_path)) == []
    moved = {
        **result,
        "chassis_model": {**result["chassis_model"], "forklift_urdf": "/abs/f.urdf"},
    }
    assert NT.condition_problems(expected, NT.run_condition(moved, tmp_path)) == []
    assert "seed" not in expected["arguments"] and "output" not in expected["arguments"]
    assert expected["input_files"]["file config/age.json"] is not None
    assert NT.condition_problems(expected, NT.run_condition(result, tmp_path)) == []
    # M-a0: a recording-only argument matches at any value (record on/off runs compare equal).
    recorded_on = {**result, "arguments": {**result["arguments"], "record_boundary": True}}
    assert NT.condition_problems(expected, NT.run_condition(recorded_on, tmp_path)) == []
    # M-a1 replay plan R3: the record's value format is recording-only too.
    format2 = {**result, "arguments": {**result["arguments"], "record_boundary": True, "record_boundary_format": 2}}
    assert NT.condition_problems(expected, NT.run_condition(format2, tmp_path)) == []
    assert NT.RECORDING_ONLY_ARGUMENTS == frozenset({"record_boundary", "record_boundary_format"})
    changed = {
        **result,
        "arguments": {**result["arguments"], "slam_noise_spec": "3sigma", "seed": 5},
        "settings_synthetic": {"wheel_torque_nm": 0.01},
    }
    (tmp_path / "config/prior.yaml").write_text("a: 2\n")
    problems = NT.condition_problems(expected, NT.run_condition(changed, tmp_path))
    for text in (
        "arguments slam_noise_spec",
        "settings_synthetic",
        "input_files argument pallet_prior",
    ):
        assert any(text in p for p in problems), (text, problems)
    assert not any("seed" in p for p in problems)
    # A field the measurement did not record cannot be checked: refused.
    assert any(
        "lidar_synthetic" in p
        for p in NT.condition_problems({**expected, "lidar_synthetic": None}, expected)
    )
    (tmp_path / "config/age.json").unlink()
    assert any(
        "input_files file config/age.json" in p
        for p in NT.condition_problems(expected, NT.run_condition(result, tmp_path))
    )


def test_a_far_run_is_refused_by_the_committed_terms_and_taken_by_terms_measured_under_it():
    """Near tracking re-measure (docs/plans/2026-10-27-insertion-lateral-drift.md 5.3): the parser no longer refuses
    --recognition far with --near-tracking; the drive terms' condition does. The committed terms were measured
    without the far recognition, so a far run differs from them; terms measured under it match the same run."""
    committed = json.loads((ROOT / "config/near_field_drive_terms.json").read_text())["condition"]
    assert "recognition" not in committed["arguments"]  # recorded before the argument: read as "off"
    far = {**committed, "arguments": {**committed["arguments"], "recognition": "far",
                                      "recognition_proposals": "learned"}}
    assert NT.condition_problems(committed, far) == [
        "arguments recognition differs from the measurement runs",
        "arguments recognition_proposals differs from the measurement runs",
    ]
    assert NT.condition_problems(committed, committed) == []
    assert NT.condition_problems(far, far) == []
    # the near re-draw's lateral box the riib launcher passes by default (0.10) is another condition again
    wide = {**far, "arguments": {**far["arguments"], "near_capture_redraw_lateral_m": 0.10}}
    assert NT.condition_problems(far, wide) == [
        "arguments near_capture_redraw_lateral_m differs from the measurement runs"
    ]


def test_a_far_runs_condition_compares_the_files_the_far_recognition_reads_by_fixed_path(tmp_path):
    """Codex near-remeasure tools review P2: the far runtime reads its track bounds, the site load defaults and the
    known shapes' geometry without an argument naming them; a far condition holds them (and what they name) by
    content, a run without the far recognition does not."""
    import shutil

    for rel in NT.FAR_RECOGNITION_FILES + ("config/obstacle_odometry_age.json",):
        (tmp_path / rel).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(ROOT / rel, tmp_path / rel)
    far = {"arguments": {"recognition": "far"}}
    expected = NT.run_condition(far, tmp_path)
    labels = set(expected["input_files"])
    assert {f"far-recognition {rel}" for rel in NT.FAR_RECOGNITION_FILES} <= labels
    assert "file config/obstacle_odometry_age.json" in labels  # named by the track bounds
    assert not any(label.startswith("far-recognition") for label in NT.run_condition({"arguments": {}}, tmp_path)["input_files"])
    def file_problems(observed):  # (the fake result records no settings: only the inputs are compared here)
        return [p for p in NT.condition_problems(expected, observed) if p.startswith("input_files")]

    assert file_problems(NT.run_condition(far, tmp_path)) == []
    bounds = tmp_path / "config/recognition_track_bounds.yaml"
    bounds.write_text(bounds.read_text() + "\n# changed\n")
    assert file_problems(NT.run_condition(far, tmp_path)) == [
        "input_files far-recognition config/recognition_track_bounds.yaml differs from the measurement runs"
    ]
    assert any("far-recognition" in p for p in file_problems(NT.run_condition(far, None)))
    # every fixed path the runtime reads is listed (a new one there must join FAR_RECOGNITION_FILES)
    source = (ROOT / "sim/isaac/far_recognition_runtime.py").read_text()
    from forklift_core.perception.pallet_model import VARIANT_OF

    sys.path.insert(0, str(ROOT / "sim/isaac"))
    import far_recognition_runtime as FR

    shapes = set(FR.SHAPES) | set(VARIANT_OF)
    assert {f"config/pallet_geometry_{k}.yaml" for k in shapes} <= set(NT.FAR_RECOGNITION_FILES)
    for rel in re.findall(r'"(config/[A-Za-z0-9_./-]+)"', source):
        assert rel in NT.FAR_RECOGNITION_FILES, rel


def test_an_argument_newer_than_the_measurement_matches_only_at_its_reproducing_value(
    tmp_path,
):
    """2026-10-13: --return-turnaround came after the measurement runs; off, a run is the
    measured one, on (or any other argument they lack) it is a difference."""

    def argument_problems(arguments):
        observed = NT.run_condition({"arguments": arguments}, tmp_path)
        return [
            p
            for p in NT.condition_problems(expected, observed)
            if p.startswith("arguments")
        ]

    expected = NT.run_condition({"arguments": {"fps": 60}}, tmp_path)
    assert argument_problems({"fps": 60, "return_turnaround": False}) == []
    assert argument_problems({"fps": 60, "return_turnaround": True}) == [
        "arguments return_turnaround differs from the measurement runs"
    ]
    assert argument_problems({"fps": 60, "new_flag": False}) == [
        "arguments new_flag differs from the measurement runs"
    ]
    assert argument_problems({"fps": 30, "return_turnaround": False}) == [
        "arguments fps differs from the measurement runs"
    ]
    # A measurement that records the argument is matched by its own value only.
    differs = ["arguments return_turnaround differs from the measurement runs"]
    for measured in (None, False, True):
        expected = NT.run_condition(
            {
                "arguments": {"fps": 60}
                if measured is None
                else {"fps": 60, "return_turnaround": measured}
            },
            tmp_path,
        )
        for run in (None, False, True):
            arguments = (
                {"fps": 60} if run is None else {"fps": 60, "return_turnaround": run}
            )
            same = run is measured or (measured is None and run is False)
            assert argument_problems(arguments) == ([] if same else differs), (
                measured,
                run,
            )


def test_the_budget_adds_the_measured_terms_and_stops_past_them():
    t = terms(
        delta_len_m=[0.001 * k for k in range(60)],
        delta_s_m=[0.0005 * k for k in range(60)],
        delta_v_mps=0.005,
    )
    b = NT.travel_budget(0.006, 0.107, 0.1, 0.06, t)
    horizon = 0.1 + NT.STOP_LATENCY_S
    d = 0.006 + t.at(t.delta_len_m, 0.107)
    travel = 0.06 * horizon + t.at(t.delta_s_m, horizon) + 0.06**2 / 3
    assert b.d_m == pytest.approx(d) and b.travel_m == pytest.approx(travel)
    assert b.ok == (d + travel <= 0.03)
    assert NT.travel_budget(None, 0.1, 0.1, 0.05, t).reason == "uncovered"
    assert (
        NT.travel_budget(0.0, 1.0, 0.1, 0.05, t).reason == "unmeasured"
    )  # age past the table


def test_vbar_is_the_largest_recent_or_pending_command_plus_delta_v():
    w = NT.CommandWindow()
    w.add(0.5, 0.055)  # the interval ending at 0.5 s
    w.add(0.6, 0.0)
    assert w.vbar(0.6, 0.004) == pytest.approx(0.059)
    assert w.vbar(0.8 - 1e-6, 0.004) == pytest.approx(0.059)
    # At 0.8 s the interval that ended at 0.5 s is outside the 0.3 s window.
    assert w.vbar(0.8, 0.004) == pytest.approx(0.004)
    assert w.vbar(0.8, 0.004, pending_mps=-0.03) == pytest.approx(0.034)


def test_the_face_points_are_the_pocket_centres_across_the_axis_seen_from_the_base_now():
    # Pockets 0.3 m either side at x = 3 in the held frame, axis along +x, widths 0.2.
    held = {"left": (3.0, 0.3, 0.1), "right": (3.0, -0.3, 0.1), "yaw": 0.0}
    assert NT.face_min_x_m(held, (0.2, 0.2), (1.0, 0.0, 0.0)) == pytest.approx(2.0)
    # The pallet turned by 0.1 rad: the wall points spread along x by +-0.1 sin 0.1.
    held["yaw"] = 0.1
    assert NT.face_min_x_m(held, (0.2, 0.2), (1.0, 0.0, 0.0)) == pytest.approx(
        2.0 - 0.1 * math.sin(0.1)
    )
    # The base turned by 90 degrees and moved: x is along its heading.
    held["yaw"] = math.pi / 2
    assert NT.face_min_x_m(held, (0.2, 0.2), (3.0, -2.0, math.pi / 2)) == pytest.approx(
        1.7
    )


def test_the_section_starts_one_and_a_half_metres_from_the_fork_tips():
    assert NT.section_entered(2.0, 0.5) and not NT.section_entered(2.0001, 0.5)


def test_the_contract_names_every_difference_from_the_bounds_condition():
    condition = {
        "video": False,
        "fps": 60,
        "mount": {"q": 1},
        "camera": {"fx": 1},
        "pallet_geometry": "p",
        "forklift_urdf": "u",
        "rear_axle_offset_m": -0.34,
        "source_sha256": {
            "pallet_geometry_sha256": "a",
            "forklift_urdf_sha256": "b",
            "pallet_urdf_sha256": "c",
        },
    }
    observed = {k: condition[k] for k in NT.CONDITION_KEYS}
    observed.update(
        pallet_geometry_sha256="a",
        forklift_urdf_sha256="b",
        pallet_urdf_sha256="c",
        slam_feedback=True,
        seed=3,
        slam_noise_seed=3,
    )
    assert NT.contract_problems(condition, observed) == []
    # The pallet geometry and URDF by content: another spelling of the same file passes.
    respelled = {**observed, "pallet_geometry": "/abs/p", "forklift_urdf": "/abs/u"}
    assert NT.contract_problems(condition, respelled) == []
    observed.update(fps=30, pallet_urdf_sha256=None, slam_noise_seed=None)
    problems = NT.contract_problems(condition, observed)
    assert any("fps" in p for p in problems) and any(
        "pallet_urdf" in p for p in problems
    )
    assert any("noise seed" in p for p in problems)


# Plan D8 S4a-2 -------------------------------------------------------------------------
BLADES = (
    (0.59, 0.95, 0.1175, 0.1725),
    (0.59, 0.95, -0.1725, -0.1175),
)  # synthetic: the 0.29 m fork spacing as recorded then, not the current model
CORNERS = ((0.604, 0.235), (0.604, -0.235))
WIDTHS = (0.2275, 0.2275)


def held_pallet(face_x, lateral=0.0, yaw=0.0):
    """EPAL 6 pockets (centres +-0.18625 m) with the face at face_x in a held frame equal
    to the base at the origin, shifted sideways and turned about the face midpoint."""
    c, s = math.cos(yaw), math.sin(yaw)
    mid = (face_x, lateral)
    return {
        "left": (mid[0] - s * 0.18625, mid[1] + c * 0.18625, 0.061),
        "right": (mid[0] + s * 0.18625, mid[1] - c * 0.18625, 0.061),
        "yaw": yaw,
    }


def test_the_tips_enter_when_they_pass_the_nearest_face_wall_point():
    assert not NT.entered(NT.carried_face(held_pallet(0.96), WIDTHS, (0, 0, 0)), 0.95)
    assert NT.entered(NT.carried_face(held_pallet(0.95), WIDTHS, (0, 0, 0)), 0.95)
    # Turned, one wall point comes nearer first.
    turned = NT.carried_face(held_pallet(0.96, yaw=0.1), WIDTHS, (0, 0, 0))
    assert NT.entered(turned, 0.95)


def test_centred_blades_leave_45_mm_inside_and_127_5_mm_outside():
    gaps = NT.blade_gaps(
        NT.carried_face(held_pallet(0.95), WIDTHS, (0, 0, 0)),
        BLADES,
        (0.0, 0.33),
        -0.34,
    )
    by = {(p["side"], p["wall"], p["depth_m"]): p["gap_m"] for p in gaps["points"]}
    for side in ("left", "right"):
        for depth in (0.0, 0.33):
            assert by[(side, "inner", depth)] == pytest.approx(0.045)
            assert by[(side, "outer", depth)] == pytest.approx(0.1275)
    assert gaps["rho_m"] == pytest.approx(math.hypot(0.95 + 0.33 + 0.34, 0.30))


def test_a_sideways_shift_closes_one_side_of_each_blade_and_a_yaw_closes_with_depth():
    shifted = NT.blade_gaps(
        NT.carried_face(held_pallet(0.95, lateral=0.02), WIDTHS, (0, 0, 0)),
        BLADES,
        (0.0,),
        -0.34,
    )
    by = {(p["side"], p["wall"]): p["gap_m"] for p in shifted["points"]}
    assert by[("left", "inner")] == pytest.approx(0.025) and by[
        ("right", "inner")
    ] == pytest.approx(0.065)
    assert by[("left", "outer")] == pytest.approx(0.1475) and by[
        ("right", "outer")
    ] == pytest.approx(0.1075)
    turned = NT.blade_gaps(
        NT.carried_face(held_pallet(0.95, yaw=0.05), WIDTHS, (0, 0, 0)),
        BLADES,
        (0.0, 0.33),
        -0.34,
    )
    left_inner = {
        p["depth_m"]: p["gap_m"]
        for p in turned["points"]
        if (p["side"], p["wall"]) == ("left", "inner")
    }
    assert (
        left_inner[0.33] < left_inner[0.0]
    )  # the walls lean +y with depth: the left inner gap closes
    assert left_inner[0.33] - left_inner[0.0] == pytest.approx(
        -0.33 * math.sin(0.05), abs=1e-9
    )


def test_the_corner_gap_is_the_axial_distance_to_the_face_line():
    face = NT.carried_face(held_pallet(0.95), WIDTHS, (0, 0, 0))
    assert NT.carriage_gaps(face, CORNERS) == pytest.approx([0.346, 0.346])
    turned = NT.carried_face(held_pallet(0.95, yaw=0.02), WIDTHS, (0, 0, 0))
    plus, minus = NT.carriage_gaps(turned, CORNERS)
    # A turned face is nearer the +y corner: (mid - c) . a with a = (cos, sin).
    assert plus == pytest.approx(0.346 * math.cos(0.02) - 0.235 * math.sin(0.02))
    assert minus == pytest.approx(0.346 * math.cos(0.02) + 0.235 * math.sin(0.02))


def test_the_stopping_model_and_the_corner_uncertainty_follow_the_plan():
    assert NT.stop_distance_m(0.055) == pytest.approx(
        0.055 * (0.15 + TICK) + 0.055**2 / 3
    )
    face = NT.carried_face(held_pallet(0.95, yaw=0.02), WIDTHS, (0, 0, 0))
    bound = {"along_m": 0.004, "yaw_rad": 0.00026}
    sigma = NT.carriage_uncertainty_m(
        bound, (0.00616, 0.00495), CORNERS[0], face, -0.34, 0.055
    )
    expected = (
        0.004
        + abs(0.235 - face["mid"][1]) * 0.00026
        + 0.00616
        + 0.00495 * (0.235 + 0.944 * math.sin(0.02 + 0.00026))
        + math.hypot(0.944, 0.235) * 0.00495**2 / 2
        + 0.00005
        + 0.055 * TICK
    )
    assert sigma == pytest.approx(expected)


def test_the_corner_sweep_grows_with_the_held_curvature_and_the_relative_yaw():
    face = NT.carried_face(held_pallet(0.95, yaw=0.02), WIDTHS, (0, 0, 0))
    k = NT.corner_sweep_factor(CORNERS[0], face, 0.0, 0.36, -0.34)
    # Codex S4a-2 review: 10.36 mm of rear-axle travel moves the corner 11.30 mm.
    assert NT.stop_distance_m(0.0582864) * k == pytest.approx(0.011308, abs=2e-6)
    assert NT.corner_sweep_factor(CORNERS[0], face, 0.0, 0.0, -0.34) == 1.0


def test_the_lateral_sweep_is_the_exact_constant_curvature_arc():
    # Codex S4a-2 2nd review: kappa 0.36, S 11.4755 mm, lever 1.29 m -> 5.354 mm (not 5.329).
    assert NT.lateral_sweep_m(0.36, 0.0114751, 1.29, 0.1725) == pytest.approx(
        0.0053542, abs=2e-7
    )
    assert NT.lateral_sweep_m(0.0, 0.0115, 1.29, 0.1725) == 0.0
    assert NT.lateral_sweep_m(-0.36, 0.0114751, 1.29, 0.1725) == NT.lateral_sweep_m(
        0.36, 0.0114751, 1.29, 0.1725
    )


def test_the_gap_threshold_adds_the_along_mixing_and_the_sweep_to_b_t():
    from forklift_core.perception.near_field_tracking import wall_erosion_m

    face = NT.carried_face(held_pallet(0.95, yaw=0.02), WIDTHS, (0, 0, 0))
    bound = {"wall_m": 0.0001, "yaw_rad": 0.00026, "along_m": 0.0079}
    got = NT.gap_threshold_m(
        {"depth_m": 0.346}, bound, (0.00616, 0.00495), 1.7, face, 0.0003
    )
    expected = (
        wall_erosion_m(bound, (0.00616, 0.00495), 0.346, 1.7, 0.00005)
        + (0.0079 + 0.00616 + 1.7 * 0.00495) * math.sin(0.02 + 0.00026)
        + 0.0003
        + 0.011
    )
    assert got == pytest.approx(expected)


def run_stops(steps, state=None):
    state = state or NT.StopState()
    out = []
    for k, kw in enumerate(steps):
        state, action = NT.decide_stop(state, float(k), **kw)
        out.append((state, action))
    return out


def tick(terminal=None, insert_end=False, waits=(), standing=False):
    return {
        "terminal": terminal,
        "insert_end": insert_end,
        "waits": tuple(waits),
        "standing": standing,
    }


def test_the_insertion_end_latches_and_arrives_only_standing():
    out = run_stops([tick(insert_end=True), tick(), tick(standing=True)])
    assert [a["hold"] for _, a in out] == [True, True, True]
    assert [a["arrive"] for _, a in out] == [
        False,
        False,
        True,
    ]  # the condition went false: still latched
    assert out[-1][0].reason == "insert_end"


def test_a_waiting_reason_holds_the_insertion_end_arrival_until_it_clears():
    out = run_stops(
        [
            tick(insert_end=True),
            tick(waits=["budget"], standing=True),
            tick(waits=["lost_wait"], standing=True),
            tick(standing=True),
        ]
    )
    assert [a["arrive"] for _, a in out] == [False, False, False, True]
    assert not any(a["released"] for _, a in out)  # never a restart


def test_a_terminal_reason_overrides_the_insertion_end_and_ends_standing():
    out = run_stops(
        [tick(insert_end=True), tick(terminal="stuck"), tick(), tick(standing=True)]
    )
    assert out[1][0].terminal and out[1][0].reason == "stuck" and out[1][1]["changed"]
    assert [a["end"] for _, a in out] == [False, False, False, True]
    assert not any(a["arrive"] for _, a in out)


def test_a_waiting_stop_releases_only_standing_and_keeps_its_first_reason():
    out = run_stops(
        [
            tick(waits=["budget"]),
            tick(waits=["lost_wait", "budget"]),
            tick(),
            tick(standing=True),
            tick(),
        ]
    )
    assert out[1][0].reason == "budget"
    assert [a["hold"] for _, a in out] == [True, True, True, False, False]
    assert [a["released"] for _, a in out] == [False, False, False, True, False]


def test_no_reason_no_stop():
    state, action = NT.decide_stop(NT.StopState(), 0.0, **tick(standing=True))
    assert state == NT.StopState() and not any(action.values())


# Plan D8 S4b: the correction points ---------------------------------------------------------
def corr(
    state,
    now,
    *,
    phase="approach",
    entered=False,
    d=2.0,
    standing=False,
    pixel=0.0,
    source="front",
    version=1,
    episode=1,
):
    return NT.correction_update(
        state,
        now,
        phase=phase,
        entered=entered,
        face_distance_m=d,
        standing=standing,
        pixel_s=pixel,
        source=source,
        frame_version=version,
        episode=episode,
    )


def test_a_correction_point_opens_once_the_estimate_passes_it_and_waits():
    state = NT.CorrectionState()
    state, act = corr(state, 1.0, d=1.0001)
    assert state.point_m is None and act["wait"] is None
    for bad in ({"phase": "insert"}, {"entered": True}):
        assert corr(NT.CorrectionState(), 1.0, d=0.9, **bad)[0].point_m is None
    state, act = corr(state, 2.0, d=0.9995)
    assert (
        state.point_m == 1.0
        and state.trigger_s == 2.0
        and act
        == {
            "wait": "correction:1.0",
            "decide": False,
            "refuse": None,
            "event": "trigger",
        }
    )
    assert state.standing_s is None


def test_a_point_decides_only_standing_on_an_observation_after_the_stop():
    state, _ = corr(NT.CorrectionState(), 2.0, d=0.99)
    state, act = corr(state, 2.2, d=0.98, standing=True, pixel=2.1)
    assert (
        state.standing_s == 2.2
        and not act["decide"]
        and act["wait"] == "correction:1.0"
    )
    state, act = corr(state, 2.3, d=0.98, standing=True, pixel=2.15)  # taken before t_s
    assert not act["decide"]
    # The stop detector sees motion again: t_s restarts.
    moved, act = corr(state, 2.35, d=0.98, standing=False, pixel=2.25)
    assert moved.standing_s is None and not act["decide"]
    again, _ = corr(moved, 2.4, d=0.98, standing=True, pixel=2.25)
    assert again.standing_s == 2.4
    assert not corr(again, 2.5, standing=True, pixel=2.3)[1][
        "decide"
    ]  # before the new t_s
    state, act = corr(again, 2.6, standing=True, pixel=2.4)
    assert act["decide"] and act["wait"] == "correction:1.0"
    # Not standing at the decision tick: wait.
    assert not corr(again, 2.6, standing=False, pixel=2.4)[1]["decide"]


def test_a_point_refuses_an_observation_not_from_the_front_or_another_hold():
    state, _ = corr(NT.CorrectionState(), 2.0, d=0.59)
    state, _ = corr(state, 2.1, standing=True)
    assert (
        corr(state, 2.5, standing=True, pixel=2.2, source="roof")[1]["refuse"]
        == "source:roof"
    )
    assert (
        corr(state, 2.5, standing=True, pixel=2.2, version=2)[1]["refuse"]
        == "frame:2!=1"
    )


def test_an_installed_point_holds_its_tick_closes_the_next_and_opens_the_next_point_later():
    state, _ = corr(NT.CorrectionState(), 2.0, d=0.99)
    state, _ = corr(state, 2.1, standing=True)
    state, act = corr(state, 2.5, standing=True, pixel=2.2)
    assert act["decide"]
    state = NT.correction_installed(state, 2.5)
    assert state.next_index == 1 and state.installed_s == 2.5
    assert corr(state, 2.5, standing=True)[1]["wait"] == "correction:1.0"
    state, act = corr(state, 2.5 + TICK, d=0.59, standing=True)
    assert (
        state == NT.CorrectionState(next_index=1)
        and act["wait"] is None
        and act["event"] == "closed"
    )
    assert not NT.corrections_missed(NT.CorrectionState(next_index=1, point_m=0.6))
    assert NT.corrections_missed(state)
    state, act = corr(state, 3.0, d=0.599)
    assert state.point_m == 0.6 and act["event"] == "trigger"
    done = NT.CorrectionState(next_index=2)
    assert corr(done, 4.0, d=0.1)[0] == done and not NT.corrections_missed(done)


def test_the_path_digest_is_the_paths_content_and_the_follower_keeps_the_same_copy():
    from forklift_core.control.path_tracking import RearAxlePathTracker

    poses = np.array([[0.0, 0.0, 0.0], [0.04, 0.0, 0.0], [0.08, 0.0, 0.0]])
    directions = np.ones(3, dtype=np.int8)
    curvatures = np.zeros(3)
    digest = NT.path_digest(poses, directions, curvatures)
    assert digest == NT.path_digest(
        poses.tolist(), directions.astype(float), curvatures
    )
    moved = poses.copy()
    moved[2, 1] = 1e-12
    assert NT.path_digest(moved, directions, curvatures) != digest
    tracker = RearAxlePathTracker(poses, directions, curvatures)
    copies = tracker.path_arrays()
    assert NT.path_digest(*copies) == digest
    copies[0][0, 0] = 5.0  # a copy: the follower's own path does not change
    assert NT.path_digest(*tracker.path_arrays()) == digest


# Plan D8 S4c-1: injections ------------------------------------------------------------------
def adapter_module():
    spec = importlib.util.spec_from_file_location(
        "perception_adapter_for_bias", ROOT / "sim/isaac/perception_adapter.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def pocket_pair(x=1.6, y=0.05, psi=0.03, depth=0.6, spacing=0.3725):
    from forklift_core.perception.pocket_observation import Pocket, PocketObservation

    axis = np.array([math.cos(psi), math.sin(psi)])
    normal = np.array([-axis[1], axis[0]])
    mid = np.array([x, y])
    pockets = [
        Pocket((*(mid + sign * spacing / 2 * normal), 0.061), 0.2275, 0.09)
        for sign in (1, -1)
    ]
    return PocketObservation(
        5,
        "synthetic",
        "base_link",
        "synthetic",
        "valid",
        *pockets,
        psi,
        0.001,
        0.001,
        None,
    )


@pytest.mark.parametrize(
    "lateral, yaw",
    [(0.02, 0.0), (-0.06, 0.0), (0.0, 0.015), (0.0, -0.05), (0.01, 0.01)],
)
def test_a_biased_observation_gives_the_biased_target(lateral, yaw):
    """S4c-1: the observation moved about its own pallet centre yields the target moved in the
    pallet frame -- the two injection routes agree."""
    from forklift_core.planning.pallet_mission import PalletSite

    adapter = adapter_module()
    obs = pocket_pair()
    base = ((2.0, -1.0), 0.4)

    def target(o):
        xy = adapter.estimate_pallet_center_m(o, 0.6)
        return adapter.estimate_world_pallet_site(xy, o.insertion_yaw_rad, *base)

    raw = target(obs)
    moved = target(NT.biased_observation(obs, 0.6, lateral, yaw))
    expected = NT.biased_site(PalletSite(raw.x_m, raw.y_m, raw.yaw_rad), lateral, yaw)
    assert (moved.x_m, moved.y_m, moved.yaw_rad) == pytest.approx(
        (expected.x_m, expected.y_m, expected.yaw_rad), abs=1e-12
    )
    # The site moves by lateral along its left normal and turns by yaw.
    c, s = math.cos(raw.yaw_rad), math.sin(raw.yaw_rad)
    d = np.array([expected.x_m - raw.x_m, expected.y_m - raw.y_m])
    assert float(d @ (-s, c)) == pytest.approx(lateral, abs=1e-12) and float(
        d @ (c, s)
    ) == pytest.approx(0, abs=1e-12)
    assert NT._wrap(expected.yaw_rad - raw.yaw_rad) == pytest.approx(yaw, abs=1e-12)


def test_a_biased_observation_keeps_sizes_spacing_stamp_and_sigmas():
    obs = pocket_pair()
    moved = NT.biased_observation(obs, 0.6, 0.06, 0.05)
    for a, b in ((obs.left, moved.left), (obs.right, moved.right)):
        assert (a.width_m, a.height_m, a.center_m[2]) == (
            b.width_m,
            b.height_m,
            b.center_m[2],
        )
    spacing = lambda o: math.dist(o.left.center_m[:2], o.right.center_m[:2])  # noqa: E731
    assert spacing(moved) == pytest.approx(spacing(obs), abs=1e-12)
    assert (moved.stamp_ns, moved.position_sigma_m, moved.yaw_sigma_rad) == (
        5,
        0.001,
        0.001,
    )
    assert (
        obs.left.center_m == pocket_pair().left.center_m
    )  # the original is not touched
    with pytest.raises(ValueError):
        NT.biased_observation(
            type(obs)(
                5,
                "synthetic",
                "base_link",
                "synthetic",
                "invalid",
                None,
                None,
                None,
                None,
                None,
                "x",
            ),
            0.6,
            0.02,
            0.0,
        )


def test_the_correction_heading_box_covers_the_followers_transient_after_the_first_correction():
    """Plan D8 S4b revision (S4c-1 smoke): from any start inside the 1.0 m box the ideal
    follower's heading stays within the correction box, and a start the old 0.05 rad box
    refused (the smoke's 0.6 m pose) still arrives within the dry-run criteria."""
    from forklift_core.control.path_tracking import TrackerConfig
    from forklift_core.control.rollout import bicycle_rollout
    from forklift_core.planning.geometry import Pose2D
    from forklift_core.planning.pallet_mission import straight_from_pose

    cfg = TrackerConfig(
        cruise_speed_mps=0.055,
        max_curvature_inv_m=0.36,
        max_acceleration_mps2=0.3,
        lookahead_m=0.28,
        position_tolerance_m=0.008,
        yaw_tolerance_rad=0.02,
        stop_speed_mps=0.012,
        max_cross_track_error_m=0.35,
        overshoot_tolerance_m=0.03,
    )
    worst = 0.0
    for lat in (-0.05, 0.0, 0.05):
        for yaw in (-0.05, 0.0, 0.05):
            st, _ = straight_from_pose(
                Pose2D(0.0, lat, yaw),
                Pose2D(-1.0, 0.0, 0.0),
                Pose2D(0.9, 0.0, 0.0),
                max_lateral_m=0.05,
                max_yaw_rad=0.05,
                min_length_m=0.3,
            )
            run = bicycle_rollout(
                st.poses,
                st.directions,
                st.curvatures_inv_m,
                cfg,
                np.array([0.0, lat, yaw]),
            )
            worst = max(worst, float(np.abs(np.asarray(run.trajectory)[:, 2]).max()))
    assert 0.05 < worst <= NT.CORRECTION_MAX_YAW_RAD
    st, _ = straight_from_pose(
        Pose2D(0.0, 0.0122, -0.0875),
        Pose2D(-1.0, 0.0, 0.0),
        Pose2D(0.5312, 0.0, 0.0),
        max_lateral_m=0.05,
        max_yaw_rad=NT.CORRECTION_MAX_YAW_RAD,
        min_length_m=0.3,
    )
    run = bicycle_rollout(
        st.poses,
        st.directions,
        st.curvatures_inv_m,
        cfg,
        np.array([0.0, 0.0122, -0.0875]),
    )
    last = np.asarray(run.trajectory[-1])
    assert run.status == "arrived" and abs(last[1]) <= 0.006 and abs(last[2]) <= 0.015


# Plan D8 S4c-2: the pickup retry ------------------------------------------------------------
def test_a_retry_needs_a_mismatch_or_refusal_before_the_protection_far_enough_and_once():
    assert NT.retry_allowed("mismatch", 0, False, 2.2)
    assert NT.retry_allowed("replan_rejected", 0, False, 0.59)
    for reason in ("lost", "unbounded", "stuck", "correction_missed", None):
        assert not NT.retry_allowed(reason, 0, False, 2.2), reason
    assert not NT.retry_allowed("mismatch", 1, False, 2.2)  # at most one per pickup
    assert not NT.retry_allowed("mismatch", 0, True, 2.2)  # the protection had started
    assert not NT.retry_allowed(
        "mismatch", 0, False, 0.5499
    )  # too close to see it again
    assert not NT.retry_allowed("mismatch", 0, False, float("nan"))
    # 0.55 m leaves the camera beyond the far detector's 0.8 m minimum (0.331 m behind the tips).
    assert NT.RETRY_MIN_FACE_M + 0.331 > 0.8 + 0.05


def test_s4c3_the_correction_points_of_both_dry_runs_by_the_trigger_distance():
    """Plan D8 S4c-3 stage 1: the rolled-out distance is the trigger's (the observation held at
    its capture, seen from each pose's control base), and every run's crossings must lie in
    the S4b correction box -- the second run included (Codex S4c-3 review 2: a name reused in
    the loop broke the second run)."""
    from forklift_core.perception.near_field_tracking import held_pockets
    from forklift_core.perception.pocket_observation import Pocket, PocketObservation
    from forklift_core.planning import Pose2D

    # The face 2.6 m ahead of a base at the origin, pockets across +-0.18625 m.
    observation = PocketObservation(
        5,
        "synthetic",
        "base_link",
        "synthetic",
        "valid",
        Pocket((2.6, 0.18625, 0.061), 0.2275, 0.09),
        Pocket((2.6, -0.18625, 0.061), 0.2275, 0.09),
        0.0,
        0.001,
        0.001,
        None,
    )
    held = held_pockets(observation, (0.0, 0.0, 0.0))
    widths = (0.2275, 0.2275)
    distance = NT.rollout_face_distance(held, widths, 0.95, -0.34)
    # The rear axle 0.34 m behind the base: the fork tips (base x 0.95) 1.65 m from the face.
    assert distance((-0.34, 0.0, 0.0)) == pytest.approx(2.6 - 0.95)
    face = NT.carried_face(held, widths, (0.0, 0.0, 0.0))
    assert NT.face_distance_m(face, 0.95) == pytest.approx(2.6 - 0.95)
    line = Pose2D(-0.34, 0.0, 0.0)

    def run(lateral, yaw):
        return [[x, lateral, yaw] for x in [-0.34 + 0.01 * i for i in range(200)]]

    points, inside = NT.correction_points_check(
        [run(0.0, 0.0), run(0.01, 0.02)], line, distance
    )
    assert inside and [(p["dry_run"], p["point_m"]) for p in points] == [
        (0, 1.0),
        (0, 0.6),
        (1, 1.0),
        (1, 0.6),
    ]
    assert points[2]["offsets"]["lateral_m"] == pytest.approx(0.01)
    # Outside the box on the second run only, then a run that never comes close.
    _, inside = NT.correction_points_check(
        [run(0.0, 0.0), run(0.06, 0.0)], line, distance
    )
    assert not inside
    _, inside = NT.correction_points_check(
        [run(0.0, 0.0), run(0.0, 0.13)], line, distance
    )
    assert not inside
    points, inside = NT.correction_points_check([run(0.0, 0.0)[:20]], line, distance)
    assert not inside and points[0]["offsets"] is None


# Plan D8 S4 near field: the protection prediction at the 0.6 m correction (design v4) -------------
def _prediction_inputs():
    import hashlib
    import json

    from insertion_geometry import read_fork_blades_m

    from forklift_core.perception.near_field_bounds import NearFieldBounds

    # The recorded cases ran at 0.29 m fork spacing: forks, bounds and drive terms pinned with them
    # (tests/fixtures/near_protection_prediction/README.md).
    fixtures = ROOT / "tests/fixtures/near_protection_prediction"
    bounds_path = fixtures / "near_field_bounds_0p29.json"
    bounds = NearFieldBounds.load(bounds_path)
    terms = NT.DriveTerms.from_dict(
        json.loads((fixtures / "near_field_drive_terms_0p29.json").read_text()),
        hashlib.sha256(bounds_path.read_bytes()).hexdigest(),
        json.loads(bounds_path.read_text())["latency"],
    )
    blades = read_fork_blades_m(fixtures / "forklift_0p29.urdf")
    cases = json.loads(
        (ROOT / "tests/fixtures/near_protection_prediction/cases.json").read_text()
    )["cases"]
    return bounds, terms, blades, cases


def _predict(case, bounds, terms, blades, **override):
    from forklift_core.control import TrackerConfig

    a = override.pop("approach", case["approach"])
    return NT.protection_prediction(
        (a["poses"], a["directions"], a["curvatures_inv_m"]),
        TrackerConfig(**case["tracker_config"]),
        override.pop("rear", case["rear"]),
        case["held"],
        tuple(case["widths_m"]),
        rear_axle_offset_m=case["rear_axle_offset_m"],
        fork_tip_x_m=case["fork_tip_x_m"],
        camera_xy=tuple(case["camera_xy_m"]),
        blades=blades,
        carriage_limit_m=case["carriage_limit_m"],
        blade_y_max_m=max(max(abs(b[2]), abs(b[3])) for b in blades),
        bounds=bounds,
        terms=terms,
    )


def test_the_protection_prediction_reproduces_the_recorded_cases():
    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    bounds, terms, blades, cases = _prediction_inputs()
    by_run = {c["run"]: c for c in cases}
    stuck = _predict(by_run["seed_6_v0.055_corr_lat_0.02"], bounds, terms, blades)
    passed = _predict(by_run["seed_0_v0.055_none_0"], bounds, terms, blades)
    for case, out in (
        (by_run["seed_6_v0.055_corr_lat_0.02"], stuck),
        (by_run["seed_0_v0.055_none_0"], passed),
    ):
        assert out["reached"] and out["face_distance_m"] <= NT.PROTECT_W_M
        assert out["margin_m"] == pytest.approx(case["expected"]["margin_m"], abs=1e-9)
        assert out["relative_yaw_rad"] == pytest.approx(
            case["expected"]["relative_yaw_rad"], abs=1e-9
        )
        assert out["loss_m"] == pytest.approx(
            NT.PREDICT_LOSS_PER_RAD * abs(out["relative_yaw_rad"]) + NT.PREDICT_LOSS_M
        )
        assert out["margin_phys_m"] == pytest.approx(out["margin_m"] - out["loss_m"])
        assert (
            out["bound"] is not None and 0.4 < out["camera_distance_m"] < 0.6
        )  # the roof bin seen there
    # The stuck run (first fresh margin -8.5 mm) screens short; the passed one (+10.4 mm) does not.
    assert stuck["margin_phys_m"] == pytest.approx(-0.0176, abs=1e-4)
    assert passed["margin_phys_m"] > 0.0


def test_the_same_recorded_case_clears_the_protection_with_the_current_fork_spacing():
    # EPAL 6 fork spacing plan (2026-10-13): the refused seed 6 decision, with the forks of the
    # current measured model (0.3725 m) and the pinned 0.29 m bounds and drive terms, predicts a
    # positive physical margin -- a geometry regression, not an Isaac result.
    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    from insertion_geometry import read_fork_blades_m

    bounds, terms, _, cases = _prediction_inputs()
    blades = read_fork_blades_m(ROOT / "sim/models/dls08_measured/forklift.urdf")
    assert max(abs(b[2] + b[3]) / 2 for b in blades) == pytest.approx(0.18625)
    case = {c["run"]: c for c in cases}["seed_6_v0.055_corr_lat_0.02"]
    out = _predict(case, bounds, terms, blades)
    assert out["margin_phys_m"] == pytest.approx(0.021123, abs=1e-5)


def test_a_path_ending_before_the_protection_is_judged_at_its_end_and_no_bound_is_unpredictable():
    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    bounds, terms, blades, cases = _prediction_inputs()
    case = cases[0]
    a = case["approach"]
    poses = np.asarray(a["poses"], dtype=float)
    # The same straight approach ending 0.5 m short: its end is reached before the protection's 0.15 m.
    end = poses[-1].copy()
    end[:2] -= 0.5 * np.array([math.cos(end[2]), math.sin(end[2])])
    short = {
        "poses": [poses[0].tolist(), end.tolist()],
        "directions": a["directions"][:2],
        "curvatures_inv_m": a["curvatures_inv_m"][:2],
    }
    # From the line's own start it arrives there: judged at that end.
    out = _predict(case, bounds, terms, blades, approach=short, rear=poses[0].tolist())
    assert (
        not out["reached"]
        and out["dry_status"] == "arrived"
        and out["margin_m"] is not None
    )
    # From the recorded stop (off the line) the short straight ends in a heading failure: reaching
    # neither the protection nor the end, it is unpredictable (design v4).
    failed = _predict(case, bounds, terms, blades, approach=short)
    assert (
        not failed["reached"]
        and failed["dry_status"] == "failed"
        and failed["margin_m"] is None
    )

    class NoRoof:
        def __init__(self, inner):
            self.inner = inner

        def __getattr__(self, name):
            return getattr(self.inner, name)

        def observation(self, source, distance):
            return None

    none = _predict(case, NoRoof(bounds), terms, blades)
    assert (
        none["bound"] is None
        and none["margin_m"] is None
        and none["margin_phys_m"] is None
    )
    assert (
        NT.prediction_disposition(
            none["margin_phys_m"], lost_injection=False, retry_allowed=True
        )
        == "unpredictable"
    )


def test_the_prediction_disposition_is_exclusive_in_its_order():
    d = NT.prediction_disposition
    assert d(None, lost_injection=True, retry_allowed=True) == "unpredictable"
    assert d(0.0, lost_injection=True, retry_allowed=True) == "accepted"
    assert d(0.002, lost_injection=False, retry_allowed=False) == "accepted"
    assert (
        d(-0.001, lost_injection=True, retry_allowed=False) == "accepted_lost_injection"
    )
    assert (
        d(-0.001, lost_injection=True, retry_allowed=True) == "accepted_lost_injection"
    )
    assert d(-0.001, lost_injection=False, retry_allowed=False) == "accepted_short"
    assert d(-0.001, lost_injection=False, retry_allowed=True) == "refused"


def test_the_slam_mode_and_map_are_compared_by_meaning_not_by_the_bundle_path(tmp_path):
    """SLAM localization plan L7 (Codex S3 design P2): an old condition reads as online mapping; a localization run
    matches only a condition measured on the same map, whatever path the bundle was given at."""

    def problems(expected_result, observed_result):
        return [p for p in NT.condition_problems(NT.run_condition(expected_result, tmp_path),
                                                  NT.run_condition(observed_result, tmp_path))
                if p.startswith("arguments")]

    old = {"arguments": {"fps": 60}}
    mapping = {"arguments": {"fps": 60}}
    on_a = {"arguments": {"fps": 60, "slam_map_bundle": "/b/one", "expected_map_id": "a" * 64},
            "slam_map": {"mode": "localization", "map_id": "a" * 64}}
    on_a_elsewhere = {"arguments": {"fps": 60, "slam_map_bundle": "/c/two", "expected_map_id": "a" * 64},
                      "slam_map": {"mode": "localization", "map_id": "a" * 64}}
    on_b = {"arguments": {"fps": 60, "slam_map_bundle": "/b/one", "expected_map_id": "b" * 64},
            "slam_map": {"mode": "localization", "map_id": "b" * 64}}
    stored_old = {"arguments": {"fps": 60}}  # a condition exported before the fields existed
    assert NT.run_condition(old, tmp_path)["arguments"]["slam_mode"] == "mapping"
    assert problems(mapping, mapping) == []
    old_condition = NT.run_condition(stored_old, tmp_path)
    for key in ("slam_mode", "map_id"):
        old_condition["arguments"].pop(key)
    assert [p for p in NT.condition_problems(old_condition, NT.run_condition(mapping, tmp_path))
            if p.startswith("arguments")] == []
    assert problems(on_a, on_a_elsewhere) == []
    assert problems(on_a, on_b) == ["arguments map_id differs from the measurement runs"]
    assert sorted(problems(mapping, on_a)) == ["arguments map_id differs from the measurement runs",
                                               "arguments slam_mode differs from the measurement runs"]
    assert "input_files" not in str(problems(on_a, on_a))  # the bundle folder is not an input file
    # live-default plan 5d: the launcher's choice is a label for the console and the record, not a run condition
    named = {"arguments": {"fps": 60, "slam_choice": "mapping (fallback: no SCENE_FILE, the factory hall has no registry row)"}}
    assert problems(mapping, named) == [] and "slam_choice" in NT.RUN_SPECIFIC_ARGUMENTS


def test_the_d8b_reference_check_compares_the_slam_map_by_meaning_too(tmp_path):
    """Codex S3 review P2: s4_drive_terms' reference check hashed the bundle folder as an input file."""
    import sys

    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "tools"))
    import s4_drive_terms

    run = {"arguments": {"slam_map_bundle": str(tmp_path), "expected_map_id": "a" * 64},
           "slam_map": {"mode": "localization", "map_id": "a" * 64}, "slam_feedback": {"x": 1}}
    elsewhere = {**run, "arguments": {"slam_map_bundle": str(tmp_path / "copy"), "expected_map_id": "a" * 64}}
    other = {**run, "slam_map": {"mode": "localization", "map_id": "b" * 64}}
    assert s4_drive_terms.reference_problems("k", run, elsewhere) == []
    assert s4_drive_terms.reference_problems("k", run, other) == ["k: argument map_id differs from the D8b run"]


def test_the_d8b_reference_check_compares_the_far_recognitions_fixed_files(tmp_path):
    """Codex near-remeasure round 2 P2: an S3 run's far track bounds changed from its S2 reference's is a difference
    (the S3 runs agreeing among themselves would otherwise link S2 bounds and S3 terms of two far conditions)."""
    import shutil
    import sys

    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "tools"))
    import s4_drive_terms

    snaps = {}
    for name in ("s2", "s3"):
        snap = tmp_path / "snapshots" / name
        for rel in NT.FAR_RECOGNITION_FILES + ("config/obstacle_odometry_age.json",):
            (snap / rel).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(root / rel, snap / rel)
        snaps[name] = {"arguments": {"recognition": "far", "lidar": str(snap / "config/site_load_defaults.yaml")},
                       "slam_feedback": {"x": 1}}
    assert s4_drive_terms.reference_problems("k", snaps["s3"], snaps["s2"]) == []
    bounds = tmp_path / "snapshots/s3/config/recognition_track_bounds.yaml"
    bounds.write_text(bounds.read_text() + "\n# changed\n")
    assert s4_drive_terms.reference_problems("k", snaps["s3"], snaps["s2"]) == [
        "k: far-recognition config/recognition_track_bounds.yaml differs from the D8b run"
    ]
