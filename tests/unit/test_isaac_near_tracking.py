"""Plan D8 S4a: the runner side of near-field tracking (no Isaac)."""

import importlib.util
import math
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
BLADES = ((0.59, 0.95, 0.1175, 0.1725), (0.59, 0.95, -0.1725, -0.1175))
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
