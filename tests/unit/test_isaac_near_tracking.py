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
