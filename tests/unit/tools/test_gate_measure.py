"""Measured-chassis gate helpers (plan v10, P0a v10): geometry only, no recorded runs."""

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("gate_measure", ROOT / "tools/gate_measure.py")
GATE = importlib.util.module_from_spec(SPEC)
sys.modules["gate_measure"] = GATE
SPEC.loader.exec_module(GATE)


def test_corner_error_is_the_largest_corner_distance():
    a = (1.0, 2.0, 0.3)
    assert GATE.corner_error_m(a, a, 0.6, 0.8) == 0.0
    shifted = (1.0, 2.01, 0.3)
    assert GATE.corner_error_m(a, shifted, 0.6, 0.8) == pytest.approx(0.01)
    turned = (1.0, 2.0, 0.3 + 0.01)
    assert GATE.corner_error_m(a, turned, 0.6, 0.8) == pytest.approx(2 * 0.5 * math.sin(0.005), rel=1e-6)


def test_the_stopping_model_matches_the_fixed_p0a_values():
    assert GATE.model_distance_m(0.6) == pytest.approx(0.6 * 0.15 + 0.36 / 3.0 + 0.05)
    assert GATE.model_distance_m(-0.3) == GATE.model_distance_m(0.3)


def test_a_corner_that_slides_off_a_straight_is_measured():
    trigger = (0.0, 0.0, 0.0)
    trace = [[0.0, 0.3, 0.0, 0.0, 0.0], [0.1, 0.1, 0.02, 0.004, 0.0]]
    assert GATE.corner_envelope_m(trigger, 0.0, trace, GATE.UNLOADED) == pytest.approx(0.004)
    turning = [[0.1, 0.1, 0.02, 0.0, 0.0]]
    # A truck that keeps straight while its arc curved: the corners leave their circles.
    assert GATE.corner_envelope_m(trigger, 0.5, turning, GATE.UNLOADED) > 0.0


def test_estop_and_tilt_summaries_from_a_minimal_result(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    probe = {"phase": "transport", "trigger_time_s": 10.0, "trigger_rear": [0.0, 0.0, 0.0], "trigger_speed_mps": 0.3,
             "direction": "forward", "loaded": True, "curvature_inv_m": 0.0, "decel_start_s": 0.0083,
             "max_arc_offset_m": 0.0, "stop_time_s": 0.25, "stop_distance_m": 0.03,
             "trace": [[0.0, 0.3, 0.0, 0.0, 0.0], [0.2, 0.0, 0.03, 0.001, 0.0]]}
    samples = [{"time_s": 9.9, "base_tilt_rad": 0.0004}, {"time_s": 10.1, "base_tilt_rad": 0.0009},
               {"time_s": 30.0, "base_tilt_rad": 0.0002}]
    (run / "result.json").write_text(json.dumps({"estop_probes": [probe], "samples": samples}))
    out = GATE.estop_command(type("A", (), {"run": [run]})())
    assert out["model_holds"] and out["probes"] == 1
    assert out["worst_model_slack_m"] == pytest.approx(GATE.model_distance_m(0.3) - 0.03)
    # The tilt comes from every recorded tick, not the 0.1 s samples (Codex stage-1 4th P2):
    # a 1 deg peak between samples is found.
    stamps = np.arange(0.0, 30.0, 1.0 / 120.0)
    tilt_rad = np.where(np.abs(stamps - 10.05) < 0.01, math.radians(1.0), 0.0002)
    quat = np.column_stack((np.cos(tilt_rad / 2), np.sin(tilt_rad / 2), np.zeros_like(stamps), np.zeros_like(stamps)))
    pose = np.column_stack((np.zeros((len(stamps), 3)), quat))
    np.savez(run / "slam_log.npz", joint_stamps_s=stamps, base_pose_world=pose)
    tilt = GATE.tilt_command(type("A", (), {"run": [run]})())
    assert tilt["max_tilt_rad"] == pytest.approx(math.radians(1.0)) == pytest.approx(tilt["braking_tilt_rad"])
    assert tilt["h_lo_m"] == pytest.approx(1.05 - 5.0 * math.tan(math.radians(1.0)))


def test_no_probe_is_not_a_pass(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    (run / "result.json").write_text(json.dumps({"estop_probes": [], "samples": []}))
    assert GATE.estop_command(type("A", (), {"run": [run]})())["model_holds"] is None
    with pytest.raises(SystemExit):
        GATE.tilt_command(type("A", (), {"run": [run]})())  # no per-tick log


def test_bad_inputs_are_refused_not_read_as_zero(tmp_path):
    with pytest.raises(SystemExit):
        GATE.withdraw_command(type("A", (), {"run": [tmp_path], "draws": 0})())
    run = tmp_path / "run"
    run.mkdir()
    (run / "result.json").write_text(json.dumps({"estop_probes": [], "samples": []}))
    pose = np.array([[0, 0, 0, 1.0, 0, 0, 0], [0, 0, 0, np.nan, 0, 0, 0]])
    np.savez(run / "slam_log.npz", joint_stamps_s=np.array([0.0, 0.01]), base_pose_world=pose)
    with pytest.raises(SystemExit):
        GATE.tilt_command(type("A", (), {"run": [run]})())

