"""The layer budget diagnosis' records extraction (docs/plans/2026-11-13-layer-budget-diagnosis.md 2): the pose error,
the last-window facts (an estimate-only jump and its time), the pallet's seat offset and destination error, and the
slowed reasons divided by their own phase's ticks. Synthetic records, no Isaac."""

import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("lat_diag_records",
                                              ROOT / "docs/validation/2026-11-11-latency/lat_diag_records.py")
R = importlib.util.module_from_spec(spec)
spec.loader.exec_module(R)


def control(n=200, jump_at=150, jump=0.04):
    t = np.arange(n) / 120.0
    truth = np.stack([0.002 * np.arange(n), np.zeros(n), np.zeros(n)], axis=1)
    est = truth.copy()
    est[jump_at:, 1] += jump  # an estimate-only step
    return np.column_stack([t, est, truth])


def test_the_window_finds_an_estimate_only_jump_and_its_time():
    sc = control()
    w = R.window_facts(sc, float(sc[-1, 0]))
    assert w["est_jump_max_m"] == pytest.approx(math.hypot(0.002, 0.04))
    assert w["est_jump_t"] == pytest.approx(150 / 120.0)
    assert w["truth_jump_at_est_jump_m"] == pytest.approx(0.002) and w["truth_jump_max_m"] == pytest.approx(0.002)
    assert w["pos_err_end_m"] == pytest.approx(0.04)
    assert R.pose_error([0, 1, 1, 0.1, 1, 1, 0.0])["yaw_rad"] == pytest.approx(0.1)
    assert R.pose_error([0, 0, 0, math.pi - 0.01, 0, 0, -math.pi + 0.01])["yaw_rad"] == pytest.approx(-0.02)


def test_a_run_s_pallet_and_slowed_facts(tmp_path):
    run = tmp_path / "r" / "run"
    run.mkdir(parents=True)
    np.save(run / "slam_control.npy", control())
    result = {"success": True, "failure_reason": None, "transitions": [{"to": "approach", "time_s": 0.5}],
              "scenario": {"destination": {"x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.0}},
              "slip_watch": {"baseline": {"seat": [0.93, -0.05, -0.01], "t": 1.0}},
              "samples": [{"phase": "transport", "pallet_position_m": [0.0, 0.0, 0.1]},
                          {"phase": "lower", "pallet_position_m": [1.03, 2.04, 0.1]},
                          {"phase": "settle", "pallet_position_m": [1.0, 2.05, 0.0]}],
              "obstacle_layer": {"control_ticks": {"observe": 100, "transport": 400},
                                 "slowed_reasons": {"observe:ok": 5, "transport:occupied": 8}},
              "delivery_docking": {"rounds": [{"accepted": True, "reason": "ok"}], "goal_error_vs_truth": [0, 0, 0]}}
    (run / "result.json").write_text(json.dumps(result))
    f = R.run_facts(run)
    assert f["pallet"]["seat_offset_at_lift"] == [0.93, -0.05, -0.01]
    assert f["pallet"]["error_at_first_lower_m"] == pytest.approx(0.05)
    assert f["pallet"]["error_at_end_m"] == pytest.approx(0.05)
    assert f["slowed_per_phase_tick"] == {"observe:ok": 0.05, "transport:occupied": 0.02}  # own phase's ticks
    assert f["delivery"]["rounds"] == [{"accepted": True, "reason": "ok", "goal_error_vs_truth": None,
                                        "estimate_vs_truth_at_match": None}]
    assert f["missing"] == [] and f["entries"][0]["phase"] == "approach"
