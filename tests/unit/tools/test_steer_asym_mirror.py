"""The steering mirror check (docs/validation/2026-11-18-steer-asym-mirror/mirror.py)."""

import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("mirror", ROOT / "docs/validation/2026-11-18-steer-asym-mirror/mirror.py")
M = importlib.util.module_from_spec(spec)
spec.loader.exec_module(M)


def stratum(m0, m1, valid=True):
    return {"valid": valid, "mabs": [m0, m1], "exposure_s": 2.0, "ratio": 0.9}


def test_a_symmetric_truck_has_no_mirror_difference_but_a_within_layer_one():
    # left turn: left (inner) 0.020, right (outer) 0.004; right turn mirrored: right (inner) 0.020, left (outer) 0.004
    runs = {"r": {"F2": {"fwd_left_slow": stratum(0.020, 0.004), "fwd_right_slow": stratum(0.004, 0.020)}}}
    rows = M.pairs(runs, "F2")
    assert len(rows) == 1
    assert rows[0]["within_left_mrad"] == pytest.approx(16.0) and rows[0]["within_right_mrad"] == pytest.approx(16.0)
    assert rows[0]["inner_diff_mrad"] == pytest.approx(0.0) and rows[0]["outer_diff_mrad"] == pytest.approx(0.0)


def test_a_left_heavy_truck_shows_in_the_mirror_check():
    runs = {"r": {"O": {"fwd_left_slow": stratum(0.035, 0.002), "fwd_right_slow": stratum(0.030, 0.001)}}}
    r = M.pairs(runs, "O")[0]
    assert r["inner_diff_mrad"] == pytest.approx(34.0) and r["outer_diff_mrad"] == pytest.approx(-28.0)


def test_only_strata_valid_in_both_turn_directions_count():
    runs = {"r": {"F2": {"fwd_left_slow": stratum(0.02, 0.0), "fwd_right_slow": stratum(0.0, 0.02, valid=False)}}}
    assert M.pairs(runs, "F2") == [] and M.summary([]) == {"n": 0}


def test_the_committed_record_reads():
    s = M.summary(M.pairs(__import__("json").loads(M.MECH.read_text())["runs"], "F2"))
    assert s["n"] > 0 and abs(s["inner_diff_median_mrad"]) < 5
