"""The latency diagnosis' record measures: the initial phase before the first transition is labelled, the acceleration
sits at the row where the speed changes, and a 10 Hz stop-go shows in the 10 Hz power share."""

import importlib.util
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("lat_diag_phase", ROOT / "docs/validation/2026-11-11-latency/lat_diag_phase.py")
D = importlib.util.module_from_spec(spec)
spec.loader.exec_module(D)


def control(speed):
    t = np.arange(len(speed) + 1) / 120
    x = np.r_[0.0, np.cumsum(speed) / 120]
    C = np.zeros((len(t), 7))
    C[:, 0], C[:, 4] = t, x
    return C


def test_the_initial_phase_is_labelled_without_transitions():
    ph = D.phase_labels(np.arange(5) / 120, [], "observe")
    assert list(ph) == ["observe"] * 5
    ph = D.phase_labels(np.arange(5) / 120, [{"time_s": 2 / 120, "from": "observe", "to": "approach"}], "x")
    assert list(ph) == ["observe", "observe", "approach", "approach", "approach"]


def test_a_dip_at_phase_three_shows_its_lowest_acceleration_at_phase_three():
    v = np.full(1200, 0.3)
    for c in range(0, 1200, 12):
        v[c + 3] = 0.25  # the speed from row c+3 to c+4 drops
    out = D.phase_profile(control(v), [], "observe")
    assert out["min_mod12"] == 3


def test_a_stop_go_at_ten_hertz_has_a_large_ten_hertz_share():
    smooth = D.phase_profile(control(np.full(1200, 0.3)), [], "observe")
    v = np.full(1200, 0.3)
    v[np.arange(1200) % 12 >= 9] = 0.15
    choppy = D.phase_profile(control(v), [], "observe")
    assert choppy["power10hz_share"] > 0.3 and smooth.get("power10hz_share", 0.0) < 0.05
