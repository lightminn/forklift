"""F2 stage C judge (plan section 3): compression against the rigid arm with the axle-tilt support correction, the
stiffness interval, the selection rule and the stop when nothing fits."""

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import f2_calibration_judge as J  # noqa: E402


def _s(mean, lo=None, hi=None):
    return {"mean": mean, "min": mean if lo is None else lo, "max": mean if hi is None else hi}


def _wheel(z, fz, tilt=0.0, sep=-1e-6, zr=0.0):
    return {"z": _s(z, z - zr, z + zr), "fz": _s(fz), "tilt": _s(tilt), "min_sep": _s(sep), "n": _s(2)}


def _static(zs, fz=(74.0, 74.0, 53.0, 53.0), tilt=0.0, ratio=1.0):
    wheels = {w: _wheel(z, f, tilt) for w, z, f in zip(J.WHEELS, zs, fz)}
    rec = {"start": {"converged": True}, "foreign_contacts": 0, "wheel_load_ratio": ratio, "wheels": wheels}
    return {"loads": {ld: rec for ld in J.LOADS}}


RIGID = _static([0.125] * 4)


def _f2(k_eff):
    """An F2 run whose every wheel compresses force / k_eff."""
    fz = (74.0, 74.0, 53.0, 53.0)
    return _static([0.125 - f / k_eff for f in fz], fz)


def test_support_height_and_tilt_correction():
    assert (J.support_height(1e-3) - J.support_height(0.0)) * 1e6 == pytest.approx(39.94, abs=0.01)
    # a wheel that only tilted (no compression): the centre rises by h(alpha); the corrected compression is 0
    rigid = _wheel(0.125, 74.0)
    tilted = _wheel(0.125 + (J.support_height(1e-3) - J.support_height(0.0)), 74.0, tilt=1e-3)
    assert J.wheel_stiffness(rigid, tilted)["compression_m"] == pytest.approx(0.0, abs=1e-12)
    assert J.wheel_stiffness(rigid, tilted)["valid"] is False


def test_stiffness_and_interval():
    r = J.wheel_stiffness(_wheel(0.125, 74.0, sep=-2e-6), _wheel(0.125 - 74.0 / 400_000, 74.0))
    assert r["k"] == pytest.approx(400_000, rel=1e-9) and r["u_m"] == pytest.approx(2e-6)
    assert r["k_lo"] < 400_000 < r["k_hi"] and r["in_band"]
    # a wide interval (noisy windows) fails the width rule
    noisy = J.wheel_stiffness(_wheel(0.125, 74.0, zr=5e-5), _wheel(0.125 - 74.0 / 400_000, 74.0, zr=5e-5))
    assert noisy["k_hi"] / noisy["k_lo"] > 2 and not noisy["in_band"]


def test_selection_picks_the_closest_passing_value():
    loads = {"rigid": {m: ["r1", "r2"] for m in J.MASSES},
             "grid": {"100000": {"O": "a", "F1": "a"}, "140000": {"O": "b", "F1": "b"}, "180000": {"O": "c", "F1": "c"}}}
    files = {"r1": RIGID, "r2": RIGID, "a": _f2(300_000), "b": _f2(380_000), "c": _f2(600_000)}
    out = J.judge(loads, load=files.__getitem__)
    assert out["chosen_k_mat"] == 140000.0 and out["chosen_c_mat"] == pytest.approx(140000 * 310 / 400000)
    assert out["grid"]["180000"]["passes"] is False and out["grid"]["100000"]["passes"] is True
    assert out["repeats"]["O"]["three_delta_z_m"] == 0.0


def test_nothing_fits_stops_and_bad_runs():
    loads = {"rigid": {m: ["r1", "r2"] for m in J.MASSES}, "grid": {"100000": {"O": "c", "F1": "c"}}}
    files = {"r1": RIGID, "r2": RIGID, "c": _f2(600_000)}
    out = J.judge(loads, load=files.__getitem__)
    assert out["chosen_k_mat"] is None and out["verdict"].startswith("none")
    # a rigid run whose loads do not sum to the weight is unusable
    files["r2"] = _static([0.125] * 4, ratio=1.02)
    files["c"] = _f2(400_000)
    out = J.judge(loads, load=files.__getitem__)
    assert out["unusable"] and out["chosen_k_mat"] is None
    # an F2 run with a foreign contact is not a passing grid point
    files["r2"] = RIGID
    bad = _f2(400_000)
    bad["loads"]["loaded"] = dict(bad["loads"]["loaded"], foreign_contacts=3)
    files["c"] = bad
    out = J.judge(loads, load=files.__getitem__)
    assert out["grid"]["100000"]["run_problems"] and out["chosen_k_mat"] is None
    assert math.isfinite(out["grid"]["100000"]["max_abs_log_ratio"])
