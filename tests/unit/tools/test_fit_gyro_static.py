import math

import numpy as np
import pytest
import yaml

from tools import fit_gyro_static as fit

FS = 400.0


def test_white_noise_density_is_recovered():
    rng = np.random.default_rng(1)
    density = 0.01  # deg/s/sqrt(Hz)
    rate = rng.normal(0.0, density * math.sqrt(FS), 400 * 1800)
    taus, adev = fit.allan_deviation(rate, FS)
    assert fit.noise_density(taus, adev) == pytest.approx(density, rel=0.1)


def test_a_constant_bias_does_not_change_the_allan_deviation_and_is_its_mean():
    rng = np.random.default_rng(2)
    rate = rng.normal(0.0, 0.2, 400 * 600)
    taus, a = fit.allan_deviation(rate, FS)
    _, b = fit.allan_deviation(rate + 0.21, FS)
    assert np.allclose(a, b)
    assert fit.segment_bias(rate + 0.21) == pytest.approx(0.21, abs=0.002)


def test_the_random_walk_bound_covers_a_true_random_walk():
    rng = np.random.default_rng(3)
    k = 0.002  # deg/s/sqrt(s)
    n = 400 * 3600
    walk = np.cumsum(rng.normal(0.0, k / math.sqrt(FS), n))
    rate = walk + rng.normal(0.0, 0.01 * math.sqrt(FS), n)
    taus, adev = fit.allan_deviation(rate, FS)
    bound = fit.random_walk_bound(taus, adev)
    assert bound >= k * 0.9  # conservative: not under the true walk
    assert bound <= k * 3.0  # and not a loose one when the walk dominates long tau


def test_the_lsb_is_the_smallest_step():
    rate = np.array([0.0, 0.1, 0.2, -0.1, 0.3, 0.1]) + 0.0
    assert fit.lsb(rate) == pytest.approx(0.1)


def test_a_warm_up_ramp_is_measured_against_the_settled_mean():
    t = np.arange(0, 3600, 1 / 50.0)
    drift = 0.013 * np.clip(1 - t / 1800.0, 0, None)  # 0.013 deg/s settling linearly over 30 min
    rate = 0.2 + drift
    out = fit.warm_up(t, rate, settled_from_s=1800.0, minute_s=60.0, tolerance_dps=0.002)
    assert out["change_dps"] == pytest.approx(0.013, abs=0.001)
    assert 1500.0 <= out["settle_s"] <= 1800.0


def test_device_timestamps_are_unwrapped_at_2_to_the_32_microseconds():
    us = np.arange(0, 6000e6, 2500.0)  # 6000 s at 400 Hz, microseconds
    wrapped = np.mod(us, 2.0**32)
    t = fit.device_seconds(wrapped)
    assert t[-1] == pytest.approx(us[-1] * 1e-6)
    assert np.all(np.diff(t) > 0)


def test_a_bias_that_never_settles_reports_the_record_end():
    t = np.arange(0, 3600, 1 / 50.0)
    rate = 0.2 + 0.02 * (t / 3600.0)  # still drifting at the end
    out = fit.warm_up(t, rate, settled_from_s=1800.0, minute_s=60.0, tolerance_dps=0.002)
    assert out["settled"] is False
    assert out["settle_s"] == pytest.approx(3600.0, abs=60.0)


def test_an_exponential_warm_up_is_fitted():
    rng = np.random.default_rng(4)
    t = np.arange(0, 7200, 1 / 20.0)
    rate = -0.19 - 0.03 * np.exp(-t / 900.0) + rng.normal(0, 0.05, len(t))
    out = fit.warm_up_exponential(t, rate)
    assert out["amplitude_dps"] == pytest.approx(-0.03, abs=0.003)
    assert out["tau_s"] == pytest.approx(900.0, rel=0.2)
    assert out["settled_dps"] == pytest.approx(-0.19, abs=0.002)


def test_the_model_file_is_a_gyro_model_with_provenance():
    values = {"scale": 0.01, "initial_bias_dps": 0.25, "temperature_dps": 0.013, "temperature_s": 1800.0,
              "random_walk_dps_rts": 0.0003, "noise_dps_rthz": 0.0115, "lsb_dps": 0.1, "range_dps": 1000.0,
              "mount_error_deg": 1.0, "zupt_s": 0.5, "zupt_k": 4.0, "gap_s": 0.05}
    text = fit.model_text(values, {k: "test" for k in values}, header="a header")
    data = yaml.safe_load(text)
    assert data["format"] == "gyro_model/1"
    assert {k: data[k] for k in values} == values
    assert text.startswith("# a header")
