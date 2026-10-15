# tests/unit/perception/test_depth_noise.py
from dataclasses import replace

import numpy as np
import pytest

from forklift_core.perception.depth_noise import NoiseModel, apply_noise, correlated_field
from tools import instance_rig, scene_rig

QUIET = dict(edge_jump_m=1e9, dropout_fraction=0.0, quantize_step_m=0.0, registration_shift_px=(0, 0))


def _wall(x):
    return instance_rig.render_instances([[scene_rig.Box((x, 0.0, 0.5), (0.02, 4.0, 1.0))]], floor=False, back_wall=False)


def test_noise_is_deterministic_and_leaves_input_alone():
    out = _wall(2.0)
    before = out.scene.depth_m.copy()
    a, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(seed=3))
    b, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(seed=3))
    assert np.array_equal(a.depth_m, b.depth_m, equal_nan=True)
    assert np.array_equal(out.scene.depth_m, before, equal_nan=True)


def test_axial_sigma_scales_with_range_squared():
    stds = []
    for x in (1.5, 3.0):
        out = _wall(x)
        noisy, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(**QUIET))
        stds.append(np.nanstd(noisy.depth_m - out.scene.depth_m))
    expected = ((3.0 - 0.76) / (1.5 - 0.76)) ** 2  # optical depth of the wall front at its centre
    assert 0.8 * expected < stds[1] / stds[0] < 1.2 * expected


def test_labels_shift_with_registration():
    out = instance_rig.render_instances([[scene_rig.Box((2.0, 0.0, 0.15), (0.3, 0.6, 0.3))]])
    _, labels = apply_noise(out.scene, out.instance_ids, NoiseModel(registration_shift_px=(3, 0)))
    assert np.argwhere(labels == 0)[:, 1].min() == np.argwhere(out.instance_ids == 0)[:, 1].min() + 3


def test_flying_pixels_move_the_far_side_towards_the_near_side():
    out = instance_rig.render_instances([[scene_rig.Box((2.0, 0.0, 0.15), (0.3, 0.6, 0.3))]], floor=False)
    noisy, _ = apply_noise(out.scene, out.instance_ids,
                           NoiseModel(axial_k=0.0, dropout_fraction=0.0, quantize_step_m=0.0,
                                      registration_shift_px=(0, 0), flying_fraction=1.0))
    changed = np.isfinite(out.scene.depth_m) & (np.abs(noisy.depth_m - out.scene.depth_m) > 1e-9)
    assert changed.any()
    assert np.all(noisy.depth_m[changed] < out.scene.depth_m[changed])  # far pixels pulled nearer


def test_correlated_field_has_unit_variance_and_gaussian_correlation():
    field = correlated_field((400, 600), (3.0, 5.0), np.random.default_rng(1))
    assert field.std() == pytest.approx(1.0, abs=0.05)
    for lag, g, axis in ((3, 3.0, 1), (5, 5.0, 0)):
        a = np.moveaxis(field, axis, 0)
        assert np.mean(a[lag:] * a[:-lag]) == pytest.approx(np.exp(-lag * lag / (4 * g * g)), abs=0.05)


def test_zero_correlated_k_leaves_the_stage1a_noise_untouched():
    out = instance_rig.render_instances([[scene_rig.Box((2.0, 0.0, 0.15), (0.3, 0.6, 0.3))]])
    base = NoiseModel(seed=11)
    a, ids_a = apply_noise(out.scene, out.instance_ids, base)
    b, ids_b = apply_noise(out.scene, out.instance_ids, replace(base, correlation_px=(5.0, 5.0)))
    assert np.array_equal(a.depth_m, b.depth_m, equal_nan=True) and np.array_equal(ids_a, ids_b)
    assert np.array_equal(a.rgb, b.rgb)
    assert set(base.to_json()) == {"axial_k", "edge_jump_m", "flying_fraction", "registration_shift_px",
                                   "dropout_fraction", "quantize_step_m", "seed"}


def test_correlated_noise_grows_with_depth_squared():
    out = instance_rig.render_instances([], floor=False)  # back wall only: z = 5.25 m everywhere
    model = NoiseModel(axial_k=0.0, correlated_k=0.002, correlation_px=(3.0, 3.0), seed=2, **QUIET)
    noisy, _ = apply_noise(out.scene, out.instance_ids, model)
    residual = noisy.depth_m - out.scene.depth_m
    assert residual.std() == pytest.approx(0.002 * 5.25 ** 2, rel=0.1)
    assert model.to_json()["correlation_px"] == [3.0, 3.0]
