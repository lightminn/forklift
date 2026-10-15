# tests/unit/perception/test_depth_noise_fit.py
import numpy as np
import pytest

from forklift_core.perception import depth_noise_fit as fit
from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from tools import instance_rig

QUIET = dict(edge_jump_m=1e9, dropout_fraction=0.0, quantize_step_m=0.0, registration_shift_px=(0, 0))
WALL = (5.0, 6.0)  # the rig's back wall sits at z = 5.25 m
MIXED = dict(axial_k=0.0006, correlated_k=0.004, correlation_px=(4.0, 3.0))


def _wall():
    return instance_rig.render_instances([], floor=False)


def _variances(depth, valid=None):
    return [{s: fit.robust_variance(fit.second_differences(depth, axis=axis, step=s, z_range=WALL, valid=valid))
             for s in fit.STEPS} for axis in (1, 0)]


def test_a_plane_has_zero_second_differences():
    d = fit.second_differences(_wall().scene.depth_m, axis=1, step=4, z_range=WALL)
    assert d.size > 100_000 and np.max(np.abs(d)) < 1e-12


def test_iid_noise_is_recovered_without_a_correlated_part():
    out = _wall()
    noisy, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(axial_k=0.0006, seed=3, **QUIET))
    s = fit.fit_noise_structure(*_variances(noisy.depth_m))
    assert s.k_iid == pytest.approx(0.0006, rel=0.10) and s.k_corr < 0.2 * 0.0006


def test_iid_and_correlated_noise_are_separated():
    out = _wall()
    noisy, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(seed=4, **MIXED, **QUIET))
    s = fit.fit_noise_structure(*_variances(noisy.depth_m))
    assert s.k_iid == pytest.approx(0.0006, rel=0.15) and s.k_corr == pytest.approx(0.004, rel=0.15)
    assert s.correlation_px[0] == pytest.approx(4.0, rel=0.2) and s.correlation_px[1] == pytest.approx(3.0, rel=0.2)
    assert not s.at_grid_edge


def test_patch_plane_rms_matches_the_model_prediction():
    out = _wall()
    noisy, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(seed=5, **MIXED, **QUIET))
    observed = np.median(fit.patch_plane_rms(noisy.depth_m, np.ones((480, 640), bool), patch=48, z_range=WALL))
    s = fit.NoiseStructure(0.0006, 0.004, (4.0, 3.0), 0.0, False)
    assert observed == pytest.approx(fit.model_patch_rms(s, patch=48, size=(480, 640)), rel=0.10)
    assert fit.patch_plane_rms(out.scene.depth_m, np.ones((480, 640), bool), z_range=WALL).max() < 1e-12


def test_robust_variance_ignores_sparse_outliers():
    x = np.random.default_rng(0).normal(0.0, 1.0, 20_000)
    x[::20] = 1e3
    assert fit.robust_variance(x) == pytest.approx(1.0, rel=0.15)
    with pytest.raises(ValueError):
        fit.robust_variance(x[:999])


def test_background_mask_and_erosion_keep_a_margin():
    depth = np.full((40, 60), 2.0)
    depth[5, 50] = np.nan
    obj = np.zeros(depth.shape, bool)
    obj[20:25, 10:15] = True
    bg = fit.background_mask(depth, [obj], margin_px=3)
    assert not bg[17, 12] and not bg[20, 12] and bg[16, 12]
    assert not bg[5, 47] and bg[5, 46] and bg[0, 0]
    inner = fit.erode(obj, 1)
    assert inner.sum() == 9 and inner[22, 12] and not inner[20, 12]


def test_dropout_fraction_counts_zero_and_nan():
    depth = np.array([[1.0, 0.0], [np.nan, 2.0]])
    assert fit.dropout_fraction(depth, np.ones_like(depth, bool)) == pytest.approx(0.5)
