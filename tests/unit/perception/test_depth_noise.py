# tests/unit/perception/test_depth_noise.py
import numpy as np

from forklift_core.perception.depth_noise import NoiseModel, apply_noise
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
