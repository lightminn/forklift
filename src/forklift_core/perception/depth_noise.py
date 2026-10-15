# src/forklift_core/perception/depth_noise.py
"""A datasheet-level D435i-like depth model for rendered scenes (design §5.3).

Every parameter is a model value, not a measurement: axial sigma k*z^2, mixed
("flying") pixels on the far side of depth edges, random dropouts, rounding, and
a fixed RGB/label shift against the depth grid. Gates are judged on noisy
renders because clean renders make p95 optimistic. An optional Gaussian-correlated
k_corr z^2 component (stage 1B, measured on real D435i depth) draws from its own
random stream, so the defaults reproduce stage 1A exactly.
"""

from dataclasses import asdict, dataclass, replace

import numpy as np

from forklift_core.perception.scene_dataset import SceneInput


@dataclass(frozen=True)
class NoiseModel:
    axial_k: float = 0.005
    edge_jump_m: float = 0.05
    flying_fraction: float = 0.5
    registration_shift_px: tuple = (2, 0)
    dropout_fraction: float = 0.01
    quantize_step_m: float = 0.001
    seed: int = 0
    correlated_k: float = 0.0
    correlation_px: tuple = (0.0, 0.0)

    def to_json(self) -> dict:
        obj = asdict(self)
        obj["registration_shift_px"] = list(self.registration_shift_px)
        if self.correlated_k > 0:
            obj["correlation_px"] = [float(v) for v in self.correlation_px]
        else:
            del obj["correlated_k"], obj["correlation_px"]
        return obj


def gaussian_kernel(sigma_px: float):
    if sigma_px <= 0:
        return np.ones(1)
    radius = int(np.ceil(4 * sigma_px))
    x = np.arange(-radius, radius + 1)
    kernel = np.exp(-0.5 * np.square(x / sigma_px))
    return kernel / kernel.sum()


def correlated_field(shape, sigma_px, rng):
    """Unit-variance Gaussian-filtered white noise; sigma_px = (along u, along v) in output pixels."""
    ku, kv = gaussian_kernel(sigma_px[0]), gaussian_kernel(sigma_px[1])
    h, w = shape
    white = rng.normal(0.0, 1.0, (h + len(kv) - 1, w + len(ku) - 1))
    rows = sum(c * white[:, i:i + w] for i, c in enumerate(ku))
    field = sum(c * rows[j:j + h, :] for j, c in enumerate(kv))
    return field / np.sqrt(np.sum(np.square(ku)) * np.sum(np.square(kv)))


def _shift(image, du, dv, fill):
    out = np.full_like(image, fill)
    h, w = image.shape[:2]
    out[max(0, dv):min(h, h + dv), max(0, du):min(w, w + du)] = \
        image[max(0, -dv):min(h, h - dv), max(0, -du):min(w, w - du)]
    return out


def apply_noise(scene: SceneInput, labels, model: NoiseModel):
    rng = np.random.default_rng(model.seed)
    clean = scene.depth_m
    finite = np.isfinite(clean)
    sigma = model.axial_k * np.square(np.where(finite, clean, 0.0))
    depth = np.where(finite, clean + rng.normal(0.0, 1.0, clean.shape) * sigma, clean)
    if model.correlated_k > 0:
        field = correlated_field(clean.shape, model.correlation_px, np.random.default_rng([model.seed, 1]))
        depth = np.where(finite, depth + field * model.correlated_k * np.square(np.where(finite, clean, 0.0)), depth)
    mix = np.zeros(clean.shape, bool)
    target = np.full(clean.shape, np.nan)
    for axis in (0, 1):
        for step in (1, -1):
            neighbour = np.roll(clean, step, axis=axis)
            edge = finite & np.isfinite(neighbour) & (clean - neighbour > model.edge_jump_m)
            if axis == 0:
                edge[0 if step == 1 else -1, :] = False
            else:
                edge[:, 0 if step == 1 else -1] = False
            fresh = edge & ~mix
            target = np.where(fresh, neighbour, target)
            mix |= edge
    mix &= rng.random(clean.shape) < model.flying_fraction
    t = rng.random(clean.shape)
    depth = np.where(mix, depth + t * (target - depth), depth)
    depth = np.where(rng.random(clean.shape) < model.dropout_fraction, np.nan, depth)
    if model.quantize_step_m > 0:
        depth = np.round(depth / model.quantize_step_m) * model.quantize_step_m
    du, dv = model.registration_shift_px
    return replace(scene, depth_m=depth, rgb=_shift(scene.rgb, du, dv, 0)), _shift(np.asarray(labels), du, dv, -1)
