# src/forklift_core/perception/depth_noise.py
"""A datasheet-level D435i-like depth model for rendered scenes (design §5.3).

Every parameter is a model value, not a measurement: axial sigma k*z^2, mixed
("flying") pixels on the far side of depth edges, random dropouts, rounding, and
a fixed RGB/label shift against the depth grid. Gates are judged on noisy
renders because clean renders make p95 optimistic. Optional Gaussian-correlated
k z^2 components (stage 1B) each draw from their own random stream, so the
defaults reproduce stage 1A exactly.
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
    components: tuple = ()

    def to_json(self) -> dict:
        obj = asdict(self)
        obj["registration_shift_px"] = list(self.registration_shift_px)
        if self.components:
            obj["components"] = [[float(k), [float(g) for g in width]] for k, width in self.components]
        else:
            del obj["components"]
        return obj

    @classmethod
    def from_yaml(cls, path, *, seed: int, target_fx: float, level: str = "mid", scale: float = 1.0) -> "NoiseModel":
        """One level of a noise config; widths are converted from the source grid to target_fx."""
        import math

        import yaml

        with open(path, encoding="utf-8") as f:
            obj = yaml.safe_load(f)

        def number(value, name, *, low=0.0, inclusive=True, high=math.inf):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) \
                    or value < low or (value == low and not inclusive) or value >= high:
                raise ValueError(f"noise config {name} out of range: {value!r}")
            return float(value)

        if not isinstance(obj, dict):
            raise ValueError("noise config must be a mapping")
        for name in ("source", "license", "revision", "files_sha256"):
            if not isinstance(obj.get(name), str) or not obj[name]:
                raise ValueError(f"noise config needs a non-empty {name}")
        if obj.get("role") not in ("official", "provisional"):
            raise ValueError("noise config role must be official or provisional")
        levels = obj.get("levels")
        if not isinstance(levels, dict) or level not in levels or not isinstance(levels[level], dict):
            raise ValueError(f"noise config has no level {level!r}")
        ratio = number(target_fx, "target_fx", inclusive=False) / number(obj.get("source_fx"), "source_fx",
                                                                         inclusive=False)
        scale = number(scale, "scale", inclusive=False)
        entry = levels[level]
        components = []
        for c in entry.get("components", []):
            px = c.get("correlation_px_source") if isinstance(c, dict) else None
            if not isinstance(px, list) or len(px) != 2:
                raise ValueError("noise config component needs k and two correlation widths")
            widths = tuple(number(g, "correlation_px_source") * ratio * scale for g in px)
            components.append((number(c.get("k"), "component k") * scale, widths))
        return cls(axial_k=number(entry.get("k_iid"), "k_iid") * scale, components=tuple(components),
                   dropout_fraction=number(obj.get("dropout_fraction"), "dropout_fraction", high=1.0),
                   seed=int(seed))


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
    for i, (k, width) in enumerate(model.components):
        field = correlated_field(clean.shape, width, np.random.default_rng([model.seed, 1 + i]))
        depth = np.where(finite, depth + field * k * np.square(np.where(finite, clean, 0.0)), depth)
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
