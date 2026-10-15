"""Measure depth-noise structure from real depth images (design §5.3, stage 1B).

Stereo depth noise is roughly constant in inverse depth w = 1/z, i.e. sigma_z = k z^2.
A plane is exactly linear in w over pixel coordinates, so the second difference
w[i-s] - 2 w[i] + w[i+s] removes a plane and keeps the noise. Its variance against
the step s separates a per-pixel part (A) from a spatially correlated part (B,
Gaussian-filtered white noise of width g pixels):

    V(s) = 6 A + B (6 - 8 rho(s) + 2 rho(2 s)),   rho(s) = exp(-s^2 / (4 g^2)).

A second difference cannot tell sensor error from real surface shape, so callers
fit each frame separately on masked background and cross-check the fitted model
against per-patch plane residuals (patch_plane_rms vs model_patch_rms).
"""

from dataclasses import dataclass

import numpy as np

from forklift_core.perception.depth_noise import correlated_field

STEPS = (1, 2, 3, 4, 6, 8, 12, 16, 24, 32)
GRID_PX = np.geomspace(0.5, 40.0, 120)


@dataclass(frozen=True)
class NoiseStructure:
    k_iid: float
    k_corr: float
    correlation_px: tuple
    rel_residual_rms: float
    at_grid_edge: bool


def _inverse(depth_m, valid):
    depth = np.asarray(depth_m, dtype=float)
    ok = np.isfinite(depth) & (depth > 0)
    if valid is not None:
        ok &= np.asarray(valid, dtype=bool)
    return np.where(ok, 1.0 / np.where(ok, depth, 1.0), np.nan), np.where(ok, depth, np.nan)


def second_differences(depth_m, *, axis: int, step: int, z_range=(1.5, 4.0), valid=None):
    w, z = _inverse(depth_m, valid)
    a = np.moveaxis(w, axis, 0)
    centre = np.moveaxis(z, axis, 0)[step:-step]
    d = a[:-2 * step] - 2.0 * a[step:-step] + a[2 * step:]
    keep = np.isfinite(d) & (centre >= z_range[0]) & (centre < z_range[1])
    return d[keep]


def robust_variance(samples) -> float:
    x = np.asarray(samples, dtype=float)
    if x.size < 1000:
        raise ValueError("robust variance needs at least 1000 samples")
    return float(np.square(1.4826 * np.median(np.abs(x - np.median(x)))))


def _rho(s, g):
    return np.exp(-np.square(s) / (4.0 * g * g))


def _shape(steps, g):
    s = np.asarray(steps, dtype=float)
    return 6.0 - 8.0 * _rho(s, g) + 2.0 * _rho(2.0 * s, g)


def _nonnegative_lstsq(x, y):
    coef = np.linalg.lstsq(x, y, rcond=None)[0]
    if np.all(coef >= 0):
        return coef
    best = None
    for j in range(x.shape[1]):
        c = np.zeros(x.shape[1])
        col = x[:, j]
        c[j] = max(0.0, float(col @ y / (col @ col)))
        r = float(np.sum(np.square(x @ c - y)))
        if best is None or r < best[0]:
            best = (r, c)
    return best[1]


def fit_noise_structure(var_u: dict, var_v: dict) -> NoiseStructure:
    su, sv = sorted(var_u), sorted(var_v)
    y = np.array([var_u[s] for s in su] + [var_v[s] for s in sv], dtype=float)
    if not np.all(np.isfinite(y)) or np.any(y <= 0):
        raise ValueError("variances must be finite and positive")
    weight = 1.0 / y
    best = None
    for gu in GRID_PX:
        cu = _shape(su, gu)
        for gv in GRID_PX:
            x = np.column_stack((np.full(y.size, 6.0), np.concatenate((cu, _shape(sv, gv)))))
            coef = _nonnegative_lstsq(x * weight[:, None], y * weight)
            r = float(np.sum(np.square((x @ coef - y) * weight)))
            if best is None or r < best[0]:
                best = (r, coef, gu, gv)
    r, (a, b), gu, gv = best
    k_iid, k_corr = float(np.sqrt(a)), float(np.sqrt(b))
    edge = k_corr >= 0.1 * k_iid and any(g in (GRID_PX[0], GRID_PX[-1]) for g in (gu, gv))
    return NoiseStructure(k_iid, k_corr, (float(gu), float(gv)), float(np.sqrt(r / y.size)), bool(edge))


def _tiles(array, patch):
    h, w = array.shape
    rows, cols = h // patch, w // patch
    t = array[: rows * patch, : cols * patch].reshape(rows, patch, cols, patch).transpose(0, 2, 1, 3)
    return t.reshape(rows * cols, patch * patch)


def _plane_rms(w_tiles, patch):
    v, u = np.mgrid[0:patch, 0:patch]
    design = np.column_stack((u.ravel(), v.ravel(), np.ones(patch * patch)))
    residual = w_tiles - w_tiles @ (design @ np.linalg.pinv(design)).T
    n = patch * patch
    return np.sqrt(np.mean(np.square(residual), axis=1) * n / (n - 3))


def patch_plane_rms(depth_m, valid, *, patch: int = 48, z_range=(1.5, 4.0)):
    w, z = _inverse(depth_m, valid)
    wt, zt = _tiles(w, patch), _tiles(z, patch)
    keep = np.all(np.isfinite(wt), axis=1)
    mean_z = np.where(keep, np.nanmean(np.where(np.isfinite(zt), zt, 0.0), axis=1), np.nan)
    keep &= (mean_z >= z_range[0]) & (mean_z < z_range[1])
    return _plane_rms(wt[keep], patch)


def model_patch_rms(structure: NoiseStructure, *, patch: int = 48, size=(720, 1280), seed: int = 0) -> float:
    rng = np.random.default_rng(seed)
    field = structure.k_iid * rng.normal(0.0, 1.0, size)
    if structure.k_corr > 0:
        field = field + structure.k_corr * correlated_field(size, structure.correlation_px, rng)
    return float(np.median(_plane_rms(_tiles(field, patch), patch)))


def _box_any(mask, radius: int):
    m = np.pad(np.asarray(mask, dtype=np.int64), ((radius + 1, radius), (radius + 1, radius)))
    c = m.cumsum(0).cumsum(1)
    h, w = mask.shape
    e = 2 * radius + 1
    return (c[e:e + h, e:e + w] - c[:h, e:e + w] - c[e:e + h, :w] + c[:h, :w]) > 0


def erode(mask, radius: int):
    return ~_box_any(~np.asarray(mask, dtype=bool), radius)


def background_mask(depth_m, object_masks, *, margin_px: int = 8):
    depth = np.asarray(depth_m, dtype=float)
    bad = ~(np.isfinite(depth) & (depth > 0))
    for mask in object_masks:
        bad |= np.asarray(mask, dtype=bool)
    return ~_box_any(bad, margin_px)


def dropout_fraction(depth_m, region) -> float:
    values = np.asarray(depth_m, dtype=float)[np.asarray(region, dtype=bool)]
    if not values.size:
        raise ValueError("empty region")
    return float(np.mean(~np.isfinite(values) | (values <= 0)))
