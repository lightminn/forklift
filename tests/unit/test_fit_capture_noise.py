# tests/unit/test_fit_capture_noise.py
import math

import numpy as np
import pytest

from forklift_core.perception.depth_noise import correlated_field
from tools import fit_capture_noise as fc


def _frames(n, shape, z, k_long, width, k_short, seed=0, static=None):
    """Depth frames at constant depth z with a correlated and a per-pixel inverse-depth component."""
    rng = np.random.default_rng(seed)
    out = []
    for i in range(n):
        w = 1.0 / z + (static if static is not None else 0.0)
        w = w + k_long * correlated_field(shape, width, np.random.default_rng([seed, i])) + k_short * rng.normal(size=shape)
        out.append(1.0 / w)
    return np.stack(out)


def test_the_temporal_residual_removes_what_stays_and_keeps_what_changes():
    shape = (60, 80)
    static = np.random.default_rng(9).normal(size=shape) * 0.01   # a fixed pattern: removed
    stack = _frames(30, shape, 2.0, 0.0, (0.0, 0.0), 0.002, static=static)
    res = fc.temporal_frames(stack, 2.0)
    r = 1.0 / res - 1.0 / 2.0
    assert np.nanstd(r) == pytest.approx(0.002, rel=0.08)


def test_a_known_long_component_is_recovered():
    stack = _frames(3, (240, 320), 2.0, 0.005, (6.0, 4.0), 0.0006, seed=1)
    # inverse-depth sigma k*z^2 / z^2 = k: the components are in inverse depth, so the fitted k is
    # the inverse-depth amplitude itself (depth sigma = k z^2)
    fits = [fc.fit_frame(f, (1.0, 3.0)) for f in stack]
    long_k = np.median([f["long"][0] for f in fits])
    assert long_k == pytest.approx(0.005, rel=0.2)
    assert np.median([f["long"][1][0] for f in fits]) == pytest.approx(6.0, rel=0.35)


def test_the_entry_face_is_taken_from_the_data():
    """Codex 2026-10-17: the named distance can miss the face; the nearest surface is used, and a capture
    with too few face pixels is unverified."""
    depth = np.full((30, 40, 60), 2.0)
    depth[:, 10:30, 10:50] = 1.03                      # a face 30 mm beyond the named 1 m
    mask, face = fc.face_pixels(depth)
    assert face == pytest.approx(1.03, abs=0.005) and mask.sum() == 20 * 40
    assert fc.surface_check(depth + np.random.default_rng(0).normal(scale=0.001, size=depth.shape), min_pixels=900)["status"] == "unverified"
    assert fc.surface_check(depth + np.random.default_rng(0).normal(scale=0.001, size=depth.shape), min_pixels=500)["status"] == "measured"


def test_levels_are_per_resolution_with_widths_at_one_source_fx():
    fits = [
        {"label": "wall_2p00m_640", "surface": "wall", "distance_m": 2.0, "fx": 388.8, "mode": 640,
         "temporal": {"short": [0.0006, [0.3, 0.0]], "long": [0.005, [5.7, 3.9]]},
         "frame": {"short": [0.0008, [0.3, 0.0]], "long": [0.006, [6.2, 4.2]]}},
        {"label": "wall_3p00m_640", "surface": "wall", "distance_m": 3.0, "fx": 388.8, "mode": 640,
         "temporal": {"short": [0.0007, [0.5, 0.0]], "long": [0.006, [6.2, 4.2]]},
         "frame": {"short": [0.0008, [0.0, 0.0]], "long": [0.0062, [6.3, 4.3]]}},
        {"label": "wall_2p00m_848", "surface": "wall", "distance_m": 2.0, "fx": 429.3, "mode": 848,
         "temporal": {"short": [0.0006, [0.3, 0.0]], "long": [0.006, [6.0, 4.0]]},
         "frame": {"short": [0.0007, [0.0, 0.0]], "long": [0.0066, [6.3, 4.3]]}},
        {"label": "wall_1p00m_848", "surface": "wall", "distance_m": 1.0, "fx": 429.3, "mode": 848,
         "temporal": {"short": [0.0006, [0.5, 0.5]], "long": [0.0048, [5.2, 3.9]]},
         "frame": {"short": [0.0007, [0.5, 0.5]], "long": [0.0053, [5.2, 3.9]]}},
    ]
    levels = fc.build_levels(fits, source_fx=388.8, distances=(2.0, 3.0))
    assert set(levels) == {"temporal_640", "frame_640", "temporal_848", "frame_848"}
    long640 = levels["temporal_640"]["components"][1]
    assert long640["k"] == pytest.approx(0.0055) and long640["correlation_px_source"] == pytest.approx([5.95, 4.05])
    long848 = levels["temporal_848"]["components"][1]
    # one 848 capture in range (the 1 m one is left out); widths moved to the 640 fx
    assert long848["k"] == pytest.approx(0.006)
    assert long848["correlation_px_source"] == pytest.approx([6.0 * 388.8 / 429.3, 4.0 * 388.8 / 429.3])
    assert all(lv["k_iid"] == 0.0 for lv in levels.values())
