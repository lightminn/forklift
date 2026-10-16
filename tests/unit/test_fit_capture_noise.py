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


def test_each_level_is_checked_against_its_own_kind_of_variance():
    """Codex review 2026-10-17: the frame levels were compared with the temporal variances, which hid a
    one-third under-prediction. The reproduction check takes the variances of the level's own kind."""
    steps = list(fc.fit.STEPS)
    row = {"distance_m": 2.0, "shape": [120, 160], "fx": 388.8,
           "var": {"temporal": {"u": {s: 1.0 for s in steps}, "v": {s: 1.0 for s in steps}},
                   "frame": {"u": {s: 2.0 for s in steps}, "v": {s: 2.0 for s in steps}}}}
    level = {"components": [{"k": 0.005, "correlation_px_source": [6.0, 4.0]}]}
    t = fc.reproduction(level, row, kind="temporal", source_fx=388.8, frames=1)
    f = fc.reproduction(level, row, kind="frame", source_fx=388.8, frames=1)
    assert np.allclose(np.array(t["ratio_u"]) / np.array(f["ratio_u"]), 2.0)


def _capture(folder, depth_mm, roi):
    """A capture folder as tools/capture_d435i_noise.py writes it (two frames)."""
    import json
    from PIL import Image

    folder.mkdir(parents=True)
    frames = []
    for i in range(2):
        name = f"z16_{i:06d}.png"
        Image.fromarray(depth_mm.astype(np.uint16)).save(folder / name)
        frames.append({"file": name, "number": i, "timestamp_ms": float(i), "domain": "global"})
    meta = {"device": {"depth_scale_m": 0.001, "stream": {"width": depth_mm.shape[1], "height": depth_mm.shape[0]},
                       "intrinsics": {"fx": 100.0}},
            "setup": {"label": folder.name, "distance_m": 1.0, "surface": "wall", "roi": roi}, "frames": frames}
    (folder / "capture.json").write_text(json.dumps(meta))


def test_a_capture_without_a_roi_uses_the_whole_frame(tmp_path):
    """Codex review 2026-10-17: the capture tool stores roi null when --roi is left out."""
    _capture(tmp_path / "c", np.full((20, 30), 1000), None)
    stack, _ = fc.load_capture(tmp_path / "c")
    assert stack.shape == (2, 20, 30)


def test_the_input_hash_covers_the_frames(tmp_path):
    """Codex review 2026-10-17: hashing capture.json alone missed a replaced depth frame."""
    from PIL import Image

    _capture(tmp_path / "c", np.full((20, 30), 1000), [0, 0, 30, 20])
    before = fc._files_sha256([tmp_path / "c"])
    Image.fromarray(np.full((20, 30), 1001, np.uint16)).save(tmp_path / "c" / "z16_000001.png")
    assert fc._files_sha256([tmp_path / "c"]) != before
