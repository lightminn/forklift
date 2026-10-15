# tests/unit/test_capture_d435i_noise.py
"""The parts of the D435i noise capture that run without a camera: frame checks and the saved record."""
import json

import numpy as np
import pytest
from PIL import Image

from tools import capture_d435i_noise as cap

SETUP = {"label": "wall_2m", "distance_m": 2.003, "distance_method": "laser rangefinder", "surface": "wall",
         "reference_flat": False, "roi": [200, 150, 440, 330], "notes": ""}
DEVICE = {"name": "Intel RealSense D435I", "serial_sha256": "ab", "firmware": "5.16", "sdk": "2.55",
          "depth_scale_m": 0.001, "stream": {"width": 640, "height": 480, "fps": 30, "format": "z16"},
          "intrinsics": {"fx": 385.0, "fy": 385.0, "ppx": 320.0, "ppy": 240.0, "model": "brown_conrady",
                         "coeffs": [0, 0, 0, 0, 0]},
          "options": {"visual_preset": 1, "enable_auto_exposure": 0, "exposure": 8500, "gain": 16,
                      "emitter_enabled": 1, "laser_power": 150}, "aligned_to_color": False,
          "post_processing": []}


def _frames(n=3):
    return [cap.Frame(np.full((480, 640), 2003 + i, np.uint16), 100 + i, 1000.0 + 33.3 * i, "global_time")
            for i in range(n)]


def test_the_record_keeps_raw_z16_and_every_setting(tmp_path):
    cap.write_capture(tmp_path / "wall_2m", _frames(), DEVICE, SETUP, warmup_s=300.0)
    meta = json.loads((tmp_path / "wall_2m" / "capture.json").read_text())
    assert meta["device"]["depth_scale_m"] == 0.001 and meta["setup"]["distance_m"] == 2.003
    assert [f["number"] for f in meta["frames"]] == [100, 101, 102] and meta["warmup_s"] == 300.0
    raw = np.asarray(Image.open(tmp_path / "wall_2m" / "z16_000000.png"))
    assert raw.dtype == np.uint16 and raw[0, 0] == 2003


def test_dropped_or_repeated_frames_are_reported():
    frames = _frames(4)
    frames[2] = frames[1]
    report = cap.frame_report(frames)
    assert report["repeated"] == 1 and report["gaps"] == [{"after": 101, "missing": 1}]


@pytest.mark.parametrize("change", [{"distance_m": None}, {"label": ""}, {"roi": [10, 10, 5, 20]}])
def test_the_setup_must_say_what_was_measured(change):
    with pytest.raises(ValueError):
        cap.check_setup({**SETUP, **change}, width=640, height=480)


def test_post_processing_must_be_off():
    with pytest.raises(ValueError, match="post-processing"):
        cap.write_capture("unused", _frames(), {**DEVICE, "post_processing": ["spatial"]}, SETUP, warmup_s=300.0)
