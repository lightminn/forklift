"""Depth colouring for the robot-camera video, on CPU."""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "sim/isaac/video_frames.py"
spec = importlib.util.spec_from_file_location("video_frames_under_test", SCRIPT)
frames = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = frames
spec.loader.exec_module(frames)


def test_near_is_warm_far_is_cool_and_missing_depth_is_black():
    depth = np.array([[0.3, 10.0, np.inf, np.nan, 0.0]])
    rgb = frames.colorize_depth(depth, near_m=0.3, far_m=10.0)
    assert rgb.shape == (1, 5, 3) and rgb.dtype == np.uint8
    near, far = rgb[0, 0].astype(int), rgb[0, 1].astype(int)
    assert near[0] > near[2]  # red over blue up close
    assert far[2] > far[0]  # blue over red far away
    np.testing.assert_array_equal(rgb[0, 2:], 0)


def test_depth_beyond_the_range_is_clipped_not_dropped():
    rgb = frames.colorize_depth(np.array([[0.1, 50.0]]), near_m=0.3, far_m=10.0)
    np.testing.assert_array_equal(
        rgb[0], frames.colorize_depth(np.array([[0.3, 10.0]]), 0.3, 10.0)[0]
    )


@pytest.mark.parametrize("near, far", [(0.0, 10.0), (5.0, 5.0), (np.nan, 1.0)])
def test_invalid_range_raises(near, far):
    with pytest.raises(ValueError):
        frames.colorize_depth(np.ones((2, 2)), near_m=near, far_m=far)


def test_detection_outlines_are_drawn_on_a_copy_with_a_label():
    rgb = np.zeros((480, 640, 3), np.uint8)
    outlines = [
        [(100, 300), (160, 300), (160, 330), (100, 330)],
        [(300, 300), (360, 300), (360, 330), (300, 330)],
    ]
    out = frames.annotate_detection(rgb, outlines, "팔레트 1.20 m", font_path=None)
    assert out.shape == rgb.shape and out is not rgb
    assert rgb.sum() == 0
    amber = np.all(out == frames.POCKET_RGB, axis=-1)
    assert amber[300, 120] and amber[330, 330]
    green = np.all(out == frames.PALLET_RGB, axis=-1)
    assert green.any()


def test_no_outlines_leaves_the_frame_unchanged():
    rgb = np.full((48, 64, 3), 7, np.uint8)
    np.testing.assert_array_equal(frames.annotate_detection(rgb, [], "x"), rgb)
