"""Uncertainty-weighted fusion of odometry increments (plan 2026-10-07 D3)."""

import math

import pytest

from forklift_core.localization.odometry_fusion import (
    FusedOdometry,
    IncrementNoise,
    fuse,
    increment,
)
from forklift_core.localization.slam_pose import compose

TIGHT = IncrementNoise(0.002, 0.0005, 0.002, 0.0005, 0.0002)
LOOSE = IncrementNoise(0.05, 0.005, 0.08, 0.002, 0.002)


def test_increment_is_the_motion_in_the_previous_frame():
    a = (1.0, 2.0, math.pi / 2)
    b = compose(a, (0.3, -0.1, 0.2))
    assert increment(a, b) == pytest.approx((0.3, -0.1, 0.2))


def test_one_source_passes_through_and_equal_sources_average():
    assert fuse({"a": (0.1, 0.0, 0.01)}, {"a": LOOSE}) == pytest.approx(
        (0.1, 0.0, 0.01)
    )
    out = fuse({"a": (0.10, 0.0, 0.0), "b": (0.12, 0.0, 0.0)}, {"a": LOOSE, "b": LOOSE})
    assert out[0] == pytest.approx(0.11, rel=0.01)


def test_the_tighter_source_dominates_per_component():
    noise = {"wheel": LOOSE, "lidar": TIGHT}
    out = fuse({"wheel": (0.105, 0.0, 0.020), "lidar": (0.100, 0.0, 0.010)}, noise)
    assert abs(out[0] - 0.100) < 0.0005 and abs(out[2] - 0.010) < 0.0005


def test_a_lost_source_drops_out_and_the_rest_carry_the_pose():
    fused = FusedOdometry({"wheel": LOOSE, "vision": TIGHT})
    fused.update({"wheel": (0.0, 0.0, 0.0), "vision": (5.0, 5.0, 1.0)})
    # Vision lost: only the wheel increment moves the pose.
    pose = fused.update({"wheel": (0.1, 0.0, 0.0), "vision": None})
    assert pose == pytest.approx((0.1, 0.0, 0.0)) and fused.last_sources == ("wheel",)
    # Vision back: no increment until it has two consecutive frames.
    fused.update({"wheel": (0.2, 0.0, 0.0), "vision": (9.0, 9.0, 0.0)})
    assert fused.last_sources == ("wheel",)
    pose = fused.update({"wheel": (0.30, 0.0, 0.0), "vision": (9.098, 9.0, 0.0)})
    assert fused.last_sources == ("vision", "wheel")
    assert pose[0] == pytest.approx(0.298, abs=0.0005)


def test_nothing_reported_keeps_the_pose_and_bad_models_are_refused():
    fused = FusedOdometry({"vision": TIGHT}, initial_pose=(1.0, 2.0, 0.3))
    assert fused.update({"vision": None}) == (1.0, 2.0, 0.3)
    with pytest.raises(ValueError, match="no noise model"):
        fused.update({"lidar": (0, 0, 0)})
    with pytest.raises(ValueError, match="floors"):
        IncrementNoise(0.1, 0.0, 0.1, 0.1, 0.1)
