"""URDF box clearances using actual full body poses, without an Isaac SDK."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "insertion_geometry", ROOT / "sim/isaac/insertion_geometry.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.fixture
def geometry():
    return MODULE.InsertionGeometry.from_urdfs(
        ROOT / "sim/models/dls08_provisional/forklift.urdf",
        ROOT / "sim/models/epal6_pallet/pallet.urdf",
    )


def check(geometry, base=(-0.89, 0, 0), q=(1, 0, 0, 0), lift=0, **kwargs):
    return geometry.forbidden_contacts(base, q, lift, (0, 0, 0), (1, 0, 0, 0), **kwargs)


def test_aligned_insertion_clears_all_source_boxes(geometry):
    assert len(geometry.pallet_boxes) == 22
    assert len(geometry.fork_boxes) == 4
    for x in np.linspace(-1.35, -0.89, 20):
        assert not check(geometry, base=(x, 0, 0))


def test_side_scrape_is_detected_without_any_pallet_displacement(geometry):
    assert check(geometry, base=(-0.89, 0.05, 0))


def test_lift_into_top_boards_is_forbidden_before_lift_phase(geometry):
    assert check(geometry, lift=0.055)


def test_pitch_and_roll_affect_clearance(geometry):
    angle = 0.12
    assert check(geometry, q=(np.cos(angle / 2), 0, -np.sin(angle / 2), 0))
    assert not check(geometry, q=(np.cos(angle / 2), np.sin(angle / 2), 0, 0))
    assert check(geometry, q=(np.cos(angle / 2), np.sin(angle / 2), 0, 0), lift=0.04)


def test_relative_yaw_can_scrape_stationary_pallet(geometry):
    for angle, expected in [(0.04, False), (0.10, True)]:
        assert (
            bool(check(geometry, q=(np.cos(angle / 2), 0, 0, np.sin(angle / 2))))
            == expected
        )


def test_exact_touch_is_rejected_at_zero_margin(geometry):
    assert check(geometry, base=(-0.89, 0.045, 0), clearance_m=0)


def test_common_world_rotation_and_translation_preserve_result(geometry):
    q = np.array([0.9, 0.1, 0.2, 0.3])
    q /= np.linalg.norm(q)
    from forklift_core.geometry import rotation_matrix_from_quaternion_xyzw

    rotation = rotation_matrix_from_quaternion_xyzw(q[[1, 2, 3, 0]])
    translation = np.array([2, -3, 1])
    for offset in [0, 0.05]:
        result = geometry.forbidden_contacts(
            rotation @ [-0.89, offset, 0] + translation,
            q,
            0,
            translation,
            q,
        )
        assert result == check(geometry, base=(-0.89, offset, 0))


def test_clearance_margin_rejects_near_scrape(geometry):
    # Inner fork edge is at -73 mm; centre block ends at -72.5 mm.
    assert not check(geometry, base=(-0.89, 0.0445, 0), clearance_m=0)
    assert check(geometry, base=(-0.89, 0.0445, 0), clearance_m=0.002)


@pytest.mark.parametrize("kwargs", [{"clearance_m": -1}, {"lift": np.nan}])
def test_invalid_geometry_state_is_rejected(geometry, kwargs):
    with pytest.raises(ValueError):
        check(geometry, **kwargs)
