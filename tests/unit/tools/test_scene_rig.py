"""The measurement rig must not drift from the frozen test fixture.

An earlier scratch rig rotated each slab in place without moving its centre.
That is not a rigid rotation, and it changed a 60-pose result from 16 to 1
before anyone noticed. These tests pin the two properties that mattered: the
rig renders exactly what the fixture renders, and its rotation is rigid.
"""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception.pallet_geometry import load_pallet_geometry
from tools.scene_rig import Box, pallet, place, render, true_pockets

ROOT = Path(__file__).resolve().parents[3]
_SPEC = importlib.util.spec_from_file_location(
    "rig_reference_scene", ROOT / "tests/fixtures/synthetic_scene.py"
)
_FIXTURE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_FIXTURE)

GEOMETRY = load_pallet_geometry(ROOT / "config/pallet_geometry_t11_06.yaml")


def _fixture_depth(boxes):
    scene, _ = _FIXTURE.make_pallet_scene(pallet=False)
    camera = scene.base_from_optical.translation_m
    rays = _FIXTURE._rays(scene.intrinsics, scene.base_from_optical.rotation)
    depth = np.asarray(scene.depth_m, dtype=float).copy()
    for box in boxes:
        hit = _FIXTURE._box_depth(
            camera, rays, np.asarray(box.centre_m), box.size_m, np.eye(3)
        )
        depth = np.fmin(depth, hit)
    depth[~np.isfinite(depth)] = np.nan
    return np.round(depth / 0.001) * 0.001


def test_empty_scene_matches_the_fixture_exactly():
    scene, _ = _FIXTURE.make_pallet_scene(pallet=False)
    mine = render([], quantize=False)
    assert np.array_equal(
        np.asarray(scene.depth_m, dtype=float), np.asarray(mine.depth_m, dtype=float)
    )


@pytest.mark.parametrize("x_m", [2.4, 3.0, 4.2])
def test_axis_aligned_pallet_matches_the_fixture_exactly(x_m):
    boxes = place(pallet(GEOMETRY), x_m=x_m)
    mine = np.asarray(render(boxes).depth_m, dtype=float)
    reference = _fixture_depth(boxes)
    assert np.array_equal(np.isnan(mine), np.isnan(reference))
    finite = ~np.isnan(mine)
    assert np.array_equal(mine[finite], reference[finite])


def test_place_rotates_centres_not_only_boxes():
    """A rigid rotation moves the assembly; rotating slabs in place does not."""
    boxes = pallet(GEOMETRY)
    yaw = 0.35
    placed = place(boxes, x_m=3.0, yaw_rad=yaw)
    # Every box carries the yaw ...
    assert all(math.isclose(b.yaw_rad, yaw) for b in placed)
    # ... and the centres have actually moved off the unrotated positions.
    unrotated = place(boxes, x_m=3.0)
    moved = max(
        abs(a.centre_m[1] - b.centre_m[1]) for a, b in zip(placed, unrotated)
    )
    assert moved > 0.05, "centres did not rotate; the rotation is not rigid"


def test_yawed_render_differs_from_rotating_slabs_in_place():
    boxes = pallet(GEOMETRY)
    yaw = 0.35
    rigid = np.asarray(render(place(boxes, x_m=3.0, yaw_rad=yaw)).depth_m, dtype=float)
    in_place = np.asarray(
        render(
            [
                Box((3.0 + b.centre_m[0], b.centre_m[1], b.centre_m[2]), b.size_m, yaw)
                for b in boxes
            ]
        ).depth_m,
        dtype=float,
    )
    both = np.isfinite(rigid) & np.isfinite(in_place)
    assert np.nanmax(np.abs(rigid[both] - in_place[both])) > 0.1


def test_true_pockets_sit_on_the_approach_face():
    pockets = true_pockets(GEOMETRY, x_m=3.0)
    assert pockets.shape == (2, 3)
    # Half the depth in front of the placement centre, not at it.
    assert np.allclose(pockets[:, 0], 3.0 - GEOMETRY.overall_depth_m / 2)
    assert np.allclose(sorted(pockets[:, 1]), [-0.145, 0.145])
    assert np.allclose(pockets[:, 2], 0.0375)
