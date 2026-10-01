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
    moved = max(abs(a.centre_m[1] - b.centre_m[1]) for a, b in zip(placed, unrotated))
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


MEASURED_URDF = ROOT / "sim/models/dls08_measured/forklift.urdf"


def test_truck_boxes_are_the_carriage_collision_boxes_of_the_model():
    from tools.scene_rig import truck_boxes

    boxes = truck_boxes(MEASURED_URDF)
    assert len(boxes) == 14
    blades = [b for b in boxes if b.size_m[0] > 0.3]
    assert len(blades) == 2
    for blade in blades:
        x0 = blade.centre_m[0] - blade.size_m[0] / 2
        x1 = blade.centre_m[0] + blade.size_m[0] / 2
        z0 = blade.centre_m[2] - blade.size_m[2] / 2
        assert (x0, x1, z0) == pytest.approx((0.59, 0.95, 0.0275))
        assert abs(blade.centre_m[1]) == pytest.approx(0.145)
    front = max(b.centre_m[0] + b.size_m[0] / 2 for b in boxes if b.size_m[0] < 0.1)
    assert front == pytest.approx(0.604)


def test_lift_raises_the_truck_boxes_only_in_z():
    from tools.scene_rig import truck_boxes

    low, high = truck_boxes(MEASURED_URDF), truck_boxes(MEASURED_URDF, lift_m=0.2)
    for a, b in zip(low, high, strict=True):
        assert b.centre_m[0] == a.centre_m[0] and b.centre_m[1] == a.centre_m[1]
        assert b.centre_m[2] == pytest.approx(a.centre_m[2] + 0.2)
        assert b.size_m == a.size_m


def test_min_range_masks_only_near_pixels_and_defaults_to_unchanged():
    from tools.scene_rig import Camera, truck_boxes

    camera = Camera((0.61, 0.0, 0.15))
    boxes = truck_boxes(MEASURED_URDF)
    plain = np.asarray(render(boxes, camera=camera).depth_m)
    same = np.asarray(render(boxes, camera=camera, min_range_m=None).depth_m)
    np.testing.assert_array_equal(
        np.nan_to_num(plain, nan=-1), np.nan_to_num(same, nan=-1)
    )
    masked = np.asarray(render(boxes, camera=camera, min_range_m=0.5).depth_m)
    near = np.isfinite(plain) & (plain < 0.5)
    assert near.any()
    assert np.isnan(masked[near]).all()
    far = np.isfinite(plain) & (plain >= 0.5)
    np.testing.assert_array_equal(masked[far], plain[far])


def test_min_range_does_not_reveal_what_a_near_object_hides():
    from tools.scene_rig import Camera

    camera = Camera((0.0, 0.0, 0.5))
    near = Box((0.3, 0.0, 0.5), (0.05, 2.0, 2.0))  # a wall 0.275 m ahead
    far = Box((2.0, 0.0, 0.5), (0.1, 2.0, 2.0))
    depth = np.asarray(render([near, far], camera=camera, min_range_m=0.3).depth_m)
    centre = depth[240, 320]
    assert np.isnan(centre)  # masked, not replaced by the far box behind it


def test_first_hit_names_the_object_and_distance():
    from tools.scene_rig import first_hit

    boxes = [
        Box((1.0, 0.0, 0.5), (0.2, 0.2, 0.2)),
        Box((2.0, 0.0, 0.5), (0.2, 0.2, 0.2)),
    ]
    kind, index, t = first_hit(boxes, (0.0, 0.0, 0.5), (2.0, 0.0, 0.5))
    assert (kind, index) == ("box", 0) and t == pytest.approx(0.45)
    kind, index, t = first_hit(boxes, (0.0, 0.0, 0.5), (1.0, 0.0, -0.5))
    assert kind == "floor" and t == pytest.approx(0.5)


def test_interpenetration_ignores_touching_faces():
    from tools.scene_rig import boxes_interpenetrate

    a = Box((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
    assert not boxes_interpenetrate(a, Box((1.0, 0.0, 0.0), (1.0, 1.0, 1.0)))
    assert boxes_interpenetrate(a, Box((0.99, 0.0, 0.0), (1.0, 1.0, 1.0)))
    assert not boxes_interpenetrate(a, Box((0.0, 0.0, 1.0), (1.0, 1.0, 1.0)))
    assert boxes_interpenetrate(a, Box((1.1, 0.0, 0.0), (1.0, 1.0, 1.0), math.pi / 4))
