# tests/unit/test_instance_rig.py
import numpy as np

from tools import instance_rig, scene_rig


def _cube(x, y, size=0.3):
    return [scene_rig.Box((x, y, size / 2), (size, size, size))]


def test_two_touching_boxes_get_their_own_ids_and_order():
    out = instance_rig.render_instances([_cube(2.0, 0.15), _cube(2.0, -0.15)])
    assert {0, 1} <= set(np.unique(out.instance_ids))
    u0 = np.argwhere(out.instance_ids == 0)[:, 1].mean()
    u1 = np.argwhere(out.instance_ids == 1)[:, 1].mean()
    assert u0 < u1  # +y (left) appears left in the image


def test_depth_matches_scene_rig_without_quantisation():
    boxes = _cube(2.0, 0.0)
    ours = instance_rig.render_instances([boxes]).scene.depth_m
    theirs = scene_rig.render(boxes, camera=instance_rig.PERCEPTION_CAMERA, quantize=False).depth_m
    assert np.array_equal(np.isnan(ours), np.isnan(theirs))
    assert np.allclose(ours[~np.isnan(ours)], theirs[~np.isnan(theirs)])


def test_hit_points_lie_on_the_box_face():
    out = instance_rig.render_instances([_cube(2.0, 0.0)])
    pts = out.points_base[out.instance_ids == 0]
    assert np.allclose(pts[:, 0].min(), 2.0 - 0.15, atol=1e-9)


def test_distractors_are_marked_minus_two():
    assert (instance_rig.render_instances([], distractors=_cube(2.0, 0.0)).instance_ids == -2).any()
