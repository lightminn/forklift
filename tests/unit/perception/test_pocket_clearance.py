"""Pocket interior free check (priority-5 plan D5) on synthetic depth images."""

import numpy as np

from forklift_core.perception.pocket_clearance import (
    DepthCamera,
    box_voxels,
    check_pocket_clearance,
)

CAM = DepthCamera(fx=430.0, fy=430.0, cx=424.0, cy=240.0, width=848, height=480)
# Optical frame: z forward, x right, y down. A tunnel (pocket) 2.0-2.5 m ahead,
# 0.2 m below the camera; the voxels are a thin fork volume inside it.
VOX = box_voxels((0.0, 0.2, 2.25), (0.02, 0.01, 0.2))


def render(planes):
    """Depth of the nearest of several fronto-parallel rectangles (x0, x1, y0, y1, z)."""
    v, u = np.mgrid[0 : CAM.height, 0 : CAM.width]
    rx = (u - CAM.cx) / CAM.fx
    ry = (v - CAM.cy) / CAM.fy
    depth = np.full((CAM.height, CAM.width), np.inf)
    for x0, x1, y0, y1, z in planes:
        x, y = rx * z, ry * z
        hit = (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
        depth = np.where(hit & (z < depth), z, depth)
    return depth


BACK = (-5, 5, -5, 5, 6.0)  # a far wall behind the pallet


def test_an_empty_pocket_is_accepted():
    r = check_pocket_clearance(render([BACK]), CAM, VOX, [])
    assert r.accepted and r.free == r.voxels


def test_a_bar_inside_the_pocket_is_an_obstacle():
    bar = (-0.01, 0.01, 0.15, 0.25, 2.2)  # a 2 cm bar across the fork path
    r = check_pocket_clearance(render([BACK, bar]), CAM, VOX, [])
    assert not r.accepted and r.reason == "obstacle" and r.obstacle > 0


def test_the_pallet_front_hiding_voxels_is_unobserved_not_free():
    face = (-0.5, 0.5, 0.18, 0.22, 2.0)  # a pallet board at the face covering the voxels' rays
    board = [((0.0, 0.2, 2.0), (0.5, 0.02, 0.01), np.eye(3))]
    r = check_pocket_clearance(render([BACK, face]), CAM, VOX, board)
    assert not r.accepted and r.reason == "unobserved" and r.occluded > 0 and r.obstacle == 0


def test_missing_pixels_reject():
    depth = render([BACK])
    depth[:, :] = np.nan
    r = check_pocket_clearance(depth, CAM, VOX, [])
    assert not r.accepted and r.reason == "unobserved"
