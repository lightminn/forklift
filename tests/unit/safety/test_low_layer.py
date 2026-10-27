"""The near-field low layer (docs/plans/2026-10-27-autonomous-survey-and-relocalization.md section S): cells seen empty by
valid depth reaching a verified floor, cells with a point in the collision band, everything else unobserved; evidence
aged out and eroded by the odometry error since it was seen; the swept memory; the envelope admissibility of a path."""

import math

import numpy as np
import pytest

from forklift_core.planning.geometry import Footprint
from forklift_core.safety import low_layer as L

FOOT = Footprint(1.29, 0.17, 0.36)  # the planner's unloaded footprint: forks to the tip, rear overhang, half width
CFG = L.LowLayerConfig()


def _floor_points(x0, x1, y0, y1, step=0.02, z=0.0):
    xs, ys = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    return np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, z)], 1)


def _box_points(x0, x1, y0, y1, height, step=0.02):
    """The faces of a box standing on the floor, as a camera in front of it would see them (top and the -x face)."""
    top = _floor_points(x0, x1, y0, y1, step, z=height)
    ys, zs = np.meshgrid(np.arange(y0, y1, step), np.arange(0.0, height, step))
    face = np.stack([np.full(ys.size, x0), ys.ravel(), zs.ravel()], 1)
    return np.vstack([top, face])


def _layer():
    return L.LowLayer(CFG, origin_xy=(-2.0, -3.0), size_cells=(160, 120))


def test_the_floor_plane_is_accepted_and_a_raised_plane_is_not():
    pts = _floor_points(1.4, 3.0, -0.8, 0.8)
    plane = L.fit_floor(pts, CFG)
    assert plane.ok and abs(plane.offset_m) < 1e-6 and plane.normal[2] > 0.999
    # only a pallet top in view (0.144 m): a horizontal plane, but not where the floor is expected -- refused
    top = _floor_points(1.4, 2.0, -0.4, 0.4, z=0.144)
    assert not L.fit_floor(top, CFG).ok
    # too few points to trust
    assert not L.fit_floor(pts[:50], CFG).ok


def test_floor_hits_are_free_band_points_occupied_and_the_rest_unobserved():
    layer = _layer()
    pts = np.vstack([_floor_points(1.4, 3.0, -0.8, 0.8), _box_points(2.0, 2.6, 0.2, 1.0, 0.144)])
    assert layer.integrate(pts, t=0.0).ok
    assert layer.state_at(1.6, 0.0) == L.FREE
    assert layer.state_at(2.3, 0.6) == L.OCCUPIED  # the box top is in the band
    assert layer.state_at(0.5, 0.0) == L.UNOBSERVED  # never seen
    assert layer.state_at(4.0, 0.0) == L.UNOBSERVED


def test_a_frame_whose_floor_plane_fails_marks_nothing():
    layer = _layer()
    result = layer.integrate(_floor_points(1.4, 2.0, -0.4, 0.4, z=0.144), t=0.0)
    assert not result.ok and layer.state_at(1.6, 0.0) == L.UNOBSERVED


def test_the_newest_evidence_wins():
    layer = _layer()
    layer.integrate(np.vstack([_floor_points(1.4, 3.0, -0.8, 0.8), _box_points(2.0, 2.6, -0.3, 0.3, 0.2)]), t=0.0)
    assert layer.state_at(2.3, 0.0) == L.OCCUPIED
    layer.integrate(_floor_points(1.4, 3.0, -0.8, 0.8), t=0.5)  # the box is gone
    assert layer.state_at(2.3, 0.0) == L.FREE
    layer.integrate(np.vstack([_floor_points(1.4, 3.0, -0.8, 0.8), _box_points(2.0, 2.6, -0.3, 0.3, 0.2)]), t=1.0)
    assert layer.state_at(2.3, 0.0) == L.OCCUPIED


def test_evidence_ages_out_and_is_eroded_by_the_odometry_error_since_it_was_seen():
    layer = _layer()
    layer.integrate(_floor_points(1.4, 3.0, -0.8, 0.8), t=0.0)
    here = (0.0, 0.0, 0.0)
    usable_now = layer.usable(t=0.0, rear_pose=here)
    usable_later = layer.usable(t=2.9, rear_pose=here)
    assert usable_now[layer.cell(2.0, 0.0)] and usable_later[layer.cell(2.0, 0.0)]
    # near the seen region's edge the older evidence is eroded further than the fresh one: the age table's position
    # error and its heading error times the distance from the rear axle (the known-pallet growth's form)
    edge = layer.cell(1.40 + CFG.resolution_m * 1.5, 0.0)
    assert usable_now[edge] and not usable_later[edge]
    assert L.erosion_m(CFG, 0.0) < L.erosion_m(CFG, 2.0) <= L.erosion_m(CFG, CFG.max_age_s)
    assert L.erosion_m(CFG, 2.0, rho_m=2.0) > L.erosion_m(CFG, 2.0)
    # beyond the age limit nothing is usable
    assert not layer.usable(t=CFG.max_age_s + 0.1, rear_pose=here)[layer.cell(2.0, 0.0)]


def test_the_trucks_own_footprint_is_swept_and_forward_may_use_swept_or_free_cells_reverse_only_swept():
    layer = _layer()
    start = (0.0, 0.0, 0.0)
    layer.mark_swept(FOOT, start, t=0.0)
    layer.integrate(_floor_points(1.25, 3.0, -0.8, 0.8), t=0.0)
    ahead = L.path_poses([start], [(1.0, 0.0)])  # 1 m straight
    ok, bad = L.envelope_admissible(layer, t=0.0, poses=ahead, footprint=FOOT, reverse=False)
    assert ok, bad
    back = L.path_poses([start], [(-0.5, 0.0)])
    ok, bad = L.envelope_admissible(layer, t=0.0, poses=back, footprint=FOOT, reverse=True)
    assert not ok and bad  # the rear overhang's cells behind the start were never swept
    # a swept band exactly the body's width is eroded at its edges: retracing it needs a margin (drift); a wider
    # swept region (as if the truck had passed with room either side) admits it
    layer.mark_swept(Footprint(1.4, 0.45, 0.5), (-0.6, 0.0, 0.0), t=0.0)
    ok, bad = L.envelope_admissible(layer, t=0.0, poses=back, footprint=FOOT, reverse=True)
    assert ok, bad
    # forward through free floor is not enough for reverse: reverse never uses camera-free cells
    fwd_as_reverse = L.path_poses([start], [(1.0, 0.0)])
    ok, _ = L.envelope_admissible(layer, t=0.0, poses=fwd_as_reverse, footprint=FOOT, reverse=True)
    assert not ok


def test_an_unobserved_or_occupied_cell_in_the_envelope_refuses_it():
    layer = _layer()
    layer.mark_swept(FOOT, (0.0, 0.0, 0.0), t=0.0)
    layer.integrate(np.vstack([_floor_points(1.25, 3.0, -0.8, 0.8), _box_points(2.0, 2.3, -0.2, 0.2, 0.144)]), t=0.0)
    ok, bad = L.envelope_admissible(layer, t=0.0, poses=L.path_poses([(0.0, 0.0, 0.0)], [(1.0, 0.0)]), footprint=FOOT,
                                    reverse=False)
    assert not ok and any(1.9 <= x <= 2.4 for x, _ in bad)
    # a left turn sweeps beside the body, where the camera never looked
    layer2 = _layer()
    layer2.mark_swept(FOOT, (0.0, 0.0, 0.0), t=0.0)
    layer2.integrate(_floor_points(1.25, 3.0, -0.3, 0.3), t=0.0)
    turn = L.path_poses([(0.0, 0.0, 0.0)], [(0.6, 0.29)])
    ok, bad = L.envelope_admissible(layer2, t=0.0, poses=turn, footprint=FOOT, reverse=False)
    assert not ok


def test_path_poses_follow_arcs_at_the_given_curvature():
    poses = L.path_poses([(0.0, 0.0, 0.0)], [(math.pi / 2 / 0.29, 0.29)], step_m=0.01)
    x, y, yaw = poses[-1]
    r = 1 / 0.29
    assert abs(yaw - math.pi / 2) < 1e-6 and abs(x - r) < 1e-3 and abs(y - r) < 1e-3
    back = L.path_poses([(0.0, 0.0, 0.0)], [(-1.0, 0.0)], step_m=0.1)
    assert back[-1][0] == pytest.approx(-1.0) and back[-1][2] == pytest.approx(0.0)
