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
CAM = (0.959, 0.0, 0.27)  # carriage_low_measured in the rear-axle frame of a truck at the origin


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
    pts = _floor_points(1.4, 5.0, -0.8, 0.8)
    plane = L.fit_floor(pts, CFG)
    assert plane.ok and abs(plane.offset_m) < 1e-6 and plane.normal[2] > 0.999
    # only a pallet top in view (0.144 m): a horizontal plane, but not where the floor is expected -- refused
    top = _floor_points(1.4, 2.0, -0.4, 0.4, z=0.144)
    assert not L.fit_floor(top, CFG).ok
    # too few points to trust
    assert not L.fit_floor(pts[:50], CFG).ok
    # S-P1 result review (N4): the foot of a box face seen without any floor -- a band of points 0.049-0.050 m up along
    # the face at x 1.30 -- is a line, not a plane, however many points it has
    ys, zs = np.meshgrid(np.arange(-0.4, 0.4, 0.0005), np.linspace(0.049, 0.0499, 2))
    band = np.stack([np.full(ys.size, 1.30), ys.ravel(), zs.ravel()], 1)
    plane = L.fit_floor(band, CFG)
    assert len(band) >= CFG.plane_min_points and not plane.ok and plane.reason == "not_spread"


def test_floor_hits_are_free_band_points_occupied_and_the_rest_unobserved():
    layer = _layer()
    pts = np.vstack([_floor_points(1.4, 5.0, -0.8, 0.8), _box_points(2.0, 2.6, 0.2, 1.0, 0.144)])
    assert layer.integrate(pts, t=0.0, origin=CAM).ok
    assert layer.state_at(1.6, 0.0) == L.FREE
    assert layer.state_at(2.3, 0.6) == L.OCCUPIED  # the box top is in the band
    assert layer.state_at(0.5, 0.0) == L.UNOBSERVED  # never seen
    assert layer.state_at(4.0, 0.0) == L.UNOBSERVED


def test_a_frame_whose_floor_plane_fails_marks_nothing():
    layer = _layer()
    result = layer.integrate(_floor_points(1.4, 2.0, -0.4, 0.4, z=0.144), t=0.0, origin=CAM)
    assert not result.ok and layer.state_at(1.6, 0.0) == L.UNOBSERVED


def test_the_newest_evidence_wins():
    layer = _layer()
    layer.integrate(np.vstack([_floor_points(1.4, 5.0, -0.8, 0.8), _box_points(2.0, 2.6, -0.3, 0.3, 0.2)]), t=0.0, origin=CAM)
    assert layer.state_at(2.3, 0.0) == L.OCCUPIED
    layer.integrate(_floor_points(1.4, 5.0, -0.8, 0.8), t=0.5, origin=CAM)  # the box is gone
    assert layer.state_at(2.3, 0.0) == L.FREE
    layer.integrate(np.vstack([_floor_points(1.4, 5.0, -0.8, 0.8), _box_points(2.0, 2.6, -0.3, 0.3, 0.2)]), t=1.0, origin=CAM)
    assert layer.state_at(2.3, 0.0) == L.OCCUPIED


def test_evidence_ages_out_and_is_eroded_by_the_odometry_error_since_it_was_seen():
    layer = _layer()
    layer.integrate(_floor_points(1.4, 5.0, -0.8, 0.8), t=0.0, origin=CAM)
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


WIDE = dict(step=0.04)  # floor patches wide enough that rays beside the forks run far before they end


def test_forward_may_use_swept_or_free_evidence_reverse_only_swept():
    layer = _layer()
    start = (0.0, 0.0, 0.0)
    layer.mark_swept(FOOT, start, t=0.0)
    layer.integrate(_floor_points(0.6, 5.0, -3.0, 3.0, **WIDE), t=0.0, origin=CAM)
    ahead = L.path_poses([start], [(1.0, 0.0)])  # 1 m straight
    ok, bad = L.envelope_admissible(layer, t=0.0, poses=ahead, footprint=FOOT, reverse=False)
    assert ok, bad
    back = L.path_poses([start], [(-0.5, 0.0)])
    ok, bad = L.envelope_admissible(layer, t=0.0, poses=back, footprint=FOOT, reverse=True)
    assert not ok and bad  # behind the rear overhang the truck has never been
    layer.mark_swept(Footprint(1.4, 0.45, 0.5), (-0.6, 0.0, 0.0), t=0.0)  # as if it had passed there with room
    ok, bad = L.envelope_admissible(layer, t=0.0, poses=back, footprint=FOOT, reverse=True)
    assert ok, bad
    # forward through free floor is not enough for reverse: reverse never uses camera-free cells
    ok, _ = L.envelope_admissible(layer, t=0.0, poses=ahead, footprint=FOOT, reverse=True)
    assert not ok


def test_a_swept_area_shrinks_with_the_odometry_error_and_a_newer_band_point_voids_it():
    layer = _layer()
    layer.mark_swept(Footprint(1.4, 0.45, 0.5), (-0.6, 0.0, 0.0), t=0.0)
    back = L.path_poses([(0.0, 0.0, 0.0)], [(-0.5, 0.0)])
    assert L.envelope_admissible(layer, t=0.0, poses=back, footprint=FOOT, reverse=True)[0]
    # 0.6 s later the rectangle has shrunk by the error since (0.025-0.03 m plus the heading term): still enough room
    assert L.envelope_admissible(layer, t=0.6, poses=back, footprint=FOOT, reverse=True)[0]
    # an exactly body-wide swept band does not survive any age: retracing it needs a margin for the drift
    tight = _layer()
    tight.mark_swept(Footprint(1.29, 0.67, 0.36), (0.0, 0.0, 0.0), t=0.0)
    assert not L.envelope_admissible(tight, t=0.2, poses=back, footprint=FOOT, reverse=True)[0]
    # S-P1 implementation review P1: a band point seen after the sweep wins over it, in reverse too
    i, j = layer.cell(-0.55, 0.0)
    layer.occupied_t[i, j] = 0.5
    assert not L.envelope_admissible(layer, t=1.0, poses=back, footprint=FOOT, reverse=True)[0]


def test_only_the_body_itself_is_exempt_not_the_cells_it_clips():
    # S-P1 implementation review P1: an obstacle in a cell the body only clips, at (0.025, -0.363), is hit by a 0.29
    # left turn of 0.155 m; the cell's clipped part is no proof
    layer = _layer()
    layer.mark_swept(FOOT, (0.0, 0.0, 0.0), t=0.0)
    layer.integrate(_floor_points(0.6, 5.0, -3.0, 3.0, **WIDE), t=0.0, origin=CAM)
    turn = L.path_poses([(0.0, 0.0, 0.0)], [(0.155, 0.29)], step_m=0.01)
    ok, bad = L.envelope_admissible(layer, t=0.0, poses=turn, footprint=FOOT, reverse=False)
    assert not ok and any(x < 0.3 and y < -0.3 for x, y in bad)  # the rear corner swings out where nothing is known


def test_evidence_of_different_times_is_not_merged_before_eroding():
    # S-P1 implementation review P1: fresh free cells next to 3 s old ones; the seam is eroded by the older error
    layer = _layer()
    layer.integrate(_floor_points(0.6, 5.0, -3.0, 3.0, **WIDE), t=0.0, origin=CAM)
    seen = layer.frames[0][1]
    left = layer.centres()[1] > 0.0
    layer.frames = [(0.0, seen & ~left), (2.9, seen & left)]  # the right half seen at t 0, the left half just now
    usable = layer.usable(t=2.9, rear_pose=(0.0, 0.0, 0.0))
    # the old half may have drifted away from the fresh one: its cells at the seam are not usable, the fresh ones are
    assert not usable[layer.cell(2.0, -0.025)] and usable[layer.cell(2.0, 0.025)]
    assert usable[layer.cell(2.0, -1.0)] and usable[layer.cell(2.0, 1.0)]


def test_straight_driving_at_any_heading_needs_nothing_beside_the_body():
    # S-P1 implementation review round 3: the raster lies on the footprint's edges, so pixels the body's side only
    # touches are neither exempt nor needed -- at a heading off the grid too
    for yaw in (0.0, 0.4, -1.1):
        layer = _layer()
        start = (0.0, 0.0, yaw)
        layer.mark_swept(FOOT, start, t=0.0)
        lx, _ = L._local(np.stack(layer.centres(), -1).reshape(-1, 2), start)
        layer.frames = [(0.0, (lx > 1.2).reshape(layer.shape))]  # seen empty ahead of the forks only, every width
        ahead = L.path_poses([start], [(0.3, 0.0)], step_m=0.01)
        ok, bad = L.envelope_admissible(layer, t=0.0, poses=ahead, footprint=FOOT, reverse=False)
        assert ok, (yaw, bad)


def test_a_gap_between_two_kinds_of_evidence_is_refused():
    # S-P1 implementation review round 3 P1: a 2 mm strip between the swept box and the free cells is no proof
    for front, admissible in ((1.40, True), (1.398, False)):
        layer = _layer()
        layer.mark_swept_box((0.0, 0.0, 0.0), -0.17, front, 0.5, t=0.0)
        layer.frames = [(0.0, (layer.centres()[0] > 1.40).reshape(layer.shape))]  # cells from x 1.40 on
        ahead = L.path_poses([(0.0, 0.0, 0.0)], [(0.3, 0.0)], step_m=0.01)
        ok, bad = L.envelope_admissible(layer, t=0.0, poses=ahead, footprint=FOOT, reverse=False)
        assert ok == admissible, (front, bad)
        if not ok:
            assert all(1.39 < x < 1.41 for x, _ in bad)


def test_a_large_turn_between_neighbouring_poses_is_refused_not_folded():
    # S-P1 implementation review round 4 P2: (0, 0, 0) -> (0, 0, 2 pi) is a full turn on the spot, not no motion
    layer = _layer()
    with pytest.raises(ValueError):
        L.envelope_admissible(layer, t=0.0, poses=[(0.0, 0.0, 0.0), (0.0, 0.0, 2 * math.pi)], footprint=FOOT,
                              reverse=False)


def test_cells_are_read_only_for_polygons_smaller_than_a_cell():
    # S-P1 implementation review round 4 P3: a 40 mm square turned 45 deg spans 3x3 cells; its corner cells miss the
    # middle one, so such a probe is refused rather than answered
    layer = _layer()
    h = 0.02 * math.sqrt(2)
    square = np.array([[[1.025 - h, 0.025], [1.025, 0.025 - h], [1.025 + h, 0.025], [1.025, 0.025 + h]]])
    with pytest.raises(ValueError):
        layer.squares_in_cells(square, np.ones(layer.shape, bool))


def test_a_footprint_outside_the_grid_is_refused():
    # S-P1 implementation review P1: nothing is known outside the grid
    layer = L.LowLayer(CFG, origin_xy=(-2.0, -3.0), size_cells=(160, 120))  # x up to 6 m
    layer.mark_swept(FOOT, (4.0, 0.0, 0.0), t=0.0)
    out = L.path_poses([(4.0, 0.0, 0.0)], [(2.0, 0.0)])
    ok, bad = L.envelope_admissible(layer, t=0.0, poses=out, footprint=FOOT, reverse=False)
    assert not ok


def test_the_evidence_must_hold_until_braking_ends():
    # plan S: judged at the end of braking, no new observation assumed in between
    layer = _layer()
    layer.mark_swept(FOOT, (0.0, 0.0, 0.0), t=0.0)
    layer.frames = [(0.0, np.ones(layer.shape, bool))]  # the whole grid seen empty at t 0 (timing under test)
    ahead = L.path_poses([(0.0, 0.0, 0.0)], [(0.3, 0.0)])
    assert L.envelope_admissible(layer, t=0.5, poses=ahead, footprint=FOOT, reverse=False, t_check=1.0)[0]
    assert not L.envelope_admissible(layer, t=2.0, poses=ahead, footprint=FOOT, reverse=False,
                                     t_check=CFG.max_age_s + 0.2)[0]
    assert L.braking_end_s(0.6) == pytest.approx(0.15 + 0.4)
    assert L.odometry_error_m(CFG, 0.0, 2.0) == 0.0 and L.odometry_error_m(CFG, 0.05, 2.0) > 0.0


def test_the_self_mask_drops_the_forks():
    layer = _layer()
    forks = _box_points(0.93, 1.29, 0.16, 0.21, 0.0525)  # a tine's top and face, inside the footprint
    pts = np.vstack([_floor_points(0.6, 5.0, -3.0, 3.0, **WIDE), forks])
    layer.integrate(pts, t=0.0, origin=CAM, self_mask=(FOOT, (0.0, 0.0, 0.0)))
    assert layer.state_at(1.1, 0.18) != L.OCCUPIED
    layer.integrate(pts, t=0.1, origin=CAM)
    assert layer.state_at(1.1, 0.18) == L.OCCUPIED  # without the mask the fork is an obstacle


def test_an_unobserved_or_occupied_cell_in_the_envelope_refuses_it():
    layer = _layer()
    layer.mark_swept(FOOT, (0.0, 0.0, 0.0), t=0.0)
    layer.integrate(np.vstack([_floor_points(0.6, 5.0, -3.0, 3.0, step=0.04), _box_points(2.0, 2.3, -0.2, 0.2, 0.144)]), t=0.0, origin=CAM)
    ok, bad = L.envelope_admissible(layer, t=0.0, poses=L.path_poses([(0.0, 0.0, 0.0)], [(1.0, 0.0)]), footprint=FOOT,
                                    reverse=False)
    assert not ok and any(1.9 <= x <= 2.4 for x, _ in bad)
    # a left turn sweeps beside the body, where the camera never looked
    layer2 = _layer()
    layer2.mark_swept(FOOT, (0.0, 0.0, 0.0), t=0.0)
    layer2.integrate(_floor_points(1.25, 5.0, -0.3, 0.3), t=0.0, origin=CAM)
    turn = L.path_poses([(0.0, 0.0, 0.0)], [(0.6, 0.29)])
    ok, bad = L.envelope_admissible(layer2, t=0.0, poses=turn, footprint=FOOT, reverse=False)
    assert not ok


def test_a_floor_hit_without_the_crossed_column_is_not_free():
    # every height bin from 0.03 to 0.15 m over the cell must be crossed by some ray of the frame
    full = _layer()
    full.integrate(_floor_points(1.4, 5.0, -0.4, 0.4), t=0.0, origin=CAM)
    assert full.state_at(1.45, 0.0) == L.FREE
    # with a gap in the returns (nothing between 1.8 and 3.5 m), no ray crosses 0.12-0.15 m over the cell at 1.45 m:
    # the rays ending at 1.5-1.8 m pass it below 0.12 m and those ending at 3.5-4.0 m above 0.21 m
    gap = _layer()
    gap.integrate(np.vstack([_floor_points(1.4, 1.8, -0.4, 0.4), _floor_points(3.5, 4.0, -0.4, 0.4)]), t=0.0,
                  origin=CAM)
    assert gap.state_at(1.45, 0.0) == L.UNOBSERVED
    # a cell whose floor hits come only from rays ending there: nothing passed over it at all
    lone = _layer()
    lone.integrate(_floor_points(1.4, 1.8, -0.4, 0.4), t=0.0, origin=CAM)
    assert lone.state_at(1.75, 0.0) == L.UNOBSERVED


def test_path_poses_follow_arcs_at_the_given_curvature():
    poses = L.path_poses([(0.0, 0.0, 0.0)], [(math.pi / 2 / 0.29, 0.29)], step_m=0.01)
    x, y, yaw = poses[-1]
    r = 1 / 0.29
    assert abs(yaw - math.pi / 2) < 1e-6 and abs(x - r) < 1e-3 and abs(y - r) < 1e-3
    back = L.path_poses([(0.0, 0.0, 0.0)], [(-1.0, 0.0)], step_m=0.1)
    assert back[-1][0] == pytest.approx(-1.0) and back[-1][2] == pytest.approx(0.0)
