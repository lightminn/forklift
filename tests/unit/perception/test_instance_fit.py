# tests/unit/perception/test_instance_fit.py
import math
from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception import instance_fit as F
from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from forklift_core.perception.fork_fit import ForkSetup, LoadDeclaration
from forklift_core.perception.instance_proposals import InstanceProposal
from forklift_core.perception.pallet_damage import DamageOp
from tools import instance_rig, scene_rig
from tools import recognition_scenes as rs
from tools import recognition_truth as rt

ROOT = Path(__file__).resolve().parents[3]
EPAL, T11 = rs.known_spec("epal6"), rs.known_spec("t11_06")
SPECS = {"epal6": EPAL, "t11_06": T11}
SITE = {s.name: LoadDeclaration(10.0, 0.0, s.overall_depth_m / 2, 0.0, "site_default") for s in (EPAL, T11)}
SPACING = {EPAL.name: 0.3725, T11.name: 0.290}


def _render(items, *, noise=None, distractors=(), depth=None):
    """items: (spec, Placement, damage) -> scene, instance ids; depth(rendered) may edit the depth."""
    placed = [rt.placed_boxes(rt.instance_boxes(s, d), p) for s, p, d in items]
    out = instance_rig.render_instances(placed, distractors=distractors)
    scene, ids = out.scene, out.instance_ids
    if depth is not None:
        from dataclasses import replace
        scene = replace(scene, depth_m=depth(out))
    if noise is not None:
        scene, ids = apply_noise(scene, ids, noise)
    return scene, ids


def _mid_noise(seed):
    return NoiseModel.from_yaml(ROOT / "config/depth_noise_d435i.yaml", seed=seed, target_fx=465.74, level="mid")


def _faces_of(scene, mask, params=F.FitParams()):
    rays = F.pixel_rays(scene)
    pts, _ = F.proposal_points(scene, rays, mask, params.erode_px)
    return F.fit_faces(pts, F.camera_xyz(scene), params, np.random.default_rng(0)), pts, rays


def _yaw(face):
    return math.atan2(face.insertion_xy[1], face.insertion_xy[0])


# ---- Task 2: faces and ray classes -------------------------------------------------------------

def test_the_front_face_plane_and_its_insertion_yaw_are_recovered():
    scene, ids = _render([(EPAL, rt.Placement(3.0, 0.3, 0.25, 0.0), ())])
    faces, _, _ = _faces_of(scene, ids == 0)
    assert faces and _yaw(faces[0]) == pytest.approx(0.25, abs=math.radians(0.3))
    front = np.array([3.0, 0.3]) - 0.3 * np.array([math.cos(0.25), math.sin(0.25)])   # EPAL 6 depth 0.6 m
    assert abs((front - faces[0].centre_xy) @ faces[0].normal_xy) < 0.005
    assert (F.camera_xyz(scene)[:2] - faces[0].centre_xy) @ faces[0].normal_xy > 0


@pytest.mark.parametrize("x_m", [2.4, 2.8])
def test_a_t11_seen_at_45_degrees_gives_only_its_exterior_faces(x_m):
    """Seen across a corner, every accepted plane is one of the two outer faces (+-45 deg)."""
    scene, ids = _render([(T11, rt.Placement(x_m, 0.0, math.pi / 4, 0.0), ())])
    faces, _, _ = _faces_of(scene, ids == 0)
    assert faces and all(abs(abs(math.degrees(_yaw(f))) - 45) < 1 for f in faces)


@pytest.mark.parametrize("placement", [
    rt.Placement(2.1500067490892496, -0.8781266206323024, 0.3974022305354298, 0.0),  # s1_t11_06_1000119 (Isaac dev)
    rt.Placement(1.9, 0.35, 0.55, 0.0),
])
def test_no_two_accepted_faces_are_parallel(placement):
    """A convex pallet shows at most one face per direction. In the first case the block sides
    seen through a lane form a plane parallel to the side face and 75 mm inside it (accepted
    without the exterior check); in the second the leftover points form a loose 'plane' with a
    48 mm scale (accepted without the scale limit)."""
    scene, ids = _render([(T11, placement, ())])
    faces, _, _ = _faces_of(scene, ids == 0)
    for i in range(len(faces)):
        for j in range(i + 1, len(faces)):
            d = abs((_yaw(faces[i]) - _yaw(faces[j]) + math.pi) % (2 * math.pi) - math.pi)
            assert d > math.radians(30)


def _wrong_faces(faces, pallet_yaw):
    """Faces that are not one of the box's four face directions (> 3 deg off) or form a parallel pair."""
    ys = [math.degrees(_yaw(f)) for f in faces]
    off = [y for y in ys if min(abs((y - math.degrees(pallet_yaw) - 90 * k + 180) % 360 - 180) for k in range(4)) > 3]
    parallel = [(a, b) for i, a in enumerate(ys) for b in ys[i + 1:] if abs((a - b + 180) % 360 - 180) < 30]
    return off + parallel


@pytest.mark.parametrize("scene_id, k, gate", [
    ("s1_epal6_1000009", 0, {"corner_tol_rad": math.pi / 2}),        # without it: a 5.2 deg face beside 5.6
    ("s2_t11_06_1000227", 1, {"exterior_max_fraction": 1.0}),        # without it: an interior 112.9 deg plane
    ("s2_t11_06_1000218", 0, {"max_face_scale_m": 1e9}),             # without it: a loose -12.2 deg 'plane'
])
def test_each_face_gate_removes_a_wrong_face_on_its_own(scene_id, k, gate):
    """Rig renders of dev recipes where turning off this one gate lets a wrong face through
    (ws1 search 2026-10-16; Codex: each gate needs a regression of its own)."""
    from dataclasses import replace
    recipe = next(r for r in rs.catalogue() if r.scene_id == scene_id)
    out = instance_rig.render_instances([rs.recipe_boxes(recipe, j) for j in range(len(recipe.instances))])
    yaw = recipe.instances[k].placement.yaw_rad
    faces, pts, _ = _faces_of(out.scene, out.instance_ids == k)
    assert not _wrong_faces(faces, yaw)
    loose = F.fit_faces(pts, F.camera_xyz(out.scene), replace(F.FitParams(), **gate), np.random.default_rng(0))
    assert _wrong_faces(loose, yaw)


def test_the_face_survives_far_range_noise():
    scene, ids = _render([(EPAL, rt.Placement(3.9, 0.0, 0.1, 0.0), ())], noise=_mid_noise(3))
    faces, _, _ = _faces_of(scene, ids == 0)
    assert faces and abs(_yaw(faces[0]) - 0.1) < math.radians(15)


def test_rays_through_an_opening_pass_behind_the_face_and_rays_on_a_block_stop_on_it():
    scene, ids = _render([(EPAL, rt.Placement(2.6, 0.0, 0.0, 0.0), ())])
    faces, _, rays = _faces_of(scene, ids == 0)
    u, h, cls, owned = F.ray_samples(scene, rays, faces[0], (-0.5, 0.5), (0.0, 0.2), F.FitParams(), mask=ids == 0)
    lane = (np.abs(u - 0.18625) < 0.05) & (h > 0.03) & (h < 0.09)   # left opening centre (EPAL 6 y = 0.18625)
    block = (np.abs(u) < 0.04) & (h > 0.03) & (h < 0.09)            # middle block
    assert (cls[lane] == F.BEHIND).mean() > 0.95 and (cls[block] == F.FACE).mean() > 0.95
    assert owned[block].mean() > 0.9   # (rays through a lane may end on the lane walls: owned, but behind the face)


def test_erode_shrinks_a_mask_by_its_radius():
    m = np.zeros((9, 9), bool)
    m[2:7, 2:7] = True
    assert F.erode(m, 1).sum() == 9 and F.erode(m, 2).sum() == 1 and not F.erode(m, 3).any()


# ---- Task 3: rasters and the placement search ---------------------------------------------------

def test_an_x_face_raster_has_open_lanes_down_to_the_support():
    occ = F.face_raster(EPAL, "x", 0.005)
    lane = occ[:int(0.10 / 0.005), int((0.4 + 0.18625) / 0.005)]       # left opening, below the ceiling
    assert occ.shape == (round(EPAL.overall_height_m / 0.005), round(0.8 / 0.005))
    assert not lane.any() and occ[int(0.11 / 0.005)].all()              # lane open to the floor, stringer row full
    assert not occ[-1].all()                                            # the top deck shows the gaps between boards
    assert occ[:, int(0.4 / 0.005)].all()                               # the middle column is solid


def test_a_y_face_lane_reaches_up_to_the_top_deck():
    occ = F.face_raster(T11, "y", 0.005)
    col = occ[:, occ.shape[1] // 2 + int(0.145 / 0.005)]               # a y-face opening column (centre 0.145 m)
    rows = np.nonzero(~col)[0] * 0.005
    assert rows.min() == pytest.approx(T11.deck_bottom_m, abs=0.005)
    assert rows.max() == pytest.approx(T11.deck_bottom_m + T11.opening_height_m + T11.stringer_m, abs=0.006)


def _evidence(scene, ids, k=0):
    p = F.FitParams()
    rays = F.pixel_rays(scene)
    pts, _ = F.proposal_points(scene, rays, ids == k, p.erode_px)
    face = F.fit_faces(pts, F.camera_xyz(scene), p, np.random.default_rng(0))[0]
    u, h = F.face_coords(face, pts[face.index])
    h_lo = float(np.percentile(h, 2)) - p.support_window_m
    rs_ = F.ray_samples(scene, rays, face, (u.min() - 0.9, u.max() + 0.9), (h_lo, h_lo + 0.4), p, mask=ids == k)
    return face, u, h, rs_, h_lo


def test_the_placement_search_finds_the_pallet_centre_and_support_height():
    scene, ids = _render([(EPAL, rt.Placement(2.8, -0.2, 0.15, 0.0), ())])
    face, u, h, rays, h_lo = _evidence(scene, ids)
    best = F.best_placement(F.face_raster(EPAL, "x", 0.005), u, h, rays, F.FitParams(), h_lo, 13)
    centre = np.array([2.8, -0.2]) - 0.3 * np.array([math.cos(0.15), math.sin(0.15)])
    assert (face.centre_xy + face.left_xy * best.u_centre - centre) @ face.left_xy == pytest.approx(0.0, abs=0.005)
    assert best.support_z == pytest.approx(0.0, abs=0.005) and best.score > 0.8


def test_the_right_shape_scores_higher_than_the_wrong_one():
    for spec, other in ((EPAL, T11), (T11, EPAL)):
        scene, ids = _render([(spec, rt.Placement(2.6, 0.0, -0.1, 0.0), ())])
        _, u, h, rays, h_lo = _evidence(scene, ids)
        right = F.best_placement(F.face_raster(spec, "x", 0.005), u, h, rays, F.FitParams(), h_lo, 13)
        wrong = F.best_placement(F.face_raster(other, "x", 0.005), u, h, rays, F.FitParams(), h_lo, 13)
        assert right.score > wrong.score + 0.2


@pytest.mark.parametrize("spec", [EPAL, T11])
def test_rays_never_add_score(spec):
    """Removing rays of either class can only raise the best score, so no placement can earn
    score over empty background (Codex 2026-10-16: the first prototype rewarded BEHIND rays)."""
    scene, ids = _render([(T11, rt.Placement(2.6, 0.0, 0.0, 0.0), ())])
    _, u, h, rays, h_lo = _evidence(scene, ids)
    occ = F.face_raster(spec, "x", 0.005)
    full = F.best_placement(occ, u, h, rays, F.FitParams(), h_lo, 13).score
    for cls in (F.BEHIND, F.FACE):
        keep = rays[2] != cls
        fewer = tuple(a[keep] for a in rays)
        assert F.best_placement(occ, u, h, fewer, F.FitParams(), h_lo, 13).score >= full - 1e-12
