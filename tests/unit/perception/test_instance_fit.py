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


def _load(spec):
    """Site default load for a known shape; a non-standard reading (spec None) has none."""
    return None if spec is None else SITE.get(spec.name)


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


# ---- Task 4: hypotheses, support, candidates ----------------------------------------------------

def _fit(items, k=0, *, source="oracle", noise=None, distractors=(), depth=None, spacing=None):
    scene, ids = _render(items, noise=noise, distractors=distractors, depth=depth)
    spec = items[k][0]
    setup = ForkSetup(spacing or SPACING[spec.name])
    cand = F.fit_instance(scene, InstanceProposal("p0", ids == k, 1.0, source), specs=SPECS, setup=setup,
                          load_for=_load)
    truths = rt.scene_truths([(s, p, d, SITE[s.name]) for s, p, d in items], setup)[0]
    return cand, truths[k]


def _errors(cand, truth):
    best = None
    for face in truth.faces:
        if not face.accessible:
            continue
        a = np.array([math.cos(face.insertion_yaw_rad), math.sin(face.insertion_yaw_rad), 0.0])
        n = np.array([-a[1], a[0], 0.0])
        dl = np.subtract(cand.left.centre_m, face.left_m)
        dr = np.subtract(cand.right.centre_m, face.right_m)
        yaw = abs((cand.insertion_yaw_rad - face.insertion_yaw_rad + math.pi) % (2 * math.pi) - math.pi)
        e = (max(abs(dl @ n), abs(dr @ n)), max(abs(dl @ a), abs(dr @ a)), yaw, max(abs(dl[2]), abs(dr[2])))
        if best is None or sum(e[:2]) < sum(best[:2]):
            best = e
    return best


@pytest.mark.parametrize("spec, placement", [
    (EPAL, rt.Placement(3.0, 0.4, 0.3, 0.0)),
    (EPAL, rt.Placement(2.2, -0.5, -0.45, 0.0)),
    (T11, rt.Placement(2.6, 0.0, 0.2, 0.0)),
    (T11, rt.Placement(2.4, 0.3, math.pi / 2 + 0.2, 0.0)),   # the y face toward the camera
])
def test_a_clean_known_pallet_is_fitted_within_the_pocket_tolerances(spec, placement):
    cand, truth = _fit([(spec, placement, ())])
    assert cand.geometry == "fitted" and cand.counted and cand.hypothesis.startswith(spec.name)
    lateral, axial, yaw, z = _errors(cand, truth)
    assert lateral <= 0.010 and axial <= 0.010 and yaw <= math.radians(0.5) and z <= 0.006
    assert cand.support_state == "visible" and cand.on_floor
    assert cand.pallet_centre_m[:2] == pytest.approx((placement.x_m, placement.y_m), abs=0.01)
    # T11 entered through a y face has no roof board edges along the forks: near_tracking_unavailable.
    # A face that passes still waits for the near-range re-observation (§9.1): pickable None, never
    # True, with the unverified items listed (Codex 2026-10-16: a known template is not a measurement).
    truth_ok = any(f.fit["pickable"] for f in truth.faces if f.accessible and f.axis == cand.axis)
    assert cand.pickable is (None if truth_ok else False)
    assert cand.pending == F.FAR_PENDING


def test_left_is_the_larger_lateral_pocket():
    cand, _ = _fit([(EPAL, rt.Placement(2.8, 0.0, 0.0, 0.0), ())])
    assert cand.left.centre_m[1] > cand.right.centre_m[1]


def test_a_damaged_pallet_is_still_fitted():
    damage = (DamageOp("top_board_missing", "top_board_0", ()), DamageOp("top_board_missing", "top_board_3", ()))
    cand, truth = _fit([(EPAL, rt.Placement(2.7, 0.1, 0.2, 0.0), damage)])
    assert cand.geometry == "fitted"
    lateral, axial, yaw, _ = _errors(cand, truth)
    assert lateral <= 0.010 and axial <= 0.010 and yaw <= math.radians(0.5)


@pytest.mark.parametrize("x_m, boards", [(2.6, ("top_board_6",)), (2.6, ("top_board_5", "top_board_6")),
                                         (2.04, ("top_board_5", "top_board_6"))])
def test_missing_edge_boards_are_damage_not_a_different_shape(x_m, boards):
    """Codex 2026-10-16: a T11 seen through its y face with its edge board missing read as
    shape_mismatch (seen-through 0.278), with two adjacent boards missing as well (0.238), and
    the vertical search drifted 12 mm, adding a false lift_alignment_needed; at 2.04 m the floor
    just below the edge is only 22 mm in front of the face and was taken for the face. Allowed damage
    (design §5.2: 1-2 top boards, edge boards included) must fit like an intact pallet."""
    damage = tuple(DamageOp("top_board_missing", b, ()) for b in boards)
    cand, truth = _fit([(T11, rt.Placement(x_m, 0.0, math.pi / 2, 0.0), damage)])
    assert cand.geometry == "fitted" and cand.axis == "y" and cand.support_state == "visible"
    lateral, axial, yaw, z = _errors(cand, truth)
    assert lateral <= 0.010 and axial <= 0.010 and yaw <= math.radians(0.5) and z <= 0.006
    assert cand.support_z_m == pytest.approx(0.0, abs=0.006) and cand.on_floor
    face = next(f for f in truth.faces if f.accessible and f.axis == "y")
    assert cand.fit_reasons == tuple(face.fit["reasons"])


def test_the_verdict_belongs_to_the_returned_placement():
    """Codex 2026-10-16: the support correction moved the placement after the gates had run, so a
    shape_mismatch (or a pass of min_score) of the old placement came back with the new one."""
    from dataclasses import replace
    damage = (DamageOp("top_board_missing", "top_board_5", ()), DamageOp("top_board_missing", "top_board_6", ()))
    noisy, _ = _fit([(T11, rt.Placement(4.0, 0.2, math.pi / 2, 0.0), damage)], noise=_mid_noise(3))
    assert noisy.geometry != "ambiguous" or "shape_mismatch" not in noisy.reasons
    scene, ids = _render([(T11, rt.Placement(4.0, 0.0, math.pi / 2, 0.0), damage)])
    strict = replace(F.FitParams(), min_score=0.6)
    cand = F.fit_instance(scene, InstanceProposal("p0", ids == 0, 1.0, "oracle"), specs=SPECS,
                          setup=ForkSetup(0.290), load_for=_load, params=strict)
    assert (cand.geometry == "unfit") == (cand.score < 0.6)


def test_a_switch_to_the_other_face_gets_that_faces_support_too():
    """Codex 2026-10-16: the first pass chose the y face (support 5.7 mm), fixing it moved the choice
    to the x face (3.2 mm uncorrected) and the second pass stopped there -- the x face came back
    pickable at 0.80 rad although every face fixed gives the y face, not pickable, at -0.77 rad."""
    damage = (DamageOp("top_board_missing", "top_board_5", ()), DamageOp("top_board_missing", "top_board_6", ()))
    cand, truth = _fit([(T11, rt.Placement(2.6, -0.2, 0.8, 0.0), damage)])
    # The expected end state is stated outright: matching the truth of whatever axis came back
    # would accept the wrong face (Codex 2026-10-16, round 6).
    assert cand.geometry == "fitted" and cand.axis == "y" and cand.support_state == "visible"
    assert cand.insertion_yaw_rad == pytest.approx(-0.7708, abs=0.02)
    assert cand.support_z_m == pytest.approx(0.0, abs=0.002)
    assert cand.fit_reasons == ("near_tracking_unavailable",) and cand.pickable is False


def test_a_placement_shifted_by_a_member_pitch_competes_with_the_true_one():
    """Codex review A 2026-10-16: two occluders leave a T11 face where the raster shifted by one block
    scores 0.7173 against 0.7154 at the true place; keeping only the best placement per shape fitted
    the wrong one, 286 mm off, as pickable."""
    occluders = [scene_rig.Box((1.2, 0.4, 0.2), (0.05, 0.77, 0.4)), scene_rig.Box((1.2, -0.5, 0.2), (0.05, 0.72, 0.4))]
    cand, truth = _fit([(T11, rt.Placement(2.0, 0.0, 0.0, 0.0), ())], distractors=occluders)
    if cand is not None and cand.geometry == "fitted":
        lateral, *_ = _errors(cand, truth)
        assert lateral <= 0.02
    else:
        assert cand is None or any(r.startswith("competing:") for r in cand.reasons) or cand.geometry != "fitted"


@pytest.mark.parametrize("occluders", [
    [scene_rig.Box((1.2, 0.4, 0.2), (0.05, 0.77, 0.4)), scene_rig.Box((1.2, -0.5, 0.2), (0.05, 0.72, 0.4))],
    # review A round 3: a third placement near the true centre (0.9696 against 0.9797) was dropped
    [scene_rig.Box((1.2, 0.4, 0.1), (0.05, 0.75, 0.2)), scene_rig.Box((1.2, -0.4, 0.1), (0.05, 0.75, 0.2))],
])
def test_the_shifted_rival_keeps_the_target_ambiguous_through_selection(occluders):
    """Codex review A round 2: with a second T11 at (3.2, 0) and an instruction at (2.6, 0) r 0.62 both
    centres are inside, but the front candidate's wrong best centre (2.0, -0.286) fell outside and the
    back one was selected. The rival placement's centre travels with the candidate."""
    from forklift_core.perception.target_selection import select_target
    scene, ids = _render([(T11, rt.Placement(2.0, 0.0, 0.0, 0.0), ()), (T11, rt.Placement(3.2, 0.0, 0.0, 0.0), ())],
                         distractors=occluders)
    props = [InstanceProposal(f"p{k}", ids == k, 1.0, "oracle") for k in (0, 1) if (ids == k).any()]
    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.290), load_for=_load)
    instr = {"present": True, "x_m": 2.6, "y_m": 0.0, "radius_m": 0.62, "layer": None, "load": None}
    assert select_target(out.candidates, instr, unresolved=out.failures).status != "selected"


def test_a_rejected_shape_still_keeps_the_target_ambiguous_through_selection():
    """Codex review A round 4: S7 dev shape s7_nonstandard_1000769 at (2, -0.4) and a T11 at
    (3.2, 0.4), instruction (2.6, 0) r 0.73 -- both centres 0.721 m away -- selected the T11 because the
    S7 candidate's rejected EPAL centre (1.779, -0.423) lay outside."""
    from forklift_core.perception.target_selection import select_target
    s7 = next(r for r in rs.catalogue() if r.scene_id == "s7_nonstandard_1000769").instances[0].spec
    scene, ids = _render([(s7, rt.Placement(2.0, -0.4, 0.0, 0.0), ()), (T11, rt.Placement(3.2, 0.4, 0.0, 0.0), ())])
    props = [InstanceProposal(f"p{k}", ids == k, 1.0, "oracle") for k in (0, 1) if (ids == k).any()]
    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.290), load_for=_load)
    instr = {"present": True, "x_m": 2.6, "y_m": 0.0, "radius_m": 0.73, "layer": None, "load": None}
    assert select_target(out.candidates, instr, unresolved=out.failures).status != "selected"


def test_a_pallet_hidden_but_for_one_block_is_not_fitted():
    """An occluder, not a cut mask: raw rays would still see members hidden only from the mask."""
    occluder = scene_rig.Box((1.6, -0.15, 0.15), (0.1, 0.6, 0.3))   # silhouette reaches y = 0.272 at the face
    cand, _ = _fit([(EPAL, rt.Placement(2.5, 0.0, 0.0, 0.0), ())], distractors=[occluder])
    assert cand is None or cand.geometry != "fitted"


@pytest.mark.parametrize("seen, geometry", [(1, "ambiguous"), (2, "fitted")])
def test_fewer_than_two_members_with_a_seen_edge_leave_the_lateral_position_unconstrained(monkeypatch, seen, geometry):
    """The gate itself (Codex 2026-10-16: the occluder test passes on other gates even when
    observed_members always says 99)."""
    monkeypatch.setattr(F, "observed_members", lambda *a, **k: seen)
    cand, _ = _fit([(EPAL, rt.Placement(2.6, 0.0, 0.0, 0.0), ())])
    assert cand.geometry == geometry and (geometry == "fitted" or "lateral_unconstrained" in cand.reasons)


def test_a_member_edge_counts_only_with_the_proposals_own_face_rays():
    hyp = F.Hypothesis("t:x", EPAL, "x", False, ((-0.1, 0.0), (0.2, 0.3)), (0.0, 0.1),
                       F.Placement2D(1.0, 0.0, 0.0), 0)
    u = np.array([-0.095, -0.094, -0.093, -0.105, -0.106, -0.107, 0.205, 0.206, 0.207, 0.195, 0.194, 0.193])
    h = np.full(u.shape, 0.05)
    cls = np.array([F.FACE] * 3 + [F.BEHIND] * 3 + [F.FACE] * 3 + [F.BEHIND] * 3, np.int8)
    for owned, expected in ((np.zeros(u.shape, bool), 0), (np.ones(u.shape, bool), 2)):
        assert F.observed_members(hyp, (u, h, cls, owned), F.FitParams(), 0.015) == expected
    far = u + np.where(u < 0.05, -0.05, 0.05) * (cls == F.BEHIND)   # behind rays 50 mm outside the window
    assert F.observed_members(hyp, (far, h, cls, np.ones(u.shape, bool)), F.FitParams(), 0.015) == 0


def test_a_hidden_lower_edge_leaves_the_support_uncertain_and_the_fit_undecided():
    """Codex 2026-10-16: a low front occluder lifted the support search to 10.8 mm and the
    candidate still came out pickable."""
    occluder = scene_rig.Box((1.8, 0.0, 0.06), (0.05, 1.5, 0.12))
    cand, _ = _fit([(EPAL, rt.Placement(2.8, 0.0, 0.0, 0.0), ())], distractors=[occluder])
    assert cand.support_state == "uncertain" and "support_uncertain" in cand.reasons
    assert cand.pickable is None and not cand.on_floor


def test_a_stack_reports_support_heights_and_how_each_edge_was_seen():
    H = EPAL.overall_height_m
    items = [(EPAL, rt.Placement(2.8, 0.0, 0.0, 0.0), ()), (EPAL, rt.Placement(2.8, 0.0, 0.0, H), ()),
             (EPAL, rt.Placement(2.8, 0.0, 0.0, 2 * H), ())]
    states = []
    for k in range(3):
        cand, _ = _fit(items, k)
        assert cand.support_z_m == pytest.approx(k * H, abs=0.006)
        states.append(cand.support_state)
    # The top support (0.288 m) is above the camera (0.27 m): its surface cannot be seen (truth: support_visible False)
    assert states == ["visible", "coplanar", "coplanar"]


def test_x_and_y_readings_of_a_t11_face_compete_instead_of_merging():
    """Codex 2026-10-16: with depth only in 20-55 mm the x and y readings scored 0.940 / 0.943 and
    their pockets were 17.5 mm apart; merging them hid a choice between different fork fits."""
    def band_only(out):
        d = out.scene.depth_m.copy()
        z = out.points_base[..., 2]
        d[(out.instance_ids == 0) & ((z < 0.020) | (z > 0.055))] = np.nan
        return d
    cand, _ = _fit([(T11, rt.Placement(2.6, 0.0, math.pi / 2, 0.0), ())], depth=band_only)
    assert cand.geometry == "ambiguous" and any(r.startswith("competing:") for r in cand.reasons)
    clean, _ = _fit([(T11, rt.Placement(2.6, 0.0, math.pi / 2, 0.0), ())])
    assert clean.geometry == "fitted" and clean.axis == "y"


@pytest.mark.parametrize("scene_id", ["s7_nonstandard_1000738", "s7_nonstandard_1000759"])
def test_a_non_standard_pallet_is_not_fitted_as_a_known_shape(scene_id):
    """Rig renders of S7 dev recipes (0.77 m wide with four blocks; 0.99 m wide) that the EPAL raster
    fits when the shape check is off (ws1 dev 2026-10-16: 76 of 135 S7 pallets came out fitted).
    With the parametric reading off the shape check leaves them ambiguous; with it on (stage 2B)
    they are read as non-standard, never as a known shape."""
    from dataclasses import replace
    recipe = next(r for r in rs.catalogue() if r.scene_id == scene_id)
    out = instance_rig.render_instances([rs.recipe_boxes(recipe, 0)])
    proposal = InstanceProposal("p0", out.instance_ids == 0, 1.0, "oracle")
    kw = dict(specs=SPECS, setup=ForkSetup(recipe.fork_spacing_m), load_for=_load)
    known_only = replace(F.FitParams(), parametric=False)
    off = replace(known_only, shape_outside_max=9.0, shape_seen_through_max=9.0)
    assert F.fit_instance(out.scene, proposal, params=off, **kw).geometry == "fitted"
    cand = F.fit_instance(out.scene, proposal, params=known_only, **kw)
    assert cand.geometry == "ambiguous" and "shape_mismatch" in cand.reasons
    full = F.fit_instance(out.scene, proposal, **kw)
    # 1000759 sits at 1.07 m with its right end out of the image: unreadable, so never a confident fit
    assert full.hypothesis == "parametric:x" or full.geometry != "fitted"


def test_a_geometric_proposal_stays_unconfirmed():
    cand, _ = _fit([(EPAL, rt.Placement(2.8, 0.0, 0.0, 0.0), ())], source="geometric")
    assert cand.geometry == "fitted" and cand.identity == "unconfirmed" and not cand.counted


def test_pickability_follows_the_fork_fit_of_the_fitted_shape():
    cand, _ = _fit([(T11, rt.Placement(2.6, 0.0, 0.0, 0.0), ())], spacing=0.3725)
    assert cand.pickable is False and "fork_spacing_adjust" in cand.fit_reasons


def test_noise_far_away_still_gives_a_candidate_with_the_right_shape():
    cand, _ = _fit([(T11, rt.Placement(3.6, 0.2, 0.1, 0.0), ())], noise=_mid_noise(5))
    assert cand is not None and cand.hypothesis.startswith(T11.name)


def test_an_over_segmented_mask_is_still_fitted():
    """A learned mask spills a few pixels onto the floor and background (Gemini review 2026-10-16).
    Dev check: masks dilated by 4 px gave the same fitted counts and errors on Isaac S1/S2/S5."""
    def dilate(mask, r=4):
        for _ in range(r):
            m = mask.copy()
            m[1:] |= mask[:-1]
            m[:-1] |= mask[1:]
            m[:, 1:] |= mask[:, :-1]
            m[:, :-1] |= mask[:, 1:]
            mask = m
        return mask
    placement = rt.Placement(2.9, 0.2, 0.2, 0.0)
    scene, ids = _render([(EPAL, placement, ())])
    cand = F.fit_instance(scene, InstanceProposal("p0", dilate(ids == 0), 1.0, "oracle"), specs=SPECS,
                          setup=ForkSetup(0.3725), load_for=_load)
    truth = rt.scene_truths([(EPAL, placement, (), SITE[EPAL.name])], ForkSetup(0.3725))[0][0]
    lateral, axial, yaw, z = _errors(cand, truth)
    assert cand.geometry == "fitted" and lateral <= 0.010 and axial <= 0.010 and z <= 0.006


def test_fit_scene_keeps_proposal_order_and_records_failures(monkeypatch):
    scene, ids = _render([(EPAL, rt.Placement(2.6, -0.6, 0.0, 0.0), ()), (EPAL, rt.Placement(2.6, 0.6, 0.0, 0.0), ())])
    tiny = np.zeros(ids.shape, bool)
    tiny[0, 0] = True
    props = [InstanceProposal("a", ids == 0, 1.0, "oracle"), InstanceProposal("b", tiny, 1.0, "oracle"),
             InstanceProposal("c", ids == 1, 1.0, "oracle")]
    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.3725), load_for=_load)
    assert [(c.candidate_id, c.proposal_id) for c in out.candidates] == [("c0", "a"), ("c2", "c")]
    assert [(f.proposal_id, f.reason) for f in out.failures] == [("b", "no_face")]
    real = F.fit_instance

    def boom(scene, proposal, **kw):
        if proposal.proposal_id == "a":
            raise FloatingPointError("injected")
        return real(scene, proposal, **kw)

    monkeypatch.setattr(F, "fit_instance", boom)
    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.3725), load_for=_load)
    assert [c.proposal_id for c in out.candidates] == ["c"]
    failed = {f.proposal_id: f for f in out.failures}
    assert failed["a"].reason == "exception:FloatingPointError"
    # The failed pallet keeps an approximate position (its visible surface) for target selection
    assert failed["a"].centre_xy == pytest.approx((2.3, -0.6), abs=0.15)


def test_a_proposal_on_the_wrong_grid_fails_alone():
    """Codex review A 2026-10-16: a 2 x 2 mask raised again while its failure position was computed
    and stopped the scene, so the next (valid) proposal was never fitted."""
    scene, ids = _render([(EPAL, rt.Placement(2.6, 0.0, 0.0, 0.0), ())])
    props = [InstanceProposal("w", np.ones((2, 2), bool), 1.0, "oracle"), InstanceProposal("a", ids == 0, 1.0, "oracle")]
    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.3725), load_for=_load)
    assert [c.proposal_id for c in out.candidates] == ["a"]
    (failure,) = out.failures
    assert failure.proposal_id == "w" and failure.reason.startswith("bad_mask") and failure.centre_xy is None
