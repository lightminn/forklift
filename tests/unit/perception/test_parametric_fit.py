# tests/unit/perception/test_parametric_fit.py
import math
from dataclasses import replace

import numpy as np
import pytest

from forklift_core.perception import instance_fit as F
from forklift_core.perception.fork_fit import ForkSetup, LoadDeclaration
from forklift_core.perception.instance_proposals import InstanceProposal
from forklift_core.perception.target_selection import select_target
from tools import instance_rig, scene_rig
from tools import recognition_scenes as rs
from tools import recognition_truth as rt

EPAL, T11 = rs.known_spec("epal6"), rs.known_spec("t11_06")
SPECS = {"epal6": EPAL, "t11_06": T11}
SITE = {s.name: LoadDeclaration(10.0, 0.0, s.overall_depth_m / 2, 0.0, "site_default") for s in (EPAL, T11)}


def _load(spec):
    return None if spec is None else SITE.get(spec.name)


def _recipe(scene_id):
    return next(r for r in rs.catalogue() if r.scene_id == scene_id)


def _fit_spec(spec, placement, spacing, *, distractors=(), load=_load):
    out = instance_rig.render_instances([rt.placed_boxes(rt.instance_boxes(spec, ()), placement)], distractors=distractors)
    cand = F.fit_instance(out.scene, InstanceProposal("p0", out.instance_ids == 0, 1.0, "oracle"), specs=SPECS,
                          setup=ForkSetup(spacing), load_for=load)
    truth = rt.scene_truths([(spec, placement, (), LoadDeclaration(2.0, 1.0, spec.overall_depth_m / 2, 0.0))],
                            ForkSetup(spacing))[0][0]
    return cand, truth


def _errors(cand, truth):
    face = next(f for f in truth.faces if f.accessible)
    a = np.array([math.cos(face.insertion_yaw_rad), math.sin(face.insertion_yaw_rad), 0.0])
    n = np.array([-a[1], a[0], 0.0])
    dl, dr = np.subtract(cand.left.centre_m, face.left_m), np.subtract(cand.right.centre_m, face.right_m)
    yaw = abs((cand.insertion_yaw_rad - face.insertion_yaw_rad + math.pi) % (2 * math.pi) - math.pi)
    return max(abs(dl @ n), abs(dr @ n)), max(abs(dl @ a), abs(dr @ a)), yaw, face


@pytest.mark.parametrize("scene_id", ["s7_nonstandard_1000738", "s7_nonstandard_1000759", "s7_nonstandard_1000745"])
def test_a_non_standard_face_is_read_from_the_data(scene_id):
    recipe = _recipe(scene_id)
    spec = recipe.instances[0].spec
    cand, truth = _fit_spec(spec, rt.Placement(2.9, 0.1, 0.15, 0.0), recipe.fork_spacing_m)
    assert cand.hypothesis == "parametric:x" and cand.geometry == "fitted" and cand.shape.spec is None
    lateral, axial, yaw, _ = _errors(cand, truth)
    assert lateral <= 0.02 and axial <= 0.01 and yaw <= math.radians(0.5)
    assert cand.shape.face.width_m == pytest.approx(spec.overall_width_m, abs=0.03)
    assert len(cand.shape.face.member_intervals_m) == len(spec.column_centres_y_m)
    assert dict(cand.shape.sources)["depth"] == "unobserved"


def test_a_known_pallet_is_never_read_as_non_standard():
    cand, _ = _fit_spec(EPAL, rt.Placement(2.9, 0.1, 0.15, 0.0), 0.3725)
    assert cand.hypothesis == "epal6:x" and cand.geometry == "fitted"


def test_a_non_standard_pallet_cut_off_at_one_end_is_not_fitted():
    """§7.4-5c: a non-standard reading needs both physical ends -- the inner part of two touching
    pallets looks like one wide non-standard pallet."""
    recipe = _recipe("s7_nonstandard_1000759")
    spec = recipe.instances[0].spec
    # Camera at x 0.75: a box at x 1.8 projects onto the face (x ~2.63) scaled by ~1.79, so y 0.22-0.80
    # hides the face from y ~0.39 outward -- the left end (0.59) is hidden, three members still show.
    occluder = scene_rig.Box((1.8, 0.51, 0.15), (0.05, 0.58, 0.3))
    cand, _ = _fit_spec(spec, rt.Placement(2.9, 0.1, 0.0, 0.0), recipe.fork_spacing_m, distractors=[occluder])
    assert cand is None or cand.geometry != "fitted"


def test_the_face_decides_only_what_it_can_see():
    """Fork spacing comes from the face and the payload from the instruction; the lane bounds, the
    depth and the top layout wait for the near-range look, so a passing face is pickable None --
    never True -- with them pending."""
    recipe = _recipe("s7_nonstandard_1000738")
    spec = recipe.instances[0].spec
    instructed = LoadDeclaration(2.0, 1.0, 0.2, 0.0)
    ok, truth = _fit_spec(spec, rt.Placement(2.9, 0.0, 0.0, 0.0), recipe.fork_spacing_m, load=lambda s: instructed)
    assert ok.pickable is None and ok.fit_reasons == () and ok.pending == F.FAR_PENDING
    wrong, _ = _fit_spec(spec, rt.Placement(2.9, 0.0, 0.0, 0.0), 0.60, load=lambda s: instructed)
    assert wrong.pickable is False and "fork_spacing_adjust" in wrong.fit_reasons
    heavy = LoadDeclaration(6.0, 6.0, 0.2, 0.0)
    loaded, _ = _fit_spec(spec, rt.Placement(2.9, 0.0, 0.0, 0.0), recipe.fork_spacing_m, load=lambda s: heavy)
    assert loaded.pickable is False and "payload_limit" in loaded.fit_reasons


def test_a_non_standard_centre_covers_the_unseen_depth_in_selection():
    """The instruction names the pallet centre, which for a non-standard pallet lies somewhere along
    half the depth range behind the face: selection must not call it outside the radius."""
    recipe = _recipe("s7_nonstandard_1000738")
    spec = recipe.instances[0].spec
    placement = rt.Placement(3.0, 0.0, 0.0, 0.0)
    instructed = LoadDeclaration(2.0, 1.0, 0.2, 0.0)  # without it the support is unverified: not_pickable
    cand, _ = _fit_spec(spec, placement, recipe.fork_spacing_m, load=lambda s: instructed)
    face_x = 3.0 - spec.overall_depth_m / 2
    far_centre = {"present": True, "x_m": face_x + 0.58, "y_m": 0.0, "radius_m": 0.05, "layer": None, "load": None}
    assert select_target([cand], far_centre).status == "selected"
    near_centre = dict(far_centre, x_m=face_x + 0.26)
    assert select_target([cand], near_centre).status == "selected"
    beyond = dict(far_centre, x_m=face_x + 1.5)
    assert select_target([cand], beyond).status == "none"
    # A fitted non-standard pallet is judged on its depth span only, not on the loose visible-points
    # fallback kept for uncertain candidates: an instruction 0.5 m to its side is somewhere else
    # (independent review 2026-10-16: it was selected).
    beside = dict(far_centre, x_m=face_x + 0.4, y_m=0.5)
    assert select_target([cand], beside).status == "none"
    # The possible centres form a segment, not a few samples: an instruction between two samples
    # still overlaps it (Codex 2026-10-16: 49 mm from the segment, 65.7 mm from the nearest sample).
    between = dict(far_centre, x_m=face_x + 0.38125, y_m=cand.face_centre_m[1] + 0.03)
    assert select_target([cand], between).status == "selected"


def test_the_parametric_reading_can_be_turned_off():
    recipe = _recipe("s7_nonstandard_1000738")
    out = instance_rig.render_instances([rs.recipe_boxes(recipe, 0)])
    cand = F.fit_instance(out.scene, InstanceProposal("p0", out.instance_ids == 0, 1.0, "oracle"), specs=SPECS,
                          setup=ForkSetup(recipe.fork_spacing_m), load_for=_load,
                          params=replace(F.FitParams(), parametric=False))
    assert cand.hypothesis != "parametric:x"


def _synthetic_face(top_row_fill, *, board_ends_m=0.0):
    """A 0.8 m face: three 0.1 m members below a deck from 0.07 to 0.115 m, rays behind the openings.

    board_ends_m: the top of the deck is that much of top-board ends (boards along the insertion
    axis, 0.085 m wide with 0.065 m gaps, fill about 0.6) above a full-width stringer."""
    rng = np.random.default_rng(1)
    u = rng.uniform(-0.4, 0.4, 40000)
    h = rng.uniform(0.0, 0.115, 40000)
    member = (np.abs(u) < 0.05) | (np.abs(np.abs(u) - 0.35) < 0.05)
    keep = member | (h >= 0.07)
    gone = rng.uniform(size=200) > top_row_fill  # ragged top edge: whole 5 mm bins missing in the top 10 mm
    keep &= ~((h >= 0.105) & gone[((u + 0.4) / 0.005).astype(int)])
    keep &= ~((h >= 0.115 - board_ends_m) & ((u + 0.4) % 0.15 >= 0.085))
    u, h = u[keep], h[keep]
    gu = rng.uniform(-0.45, 0.45, 4000)
    gh = rng.uniform(0.012, 0.06, 4000)
    hole = ~((np.abs(gu) < 0.05) | (np.abs(np.abs(gu) - 0.35) < 0.05)) | (np.abs(gu) > 0.4)
    gu, gh = gu[hole], gh[hole]
    ru, rh = np.concatenate([u, gu]), np.concatenate([h, gh])
    rc = np.concatenate([np.full(u.size, F.FACE), np.full(gu.size, F.BEHIND)])
    return u, h, (ru, rh, rc, np.ones(ru.size, bool))


def test_a_ragged_top_edge_does_not_lift_the_lane_ceiling_to_the_top():
    face = F.FacePlane(np.zeros(2), np.array([1.0, 0.0]), 0.002, np.arange(1))
    u, h, rays = _synthetic_face(top_row_fill=0.3)
    reading, why = F.read_parametric_face(face, u, h, rays, 0.0, F.FitParams())
    assert why is None
    assert reading.layout.lane_ceiling_m == pytest.approx(0.07, abs=0.006)
    assert len(reading.layout.member_intervals_m) == 3


def test_top_board_ends_above_a_stringer_are_part_of_the_deck():
    # S7 boards run along the insertion axis (tools/recognition_scenes.py): from the front the top
    # 12-25 mm is board ends with gaps above a full-width stringer (dev 2026-10-16: fill 0.64).
    face = F.FacePlane(np.zeros(2), np.array([1.0, 0.0]), 0.002, np.arange(1))
    u, h, rays = _synthetic_face(top_row_fill=0.3, board_ends_m=0.025)
    reading, why = F.read_parametric_face(face, u, h, rays, 0.0, F.FitParams())
    assert why is None
    assert reading.layout.lane_ceiling_m == pytest.approx(0.07, abs=0.006)


def _layout(openings, width, ceiling=0.08):
    edges = [-width / 2] + [e for o in openings for e in o] + [width / 2]
    members = tuple((edges[k], edges[k + 1]) for k in range(0, len(edges), 2))
    return F.FaceLayout("x", width, math.nan, members, tuple(openings), (0.0, ceiling), 0.0, ceiling)


def test_the_opening_pair_is_chosen_with_its_support():
    """Codex 2026-10-16: the widest-margin pair (1, 2) is 205 mm off centre and fails lateral support;
    pair (0, 1) passes both, so the face is not stopped."""
    layout = _layout([(-0.325, -0.135), (-0.05, 0.17), (0.24, 0.46)], 1.04)
    pair, pickable, reasons = F._parametric_fit(layout, ForkSetup(0.29), LoadDeclaration(2.0, 2.0, 0.2, 0.0), 1.2)
    assert pair == (0, 1) and pickable is None and reasons == ()


def test_a_load_beyond_any_insertion_depth_stops_the_pick_without_the_depth():
    # The insertion target is at most carriage - reserve = 0.33 m whatever the depth: a centre of
    # mass at 0.31 m needs 0.346 m of fork under it (design §9.3 support).
    layout = _layout([(-0.25, -0.05), (0.05, 0.25)], 0.7)
    _, pickable, reasons = F._parametric_fit(layout, ForkSetup(0.30), LoadDeclaration(2.0, 2.0, 0.31, 0.0), 1.2)
    assert pickable is False and reasons == ("support_unverified",)


def test_a_low_lane_is_left_to_the_near_range():
    """The far reading's rows are 5 mm and its error is not bounded, so the lift is not decided from
    it (Codex 2026-10-16): the lane bounds stay pending."""
    layout = _layout([(-0.25, -0.05), (0.05, 0.25)], 0.7, ceiling=0.045)
    _, pickable, reasons = F._parametric_fit(layout, ForkSetup(0.30), LoadDeclaration(2.0, 2.0, 0.2, 0.0), 1.2)
    assert pickable is None and reasons == ()


@pytest.mark.parametrize("load", [None, LoadDeclaration(2.0, 2.0, 0.2, 0.0, "site_default")])
def test_a_non_standard_pallet_without_an_instructed_load_is_not_picked(load):
    """A non-standard pallet has no template mass: without the instruction's load the support is
    unverified (fork_fit, §9.3), and the near range cannot supply it (Codex 2026-10-16)."""
    layout = _layout([(-0.25, -0.05), (0.05, 0.25)], 0.7)
    _, pickable, reasons = F._parametric_fit(layout, ForkSetup(0.30), load, 1.2)
    assert pickable is False and reasons == ("support_unverified",)


@pytest.mark.parametrize("opening_m, stopped", [(0.155, False), (0.115, True)])
def test_a_narrow_lane_stops_the_pick_only_well_below_the_requirement(opening_m, stopped):
    """Far-range margins read low under noise (dev 2026-10-16: up to -42.5 mm at the high provisional
    level, while pickable S7 pallets sit 4-6 mm above 66 mm), so the far range stops only below
    66 - 30 mm; between that and 66 mm the lane bounds wait for the near range, which applies 66 mm.
    30 mm is an empirical dev margin for early rejection, not a measured error bound (Codex)."""
    layout = _layout([(-0.15 - opening_m / 2, -0.15 + opening_m / 2), (0.15 - opening_m / 2, 0.15 + opening_m / 2)], 0.7)
    _, pickable, reasons = F._parametric_fit(layout, ForkSetup(0.30), LoadDeclaration(2.0, 2.0, 0.2, 0.0), 1.2)
    # margins: (0.155 - 0.055) / 2 = 50 mm (between 36 and 66), (0.115 - 0.055) / 2 = 30 mm (below 36)
    assert (pickable, reasons) == ((False, ("fork_spacing_adjust",)) if stopped else (None, ()))


def test_the_evaluator_gives_a_non_standard_reading_no_site_default():
    """The CLI's load_for is called with spec None for a parametric reading: it must return only an
    instructed load (or None), never look up a site default (it raised AttributeError, Codex 2026-10-16)."""
    from tools import evaluate_recognition as E
    site = {EPAL.name: SITE[EPAL.name]}
    assert E._load_for({"present": False}, site)(None) is None
    given = {"pallet_mass_kg": 2.0, "load_mass_kg": 1.0, "com_x_max_m": 0.2, "com_y_max_m": 0.0, "source": "instruction"}
    assert E._load_for({"present": True, "load": given}, site)(None) == LoadDeclaration(**given)


def test_a_mask_cut_through_two_touching_pallets_has_no_physical_ends():
    """Codex review 2026-10-16: two EPAL 6 touching at y = 0, proposed as |y| < 0.4525 m (the inner
    halves). Beyond each cut the members go on in the face plane, so a cut is not a physical end and
    the reading must not be confirmed (it was fitted at 0.947 and selected)."""
    boxes = [rt.placed_boxes(rt.instance_boxes(EPAL, ()), rt.Placement(3.0, y, 0.0, 0.0)) for y in (0.4, -0.4)]
    out = instance_rig.render_instances(boxes)
    rays = F.pixel_rays(out.scene)
    with np.errstate(invalid="ignore"):
        y = (F.camera_xyz(out.scene) + out.scene.depth_m[..., None] * rays)[..., 1]
    mask = (out.instance_ids >= 0) & (np.abs(y) < 0.4525)
    cand = F.fit_instance(out.scene, InstanceProposal("p0", mask, 1.0, "oracle"), specs=SPECS, setup=ForkSetup(0.3725),
                          load_for=_load)
    assert cand is None or not cand.counted, (cand.hypothesis, cand.geometry, cand.reasons)


def test_a_fitted_non_standard_pallet_is_not_selected_through_a_rejected_reading():
    """Codex review 2026-10-16: the centre of the known reading that was rejected stays among the
    alternatives for uncertain candidates, but a counted non-standard candidate is where its own
    depth segment says -- an instruction at the rejected centre, off the segment, is not it."""
    recipe = _recipe("s7_nonstandard_1000738")
    spec = recipe.instances[0].spec
    instructed = LoadDeclaration(2.0, 1.0, 0.2, 0.0)
    cand, _ = _fit_spec(spec, rt.Placement(3.0, 0.0, 0.0, 0.0), recipe.fork_spacing_m, load=lambda s: instructed)
    assert cand.counted and cand.hypothesis == "parametric:x"
    rejected = (cand.face_centre_m[0] + 0.4, cand.face_centre_m[1] + 0.3, 0.0)
    cand = replace(cand, alternative_centres_m=(rejected,))
    at_rejected = {"present": True, "x_m": rejected[0], "y_m": rejected[1], "radius_m": 0.05, "layer": None, "load": None}
    assert select_target([cand], at_rejected).status == "none"


def test_a_side_face_read_when_the_front_face_does_not_read_is_not_confirmed():
    """Codex review 2026-10-16: s7_nonstandard_1000873 is cut by the image edge; its front (entry)
    face has the most points but does not read, and the side face does -- it was fitted at -67.5 deg
    against the entry yaw 22.5 deg. Nothing on the side face says it is an entry face, so a reading
    off any face but the best-evidence one stays ambiguous (Isaac dev: every correct match was on
    that face; 1000873, 1000802 and 1000828 were not)."""
    recipe = _recipe("s7_nonstandard_1000873")
    out = instance_rig.render_instances([rs.recipe_boxes(recipe, k) for k in range(len(recipe.instances))])
    cand = F.fit_instance(out.scene, InstanceProposal("p0", out.instance_ids == 0, 1.0, "oracle"), specs=SPECS,
                          setup=ForkSetup(recipe.fork_spacing_m), load_for=lambda s: LoadDeclaration(2.0, 1.0, 0.2, 0.0))
    assert not cand.counted and cand.reasons[0] == "entry_face_unresolved"
