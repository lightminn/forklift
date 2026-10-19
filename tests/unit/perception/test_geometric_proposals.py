# tests/unit/perception/test_geometric_proposals.py
"""The geometric fallback proposer (stage 3 plan, task 2) on the CPU rig."""
import numpy as np
import pytest

from forklift_core.perception import geometric_proposals as G
from forklift_core.perception.fork_fit import ForkSetup, LoadDeclaration
from forklift_core.perception.instance_fit import fit_scene, proposal_points, pixel_rays
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


def _render(items, distractors=()):
    placed = [rt.placed_boxes(rt.instance_boxes(s, ()), p) for s, p in items]
    out = instance_rig.render_instances(placed, distractors=distractors, back_wall=False)
    return out.scene, out.instance_ids


def _iou(a, b):
    return (a & b).sum() / max(1, (a | b).sum())


def test_one_pallet_gives_one_proposal_on_its_pixels():
    scene, ids = _render([(EPAL, rt.Placement(2.8, 0.0, 0.0, 0.0))])
    out = G.propose(scene)
    assert len(out.proposals) == 1 and out.proposals[0].source == "geometric"
    assert _iou(out.proposals[0].mask, ids == 0) >= 0.9


def test_two_pallets_front_and_back_are_two_proposals():
    scene, ids = _render([(EPAL, rt.Placement(2.4, 0.3, 0.0, 0.0)), (EPAL, rt.Placement(3.4, -0.3, 0.0, 0.0))])
    out = G.propose(scene)
    assert len(out.proposals) == 2
    assert max(_iou(p.mask, ids == 0) for p in out.proposals) >= 0.85
    assert max(_iou(p.mask, ids == 1) for p in out.proposals) >= 0.85


@pytest.mark.parametrize("items", [
    [(EPAL, rt.Placement(2.8, -0.31, 0.0, 0.0)), (EPAL, rt.Placement(2.8, 0.31, 0.0, 0.0))],  # touching side by side
    [(EPAL, rt.Placement(2.8, 0.0, 0.0, 0.0)), (EPAL, rt.Placement(2.8, 0.0, 0.0, EPAL.overall_height_m))],  # stack
])
def test_merged_pallets_stay_unconfirmed_and_are_never_selected(items):
    # Codex v1/v2: depth alone cannot split these; the safety contract is the identity
    scene, ids = _render(items)
    out = G.propose(scene)
    fitted = fit_scene(scene, out.proposals, specs=SPECS, setup=ForkSetup(0.3725), load_for=_load)
    assert all(c.identity == "unconfirmed" for c in fitted.candidates)
    assert all("geometric_unsplit" in out.warnings[p.proposal_id] for p in out.proposals)
    sel = select_target(fitted.candidates, {"present": True, "x_m": 3.0, "y_m": 0.0, "radius_m": 1.0},
                        unresolved=tuple(fitted.failures) + tuple(out.failures))
    assert sel.status != "selected"


def test_the_floor_alone_gives_nothing():
    scene, _ = _render([])
    assert G.propose(scene).proposals == []


def test_an_oversize_component_is_a_failure_not_a_proposal():
    wall = scene_rig.Box((3.0, 0.0, 0.3), (0.2, 3.0, 0.6), 0.0)
    scene, _ = _render([], distractors=[wall])
    out = G.propose(scene)
    assert out.proposals == [] and [f.reason for f in out.failures] == ["oversize"]


def test_dropout_holes_inside_the_mask_do_not_eat_the_fit_points():
    # Codex v1: 5 % invalid depth inside the face shrank the eroded points from 2,015 to 519
    scene, ids = _render([(T11, rt.Placement(2.4, 0.0, 0.0, 0.0))])
    rng = np.random.default_rng(3)
    depth = scene.depth_m.copy()
    holes = (ids == 0) & (rng.random(ids.shape) < 0.05)
    depth[holes] = np.nan
    from dataclasses import replace
    holed = replace(scene, depth_m=depth)
    out = G.propose(holed)
    rays = pixel_rays(holed)
    got, _ = proposal_points(holed, rays, out.proposals[0].mask, 3)
    want, _ = proposal_points(holed, rays, ids == 0, 3)
    assert len(got) >= 0.9 * len(want)


def test_the_proposals_are_capped_with_the_dropped_count():
    items = [(EPAL, rt.Placement(2.4, -0.9, 0.0, 0.0)), (EPAL, rt.Placement(2.4, 0.0, 0.0, 0.0)),
             (EPAL, rt.Placement(2.4, 0.9, 0.0, 0.0))]
    scene, _ = _render(items)
    assert len(G.propose(scene).proposals) == 3
    out = G.propose(scene, G.GeometricParams(max_proposals=2))
    assert len(out.proposals) == 2 and out.dropped == 1


def test_the_proposals_are_deterministic():
    scene, _ = _render([(EPAL, rt.Placement(2.4, 0.3, 0.0, 0.0)), (EPAL, rt.Placement(3.4, -0.3, 0.0, 0.0))])
    a, b = G.propose(scene), G.propose(scene)
    assert [p.proposal_id for p in a.proposals] == [p.proposal_id for p in b.proposals]
    assert all((p.mask == q.mask).all() for p, q in zip(a.proposals, b.proposals))
