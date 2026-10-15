# tests/unit/test_recognition_truth.py
import math
from pathlib import Path

import pytest

from forklift_core.perception.fork_fit import ForkSetup, LoadDeclaration
from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_model import KNOWN_ENTRY_AXES, spec_from_geometry
from tools import recognition_truth as rt
from tools import scene_rig

ROOT = Path(__file__).resolve().parents[2]
SITE = LoadDeclaration(10.0, 0.0, 0.3, 0.0, "site_default")  # EPAL 6 site default (config/site_load_defaults.yaml)


def _spec(name):
    g = load_pallet_geometry(ROOT / f"config/pallet_geometry_{name}.yaml")
    return g, spec_from_geometry(g, entry_axes=KNOWN_ENTRY_AXES[name])


def _truth(items, spacing=0.3725, **kw):
    return rt.scene_truths(items, ForkSetup(spacing), **kw)[0]


def test_x_face_pockets_match_scene_rig_true_pockets():
    g, spec = _spec("epal6")
    t = _truth([(spec, rt.Placement(2.5, 0.2, 0.1, 0.0), (), None)])[0]
    front = next(f for f in t.faces if f.axis == "x" and f.sign == 1)
    truth = scene_rig.true_pockets(g, x_m=2.5, y_m=0.2, yaw_rad=0.1)
    assert front.left_m == pytest.approx(tuple(truth[0]), abs=1e-9)
    assert front.right_m == pytest.approx(tuple(truth[1]), abs=1e-9)
    back = next(f for f in t.faces if f.axis == "x" and f.sign == -1)
    assert front.accessible and not back.accessible
    assert back.insertion_yaw_rad == pytest.approx(0.1 - math.pi)


def test_t11_faces_at_45_degrees_and_y_face_height():
    _, spec = _spec("t11_06")
    t = _truth([(spec, rt.Placement(2.5, 0.0, math.pi / 4, 0.0), (), None)], spacing=0.290)[0]
    assert len(t.faces) == 4 and sum(f.accessible for f in t.faces) == 2
    y_face = next(f for f in t.faces if f.axis == "y")
    assert y_face.left_m[2] == pytest.approx(0.045) and y_face.pocket_height_m == pytest.approx(0.060)


def test_stacked_layer_raises_the_pockets_and_needs_lift():
    _, spec = _spec("epal6")
    lower = (spec, rt.Placement(2.5, 0.0, 0.0, 0.0), (), SITE)
    upper = (spec, rt.Placement(2.5, 0.0, 0.0, 0.144), (), SITE)
    _, up_t = _truth([lower, upper], layers=[0, 1])
    front = up_t.faces[0]
    assert front.left_m[2] == pytest.approx(0.144 + 0.022 + 0.039)
    assert front.fit["reasons"] == ["lift_alignment_needed"] and up_t.lane_reachable
    assert front.fit["load_source"] == "site_default"
    assert up_t.support_visible


def test_touching_pallets_keep_their_own_face_exposure():
    _, spec = _spec("epal6")
    a, b = _truth([(spec, rt.Placement(2.5, 0.4, 0.0, 0.0), (), None),
                   (spec, rt.Placement(2.5, -0.4, 0.0, 0.0), (), None)])
    for t in (a, b):
        front = next(f for f in t.faces if f.accessible)
        assert 0.95 < front.exposure <= 1.0 and t.eligible


def test_a_low_screen_in_front_cuts_face_exposure_below_half():
    _, spec = _spec("epal6")
    screen = [scene_rig.Box((2.5 - 0.3 - 0.05, 0.0, 0.0525), (0.01, 1.0, 0.105))]
    t = _truth([(spec, rt.Placement(2.5, 0.0, 0.0, 0.0), (), None)], distractors=screen)[0]
    front = next(f for f in t.faces if f.accessible)
    assert front.exposure < 0.5 and not t.eligible


def test_support_surface_above_the_camera_is_not_visible():
    _, spec = _spec("epal6")
    tiers = [(spec, rt.Placement(2.5, 0.0, 0.0, k * 0.144), (), None) for k in range(3)]
    truths = _truth(tiers, layers=[0, 1, 2])
    assert truths[0].support_visible and not truths[2].support_visible


def test_truth_can_be_computed_at_another_camera_k():
    from dataclasses import replace

    from tools import generate_recognition_scenes as gen
    from tools import recognition_scenes as rs

    recipe = next(r for r in rs.catalogue() if r.case == "S3" and r.split == "dev")
    base = gen.recipe_record(recipe, gen.load_site())
    k = base.rendered.scene.intrinsics
    shifted = gen.recipe_record(recipe, gen.load_site(), intrinsics=replace(k, cx=k.cx - 0.5, cy=k.cy - 0.5))
    assert shifted.rendered.scene.intrinsics.cx == k.cx - 0.5
    assert [f.exposure for t in base.truths for f in t.faces] != [f.exposure for t in shifted.truths for f in t.faces]
