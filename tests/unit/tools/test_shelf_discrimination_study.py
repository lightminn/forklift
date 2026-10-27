import numpy as np
import pytest

from forklift_core.perception.bottom_deck_evidence import evaluate_side
from forklift_core.perception.pocket_detector import detect_pockets
from tools import bottom_deck_scenes as S
from tools import recognition_scenes as rs
from tools import shelf_discrimination_study as st


def test_every_front_structure_builds():
    for shape in ("epal6", "t11_06"):
        for kind in st.FRONT_STRUCTURES:
            assert st.boxes_for(shape, kind), (shape, kind)


def test_kinds_of_the_catalogues():
    n2 = next(r for r in rs.catalogue_bottomless() if r.case == "N2")
    assert st.kind_of(n2) == rs.n2_kind(n2.rank)
    s8 = next(r for r in rs.catalogue_bottomless() if r.case == "S8")
    assert st.kind_of(s8) == f"S8:{s8.shape_key}"


def test_the_side_renderer_takes_the_new_negatives():
    case = S.SideCase("t11_06", "shelf_closed", face=1, theta_e_deg=35.0, range_m=1.35, noise_seed=None)
    scene, target, _ = S.render_side(case)
    assert evaluate_side(scene, target, face=1).verdict in ("present", "absent", "unobserved")
    assert np.isfinite(np.asarray(scene.depth_m)).any()


@pytest.mark.parametrize("lower", (True, False))
def test_front_terms_are_the_detectors_own_selected_pattern(lower):
    # Codex round 2: grounded T11 at 2.0 m, yaw 0.26, team frame_640 seed 2 -- the detector selects a pattern with
    # upper 132 / 60, a first-two-gaps reading of the largest plane gave 60 / 0.
    prior, params = st.detector_setup("t11_06")
    scene = st.front_scene("t11_06", "grounded", 2.0, 0.26, "team_frame_640", 2)
    terms = st.selected_terms(scene, prior, params, lower)
    d = detect_pockets(scene, prior, params, lower_evidence=lower).diagnostics
    assert terms["replica_matches"]
    assert (terms["upper_left"], terms["upper_right"], terms["lower"], terms["support_min"]) == (
        d.selected_upper_left, d.selected_upper_right, d.selected_lower, d.selected_support_min)
    assert terms["upper_over_supports"] == pytest.approx(
        (terms["upper_left"] + terms["upper_right"]) / terms["support_sum"])
