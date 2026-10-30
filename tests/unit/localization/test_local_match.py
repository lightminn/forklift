"""F1's local re-match (docs/plans/2026-10-31-operator-placed-start.md 1.3): the lattice search restricted to a window
around the operator's placed base pose, the refined top checked for fit, refinement movement, the window's edge
bands and ambiguity -- every pre-suppression leaf outside the success tolerance of the refined pose scoring >= R_AMB
of the lattice best refuses."""

import math

import numpy as np
import pytest
import test_global_match as TG

from forklift_core.localization import global_match as G
from forklift_core.localization import local_match as L
from forklift_core.localization import start_reloc as R

CFG = G.MatchConfig(yaw_step_rad=math.radians(2.0))


def _points_at(grid, base):
    return TG._points(grid, R.laser_from_base(base))


def _close(a, b, m=0.10, rad=0.05):
    return math.hypot(a[0] - b[0], a[1] - b[1]) <= m and abs(L.wrap(a[2] - b[2])) <= rad


def _run(grid, truth, click, **kw):
    window = L.Window(base=click, **kw.pop("window", {}))
    return L.local_match(grid, _points_at(grid, truth), window, CFG, e_min=0.8, r_amb=kw.pop("r_amb", 0.9), **kw)


def test_a_click_near_the_truth_in_an_asymmetric_room_confirms_it():
    grid = TG._grid(*TG.ROOM)
    truth = (2.3, 1.7, 0.6)
    out = _run(grid, truth, (2.8, 1.4, 0.8))
    assert out.confirmed and out.reason == "confirmed" and _close(out.base, truth)


def test_the_heading_window_leaves_out_the_turned_twin_of_a_symmetric_room():
    grid = TG._grid(6.0, 4.0, [])  # rotationally symmetric: the global match competes with the turned twin
    truth = (2.0, 1.4, 0.3)
    glob = G.refine(grid, G.global_match(grid, _points_at(grid, truth), CFG), _points_at(grid, truth), CFG)
    assert not G.decide(glob, e_min=0.8, r_max=0.95).confirmed
    out = _run(grid, truth, (2.4, 1.6, 0.5))
    assert out.confirmed and _close(out.base, truth)


def test_a_corridor_with_nothing_along_it_is_ambiguous():
    grid = TG._grid(30.0, 2.4, [])  # the ends lie beyond the match range from its middle
    truth = (15.0, 1.2, 0.0)
    out = _run(grid, truth, (15.3, 1.1, 0.1))
    assert not out.confirmed and out.reason == "ambiguous" and out.amb_ratio >= 0.9


def test_a_top_in_the_edge_band_is_refused():
    grid = TG._grid(*TG.ROOM)
    truth = (2.3, 1.7, 0.6)
    out = _run(grid, truth, (2.3 + 1.85, 1.7, 0.6))  # the truth 1.85 m away: inside the window, in its 0.3 m band
    assert not out.confirmed and out.reason == "edge"
    out = _run(grid, truth, (2.3, 1.7, 0.6 + 0.55))  # 0.55 rad: inside +-0.6, in its 0.1 rad band
    assert not out.confirmed and out.reason == "edge"


def test_a_click_far_from_the_truth_never_confirms_the_truth_or_a_pose_outside_the_window():
    grid = TG._grid(*TG.ROOM)
    truth = (2.3, 1.7, 0.6)
    click = (2.3 + 3.0, 1.7, 0.6)
    out = _run(grid, truth, click)
    if out.confirmed:  # whatever it confirms lies inside the window, never beyond it
        assert math.hypot(out.base[0] - click[0], out.base[1] - click[1]) <= 2.0 - 0.3
    assert not (out.confirmed and _close(out.base, truth))


def test_the_ambiguity_is_measured_from_the_refined_pose_against_the_lattice_best():
    # Codex F1 v2 review: lattice top (0, 0, 0), refined (0.09, 0, 0.04), an equal leaf at (-0.05, 0, -0.04363):
    # 0.05 m / 0.044 rad from the lattice top (inside the success tolerance) but 0.14 m / 0.084 rad from the refined pose
    bases = np.array([[0.0, 0.0, 0.0], [-0.05, 0.0, -0.04363]])
    scores = np.array([1.0, 1.0])
    ratio = L.ambiguity_ratio(bases, scores, best=1.0, refined=(0.09, 0.0, 0.04), success_m=0.10, success_rad=0.05)
    assert ratio == pytest.approx(1.0)  # the competitor counts, and the top's own leaf (0.09 m, 0.04 rad) does not
    ratio = L.ambiguity_ratio(bases, scores, best=1.0, refined=(0.0, 0.0, 0.0), success_m=0.10, success_rad=0.05)
    assert ratio == 0.0


def test_the_global_match_is_unchanged():
    grid = TG._grid(*TG.ROOM)
    truth = (2.3, 1.7, 0.6)
    pts = TG._points(grid, truth)
    match = G.global_match(grid, pts, CFG)
    assert match.status == "ok" and _close(match.candidates[0].pose, truth)
