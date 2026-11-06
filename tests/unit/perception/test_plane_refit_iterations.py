"""Plane refit iterations (plan 2026-11-06 near-capture plane refit, arm (a)): K = 0 is the frozen extraction, K > 0
refits on the band it collected until the band stops changing, and every round says why it stopped."""

from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception import pocket_detector as det
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import (
    DetectorParams,
    _refine_vertical_plane,
    _vertical_plane_candidates,
)

REPO_PRIOR = Path(__file__).resolve().parents[3] / "config" / "pallet_prior_v1.yaml"
BAND = 0.0078
CAMERA = np.array([0.0, 0.0, 0.9])
BLOCKS = ((-0.45, -0.35), (-0.12, 0.02), (0.24, 0.35))


def _face_and_deck(seed=0):
    """A pallet-like front (three block faces and the top deck's front edge at x = 3) and the deck's top surface
    behind it, seen from above: the horizontal surface gives a tilted vertical band points at every lateral."""
    rng = np.random.default_rng(seed)
    parts = []
    for lo, hi in BLOCKS:
        y, z = np.meshgrid(np.linspace(lo, hi, 12), np.linspace(0.03, 0.11, 10))
        parts.append(np.column_stack((3.0 + rng.normal(0, 0.002, y.size), y.ravel(), z.ravel())))
    y, z = np.meshgrid(np.linspace(-0.45, 0.35, 80), np.linspace(0.12, 0.144, 3))
    parts.append(np.column_stack((3.0 + rng.normal(0, 0.002, y.size), y.ravel(), z.ravel())))
    x, y = np.meshgrid(np.linspace(3.0, 3.5, 250), np.linspace(-0.45, 0.35, 80))
    parts.append(np.column_stack((x.ravel(), y.ravel(), 0.144 + rng.normal(0, 0.002, x.size))))
    return np.vstack(parts)


def _tilted_band(points, degrees=1.17):
    tilt = np.radians(degrees)
    normal = np.array([-np.cos(tilt), -np.sin(tilt), 0.0])
    return np.abs((points - np.array([3.0, -0.40, 0.07])) @ normal) <= BAND


def _yaw_deg(plane):
    return abs(np.degrees(np.arctan2(plane.normal[1], -plane.normal[0])))


def _params(**overrides):
    return DetectorParams(plane_inlier_m=BAND, min_plane_points=30, **overrides)


def test_zero_refit_iterations_is_the_frozen_extraction():
    points = _face_and_deck()
    frozen = _vertical_plane_candidates(points, CAMERA, _params())
    explicit = _vertical_plane_candidates(points, CAMERA, _params(), refit_iterations=0)
    assert len(frozen) == len(explicit) > 0
    for a, b in zip(frozen, explicit):
        np.testing.assert_array_equal(a.point, b.point)
        np.testing.assert_array_equal(a.normal, b.normal)
        np.testing.assert_array_equal(a.points, b.points)
        assert a.residual_p95_m == b.residual_p95_m


def test_refit_iterations_take_a_tilted_first_fit_back_to_the_face():
    """One refit of a 1.17 deg hypothesis band leaves about a third of a degree; iterating converges to the face."""
    points = _face_and_deck()
    first = _tilted_band(points)
    once, _, record0 = _refine_vertical_plane(points, CAMERA, points[first], _params(), 0)
    iterated, mask, record5 = _refine_vertical_plane(points, CAMERA, points[first], _params(), 5)
    assert _yaw_deg(once) > 0.25
    assert _yaw_deg(iterated) < 0.05
    assert record0["stop"] == "frozen" and record0["iterations"] == 0
    assert record5["stop"] == "converged" and 1 <= record5["iterations"] <= 5
    assert int(mask.sum()) == len(iterated.points)


def test_a_refit_that_would_drop_below_the_minimum_keeps_the_previous_pair():
    points = _face_and_deck()
    first = _tilted_band(points)
    plane0, mask0, _ = _refine_vertical_plane(points, CAMERA, points[first], _params(), 0)
    # the next band holds fewer points than this minimum: the refit stops on the pair it had
    params = _params()
    calls = []
    real = det._refit_vertical

    def shrinking(pts, camera, **kw):
        calls.append(len(pts))
        plane = real(pts, camera, **kw)
        if len(calls) >= 2:  # move the second refit far off the data, so its band is empty
            return det._Plane(plane.point + 10 * plane.normal, plane.normal, plane.points, plane.residual_p95_m)
        return plane

    det._refit_vertical = shrinking
    try:
        plane, mask, record = _refine_vertical_plane(points, CAMERA, points[first], params, 5)
    finally:
        det._refit_vertical = real
    assert record["stop"] == "too_few" and record["iterations"] == 0
    np.testing.assert_array_equal(mask, mask0)
    np.testing.assert_allclose(plane.normal, plane0.normal)


def test_the_cap_stops_a_refit_that_is_still_moving():
    points = _face_and_deck()
    _, _, record = _refine_vertical_plane(points, CAMERA, points[_tilted_band(points)], _params(), 1)
    assert record["stop"] == "cap" and record["iterations"] == 1


def test_a_band_that_comes_back_to_an_earlier_set_stops_as_a_cycle(monkeypatch):
    points = _face_and_deck()
    first = _tilted_band(points)
    real = det._refit_vertical
    tilts = iter([0.0, 0.6, 0.0, 0.6, 0.0, 0.6, 0.0])

    def alternating(pts, camera, **kw):
        tilt = np.radians(next(tilts))
        normal = np.array([-np.cos(tilt), -np.sin(tilt), 0.0])
        plane = real(pts, camera, **kw)
        return det._Plane(np.array([3.0, -0.05, 0.07]), normal, plane.points, plane.residual_p95_m)

    monkeypatch.setattr(det, "_refit_vertical", alternating)
    _, _, record = _refine_vertical_plane(points, CAMERA, points[first], _params(), 5)
    assert record["stop"] == "cycle" and record["iterations"] == 2


def test_parallel_layers_are_reported_as_lost_support_and_inflow():
    """Codex review counterexample: layers 6 mm apart with 1:10:100 points let the band walk to the dense layer with
    no change of normal; the record must show the original support lost."""
    rng = np.random.default_rng(3)
    layers = []
    for depth, count in ((0.0, 40), (0.006, 400), (0.012, 4000)):
        y = rng.uniform(-0.4, 0.4, count)
        z = rng.uniform(0.03, 0.11, count)
        layers.append(np.column_stack((np.full(count, 3.0 + depth), y, z)))
    points = np.vstack(layers)
    first = np.abs(points[:, 0] - 3.0) <= BAND
    plane, mask, record = _refine_vertical_plane(points, CAMERA, points[first], _params(), 5)
    # S0 (the band of the first refit, about 5.5 mm in) still holds all three layers; the walk to the dense layer
    # then drops the front layer entirely with no change of normal -- below the 'large model change' thresholds
    assert record["sizes"][0] == len(points)
    assert not mask[:40].any()
    assert record["support_loss"] == pytest.approx(40 / len(points))
    assert record["inflow"] == 0.0
    assert record["normal_change_deg"] < 0.01  # sampling only: the layers are parallel
    assert 0.005 < record["offset_change_m"] < 2 * BAND
    assert plane.point[0] > 3.009


def test_detect_pockets_expected_passes_the_refit_count_and_the_log(monkeypatch, pallet_scene):
    seen = []
    real = det._vertical_plane_candidates

    def spy(points, camera, params, **kw):
        seen.append(kw)
        return real(points, camera, params, **kw)

    monkeypatch.setattr(det, "_vertical_plane_candidates", spy)
    scene, _ = pallet_scene(centre_xy_m=(2.5, 0.0), yaw_rad=0.0, opening_width_m=0.24)
    prior = load_pallet_prior(REPO_PRIOR)
    log = []
    det.detect_pockets_expected(scene, prior, DetectorParams(), None, plane_refit_iterations=5, refit_log=log)
    assert seen and seen[-1]["refit_iterations"] == 5
    assert seen[-1]["refit_log"] is log and log  # one record per extracted round
    seen.clear()
    det.detect_pockets(scene, prior, DetectorParams())
    assert seen and seen[-1] == {}  # the frozen call passes nothing new
    seen.clear()
    det.detect_pockets_expected(scene, prior, DetectorParams(), None)
    assert seen and seen[-1] == {}


def test_refit_iterations_must_be_a_non_negative_integer():
    points = _face_and_deck()
    for bad in (-1, 1.5, True):
        with pytest.raises(ValueError):
            _vertical_plane_candidates(points, CAMERA, _params(), refit_iterations=bad)
