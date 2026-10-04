"""2D scan-to-reference matching for destination docking (plan v3.8)."""

import math

import numpy as np
import pytest

from forklift_core.localization.scan_match import match_scans, transform_points


def _walls(rng, noise=0.0):
    # An L corner plus a box: enough structure to fix x, y and yaw.
    xs = np.linspace(-3, 3, 400)
    ys = np.linspace(-2, 4, 400)
    pts = [np.column_stack((xs, np.full_like(xs, 4.0))), np.column_stack((np.full_like(ys, 3.0), ys))]
    bx = np.linspace(-1.0, -0.4, 60)
    by = np.linspace(1.0, 1.6, 60)
    pts += [np.column_stack((bx, np.full_like(bx, 1.0))), np.column_stack((np.full_like(by, -1.0), by))]
    p = np.vstack(pts)
    return p + rng.normal(0, noise, p.shape)


def test_a_known_offset_is_recovered_on_a_corner():
    rng = np.random.default_rng(0)
    reference = _walls(rng)
    true = (0.08, -0.05, 0.03)  # pose of the live scan frame in the reference frame
    live = transform_points(_walls(rng, 0.01), true, inverse=True)
    result = match_scans(reference, live)
    assert result.accepted, result
    np.testing.assert_allclose(result.pose[:2], true[:2], atol=0.005)
    assert result.pose[2] == pytest.approx(true[2], abs=0.002)


def test_a_single_wall_is_reported_degenerate():
    rng = np.random.default_rng(1)
    xs = np.linspace(-3, 3, 400)
    reference = np.column_stack((xs, np.full_like(xs, 4.0)))
    live = transform_points(reference + rng.normal(0, 0.005, reference.shape), (0.1, 0.0, 0.0), inverse=True)
    result = match_scans(reference, live)
    assert not result.accepted and result.reason == "degenerate"


def test_unrelated_scans_are_rejected():
    rng = np.random.default_rng(2)
    reference = _walls(rng)
    live = rng.uniform(-5, 5, (500, 2))
    result = match_scans(reference, live)
    assert not result.accepted


def test_transform_points_round_trips():
    p = np.array([[1.0, 2.0], [-0.5, 0.3]])
    pose = (0.2, -0.1, math.pi / 7)
    np.testing.assert_allclose(transform_points(transform_points(p, pose), pose, inverse=True), p, atol=1e-12)
