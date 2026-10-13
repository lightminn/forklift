"""2D scan-to-reference matching for destination docking (plan v3.8)."""

import math
from pathlib import Path

import numpy as np
import pytest

from forklift_core.localization.scan_match import _midpoint, _two_cycle, match_scans, transform_points
from forklift_core.localization.slam_pose import compose, invert


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


def test_empty_or_tiny_inputs_are_refused_not_crashed():
    rng = np.random.default_rng(3)
    reference = _walls(rng)
    assert match_scans(np.empty((0, 2)), reference).reason == "too_few_points"
    assert match_scans(reference, np.empty((0, 2))).reason == "too_few_points"


def test_a_run_that_hits_the_iteration_cap_is_not_accepted():
    rng = np.random.default_rng(4)
    reference = _walls(rng)
    live = transform_points(reference, (0.2, 0.1, 0.05), inverse=True)
    assert match_scans(reference, live, max_iterations=1).reason == "not_converged"
    with pytest.raises(ValueError):
        match_scans(reference, live, max_iterations=0)


# A recorded delivery match (tests/fixtures/delivery_match/README.md): every start falls into
# the same period-two cycle, two correspondences trading places each iteration (2026-10-13).
FIXTURE = Path(__file__).resolve().parents[2] / "fixtures/delivery_match/seed6_cycle.npz"


def _recorded():
    z = np.load(FIXTURE)
    x = tuple(z["rear_from_laser"])
    planned = compose(invert(tuple(z["seed"])), tuple(z["estimate"]))
    initial = compose(compose(invert(x), planned), x)
    truth = compose(compose(invert(x), compose(invert(tuple(z["reference_rear"])), tuple(z["truth_rear"]))), x)
    return z["reference"], z["live"], initial, truth


def test_a_recorded_two_cycle_settles_at_the_midpoint_of_its_states():
    reference, live, initial, truth = _recorded()
    result = match_scans(reference, live, initial=initial)
    assert result.accepted and result.reason == "ok", result
    half_m, half_rad, detected = result.cycle
    assert half_m == pytest.approx(5.4716e-5, abs=1e-8) and half_rad == pytest.approx(7.745e-6, abs=1e-9)
    assert detected == result.iterations <= 12
    # The midpoint is the usual match bias away from the truth, well inside a millimetre.
    assert math.hypot(result.pose[0] - truth[0], result.pose[1] - truth[1]) < 1e-3
    assert abs(result.pose[2] - truth[2]) < 1e-4
    # With no room for any cycle the same input is refused as one, standing on the last state.
    wide = match_scans(reference, live, initial=initial, max_cycle_half_m=-1.0)
    assert wide.reason == "wide_cycle" and wide.iterations == detected
    # The settled pose is the midpoint of the cycle's two states (iterations k and k - 1).
    before = match_scans(reference, live, initial=initial, max_iterations=detected - 1)
    assert before.reason == "not_converged" and before.cycle is None
    np.testing.assert_allclose(result.pose, _midpoint(wide.pose, before.pose), rtol=0, atol=1e-12)
    assert math.hypot(wide.pose[0] - before.pose[0], wide.pose[1] - before.pose[1]) == pytest.approx(2 * half_m)


def test_a_cycle_wider_than_the_bound_is_refused_as_wide():
    reference, live, initial, _ = _recorded()
    result = match_scans(reference, live, initial=initial, max_cycle_half_m=5e-5)
    assert not result.accepted and result.reason == "wide_cycle"
    assert result.cycle[0] == pytest.approx(5.4716e-5, abs=1e-8)
    result = match_scans(reference, live, initial=initial, max_cycle_half_rad=7e-6)
    assert not result.accepted and result.reason == "wide_cycle"


def test_a_converging_run_records_no_cycle():
    rng = np.random.default_rng(0)
    reference = _walls(rng)
    live = transform_points(_walls(rng, 0.01), (0.08, -0.05, 0.03), inverse=True)
    result = match_scans(reference, live)
    assert result.accepted and result.cycle is None


def _pairs(n, flip):
    # The correspondences of one iteration: nearest indices and the in-range mask.
    index = np.arange(n)
    if flip:
        index = index.copy()
        index[0] = 1
    return index, np.ones(n, dtype=bool)


def test_only_a_repeat_of_four_poses_with_the_same_correspondences_is_a_two_cycle():
    a, b = np.array([1.0, 2.0, 0.3]), np.array([1.0001, 2.0, 0.30001])
    poses = [a, b, a.copy(), b.copy()]
    pairs = [_pairs(5, False), _pairs(5, True), _pairs(5, False), _pairs(5, True)]
    assert _two_cycle(poses, pairs)
    assert not _two_cycle(poses[1:], pairs[1:])  # three iterations are not enough
    # The poses repeat but one iteration used other correspondences.
    assert not _two_cycle(poses, [_pairs(5, False), _pairs(5, True), _pairs(5, True), _pairs(5, True)])
    # Only the last pair repeats.
    assert not _two_cycle([a, b, a.copy(), b + [1e-6, 0, 0]], pairs)
    # A repeat to 1e-6 m is an approach, not a cycle (the step test's 1e-5 m would take it).
    assert not _two_cycle([a, b, a + [1e-6, 0, 0], b.copy()], pairs)


def test_a_longer_period_or_a_slow_decay_is_not_taken_for_a_two_cycle():
    same = [_pairs(3, False)] * 20
    period5 = [np.array([v * 1e-3, 0.0, 0.0]) for v in [0, 0.8, 0.005, 2.5, -2.5] * 3]
    assert not any(_two_cycle(period5[:k], same[:k]) for k in range(4, len(period5) + 1))
    decay, x, yaw = [], 5e-3, 4e-4
    for _ in range(20):
        x, yaw = 0.999 * x, -0.999 * yaw
        decay.append(np.array([x, 0.0, yaw]))
    assert not any(_two_cycle(decay[:k], same[:k]) for k in range(4, len(decay) + 1))


def test_the_cycle_states_are_compared_and_averaged_across_the_yaw_seam():
    a, b = np.array([0.0, 0.0, math.pi - 1e-5]), np.array([0.0, 0.0, -math.pi + 1e-5])
    pairs = [_pairs(3, False), _pairs(3, True)] * 2
    assert _two_cycle([a, b, a + [0, 0, 2 * math.pi], b.copy()], pairs)
    mid = _midpoint(a, b)
    assert abs(math.atan2(math.sin(mid[2] - math.pi), math.cos(mid[2] - math.pi))) < 1e-12
