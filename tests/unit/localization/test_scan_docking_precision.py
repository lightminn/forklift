"""The sim-only weak-direction acceptance of a delivery docking match (docs/plans/2026-10-22-open-zone-docking.md D2):
a single far wall with a few sideways-facing faces is refused as degenerate by the information gate; with
acceptance="precision" it is taken when the conditional precision, the face-deletion, the range and the bound checks
pass. Two of the tests show what it cannot catch (faces moved together, a repeated pattern) -- the documented risk."""

import math

import numpy as np
import pytest

from forklift_core.localization.scan_docking import dock, laser_points, ray_scan
from forklift_core.localization.slam_pose import compose, invert

LASER = (-0.12 + 0.34, 0.0, 0.0)
ANGLES = -math.pi + np.arange(1600) * (2 * math.pi / 1600)
DELIVERY = (0.0, 0.0, 0.0)


def pilaster(y0: float, width: float = 0.60, depth: float = 0.40, wall_x: float = 5.0, shift: float = 0.0):
    """A pilaster standing out of the wall towards the truck: its two side faces look along +-y."""
    y0 += shift
    return [((wall_x, y0), (wall_x - depth, y0)), ((wall_x - depth, y0), (wall_x - depth, y0 + width)),
            ((wall_x - depth, y0 + width), (wall_x, y0 + width))]


def wall(*features):
    segs = [((5.0, -10.0), (5.0, 10.0))]
    for f in features:
        segs += f
    return segs


def scan(segments, rear_world, noise=0.0, seed=0):
    laser_world = compose(rear_world, LASER)
    ranges = ray_scan(segments, laser_world, ANGLES, 12.0)
    noisy = ranges + np.random.default_rng(seed).normal(0, noise, ranges.shape)
    return laser_points(np.where(np.isfinite(ranges), noisy, ranges), ANGLES)


def run(ref_segments, live_segments=None, *, slam_error=(0.0, 0.05, 0.0), acceptance="precision", seed=1):
    reference = scan(ref_segments, DELIVERY)
    truth_now = compose(DELIVERY, (-1.5, 0.0, 0.0))
    estimate_now = compose(truth_now, slam_error)
    live = scan(live_segments or ref_segments, truth_now, noise=0.02, seed=seed)
    kwargs = {} if acceptance is None else {"acceptance": acceptance}
    result = dock(reference, live, rear_from_laser=LASER, estimate_rear=estimate_now, delivery_rear=DELIVERY, **kwargs)
    expected = compose(estimate_now, compose(invert(truth_now), DELIVERY))
    error = None if result.goal_estimate is None else math.hypot(result.goal_estimate[0] - expected[0],
                                                                 result.goal_estimate[1] - expected[1])
    return result, error


TWO = wall(pilaster(-4.0), pilaster(3.5))


def test_two_sideways_faces_are_refused_by_the_information_gate_and_taken_by_precision():
    plain, _ = run(TWO, acceptance="information")
    assert plain.reason == "start_rejected:degenerate"
    taken, error = run(TWO)
    assert taken.accepted and taken.reason == "ok:precision", taken.reason
    assert error < 0.005
    record = taken.precision
    assert record["sigma_m"][1] <= 0.005 and record["groups"] >= 2 and record["range_agree"] is True


def test_a_wall_alone_is_refused_in_either_mode():
    plain, _ = run(wall(), acceptance="information")
    assert plain.reason == "start_rejected:degenerate"  # as today: the degenerate gate comes first
    weak, _ = run(wall(), acceptance="precision")
    assert not weak.accepted and weak.reason == "ambiguous", weak.reason  # the starts slide along the wall


def test_one_sideways_feature_is_not_enough():
    result, _ = run(wall(pilaster(-4.0)))
    assert not result.accepted and result.reason.startswith("degenerate:"), result.reason


def test_a_face_moved_alone_is_caught_by_the_deletion_check():
    moved = wall(pilaster(-4.0), pilaster(3.5, shift=0.04))  # one pilaster 4 cm off where it was taught
    result, _ = run(TWO, moved)
    assert not result.accepted and result.reason in ("degenerate:group_shift", "degenerate:sd"), result.reason


def test_faces_moved_together_are_taken_wrongly__the_documented_risk():
    together = wall(pilaster(-4.0, shift=0.10), pilaster(3.5, shift=0.10))
    result, error = run(TWO, together)
    assert result.accepted and result.reason == "ok:precision"
    assert error == pytest.approx(0.10, abs=0.01)  # the goal is placed 10 cm off: the check cannot see it


def test_a_repeated_pattern_finer_than_the_starts_is_ambiguous():
    comb = wall(*[pilaster(-3.0 + 0.15 * k, width=0.075, depth=0.10) for k in range(40)])
    result, _ = run(comb)
    assert not result.accepted and result.reason == "ambiguous", result.reason


@pytest.mark.parametrize("prior_dy, reason", [(0.0, "ok"), (0.12, "not_unique")])
def test_the_range_check_about_the_prior_finds_the_next_tooth(prior_dy, reason):
    # teeth 0.35 m apart, checked directly (the dock's own starts already disagree on such a pattern): the range
    # +-0.20 m about a prior 0.12 m off reaches the next tooth; about the true pose it does not (Codex P2-2: the range
    # is centred on the prior, not on the solved pose)
    from forklift_core.localization.scan_match import match_scans, precision_check

    comb = wall(*[pilaster(-3.0 + 0.35 * k, width=0.15, depth=0.30) for k in range(17)])
    reference = scan(comb, DELIVERY)
    truth_now = compose(DELIVERY, (-1.5, 0.0, 0.0))
    live = scan(comb, truth_now, noise=0.02, seed=1)
    q_true = compose(compose(invert(LASER), compose(invert(DELIVERY), truth_now)), LASER)
    pose = match_scans(reference, live, initial=q_true, min_information=0.0).pose
    check = precision_check(reference, live, pose, (q_true[0], q_true[1] + prior_dy, q_true[2]))
    assert check.reason == reason, (check.reason, check.record.get("range_ends"))


def test_the_live_points_order_does_not_change_the_result():
    # Codex implementation review P2-1: the facet lookup used the inlier rows' index into the full live array
    reference = scan(TWO, DELIVERY)
    truth_now = compose(DELIVERY, (-1.5, 0.0, 0.0))
    estimate_now = compose(truth_now, (0.0, 0.05, 0.0))
    live = scan(TWO, truth_now, noise=0.02, seed=1)
    reasons = set()
    for seed in range(3):
        order = np.random.default_rng(seed).permutation(len(live))
        r = dock(reference, live[order], rear_from_laser=LASER, estimate_rear=estimate_now, delivery_rear=DELIVERY,
                 acceptance="precision")
        reasons.add(r.reason)
    assert reasons == {"ok:precision"}


def test_a_range_start_that_fails_its_own_solve_is_not_agreement(monkeypatch):
    from dataclasses import replace

    from forklift_core.localization import scan_match as SM

    real = SM.match_scans
    monkeypatch.setattr(SM, "match_scans", lambda *a, **k: replace(real(*a, **k), accepted=False, reason="not_converged"))
    reference = scan(TWO, DELIVERY)
    live = scan(TWO, compose(DELIVERY, (-1.5, 0.0, 0.0)), noise=0.02, seed=1)
    pose = real(reference, live, initial=(-1.5, 0.0, 0.0), min_information=0.0).pose
    agree, _, reasons = SM._range_check(reference, live, pose, (0.0, 1.0, 0.0), pose,
                                        dict(min_information=0.0), 0.3)
    assert agree is False and set(reasons) == {"not_converged"}


def test_non_finite_diagnostics_are_recorded_as_none():
    from forklift_core.localization.scan_match import _finite_or_none

    assert _finite_or_none([float("inf"), 0.003, float("nan")]) == [None, 0.003, None]
    import json

    taken, _ = run(TWO)
    json.dumps(taken.precision, allow_nan=False)  # the runner's record_json would raise on inf


def test_a_well_constrained_match_is_the_same_in_either_mode():
    corner = [((-6.0, 3.0), (6.0, 3.0)), ((5.0, -4.0), (5.0, 3.0)), ((-2.0, -1.5), (-1.2, -1.5)),
              ((-1.2, -1.5), (-1.2, -0.7)), ((1.5, 1.0), (2.3, 1.0)), ((2.3, 1.0), (2.3, 1.6))]
    a, _ = run(corner, acceptance="information")
    b, _ = run(corner)
    assert a.accepted and b.accepted and a.reason == b.reason == "ok"
    assert a.goal_estimate == b.goal_estimate and b.precision is None


def test_without_the_argument_the_result_is_the_information_one_and_records_nothing_new():
    a, _ = run(TWO, acceptance=None)
    b, _ = run(TWO, acceptance="information")
    assert a == b and a.precision is None
    import dataclasses

    from forklift_core.mission.replay_codec import recorded_fields

    assert "precision" not in {f.name for f in recorded_fields(a)}


def test_a_correction_beyond_the_weak_bound_is_refused():
    result, _ = run(TWO, slam_error=(0.0, 0.25, 0.0))  # 0.25 m sideways: inside the 0.3 m bound, outside 0.20 m
    assert not result.accepted and result.reason in ("degenerate:bound", "correction_out_of_bounds"), result.reason


def test_an_unknown_acceptance_is_refused():
    with pytest.raises(ValueError):
        run(TWO, acceptance="lenient")


def test_a_singular_decimated_sigma_is_recorded_as_none_through_dock(monkeypatch):
    # Codex re-review P3: the whole dock() record, with the decimated information made singular
    import json

    from forklift_core.localization import scan_match as SM

    real, calls = SM._sigma, []

    def sigma(jac, residual):
        calls.append(len(jac))
        if len(calls) % 2 == 0:  # the second call of each check is the 1/4-decimated one
            return 0.01, (float("inf"),) * 3
        return real(jac, residual)

    monkeypatch.setattr(SM, "_sigma", sigma)
    taken, _ = run(TWO)
    assert taken.reason == "ok:precision" and taken.precision["sigma_decimated_m"] == [None, None, None]
    json.dumps(taken.precision, allow_nan=False)
