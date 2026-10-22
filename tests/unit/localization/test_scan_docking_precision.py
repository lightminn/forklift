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


def test_a_repeated_pattern_inside_the_range_is_refused_as_not_unique():
    comb = wall(*[pilaster(-3.0 + 0.15 * k, width=0.075, depth=0.10) for k in range(40)])
    result, _ = run(comb)
    assert not result.accepted, result.reason


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
