"""Plan D8c near-field tracking: the D8e observation speed budget (CPU)."""

import math

import pytest

from forklift_core.perception.near_field_tracking import RoofHandoff, budget_speed

T, L_MAX, DECEL = 0.1, 0.0917, 0.3
# The runner's braking: the drive permission's stopping model (1.5 m/s^2, 0.15 s latency).
STOP_DECEL, STOP_LATENCY = 1.5, 0.15


def test_the_budget_holds_with_equality_and_braking():
    v = budget_speed(0.0, T + L_MAX, DECEL)
    assert v * (T + L_MAX) + v * v / (2 * DECEL) == pytest.approx(0.03, abs=1e-12)
    # With braking in the budget the no-loss near-field speed is ~0.088 m/s, not the
    # 0.157 m/s of 0.03 / (T + L_max) that ignored the stop.
    assert v == pytest.approx(0.0885, abs=5e-4)
    assert 0.03 / (T + L_MAX) == pytest.approx(0.1565, abs=5e-4)


def test_a_lost_frame_slows_the_truck_and_a_spent_budget_stops_it():
    steady = budget_speed(0.0, T + L_MAX, DECEL)
    one_lost = budget_speed(steady * T, T + L_MAX, DECEL)  # a frame's worth already travelled blind
    assert 0.0 < one_lost < steady
    assert budget_speed(0.03, T + L_MAX, DECEL) == 0.0
    assert budget_speed(0.05, 0.0, DECEL) == 0.0


def test_the_budget_speed_falls_with_travel_and_time():
    assert budget_speed(0.01, 0.2, DECEL) < budget_speed(0.0, 0.2, DECEL)
    assert budget_speed(0.0, 0.3, DECEL) < budget_speed(0.0, 0.2, DECEL)
    # No wait for the next result: only the stop limits the speed.
    assert budget_speed(0.0, 0.0, DECEL) == pytest.approx(math.sqrt(2 * DECEL * 0.03))


@pytest.mark.parametrize("args,kwargs", [((-0.01, 0.2, DECEL), {}), ((0.0, -0.1, DECEL), {}), ((0.0, 0.2, 0.0), {}),
                                         ((0.0, 0.2, DECEL), {"budget_m": 0.0}), ((0.0, 0.2, DECEL, -0.1), {}),
                                         ((float("nan"), 0.2, DECEL), {})])
def test_bad_budget_inputs_are_refused(args, kwargs):
    with pytest.raises(ValueError):
        budget_speed(*args, **kwargs)


def test_the_handoff_now_lives_in_the_core_and_still_needs_its_gate():
    with pytest.raises(TypeError):
        RoofHandoff()
    assert RoofHandoff(start_front_x_m=1.4).start_front_x_m == 1.4


L, DT = 0.075, 1 / 120  # alignment latency assumed until D8b; the runner's control tick
LATENCY = STOP_LATENCY + DT  # reaction plus the tick that notices a missing result


def steady_no_loss_speed():
    """The constant speed the budget allows at its tightest instant: a result stamped L after
    its pixels arrives, d = v * L_max since the conservative pixel time, and the next
    result can come T - L + L_max later (Codex D8c re-review P3: d = 0 overstated it)."""
    lo, hi = 0.0, 1.0
    for _ in range(80):
        v = (lo + hi) / 2
        ok = budget_speed(v * L_MAX, T - L + L_MAX, STOP_DECEL, LATENCY) >= v
        lo, hi = (v, hi) if ok else (lo, v)
    return lo


def test_the_runners_braking_gives_about_0_0765_mps_without_loss():
    v = steady_no_loss_speed()
    total = v * L_MAX + v * (T - L + L_MAX + LATENCY) + v * v / (2 * STOP_DECEL)
    assert total == pytest.approx(0.03, abs=1e-9)
    assert v == pytest.approx(0.0765, abs=5e-4)


def test_stopping_at_once_after_a_lost_result_stays_inside_the_budget():
    # 120 Hz ticks at the steady speed; captures every T, each result stamped L later. The
    # result of capture 1 never comes; the controller notices on the first tick after its
    # latest arrival (capture 1 + L_max) and brakes.
    v = steady_no_loss_speed()
    last_stamp = 0.0 + L  # the result of capture 0
    now = last_stamp
    worst = 0.0
    while True:
        now += DT
        latest_next = T + L_MAX  # capture 1's latest arrival
        if now <= latest_next:
            tau = latest_next - now
            d = v * (now - (last_stamp - L_MAX))
            assert v <= budget_speed(d, tau, STOP_DECEL, LATENCY) + 1e-12
            continue
        # Missing: the next possible result is capture 2's.
        d = v * (now - (last_stamp - L_MAX))
        assert v > budget_speed(d, 2 * T - now + L_MAX, STOP_DECEL, LATENCY)
        worst = d + v * STOP_LATENCY + v * v / (2 * STOP_DECEL)
        break
    assert worst <= 0.03 + 1e-12


def test_the_handoff_reset_breaks_a_run_of_agreeing_frames():
    handoff = RoofHandoff(start_front_x_m=1.4)
    handoff.confirmed = 2
    handoff.reset()
    assert handoff.confirmed == 0 and not handoff.qualified


# ---- NearFieldTracker (plan D8c module) on synthetic observations ----
from forklift_core.perception.near_field_tracking import NearFieldConfig, NearFieldTracker  # noqa: E402
from forklift_core.perception.pocket_observation import Pocket, PocketObservation  # noqa: E402

HALF = 0.18625  # EPAL 6 pocket centres, base_link lateral


def pockets(stamp, x=1.6, y=0.0, yaw=0.0, width=0.2275, status="valid"):
    if status != "valid":
        return PocketObservation(stamp, "synthetic", "base_link", "synthetic", status, None, None, None, None, None, "none")
    c, s = math.cos(yaw), math.sin(yaw)
    left = Pocket((x - s * HALF, y + c * HALF, 0.061), width, 0.078)
    right = Pocket((x + s * HALF, y - c * HALF, 0.061), width, 0.078)
    return PocketObservation(stamp, "synthetic", "base_link", "synthetic", "valid", left, right, yaw, None, None, None)


CONFIG = NearFieldConfig(align_latency_s=0.075, max_latency_s=0.0917, handoff_start_front_x_m=1.4)


def tracker(front, roof=None, pose=(0.0, 0.0, 0.0)):
    """Feeds queued front/roof observations; the near capture is at x 1.6, y 0."""
    fronts, roofs = iter(front), iter(roof or [])
    return NearFieldTracker(CONFIG, pockets(0, 1.6), pose, 0.0,
                            front_fn=lambda scene: next(fronts), roof_fn=lambda scene, xy, yaw: next(roofs))


def test_the_estimate_cannot_walk_away_from_the_anchor():
    # Codex D8c module review P1-1: 20 -> 40 -> 60 mm each passes a 30 mm gate on the last estimate.
    t = tracker([pockets(1, y=0.02), pockets(2, y=0.04), pockets(3, y=0.06)])
    results = [t.on_frame(None, k * 0.1, (0.0, 0.0, 0.0)) for k in (1, 2, 3)]
    assert [r.accepted for r in results] == [True, False, False]
    assert results[1].reason == "left_position" and t.latest_observation.left.center_m[1] == pytest.approx(HALF + 0.02)


def test_a_narrower_pocket_is_rejected_even_with_the_same_midpoint():
    # 0.2275 -> 0.150 m moves each wall 38.75 mm while the centres stay put (review P1-2).
    t = tracker([pockets(1, width=0.150)])
    r = t.on_frame(None, 0.1, (0.0, 0.0, 0.0))
    assert r.rejected and r.reason == "left_size"


def test_three_consecutive_rejections_are_a_mismatch_and_an_accept_resets_the_count():
    t = tracker([pockets(k, y=0.05) for k in (1, 2)] + [pockets(3)] + [pockets(k, y=0.05) for k in (4, 5, 6)])
    flags = [t.on_frame(None, k * 0.1, (0.0, 0.0, 0.0)).mismatch for k in range(1, 7)]
    assert flags == [False, False, False, False, False, True]


def test_observations_are_placed_with_the_capture_pose():
    # The truck advanced 0.5 m in the held frame: the pallet is now 0.5 m closer in base_link.
    t = tracker([pockets(1, x=1.1)], [pockets(1, x=1.1)], pose=(0.0, 0.0, 0.0))
    assert t.on_frame(None, 0.1, (0.5, 0.0, 0.0)).accepted
    assert t.latest_held["left"][0] == pytest.approx(1.6)


def test_the_handoff_needs_three_consecutive_paired_frames_inside_the_gate():
    # Codex review P2-4: 1.399, 1.401, 1.399, 1.399 must not hand off (the 1.401 frame resets).
    xs = [1.399, 1.401, 1.399, 1.399, 1.399]
    poses = [(1.6 - x, 0.0, 0.0) for x in xs]  # the pallet stays at held x 1.6
    t = tracker([pockets(k + 1, x=x) for k, x in enumerate(xs)],
                [pockets(k + 1, x=x) for k, x in enumerate(xs) if x <= 1.4])
    modes = [t.on_frame(None, (k + 1) * 0.1, pose).mode for k, pose in enumerate(poses)]
    assert modes == ["front", "front", "front", "front", "roof"]


def test_after_the_handoff_the_roof_is_associated_and_updates_the_estimate():
    xs = [1.3, 1.3, 1.3]
    t = tracker([pockets(k + 1, x=x) for k, x in enumerate(xs)],
                [pockets(k + 1, x=1.3) for k in range(3)] + [pockets(4, x=1.2, y=0.01), pockets(5, x=1.2, y=0.08)])
    for k in range(3):
        t.on_frame(None, (k + 1) * 0.1, (0.3, 0.0, 0.0))
    assert t.mode == "roof"
    assert t.on_frame(None, 0.4, (0.4, 0.0, 0.0)).accepted
    r = t.on_frame(None, 0.5, (0.4, 0.0, 0.0))
    assert r.rejected and t.latest_observation.left.center_m[1] == pytest.approx(HALF + 0.01)


def test_the_safety_age_loss_and_failure():
    t = tracker([pockets(1)])
    t.on_frame(None, 1.0, (0.0, 0.0, 0.0))
    s = t.status(1.1)
    assert s.age_s == pytest.approx(0.1 + 0.0917) and not s.lost
    s = t.status(1.25)  # 0.25 + 0.0917 > 0.3
    assert s.lost and not s.failed and s.lost_since_s == pytest.approx(1.25)
    assert not t.status(4.2).failed and t.status(4.3).failed


def test_an_invalid_frame_breaks_a_run_of_rejections():
    # Codex D8a implementation review P3-3: reject, invalid, reject, invalid, reject is not a mismatch.
    bad = lambda k: pockets(k, y=0.05)  # noqa: E731
    invalid = lambda k: pockets(k, status="no_pallet")  # noqa: E731
    t = tracker([bad(1), invalid(2), bad(3), invalid(4), bad(5)])
    results = [t.on_frame(None, k * 0.1, (0.0, 0.0, 0.0)) for k in range(1, 6)]
    assert not any(r.mismatch for r in results) and results[-1].consecutive_rejects == 1
