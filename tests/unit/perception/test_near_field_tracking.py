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
    one_lost = budget_speed(
        steady * T, T + L_MAX, DECEL
    )  # a frame's worth already travelled blind
    assert 0.0 < one_lost < steady
    assert budget_speed(0.03, T + L_MAX, DECEL) == 0.0
    assert budget_speed(0.05, 0.0, DECEL) == 0.0


def test_the_budget_speed_falls_with_travel_and_time():
    assert budget_speed(0.01, 0.2, DECEL) < budget_speed(0.0, 0.2, DECEL)
    assert budget_speed(0.0, 0.3, DECEL) < budget_speed(0.0, 0.2, DECEL)
    # No wait for the next result: only the stop limits the speed.
    assert budget_speed(0.0, 0.0, DECEL) == pytest.approx(math.sqrt(2 * DECEL * 0.03))


@pytest.mark.parametrize(
    "args,kwargs",
    [
        ((-0.01, 0.2, DECEL), {}),
        ((0.0, -0.1, DECEL), {}),
        ((0.0, 0.2, 0.0), {}),
        ((0.0, 0.2, DECEL), {"budget_m": 0.0}),
        ((0.0, 0.2, DECEL, -0.1), {}),
        ((float("nan"), 0.2, DECEL), {}),
    ],
)
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


# ---- NearFieldTracker 3판 (plan D8c) on synthetic observations and bounds ----
from forklift_core.perception.near_field_bounds import NearFieldBounds  # noqa: E402
from forklift_core.perception.near_field_tracking import (  # noqa: E402
    NearFieldConfig,
    NearFieldTracker,
    wall_erosion_m,
)
from forklift_core.perception.pocket_observation import (  # noqa: E402
    Pocket,
    PocketObservation,
)

HALF = 0.18625  # EPAL 6 pocket centres, base_link lateral
CAM_X, REAR_X = 0.619, -0.34
L_ALIGN, L_TOP = 0.0917, 0.0989


def bins(lateral, along, yaw, wall, width=None, below=0.0, above=3.0):
    rows = []
    for k in range(30, 0, -1):
        hi, lo = round(k * 0.1, 3), round((k - 1) * 0.1, 3)
        if lo < below or hi > above:
            rows.append({"bin_m": [hi, lo], "valid": 0, "unbounded": True})
            continue
        cell = {
            "bin_m": [hi, lo],
            "valid": 9,
            "lateral_m": lateral,
            "along_m": along,
            "yaw_rad": yaw,
            "wall_m": wall,
        }
        if width is not None:
            cell["width_m"] = width
        rows.append(cell)
    return rows


AGES = [0.1, 0.2, 0.3, 0.4, 0.5, 1.0, 3.5, 70.0]
BOUNDS = NearFieldBounds.from_dict(
    {
        "schema": "near_field_bounds/v1",
        "latency": {
            "align_s": L_ALIGN,
            "max_s": L_TOP,
            "read_delay_s": 1 / 120,
            "render_period_s": 0.1,
        },
        "observation": {
            "front": bins(0.005, 0.0005, 0.0007, 0.007, width=0.0125, below=0.4),
            "roof": bins(0.0002, 0.02, 0.0005, 0.0002, above=1.8),
        },
        "odometry": {
            "age_s": AGES,
            "e_m": [0.0035, 0.005, 0.0062, 0.0074, 0.0088, 0.0142, 0.0207, 0.0288],
            "psi_rad": [0.0019, 0.0035, 0.005, 0.0064, 0.0078, 0.0124, 0.0132, 0.0132],
        },
        "near_capture": {
            "bound": {
                "lateral_m": 0.006,
                "along_m": 0.0004,
                "yaw_rad": 0.0008,
                "wall_m": 0.0098,
                "width_m": 0.0125,
            }
        },
    }
)
CONFIG = NearFieldConfig(
    handoff_start_front_x_m=1.52,
    camera_xy_m=(CAM_X, 0.0),
    rear_axle_x_m=REAR_X,
    opening_width_m=0.2275,
    pocket_spacing_m=2 * HALF,
)
FACE_X = 3.2  # the near capture: camera-face 2.58 m


def pockets(
    stamp, x=FACE_X, y=0.0, yaw=0.0, width=0.2275, half=HALF, status="valid", sigma=None
):
    if status != "valid":
        return PocketObservation(
            stamp,
            "synthetic",
            "base_link",
            "synthetic",
            status,
            None,
            None,
            None,
            None,
            None,
            "none",
        )
    c, s = math.cos(yaw), math.sin(yaw)
    left = Pocket((x - s * half, y + c * half, 0.061), width, 0.078)
    right = Pocket((x + s * half, y - c * half, 0.061), width, 0.078)
    return PocketObservation(
        stamp,
        "synthetic",
        "base_link",
        "synthetic",
        "valid",
        left,
        right,
        yaw,
        sigma,
        None,
        None,
    )


def tracker(front, roof=None):
    """Feeds queued front/roof observations; the near capture at FACE_X, aligned at 0."""
    fronts, roofs = iter(front), iter(roof or [])
    return NearFieldTracker(
        CONFIG,
        BOUNDS,
        pockets(0),
        (0.0, 0.0, 0.0),
        0.0,
        front_fn=lambda scene: next(fronts),
        roof_fn=lambda scene, xy, yaw: next(roofs),
        frame_version="hold-1",
    )


A = 1 / 120  # read delay: a result arrives one tick after its stamp


def frame(t, k, pose=(0.0, 0.0, 0.0), stamp=None):
    """Frame k (aligned at k * 0.1 s), delivered A after its stamp."""
    stamp = k * 0.1 + L_ALIGN if stamp is None else stamp
    return t.on_frame(None, stamp, pose, stamp + A, "hold-1")


def test_a_steady_approach_is_accepted_frame_by_frame():
    poses = [(0.0075 * k, 0.0, 0.0) for k in range(1, 6)]
    t = tracker([pockets(k, x=FACE_X - p[0]) for k, p in enumerate(poses, start=1)])
    results = [frame(t, k, p) for k, p in enumerate(poses, start=1)]
    assert all(r.accepted for r in results)
    assert t.latest_held["left"][0] == pytest.approx(FACE_X)


def test_a_jump_beyond_the_measured_step_tolerance_is_rejected():
    # step lateral tolerance at 0.1 s: 6 + 5 + 3.5 mm + rho 3.54 m x 1.9 mrad = 21.3 mm (+ mixing)
    t = tracker([pockets(1, y=0.019), pockets(2, y=0.03)])
    assert frame(t, 1).accepted
    t2 = tracker([pockets(1, y=0.03)])
    r = frame(t2, 1)
    assert r.rejected and r.reason == "left_lateral"


def test_the_anchor_check_stops_a_walk_that_every_step_allows():
    # 12 mm per frame passes each step (about 21 mm) but drifts from the near capture:
    # 36 mm at 0.3 s exceeds the anchor's 6 + 5 + 6.2 mm + 3.55 m x 5 mrad = 35.2 mm.
    t = tracker([pockets(k, y=0.012 * k) for k in range(1, 4)])
    results = [frame(t, k) for k in range(1, 4)]
    assert [r.accepted for r in results] == [True, True, False]
    assert results[2].rejected and results[2].reason == "anchor_left_lateral"


def test_the_front_must_agree_with_the_pallet_model():
    r = frame(tracker([pockets(1, width=0.150)]), 1)
    assert r.rejected and r.reason == "left_width"
    r = frame(tracker([pockets(1, half=HALF + 0.02)]), 1)
    assert r.rejected and r.reason == "spacing"


def test_three_consecutive_rejections_are_a_mismatch_and_an_invalid_frame_breaks_the_run():
    bad = (
        [pockets(k, y=0.05) for k in (1, 2)]
        + [pockets(3)]
        + [pockets(k, y=0.05) for k in (4, 5, 6)]
    )
    t = tracker(bad)
    assert [frame(t, k).mismatch for k in range(1, 7)] == [False] * 5 + [True]
    t = tracker(
        [pockets(1, y=0.05), pockets(2, status="no_pallet"), pockets(3, y=0.05)]
    )
    results = [frame(t, k) for k in (1, 2, 3)]
    assert not any(r.mismatch for r in results) and results[-1].consecutive_rejects == 1


def test_an_observation_without_measured_bounds_is_invalid_not_rejected():
    # camera-face 0.35 m: the front table is unmeasured below 0.4 m
    t = tracker([pockets(1, x=FACE_X - (FACE_X - CAM_X - 0.35))])
    r = frame(t, 1, (FACE_X - CAM_X - 0.35, 0.0, 0.0))
    assert (
        not r.accepted
        and not r.rejected
        and r.reason == "unbounded_bin"
        and r.consecutive_rejects == 0
    )


def test_an_age_past_the_odometry_table_is_an_unbounded_event():
    t = tracker([pockets(1)])
    r = frame(t, 0, stamp=71.0 + L_ALIGN)
    assert r.unbounded and r.reason == "unbounded_age" and not r.accepted


def test_observations_are_placed_with_their_pose():
    t = tracker([pockets(1, x=FACE_X - 0.5)])
    assert frame(t, 1, (0.5, 0.0, 0.0)).accepted
    assert t.latest_held["left"][0] == pytest.approx(FACE_X)


def roofs(k, x, sigma=0.005):
    return pockets(k, x=x, sigma=sigma)


def test_the_handoff_needs_three_paired_frames_inside_the_gate_and_a_skipped_frame_resets_it():
    x = 1.50  # front midpoint inside g = 1.52
    pose = (FACE_X - x, 0.0, 0.0)
    t = tracker(
        [pockets(k, x=x) for k in range(1, 6)], [roofs(k, x) for k in range(1, 6)]
    )
    assert frame(t, 1, pose).handoff_confirmed == 1
    assert frame(t, 2, pose).handoff_confirmed == 2
    r = frame(t, 4, pose)  # frame 3 never came
    assert r.handoff_confirmed == 1 and r.mode == "front"
    frame(t, 5, pose)
    assert frame(t, 6, pose).mode == "roof"


def test_no_handoff_outside_the_gate_or_with_a_noisy_roof():
    x = 1.55
    t = tracker([pockets(k, x=x) for k in range(1, 4)])
    for k in range(1, 4):
        assert frame(t, k, (FACE_X - x, 0.0, 0.0)).handoff_confirmed == 0
    x = 1.50
    t = tracker(
        [pockets(k, x=x) for k in range(1, 4)],
        [roofs(k, x, sigma=0.03) for k in range(1, 4)],
    )
    results = [frame(t, k, (FACE_X - x, 0.0, 0.0)) for k in range(1, 4)]
    assert all(
        r.accepted and r.handoff_confirmed == 0 and r.mode == "front" for r in results
    )


def test_the_safety_age_uses_the_top_of_the_band():
    t = tracker([pockets(1)])
    t.arm(0.0)
    frame(t, 1)  # stamp 0.1 + L
    stamp = 0.1 + L_ALIGN
    s = t.status(stamp + 0.1)
    assert s.age_s == pytest.approx(0.1 + L_TOP) and not s.lost
    assert t.status(stamp + 0.21).lost  # 0.21 + 0.0989 > 0.3
    assert (
        not t.status(stamp + 0.21 + 2.99).failed
        and t.status(stamp + 0.21 + 3.01).failed
    )


def test_the_camera_face_distance_projects_on_the_observed_axis():
    t = tracker([])
    assert t.distance_m(pockets(0, x=2.0)) == pytest.approx(2.0 - CAM_X)
    yaw = 0.05
    obs = pockets(0, x=2.0, y=0.1, yaw=yaw)
    assert t.distance_m(obs) == pytest.approx(
        (2.0 - CAM_X) * math.cos(yaw) + 0.1 * math.sin(yaw)
    )


def test_the_wall_erosion_adds_its_terms():
    bound = {"wall_m": 0.0001, "yaw_rad": 0.0002}
    got = wall_erosion_m(bound, (0.005, 0.0035), 0.33, 1.62, 0.00005)
    assert got == pytest.approx(
        0.0001 + 0.33 * 0.0002 + 0.005 + 1.62 * 0.0035 + 0.00005
    )


def test_non_finite_poses_and_times_are_refused():
    t = tracker([pockets(1, y=0.05)])
    with pytest.raises(ValueError):
        t.on_frame(None, 0.1 + L_ALIGN, (math.nan, 0.0, 0.0), 0.2, "hold-1")
    with pytest.raises(ValueError):
        t.status(math.nan)
    with pytest.raises(ValueError):
        NearFieldTracker(
            CONFIG,
            BOUNDS,
            pockets(0),
            (0.0, math.inf, 0.0),
            0.0,
            front_fn=None,
            roof_fn=None,
            frame_version="hold-1",
        )


def test_a_missing_frame_breaks_a_run_of_rejections_like_an_invalid_one():
    t = tracker([pockets(k, y=0.05) for k in (1, 2, 4)])
    results = [frame(t, k) for k in (1, 2, 4)]  # frame 3 never came
    assert [r.consecutive_rejects for r in results] == [1, 2, 1] and not results[
        -1
    ].mismatch


def test_a_frame_off_the_render_cadence_resets_the_handoff():
    x = 1.50
    pose = (FACE_X - x, 0.0, 0.0)
    t = tracker(
        [pockets(k, x=x) for k in range(1, 4)], [roofs(k, x) for k in range(1, 4)]
    )
    frame(t, 1, pose)
    frame(t, 2, pose)
    r = frame(t, 3, pose, stamp=0.3 + L_ALIGN + A)  # one control tick late
    assert r.handoff_confirmed == 1 and r.mode == "front"


def test_a_result_after_the_reacquisition_deadline_does_not_erase_the_failure():
    t = tracker([pockets(1), pockets(2)])
    t.arm(0.0)
    frame(t, 1)
    stamp = 0.1 + L_ALIGN
    lost_at = stamp - L_TOP + 0.3 + 0.001
    assert t.status(lost_at).lost and not t.status(lost_at).failed
    late = lost_at + 3.0 + 0.008  # the next result arrives after the deadline
    r = t.on_frame(None, late - A, (0.0, 0.0, 0.0), late, "hold-1")
    assert r.accepted and r.failed
    assert t.status(late + A).failed


def test_a_changed_held_frame_needs_a_new_tracker():
    t = tracker([pockets(1)])
    with pytest.raises(ValueError, match="held frame"):
        t.on_frame(None, 0.1 + L_ALIGN, (0.0, 0.0, 0.0), 0.2, "hold-2")
    assert frame(t, 1).accepted and t.latest.frame_version == "hold-1"


def test_a_loss_outside_the_section_never_fails_and_the_wait_starts_at_entry():
    t = tracker([pockets(1), pockets(2)])
    frame(t, 1)
    stamp = 0.1 + L_ALIGN
    s = t.status(stamp + 4.0)  # lost for about 3.8 s, not armed
    assert s.lost and not s.failed
    t.arm(stamp + 4.0)  # enters the section while lost
    assert not t.status(stamp + 6.9).failed and t.status(stamp + 7.01).failed
    t2 = tracker([pockets(1), pockets(2)])
    frame(t2, 1)
    t2.status(stamp + 4.0)
    r = t2.on_frame(None, stamp + 4.0 - A, (0.0, 0.0, 0.0), stamp + 4.0, "hold-1")
    assert r.accepted and not r.failed and not t2.status(stamp + 4.0 + A).lost


def test_the_frame_version_is_an_immutable_token():
    with pytest.raises(ValueError, match="immutable"):
        NearFieldTracker(
            CONFIG,
            BOUNDS,
            pockets(0),
            (0.0, 0.0, 0.0),
            0.0,
            front_fn=None,
            roof_fn=None,
            frame_version={"hold": 1},
        )


def test_each_source_reports_its_own_verdict():
    x = 1.50
    pose = (FACE_X - x, 0.0, 0.0)
    t = tracker(
        [pockets(1, x=x), pockets(2, x=x)], [roofs(1, x, sigma=0.03), roofs(2, x)]
    )
    r = frame(t, 1, pose)
    assert (r.front_eval, r.roof_eval, r.roof_reason) == (
        "accepted",
        "invalid",
        "roof_sigma",
    )
    r = frame(t, 2, pose)
    assert (r.front_eval, r.roof_eval, r.handoff_confirmed) == (
        "accepted",
        "accepted",
        1,
    )


def test_the_frame_version_type_is_checked_on_every_frame():
    t = NearFieldTracker(
        CONFIG,
        BOUNDS,
        pockets(0),
        (0.0, 0.0, 0.0),
        0.0,
        front_fn=lambda scene: pockets(1),
        roof_fn=None,
        frame_version=1,
    )
    for token in (True, 1.0, "1"):
        with pytest.raises(ValueError):
            t.on_frame(None, 0.1 + L_ALIGN, (0.0, 0.0, 0.0), 0.2, token)
    assert t.on_frame(None, 0.1 + L_ALIGN, (0.0, 0.0, 0.0), 0.2, 1).accepted


def test_arming_while_already_old_starts_the_wait_at_entry_without_a_status_call():
    t = tracker([pockets(1)])
    frame(t, 1)
    stamp = 0.1 + L_ALIGN
    t.arm(stamp + 5.0)  # old, never asked for status
    assert not t.status(stamp + 7.9).failed and t.status(stamp + 8.01).failed


def test_an_unbounded_age_is_the_evaluated_sources_verdict():
    t = tracker([pockets(1)])
    r = frame(t, 0, stamp=71.0 + L_ALIGN)
    assert (r.front_eval, r.front_reason, r.roof_eval) == (
        "invalid",
        "unbounded_age",
        None,
    )
