"""The live-default plan's Isaac verdict (docs/plans/2026-10-26-slam-localization-default.md step 3): a localization run
(L) against a mapping run (R) of the same scene, seed and order -- sim-time windows and observed length, the response
and applied pose against the truth in position and heading, correction jumps, missions, compared on the common sim span,
L's remainder on absolute bounds when R ends first."""

import math

import pytest

from tools import slam_live_compare as cmp


def _records(t_end, *, dx=0.0, dyaw=0.0, wall=0.1, every=2.0):
    out, t, i = [], 0.0, 0
    while t <= t_end + 1e-9:
        odom = [0.1 * t, 0.0, 0.0]
        truth = [0.1 * t, 0.0, -dyaw]  # the heading error on the truth side: a turned correction moves far points
        out.append({"scan_id": i, "stamp_s": t, "status": "processed", "map_from_odom": [dx, 0.0, 0.0],
                    "applied_map_from_odom": [dx, 0.0, 0.0], "odom_base": odom, "truth_base": truth, "slam_wall_s": wall})
        t += every
        i += 1
    return out


def _control(t_end, *, dx=0.0, dyaw=0.0, every=1.0):
    return [[k * every, 0.1 * k * every + dx, 0.0, dyaw, 0.1 * k * every, 0.0, 0.0] for k in range(int(t_end / every) + 1)]


def _missions(t_end, leg_s=150.0):
    out, t, k = [], 0.0, 0
    while t + leg_s <= t_end:
        t += leg_s
        k += 1
        out.append({"mission_id": f"m{k}", "outcome": "done", "reason": None, "ended_s": t, "delivery_error_m": 0.02})
    return out


def _run(t_end=3700.0, **k):
    return cmp.RunData(records=_records(t_end, **{a: b for a, b in k.items() if a in ("dx", "dyaw", "wall")}),
                       control=_control(t_end, **{a: b for a, b in k.items() if a in ("dx", "dyaw")}),
                       missions=_missions(t_end), failure=k.get("failure"))


def test_errors_are_split_into_response_and_applied_with_heading():
    m = cmp.metrics(_run(600.0, dx=0.03, dyaw=0.02))
    assert m["response"]["position_max_m"] == pytest.approx(0.03) and m["response"]["yaw_max_rad"] == pytest.approx(0.02)
    assert m["applied"]["position_median_m"] == pytest.approx(0.03) and m["applied"]["yaw_max_rad"] == pytest.approx(0.02)
    assert m["missions_done"] == 4 and m["delivery_error_max_m"] == pytest.approx(0.02)


def test_six_sim_windows_and_the_observed_length_decide_progress():
    good = cmp.progress(_run(3700.0))
    assert good["ok"] and good["observed_s"] >= 3600 and len(good["windows"]) == 6
    short = cmp.progress(_run(3000.0))
    assert not short["ok"] and short["observed_s"] < 3600
    stalled = _run(3700.0)
    stalled.missions[:] = [m for m in stalled.missions if not 1200 <= m["ended_s"] < 1800]  # nothing done in window 3
    assert not cmp.progress(stalled)["ok"] and not cmp.progress(stalled)["windows"][2]["ok"]


def test_l_passes_when_no_worse_than_r_on_the_common_span():
    out = cmp.verdict(_run(dx=0.02), _run(dx=0.04))
    assert out["ok"] and out["common_span_s"] == pytest.approx([0.0, 3700.0])
    worse = cmp.verdict(_run(dx=0.05), _run(dx=0.04))
    assert not worse["ok"] and "applied position" in " ".join(worse["failed"])


def test_a_slow_keyframe_or_a_slam_end_fails_l():
    assert "keyframe" in " ".join(cmp.verdict(_run(wall=1.2), _run())["failed"])
    assert "end" in " ".join(cmp.verdict(_run(failure="slam_link_failed: no pose"), _run())["failed"])


def test_when_r_ends_first_l_keeps_going_on_absolute_bounds():
    r = _run(1500.0, dx=0.04, failure="slam_link_failed: keyframe")
    out = cmp.verdict(_run(dx=0.02), r)
    assert out["ok"] and out["common_span_s"] == pytest.approx([0.0, 1500.0])
    assert out["r_failure"].startswith("slam_link_failed") and out["l_after_r"]["ok"]
    drifting = _run(dx=0.0)
    for row in drifting.control:  # the second half drifts away
        if row[0] > 2000:
            row[1] += 0.08
    out = cmp.verdict(drifting, r)
    assert not out["ok"] and "second half" in " ".join(out["failed"])
    assert not cmp.verdict(_run(dx=0.2), r)["ok"]  # past the V5 bound 0.15 m


def test_no_common_span_is_not_comparable():
    r = cmp.RunData(records=[], control=[], missions=[], failure="slam_map_not_loaded")
    out = cmp.verdict(_run(), r)
    assert out["common_span_s"] is None and "not comparable" in out["comparison"]


def test_a_correction_jump_is_measured_between_processed_keyframes():
    run = _run(600.0)
    run.records[10]["map_from_odom"] = [0.07, 0.0, 0.0]
    j = cmp.metrics(run)["correction_jumps"]
    assert j["position_max_m"] == pytest.approx(0.07) and j["position_over_0_05_m"] == 2
    assert math.isfinite(j["yaw_max_rad"])
