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


def test_missing_or_non_finite_samples_never_pass():
    # Codex implementation review P2: a run with keyframe times missing, NaN applied poses, or an R without control rows
    gaps = _run()
    for r in gaps.records[1:]:
        r["slam_wall_s"] = None
    assert "keyframe" in " ".join(cmp.verdict(gaps, _run(dx=0.04))["failed"])
    nan = _run()
    for row in nan.control:
        row[1] = float("nan")
    assert not cmp.verdict(nan, _run(dx=0.04))["ok"]
    bare = _run(dx=0.04)
    bare.control[:] = []
    assert not cmp.verdict(_run(dx=0.02), bare)["ok"]


def test_not_comparable_is_not_a_pass():
    r = cmp.RunData(records=[], control=[], missions=[], failure="slam_map_not_loaded")
    out = cmp.verdict(_run(), r)
    assert not out["ok"] and "not comparable" in " ".join(out["failed"])


def test_every_sample_after_the_common_span_is_held_to_the_absolute_bounds():
    # Codex: L to 3,700 s, R to 3,699 s, L's last applied error 0.22 m passed under the 1 s allowance
    l_run, r_run = _run(dx=0.02), _run(3699.0, dx=0.04)
    l_run.control[-1][1] += 0.2
    out = cmp.verdict(l_run, r_run)
    assert not out["ok"] and out["l_after_r"] is not None and "0.15" in " ".join(out["failed"])


def test_the_drift_halves_are_the_first_hours_two_half_hours():
    # S3 result review (Codex): the halves were the first and the LAST 30 min; the plan means 0-30 vs 30-60 min of the
    # first hour from the first response -- a run longer than an hour drifting in its second half-hour fails
    r_run = _run(1000.0, dx=0.04, failure="slam_link_failed: keyframe")
    l_run = _run(4600.0, dx=0.02)
    for row in l_run.control:
        if 1800 <= row[0] < 3600:
            row[1] += 0.005  # the second half-hour 5 mm worse; most of the last 30 min (2,800-4,600 s) is clean
    out = cmp.verdict(l_run, r_run)
    assert out["l_after_r"]["second_half_median_m"] > out["l_after_r"]["first_half_median_m"]
    assert not out["ok"] and "second half" in " ".join(out["failed"])


def test_mission_failures_in_live_failures_are_reported_for_both_arms(tmp_path):
    import json

    run = tmp_path / "run"
    (run / "spool").mkdir(parents=True)
    (run / "result.json").write_text(json.dumps({"failure_reason": None, "live_failures": [
        {"time_s": 1874.4, "phase": "transport", "reason": "docking_retry_match_refused:correction_out_of_bounds",
         "class": "mission"}]}))
    assert cmp.failures_of(json.loads((run / "result.json").read_text())) == [
        {"time_s": 1874.4, "phase": "transport", "reason": "docking_retry_match_refused:correction_out_of_bounds",
         "class": "mission"}]
    out = cmp.verdict(_run(dx=0.02), cmp.RunData(**{**vars(_run(1874.0, dx=0.04)), "failures": [
        {"time_s": 1874.4, "reason": "docking_retry_match_refused:correction_out_of_bounds", "class": "mission"}]}))
    assert out["r_failures"][0]["reason"].startswith("docking_retry_match_refused")


def test_the_applied_delay_is_reported():
    run = _run(600.0)
    for r in run.records:
        r["applied_tick_s"] = r["stamp_s"] + 0.25
    d = cmp.metrics(run)["applied_delay_s"]
    assert d["median"] == pytest.approx(0.25) and d["max"] == pytest.approx(0.25)


@pytest.mark.parametrize("damage", ["no_correction", "no_truth", "no_stamp", "control_time_nan"])
def test_a_damaged_raw_sample_fails_before_any_filtering(damage):
    # Codex implementation review round 2 P2: samples dropped by the filters passed as if they were not there
    l_run = _run(dx=0.02)
    r = l_run.records[100]
    if damage == "no_correction":
        r["map_from_odom"] = None
    elif damage == "no_truth":
        r["truth_base"] = None
    elif damage == "no_stamp":
        r["stamp_s"], r["slam_wall_s"] = None, 2.0
    else:
        l_run.control[500][0] = float("nan")
        l_run.control[500][1] += 0.2
    out = cmp.verdict(l_run, _run(dx=0.04))
    assert not out["ok"] and "L record" in " ".join(out["failed"])
    assert cmp.verdict(_run(dx=0.02), l_run)["ok"] is False  # the same damage on R's side
