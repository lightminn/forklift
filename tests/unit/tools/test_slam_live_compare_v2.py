"""The second confirmation's criteria (docs/plans/2026-10-26-slam-localization-default.md 5c, pre-registered after S3):
holding rows from the hold/release timeline and the records' mode, heading compared on tracking rows only, the run's
own drift allowance, every delivery attempt, comparable pairs, the hold-drift gate per scene and the overall pass."""

import math

import pytest

from tools import slam_live_compare as cmp


def _run(t_end=3700.0, *, dx=0.0, dyaw=0.0, holds=(), hold_yaw=0.0, every=2.0, leg_s=150.0, failure=None,
         delivery=0.02, jump=0.01, travel=1.0):
    """A synthetic session: records every ``every`` s (tracking, or holding inside ``holds`` = [(t_h, t_r)]), control
    rows every 1 s with ``dx`` position error and ``dyaw`` heading error (``hold_yaw`` more inside holds), a mission
    every ``leg_s``; the timeline holds one hold/release pair per hold with ``jump`` rad, the odometry moving
    ``travel`` m inside each hold."""
    def holding(t):
        return any(a <= t < b for a, b in holds)

    records, t, i = [], 0.0, 0
    while t <= t_end + 1e-9:
        inside = [h for h in holds if h[0] <= t < h[1]]
        x = 0.1 * t
        if inside:  # the odometry inside a hold moves `travel` m over the hold
            a, b = inside[0]
            x = 0.1 * a + travel * (t - a) / (b - a)
        records.append({"scan_id": i, "stamp_s": t, "applied_tick_s": t, "status": "processed",
                        "mode": "holding" if holding(t) else "tracking", "map_from_odom": [dx, 0.0, 0.0],
                        "odom_base": [x, 0.0, 0.0], "truth_base": [x, 0.0, -0.001], "slam_wall_s": 0.1})
        t += every
        i += 1
    control = [[k, 0.1 * k + dx, 0.0, dyaw + (hold_yaw if holding(k) else 0.0), 0.1 * k, 0.0, 0.0]
               for k in range(int(t_end) + 1)]
    missions, k, tm = [], 0, 0.0
    while tm + leg_s <= t_end:
        tm += leg_s
        k += 1
        missions.append({"mission_id": f"m{k}", "outcome": "done", "reason": None, "ended_s": tm,
                         "delivery_error_m": delivery})
    timeline = []
    for a, b in holds:
        timeline += [{"event": "hold", "time_s": a}, {"event": "release", "time_s": b, "jump_rad": jump, "jump_m": jump}]
    return cmp.RunData(records=records, control=control, missions=missions, failure=failure, holds=timeline)


def test_tracking_rows_exclude_closed_hold_intervals_and_record_mode_boundaries():
    run = _run(100.0, holds=[(20.0, 30.0)])
    mask = cmp.tracking_mask(run)
    times = [row[0] for row in run.control]
    assert not any(mask[i] for i, t in enumerate(times) if 20 <= t <= 30)  # closed: the release tick too
    assert mask[times.index(10)] and mask[times.index(40)]
    # an event-less hold (start_clear_out) still leaves tracking through the records' mode
    silent = _run(100.0, holds=[(50.0, 60.0)])
    silent.holds = []
    m2 = cmp.tracking_mask(silent)
    assert not m2[silent.control.index(next(r for r in silent.control if r[0] == 55))]


def test_heading_is_compared_on_tracking_rows_only_and_holding_rows_on_v5():
    holds = [(100.0 + 300 * k, 104.0 + 300 * k) for k in range(12)]
    # L's hold heading 0.04 (under V5) above R's 0.0; tracking equal -> pass on heading
    out = cmp.pair_verdict(_run(dx=0.02, holds=holds, hold_yaw=0.04), _run(dx=0.04, holds=holds))
    assert out["comparable"] and out["ok"], out["failed"]
    # past V5 on a holding row fails L's absolute bound
    out = cmp.pair_verdict(_run(dx=0.02, holds=holds, hold_yaw=0.06), _run(dx=0.04, holds=holds))
    assert not out["ok"] and "holding" in " ".join(out["failed"])
    # worse on tracking rows fails the comparison
    out = cmp.pair_verdict(_run(dx=0.02, dyaw=0.02), _run(dx=0.04, dyaw=0.01))
    assert not out["ok"] and "tracking" in " ".join(out["failed"])


def test_the_drift_allowance_is_the_runs_own_budget():
    # A = 0.064 m: allowance (0.15 - 0.064) / 15 = 5.7 mm; a rise of 8.5 mm fails, 4 mm passes
    def drifting(rise):
        run = _run(dx=0.064)
        for row in run.control:
            if 1800 <= row[0] < 3600:
                row[1] += rise
        return run

    assert "drift" in " ".join(cmp.pair_verdict(drifting(0.0085), _run(dx=0.1))["failed"])
    assert cmp.pair_verdict(drifting(0.004), _run(dx=0.1))["ok"]


def test_every_delivery_attempt_counts():
    l_run = _run(dx=0.02)
    l_run.missions[3] = {**l_run.missions[3], "outcome": "failed", "reason": cmp.DELIVERY_REASONS[0],
                         "delivery_error_m": 0.12}
    assert "delivery" in " ".join(cmp.pair_verdict(l_run, _run(dx=0.04))["failed"])
    missing = _run(dx=0.02)
    missing.missions[2] = {**missing.missions[2], "delivery_error_m": None}
    assert "delivery" in " ".join(cmp.pair_verdict(missing, _run(dx=0.04))["failed"])


def test_a_short_r_makes_the_pair_not_comparable_and_l_is_held_to_absolute_bounds():
    short = _run(500.0, dx=0.04)  # 500 s common span < 600 s
    out = cmp.pair_verdict(_run(dx=0.02), short)
    assert out["comparable"] is False and out["ok"]  # not comparable is not a failure
    out = cmp.pair_verdict(_run(dx=0.2), short)
    assert out["comparable"] is False and not out["ok"]  # but L past V5 is
    one_mission = _run(700.0, dx=0.04, leg_s=400.0)  # 700 s, only one R mission done
    assert cmp.pair_verdict(_run(dx=0.02), one_mission)["comparable"] is False


def test_hold_values_pair_releases_and_rebuild_event_less_holds():
    run = _run(1000.0, holds=[(100.0, 110.0), (300.0, 310.0)], jump=0.02, travel=2.0)
    values, incomplete = cmp.hold_values(run)
    assert not incomplete and values == pytest.approx([0.01, 0.01], rel=0.15)  # the synthetic odometry steps back after the hold
    # the second hold has no hold event (start_clear_out): rebuilt from the records' mode
    run.holds = [e for e in run.holds if not (e["event"] == "hold" and e["time_s"] == 300.0)]
    values, incomplete = cmp.hold_values(run)
    assert not incomplete and len(values) == 2
    # a recorded holding span with no release at all (and not trailing) is incomplete
    run.holds = [e for e in run.holds if e["time_s"] < 200.0]
    _, incomplete = cmp.hold_values(run)
    assert incomplete


def test_the_scene_gate_uses_pooled_median_and_p90_higher():
    def runs_with(jumps):
        out = []
        for j in jumps:
            out.append(_run(1000.0, dx=0.01, holds=[(100.0, 110.0)], jump=j, travel=1.0))
        return out

    r = runs_with([0.010] * 10)
    assert cmp.scene_gate(runs_with([0.010] * 10), r)["state"] == "ok"
    tail = runs_with([0.010] * 9 + [0.049])  # one tail hold: median ratio 1.0, p90 'higher' 4.9
    assert cmp.scene_gate(tail, r)["state"] == "fail"
    assert cmp.scene_gate(runs_with([0.010] * 3), runs_with([0.010] * 5))["state"] == "insufficient"


def test_a_slam_mission_failure_of_l_is_no_pass():
    # agy review (a): failures before the delivery check were not judged
    l_run = _run(dx=0.02)
    l_run.failures = [{"time_s": 900.0, "phase": "transport", "reason": "docking_retry_match_refused:"
                       "correction_out_of_bounds", "class": "mission"}]
    out = cmp.pair_verdict(l_run, _run(dx=0.04))
    assert not out["ok"] and "SLAM" in " ".join(out["failed"])
    other = _run(dx=0.02)
    other.failures = [{"time_s": 900.0, "phase": "observe", "reason": "Tracking failed in observe", "class": "mission"}]
    assert cmp.pair_verdict(other, _run(dx=0.04))["ok"]  # reported, not a SLAM failure


def test_the_last_ten_minutes_are_held_to_the_same_allowance():
    # agy review (d): a permanent jump at 52 min hides in the 30-60 min median
    run = _run(dx=0.03)
    for row in run.control:
        if 3120 <= row[0] < 3600:
            row[1] += 0.06
    assert "drift" in " ".join(cmp.pair_verdict(run, _run(dx=0.1))["failed"])


def test_the_gate_adds_raw_release_jumps_and_hold_time_responses_and_skips_short_holds_per_metre():
    def runs(jump, travel, resp_err=0.0, n=10):
        out = []
        for _ in range(n):
            r = _run(1000.0, holds=[(100.0, 110.0)], jump=jump, travel=travel)
            for rec in r.records:
                if rec["mode"] == "holding":
                    rec["map_from_odom"] = [resp_err, 0.0, 0.0]
            out.append(r)
        return out

    base = runs(0.01, 1.0, resp_err=0.01)
    # a short hold (0.05 m) with an ordinary jump: no per-metre verdict (travel < 0.3 m), raw jump equal -> ok
    assert cmp.scene_gate(runs(0.01, 0.05, resp_err=0.01), base)["state"] == "ok"
    # raw jumps 3x larger on long holds: per-metre AND raw fail
    assert cmp.scene_gate(runs(0.03, 1.0, resp_err=0.01), base)["state"] == "fail"
    # responses during holds 3x worse
    worse = cmp.scene_gate(runs(0.01, 1.0, resp_err=0.03), runs(0.01, 1.0, resp_err=0.01))
    assert worse["state"] == "fail" and "hold_response_position" in worse["stats"]


def test_judge_puts_pairs_gate_and_overall_together_with_replaced_r_runs_in_the_gate():
    holds = [(100.0 + 300 * k, 104.0 + 300 * k) for k in range(12)]

    def arm(mode, seed, scene, t_end=3700.0, with_holds=True, **k):
        r = _run(t_end, holds=holds if with_holds else (), **k)
        r.slam_mode, r.seed, r.scene = mode, seed, scene
        return r

    runs = {}
    for s in ("line_m", "random_m"):
        for n in (3, 4):
            runs[f"L_{s}_{n}"] = arm("localization", n, s, dx=0.02)
            runs[f"R_{s}_{n}"] = arm("mapping", n, s, dx=0.04)
    runs["R_line_m_4_short"] = arm("mapping", 4, "line_m", t_end=500.0, with_holds=False, dx=0.04)
    runs["R_line_m_3"] = arm("mapping", 3, "line_m", t_end=500.0, with_holds=False, dx=0.04)  # no comparable pair
    runs["R_line_m_4_short"].failures = [{"time_s": 480.0, "reason": "Tracking timeout in transport"}]
    spec = {"scenes": {s: {"pairs": [[f"L_{s}_{n}", f"R_{s}_{n}"] for n in (3, 4)]} for s in ("line_m", "random_m")}}
    # line_m: neither original pair comparable; seed 4's R re-run (R_line_m_4) replaces the short original
    spec["scenes"]["line_m"]["pairs"][1] = {"l": "L_line_m_4", "r": "R_line_m_4", "r_original": "R_line_m_4_short"}
    out = cmp.judge(spec, loader=runs.__getitem__)
    assert out["overall"]["ok"], out["overall"]["failed"]
    assert out["scenes"]["line_m"]["gate"]["counts"]["jump_rad"][1] == 12  # the re-run's holds; the short ones none
    assert out["scenes"]["line_m"]["r_originals"][0]["failures"][0]["reason"] == "Tracking timeout in transport"


def test_v5_holds_on_the_whole_l_run_even_inside_a_comparable_span():
    # Codex tool review P2: L 0.20 m / 0.06 rad passed when R was worse on the common span
    out = cmp.pair_verdict(_run(dx=0.2, dyaw=0.06), _run(dx=0.3, dyaw=0.08))
    assert out["comparable"] and not out["ok"] and "V5" in " ".join(out["failed"])


def test_a_holding_record_outside_every_closed_interval_is_incomplete():
    # Codex P2: a holding span paired only in part passed as complete
    run = _run(1000.0, holds=[(100.0, 110.0)])
    for r in run.records:
        if 110.0 <= r["applied_tick_s"] < 140.0:
            r["mode"] = "holding"  # a holding stretch after the release with no hold event or release of its own
    _, incomplete = cmp.hold_records(run)
    assert incomplete


@pytest.mark.parametrize("bad", [None, float("nan")])
def test_a_missing_or_nan_jump_is_incomplete(bad):
    run = _run(1000.0, holds=[(100.0, 110.0)])
    run.holds[1]["jump_rad"] = bad
    _, incomplete = cmp.hold_records(run)
    assert incomplete


def test_an_empty_gate_is_insufficient_not_ok():
    assert cmp.scene_gate([], [])["state"] == "insufficient"


def test_judge_checks_the_preregistered_configuration():
    holds = [(100.0 + 300 * k, 104.0 + 300 * k) for k in range(12)]

    def arm(mode, seed, scene, **k):
        r = _run(holds=holds, **k)
        r.slam_mode, r.seed, r.scene = mode, seed, scene
        return r

    runs = {f"{a}_{s}_{n}": arm("localization" if a == "L" else "mapping", n, s, dx=0.02 if a == "L" else 0.04)
            for a in "LR" for s in ("line_m", "random_m") for n in (3, 4)}
    spec = {"scenes": {s: {"pairs": [[f"L_{s}_{n}", f"R_{s}_{n}"] for n in (3, 4)]} for s in ("line_m", "random_m")}}
    assert cmp.judge(spec, loader=runs.__getitem__)["overall"]["ok"]
    assert not cmp.judge({"scenes": {}}, loader=runs.__getitem__)["overall"]["ok"]
    one = {"scenes": {"line_m": {"pairs": [["L_line_m_3", "R_line_m_3"]]}}}
    assert not cmp.judge(one, loader=runs.__getitem__)["overall"]["ok"]
    swapped = {"scenes": {s: {"pairs": [[f"R_{s}_{n}", f"L_{s}_{n}"] for n in (3, 4)]} for s in ("line_m", "random_m")}}
    assert not cmp.judge(swapped, loader=runs.__getitem__)["overall"]["ok"]  # L must localize, R must map
    wrong_seed = dict(runs, R_line_m_4=arm("mapping", 5, "line_m", dx=0.04))
    assert not cmp.judge(spec, loader=wrong_seed.__getitem__)["overall"]["ok"]


def _config_runs():
    holds = [(100.0 + 300 * k, 104.0 + 300 * k) for k in range(12)]

    def arm(mode, seed, scene, t_end=3700.0, with_holds=True, **k):
        r = _run(t_end, holds=holds if with_holds else (), **k)
        r.slam_mode, r.seed, r.scene = mode, seed, scene
        return r

    runs = {}
    for s in ("line_m", "random_m"):
        for n in (3, 4):
            runs[f"L_{s}_{n}"] = arm("localization", n, s, dx=0.02)
            runs[f"R_{s}_{n}"] = arm("mapping", n, s, dx=0.04)
    return runs, arm


def test_judge_refuses_unknown_scenes_and_a_run_named_twice():
    runs, _ = _config_runs()
    runs["R_line_m_4"].scene = None
    spec = {"scenes": {s: {"pairs": [[f"L_{s}_{n}", f"R_{s}_{n}"] for n in (3, 4)]} for s in ("line_m", "random_m")}}
    assert not cmp.judge(spec, loader=runs.__getitem__)["overall"]["ok"]
    runs, _ = _config_runs()
    runs["R_line_m_3/."] = runs["R_line_m_3"]
    spec["scenes"]["line_m"]["pairs"][1] = {"l": "L_line_m_4", "r": "R_line_m_3/.", "r_original": "R_line_m_4"}
    assert any("twice" in p for p in cmp.judge(spec, loader=runs.__getitem__)["config_problems"])


def test_a_rerun_r_may_replace_only_a_not_comparable_r_in_a_scene_without_a_comparable_pair():
    runs, arm = _config_runs()
    # line_m: both original R short (not comparable), seed 4's R re-run long -> allowed
    runs["R_line_m_3"] = arm("mapping", 3, "line_m", t_end=500.0, with_holds=False, dx=0.04)
    runs["R_line_m_4"] = arm("mapping", 4, "line_m", t_end=500.0, with_holds=False, dx=0.04)
    runs["R_line_m_4_b"] = arm("mapping", 4, "line_m", dx=0.04)
    spec = {"scenes": {s: {"pairs": [[f"L_{s}_{n}", f"R_{s}_{n}"] for n in (3, 4)]} for s in ("line_m", "random_m")}}
    spec["scenes"]["line_m"]["pairs"][1] = {"l": "L_line_m_4", "r": "R_line_m_4_b", "r_original": "R_line_m_4"}
    out = cmp.judge(spec, loader=runs.__getitem__)
    assert out["overall"]["ok"], out["overall"]["failed"]
    # random_m: its original pair was comparable (and failing) -> a replacement is refused
    runs["L_random_m_4"] = arm("localization", 4, "random_m", dx=0.05)
    runs["R_random_m_4_b"] = arm("mapping", 4, "random_m", dx=0.08)
    spec["scenes"]["random_m"]["pairs"][1] = {"l": "L_random_m_4", "r": "R_random_m_4_b", "r_original": "R_random_m_4"}
    out = cmp.judge(spec, loader=runs.__getitem__)
    assert not out["overall"]["ok"] and any("replace" in p for p in out["config_problems"])


def test_a_non_finite_statistic_on_either_side_is_incomplete():
    runs, _ = _config_runs()
    r = runs["R_line_m_3"]
    for rec in r.records:
        if rec["mode"] == "holding":
            rec["truth_base"] = [float("inf"), 0.0, -0.001]
            break
    gate = cmp.scene_gate([runs["L_line_m_3"]], [r, runs["R_line_m_4"]])
    assert gate["state"] in ("incomplete", "insufficient")


def test_overall_needs_a_comparable_pair_in_every_scene():
    good = {"ok": True, "comparable": True, "l_absolute_ok": True}
    nc = {"ok": True, "comparable": False, "l_absolute_ok": True}
    gate = {"state": "ok"}
    assert cmp.overall({"line_m": ([good, nc], gate), "random_m": ([good, good], gate)})["ok"]
    out = cmp.overall({"line_m": ([good, good], gate), "random_m": ([nc, nc], gate)})
    assert not out["ok"] and "random_m" in " ".join(out["failed"])
    assert not cmp.overall({"line_m": ([good, good], {"state": "insufficient"}),
                            "random_m": ([good, good], gate)})["ok"]
    assert math.isclose(cmp.DRIFT_HALF_HOURS, 15)
