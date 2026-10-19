"""Real-time plan 5.10: the SLAM async on/off judge -- its randomisation tests reproduce the counterexamples of
the plan's Codex rounds 4-8, the decision table's rows, and the per-run measures from a run directory."""

import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("slam_async_ab", ROOT / "tools/slam_async_ab.py")
AB = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AB)

BLOCKS = [0, 0, 1, 1, 0, 0]  # the plan's drawn directions
ARMS = [a for b in BLOCKS for a in AB.BLOCK_ORDER[b]]


# --- the randomisation tests ---------------------------------------------------------------------------------------
def test_there_are_64_assignments_and_the_drawn_one_is_among_them():
    every = list(AB.assignments(BLOCKS))
    assert len(every) == 64 and ARMS in every and all(a.count("on") == 12 for a in every)


def test_no_gaps_anywhere_shows_nothing_and_one_burst_in_one_run_is_not_a_gain():
    assert AB.shown([0] * 24, ARMS, BLOCKS, 0.0, "<", "mean")["p"] == 1.0
    burst = [0] * 24
    burst[ARMS.index("off")] = 5  # Codex round 4: five gaps in one off run
    assert AB.shown(burst, ARMS, BLOCKS, 0.0, "<", "mean")["p"] == pytest.approx(0.5)


def test_a_drift_in_run_order_is_not_read_as_the_flag():
    """Codex round 7: the pacing ratio rises with run position and four runs (1st, 4th, 5th, 8th) are high --
    all of them on in the drawn order; only a quarter of the assignments put them all on."""
    ratio = [0.70 + 0.00001 * i + (0.20 if i in (0, 3, 4, 7) else 0.0) for i in range(24)]
    res = AB.shown(ratio, ARMS, BLOCKS, 0.0, ">", "mean")
    assert res["observed"] > 0 and res["p"] == pytest.approx(0.25) and not res["shown"]


def test_equal_values_do_not_show_non_inferiority_of_the_median():
    """Codex round 8: all values tied -- non-inferiority not shown (p = 22/64), which is no evidence of harm."""
    res = AB.shown([30.0] * 24, ARMS, BLOCKS, 10.0, "<", "median")
    assert res["p"] == pytest.approx(22 / 64) and not res["shown"]


def test_a_constant_shift_beyond_the_margin_is_not_shown_and_well_inside_it_is():
    base = [20.0 + 3.0 * ((7 * i) % 24) for i in range(24)]  # distinct, unordered values
    worse = [v + (25.0 if a == "on" else 0.0) for v, a in zip(base, ARMS)]
    assert not AB.shown(worse, ARMS, BLOCKS, 10.0, "<", "median")["shown"]
    same = [v + (-30.0 if a == "on" else 0.0) for v, a in zip(base, ARMS)]
    assert AB.shown(same, ARMS, BLOCKS, 10.0, "<", "median")["shown"]


def test_the_spread_ratio_is_reported():
    mixed = [float((7 * i) % 24) for i in range(24)]
    wide = [v * (5.0 if a == "on" else 1.0) for v, a in zip(mixed, ARMS)]
    assert AB.iqr_ratio(wide, ARMS) == pytest.approx(5.0 * AB.iqr_ratio(mixed, ARMS))
    assert AB.iqr_ratio([0.0] * 24, ARMS) is None


# --- the decision table --------------------------------------------------------------------------------------------
def run(i, arm, *, set_="decision", seed=0, ok=True, unpermitted=0, forbidden=0, e50=40.0, e99=110.0, gaps=0,
        ratio=0.74, lat=70.0, beyond=10.0, yaw=0.003, t=True, e=True, p=True):
    return {"tag": f"r{i}", "arm": arm, "set": set_, "block": i // 4 if set_ == "decision" else None, "seed": seed,
            "rerun_of": None, "success": ok,
            "failure_reason": None if ok else "x", "unpermitted": unpermitted, "forbidden": forbidden,
            "E": {"p50_mm": e50, "p99_mm": e99} if e else None,
            "T": {"gaps": gaps, "ratio": ratio} if t else None,
            "P": {"lat_min_mm": lat, "beyond_mm": beyond, "yaw_abs_rad": yaw} if p else None}


def good_runs(**over):
    """Async better on gaps, nothing else changes; values vary from run to run."""
    out = []
    for i, arm in enumerate(ARMS):
        k = (7 * i) % 24
        out.append(run(i, arm, e50=30.0 + 0.2 * k, e99=100.0 + 0.3 * k, gaps=(0 if arm == "on" else 4 + k % 3),
                       ratio=0.73 + 0.001 * k, lat=60.0 + 0.2 * k, beyond=5.0 + 0.1 * k, yaw=0.001 + 0.00002 * k,
                       **over))
    return out


def general(**over):
    return [run(24 + k, arm, set_="general", seed=seed, **over)
            for k, (seed, arm) in enumerate(((1, "on"), (1, "off"), (2, "off"), (2, "on")))]


def decide(runs, extra=None):
    return AB.judge({"blocks": BLOCKS}, runs + (general() if extra is None else extra))


def test_async_better_on_gaps_and_no_worse_elsewhere_keeps_it_on():
    v = decide(good_runs())
    assert v["outcome"] == "keep_on" and v["row"] == 9 and v["rules"]["gap_gain"]


def test_a_breach_on_turns_it_off_unless_off_breached_too():
    runs = good_runs()
    runs[0]["forbidden"] = 1  # an on run
    assert decide(runs)["outcome"] == "off"
    runs[1]["unpermitted"] = 2  # an off run as well
    assert decide(runs)["row"] == 1
    extra = general()
    extra[0]["forbidden"] = 1  # a generalisation run (on) counts for R1
    assert decide(good_runs(), extra)["outcome"] == "off"


def test_no_gain_turns_it_off_only_when_off_is_safe():
    runs = [run(i, a, gaps=0, ratio=0.74 + 0.001 * ((7 * i) % 24)) for i, a in enumerate(ARMS)]
    assert decide(runs)["row"] == 6 and decide(runs)["outcome"] == "off"
    runs[1]["forbidden"] = 1  # off breached
    assert decide(runs)["outcome"] == "user:off_arm_unsafe"


def test_a_missing_measure_stops_the_automatic_decision():
    runs = good_runs()
    runs[5]["T"] = None
    assert decide(runs)["outcome"] == "undecidable:user" and decide(runs)["row"] == 4
    runs = good_runs()
    runs[5]["P"] = None
    assert decide(runs)["row"] == 7


def test_two_failures_more_on_turn_it_off():
    runs = good_runs()
    for i in [i for i, a in enumerate(ARMS) if a == "on"][:2]:
        runs[i]["success"] = False
    assert decide(runs)["row"] == 3


def test_worse_estimates_go_to_the_user():
    runs = good_runs()
    for r in runs:
        if r["arm"] == "on":
            r["E"]["p50_mm"] += 30.0
    v = decide(runs)
    assert v["row"] == 8 and v["outcome"] == "trade_off:user" and not v["rules"]["R3"]


def test_a_gain_with_one_run_full_of_gaps_is_a_trade_off():
    runs = good_runs()
    runs[0]["T"]["gaps"] = 12  # one on run over the off maximum
    assert decide(runs)["row"] == 5


def test_arms_must_follow_the_drawn_blocks():
    runs = good_runs()
    runs[0]["arm"], runs[1]["arm"] = runs[1]["arm"], runs[0]["arm"]
    with pytest.raises(ValueError, match="drawn blocks"):
        decide(runs)


# --- one run directory ---------------------------------------------------------------------------------------------
def write_run(tmp_path, *, async_=True, failure=None, arm="on"):
    d = tmp_path / "r" / "run"
    d.mkdir(parents=True)
    recs = [{"mode": "warming_up", "stamp_s": 0.1, "link": "lockstep", "applied_map_from_odom": [9.0, 0.0, 0.0],
             "odom_base": [0.0, 0.0, 0.0], "truth_base": [0.0, 0.0, 0.0]}]
    for k in range(1, 6):
        r = {"mode": "tracking", "stamp_s": 0.4 + 0.1 * k, "odom_base": [0.1 * k, 0.0, 0.0],
             "truth_base": [0.1 * k + 0.02, 0.0, 0.0], "phase": "observe"}
        if async_:
            r.update(link="async", scan_correction=[0.0, 0.0, 0.0], applied_after=[0.01, 0.0, 0.0])
        else:
            r.update(link="lockstep", applied_map_from_odom=[0.0, 0.0, 0.0])
        recs.append(r)
    (d / "slam_records.json").write_text(json.dumps(recs))
    lines, mono = [], 100.0
    for k in range(40):
        t = 0.1 * k
        mono += 1.2 if k == 2 else (0.5 if k == 20 else 0.1)  # a warm-up wait at 0.2 s, a driving gap at 2.0 s
        lines.append({"tick_t": t, "mono": mono, "mode": "AUTO", "curvature_inv_m": 0.0})
    (d / "live_log.jsonl").write_text("\n".join(json.dumps(x) for x in lines))
    result = {"success": failure is None, "failure_reason": failure, "forbidden_pocket_contacts": [],
              "obstacle_layer": {"unpermitted": []},
              "lateral_clearance_min_m": {"insert": {"left": 0.07, "right": 0.06}},
              "insertion_measured": {"left": {"beyond_centre_m": 0.01}, "right": {"beyond_centre_m": 0.02}},
              "insertion_truth_yaw_rad": -0.002, "insertion_error": {"position_m": 0.007}}
    if async_:
        result["slam_async"] = {"prewarm_steps": 49}
    (d / "result.json").write_text(json.dumps(result))
    return {"tag": "r", "arm": arm, "set": "decision", "block": 0, "dir": str(tmp_path / "r")}


def test_measures_from_a_run_directory(tmp_path):
    m = AB.measure(write_run(tmp_path))
    assert m["E"]["n"] == 5 and m["E"]["p50_mm"] == pytest.approx(20.0)  # the warm-up record is left out
    assert m["E"]["lag_over_20mm"] == 0 and m["E"]["lag_max_mm"] == pytest.approx(10.0)
    assert m["T"]["gaps"] == 1 and m["T"]["gap_s"] == [0.5]  # the wait before the first tracking scan is not counted
    assert m["T"]["warmup_gaps"] == 1
    assert m["P"] == pytest.approx({"lat_min_mm": 60.0, "beyond_mm": 15.0, "yaw_abs_rad": 0.002,
                                    "tracking_error_mm": 7.0})
    assert m["unpermitted"] == 0 and m["forbidden"] == 0 and m["slam_async_record"]


def test_a_link_failure_gives_no_time_or_estimate_and_an_off_run_with_async_records_is_refused(tmp_path):
    m = AB.measure(write_run(tmp_path, failure="slam_link_failed: scan 9: no reply within 1.00 s"))
    assert m["T"] is None and m["E"] is None and not m["success"]
    with pytest.raises(ValueError, match="off run carries async SLAM records"):
        AB.measure(write_run(tmp_path / "b", arm="off"))


def test_only_the_plans_set_of_runs_is_judged():
    """Codex 5.10 script review P2: five blocks, no generalisation runs or a repeated tag were judged."""
    with pytest.raises(ValueError, match="drawn"):
        AB.judge({"blocks": BLOCKS[:5]}, good_runs()[:20] + general())
    with pytest.raises(ValueError, match="generalisation"):
        decide(good_runs(), [])
    runs = good_runs()
    runs[3]["tag"] = runs[2]["tag"]
    with pytest.raises(ValueError, match="twice"):
        decide(runs)


def test_a_missing_safety_field_is_unknown_and_blocks_both_choices():
    runs = good_runs()
    runs[0]["forbidden"] = None  # an on run whose contacts were not recorded
    assert not decide(runs)["rules"]["R1"]
    runs = [run(i, a, gaps=0, ratio=0.74 + 0.001 * ((7 * i) % 24)) for i, a in enumerate(ARMS)]
    runs[1]["unpermitted"] = None  # an off run: no automatic turn to off
    assert decide(runs)["outcome"] == "user:off_arm_unsafe"


def test_a_non_finite_value_stops_the_automatic_decision():
    runs = good_runs()
    runs[0]["E"]["p50_mm"] = float("nan")
    assert decide(runs)["row"] == 7
    with pytest.raises(ValueError, match="non-finite"):
        AB.shown([float("nan")] + [1.0] * 23, ARMS, BLOCKS, 0.0, "<", "mean")


def test_every_estimate_and_insertion_test_is_reported_even_after_one_fails():
    runs = good_runs()
    for r in runs:
        if r["arm"] == "on":
            r["E"]["p50_mm"] += 30.0
    assert {"e_p50<10", "e_p99<15", "lat_min>-10", "beyond>-10", "beyond<10", "yaw<0.005"} <= set(
        decide(runs)["tests"])


def test_the_records_must_agree_with_the_scheduled_arm(tmp_path):
    with pytest.raises(ValueError, match="off run carries"):
        AB.measure(write_run(tmp_path / "a", arm="off"))
    spec = write_run(tmp_path / "b")
    result = json.loads((Path(spec["dir"]) / "run/result.json").read_text())
    del result["slam_async"]
    (Path(spec["dir"]) / "run/result.json").write_text(json.dumps(result))
    with pytest.raises(ValueError, match="no slam_async"):
        AB.measure(spec)


def test_insertion_needs_both_forks_and_survives_null_descriptives(tmp_path):
    good = {"lateral_clearance_min_m": {"insert": {"left": 0.07, "right": 0.06}},
            "insertion_measured": {"left": {"beyond_centre_m": 0.01}, "right": {"beyond_centre_m": 0.02}},
            "insertion_truth_yaw_rad": 0.001, "insertion_error": {"position_m": None}}
    assert AB.insertion(good)["tracking_error_mm"] is None
    one = json.loads(json.dumps(good))
    del one["insertion_measured"]["right"]
    assert AB.insertion(one) is None
    null = json.loads(json.dumps(good))
    null["insertion_measured"]["left"]["beyond_centre_m"] = None
    assert AB.insertion(null) is None


def test_a_jump_is_timed_at_its_application_tick_and_its_first_command_counts():
    e = {"jumps_mm": [(50.0, 1.0)]}
    t = {"curvature": [(0.9, 0.0), (1.05, 1.0), (1.2, 1.0)]}
    assert AB.jumps(e, t)["curvature_step_after_max"] == pytest.approx(1.0)


def test_a_non_finite_side_and_a_wrong_decision_seed_are_refused():
    """Codex round 10: one NaN side hid behind min(); decision runs on another seed were judged."""
    side = {"lateral_clearance_min_m": {"insert": {"left": 0.06, "right": float("nan")}},
            "insertion_measured": {"left": {"beyond_centre_m": 0.01}, "right": {"beyond_centre_m": 0.02}},
            "insertion_truth_yaw_rad": 0.001}
    assert AB.insertion(side) is None
    runs = good_runs()
    runs[4]["seed"] = 99
    with pytest.raises(ValueError, match="seed 0"):
        decide(runs)

