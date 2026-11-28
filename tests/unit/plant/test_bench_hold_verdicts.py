"""bench7 hold verdict ② and the deadband-entry window (bench console plan 4.3 A1b-2 decisions 7 and 9)."""

from forklift_core.mcu_link.messages import BenchDriveState, BenchRunFlag, BenchState, Mode
from forklift_core.plant import bench_session as bs_

RUN = 0xC0FFEE
TARGET = 1000


def bstate(cum, *, drive=BenchDriveState.DRIVING, raw=100, result=0, stop=0, flags=0, age=5, entries=0, run=RUN):
    return BenchState(1, run, result, 2, 1, 1, 0, raw, cum, TARGET, 0, 0, stop, 0, 0, 0, age, age, 0,
                      int(drive) | (2 << 4), 0, flags, 2, 1, entries, 0)


def series(errs, t0=0, dt=10_000):
    """(mcu_time_us, mode, BenchState) driving pairs at 100 Hz for the given errors (target - cumulative)."""
    return [(t0 + i * dt, Mode.ARMED, bstate(TARGET - e)) for i, e in enumerate(errs)]


def ended(t, cut_err=0):
    return (t, Mode.STOPPING, bstate(TARGET - cut_err, drive=BenchDriveState.ENDED, result=RUN, stop=1))


RESULT_OK = {"stop_reason": "end_zero", "run_flags": int(BenchRunFlag.RES_HOLD), "end_evidence_age_ms": 5,
             "target": TARGET, "cut_position": TARGET - 3}


def verdict(pairs, result=None, db=12):
    return bs_.hold_verdict(pairs, RESULT_OK if result is None else result, deadband=db, run_id=RUN)


def test_a_settled_hold_is_met():
    pairs = series([40] * 10 + [5] * 150)
    pairs.append(ended(pairs[-1][0] + 10_000, 3))
    v = verdict(pairs)
    assert v["result"] == "met", v
    assert v["end_error"] == 3


def test_failure_comes_first_then_hold_back():
    pairs = series([5] * 150) + [ended(1_500_000)]
    assert verdict(pairs, {**RESULT_OK, "stop_reason": "no_progress"})["result"] == "failed"
    assert verdict(pairs, {**RESULT_OK, "stop_reason": "kth_lost"})["result"] == "failed"
    unclean = {**RESULT_OK, "run_flags": int(BenchRunFlag.RES_HOLD | BenchRunFlag.RES_UNCLEAN)}
    assert verdict(pairs, unclean)["result"] == "failed"
    assert verdict(pairs, {**RESULT_OK, "end_evidence_age_ms": 0xFFFF})["result"] == "held"
    assert verdict(pairs, {**RESULT_OK, "end_evidence_age_ms": 101})["result"] == "held"
    tentative = {**RESULT_OK, "run_flags": int(BenchRunFlag.RES_HOLD | BenchRunFlag.RES_TENTATIVE)}
    assert verdict(pairs, tentative)["result"] == "held"


def test_the_end_error_is_the_result_not_a_later_sample():
    pairs = series([5] * 150) + [ended(1_500_000)]
    v = verdict(pairs, {**RESULT_OK, "cut_position": TARGET - 13})
    assert v["result"] == "unmet" and "end_outside" in v["why"]


def test_undecidable_without_settings_or_samples():
    pairs = series([5] * 150) + [ended(1_500_000)]
    assert bs_.hold_verdict(pairs, RESULT_OK, deadband=None, run_id=RUN)["result"] == "undecidable"
    assert bs_.hold_verdict([], RESULT_OK, deadband=12, run_id=RUN)["result"] == "undecidable"


def test_counterexample_1_late_settling_is_unmet():
    # 0-1000 ms, 101 samples, the first 20 outside, 81 inside -> settled from 200 ms: 800 ms
    pairs = series([30] * 20 + [5] * 81) + [ended(1_010_000)]
    assert verdict(pairs)["result"] == "unmet"


def test_counterexample_2_one_returned_sample_is_unmet():
    pairs = series([5] * 150 + [40] * 20 + [5]) + [ended(1_710_000)]
    assert verdict(pairs)["result"] == "unmet"


def test_the_settling_right_edge_is_the_last_driving_pair():
    # last driving 990 ms, first ended 1000 ms: counted to 990 ms (1 s from 0 needs the driving pair at 1000 ms)
    pairs = series([5] * 100) + [ended(1_000_000)]  # 0..990 ms: 990 ms < 1 s
    assert verdict(pairs)["result"] == "unmet"
    pairs = series([5] * 101) + [ended(1_010_000)]  # 0..1000 ms
    assert verdict(pairs)["result"] == "met"


def test_jitter_is_joined_and_a_lost_pair_breaks():
    times, t = [], 0
    for i in range(130):
        times.append(t)
        t += 11_000 if i % 2 else 14_000  # 11-14 ms jitter: joined
    pairs = [(tt, Mode.ARMED, bstate(TARGET - 5)) for tt in times] + [ended(times[-1] + 10_000)]
    assert verdict(pairs)["result"] == "met"
    pairs = series([5] * 150)
    del pairs[140]  # one pair lost near the end: 20 ms gap -> the segment restarts after it
    pairs.append(ended(pairs[-1][0] + 10_000))
    assert verdict(pairs)["result"] == "unmet"


def test_a_gap_before_the_end_is_unmet():
    pairs = series([5] * 150) + [ended(1_490_000 + 30_000)]
    v = verdict(pairs)
    assert v["result"] == "unmet" and "end_gap" in v["why"]


def test_a_tentative_zero_sample_is_never_inside():
    pairs = [(t, m, bstate(b.cumulative, raw=0)) for t, m, b in series([0] * 150)] + [ended(1_500_000)]
    assert verdict(pairs)["result"] == "unmet"


def test_the_deadband_edge_is_inside_with_le():
    for err, want in ((11, "met"), (12, "met"), (13, "unmet")):
        pairs = series([err] * 150) + [ended(1_500_000, cut_err=min(err, 12))]
        assert verdict(pairs)["result"] == want, err
    pairs = series([0] * 150) + [ended(1_500_000, cut_err=0)]
    assert verdict(pairs, {**RESULT_OK, "cut_position": TARGET}, db=0)["result"] == "met"
    pairs = series([1] * 150) + [ended(1_500_000, cut_err=0)]
    assert verdict(pairs, {**RESULT_OK, "cut_position": TARGET}, db=0)["result"] == "unmet"


# ---- the deadband-entry window (decision 7, rounds 10-12) -----------------------------------------------------------
def test_entry_window_wraps_mod_65536():
    w = bs_.entry_window([(0, 65530), (200_000, 65534), (1_100_000, 4)], 1_100_000)
    assert (w["lower"], w["upper"]) == (6, 10)


def test_entry_window_across_its_start_is_unknown():
    w = bs_.entry_window([(0, 0), (40_000, 10), (1_020_000, 10)], 1_020_000)
    assert (w["lower"], w["upper"]) == (0, 10) and w["label"] == "unknown"


def test_entry_window_warns_on_its_lower_bound():
    pts = [(0, 0)] + [(i * 10_000, i) for i in range(1, 120)]
    w = bs_.entry_window(pts, 119 * 10_000)
    assert w["lower"] >= 10 and w["label"] == "warn"


def test_entry_window_with_the_arm_seen():
    # round 12: the last pre-run pair at 700 ms, the run's first at 900 ms with 10, now 1100 ms with 10 -> 10/10
    w = bs_.entry_window([(700_000, 0, True), (900_000, 10), (1_100_000, 10)], 1_100_000)
    assert (w["lower"], w["upper"], w["label"]) == (10, 10, "warn")
    # the pre-run pair at 50 ms: the ARM may be before the window -> 0..10 unknown
    w = bs_.entry_window([(50_000, 0, True), (900_000, 10), (1_100_000, 10)], 1_100_000)
    assert (w["lower"], w["upper"], w["label"]) == (0, 10, "unknown")
    # round 11: right after the ARM, a 1 s window from the zero base
    w = bs_.entry_window([(0, 0, True), (500_000, 8)], 500_000)
    assert (w["lower"], w["upper"]) == (8, 8) and w["label"] == "none"


def test_entry_window_without_a_base_or_across_a_long_gap_is_unknown():
    assert bs_.entry_window([(900_000, 10), (1_100_000, 10)], 1_100_000)["label"] == "unknown"  # ARM not seen
    w = bs_.entry_window([(0, 0, True), (70_000_000, 3), (70_500_000, 3)], 70_500_000)
    assert w["label"] in ("unknown", "none")


def test_entry_points_keep_one_run():
    # the previous run's 65534 never mixes with the new run's 8 (round 11)
    pairs = [(0, Mode.DISARMED, bstate(0, drive=BenchDriveState.ENDED, entries=65534, run=7)),
             (100_000, Mode.ARMED, bstate(0, entries=0)), (600_000, Mode.ARMED, bstate(0, entries=8))]
    pts = bs_.entry_points(pairs, RUN)
    w = bs_.entry_window(pts, 600_000)
    assert (w["lower"], w["upper"]) == (8, 8)
