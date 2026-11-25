"""PR 7 design 5 (7b): ImuWatch on acquisition intervals ``[lo, hi]`` and producer generations. ``lo == hi`` with no
generation is the 5b contract unchanged; wider intervals are judged at their worst (freshness from ``lo``, a gap as
the next ``hi`` minus the previous ``lo``, the ARMING rule for any sample that may precede READY, a tilt hold at its
longest possible length); a new generation is never joined to the old one."""

import math

from forklift_core.safety.real_watch import ImuWatch

from test_imu_watch import CFG, G, MS, armed, fed, tilted


def fed_iv(watch, start_ms, n, *, width_ms=0, accel=G, step_ms=5, gen=None):
    """``n`` samples every ``step_ms``: each ``[t - width, t]``, added at ``t``."""
    t = start_ms
    for _ in range(n):
        assert watch.add_interval(gen, (t - width_ms) * MS, t * MS, accel, t * MS) is None
        t += step_ms
    return t


def armed_iv(width_ms=0, gen=None):
    w = ImuWatch(CFG)
    end = fed_iv(w, 1000, 20, width_ms=width_ms, gen=gen)
    b, why = w.baseline(1000 * MS)
    assert why is None, why
    w.arm(b)
    w.settled((end - 5) * MS)
    return w, end


# --- lo == hi is the 5b watch ----------------------------------------------------------------------------------------
def test_point_intervals_judge_exactly_as_the_point_samples():
    a, end = armed()
    b, end_b = armed_iv()
    assert end == end_b
    trace_a, trace_b = [], []
    t = end
    for k in range(80):
        accel = tilted(12) if 10 <= k < 75 else G
        assert a.add(t * MS, accel, t * MS) is None
        assert b.add_interval(None, t * MS, t * MS, accel, t * MS) is None
        trace_a.append(a.judge(t * MS))
        trace_b.append(b.judge(t * MS))
        t += 5
    assert trace_a == trace_b and "watch_tilt" in trace_a


def test_add_is_the_point_interval_without_a_generation():
    w = ImuWatch(CFG)
    w.add(5 * MS, G, 5 * MS)
    hi, unit, lo, gen, _serial = w.samples[-1]
    assert (hi, lo, gen) == (5 * MS, 5 * MS, None) and math.isclose(unit[2], 1.0)


# --- validity --------------------------------------------------------------------------------------------------------
def test_an_interval_ending_before_it_starts_is_not_a_sample():
    w = ImuWatch(CFG)
    assert w.add_interval(None, 10 * MS, 9 * MS, G, 20 * MS) == "bad_interval"
    assert w.counts["bad_interval"] == 1 and not w.samples


def test_newer_and_future_are_judged_on_the_interval_end():
    w = ImuWatch(CFG)
    assert w.add_interval(None, 0, 10 * MS, G, 10 * MS) is None
    assert w.add_interval(None, 5 * MS, 10 * MS, G, 20 * MS) == "not_newer"  # the same end: the same sample again
    assert w.add_interval(None, 5 * MS, 21 * MS, G, 20 * MS) == "future"  # may be taken after the read
    assert w.add_interval(None, 5 * MS, 15 * MS, G, 20 * MS) is None  # a later end, even with an earlier start


# --- the judgements at their worst ----------------------------------------------------------------------------------
def test_a_gap_is_the_next_end_minus_the_previous_start():
    """design 5's example: [0, 10] then [49, 59] ms -- 49 ms between starts, but up to 59 ms apart: over 50."""
    w, end = armed_iv(width_ms=10)
    t = end + 44  # the previous sample was [end-15, end-5]: the next [end+34, end+44] -> 59 ms
    assert w.add_interval(None, (t - 10) * MS, t * MS, G, t * MS) is None
    assert w.judge(t * MS) == "watch_imu_gap"


def test_the_same_spacing_with_point_samples_is_no_gap():
    w, end = armed()
    t = end + 44  # 49 ms after the last point sample
    assert w.add(t * MS, G, t * MS) is None
    assert w.judge(t * MS) is None


def test_freshness_counts_from_the_interval_start():
    w, end = armed_iv(width_ms=30)
    last_lo = end - 5 - 30
    assert w.judge((last_lo + 100) * MS) is None
    assert w.judge((last_lo + 101) * MS) == "watch_imu_stale"


def test_a_sample_that_may_precede_ready_meets_the_arming_rule():
    """READY at ``r``: a 3 deg sample [r - 1, r + 4] may have been taken while ARMING -- watch_imu_moved."""
    w = ImuWatch(CFG)
    end = fed_iv(w, 1000, 20, width_ms=0)
    b, _ = w.baseline(1000 * MS)
    w.arm(b)
    ready = end - 5
    w.settled(ready * MS)
    assert w.add_interval(None, (ready + 1 - 2) * MS, (ready + 4) * MS, tilted(3), (ready + 4) * MS) is None
    assert w.judge((ready + 4) * MS) == "watch_imu_moved"


def test_a_sample_wholly_after_ready_takes_the_tilt_hold():
    w = ImuWatch(CFG)
    end = fed_iv(w, 1000, 20, width_ms=0)
    b, _ = w.baseline(1000 * MS)
    w.arm(b)
    ready = end - 5
    w.settled(ready * MS)
    assert w.add_interval(None, (ready + 1) * MS, (ready + 4) * MS, tilted(3), (ready + 4) * MS) is None
    assert w.judge((ready + 4) * MS) is None


def test_the_tilt_hold_trips_at_its_longest_possible_length():
    """tilted samples 5 ms apart, each 20 ms wide: the run may have lasted from the first start to the last end --
    0.3 s is reached 20 ms before the point-sample watch would see it."""
    w, end = armed_iv(width_ms=20)
    t = first = end + 20  # starts after READY (end - 5); 45 ms from the baseline's last start
    trips = []
    while not trips and t < end + 400:
        w.add_interval(None, (t - 20) * MS, t * MS, tilted(12), t * MS)
        why = w.judge(t * MS)
        if why is not None:
            trips.append((t, why))
        t += 5
    (t_trip, why), = trips
    assert why == "watch_tilt"
    assert t_trip - (first - 20) >= 300 and t_trip - (first - 20) < 305  # the longest length reaches 0.3 s here
    assert t_trip - first < 300  # the point samples alone are not 0.3 s apart yet


def test_baseline_takes_only_samples_that_started_in_the_stand():
    w = ImuWatch(CFG)
    fed_iv(w, 1000, 20, width_ms=8)
    # still since 1001 ms: the first two samples may predate the stand -- 18 left, still >= baseline_min
    b, why = w.baseline(1001 * MS)
    assert why is None
    b2, why2 = w.baseline((1000 + 5 * 10 - 8 + 1) * MS)  # only 9 samples start inside: too few
    assert why2 == "imu_no_baseline" and b2 is None


def test_a_baseline_gap_is_judged_on_the_interval_too():
    """two runs 49 ms apart by their ends but 59 ms by end minus start: the baseline uses only the later run."""
    w = ImuWatch(CFG)
    t = fed_iv(w, 1000, 12, width_ms=10)
    t = fed_iv(w, t - 5 + 49, 4, width_ms=10)
    assert w.baseline(990 * MS) == (None, "imu_no_baseline")


# --- generations ---------------------------------------------------------------------------------------------------
def test_a_producer_restart_between_two_samples_5_ms_apart_is_a_lost_watch():
    """design 5's test: same batch, 5 ms spacing, only the producer restarted -- armed: watch_imu_gap."""
    w, end = armed_iv(gen=1)
    assert w.add_interval(1, end * MS, end * MS, G, end * MS) is None
    assert w.add_interval(2, (end + 5) * MS, (end + 5) * MS, G, (end + 5) * MS) is None
    assert w.judge((end + 5) * MS) == "watch_imu_gap"
    assert w.counts["generation"] == 1


def test_a_restart_with_no_new_sample_yet_is_a_lost_watch():
    """design 5's second test: the previous sample still fresh, the source reports a restart, no new sample."""
    w, end = armed_iv(gen=1)
    w.note_generation(2)
    assert w.judge(end * MS) == "watch_imu_gap"


def test_noting_the_same_generation_changes_nothing():
    w, end = armed_iv(gen=1)
    w.note_generation(1)
    assert w.judge(end * MS) is None and w.counts["generation"] == 0


def test_before_the_arm_a_new_generation_drops_the_old_baseline():
    w = ImuWatch(CFG)
    t = fed_iv(w, 1000, 20, gen=1)
    assert w.baseline(1000 * MS)[1] is None
    w.note_generation(2)
    assert not w.samples
    assert w.baseline(1000 * MS) == (None, "imu_no_baseline")
    t = fed_iv(w, t, CFG.baseline_min - 1, gen=2)
    assert w.baseline(1000 * MS) == (None, "imu_no_baseline")  # the old samples never complete the new baseline
    fed_iv(w, t, 1, gen=2)
    assert w.baseline(1000 * MS)[1] is None


def test_a_new_generation_sample_before_the_arm_starts_a_new_run():
    w = ImuWatch(CFG)
    t = fed_iv(w, 1000, 20, gen=1)
    assert w.add_interval(2, t * MS, t * MS, G, t * MS) is None
    assert [s[3] for s in w.samples] == [2]


def test_the_lost_watch_from_a_generation_stays_until_the_disarm():
    w, end = armed_iv(gen=1)
    w.note_generation(2)
    fed_iv(w, end, 10, gen=2)
    assert w.judge((end + 50) * MS) == "watch_imu_gap"
    w.disarm()
    assert w.judge((end + 50) * MS) is None and not w.armed


def test_generations_of_point_samples_without_one_never_change():
    w, end = armed()
    w.note_generation(None)
    assert w.judge(end * MS) is None and w.counts["generation"] == 0


def test_the_arm_after_a_pruned_baseline_sample_measures_from_its_end():
    """``arm`` given a baseline whose last sample is no longer kept falls back to that end as a point (5b's rule)."""
    w = ImuWatch(CFG)
    fed(w, 1000, 20)
    w.arm(((0.0, 0.0, 1.0), 1097 * MS))  # no kept sample ends there
    assert w.add((1097 + 49) * MS, G, (1097 + 49) * MS) is None
    assert w.judge((1097 + 49) * MS) is None
