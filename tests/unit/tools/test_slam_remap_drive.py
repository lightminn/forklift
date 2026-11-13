"""slam_link_failed plan E0 (docs/plans/2026-10-25-slam-link-failed.md 3): the replay's live pacing (scan wall times
rebuilt from the bridge's /map arrivals), the splice point (a later scan back at the start pose) and the spliced
stream (stamps monotonic, odometry continued by relative increments, truth repeated), and a failed run's unanswered
tail."""

import math

import numpy as np
import pytest

from tools import slam_remap_drive as rd


def test_pace_offsets_are_the_records_own_send_times():
    recs = [{"scan_id": i, "stamp_s": 0.1 * i, "sent_mono": 100.0 + 0.25 * i} for i in range(6)]
    off, estimated, first = rd.pace_offsets(recs)
    assert off == pytest.approx([0.0, 0.25, 0.5, 0.75, 1.0, 1.25]) and estimated == 0 and first == 0


def test_pace_offsets_refuse_a_decreasing_or_missing_send_time():
    recs = [{"scan_id": i, "stamp_s": 0.1 * i, "sent_mono": 10.0 + i} for i in range(4)]
    recs[2]["sent_mono"] = 9.0
    with pytest.raises(SystemExit):
        rd.pace_offsets(recs)
    recs[2]["sent_mono"] = float("nan")
    with pytest.raises(SystemExit):
        rd.pace_offsets(recs)


def test_planned_wall_covers_the_last_due_scan_of_the_spliced_units():
    truth = _loop_truth(11)
    plan = rd.splice_plan([0.1 * i for i in range(11)], truth, truth, end=10, until_sim_s=2.5)
    pace = [0.25 * i for i in range(11)]
    assert rd.planned_wall_s(plan, pace, unit_wall=2.5) == pytest.approx(2.5 * 2 + 0.25 * 5)


def test_pace_offsets_fill_scans_without_a_send_time_by_stamp_and_count_them():
    # the start-up scans go through the synchronous link and carry no sent_mono
    recs = [{"scan_id": i, "stamp_s": 0.1 * i} for i in range(3)]
    recs += [{"scan_id": i, "stamp_s": 0.1 * i, "sent_mono": 50.0 + 0.5 * (i - 3)} for i in range(3, 6)]
    recs.insert(4, {"scan_id": 99, "stamp_s": 0.35})  # a gap inside the timed part, filled between its neighbours
    off, estimated, first = rd.pace_offsets(recs)
    assert estimated == 4 and first == 3
    assert off[:4] == pytest.approx([0.0, 0.0, 0.0, 0.0])  # before the first timed scan: no waiting
    assert off[4] == pytest.approx(0.25)  # halfway in stamp between 0.3 (0.0) and 0.4 (0.5)
    assert np.all(np.diff(off) >= 0)
    with pytest.raises(SystemExit):
        rd.pace_offsets([{"scan_id": 0, "stamp_s": 0.0}])


def _loop_truth(n=200):
    t = np.linspace(0.0, 2 * math.pi, n)
    return [(2.0 * math.cos(a) - 2.0, 2.0 * math.sin(a), a + math.pi / 2) for a in t]


def test_splice_end_is_the_last_scan_back_at_the_start_pose():
    truth = _loop_truth()
    k = rd.splice_end(truth)
    assert k == len(truth) - 1
    off = list(truth)
    off[-1] = (truth[-1][0] + 0.2, truth[-1][1], truth[-1][2])
    assert rd.splice_end(off) < len(truth) - 1
    with pytest.raises(SystemExit):
        rd.splice_end([(0, 0, 0)] + [(5.0, 5.0, 0.0)] * 50)


def test_spliced_stream_is_monotonic_and_continues_odometry():
    truth = _loop_truth(101)
    odom = [(x + 0.01 * i, y, yaw) for i, (x, y, yaw) in enumerate(truth)]  # drifting odometry
    stamps = [0.1 * i for i in range(101)]
    plan = rd.splice_plan(stamps, odom, truth, end=100, until_sim_s=25.0)
    st = [p.stamp_s for p in plan]
    assert all(b > a for a, b in zip(st, st[1:]))
    assert st[-1] >= 25.0 - 0.1 and st[-1] < 25.0 + 10.0
    assert [p.unit for p in plan[:101]] == [0] * 100 + [1]
    # within a unit the odometry increments are the source's; across the seam it is continuous
    first, second = plan[100], plan[101]
    assert first.src == 0 and second.src == 1
    inc_src = rd.relative(odom[0], odom[1])
    inc_out = rd.relative(first.odom, second.odom)
    assert inc_out == pytest.approx(inc_src)
    seam = rd.relative(plan[99].odom, plan[100].odom)
    assert seam == pytest.approx(rd.relative(odom[99], odom[100]))
    assert plan[150].truth == truth[50]


def test_no_splice_replays_the_record_once():
    truth = _loop_truth(11)
    plan = rd.splice_plan([0.1 * i for i in range(11)], truth, truth, end=None, until_sim_s=None)
    assert [p.src for p in plan] == list(range(11)) and {p.unit for p in plan} == {0}


def test_unanswered_tail_is_accepted_only_when_allowed():
    assert rd.answered_length(4874, 4878, allow_tail=8) == 4874
    with pytest.raises(SystemExit):
        rd.answered_length(4874, 4878, allow_tail=0)
    with pytest.raises(SystemExit):
        rd.answered_length(4874, 4890, allow_tail=8)
    assert rd.answered_length(100, 100, allow_tail=0) == 100


def _source(name, n, shift):
    truth = [(x + shift, y, yaw) for x, y, yaw in _loop_truth(n)]
    odom = [(x + 0.01 * i, y, yaw) for i, (x, y, yaw) in enumerate(truth)]
    return rd.Source(name=name, stamps=[0.1 * i for i in range(n)], odom=odom, truth=truth, ranges=None, end=n - 1)


def test_mixed_splice_alternates_source_runs_and_keeps_their_ids():
    # plan 2026-10-25-slam-bounded-graph 4.1: units taken in turn from different runs of the same scene
    a, b = _source("a", 101, 0.0), _source("b", 51, 0.001)
    plan = rd.splice_plan_multi([a, b], until_sim_s=40.0)
    runs = [p.run for p in plan]
    assert runs[:100] == [0] * 100 and runs[100:150] == [1] * 50 and runs[150] == 0
    st = [p.stamp_s for p in plan]
    assert all(y > x for x, y in zip(st, st[1:])) and st[-1] <= 40.0
    # a unit's truth is its own run's, its scan ids the run's own
    assert plan[100].truth == b.truth[0] and plan[100].src == 0 and plan[149].src == 49
    # across the seam the odometry continues by the previous run's step into its return scan
    assert rd.relative(plan[99].odom, plan[100].odom) == pytest.approx(rd.relative(a.odom[99], a.odom[100]))
    # within a unit, the run's own increments
    assert rd.relative(plan[100].odom, plan[101].odom) == pytest.approx(rd.relative(b.odom[0], b.odom[1]))
    # the next unit of run a starts after b's period
    assert plan[150].stamp_s == pytest.approx(10.0 + 5.0)


def test_one_source_matches_the_single_run_splice():
    truth = _loop_truth(101)
    odom = [(x + 0.01 * i, y, yaw) for i, (x, y, yaw) in enumerate(truth)]
    stamps = [0.1 * i for i in range(101)]
    single = rd.splice_plan(stamps, odom, truth, end=100, until_sim_s=25.0)
    multi = rd.splice_plan_multi([rd.Source("a", stamps, odom, truth, None, 100)], until_sim_s=25.0)
    assert [(p.src, p.unit, p.stamp_s, p.odom) for p in single] == [(p.src, p.unit, p.stamp_s, p.odom) for p in multi]


def test_first_unit_only_is_the_record_up_to_its_return_scan():
    # bounded-graph plan candidate A: the map is built from one run's first unit
    truth = _loop_truth(101)
    plan = rd.first_unit_plan([0.1 * i for i in range(101)], truth, truth, end=100)
    assert [p.src for p in plan] == list(range(100)) and {p.unit for p in plan} == {0}


# --- online-SLAM plan (docs/plans/2026-11-13-slam-online-bounded.md 5 S1): H's switch and the paced splice -----------

def _ranged_source(name, n, shift, seed):
    src = _source(name, n, shift)
    ranges = np.random.default_rng(seed).uniform(1.0, 5.0, (n, 8))
    return rd.Source(src.name, src.stamps, src.odom, src.truth, ranges, src.end)


def test_a_replay_from_a_later_unit_draws_the_same_noise_as_the_whole_stream():
    """H's second phase sends units 1.. only; each of its scans must carry the noise the whole stream gives it, so the
    arms see identical scans (the skipped units' draws are taken and dropped)."""
    a, b = _ranged_source("a", 31, 0.0, 1), _ranged_source("b", 21, 0.001, 2)
    plan = rd.splice_plan_multi([a, b], until_sim_s=12.0)
    whole = list(rd.noisy_stream(plan, [a, b], seed=5))
    later = list(rd.noisy_stream(plan, [a, b], seed=5, start_unit=1))
    assert [k for k, _, _ in later] == [k for k, item, _ in whole if item.unit >= 1]
    by_k = {k: r for k, _, r in whole}
    assert all(np.array_equal(r, by_k[k]) for k, _, r in later)
    assert len(later) < len(whole) and later[0][1].unit == 1
    first = list(rd.noisy_stream(plan, [a, b], seed=5, end_unit=1))  # H's first phase: unit 0 only
    assert [k for k, _, _ in first] + [k for k, _, _ in later] == [k for k, _, _ in whole]
    assert all(np.array_equal(r, by_k[k]) for k, _, r in first)
    assert len(rd.sim_due_offsets(plan, factor=1.0, end_unit=1)) == len(first)


def test_sim_pacing_sends_each_scan_at_its_stamp_over_the_factor():
    # the D arms' fixed feed (plan 4: the paced replay at factor 1.0), from the first scan sent
    a = _source("a", 31, 0.0)
    plan = rd.splice_plan_multi([a], until_sim_s=8.0)
    due = rd.sim_due_offsets(plan, factor=1.0)
    assert due[0] == 0.0 and all(y >= x for x, y in zip(due, due[1:]))
    assert due[-1] == pytest.approx(plan[-1].stamp_s - plan[0].stamp_s)
    half = rd.sim_due_offsets(plan, factor=0.5)
    assert half[-1] == pytest.approx(2 * due[-1])
    later = rd.sim_due_offsets(plan, factor=1.0, start_unit=1)
    assert later[0] == 0.0 and len(later) == sum(1 for p in plan if p.unit >= 1)
    with pytest.raises(SystemExit):
        rd.sim_due_offsets(plan, factor=0.0)


def test_the_switched_localization_uses_only_this_sessions_unit_0_map(tmp_path):
    """H (plan 1-H, lead 2026-11-13): after the switch the localization must run on the map this session built from
    its own unit 0 -- never a registry bundle or another run's map."""
    run = tmp_path / "run"
    run.mkdir()
    good = {"format": 1, "map_id": "abc", "replay": True, "source": {"run": str(run), "units": [0]}}
    assert rd.check_session_bundle(good, run=run, map_id="abc") is None
    for bad in ({**good, "replay": False}, {**good, "source": {"run": str(tmp_path / "other"), "units": [0]}},
                {**good, "source": {"run": str(run), "units": [0, 1]}}, {**good, "map_id": "zzz"},
                {k: v for k, v in good.items() if k != "source"}):
        with pytest.raises(SystemExit):
            rd.check_session_bundle(bad, run=run, map_id="abc")


def test_the_fixed_feed_queues_scans_on_time_behind_a_slow_bridge_and_records_every_reply():
    import socket
    import threading
    import time

    from forklift_core.localization import slam_link

    a = _ranged_source("a", 31, 0.0, 1)
    plan = rd.splice_plan_multi([a], until_sim_s=3.0)[:12]
    due = rd.sim_due_offsets(plan, factor=20.0)  # stamps 0.1 s apart -> 5 ms apart
    ours, theirs = socket.socketpair()
    seen = []

    def bridge():
        for _ in plan:
            scan = slam_link.recv_scan(theirs)
            seen.append(scan.scan_id)
            if len(seen) == 3:
                time.sleep(0.08)  # one slow loop closure: the next scans wait in the queue
            status = "skipped" if len(seen) == 5 else "processed"
            theirs.sendall(slam_link.encode_reply(slam_link.Reply(scan.scan_id, scan.stamp_s, status,
                                                                  (0.0, 0.0, 0.0), 1, 0.001)))

    t = threading.Thread(target=bridge)
    t.start()
    client = object.__new__(slam_link.SlamLinkClient)
    client.sock = ours
    rows = []
    anchor = time.monotonic()
    stats = rd.feed_sim(client, rd.noisy_stream(plan, [a], seed=3), due,
                        lambda k, item, reply, q, s, r: rows.append((k, q, s, r)))
    t.join(5)
    assert seen == list(range(len(plan)))
    assert [k for k, *_ in rows] == [k for k in range(len(plan)) if k != 4]
    assert stats == {"sent": len(plan), "not_processed": 1, "max_depth": stats["max_depth"], "long_poll_gaps": 0}
    assert stats["max_depth"] >= 3  # scans kept being queued while the bridge was busy
    assert all(q <= s <= r for _, q, s, r in rows)
    assert all(q - anchor >= due[k] - 1e-3 for k, q, _, _ in rows)  # never queued before its due time
    with pytest.raises(ValueError):  # a due list that does not match the stream is refused
        rd.feed_sim(client, rd.noisy_stream(plan, [a], seed=3), due[:-1], lambda *x: None)
