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
    off, estimated = rd.pace_offsets(recs)
    assert off == pytest.approx([0.0, 0.25, 0.5, 0.75, 1.0, 1.25]) and estimated == 0


def test_pace_offsets_fill_scans_without_a_send_time_by_stamp_and_count_them():
    # the start-up scans go through the synchronous link and carry no sent_mono
    recs = [{"scan_id": i, "stamp_s": 0.1 * i} for i in range(3)]
    recs += [{"scan_id": i, "stamp_s": 0.1 * i, "sent_mono": 50.0 + 0.5 * (i - 3)} for i in range(3, 6)]
    recs.insert(4, {"scan_id": 99, "stamp_s": 0.35})  # a gap inside the timed part, filled between its neighbours
    off, estimated = rd.pace_offsets(recs)
    assert estimated == 4
    assert off[:3] == pytest.approx([0.0, 0.0, 0.0])  # held at the first timed scan
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
