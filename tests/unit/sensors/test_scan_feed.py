"""The real scan's host side (docs/design/2026-11-12-real-scan-obstacle-layer.md 1.2-1.3): the scan/1 frame, its
checks, NaN for invalid ranges, per-scan beam angles, the stamp clock pair with jump refusal and re-establishment, and
a raw record that replays to the same verdicts."""

import math
import socket

import numpy as np
import pytest

from forklift_core.sensors import scan_feed as sf

MS = 1_000_000
OFF = 1_700_000_000_000 * MS  # system minus monotonic


def head(k, *, n=8, recv=None, off=OFF, stamp=None, **kw):
    recv = recv if recv is not None else 10_000 * MS + k * 100 * MS
    stamp = stamp if stamp is not None else recv - 20 * MS + off  # 20 ms stamp -> receipt
    h = {"scan_id": k, "ros_stamp_ns": stamp, "recv_mono_ns": recv, "sys_minus_mono_ns": off,
         "angle_min": -math.pi, "angle_increment": 2 * math.pi / n, "time_increment": 0.1 / n, "scan_time": 0.1,
         "range_min": 0.2, "range_max": 12.0, "frame_id": "laser", "n": n}
    h.update(kw)
    return h


def frame(k, ranges=None, **kw):
    h = head(k, **kw)
    r = np.full(h["n"], 2.0, dtype=np.float32) if ranges is None else np.asarray(ranges, dtype=np.float32)
    return sf.encode_frame(h, r, None)


def settled(feed, start=0, count=sf.ESTABLISH_COUNT, **kw):
    out = []
    for k in range(start, start + count):
        out.append(feed.judge(frame(k, **kw), 0))
    return out


def test_a_frame_round_trips():
    h = head(3, n=5)
    r = np.array([1, 2, 3, 4, 5], dtype=np.float32)
    i = np.array([9, 8, 7, 6, 5], dtype=np.float32)
    h2, r2, i2 = sf.decode_frame(sf.encode_frame(h, r, i))
    assert h2["scan_id"] == 3 and np.array_equal(r2, r) and np.array_equal(i2, i)
    h3, r3, i3 = sf.decode_frame(sf.encode_frame(h, r, None))
    assert i3 is None and np.array_equal(r3, r)


def test_the_clock_is_established_before_any_scan_is_given():
    feed = sf.ScanFeed()
    verdicts = settled(feed, count=sf.ESTABLISH_COUNT - 1)
    assert all(scan is None and why == "establishing" for scan, why in verdicts)
    scan, why = feed.judge(frame(sf.ESTABLISH_COUNT), 0)
    assert why is None and scan is not None and scan.stamp_to_recv_ns == 20 * MS


def test_invalid_ranges_are_unknown_and_angles_are_per_scan():
    feed = sf.ScanFeed()
    settled(feed)
    scan, why = feed.judge(frame(20, ranges=[0.0, math.inf, 0.1, 13.0, 2.0, math.nan, 0.2, 12.0]), 0)
    assert why is None
    assert np.isnan(scan.ranges[:4]).all() and np.isnan(scan.ranges[5])
    assert scan.ranges[4] == pytest.approx(2.0) and scan.ranges[6] == pytest.approx(0.2) and scan.ranges[7] == 12.0
    assert scan.angles[0] == pytest.approx(-math.pi) and scan.angles[1] - scan.angles[0] == pytest.approx(math.pi / 4)
    scan2, _ = feed.judge(frame(21, n=4, ranges=[1, 1, 1, 1]), 0)
    assert len(scan2.angles) == 4 and scan2.angles[1] - scan2.angles[0] == pytest.approx(math.pi / 2)


@pytest.mark.parametrize("change, why", [
    ({"n": 5000}, "too_many_beams"),
    ({"angle_increment": 0.0}, "bad_meta"),
    ({"angle_increment": math.nan}, "bad_meta"),
    ({"range_min": 5.0, "range_max": 1.0}, "bad_meta"),
    ({"scan_id": "7"}, "bad_meta"),
])
def test_a_malformed_frame_is_refused_and_counted(change, why):
    feed = sf.ScanFeed()
    settled(feed)
    h = head(30, **change)
    n = h["n"] if isinstance(h["n"], int) and h["n"] <= 5000 else 8
    data = sf.encode_frame(h, np.ones(n, dtype=np.float32), None)
    scan, got = feed.judge(data, 0)
    assert scan is None and got == why and feed.refused[why] == 1


def test_a_truncated_or_foreign_frame_is_refused():
    feed = sf.ScanFeed()
    settled(feed)
    good = frame(30)
    for bad in (good[:-3], b"XXXX" + good[4:], good[:6]):
        scan, why = feed.judge(bad, 0)
        assert scan is None and why == "bad_frame"


def test_a_negative_stamp_to_receipt_is_refused():
    feed = sf.ScanFeed()
    settled(feed)
    h = head(30)
    h["ros_stamp_ns"] = h["recv_mono_ns"] + OFF + 10 * MS  # stamped 10 ms after its receipt
    scan, why = feed.judge(sf.encode_frame(h, np.ones(8, dtype=np.float32), None), 0)
    assert scan is None and why == "negative_latency"


def test_a_clock_jump_is_refused_and_re_established_with_normal_jitter():
    """Codex 2nd-ed. P2-1: the two clocks read apart -- never require identical nanoseconds."""
    feed = sf.ScanFeed()
    settled(feed)
    jumped = OFF + 2_000 * MS
    rng = np.random.default_rng(1)
    verdicts = []
    for k in range(40, 40 + sf.ESTABLISH_COUNT + 2):
        off = jumped + int(rng.integers(-300_000, 300_000))  # +-0.3 ms of read jitter
        verdicts.append(feed.judge(frame(k, off=off), 0)[1])
    assert verdicts[0] == "clock_jump"
    assert verdicts[1:sf.ESTABLISH_COUNT] == ["establishing"] * (sf.ESTABLISH_COUNT - 1)
    assert verdicts[-1] is None


def test_a_jump_while_establishing_starts_the_count_again():
    feed = sf.ScanFeed()
    settled(feed)
    v = [feed.judge(frame(40, off=OFF + 2_000 * MS), 0)[1]]
    v += [feed.judge(frame(41 + k, off=OFF + 2_000 * MS), 0)[1] for k in range(4)]
    v.append(feed.judge(frame(50, off=OFF + 4_000 * MS), 0)[1])  # jumps again
    v += [feed.judge(frame(51 + k, off=OFF + 4_000 * MS), 0)[1] for k in range(sf.ESTABLISH_COUNT - 1)]
    assert v[0] == "clock_jump" and v[5] == "establishing" and v[-1] is None
    assert all(x == "establishing" for x in v[1:-1])


def test_small_offset_wander_is_kept_not_refused():
    feed = sf.ScanFeed()
    settled(feed)
    scan, why = feed.judge(frame(30, off=OFF + 20 * MS), 0)
    assert why is None and scan is not None


def test_the_poll_never_blocks_and_records_frames_before_the_verdict(tmp_path):
    a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    try:
        rec = tmp_path / "raw.scan.jsonl"
        with open(rec, "w", encoding="utf-8") as out:
            feed = sf.ScanFeed(sock=b, record=out)
            assert feed.poll() == []  # nothing waiting: back at once
            for k in range(sf.ESTABLISH_COUNT + 3):
                a.send(frame(k))
            a.send(frame(99, off=OFF + 3_000 * MS))  # a jump
            got = feed.poll()
            assert len(got) == 4  # the tenth agreeing scan is itself given, then three more
        replay = sf.replay(rec)
        assert [v for v in replay["verdicts"]] == [v for v in replay["recorded"]]
        assert replay["recorded"][-1] == "clock_jump" and replay["recorded"].count(None) == 4
    finally:
        a.close()
        b.close()
