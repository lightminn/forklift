"""PR 7 design 3.2 and 4 (7a-1): the scan feed's generations, and the conversion of a real scan to the obstacle
layer's scan -- only with a bounded acquisition time, first and last beam as ranges, invalid beams never free, the
mask and band from config/real/lidar.yaml."""

import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from forklift_core.live.real_layer import RealScan as LayerScan
from forklift_core.sensors import lidar_cfg as lc
from forklift_core.sensors import scan_feed as sf

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_scan_feed import MS, frame, settled  # noqa: E402

REPO = Path(__file__).resolve().parents[3]


def cfg(err_s=0.002, period=0.1, mask=(), measured=True):
    return lc.LidarConfig(mount=(0.1, 0.0, 0.3, 0.0), mount_measured=False, mask_rad=tuple(mask), limit_m=5.7,
                          scan_period_s=period, first_beam_err_s=err_s, timing_measured=measured)


def scan(n=8, first=None, step=None, ranges=None, synthetic=False, recv=10**12):
    step = (0.09 / (n - 1)) if step is None else step
    meta = {"time_increment": step}
    if first is not None:
        meta["first_beam_mono_ns"] = first
        if step > 0:  # the buffer's own last sample (forklift patch 0001), here (n - 1) steps on
            meta["last_beam_mono_ns"] = first + int(round((n - 1) * step * 1e9))
    if synthetic:
        meta["synthetic"] = True
    r = np.full(n, 2.0) if ranges is None else np.asarray(ranges, dtype=float)
    return sf.RealScan(1, 0, recv, 0, np.linspace(-math.pi, math.pi, n, endpoint=False), r, None, meta)


def test_the_shipped_config_keeps_every_scan_diagnostic():
    c = lc.load_lidar_config(REPO / "config" / "real" / "lidar.yaml")
    assert c.first_beam_err_s is None and not c.mount_measured and not c.timing_measured
    assert c.limit_m == pytest.approx(0.10 / math.sin(math.radians(1.0)))
    assert lc.to_layer_scan(scan(first=5 * 10**9), c, 0) == (None, "no_time_bound")


def test_no_acquisition_time_is_diagnostic():
    assert lc.to_layer_scan(scan(), cfg(), 0) == (None, "no_acquisition_time")


def test_first_and_last_beam_ranges():
    start = 1_000 * MS
    first = 2_000 * MS
    out, why = lc.to_layer_scan(scan(n=10, first=first, step=0.01), cfg(err_s=0.002), start)
    assert why is None and isinstance(out, LayerScan)
    assert out.first_s == pytest.approx((first - 2 * MS - start) / 1e9)        # Flo
    assert out.last_s == pytest.approx((first + 90 * MS + 2 * MS - start) / 1e9)  # Ehi
    assert out.deadline_last_s == pytest.approx((first + 90 * MS - 2 * MS - start) / 1e9)  # Elo
    assert out.origin == "real" and len(out.limit_m) == 10


def test_a_scan_longer_than_the_period_is_refused_for_the_layer():
    """Codex PR 7 design round 4 P2-1: the stop model counts scan_period + B from the first beam."""
    assert lc.to_layer_scan(scan(n=10, first=10**9, step=0.011), cfg(err_s=0.002), 0)[1] == "scan_too_long"
    assert lc.to_layer_scan(scan(n=10, first=10**9, step=0.011), cfg(err_s=0.0), 0)[1] is None


def test_invalid_ranges_are_never_free_and_the_mask_marks_self_hits():
    r = [2.0, np.nan, 3.0, np.nan, 1.0, 1.0, 1.0, 1.0]
    out, _ = lc.to_layer_scan(scan(first=10**9, ranges=r), cfg(mask=[(math.radians(-180), math.radians(-170))]), 0)
    assert list(out.valid) == [True, False, True, False, True, True, True, True]
    assert out.self_hit[0] and not out.self_hit[2]


def test_a_synthetic_frame_stays_marked():
    out, _ = lc.to_layer_scan(scan(first=10**9, synthetic=True), cfg(), 0)
    assert out.origin == "synthetic"


def test_no_beam_timing_is_refused():
    assert lc.to_layer_scan(scan(first=10**9, step=0.0), cfg(), 0)[1] == "no_beam_timing"


# --- the feed's generations (design 3.2) ---

def test_a_new_producer_opens_a_generation():
    feed = sf.ScanFeed()
    out = settled(feed, gen=11)
    assert out[-1][0].producer_gen == 11 and out[-1][0].feed_gen == 0
    s, why = feed.judge(frame(sf.ESTABLISH_COUNT, gen=22), 0)
    assert why is None and s.feed_gen == 1 and s.producer_gen == 22
    assert feed.id_gaps == 0  # the new producer's ids are not counted against the old ones


def test_a_scan_id_going_back_without_a_nonce_opens_a_generation():
    feed = sf.ScanFeed()
    settled(feed, start=100)
    s, why = feed.judge(frame(1), 0)
    assert why is None and s.feed_gen == 1


def test_a_clock_jump_then_re_establishment_opens_a_generation():
    feed = sf.ScanFeed()
    settled(feed)
    off2 = 1_700_000_001_000 * MS  # a step of one second in (system - monotonic)
    assert feed.judge(frame(sf.ESTABLISH_COUNT, off=off2), 0)[1] == "clock_jump"
    out = settled(feed, start=sf.ESTABLISH_COUNT + 1, off=off2)
    accepted = [s for s, why in out if why is None]
    assert accepted and all(s.feed_gen == 1 for s in accepted)


def test_the_first_and_last_sample_times_are_carried_only_as_a_pair():
    feed = sf.ScanFeed()
    out = settled(feed, first_beam_mono_ns=123, last_beam_mono_ns=456)
    assert out[-1][0].meta["first_beam_mono_ns"] == 123 and out[-1][0].meta["last_beam_mono_ns"] == 456
    feed = sf.ScanFeed()
    out = settled(feed, first_beam_mono_ns=123)  # no last: neither is carried
    assert "first_beam_mono_ns" not in out[-1][0].meta


def test_the_adapter_frame_carries_gen_and_first_beam():
    sys.path.insert(0, str(REPO / "ros2" / "src" / "forklift_ros" / "forklift_ros"))
    import real_scan_adapter as ra
    from types import SimpleNamespace

    msg = SimpleNamespace(ranges=[1.0, 2.0], intensities=[], angle_min=-1.0, angle_increment=0.5,
                          time_increment=0.01, scan_time=0.1, range_min=0.1, range_max=10.0,
                          header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=2), frame_id="laser"))
    h, _r, _i = sf.decode_frame(ra.make_frame(msg, scan_id=3, recv_mono_ns=5, sys_minus_mono_ns=7, gen=99,
                                              first_beam_mono_ns=41))
    assert h["gen"] == 99 and h["first_beam_mono_ns"] == 41


# --- the layer's deadline from the last beam's earliest time ---

def test_the_layer_deadline_counts_from_deadline_last_s():
    from forklift_core.live import real_layer as rl

    base = LayerScan("lidar", 1.0, 1.10, np.zeros(4), np.ones(4), np.ones(4, bool), np.zeros(4, bool),
                     np.full(4, 5.0))
    for s, expect in ((base, 1.10), (replace(base, deadline_last_s=1.06), 1.06)):
        layer = rl.RealLayer.__new__(rl.RealLayer)  # only add_scan's deadline bookkeeping is exercised
        layer.counts = {"scans": 0}
        layer._expected_s = None  # noqa: SLF001
        layer._missed_expected = None  # noqa: SLF001
        layer._scan_id = 0  # noqa: SLF001
        layer._deadlines = {}  # noqa: SLF001
        layer.budget_s, layer.scan_period_s = 0.1, 0.1
        layer.sensors = {}
        layer._pending = []  # noqa: SLF001
        try:
            layer.add_scan(s, 1.2)
        except Exception:  # noqa: BLE001 -- the rest of add_scan needs a full layer; the deadline is set first
            pass
        assert layer._deadlines[1][0] == pytest.approx(expect + 0.1)  # noqa: SLF001


def test_an_unmeasured_bound_or_a_time_after_receipt_keeps_the_scan_diagnostic():
    """Codex 7a-1 P2-3."""
    assert lc.to_layer_scan(scan(first=10**9), cfg(measured=False), 0)[1] == "no_time_bound"
    # received at 2 s, its first beam said to be at 9 s
    assert lc.to_layer_scan(scan(first=9 * 10**9, recv=2 * 10**9), cfg(), 0)[1] == "after_receipt"


def test_generation_rules():
    """Codex 7a-1 P2-4: a broken frame changes nothing, a duplicate and a late frame of a replaced producer are
    refused, an adapter without a nonce restarting opens a generation."""
    from test_scan_feed import frame as fr

    feed = sf.ScanFeed()
    settled(feed, gen=11)
    n = sf.ESTABLISH_COUNT
    assert feed.judge(fr(n - 1, gen=11), 0)[1] == "stale_id"            # a duplicate
    assert feed.judge(fr(n, gen=11, angle_increment=-1.0), 0)[1] == "bad_meta"
    assert feed.generation == 0                                          # neither opened a generation
    assert feed.judge(fr(n, gen=22), 0)[1] is None and feed.generation == 1
    assert feed.judge(fr(n + 1, gen=11), 0)[1] == "old_producer"        # A -> B -> late A
    feed.new_generation("receive_gap")
    assert feed.generation == 2 and feed.generation_why == ["producer", "receive_gap"]


def test_a_refused_frame_never_retires_the_producer_still_sending():
    """Codex 7a-1 round 2 P2-3: a malformed nonce is bad_meta, and a new producer's frame refused on its clock does
    not end the current producer."""
    from test_scan_feed import frame as fr

    feed = sf.ScanFeed()
    settled(feed, gen=11)
    n = sf.ESTABLISH_COUNT
    assert feed.judge(fr(n, gen="x"), 0)[1] == "bad_meta"
    assert feed.judge(fr(n, gen=11), 0)[1] is None
    # a new producer whose first frame is refused (stamped long after its receipt)
    from test_scan_feed import OFF

    late = fr(1, gen=22, stamp=10_000 * MS + 100 * MS + OFF + 10**9)  # stamped a second after its receipt
    assert feed.judge(late, 0)[1] == "negative_latency"
    assert feed.judge(fr(n + 1, gen=11), 0)[1] is None and feed.generation == 0


def test_a_refused_candidate_producer_leaves_the_clock_as_it_was():
    """Codex 7a-1 round 3 P2-2: B's frame refused on a clock jump does not send A back to establishing."""
    from test_scan_feed import frame as fr

    feed = sf.ScanFeed()
    settled(feed, gen=11)
    n = sf.ESTABLISH_COUNT
    assert feed.judge(fr(n, gen=22, off=1_700_000_001_000 * MS), 0)[1] == "clock_jump"
    assert feed.judge(fr(n + 1, gen=11), 0)[1] is None and feed.generation == 0


def test_a_new_producer_on_a_jumped_clock_establishes_its_own_and_switches():
    """Codex 7a-1 round 4 P2-1: A stops; B arrives with a clock pair a second off -- B's own window establishes it
    and B takes over, A's clock untouched meanwhile."""
    from test_scan_feed import frame as fr

    feed = sf.ScanFeed()
    settled(feed, gen=11)
    off2 = 1_700_000_001_000 * MS
    verdicts = [feed.judge(fr(k, gen=22, off=off2), 0)[1] for k in range(1, 2 + sf.ESTABLISH_COUNT)]
    assert verdicts[-1] is None and feed.generation == 1  # B established in its own window and took over
    assert feed.judge(fr(50, gen=11), 0)[1] == "old_producer"


def test_a_candidate_on_the_same_clock_switches_at_once():
    from test_scan_feed import frame as fr

    feed = sf.ScanFeed()
    settled(feed, gen=11)
    s, why = feed.judge(fr(1, gen=22), 0)
    assert why is None and s.feed_gen == 1


def test_a_nonce_less_restart_on_a_jumped_clock_is_a_candidate_too():
    """Codex 7a-1 round 5 P2-1: a low id with another clock pair is refused without sending the current stream back
    to establishing; a real restart establishes in its own window and opens one generation."""
    from test_scan_feed import frame as fr

    feed = sf.ScanFeed()
    settled(feed, start=100)
    n = 100 + sf.ESTABLISH_COUNT
    off2 = 1_700_000_001_000 * MS
    assert feed.judge(fr(1, off=off2), 0)[1] == "clock_jump"
    assert feed.judge(fr(n, ), 0)[1] is None and feed.generation == 0  # the current stream untouched
    verdicts = [feed.judge(fr(k, off=off2), 0)[1] for k in range(2, 3 + sf.ESTABLISH_COUNT)]
    assert verdicts[-1] is None and feed.generation == 1


def test_a_candidates_duplicates_never_establish_it():
    """Codex 7a-1 round 5 P2-2."""
    from test_scan_feed import frame as fr

    feed = sf.ScanFeed()
    settled(feed, gen=11)
    off2 = 1_700_000_001_000 * MS
    first = feed.judge(fr(1, gen=22, off=off2), 0)[1]
    dups = [feed.judge(fr(1, gen=22, off=off2), 0)[1] for _ in range(12)]
    assert first == "clock_jump" and set(dups) == {"stale_id"}
    assert feed.judge(fr(sf.ESTABLISH_COUNT, gen=11), 0)[1] is None and feed.generation == 0


def test_a_stray_candidate_expires_and_never_blocks_a_real_restart():
    """Codex 7a-1 round 6 P2: a delayed low id refused on its clock, then the current stream goes on and a gap
    opens -- a real restart later is not held by the stray candidate's id."""
    from test_scan_feed import frame as fr

    feed = sf.ScanFeed()
    settled(feed, start=30_000)
    n = 30_000 + sf.ESTABLISH_COUNT
    off2 = 1_700_000_001_000 * MS
    assert feed.judge(fr(30_005, off=off2), 0)[1] == "clock_jump"  # an id_back candidate, refused on its clock
    assert feed.judge(fr(n, ), 0)[1] is None                       # the current stream goes on: candidate gone
    feed.new_generation("receive_gap")
    verdicts = [feed.judge(fr(k, off=off2), 0)[1] for k in range(1, 2 + sf.ESTABLISH_COUNT)]
    assert verdicts[-1] is None


@pytest.mark.parametrize("expire", ["current_frame", "receive_gap"])
def test_each_candidate_expiry_path_alone(expire):
    """Codex 7a-1 round 7 P3: each way a stray candidate ends, on its own."""
    from test_scan_feed import frame as fr

    feed = sf.ScanFeed()
    settled(feed, start=30_000)
    n = 30_000 + sf.ESTABLISH_COUNT
    off2 = 1_700_000_001_000 * MS
    assert feed.judge(fr(30_005, off=off2), 0)[1] == "clock_jump"
    assert feed._cand is not None  # noqa: SLF001
    if expire == "current_frame":
        assert feed.judge(fr(n), 0)[1] is None
    else:
        feed.new_generation("receive_gap")
    assert feed._cand is None  # noqa: SLF001
