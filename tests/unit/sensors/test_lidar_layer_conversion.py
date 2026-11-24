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


def cfg(err_s=0.002, period=0.1, mask=()):
    return lc.LidarConfig(mount=(0.1, 0.0, 0.3, 0.0), mount_measured=False, mask_rad=tuple(mask), limit_m=5.7,
                          scan_period_s=period, first_beam_err_s=err_s, timing_measured=False)


def scan(n=8, first=None, step=None, ranges=None, synthetic=False):
    meta = {"time_increment": (0.09 / (n - 1)) if step is None else step}
    if first is not None:
        meta["first_beam_mono_ns"] = first
    if synthetic:
        meta["synthetic"] = True
    r = np.full(n, 2.0) if ranges is None else np.asarray(ranges, dtype=float)
    return sf.RealScan(1, 0, 0, 0, np.linspace(-math.pi, math.pi, n, endpoint=False), r, None, meta)


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


def test_the_first_beam_time_is_carried():
    feed = sf.ScanFeed()
    out = settled(feed, first_beam_mono_ns=123)
    assert out[-1][0].meta["first_beam_mono_ns"] == 123


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
