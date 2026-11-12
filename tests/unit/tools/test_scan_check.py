"""tools/scan_check.py (real-scan design 1.4): the laptop A2M12 check -- scan-only statistics, the angle convention
probe, the stamp-to-receipt distribution named as such, adapter drops from scan_id gaps, and the raw record."""

import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
import scan_check as sc  # noqa: E402

from forklift_core.sensors import scan_feed as sf  # noqa: E402

MS = 1_000_000
OFF = 1_700_000_000_000 * MS


def frame(k, *, n=360, latency_ms=20, period_ms=100, ranges=None):
    recv = 10_000 * MS + k * period_ms * MS
    r = np.full(n, 3.0, dtype=np.float32) if ranges is None else np.asarray(ranges, dtype=np.float32)
    h = {"scan_id": k, "ros_stamp_ns": recv - latency_ms * MS + OFF, "recv_mono_ns": recv, "sys_minus_mono_ns": OFF,
         "angle_min": -math.pi, "angle_increment": 2 * math.pi / n, "time_increment": 0.1 / n, "scan_time": 0.1,
         "range_min": 0.05, "range_max": 12.0, "frame_id": "laser", "n": n}
    return sf.encode_frame(h, r, None)


def accepted(frames):
    feed = sf.ScanFeed()
    scans = [s for s in (feed.judge(f, 0)[0] for f in frames) if s is not None]
    return feed, scans


def with_box(n, angle_rad, dist=0.3, width_rad=0.15):
    a = -math.pi + 2 * math.pi * np.arange(n) / n
    r = np.full(n, 3.0)
    d = np.abs((a - angle_rad + math.pi) % (2 * math.pi) - math.pi)
    r[d < width_rad] = dist
    return r


def test_the_summary_counts_rate_beams_unknown_and_stamp_to_receipt():
    ranges = np.full(360, 3.0)
    ranges[:36] = np.inf  # 10 % no return
    feed, scans = accepted([frame(k, ranges=ranges) for k in range(40)])
    s = sc.summarize(scans, feed)
    assert s["scans"] == 40 - (sf.ESTABLISH_COUNT - 1)
    assert s["beams"] == {"min": 360, "max": 360} and s["unknown_fraction"] == pytest.approx(0.1)
    assert s["period_s"]["p50"] == pytest.approx(0.1) and s["angle_increment_rad"] == pytest.approx(2 * math.pi / 360)
    assert s["stamp_to_receipt_ms"]["p50"] == pytest.approx(20.0) and s["stamp_to_receipt_ms"]["max"] == pytest.approx(20.0)
    assert "not the acquisition age" in s["stamp_to_receipt_ms"]["note"]
    assert s["adapter_dropped"] == 0 and s["refused"] == {"establishing": sf.ESTABLISH_COUNT - 1}


def test_scan_id_gaps_are_the_adapters_drops():
    _feed, scans = accepted([frame(k) for k in range(30) if k not in (20, 21, 25)])
    assert sc.summarize(scans, _feed)["adapter_dropped"] == 3


def test_a_receipt_gap_after_a_stall_shows_in_the_receipt_intervals():
    frames = [frame(k) for k in range(30)]
    late = [sf.encode_frame({**sf.decode_frame(f)[0], "recv_mono_ns": sf.decode_frame(f)[0]["recv_mono_ns"]
                             + 500 * MS, "ros_stamp_ns": sf.decode_frame(f)[0]["ros_stamp_ns"] + 500 * MS},
                            sf.decode_frame(f)[1], None) for f in [frame(k) for k in range(30, 33)]]
    feed, scans = accepted(frames + late)
    s = sc.summarize(scans, feed)
    assert s["receipt_gap_max_s"] == pytest.approx(0.6)


@pytest.mark.parametrize("side, angle, ok", [
    ("right", -math.pi / 2, True), ("left", math.pi / 2, True), ("front", 0.0, True), ("back", math.pi, True),
    ("right", math.pi / 2, False),  # a box on the left read as the right: the convention is mirrored
])
def test_the_probe_judges_the_angle_convention(side, angle, ok):
    feed, scans = accepted([frame(k, ranges=with_box(360, angle)) for k in range(15)])
    p = sc.probe(scans, side)
    assert p["ok"] is ok and p["nearest_m"] == pytest.approx(0.3)
    assert abs(((p["nearest_angle_rad"] - angle) + math.pi) % (2 * math.pi) - math.pi) < 0.2


def test_the_probe_wants_something_close():
    feed, scans = accepted([frame(k) for k in range(15)])
    p = sc.probe(scans, "right")
    assert p["ok"] is False and "no return" in p["why"]


def test_main_binds_reads_and_records(tmp_path):
    sock = tmp_path / "scan.sock"
    rec = tmp_path / "out.scan.jsonl"
    out = tmp_path / "out.json"

    def send():
        import importlib.util

        spec = importlib.util.spec_from_file_location(
            "adapter", REPO / "ros2/src/forklift_ros/forklift_ros/real_scan_adapter.py")
        ad = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ad)
        for _ in range(100):
            if sock.exists():
                break
            time.sleep(0.01)
        sender = ad.DatagramSender(str(sock))
        for k in range(15):
            sender.send(frame(k))
        sender.close()

    t = threading.Thread(target=send)
    t.start()
    rc = sc.main(["--socket", str(sock), "--seconds", "0.5", "--record", str(rec), "--out", str(out)])
    t.join()
    assert rc == 0
    s = json.loads(out.read_text())
    assert s["scans"] == 15 - (sf.ESTABLISH_COUNT - 1)
    replay = sf.replay(rec)
    assert replay["verdicts"] == replay["recorded"] and len(replay["recorded"]) == 15


def test_the_probe_needs_most_scans_on_the_side_not_a_mean_between_two():
    """Codex 7a P2-4: nearest returns alternating +60 and -60 degrees average to ahead; that is not 'front'."""
    frames = [frame(k, ranges=with_box(360, math.radians(60 if k % 2 else -60))) for k in range(30)]
    _feed, scans = accepted(frames)
    p = sc.probe(scans, "front")
    assert p["ok"] is False and p["support"] < 0.8


def test_adapter_drops_come_from_received_ids():
    frames = [frame(k) for k in range(30) if k not in (20, 21)]
    feed, scans = accepted(frames)
    assert sc.summarize(scans, feed)["adapter_dropped"] == 2
