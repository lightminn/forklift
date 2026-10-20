"""SLAM localization plan V5: the keyframe windows of a live session's report."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import slam_v5_report as v5  # noqa: E402


def test_keyframes_are_windowed_by_the_runner_s_wall_clock_and_only_processed_ones_count():
    records = ([{"status": "processed", "queued_mono": 100.0 + 10 * i, "slam_wall_s": 0.05} for i in range(200)]
               + [{"status": "skipped", "queued_mono": 150.0, "slam_wall_s": 9.0},
                  {"status": "processed", "slam_wall_s": 9.0}])  # a lockstep warm-up reply has no queue clock
    windows = v5.keyframe_windows(records)
    assert [w["samples"] for w in windows] == [180, 20]
    assert all(abs(w["max_s"] - 0.05) < 1e-12 for w in windows)
