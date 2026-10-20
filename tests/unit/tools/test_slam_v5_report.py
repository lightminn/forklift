"""SLAM localization plan V5: a live session's continuous-operation verdict (tools/slam_v5_report.py)."""

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import slam_v5_report as v5  # noqa: E402

from forklift_core.live.spool import RecordSpool  # noqa: E402


def test_keyframes_are_windowed_by_the_runner_s_wall_clock_and_only_processed_ones_count():
    records = ([{"status": "processed", "queued_mono": 100.0 + 10 * i, "slam_wall_s": 0.05} for i in range(400)]
               + [{"status": "skipped", "queued_mono": 150.0, "slam_wall_s": 9.0},
                  {"status": "processed", "slam_wall_s": 9.0}])  # a lockstep warm-up reply has no queue clock
    windows, span = v5.keyframe_windows(records)
    assert span == 3990.0 and [w["samples"] for w in windows] == [180, 180]  # the trailing 390 s is not a window
    assert all(abs(w["max_s"] - 0.05) < 1e-12 for w in windows)


def _session(tmp_path, *, hours=2.0, gap_window=None, control=None, ends=None, drop_result=None, wall=None):
    run = tmp_path / "run"
    (run / "missions").mkdir(parents=True)
    t = 0.0
    records = []
    while t <= hours * 3600:
        if gap_window is None or int(t // 1800) != gap_window:
            records.append({"status": "processed", "queued_mono": 1000.0 + t, "slam_wall_s": 0.05 if wall is None
                            else wall(t)})
        t += 10.0
    rows = control or [[i, 1.0, 2.0, 0.1, 1.01, 2.0, 0.11] for i in range(100)]
    spool = RecordSpool(run / "spool", interval_s=1.0)
    spool.register("slam.records", records)
    spool.register("slam.control", rows)
    spool.close()
    ends = ends or [("m%d" % i, "done", None, 0.02) for i in range(1, 8)]
    with open(run / "live_log.jsonl", "w") as log:
        for mission_id, outcome, reason, error in ends:
            log.write(json.dumps({"mission_end": {"mission_id": mission_id, "outcome": outcome, "reason": reason}}) + "\n")
            if mission_id != drop_result:
                (run / "missions" / mission_id).mkdir()
                (run / "missions" / mission_id / "result.json").write_text(json.dumps({"delivery_error_m": error}))
    (run / "result.json").write_text(json.dumps({"failure_reason": None}))
    return run


def test_a_clean_two_hour_session_passes(tmp_path):
    out = v5.report(_session(tmp_path))
    assert out["ok"], out["checks"]
    assert out["checks"]["windows"]["need"] == 4 and len(out["checks"]["windows"]["windows"]) == 4


@pytest.mark.parametrize("kw, check", [
    (dict(hours=1.5), "windows"),  # 90 minutes: three windows, not the 2 h V5 runs
    (dict(gap_window=1), "windows"),  # an empty window is a window
    (dict(wall=lambda t: 0.05 if t < 5400 else 0.09), "windows"),  # the last p99 past 1.5 x the first
    (dict(control=[[0, 0, 0, 0, 0, 0, 0], [1, float("nan"), 0, 0, 0, 0, 0], [2, 1.0, 0, 0.5, 0, 0, 0]]), "estimate"),
    (dict(ends=[("m%d" % i, "done", None, 0.02) for i in range(1, 8)]
               + [("m8", "failed", "Pallet missed the green destination center", 0.20)]), "deliveries"),
    (dict(drop_result="m3"), "missions"),  # a mission the log ended with no result file
    (dict(ends=[("m%d" % i, "done", None, 0.02) for i in range(1, 7)]), "missions"),  # six done
])
def test_what_v5_does_not_accept(tmp_path, kw, check):
    out = v5.report(_session(tmp_path, **kw))
    assert not out["ok"] and not out["checks"][check]["ok"]


def test_a_control_chunk_missing_from_the_spool_is_refused(tmp_path):
    run = _session(tmp_path)
    manifest = run / "spool" / "manifest.json"
    data = json.loads(manifest.read_text())
    data["slam.control"]["items"] += 1  # the manifest says one more row was written than is there
    manifest.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="slam.control"):
        v5.report(run)
