"""H's switch (online-SLAM plan 1-H): the start pose handed to the localization and the switch summary."""

from __future__ import annotations

import json
import math

import pytest

from tools import slam_h_switch as hs


def _line(k, unit, recv, wall, mfo=(0.0, 0.0, 0.0), odom=(0.0, 0.0, 0.0)):
    return json.dumps({"scan_id": k, "unit": unit, "slam_wall_s": wall, "recv_mono": recv, "sent_mono": recv - wall,
                       "map_from_odom": list(mfo), "odom_base": list(odom), "stamp_s": float(k)})


def _write(path, lines):
    path.write_text("\n".join(lines) + "\n")
    return path


def _log(path, first, last, extra=""):
    def row(k, unit, recv, status="processed", mfo=(0.0, 0.0, 0.0), odom=(0.0, 0.0, 0.0)):
        return {"k": k, "unit": unit, "status": status, "recv_mono": recv, "map_from_odom": list(mfo),
                "odom_base": list(odom)}
    rec = {"first": row(*first[:3], *first[3:]), "last": row(*last[:3], *last[3:]), "by_status": {"processed": 1}}
    path.write_text("config {}\n" + extra + "replies " + json.dumps(rec) + "\nsent 1 scans in 1 s\n")
    return path


def test_the_start_pose_is_the_sessions_own_start_and_the_latest_estimate_is_recorded(tmp_path):
    drive = _write(tmp_path / "map.jsonl", [_line(0, 0, 1.0, 0.1, odom=(-2.6, 0.0, 0.0)),
                                            _line(1, 0, 2.0, 0.1, mfo=(1.0, 0.0, math.pi / 2), odom=(2.0, 0.0, 0.0))])
    nxt = 'next {"k": 5, "unit": 1, "stamp_s": 5.0, "odom": [3.0, 0.0, 0.1]}\n'
    # the last reply (a skipped one after the last keyframe) carries the correction the estimate uses
    log = _log(tmp_path / "drive.log", (0, 0, 1.0), (4, 0, 3.0, "skipped", (1.0, 0.0, math.pi / 2)), nxt)
    out = hs.start_pose(drive, log)
    assert out["used"] == pytest.approx([-2.6, 0.0, 0.0]) and out["pose"] == "-2.600000,0.000000,0.000000"
    assert out["estimate_at_next_scan"] == pytest.approx([1.0, 3.0, math.pi / 2 + 0.1])
    assert out["estimate_from_start_m"] == pytest.approx(math.hypot(3.6, 3.0))
    assert out["estimate_from_start_rad"] == pytest.approx(math.pi / 2 + 0.1)
    assert (out["last_answered_scan"], out["last_answered_status"], out["last_processed_scan"]) == (4, "skipped", 1)
    for bad in ("", "next null\n", 'next {"k": 4, "unit": 1, "stamp_s": 1.0, "odom": [0, 0, 0]}\n'):
        with pytest.raises(SystemExit):
            hs.start_pose(drive, _log(tmp_path / "bad.log", (0, 0, 1.0), (4, 0, 3.0), bad))


def test_the_switch_wait_runs_from_the_last_mapping_reply_of_any_status_to_the_first_localization_reply(tmp_path):
    m = _write(tmp_path / "map.jsonl", [_line(k, 0, 10.0 + k, 0.2) for k in range(5)])
    m_log = _log(tmp_path / "map.log", (0, 0, 10.0), (7, 0, 16.0, "skipped"))  # later than the last processed (14)
    loc = _write(tmp_path / "loc.jsonl", [_line(8, 1, 50.0, 0.3), _line(9, 1, 51.0, 1.2), _line(10, 2, 52.0, 0.4)])
    l_log = _log(tmp_path / "loc.log", (8, 1, 49.5, "skipped"), (10, 2, 52.0))  # before the first processed (50)
    out = hs.summarize(m, m_log, loc, l_log, steps={"map_container_end": 30.0, "bundle_end": 31.0})
    assert out["switch_wait_s"] == pytest.approx(49.5 - 16.0)
    assert out["switch_pass"] is True
    assert out["after_keyframe_max_s"] == pytest.approx(1.2)
    assert out["after_keyframes_at_or_over_1s"] == 1 and out["after_pass"] is False
    assert out["steps_from_last_mapping_reply_s"] == pytest.approx({"map_container_end": 14.0, "bundle_end": 15.0})
    assert out["mapping_units"] == [0] and out["localization_units"] == [1, 2]
    assert (out["last_mapping_reply"], out["first_localization_reply"]) == (7, 8)


def test_a_wait_over_60_s_fails_and_phases_that_overlap_or_mix_units_are_refused(tmp_path):
    m = _write(tmp_path / "map.jsonl", [_line(0, 0, 10.0, 0.2)])
    m_log = _log(tmp_path / "map.log", (0, 0, 10.0), (0, 0, 10.0))
    late = _write(tmp_path / "late.jsonl", [_line(1, 1, 70.5, 0.2)])
    late_log = _log(tmp_path / "late.log", (1, 1, 70.5), (1, 1, 70.5))
    assert hs.summarize(m, m_log, late, late_log)["switch_pass"] is False
    for bad, bad_log in (([_line(1, 1, 9.0, 0.2)], (1, 1, 9.0)),  # a localization reply before the mapping ended
                         ([_line(1, 0, 20.0, 0.2)], (1, 0, 20.0))):  # unit 0 again after the switch
        with pytest.raises(SystemExit):
            hs.summarize(m, m_log, _write(tmp_path / "bad.jsonl", bad), _log(tmp_path / "bad.log", bad_log, bad_log))
    with pytest.raises(SystemExit):  # the mapping phase sent a later unit
        hs.summarize(_write(tmp_path / "m2.jsonl", [_line(0, 0, 1.0, 0.1), _line(1, 1, 2.0, 0.1)]), m_log, late,
                     late_log)
    with pytest.raises(SystemExit):  # an empty phase
        hs.summarize(_write(tmp_path / "empty.jsonl", []), m_log, late, late_log)
    with pytest.raises(SystemExit):  # a log without its replies line
        hs.summarize(m, _write(tmp_path / "noreplies.log", ["config {}"]), late, late_log)
