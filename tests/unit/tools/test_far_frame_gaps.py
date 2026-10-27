import json

import pytest

from tools import far_frame_gaps as g


def record(times, statuses, model_ms=None, stop_status="applied", leg=0, mission="m1", skipped=(), late=None):
    jobs = [{"seq": i + 1, "kind": "drive", "status": s, "model_ms": (model_ms or [50.0] * len(statuses))[i]}
            for i, s in enumerate(statuses)]
    jobs.append({"seq": len(jobs) + 1, "kind": "stop", "status": stop_status, "model_ms": 60.0})
    frames = [{"kind": "drive", "leg": leg, "pixel_t_s": t} for t in times]
    frames += [{"kind": "drive", "leg": leg, "pixel_t_s": t, "skipped": "no_pose"} for t in skipped]
    live = {"mission_id": mission, "jobs": jobs}
    if late:
        live["late_calls"] = late
    return {"recognition": {"frames": frames, "live": live}}


def test_skipped_frames_do_not_hide_an_expiry_gap():
    # Applied frames at 0.0 and 0.6 s with skipped ones between: the tracker was not updated for 0.6 s.
    m = g.mission_metrics(record([0.0, 0.6], ["applied", "applied"], skipped=[0.2, 0.4]))
    assert m["max_gap_s"] == pytest.approx(0.6) and m["gaps_over_expiry"] == 1 and m["skipped_frames"] == 2
    assert not g.summarize([m])["primary_ok"]


def test_missing_input_is_undetermined_not_a_pass(tmp_path):
    out = tmp_path / "gaps.json"
    assert g.main([str(tmp_path / "nowhere"), "--out", str(out)]) == 2
    (tmp_path / "empty").mkdir()
    assert g.main([str(tmp_path / "empty"), "--out", str(out)]) == 2
    d = tmp_path / "bad" / "recognition_missions" / "m1"
    d.mkdir(parents=True)
    (d / "result.json").write_text(json.dumps({"something": 1}))
    assert g.main([str(tmp_path / "bad"), "--out", str(out)]) == 2


def test_a_call_finished_after_its_mission_counts_in_the_round_trip():
    rec = record([0.0, 0.2], ["applied", "applied"], model_ms=[900.0, 100.0])
    rec["recognition"]["live"]["jobs"].append({"seq": 9, "kind": "drive", "status": "in_flight", "model_ms": None})
    other = record([5.0], ["applied"], mission="m2", late=[{"mission": "m1", "seq": 9, "model_ms": 5000.0}])
    missions = [g.mission_metrics(rec), g.mission_metrics(other)]
    rt = g.summarize(missions, g.late_calls([rec, other]))["round_trip_ms"]
    assert rt["max"] == 5000.0 and rt["in_flight_without_time"] == 0
    assert rt["first_call_max"] == 900.0
    rt_without = g.summarize(missions)["round_trip_ms"]
    assert rt_without["in_flight_without_time"] == 1


def test_one_missing_frame_is_a_watch_gap_not_an_expiry():
    m = g.mission_metrics(record([0.0, 0.2, 0.6, 0.8], ["applied", "applied", "dropped", "applied", "applied"]))
    assert m["max_gap_s"] == pytest.approx(0.4)
    assert m["gaps_over_watch"] == 0 and m["gaps_over_expiry"] == 0  # exactly 0.4 s is not over
    s = g.summarize([m])
    assert s["primary_ok"] and not s["alert"]


def test_the_recorded_float_error_of_one_missing_frame_is_not_a_watch_gap():
    # The live records give 0.40000002086162567 for one missing frame at 5 Hz.
    m = g.mission_metrics(record([1.0000000521540642, 1.4000000730156898], ["applied", "dropped", "applied"]))
    assert m["max_gap_s"] == pytest.approx(0.4) and m["gaps_over_watch"] == 0


def test_two_dropped_in_a_row_alert_and_break_the_primary_condition():
    m = g.mission_metrics(record([0.0, 0.2, 0.8], ["applied", "applied", "dropped", "dropped", "applied"]))
    assert m["consecutive_drop_runs"] == 1
    assert m["gaps_over_expiry"] == 1 and m["gaps_over_watch"] == 1
    s = g.summarize([m])
    assert not s["primary_ok"] and s["alert"]


def test_gaps_are_taken_within_a_leg():
    a = g.mission_metrics(record([0.0, 0.2], ["applied", "applied"], leg=0))
    b = record([5.0, 5.2], ["applied", "applied"], leg=1)
    both = a["max_gap_s"], g.mission_metrics(b)["max_gap_s"]
    assert both == (pytest.approx(0.2), pytest.approx(0.2))


def test_a_stop_not_applied_alerts():
    s = g.summarize([g.mission_metrics(record([0.0, 0.2], ["applied", "applied"], stop_status="dropped"))])
    assert s["stop_not_applied"] == 1 and s["alert"]


def test_round_trip_leaves_out_each_missions_first_call():
    m = g.mission_metrics(record([0.0, 0.2, 0.4], ["applied"] * 3, model_ms=[900.0, 100.0, 300.0]))
    rt = g.summarize([m])["round_trip_ms"]
    assert rt["first_call_max"] == 900.0
    assert rt["max"] == 300.0 and rt["over_200_share"] == pytest.approx(1 / 3)


def test_cli_reads_mission_records(tmp_path):
    d = tmp_path / "run" / "recognition_missions" / "m1"
    d.mkdir(parents=True)
    (d / "result.json").write_text(json.dumps(record([0.0, 0.2], ["applied", "applied"])))
    out = tmp_path / "gaps.json"
    assert g.main([str(tmp_path), "--out", str(out)]) == 0
    assert json.loads(out.read_text())[str(tmp_path)]["missions"] == 1
