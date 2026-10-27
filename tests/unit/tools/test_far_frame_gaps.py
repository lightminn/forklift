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


def summ(missions, late=None):
    return g.summarize(missions, [g.resolve_calls(m, late or {}) for m in missions])


def write_run(root, run, records, run_result=None):
    for i, r in enumerate(records):
        d = root / run / "run" / "recognition_missions" / f"m{i + 1}"
        d.mkdir(parents=True)
        (d / "result.json").write_text(json.dumps(r))
    if run_result is not None:
        (root / run / "run" / "result.json").write_text(json.dumps(run_result))


@pytest.mark.parametrize("bad", [
    {"recognition": {"frames": None, "live": {"jobs": None}}},
    {"recognition": {"frames": [{"kind": "drive", "pixel_t_s": float("nan")}], "live": {"jobs": []}}},
    {"recognition": {"frames": [], "live": {"jobs": [{"seq": 1, "kind": "drive", "status": "applied",
                                                      "model_ms": float("inf")}]}}},
])
def test_damaged_values_are_undetermined(bad):
    with pytest.raises(g.RecordError):
        g.mission_metrics(bad)


@pytest.mark.parametrize("damage", [{"kind": None}, {"kind": "drvie"}, {"leg": None}, {"leg": "0"}])
def test_a_damaged_frame_cannot_hide_a_gap(damage):
    # [0.0, 0.2, 0.8] fails at 0.6 s; damaging the last frame's kind or leg must not turn that into a pass.
    r = record([0.0, 0.2, 0.8], ["applied"] * 3)
    r["recognition"]["frames"][2].update(damage)
    if damage.get("kind", "drive") is None:
        del r["recognition"]["frames"][2]["kind"]
    with pytest.raises(g.RecordError):
        g.mission_metrics(r)


def test_late_calls_of_different_runs_do_not_collide(tmp_path):
    # Mission ids restart in every run: run A's slow late call must not be replaced by run B's fast one.
    def rec(late_ms):
        r = record([0.0, 0.2], ["applied", "applied"], model_ms=[10.0, 20.0])
        r["recognition"]["live"]["jobs"].append({"seq": 9, "kind": "drive", "status": "in_flight", "model_ms": None})
        return r, {"far_late_calls": [{"mission": "m1", "seq": 9, "model_ms": late_ms}]}
    a, a_run = rec(5000.0)
    b, b_run = rec(100.0)
    write_run(tmp_path, "A", [a], a_run)
    write_run(tmp_path, "B", [b], b_run)
    s = g.evaluate_dir(tmp_path)
    assert s["round_trip_ms"]["max"] == 5000.0 and s["round_trip_ms"]["in_flight_without_time"] == 0


def test_a_skipped_first_job_is_not_the_first_model_call():
    r = record([0.0, 0.2], ["applied", "applied", "applied"], model_ms=[None, 900.0, 100.0])
    rt = summ([g.mission_metrics(r)])["round_trip_ms"]
    assert rt["first_call_max"] == 900.0
    assert rt["max"] == 100.0  # 900 is the first model call, kept apart


def test_a_run_directory_given_directly_reads_its_own_result(tmp_path):
    r = record([0.0, 0.2], ["applied", "applied"], model_ms=[10.0, 20.0])
    r["recognition"]["live"]["jobs"].append({"seq": 9, "kind": "drive", "status": "in_flight", "model_ms": None})
    write_run(tmp_path, "A", [r], {"far_late_calls": [{"mission": "m1", "seq": 9, "model_ms": 5000.0}]})
    s = g.evaluate_dir(tmp_path / "A" / "run")
    assert s["round_trip_ms"]["max"] == 5000.0


def test_skipped_frames_do_not_hide_an_expiry_gap():
    # Applied frames at 0.0 and 0.6 s with skipped ones between: the tracker was not updated for 0.6 s.
    m = g.mission_metrics(record([0.0, 0.6], ["applied", "applied"], skipped=[0.2, 0.4]))
    assert m["max_gap_s"] == pytest.approx(0.6) and m["gaps_over_expiry"] == 1 and m["skipped_frames"] == 2
    assert not summ([m])["primary_ok"]


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
    rt = summ(missions, g.late_calls([rec, other]))["round_trip_ms"]
    assert rt["max"] == 5000.0 and rt["in_flight_without_time"] == 0
    assert rt["first_call_max"] == 900.0
    rt_without = summ(missions)["round_trip_ms"]
    assert rt_without["in_flight_without_time"] == 1


def test_one_missing_frame_is_a_watch_gap_not_an_expiry():
    m = g.mission_metrics(record([0.0, 0.2, 0.6, 0.8], ["applied", "applied", "dropped", "applied", "applied"]))
    assert m["max_gap_s"] == pytest.approx(0.4)
    assert m["gaps_over_watch"] == 0 and m["gaps_over_expiry"] == 0  # exactly 0.4 s is not over
    s = summ([m])
    assert s["primary_ok"] and not s["alert"]


def test_the_recorded_float_error_of_one_missing_frame_is_not_a_watch_gap():
    # The live records give 0.40000002086162567 for one missing frame at 5 Hz.
    m = g.mission_metrics(record([1.0000000521540642, 1.4000000730156898], ["applied", "dropped", "applied"]))
    assert m["max_gap_s"] == pytest.approx(0.4) and m["gaps_over_watch"] == 0


def test_two_dropped_in_a_row_alert_and_break_the_primary_condition():
    m = g.mission_metrics(record([0.0, 0.2, 0.8], ["applied", "applied", "dropped", "dropped", "applied"]))
    assert m["consecutive_drop_runs"] == 1
    assert m["gaps_over_expiry"] == 1 and m["gaps_over_watch"] == 1
    s = summ([m])
    assert not s["primary_ok"] and s["alert"]


def test_gaps_are_taken_within_a_leg():
    a = g.mission_metrics(record([0.0, 0.2], ["applied", "applied"], leg=0))
    b = record([5.0, 5.2], ["applied", "applied"], leg=1)
    both = a["max_gap_s"], g.mission_metrics(b)["max_gap_s"]
    assert both == (pytest.approx(0.2), pytest.approx(0.2))


def test_a_stop_not_applied_alerts():
    s = summ([g.mission_metrics(record([0.0, 0.2], ["applied", "applied"], stop_status="dropped"))])
    assert s["stop_not_applied"] == 1 and s["alert"]


def test_round_trip_leaves_out_each_missions_first_call():
    m = g.mission_metrics(record([0.0, 0.2, 0.4], ["applied"] * 3, model_ms=[900.0, 100.0, 300.0]))
    rt = summ([m])["round_trip_ms"]
    assert rt["first_call_max"] == 900.0
    assert rt["max"] == 300.0 and rt["over_200_share"] == pytest.approx(1 / 3)


def test_cli_reads_mission_records(tmp_path):
    d = tmp_path / "run" / "recognition_missions" / "m1"
    d.mkdir(parents=True)
    (d / "result.json").write_text(json.dumps(record([0.0, 0.2], ["applied", "applied"])))
    out = tmp_path / "gaps.json"
    assert g.main([str(tmp_path), "--out", str(out)]) == 0
    assert json.loads(out.read_text())[str(tmp_path)]["missions"] == 1
