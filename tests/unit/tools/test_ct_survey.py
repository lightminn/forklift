"""tools/ct_survey.py on synthetic live session folders (docs/plans/2026-11-06-ct-status-survey.md)."""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import ct_survey as C  # noqa: E402


def _write(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


def session(tmp_path, name, submits, missions=(), run=None, queue=None, epoch=1):
    """A session folder: probe submissions [(item, t, mission_id or reason)], mission folders
    [(mid, outcome, reason, started, ended, phase)], the epoch's run record and the queue report."""
    d = tmp_path / name
    runs = []
    for item, t, x in submits:
        submit = {"t": t, "mission_id": x} if t == "ack" else ({"t": t, "reason": x} if t == "reject" else {})
        runs.append({"item_id": item, "pallet_id": "P", "destination_id": "D", "submit": submit})
    _write(d / "probe.json", {"missions_run": runs, "queue": queue or {"stop_reason": None, "stops": 0}})
    base = d / "live" / f"epoch_{epoch}" / "run"
    for mid, outcome, reason, started, ended, phase in missions:
        _write(base / "missions" / mid / "result.json",
               {"result": {"mission_id": mid, "outcome": outcome, "reason": reason, "started_s": started, "ended_s": ended}})
        _write(base / "missions" / mid / "records.json", {"state": {"phase": phase}})
    if run is not None:
        _write(base / "result.json", run)
    base.mkdir(parents=True, exist_ok=True)
    return d


def test_start_states_separate_confirmed_unknown_rejected_and_not_submitted():
    probe = {"missions_run": [{"item_id": "a", "submit": {"t": "ack", "mission_id": "m1"}},
                              {"item_id": "b", "submit": {"t": "reject", "reason": "busy"}},
                              {"item_id": "c", "submit": {"t": "reject", "reason": "not_stopped"}}]}
    st = C.start_states(probe, 4)
    assert [s["start"] for s in st] == ["confirmed", "unknown", "rejected", "not_submitted"]
    # a resubmitted item keeps its last answer
    probe = {"missions_run": [{"item_id": "a", "submit": {"t": "reject", "reason": "not_stopped"}},
                              {"item_id": "a", "submit": {"t": "ack", "mission_id": "m1"}}]}
    assert [s["start"] for s in C.start_states(probe, 1)] == ["confirmed"]


def test_a_finished_session_gives_stage_from_the_reason_and_exposure_from_transitions(tmp_path):
    run = {"success": True, "transitions": [
        {"from": "observe", "to": "approach", "time_s": 5}, {"from": "approach", "to": "insert", "time_s": 8},
        {"from": "insert", "to": "lift", "time_s": 10}, {"from": "lift", "to": "extract", "time_s": 11},
        {"from": "extract", "to": "transport", "time_s": 12}]}
    d = session(tmp_path, "D_line_m_s0", [("a", "ack", "m1"), ("b", "ack", "m2")],
                [("m1", "done", None, 0.5, 4.0, "complete"),
                 ("m2", "failed", "Tracking failed in transport", 4.5, 20.0, "transport")], run=run)
    s = C.session_rows(d, 3)
    r1, r2, r3 = s["rows"]
    assert r1["outcome"] == "done" and r1["class"] is None and r1["exposure_from"] == "order"
    assert r2["stage"] == "transport" and r2["class"] == "tracking"
    assert r2["exposure"] == ["preparing", "observe", "approach", "insert", "lift", "extract", "transport"]
    assert r2["exposure_from"] == "transitions"
    assert r3["start"] == "not_submitted" and r3["outcome"] is None


def test_a_cut_mission_takes_the_session_end_and_never_overwrites_a_settled_result(tmp_path):
    # Codex review: the cut mission (no folder) keeps the run record's reason and phase even with no docking retries
    run = {"success": False, "failure_reason": "slam_link_failed: timeout",
           "transitions": [{"from": "extract", "to": "transport", "time_s": 30}]}
    d = session(tmp_path, "D_random_m_s1", [("a", "ack", "m1"), ("b", "ack", "m2")],
                [("m1", "done", None, 0.5, 20.0, "complete")], run=run)
    s = C.session_rows(d, 3)
    assert s["rows"][0]["outcome"] == "done"  # settled, not overwritten
    cut = s["rows"][1]
    assert cut["outcome"] == "session_failed" and cut["class"] == "slam_link" and cut["stage"] == "transport"
    assert cut["exposure"][-1] == "transport"


def test_a_session_end_is_not_attached_to_a_mission_before_a_later_started_one(tmp_path):
    run = {"success": False, "failure_reason": "slam_link_failed: timeout", "transitions": []}
    d = session(tmp_path, "D_x_s2", [("a", "ack", "m1"), ("b", "ack", "m2")], [], run=run)
    rows = C.session_rows(d, 2)["rows"]
    assert rows[0]["outcome"] == "no_record" and rows[1]["outcome"] == "session_failed"


def test_queue_blocked_when_every_started_mission_succeeded_but_the_queue_stopped(tmp_path):
    d = session(tmp_path, "O0_line_m_s0", [("a", "ack", "m1")], [("m1", "done", None, 0, 10, "complete")],
                run={"success": True, "transitions": []}, queue={"stop_reason": "상태 수신 끊김", "stops": 1})
    s = C.session_rows(d, 3)
    assert s["queue_blocked"] and s["queue"]["stop_reason"] == "상태 수신 끊김"


def test_classes_and_stage_parsing():
    assert C.classify("Tracking timeout in return_home") == "tracking"
    assert C.classify("scene_replan_refused:permit_stop:blocked:occupied") == "planning"
    assert C.classify("transport_recovery_after_docking") == "docking"
    assert C.classify("Excessive body tilt in lift") == "load_safety"
    assert C.classify("observe:stall_exhausted") == "perception"
    assert C.classify("something new") == "other" and C.classify(None) == "unknown"
    assert C.stage_of("Excessive body tilt in lift", "transport") == "lift"
    assert C.stage_of("transport_recovery_after_docking", "lower") == "lower"


def test_pair_comparison_is_undecidable_when_a_value_is_missing():
    a = {"session": "A", "rows": [{"order": 1, "start": "confirmed", "outcome": "done", "reason": None, "stage": None,
                                   "ended_s": 10.0, "docking_rounds": 2},
                                  {"order": 2, "start": "confirmed", "outcome": "failed", "reason": "x", "stage": "lift",
                                   "ended_s": 20.0, "docking_rounds": 0},
                                  {"order": 3, "start": "confirmed", "outcome": None, "reason": None, "stage": None,
                                   "ended_s": None, "docking_rounds": None}]}
    b = json.loads(json.dumps(a))
    b["session"] = "B"
    b["rows"][1]["ended_s"] = 21.0
    out = C.compare_pair(a, b)["missions"]
    assert out[0]["verdict"] == "same"
    assert out[1]["verdict"] == "different" and out[1]["first_difference"] == "ended_s"
    assert out[2]["verdict"] == "undecidable"
    b["rows"][0]["docking_rounds"] = None
    assert C.compare_pair(a, b)["missions"][0]["verdict"] == "undecidable"


def test_survey_groups_arms_and_flags_transport_tracking(tmp_path):
    session(tmp_path, "D_line_m_s0", [("a", "ack", "m1")],
            [("m1", "failed", "Tracking failed in transport", 0, 9, "transport")], run={"success": True, "transitions": []})
    session(tmp_path, "O6_line_m_s0", [("a", "ack", "m1")], [("m1", "done", None, 0, 9, "complete")],
            run={"success": True, "transitions": []})
    res = C.survey(tmp_path, 1)
    keys = set(res["summary"])
    assert keys == {"D|line_m", "O6|line_m"}
    d = res["summary"]["D|line_m"]
    assert d["transport_tracking"] and d["class"] == {"tracking": 1} and d["first_block"] == {"tracking@1": 1}
    assert res["summary"]["O6|line_m"]["all_done"] == 1
    json.dumps(res, allow_nan=False)


def test_a_preparation_failure_is_its_own_stage_and_loaded_failures_carry_the_docking_substage(tmp_path):
    d = session(tmp_path, "D_line_m_s3", [("a", "ack", "m1")], [("m1", "failed", "prepare:timeout", 0, 3, None)],
                run={"success": True, "transitions": []})
    r = C.session_rows(d, 1)["rows"][0]
    assert r["stage"] == "preparing" and r["class"] == "preparation" and r["exposure"] == ["preparing"]
    assert C.substage("transport", {"docking_retries": 0, "docking_rounds": 1}) == "after_delivery_match"
    assert C.substage("transport", {"docking_retries": 2, "docking_rounds": 1}) == "reapproach"
    assert C.substage("transport", {"docking_retries": 0, "docking_rounds": 0}) == "before_delivery_match"
    assert C.substage("lift", {}) is None


def test_the_spool_manifest_is_read_per_epoch(tmp_path):
    d = session(tmp_path, "D_line_m_s4", [("a", "ack", "m1")], [("m1", "done", None, 0, 3, "complete")],
                run={"success": True, "transitions": []})
    _write(d / "live" / "epoch_1" / "run" / "spool" / "manifest.json",
           {"slam.control": {"chunks": 3, "dropped": 2}, "complete": False, "error_count": 1})
    sp = C.session_rows(d, 1)["spool"][0]
    assert sp == {"complete": False, "error_count": 1, "dropped": {"slam.control": 2}}
