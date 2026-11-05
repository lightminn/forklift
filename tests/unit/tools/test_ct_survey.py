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
        {"from": "complete", "to": "observe", "time_s": 4.6},
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
                                   "ended_s": 10.0, "docking_rounds": 2, "delivery_error": 0.02},
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
    retry = {"time_s": 10.0, "counted": True, "refused": False}
    assert C.substage("transport", {"docking_retries": [], "docking_round_times": [5.0]}, 20) == "after_delivery_match"
    assert C.substage("transport", {"docking_retries": [retry], "docking_round_times": [5.0]}, 20) == "reapproach"
    # a re-approach that reached its retry match is over: a later failure is after a match
    assert C.substage("lower", {"docking_retries": [retry], "docking_round_times": [5.0, 15.0]}, 20) == "after_delivery_match"
    assert C.substage("transport", {"docking_retries": [dict(retry, refused=True)], "docking_round_times": [5.0]},
                      20) == "reapproach_refused"
    # a retry logged after the end does not count
    assert C.substage("transport", {"docking_retries": [dict(retry, time_s=30.0)], "docking_round_times": []},
                      20) == "before_delivery_match"
    assert C.substage("lift", {}, 1.0) is None


def test_the_spool_manifest_is_read_per_epoch(tmp_path):
    d = session(tmp_path, "D_line_m_s4", [("a", "ack", "m1")], [("m1", "done", None, 0, 3, "complete")],
                run={"success": True, "transitions": []})
    _write(d / "live" / "epoch_1" / "run" / "spool" / "manifest.json",
           {"slam.control": {"chunks": 3, "items": 9, "dropped_items": 2, "unsettled_items": 1}, "complete": False,
            "error_count": 1})
    sp = C.session_rows(d, 1)["spool"][0]
    assert sp == {"complete": False, "error_count": 1, "dropped_items": {"slam.control": 2},
                  "unsettled_items": {"slam.control": 1}}


# --- Codex tool review (round 3 of the plan thread) counterexamples ---

def test_a_mission_folder_confirms_a_start_the_probe_did_not_ack(tmp_path):
    d = session(tmp_path, "D_line_m_s5", [("a", "none", None)], [("m1", "done", None, 0, 9, "complete")],
                run={"success": True, "transitions": []})
    r = C.session_rows(d, 2)["rows"]
    assert r[0]["start"] == "confirmed" and r[0]["start_evidence"] == "mission_folder" and r[0]["outcome"] == "done"
    # no probe at all: the folders give the rows
    d2 = tmp_path / "D_line_m_s6"
    base = d2 / "live" / "epoch_1" / "run"
    _write(base / "missions" / "m1" / "result.json", {"result": {"outcome": "done", "started_s": 0, "ended_s": 5}})
    rows = C.session_rows(d2, 3)["rows"]
    assert rows[0]["start"] == "confirmed" and rows[0]["outcome"] == "done"


def test_a_settled_probe_result_is_not_overwritten_by_the_session_end(tmp_path):
    d = tmp_path / "D_line_m_s7"
    _write(d / "probe.json", {"missions_run": [{"item_id": "a", "submit": {"t": "ack", "mission_id": "m1"},
                                                "result": {"mission_id": "m1", "outcome": "done", "reason": None}}]})
    _write(d / "live" / "epoch_1" / "run" / "result.json", {"success": False, "failure_reason": "cleanup_failed"})
    r = C.session_rows(d, 1)["rows"][0]
    assert r["outcome"] == "done" and r["outcome_from"] == "probe_result"


def test_the_previous_missions_complete_is_not_the_next_ones_exposure(tmp_path):
    run = {"success": True, "transitions": [
        {"from": "observe", "to": "approach", "time_s": 2}, {"from": "approach", "to": "complete", "time_s": 4},
        {"from": "complete", "to": "observe", "time_s": 6}, {"from": "observe", "to": "approach", "time_s": 8}]}
    d = session(tmp_path, "D_line_m_s8", [("a", "ack", "m1"), ("b", "ack", "m2")],
                [("m1", "done", None, 0, 5, "complete"), ("m2", "failed", "Tracking failed in approach", 6, 9, "approach")],
                run=run)
    r2 = C.session_rows(d, 2)["rows"][1]
    assert "complete" not in r2["exposure"] and r2["exposure"] == ["preparing", "observe", "approach"]


def test_grid_plans_come_from_the_run_record_within_the_mission(tmp_path):
    run = {"success": True, "transitions": [], "obstacle_layer": {"grid_plans": [
        {"stamp_s": 1.0, "slam_used": True, "slam_map_note": None},
        {"stamp_s": 2.0, "slam_used": False, "slam_map_note": {"slam_static_layer": "off_deterministic"}},
        {"stamp_s": 50.0, "slam_used": True}]}}
    d = session(tmp_path, "D_line_m_s9", [("a", "ack", "m1")], [("m1", "done", None, 0.5, 10, "complete")], run=run)
    r = C.session_rows(d, 1)["rows"][0]
    assert (r["plans"], r["plans_slam_used"], r["plans_static_off"]) == (2, 1, 1)


def test_pairs_compare_the_delivery_error_and_need_the_docking_rounds(tmp_path):
    base = {"order": 1, "start": "confirmed", "outcome": "done", "reason": None, "stage": None, "substage": None,
            "exposure": ["preparing"], "ended_s": 10.0, "docking_rounds": 2, "delivery_error": 0.01}
    a = {"session": "A", "rows": [dict(base)]}
    b = {"session": "B", "rows": [dict(base, delivery_error=0.20)]}
    assert C.compare_pair(a, b)["missions"][0]["first_difference"] == "delivery_error"
    a2 = {"session": "A", "rows": [dict(base, docking_rounds=None)]}
    b2 = {"session": "B", "rows": [dict(base, docking_rounds=None)]}
    assert C.compare_pair(a2, b2)["missions"][0]["verdict"] == "undecidable"


def test_a_settled_reject_of_the_last_mission_is_a_queue_block(tmp_path):
    d = session(tmp_path, "D_line_m_s10", [("a", "ack", "m1"), ("b", "ack", "m2"), ("c", "reject", "localization_lost")],
                [("m1", "done", None, 0, 5, "complete"), ("m2", "done", None, 6, 10, "complete")],
                run={"success": True, "transitions": []})
    s = C.session_rows(d, 3)
    assert s["queue_blocked"]
    r3 = s["rows"][2]
    assert r3["start"] == "rejected" and r3["class"] == "queue" and r3["reason"] == "localization_lost"
    res = C.survey(tmp_path, 3)
    assert res["summary"]["D|line_m"]["first_block"] == {"queue@3": 1}


def test_probe_errors_and_missing_attempts_are_infrastructure(tmp_path):
    d = tmp_path / "O0_line_m_s0"
    _write(d / "probe.json", {"error": "launcher_ended", "missions_run": []})
    s = C.session_rows(d, 3)
    assert s["infra"] == "probe_error: launcher_ended" and s["rows"][0]["class"] == "infra"
    runs = tmp_path / "RUNS.jsonl"
    runs.write_text(json.dumps({"name": "O0_line_m_s0"}) + "\n" + json.dumps({"name": "D_line_m_s1"}) + "\n")
    res = C.survey(tmp_path, 3, runs_list=runs)
    names = {x["session"]: x for x in res["sessions"]}
    assert names["D_line_m_s1"]["infra"] == "no_records"
    assert res["summary"]["D|line_m"]["infra"] == 1


# --- Codex round 4 counterexamples ---

def test_unknown_starts_and_repeated_phases_are_not_same():
    base = {"order": 1, "start": "unknown", "outcome": None, "reason": None, "stage": None, "substage": None,
            "phase_history": None, "ended_s": None, "docking_rounds": None, "delivery_error": None}
    assert C.compare_pair({"session": "A", "rows": [dict(base)]},
                          {"session": "B", "rows": [dict(base)]})["missions"][0]["verdict"] == "undecidable"
    done = dict(base, start="confirmed", outcome="failed", reason="x", stage="insert", ended_s=5.0, docking_rounds=0)
    a = dict(done, phase_history=["observe", "approach", "insert"])
    b = dict(done, phase_history=["observe", "approach", "observe", "approach", "insert"])
    out = C.compare_pair({"session": "A", "rows": [a]}, {"session": "B", "rows": [b]})["missions"][0]
    assert out["verdict"] == "different" and out["first_difference"] == "phase_history"


def test_a_missing_middle_folder_keeps_its_place(tmp_path):
    d = tmp_path / "D_line_m_s11"
    base = d / "live" / "epoch_1" / "run"
    for mid, pal in (("m1", "P-1"), ("m3", "P-3")):
        _write(base / "missions" / mid / "result.json",
               {"result": {"mission_id": mid, "outcome": "done", "started_s": 0, "ended_s": 5, "pallet_id": pal}})
    _write(base / "result.json", {"success": True, "transitions": [], "mission_results": [
        {"mission_id": "m2", "outcome": "done", "started_s": 6, "ended_s": 9, "pallet_id": "P-2"}]})
    rows = C.session_rows(d, 3)["rows"]
    assert [r["mission_id"] for r in rows] == ["m1", "m2", "m3"]
    assert [r["pallet_id"] for r in rows] == ["P-1", "P-2", "P-3"]
    assert rows[1]["start_evidence"] == "results_book" and rows[1]["outcome"] == "done"
    # without the book the gap still keeps its place, as a mission with no record
    _write(base / "result.json", {"success": True, "transitions": []})
    rows = C.session_rows(d, 3)["rows"]
    assert [r["mission_id"] for r in rows] == ["m1", "m2", "m3"]
    assert rows[1]["start_evidence"] == "mission_numbering" and rows[1]["outcome"] == "no_record"


def test_a_mission_cut_while_preparing_does_not_borrow_the_previous_complete(tmp_path):
    run = {"success": False, "failure_reason": "slam_link_failed: timeout",
           "transitions": [{"from": "observe", "to": "approach", "time_s": 2}, {"from": "approach", "to": "complete", "time_s": 4}]}
    d = session(tmp_path, "D_line_m_s12", [("a", "ack", "m1"), ("b", "ack", "m2")],
                [("m1", "done", None, 0, 5, "complete")], run=run)
    cut = C.session_rows(d, 2)["rows"][1]
    assert cut["outcome"] == "session_failed" and cut["stage"] == "preparing" and cut["exposure"] == ["preparing"]


def test_a_first_submission_rejected_is_a_queue_block(tmp_path):
    d = session(tmp_path, "D_line_m_s13", [("a", "reject", "localization_lost")], [], run=None)
    s = C.session_rows(d, 3)
    assert s["queue_blocked"] and s["rows"][0]["class"] == "queue" and s["rows"][0]["reason"] == "localization_lost"
