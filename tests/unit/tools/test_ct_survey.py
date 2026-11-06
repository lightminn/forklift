"""tools/ct_survey.py on synthetic live session folders (docs/plans/2026-11-06-ct-status-survey.md)."""

import json
import sys

import pytest
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
                                   "ended_s": 10.0, "docking_rounds": 2, "delivery_error": 0.02,
                                   "phase_history": ["observe", "complete"]},
                                  {"order": 2, "start": "confirmed", "outcome": "failed", "reason": "x", "stage": "lift",
                                   "ended_s": 20.0, "docking_rounds": 0, "phase_history": ["observe", "lift"]},
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
            "exposure": ["preparing"], "phase_history": ["observe", "complete"], "ended_s": 10.0, "docking_rounds": 2,
            "delivery_error": 0.01}
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


# --- Codex round 5 counterexamples ---

def test_the_previous_complete_at_the_same_time_is_not_the_cut_missions(tmp_path):
    run = {"success": False, "failure_reason": "slam_link_failed: timeout",
           "transitions": [{"from": "approach", "to": "complete", "time_s": 5.0}]}
    d = session(tmp_path, "D_line_m_s14", [("a", "ack", "m1"), ("b", "ack", "m2")],
                [("m1", "done", None, 0, 5.0, "complete")], run=run)
    cut = C.session_rows(d, 2)["rows"][1]
    assert cut["stage"] == "preparing" and cut["exposure"] == ["preparing"] and cut["phase_history"] == []


def test_the_book_fills_a_folder_without_its_result(tmp_path):
    d = tmp_path / "D_line_m_s15"
    base = d / "live" / "epoch_1" / "run"
    _write(base / "missions" / "m1" / "records.json", {"state": {"phase": "complete"}})
    _write(base / "result.json", {"success": True, "transitions": [], "mission_results": [
        {"mission_id": "m1", "outcome": "done", "started_s": 1.0, "ended_s": 9.0}]})
    r = C.session_rows(d, 1)["rows"][0]
    assert r["outcome"] == "done" and r["ended_s"] == 9.0


def test_pairs_need_the_phase_history_past_the_preparation():
    row = {"order": 1, "start": "confirmed", "outcome": "failed", "reason": "Tracking failed in transport",
           "stage": "transport", "substage": "before_delivery_match", "phase_history": None, "ended_s": 9.0,
           "docking_rounds": 0, "delivery_error": None}
    out = C.compare_pair({"session": "A", "rows": [dict(row)]}, {"session": "B", "rows": [dict(row)]})["missions"][0]
    assert out["verdict"] == "undecidable" and "phase_history" in out["missing"]
    prep = dict(row, stage="preparing", reason="prepare:timeout", substage=None, phase_history=[])
    out = C.compare_pair({"session": "A", "rows": [dict(prep)]}, {"session": "B", "rows": [dict(prep)]})["missions"][0]
    assert out["verdict"] == "same"


def test_indicators_count_the_recoveries_a_mission_needed(tmp_path):
    st = {"stall_replans": [{"phase": "return_home"}], "cusp_stalls": [], "observe_replans": [{"phase": "observe"}],
          "docking_retries": [{"counted": True, "replaces": "docking_unaligned:{...}"}, {"counted": False}],
          "observation_attempts": [1, 2], "delivery_docking": {"status": "done"},
          "insertion_error": {"position_m": 0.01}, "lateral_clearance_min_m": {"insert": {"left": 0.07, "right": 0.05}},
          "return_home_error": {"position_m": 0.03}}
    ind = C.indicators(st)
    assert ind["stall_replans"] == 1 and ind["observe_replans"] == 1 and ind["d7_reapproaches"] == 1
    assert ind["d7_triggers"] == ["docking_unaligned"] and ind["recovery_phases"] == ["observe", "return_home"]
    assert ind["lateral_clearance_insert_m"] == 0.05 and ind["docking_status"] == "done"
    d = tmp_path / "D_line_m_s20"
    base = d / "live" / "epoch_1" / "run"
    _write(base / "missions" / "m1" / "result.json", {"result": {"mission_id": "m1", "outcome": "done", "started_s": 0,
                                                                  "ended_s": 5}, "delivery_error_m": 0.02})
    _write(base / "missions" / "m1" / "records.json", {"state": dict(st, phase="complete")})
    _write(base / "result.json", {"success": True, "transitions": [], "obstacle_layer": {"grid_plans": [], "live_plans": []}})
    res = C.survey(tmp_path, 1)
    o = res["summary"]["D|line_m"]
    assert o["missions_with_recovery"] == 1 and "observation_retry" not in o["recoveries"]
    assert o["observation_attempts"] == {2: 1}  # the normal flow observes twice: a count, not a recovery
    assert o["delivery_error_m"] == {"n": 1, "median": 0.02, "max": 0.02}
    assert res["sessions"][0]["rows"][0]["plans"] is None  # no plan list kept: not recorded, not zero


def test_the_delivery_error_median_of_an_even_sample_is_the_mean_of_the_middle_two(tmp_path):
    for k, err in enumerate((0.01, 0.02, 0.03, 0.10)):
        d = tmp_path / f"D_line_m_s{30 + k}"
        base = d / "live" / "epoch_1" / "run"
        _write(base / "missions" / "m1" / "result.json",
               {"result": {"mission_id": "m1", "outcome": "done", "started_s": 0, "ended_s": 5}, "delivery_error_m": err})
        _write(base / "result.json", {"success": True, "transitions": []})
    o = C.survey(tmp_path, 1)["summary"]["D|line_m"]
    assert o["delivery_error_m"]["median"] == pytest.approx(0.025) and o["delivery_error_m"]["max"] == 0.10


# --- near_capture_tally (docs/plans/2026-11-07-near-capture-frequency.md) -------------------------------------------


NEAR_FAIL = "near_capture_failed:perception_no_pallet:no_expected_candidate"
ARGS = {"output": "/x/run", "seed": 100, "slam_noise_seed": 100, "live_port": 9100, "live_port_file": "/x/p.json",
        "slam_feedback": "/x/s", "slam_map_dir": "/x/b", "scene_file": "/nowhere/line_m_session.json",
        "slam_map_bundle": "/m/a", "expected_map_id": "aa", "slam_choice": "localization (registry: a)",
        "pallet_prior_loaded": {"catalogue_version": "epal6"}, "recognition_proposals": "learned"}


def near_attempt(status="valid", reason=None, sha="d1", t=10.0, first=None, applied=None, far=False):
    """An observation attempt: far, or near (its expected record), with the waiver's record when ``first`` is given."""
    a = {"pocket_observation": {"status": status, "reason": reason}, "depth_sha256": sha, "t_before_capture_s": t,
         "loop_step": int(t * 10), "accepted_pose": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]}
    if not far:
        a["expected"] = {"candidates": []}
    if first is not None:
        a["lower_waiver"] = {"first": first, "second": {"candidates": []}, "applied": bool(applied)}
    return a


def mission_state(attempts, near=True):
    state = {"phase": "approach", "observation_attempts": attempts}
    if near:
        state["near_capture"] = {"status": "done", "far_attempt": 1}
    return state


def tally_session(tmp_path, name, rows, submits=None, run=None, args=ARGS, samples=()):
    """rows: [(mid, outcome, reason, attempts or None)]; attempts None leaves the mission folder without its records."""
    submits = submits if submits is not None else [(r[0], "ack", r[0]) for r in rows]
    top = dict(run or {})
    if args is not None:
        top.setdefault("arguments", args)
    d = session(tmp_path, name, submits, [(r[0], r[1], r[2], float(i), float(i) + 0.5, "approach")
                                          for i, r in enumerate(rows)], run=top)
    base = d / "live" / "epoch_1" / "run"
    for mid, _, _, attempts in rows:
        if attempts is None:
            (base / "missions" / mid / "records.json").unlink()
        else:
            _write(base / "missions" / mid / "records.json", {"state": mission_state(attempts)})
    if samples:
        spool = base / "spool" / "state.samples"
        spool.mkdir(parents=True)
        (spool / "0.jsonl").write_text("".join(json.dumps(s) + "\n" for s in samples))
    return d


def tally(tmp_path, scheduled=3):
    res = C.survey(tmp_path, scheduled)
    return C.near_capture_tally(tmp_path, res["sessions"])


def only(out, key="populations"):
    assert len(out[key]) == 1
    return out[key][0]


def test_config_hashes_ignore_the_run_and_the_scene_but_not_a_setting():
    a = dict(ARGS)
    b = dict(ARGS, output="/y", seed=7, slam_noise_seed=7, live_port=1, live_port_file="/y", slam_feedback="/y",
             slam_map_dir="/y")
    assert C.config_hashes(a) == C.config_hashes(b)
    c = dict(ARGS, scene_file="/nowhere/random_m_session.json", slam_map_bundle="/m/b", expected_map_id="bb",
             slam_choice="localization (registry: b)")
    assert C.config_hashes(a)[0] != C.config_hashes(c)[0]
    assert C.config_hashes(a)[1] == C.config_hashes(c)[1]
    d = dict(ARGS, recognition_proposals="oracle")
    assert C.config_hashes(a)[0] != C.config_hashes(d)[0] and C.config_hashes(a)[1] != C.config_hashes(d)[1]
    assert C.config_hashes(None) == (None, None)


def test_observation_waypoints_leave_the_hash_only_as_the_scene_own_value(tmp_path):
    scene = tmp_path / "s.json"
    scene.write_text(json.dumps({"pallets": [{"observation_waypoints": [[1.0, 0.0, 0.0]]},
                                             {"observation_waypoints": [[2.0, 0.0, 0.0]]}]}))
    base = dict(ARGS, scene_file=str(scene))
    one = dict(base, observation_waypoints=[[1.0, 0.0, 0.0]])
    two = dict(base, observation_waypoints=[[2.0, 0.0, 0.0]])
    assert C.config_hashes(one) == C.config_hashes(two)  # the queue stopped at another mission: the same condition
    named = dict(base, observation_waypoints=[[9.0, 0.0, 0.0]])
    assert C.config_hashes(one)[0] != C.config_hashes(named)[0]  # set by hand: a setting
    gone = dict(one, scene_file="/nowhere.json")
    assert C.config_hashes(gone)[1] != C.config_hashes(dict(two, scene_file="/nowhere.json"))[1]  # unreadable: it stays


def test_a_rescued_capture_counts_as_a_first_call_invalid_and_never_in_the_numerator(tmp_path):
    bad = {"status": "no_pallet", "reason": "no_expected_candidate"}
    tally_session(tmp_path, "O0_line_m_s100", [
        ("m1", "done", None, [near_attempt(far=True), near_attempt(first=bad, applied=True)])])
    out = tally(tmp_path)
    cell = only(out, "cells")
    assert cell["first_call_invalid_by_reason"] == {"no_pallet:no_expected_candidate": 1}
    assert cell["final_invalid_by_reason"] == {}
    assert (cell["waiver_second_calls"], cell["waiver_rescues"]) == (1, 1)
    assert cell["missions_recovered_same_capture"] == 1 and cell["missions_recovered_new_capture"] == 0
    assert cell["missions_ended_near"] == 0 and cell["capture_attempts"] == 1 and cell["missions_reached_near"] == 1


def test_a_later_valid_capture_is_a_new_capture_recovery(tmp_path):
    tally_session(tmp_path, "O0_line_m_s100", [
        ("m1", "done", None, [near_attempt(far=True), near_attempt("no_pallet", "x", sha="d2", t=11.0),
                              near_attempt(sha="d3", t=12.0)])])
    cell = only(tally(tmp_path), "cells")
    assert cell["missions_recovered_new_capture"] == 1 and cell["missions_recovered_same_capture"] == 0
    assert cell["final_invalid_by_reason"] == {"no_pallet:x": 1}


def test_a_near_end_counts_once_with_its_reason_and_stops_the_session(tmp_path):
    tally_session(tmp_path, "O0_random_m_s100", [
        ("m1", "done", None, [near_attempt(far=True), near_attempt()]),
        ("m2", "failed", NEAR_FAIL, [near_attempt(far=True), near_attempt("no_pallet", "no_expected_candidate", sha="d2"),
                                     near_attempt("no_pallet", "no_expected_candidate", sha="d3", t=12.0)])],
                  submits=[("m1", "ack", "m1"), ("m2", "ack", "m2")])
    out = tally(tmp_path)
    pop = only(out)
    assert (pop["rates"]["N"], pop["rates"]["k"], pop["rates"]["R"]) == (2, 1, 2)
    ended = [c for c in out["cells"] if c["missions_ended_near"]]
    assert len(ended) == 1 and ended[0]["mission_seq"] == 2
    assert ended[0]["ended_by_reason"] == {"no_pallet:no_expected_candidate": 1}
    assert ended[0]["ended_session_ids"] == ["O0_random_m_s100"]
    cell = only(out, "session_cells")
    assert cell["stopped_near_session_ids"] == ["O0_random_m_s100"] and cell["unsubmitted_after_stop"] == 1
    assert cell["started_session_ids"] == ["O0_random_m_s100"]


def test_a_misaligned_end_is_in_the_numerator_and_a_frameless_one_is_marked(tmp_path):
    reason = "near_capture_misaligned:{'lateral': 0.1}"
    tally_session(tmp_path, "O0_line_m_s100", [("m1", "failed", reason, [near_attempt(far=True), near_attempt()])],
                  submits=[("m1", "ack", "m1")])
    tally_session(tmp_path, "O0_line_m_s101", [
        ("m1", "failed", "near_capture_no_final_straight", [near_attempt(far=True)])], submits=[("m1", "ack", "m1")])
    out = tally(tmp_path, 1)
    pop = only(out)
    assert pop["rates"]["k"] == 2 and pop["rates"]["R"] == 2  # k is inside R
    assert pop["rates"]["ended_without_capture"] == 1
    assert pop["ended_by_reason"] == {"near_capture_misaligned": 1, "near_capture_no_final_straight": 1}


def test_the_attempts_of_the_mission_a_run_ended_in_come_from_the_top_and_count_once(tmp_path):
    shared = near_attempt(far=True, sha="d0", t=1.0)
    near = near_attempt("no_pallet", "no_expected_candidate", sha="d9", t=30.0)
    run = {"success": False, "failure_reason": NEAR_FAIL, "observation_attempts": [shared, near],
           "near_capture": {"status": "failed", "far_attempt": 1}}
    tally_session(tmp_path, "O0_line_m_s100", [("m1", "done", None, [shared, near_attempt(sha="d1")])],
                  submits=[("m1", "ack", "m1"), ("m2", "ack", "m2")], run=run)
    out = tally(tmp_path)
    second = next(c for c in out["cells"] if c["mission_seq"] == 2)
    assert second["missions_ended_near"] == 1 and second["capture_attempts"] == 1
    first = next(c for c in out["cells"] if c["mission_seq"] == 1)
    assert first["capture_attempts"] == 1


def test_top_attempts_nobody_can_place_stay_in_an_unknown_cell(tmp_path):
    run = {"success": True, "observation_attempts": [near_attempt(sha="d9", t=30.0)]}
    samples = [{"time_s": 1.0, "pallet_id": "P-9", "pallet_position_m": [3.0, 0.0, 0.0], "pallet_yaw_rad": 0.0}]
    tally_session(tmp_path, "O0_line_m_s100", [("m1", "done", None, [near_attempt(far=True), near_attempt()])],
                  submits=[("m1", "ack", "m1")], run=run, samples=samples)
    out = tally(tmp_path, 1)
    unknown = [c for c in out["cells"] if c["mission_seq"] == "unknown"]
    assert len(unknown) == 1 and unknown[0]["capture_attempts"] == 1 and unknown[0]["missions_started"] == 0


def test_rates_give_a_range_for_missing_results(tmp_path):
    r = C.near_rates(k=2, N=100, S=0, R=20, U=80, m_R=0, m_U=80)
    assert r["conditional"]["low"] == pytest.approx(2 / 100) and r["conditional"]["high"] == pytest.approx(82 / 100)
    assert r["near_end"]["low"] == pytest.approx(0.02) and r["near_end"]["high"] == pytest.approx(0.82)
    r = C.near_rates(k=0, N=0, S=0, R=0, U=0, m_R=0, m_U=0)
    assert r["near_end"] is None and r["conditional"] is None
    r = C.near_rates(k=1, N=4, S=1, R=2, U=0, m_R=0, m_U=0)
    assert r["near_end"]["low"] == pytest.approx(1 / 5) and r["near_end"]["high"] == pytest.approx(2 / 5)
    assert r["conditional"]["low"] == pytest.approx(1 / 3) and r["conditional"]["high"] == pytest.approx(2 / 3)
    assert C.wilson(0, 60)[1] == pytest.approx(0.0602, abs=2e-4) and C.wilson(1, 60)[1] == pytest.approx(0.0886, abs=2e-4)


def mission(session, near=False, cause=None, start="confirmed", missing=False):
    return {"session": session, "start": start, "ended_near": near, "cause": cause, "result_missing": missing}


def missions_of(n, near_sessions=(), other=()):
    out = [mission(f"s{i}") for i in range(n - len(near_sessions) - len(other))]
    out += [mission(s, near=True, cause="near_capture_") for s in near_sessions]
    out += [mission(s, cause=c) for s, c in other]
    return out


@pytest.mark.parametrize("n,sessions,want", [(49, ("a", "b"), "denominator_short"), (50, ("a", "b"), "met"),
                                             (50, ("a",), "not_met"), (60, ("a", "b"), "met"), (100, ("a",), "not_met")])
def test_trigger_a_boundaries(n, sessions, want):
    assert C.review_triggers(missions_of(n, sessions))["a"] == want


def test_trigger_a_needs_the_rate():
    assert C.review_triggers(missions_of(101, ("a", "b")))["a"] == "not_met"  # 1.98 %
    assert C.review_triggers(missions_of(100, ("a", "b")))["a"] == "met"  # 2 % exactly


def test_trigger_b_needs_two_sessions_and_the_top_cause_with_ties():
    assert C.review_triggers(missions_of(10, ("a", "b"), [("c", "Tracking failed"), ("d", "Tracking failed")]))["b"] == "met"
    assert C.review_triggers(missions_of(10, ("a", "b"), [("c", "T"), ("d", "T"), ("e", "T")]))["b"] == "not_met"
    assert C.review_triggers(missions_of(10, ("a",)))["b"] == "not_met"


def test_trigger_undecidable_when_the_assignments_differ():
    ms = missions_of(100, ("a", "b"), [("c", "T")])
    ms[0], ms[1] = mission("x", missing=True), mission("y", missing=True)  # two others could beat the near count
    out = C.review_triggers(ms)
    assert out["b"] == "undecidable" and out["a"] == "met"
    ms = missions_of(49, ("a", "b")) + [mission("z", start="unknown", missing=True)]
    assert C.review_triggers(ms)["a"] == "undecidable"  # 49 or 50 started
    assert C.review_triggers(missions_of(47, ("a", "b")))["a"] == "denominator_short"


def test_too_many_unknown_missions_are_undecidable_not_a_long_wait():
    ms = missions_of(100, ("a", "b")) + [mission(f"u{i}", missing=True) for i in range(C.MAX_ASSIGNED + 1)]
    assert C.review_triggers(ms) == {"a": "undecidable", "b": "undecidable", "assigned": None}  # 2 of 113, or more
    few = [mission(f"u{i}", start="unknown", missing=True) for i in range(C.MAX_ASSIGNED + 1)]
    assert C.review_triggers(few)["a"] == "denominator_short"  # no assignment reaches the count
    sure = missions_of(47, ("a", "b")) + [mission(f"u{i}", missing=True) for i in range(C.MAX_ASSIGNED + 1)]
    assert C.review_triggers(sure)["a"] == "met"  # 60 started in every assignment, k at least 2 of them
    many = missions_of(40, [f"n{i}" for i in range(16)]) + [mission(f"u{i}", missing=True) for i in range(C.MAX_ASSIGNED + 1)]
    assert C.review_triggers(many)["b"] == "met"  # no assignment gives another cause more


def test_two_scenes_pool_by_the_operating_configuration_and_arms_do_not(tmp_path):
    random_args = dict(ARGS, scene_file="/nowhere/random_m_session.json", slam_map_bundle="/m/b", expected_map_id="bb",
                       slam_choice="localization (registry: b)", seed=101, slam_noise_seed=101)
    tally_session(tmp_path, "O0_line_m_s100", [("m1", "done", None, [near_attempt(far=True), near_attempt()])],
                  submits=[("m1", "ack", "m1")])
    tally_session(tmp_path, "O0_random_m_s101", [("m1", "done", None, [near_attempt(far=True), near_attempt()])],
                  submits=[("m1", "ack", "m1")], args=random_args)
    out = tally(tmp_path, 1)
    pop = only(out)
    assert pop["rates"]["N"] == 2 and set(pop["by_scene"]) == {"line_m", "random_m"}
    assert len(out["cells"]) == 2 and out["cells"][0]["operating_config_sha256"] == out["cells"][1]["operating_config_sha256"]
    tally_session(tmp_path, "O6_line_m_s100", [("m1", "done", None, [near_attempt(far=True), near_attempt()])],
                  submits=[("m1", "ack", "m1")], args=dict(ARGS, near_end_stand="on"))
    assert len(tally(tmp_path, 1)["populations"]) == 2


def test_a_session_with_no_arguments_joins_its_arm_and_is_named(tmp_path):
    tally_session(tmp_path, "O0_line_m_s100", [("m1", "done", None, [near_attempt(far=True), near_attempt()])],
                  submits=[("m1", "ack", "m1")])
    tally_session(tmp_path, "O0_line_m_s101", [("m1", "failed", "Tracking failed in approach", None)],
                  submits=[("m1", "ack", "m1")], args=None)
    out = tally(tmp_path, 1)
    pop = only(out)
    assert pop["rates"]["N"] == 2 and pop["config_assumed_sessions"] == ["O0_line_m_s101"]


def test_start_unknown_stays_apart_from_a_missing_result(tmp_path):
    tally_session(tmp_path, "O0_line_m_s100", [("m1", "done", None, [near_attempt(far=True), near_attempt()])],
                  submits=[("m1", "ack", "m1"), ("b", "none", None), ("c", "ack", "m9")])
    r = only(tally(tmp_path))["rates"]
    assert (r["N"], r["S"], r["result_missing"]) == (2, 1, 1)
    assert r["U"] == 1  # the acked m9 with no record may have reached it; the unknown start is S, not U


def test_an_unexposed_shape_is_named_unverified(tmp_path):
    tally_session(tmp_path, "O0_line_m_s100", [("m1", "done", None, [near_attempt(far=True), near_attempt()])],
                  submits=[("m1", "ack", "m1")])
    assert tally(tmp_path, 1)["unverified_shapes"] == ["t11"]


def test_an_attempt_that_took_no_submission_is_left_out_and_named(tmp_path):
    tally_session(tmp_path, "O0_line_m_s100", [("m1", "done", None, [near_attempt(far=True), near_attempt()])],
                  submits=[("m1", "ack", "m1")])
    (tmp_path / "O0_line_m_s100.attempt1" / "live" / "epoch_1" / "run").mkdir(parents=True)
    out = tally(tmp_path, 1)
    assert only(out)["rates"]["N"] == 1 and out["excluded_attempts"] == ["O0_line_m_s100.attempt1"]


def test_the_first_attempt_that_took_a_submission_is_the_valid_one_whatever_its_name(tmp_path):
    tally_session(tmp_path, "O0_line_m_s100", [("m1", "done", None, [near_attempt(far=True), near_attempt()])],
                  submits=[("m1", "ack", "m1")])
    tally_session(tmp_path, "O0_line_m_s100.attempt1", [("m1", "failed", NEAR_FAIL, [near_attempt()])],
                  submits=[("m1", "ack", "m1")])
    out = tally(tmp_path, 1)
    assert only(out)["rates"]["k"] == 1 and out["excluded_attempts"] == ["O0_line_m_s100"]


def test_an_attempt_known_only_from_its_run_record_is_the_first_valid_one(tmp_path):
    tally_session(tmp_path, "O0_line_m_s100", [("m1", "done", None, [near_attempt(far=True), near_attempt()])],
                  submits=[("m1", "ack", "m1")])
    run = {"success": False, "failure_reason": NEAR_FAIL, "near_capture": {"status": "failed", "far_attempt": 1},
           "observation_attempts": [near_attempt(far=True, sha="d0", t=1.0),
                                    near_attempt("no_pallet", "no_expected_candidate", sha="d9", t=30.0)]}
    tally_session(tmp_path, "O0_line_m_s100.attempt1", [], submits=[("b", "none", None)], run=run)
    out = tally(tmp_path, 1)
    r = only(out)["rates"]
    assert (r["N"], r["k"]) == (1, 1) and out["excluded_attempts"] == ["O0_line_m_s100"]


def test_a_transition_of_its_own_starts_an_unacknowledged_mission(tmp_path):
    run = {"success": False, "transitions": [{"from": "complete", "to": "observe", "time_s": 5.0}]}
    tally_session(tmp_path, "O0_line_m_s100.attempt1", [], submits=[("b", "none", None)], run=run)
    r = only(tally(tmp_path, 1))["rates"]
    assert (r["N"], r["S"], r["result_missing"]) == (1, 0, 1)
    tally_session(tmp_path, "O0_line_m_s100", [("m1", "done", None, [near_attempt(far=True), near_attempt()])],
                  submits=[("m1", "ack", "m1")])
    assert tally(tmp_path, 1)["excluded_attempts"] == ["O0_line_m_s100"]  # the first attempt that took one


def test_a_run_failure_alone_does_not_start_an_unacknowledged_mission(tmp_path):
    tally_session(tmp_path, "O0_line_m_s100", [("m1", "done", None, [near_attempt(far=True), near_attempt()])],
                  submits=[("m1", "ack", "m1"), ("b", "none", None)],
                  run={"success": False, "failure_reason": NEAR_FAIL})
    r = only(tally(tmp_path, 2))["rates"]
    assert (r["N"], r["S"], r["k"], r["result_missing"]) == (1, 1, 0, 0)


def test_a_mission_with_no_records_that_ended_in_observe_may_have_reached_near(tmp_path):
    tally_session(tmp_path, "O0_line_m_s100", [("m1", "failed", NEAR_FAIL, [near_attempt(far=True), near_attempt()])],
                  submits=[("m1", "ack", "m1")])
    tally_session(tmp_path, "O0_line_m_s101", [("m1", "failed", "observe:timeout", None)], submits=[("m1", "ack", "m1")])
    r = only(tally(tmp_path, 1))["rates"]
    assert (r["R"], r["U"]) == (1, 1) and r["conditional"]["low"] == pytest.approx(0.5)


def test_session_counts_keep_a_start_unknown_session_apart(tmp_path):
    tally_session(tmp_path, "O0_line_m_s100", [("m1", "done", None, [near_attempt(far=True), near_attempt()])],
                  submits=[("m1", "ack", "m1")])
    tally_session(tmp_path, "O0_line_m_s101", [], submits=[("b", "none", None)])
    out = tally(tmp_path, 1)
    assert only(out)["sessions"]["started"] == 1 and only(out)["sessions"]["start_unknown"] == 1
    started = sorted(i for c in out["session_cells"] for i in c["started_session_ids"])
    unknown = sorted(i for c in out["session_cells"] for i in c["start_unknown_session_ids"])
    assert started == ["O0_line_m_s100"] and unknown == ["O0_line_m_s101"]


def test_a_refused_submission_of_the_reverse_scenario_starts_nothing_and_no_samples_leave_the_face_unknown(tmp_path):
    d = tmp_path / "C3_line_m_s0"
    _write(d / "probe.json", {"reverse": {"mission": {"pallet_id": "P-1", "destination_id": "D-1",
                                                      "submit": {"t": "reject", "reason": "pose_unconfirmed"}}}})
    (d / "live" / "epoch_1" / "run").mkdir(parents=True)
    out = tally(tmp_path, 1)
    assert only(out)["rates"]["N"] == 0
    e = tmp_path / "C3_line_m_s1"
    _write(e / "probe.json", {"reverse": {"mission": {"pallet_id": "P-1", "destination_id": "D-1",
                                                      "submit": {"t": "ack", "mission_id": "m1"},
                                                      "result": {"outcome": "done"}}}})
    _write(e / "live" / "epoch_1" / "run" / "result.json", {"success": True, "arguments": ARGS})
    _write(e / "live" / "epoch_1" / "run" / "missions" / "m1" / "records.json",
           {"state": mission_state([near_attempt(far=True), near_attempt()])})
    _write(e / "live" / "epoch_1" / "run" / "missions" / "m1" / "result.json",
           {"result": {"mission_id": "m1", "outcome": "done", "started_s": 0.0, "ended_s": 1.0}})
    res = C.survey(tmp_path, 1)
    out = C.near_capture_tally(tmp_path, res["sessions"])
    started = [c for c in out["cells"] if c["missions_started"]]
    assert len(started) == 1 and started[0]["entry_face"] == "unknown"


def test_the_entry_face_comes_from_the_spooled_samples(tmp_path):
    samples = [{"time_s": 1.0, "pallet_id": "P", "pallet_position_m": [3.0, 0.0, 0.0], "pallet_yaw_rad": 0.0}]
    tally_session(tmp_path, "O0_line_m_s100", [("m1", "done", None, [near_attempt(far=True), near_attempt()])],
                  submits=[("m1", "ack", "m1")], samples=samples)
    assert only(tally(tmp_path, 1), "cells")["entry_face"] == "-x"  # the truck at the origin looks at the -x face
