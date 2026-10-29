"""B-P2d's judgement (docs/plans/2026-10-29-start-relocalization.md 4.0-4.3, 4.4b, 7): one unattended relocalization
session per start, read from its own records -- invalid sessions out of the denominator; started sessions judged by
the SLAM pose at the running tick against the same tick's truth; success with the stationary hold; the safety and
success floors and bounds; the F1/F2 records; and the verdict-path fingerprint that decides what may be pooled."""

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import reloc_judge as J  # noqa: E402

from forklift_core.live.spool import RecordSpool  # noqa: E402

TRUTH = [2.0, 1.0, 0.5]


SOURCES = {"forklift_core/localization/global_match.py": "a" * 64, "sim/isaac/run_transport.py": "b" * 64}
ARGS = {"seed": 5, "slam_map_bundle": "/bundle", "start_pose": [2.0, 1.0, 0.5], "output": "x", "reloc_dir": "y"}


def _session(root: Path, name: str, *, status="chosen", verdict_off=(0.02, 0.0, 0.0), hold=True, records=None,
             exit_code=0, reason=None, shown=None, attempts=None, gaps=None, estimate_source="matcher",
             probe_end=True, reloc_wall=(100.0, 103.0, 120.0), sources=None, arguments=None, result=True,
             manifest=True):
    d = root / name
    run = d / "epoch_1" / "run"
    run.mkdir(parents=True)
    rec = {"status": status, "reason": "moved" if status == "invalid" else "", "truth": TRUTH, "shown": shown or [],
           "attempts": attempts if attempts is not None else [
               {"reason": "confirmed", "status": "ok", "wall_s": 2.5, "stage_s": 3.0,
                "candidates": [{"base": TRUTH, "score": 0.9, "endpoint_fraction": 0.97}]}],
           "estimate": TRUTH if status == "chosen" else None, "source": estimate_source if status == "chosen" else None}
    started = status == "chosen" and verdict_off is not None
    if started:
        rec["verdict"] = {"sim_s": 1.0, "truth_base": TRUTH,
                          "slam_base": [TRUTH[0] + verdict_off[0], TRUTH[1] + verdict_off[1], TRUTH[2] + verdict_off[2]],
                          "estimate_base": TRUTH}
    record = {"success": exit_code == 0, "start_reloc": rec, "reloc_console": {"gaps": gaps or []},
              "failure_reason": reason, "source_sha256": dict(sources or SOURCES),
              "arguments": dict(arguments or ARGS)}
    if result:
        (run / "result.json").write_text(json.dumps(record))
    spool = RecordSpool(run / "spool")
    items = records if records is not None else (
        [{"scan_id": k, "stamp_s": 1.0 + 0.1 * k, "status": "processed", "mode": "tracking"} for k in range(0, 115)]
        if started else [])
    spool.register("slam.records", list(items), keep=0)
    spool.close()
    if not manifest:
        (run / "spool" / "manifest.json").unlink()
    sup = [{"wall": 99.0, "event": "start", "epoch": 1}]
    if status == "chosen":
        sup += [{"wall": reloc_wall[0], "event": "reloc_pose"}, {"wall": reloc_wall[1], "event": "bridge_go"}]
        if started:
            sup.append({"wall": reloc_wall[2], "event": "localization_ready", "epoch": 1})
    sup.append({"wall": 200.0, "event": "exit", "epoch": 1, "code": exit_code, "reason": reason})
    (d / "supervisor.jsonl").write_text("\n".join(json.dumps(e) for e in sup) + "\n")
    probe = {"hold": {"hold_sim_s": 10.0, "closed_during": None if hold else 1006}}
    if started and hold and probe_end:
        probe["session_end"] = {"t": "ack"}
    (d / "probe.json").write_text(json.dumps(probe))
    return d


def _starts(*names):
    return [{"id": n, "stratum": "A", "base_world": TRUTH} for n in names]


def test_a_matcher_start_held_in_bounds_is_a_success(tmp_path):
    _session(tmp_path, "s1")
    [row] = J.judge_sessions(tmp_path, _starts("s1"))
    assert row["class"] == "valid" and row["started"] and row["outcome"] == "success"
    assert row["error_m"] == pytest.approx(0.02) and row["hold_ok"] is True
    assert row["bridge_stage_s"] == pytest.approx(20.0) and row["matcher_wall_max_s"] == 2.5


def test_an_invalid_session_leaves_the_denominator(tmp_path):
    _session(tmp_path, "s1", status="invalid", exit_code=1, reason="reloc_invalid_input: moved")
    [row] = J.judge_sessions(tmp_path, _starts("s1"))
    assert row["class"] == "invalid" and row["outcome"] == "invalid"
    s = J.summarize([row])
    assert s["valid"] == 0 and s["invalid"] == 1


@pytest.mark.parametrize("off, outcome", [((0.15, 0.0, 0.0), "imprecise"), ((0.0, 0.0, 0.07), "imprecise"),
                                          ((0.31, 0.0, 0.0), "wrong"), ((0.0, 0.0, 0.11), "wrong"),
                                          ((0.10, 0.0, 0.05), "success")])
def test_a_started_session_is_judged_by_the_slam_pose_at_the_running_tick(tmp_path, off, outcome):
    _session(tmp_path, "s1", verdict_off=off)
    [row] = J.judge_sessions(tmp_path, _starts("s1"))
    assert row["started"] and row["pose_class"] == outcome


def test_a_hold_that_did_not_last_is_no_success(tmp_path):
    lost = [{"scan_id": k, "stamp_s": 1.0 + 0.1 * k, "status": "processed", "mode": "tracking"} for k in range(40)]
    lost.append({"scan_id": 40, "stamp_s": 5.0, "status": "link_failure", "error": "x"})
    _session(tmp_path, "s1", records=lost, exit_code=1, reason="slam_link_failed: x", hold=False)
    [row] = J.judge_sessions(tmp_path, _starts("s1"))
    assert row["started"] and row["hold_ok"] is False and row["outcome"] == "hold_failed"
    short = [{"scan_id": k, "stamp_s": 1.0 + 0.1 * k, "status": "processed", "mode": "tracking"} for k in range(50)]  # 5 s only
    _session(tmp_path, "s2", records=short)
    row2 = J.judge_sessions(tmp_path, _starts("s2"))[0]
    assert row2["hold_ok"] is False


def test_the_operators_ending_and_the_truth_among_the_candidates(tmp_path):
    shown = [{"base": [TRUTH[0] + 0.05, TRUTH[1], TRUTH[2]], "score": 0.9, "endpoint_fraction": 0.95},
             {"base": [9.0, 9.0, 0.5], "score": 0.88, "endpoint_fraction": 0.94}]
    att = [{"reason": "competitor", "status": "ok", "wall_s": 3.0, "candidates": shown}] * 3
    _session(tmp_path, "s1", status="needs_operator", shown=shown, attempts=att, exit_code=1,
             reason="reloc_needs_operator: 2 candidates, no operator (unattended)")
    far = [{"base": [9.0, 9.0, 0.5], "score": 0.88, "endpoint_fraction": 0.94}]
    _session(tmp_path, "s2", status="needs_operator", shown=far,
             attempts=[{"reason": "periodic", "status": "ok", "wall_s": 3.0, "candidates": far + shown}],
             exit_code=1, reason="reloc_needs_operator: 1 candidates, no operator (unattended)")
    r1, r2 = J.judge_sessions(tmp_path, _starts("s1", "s2"))
    assert r1["outcome"] == "needs_operator" and r1["truth_in_shown"] is True and r1["truth_in_candidates"] is True
    assert r2["truth_in_shown"] is False and r2["truth_in_candidates"] is True  # plan 7 F1


def test_a_bridge_refusal_is_valid_and_not_started(tmp_path):
    _session(tmp_path, "s1", verdict_off=None, exit_code=1,
             reason="slam_not_localized: the map pose at the start is 0.2 m")
    [row] = J.judge_sessions(tmp_path, _starts("s1"))
    assert row["class"] == "valid" and not row["started"] and row["outcome"] == "bridge_refused"


def test_a_missing_session_is_a_valid_failure_never_dropped(tmp_path):
    rows = J.judge_sessions(tmp_path, _starts("absent"))
    assert rows[0]["class"] == "valid" and rows[0]["outcome"] == "no_record"


def test_the_summary_holds_the_registered_floors_and_bounds():
    rows = [{"class": "valid", "started": True, "outcome": "success", "pose_class": "success",
             "matcher_wall_max_s": 3.0, "matcher_stage_max_s": 3.5, "deadline_hit": False, "bridge_stage_s": 20.0, "gaps": []}] * 42
    rows += [{"class": "valid", "started": False, "outcome": "needs_operator", "pose_class": None,
              "matcher_wall_max_s": 9.0, "matcher_stage_max_s": 28.0, "deadline_hit": False, "bridge_stage_s": None, "gaps": [],
              "truth_in_shown": False, "truth_in_candidates": True}] * 18
    s = J.summarize(rows, bridge_bound_s=60.0)
    assert s["valid"] == 60 and s["started"] == 42 and s["wrong"] == 0 and s["imprecise"] == 0
    assert s["safety_upper_valid"] == pytest.approx(1 - 0.05 ** (1 / 60), abs=1e-6)  # 4.87 %
    assert s["safety_upper_started"] == pytest.approx(1 - 0.05 ** (1 / 42), abs=1e-6)  # 6.88 %
    assert s["success_rate"] == pytest.approx(0.70) and s["success_cp_lower"] == pytest.approx(0.588, abs=0.001)
    assert s["floors"] == {"safety": True, "success": True, "matcher_time": True, "bridge_time": True}
    assert s["f1"] == {"requests": 18, "truth_in_shown": 0, "truth_in_candidates": 18}


def test_too_few_valid_sessions_are_not_judged():
    rows = [{"class": "valid", "started": True, "outcome": "success", "pose_class": "success",
             "matcher_wall_max_s": 3.0, "matcher_stage_max_s": 3.5, "deadline_hit": False, "bridge_stage_s": 20.0, "gaps": []}] * 59
    s = J.summarize(rows, bridge_bound_s=60.0)
    assert s["floors"]["safety"] is None and s["judged"] is False  # plan 4.4: below 60 valid, reported, not judged


def test_the_fingerprint_covers_the_verdict_path_and_refuses_a_mismatch(tmp_path):
    fp = J.fingerprint(ROOT, image="sha256:abc")
    assert set(J.FINGERPRINT_FILES) <= set(fp["files"]) and fp["image"] == "sha256:abc"
    assert "src/forklift_core/localization/global_match.py" in fp["files"]
    assert "tools/reloc_judge.py" in fp["files"]
    other = json.loads(json.dumps(fp))
    other["files"]["sim/isaac/run_transport.py"] = "0" * 64
    assert J.fingerprint_mismatches(fp, other) == ["sim/isaac/run_transport.py"]
    assert J.fingerprint_mismatches(fp, fp) == []


# --- Codex B-P2d tooling review round 1 -------------------------------------------------------------------------------
def _good_row(**over):
    row = {"class": "valid", "started": True, "outcome": "success", "pose_class": "success", "matcher_wall_max_s": 3.0,
           "matcher_stage_max_s": 3.0, "deadline_hit": False, "bridge_stage_s": 20.0, "gaps": [],
           "source_mismatches": [], "args_signature": "s"}
    row.update(over)
    return row


def test_a_start_whose_record_is_lost_is_started_and_leaves_the_safety_unjudged(tmp_path):
    # P2 1: localization_ready in the supervisor, then a crash with no result.json -- never "no start"
    _session(tmp_path, "s1", result=False, exit_code=139, reason="signal 11")
    [row] = J.judge_sessions(tmp_path, _starts("s1"))
    assert row["class"] == "valid" and row["started"] and row["outcome"] == "started_unrecorded"
    s = J.summarize([_good_row()] * 59 + [row], bridge_bound_s=60.0)
    assert s["unjudged_starts"] == 1 and s["floors"]["safety"] is None


def test_the_hold_needs_a_complete_spool_continuous_tracking_replies(tmp_path):
    # P2 2: an incomplete spool, a gap, or a mode other than tracking is no shown hold
    _session(tmp_path, "s1", manifest=False)
    holding = [{"scan_id": k, "stamp_s": 1.0 + 0.1 * k, "status": "processed", "mode": "holding"} for k in range(115)]
    _session(tmp_path, "s2", records=holding)
    gappy = [{"scan_id": k, "stamp_s": 1.0 + 0.1 * k, "status": "processed", "mode": "tracking"}
             for k in list(range(10)) + list(range(100, 115))]
    _session(tmp_path, "s3", records=gappy)
    rows = J.judge_sessions(tmp_path, _starts("s1", "s2", "s3"))
    assert all(r["hold_ok"] is False and r["outcome"] == "hold_failed" for r in rows)


def test_a_started_session_without_its_bridge_times_leaves_the_bridge_floor_unjudged():
    # P2 3: a missing time is no pass
    s = J.summarize([_good_row()] * 59 + [_good_row(bridge_stage_s=None)], bridge_bound_s=60.0)
    assert s["floors"]["bridge_time"] is None and s["bridge_stage_missing"] == 1


def test_the_matcher_time_is_the_stage_from_the_first_scan_and_a_late_refusal_is_a_deadline_hit(tmp_path):
    # P2 5: two attempts of 15 s and 18 s return 33 s after the first scan
    att = [{"reason": "competitor", "status": "ok", "wall_s": 15.0, "stage_s": 16.0, "candidates": []},
           {"reason": "deadline", "status": "ok", "wall_s": 18.0, "stage_s": 33.0, "candidates": []}]
    _session(tmp_path, "s1", status="failed", attempts=att, exit_code=1, reason="reloc_failed: none")
    [row] = J.judge_sessions(tmp_path, _starts("s1"))
    assert row["matcher_stage_max_s"] == 33.0 and row["deadline_hit"] is True
    s = J.summarize([_good_row()] * 59 + [row], bridge_bound_s=60.0)
    assert s["worker_over_32s"] == 1 and s["floors"]["matcher_time"] is False


def test_the_safety_bound_counts_the_wrong_and_imprecise_starts():
    # P2 6: a 0/n bound only when there is no event
    from scipy.stats import beta

    rows = [_good_row()] * 59 + [_good_row(outcome="wrong", pose_class="wrong")]
    s = J.summarize(rows, bridge_bound_s=60.0)
    assert s["safety_upper_valid"] == pytest.approx(float(beta.ppf(0.95, 2, 59)), abs=1e-9)
    assert s["safety_upper_valid"] > 0.0487 and s["floors"]["safety"] is False


def test_an_invalid_input_before_the_record_is_invalid_and_a_set_up_error_is_not(tmp_path):
    # P2 7: classified by the reason, not by whether the record was made; a configuration error stays valid
    _session(tmp_path, "s1", result=False, exit_code=1, reason="reloc_invalid_input: not_at_target", status="invalid")
    _session(tmp_path, "s2", result=False, exit_code=1, reason="reloc_config_error: the LiDAR mount", status="failed")
    r1, r2 = J.judge_sessions(tmp_path, _starts("s1", "s2"))
    assert r1["class"] == "invalid"
    assert r2["class"] == "valid" and r2["outcome"] == "config_error"


def test_each_session_is_held_to_the_fingerprint_and_one_argument_set(tmp_path):
    # P2 4: the runner's own source hashes and the arguments it ran with, per session
    fp = {"files": {"src/forklift_core/localization/global_match.py": "a" * 64,
                    "sim/isaac/run_transport.py": "b" * 64}, "image": "i"}
    _session(tmp_path, "s1")
    _session(tmp_path, "s2", sources={**SOURCES, "sim/isaac/run_transport.py": "c" * 64})
    _session(tmp_path, "s3", arguments={**ARGS, "slam_noise_seed": 7})
    _session(tmp_path, "s4", arguments={**ARGS, "start_pose": [9.0, 9.0, 0.0], "output": "z", "reloc_dir": "w"})
    rows = J.judge_sessions(tmp_path, _starts("s1", "s2", "s3", "s4"), fp)
    assert rows[0]["source_mismatches"] == [] and rows[1]["source_mismatches"] == ["sim/isaac/run_transport.py"]
    assert rows[0]["args_signature"] == rows[3]["args_signature"] != rows[2]["args_signature"]  # start/output aside
    s = J.summarize(rows, bridge_bound_s=60.0)
    assert s["sessions_off_fingerprint"] == 1 and s["argument_sets"] == 2


# --- Codex B-P2d tooling review round 2 -------------------------------------------------------------------------------
def test_the_warm_up_before_the_running_tick_is_not_part_of_the_hold(tmp_path):
    # P2 1: warming_up replies before running are the normal bootstrap
    recs = [{"scan_id": k, "stamp_s": 0.1 * k, "status": "processed", "mode": "warming_up"} for k in range(1, 7)]
    recs += [{"scan_id": k, "stamp_s": 0.1 * k, "status": "processed", "mode": "tracking"} for k in range(7, 115)]
    _session(tmp_path, "s1", records=recs)
    [row] = J.judge_sessions(tmp_path, _starts("s1"))
    assert row["hold_ok"] is True and row["outcome"] == "success"


def test_any_attempt_past_the_shared_deadline_is_a_deadline_hit(tmp_path):
    # P2 2: an unconfirmed result handled after the deadline is a deadline hit too
    att = [{"reason": "competitor", "status": "ok", "wall_s": 20.0, "stage_s": 31.0, "candidates": []}]
    _session(tmp_path, "s1", status="needs_operator", attempts=att, exit_code=1, reason="reloc_needs_operator: 0")
    [row] = J.judge_sessions(tmp_path, _starts("s1"))
    assert row["deadline_hit"] is True


def test_an_unmeasured_matcher_time_leaves_the_time_floor_unjudged(tmp_path):
    # P2 3: a session that died with no record cannot show its matcher kept the time
    s = J.summarize([_good_row()] * 59 + [_good_row(started=False, outcome="no_record", pose_class=None,
                                                    matcher_stage_max_s=None, bridge_stage_s=None)],
                    bridge_bound_s=60.0)
    assert s["matcher_time_unmeasured"] == 1 and s["floors"]["matcher_time"] is None


def test_the_world_is_not_part_of_the_argument_set(tmp_path):
    # P2 4: plan 4.4a pools worlds (seed, scene, map) under one measurement condition
    _session(tmp_path, "s1")
    _session(tmp_path, "s2", arguments={**ARGS, "seed": 3, "slam_map_bundle": "/other", "expected_map_id": "f" * 64})
    r1, r2 = J.judge_sessions(tmp_path, _starts("s1", "s2"))
    assert r1["args_signature"] == r2["args_signature"] and r1["world"] != r2["world"]


def test_the_fingerprint_takes_the_whole_core_and_runner_trees():
    # P2 5: a transitive dependency (e.g. _validation.py) changes the scan without any listed file changing
    fp = J.fingerprint(ROOT, image="i")
    assert "src/forklift_core/_validation.py" in fp["files"] and "sim/isaac/obstacle_layer.py" in fp["files"]
    assert len([k for k in fp["files"] if k.startswith("src/forklift_core/")]) > 50


def test_missing_evidence_is_not_a_match(tmp_path):
    # P2 6: no source hashes, no arguments or a frozen file that is not there never pass as pooled
    fp = {"files": {"sim/isaac/run_transport.py": "b" * 64}, "image": "i"}
    _session(tmp_path, "s1", sources={"unrelated.py": "x"})
    rows = J.judge_sessions(tmp_path, _starts("s1"), fp)
    assert rows[0]["source_mismatches"] == ["sim/isaac/run_transport.py (not recorded)"]
    run = tmp_path / "s2" / "epoch_1" / "run"
    _session(tmp_path, "s2")
    data = json.loads((run / "result.json").read_text())
    data.pop("arguments")
    (run / "result.json").write_text(json.dumps(data))
    [r2] = J.judge_sessions(tmp_path, _starts("s2"), fp)
    assert r2["args_signature"] is None and J.summarize([r2])["arguments_missing"] == 1
    starts = tmp_path / "starts.json"
    starts.write_text(json.dumps({"poses": _starts("s1")}))
    rc = J.main(["--sessions", str(tmp_path), "--starts", str(starts), "--code-root", str(ROOT), "--image", "i",
                 "--frozen", str(tmp_path / "missing.json"), "--out", str(tmp_path / "out.json")])
    assert rc == 2


def test_an_unjudged_start_leaves_the_safety_bounds_unjudged_and_gives_a_conservative_one():
    # P2 7: an unrecorded start might have been wrong
    rows = [_good_row()] * 59 + [_good_row(outcome="started_unrecorded", pose_class=None)]
    s = J.summarize(rows, bridge_bound_s=60.0)
    assert s["safety_upper_valid"] is None and s["safety_upper_started"] is None
    assert s["safety_upper_valid_conservative"] == pytest.approx(J.cp_upper(1, 60))


def test_a_judgement_needs_the_frozen_record_which_has_its_own_mode(tmp_path):
    # Codex round 3 P2: no frozen record, no pooled judgement; the freeze record is written by an explicit mode
    _session(tmp_path, "s1")
    starts = tmp_path / "starts.json"
    starts.write_text(json.dumps({"poses": _starts("s1")}))
    base = ["--sessions", str(tmp_path), "--starts", str(starts), "--code-root", str(ROOT), "--image", "i",
            "--out", str(tmp_path / "out.json")]
    assert J.main(base) == 2  # no --frozen: judged, never pooled
    frozen = tmp_path / "frozen.json"
    assert J.main(["--make-frozen", str(frozen), "--code-root", str(ROOT), "--image", "i",
                   "--bridge-bound-s", "45"]) == 0
    data = json.loads(frozen.read_text())
    assert data["bridge_bound_s"] == 45.0 and data["fingerprint"] == J.fingerprint(ROOT, "i")
    assert not (tmp_path / "out.json").exists() or json.loads((tmp_path / "out.json").read_text())


def test_the_fingerprint_covers_the_scan_odometry_and_runner_inputs():
    for rel in ("sim/isaac/obstacle_layer.py", "sim/isaac/planar_lidar.py", "src/forklift_core/sensors/lidar.py",
                "src/forklift_core/localization/wheel_odometry.py", "src/forklift_core/localization/gyro.py",
                "src/forklift_core/mission/core.py", "config/obstacle_layer_single_team_spec.yaml",
                "config/isaac_transport_measured.yaml"):
        assert rel in J.FINGERPRINT_FILES and (ROOT / rel).is_file(), rel
