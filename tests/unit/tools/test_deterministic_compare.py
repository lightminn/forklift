"""Deterministic-run plan D8: the strict comparison of two runs of one deterministic configuration. Everything the
run recorded is compared -- sensor and proposal arrays bit for bit, JSON by value -- except an explicit list of wall,
identity and diagnostic fields; spools are compared as their joined logical streams; any deterministic_broken reason
fails the comparison, even the same one on both runs."""

import json
from pathlib import Path

import numpy as np

from tools import deterministic_compare as dc


def make_run(root: Path, *, frame_bit=False, broken=None, wall=1.0, path_tag="a", spool_cut=2, cycle_extra=0,
             event_tick=0.6, renders=3) -> Path:
    run = root / path_tag / "run"
    (run / "recognition").mkdir(parents=True)
    (run / "spool" / "slam.control").mkdir(parents=True)
    (run / "spool" / "manifest.json").write_text(json.dumps({"complete": True, "cuts": 6 // spool_cut}))
    result = {
        "phase": "done", "planning_wall_s": wall, "output": str(run / "x.npz"),
        "async_plans": [{"site": "observe", "requested_s": 3.3, "finished_s": 3.35, "wall_s": wall,
                         "queued_mono": 1000.0 + wall}],
        "deterministic": {"enabled": True, "broken": list(broken or []),
                          "diagnostics": {"frame_sync": {"renders": renders}}},
    }
    (run / "result.json").write_text(json.dumps(result))
    rgb = np.zeros((4, 4, 3), np.uint8)
    depth = np.ones((4, 4), np.float32)
    if frame_bit:  # a depth bit: RGB members are left out of the comparison (plan D1b)
        depth.view(np.uint32)[1, 2] ^= 1
    np.savez(run / "recognition" / "frame_0003.npz", rgb=rgb, depth_m=depth)
    rows = [[0.01 * k, 1.0, 2.0] for k in range(6)]
    for i in range(0, len(rows), spool_cut):
        with open(run / "spool" / "slam.control" / f"{i:06d}.jsonl", "w") as f:
            for r in rows[i:i + spool_cut]:
                f.write(json.dumps(r) + "\n")
    with open(run / "live_log.jsonl", "w") as f:
        f.write(json.dumps({"event": "start", "mono": 5.0 + wall}) + "\n")
        for k in range(3 + cycle_extra):
            f.write(json.dumps({"mono": 6.0 + k, "where": "cycle", "t": "echo"}) + "\n")
        f.write(json.dumps({"t": 9.0 + wall, "event": "mode", "from": "PAUSED", "to": "AUTO", "tick_t": event_tick,
                            "mono": 9.0 + wall}) + "\n")
    return run.parent


def test_two_identical_runs_are_equal_with_per_stream_hashes(tmp_path):
    a, b = make_run(tmp_path, path_tag="a"), make_run(tmp_path, path_tag="b")
    out = dc.compare(a, b)
    assert out["equal"] is True and out["differences"] == []
    assert out["hashes"]["A"] == out["hashes"]["B"]
    assert "run/spool/slam.control" in out["hashes"]["A"]
    assert "run/recognition/frame_0003.npz" in out["hashes"]["A"]


def test_one_bit_in_a_frame_array_fails_with_the_summaries_unchanged(tmp_path):
    out = dc.compare(make_run(tmp_path, path_tag="a"), make_run(tmp_path, path_tag="b", frame_bit=True))
    assert out["equal"] is False
    assert any("frame_0003.npz" in d and "depth_m" in d for d in out["differences"])


def test_the_same_broken_reason_on_both_runs_still_fails(tmp_path):
    reason = ["event_during_wait:lease_expired"]
    out = dc.compare(make_run(tmp_path, path_tag="a", broken=reason), make_run(tmp_path, path_tag="b", broken=reason))
    assert out["differences"] == []
    assert out["equal"] is False and out["broken"] == {"A": reason, "B": reason}


def test_wall_identity_path_and_diagnostic_fields_are_left_out(tmp_path):
    out = dc.compare(make_run(tmp_path, path_tag="a", wall=1.0, renders=2),
                     make_run(tmp_path, path_tag="b", wall=7.5, renders=4))
    assert out["equal"] is True, out["differences"]


def test_the_learned_call_times_in_the_live_job_log_are_wall_times():
    # learned VRAM plan 3.1 (console review): the job log's model_ms and infer_wall_s differ run to run
    assert dc.WALL_KEY.search("model_ms") and dc.WALL_KEY.search("infer_wall_s") and dc.WALL_KEY.search("request_mono")
    assert not dc.WALL_KEY.search("model_source")


def test_spools_compare_as_their_joined_rows_whatever_the_cut(tmp_path):
    out = dc.compare(make_run(tmp_path, path_tag="a", spool_cut=2), make_run(tmp_path, path_tag="b", spool_cut=5))
    assert out["equal"] is True, out["differences"]


def test_console_traffic_is_left_out_but_an_events_tick_is_compared(tmp_path):
    assert dc.compare(make_run(tmp_path, path_tag="a"), make_run(tmp_path, path_tag="b", cycle_extra=4))["equal"]
    out = dc.compare(make_run(tmp_path, path_tag="c"), make_run(tmp_path, path_tag="d", event_tick=0.625))
    assert out["equal"] is False and any("live_log" in d for d in out["differences"])


def test_rgb_members_are_left_out_and_other_members_still_compared(tmp_path):
    # plan D1b: RTX RGB is not reproducible across processes and is off the decision path in a deterministic run
    # (the oracle masks replace the learned proposer); depth and segmentation in the same file still count
    a, b = make_run(tmp_path, path_tag="a"), make_run(tmp_path, path_tag="b")
    rng = np.random.default_rng(0)
    rgb = rng.integers(0, 255, (4, 4, 3), dtype=np.uint8)
    depth = rng.random((4, 4)).astype(np.float32)
    for run in (a, b):
        (run / "recognition").mkdir(parents=True, exist_ok=True)
    np.savez(a / "recognition" / "frame_x.npz", rgb=rgb, depth=depth)
    np.savez(b / "recognition" / "frame_x.npz", rgb=rgb ^ 1, depth=depth)
    assert dc.compare(a, b)["equal"]
    np.savez(b / "recognition" / "frame_x.npz", rgb=rgb ^ 1, depth=depth + 1e-6)
    assert not dc.compare(a, b)["equal"]


def test_a_file_only_in_one_run_fails(tmp_path):
    a, b = make_run(tmp_path, path_tag="a"), make_run(tmp_path, path_tag="b")
    np.savez(b / "run" / "recognition" / "frame_0004.npz", rgb=np.zeros((2, 2, 3), np.uint8))
    out = dc.compare(a, b)
    assert out["equal"] is False and any("frame_0004.npz" in d for d in out["differences"])


def test_a_sim_time_value_change_fails(tmp_path):
    a, b = make_run(tmp_path, path_tag="a"), make_run(tmp_path, path_tag="b")
    r = json.loads((b / "run" / "result.json").read_text())
    r["async_plans"][0]["finished_s"] = 3.3583
    (b / "run" / "result.json").write_text(json.dumps(r))
    out = dc.compare(a, b)
    assert out["equal"] is False and any("finished_s" in d for d in out["differences"])


def test_paths_beside_a_run_folder_are_the_runs_own(tmp_path):
    # a bundle slot is <slot>/run with sock/ and bridge/ beside it, and result.json's arguments name them (dv3 V1)
    runs = []
    for tag in ("a", "b"):
        run = make_run(tmp_path, path_tag=tag) / "run"
        result = json.loads((run / "result.json").read_text())
        result["arguments"] = {"slam_feedback": str(run.parent / "sock" / "slam.sock")}
        (run / "result.json").write_text(json.dumps(result))
        runs.append(run)
    out = dc.compare(*runs)
    assert out["equal"] is True, out["differences"]


def test_render_and_object_identities_are_left_out_but_the_same_tracker_still_compares(tmp_path):
    # dv3 V1: the Kit render frame number (the sync takes a varying number of renders), the scene-replan tracker
    # serial (id() % 10**6) and the cusp hold key's id(tracker) differed between deterministic runs, nothing else
    def write(tag, frame, serial, ids):
        run = make_run(tmp_path, path_tag=tag) / "run"
        result = json.loads((run / "result.json").read_text())
        result["perception"] = {"capture_diagnostics": {"sensor_frame_id": {"referenceTimeNumerator": frame},
                                                        "depth_valid": 0.9}}
        result["scene_replans"] = [{"tracker_serial": serial, "leg": 2}]
        result["cusp_heading_holds"] = [{"key": ["approach", i, 1], "cancelled_by": None} for i in ids]
        (run / "result.json").write_text(json.dumps(result))
        return run

    a = write("a", 892, 27408, [1111, 2222, 1111])
    b = write("b", 1124, 76368, [9999, 5555, 9999])
    assert dc.compare(a, b)["equal"] is True, dc.compare(a, b)["differences"]
    c = write("c", 1124, 76368, [9999, 5555, 5555])  # the third hold on another tracker: a real difference
    assert dc.compare(a, c)["equal"] is False


def test_one_tracker_table_across_the_closed_holds_the_open_hold_and_the_watch(tmp_path):
    # Codex PR 1 review P3: the open hold on the same tracker as a closed one, and the watch's raw id(tracker)
    def write(tag, closed, open_, watch):
        run = make_run(tmp_path, path_tag=tag) / "run"
        result = json.loads((run / "result.json").read_text())
        result["cusp_heading_holds"] = [{"key": ["approach", closed, 1], "cancelled_by": None}]
        result["cusp_heading_hold"] = {"key": ["approach", open_, 2], "cancelled_by": None}
        result["cusp_heading_watch"] = {"tracker": watch, "since_s": 3.0}
        (run / "result.json").write_text(json.dumps(result))
        return run

    a = write("a", 111, 111, 111)
    assert dc.compare(a, write("b", 999, 999, 999))["equal"] is True
    assert dc.compare(a, write("c", 999, 555, 999))["equal"] is False  # the open hold on another tracker
    assert dc.compare(a, write("d", 999, 999, 555))["equal"] is False  # the watch on another tracker


def test_a_stamp_named_decided_is_left_out_but_a_decided_record_is_compared(tmp_path):
    # Codex PR 1 review P2: "submitted"/"decided"/"started" are wall stamps (floats) in the far worker and the live
    # runtime, but "decided" is also the recognition's candidate record and the cusp hold's motion state
    def write(tag, stamp, hypothesis):
        run = make_run(tmp_path, path_tag=tag) / "run"
        result = json.loads((run / "result.json").read_text())
        result["far_jobs"] = [{"seq": 1, "submitted": stamp, "decided": stamp + 0.2}]
        result["recognition"] = {"decided": {"candidate_id": 3, "hypothesis": hypothesis}}
        (run / "result.json").write_text(json.dumps(result))
        return run

    a = write("a", 1000.0, "epal6")
    assert dc.compare(a, write("b", 2345.6, "epal6"))["equal"] is True
    assert dc.compare(a, write("c", 1000.0, "t11"))["equal"] is False


def test_runs_that_are_not_valid_deterministic_records_never_compare_equal(tmp_path):
    # Codex PR 1 review P2: two missing folders, two results without the deterministic block, two incomplete spools
    # and two runs with record errors were all EQUAL
    out = dc.compare(tmp_path / "nope_a", tmp_path / "nope_b")
    assert out["equal"] is False and out["invalid"]["A"] and out["invalid"]["B"]

    def edit(tag, change):
        slot = make_run(tmp_path, path_tag=tag)
        result = json.loads((slot / "run" / "result.json").read_text())
        change(slot, result)
        (slot / "run" / "result.json").write_text(json.dumps(result))
        return slot

    plain = [edit(t, lambda s, r: r.pop("deterministic")) for t in ("p1", "p2")]
    out = dc.compare(*plain)
    assert out["equal"] is False and any("deterministic" in p for p in out["invalid"]["A"])

    def incomplete(slot, result):
        (slot / "run" / "spool" / "manifest.json").write_text(json.dumps({"complete": False}))

    out = dc.compare(*[edit(t, incomplete) for t in ("i1", "i2")])
    assert out["equal"] is False and any("spool" in p for p in out["invalid"]["A"])

    def errors(slot, result):
        result["far"] = {"live": {"record_errors": ["write failed"]}}

    out = dc.compare(*[edit(t, errors) for t in ("e1", "e2")])
    assert out["equal"] is False and any("record_errors" in p for p in out["invalid"]["A"])


def test_the_spool_manifest_is_checked_not_compared(tmp_path):
    # its chunk counts follow the wall-clock cut interval; the streams themselves are compared joined
    slots = []
    for tag, cuts in (("a", 3), ("b", 5)):
        slot = make_run(tmp_path, path_tag=tag)
        (slot / "run" / "spool" / "manifest.json").write_text(json.dumps({"complete": True, "cuts": cuts}))
        slots.append(slot)
    assert dc.compare(*slots)["equal"] is True


def test_a_different_value_beside_the_run_folder_still_fails(tmp_path):
    runs = []
    for tag, name in (("a", "slam.sock"), ("b", "other.sock")):
        run = make_run(tmp_path, path_tag=tag) / "run"
        result = json.loads((run / "result.json").read_text())
        result["arguments"] = {"slam_feedback": str(run.parent / "sock" / name)}
        (run / "result.json").write_text(json.dumps(result))
        runs.append(run)
    assert dc.compare(*runs)["equal"] is False
