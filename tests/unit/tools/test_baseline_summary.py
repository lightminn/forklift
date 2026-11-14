"""Post-fix baseline per-run fields and aggregates (docs/validation/2026-11-15-post-fix-baseline/baseline_summary.py,
plan docs/plans/2026-11-15-post-fix-baseline.md 3.3 and 4) on synthetic riib run folders."""

import copy
import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
_spec = importlib.util.spec_from_file_location(
    "baseline_summary", ROOT / "docs/validation/2026-11-15-post-fix-baseline/baseline_summary.py")
bs = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bs)

TRANSITIONS = [{"from": "observe", "to": "approach", "time_s": 40.0},
               {"from": "approach", "to": "insert", "time_s": 47.0},
               {"from": "insert", "to": "lift", "time_s": 55.0},
               {"from": "lift", "to": "extract", "time_s": 60.0},
               {"from": "extract", "to": "transport", "time_s": 70.0},
               {"from": "transport", "to": "lower", "time_s": 150.0},
               {"from": "lower", "to": "withdraw", "time_s": 160.0},
               {"from": "withdraw", "to": "return_home", "time_s": 170.0},
               {"from": "return_home", "to": "complete", "time_s": 250.0}]

ARGS = {"near_end_stand": "on", "near_observe_arrival": "0.05,0.05", "layer_latency_budget": "config/layer_latency_budget.yaml",
        "return_cusp_replans": 2, "deterministic": True, "encoder_model": "team_spec_quantized", "path_display": "overlay"}


def attempt(rec="selected", pocket="valid", **extra):
    return {"recognition": {"status": rec} if rec is not None else None,
            "pocket_observation": {"status": pocket} if pocket is not None else None, **extra}


def done_result(**over):
    r = {"success": True, "phase": "complete", "failure_reason": None, "arguments": dict(ARGS),
         "observation_attempts": [attempt(), attempt(rec=None)], "observe_replans": [],
         "near_capture": {"status": "done", "far_attempt": 1, "near_attempt": 2},
         "transitions": copy.deepcopy(TRANSITIONS), "tracking_failure": None,
         "cusp_replans": [{"time_s": 100.0, "status": "success"}, {"time_s": 200.0, "status": "success"}],
         "delivery_error_m": 0.01, "simulated_time_s": 250.0, "forbidden_pocket_contacts": [],
         "deterministic": {"enabled": True, "broken": [], "overrides": {}}}
    r.update(over)
    return r


def write_run(root: Path, name: str, result: dict, header_mode="overlay", stats=None, defaults="MODE=slam\n"):
    d = root / name
    (d / "run").mkdir(parents=True)
    (d / "run" / "result.json").write_text(json.dumps(result))
    if defaults is not None:
        (d / "riib_defaults.txt").write_text(defaults)
    if header_mode is not None:
        (d / "run" / "path_display.jsonl").write_text(json.dumps({"t": "header", "mode": header_mode}) + "\n")
    if stats is not None:
        (d / "run" / "path_display.json").write_text(json.dumps(stats))
    return d


def test_name_parse():
    assert bs.parse_name("pfb_R_random_1_s12") == ("R", "random_1", 12)
    assert bs.parse_name("lcr_B0_aisle_rack_s3") == ("B0", "aisle_rack", 3)
    assert bs.parse_name("k1c_K1_line_2_s0") == ("K1", "line_2", 0)
    with pytest.raises(ValueError):
        bs.parse_name("nonsense")


def test_done_run_fields(tmp_path):
    d = write_run(tmp_path, "pfb_R_random_1_s0", done_result(), stats={"compose_failed": 0, "pixel_lag_s": 0.0})
    f = bs.run_fields(d, mode="overlay")
    assert f["refused"] is None
    assert (f["scene"], f["seed"], f["success"]) == ("random_1", 0, True)
    assert f["far_selected"] is True and f["far_pocket_valid"] is True and f["far_capture_failure"] is None
    assert f["observe_replans"] == 0
    assert f["near_status"] == "done"
    assert f["transport_reached"] is True
    assert f["transport_tracking_failure"] is None
    assert f["transport_cusp_replans"] == 1 and f["return_cusp_replans"] == 1
    assert f["delivery_error_m"] == pytest.approx(0.01)
    assert f["simulated_time_s"] == pytest.approx(250.0)
    assert f["failure_class"] is None
    assert f["path_display_faults"] == {"compose_failed": 0}


def test_transport_failure_fields(tmp_path):
    r = done_result(success=False, phase="transport", failure_reason="Tracking failed in transport: pos=2.99,yaw=1.01",
                    tracking_failure={"phase": "transport", "reason": "failed",
                                      "tracking": {"failure": "cross_track", "status": "failed"}},
                    transitions=TRANSITIONS[:5], delivery_error_m=None, simulated_time_s=None,
                    cusp_replans=[{"time_s": 100.0, "status": "success"}])
    d = write_run(tmp_path, "pfb_R_random_1_s1", r)
    f = bs.run_fields(d, mode="overlay")
    assert f["success"] is False
    assert f["transport_tracking_failure"] == "cross_track"
    assert f["transport_cusp_replans"] == 1 and f["return_cusp_replans"] == 0
    assert f["failure_class"] == "tracking" and f["failure_stage"] == "transport"
    assert f["simulated_time_s"] is None and f["delivery_error_m"] is None


def test_observe_timeout_is_not_a_transport_failure(tmp_path):
    r = done_result(success=False, phase="observe", failure_reason="Tracking timeout in observe",
                    tracking_failure={"phase": "observe", "reason": "timeout", "tracking": {"failure": "timeout"}},
                    transitions=[], near_capture={"status": "pending", "far_attempt": 1, "near_attempt": None},
                    cusp_replans=[], delivery_error_m=None, simulated_time_s=None)
    f = bs.run_fields(write_run(tmp_path, "pfb_R_random_1_s2", r), mode="overlay")
    assert f["transport_reached"] is False and f["transport_tracking_failure"] is None
    assert f["near_status"] == "pending"
    assert f["failure_stage"] == "observe"


def test_mode_overlay_requires_both_records(tmp_path):
    ok = bs.run_fields(write_run(tmp_path, "a_R_line_2_s0", done_result()), mode="overlay")
    assert ok["refused"] is None
    no_arg = done_result(arguments={**ARGS, "path_display": None})
    assert "path_display" in bs.run_fields(write_run(tmp_path, "b_R_line_2_s1", no_arg), mode="overlay")["refused"]
    assert "header" in bs.run_fields(write_run(tmp_path, "c_R_line_2_s2", done_result(), header_mode=None),
                                     mode="overlay")["refused"]
    wrong = bs.run_fields(write_run(tmp_path, "d_R_line_2_s3", done_result(), header_mode="off"), mode="overlay")
    assert wrong["refused"]
    off = bs.run_fields(write_run(tmp_path, "e_R_line_2_s4", done_result(arguments={**ARGS, "path_display": "off"}),
                                  header_mode="off"), mode="overlay")
    assert off["refused"]


def test_mode_stage_and_prefix(tmp_path):
    stage = done_result(arguments={k: v for k, v in ARGS.items() if k != "path_display"})
    assert bs.run_fields(write_run(tmp_path, "s_S_aisle_rack_s0", stage, header_mode="stage"), mode="stage")["refused"] is None
    # a stage run is never an overlay run, and an overlay run never a stage run
    assert bs.run_fields(write_run(tmp_path, "s_S_aisle_rack_s1", stage, header_mode="stage"), mode="overlay")["refused"]
    assert bs.run_fields(write_run(tmp_path, "s_S_aisle_rack_s2", done_result()), mode="stage")["refused"]
    # pre-fix comparator: no header file and no recorded value
    pre = bs.run_fields(write_run(tmp_path, "lcr_B0_aisle_rack_s0", stage, header_mode=None), mode="prefix")
    assert pre["refused"] is None
    assert bs.run_fields(write_run(tmp_path, "lcr_B0_aisle_rack_s1", done_result()), mode="prefix")["refused"]


def test_missing_fields_are_none_not_defaults(tmp_path):
    r = done_result()
    for k in ("observation_attempts", "near_capture", "transitions", "cusp_replans", "delivery_error_m"):
        r.pop(k)
    f = bs.run_fields(write_run(tmp_path, "pfb_R_random_1_s5", r), mode="overlay")
    assert f["far_selected"] is None and f["near_status"] is None
    assert f["transport_reached"] is None and f["transport_cusp_replans"] is None
    assert f["delivery_error_m"] is None
    assert set(f["missing"]) >= {"observation_attempts", "near_capture", "transitions", "cusp_replans"}


def test_wrong_shape_is_missing(tmp_path):
    r = done_result(observation_attempts="garbage", near_capture=[1, 2])
    f = bs.run_fields(write_run(tmp_path, "pfb_R_random_1_s6", r), mode="overlay")
    assert f["far_selected"] is None and f["near_status"] is None
    assert {"observation_attempts", "near_capture"} <= set(f["missing"])


def test_applied_values_recorded(tmp_path):
    f = bs.run_fields(write_run(tmp_path, "pfb_R_random_1_s7", done_result(),
                                defaults="MODE=slam NEAR=on\nNEAR_END_STAND=on NEAR_END_STAND_FROM=default\n"), mode="overlay")
    assert f["applied"]["near_observe_arrival"] == "0.05,0.05"
    assert f["applied"]["layer_latency_budget"] == "config/layer_latency_budget.yaml"
    assert "NEAR_END_STAND=on" in f["riib_defaults"]


def test_aggregate_by_scene(tmp_path):
    rows = []
    for s in range(3):
        rows.append(bs.run_fields(write_run(tmp_path, f"pfb_R_random_1_s{s}", done_result(delivery_error_m=0.01 * (s + 1),
                                                                                         simulated_time_s=200.0 + s)),
                                  mode="overlay"))
    fail = done_result(success=False, phase="transport", failure_reason="Tracking failed in transport: pos=1,yaw=0",
                       tracking_failure={"phase": "transport", "reason": "failed",
                                         "tracking": {"failure": "endpoint_heading"}},
                       transitions=TRANSITIONS[:5], delivery_error_m=None, simulated_time_s=None, cusp_replans=[])
    rows.append(bs.run_fields(write_run(tmp_path, "pfb_R_aisle_rack_s0", fail), mode="overlay"))
    agg = bs.aggregate(rows)
    r1 = agg["scenes"]["random_1"]
    assert r1["n"] == 3 and r1["success"] == [3, 3]
    assert r1["delivery_error_m"]["median"] == pytest.approx(0.02)
    assert r1["delivery_error_m"]["max"] == pytest.approx(0.03)
    assert r1["simulated_time_s"]["median"] == pytest.approx(201.0)
    ar = agg["scenes"]["aisle_rack"]
    assert ar["success"] == [0, 1]
    assert ar["transport_tracking_failures"] == {"endpoint_heading": 1}
    assert ar["failure_classes"] == {"tracking": 1}
    assert agg["all"]["n"] == 4 and agg["all"]["success"] == [3, 4]


def test_aggregate_marks_metric_not_comparable(tmp_path):
    r = done_result()
    r.pop("near_capture")
    rows = [bs.run_fields(write_run(tmp_path, "pfb_R_line_2_s0", r), mode="overlay")]
    agg = bs.aggregate(rows)
    assert agg["scenes"]["line_2"]["near_status"] == "not_comparable"


def test_pair_differences(tmp_path):
    a = bs.run_fields(write_run(tmp_path / "a", "pfb_R_random_1_s0", done_result()), mode="overlay")
    b_res = done_result(success=False, phase="transport", failure_reason="Tracking failed in transport: pos=2,yaw=1",
                        tracking_failure={"phase": "transport", "tracking": {"failure": "cross_track"}},
                        arguments={k: v for k, v in ARGS.items() if k != "path_display"})
    b = bs.run_fields(write_run(tmp_path / "b", "lcr_B0_random_1_s0", b_res, header_mode=None), mode="prefix")
    c = bs.run_fields(write_run(tmp_path / "a", "pfb_R_line_2_s0", done_result()), mode="overlay")
    diff = bs.pair_differences([a, c], [b])
    assert diff["common"] == 1 and diff["only_a"] == [["line_2", 0]] and diff["only_b"] == []
    assert len(diff["changed"]) == 1
    ch = diff["changed"][0]
    assert ch["scene"] == "random_1" and ch["a"]["success"] is True and ch["b"]["failure_stage"] == "transport"


def test_refused_runs_are_out_of_the_aggregate(tmp_path):
    good = bs.run_fields(write_run(tmp_path, "pfb_R_random_1_s0", done_result()), mode="overlay")
    bad = bs.run_fields(write_run(tmp_path, "pfb_R_random_1_s1", done_result(), header_mode="stage"), mode="overlay")
    agg = bs.aggregate([good, bad])
    assert agg["all"]["n"] == 1 and agg["refused"] == [["pfb_R_random_1_s1", bad["refused"]]]


def test_collect_cli(tmp_path):
    root = tmp_path / "runs"
    write_run(root, "pfb_R_random_1_s0", done_result())
    write_run(root, "pfb_R_random_1_s1", done_result(), header_mode="stage")
    out = tmp_path / "o.json"
    rc = bs.main([str(root), "--mode", "overlay", "--out", str(out), "--fields-out", str(tmp_path / "f.json.gz")])
    assert rc == 0
    data = json.loads(out.read_text())
    assert data["aggregate"]["all"]["n"] == 1 and len(data["aggregate"]["refused"]) == 1
    assert (tmp_path / "f.json.gz").exists()


def test_mode_overlay_noheader_for_pre_513_comparators(tmp_path):
    ok = bs.run_fields(write_run(tmp_path, "k1r_K1_random_1_s0", done_result(), header_mode=None), mode="overlay-noheader")
    assert ok["refused"] is None
    assert bs.run_fields(write_run(tmp_path, "k1r_K1_random_1_s1", done_result()), mode="overlay-noheader")["refused"]
    pre = done_result(arguments={k: v for k, v in ARGS.items() if k != "path_display"})
    assert bs.run_fields(write_run(tmp_path, "k1r_K1_random_1_s2", pre, header_mode=None), mode="overlay-noheader")["refused"]
    # the baseline's own runs keep both checks
    assert bs.run_fields(write_run(tmp_path, "pfb_R_random_1_s3", done_result(), header_mode=None), mode="overlay")["refused"]
