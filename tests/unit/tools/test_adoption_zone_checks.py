import json

import pytest

from tools import adoption_zone_checks as C


def check(stage, site, truth, edge, source="session_zone"):
    c = {"stage": stage, "site": list(site), "edge_m": edge, "zone_source": source, "margin_m": 5.0, "inside": True}
    if truth is not None:
        c["truth"] = list(truth)
    return c


def write_run(root, name, missions, arguments=None):
    run = root / name / "live" / "epoch_1" / "run"
    run.mkdir(parents=True)
    (run / "result.json").write_text(json.dumps({"arguments": arguments or {"slam_map_bundle": "/b/line_m"}}))
    for i, checks in enumerate(missions, start=1):
        d = run / "missions" / f"m{i}"
        d.mkdir(parents=True)
        (d / "records.json").write_text(json.dumps({"state": {"adoption_zone": {"checks": checks}}}))


def test_per_stage_error_edge_and_refusals_at_a_margin(tmp_path):
    write_run(tmp_path, "a", [
        [check("far", (3.0, 0.1), (3.0, 0.0), -0.2), check("insert", (3.02, 0.0), (3.0, 0.0), -0.1)],
        [check("near", (9.1, 0.0), (9.0, 0.0), 0.12, source="put_down")],
    ])
    s = C.summarize_dirs([tmp_path], margins=(0.10, 0.30))
    assert s["stages"]["far"]["max_abs_m"] == pytest.approx(0.1)
    assert s["stages"]["near"]["max_edge_over_m"] == pytest.approx(0.12)
    assert s["refused_at"] == {"0.10": 1, "0.30": 0}
    assert s["put_down_sessions"] == 1 and s["sessions"] == 1 and s["missions"] == 2
    assert s["margin_m"] == pytest.approx(0.10)  # the pre-registered rule: the error 0.1 rounded up
    assert s["edge_margin_m"] == pytest.approx(0.15)  # what the decision quantity needs: 0.12 rounded up


def test_a_check_without_truth_is_undetermined(tmp_path):
    write_run(tmp_path, "a", [[check("far", (3.0, 0.1), None, -0.2)]])
    s = C.summarize_dirs([tmp_path], margins=(0.30,))
    assert s["undetermined_checks"] == 1 and s["stages"] == {}


def test_a_mapping_fallback_run_is_kept_apart(tmp_path):
    write_run(tmp_path, "a", [[check("far", (3.0, 0.1), (3.0, 0.0), -0.2)]], arguments={"slam_map_bundle": None})
    s = C.summarize_dirs([tmp_path], margins=(0.30,))
    assert s["sessions"] == 0 and s["excluded_runs"] == [str(tmp_path / "a" / "live" / "epoch_1" / "run")]
