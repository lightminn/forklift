"""The calibration tool (tools/global_match_eval.py, plan B4a): outcomes, the dev threshold choice and its tie-breaks,
the validation verdict and the shared-wall metric, on synthetic captures."""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from tools import global_match_eval as E

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "localization"))
import test_global_match as TG  # noqa: E402


def test_outcomes_follow_the_preregistered_tolerances():
    truth = (1.0, 1.0, 0.0)
    assert E.outcome(False, None, truth) == "request"
    assert E.outcome(True, (1.05, 1.0, 0.02), truth) == "success"
    assert E.outcome(True, (1.2, 1.0, 0.0), truth) == "imprecise"
    assert E.outcome(True, (1.0, 1.0, 0.15), truth) == "wrong"
    assert E.outcome(True, (1.4, 1.0, 0.0), truth) == "wrong"
    assert E.laser_to_base((1.0, 2.0, math.pi / 2)) == pytest.approx((1.0, 2.12, math.pi / 2))


def _row(rid, split, *, ratio=0.5, frac=0.9, err=0.0, map_index=0):
    truth = (2.0, 1.0, 0.0)
    laser = (truth[0] - 0.12 + err, truth[1], 0.0)
    cands = [{"pose": list(laser), "score": 0.9, "endpoint_fraction": frac, "refined": True,
              "lattice_pose": list(laser), "lattice_score": 0.9},
             {"pose": [5.0, 1.0, 0.0], "score": 0.9 * ratio, "endpoint_fraction": 0.5, "refined": True,
              "lattice_pose": [5.0, 1.0, 0.0], "lattice_score": 0.9 * ratio}]
    return {"id": rid, "split": split, "map_index": map_index, "valid": True, "status": "ok", "candidates": cands,
            "truth_base": list(truth), "wall_s": 1.0}


def _write(path, rows):
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    return str(path)


def test_select_keeps_only_clean_combinations_and_breaks_ties_strictly(tmp_path):
    rows = [_row("a", 0, ratio=0.5), _row("b", 0, ratio=0.7),
            _row("c", 0, ratio=0.6, err=0.5)]  # c: a wrong pose that must stay unconfirmed
    out = tmp_path / "choice.json"
    assert E.main(["select", "--rules", "b4a", "--rows", _write(tmp_path / "dev.jsonl", rows), "--out", str(out)]) == 0
    choice = json.loads(out.read_text())["choice"]
    # c's ratio 0.6 must be refused (r_max < 0.6), so a (0.5) is the only success; ties go to the highest E_MIN that
    # still confirms a (its fraction 0.9), then the lowest R_MAX (0.5 confirms a: 0.5 <= 0.5), then R_STRUCT 0.4
    assert choice["wrong"] == 0 and choice["success"] == 1
    assert (choice["e_min"], choice["r_max"], choice["r_struct"]) == (0.9, 0.5, 0.4)
    with pytest.raises(SystemExit):
        E.main(["select", "--rules", "b4a", "--rows", _write(tmp_path / "v.jsonl", [_row("x", 1)]), "--out", str(out)])


def test_judge_counts_per_map_and_bounds_only_without_a_wrong_one(tmp_path):
    choice = tmp_path / "c.json"
    choice.write_text(json.dumps({"choice": {"e_min": 0.85, "r_max": 0.8, "r_struct": 0.4}}))
    rows = [_row(f"v{i}", 1, map_index=i % 2) for i in range(60)]
    out = tmp_path / "verdict.json"
    E.main(["judge", "--rules", "b4a", "--rows", _write(tmp_path / "val.jsonl", rows), "--choice", str(choice),
            "--out", str(out)])
    v = json.loads(out.read_text())
    assert v["pass"] and v["success"] == 60 and v["bound_95"] == pytest.approx(0.0487, abs=1e-4)
    assert v["by_map"]["0"]["starts"] == 30 and v["by_map"]["1"]["bound_95"] == pytest.approx(0.0950, abs=1e-4)
    rows[0] = _row("v0", 1, err=0.5)
    E.main(["judge", "--rules", "b4a", "--rows", _write(tmp_path / "val2.jsonl", rows), "--choice", str(choice),
            "--out", str(out)])
    v = json.loads(out.read_text())
    assert not v["pass"] and v["wrong"] == 1 and v["bound_95"] is None


def _capture(path, poses, split, grid):
    raw, laser = [], []
    for base in poses:
        lx, ly = base[0] - 0.12 * math.cos(base[2]), base[1] - 0.12 * math.sin(base[2])
        r, a0, inc = TG._scan(grid, (lx, ly, base[2]), beams=360)
        raw.append(np.stack([r] * 5).astype(np.float32))
        laser.append((lx, ly, base[2]))
    meta = {"map_id": "m" * 64, "split": split, "map_index": 0, "angle_min_rad": 0.0,
            "angle_increment_rad": 2 * math.pi / 360, "range_min_m": 0.2, "range_max_m": 12.0}
    n = len(poses)
    np.savez(path, meta=json.dumps(meta), id=np.array([f"p{i}" for i in range(n)]), stratum=np.array(["A"] * n),
             valid=np.ones(n, bool), reason=np.array([""] * n), raw=np.stack(raw), sent=np.stack(raw),
             laser_truth=np.array(laser), base_truth=np.array(poses, float))
    return path


def test_run_matches_a_capture_and_overlap_measures_shared_walls(tmp_path):
    grid = TG._grid(*TG.ROOM)
    bundle = tmp_path / "b"
    bundle.mkdir()
    np.savez(bundle / "grid.npz", data=grid.data, origin=np.array(grid.origin), resolution_m=grid.resolution_m)
    (bundle / "manifest.json").write_text(json.dumps({"map_id": "m" * 64, "map_from_world": [0, 0, 0]}))
    cap = _capture(tmp_path / "cap.npz", [(2.42, 1.7, 0.6)], 1, grid)
    rows = tmp_path / "rows.jsonl"
    E.main(["run", "--capture", str(cap), "--bundle", str(bundle), "--out", str(rows)])
    row = json.loads(rows.read_text())
    assert row["status"] == "ok" and row["candidates"]
    assert E.outcome(True, E.laser_to_base(row["candidates"][0]["pose"]), row["truth_base"]) == "success"
    dev = _capture(tmp_path / "dev.npz", [(2.42, 1.7, 0.6)], 0, grid)  # the same spot: every wall shared
    far = _capture(tmp_path / "far.npz", [(5.2, 0.8, 0.0)], 0, grid)
    o = tmp_path / "o.json"
    E.main(["overlap", "--rules", "b4a", "--val", str(cap), "--dev", str(dev), "--out", str(o)])
    assert json.loads(o.read_text())["starts"][0]["shared_share"] == pytest.approx(1.0)
    E.main(["overlap", "--rules", "b4a", "--val", str(cap), "--dev", str(far), "--out", str(o)])
    assert json.loads(o.read_text())["starts"][0]["shared_share"] < 1.0


# --- plan B4b ------------------------------------------------------------------------------------------------------


def _choice_of(tmp_path, rows, *extra):
    out = tmp_path / "choice.json"
    code = E.main(["select", "--rows", _write(tmp_path / "dev.jsonl", rows), "--out", str(out), *extra])
    return code, json.loads(out.read_text())


def test_b4b_selects_only_the_exact_region_of_the_reduced_grid(tmp_path):
    code, out = _choice_of(tmp_path, [_row("a", 0, ratio=0.5, frac=0.97)])
    assert code == 0
    assert {(t["r_max"], t["r_struct"]) for t in out["table"]} == {(0.85, 0.8), (0.9, 0.8), (0.95, 0.8)}
    assert sorted({t["e_min"] for t in out["table"]}) == E.E_GRID


def test_b4b_margin_keeps_e_min_above_a_wrong_top_that_only_e_min_stops(tmp_path):
    # w: a wrong top with no competitor (ratio 0 after keep 0.80) and fraction 0.86 -- only E_MIN can refuse it;
    # W = 0.86, so E_MIN >= 0.91 and the grid gives 0.95. a: a right top at 0.97 still succeeds there.
    rows = [_row("a", 0, ratio=0.0, frac=0.97), _row("w", 0, ratio=0.0, frac=0.86, err=6.0)]
    code, out = _choice_of(tmp_path, rows)
    assert code == 0 and out["choice"]["e_min"] == 0.95 and out["choice"]["success"] == 1
    assert out["margin"]["delta"] == 0.05 and out["margin"]["w"] == pytest.approx(0.86)
    assert out["margin"]["wrong_tops"] == [{"id": "w", "endpoint_fraction": 0.86, "refused_by": "low_fit"}]


def test_b4b_margin_ignores_a_wrong_top_another_check_refuses(tmp_path):
    # the shape of dev1_035: 6 m wrong, fraction 1.0, but a competitor at 0.997 refuses it -- W must stay 0
    rows = [_row("a", 0, ratio=0.0, frac=0.97), _row("c", 0, ratio=0.997, frac=1.0, err=6.0)]
    code, out = _choice_of(tmp_path, rows)
    assert code == 0 and out["choice"]["e_min"] == 0.95 and out["margin"]["w"] == 0.0
    assert out["margin"]["wrong_tops"][0]["refused_by"] == "competitor"


def test_b4b_admits_an_opened_validation_set_as_dev_only_by_name(tmp_path):
    opened = _write(tmp_path / "g0.jsonl", [_row("g", 1, ratio=0.0, frac=0.97)])
    out = tmp_path / "choice.json"
    with pytest.raises(SystemExit):
        E.main(["select", "--rows", opened, "--out", str(out)])
    assert E.main(["select", "--opened-rows", opened, "--out", str(out)]) == 0
    assert json.loads(out.read_text())["opened_rows"] == [opened]


def _judge(tmp_path, rows, choice=(0.95, 0.95, 0.8), *extra):
    c = tmp_path / "c.json"
    c.write_text(json.dumps({"choice": dict(zip(("e_min", "r_max", "r_struct"), choice, strict=True))}))
    out = tmp_path / "verdict.json"
    E.main(["judge", "--rows", _write(tmp_path / "val.jsonl", rows), "--choice", str(c), "--out", str(out), *extra])
    return json.loads(out.read_text())


def _val(i, **kw):
    return _row(f"v{i}", 1, ratio=0.0, frac=0.97, **kw)


def test_b4b_judge_fails_on_an_imprecise_confirmation(tmp_path):
    rows = [_val(i) for i in range(60)]
    assert _judge(tmp_path, rows)["pass"]
    rows[0] = _val(0, err=0.2)
    v = _judge(tmp_path, rows)
    assert v["imprecise"] == 1 and not v["pass"] and not v["safety_pass"]


def test_b4b_judge_fails_when_requests_take_longer_than_the_allowed_overrun(tmp_path):
    # Codex B4b review: 60 starts that all took 31 s must not pass; a confirmation after 30 s counts as a timeout
    late = [dict(_val(i), wall_s=31.0) for i in range(60)]
    v = _judge(tmp_path, late)
    assert not v["pass"] and v["confirmed_after_deadline"] == 60 and v["timeouts"] == 60
    over = [_val(i) for i in range(60)]
    over[0] = dict(_val(0), status="timeout", candidates=[], wall_s=32.5)
    v = _judge(tmp_path, over)
    assert not v["time_pass"] and v["over_32_s"] == 1


def test_b4b_judge_allows_up_to_ten_percent_timeouts(tmp_path):
    def rows(n_timeout):
        out = [_val(i) for i in range(60)]
        for i in range(n_timeout):
            out[i] = dict(_val(i), status="timeout", candidates=[], wall_s=30.4)
        return out

    assert _judge(tmp_path, rows(6))["time_pass"]
    assert not _judge(tmp_path, rows(7))["time_pass"]


def test_b4b_judge_reports_the_low_overlap_subset(tmp_path):
    rows = [_val(i) for i in range(60)]
    o = tmp_path / "o.json"
    o.write_text(json.dumps({"starts": [{"id": f"v{i}", "shared_share": 0.1 if i < 12 else 0.5} for i in range(60)]}))
    v = _judge(tmp_path, rows, (0.95, 0.95, 0.8), "--overlap", str(o))
    assert v["low_overlap"] == {"threshold": 0.2, "starts": 12, "wrong": 0, "short": True}


def test_b4b_overlap_reads_the_matcher_input_not_the_raw_far_endpoints(tmp_path):
    # Codex B4b review: two captures at one spot whose scans differ only beyond 8 m (a far box) share every point
    # the matcher reads; the raw 12 m metric called that a partial overlap
    near = TG._grid(14.0, 4.0, [])
    far = TG._grid(14.0, 4.0, [(11.0, 1.0, 12.0, 2.0)])
    pose = [(1.5, 2.0, 0.0)]
    # the validation start sees the far box at ~9.6 m, the dev start does not: only the raw metric counts that
    val, dev = _capture(tmp_path / "v.npz", pose, 1, far), _capture(tmp_path / "d.npz", pose, 0, near)
    o = tmp_path / "o.json"
    E.main(["overlap", "--rules", "b4a", "--val", str(val), "--dev", str(dev), "--out", str(o)])
    assert json.loads(o.read_text())["starts"][0]["shared_share"] < 1.0
    E.main(["overlap", "--val", str(val), "--dev", str(dev), "--out", str(o)])
    assert json.loads(o.read_text())["starts"][0]["shared_share"] == pytest.approx(1.0)


def test_b4b_run_records_the_index_the_work_and_a_timeout(tmp_path):
    grid = TG._grid(*TG.ROOM)
    bundle = tmp_path / "b"
    bundle.mkdir()
    np.savez(bundle / "grid.npz", data=grid.data, origin=np.array(grid.origin), resolution_m=grid.resolution_m)
    (bundle / "manifest.json").write_text(json.dumps({"map_id": "m" * 64, "map_from_world": [0, 0, 0]}))
    cap = _capture(tmp_path / "cap.npz", [(2.42, 1.7, 0.6)], 1, grid)
    rows = tmp_path / "rows.jsonl"
    E.main(["run", "--capture", str(cap), "--bundle", str(bundle), "--out", str(rows),
            "--config", json.dumps({"yaw_step_rad": math.radians(2.0)})])
    row = json.loads(rows.read_text())
    index = json.loads((tmp_path / "rows.jsonl.index.json").read_text())
    assert row["status"] == "ok" and row["work"] > 0 and index["build_s"] > 0 and index["bytes"] > 0
    E.main(["run", "--capture", str(cap), "--bundle", str(bundle), "--out", str(rows),
            "--config", json.dumps({"yaw_step_rad": math.radians(2.0), "deadline_s": 0.0})])
    row = json.loads(rows.read_text())
    assert row["status"] == "timeout" and row["candidates"] == []
