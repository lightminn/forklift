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
    assert E.main(["select", "--rows", _write(tmp_path / "dev.jsonl", rows), "--out", str(out)]) == 0
    choice = json.loads(out.read_text())["choice"]
    # c's ratio 0.6 must be refused (r_max < 0.6), so a (0.5) is the only success; ties go to the highest E_MIN that
    # still confirms a (its fraction 0.9), then the lowest R_MAX (0.5 confirms a: 0.5 <= 0.5), then R_STRUCT 0.4
    assert choice["wrong"] == 0 and choice["success"] == 1
    assert (choice["e_min"], choice["r_max"], choice["r_struct"]) == (0.9, 0.5, 0.4)
    with pytest.raises(SystemExit):
        E.main(["select", "--rows", _write(tmp_path / "v.jsonl", [_row("x", 1)]), "--out", str(out)])


def test_judge_counts_per_map_and_bounds_only_without_a_wrong_one(tmp_path):
    choice = tmp_path / "c.json"
    choice.write_text(json.dumps({"choice": {"e_min": 0.85, "r_max": 0.8, "r_struct": 0.4}}))
    rows = [_row(f"v{i}", 1, map_index=i % 2) for i in range(60)]
    out = tmp_path / "verdict.json"
    E.main(["judge", "--rows", _write(tmp_path / "val.jsonl", rows), "--choice", str(choice), "--out", str(out)])
    v = json.loads(out.read_text())
    assert v["pass"] and v["success"] == 60 and v["bound_95"] == pytest.approx(0.0487, abs=1e-4)
    assert v["by_map"]["0"]["starts"] == 30 and v["by_map"]["1"]["bound_95"] == pytest.approx(0.0950, abs=1e-4)
    rows[0] = _row("v0", 1, err=0.5)
    E.main(["judge", "--rows", _write(tmp_path / "val2.jsonl", rows), "--choice", str(choice), "--out", str(out)])
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
    E.main(["overlap", "--val", str(cap), "--dev", str(dev), "--out", str(o)])
    assert json.loads(o.read_text())["starts"][0]["shared_share"] == pytest.approx(1.0)
    E.main(["overlap", "--val", str(cap), "--dev", str(far), "--out", str(o)])
    assert json.loads(o.read_text())["starts"][0]["shared_share"] < 1.0
