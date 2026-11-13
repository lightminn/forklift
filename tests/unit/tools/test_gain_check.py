"""Baseline fragility gain check (docs/validation/2026-11-13-baseline-fragility/gain_check.py): a rotation-gain fit of
the bias change's yaw against the true rotation. Synthetic records, no Isaac."""

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    "gain_check", ROOT / "docs/validation/2026-11-13-baseline-fragility/gain_check.py")
G = importlib.util.module_from_spec(spec)
spec.loader.exec_module(G)


def write_run(root: Path, name: str, scene: str, turn: float, k: float) -> dict:
    """A truck turning by `turn` from the capture (5 s) to lift (6 s); its estimate turns (1 + k) times as much."""
    run = root / name / "run"
    run.mkdir(parents=True)
    t = np.linspace(5.0, 6.0, 11)
    truth_yaw = np.linspace(0.0, turn, 11)
    sc = np.column_stack([t, np.zeros(11), np.zeros(11), (1 + k) * truth_yaw, np.zeros(11), np.zeros(11), truth_yaw])
    np.save(run / "slam_control.npy", sc)
    result = {"near_capture": {"near_attempt": 1},
              "observation_attempts": [{"capture_control": {"time_s": 5.0, "truth_rear": [0.0, 0.0, 0.0],
                                                            "control_rear": [0.0, 0.0, 0.0]}}]}
    (run / "result.json").write_text(json.dumps(result))
    return {"root": root.name, "name": name, "scene": scene, "t_lift": 6.0, "bias_course": {"dpsi_rad": k * turn}}


def test_a_true_gain_is_recovered_and_carries_across_scenes(tmp_path):
    runs = [write_run(tmp_path, f"r{i}", ("a", "b", "c")[i % 3], turn, 0.08)
            for i, turn in enumerate((0.05, -0.03, 0.02, -0.06, 0.04, -0.01, 0.07, -0.02, 0.03))]
    out = G.analyse(runs, tmp_path.parent)
    assert out["single_gain"]["k"] == pytest.approx(0.08) and out["single_gain"]["r2"] == pytest.approx(1.0)
    assert out["left_right"]["kL"] == pytest.approx(0.08) and out["left_right"]["kR"] == pytest.approx(0.08)
    assert all(v["r2_on_held_out"] == pytest.approx(1.0) for v in out["scene_held_out"].values())


def test_unrelated_yaw_errors_give_no_fit(tmp_path):
    rng = np.random.default_rng(0)
    runs = []
    for i in range(30):
        r = write_run(tmp_path, f"r{i}", ("a", "b", "c")[i % 3], float(rng.uniform(-0.05, 0.05)), 0.0)
        r["bias_course"]["dpsi_rad"] = float(rng.normal(0, 0.003))
        runs.append(r)
    out = G.analyse(runs, tmp_path.parent)
    assert out["single_gain"]["r2"] < 0.3 and out["n"] == 30


def test_the_cli(tmp_path):
    run = write_run(tmp_path, "lbg_B0_random_1_s0", "random_1", 0.05, 0.1)
    runs = [dict(run, judgement={"status": "judged"}),
            dict(write_run(tmp_path, "lbg_B0_random_1_s1", "random_1", -0.04, 0.1), judgement={"status": "not_selected"}),
            dict(write_run(tmp_path, "lbg_B0_line_2_s0", "line_2", 0.03, 0.1), judgement={"status": "not_selected"}),
            dict(write_run(tmp_path, "lbg_B0_line_2_s1", "line_2", -0.02, 0.1), judgement={"status": "not_selected"}),
            {"name": "x", "judgement": {"status": "held"}}]
    decomp, out = tmp_path / "d.json", tmp_path / "g.json"
    decomp.write_text(json.dumps({"runs": runs}))
    assert G.main([str(decomp), "--raw-root", str(tmp_path.parent), "--out", str(out)]) == 0
    assert json.loads(out.read_text())["single_gain"]["k"] == pytest.approx(0.1)
