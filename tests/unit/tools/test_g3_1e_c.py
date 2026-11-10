"""1e (c) core-only direct comparison (docs/plans/2026-11-11-g3-1e-core-only.md): four single-session jobs A B B A --
A one physical core (job CPU 4), B two (job CPU 6) -- 1e off, the same snapshot; the per-pair criterion
W(A) - W(B) >= 0.03707 in both pairs; invalidity kinds; the allocation check against ws1's sibling layout."""

import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
D = ROOT / "docs" / "validation" / "2026-11-11-g3-1e-c"


def load(name):
    spec = importlib.util.spec_from_file_location(name, D / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


lists = load("g3_1e_c_lists")
an = load("g3_1e_c_analyze")


def test_the_four_jobs_are_a_b_b_a_with_1e_off_and_the_same_switches():
    jobs = lists.jobs("/snap", "/out")
    assert [(j["arm"], j["pair"], j["cpus"]) for j in jobs] == [("A", 1, 4), ("B", 1, 6), ("B", 2, 6), ("A", 2, 4)]
    for j in jobs:
        (e,) = j["entries"]
        env = e["env"]
        assert "LAYER_WORKER" not in env and "LAYER_WORKER_CPUS" not in env  # 1e off in both arms
        assert (env["FORKLIFT_GRID_CLEAR_KERNEL"], env["FORKLIFT_SHADOW_KERNEL"], env["LIDAR_CAST"], env["SET"],
                env["ARM"], env["FACTOR"], env["INSTR"], env["SEED"]) == ("1", "1", "fast", "far", "K", "1.0", "off", "0")
        assert "RETURN_START" not in env and "NEAR_END_STAND" not in env  # the launcher's defaults, both arms
        assert e["name"] == j["name"] and e["argv"][1].endswith("r1c_session.sh")
    assert len({j["name"] for j in jobs}) == 4


def test_the_allocation_check_reads_whole_cores():
    assert an.whole_cores([0, 12], offset=12) == 1
    assert an.whole_cores([0, 1, 12, 13], offset=12) == 2
    assert an.whole_cores([0, 1, 12], offset=12) is None  # an incomplete core
    assert an.allocation_problem({"slots": 1, "session_cpus": [0, 12]}, "A") is None
    assert an.allocation_problem({"slots": 1, "session_cpus": [0, 1, 12, 13]}, "B") is None
    assert an.allocation_problem({"slots": 1, "session_cpus": [0, 1, 12, 13]}, "A")
    assert an.allocation_problem({"slots": 2, "session_cpus": [0, 12]}, "A")
    assert an.allocation_problem({}, "B")


def test_the_attempts_must_be_one_ended_run():
    assert an.attempt_problem([{"attempt": 1, "status": "ended", "retried": False}]) is None
    assert an.attempt_problem([{"attempt": 1, "status": "ended", "retried": False},
                               {"attempt": 2, "status": "ended", "retried": True}])
    assert an.attempt_problem([])


def test_the_launcher_values_must_agree_between_the_arms():
    a = {"RETURN_START": "boxed", "NEAR_END_STAND": "on"}
    assert an.launcher_problem([a, dict(a), dict(a), dict(a)]) is None
    assert an.launcher_problem([a, {**a, "NEAR_END_STAND": "off"}, a, a])
    assert an.launcher_problem([a, {}, a, a])


def test_the_judgement_is_per_pair():
    assert an.judge({1: (0.874, 0.800), 2: (0.870, 0.790)})["decision"] == "(c) passes"
    r = an.judge({1: (0.874, 0.800), 2: (0.820, 0.790)})  # pair 2 saves 0.030
    assert r["decision"] == "(c) fails" and r["pairs"][2]["passes"] is False
    assert an.judge({1: (0.80, 0.82), 2: (0.80, 0.70)})["decision"] == "(c) fails"
    assert abs(an.judge({1: (0.874, 0.800), 2: (0.870, 0.790)})["pairs"][1]["saving"] - 0.074) < 1e-12


def test_the_list_set_must_be_registered(tmp_path, monkeypatch, capsys):
    jobs = lists.jobs("/s", str(tmp_path))
    files = []
    for j in reversed(jobs):
        f = tmp_path / f"{j['name']}.jsonl"
        f.write_text(json.dumps(j["entries"][0]) + "\n")
        files.append(str(f))
    monkeypatch.setattr(sys, "argv", ["g3_1e_c_analyze.py", "--lists", *files])
    assert an.main() == 2 and "list_not_registered" in capsys.readouterr().out


def test_the_submit_script_chains_four_jobs_with_their_sizes():
    text = (D / "g3_1e_c_submit.sh").read_text()
    assert text.count("sbatch ") == 1 and "--dependency=afterany:" in text
    assert "--cpus-per-task=\"$CPUS\"" in text and "--slots 1 --retries 0" in text
    with pytest.raises(SystemExit):
        lists.main_args(["x"])
