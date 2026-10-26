import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from tools import isaac_bundle as ib

ROOT = Path(__file__).resolve().parents[3]


def test_array_sizes_give_one_task_its_own_share():
    z = ib.array_sizes("live", 16, 31)
    assert z == {"array": "0-15%5", "gres": "shard:1", "cpus": 4, "mem_mib": 8192, "time": "00:55:00"}
    r = ib.array_sizes("riib", 12, 63)
    assert r["gres"] == "shard:2" and r["cpus"] == 6  # 4.47 GB a session: more than one shard
    assert ib.array_sizes("live", 3, 31)["array"] == "0-2%3"


def test_a_four_cpu_task_gives_the_bridge_a_core_of_its_own():
    # 11590 (2026-10-26): at 2 CPUs the session and its bridge shared one core and 3 of 3 runs lost the SLAM link
    siblings = {7: (7, 19), 19: (7, 19), 8: (8, 20), 20: (8, 20)}.get
    cpus = [7, 19, 8, 20]
    n = ib.auto_bridge_cpus(ib.ARRAY["live"]["cpus"], 1)
    assert ib.split_bridge_cpus(cpus, n, siblings) == ([8, 20], [7, 19])


def test_array_sizes_need_the_batch_s_own_run_length():
    # S3 (2026-10-26): one live list held 21-min and 209-min runs -- a kind-wide default would cut the long ones
    with pytest.raises(ValueError, match="run-min"):
        ib.array_sizes("live", 8, None)
    assert ib.array_sizes("live", 8, 210)["time"] == "04:20:00"


def test_sizes_cli_prints_the_array_options(capsys):
    assert ib.main(["sizes", "live", "--array", "--runs", "16", "--run-min", "31"]) == 0
    assert capsys.readouterr().out.strip() == (
        "--array=0-15%5 --gres=shard:1 --cpus-per-task=4 --mem=8192M --time=00:55:00")
    assert ib.main(["sizes", "live", "--array", "--runs", "16"]) == 2


def test_collect_reports_every_task_and_the_missing_ones(tmp_path, capsys):
    for i, (status, rc) in {0: ("ended", 0), 2: ("stalled", -15)}.items():
        d = tmp_path / f"task_{i}" / "bundle"
        d.mkdir(parents=True)
        (d / "results.jsonl").write_text(json.dumps({"name": f"r{i}", "status": status, "rc": rc}) + "\n")
        (d / "summary.json").write_text(json.dumps({"exit_code": 0 if rc == 0 else 1}))
    rows, missing = ib.collect(tmp_path, 3)
    assert [r["task"] for r in rows] == [0, 2] and missing == [1]
    assert ib.main(["collect", str(tmp_path), "--expect", "3"]) == 1
    assert "1 missing, 2 not ended with rc 0" in capsys.readouterr().out
    assert ib.main(["collect", str(tmp_path), "--expect", "1"]) == 0


@pytest.mark.parametrize("summary", ['{"exit_code": 1, "errors": [{"where": "cleanup"}]}', "{not json", None])
def test_collect_takes_the_runner_s_verdict_over_the_run_s_rc(tmp_path, summary):
    # a run that ended rc 0 whose bridge cleanup failed leaves the summary at exit_code 1 (Codex, 2026-10-26)
    d = tmp_path / "task_0" / "bundle"
    d.mkdir(parents=True)
    (d / "results.jsonl").write_text(json.dumps({"name": "r0", "status": "ended", "rc": 0}) + "\n")
    if summary is not None:
        (d / "summary.json").write_text(summary)
    assert ib.main(["collect", str(tmp_path), "--expect", "1"]) == 1


def test_the_array_sbatch_runs_its_own_line(tmp_path):
    runs = tmp_path / "runs.jsonl"
    # a blank line after the first run, as load_runs allows: task 1 is still the second run
    runs.write_text("\n".join(json.dumps({"name": f"r{i}", "argv": [sys.executable, "-c", f"print({i})"]})
                              for i in range(3)).replace("\n", "\n\n", 1) + "\n")
    env = {k: v for k, v in os.environ.items() if k not in ("CUDA_VISIBLE_DEVICES", "SLURM_JOB_GPUS",
                                                             "SLURM_JOB_ID", "SLURM_JOB_END_TIME")}
    env.update({"SLURM_ARRAY_TASK_ID": "1", "BUNDLE_PY": sys.executable,
                "BUNDLE_ARGS": f"--no-vram --no-cleanup --node-ram-min-mib 0 --stagger-s 0 "
                               f"--list-registry-dir {tmp_path / 'reg'} --node-ledger-dir {tmp_path / 'ledger'}"})
    out = tmp_path / "out"
    p = subprocess.run(["bash", str(ROOT / "deploy/slurm/isaac_array.sbatch"), str(runs), str(out), "live"],
                       cwd=ROOT, env=env, capture_output=True, text=True, timeout=120)
    assert p.returncode == 0, p.stdout + p.stderr
    assert json.loads((out / "task_1" / "run.jsonl").read_text())["name"] == "r1"
    (rec,) = [json.loads(x) for x in (out / "task_1" / "bundle" / "results.jsonl").read_text().splitlines()]
    assert rec["name"] == "r1" and rec["status"] == "ended" and rec["rc"] == 0


def test_the_array_sbatch_refuses_a_task_past_the_list(tmp_path):
    runs = tmp_path / "runs.jsonl"
    runs.write_text(json.dumps({"name": "only", "argv": ["true"]}) + "\n")
    env = dict(os.environ, SLURM_ARRAY_TASK_ID="5")
    p = subprocess.run(["bash", str(ROOT / "deploy/slurm/isaac_array.sbatch"), str(runs), str(tmp_path / "o")],
                       cwd=ROOT, env=env, capture_output=True, text=True, timeout=60)
    assert p.returncode == 2 and "no line 5" in p.stderr
