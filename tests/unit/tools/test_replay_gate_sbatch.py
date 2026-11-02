"""deploy/slurm/replay_gate.sbatch run as a plain script on a stub snapshot: a private copy per task, the recorded
setup inputs written onto the copy only, and the task's exit code is the outcome (Codex gate-overlay review 1 P2)."""

import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
SCRIPT = REPO / "deploy/slurm/replay_gate.sbatch"
# The stub replay: checks the copy's config is the recorded one when asked to, and exits with STUB_EXIT.
STUB = """import os, sys, pathlib
want = os.environ.get("STUB_WANT")
if want is not None and pathlib.Path("config/a.yaml").read_text() != want:
    sys.exit(9)
pathlib.Path(sys.argv[sys.argv.index("--json") + 1]).write_text("{}")
sys.exit(int(os.environ.get("STUB_EXIT", "0")))
"""


def sha(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


@pytest.fixture
def world(tmp_path):
    snap = tmp_path / "snap"
    (snap / "tools").mkdir(parents=True)
    (snap / "tools/boundary_replay.py").write_text(STUB)
    shutil.copy(REPO / "tools/gate_setup_overlay.py", snap / "tools/gate_setup_overlay.py")
    (snap / "config").mkdir()
    (snap / "config/a.yaml").write_bytes(b"mass: 4.93\n")
    rec = tmp_path / "snapshots/rec"
    (rec / "config").mkdir(parents=True)
    (rec / "config/a.yaml").write_bytes(b"mass: 10\n")
    (rec / "src").mkdir()
    (rec / "src/m.py").write_bytes(b"x\n")
    run = tmp_path / "runs/r1"
    (run / "boundary").mkdir(parents=True)
    files = [{"path": str(rec / "src/m.py"), "repo_relative": "src/m.py", "sha256": sha(b"x\n")},
             {"path": "config/a.yaml", "repo_relative": "config/a.yaml", "sha256": sha(b"mass: 10\n")}]
    (run / "boundary/setup_snapshot.json").write_text(json.dumps({"files": files}))
    lst = tmp_path / "list.txt"
    lst.write_text("r1\n")
    return {"tmp": tmp_path, "snap": snap, "run": run, "list": lst, "out": tmp_path / "out"}


def gate(w, **env):
    e = dict(os.environ, VENV_PY=sys.executable, RUNS_ROOT=str(w["tmp"] / "runs"), SLURM_ARRAY_TASK_ID="1", **env)
    p = subprocess.run(["bash", str(SCRIPT), str(w["snap"]), str(w["list"]), str(w["out"])], env=e,
                       capture_output=True, text=True, timeout=60)
    return p.returncode, p.stdout


def test_recorded_writes_the_recorded_config_onto_the_copy_and_reports_it(world):
    code, out = gate(world, SETUP_INPUTS="recorded", STUB_WANT="mass: 10\n")
    assert code == 0, out
    assert f"SETUP_WRITTEN config/a.yaml {sha(b'mass: 10' + bytes([10]))} {sha(b'mass: 4.93' + bytes([10]))}" in out
    assert "REPLAY_EXIT 0 r1 setup=recorded" in out
    assert (world["snap"] / "config/a.yaml").read_bytes() == b"mass: 4.93\n"  # the snapshot itself is untouched
    assert not (world["out"] / "work_1").exists()


def test_strict_writes_nothing(world):
    code, out = gate(world, STUB_WANT="mass: 4.93\n")
    assert code == 0 and "SETUP_WRITTEN" not in out and "REPLAY_EXIT 0 r1 setup=strict" in out


def test_the_replay_code_is_the_task_code(world):
    code, out = gate(world, STUB_EXIT="1")
    assert code == 1 and "REPLAY_EXIT 1 r1 setup=strict" in out


def test_an_overlay_refusal_ends_the_task_with_3_before_any_replay(world):
    (world["snap"] / "config/a.yaml").unlink()
    (world["snap"] / "config/a.yaml").symlink_to(world["snap"] / "tools/boundary_replay.py")
    code, out = gate(world, SETUP_INPUTS="recorded")
    assert code == 3 and "SETUP_REFUSED" in out and not (world["out"] / "r_1.json").exists()
    assert (world["snap"] / "tools/boundary_replay.py").read_text() == STUB


def test_a_result_already_there_is_never_replaced(world):
    world["out"].mkdir()
    (world["out"] / "r_1.json").write_text("earlier")
    code, out = gate(world)
    assert code == 2 and (world["out"] / "r_1.json").read_text() == "earlier"


def test_a_claimed_work_directory_is_left_to_its_owner(world):
    (world["out"] / "work_1").mkdir(parents=True)
    (world["out"] / "work_1/theirs").write_text("x")
    code, out = gate(world)
    assert code == 2 and (world["out"] / "work_1/theirs").exists()


def test_a_directory_that_is_not_a_snapshot_is_refused(world):
    (world["snap"] / "tools/boundary_replay.py").unlink()
    code, out = gate(world)
    assert code == 2 and "not a snapshot" in out
