"""B-P2d's session wrapper (deploy/console/reloc_session.sh): the console launcher and the probe's hold; a probe that
fails ends the launcher at once (nothing else would end a live session), and a launcher that outlives a normal end is
ended after a grace (Codex B-P2d tooling review P2)."""

import os
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / "deploy/console/reloc_session.sh"


def _fakes(tmp_path, probe_rc: int, launcher_s: float):
    launcher = tmp_path / "launcher.sh"
    launcher.write_text("#!/bin/bash\ntrap 'echo terminated >> \"$1/launcher_term\"; exit 143' TERM\n"
                        f"echo \"$RELOC_UNATTENDED $MAX_EPOCHS $START_POSE\" > \"$1/launcher_env\"\n"
                        f"for i in $(seq 1 {int(launcher_s * 10)}); do sleep 0.1; done\nexit 0\n")
    python = tmp_path / "python"
    python.write_text(f"#!/bin/bash\necho \"$@\" > {tmp_path}/probe_args\nexit {probe_rc}\n")
    python.chmod(0o755)
    return launcher, python


def _run(tmp_path, probe_rc, launcher_s, grace="1"):
    launcher, python = _fakes(tmp_path, probe_rc, launcher_s)
    env = {**os.environ, "START_POSE": "1,2,3", "VENV_PY": str(python), "RELOC_LAUNCHER": str(launcher),
           "RELOC_END_GRACE_S": grace, "FACTOR": "0.3"}
    t0 = time.monotonic()
    p = subprocess.run(["bash", str(WRAPPER), str(tmp_path / "out")], cwd=ROOT, env=env, capture_output=True,
                       text=True, timeout=60)
    return p, time.monotonic() - t0, tmp_path / "out"


def test_a_failing_probe_ends_the_launcher_at_once(tmp_path):
    p, took, out = _run(tmp_path, probe_rc=1, launcher_s=60)
    assert took < 15 and (out / "launcher_term").exists()
    assert p.returncode != 0 and "probe failed" in p.stdout


def test_a_normal_end_waits_for_the_launcher_and_passes_its_code(tmp_path):
    p, took, out = _run(tmp_path, probe_rc=0, launcher_s=1, grace="10")  # well past the launcher's 1 s
    assert p.returncode == 0 and not (out / "launcher_term").exists()
    assert (out / "launcher_env").read_text().split() == ["1", "1", "1,2,3"]  # unattended, one epoch, the start
    args = (tmp_path / "probe_args").read_text().split()
    assert args[args.index("--scenario") + 1] == "hold" and args[args.index("--live-factor") + 1] == "0.3"


def test_a_dry_run_is_the_launchers_with_the_sessions_environment_and_no_probe(tmp_path):
    # B-P2d development: the bundle refuses a list without isaac_bundle.py check's preflight proof, and check runs only
    # a launcher with a DRY_RUN mode printing RUNNER_ARGS -- the wrapper hands its DRY_RUN to the console launcher's
    launcher = tmp_path / "launcher.sh"
    launcher.write_text("#!/bin/bash\necho \"$DRY_RUN $RELOC_UNATTENDED $MAX_EPOCHS $START_POSE $1\"\n"
                        "echo RUNNER_ARGS\necho --start-pose=1,2,3\necho ARGS_OK\nexit 0\n")
    python = tmp_path / "python"
    python.write_text(f"#!/bin/bash\ntouch {tmp_path}/probe_ran\nexit 0\n")
    python.chmod(0o755)
    env = {**os.environ, "START_POSE": "1,2,3", "VENV_PY": str(python), "RELOC_LAUNCHER": str(launcher),
           "DRY_RUN": "1"}
    p = subprocess.run(["bash", str(WRAPPER), str(tmp_path / "out")], cwd=ROOT, env=env, capture_output=True,
                       text=True, timeout=60)
    assert p.returncode == 0 and not (tmp_path / "probe_ran").exists()
    assert p.stdout.splitlines()[0].split() == ["1", "1", "1", "1,2,3", str(tmp_path / "out")]
    assert "RUNNER_ARGS" in p.stdout.splitlines() and "ARGS_OK" in p.stdout.splitlines()
    assert "RUNNER_ARGS" in WRAPPER.read_text()  # what isaac_bundle.py's _dry_run_launcher looks for


def test_a_launcher_that_outlives_a_normal_end_is_ended_after_the_grace(tmp_path):
    p, took, out = _run(tmp_path, probe_rc=0, launcher_s=60, grace="1")
    assert took < 15 and (out / "launcher_term").exists()


def test_a_place_session_is_attended_and_its_probe_places_around_the_truth(tmp_path):
    # F1 (plan 2026-10-31-operator-placed-start 4.2 b): the console waits for the operator, the probe is the operator
    launcher, python = _fakes(tmp_path, probe_rc=0, launcher_s=1)
    env = {**os.environ, "START_POSE": "1,2,3", "VENV_PY": str(python), "RELOC_LAUNCHER": str(launcher),
           "RELOC_END_GRACE_S": "10", "FACTOR": "0.3", "RELOC_PROBE": "place", "CLICK_SEED": "11", "CLICK_INDEX": "4"}
    env["START_POSE"] = "-1,2,3"  # a negative x is one word (Codex F1 implementation review P2 4)
    p = subprocess.run(["bash", str(WRAPPER), str(tmp_path / "out")], cwd=ROOT, env=env, capture_output=True,
                       text=True, timeout=60)
    assert p.returncode == 0, p.stdout + p.stderr
    assert (tmp_path / "out" / "launcher_env").read_text().split() == ["1", "-1,2,3"]  # no RELOC_UNATTENDED
    args = (tmp_path / "probe_args").read_text().split()
    assert args[args.index("--scenario") + 1] == "place" and "--truth=-1,2,3" in args
    assert args[args.index("--click-seed") + 1] == "11" and args[args.index("--click-index") + 1] == "4"
    assert "--click-kind" not in args  # the click model, unless a record click is named


def test_a_place_session_can_name_a_record_click(tmp_path):
    # the lead's safety-net check (2026-10-31): does the bridge's start check stop a wrong placement an operator confirms
    launcher, python = _fakes(tmp_path, probe_rc=0, launcher_s=1)
    env = {**os.environ, "START_POSE": "1,2,3", "VENV_PY": str(python), "RELOC_LAUNCHER": str(launcher),
           "RELOC_END_GRACE_S": "10", "FACTOR": "0.3", "RELOC_PROBE": "place", "CLICK_KIND": "bay+y"}
    p = subprocess.run(["bash", str(WRAPPER), str(tmp_path / "out")], cwd=ROOT, env=env, capture_output=True,
                       text=True, timeout=60)
    assert p.returncode == 0, p.stdout + p.stderr
    args = (tmp_path / "probe_args").read_text().split()
    assert args[args.index("--click-kind") + 1] == "bay+y"


def test_an_unknown_probe_mode_is_refused(tmp_path):
    launcher, python = _fakes(tmp_path, probe_rc=0, launcher_s=1)
    env = {**os.environ, "START_POSE": "1,2,3", "VENV_PY": str(python), "RELOC_LAUNCHER": str(launcher),
           "RELOC_PROBE": "both"}
    p = subprocess.run(["bash", str(WRAPPER), str(tmp_path / "out")], cwd=ROOT, env=env, capture_output=True,
                       text=True, timeout=60)
    assert p.returncode == 2 and "RELOC_PROBE" in p.stdout + p.stderr
