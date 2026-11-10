"""④⑤ as the live console's defaults (the lead's promotion decision of 2026-11-11 after rv4,
docs/validation/2026-11-11-live-riib-rv4-results.md): the console launcher passes --return-start-backoff boxed and
--near-end-stand on unless told otherwise; an explicit RETURN_START / NEAR_END_STAND wins, an EXTRA_ARGS --near-end-stand
comes later and wins; a --near-tracking run keeps both off (its contract records the return-start backoff off) and
refuses an explicit return-start backoff; the survey keeps both off on the console's runner word for word."""
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
LAUNCHER = ROOT / "deploy/console/isaac_live.sbatch"


@pytest.fixture
def tmp_path():
    """A short root under /tmp (the launcher refuses an OUT whose SLAM socket path passes 107 bytes)."""
    root = Path(tempfile.mkdtemp(dir="/tmp", prefix="r45"))
    yield root
    shutil.rmtree(root, ignore_errors=True)


def _launch(tmp_path, **env):
    """DRY_RUN with a stand-in Python: ``-c`` snippets run for real; the runner's check writes its arguments."""
    bin_ = tmp_path / "bin"
    bin_.mkdir(exist_ok=True)
    docker = bin_ / "docker"
    docker.write_text(f"#!/bin/bash\necho \"$0 $*\" >> {tmp_path / 'started'}\n")
    docker.chmod(0o755)
    fake = tmp_path / "fakepy"
    fake.write_text(f"#!/bin/bash\nif [ \"$1\" = -c ]; then exec {sys.executable} \"$@\"; fi\n"
                    f"printf '%s\\n' \"$@\" > {tmp_path / 'runner_args'}\necho ARGS_OK\n")
    fake.chmod(0o755)
    base = {"PATH": f"{bin_}:{os.environ['PATH']}", "BASE_SCENE": "scene.usda",
            "FORKLIFT_URDF": str(ROOT / "sim/models/dls08_measured/forklift.urdf"), "VENV_PY": str(fake),
            "PYTHONPATH": str(ROOT / "src"), "MAP_BUNDLE": "none", "DRY_RUN": "1"}
    base.update(env)
    return subprocess.run(["bash", str(LAUNCHER), str(tmp_path / "out")], cwd=ROOT, env=base, capture_output=True,
                          text=True, timeout=180)


def _env_sh(tmp_path) -> dict:
    out = {}
    for line in (tmp_path / "out" / "env.sh").read_text().splitlines():
        k, _, v = line.partition("=")
        out[k] = v
    return out


def _values(tmp_path, flag) -> list:
    """Every value the runner was given for ``flag``, in order (argparse keeps the last)."""
    args = (tmp_path / "runner_args").read_text().splitlines()
    return [args[i + 1] if a == flag else a.split("=", 1)[1]
            for i, a in enumerate(args) if a == flag or a.startswith(flag + "=")]


def test_the_console_runs_4_and_5_by_default(tmp_path):
    p = _launch(tmp_path)
    assert p.returncode == 0, p.stdout + p.stderr
    assert "RIIB_DEFAULTS return_start_backoff=boxed near_end_stand=on" in p.stdout
    e = _env_sh(tmp_path)
    assert (e["RETURN_START"], e["NEAR_END_STAND"]) == ("boxed", "on")
    assert _values(tmp_path, "--return-start-backoff") == ["boxed"]
    assert _values(tmp_path, "--near-end-stand") == ["on"]
    assert not (tmp_path / "started").exists()


def test_an_explicit_value_wins(tmp_path):
    p = _launch(tmp_path, RETURN_START="off", NEAR_END_STAND="shadow")
    assert p.returncode == 0, p.stdout + p.stderr
    assert "RIIB_DEFAULTS return_start_backoff=off near_end_stand=shadow" in p.stdout
    assert _values(tmp_path, "--return-start-backoff") == ["off"]
    assert _values(tmp_path, "--near-end-stand") == ["shadow"]


def test_an_extra_args_near_end_stand_comes_later_and_wins(tmp_path):
    p = _launch(tmp_path, EXTRA_ARGS="--near-end-stand off")
    assert p.returncode == 0, p.stdout + p.stderr
    assert _values(tmp_path, "--near-end-stand") == ["on", "off"]  # the launcher's first, the list's last


def test_a_near_tracking_run_keeps_both_off_and_refuses_an_explicit_return_start_backoff(tmp_path):
    _launch(tmp_path, EXTRA_ARGS="--near-tracking")  # the runner's own check may refuse a bare --near-tracking
    e = _env_sh(tmp_path)
    assert (e["RETURN_START"], e["NEAR_END_STAND"]) == ("off", "off")
    for env in ({"RETURN_START": "boxed"}, {"RETURN_START": "on"},
                {"EXTRA_ARGS": "--near-tracking --return-start-backoff boxed"}):
        p = _launch(tmp_path, **{"EXTRA_ARGS": "--near-tracking", **env})
        assert p.returncode == 2 and "--near-tracking keeps the return-start backoff off" in p.stdout, \
            (env, p.stdout + p.stderr)


@pytest.mark.parametrize("env, message", [({"RETURN_START": "yes"}, "RETURN_START must be on, boxed or off"),
                                          ({"NEAR_END_STAND": "1"}, "NEAR_END_STAND must be on, shadow or off")])
def test_anything_else_is_refused(tmp_path, env, message):
    p = _launch(tmp_path, **env)
    assert p.returncode == 2 and message in p.stdout, p.stdout + p.stderr


def test_the_survey_keeps_both_off():
    survey = (ROOT / "deploy/console/slam_survey.sbatch").read_text()
    assert "printf 'RETURN_START=off\\nNEAR_END_STAND=off\\n'" in survey
