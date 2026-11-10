"""The live launcher's CPUs (1e (c), docs/validation/2026-11-11-g3-1e-c-results.md, the lead's decision of 2026-11-11):
alone it asks for CPU 6 and splits the job's CPUs as a single-slot bundle -- a whole core for the SLAM bridge, the
session pinned to the rest (two cores at CPU 6, one at CPU 4, nothing split below); in a bundle BRIDGE_CPU wins and the
session is the bundle's to pin. The bundle sizes take --session-cores 2."""

import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
LAUNCHER = ROOT / "deploy/console/isaac_live.sbatch"
SIB = '{"0":[0,12],"12":[0,12],"1":[1,13],"13":[1,13],"2":[2,14],"14":[2,14]}'


@pytest.fixture
def tmp_path():
    """A short root under /tmp (the launcher refuses an OUT whose SLAM socket path passes 107 bytes)."""
    root = Path(tempfile.mkdtemp(dir="/tmp", prefix="lc"))
    yield root
    shutil.rmtree(root, ignore_errors=True)


def _launch(tmp_path, **env):
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
            "PYTHONPATH": str(ROOT / "src"), "MAP_BUNDLE": "none", "DRY_RUN": "1", "LIVE_TEST_SIBLINGS": SIB}
    base.update(env)
    return subprocess.run(["bash", str(LAUNCHER), str(tmp_path / "out")], cwd=ROOT, env=base, capture_output=True,
                          text=True, timeout=180)


def _env_sh(tmp_path) -> dict:
    """env.sh as the epoch scripts read it (sourced): printf %q escapes commas."""
    out = {}
    for line in (tmp_path / "out" / "env.sh").read_text().splitlines():
        k, _, v = line.partition("=")
        out[k] = v.replace("\\,", ",")
    return out


def test_the_launcher_asks_for_six_cpus():
    assert "#SBATCH --cpus-per-task=6\n" in LAUNCHER.read_text()


@pytest.mark.parametrize("cpus,bridge,session", [
    ("0,1,2,12,13,14", "2,14", "0,1,12,13"),  # CPU 6: the session on two whole cores
    ("0,1,12,13", "1,13", "0,12"),  # CPU 4: one core each
])
def test_alone_the_job_splits_a_bridge_core_and_pins_the_session(tmp_path, cpus, bridge, session):
    r = _launch(tmp_path, LIVE_TEST_CPUS=cpus)
    assert r.returncode == 0, r.stdout + r.stderr
    env = _env_sh(tmp_path)
    assert (env["CPU0"], env["SESSION_CPUS"]) == (bridge, session)
    assert f"cpus: bridge {bridge}, session {session}" in r.stdout
    runner = (tmp_path / "out" / "runner.sh").read_text()
    assert 'PIN=(taskset -c "$SESSION_CPUS")' in runner and 'exec "${PIN[@]}" "$VENV_PY"' in runner


def test_too_few_cpus_split_nothing(tmp_path):
    r = _launch(tmp_path, LIVE_TEST_CPUS="0,12")
    assert r.returncode == 0, r.stdout + r.stderr
    env = _env_sh(tmp_path)
    assert env["SESSION_CPUS"] in ("''", "") and "session unpinned" in r.stdout


def test_a_bundle_bridge_cpu_wins_and_leaves_the_session_to_the_bundle(tmp_path):
    r = _launch(tmp_path, LIVE_TEST_CPUS="0,1,2,12,13,14", BRIDGE_CPU="5")
    assert r.returncode == 0, r.stdout + r.stderr
    env = _env_sh(tmp_path)
    assert env["CPU0"] == "5" and env["SESSION_CPUS"] in ("''", "")


def test_the_bundle_sizes_take_two_session_cores():
    sys.path.insert(0, str(ROOT / "tools"))
    import isaac_bundle as b

    assert b.sbatch_sizes("live", 1)["cpus"] == 4  # the default stays one core a session
    assert b.sbatch_sizes("live", 1, session_cores=2)["cpus"] == 6
    assert b.sbatch_sizes("live", 3, session_cores=2)["cpus"] == 14
    with pytest.raises(ValueError):
        b.sbatch_sizes("live", 1, session_cores=3)
    assert b.auto_bridge_cpus(6, 1) == b.BRIDGE_CORE_CPUS  # the runner still sets a core apart
