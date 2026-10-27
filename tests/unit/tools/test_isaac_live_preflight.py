"""The live launcher's preflight (docs/plans/2026-10-27-ws1-preflight.md): DRY_RUN=1 resolves the SLAM mode, prints
the runner's arguments and ends with the runner's own check, starting no container, supervisor or Isaac; a real launch
makes the same check first and ends at once when it fails (lvram, d7oor 2026-10-27)."""
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
LAUNCHER = ROOT / "deploy/console/isaac_live.sbatch"


def _launch(tmp_path, venv_py=sys.executable, **env):
    bin_ = tmp_path / "bin"
    bin_.mkdir(exist_ok=True)
    for tool in ("docker",):  # anything that would start a container leaves a trace instead
        fake = bin_ / tool
        fake.write_text(f"#!/bin/bash\necho \"$0 $*\" >> {tmp_path / 'started'}\n")
        fake.chmod(0o755)
    base = {"PATH": f"{bin_}:{os.environ['PATH']}", "BASE_SCENE": "scene.usda",
            "FORKLIFT_URDF": str(ROOT / "sim/models/dls08_measured/forklift.urdf"), "VENV_PY": str(venv_py),
            "PYTHONPATH": str(ROOT / "src"), "MAP_BUNDLE": "none", "DRY_RUN": "1"}
    base.update(env)
    return subprocess.run(["bash", str(LAUNCHER), str(tmp_path / "out")], cwd=ROOT, env=base, capture_output=True,
                          text=True, timeout=180)


def test_the_dry_run_prints_the_arguments_and_the_runner_s_verdict_and_starts_nothing(tmp_path):
    p = _launch(tmp_path)
    assert p.returncode == 0, p.stdout + p.stderr
    lines = p.stdout.splitlines()
    assert "RUNNER_ARGS" in lines and lines[-1] == "ARGS_OK"
    assert "--live" in lines and "--check-args" not in lines  # the run's own arguments, the check added after them
    assert not (tmp_path / "started").exists() and not (tmp_path / "out" / "live_port.json").exists()


def test_the_dry_run_refuses_what_the_launch_would(tmp_path):
    # a registry bundle asked for without a scene file, and a bundle folder without its start pose (lvram's end)
    p = _launch(tmp_path, MAP_BUNDLE="registry")
    assert p.returncode == 2 and "needs SCENE_FILE" in p.stderr
    folder = tmp_path / "bundle"
    folder.mkdir()
    p = _launch(tmp_path, MAP_BUNDLE=str(folder))
    assert p.returncode != 0 and "MAP_START_POSE" in p.stderr


def test_a_runner_check_that_fails_ends_the_launch_before_any_container(tmp_path):
    fake = tmp_path / "fakepy"
    fake.write_text("#!/bin/bash\nif [ \"$1\" = -c ]; then exec python3 \"$@\"; fi\n"
                    'for a in "$@"; do [ "$a" = --check-args ] && { echo "check-args: refused" >&2; exit 2; }; done\n'
                    f"echo \"$0 $*\" >> {tmp_path / 'started'}\nexit 0\n")
    fake.chmod(0o755)
    p = _launch(tmp_path, venv_py=fake, DRY_RUN="0")
    assert p.returncode == 2 and "refused" in p.stdout and not (tmp_path / "started").exists()
