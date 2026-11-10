"""The G3 scan-step switches as operating defaults (real-time plan 5.26-5.27, lead decision 2026-11-10): the console
launcher runs FORKLIFT_GRID_CLEAR_KERNEL=1, FORKLIFT_SHADOW_KERNEL=1 and --lidar-cast fast unless told otherwise; an
explicit value and an EXTRA_ARGS --lidar-cast win; a --near-tracking run keeps all three off; the survey and the
measurement wrapper keep them off; the runner records what it applied."""
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
    root = Path(tempfile.mkdtemp(dir="/tmp", prefix="ssd"))
    yield root
    shutil.rmtree(root, ignore_errors=True)


def _launch(tmp_path, **env):
    """DRY_RUN with a stand-in Python: ``-c`` snippets run for real; the runner's check writes the FORKLIFT_ switches
    it was started with and its arguments, then passes."""
    bin_ = tmp_path / "bin"
    bin_.mkdir(exist_ok=True)
    docker = bin_ / "docker"
    docker.write_text(f"#!/bin/bash\necho \"$0 $*\" >> {tmp_path / 'started'}\n")
    docker.chmod(0o755)
    fake = tmp_path / "fakepy"
    fake.write_text(f"#!/bin/bash\nif [ \"$1\" = -c ]; then exec {sys.executable} \"$@\"; fi\n"
                    f"env | grep -E '^FORKLIFT_(GRID_CLEAR|SHADOW)_KERNEL=' | sort > {tmp_path / 'runner_env'}\n"
                    f"printf '%s\\n' \"$@\" > {tmp_path / 'runner_args'}\necho ARGS_OK\n")
    fake.chmod(0o755)
    base = {"PATH": f"{bin_}:{os.environ['PATH']}", "BASE_SCENE": "scene.usda",
            "FORKLIFT_URDF": str(ROOT / "sim/models/dls08_measured/forklift.urdf"), "VENV_PY": str(fake),
            "PYTHONPATH": str(ROOT / "src"), "MAP_BUNDLE": "none", "DRY_RUN": "1"}
    base.update(env)
    p = subprocess.run(["bash", str(LAUNCHER), str(tmp_path / "out")], cwd=ROOT, env=base, capture_output=True,
                       text=True, timeout=180)
    return p


def _env_sh(tmp_path) -> dict:
    out = {}
    for line in (tmp_path / "out" / "env.sh").read_text().splitlines():
        k, _, v = line.partition("=")
        out[k] = v
    return out


def _runner(tmp_path):
    env = dict(line.split("=", 1) for line in (tmp_path / "runner_env").read_text().split())
    args = (tmp_path / "runner_args").read_text().splitlines()
    casts = [args[i + 1] if a == "--lidar-cast" else a.split("=", 1)[1]  # in order: argparse keeps the last
             for i, a in enumerate(args) if a == "--lidar-cast" or a.startswith("--lidar-cast=")]
    return env, args, casts


def test_the_console_runs_the_three_switches_on_by_default(tmp_path):
    p = _launch(tmp_path)
    assert p.returncode == 0, p.stdout + p.stderr
    assert "SCAN_SWITCHES grid_clear_kernel=1 shadow_kernel=1 lidar_cast=fast" in p.stdout
    e = _env_sh(tmp_path)
    assert (e["FORKLIFT_GRID_CLEAR_KERNEL"], e["FORKLIFT_SHADOW_KERNEL"], e["LIDAR_CAST"]) == ("1", "1", "fast")
    env, args, casts = _runner(tmp_path)
    assert env == {"FORKLIFT_GRID_CLEAR_KERNEL": "1", "FORKLIFT_SHADOW_KERNEL": "1"}
    assert casts == ["fast"] and args[-1] == "--check-args"
    assert not (tmp_path / "started").exists()


def test_an_explicit_value_wins_and_a_measurement_list_s_zero_stays_zero(tmp_path):
    p = _launch(tmp_path, FORKLIFT_GRID_CLEAR_KERNEL="0", FORKLIFT_SHADOW_KERNEL="1", LIDAR_CAST="loop")
    assert p.returncode == 0, p.stdout + p.stderr
    assert "SCAN_SWITCHES grid_clear_kernel=0 shadow_kernel=1 lidar_cast=loop" in p.stdout
    env, _, casts = _runner(tmp_path)
    assert env == {"FORKLIFT_GRID_CLEAR_KERNEL": "0", "FORKLIFT_SHADOW_KERNEL": "1"} and casts == ["loop"]


@pytest.mark.parametrize("extra, final", [("--lidar-cast oracle", "oracle"), ("--trace\t--lidar-cast=loop", "loop"),
                                          ("--lidar-cast fast\n--lidar-cast loop", "loop"),
                                          ("--lidar-cast=oracle --lidar-cast fast", "fast")])
def test_an_extra_args_lidar_cast_comes_later_and_wins(tmp_path, extra, final):
    p = _launch(tmp_path, EXTRA_ARGS=extra)
    assert p.returncode == 0, p.stdout + p.stderr
    assert f"lidar_cast={final} (EXTRA_ARGS)" in p.stdout
    _, _, casts = _runner(tmp_path)
    assert casts[0] == "fast" and casts[-1] == final  # the launcher's first, the list's last: argparse keeps the last


def test_a_near_tracking_run_keeps_all_three_off_and_refuses_an_explicit_on(tmp_path):
    _launch(tmp_path, EXTRA_ARGS="--near-tracking")  # the runner's own check may refuse a bare --near-tracking
    e = _env_sh(tmp_path)
    assert (e["FORKLIFT_GRID_CLEAR_KERNEL"], e["FORKLIFT_SHADOW_KERNEL"], e["LIDAR_CAST"]) == ("0", "0", "loop")
    for env in ({"FORKLIFT_GRID_CLEAR_KERNEL": "1"}, {"FORKLIFT_SHADOW_KERNEL": "1"}, {"LIDAR_CAST": "fast"},
                {"EXTRA_ARGS": "--near-tracking --lidar-cast=fast"}):
        p = _launch(tmp_path, **{"EXTRA_ARGS": "--near-tracking", **env})
        assert p.returncode == 2 and "--near-tracking keeps" in p.stdout, (env, p.stdout + p.stderr)


@pytest.mark.parametrize("extra", ["--lidar-c oracle", "--lidar-cas=loop", "--near-track", "--lidar-cast oracle --lidar-c loop"])
def test_an_abbreviation_argparse_would_take_is_refused(tmp_path, extra):
    p = _launch(tmp_path, EXTRA_ARGS=extra)
    assert p.returncode == 2 and "in full" in p.stdout, p.stdout + p.stderr


@pytest.mark.parametrize("env, message", [({"FORKLIFT_GRID_CLEAR_KERNEL": "on"}, "must be 0 or 1"),
                                          ({"FORKLIFT_SHADOW_KERNEL": "2"}, "must be 0 or 1"),
                                          ({"LIDAR_CAST": "batch"}, "loop, fast or oracle"),
                                          ({"EXTRA_ARGS": "--lidar-cast=batch"}, "loop, fast or oracle")])
def test_anything_else_is_refused(tmp_path, env, message):
    p = _launch(tmp_path, **env)
    assert p.returncode == 2 and message in p.stdout, p.stdout + p.stderr


def test_the_survey_and_the_measurement_wrapper_keep_them_off():
    survey = (ROOT / "deploy/console/slam_survey.sbatch").read_text()
    assert "printf 'FORKLIFT_GRID_CLEAR_KERNEL=0\\nFORKLIFT_SHADOW_KERNEL=0\\nLIDAR_CAST=loop\\n'" in survey
    wrapper = (ROOT / "docs/validation/2026-11-08-r1-confirmation/r1c_session.sh").read_text()
    assert ("export FORKLIFT_GRID_CLEAR_KERNEL=${FORKLIFT_GRID_CLEAR_KERNEL:-0} "
            "FORKLIFT_SHADOW_KERNEL=${FORKLIFT_SHADOW_KERNEL:-0}") in wrapper
    assert "export LIDAR_CAST=${LIDAR_CAST:-loop}" in wrapper


def test_the_runner_records_what_it_applied(monkeypatch):
    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    import planar_lidar
    import run_transport
    from forklift_core import kernels
    from forklift_core.control import shadow_memory
    from forklift_core.perception import obstacle_grid

    monkeypatch.setattr(obstacle_grid, "CLEAR_KERNEL", True)
    monkeypatch.setattr(shadow_memory, "SHADOW_KERNEL", True)
    monkeypatch.setattr(planar_lidar, "CAST_MODE", "fast")
    mode = kernels.mode()
    try:
        kernels.set_mode("numpy")
        r = run_transport.scan_switches_record()
        assert (r["grid_clear_kernel"], r["shadow_kernel"], r["shadow_arrays_enabled"], r["kernels_mode"],
                r["lidar_cast"]) == (True, True, True, "numpy", "fast")
        assert r["numpy"]
        kernels.set_mode("reference")  # the shadow memory's array path does not run in reference mode
        assert run_transport.scan_switches_record()["shadow_arrays_enabled"] is False
    finally:
        kernels.set_mode(mode)
