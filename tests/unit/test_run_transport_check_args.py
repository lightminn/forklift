"""The runner's preflight (docs/plans/2026-10-27-ws1-preflight.md): --check-args makes every check a run makes before
Isaac and ends there -- ARGS_OK and 0, or 2 and why -- without importing Isaac or writing its output."""
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/isaac"))

# isaacsim made unimportable: a check that reached for Isaac fails instead of starting it
RUN = ("import runpy, sys; sys.modules['isaacsim'] = None; sys.argv = sys.argv[1:]; "
       "runpy.run_path(sys.argv[0], run_name='__main__')")


def _base(out):
    return ["--base-scene", "scene.usda", "--output", str(out), "--seed", "0",
            "--forklift-urdf", str(ROOT / "sim/models/dls08_measured/forklift.urdf"),
            "--pallet-urdf", str(ROOT / "sim/models/epal6_pallet/pallet.urdf"),
            "--pallet-geometry", str(ROOT / "config/pallet_geometry_epal6.yaml"),
            "--settings", str(ROOT / "config/isaac_transport_measured.yaml"),
            "--use-perception", "--planning-target", "perception", "--record-slam",
            "--pallet-prior", str(ROOT / "config/pallet_prior_epal6.yaml"),
            "--obstacle-layer", str(ROOT / "config/obstacle_layer_single.yaml"), "--obstacle-act"]


def _check(*argv):
    env = {**os.environ, "PYTHONPATH": str(ROOT / "src")}
    return subprocess.run([sys.executable, "-c", RUN, str(ROOT / "sim/isaac/run_transport.py"), *argv, "--check-args"],
                          capture_output=True, text=True, timeout=120, env=env, cwd=ROOT)


def test_good_arguments_pass_without_isaac_or_an_output(tmp_path):
    out = tmp_path / "run"
    p = _check(*_base(out))
    assert p.returncode == 0, p.stderr
    assert p.stdout.strip().splitlines()[-1] == "ARGS_OK" and not out.exists()


def test_an_existing_output_and_a_misspelt_option_are_refused(tmp_path):
    out = tmp_path / "run"
    out.mkdir()
    p = _check(*_base(out))
    assert p.returncode == 2 and "exists" in p.stderr
    p = _check(*_base(tmp_path / "fresh"), "--recognition-threadz", "4")
    assert p.returncode == 2 and "--recognition-threadz" in p.stderr and "ARGS_OK" not in p.stdout


def test_a_session_scene_without_the_redraw_is_refused_as_the_run_would(tmp_path):
    # d7oor 11728 (2026-10-27): the runner refused this three epochs in a row, the job held its shard 15 min
    from tools import riibotics_isaac_scenes as tool

    scene = tmp_path / "line_session.json"
    scene.write_text(json.dumps(tool.session_record("line")))
    p = _check(*_base(tmp_path / "run"), "--layout", "scene", "--scene-file", str(scene), "--live")
    assert p.returncode == 2 and "needs --near-capture-redraw always" in p.stderr
    p = _check(*_base(tmp_path / "run"), "--layout", "scene", "--scene-file", str(scene), "--live",
               "--near-capture-redraw", "always")
    assert p.returncode == 0, p.stderr


def test_the_check_flag_is_never_recorded():
    # the near-field contract compares every recorded argument with the measurement runs' (Codex preflight review)
    import run_transport

    argv = sys.argv
    sys.argv = ["run_transport.py", *_base("out")]
    try:
        args = run_transport.arguments()
    finally:
        sys.argv = argv
    assert args.check_args is False and "check_args" not in run_transport.recorded_arguments(args)
    args.check_args = True
    assert "check_args" not in run_transport.recorded_arguments(args)
