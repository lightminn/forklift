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


def test_a_settings_table_without_the_runs_pallet_shape_is_refused(tmp_path):
    # plan 2026-11-01-pallet-mass: the table is read by the geometry's version; a shape it lacks never reaches Isaac
    settings = tmp_path / "settings.yaml"
    text = (ROOT / "config/isaac_transport_measured.yaml").read_text()
    settings.write_text(text.replace("pallet_mass_kg: {epal6: 4.93, ", "pallet_mass_kg: {"))
    argv = [a if a != str(ROOT / "config/isaac_transport_measured.yaml") else str(settings)
            for a in _base(tmp_path / "run")]
    p = _check(*argv)
    assert p.returncode == 2 and "no entry for the pallet shape 'epal6'" in p.stderr


def test_the_line2_robustness_options_are_recorded_only_when_set():
    # docs/plans/2026-11-04-line2-delivery-robustness.md D6: off runs record what they always did
    import run_transport

    argv = sys.argv
    sys.argv = ["run_transport.py", *_base("out")]
    try:
        args = run_transport.arguments()
    finally:
        sys.argv = argv
    assert (args.docking_degenerate_continue, args.inject_round1_degenerate, args.destination_prior_error_along_m) == (
        "off", False, 0.0)
    recorded = run_transport.recorded_arguments(args)
    assert not {"docking_degenerate_continue", "inject_round1_degenerate",
                "destination_prior_error_along_m"} & set(recorded)
    args.docking_degenerate_continue, args.inject_round1_degenerate, args.destination_prior_error_along_m = (
        "on", True, 0.08)
    recorded = run_transport.recorded_arguments(args)
    assert (recorded["docking_degenerate_continue"], recorded["inject_round1_degenerate"],
            recorded["destination_prior_error_along_m"]) == ("on", True, 0.08)


def test_the_line2_stress_tools_are_batch_only(tmp_path):
    p = _check(*_base(tmp_path / "run"), "--inject-round1-degenerate", "--live")
    assert p.returncode == 2 and "batch-run tools" in p.stderr
    p = _check(*_base(tmp_path / "run"), "--destination-prior-error-along-m", "0.08")
    assert p.returncode == 0, p.stderr
    p = _check(*_base(tmp_path / "run"), "--docking-degenerate-continue", "on")
    assert p.returncode == 2 and "needs --d7-docking" in p.stderr


def test_the_g5_0_prep_arms_need_a_live_run_and_one_arm_at_a_time(tmp_path):
    # real-time plan 5.24: test-only arms; a batch run refuses them, and renders and a wait are separate arms
    p = _check(*_base(tmp_path / "a"), "--live-test-prep-renders", "1")
    assert p.returncode == 2 and "need --live" in p.stderr
    p = _check(*_base(tmp_path / "b"), "--live", "--live-test-prep-renders", "1", "--live-test-prep-wait-s", "3.5")
    assert p.returncode == 2 and "separate arms" in p.stderr
    p = _check(*_base(tmp_path / "c"), "--live-test-prep-renders", "4")
    assert p.returncode == 2


def test_the_async_capture_dump_needs_a_live_run(tmp_path):
    # real-time plan G2-1a: a batch run writes its capture files in place; async is refused, sync accepted
    p = _check(*_base(tmp_path / "a"), "--live-capture-dump", "async")
    assert p.returncode == 2 and "--live-capture-dump async needs --live" in p.stderr
    p = _check(*_base(tmp_path / "b"), "--live-capture-dump", "sync")
    assert p.returncode == 0, p.stderr


def test_the_latency_options_are_checked_before_isaac(tmp_path):
    """Latency test (docs/plans/2026-11-11-latency-tolerance.md): A batch only, B live only, bad specs refused."""
    slam = ["--slam-feedback", str(tmp_path / "s.sock")]
    ok = _check(*_base(tmp_path / "a"), *slam, "--latency-slam", "ticks:6", "--latency-layer", "undeclared:12",
                "--latency-seed", "3")
    assert ok.returncode == 0, ok.stderr
    for extra, why in (
        (["--latency-slam", "ticks:6"], "--latency-slam needs the SLAM feedback"),
        ([*slam, "--latency-slam", "late:3"], "--latency-slam"),
        (["--latency-layer", "undeclared:13"], "N out of range"),
        (["--latency-layer", "jitter:12", "--layer-worker", "delay"], "--layer-worker"),  # refused either way
        (["--latency-plan-ms", "50"], "live only"),
    ):
        p = _check(*_base(tmp_path / f"r{len(why)}"), *extra)
        assert p.returncode == 2 and why in p.stderr, (extra, p.stderr[-400:])


def test_the_latency_options_are_registered_in_the_near_contract():
    text = (ROOT / "sim/isaac/near_tracking.py").read_text()
    assert '"latency_slam": "off", "latency_layer": "off", "latency_seed": None,' in text
    assert '"latency_plan_ms": 0.0,' in text


def test_the_layer_latency_budget_file_is_checked_before_isaac(tmp_path):
    """The layer latency budget (docs/plans/2026-11-11-layer-latency-budget.md): its own file, read before Isaac."""
    ok = _check(*_base(tmp_path / "a"), "--layer-latency-budget", str(ROOT / "config/layer_latency_budget.yaml"))
    assert ok.returncode == 0, ok.stderr
    bad = tmp_path / "bad.yaml"
    for i, (text, why) in enumerate([("latency_budget_s: 0.1\n", "exactly"),
                                     ("latency_budget_s: -0.1\noverdue_end_s: 1.0\n", ">= 0"),
                                     ("latency_budget_s: 0.1\noverdue_end_s: 0\n", "> 0")]):
        bad.write_text(text)
        p = _check(*_base(tmp_path / f"b{i}"), "--layer-latency-budget", str(bad))
        assert p.returncode == 2 and why in p.stderr, (text, p.stderr)
    p = _check(*_base(tmp_path / "c"), "--layer-latency-budget", str(tmp_path / "none.yaml"))
    assert p.returncode == 2 and "no such file" in p.stderr
    p = _check(*_base(tmp_path / "d"), "--layer-latency-budget-s", "-1")
    assert p.returncode == 2 and "finite number >= 0" in p.stderr


def test_the_launchers_pass_the_layer_latency_budget_and_the_layer_files_stay_unbudgeted():
    import yaml

    doc = yaml.safe_load((ROOT / "config/layer_latency_budget.yaml").read_text())
    assert doc == {"latency_budget_s": 0.10, "overdue_end_s": 1.0}  # the lead's decision 2026-11-11
    for launcher in ("deploy/console/isaac_live.sbatch", "deploy/console/slam_survey.sbatch",
                     "deploy/riibotics/riib_scene.sbatch"):
        text = (ROOT / launcher).read_text()
        assert "LAYER_BUDGET=${LAYER_BUDGET:-config/layer_latency_budget.yaml}" in text, launcher
        assert '--obstacle-layer "$LAYER" ${BUDGET[@]+"${BUDGET[@]}"}' in text, launcher
    # survey map bundles hash the obstacle-layer file as a shared input: the budget is never written into one
    for layer in sorted((ROOT / "config").glob("obstacle_layer_single*.yaml")):
        assert "latency_budget_s" not in yaml.safe_load(layer.read_text()), layer
