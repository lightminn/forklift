"""The path display fix's contract (docs/plans/2026-11-14-path-display-perception-leak.md 2-4): how a run records the
mode, how the near-field and D8b comparisons read it (either way round), the diagnostic stage refused by the runner, the
operating launchers and a bundle's check, and the runner's wiring."""
import argparse
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/isaac"))
sys.path.insert(0, str(ROOT / "tools"))
SCRIPT = ROOT / "sim/isaac/run_transport.py"

sys.path.insert(0, str(ROOT))
from sim.isaac import near_tracking as NT  # noqa: E402 -- as s4_drive_terms imports it


def test_stage_records_nothing_and_the_others_record_their_mode():
    import run_transport

    assert run_transport.recorded_arguments(argparse.Namespace(seed=0, path_display="stage")) == {"seed": 0}
    for mode in ("overlay", "off", "both"):
        got = run_transport.recorded_arguments(argparse.Namespace(seed=0, path_display=mode))
        assert got == {"seed": 0, "path_display": mode}


@pytest.mark.parametrize("mode", ["overlay", "off"])
def test_the_near_field_comparison_refuses_a_run_across_the_fix_either_way_round(mode):
    before = {"arguments": {"fps": 60}}  # a measurement run before the fix (or a new stage run)
    after = {"arguments": {"fps": 60, "path_display": mode}}
    for expected, observed in ((before, after), (after, before)):
        (problem,) = NT.condition_problems(expected, observed)
        assert problem.startswith("arguments path_display differs from the measurement runs")
        assert "intended" in problem and "2026-11-14-path-display-perception-leak" in problem
    assert NT.condition_problems(before, {"arguments": {"fps": 60}}) == []
    assert NT.condition_problems(after, {"arguments": {"fps": 60, "path_display": mode}}) == []
    assert "path_display" not in NT.RUN_SPECIFIC_ARGUMENTS and "path_display" not in NT.POST_MEASUREMENT_ARGUMENTS


@pytest.mark.parametrize("mode", ["overlay", "off"])
def test_the_d8b_reference_check_reads_the_mode_the_same_way(mode):
    import s4_drive_terms

    before = {"arguments": {"fps": 60}, "slam_feedback": {"x": 1}}
    after = {"arguments": {"fps": 60, "path_display": mode}, "slam_feedback": {"x": 1}}
    for run, reference in ((before, after), (after, before)):
        (problem,) = s4_drive_terms.reference_problems("k", run, reference)
        assert problem.startswith("k: argument path_display differs from the D8b run") and "intended" in problem
    assert s4_drive_terms.reference_problems("k", before, dict(before)) == []


# as tests/unit/test_run_transport_check_args.py: isaacsim unimportable, the runner's own preflight only
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


def _check_args(env_extra, *extra):
    env = {**{k: v for k, v in os.environ.items() if k != "FORKLIFT_PATH_DISPLAY_DIAGNOSTIC"},
           "PYTHONPATH": str(ROOT / "src"), **env_extra}
    return subprocess.run([sys.executable, "-c", RUN, str(SCRIPT), *_base(Path("/nonexistent/pd_run")), *extra,
                           "--check-args"], capture_output=True, text=True, timeout=120, env=env, cwd=ROOT)


def test_the_runner_refuses_stage_without_the_diagnostic_variable():
    p = _check_args({}, "--path-display", "stage")
    assert p.returncode == 2 and "diagnostic only" in p.stderr and "ARGS_OK" not in p.stdout
    p = _check_args({"FORKLIFT_PATH_DISPLAY_DIAGNOSTIC": "stage"}, "--path-display", "stage")
    assert p.returncode == 0, p.stderr
    p = _check_args({"FORKLIFT_PATH_DISPLAY_DIAGNOSTIC": "maybe"})
    assert p.returncode == 2 and "FORKLIFT_PATH_DISPLAY_DIAGNOSTIC" in p.stderr
    assert _check_args({}).returncode == 0  # overlay by default


LAUNCHERS = {
    "deploy/console/isaac_live.sbatch": "refuse",
    "deploy/console/slam_survey.sbatch": "refuse",
    "deploy/riibotics/riib_scene.sbatch": "pass",
}


def _guard(launcher):
    text = (ROOT / launcher).read_text()
    start = text.index('pd_prev=""')
    end = text.index("fi\n", text.index('if [ -n "${FORKLIFT_PATH_DISPLAY_DIAGNOSTIC:-}" ]', start)) + 3
    return text[start:end]


@pytest.mark.parametrize("launcher", sorted(LAUNCHERS))
def test_the_operating_launchers_refuse_stage(launcher):
    guard = _guard(launcher) + "echo PASSED\n"
    for words in ("--path-display stage", "--path-display=stage", "--path-d stage", "--seed 1 --path-disp=stage"):
        p = subprocess.run(["bash", "-c", guard], env={"EXTRA_ARGS": words, "PATH": os.environ["PATH"]},
                           capture_output=True, text=True)
        assert p.returncode == 2 and "diagnostic only" in p.stdout, words
    for words in ("", "--path-display overlay", "--seed 1 stage", "--path-display off"):
        p = subprocess.run(["bash", "-c", guard], env={"EXTRA_ARGS": words, "PATH": os.environ["PATH"]},
                           capture_output=True, text=True)
        assert p.returncode == 0 and "PASSED" in p.stdout, words
    p = subprocess.run(["bash", "-c", guard], capture_output=True, text=True,
                       env={"EXTRA_ARGS": "", "PATH": os.environ["PATH"], "FORKLIFT_PATH_DISPLAY_DIAGNOSTIC": "both"})
    if LAUNCHERS[launcher] == "refuse":
        assert p.returncode == 2 and "diagnostic only" in p.stdout
    else:  # the riib launcher passes it on, said aloud (the S3 alignment smoke; a bundle's check refuses it)
        assert p.returncode == 0 and "PATH_DISPLAY_DIAGNOSTIC both" in p.stdout


def test_a_bundle_refuses_the_diagnostic_variable(tmp_path):
    import isaac_bundle as ib

    RUNNER_SRC = 'ap.add_argument("--seed")\n'  # as test_isaac_bundle's check cases
    LAUNCHER = ('if [ "$DRY_RUN" = 1 ]; then echo RUNNER_ARGS; printf "%s\\n" --seed "$SEED"; echo ARGS_OK; exit 0; fi;'
                ' exit 9\n')

    (tmp_path / "runner.py").write_text(RUNNER_SRC)
    (tmp_path / "launch.sh").write_text(LAUNCHER)
    out = tmp_path / "out_a"
    run = ib.Run(name="a", argv=["bash", str(tmp_path / "launch.sh"), str(out)],
                 env={"SEED": "1", "EXTRA": "", ib.PATH_DISPLAY_DIAGNOSTIC_ENV: "stage"}, retry_move=[str(out)])
    (problem,) = ib.check_runs([run], tmp_path / "runner.py")
    assert "diagnostic only" in problem
    run = ib.Run(name="a", argv=run.argv, env={"SEED": "1", "EXTRA": ""}, retry_move=[str(out)])
    (problem,) = ib.check_runs([run], tmp_path / "runner.py", environ={**os.environ, ib.PATH_DISPLAY_DIAGNOSTIC_ENV: "both"})
    assert "diagnostic only" in problem
    assert ib.PATH_DISPLAY_DIAGNOSTIC_ENV == "FORKLIFT_PATH_DISPLAY_DIAGNOSTIC"


def test_the_runner_draws_the_display_by_mode_and_the_inset_stays_clear():
    source = SCRIPT.read_text()
    # every planned path the core or the runner shows goes through the mode's adapter, never straight to the scene
    assert "add_path_display=display_path," in source
    assert source.count("add_path_display(stage_, path_, name_, colour_)") == 1
    assert "path_overlay.add_path(name_, path_.poses, colour_)" in source
    # the destination rings likewise; the state keeps the prim path either way
    assert source.count("add_destination(stage, x_, y_, path=path_)") == 1
    assert 'state["destination_marker"] = destination_ring(' in source
    # the overview: lines first, then the inset panel (the perception camera's own picture, no line)
    overview = source.index("video_rgb = path_overlay.compose(rgba[:, :, :3], camera)")
    assert overview < source.index("video_rgb = inset.compose_frame(", overview)
    assert "live = frame_sync_rgba(perception_camera)" in source  # the inset is still the sensor's own picture
    # the robot camera: lines under the detection drawing; the live stream: lines before the halving
    robot = source.index("colour = path_overlay.compose(np.asarray(rgb)[:, :, :3], camera,")
    assert robot < source.index("colour = video_frames_module.annotate_detection(", robot)
    live = source.index("rgb_ = np.asarray(path_overlay.compose(rgba_[:, :, :3], live_cam_,")
    assert live < source.index("while rgb_.shape[1] > 640:", live)
    # the perception view: the picture half only (before the depth half joins it)
    view = source.index("path_overlay.compose(view_rgba[:, :, :3], view_camera,")
    assert view < source.index("MISSION_VIEWS.depth_colormap(depth)", view)
    # every physics step keeps the moving cameras' poses (a no-op in a deterministic run: nothing registered)
    step = source.index("world.step(render=render or frame_due or far_due)")
    assert source.index("path_overlay.note_moving()", step) - step < 120
    assert source.count("path_overlay.register_moving(") == 2 and source.count("if not args.deterministic:") >= 2
