"""The live launcher's preflight (docs/plans/2026-10-27-ws1-preflight.md): DRY_RUN=1 resolves the SLAM mode, prints
the runner's arguments and ends with the runner's own check, starting no container, supervisor or Isaac; a real launch
makes the same check first and ends at once when it fails (lvram, d7oor 2026-10-27)."""
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
LAUNCHER = ROOT / "deploy/console/isaac_live.sbatch"


def _launch(tmp_path, venv_py=sys.executable, out=None, **env):
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
    return subprocess.run(["bash", str(LAUNCHER), str(out or tmp_path / "out")], cwd=ROOT, env=base, capture_output=True,
                          text=True, timeout=180)


def test_the_dry_run_prints_the_arguments_and_the_runner_s_verdict_and_starts_nothing(tmp_path):
    p = _launch(tmp_path)
    assert p.returncode == 0, p.stdout + p.stderr
    lines = p.stdout.splitlines()
    assert "RUNNER_ARGS" in lines and lines[-1] == "ARGS_OK"
    assert "--live" in lines and "--check-args" not in lines  # the run's own arguments, the check added after them
    assert not (tmp_path / "started").exists() and not (tmp_path / "out" / "live_port.json").exists()


def test_the_far_capture_is_bound_to_the_mission_by_default(tmp_path):
    # far target binding F1 (lead's exception decision 2026-10-28): on unless FAR_EXPECTED=off; anything else refused
    def far(lines):
        i = lines.index("--far-capture-expected")
        return lines[i + 1]

    p = _launch(tmp_path)
    assert p.returncode == 0 and far(p.stdout.splitlines()) == "on", p.stdout + p.stderr
    p = _launch(tmp_path, FAR_EXPECTED="off")
    assert p.returncode == 0 and far(p.stdout.splitlines()) == "off", p.stdout + p.stderr
    p = _launch(tmp_path, FAR_EXPECTED="maybe")
    assert p.returncode == 2 and "FAR_EXPECTED must be on or off" in p.stdout + p.stderr


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


def test_an_out_too_long_for_the_slam_socket_is_refused(tmp_path):
    # the lvram random_m refill (11840) died at connect: <run>/live/epoch_1/sock/slam.sock was 112 bytes
    out = tmp_path / ("x" * max(1, 110 - len(str(tmp_path))))
    p = _launch(tmp_path, out=out)
    assert p.returncode == 2 and "AF_UNIX" in p.stderr, p.stdout + p.stderr


def _fake_python(tmp_path):
    """The Isaac Python stand-in: inline code runs (a bundle load answers a map id), the runner's check passes, and
    anything else (the supervisor, the stall watch) is recorded instead of started."""
    fake = tmp_path / "fakepy"
    fake.write_text("#!/bin/bash\n"
                    "if [ \"$1\" = -c ]; then case \"$2\" in *load_bundle*) echo " + "f" * 64 + "; exit 0;; esac;"
                    f" exec {sys.executable} \"$@\"; fi\n"
                    # the registry's lookup runs, under this test's interpreter (ws1's python3 has no numpy)
                    f'case "$1" in */slam_bundle_registry.py) exec {sys.executable} "$@";; esac\n'
                    'for a in "$@"; do [ "$a" = --check-args ] && { echo ARGS_OK; exit 0; }; done\n'
                    f"printf '%s\\n' \"$*\" >> {tmp_path / 'started'}\nexit 0\n")
    fake.chmod(0o755)
    return fake


def _relocalizing(tmp_path, **env):
    bundle = tmp_path / "bundle"
    bundle.mkdir(parents=True, exist_ok=True)
    return _launch(tmp_path, venv_py=_fake_python(tmp_path), DRY_RUN="0", MAP_BUNDLE=str(bundle),
                   MAP_START_POSE="0.5,-1.0,0.3", **env)


def _bridge(tmp_path, *extra):
    return subprocess.run(["bash", str(tmp_path / "out" / "bridge.sh"), "1", str(tmp_path / "e1"), "m" * 64,
                           str(tmp_path / "bundle"), *extra],
                          env={**os.environ, "PATH": f"{tmp_path / 'bin'}:{os.environ['PATH']}"}, capture_output=True)


def _supervisor(tmp_path) -> str:
    return next(line for line in (tmp_path / "started").read_text().splitlines() if "live_supervisor" in line)


def test_a_start_pose_relocalizes_the_session(tmp_path):
    # B-P2b (start relocalization plan 1.2): the runner placed and relocalizing, the supervisor running it first, the
    # bridge started at the runner's estimate
    p = _relocalizing(tmp_path, START_POSE="-4.0,3.0,-2.0")
    assert p.returncode == 0, p.stdout + p.stderr
    out = tmp_path / "out"
    preflight = (out / "preflight.log").read_text().splitlines()
    i = preflight.index("--start-pose=-4.0,3.0,-2.0")  # one word: a negative x is no option (Codex B-P2b P2)
    assert preflight[i:i + 6] == ["--start-pose=-4.0,3.0,-2.0", "--start-relocalize", "--reloc-dir",
                                  str(out / "preflight"), "--reloc-operator-timeout-s", "540"]
    assert "--reloc-unattended" not in preflight  # B-P2c: the operator answers on the console by default
    supervisor = _supervisor(tmp_path)
    assert "--relocalize" in supervisor.split() and '"{map_start_pose}"' in supervisor
    words = supervisor.split()
    assert words[words.index("--reloc-wait-s") + 1] == "600"
    (tmp_path / "started").unlink()
    assert _bridge(tmp_path, "7,8,1").returncode == 0
    assert "SLAM_MAP_START_POSE=7,8,1" in (tmp_path / "started").read_text()  # the estimate, not the registry's
    assert _bridge(tmp_path).returncode != 0  # no estimate: no bridge


def test_the_bridge_runs_the_named_image_and_records_the_one_it_ran(tmp_path):
    # Codex B-P2d freeze review P2: a mutable tag checked before and after a collection misses A -> B -> A; the
    # image is named by digest for a judged batch and each epoch keeps the image id its container ran
    p = _relocalizing(tmp_path, START_POSE="-4.0,3.0,-2.0", BRIDGE_IMAGE="forklift/ros2-dev@sha256:" + "e" * 64)
    assert p.returncode == 0, p.stdout + p.stderr
    docker = tmp_path / "bin" / "docker"
    docker.write_text(f"#!/bin/bash\necho \"$0 $*\" >> {tmp_path / 'started'}\n"
                      "[ \"$1\" = inspect ] && echo sha256:" + "e" * 64 + "\nexit 0\n")
    assert _bridge(tmp_path, "7,8,1").returncode == 0
    runs = [line for line in (tmp_path / "started").read_text().splitlines() if " run " in line]
    assert runs and ("forklift/ros2-dev@sha256:" + "e" * 64) in runs[-1].split()
    assert (tmp_path / "e1" / "bridge" / "image_id").read_text().strip() == "sha256:" + "e" * 64
    _launch(tmp_path, venv_py=_fake_python(tmp_path), out=tmp_path / "out2")  # unset: the tag, as before
    assert "BRIDGE_IMAGE=forklift/ros2-dev:jazzy\n" in (tmp_path / "out2" / "env.sh").read_text()


def test_without_a_start_pose_the_session_starts_as_before(tmp_path):
    p = _relocalizing(tmp_path)
    assert p.returncode == 0, p.stdout + p.stderr
    assert "--start-relocalize" not in (tmp_path / "out" / "preflight.log").read_text().splitlines()
    supervisor = _supervisor(tmp_path)
    assert "--relocalize" not in supervisor.split() and "map_start_pose" not in supervisor
    (tmp_path / "started").unlink()
    assert _bridge(tmp_path).returncode == 0
    assert "SLAM_MAP_START_POSE=0.5,-1.0,0.3" in (tmp_path / "started").read_text()


def test_an_unattended_relocalization_and_its_wait(tmp_path):
    # B-P2c: unattended batches end at the operator's request; the operator's time is the supervisor's wait less 60 s
    p = _relocalizing(tmp_path, START_POSE="4.0,3.0,-2.0", RELOC_UNATTENDED="1", RELOC_WAIT_S="300")
    assert p.returncode == 0, p.stdout + p.stderr
    preflight = (tmp_path / "out" / "preflight.log").read_text().splitlines()
    assert "--reloc-unattended" in preflight
    assert preflight[preflight.index("--reloc-operator-timeout-s") + 1] == "240"
    words = _supervisor(tmp_path).split()
    assert words[words.index("--reloc-wait-s") + 1] == "300"
    for bad in ("60", "x", "-5"):  # (empty is unset: the 600 s default)
        q = _relocalizing(tmp_path, START_POSE="4.0,3.0,-2.0", RELOC_WAIT_S=bad)
        assert q.returncode == 2 and "RELOC_WAIT_S" in q.stdout + q.stderr


def test_max_epochs_ends_the_session_after_that_many_epochs(tmp_path):
    # B-P2d: one validation session is one epoch (a failure is the session's outcome, never retried)
    p = _relocalizing(tmp_path, START_POSE="4.0,3.0,-2.0", MAX_EPOCHS="1")
    assert p.returncode == 0, p.stdout + p.stderr
    words = _supervisor(tmp_path).split()
    assert words[words.index("--max-epochs") + 1] == "1"
    (tmp_path / "started").unlink()
    p = _relocalizing(tmp_path, START_POSE="4.0,3.0,-2.0")
    assert "--max-epochs" not in _supervisor(tmp_path).split()  # the console keeps its epochs
    for bad in ("0", "x", "12oops", "01"):
        q = _relocalizing(tmp_path, MAX_EPOCHS=bad)
        assert q.returncode == 2 and "MAX_EPOCHS" in q.stdout + q.stderr


def test_a_start_pose_needs_a_survey_map(tmp_path):
    p = _launch(tmp_path, START_POSE="4.0,3.0,-2.0")  # MAP_BUNDLE=none: online mapping
    assert p.returncode == 2 and "START_POSE relocalizes on a survey map" in p.stderr


# --- the start relocalization as the operator console's default (lead 2026-10-31, start relocalization plan 7.1) ---
def _registry_launch(tmp_path, relocalizable=True, **env):
    """A launch on the registry's survey map of a scene file (deploy/console/slam_choice.sh's default path), its row
    marked relocalizable at its start unless ``relocalizable`` is false."""
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from test_slam_choice import _registry

    tmp_path.mkdir(parents=True, exist_ok=True)
    scene, book = _registry(tmp_path, relocalizable=relocalizable)  # its row's start: [1.5, -2.0, 0.25]
    (tmp_path / "root" / "line_m_tsq_aaaaaaaaaaaa").mkdir(parents=True, exist_ok=True)
    base = {"SCENE_FILE": str(scene), "SLAM_BUNDLE_ROOT": str(tmp_path / "root"), "SLAM_BUNDLE_REGISTRY": str(book)}
    base.update(env)
    base.setdefault("MAP_BUNDLE", "")  # empty: slam_choice.sh's default, the registry
    p = _launch(tmp_path, venv_py=_fake_python(tmp_path), DRY_RUN="0", **base)
    env_sh = tmp_path / "out" / "env.sh"
    return p, env_sh.read_text() if env_sh.exists() else ""


def _preflight(tmp_path) -> list:
    return (tmp_path / "out" / "preflight.log").read_text().splitlines()


def test_the_operator_console_relocalizes_by_default_from_the_registrys_scene_start(tmp_path):
    p, env_sh = _registry_launch(tmp_path, MAP_BUNDLE="")
    assert p.returncode == 0, p.stdout + p.stderr
    pre = _preflight(tmp_path)
    assert "--start-relocalize" in pre and "--start-pose=1.5,-2.0,0.25" in pre  # placed at the survey's own start
    assert "start: relocalization (default: placed at the registry's scene start)" in p.stdout
    assert "START_MODE=" in env_sh and "--relocalize" in _supervisor(tmp_path).split()


def test_a_bundles_run_keeps_the_known_start_unless_its_start_pose_says_otherwise(tmp_path):
    p, _ = _registry_launch(tmp_path, BUNDLE_TOKEN="tok")
    assert p.returncode == 0, p.stdout + p.stderr
    assert "--start-relocalize" not in _preflight(tmp_path)
    assert "start: known (a bundle's run: unattended)" in p.stdout
    p, _ = _registry_launch(tmp_path / "b", BUNDLE_TOKEN="tok", START_POSE="-4.0,3.0,-2.0")  # B-P2d's sessions
    assert p.returncode == 0, p.stdout + p.stderr
    assert "--start-pose=-4.0,3.0,-2.0" in _preflight(tmp_path / "b")
    assert "start: relocalization (START_POSE)" in p.stdout


def test_the_start_choice_refuses_what_it_cannot_do(tmp_path):
    for bad in ("maybe", ""):  # set but empty is no "unset" (Codex B-P2e review P3)
        p, _ = _registry_launch(tmp_path / f"bad{len(bad)}", START_RELOC=bad)
        assert p.returncode == 2 and "START_RELOC must be on or off" in p.stdout + p.stderr
    p, _ = _registry_launch(tmp_path / "b", START_RELOC="off", START_POSE="-4.0,3.0,-2.0")
    assert p.returncode == 2 and "contradicts START_POSE" in p.stdout + p.stderr
    p, _ = _registry_launch(tmp_path / "c", START_RELOC="off")
    assert p.returncode == 0 and "start: known (START_RELOC=off)" in p.stdout
    # asked for by name where no registry map is there to relocalize on: no launch
    p = _relocalizing(tmp_path / "d", START_RELOC="on")
    assert p.returncode == 2 and "START_RELOC=on needs" in p.stdout + p.stderr


def test_a_row_not_marked_relocalizable_keeps_the_known_start_and_says_why(tmp_path):
    # lead 2026-10-31 (a): the default relocalizes only where the Isaac smoke showed the start can be found; elsewhere
    # the known start, its reason in the launch log, the supervisor's log and the console's state (slam.choice)
    p, env_sh = _registry_launch(tmp_path, relocalizable=False)
    assert p.returncode == 0, p.stdout + p.stderr
    assert "--start-relocalize" not in _preflight(tmp_path)
    assert "start: known (the registry row is not marked relocalizable at its start)" in p.stdout
    choice = "localization (registry: line_m_tsq_aaaaaaaaaaaa); start known: the row is not marked relocalizable"
    assert f"slam: {choice}" in p.stdout and "--relocalize" not in _supervisor(tmp_path).split()
    assert choice in _supervisor(tmp_path) and choice in "\n".join(_preflight(tmp_path))
    # asked for by name, the operator's session relocalizes on it all the same
    p, _ = _registry_launch(tmp_path / "on", relocalizable=False, START_RELOC="on")
    assert p.returncode == 0, p.stdout + p.stderr
    assert "--start-pose=1.5,-2.0,0.25" in _preflight(tmp_path / "on")
    assert "start: relocalization (START_RELOC=on: the registry row is not marked relocalizable at its start)" in p.stdout


def test_a_map_folder_given_by_hand_keeps_the_known_start(tmp_path):
    # Codex B-P2e design review P2: its MAP_START_POSE is the SLAM's start estimate, not where the truck stands
    p = _relocalizing(tmp_path)
    assert p.returncode == 0, p.stdout + p.stderr
    assert "--start-relocalize" not in _preflight(tmp_path)
    assert "start: known (a map folder given by hand: START_POSE relocalizes on it)" in p.stdout
