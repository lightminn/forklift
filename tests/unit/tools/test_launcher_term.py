"""A launcher's TERM reaches the children it starts, however early it comes, and the launcher waits for their own end
(deploy/lib/term_relay.sh; the pattern of the console launcher's fix, fix/live-launcher-term). Before it, the bundle,
array, Riibotics scene and survey launchers and the relocalization wrapper set their trap after the start (a TERM in
between ended the launcher alone and left its child running), the bundle, array and survey launchers gave up their wait
on a second TERM, and the survey and the wrapper ran their probe in the foreground, which held a trapped TERM until the
probe ended. The window between a start and its ``$!`` is held open with the helper's test hook TERM_RELAY_TEST_HOLD."""

import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import textwrap
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import isaac_bundle as ib  # noqa: E402

FAKE_CHILD = textwrap.dedent("""
    import os, signal, sys, time
    from pathlib import Path
    marks, name = Path(sys.argv[1]), sys.argv[2]
    up = name.upper()
    def term(*_):
        n = len(list(marks.glob(f"{name}_term*")))
        (marks / f"{name}_term{n}").write_text(str(time.monotonic()))
        if n == 0:
            time.sleep(float(os.environ.get(f"FAKE_{up}_CLEANUP_S", "1.0")))  # what it ends of its own
            (marks / f"{name}_done").write_text("")
            sys.exit(int(os.environ.get(f"FAKE_{up}_TERM_RC", "7")))
    signal.signal(signal.SIGTERM, term)
    (marks / f"{name}_pid").write_text(str(os.getpid()))
    (marks / f"{name}_ready").write_text("")
    end = time.monotonic() + float(os.environ.get(f"FAKE_{up}_LIFE_S", "120"))
    while time.monotonic() < end:
        time.sleep(0.05)
    (marks / f"{name}_natural").write_text("")
    sys.exit(int(os.environ.get(f"FAKE_{up}_RC", "0")))
""")


@pytest.fixture
def tmp_path():
    """A short root under /tmp (ws1's pytest basetemp makes the survey's socket paths too long, as in #325)."""
    root = Path(tempfile.mkdtemp(dir="/tmp", prefix="ltt"))
    yield root
    shutil.rmtree(root, ignore_errors=True)


class Launch:
    """One launcher started in a session of its own, its log in a file (a child left behind would hold a pipe)."""

    def __init__(self, tmp, argv, env, cwd=ROOT):
        self.tmp, self.marks = tmp, tmp / "marks"
        self.marks.mkdir(exist_ok=True)
        self.hold = tmp / "hold"
        log = open(tmp / "launcher.log", "w")  # noqa: SIM115
        base = {**os.environ, "TERM_RELAY_TEST_HOLD": str(self.hold), **env}
        for k in ("SLURM_JOB_ID", "SLURM_JOB_GPUS", "CUDA_VISIBLE_DEVICES", "SLURM_JOB_END_TIME"):
            base.pop(k, None)
        self.proc = subprocess.Popen(argv, cwd=cwd, env=base, stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
        log.close()

    def release(self, name):
        Path(f"{self.hold}.{name}").write_text("")

    def wait_for(self, path, s=60.0):
        end = time.monotonic() + s
        while not Path(path).exists():
            assert self.proc.poll() is None or Path(path).exists(), f"launcher ended before {Path(path).name}:\n{self.log()}"
            assert time.monotonic() < end, f"no {Path(path).name}:\n{self.log()}"
            time.sleep(0.02)

    def mark(self, name):
        return self.marks / name

    def term(self):
        self.proc.send_signal(signal.SIGTERM)

    def end(self, s=60):
        try:
            return self.proc.wait(timeout=s)
        except subprocess.TimeoutExpired:
            pytest.fail(f"the launcher did not end:\n{self.log()}")

    def log(self):
        return (self.tmp / "launcher.log").read_text()

    def child_alive(self, name):
        pid = self.mark(f"{name}_pid")
        if not pid.exists():
            return False
        try:
            os.kill(int(pid.read_text()), 0)
        except ProcessLookupError:
            return False
        return True

    def cleanup(self):
        for p in self.marks.glob("*_pid"):
            try:
                os.kill(int(p.read_text()), signal.SIGKILL)
            except (ProcessLookupError, ValueError):
                pass
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


@pytest.fixture
def launches():
    started = []
    yield started
    for launch in started:
        launch.cleanup()


def _fake_child(tmp):
    path = tmp / "fake_child.py"
    path.write_text(FAKE_CHILD)
    return path


def _exe(path, text):
    path.write_text(text)
    path.chmod(0o755)
    return path


# --- the launchers, each with its child faked -------------------------------------------------------------------------

def bundle(tmp, launches, **env):
    child = _fake_child(tmp)
    py = _exe(tmp / "bundle_py", f"#!/bin/bash\nexec {sys.executable} {child} {tmp / 'marks'} runner\n")
    runs = tmp / "runs.jsonl"
    runs.write_text(json.dumps({"name": "r0", "argv": ["true"]}) + "\n")
    launch = Launch(tmp, ["bash", str(ROOT / "deploy/slurm/isaac_bundle.sbatch"), str(runs), str(tmp / "out")],
                    {"BUNDLE_PY": str(py), **env})
    launches.append(launch)
    return launch


def array(tmp, launches, **env):
    child = _fake_child(tmp)
    py = _exe(tmp / "bundle_py", f"#!/bin/bash\nexec {sys.executable} {child} {tmp / 'marks'} runner\n")
    runs = tmp / "runs.jsonl"
    runs.write_text(json.dumps({"name": "r0", "argv": ["true"]}) + "\n")
    launch = Launch(tmp, ["bash", str(ROOT / "deploy/slurm/isaac_array.sbatch"), str(runs), str(tmp / "out")],
                    {"BUNDLE_PY": str(py), "SLURM_ARRAY_TASK_ID": "0", "PREFLIGHT": "off",
                     "PREFLIGHT_WHY": "the TERM test", **env})
    launches.append(launch)
    return launch


def riib(tmp, launches, **env):
    child = _fake_child(tmp)
    scenes = tmp / "scenes"
    scenes.mkdir()
    for name in ("line_2", "line_2_m"):
        (scenes / f"{name}.json").write_text(json.dumps({"views": {"quarter_eye": [1, 2, 3], "quarter_target": [4, 5, 6]}}))
    (tmp / "weights.onnx").write_bytes(b"")
    py = _exe(tmp / "python", textwrap.dedent(f"""\
        #!/bin/bash
        for a in "$@"; do [ "$a" = --check-args ] && {{ echo ARGS_OK; exit 0; }}; done
        case "$1" in */run_transport.py) exec {sys.executable} {child} {tmp / 'marks'} runner ;; esac
        exec {sys.executable} "$@"
        """))
    launch = Launch(tmp, ["bash", str(ROOT / "deploy/riibotics/riib_scene.sbatch"), str(tmp / "out")],
                    {"SNAP": str(ROOT), "SCENE_DIR": str(scenes), "SCENE": "line_2", "BASE_SCENE": "scene.usda",
                     "FORKLIFT_URDF": str(ROOT / "sim/models/dls08_measured/forklift.urdf"), "VENV_PY": str(py),
                     "ONNX": str(tmp / "weights.onnx"), "LEARNED_PY": sys.executable, "MODE": "truth", **env})
    launches.append(launch)
    return launch


def survey(tmp, launches, **env):
    child = _fake_child(tmp)
    bin_ = tmp / "bin"
    bin_.mkdir()
    _exe(bin_ / "docker", f"#!/bin/bash\necho \"docker $*\" >> {tmp / 'docker.log'}\n"
                          "[ \"$1 $2\" = 'image inspect' ] && echo sha256:fake\nexit 0\n")
    py = _exe(tmp / "python", textwrap.dedent(f"""\
        #!/bin/bash
        case "$1" in
          */live_supervisor.py) exec {sys.executable} {child} {tmp / 'marks'} sup ;;
          */live_console_probe.py) exec {sys.executable} {child} {tmp / 'marks'} probe ;;
          */slam_survey_bundle.py|*/list_rigid_bodies.py) exit 0 ;;
        esac
        exec {sys.executable} "$@"
        """))
    launch = Launch(tmp, ["bash", str(ROOT / "deploy/console/slam_survey.sbatch"), str(tmp / "out")],
                    {"PATH": f"{bin_}:{os.environ['PATH']}", "BASE_SCENE": "scene.usda",
                     "FORKLIFT_URDF": str(ROOT / "sim/models/dls08_measured/forklift.urdf"), "VENV_PY": str(py),
                     **env})
    launches.append(launch)
    return launch


def reloc(tmp, launches, **env):
    child = _fake_child(tmp)
    launcher = _exe(tmp / "launcher.sh", f"#!/bin/bash\nexec {sys.executable} {child} {tmp / 'marks'} launcher\n")
    py = _exe(tmp / "python", textwrap.dedent(f"""\
        #!/bin/bash
        case "$1" in */live_console_probe.py) exec {sys.executable} {child} {tmp / 'marks'} probe ;; esac
        exec {sys.executable} "$@"
        """))
    launch = Launch(tmp, ["bash", str(ROOT / "deploy/console/reloc_session.sh"), str(tmp / "out")],
                    {"START_POSE": "1,2,3", "VENV_PY": str(py), "RELOC_LAUNCHER": str(launcher), **env})
    launches.append(launch)
    return launch


# the single-child launchers: the child's name, and what else to release
SINGLE = {"bundle": (bundle, "runner"), "array": (array, "runner"), "riib": (riib, "runner")}


@pytest.mark.parametrize("kind", sorted(SINGLE))
def test_a_term_before_the_launcher_knows_its_childs_pid_ends_the_child(tmp_path, launches, kind):
    start, name = SINGLE[kind]
    launch = start(tmp_path, launches)
    launch.wait_for(f"{launch.hold}.{name}.at")
    launch.wait_for(launch.mark(f"{name}_ready"))  # its own TERM handler is set
    launch.term()  # the launcher has started it but not yet read its pid
    time.sleep(0.3)
    launch.release(name)
    rc = launch.end()
    assert launch.mark(f"{name}_term0").exists() and launch.mark(f"{name}_done").exists(), launch.log()
    assert not launch.child_alive(name), launch.log()
    assert rc == 7, launch.log()  # the child's own code


@pytest.mark.parametrize("kind", sorted(SINGLE))
def test_a_second_term_while_the_child_ends_does_not_leave_it_behind(tmp_path, launches, kind):
    start, name = SINGLE[kind]
    launch = start(tmp_path, launches, **{f"FAKE_{name.upper()}_CLEANUP_S": "2.0"})
    launch.release(name)
    launch.wait_for(launch.mark(f"{name}_ready"))
    launch.term()
    launch.wait_for(launch.mark(f"{name}_term0"), 30)
    launch.term()
    rc = launch.end()
    assert launch.mark(f"{name}_done").exists() and not launch.child_alive(name), launch.log()
    assert rc == 7, launch.log()


@pytest.mark.parametrize("kind", sorted(SINGLE))
def test_without_a_term_the_launcher_returns_its_childs_code(tmp_path, launches, kind):
    start, name = SINGLE[kind]
    launch = start(tmp_path, launches, **{f"FAKE_{name.upper()}_LIFE_S": "0.5", f"FAKE_{name.upper()}_RC": "5"})
    launch.release(name)
    assert launch.end() == 5, launch.log()
    assert launch.mark(f"{name}_natural").exists() and not launch.mark(f"{name}_term0").exists()


def test_riib_a_term_while_the_bridge_starts_still_ends_the_launcher_and_its_bridge(tmp_path, launches):
    # before the runner starts nothing is armed: a TERM ends the launcher and its EXIT trap stops the bridge, as before
    bin_ = tmp_path / "bin"
    bin_.mkdir()
    _exe(bin_ / "docker", f"#!/bin/bash\necho \"docker $1\" >> {tmp_path / 'docker.log'}\nexit 0\n")
    launch = riib(tmp_path, launches, MODE="slam", PATH=f"{bin_}:{os.environ['PATH']}")
    launch.wait_for(tmp_path / "docker.log")
    time.sleep(0.5)  # in its wait for bridge_ready
    launch.term()
    assert launch.end() != 0
    log = (tmp_path / "docker.log").read_text()
    assert "docker stop" in log and "docker rm" in log, log
    assert not launch.mark("runner_ready").exists()


# --- the survey: supervisor and probe ---------------------------------------------------------------------------------

def test_survey_a_term_before_the_supervisors_pid_is_known_ends_supervisor_and_probe(tmp_path, launches):
    launch = survey(tmp_path, launches)
    launch.release("probe")
    launch.wait_for(f"{launch.hold}.sup.at")
    launch.wait_for(launch.mark("sup_ready"))
    launch.term()
    time.sleep(0.3)
    launch.release("sup")
    rc = launch.end()
    assert launch.mark("sup_term0").exists() and not launch.child_alive("sup"), launch.log()
    assert not launch.child_alive("probe"), launch.log()
    assert rc == 7, launch.log()


def test_survey_a_term_while_the_probe_runs_reaches_the_supervisor_at_once(tmp_path, launches):
    # the probe used to run in the foreground: bash held the trapped TERM until the probe ended (here 120 s)
    launch = survey(tmp_path, launches)
    launch.release("sup")
    launch.release("probe")
    launch.wait_for(launch.mark("sup_ready"))
    launch.wait_for(launch.mark("probe_ready"))
    t0 = time.monotonic()
    launch.term()
    launch.wait_for(launch.mark("sup_term0"), 10)
    assert time.monotonic() - t0 < 5, launch.log()
    rc = launch.end()
    assert not launch.child_alive("sup") and not launch.child_alive("probe"), launch.log()
    assert rc == 7, launch.log()
    assert "probe exit" in launch.log() and "supervisor exit 7" in launch.log()


def test_survey_a_second_term_while_the_supervisor_ends_does_not_leave_it_behind(tmp_path, launches):
    launch = survey(tmp_path, launches, FAKE_SUP_CLEANUP_S="2.0")
    launch.release("sup")
    launch.release("probe")
    launch.wait_for(launch.mark("sup_ready"))
    launch.wait_for(launch.mark("probe_ready"))
    launch.term()
    launch.wait_for(launch.mark("sup_term0"), 10)
    launch.term()
    rc = launch.end()
    assert launch.mark("sup_done").exists() and not launch.child_alive("sup"), launch.log()
    assert rc == 7, launch.log()


# --- the relocalization wrapper: launcher and probe, its own cancel policy (exit 143) ---------------------------------

def test_reloc_a_term_before_the_launchers_pid_is_known_ends_the_launcher(tmp_path, launches):
    launch = reloc(tmp_path, launches)
    launch.release("probe")
    launch.wait_for(f"{launch.hold}.launcher.at")
    launch.wait_for(launch.mark("launcher_ready"))
    launch.term()
    time.sleep(0.3)
    launch.release("launcher")
    rc = launch.end()
    assert launch.mark("launcher_term0").exists() and not launch.child_alive("launcher"), launch.log()
    assert not launch.child_alive("probe")
    assert rc == 143, launch.log()


def test_reloc_a_term_while_the_probe_runs_ends_the_launcher_at_once(tmp_path, launches):
    launch = reloc(tmp_path, launches)
    launch.release("launcher")
    launch.release("probe")
    launch.wait_for(launch.mark("launcher_ready"))
    launch.wait_for(launch.mark("probe_ready"))
    t0 = time.monotonic()
    launch.term()
    launch.wait_for(launch.mark("launcher_term0"), 10)
    assert time.monotonic() - t0 < 5, launch.log()
    assert launch.end() == 143, launch.log()
    assert not launch.child_alive("launcher") and not launch.child_alive("probe")


def test_reloc_a_term_in_the_grace_after_the_probe_still_ends_with_143(tmp_path, launches):
    # Codex design P2: the cancel policy holds after the probe too (the launcher would otherwise return its own 7)
    launch = reloc(tmp_path, launches, FAKE_PROBE_LIFE_S="0.3", RELOC_END_GRACE_S="60")
    launch.release("launcher")
    launch.release("probe")
    launch.wait_for(launch.mark("probe_natural"))
    time.sleep(0.5)  # in the grace
    launch.term()
    assert launch.end() == 143, launch.log()
    assert launch.mark("launcher_term0").exists() and not launch.child_alive("launcher")


def test_reloc_the_cancel_does_not_wait_for_a_probe_slow_to_end(tmp_path, launches):
    launch = reloc(tmp_path, launches, FAKE_PROBE_CLEANUP_S="30")
    launch.release("launcher")
    launch.release("probe")
    launch.wait_for(launch.mark("launcher_ready"))
    launch.wait_for(launch.mark("probe_ready"))
    t0 = time.monotonic()
    launch.term()
    assert launch.end() == 143, launch.log()
    assert time.monotonic() - t0 < 15, launch.log()
    assert not launch.child_alive("launcher") and not launch.child_alive("probe")


def test_reloc_a_launcher_that_ignores_the_term_is_killed_after_the_cancel_time(tmp_path, launches):
    launch = reloc(tmp_path, launches, FAKE_LAUNCHER_CLEANUP_S="60", RELOC_END_KILL_S="3")
    launch.release("launcher")
    launch.release("probe")
    launch.wait_for(launch.mark("launcher_ready"))
    launch.wait_for(launch.mark("probe_ready"))
    t0 = time.monotonic()
    launch.term()
    time.sleep(2.0)
    launch.term()  # late in the time: restarting it would end the wrapper about 5 s after the first TERM
    assert launch.end() == 143, launch.log()
    assert 2.8 < time.monotonic() - t0 < 4.5, launch.log()
    assert not launch.child_alive("launcher")


def test_reloc_without_a_term_a_session_that_ended_first_keeps_its_exit_3_path(tmp_path, launches):
    launch = reloc(tmp_path, launches, FAKE_LAUNCHER_LIFE_S="0.3", FAKE_LAUNCHER_RC="0",
                   FAKE_PROBE_LIFE_S="1.0", FAKE_PROBE_RC="3")
    launch.release("launcher")
    launch.release("probe")
    assert launch.end() == 0, launch.log()
    assert "probe exit 3" in launch.log() and "probe failed" not in launch.log()


# --- the helper itself, and the bundle preflight ---------------------------------------------------------------------

def test_sourcing_the_helper_sets_no_trap_and_no_option():
    out = subprocess.run(["bash", "-c", f"source {ROOT / 'deploy/lib/term_relay.sh'}; trap -p; echo \"$-\""],
                         capture_output=True, text=True, check=True).stdout.splitlines()
    assert out == [subprocess.run(["bash", "-c", "echo \"$-\""], capture_output=True, text=True).stdout.strip()]


@pytest.mark.parametrize("script", ["deploy/slurm/isaac_bundle.sbatch", "deploy/slurm/isaac_array.sbatch",
                                    "deploy/console/slam_survey.sbatch", "deploy/console/reloc_session.sh"])
def test_a_root_without_the_helper_is_refused_before_anything_starts(tmp_path, script):
    (tmp_path / Path(script).parent).mkdir(parents=True)
    shutil.copy(ROOT / script, tmp_path / script)
    runs = tmp_path / "runs.jsonl"
    runs.write_text(json.dumps({"name": "r0", "argv": ["true"]}) + "\n")
    env = {**os.environ, "SLURM_ARRAY_TASK_ID": "0", "PREFLIGHT": "off", "PREFLIGHT_WHY": "x", "BUNDLE_PY": "false",
           "START_POSE": "1,2,3", "VENV_PY": "false", "BASE_SCENE": "s", "FORKLIFT_URDF": "u",
           "MAP_PARAMS": script, "RELOC_LAUNCHER": "true"}
    argv = ["bash", script] + ([str(runs)] if "slurm" in script else []) + [str(tmp_path / "out")]
    p = subprocess.run(argv, cwd=tmp_path, env=env, capture_output=True, text=True, timeout=60)
    assert p.returncode == 2 and "term_relay.sh" in p.stdout + p.stderr, p.stdout + p.stderr


def test_riib_a_snapshot_without_the_helper_fails_its_dry_run(tmp_path):
    snap = tmp_path / "snap"
    shutil.copytree(ROOT, snap, ignore=shutil.ignore_patterns(".git", ".claude", "__pycache__", "*.npz", "docs",
                                                               "presentation", "artifacts"))
    os.remove(snap / "deploy/lib/term_relay.sh")
    scenes = tmp_path / "scenes"
    scenes.mkdir()
    (scenes / "line_2_m.json").write_text(json.dumps({"views": {"quarter_eye": [1, 2, 3], "quarter_target": [4, 5, 6]}}))
    (tmp_path / "w.onnx").write_bytes(b"")
    env = {"SNAP": str(snap), "SCENE_DIR": str(scenes), "SCENE": "line_2", "BASE_SCENE": "scene.usda",
           "FORKLIFT_URDF": str(snap / "sim/models/dls08_measured/forklift.urdf"), "VENV_PY": sys.executable,
           "ONNX": str(tmp_path / "w.onnx"), "LEARNED_PY": sys.executable, "DRY_RUN": "1", "DRY_RUN_CHECK": "0",
           "PATH": os.environ["PATH"]}
    p = subprocess.run(["bash", str(snap / "deploy/riibotics/riib_scene.sbatch"), str(tmp_path / "out")], env=env,
                       capture_output=True, text=True, timeout=60)
    assert p.returncode == 2 and "term_relay.sh" in p.stdout + p.stderr, p.stdout + p.stderr


def test_the_helper_is_a_preflight_input_and_its_settings_are_checked(tmp_path):
    lib = tmp_path / "deploy/lib"
    lib.mkdir(parents=True)
    helper = lib / "term_relay.sh"
    helper.write_text((ROOT / "deploy/lib/term_relay.sh").read_text())
    run = ib.Run(name="r0", argv=["bash", "x.sbatch"], cwd=str(tmp_path), env={})
    before = ib.preflight_inputs([run], ROOT / "sim/isaac/run_transport.py")
    assert os.path.realpath(helper) in before
    helper.write_text(helper.read_text() + "\n# changed\n")
    assert ib.preflight_inputs([run], ROOT / "sim/isaac/run_transport.py") != before  # an earlier proof no longer holds
    assert "TERM_RELAY_TEST_HOLD" in ib.inherited_controls(run, {"TERM_RELAY_TEST_HOLD": "/tmp/h"})
