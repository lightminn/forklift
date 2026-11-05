"""The live launcher's TERM (deploy/console/isaac_live.sbatch): a TERM -- Slurm's, or reloc_session.sh ending it when its
probe fails -- ends the supervisor it started, however early it comes after the start, and the launcher waits for
the supervisor's own end and returns its code. Before the fix a TERM between the supervisor's start and the launcher's
trap killed the launcher alone; the supervisor stayed up and could start sessions (found by Codex in #322's review).
The window is held open with the launcher's test hook LIVE_TEST_HOLD_FILE (between the start and SUP=$!)."""
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
LAUNCHER = ROOT / "deploy/console/isaac_live.sbatch"

@pytest.fixture
def tmp_path():
    """A short root under /tmp, as test_isaac_live_preflight.py's (#325): the launcher refuses an OUT whose SLAM socket
    path passes 107 bytes, which ws1's pytest basetemp reaches."""
    root = Path(tempfile.mkdtemp(dir="/tmp", prefix="ilt"))
    yield root
    shutil.rmtree(root, ignore_errors=True)


FAKE_SUPERVISOR = textwrap.dedent("""
    import os, signal, sys, time
    from pathlib import Path
    out = Path(sys.argv[1])
    def term(*_):
        n = len(list(out.glob("sup_term*")))
        (out / f"sup_term{n}").write_text(str(time.monotonic()))
        if n == 0:
            time.sleep(float(os.environ.get("FAKE_CLEANUP_S", "1.0")))  # its runner and bridge to end
            (out / "sup_done").write_text("")
            sys.exit(7)
    signal.signal(signal.SIGTERM, term)
    (out / "sup_pid").write_text(str(os.getpid()))
    (out / "sup_ready").write_text("")
    while True:
        time.sleep(0.05)
""")


def _fakes(tmp_path):
    bin_ = tmp_path / "bin"
    bin_.mkdir(exist_ok=True)
    fake = bin_ / "docker"
    fake.write_text(f"#!/bin/bash\necho \"$0 $*\" >> {tmp_path / 'started'}\n")
    fake.chmod(0o755)
    sup = tmp_path / "fake_supervisor.py"
    sup.write_text(FAKE_SUPERVISOR)
    marks = tmp_path / "marks"
    marks.mkdir(exist_ok=True)
    py = tmp_path / "python"  # the supervisor and the stall watch faked, the preflight's checks real
    py.write_text(textwrap.dedent(f"""\
        #!/bin/bash
        case "$1" in
          */live_supervisor.py) exec {sys.executable} {sup} {marks} ;;
          */stall_watch.py) exec sleep 120 ;;
        esac
        exec {sys.executable} "$@"
        """))
    py.chmod(0o755)
    return bin_, py, marks


def _start(tmp_path, **env):
    bin_, py, marks = _fakes(tmp_path)
    base = {"PATH": f"{bin_}:{os.environ['PATH']}", "BASE_SCENE": "scene.usda",
            "FORKLIFT_URDF": str(ROOT / "sim/models/dls08_measured/forklift.urdf"), "VENV_PY": str(py),
            "PYTHONPATH": str(ROOT / "src"), "MAP_BUNDLE": "none", "STALL_MIN": "0", **env}
    log = open(tmp_path / "launcher.log", "w")  # noqa: SIM115 -- a file, not a pipe: its children may outlive it
    proc = subprocess.Popen(["bash", str(LAUNCHER), str(tmp_path / "out")], cwd=ROOT, env=base,
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    log.close()
    return proc, marks


def _wait(path, s=120.0):
    end = time.monotonic() + s
    while not path.exists():
        assert time.monotonic() < end, f"no {path.name}"
        time.sleep(0.02)


def _alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def _cleanup(proc, marks):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGKILL)
    pid = marks / "sup_pid"
    if pid.exists() and _alive(int(pid.read_text())):
        os.kill(int(pid.read_text()), signal.SIGKILL)
    try:  # the stall watch's stand-in, in the launcher's session
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def test_a_term_before_the_launcher_knows_the_supervisors_pid_ends_the_supervisor(tmp_path):
    hold = tmp_path / "hold"
    proc, marks = _start(tmp_path, LIVE_TEST_HOLD_FILE=str(hold))
    try:
        _wait(Path(f"{hold}.at"))
        _wait(marks / "sup_ready")  # its own TERM handler is set
        proc.send_signal(signal.SIGTERM)  # the launcher has started it but not yet read its pid
        time.sleep(0.3)
        hold.write_text("")
        proc.wait(timeout=60)
        out = (tmp_path / "launcher.log").read_text()
        sup = int((marks / "sup_pid").read_text())
        assert (marks / "sup_term0").exists() and (marks / "sup_done").exists() and not _alive(sup), out
        assert proc.returncode == 7, out  # the supervisor's own code
    finally:
        _cleanup(proc, marks)


def test_a_second_term_while_the_supervisor_ends_does_not_leave_it_behind(tmp_path):
    # Codex design P2: the old second wait gave up on a second TERM and the launcher ended before the supervisor
    proc, marks = _start(tmp_path, FAKE_CLEANUP_S="2.0")
    try:
        _wait(marks / "sup_ready")
        proc.send_signal(signal.SIGTERM)
        _wait(marks / "sup_term0", 30)
        proc.send_signal(signal.SIGTERM)
        proc.wait(timeout=60)
        out = (tmp_path / "launcher.log").read_text()
        sup = int((marks / "sup_pid").read_text())
        assert (marks / "sup_done").exists() and not _alive(sup), out
        assert proc.returncode == 7, out
    finally:
        _cleanup(proc, marks)


def test_the_test_hook_is_inert_unless_set():
    src = LAUNCHER.read_text()
    assert 'if [ -n "${LIVE_TEST_HOLD_FILE:-}" ]' in src
