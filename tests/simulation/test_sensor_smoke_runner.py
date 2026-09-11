"""Failure behavior of the bounded process/output boundary."""

import importlib.util
import os
import signal
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


def runner():
    path = ROOT / "sim/gazebo/run_sensor_smoke.py"
    assert path.exists(), "bounded runner is not implemented"
    spec = importlib.util.spec_from_file_location("smoke_runner", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_output_root_may_exist_but_run_child_must_be_fresh(tmp_path):
    api = runner()
    (tmp_path / "infrastructure.log").write_text("retained")
    path = api.fresh_output(tmp_path)
    assert path == tmp_path / "sensor_smoke"
    with pytest.raises(FileExistsError):
        api.fresh_output(tmp_path)
    assert (tmp_path / "infrastructure.log").read_text() == "retained"


def test_child_timeout_fails_and_cleanup_reaps_own_process(tmp_path):
    api = runner()
    children = api.Children(tmp_path, {})
    process = children.start(
        "sleep", [sys.executable, "-c", "import time;time.sleep(60)"]
    )
    try:
        with pytest.raises(TimeoutError):
            children.wait(process, timeout_s=0.05)
    finally:
        children.close()
    assert process.poll() is not None


def test_child_nonzero_is_not_success(tmp_path):
    api = runner()
    children = api.Children(tmp_path, {})
    process = children.start("fail", [sys.executable, "-c", "raise SystemExit(7)"])
    try:
        with pytest.raises(RuntimeError, match="7"):
            children.wait(process, timeout_s=3)
    finally:
        children.close()


def test_clock_readiness_requires_an_actual_received_clock(tmp_path):
    import json

    api = runner()
    path = tmp_path / "progress.json"
    path.write_text(json.dumps({"counts": {"clock": 0}}))
    with pytest.raises(TimeoutError):
        api.wait_clock(path, timeout_s=0.05)
    path.write_text(json.dumps({"counts": {"clock": 1}}))
    api.wait_clock(path, timeout_s=0.05)


@pytest.mark.parametrize("ignores_interrupt", [False, True])
def test_exited_leader_does_not_leave_owned_descendant(tmp_path, ignores_interrupt):
    api = runner()
    children = api.Children(tmp_path, {})
    ready = tmp_path / "descendant.pid"
    script = """
import os, signal, sys, time
from pathlib import Path
child = os.fork()
if child:
    while not Path(sys.argv[1]).exists():
        time.sleep(.01)
    raise SystemExit(0)
signal.signal(signal.SIGINT, signal.SIG_IGN if sys.argv[2] == 'True' else lambda *_: sys.exit(0))
Path(sys.argv[1]).write_text(str(os.getpid()))
time.sleep(60)
"""
    process = children.start(
        "orphan", [sys.executable, "-c", script, str(ready), str(ignores_interrupt)]
    )
    try:
        process.wait(timeout=5)
        assert process.returncode == 0
        os.killpg(process.pid, 0)
        if ignores_interrupt:
            with pytest.raises(RuntimeError, match="SIGKILL"):
                children.stop(process, grace_s=0.05)
        else:
            assert children.close() == []
        deadline = time.monotonic() + 2
        while (
            Path(f"/proc/{ready.read_text()}").exists() and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        assert not Path(f"/proc/{ready.read_text()}").exists()
    finally:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        children.close()
