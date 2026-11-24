"""docs/design/2026-11-19-real-safety-wiring.md 8.1: every tool that opens the MCU device takes the executor lock (the
listeners too), a device held at the tool's first opening is refused with nothing written and exit 2, a listener
writes nothing, and a pin over tools/*.py keeps new openings from skipping the lock."""

import ast
import json
import os
import sys
import tty
from pathlib import Path

import pytest

from forklift_core.live.executor_lock import lock_fd

REPO = Path(__file__).resolve().parents[3]
TOOLS = REPO / "tools"
sys.path.insert(0, str(TOOLS))

pytestmark = pytest.mark.skipif(not sys.platform.startswith("linux"), reason="flock and /proc/locks are Linux's")

# Calls that may skip the lock, by file: none in the operating tools today (tests use their own pty links).
ALLOWED: dict[str, set[str]] = {}


def _calls(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            f = node.func
            if (isinstance(f, ast.Name) and f.id == name) or (isinstance(f, ast.Attribute) and f.attr == name):
                yield node


def _kw(call, name):
    return next((k.value for k in call.keywords if k.arg == name), None)


def test_every_opening_in_the_tools_takes_the_lock():
    """An aid, not the guarantee (design 8.1): ReconnectingLink(...) with exclusive=True, open_raw(...) with
    before_config= -- outside the module that defines them."""
    bad = []
    for path in sorted(TOOLS.glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        # real_mcu_link's own CLI is a tool too: the class's default stays False, its call in main() says True
        for c in _calls(tree, "ReconnectingLink"):
            v = _kw(c, "exclusive")
            if not (isinstance(v, ast.Constant) and v.value is True) and "ReconnectingLink" not in ALLOWED.get(
                    path.name, set()):
                bad.append(f"{path.name}:{c.lineno} ReconnectingLink without exclusive=True")
        if path.name != "real_mcu_link.py":  # the module that defines open_raw calls nothing of it
            for c in _calls(tree, "open_raw"):
                if _kw(c, "before_config") is None and "open_raw" not in ALLOWED.get(path.name, set()):
                    bad.append(f"{path.name}:{c.lineno} open_raw without before_config=")
    assert not bad, bad


@pytest.fixture
def held_pty():
    """A pty whose slave another fd of this process holds the executor lock on; yields (path, master)."""
    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    path = os.ttyname(slave)
    other = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    lock_fd(other, path)
    yield path, master
    os.close(other)
    os.close(slave)
    os.close(master)


def _written(master) -> bytes:
    try:
        return os.read(master, 65536)
    except BlockingIOError:
        return b""


def _no_bench(monkeypatch, module):
    monkeypatch.setattr(module, "writer_marker", lambda device, bench: None)  # a real device: past the bench check


def test_bringup_commission_refuses_a_held_device_and_writes_nothing(held_pty, tmp_path, monkeypatch, capsys):
    import bringup

    path, master = held_pty
    _no_bench(monkeypatch, bringup)
    out = tmp_path / "run.jsonl"
    code = bringup.main(["commission", "--device", path, "--out", str(out), "--axis", "L", "--duty", "0.1",
                         "--seconds", "0.6", "--yes", "--power-switch-in-hand"])
    res = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
    assert code == 2 and res["result"] == "refused" and "held by another executor" in res["why"]
    assert res["holder"]["pid"] == os.getpid()
    assert _written(master) == b""
    assert json.loads(out.read_text().splitlines()[-1])["result"] == "refused"


def test_bringup_record_refuses_a_held_device(held_pty, tmp_path, monkeypatch, capsys):
    import bringup

    path, master = held_pty
    _no_bench(monkeypatch, bringup)
    code = bringup.main(["record", "--device", path, "--out", str(tmp_path / "r.jsonl"), "--seconds", "0.3"])
    assert code == 2 and json.loads(capsys.readouterr().out.strip().splitlines()[-1])["result"] == "refused"
    assert _written(master) == b""


def test_a_listener_writes_nothing_on_a_free_device(tmp_path, monkeypatch, capsys):
    """Design 8.1: no frame, no poison -- the MCU counts no receive error for a record (S2's 10 s window)."""
    import bringup

    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    try:
        _no_bench(monkeypatch, bringup)
        code = bringup.main(["record", "--device", os.ttyname(slave), "--out", str(tmp_path / "r.jsonl"),
                             "--seconds", "0.3"])
        assert code == 0 and _written(master) == b""
    finally:
        os.close(slave)
        os.close(master)


def test_rearm_hold_check_refuses_a_held_device(held_pty, tmp_path, monkeypatch, capsys):
    import rearm_hold_check

    path, master = held_pty
    _no_bench(monkeypatch, rearm_hold_check)
    code = rearm_hold_check.main(["--device", path, "--out", str(tmp_path / "r.jsonl"), "--stop-grace-ms", "300"])
    res = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
    assert code == 2 and res["result"] == "refused" and res["holder"]["pid"] == os.getpid()
    assert _written(master) == b""


def test_h743_link_check_refuses_a_held_device_and_leaves_a_refused_record(held_pty, tmp_path, capsys):
    import h743_link_check

    path, master = held_pty
    out = tmp_path / "link.jsonl"
    out.write_text('{"t": "state", "from": "an earlier run"}\n')  # a stale record a wrapper could fetch
    code = h743_link_check.main(["--device", path, "--out", str(out), "--seconds", "1"])
    assert code == 2 and "REFUSED" in capsys.readouterr().out
    assert json.loads(out.read_text().splitlines()[0])["t"] == "refused"
    assert _written(master) == b""


def test_the_link_monitor_refuses_a_held_device(held_pty, capsys):
    import real_mcu_link

    path, master = held_pty
    assert real_mcu_link.main(["--device", path, "--seconds", "0.3"]) == 2
    assert "held by another executor" in capsys.readouterr().err
    assert _written(master) == b""
