"""PR 7c-1a: slam_toolbox instances as direct children -- stopping empties the whole process group (a child that ignores
SIGTERM is killed), reaps the child, waits for the graph and drains, and says when any of that did not happen."""

import sys
import textwrap
import time

from forklift_core.localization.slam_instance import SlamInstances, group_members

FAKE = textwrap.dedent("""
    import os, signal, sys, time
    if sys.argv[1] == "stubborn":
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
    if os.fork() == 0:  # a child in the same group, as a node's helper would be
        if sys.argv[1] == "stubborn":
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
        time.sleep(60)
        os._exit(0)
    time.sleep(60)
""")


def instances(tmp_path, mode, **kw):
    script = tmp_path / "fake_node.py"
    script.write_text(FAKE)
    return SlamInstances(lambda k, pose: [sys.executable, str(script), mode], log_dir=tmp_path, term_s=1.0,
                         kill_s=2.0, graph_s=1.0, **kw)


def test_a_stop_empties_the_group_and_reaps(tmp_path):
    drained = []
    s = instances(tmp_path, "plain", drain=drained.append)
    proc = s.start(1, (0.0, 0.0, 0.0))
    time.sleep(0.3)
    assert len(group_members(proc.pid)) == 2
    assert s.stop(1) and group_members(proc.pid) == [] and drained == [1]
    assert s.records[-1]["sigkill"] is False


def test_a_member_ignoring_sigterm_is_killed(tmp_path):
    s = instances(tmp_path, "stubborn")
    proc = s.start(1, (0.0, 0.0, 0.0))
    time.sleep(0.3)
    assert s.stop(1) and s.records[-1]["sigkill"] is True and group_members(proc.pid) == []


def test_a_node_that_stays_in_the_graph_is_no_stop(tmp_path):
    drained = []
    s = instances(tmp_path, "plain", graph_has=lambda k: True, drain=drained.append)
    s.start(1, (0.0, 0.0, 0.0))
    time.sleep(0.2)
    assert not s.stop(1) and s.records[-1]["graph_gone"] is False and drained == []


def test_stopping_an_unknown_instance_is_a_no_op(tmp_path):
    assert instances(tmp_path, "plain").stop(9)


def test_a_child_not_reaped_stays_for_stop_all(tmp_path, monkeypatch):
    """Codex 7c-1a round 1 P2-4: the Popen is kept until it is reaped."""
    import subprocess

    s = instances(tmp_path, "plain")
    proc = s.start(1, (0.0, 0.0, 0.0))
    time.sleep(0.2)
    real_wait = proc.wait
    calls = []

    def wait(timeout=None):
        calls.append(timeout)
        if len(calls) == 1:
            raise subprocess.TimeoutExpired("x", timeout)
        return real_wait(timeout)

    monkeypatch.setattr(proc, "wait", wait)
    assert not s.stop(1) and 1 in s.procs
    s.stop_all()
    assert 1 not in s.procs and s.records[-1]["ok"]


def test_a_stop_fits_the_cancel_to_void_bound(tmp_path):
    """With the default bounds, a group ignoring SIGTERM is stopped well inside the executor's 5 s to void."""
    script = tmp_path / "fake_node.py"
    script.write_text(FAKE)
    s = SlamInstances(lambda k, pose: [sys.executable, str(script), "stubborn"], log_dir=tmp_path)
    s.start(1, (0.0, 0.0, 0.0))
    time.sleep(0.2)
    t0 = time.monotonic()
    assert s.stop(1) and time.monotonic() - t0 < 4.0


def test_a_group_left_after_the_leader_is_reaped_is_retried(tmp_path, monkeypatch):
    """Codex 7c-1a round 2 P2-3: the leader reaped but a member still listed: the entry stays for stop_all."""
    import forklift_core.localization.slam_instance as mod

    s = instances(tmp_path, "plain")
    s.start(1, (0.0, 0.0, 0.0))
    time.sleep(0.2)
    real = mod.group_members
    monkeypatch.setattr(mod, "group_members", lambda pgid: [999999])  # a member that does not leave in time
    assert not s.stop(1) and 1 in s.procs and s.records[-1]["group_empty"] is False
    monkeypatch.setattr(mod, "group_members", real)
    s.stop_all()
    assert 1 not in s.procs and s.records[-1]["ok"]
