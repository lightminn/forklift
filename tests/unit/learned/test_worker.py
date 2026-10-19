"""The learned proposer's worker process protocol (stage 3 plan, task 6)."""
import sys
import textwrap

import numpy as np
import pytest

from forklift_learned import worker as W


class _Scene:
    rgb = np.zeros((48, 64, 3), np.uint8)


def _fake_worker(tmp_path, body):
    head = ("import numpy as np\n"
            "from forklift_core.perception.instance_proposals import InstanceProposal\n"
            "from forklift_learned.worker import serve\n"
            "class P:\n"
            "    def propose(self, scene):\n")
    script = tmp_path / "fake.py"
    script.write_text(head + textwrap.indent(textwrap.dedent(body).strip(), " " * 8) + "\nserve(P())\n")
    return W.WorkerProposer(None, None, command=[sys.executable, str(script)])


def test_the_worker_returns_the_model_proposals(tmp_path):
    w = _fake_worker(tmp_path, """
        m = np.zeros(scene.rgb.shape[:2], bool); m[10:20, 10:30] = True
        return [InstanceProposal("l0", m, 0.8, "learned")]
    """)
    try:
        out = w.propose(_Scene())
        assert len(out) == 1 and out[0].source == "learned" and out[0].score == pytest.approx(0.8)
        assert out[0].mask.shape == (48, 64) and out[0].mask.sum() == 200
        assert w.propose(_Scene())[0].proposal_id == "l0"  # serves repeatedly
    finally:
        w.close()


def test_a_model_error_is_raised_on_the_client(tmp_path):
    w = _fake_worker(tmp_path, """
        raise ValueError("bad input")
    """)
    try:
        with pytest.raises(RuntimeError, match="bad input"):
            w.propose(_Scene())
    finally:
        w.close()


def test_no_proposals_come_back_empty(tmp_path):
    w = _fake_worker(tmp_path, """
        return []
    """)
    try:
        assert w.propose(_Scene()) == []
    finally:
        w.close()


def test_a_hung_worker_is_killed_at_the_deadline_and_raises(tmp_path):
    # Codex review 1: a silent worker blocked the mission forever and the fallback never ran
    import time

    w = _fake_worker(tmp_path, """
        import time; time.sleep(60)
    """)
    w.timeout_s = w.first_timeout_s = 1.0
    t0 = time.monotonic()
    with pytest.raises(RuntimeError, match="deadline"):
        w.propose(_Scene())
    assert time.monotonic() - t0 < 10 and w.proc.poll() is not None  # killed and reaped
    with pytest.raises(RuntimeError, match="ended"):
        w.propose(_Scene())  # later frames fall back at once


def test_close_kills_a_worker_that_does_not_exit(tmp_path):
    w = W.WorkerProposer(None, None, command=[sys.executable, "-c",
                                              "import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)"])
    w.close(timeout_s=1.0)
    assert w.proc.poll() is not None


def test_close_is_bounded_while_a_send_is_stuck_in_the_pipe():
    # Codex re-review 1: closing stdin waited on the buffer lock a stuck send held (1.2 s past close(0.05))
    import threading
    import time

    w = W.WorkerProposer(None, None, command=[sys.executable, "-c", "import time; time.sleep(60)"])
    def send():
        try:
            W._send(w.proc.stdin, b"\0" * (4 << 20))
        except (OSError, ValueError):
            pass  # the pipe breaks when close kills the child

    stuck = threading.Thread(target=send, daemon=True)
    stuck.start()
    time.sleep(0.3)  # the child never reads: the 4 MB write blocks holding the writer's lock
    t0 = time.monotonic()
    w.close(timeout_s=0.2)
    assert time.monotonic() - t0 < 1.0 and w.proc.poll() is not None
