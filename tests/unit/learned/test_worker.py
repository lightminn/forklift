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


def test_the_reply_carries_the_worker_s_own_time_for_the_model(tmp_path):
    # learned VRAM plan 3.1: the worker's time apart from the round trip (their difference is the pipe and waiting)
    w = _fake_worker(tmp_path, """
        import time; time.sleep(0.05)
        return []
    """)
    try:
        assert w.last_infer_ms is None
        w.propose(_Scene())
        assert 45 <= w.last_infer_ms < 5000
    finally:
        w.close()
    bad = _fake_worker(tmp_path, """
        raise ValueError("model broke")
    """)
    try:
        with pytest.raises(RuntimeError):
            bad.propose(_Scene())
        assert bad.last_infer_ms is not None  # an error reply still says how long the call took
    finally:
        bad.close()


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


def test_the_worker_pins_its_onnxruntime_threads(monkeypatch):
    # ws1 timing: unpinned, onnxruntime sized its pool to the machine (24 cores) inside a 4-CPU allocation and a
    # round trip took 830 ms against 106 ms in-process at 4 threads
    seen = {}

    class FakePopen:
        def __init__(self, cmd, **kw):
            seen["cmd"] = cmd

    monkeypatch.setattr(W.subprocess, "Popen", FakePopen)
    W.WorkerProposer("py", "m.onnx", conf=0.05, threads=3)
    assert seen["cmd"][-2] == "3"
    built = {}
    monkeypatch.setattr(W, "serve", lambda proposer, stdout=None: None)
    monkeypatch.setattr(W, "protocol_stdout", lambda: None)  # never redirect the test process's own stdout
    import forklift_learned.proposer as P

    monkeypatch.setattr(P, "LearnedProposer", lambda *a, **kw: built.update(kw))
    W.main(["m.onnx", "0.05", "CPUExecutionProvider", "3"])
    assert built["threads"] == 3 and built["providers"] == ["CPUExecutionProvider"]


def test_the_worker_reports_the_providers_its_session_runs_on(tmp_path):
    script = tmp_path / "fake.py"
    script.write_text("from forklift_learned.worker import serve\n"
                      "class P:\n"
                      "    active_providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']\n"
                      "    def propose(self, scene):\n"
                      "        return []\n"
                      "serve(P())\n")
    w = W.WorkerProposer(None, None, command=[sys.executable, str(script)])
    try:
        assert w.providers() == ["CUDAExecutionProvider", "CPUExecutionProvider"]
        assert w.propose(_Scene()) == []  # the info request does not disturb the frames
    finally:
        w.close()


def test_the_worker_passes_the_cuda_device(monkeypatch):
    seen = {}

    class FakePopen:
        def __init__(self, cmd, **kw):
            seen["cmd"] = cmd

    monkeypatch.setattr(W.subprocess, "Popen", FakePopen)
    W.WorkerProposer("py", "m.onnx", providers="CUDAExecutionProvider,CPUExecutionProvider", threads=2, device_id=1)
    assert seen["cmd"][-2:] == ["2", "1"]
    built = {}
    monkeypatch.setattr(W, "serve", lambda proposer, stdout=None: None)
    monkeypatch.setattr(W, "protocol_stdout", lambda: None)  # never redirect the test process's own stdout
    import forklift_learned.proposer as P

    monkeypatch.setattr(P, "LearnedProposer", lambda *a, **kw: built.update(kw))
    W.main(["m.onnx", "0.05", "CUDAExecutionProvider,CPUExecutionProvider", "2", "1"])
    assert built["device_id"] == 1


def test_serve_warms_the_model_up_before_the_first_request(tmp_path):
    script = tmp_path / "fake.py"
    script.write_text("from forklift_learned.worker import serve\n"
                      "class P:\n"
                      "    warmed = 0\n"
                      "    active_providers = []\n"
                      "    def warm_up(self, n=3):\n"
                      "        P.warmed += n\n"
                      "    def propose(self, scene):\n"
                      "        from forklift_core.perception.instance_proposals import InstanceProposal\n"
                      "        import numpy as np\n"
                      "        return [InstanceProposal('l0', np.ones(scene.rgb.shape[:2], bool), P.warmed / 10, 'learned')]\n"
                      "serve(P())\n")
    w = W.WorkerProposer(None, None, command=[sys.executable, str(script)])
    try:
        assert w.propose(_Scene())[0].score == pytest.approx(0.3)
    finally:
        w.close()


def test_stray_stdout_output_does_not_corrupt_the_protocol(tmp_path):
    # Codex review (stage 3b): preload_dlls prints its diagnostics to stdout; unbuffered, they reached the client
    # before the reply and were read as a length
    script = tmp_path / "fake.py"
    script.write_text("import os, sys\n"
                      "from forklift_learned.worker import protocol_stdout, serve\n"
                      "out = protocol_stdout()\n"
                      "print('preload: could not load from the pip path, retrying', flush=True)\n"
                      "os.write(1, b'C-level noise\\n')\n"
                      "class P:\n"
                      "    active_providers = ['CPUExecutionProvider']\n"
                      "    def warm_up(self, n=3):\n"
                      "        print('warming', flush=True)\n"
                      "    def propose(self, scene):\n"
                      "        print('frame', flush=True)\n"
                      "        return []\n"
                      "serve(P(), stdout=out)\n")
    import os

    w = W.WorkerProposer(None, None, command=[sys.executable, str(script)],
                         env={**os.environ, "PYTHONUNBUFFERED": "1"})
    try:
        assert w.providers() == ["CPUExecutionProvider"]
        assert w.propose(_Scene()) == []
    finally:
        w.close()


def test_an_implausible_reply_length_is_an_error_not_an_allocation():
    import io

    with pytest.raises(RuntimeError, match="length"):
        W._recv(io.BytesIO(b"preload!" + b"x" * 16))
