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
