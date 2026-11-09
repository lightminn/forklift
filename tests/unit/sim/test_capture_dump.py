"""Real-time plan G2-1a: the capture dumper writes on a thread of its own, in one order for one path, and says what it
did: a record open writes in place; a handed-over capture is written later from copies; a refusal waits for the queue
and writes in place after it; a close past its deadline marks what was not written; a failure is kept, never raised
(Codex G2-1a review round 1: P1 records, P2-2 its own writer, P2-3 close, P2-4 order)."""

import sys
import threading
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))
from capture_dump import CaptureDumper  # noqa: E402


def _arrays(v=0):
    return np.full((4, 5, 3), v, np.uint8), np.full((4, 5), 1.5 + v)


class Files:
    """The write function: what each path got, in the order the writes finished."""

    def __init__(self, gate=None):
        self.got, self.gate = [], gate

    def __call__(self, n, rgb, depth):
        if self.gate is not None:
            self.gate.wait(5)
        self.got.append((n, int(rgb[0, 0, 0]), float(depth[0, 0])))


def test_a_record_open_writes_in_place():
    files = Files()
    d = CaptureDumper(files)
    rgb, depth = _arrays()
    assert d.dump(1, rgb, depth, record_open=True) == "sync"
    assert files.got == [(1, 0, 1.5)] and d.entries == []
    assert d.close(5.0) is True


def test_handed_over_captures_are_written_later_from_copies():
    gate = threading.Event()
    files = Files(gate)
    d = CaptureDumper(files)
    rgb, depth = _arrays()
    assert d.dump(1, rgb, depth) == "async"
    rgb[:] = 9  # the capture changes after the hand-over: its files keep what it was
    depth[:] = 0.0
    gate.set()
    assert d.close(5.0) is True
    assert files.got == [(1, 0, 1.5)]
    rec = d.record()
    assert (rec["closed"], rec["refused"], rec["errors"], rec["unsettled"]) == (True, 0, 0, 0)
    assert rec["entries"] == [{"attempt": 1, "mode": "async", "done": True}]


def test_a_refusal_waits_for_the_queue_so_one_path_keeps_its_order():
    gate = threading.Event()
    files = Files(gate)
    d = CaptureDumper(files, max_pending=1, settle_s=5.0)
    a = _arrays(1)
    assert d.dump(1, *a) == "async"  # an earlier mission's attempt 1, still waiting on the disk
    threading.Timer(0.2, gate.set).start()
    assert d.dump(1, *_arrays(2)) == "sync_refused"  # the next mission's attempt 1: same path
    assert files.got == [(1, 1, 2.5), (1, 2, 3.5)]  # the old one first, the new one last: the file is the new one
    assert "order_unsafe" not in d.entries[1]
    d.close(5.0)


def test_a_close_past_its_deadline_marks_what_was_not_written():
    gate = threading.Event()
    files = Files(gate)
    d = CaptureDumper(files)
    d.dump(1, *_arrays())
    assert d.close(0.1) is False
    rec = d.record()
    assert rec["unsettled"] == 1 and rec["entries"][0]["unsettled"] is True
    gate.set()


def test_a_failing_write_is_kept_not_raised(capsys):
    def write(n, r, dd):
        raise OSError("disk full")
    d = CaptureDumper(write)
    d.dump(5, *_arrays())
    d.close(5.0)
    assert d.record()["errors"] == 1 and d.entries[0]["error"] == "OSError: disk full"
    assert "unsettled" not in d.entries[0]
    assert "capture dump 5 failed" in capsys.readouterr().err


def test_its_writer_is_not_the_live_logs():
    from forklift_core.live.io_writer import OrderedWriter

    log = OrderedWriter()
    d = CaptureDumper(Files())
    assert d.io is not log and d.io._thread.name == "live-capture"
    d.dump(1, *_arrays())
    d.close(5.0)
    assert log.dropped_lines == 0
    log.close(5.0)
