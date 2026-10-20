"""The live session's one file writer (persistent console plan D1 step 7, U29): the control thread only hands jobs
over; the writer does them in order, reports each keyed job's outcome, and is bounded."""

import threading
import time

from forklift_core.live.io_writer import OrderedWriter


def test_jobs_run_in_order_and_keyed_outcomes_are_reported():
    w = OrderedWriter()
    done = []
    w.put_line(lambda: done.append("a"), 1)
    assert w.put_keyed(1, lambda: done.append("m1") or [])
    w.put_line(lambda: done.append("b"), 1)
    assert w.put_keyed(2, lambda: ["result.json: OSError: full"])
    assert w.settle(5.0) == []
    assert done == ["a", "m1", "b"]
    assert w.outcomes() == [(1, []), (2, ["result.json: OSError: full"])] and w.outcomes() == []
    assert w.close(5.0)


def test_a_job_that_raises_is_an_outcome_and_the_writer_goes_on():
    """Codex U29 design P2-3: one failing job (a log write) must not stop every later job."""
    w = OrderedWriter()

    def boom():
        raise OSError("disk")

    w.put_line(boom, 1)
    w.put_keyed(1, lambda: (_ for _ in ()).throw(ValueError("bad")))
    done = []
    w.put_keyed(2, lambda: done.append(2) or [])
    assert w.settle(5.0) == [] and done == [2]
    assert w.outcomes() == [(1, ["ValueError: bad"]), (2, [])]
    assert w.line_errors == 1
    w.close(5.0)


def test_settle_waits_for_the_job_being_written_not_only_the_queue():
    """Codex U29 design P2-3: the queue is empty once the writer took the last job; the job is not done then."""
    w = OrderedWriter()
    gate, taken = threading.Event(), threading.Event()
    w.put_keyed(1, lambda: (taken.set(), gate.wait(5.0)) and [])
    assert taken.wait(5.0)
    assert w.settle(0.05) == [1]  # taken, being written: still pending at the deadline
    gate.set()
    assert w.settle(5.0) == []
    w.close(5.0)


def test_one_deadline_for_every_pending_job():
    w = OrderedWriter()
    gate = threading.Event()
    w.put_keyed(1, lambda: gate.wait(5.0) and [])
    w.put_keyed(2, lambda: [])
    began = time.monotonic()
    assert w.settle(0.1) == [1, 2]
    assert time.monotonic() - began < 0.5
    gate.set()
    w.close(5.0)


def test_a_full_writer_refuses_a_keyed_job_and_drops_lines_past_its_bytes_counting_them():
    """Codex U29 design P2-2: bounded -- the job being written counts -- and the hand-over never waits."""
    w = OrderedWriter(max_keyed=2, max_line_bytes=10)
    gate = threading.Event()
    assert w.put_keyed(1, lambda: gate.wait(5.0) and [])
    assert w.put_keyed(2, lambda: [])
    began = time.monotonic()
    assert not w.put_keyed(3, lambda: [])
    assert w.put_line(lambda: None, 6) and not w.put_line(lambda: None, 6)
    assert time.monotonic() - began < 0.05
    assert w.dropped_keyed == 1 and w.dropped_lines == 1
    gate.set()
    assert w.settle(5.0) == []
    w.close(5.0)


def test_close_drains_within_its_deadline_and_refuses_jobs_after():
    w = OrderedWriter()
    done = []
    for i in range(50):
        w.put_line(lambda i=i: done.append(i), 1)
    assert w.close(5.0) and done == list(range(50))
    assert not w.put_line(lambda: None, 1) and not w.put_keyed(9, lambda: [])


def test_a_close_past_its_deadline_returns_and_the_late_job_is_not_reported():
    w = OrderedWriter()
    gate = threading.Event()
    w.put_keyed(1, lambda: gate.wait(5.0) and [])
    began = time.monotonic()
    assert not w.close(0.1)
    assert time.monotonic() - began < 0.5
    gate.set()
    time.sleep(0.05)
    assert w.outcomes() == []  # closed: a late outcome does not come back


def test_the_pieced_encoding_is_the_one_shot_encoding():
    """Plan U29: a mission's records are encoded in pieces, so the writer gives the interpreter lock back between
    them (one json.dumps of a large record held it for its whole length against the control thread)."""
    import json

    import numpy as np

    from forklift_core.live.io_writer import encode_in_pieces

    def default(value):
        return value.tolist() if isinstance(value, np.ndarray) else str(value)

    records = {"state": {"scans": [[0.5 + k * 1e-6, k] for k in range(2000)], "a": np.arange(3), "o": object,
                         "n": None, "nested": [{"x": [1, 2, {"y": "z"}]}] * 700, "empty": [], "d": {}},
               "obstacle": {}, "pocket": {"t": (1, 2)}}
    assert "".join(encode_in_pieces(records, default, piece_items=64)) == json.dumps(records, default=default)


def test_a_disposed_value_is_let_go_off_the_caller_thread_after_its_delay():
    """Codex U29 implementation review round 2 P2-3: records a full writer refused were freed on the control thread
    (120,000 rows: 31 ms). The writer's disposer lets them go on its own thread, a delay after the hand-over, so the
    caller's own references are gone by then."""
    freed = []

    class Tracked:
        def __del__(self):
            freed.append(threading.current_thread().name)

    w = OrderedWriter(dispose_after_s=0.05)
    w.dispose([Tracked()])
    time.sleep(0.01)
    assert freed == []  # not before its delay
    deadline = time.monotonic() + 5.0
    while not freed and time.monotonic() < deadline:
        time.sleep(0.01)
    assert freed == ["live-io-free"]
    w.close(5.0)


def test_a_value_is_freed_gradually_and_a_shared_part_is_left_alone():
    """Codex U29 implementation review round 2: freeing a large record in one go held the interpreter lock (57 ms
    late wake-ups of the control thread in the gate). Only containers nothing else holds are emptied, a bit at a time;
    a part shared with live code is only let go."""
    from forklift_core.live.io_writer import free_gradually

    shared = {"keep": [1, 2, 3]}
    value = {"rows": [[k, k + 0.5] for k in range(5000)], "shared": shared, "t": ([1], [2])}
    holder = [value]
    del value
    free_gradually(holder)
    assert holder == [] and shared == {"keep": [1, 2, 3]}


def test_the_pieces_stream_to_the_file_atomically(tmp_path):
    import json

    from forklift_core.live.io_writer import encode_in_pieces, write_pieces_atomic

    value = {"rows": [[k, k * 0.5] for k in range(3000)]}
    size = write_pieces_atomic(tmp_path / "r.json", encode_in_pieces(value, str))
    assert json.loads((tmp_path / "r.json").read_text()) == value and size == (tmp_path / "r.json").stat().st_size
    assert not (tmp_path / "r.json.tmp").exists()


def test_the_yielder_sleeps_only_after_its_work_time():
    from forklift_core.live.io_writer import Yielder

    y = Yielder(work_s=10.0, rest_s=0.5)
    began = time.perf_counter()
    for _ in range(1000):
        y.maybe()
    assert time.perf_counter() - began < 0.1
