"""Real-time plan G2-1a: the capture's diagnostic files written in place without a writer or with a boundary record
open; handed over on copies with one; written in place when the writer refuses them; a failing write kept on its
entry, never raised into the mission."""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))
from capture_dump import dispatch_capture_dump  # noqa: E402


class Writer:
    def __init__(self, accept=True):
        self.accept, self.jobs = accept, []

    def put_line(self, job, nbytes):
        self.jobs.append((job, nbytes))
        return self.accept


def _arrays():
    return np.zeros((4, 5, 3), np.uint8), np.full((4, 5), 1.5)


def test_without_a_writer_or_with_a_record_open_it_writes_in_place():
    rgb, depth = _arrays()
    seen, entries = [], []
    write = lambda n, r, d: seen.append((n, r, d))  # noqa: E731
    assert dispatch_capture_dump(None, False, write, 1, rgb, depth, entries) == "sync"
    assert dispatch_capture_dump(Writer(), True, write, 2, rgb, depth, entries) == "sync"
    assert [s[0] for s in seen] == [1, 2] and seen[0][1] is rgb and entries == []


def test_a_writer_gets_copies_and_writes_them_later():
    rgb, depth = _arrays()
    seen, entries, w = [], [], Writer()
    assert dispatch_capture_dump(w, False, lambda n, r, d: seen.append((n, r.copy(), d.copy())), 3, rgb, depth,
                                 entries) == "async"
    assert seen == [] and entries == [{"attempt": 3, "mode": "async"}]
    (job, nbytes), = w.jobs
    assert nbytes == rgb.nbytes + depth.nbytes
    rgb[:] = 9  # the capture changes after the hand-over: the files keep what it was
    depth[:] = 0.0
    job()
    assert seen[0][0] == 3 and not seen[0][1].any() and (seen[0][2] == 1.5).all()


def test_a_refusing_writer_means_written_in_place():
    rgb, depth = _arrays()
    seen, entries = [], []
    assert dispatch_capture_dump(Writer(accept=False), False, lambda n, r, d: seen.append(n), 4, rgb, depth,
                                 entries) == "sync_refused"
    assert seen == [4] and entries == [{"attempt": 4, "mode": "sync_refused"}]


def test_a_failing_write_on_the_writer_is_kept_not_raised(capsys):
    rgb, depth = _arrays()
    entries, w = [], Writer()

    def write(n, r, d):
        raise OSError("disk full")
    dispatch_capture_dump(w, False, write, 5, rgb, depth, entries)
    w.jobs[0][0]()  # the writer thread's call: no exception escapes
    assert entries == [{"attempt": 5, "mode": "async", "error": "OSError: disk full"}]
    assert "capture dump 5 failed" in capsys.readouterr().err
