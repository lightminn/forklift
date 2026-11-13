"""Operator markers (record design 4): whole lines only, never blocking, a bounded amount per poll, inactive with a
reason off a terminal."""

from __future__ import annotations

import os
import time

from forklift_core.plant.markers import marker_reader


def pty_reader(**kw):
    master, slave = os.openpty()
    marks = []
    r = marker_reader(lambda label, ns: marks.append((label, ns)), fd=slave, **kw)
    return master, slave, r, marks


def test_lines_mark_with_their_labels_and_a_partial_line_waits():
    master, slave, r, marks = pty_reader()
    try:
        assert r.active and r.reason is None
        os.write(master, "m1 시작".encode()[:5])  # a label split inside a UTF-8 character
        time.sleep(0.05)
        r.poll()
        assert marks == []
        os.write(master, "m1 시작".encode()[5:] + b"\n\nend\n")
        time.sleep(0.05)
        r.poll()
        assert [m[0] for m in marks] == ["m1 시작", "", "end"]
    finally:
        os.close(master)
        os.close(slave)


def test_a_burst_is_taken_a_bounded_amount_per_poll_and_a_long_label_is_cut():
    master, slave, r, marks = pty_reader(max_marks=4, max_label=10)
    try:
        os.write(master, b"".join(f"l{k}\n".encode() for k in range(10)))
        time.sleep(0.05)
        t0 = time.monotonic()
        r.poll()
        assert len(marks) == 4 and time.monotonic() - t0 < 0.02
        r.poll()
        r.poll()
        assert [m[0] for m in marks][:10] == [f"l{k}" for k in range(10)]
        os.write(master, b"x" * 50 + b"\n")
        time.sleep(0.05)
        r.poll()
        assert marks[-1][0] == "x" * 10
    finally:
        os.close(master)
        os.close(slave)


def test_off_a_terminal_the_reader_is_inactive_and_says_why(tmp_path):
    f = open(tmp_path / "in.txt", "w+")
    try:
        r = marker_reader(lambda label, ns: None, fd=f.fileno())
        assert not r.active and "terminal" in r.reason
        r.poll()  # a no-op
    finally:
        f.close()
