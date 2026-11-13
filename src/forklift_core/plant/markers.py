"""Operator markers from the terminal (bring-up tool design 2; teleop record design 4): Enter marks an event, the text
typed before it is the label. Never blocks the loop that polls it: only the bytes there are are read (``os.read``
after ``select``), at most ``max_bytes`` and ``max_marks`` per poll, and only whole lines mark -- a ``readline`` would
wait for a newline after a label flushed with Ctrl-D. A line's time is when the loop read it, not when Enter was hit;
the gap is not bounded (a burst can wait several polls)."""

from __future__ import annotations

import os
import select
import sys
import time
from dataclasses import dataclass
from typing import Callable

MAX_BYTES = 4096
MAX_MARKS = 16
MAX_LABEL = 200
MAX_BUFFER = 4 * MAX_BYTES  # read no more while this much is waiting: a flood stays in the terminal, not here


def _cut_utf8(raw: bytes, limit: int) -> str:
    """At most ``limit`` bytes, cut on a UTF-8 character boundary (never half a character)."""
    if len(raw) > limit:
        raw = raw[:limit]
        while raw and (raw[-1] & 0xC0) == 0x80:  # a continuation byte: back off to the character's lead
            raw = raw[:-1]
        if raw and raw[-1] >= 0xC0:  # a lead byte whose continuation was cut
            raw = raw[:-1]
    return raw.decode("utf-8", "replace").strip()


@dataclass(frozen=True)
class MarkerReader:
    poll: Callable[[], None]
    active: bool
    reason: str | None


def marker_reader(emit: Callable[[str, int], None], fd: int | None = None, *, max_bytes: int = MAX_BYTES,
                  max_marks: int = MAX_MARKS, max_label: int = MAX_LABEL, max_buffer: int = MAX_BUFFER,
                  clock=time.monotonic_ns) -> MarkerReader:
    """``emit(label, host_ns)`` for every whole line. Inactive (a no-op poll and the reason) without a tty."""
    if fd is None:
        try:
            fd = sys.stdin.fileno()
        except (AttributeError, OSError, ValueError):
            return MarkerReader(lambda: None, False, "no stdin")
    if not os.isatty(fd):
        return MarkerReader(lambda: None, False, "stdin is not a terminal")
    buf = bytearray()
    state = {"eof": False, "pending": bytearray()}

    def poll():
        if state["eof"]:
            return
        read = marks = 0
        while read < max_bytes and len(buf) < max_buffer and select.select([fd], [], [], 0)[0]:
            try:
                chunk = os.read(fd, max_bytes - read)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                state["eof"] = True
                break
            if not chunk:
                break  # a Ctrl-D on an empty line (a tty: later lines still come) -- this poll is done
            read += len(chunk)
            buf.extend(chunk)
        now = clock()
        while marks < max_marks and (i := buf.find(b"\n")) >= 0:
            label = _cut_utf8(bytes(buf[:i]), max_label)
            del buf[: i + 1]
            emit(label, now)
            marks += 1
        if buf.find(b"\n") < 0 and len(buf) > max_label:  # an unfinished label: keep its first max_label bytes
            del buf[max_label:]

    return MarkerReader(poll, True, None)
