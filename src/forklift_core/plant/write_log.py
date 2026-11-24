"""Which recorded STATE accepted which written frame (docs/design/2026-11-25-bench-commission-limits.md 1.2, PR-2a).

``STATE.last_rx_seq`` names the last frame the MCU accepted, as a 16-bit sequence number. At 100 Hz the host's seq
wraps in about 655 s, so a long run writes the same seq more than once; and a USB-only reconnect brings STATEs of the
same boot on a new file descriptor. A seq alone cannot say which write a STATE means. This module matches with what
the record keeps for every write -- its order, its connection, when its write finished and the command's own fields --
and says "ambiguous" instead of guessing.

Rules for a candidate write ``w`` of a STATE sample ``s`` (``s`` read at ``host_ns`` on connection ``conn``):
- the same seq, the same connection, and the write finished before the STATE was read;
- a command carrying an echo (ARM, COMMISSION, DRIVE) was echoed at or before the STATE's MCU time, from the same boot;
- the latest such write is the candidate; an older write with the same seq is ruled out only if some STATE on that
  connection, read after the older write finished and before ``s``, already named another seq (the MCU accepted
  something else after it, so the older one cannot be the last accepted again). Otherwise the match is ambiguous.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass


@dataclass(frozen=True)
class Write:
    order: int  # 0, 1, ... in write order over the whole run
    conn: int  # the connection (file descriptor opening) it was written on
    kind: str  # ARM, DISARM, COMMISSION, DRIVE, CLEAR_FAULTS
    seq: int
    start_ns: int  # when its first byte was about to be written (the host's existing write time)
    done_ns: int  # when its last byte was written
    boot_id: int | None = None  # the command's boot_id field (None: DISARM)
    echo_mcu_time_us: int | None = None
    axis: int | None = None  # COMMISSION
    duty_e4: int | None = None  # COMMISSION


@dataclass(frozen=True)
class Seen:
    """A recorded STATE as the matcher needs it."""

    host_ns: int  # read time
    conn: int
    boot_id: int
    mcu_time_us: int
    last_rx_seq: int


@dataclass(frozen=True)
class Match:
    index: int | None  # the first sample at or after ``start`` that accepted a write the predicate selects
    write: Write | None
    ambiguous_at: int | None = None  # a sample whose seq could not be told apart (the match is then unjudgeable)


def _index(writes: list[Write]) -> dict[tuple[int, int], list[Write]]:
    by: dict[tuple[int, int], list[Write]] = {}
    for w in sorted(writes, key=lambda w: w.order):
        by.setdefault((w.conn, w.seq), []).append(w)
    return by


def _superseded(samples: list[Seen], i: int, older: Write) -> bool:
    """Some STATE on the connection, read after ``older`` finished and before sample ``i``, named another seq."""
    s = samples[i]
    for j in range(i - 1, -1, -1):
        o = samples[j]
        if o.host_ns < older.done_ns:
            break  # samples are in read order: nothing earlier can be after the write
        if o.conn == s.conn and o.last_rx_seq != s.last_rx_seq:
            return True
    return False


def _candidates(samples: list[Seen], i: int, by: dict) -> tuple[Write | None, list[Write]]:
    """The latest matching write for sample ``i`` and the older ones it cannot be told apart from."""
    s = samples[i]
    same = [w for w in by.get((s.conn, s.last_rx_seq), ()) if w.done_ns <= s.host_ns
            and (w.echo_mcu_time_us is None or (w.boot_id == s.boot_id and w.echo_mcu_time_us <= s.mcu_time_us))]
    if not same:
        return None, []
    latest = same[-1]  # the index keeps write order
    unresolved = [older for older in same[:-1] if not _superseded(samples, i, older)]
    return latest, unresolved


def accepted(samples: list[Seen], writes: list[Write], pred: Callable[[Write], bool], start: int = 0) -> Match:
    """The first sample from ``start`` whose accepted frame is a write ``pred`` selects. A sample whose seq could
    name either a selected write or another one stops the search: the caller reports the run as unjudgeable rather
    than pick a frame. ``samples`` are in read order."""
    by = _index(writes)
    for i in range(start, len(samples)):
        latest, unresolved = _candidates(samples, i, by)
        if latest is None:
            continue
        if not any(pred(w) for w in [latest, *unresolved]):
            continue
        if unresolved:
            return Match(None, None, ambiguous_at=i)
        return Match(i, latest)
    return Match(None, None)
