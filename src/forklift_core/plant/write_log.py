"""Which recorded STATE accepted which written frame (docs/design/2026-11-25-bench-commission-limits.md 1.2, PR-2a).

``STATE.last_rx_seq`` names the last frame the MCU accepted, as a 16-bit sequence number. At 100 Hz the host's seq
wraps in about 655 s, so a long run writes the same seq more than once; and a USB-only reconnect brings STATEs of the
same boot on a new file descriptor. A seq alone cannot say which write a STATE means. This module matches with what
the record keeps for every write -- its order, its connection, when its write finished, its phase and the command's
own fields -- and says "ambiguous" instead of guessing.

An acceptance is a **transition**: a sample ``s`` whose ``last_rx_seq`` differs from the previous sample ``p`` read on
the same connection from the same boot (a first sample has no ``p`` and proves nothing). The frame accepted between
them is a write ``w`` with
- the same seq and connection, its last byte written no later than ``s`` was read, and no earlier than
  ``TRANSIT_NS`` before ``p`` was read (a frame is taken within milliseconds of arriving: one written long before
  ``p`` was read would already show in ``p``);
- for a command that carries them (ARM, COMMISSION, DRIVE): the same boot, its echo not later than ``s``'s MCU time,
  and its validity window (echo .. echo + valid) not over before ``p``'s MCU time; a COMMISSION or DRIVE also names
  the arm generation ``s`` reports.
One such write is the match; none is no acceptance of ours; more than one (only after a gap in the samples longer
than a seq wrap) is ambiguous.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

TRANSIT_NS = 1_000_000_000  # how long before the previous read a frame may have been written and still be in flight


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
    valid_ms: int | None = None  # valid_after_echo_ms (COMMISSION, DRIVE); None: not bounded here
    arm_gen: int | None = None  # COMMISSION, DRIVE
    axis: int | None = None  # COMMISSION
    duty_e4: int | None = None  # COMMISSION
    phase: str | None = None  # the tool's step it was queued in (pre_zero, on, post_zero, cleanup, ...)


@dataclass(frozen=True)
class Seen:
    """A recorded STATE as the matcher needs it."""

    host_ns: int  # read time
    conn: int
    boot_id: int
    mcu_time_us: int
    last_rx_seq: int
    arm_gen: int = 0


@dataclass(frozen=True)
class Match:
    index: int | None  # the first sample at or after ``start`` that accepted a write the predicate selects
    write: Write | None
    ambiguous_at: int | None = None  # a sample whose acceptance could not be told apart (then unjudgeable)


def _index(writes: list[Write]) -> dict[tuple[int, int], list[Write]]:
    by: dict[tuple[int, int], list[Write]] = {}
    for w in sorted(writes, key=lambda w: w.order):
        by.setdefault((w.conn, w.seq), []).append(w)
    return by


def _fits(w: Write, p: Seen, s: Seen) -> bool:
    if not (p.host_ns - TRANSIT_NS <= w.done_ns <= s.host_ns):
        return False
    if w.echo_mcu_time_us is None:
        return True
    if w.boot_id != s.boot_id or w.echo_mcu_time_us > s.mcu_time_us:
        return False
    if w.valid_ms is not None and w.echo_mcu_time_us + w.valid_ms * 1000 < p.mcu_time_us:
        return False  # its validity was over before the previous STATE: it cannot have been taken after it
    return w.arm_gen is None or w.arm_gen == s.arm_gen


def transitions(samples: list[Seen]):
    """(index, previous sample) of every sample whose accepted seq changed since the previous one read on its
    connection from the same boot."""
    last: dict[int, Seen] = {}
    for i, s in enumerate(samples):
        p = last.get(s.conn)
        last[s.conn] = s
        if p is not None and p.boot_id == s.boot_id and p.last_rx_seq != s.last_rx_seq:
            yield i, p


def accepted(samples: list[Seen], writes: list[Write], pred: Callable[[Write], bool], start: int = 0) -> Match:
    """The first transition at or after ``start`` whose accepted frame is a write ``pred`` selects. A transition whose
    frame could be either a selected write or another one stops the search: the caller reports the run as unjudgeable
    rather than pick a frame. ``samples`` are in read order."""
    by = _index(writes)
    for i, p in transitions(samples):
        if i < start:
            continue
        s = samples[i]
        cands = [w for w in by.get((s.conn, s.last_rx_seq), ()) if _fits(w, p, s)]
        if not any(pred(w) for w in cands):
            continue
        if len(cands) > 1:
            return Match(None, None, ambiguous_at=i)
        return Match(i, cands[0])
    return Match(None, None)


_WRITE_FIELDS = ("order", "conn", "kind", "seq", "start_ns", "done_ns", "boot_id", "echo_mcu_time_us", "valid_ms",
                 "arm_gen", "axis", "duty_e4", "phase")


def write_row(w: Write) -> dict:
    """A write's record fields (the bring-up JSONL "written" row carries them; ``start_ns`` is also its host_ns)."""
    return {name: getattr(w, name) for name in _WRITE_FIELDS}


def writes_from_rows(rows: list[dict]) -> list[Write]:
    """The write log of a JSONL record, for offline matching."""
    return [Write(**{k: r.get(k) for k in _WRITE_FIELDS}) for r in rows if r.get("t") == "written" and "order" in r]


def seen_from_rows(rows: list[dict]) -> list[Seen]:
    """The STATE rows of a JSONL record (read time, connection and the fields matching needs)."""
    return [Seen(r["host_ns"], r["conn"], r["boot_id"], r["mcu_time_us"], r["last_rx_seq"], r.get("arm_gen", 0))
            for r in rows if r.get("t") == "state" and "conn" in r]
