"""Write-to-acceptance matching (docs/design/2026-11-25-bench-commission-limits.md 1.2, PR-2a): acceptances are
transitions of last_rx_seq, matched to writes of the same connection, boot, generation and time window."""

from forklift_core.plant.write_log import (
    TRANSIT_NS,
    Seen,
    Write,
    accepted,
    seen_from_rows,
    write_row,
    writes_from_rows,
)

BOOT = 0xB007
MS = 1_000_000


def w(order, seq, done, *, conn=1, kind="COMMISSION", duty=1000, echo=None, valid=60, gen=1, phase="on"):
    echo = done // 1000 if echo is None and kind != "DISARM" else echo
    if kind == "DISARM":
        return Write(order=order, conn=conn, kind=kind, seq=seq, start_ns=done - 10, done_ns=done, phase=phase)
    return Write(order=order, conn=conn, kind=kind, seq=seq, start_ns=done - 10, done_ns=done, boot_id=BOOT,
                 echo_mcu_time_us=echo, valid_ms=valid, arm_gen=gen, axis=0, duty_e4=duty, phase=phase)


def s(t, seq, *, conn=1, boot=BOOT, gen=1):
    """A STATE read at host time ``t`` ns; its MCU time is the same number of microseconds (a shared origin)."""
    return Seen(host_ns=t, conn=conn, boot_id=boot, mcu_time_us=t // 1000, last_rx_seq=seq, arm_gen=gen)


def nonzero(x):
    return x.kind == "COMMISSION" and x.duty_e4 != 0


def test_a_plain_acceptance_is_a_transition():
    writes = [w(0, 5, 100 * MS, duty=0), w(1, 6, 200 * MS)]
    samples = [s(150 * MS, 5), s(250 * MS, 6)]
    m = accepted(samples, writes, nonzero)
    assert m.index == 1 and m.write.order == 1


def test_no_transition_is_no_acceptance():
    """The first sample of a connection proves nothing, and neither does a seq that stays (Codex PR-2a P2-2)."""
    writes = [w(0, 6, 100 * MS)]
    assert accepted([s(150 * MS, 6)], writes, nonzero).index is None
    assert accepted([s(150 * MS, 6), s(160 * MS, 6)], writes, nonzero).index is None


def test_a_reboot_between_samples_is_no_transition():
    writes = [w(0, 0, 100 * MS, kind="DISARM")]
    samples = [s(90 * MS, 5), s(150 * MS, 0, boot=0x0BAD)]  # a new boot reports last_rx_seq 0
    assert accepted(samples, writes, lambda x: x.kind == "DISARM").index is None


def test_the_write_must_fall_in_the_transitions_window():
    late = [w(0, 6, 300 * MS)]  # finished after the STATE was read
    assert accepted([s(150 * MS, 5), s(250 * MS, 6)], late, nonzero).index is None
    early = [w(0, 6, 100 * MS - TRANSIT_NS - 1)]  # long before the previous read: it would already show
    assert accepted([s(100 * MS, 5), s(250 * MS, 6)], early, nonzero).index is None


def test_an_expired_or_wrong_generation_command_is_not_the_one():
    expired = [w(0, 6, 100 * MS, echo=100_000, valid=10)]  # valid until 110 ms MCU, over before the previous STATE
    assert accepted([s(150 * MS, 5), s(250 * MS, 6)], expired, nonzero).index is None
    other_gen = [w(0, 6, 200 * MS, gen=2)]
    assert accepted([s(150 * MS, 5), s(250 * MS, 6, gen=1)], other_gen, nonzero).index is None
    assert accepted([s(150 * MS, 5), s(250 * MS, 6, boot=0xBEEF)], [w(0, 6, 200 * MS)], nonzero).index is None


def test_the_same_seq_written_twice_is_told_apart_by_its_window():
    """After the 16-bit wrap (655 s later) the old write with seq 7 is far outside the transition's window."""
    writes = [w(0, 7, 100 * MS, duty=0), w(65536, 7, 655_100 * MS)]
    samples = [s(150 * MS, 7), s(655_050 * MS, 6), s(655_150 * MS, 7)]
    m = accepted(samples, writes, nonzero)
    assert m.index == 2 and m.write.order == 65536


def test_a_gap_longer_than_a_wrap_leaves_it_unjudgeable():
    """Samples missing for longer than a seq wrap: both writes with seq 7 fall in the window."""
    writes = [w(0, 7, 100 * MS, duty=0, valid=None), w(65536, 7, 655_100 * MS, valid=None)]
    samples = [s(50 * MS, 3), s(655_150 * MS, 7)]
    m = accepted(samples, writes, nonzero)
    assert m.index is None and m.ambiguous_at == 1


def test_a_same_boot_reconnect_does_not_count_the_old_connections_write():
    """A USB-only reconnect: the MCU (same boot) reports seq 6 on the new fd; the host's seq 6 went out on the old one."""
    writes = [w(0, 6, 100 * MS, conn=1)]
    samples = [s(90 * MS, 5, conn=1), s(400 * MS, 5, conn=2), s(410 * MS, 6, conn=2)]
    assert accepted(samples, writes, nonzero).index is None
    writes.append(w(1, 6, 405 * MS, conn=2))
    m = accepted(samples, writes, nonzero)
    assert m.index == 2 and m.write.conn == 2


def test_start_skips_earlier_transitions():
    writes = [w(0, 5, 100 * MS), w(1, 6, 200 * MS, duty=0), w(2, 7, 300 * MS)]
    samples = [s(90 * MS, 4), s(150 * MS, 5), s(250 * MS, 6), s(350 * MS, 7)]
    assert accepted(samples, writes, nonzero).index == 1
    assert accepted(samples, writes, nonzero, start=2).index == 3


def test_a_record_redoes_the_same_match_from_its_rows():
    """The JSONL rows alone are enough to redo a match (the record contract)."""
    writes = [w(0, 5, 100 * MS, duty=0, phase="pre_zero"), w(1, 6, 200 * MS)]
    samples = [s(150 * MS, 5), s(250 * MS, 6)]
    rows = [{"t": "written", "host_ns": x.start_ns, **write_row(x)} for x in writes]
    rows += [{"t": "state", "host_ns": x.host_ns, "conn": x.conn, "boot_id": x.boot_id, "mcu_time_us": x.mcu_time_us,
              "last_rx_seq": x.last_rx_seq, "arm_gen": x.arm_gen} for x in samples]
    assert writes_from_rows(rows) == writes and seen_from_rows(rows) == samples
    live = accepted(samples, writes, lambda x: x.phase == "on")
    redone = accepted(seen_from_rows(rows), writes_from_rows(rows), lambda x: x.phase == "on")
    assert live == redone and live.index == 1
