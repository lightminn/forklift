"""Write-to-acceptance matching (docs/design/2026-11-25-bench-commission-limits.md 1.2, PR-2a)."""

from forklift_core.plant.write_log import Seen, Write, accepted

BOOT = 0xB007


def w(order, seq, done, *, conn=1, kind="COMMISSION", duty=1000, echo=0):
    return Write(order=order, conn=conn, kind=kind, seq=seq, start_ns=done - 10, done_ns=done, boot_id=BOOT,
                 echo_mcu_time_us=echo, axis=0, duty_e4=duty)


def s(t, seq, *, conn=1, mcu=None, boot=BOOT):
    return Seen(host_ns=t, conn=conn, boot_id=boot, mcu_time_us=t if mcu is None else mcu, last_rx_seq=seq)


def nonzero(x):
    return x.kind == "COMMISSION" and x.duty_e4 != 0


def test_a_plain_acceptance():
    writes = [w(0, 5, 100, duty=0), w(1, 6, 200)]
    samples = [s(150, 5), s(250, 6)]
    m = accepted(samples, writes, nonzero)
    assert m.index == 1 and m.write.order == 1 and m.ambiguous_at is None


def test_a_write_not_yet_done_or_echoed_later_is_not_the_one():
    writes = [w(0, 6, 300)]  # finished after the STATE was read
    assert accepted([s(250, 6)], writes, nonzero).index is None
    writes = [w(0, 6, 100, echo=500)]  # echoed an MCU time the STATE has not reached
    assert accepted([s(250, 6, mcu=400)], writes, nonzero).index is None
    writes = [w(0, 6, 100)]
    assert accepted([s(250, 6, boot=0xBEEF)], writes, nonzero).index is None  # another boot


def test_the_same_seq_written_twice_is_told_apart_when_something_else_was_accepted_in_between():
    """After the 16-bit wrap: the older write with seq 7 was followed by an accepted seq 8, so a STATE naming 7 after
    the newer write means the newer one."""
    writes = [w(0, 7, 100, duty=0), w(1, 8, 200, duty=0), w(65536, 7, 1000)]
    samples = [s(150, 7), s(250, 8), s(1100, 7)]
    m = accepted(samples, writes, nonzero)
    assert m.index == 2 and m.write.order == 65536


def test_a_gap_in_the_samples_leaves_it_unjudgeable():
    """No STATE between the two writes named another seq (samples missing): either write could be the accepted one."""
    writes = [w(0, 7, 100, duty=0), w(65536, 7, 1000)]
    samples = [s(150, 7), s(1100, 7)]
    m = accepted(samples, writes, nonzero)
    assert m.index is None and m.ambiguous_at == 1


def test_an_ambiguity_that_cannot_touch_the_selected_writes_is_passed_over():
    writes = [w(0, 7, 100, kind="DISARM", duty=None), w(65536, 7, 1000, kind="DISARM", duty=None), w(65537, 9, 1200)]
    samples = [s(150, 7), s(1100, 7), s(1300, 9)]
    m = accepted(samples, writes, nonzero)
    assert m.index == 2 and m.ambiguous_at is None


def test_a_same_boot_reconnect_does_not_count_the_old_connections_write():
    """A USB-only reconnect: the MCU (same boot) reports seq 6 on the new fd; the host's seq 6 went out on the old one."""
    writes = [w(0, 6, 100, conn=1)]
    samples = [s(150, 5, conn=1), s(400, 6, conn=2)]
    assert accepted(samples, writes, nonzero).index is None
    writes.append(w(1, 6, 350, conn=2))
    m = accepted(samples, writes, nonzero)
    assert m.index == 1 and m.write.conn == 2


def test_start_skips_earlier_samples():
    writes = [w(0, 5, 100), w(1, 6, 200, duty=0), w(2, 7, 300)]
    samples = [s(150, 5), s(250, 6), s(350, 7)]
    assert accepted(samples, writes, nonzero, start=1).index == 2


def test_disarm_matching_without_an_echo():
    writes = [w(0, 9, 100, kind="DISARM", duty=None)]
    writes[0] = Write(order=0, conn=1, kind="DISARM", seq=9, start_ns=90, done_ns=100)
    m = accepted([s(150, 9)], writes, lambda x: x.kind == "DISARM")
    assert m.index == 0
