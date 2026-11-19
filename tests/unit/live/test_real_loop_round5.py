"""Codex 4c impl round 5: no motion frame once the permit latched (a half DRIVE included), the boundary's check
reading the channels without a deadlock, a slow evidence read refused, the wait's target held through first-byte
refusals, one DISARM per tick through a rejected stuck S10 stop, the record naming what was really sent."""

from dataclasses import replace

from forklift_core.live import real_autonomy as ra
from forklift_core.mcu_link.messages import RejectReason

from real_loop_rig import MS, Rig


def run_until(r, cond, limit=4000):
    for _ in range(limit):
        rec = r.tick()
        if cond(rec):
            return rec
    raise AssertionError(f"not reached: {r.loop.records[-1]}")


def driving(**kw):
    r = Rig(frames=True, **kw)
    r.start()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING and r.loop.book.confirmed is not None
              and r.loop.book.confirmed.speed_mps != 0.0)
    return r


def test_a_half_forward_drive_is_not_finished_after_a_console_disarm():
    r = driving()
    r.transport.stall = True
    r.tick()
    partial = r.transport.partial_frame
    assert partial is not None and partial.kind == "DRIVE"
    r.transport.stall = False
    r.say("disarm")
    k = len(r.transport.sent)
    r.tick()
    sent = [(kind, seq) for _, kind, seq, _ in r.transport.sent[k:]]
    assert sent and sent[0][0] == "DISARM" and (partial.kind, partial.seq) not in sent


def test_the_boundarys_check_may_read_the_channels():
    r = driving()
    ch = r.channels
    r.loop.evidence = replace(r.loop.evidence, link_reason=lambda n: (ch.serial(), None)[1])
    rec = r.tick()  # a non-reentrant lock would hang here
    assert rec.branch in ("auto", "wait")


def test_an_evidence_read_slower_than_its_bound_refuses_the_write():
    r = driving()
    ev = r.loop.evidence
    calls = {"n": 0}

    def slow_snapshot(n):
        calls["n"] += 1
        if calls["n"] > 1:  # the first byte's read (after the tick's own)
            r.tick_cost_ns += 10 * MS
        return ev.snapshot(n)

    r.loop.evidence = replace(ev, snapshot=slow_snapshot)
    k = len(r.transport.sent)
    r.tick()
    r.tick_cost_ns = 0
    assert not [m for _, kind, _, m in r.transport.sent[k:] if kind == "DRIVE" and m.left_wheel_rad_s]
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)


def test_the_wait_target_holds_through_repeated_first_byte_refusals():
    """Round 5 P1-4: each tick's moving candidate refused at its first byte (35 ms) -- the wait goes on at the target
    it took first, on the same generation, while the measured lift sags."""
    r = driving()
    real = r.transport.transmit

    def slow(host, now, gate):
        r.tick_cost_ns = 35 * MS
        try:
            return real(host, now, gate)
        finally:
            r.tick_cost_ns = 0

    r.transport.transmit = slow
    rec = r.tick()
    assert rec.branch == "wait"
    target, gen = r.loop.waiting, r.port.control_gen
    assert target is not None
    for _ in range(3):
        r.mcu.lift -= 0.001  # the forks sag a little
        rec = r.tick()
        assert rec.branch == "wait" and r.loop.waiting == target and r.port.control_gen == gen


def test_one_disarm_per_tick_through_a_rejected_stuck_s10_stop():
    r = Rig(frames=True, operator=[(1, "confirm")])
    r.start()
    r.submit()
    real = r.transport.transmit

    def stall_s10(host, now, gate):
        if r.permit.state == ra.STOP_PENDING and not r.transport.resyncs:
            r.transport.stall = True
            r.mcu.reject = RejectReason(1)
        return real(host, now, gate)

    r.transport.transmit = stall_s10
    run_until(r, lambda rec: rec.branch == "s10")
    r.tick_cost_ns = 65 * MS
    for _ in range(8):
        k = len(r.transport.sent)
        r.tick()
        assert len({seq for _, kind, seq, _ in r.transport.sent[k:] if kind == "DISARM"}) <= 1
    r.tick_cost_ns = 0
    assert r.permit.reason != "s10_stopped"


def test_a_tick_turned_into_a_wait_at_the_first_byte_records_the_wait():
    r = driving()
    ev = r.loop.evidence
    real = r.transport.transmit

    def snapshot_now(host, now, gate):
        r.loop.evidence = replace(ev, snapshot=lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), n, True,
                                                                        0, 0))
        return real(host, now, gate)

    r.transport.transmit = snapshot_now
    rec = r.tick()
    r.transport.transmit, r.loop.evidence = real, ev
    written = {seq for _, kind, seq, _ in r.transport.sent if kind == "DRIVE"}
    assert rec.branch == "wait" and rec.seqs and set(rec.seqs) <= written
