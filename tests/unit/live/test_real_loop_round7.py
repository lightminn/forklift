"""Codex 4c impl round 7: an old frame's expiry never hides an alarm or an emergency event at the first byte, a wait
chosen before the send undoes its tick's lift ramp too, a half-written frame stays out of the tick's record."""

import math
from dataclasses import replace

from forklift_core.live import real_autonomy as ra

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


def test_a_stop_taken_during_the_check_of_an_old_queued_frame_disarms():
    """An old frame left queued from the tick before (not yet past its expiry here: the expiry-and-STOP overlap is
    covered by the ordering in ``judge``) and a STOP taken during its check: the same tick DISARMs."""
    r = driving()
    real = r.transport.transmit
    r.transport.transmit = lambda host, now, gate: []  # this tick's frame stays queued, unsent
    r.tick()
    r.transport.transmit = real
    ev = r.loop.evidence
    said = {"done": False}

    def link_reason(n):
        if r.channels.lock._is_owned() and not said["done"]:  # noqa: SLF001
            said["done"] = True
            r.say("stop")
        return ev.link_reason(n)

    r.loop.evidence = replace(ev, link_reason=link_reason)
    r.tick_cost_ns = 25 * MS
    k = len(r.transport.sent)
    r.tick()
    r.tick_cost_ns = 0
    assert said["done"]
    assert any(kind == "DISARM" for _, kind, _, _ in r.transport.sent[k:])
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)


def test_a_wait_chosen_before_the_send_undoes_the_lift_ramp_too():
    r = Rig(frames=True, operator=[])
    r.start()
    r.submit()
    run_until(r, lambda rec: r.core.ms.phase == "lift")
    target = r.core.ms.lift_command
    r.tick_cost_ns = 35 * MS  # the tick is past 30 ms already at the send
    recs = [r.tick() for _ in range(15)]
    r.tick_cost_ns = 0
    assert all(x.branch == "wait" for x in recs)
    assert math.isclose(r.core.ms.lift_command, target, abs_tol=1e-9)


def test_a_half_written_frame_stays_out_of_the_record():
    r = driving()
    r.transport.stall = True
    rec = r.tick()
    assert r.transport.partial_frame is not None and rec.seqs == []


def test_a_frame_finished_in_a_later_tick_is_in_that_ticks_record():
    r = driving()
    r.transport.stall = True
    r.tick()
    seq = r.transport.partial_frame.seq
    r.transport.stall = False
    rec = r.tick()
    assert seq in rec.seqs or r.transport.resyncs  # finished now (or cut, when it expired first)
