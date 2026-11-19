"""Codex 4c impl round 4: the send boundary under the lock -- a half frame continued only through the gate, nothing
moving after an alarm (the DISARM first), a lease that runs out while the lock is awaited, a wait reason or 30 ms at
the first byte turning the DRIVE into the wait frame, no ① re-entry from the permit's own sends, a stuck DISARM
asked again on the reopened link, an emergency event at the first byte DISARMing in the same tick."""

from dataclasses import replace

from forklift_core.live import real_autonomy as ra
from forklift_core.mcu_link.messages import Disarm, Drive

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


def kinds_after(r, k):
    return [(kind, m) for _, kind, _, m in r.transport.sent[k:]]


def test_a_half_frame_that_expires_inside_the_transmitter_is_abandoned_and_the_disarm_goes_first():
    r = driving()
    r.transport.stall = True
    r.tick()
    partial = r.transport.partial_frame
    r.transport.stall = False
    r.transport.before_continue = lambda: setattr(r, "tick_cost_ns", int(partial.expires_ns) - r.now + MS)
    k = len(r.transport.sent)
    r.tick()
    r.transport.before_continue, r.tick_cost_ns = None, 0
    sent = kinds_after(r, k)
    assert r.transport.resyncs == 1 and sent and sent[0][0] == "DISARM"
    assert not [m for kind, m in sent if kind == "DRIVE"]


class SlowLock:
    """The channels' lock taken ``cost`` after it was asked for (the clock moves while it is awaited)."""

    def __init__(self, rig, lock, cost_ns):
        self.rig, self.lock, self.cost = rig, lock, cost_ns

    def __enter__(self):
        self.rig.tick_cost_ns += self.cost
        return self.lock.__enter__()

    def __exit__(self, *exc):
        return self.lock.__exit__(*exc)


def test_a_lease_that_runs_out_while_the_lock_is_awaited_writes_nothing_and_disarms():
    r = driving()
    r.echo = False
    lease = r.arbiter.lease
    timeout = r.arbiter.config.lease_timeout_s
    while (r.now + 20 * MS) / 1e9 + 0.012 <= lease.renewed_mono + timeout:
        r.tick()
    real_lock = r.channels._lock  # noqa: SLF001
    r.channels._lock = SlowLock(r, real_lock, 12 * MS)  # noqa: SLF001
    k = len(r.transport.sent)
    r.tick()
    r.channels._lock, r.tick_cost_ns = real_lock, 0  # noqa: SLF001
    sent = kinds_after(r, k)
    assert not [m for kind, m in sent if kind == "DRIVE" and m.left_wheel_rad_s]
    assert any(kind == "DISARM" for kind, _ in sent) and r.permit.state in (ra.DISARMING, ra.LATCHED)


def test_a_wait_reason_that_comes_up_at_the_first_byte_sends_the_wait_frame():
    r = driving()
    ev = r.loop.evidence
    real = r.transport.transmit

    def snapshot_now(host, now, gate):
        r.loop.evidence = replace(ev, snapshot=lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), n, True,
                                                                        0, 0))
        return real(host, now, gate)

    r.transport.transmit = snapshot_now
    k = len(r.transport.sent)
    r.tick()
    r.transport.transmit, r.loop.evidence = real, ev
    drives = [m for kind, m in kinds_after(r, k) if kind == "DRIVE"]
    assert drives and all(m.left_wheel_rad_s == 0.0 == m.right_wheel_rad_s for m in drives)
    assert r.permit.state in (ra.READY, ra.DRIVING)  # a wait, not a latch


def test_30_ms_spent_before_the_first_byte_sends_the_wait_frame():
    r = driving()
    real = r.transport.transmit

    def slow(host, now, gate):
        r.tick_cost_ns = 35 * MS
        return real(host, now, gate)

    r.transport.transmit = slow
    k = len(r.transport.sent)
    r.tick()
    r.transport.transmit, r.tick_cost_ns = real, 0
    drives = [m for kind, m in kinds_after(r, k) if kind == "DRIVE"]
    assert drives and all(m.left_wheel_rad_s == 0.0 == m.right_wheel_rad_s for m in drives)


def test_an_emergency_event_at_the_first_byte_disarms_in_the_same_tick():
    r = driving()
    real = r.transport.transmit

    def stop_then_send(host, now, gate):
        r.say("stop")  # after this tick's emergency batch
        return real(host, now, gate)

    r.transport.transmit = stop_then_send
    k = len(r.transport.sent)
    r.tick()
    r.transport.transmit = real
    sent = kinds_after(r, k)
    assert not [m for kind, m in sent if kind == "DRIVE" and m.left_wheel_rad_s]
    assert any(kind == "DISARM" for kind, _ in sent) and r.permit.state in (ra.DISARMING, ra.LATCHED)


def test_a_stuck_disarm_is_asked_again_on_the_reopened_link_in_the_same_tick():
    r = driving()
    r.mcu.deaf = True
    r.transport.stall = True
    r.say("stop")
    r.tick()  # the DISARM's first bytes only
    assert r.transport.partial_frame is not None and r.transport.partial_frame.kind == "DISARM"
    for _ in range(8):
        k = len(r.transport.sent)
        r.tick()
        if r.transport.resyncs:
            break
    assert r.transport.resyncs == 1
    assert any(kind == "DISARM" for kind, _ in kinds_after(r, k))
    assert any(e.event == "disarm_relink" for e in r.permit.events)


def test_a_resync_that_keeps_failing_still_reaches_the_disarm_deadline():
    r = driving()
    r.mcu.deaf = True
    r.transport.stall = True
    r.say("stop")
    r.tick()

    def broken(*a, **k):
        raise OSError("reopen")

    r.transport.resync = broken
    r.tick(31)  # 620 ms
    assert r.permit.disarm_unconfirmed


def test_one_disarm_per_tick_while_s10s_stop_is_stuck():
    r = Rig(frames=True, operator=[(1, "confirm")])
    r.start()
    r.submit()
    real = r.transport.transmit

    def stall_s10(host, now, gate):
        if r.permit.state == ra.STOP_PENDING and not r.transport.resyncs:
            r.transport.stall = True
        return real(host, now, gate)

    r.transport.transmit = stall_s10
    run_until(r, lambda rec: rec.branch == "s10")
    for _ in range(10):
        k = len(r.transport.sent)
        r.tick()
        assert len({seq for _, kind, seq, _ in r.transport.sent[k:] if kind == "DISARM"}) <= 1
    assert r.permit.reason != "s10_stopped"
