"""PR 6a-1 (docs/design/2026-11-20-real-hil-6a.md 2): the last clock comparison on every frame but DISARM, the
first-write transitions per frame kind, position-holding frames from this tick's fresh measurement and only within
their own tick, the stop-decided tick refusing every half normal DRIVE, the axis watch over a holding span, and the
tick-time evaluations of a STATE read after the tick's start."""

from dataclasses import replace

import pytest

from forklift_core.live import real_autonomy as ra
from forklift_core.live import real_loop as rl
from forklift_core.mcu_link.messages import Drive, encode

from real_loop_rig import MS, FakeTransport, Rig


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


def installing(r):
    """Every first byte from now on finds a snapshot installing: each tick's candidate turns into the wait frame."""
    ev = r.loop.evidence
    r.loop.evidence = replace(ev, snapshot=lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), n, True,
                                                                    0, 0))
    return ev


def drives_sent(r, k):
    return [(seq, m) for _, kind, seq, m in r.transport.sent[k:] if kind == "DRIVE"]


def late_after_lease(r, extra_ns):
    """The judge's lease read (after its evidence-slow check) costs ``extra_ns``: its last clock comparison is later."""
    real = r.loop._lease_valid

    def slow(ns):
        r.tick_cost_ns += extra_ns
        return real(ns)

    r.loop._lease_valid = slow
    return real


# --- the last clock comparison and the judged instant ---------------------------------------------------------------
def test_a_normal_drive_records_the_judged_instant_within_its_bounds():
    r = driving()
    k = len(r.transport.sent)
    r.tick()
    sent = drives_sent(r, k)
    assert sent
    for seq, _ in sent:
        judged = r.transport.judged[seq]
        assert judged is not None
        booked_decided = r.now  # this tick's decision
        assert judged <= booked_decided + int((rl.WRITE_MARGIN_S - rl.WRITE_ALLOWANCE_S) * 1e9)


def test_a_drive_judged_past_ns_minus_a_is_not_written():
    r = driving()
    late_after_lease(r, int(4.5 * MS))  # ns0 + 4.5 ms > ns - A = ns0 + 4 ms
    k = len(r.transport.sent)
    rec = r.tick()
    assert not [m for _, m in drives_sent(r, k) if m.left_wheel_rad_s or m.right_wheel_rad_s]
    assert rec.branch == "wait"


def test_a_late_wait_frame_is_dropped_and_not_made_again_in_the_same_tick():
    r = driving()
    late_after_lease(r, int(4.5 * MS))
    q0 = r.port.control_gen
    seq0 = r.host.last_queued_seq
    r.tick()
    queued = (r.host.last_queued_seq - seq0) & 0xFFFF
    assert queued <= 2  # the candidate and one wait frame at most (round 10 P2-1)
    assert r.port.control_gen >= q0


# --- position-holding frames -----------------------------------------------------------------------------------------
def test_the_hold_carries_this_ticks_measurement_not_the_waits_first_target():
    r = driving()
    installing(r)
    r.tick()
    r.tick()
    for _ in range(3):
        r.mcu.lift -= 0.002  # the forks sag
        k = len(r.transport.sent)
        r.tick()
        sent = drives_sent(r, k)
        assert sent, r.loop.records[-1]
        assert sent[-1][1].lift_m == pytest.approx(r.mcu.lift, abs=1e-6)


def test_no_hold_without_a_state_this_tick_and_the_unsent_drives_are_dropped():
    r = driving()
    installing(r)
    r.tick()
    r.tick()
    r.transport.silent = True
    k = len(r.transport.sent)
    rec = r.tick()
    r.transport.silent = False
    assert not drives_sent(r, k)
    assert "wait_frame_skipped:no_state" in (rec.notes or [])
    assert not [o for o in r.host._outbox if o.kind == "DRIVE"]  # noqa: SLF001


def test_a_half_hold_is_not_finished_in_a_later_tick():
    r = driving()
    installing(r)
    r.tick()
    r.tick()
    r.transport.stall = True
    r.tick()
    partial = r.transport.partial_frame
    assert partial is not None and partial.kind == "DRIVE"
    r.transport.stall = False
    k = len(r.transport.sent)
    r.tick()
    sent = [(kind, seq) for _, kind, seq, _ in r.transport.sent[k:]]
    assert (partial.kind, partial.seq) not in sent
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)


def test_a_stop_decided_tick_refuses_a_half_lift_only_drive():
    """Impl round 1 P3: a real lift-only DRIVE (wheels 0, lift moving) half written, then an arrival wait decided in
    the next tick with the evidence normal -- only the stop-decided rule refuses its continuation."""
    r = Rig(frames=True, operator=[])
    r.start()
    r.submit()
    run_until(r, lambda rec: r.core.ms.phase == "lift")
    real = r.transport.transmit
    seen = {}

    def stall_lift(host, now, gate):
        o = host._outbox[0] if host._outbox else None  # noqa: SLF001
        b = None if o is None else next((f for f in r.loop.book.frames if f.seq == (int(o.seq) & 0xFFFF)), None)
        if not seen and b is not None and b.kind == "drive" and b.speed_mps == 0.0:
            seen["seq"] = o.seq
            r.transport.stall = True
        return real(host, now, gate)

    r.transport.transmit = stall_lift
    run_until(r, lambda rec: "seq" in seen, limit=200)
    r.transport.transmit = real
    r.transport.stall = False
    r.loop._wait_reason_now = lambda now_ns, out: "arrival_wait"  # noqa: SLF001
    k = len(r.transport.sent)
    r.tick()
    sent = [(kind, seq) for _, kind, seq, _ in r.transport.sent[k:]]
    assert ("DRIVE", seen["seq"]) not in sent


# --- the axis watch --------------------------------------------------------------------------------------------------
def holding(r):
    r._ev0 = installing(r)  # noqa: SLF001
    run_until(r, lambda rec: r.loop._axis is not None)  # noqa: SLF001
    return r


def test_a_lift_rise_over_the_band_latches():
    r = holding(driving())
    r.mcu.lift += 0.006
    rec = r.tick()
    assert rec.branch == "emergency" and rec.reason == "wait_axis_moved:lift"
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)


def test_a_lift_rise_within_the_band_does_not():
    r = holding(driving())
    r.mcu.lift += 0.004
    rec = r.tick()
    assert rec.reason != "wait_axis_moved:lift"


def test_a_steering_move_over_the_band_latches_either_way():
    for d in (0.025, -0.025):
        r = holding(driving())
        r.mcu.q += d
        rec = r.tick()
        assert rec.reason == "wait_axis_moved:steer"


def test_the_watch_is_not_bypassable():
    assert "wait_axis_moved" in rl.NOT_BYPASSABLE


def test_the_watch_ends_after_a_confirmed_normal_drive_and_the_next_state():
    r = holding(driving())
    r.loop.evidence = r._ev0  # noqa: SLF001 -- the snapshot installed: the wait ends
    rec = run_until(r, lambda rec: r.loop._axis is None, limit=50)  # noqa: SLF001
    c = r.loop.book.confirmed
    assert rec.branch != "emergency"
    assert c is not None and c.kind == "drive"
    assert not [f for f in r.loop.book.frames if f.kind in ("hold", "stop") and f.written_ns is not None]


def test_the_watch_end_waits_while_a_later_hold_is_still_on_the_book():
    r = holding(driving())
    r.loop._axis["end_pending"] = True  # noqa: SLF001 -- a confirming STATE seen
    r.tick()  # still waiting: a new hold is written in this tick
    assert r.loop._axis is not None and not r.loop._axis["end_pending"]  # noqa: SLF001


# --- a STATE read after the tick's start --------------------------------------------------------------------------
class LateRx(FakeTransport):
    """The STATEs reach the host 6 ms after the tick's start (an actual read time, design 2)."""

    def receive(self, host, now_ns, limit=64):
        steps = 0 if self.last_ns is None else max(0, int((now_ns - self.last_ns) // (10 * MS)))
        self.last_ns = now_ns if self.last_ns is None else self.last_ns + steps * 10 * MS
        for _ in range(min(steps, limit)):
            self.mcu.advance(10_000)
            host.on_bytes(encode(self.mcu.state(), 1), now_ns + 6 * MS)
        if steps == 0:
            host.on_bytes(encode(self.mcu.state(), 1), now_ns + 6 * MS)


def test_states_read_after_the_ticks_start_still_drive():
    r = Rig(frames=True)
    late = LateRx(r.mcu)
    r.transport = late
    r.loop.transport = late
    r.start()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING and r.loop.book.confirmed is not None
              and r.loop.book.confirmed.speed_mps != 0.0)


# --- S10's stop within its own tick (design 4, rounds 13-14) -------------------------------------------------------
def s10_stop_continued_in_its_tick(change_authority):
    """S10's stop frame is half written, then -- in the same tick, with the same gate (the test hook of design 8) --
    the transport tries to finish it, the authority moved between the two or not."""
    r = Rig(frames=True, operator=[(1, "confirm")])
    r.start()
    r.submit()
    real = r.transport.transmit
    seen = {}

    def twice(host, now, gate):
        if r.permit.state == ra.STOP_PENDING and not seen:
            r.transport.stall = True
            out = real(host, now, gate)
            seen["partial"] = r.transport.partial_frame
            r.transport.stall = False
            if change_authority:
                r.link.a.control_gen += 1
            seen["k"] = len(r.transport.sent)
            return out + real(host, now, gate)
        return real(host, now, gate)

    r.transport.transmit = twice
    run_until(r, lambda rec: rec.branch == "s10")
    partial = seen["partial"]
    assert partial is not None and partial.kind == "DRIVE"
    sent = [(kind, seq) for _, kind, seq, _ in r.transport.sent[seen["k"]:]]
    return r, partial, sent


def test_an_s10_stop_finishes_within_its_tick_when_the_authority_stays():
    r, partial, sent = s10_stop_continued_in_its_tick(False)
    assert (partial.kind, partial.seq) in sent


def test_an_s10_stop_is_not_finished_after_the_authority_changed_within_its_tick():
    r, partial, sent = s10_stop_continued_in_its_tick(True)
    assert (partial.kind, partial.seq) not in sent
    r.tick()
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)


# --- 6a-2a: the round trip's write instant -------------------------------------------------------------------------
def test_the_delay_bound_takes_the_first_bytes_instant_when_the_transport_has_one():
    r = Rig(frames=True, operator=[(1, "confirm")], real_layer=True)
    seen = []
    real = r.loop.delay_bound.written
    r.loop.delay_bound.written = lambda seq, ns: (seen.append((seq & 0xFFFF, ns)), real(seq, ns))[1]

    class First(dict):
        def get(self, key, default=None):
            return 42  # every frame's first byte, as the TTY transport would report it

    r.transport.first_byte_ns = First()
    r.start()  # the session's two DISARMs are written
    assert seen and all(ns == 42 for _, ns in seen)


def test_a_loss_inside_a_send_is_the_emergency_of_that_tick():
    r = driving()
    real = r.transport.transmit

    def lose(host, now, gate):
        out = real(host, now, gate)
        r.transport.lost = {"first": "write_eio", "stream_tainted": False,
                            "reason": "device_lost:first=write_eio:stream_tainted=0"}
        return out

    r.transport.transmit = lose
    rec = r.tick()
    assert rec.branch == "emergency" and rec.reason.startswith("device_lost")
