"""Codex 4c impl round 6: evidence ages judged at the latest write instant, an emergency event taken during the
boundary's own check, any normal DRIVE (a small lift step too) turned into the wait, the tick's lift ramp undone
with it, the record keeping written seqs only."""

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


def moving_after(r, k):
    return [m for _, kind, _, m in r.transport.sent[k:] if kind == "DRIVE" and m.left_wheel_rad_s]


def test_a_scan_young_at_the_read_but_over_its_limit_at_the_write_refuses():
    r = driving()
    ev = r.loop.evidence
    scan_ns = r.now + 20 * MS - int(ra.SCAN_MAX_AGE_S * 1e9) + 3 * MS  # 3 ms left at the next tick's start
    r.loop.evidence = replace(ev, snapshot=lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), scan_ns,
                                                                    False, 0, 0))
    k = len(r.transport.sent)
    r.tick()
    assert not moving_after(r, k) and r.permit.state in (ra.DISARMING, ra.LATCHED)


def test_an_emergency_event_taken_during_the_check_refuses_the_write():
    r = driving()
    ev = r.loop.evidence
    said = {"done": False}

    def link_reason(n):
        if r.channels.lock._is_owned() and not said["done"]:  # noqa: SLF001 -- inside the boundary's check
            said["done"] = True
            r.say("stop")
        return ev.link_reason(n)

    r.loop.evidence = replace(ev, link_reason=link_reason)
    k = len(r.transport.sent)
    r.tick()
    assert said["done"] and not moving_after(r, k)
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)


def lift_rig():
    r = Rig(frames=True, operator=[])
    r.start()
    r.submit()
    run_until(r, lambda rec: r.core.ms.phase == "lift")
    return r


def test_a_snapshot_wait_at_the_first_byte_holds_a_small_lift_step_too():
    r = lift_rig()
    ev = r.loop.evidence
    real = r.transport.transmit

    def snapshot_now(host, now, gate):
        r.loop.evidence = replace(ev, snapshot=lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), n, True,
                                                                        0, 0))
        try:
            return real(host, now, gate)
        finally:
            r.loop.evidence = ev

    r.transport.transmit = snapshot_now
    k = len(r.transport.sent)
    r.tick(5)
    lifts = {round(m.lift_m, 6) for _, kind, _, m in r.transport.sent[k:] if kind == "DRIVE"}
    assert len(lifts) == 1  # one wait target, no 1 mm steps


def test_the_lift_ramp_of_a_tick_turned_into_a_wait_is_undone():
    r = lift_rig()
    real = r.transport.transmit

    def slow(host, now, gate):
        r.tick_cost_ns = 35 * MS
        try:
            return real(host, now, gate)
        finally:
            r.tick_cost_ns = 0

    r.transport.transmit = slow
    target = r.core.ms.lift_command
    r.tick(15)
    assert math.isclose(r.core.ms.lift_command, target, abs_tol=1e-9)


def test_a_wait_frame_refused_too_leaves_no_unwritten_seq_in_the_record():
    r = driving()
    real = r.transport.transmit
    calls = {"n": 0}

    def slower(host, now, gate):
        calls["n"] += 1
        r.tick_cost_ns = (35 if calls["n"] == 1 else 65) * MS
        try:
            return real(host, now, gate)
        finally:
            r.tick_cost_ns = 0

    r.transport.transmit = slower
    rec = r.tick()
    written = {seq for _, kind, seq, _ in r.transport.sent if kind == "DRIVE"}
    assert rec.branch == "emergency" and set(rec.seqs) <= written
