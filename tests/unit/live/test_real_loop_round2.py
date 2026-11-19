"""Codex 4c impl round 2: the authority and the evidence at the send, each frame's own authority, the core's holds
to the wait frame, stands on the MCU clock, device losses, a failing transmit, a pause while arming, the lift ramp
in a snapshot wait."""

import math

from forklift_core.live import real_autonomy as ra
from forklift_core.mcu_link.messages import Arm, Disarm, Drive

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
    return [m for _, _, _, m in r.transport.sent[k:] if isinstance(m, Drive) and m.left_wheel_rad_s]


def test_a_lease_that_runs_out_during_the_tick_is_the_emergency_before_the_first_write():
    r = driving()
    r.echo = False
    lease = r.arbiter.lease
    timeout = r.arbiter.config.lease_timeout_s
    while (r.now + 20 * MS) / 1e9 + 0.025 <= lease.renewed_mono + timeout:
        r.tick()
    assert (r.now + 20 * MS) / 1e9 <= lease.renewed_mono + timeout  # still held at the next tick's start
    r.tick_cost_ns = 25 * MS
    k = len(r.transport.sent)
    rec = r.tick()
    assert rec.reason == "lease_expired" and not moving_after(r, k)


def test_a_frame_decided_under_another_control_generation_is_never_written():
    """Round 2 P1-2: the stop frame held before its first byte; a boundary (a new control generation) comes before
    the write -- the frame is not written, the permit DISARMs."""
    r = Rig(frames=True, operator=[(1, "confirm")])
    r.start()
    r.submit()
    real_transmit = r.transport.transmit
    r.transport.transmit = lambda host, now, gate: [] if r.permit.state == ra.STOP_PENDING else \
        real_transmit(host, now, gate)
    run_until(r, lambda rec: rec.branch == "s10")
    seq = r.permit._stop["seq"]  # noqa: SLF001
    r.arbiter.control_gen += 1  # e.g. a session end accepted in PAUSED
    r.transport.transmit = real_transmit
    r.tick(3)
    assert not [x for x in r.transport.sent if x[2] == seq and x[1] == "DRIVE"]
    assert r.permit.reason != "s10_stopped"


def test_a_core_hold_on_an_unconfirmed_output_is_the_wait_frame():
    """Round 2 P1-4: the layer allows less than what may still act -- wheels 0 through the wait frame (its new
    generation drops the old queue), not an AUTO DRIVE behind the old ones."""
    r = driving()
    r.core.obstacle["layer"].limit = lambda *a, **k: (0.001, "occupied")
    rec = run_until(r, lambda rec: (rec.reason or "").startswith("hold:"), limit=10)
    assert rec.branch == "wait"


def test_samples_measured_before_a_frames_mcu_deadline_are_no_stand():
    """Round 2 P1-5: a stand is judged on samples measured after every movable frame's MCU deadline."""
    r = Rig(frames=True)
    r.start()
    r.tick(30)
    assert r.core.slam["stop_now"]
    r.loop._motion_mcu_until = r.loop.samples[-1].state.mcu_time_us + 10_000  # noqa: SLF001
    r.tick(1)
    assert not r.core.slam["stop_now"]


def test_a_device_loss_drops_the_samples_and_the_stand():
    r = Rig(frames=True)
    r.start()
    r.tick(30)
    real_receive = r.transport.receive

    def lost_then_back(host, now, limit=64):
        host.disconnected(now)
        return real_receive(host, now, limit)

    r.transport.receive = lost_then_back
    r.tick(1)
    r.transport.receive = real_receive
    assert not r.core.slam["stop_now"] and len(r.loop.samples) <= 2


def test_a_receive_that_raises_leaves_no_stand_for_the_collection():
    r = Rig(frames=True)
    r.start()
    r.tick(30)

    def boom(*a, **k):
        raise OSError("usb")

    r.transport.receive = boom
    rec = r.tick()
    assert rec.branch == "emergency" and not r.core.slam["stop_now"]


def test_a_transmit_that_keeps_failing_still_collects_and_returns():
    r = driving()

    def boom(*a, **k):
        raise OSError("tty")

    r.transport.transmit = boom
    rec = r.tick()
    assert rec.branch == "emergency"
    r.tick(3)
    assert len(r.loop.session.results) == 1


def test_a_lasting_overspeed_keeps_one_disarm_until_its_retry():
    """Round 2 P2-2: once latched, more emergencies do not renew the DISARM each tick."""
    r = driving()
    r.mcu.deaf = True
    r.mcu.extra_rate = 30.0
    k = len(r.transport.sent)
    r.tick(4)  # 80 ms
    assert len({seq for _, kind, seq, _ in r.transport.sent[k:] if kind == "DISARM"}) == 1


def test_a_pause_while_arming_latches_before_the_hosts_zero_drive():
    r = Rig(frames=True)
    r.mcu.arm_delay = 10  # the ARM answered 100 ms late
    r.start()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.ARMING)
    r.say("mission_pause", mission_id=r.core.commands.mission_id)
    k = len(r.transport.sent)
    rec = run_until(r, lambda rec: rec.branch == "emergency", limit=5)
    assert rec.reason == "unsupported:mission_paused"
    assert not [m for _, _, _, m in r.transport.sent[k:] if isinstance(m, Drive)]


def test_a_snapshot_wait_in_the_lift_holds_the_lift_target():
    from dataclasses import replace

    r = Rig(frames=True, operator=[])
    r.start()
    r.submit()
    run_until(r, lambda rec: r.core.ms.phase == "lift")
    target = r.core.ms.lift_command
    ev = r.loop.evidence
    r.loop.evidence = replace(ev, snapshot=lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), n, True, 0, 0))
    r.tick(15)
    assert math.isclose(r.core.ms.lift_command, target)
