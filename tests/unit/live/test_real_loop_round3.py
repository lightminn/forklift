"""Codex 4c impl round 3: the first byte re-reads expiry, time, evidence and authority (ARM and S10's stop too); a
partial frame is cut on the clock at its expiry; a reboot seen across an empty tick; D5 poses never joined across a
pose of another correction; RX failures keep the DISARM retries; a hold is its tick's only."""

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


def slow_transmit(r, cost_ns):
    real = r.transport.transmit

    def late(host, now, gate):
        r.tick_cost_ns = cost_ns  # the transport reaches the first byte this long after the tick's start
        try:
            return real(host, now, gate)
        finally:
            r.tick_cost_ns = 0
    r.transport.transmit = late
    return real


def test_a_drive_whose_first_byte_comes_too_late_is_not_written_and_disarms_the_same_tick():
    r = driving()
    k = len(r.transport.sent)
    slow_transmit(r, 105 * MS)
    rec = r.tick()
    assert not [m for _, _, _, m in r.transport.sent[k:] if isinstance(m, Drive)]
    assert any(isinstance(m, Disarm) for _, _, _, m in r.transport.sent[k:])
    assert r.permit.state in (ra.DISARMING, ra.LATCHED) and rec is not None


def test_s10s_stop_frame_whose_first_byte_is_late_is_not_written():
    r = Rig(frames=True, operator=[(1, "confirm")])
    r.start()
    r.submit()
    real = r.transport.transmit
    r.transport.transmit = lambda host, now, gate: (setattr(r, "tick_cost_ns", 65 * MS if r.permit.state ==
                                                            ra.STOP_PENDING else 0), real(host, now, gate))[1]
    run_until(r, lambda rec: rec.branch == "s10")
    seq = (r.permit._stop or {}).get("seq")  # noqa: SLF001 -- None once it already DISARMed
    r.transport.transmit, r.tick_cost_ns = real, 0
    r.tick(3)
    assert seq is None or not [x for x in r.transport.sent if x[2] == seq and x[1] == "DRIVE"]
    assert r.permit.reason != "s10_stopped"


def test_a_partial_frame_past_its_expiry_during_the_tick_is_cut_not_finished():
    r = driving()
    r.transport.stall = True
    r.tick()  # a frame's first bytes go out, then nothing
    partial = r.transport.partial_frame
    assert partial is not None
    r.transport.stall = False
    r.tick_cost_ns = int(partial.expires_ns) - (r.now + 20 * MS) + 1 * MS  # the clock passes its expiry mid-tick
    r.tick()
    r.tick_cost_ns = 0
    assert r.transport.resyncs == 1
    assert not [x for x in r.transport.sent if x[2] == partial.seq]


def test_a_reboot_after_a_device_loss_and_an_empty_tick_asks_for_a_relocalization():
    r = Rig(frames=True)
    r.start()
    r.tick(5)
    r.transport.silent = True
    r.host.disconnected(r.now)
    r.tick(1)  # no STATE at all this tick
    r.transport.silent = False
    r.mcu.boot = 0x6464
    r.tick(1)
    assert r.core.cargo.relocalize_required


def test_poses_of_one_correction_are_not_joined_across_a_pose_of_another():
    r = Rig(frames=True)
    r.start()
    loop = r.loop
    A, B = (0.0, 0.0, 0.0), (0.2, 0.0, 0.0)
    loop.pose_history.clear()
    loop.pose_history.extend([(1.00, (0.0, 0.0, 0.0), True, 0.0, 0.0, A), (1.02, (0.1, 0.0, 0.0), False, 0.0, 0.0, B),
                              (1.04, (0.2, 0.0, 0.0), True, 0.0, 0.0, A)])
    assert loop._pose_at(1.03, A) is None  # noqa: SLF001


def test_an_rx_failure_keeps_the_disarm_retries_and_the_deadline():
    r = driving()
    r.mcu.deaf = True
    r.say("stop")
    r.tick(1)

    def boom(*a, **k):
        raise OSError("rx")

    r.transport.receive = boom
    k = len(r.transport.sent)
    r.tick(30)  # 600 ms
    assert len({seq for _, kind, seq, _ in r.transport.sent[k:] if kind == "DISARM"}) >= 2
    assert r.permit.disarm_unconfirmed


def test_a_hold_does_not_outlive_its_tick():
    r = driving()
    r.core.state["motion_hold"] = "occupied"  # left from an earlier tick
    rec = r.tick()
    assert not (rec.reason or "").startswith("hold:")
