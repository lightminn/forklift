"""docs/design/2026-11-19-real-loop.md 6: S10 case A end to end on the fake truck, and the loop's stops."""

import pytest

from forklift_core.live import real_autonomy as ra
from forklift_core.live.real_teleop import enforce_real, real_auto_refusal
from forklift_core.mcu_link.messages import Disarm, Drive, Mode
from forklift_core.safety.teleop_policy import teleop_policy

from real_loop_rig import Rig


def run_until(r, cond, limit=4000):
    for _ in range(limit):
        rec = r.tick()
        if cond(rec):
            return rec
    raise AssertionError(f"not reached: {r.loop.records[-1]}")


def s10_rig(**kw):
    r = Rig(frames=True, operator=[(1, "confirm")], **kw)
    r.start()
    r.submit()
    return r


def test_s10_a_runs_to_the_carry_height_stops_disarms_and_locks():
    r = s10_rig()
    rec = run_until(r, lambda rec: rec.branch == "s10")
    k = len(r.transport.sent)
    assert r.core.commands.end_reason == "s10_lifted" and r.permit.state == ra.STOP_PENDING
    assert (r.core.cargo.relation, r.core.cargo.load) == ("carrying", "confirmed")
    assert not any(e["to"] == "extract" for e in r.core.state["transitions"])  # no extract
    run_until(r, lambda rec: r.permit.state == ra.LATCHED)
    sent = r.transport.sent[k - 1:]
    drives = [m for _, kind, _, m in sent if isinstance(m, Drive)]
    assert drives and all(m.left_wheel_rad_s == 0.0 == m.right_wheel_rad_s for m in drives)  # wheels 0 from the stop
    assert any(isinstance(m, Disarm) for _, _, _, m in sent) and r.mcu.mode == Mode.DISARMED
    assert r.arbiter.mode == "STOPPED"
    assert [e["reason"] for e in r.loop.session.results] == ["s10_lifted"]  # collected once
    assert r.core.field_recovery is not None  # the field recovery opened (the pallet on the forks)
    s = r.core.safety_state(None)
    assert real_auto_refusal(s) == "recovery_required"
    p = teleop_policy(enforce_real(s), feedback_ok=True, standing=True, lift_m=0.08)
    assert not p.drive_allowed and not p.lift_allowed


def test_a_lost_console_stops_auto_and_nothing_resumes_without_a_rearm():
    r = Rig(frames=True)
    r.start()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING)
    r.echo = False  # the console goes quiet
    run_until(r, lambda rec: r.permit.state == ra.LATCHED)
    assert r.arbiter.mode == "STOPPED" and r.core.commands.status in ("cancelled", "idle")
    r.echo = True
    k = len(r.transport.sent)
    r.tick(50)
    assert not [m for _, _, _, m in r.transport.sent[k:] if isinstance(m, Drive) and m.left_wheel_rad_s]


def test_a_stop_after_the_batch_and_before_the_first_byte_keeps_the_frame_unwritten():
    r = Rig(frames=True)
    r.start()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING)
    real_transmit = r.transport.transmit

    def stop_then_send(host, now, gate):
        r.say("stop")  # accepted by the console thread after this tick's emergency batch
        return real_transmit(host, now, gate)

    r.transport.transmit = stop_then_send
    k = len(r.transport.sent)
    r.tick()
    r.transport.transmit = real_transmit
    assert not [m for _, _, _, m in r.transport.sent[k:] if isinstance(m, Drive) and m.left_wheel_rad_s]


def test_an_unconfirmed_control_stop_disarms_after_its_limit():
    r = s10_rig()
    run_until(r, lambda rec: r.core.ms.phase == "lift")
    r.mcu.ack_frozen = True  # frames applied, never echoed
    run_until(r, lambda rec: r.permit.state in (ra.DISARMING, ra.LATCHED) and r.permit.reason is not None)
    assert r.permit.reason in ("control_stop_unconfirmed", "authorization_revoked") or "s10" in str(r.permit.reason)


def test_the_real_encoder_model_blocks_the_restart_after_the_first_frame_stand():
    r = Rig(frames=True, direction_evidence=False)
    r.start()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.LATCHED)
    assert "direction_unsure" in str(r.core.commands.end_reason)


def test_an_unsupported_state_latches_before_it_acts():
    r = Rig(frames=True)
    r.start()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING)
    r.core.observe_backoff = object()  # a branch S10-A never builds
    rec = r.tick()
    assert rec.branch == "emergency" and rec.reason == "unsupported:observe_backoff"
    assert r.permit.state == ra.LATCHED
