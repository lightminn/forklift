"""Design 2026-11-19-real-assembly 7: the real truck's arbiter settings -- AUTO's lease loss stops (``auto_lease_loss``
"stop"), and a rearm passes the real conditions before its ACK and mode change (``rearm_refusal``) and tells the real
permit when it really happened (``on_rearmed``). Their defaults keep the simulation's behaviour (pinned here: the
replay gate never loses a live lease)."""

import pytest

from forklift_core.safety.arbiter import ArbiterConfig

from test_arbiter_modes import Rig, auto


def lose_lease(r):
    r.a._expire(r.t + 5.0)  # the holder went quiet


def test_by_default_auto_keeps_working_after_the_lease_is_lost():
    r = Rig()
    auto(r)
    gen = r.a.control_gen
    lose_lease(r)
    assert r.a.mode == "AUTO" and r.a.lease is None and r.a.preempt_latch is None
    assert r.a.control_gen == gen + 1  # a control boundary, as before
    r.cycle()
    assert r.m.status == "running" and r.out.source == "auto" and r.out.speed_mps == 0.3


def test_the_real_setting_stops_auto_once_when_the_lease_is_lost():
    r = Rig()
    r.a.config = ArbiterConfig(auto_lease_loss="stop")
    auto(r)
    gen = r.a.control_gen
    lose_lease(r)
    assert r.a.mode == "STOPPED" and r.a.control_gen == gen + 1  # one boundary, not two
    r.cycle()
    assert r.out.source == "zero" and r.out.speed_mps == 0.0


def test_the_setting_takes_only_its_two_values():
    with pytest.raises(ValueError):
        ArbiterConfig(auto_lease_loss="pause")


def stopped(r):
    auto(r)
    r.cycle(r.msg("stop"))
    r.stand()
    assert r.a.mode == "STOPPED"


def test_a_rearm_the_real_conditions_refuse_changes_neither_the_mode_nor_the_permit():
    r = Rig()
    calls = []
    r.a.rearm_refusal = lambda now: "outstanding_unknown"
    r.a.on_rearmed = lambda now, gen: calls.append(gen)
    stopped(r)
    gen = r.a.control_gen
    (reply,) = r.cycle(r.msg("rearm"))
    assert reply["t"] == "reject" and reply["reason"] == "real_rearm_refused"
    assert reply["detail"] == "outstanding_unknown"
    assert r.a.mode == "STOPPED" and r.a.control_gen == gen and calls == []


def test_a_real_rearm_tells_the_permit_at_the_transition_and_a_replayed_one_does_not():
    r = Rig()
    calls = []
    r.a.rearm_refusal = lambda now: None
    r.a.on_rearmed = lambda now, gen: calls.append(gen)
    stopped(r)
    msg = r.msg("rearm")
    (reply,) = r.cycle(msg)
    assert reply["t"] == "ack" and r.a.mode == "PAUSED" and calls == [r.a.control_gen]
    r.cycle(r.msg("stop"))  # a new STOP
    r.stand()
    r.cycle(msg)  # the old rearm sent again
    assert r.a.mode == "STOPPED" and len(calls) == 1


def test_without_the_callbacks_a_rearm_is_as_before():
    r = Rig()
    stopped(r)
    (reply,) = r.cycle(r.msg("rearm"))
    assert reply["t"] == "ack" and r.a.mode == "PAUSED"
