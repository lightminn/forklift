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


# --- 4c (docs/design/2026-11-19-real-loop.md 2.1): external stop evidence, DISARM across a generation ---------------

def test_external_stop_evidence_is_taken_inside_the_one_observe_and_a_bad_one_drops_the_stop_at_once():
    r = Rig()
    r.a.config = ArbiterConfig(external_stop=True)
    evidence = {"fresh": True, "standing": False}
    r.a.stop_evidence = lambda now: (evidence["fresh"], evidence["standing"])
    r.speed = 0.0  # the arbiter's own detector would confirm a stand from these
    r.stand()
    assert r.a.stop_confirmed is False  # the loop's evidence says not standing: it wins
    evidence["standing"] = True
    r.cycle()
    assert r.a.stop_confirmed is True and r.a.feedback_ok is True
    evidence["fresh"] = False
    r.cycle()
    assert r.a.stop_confirmed is False and r.a.feedback_ok is False


def test_without_external_stop_the_arbiter_judges_the_stand_itself_as_before():
    r = Rig()
    r.a.stop_evidence = lambda now: (False, False)  # ignored while the setting is off
    r.speed = 0.0
    r.stand()
    assert r.a.stop_confirmed is True


def test_a_disarm_one_generation_old_is_taken_on_the_real_truck_and_tells_the_loop():
    r = Rig()
    r.a.config = ArbiterConfig(disarm_ignores_control_gen=True)
    calls = []
    r.a.on_disarm = lambda now: calls.append(now)
    auto(r)
    old = r.msg("disarm")  # made at generation G
    r.a._boundary(r.t, "test")  # noqa: SLF001 -- G -> G+1 (the S10 end's AUTO -> PAUSED does this)
    (reply,) = r.cycle(old)
    assert reply["t"] == "ack" and calls


def test_by_default_an_old_generation_disarm_is_refused_as_before():
    r = Rig()
    auto(r)
    old = r.msg("disarm")
    r.a._boundary(r.t, "test")  # noqa: SLF001
    (reply,) = r.cycle(old)
    assert reply["t"] == "reject" and reply["reason"] == "stale_control"


def test_a_disarm_with_another_lease_is_still_refused():
    r = Rig()
    r.a.config = ArbiterConfig(disarm_ignores_control_gen=True)
    auto(r)
    bad = dict(r.msg("disarm"), lease_id="not-the-lease")
    (reply,) = r.cycle(bad)
    assert reply["t"] == "reject" and reply["reason"] == "stale_lease"
