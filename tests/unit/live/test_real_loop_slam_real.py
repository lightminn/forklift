"""PR 7c-1b-1 (docs/design/2026-11-25-real-sources-pr7.md 6.1): the loop's side of the real SLAM source on the fake
truck -- the three times of a reply (order stamp, first-beam evidence interval [Flo, Fhi]), the side-effect-free
acceptance checks (``can_confirm`` / ``can_seed``), a declaration that completes only on its seed's end state within
20 s + 0.5 s, and the session's manual-only latch read through the slam bypass at the three start boundaries."""

import pytest

from forklift_core.live import real_loop as rl

from test_real_loop_console import ask, run_until
from test_real_loop_relocalize import open_generation, rig, stand, status


def seeding(r, *, ready=True):
    """The source of the real link (6.1, 2): a seed runs `seeding` until the test ends it; confirmations only on
    `ready`; one seed at a time."""
    src = r.slam
    src.seed_states = {}
    src.session = "ready" if ready else "seeding"
    plain_seed = src.seed

    def seed(epoch, base_pose, stamp_s):
        plain_seed(epoch, base_pose, stamp_s)
        src.seed_states[int(epoch)] = ("seeding", None)
        src.session = "seeding"
        src.auto = False  # a new instance answers nothing until its end state (Codex 7c-1b-1 round 1 P3-7)
        return f"seed-{epoch}"

    def end(kind, reason=None):
        epoch = max(src.seed_states)
        src.seed_states[epoch] = (kind, reason)
        src.session = kind if kind != "void" else "ready"
        src.auto = kind == "ready"  # processed replies only from a localized instance

    src.seed = seed
    src.end = end
    src.seed_state = lambda epoch: src.seed_states.get(int(epoch))
    src.can_confirm = lambda: None if src.session == "ready" else "slam_not_ready"
    src.can_seed = lambda: "seed_in_progress" if src.session == "seeding" else None
    return src


def reply(r, stamp, lo, hi, **kw):
    item = r.slam.reply(stamp, **kw)
    item[0].evidence_lo_s, item[0].evidence_hi_s = float(lo), float(hi)
    return item


# --- the three times (6.1, 0) ------------------------------------------------------------------------------------
def test_before_epoch_is_judged_on_the_evidence_low_end():
    r = rig()
    open_generation(r)
    r.slam.auto = False
    t0 = r.loop.t_epoch
    r.slam.extra = [reply(r, t0 + 0.01, t0 - 0.01, t0 + 0.02)]  # order time after the epoch, first beam may be before
    r.tick()
    assert r.loop.reloc_dropped["before_epoch"] == 1


def test_future_is_judged_on_the_evidence_high_end():
    r = rig()
    open_generation(r)
    r.slam.auto = False
    r.tick(3)
    t = r.loop._t(r.now + r.step_ns)
    r.slam.extra = [reply(r, t - 0.05, t - 0.06, t + 0.5)]
    r.tick()
    assert r.loop.reloc_dropped["future"] == 1


def test_out_of_order_is_judged_on_the_order_time():
    r = rig()
    open_generation(r)
    r.slam.auto = False
    r.tick(10)
    t = r.loop._t(r.now + r.step_ns)
    # the second's interval starts before the first's, its order time after: in order
    r.slam.extra = [reply(r, t - 0.10, t - 0.11, t - 0.09), reply(r, t - 0.05, t - 0.15, t - 0.04)]
    r.tick()
    assert r.loop.reloc_dropped["out_of_order"] == 0
    r.slam.extra = [reply(r, t - 0.07, t - 0.03, t - 0.02)]  # interval later, order time earlier: out of order
    r.tick()
    assert r.loop.reloc_dropped["out_of_order"] == 1


def test_freshness_is_the_newest_replys_own_evidence_even_when_earlier():
    """Codex 7c-1b-1 round 1 P1: order times rise, a later reply's first beam may be earlier -- the freshness is that
    reply's own, as the provenance is (a max over replies made a 0.510 s old correction look 0.495 s old)."""
    r = rig()
    open_generation(r)
    r.slam.auto = False
    r.tick(10)
    t = r.loop._t(r.now + r.step_ns)
    r.slam.extra = [reply(r, t - 0.03, t - 0.02, t - 0.01), reply(r, t - 0.02, t - 0.05, t - 0.01)]
    r.tick()
    assert r.loop.provenance[3] == pytest.approx(t - 0.05)
    assert r.loop._fresh_ns == r.loop.start_ns + int(round((t - 0.05) * 1e9))


def test_freshness_and_provenance_use_the_evidence_low_end():
    r = rig()
    open_generation(r)
    r.slam.auto = False
    r.tick(10)
    t = r.loop._t(r.now + r.step_ns)
    r.slam.extra = [reply(r, t - 0.02, t - 0.04, t - 0.01)]
    r.tick()
    assert r.loop._fresh_ns == r.loop.start_ns + int(round((t - 0.04) * 1e9))
    assert r.loop.provenance[3] == pytest.approx(t - 0.04)


def test_a_confirmation_reference_needs_its_evidence_after_the_request():
    r = rig()
    gen = open_generation(r)
    r.slam.auto = False
    r.slam.request_processed = lambda *a, **k: None
    stand(r)
    ask(r, "relocalize_confirm", generation=gen)
    a = r.loop.attempt
    t = r.loop._t(r.now + r.step_ns)
    r.slam.extra = [reply(r, t, a["t"] - 0.01, t, attempt=a["attempt"])]  # its first beam may predate the request
    r.tick()
    assert not r.loop.attempt["reference"]
    t = r.loop._t(r.now + r.step_ns)
    r.slam.extra = [reply(r, t, t - 0.01, t, attempt=a["attempt"])]
    r.tick()
    assert r.loop.attempt is None or r.loop.attempt["reference"]


# --- acceptance without side effects (6.1, 2) -------------------------------------------------------------------
def test_a_confirmation_while_the_source_is_not_ready_is_refused_and_changes_nothing():
    r = rig()
    gen = open_generation(r)
    src = seeding(r)
    stand(r)
    first, _ = ask(r, "relocalize_declare", generation=gen)
    r.tick(2)
    assert src.session == "seeding"
    epoch, attempts = r.loop.epoch, r.loop._attempt_serial
    second, _ = ask(r, "relocalize_confirm", generation=gen)
    assert second["t"] == "reject" and second["reason"] == "slam_not_ready"
    assert r.loop.attempt is not None and r.loop.attempt["attempt"] == first["attempt"]
    assert r.loop.epoch == epoch and r.loop._attempt_serial == attempts
    assert status(r)["state"] == "waiting" and status(r)["kind"] == "declare"


def test_a_second_declaration_while_seeding_is_refused():
    r = rig()
    gen = open_generation(r)
    src = seeding(r)
    stand(r)
    first, _ = ask(r, "relocalize_declare", generation=gen)
    r.tick(2)
    second, _ = ask(r, "relocalize_declare", generation=gen)
    assert second["t"] == "reject" and second["reason"] == "seed_in_progress"
    assert r.loop.attempt["attempt"] == first["attempt"]
    assert sum(1 for c in src.calls if c[0] == "seed") == 1


# --- the declaration's completion and limit (6.1, 2) ------------------------------------------------------------
def test_a_declaration_waits_for_its_seed_end_state_past_the_confirmation_limit():
    r = rig()
    gen = open_generation(r)
    src = seeding(r)
    stand(r)
    ask(r, "relocalize_declare", generation=gen)
    r.tick(int((rl.RELOCALIZE_WAIT_S + 2.0) / 0.02))
    assert status(r)["state"] == "waiting" and r.core.relocalize_generation() == gen
    src.end("ready")
    run_until(r, lambda rec: status(r)["state"] != "waiting", 50)
    assert status(r)["state"] == "declared" and r.core.relocalize_generation() is None
    assert r.loop.manual_only is None


def test_a_disarm_waiting_before_the_seed_asks_no_seed_state():
    """Codex 7c-1b-1 round 1 P2-2: the seed not issued yet (a DISARM waiting), nothing asks the source's state."""
    from collections import Counter

    from forklift_core.live.real_run import _SlamGuard
    from test_real_loop_relocalize import _before_commit

    r = rig()
    gen = open_generation(r)
    src = seeding(r)
    marks = []
    r.loop.slam_source = _SlamGuard(src, marks.append, Counter())
    stand(r)
    asked = []
    plain = src.seed_state
    src.seed_state = lambda epoch: asked.append(epoch) or plain(epoch)
    r.loop.slam_source = _SlamGuard(src, marks.append, Counter())
    _before_commit(r, lambda: r.say("disarm"))
    r.say("relocalize_declare", generation=gen)
    r.tick(3)
    assert not marks and r.loop.fatal_source is None
    assert None not in asked and not any(c[0] == "seed" for c in src.calls)


def test_a_failing_synthetic_reply_after_the_seed_refuses_without_a_second_seed():
    """Codex 7c-1b-1 round 1 P2-3: the seed issued once; the synthetic reply failing ends the attempt."""
    r = rig()
    gen = open_generation(r)
    src = seeding(r)
    stand(r)
    orig = r.core.slam_replies

    def fail(t, replies, loaded):
        if replies and replies[0][1].get("source") == "operator_declared":
            raise RuntimeError("injected")
        return orig(t, replies, loaded)

    r.core.slam_replies = fail
    r.say("relocalize_declare", generation=gen)
    for _ in range(4):
        try:
            r.tick()
        except RuntimeError:
            pass
    r.core.slam_replies = orig
    r.tick(3)
    assert sum(1 for c in src.calls if c[0] == "seed") == 1
    assert status(r)["state"] == "refused" and status(r)["reason"] == "relocalize_declare_failed"


def test_the_commit_follows_the_end_state_within_half_a_second():
    """Codex 7c-1b-1 round 1 P3-5: once the end state is seen, the commit within 0.5 s of it."""
    r = rig()
    gen = open_generation(r)
    src = seeding(r)
    stand(r)
    ask(r, "relocalize_declare", generation=gen)
    r.tick(5)
    lay = r.loop.real_layer
    plain_ok = lay.ok
    lay.ok = lambda t: False  # the layer holds the close
    src.end("ready")
    r.tick()
    seen = r.loop.attempt["seed_end"][2]
    run_until(r, lambda rec: status(r)["state"] != "waiting", 60)
    lay.ok = plain_ok
    assert status(r)["state"] == "refused" and status(r)["reason"] == "relocalize_layer"
    assert r.loop._t(r.now) - seen == pytest.approx(rl.DECLARE_COMMIT_S, abs=0.05)


def test_the_end_state_is_timed_on_the_clock_when_it_is_seen():
    """Codex 7c-1b-1 round 2 P2: the tick's own time before the source is read does not come off the 0.5 s commit."""
    r = rig()
    gen = open_generation(r)
    src = seeding(r)
    stand(r)
    ask(r, "relocalize_declare", generation=gen)
    r.tick(5)
    src.end("not_localized", "NOT_LOCALIZED")
    r.tick_cost_ns = 40 * 1_000_000  # 40 ms of the tick spent before the state is read
    r.tick()
    r.tick_cost_ns = 0
    a = r.loop.attempt
    if a is not None:
        assert a["seed_end"][2] == pytest.approx(r.loop._t(r.now + 40 * 1_000_000))
    run_until(r, lambda rec: status(r)["state"] != "waiting", 50)
    assert status(r)["state"] == "declared"


def test_a_void_seen_past_the_limit_is_the_timeout():
    r = rig()
    gen = open_generation(r)
    src = seeding(r)
    stand(r)
    ask(r, "relocalize_declare", generation=gen)
    a = r.loop.attempt
    plain_state = src.seed_state
    src.seed_state = lambda epoch: ("seeding", None)  # nothing seen until past the limit
    while r.loop._t(r.now + r.step_ns) <= a["t"] + rl.SEED_WAIT_S + rl.DECLARE_COMMIT_S:
        r.tick()
    src.seed_state = plain_state
    src.end("void", "seed_moved")
    run_until(r, lambda rec: status(r)["state"] != "waiting", 5)
    assert status(r)["reason"] == "relocalize_timeout"


def test_a_declaration_without_an_end_state_times_out_at_its_own_limit():
    r = rig()
    gen = open_generation(r)
    seeding(r)
    stand(r)
    ask(r, "relocalize_declare", generation=gen)
    a_t = r.loop.attempt["t"]
    limit = rl.SEED_WAIT_S + rl.DECLARE_COMMIT_S
    run_until(r, lambda rec: status(r)["state"] != "waiting", int(limit / 0.02) + 20)
    assert status(r)["state"] == "refused" and status(r)["reason"] == "relocalize_timeout"
    assert r.loop._t(r.now) - a_t > limit - 0.05
    assert r.core.relocalize_generation() == gen


def test_a_void_seed_refuses_the_declaration():
    r = rig()
    gen = open_generation(r)
    src = seeding(r)
    stand(r)
    ask(r, "relocalize_declare", generation=gen)
    r.tick(5)
    src.end("void", "seed_moved")
    run_until(r, lambda rec: status(r)["state"] != "waiting", 20)
    assert status(r)["state"] == "refused" and status(r)["reason"] == "relocalize_voided:seed:seed_moved"
    assert r.core.relocalize_generation() == gen


def test_an_end_state_after_the_attempt_ended_completes_nothing():
    r = rig()
    gen = open_generation(r)
    src = seeding(r)
    stand(r)
    ask(r, "relocalize_declare", generation=gen)
    r.tick(3)
    r.say("lease_release")
    run_until(r, lambda rec: status(r)["state"] != "waiting", 20)
    assert status(r)["state"] == "refused"
    src.end("ready")
    r.tick(5)
    assert status(r)["state"] == "refused" and r.core.relocalize_generation() == gen


def test_the_confirmation_keeps_its_three_second_limit():
    r = rig()
    gen = open_generation(r)
    seeding(r)
    r.slam.auto = False
    r.slam.request_processed = lambda *a, **k: None
    stand(r)
    ask(r, "relocalize_confirm", generation=gen)
    run_until(r, lambda rec: status(r)["state"] != "waiting", int((rl.RELOCALIZE_WAIT_S + 1.0) / 0.02))
    assert status(r)["reason"] in ("relocalize_no_reply", "relocalize_timeout")


# --- the manual-only latch (6.1, 2) -----------------------------------------------------------------------------
def _not_localized(r):
    gen = open_generation(r)
    src = seeding(r)
    stand(r)
    ask(r, "relocalize_declare", generation=gen)
    r.tick(5)
    src.auto = False  # no processed reply after a failed match
    src.end("not_localized", "NOT_LOCALIZED")
    run_until(r, lambda rec: status(r)["state"] != "waiting", 50)
    assert status(r)["state"] == "declared" and r.core.relocalize_generation() is None
    return src


def test_a_not_localized_declaration_latches_manual_only():
    r = rig()
    _not_localized(r)
    m = r.loop.manual_only
    assert m is not None and m["reason"] == "not_localized" and m["session_epoch"] == r.arbiter.session_epoch
    assert m["seed_id"] == f"seed-{m['epoch']}"  # the source's own seed id (round 1 P3-6)
    assert r.loop._fresh_ns is None  # no processed reply of the seed's epoch (round 1 P3-7)
    reply_, _ = ask(r, "operator_start")
    assert reply_["t"] == "reject"
    st = r.loop.real_status(r.now)
    assert st["manual_only"] == m and st["manual_only_blocking"] is True


def test_the_latch_blocks_the_three_start_boundaries_with_the_slam_bypass_off():
    r = rig()
    _not_localized(r)
    reply_, _ = ask(r, "mode_request", to="AUTO")
    assert reply_["t"] == "reject" and reply_["reason"] == "operator_placed_start"
    reply_, _ = ask(r, "mission_submit", pallet_id="P-S10", destination_id="D-S10", targets_revision=0)
    assert reply_["t"] == "reject" and reply_["reason"] == "operator_placed_start"
    assert r.loop._start_blocked() is True
    reply_, _ = ask(r, "mode_request", to="TELEOP")
    assert reply_.get("reason") != "operator_placed_start"


def test_the_mission_commands_refuse_with_the_latch_past_the_arbiter():
    """Codex 7c-1b-1 round 1 P3-7: the Context boundary on its own (the arbiter's gate taken away)."""
    r = rig()
    _not_localized(r)
    r.arbiter.start_gate = None
    seen = []
    plain = r.core.commands.apply

    def spy(t, ctx):
        recs = plain(t, ctx)
        seen.extend((ctx.start_blocked, rec) for rec in recs)
        return recs

    r.core.commands.apply = spy
    r.say("mission_submit", pallet_id="P-S10", destination_id="D-S10", targets_revision=r.core.targets.revision)
    r.tick(2)
    assert seen and all(blocked for blocked, _ in seen)
    assert any(rec.get("reason") == "operator_placed_start" for _, rec in seen)


def test_the_slam_bypass_lifts_the_latch_refusal_and_turning_it_off_blocks_again():
    r = rig()
    _not_localized(r)
    reply_, _ = ask(r, "check_bypass", check="slam", on=True)
    assert reply_["t"] == "ack"
    r.tick()
    assert r.loop._start_blocked() is False and r.loop.real_status(r.now)["manual_only_blocking"] is False
    reply_, _ = ask(r, "mode_request", to="AUTO")
    assert reply_.get("reason") != "operator_placed_start"
    assert r.loop.manual_only is not None  # recorded, kept
    reply_, _ = ask(r, "check_bypass", check="slam", on=False)
    assert reply_["t"] == "ack"
    r.tick()
    assert r.loop._start_blocked() is True
    reply_, _ = ask(r, "mode_request", to="AUTO")
    assert reply_["reason"] == "operator_placed_start"


def test_the_bypass_never_lifts_the_one_way_block():
    r = rig()
    _not_localized(r)
    ask(r, "check_bypass", check="slam", on=True)
    r.tick()
    r.arbiter.block_start("test", r.now / 1e9)
    assert r.loop._start_blocked() is True
    reply_, _ = ask(r, "mode_request", to="AUTO")
    assert reply_["reason"] == "operator_placed_start"


def test_the_latch_outlives_a_later_ready_seed():
    r = rig()
    src = _not_localized(r)
    first = dict(r.loop.manual_only)
    gen = open_generation(r, "reboot")
    stand(r)
    ask(r, "relocalize_declare", generation=gen)
    r.tick(5)
    src.end("ready")
    run_until(r, lambda rec: status(r)["state"] != "waiting", 50)
    assert status(r)["state"] == "declared"
    assert r.loop.manual_only == first and r.loop.real_status(r.now)["manual_only"] == first


def test_the_latch_does_not_refuse_the_field_recovery_report():
    from forklift_core.safety import arbiter as arb

    assert "recovery" in arb.START_REFUSED and "recovery" not in arb.START_GATE_REFUSED


def test_the_run_guard_passes_the_acceptance_and_seed_states_through():
    from collections import Counter

    from forklift_core.live.real_run import _SlamGuard

    marks = []
    plain = _SlamGuard(type("Plain", (), {"replies": lambda self, n: []})(), marks.append, Counter())
    assert not hasattr(plain, "can_confirm") and not hasattr(plain, "seed_state")

    class Src:
        def replies(self, n):
            return []

        def can_confirm(self):
            return "slam_not_ready"

        def can_seed(self):
            return None

        def seed_state(self, epoch):
            raise RuntimeError("link down")

    g = _SlamGuard(Src(), marks.append, Counter())
    assert g.can_confirm() == "slam_not_ready" and g.can_seed() is None
    with pytest.raises(RuntimeError):
        g.seed_state(3)
    assert marks and marks[-1].startswith("slam:seed_state:RuntimeError")


def test_each_mission_result_carries_the_bypasses_applied_during_it():
    from forklift_core.live import real_autonomy as ra
    from test_real_loop_console import start_and_submit

    r = rig()
    reply_, _ = ask(r, "check_bypass", check="slam", on=True)
    assert reply_["t"] == "ack"
    r.tick()
    start_and_submit(r)
    run_until(r, lambda rec: r.permit.state == ra.LATCHED and r.loop.session.results, 4000)
    assert r.loop.session.results[-1]["bypassed"] == ["slam"]
