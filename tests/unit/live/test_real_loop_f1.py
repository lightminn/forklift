"""PR 7c-2b (docs/design/2026-11-25-real-sources-pr7.md 6.1, 7, and lead #623 decisions 1-2): the F1 start block on
the real path. Every seed is recorded by the loop; once a generation closes, a session whose last seed is not a
completed declaration is blocked at the three start boundaries (the arbiter's request check, the Context, the AUTO
entry) to its end -- read through the f1 bypass, which caps the speed at 0.06 m/s and takes the missions it covers out
of the F1 and validation counts. A start with no fresh real processed reply is never bypassed: the ARM's write itself
needs one."""

import pytest

from forklift_core.live import real_autonomy as ra
from forklift_core.live import real_loop as rl

from test_real_loop_console import ask, run_until, start_and_submit
from test_real_loop_relocalize import open_generation, rig, stand, status
from test_real_loop_slam_real import seeding


def _declare(r, src, gen, end="ready"):
    """A declaration on the seeding source; ``end``: the seed's end state (ready / not_localized / void) or None for
    a declaration left waiting."""
    stand(r)
    reply_, _ = ask(r, "relocalize_declare", generation=gen)
    assert reply_["t"] == "ack", reply_
    r.tick(5)
    if end is not None:
        src.end(end)
        run_until(r, lambda rec: status(r)["state"] != "waiting", 60)


def _confirm(r, gen):
    stand(r)
    reply_, _ = ask(r, "relocalize_confirm", generation=gen)
    assert reply_["t"] == "ack", reply_
    run_until(r, lambda rec: status(r)["state"] != "waiting", 200)
    assert status(r)["state"] == "confirmed", status(r)


def _refused_declaration_then_confirmation(r, reason="reboot"):
    """G1: a declaration seeded and then refused (its seed's end state came past the limit: relocalize_timeout);
    the instance is localized later; G2 opened and closed by a confirmation (the design's G1/G2 example)."""
    src = seeding(r)
    g1 = open_generation(r)
    _declare(r, src, g1, end=None)
    run_until(r, lambda rec: status(r)["state"] != "waiting", 2000)
    assert status(r)["state"] == "refused" and status(r)["reason"] == "relocalize_timeout"
    assert r.loop.seed_log and r.loop.seed_log[-1]["declared"] is False
    assert r.loop.f1_block is None  # no judgement right after a seed or a refusal: only when a generation closes
    src.end("ready")
    r.tick(20)
    g2 = open_generation(r, reason)
    _confirm(r, g2)
    return src


def test_a_start_generation_closed_by_a_confirmation_has_no_seed_and_no_block():
    r = rig()
    gen = open_generation(r, "session_start")
    _confirm(r, gen)
    assert r.loop.seed_log == [] and r.loop.f1_block is None and r.loop._start_blocked() is False


def test_a_completed_declaration_leaves_no_block_and_caps_the_speed():
    r = rig()
    src = seeding(r)
    gen = open_generation(r)
    _declare(r, src, gen)
    assert status(r)["state"] == "declared"
    assert [row["declared"] for row in r.loop.seed_log] == [True]
    assert r.loop.f1_block is None and r.loop._speed_cap() == pytest.approx(0.06)


def test_a_refused_declaration_then_a_confirmation_blocks_the_three_boundaries():
    r = rig()
    _refused_declaration_then_confirmation(r)
    blk = r.loop.f1_block
    assert blk is not None and blk["source_kind"] == "seed_not_declared" and blk["closed_kind"] == "confirm"
    assert blk["seed"]["seed_id"] == r.loop.seed_log[-1]["seed_id"]  # the G1 seed named in the block
    assert any(e["event"] == "f1_block" for e in r.loop.reloc_events)
    # the request check and the AUTO entry read the gate
    reply_, _ = ask(r, "mission_submit", pallet_id="P-S10", destination_id="D-S10", targets_revision=0)
    assert reply_["t"] == "reject" and reply_["reason"] == "operator_placed_start"
    reply_, _ = ask(r, "mode_request", to="AUTO")
    assert reply_["t"] == "reject" and reply_["reason"] == "operator_placed_start"
    st = r.loop.real_status(r.now)
    assert st["f1_block"] == blk and st["f1_blocking"] is True
    # the Context on its own (the arbiter's gate taken away)
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


def test_the_auto_entry_assertion_reads_the_same_gate():
    """The arbiter's AUTO entry raises on a closed gate (its own path past the two refusals)."""
    r = rig()
    _refused_declaration_then_confirmation(r)
    gate = r.arbiter._gate()
    assert gate is not None and gate["source"] == "f1"


def test_g1_declared_then_g2_confirmed_is_not_blocked_and_keeps_the_cap():
    r = rig()
    src = seeding(r)
    g1 = open_generation(r)
    _declare(r, src, g1)
    assert status(r)["state"] == "declared"
    g2 = open_generation(r, "reboot")
    _confirm(r, g2)
    assert r.loop.f1_block is None and r.loop._start_blocked() is False
    assert r.loop._speed_cap() == pytest.approx(0.06)  # the declaration's cap holds to the session's end


def test_a_declaration_without_its_seed_row_is_unsourced():
    """Codex 7c-2b design P2: the whole key must match -- a G1 row completed does not stand for a G2 declaration."""
    r = rig()
    src = seeding(r)
    g1 = open_generation(r)
    _declare(r, src, g1)
    g2 = open_generation(r, "reboot")
    plain_append = r.loop.seed_log.append
    r.loop.seed_log = type("NoRecord", (list,), {"append": lambda self, row: None})(r.loop.seed_log)
    _declare(r, src, g2)
    assert r.loop.f1_block is not None and r.loop.f1_block["source_kind"] == "declare_unsourced"
    del plain_append


def test_the_block_holds_through_stop_rearm_and_a_lease_change():
    """STOP, a lease's end and a new lease write no record and lift nothing (6.1, 7)."""
    r = rig()
    _refused_declaration_then_confirmation(r)
    blk = dict(r.loop.f1_block)
    ask(r, "stop")
    r.tick(5)
    assert r.loop.f1_block == blk and r.loop._start_blocked() is True
    r.arbiter.lease = None  # the lease gone (an expiry): the block is the session's, not the lease's
    r.tick(5)
    assert r.loop.f1_block == blk and r.loop._start_blocked() is True
    assert len(r.loop.seed_log) == 1  # no new record from either


def test_teleop_and_the_field_recovery_report_stay_open_while_blocked():
    from forklift_core.safety import arbiter as arb

    r = rig()
    _refused_declaration_then_confirmation(r)
    reply_, _ = ask(r, "mode_request", to="TELEOP")
    assert reply_.get("reason") != "operator_placed_start"
    assert "recovery" not in arb.START_GATE_REFUSED  # the gate's list: the report goes through


def test_the_f1_bypass_lifts_the_block_caps_and_turning_it_off_blocks_again():
    r = rig()
    _refused_declaration_then_confirmation(r)
    reply_, _ = ask(r, "check_bypass", check="f1", on=True)
    assert reply_["t"] == "ack"
    r.tick()
    assert r.loop._start_blocked() is False and r.loop.real_status(r.now)["f1_blocking"] is False
    assert "f1" in r.loop.real_status(r.now)["bypassed"]
    reply_, _ = ask(r, "mode_request", to="AUTO")
    assert reply_.get("reason") != "operator_placed_start"
    assert r.loop.f1_block is not None  # recorded, kept
    ask(r, "check_bypass", check="f1", on=False)
    r.tick()
    assert r.loop._start_blocked() is True


def test_the_f1_bypass_alone_caps_the_written_drives_and_its_missions_are_not_counted():
    """Codex 7c-2b design P3: f1 alone, no declaration in the session -- the new branch of the cap; the mission's
    result is out of the F1 and validation counts."""
    r = rig()
    reply_, _ = ask(r, "check_bypass", check="f1", on=True)
    assert reply_["t"] == "ack"
    r.tick()
    assert not r.loop._declared() and r.loop._speed_cap() == pytest.approx(0.06)
    booked, written = {}, []
    orig_book, orig_written = r.loop._book, r.loop.book.written

    def book_spy(now_ns, seq, speed, *a, **k):
        booked[int(seq) & 0xFFFF] = float(speed)
        return orig_book(now_ns, seq, speed, *a, **k)

    def written_spy(seq, written_ns):
        written.append(booked.get(int(seq) & 0xFFFF))
        return orig_written(seq, written_ns)

    r.loop._book, r.loop.book.written = book_spy, written_spy
    start_and_submit(r)
    run_until(r, lambda rec: any(abs(v or 0.0) > 0 for v in written), 1500)
    r.tick(40)
    moving = [abs(v) for v in written if v]
    assert moving and max(moving) <= 0.06 + 1e-9
    run_until(r, lambda rec: r.permit.state == ra.LATCHED and r.loop.session.results, 4000)
    res = r.loop.session.results[-1]
    assert "f1" in res["bypassed"] and res["excluded_counts"] == ["f1", "validation"]
    assert rl.mission_counts(r.loop.session.results) == {"all": 1, "counted": 0, "excluded_f1": 1}


def test_a_new_session_starts_with_no_seed_and_no_block():
    r = rig()
    _refused_declaration_then_confirmation(r)
    r.arbiter.session_epoch += 1  # a new session (a new bridge: no seed)
    r.tick()
    assert r.loop.seed_log == [] and r.loop.f1_block is None


def test_the_arm_is_never_written_without_a_fresh_processed_reply_whatever_is_bypassed():
    """Lead #623 decision 1 (Codex 7c-2b design P1): with slam, f1 and operator_start bypassed, the ARM's own write
    needs this round's processed reply fresh. Each tick head sees the reply 0.497 s old (every earlier gate passes);
    at the write it is past SLAM_MAX_AGE_S: the ARM is refused there."""
    import dataclasses

    from forklift_core.mcu_link.messages import Arm

    r = rig()
    for check in ("slam", "f1", "operator_start"):
        ask(r, "check_bypass", check=check, on=True)
        r.tick()
    plain, head = r.loop.evidence, {"ns": None}
    cycle = r.loop.cycle

    def cycle_spy(now_ns):
        head["ns"] = int(now_ns)
        return cycle(now_ns)

    r.loop.cycle = cycle_spy

    def aged(n):
        sl = plain.slam(n)
        if head["ns"] is None:
            return sl
        return dataclasses.replace(sl, pose_reply_ns=head["ns"] - int(0.497 * 1e9))  # 0.497 s at the head

    r.loop.evidence = dataclasses.replace(plain, slam=aged)
    since = len(r.transport.sent)
    start_and_submit(r)
    r.tick(300)
    assert not any(isinstance(m, Arm) for _, k, _, m in r.transport.sent[since:])
    assert any("arm_slam_stale" in str(e) for e in r.loop.disarm_decisions) or r.permit.reason == "arm_slam_stale" \
        or any("arm_slam_stale" in str(getattr(rec, "reason", "")) for rec in r.loop.records)
