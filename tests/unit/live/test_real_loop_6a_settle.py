"""PR 6a-1 (docs/design/2026-11-20-real-hil-6a.md 2, 5): an exception tick's console settlement (apply before, during
and after, a begin that raised part-way, tick_end tried once), duplicate request ids refused, and the arming exchange --
a late ARM kept and written next tick, the ARM's binding over its zero, a kept ARM's echo window."""

from forklift_core.live import real_autonomy as ra

from real_loop_rig import MS, Rig


def run_until(r, cond, limit=4000):
    for _ in range(limit):
        rec = r.tick()
        if cond(rec):
            return rec
    raise AssertionError(f"not reached: {r.loop.records[-1]}")


def replies(r):
    out = [m for m in r.console_out if m.get("t") in ("ack", "reject")]
    r.console_out.clear()
    return out


def submit_with(r, inject):
    """The rig's submit with ``inject()`` run just before the request goes in (its tick is the next one)."""
    from real_loop_rig import PALLET, asm

    rev = asm.register_pallet(r.core, next(iter(r.core.targets.pallets)), PALLET)
    r.tick(30)  # stands
    inject()
    r.say("mission_submit", pallet_id="P-S10", destination_id="D-S10", targets_revision=rev)


def ready_to_submit():
    r = Rig(frames=True)
    r.start()
    return r


def failing_once(obj, name, *, after=False):
    """``obj.name`` raises on its next call (after running the original when ``after``), then behaves again."""
    real = getattr(obj, name)
    state = {"armed": True}

    def wrapper(*a, **k):
        if state["armed"]:
            state["armed"] = False
            if after:
                real(*a, **k)
            raise RuntimeError(f"injected:{name}")
        return real(*a, **k)

    setattr(obj, name, wrapper)
    return state


def test_an_exception_before_the_apply_refuses_the_queued_submit_and_it_never_runs():
    r = ready_to_submit()
    submit_with(r, lambda: failing_once(r.core.commands, "apply"))
    rec = r.tick()
    assert rec.branch == "emergency" and "injected:apply" in rec.reason
    got = replies(r)
    assert any(m.get("reason") == "tick_failed" for m in got), got
    assert r.core.commands._queue == []  # noqa: SLF001
    for _ in range(40):
        r.tick()
    assert r.core.commands.status in ("idle",) + tuple(("failed", "cancelled"))
    assert r.link.a.mode != "AUTO"


def test_an_exception_inside_the_apply_ends_the_changed_mission_and_answers_tick_failed():
    r = ready_to_submit()
    submit_with(r, lambda: failing_once(r.core.commands, "_submit", after=True))  # state changed, no record
    r.tick()
    got = replies(r)
    assert any(m.get("reason") == "tick_failed" for m in got), got
    assert r.core.commands.status in ("failed", "idle")
    assert r.link.a.mode != "AUTO"


def test_a_begin_that_raised_after_queueing_still_empties_the_queue():
    r = ready_to_submit()
    submit_with(r, lambda: failing_once(r.link.a, "mission_context", after=True))
    r.tick()
    assert r.core.commands._queue == []  # noqa: SLF001
    got = replies(r)
    assert any(m.get("reason") == "tick_failed" for m in got), got
    for _ in range(40):
        r.tick()
    assert r.link.a.mode != "AUTO"


def test_a_tick_end_that_raised_is_not_tried_again_in_the_tick():
    r = ready_to_submit()
    calls = {"n": 0}
    real = r.loop.session.tick_end

    def counting(*a, **k):
        calls["n"] += 1
        if calls["n"] == 1:
            raise RuntimeError("injected:tick_end")
        return real(*a, **k)

    submit_with(r, lambda: setattr(r.loop.session, "tick_end", counting))
    r.tick()
    assert calls["n"] == 1


def test_a_second_unsettled_request_with_the_same_id_is_refused():
    r = ready_to_submit()
    a = r.link.a
    first = a._mission_request("c", "dup", "mission_cancel", object())  # noqa: SLF001
    second = a._mission_request("c", "dup", "mission_cancel", object())  # noqa: SLF001
    assert first is None
    assert second is not None and second["reason"] == "duplicate_request"
    a.pending = []


# --- the arming exchange -----------------------------------------------------------------------------------------
def test_a_late_arm_is_kept_and_written_next_tick():
    r = ready_to_submit()
    real = r.loop._lease_valid
    state = {"done": False}

    def slow_once(ns):
        if not state["done"] and any(o.kind == "ARM" for o in [r.host._inflight] if o is not None):  # noqa: SLF001
            state["done"] = True
            r.tick_cost_ns += int(4.5 * MS)
        return real(ns)

    r.loop._lease_valid = slow_once
    r.submit()
    run_until(r, lambda rec: r.permit.state in (ra.READY, ra.DRIVING, ra.LATCHED), limit=400)
    assert state["done"]
    assert r.permit.state in (ra.READY, ra.DRIVING)


def test_the_arming_zero_is_refused_when_the_authority_moved_after_the_arm():
    r = ready_to_submit()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.ARMING, limit=400)
    r.loop._arm_binding = ("other-lease", -1)  # noqa: SLF001 -- as if the lease or generation had moved
    rec = run_until(r, lambda rec: rec.branch == "emergency", limit=50)
    assert "authority_changed" in (rec.reason or "")


def late_arm_always(r):
    """Every judgement of an ARM in flight is 4.5 ms late at its last clock read (each transmit starts on time)."""
    real_lease, real_tx = r.loop._lease_valid, r.transport.transmit

    def slow(ns):
        o = r.host._inflight  # noqa: SLF001
        if o is not None and o.kind == "ARM":
            r.tick_cost_ns += int(4.5 * MS)
        return real_lease(ns)

    def tx(host, now, gate):
        try:
            return real_tx(host, now, gate)
        finally:
            r.tick_cost_ns = 0

    r.loop._lease_valid, r.transport.transmit = slow, tx


def layer_rig():
    r = Rig(frames=True, operator=[(1, "confirm")], real_layer=True)
    r.start()
    r.tick(40)
    return r


def test_a_kept_arm_past_its_echo_window_is_given_up_before_the_arm_timeout():
    r = layer_rig()
    late_arm_always(r)
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.ARMING, limit=400)
    t0 = r.now
    run_until(r, lambda rec: r.permit.state == ra.LATCHED, limit=100)
    assert r.now - t0 < int(0.3e9)  # the echo window (cap 150 ms), not the host's 300 ms ARM timeout
    assert not [m for _, kind, _, m in r.transport.sent if kind == "ARM"]


def test_a_kept_arm_without_its_echo_delay_bound_is_given_up():
    r = layer_rig()
    late_arm_always(r)
    real = r.loop._lease_valid

    def forget(ns):
        r.loop._sample_delay.clear()  # noqa: SLF001 -- no delay bound for the echo STATE any more
        return real(ns)

    r.loop._lease_valid = forget
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.LATCHED, limit=400)
    assert not [m for _, kind, _, m in r.transport.sent if kind == "ARM"]


# --- Codex 6a-1 impl round 1 -------------------------------------------------------------------------------------
def test_a_half_arming_zero_is_not_finished_next_tick():
    r = ready_to_submit()
    real = r.transport.transmit
    seen = {}

    def stall_zero(host, now, gate):
        front = host._outbox[0] if host._outbox else None  # noqa: SLF001
        if not seen and front is not None and front.kind == "DRIVE" and front.echo is not None:
            seen["zero"] = front
            r.transport.stall = True
        elif seen and r.transport.stall:
            r.transport.stall = False
        return real(host, now, gate)

    r.transport.transmit = stall_zero
    r.submit()
    run_until(r, lambda rec: "zero" in seen and not r.transport.stall, limit=400)
    k = len(r.transport.sent)
    r.tick()
    sent = [(kind, seq) for _, kind, seq, _ in r.transport.sent[k - 1:]]
    assert ("DRIVE", seen["zero"].seq) not in sent
    run_until(r, lambda rec: r.permit.state in (ra.DISARMING, ra.LATCHED), limit=20)


def test_a_failing_collection_after_s10s_stop_still_writes_the_disarm_in_its_tick():
    r = Rig(frames=True, operator=[(1, "confirm")])
    r.start()
    r.submit()
    run_until(r, lambda rec: r.core.ms.phase == "lift", limit=4000)
    failing_once(r.core, "abandon_jobs")  # raises at the mission's collection (its end after S10's stop)
    for _ in range(2000):
        k = len(r.transport.sent)
        rec = r.tick()
        if rec.reason and "exception:collect" in rec.reason:
            assert [kind for _, kind, _, _ in r.transport.sent[k:] if kind == "DISARM"]
            return
    raise AssertionError("the collection never ran")


def test_an_arm_sent_back_by_eagain_is_given_up_past_its_echo_window():
    r = layer_rig()
    r.transport.eagain = {"ARM"}
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.ARMING, limit=400)
    t0 = r.now
    run_until(r, lambda rec: r.permit.state == ra.LATCHED, limit=100)
    assert r.now - t0 < int(0.3e9)
    assert not [m for _, kind, _, m in r.transport.sent if kind == "ARM"]


def test_a_settled_request_is_not_answered_again_by_resolve():
    r = ready_to_submit()
    a = r.link.a
    a._mission_request("c", "x1", "mission_cancel", object())  # noqa: SLF001
    a._mission_request("c", "x2", "mission_cancel", object())  # noqa: SLF001
    a._settle(a.pending[0], a._reply("mission_cancel", "x1", "stale_control"))  # noqa: SLF001
    out = a.resolve(r.now / 1e9, [])
    assert [m["request_id"] for m in out].count("x1") == 1


def test_the_disarm_decision_instant_is_recorded_on_the_clock():
    r = ready_to_submit()
    r.tick_cost_ns = 6 * MS
    r.loop._emergency(r.now, "injected")  # noqa: SLF001
    r.tick_cost_ns = 0
    assert r.loop.disarm_decisions[-1] == (r.now + 6 * MS, "injected")


# --- Codex 6a-1 impl round 2 -------------------------------------------------------------------------------------
def test_a_failing_end_collection_after_an_early_return_still_answers_the_ticks_requests():
    r = ready_to_submit()
    real_ctrl, real_collect = r.loop._control_and_output, r.loop._collect  # noqa: SLF001
    state = {"armed": False, "calls": 0}

    def early(now_ns, t, rec):
        if state["armed"]:
            return None  # an early return before tick_end (as a mission ended at the head)
        return real_ctrl(now_ns, t, rec)

    def collect(now_ns, t, rec):
        if state["armed"]:
            state["calls"] += 1
            if state["calls"] == 3:  # the tick's final collection
                state["armed"] = False
                raise RuntimeError("injected:collect")
        return real_collect(now_ns, t, rec)

    r.loop._control_and_output, r.loop._collect = early, collect  # noqa: SLF001
    submit_with(r, lambda: state.update(armed=True))
    rec = r.tick()
    assert "exception:collect" in (rec.reason or "")
    got = replies(r)
    assert [m for m in got if m.get("of") == "mission_submit"], got  # answered in this tick, not invalid next tick


def test_a_collection_that_raised_is_kept_with_the_failure_and_new_missions_are_refused():
    r = Rig(frames=True, operator=[(1, "confirm")])
    r.start()
    r.submit()
    run_until(r, lambda rec: r.core.ms.phase == "lift", limit=4000)
    failing_once(r.core, "abandon_jobs")
    run_until(r, lambda rec: bool(r.loop.session.results), limit=2000)
    res = r.loop.session.results[-1]
    assert res["reason"] == "s10_lifted" and res["collect_failed"].startswith("RuntimeError")
    assert r.loop.real_status(r.now)["restart_required"] == res["collect_failed"]
    # through the console: rearm, then a submit is refused with the restart need, ahead of the cargo's refusal
    replies(r)
    r.say("rearm")
    r.tick(5)
    submit_with(r, lambda: None)
    r.tick()
    got = [m for m in replies(r) if m.get("of") == "mission_submit"]
    assert got and got[-1].get("reason") == "not_ready:collect_failed", got

def test_an_arming_zero_first_judged_after_its_expiry_ends_the_exchange():
    """Impl round 3 P3: the zero's first gate opens 98 ms after its queueing (the queue still hands it out, the gate's
    5 ms lookahead finds it expired): the exchange ends at once, not at the 300 ms ARM timeout."""
    r = ready_to_submit()
    real = r.transport.transmit
    seen = {}

    def hold_zero(host, now, gate):
        front = host._outbox[0] if host._outbox else None  # noqa: SLF001
        if "q" not in seen and front is not None and front.kind == "DRIVE" and front.echo is not None:
            seen["q"] = now  # queued at this tick's head
        q = seen.get("q")
        if q is not None and now - q < 80 * MS:
            return []  # no send judges it for four ticks
        if q is not None and now - q == 80 * MS:
            r.tick_cost_ns = 16 * MS  # its first gate: the queue (tick start) still hands it out, ns = +101 ms
        try:
            return real(host, now, gate)
        finally:
            r.tick_cost_ns = 0

    r.transport.transmit = hold_zero
    r.submit()
    run_until(r, lambda rec: "q" in seen, limit=400)
    r.tick(4)
    run_until(r, lambda rec: r.permit.state == ra.LATCHED, limit=2)
    assert not r.host.authorized


def test_an_arming_zero_that_cannot_go_out_in_time_ends_the_exchange():
    r = ready_to_submit()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.ARMING, limit=400)
    r.transport.eagain = {"DRIVE"}
    t0 = r.now
    run_until(r, lambda rec: r.permit.state == ra.LATCHED, limit=100)
    assert r.now - t0 < int(0.3e9)


def test_a_disarm_the_permit_decides_itself_is_recorded():
    from test_real_loop_6a_gate import driving

    r = driving()
    n = len(r.loop.disarm_decisions)
    r.transport.silent = True
    run_until(r, lambda rec: r.permit.state in (ra.DISARMING, ra.LATCHED), limit=50)
    assert len(r.loop.disarm_decisions) > n


# --- Codex 6a-1 impl round 3 -------------------------------------------------------------------------------------
def test_a_permit_disarm_is_recorded_at_its_decision_before_its_send():
    from test_real_loop_6a_gate import driving

    r = driving()
    real = r.transport.transmit
    entries = []

    def slow(host, now, gate):
        entries.append(r.now + r.tick_cost_ns)
        r.tick_cost_ns += 7 * MS
        return real(host, now, gate)

    r.transport.transmit = slow
    n = len(r.loop.disarm_decisions)
    r.transport.silent = True
    for _ in range(50):
        entries.clear()
        r.tick_cost_ns = 0
        rec = r.tick()
        if len(r.loop.disarm_decisions) > n:
            break
    at, why = r.loop.disarm_decisions[-1]
    # taken at the transition: before the send that carried its DISARM (the last send of the tick), never after it
    assert entries and at <= entries[-1] and at < r.now + r.tick_cost_ns


def test_an_axis_judgement_on_a_sample_past_the_age_bound_is_marked_invalid():
    from test_real_loop_6a_gate import driving, holding

    r = holding(driving())
    r.tick_cost_ns = 75 * MS
    r.loop._axis_judge(r.host.state)  # noqa: SLF001
    r.tick_cost_ns = 0
    assert r.loop.axis_watch_times[-1][1] is False


def test_an_s10_stop_discarded_at_its_first_byte_records_the_disarm_decision():
    r = Rig(frames=True, operator=[(1, "confirm")])
    r.start()
    r.submit()
    real = r.transport.transmit

    def eagain_stop(host, now, gate):
        r.transport.eagain = {"DRIVE"} if r.permit.state == ra.STOP_PENDING else set()
        return real(host, now, gate)

    r.transport.transmit = eagain_stop
    k = {"n": len(r.transport.sent)}

    def before(rec):
        if r.permit.state in (ra.DISARMING, ra.LATCHED):
            return True
        k["n"] = len(r.transport.sent)
        return False

    run_until(r, before, limit=4000)
    assert r.permit.reason == "control_stop_discarded"
    assert r.loop.disarm_decisions[-1][1] == "control_stop_discarded"
    assert [kind for _, kind, _, _ in r.transport.sent[k["n"]:] if kind == "DISARM"]  # written in the discard's tick

def test_an_axis_judgement_without_a_lift_measurement_is_marked_invalid():
    from dataclasses import replace as _r

    from test_real_loop_6a_gate import driving, holding

    r = holding(driving())
    r.loop._axis_judge(_r(r.host.state, lift_m=float("nan")))  # noqa: SLF001
    assert r.loop.axis_watch_times[-1][1] is False


# --- 6a-2b design 10.1 ----------------------------------------------------------------------------------------------
def rearm_after_stop(stale):
    r = ready_to_submit()
    r.say("disarm")
    run_until(r, lambda rec: r.permit.state == ra.LATCHED or r.link.a.mode == "STOPPED", limit=50)
    r.link.a.stop(r.now / 1e9, "test")
    r.tick(30)  # standing, DISARMED
    if stale:
        r.transport.silent = True
        r.tick(3)  # no STATE for 60 ms: the host's limit (50 ms) passed, the arbiter's feedback (100 ms) not yet
    replies(r)
    r.say("rearm")
    r.tick()
    return [m for m in replies(r) if m.get("of") == "rearm"]


def test_a_console_rearm_is_taken_on_a_fresh_state_and_refused_on_a_stale_one():
    """6a-2b design 10.1: the arbiter's seconds reach the permit as ns -- the fresh case is the control (with seconds
    passed straight it is refused for another condition), the stale case the check itself."""
    fresh = rearm_after_stop(False)
    assert fresh and fresh[-1]["t"] == "ack", fresh
    stale = rearm_after_stop(True)
    assert stale and stale[-1].get("reason") == "real_rearm_refused" and stale[-1].get("detail") == "state_stale", stale


def test_the_run_host_takes_the_runs_client():
    from forklift_core.live.real_loop import real_host
    from forklift_core.mcu_link.build_policy import Client
    from forklift_core.plant.mcu_host import HostConfig

    cfg = HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100, outbox_max=16)
    assert real_host(cfg).client is Client.REAL
    assert real_host(cfg, client=Client.HIL).client is Client.HIL
