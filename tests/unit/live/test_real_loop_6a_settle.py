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
