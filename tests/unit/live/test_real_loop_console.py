"""docs/design/2026-11-19-real-safety-wiring.md 5 (stage 5c-1) on the fake truck: the real console requests through
the arbiter (real_requests, request tokens) -- pallet registration, the operator's start check, the operator's answer
to the lifted check, the session's check bypass (an unlevel IMU bypassed to run S10-A), real_status."""

import math

from forklift_core.live import real_autonomy as ra

from real_loop_rig import EPOCH, PALLET, Rig


def run_until(r, cond, limit=3000):
    for _ in range(limit):
        rec = r.tick()
        if cond(rec):
            return rec
    raise AssertionError(f"not reached: {r.loop.records[-1]}")


def replies(r):
    out, r.console_out[:] = list(r.console_out), []
    return [m for m in out if m.get("t") in ("ack", "reject")], [m for m in out if m.get("t") == "real_status"]


def rig(**kw):
    r = Rig(frames=True, real_requests=True, **kw)
    r.start()
    r.console_out.clear()
    return r


def ask(r, request_kind, /, **fields):
    kind = request_kind
    rid = r.say(kind, **fields)
    r.tick()
    got, statuses = replies(r)
    mine = [m for m in got if m.get("request_id") == rid]
    assert mine, (kind, got)
    return mine[0], statuses


def start_and_submit(r):
    pid = next(iter(r.core.targets.pallets))
    reply, _ = ask(r, "pallet_register", pallet_id=pid, pose=list(PALLET))
    assert reply["t"] == "ack", reply
    rev = reply["targets_revision"]
    r.tick(30)  # stands
    reply, _ = ask(r, "operator_start")
    assert reply["t"] == "ack", reply
    r.say("mission_submit", pallet_id="P-S10", destination_id="D-S10", targets_revision=rev)


def test_s10_a_through_the_real_console_requests():
    r = rig()
    start_and_submit(r)
    run_until(r, lambda rec: r.core.operator_open() is not None)
    op = r.core.operator_open()
    r.tick()
    _, statuses = replies(r)
    r.tick(5)
    _, more = replies(r)
    assert any(s["operator_request"] == op for s in statuses + more)
    reply, _ = ask(r, "operator_answer", kind=op["kind"], serial=op["serial"], verdict="confirm")
    assert reply["t"] == "ack", reply
    again, _ = ask(r, "operator_answer", kind=op["kind"], serial=op["serial"], verdict="reject")
    assert again["reason"] in ("already_answered", "no_operator_request", "stale_operator_request")
    run_until(r, lambda rec: r.permit.state == ra.LATCHED and r.loop.session.results)
    assert [e["reason"] for e in r.loop.session.results] == ["s10_lifted"]


def test_refusals_by_reason():
    r = rig()
    reply, _ = ask(r, "pallet_register", pallet_id="NOPE", pose=[1.0, 0.0, 0.0])
    assert reply["reason"] == "unknown_pallet"
    reply, _ = ask(r, "operator_answer", kind="lifted", serial=1, verdict="confirm")
    assert reply["reason"] == "no_operator_request"
    reply, _ = ask(r, "check_bypass", check="stop", on=True)
    assert reply["reason"] == "not_bypassable"
    reply, _ = ask(r, "check_bypass", check="nonsense", on=True)
    assert reply["reason"] == "unknown_check"
    reply, _ = ask(r, "recovery", recovery_id=1, action="done")
    assert reply["reason"] == "no_open_recovery"
    msg = {"t": "check_bypass", "request_id": "nt1", "session_epoch": EPOCH, "control_gen": r.arbiter.control_gen,
           "lease_id": r.lease, "check": "imu", "on": True}
    r.channels.message("c1", "op", msg)  # no request token
    r.tick()
    got, _ = replies(r)
    assert [m["reason"] for m in got if m.get("request_id") == "nt1"] == ["token_required"]


def test_a_bypassed_imu_lets_an_unlevel_truck_run_s10_and_real_status_shows_it():
    r = rig(imu=True)
    a = math.radians(12)
    r.imu.accel = (9.81 * math.sin(a), 0.0, 9.81 * math.cos(a))  # parked on a slope: not level
    reply, _ = ask(r, "check_bypass", check="imu", on=True)
    assert reply["t"] == "ack" and reply["applies"] == "next_tick"
    r.tick(3)
    _, statuses = replies(r)
    r.tick(5)
    _, more = replies(r)
    assert any(s["bypassed"] == ["imu"] for s in statuses + more)
    start_and_submit(r)
    run_until(r, lambda rec: r.mcu.arms > 0, limit=400)
    assert not r.loop.imu_watch.armed  # bypassed: never armed, never judged
    op = None
    run_until(r, lambda rec: r.core.operator_open() is not None or (r.permit.state == ra.LATCHED))
    op = r.core.operator_open()
    if op is not None:
        ask(r, "operator_answer", kind=op["kind"], serial=op["serial"], verdict="confirm")
    run_until(r, lambda rec: r.permit.state == ra.LATCHED and r.loop.session.results)
    assert [e["reason"] for e in r.loop.session.results] == ["s10_lifted"]


def test_without_the_bypass_the_unlevel_truck_is_refused():
    r = rig(imu=True)
    a = math.radians(12)
    r.imu.accel = (9.81 * math.sin(a), 0.0, 9.81 * math.cos(a))
    start_and_submit(r)
    rec = run_until(r, lambda rec: rec.branch == "emergency", limit=400)
    assert ("imu_not_level" in rec.reason or "imu_baseline_unsteady" in rec.reason) and r.mcu.arms == 0


def test_a_bypass_toggle_is_refused_while_armed_and_cleared_with_the_session():
    r = rig()
    start_and_submit(r)
    run_until(r, lambda rec: r.permit.state in (ra.READY, ra.DRIVING), limit=400)
    reply, _ = ask(r, "check_bypass", check="slam", on=True)
    assert reply["reason"] == "armed"
    r2 = rig()
    ask(r2, "check_bypass", check="slam", on=True)
    r2.tick()
    assert r2.loop.bypass == {"slam"}
    r2.arbiter.session_epoch += 1  # a new session
    r2.tick()
    assert r2.loop.bypass == set()


def test_the_disarm_is_not_confirmed_while_a_console_disarm_or_a_queued_disarm_waits():
    """5c-1 r1 P2-4 / r2 P2-1: recovery's 'disarmed' needs no DISARM pending anywhere."""
    r = rig()
    r.tick(5)
    loop = r.loop
    ok = loop._disarm_confirmed(int(loop.clock()))  # noqa: SLF001
    loop._console_disarm = True  # noqa: SLF001 -- accepted this tick, not yet run
    assert not loop._disarm_confirmed(int(loop.clock()))  # noqa: SLF001
    loop._console_disarm = False  # noqa: SLF001
    r.host.request_disarm(r.now)  # an anchor-like DISARM queued, not written
    assert not loop._disarm_confirmed(int(loop.clock()))  # noqa: SLF001
    assert isinstance(ok, bool)


def test_an_operator_start_given_then_moved_is_void():
    r = rig()
    r.tick(30)
    reply, _ = ask(r, "operator_start")
    assert reply["t"] == "ack" and r.loop.operator_start.ok(r.lease)
    r.mcu.counts[0] += 500  # the truck is pushed: not still
    r.mcu.counts[1] += 500
    r.tick(3)
    assert not r.loop.operator_start.ok(r.lease)
