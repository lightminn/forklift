"""The operating unlock U3b-2 (docs/plans/2026-11-26-real-operating-assembly.md 2.6): on the operating run without a
depth source the operator's start check is the D5 confirmation -- bound to the pallet and registration revision the
console shows, this lease and the relocalization generation, carrying the whole bypass region computed from that
registration; it cannot be bypassed, the submit needs the D5 bypass and that very confirmation, a submit uses it up
with its mission id (recorded), and the bypass check the core builds must apply the confirmed region."""

import math
from types import SimpleNamespace

import pytest

from forklift_core.live import real_assembly as asm
from forklift_core.live.real_assembly import RealRequireFailed

from real_loop_rig import PALLET, Rig
from test_real_loop_console import ask, run_until

REGION = asm.bypass_region(PALLET, 0.6, 0.8)  # the rig's transport geometry (not the loop's pallet_size)


# --- the pure rules -------------------------------------------------------------------------------------------------
def test_the_region_is_the_pallet_grown_by_the_bands_about_its_axis():
    x, y, length, width, yaw = asm.bypass_region((1.0, 2.0, 0.3), 0.6, 0.4)
    assert (x, y, yaw) == (1.0, 2.0, 0.3)
    assert length == pytest.approx(0.6 + 2 * asm.BYPASS_BAND_ALONG_M)
    assert width == pytest.approx(0.4 + 2 * asm.BYPASS_BAND_ACROSS_M)


@pytest.mark.parametrize("applied, ok", [
    ((1.0, 2.0, 1.8, 0.8, 0.3), True),
    ((1.0, 2.0, 1.8, 0.8, 0.3 + math.radians(4.9)), True),  # the approach axis within case A's 5 deg
    ((1.0, 2.0, 1.8, 0.8, 0.3 + math.radians(5.1)), False),
    ((1.01, 2.0, 1.8, 0.8, 0.3), False),  # another centre: another registration
    ((1.0, 2.0, 1.9, 0.8, 0.3), False),
    ((1.0, 2.0, 1.8, 0.8), False),
])
def test_the_applied_region_must_be_the_confirmed_one(applied, ok):
    assert asm.bypass_region_matches(applied, [1.0, 2.0, 1.8, 0.8, 0.3]) is ok


def test_a_confirmation_is_taken_only_for_its_own_registration():
    st = asm.OperatorStart(7)
    b = {"pallet_id": "P", "targets_revision": 3, "reloc_gen": None, "region": [0.0] * 5}
    assert st.confirm(7, "L1", b)
    assert not st.ok_for("L1", ("P", 4, None)) and not st.ok_for("L2", ("P", 3, None))
    assert not st.take("L1", ("P", 4, None), "m1") and st.ok_for("L1", ("P", 3, None))  # a refusal keeps it
    assert st.take("L1", ("P", 3, None), "m1")
    assert st.used == {**b, "mission_id": "m1"} and not st.ok("L1")
    st.confirm(7, "L1", b)
    st.void("moved")
    assert st.binding is None and not st.ok_for("L1", ("P", 3, None))


def _hooks(confirmed):
    g = SimpleNamespace(pallet_depth_m=0.6, pallet_width_m=0.8)
    return asm.RealHooks(lift_m=lambda: 0.0, odom_base=lambda: (0.0, 0.0, 0.0), ready=lambda *a: None,
                         started=lambda *a: None, record=lambda *a: None, geometry=g, confirmed_region=confirmed)


def test_the_bypass_check_refuses_without_or_against_the_confirmation():
    pickup = SimpleNamespace(x_m=PALLET[0], y_m=PALLET[1], yaw_rad=PALLET[2])
    assert _hooks(lambda: list(REGION)).build_bypass_pocket_check(PALLET[2], pickup).region_control()
    with pytest.raises(RealRequireFailed, match="d5_confirmation_missing"):
        _hooks(lambda: None).build_bypass_pocket_check(PALLET[2], pickup)
    moved = SimpleNamespace(x_m=PALLET[0] + 0.05, y_m=PALLET[1], yaw_rad=PALLET[2])
    with pytest.raises(RealRequireFailed, match="d5_region_mismatch"):
        _hooks(lambda: list(REGION)).build_bypass_pocket_check(PALLET[2], moved)
    # no confirmation wired (HIL, replay, a run with a depth source): as before
    assert _hooks(None).build_bypass_pocket_check(PALLET[2], moved).region_control()[0] == pytest.approx(moved.x_m)


# --- the console on the fake truck ----------------------------------------------------------------------------------
def rig(no_depth=True, **kw):
    r = Rig(real_requests=True, operating=True, no_depth=no_depth, **kw)
    r.start()
    r.console_out.clear()
    r.loop.run_deadline_ns = r.now + int(1000 * 1e9)
    r.events = []
    r.loop.events = r.events.append
    r.serial = 0
    return r


def register(r, pose=PALLET):
    pid = next(iter(r.core.targets.pallets))
    reply, _ = ask(r, "pallet_register", pallet_id=pid, pose=list(pose))
    assert reply["t"] == "ack", reply
    r.serial = reply["register_serial"]
    r.tick(30)  # stands
    return pid, reply["targets_revision"]


def bypass_d5(r):
    reply, _ = ask(r, "check_bypass", check="d5_pocket", on=True)
    assert reply["t"] == "ack", reply
    r.tick()


def submit(r, rev):
    reply, _ = ask(r, "mission_submit", pallet_id="P-S10", destination_id="D-S10", targets_revision=rev)
    return reply


def test_the_start_check_names_the_registration_and_answers_the_region():
    r = rig()
    reply, _ = ask(r, "operator_start")
    assert reply["t"] == "reject" and reply["reason"] == "operator_start_needs_registration", reply
    pid, rev = register(r)
    sr = r.serial
    for bad in ({"pallet_id": 3, "targets_revision": rev, "register_serial": sr},
                {"pallet_id": pid, "targets_revision": True, "register_serial": sr},
                {"pallet_id": pid, "targets_revision": float(rev), "register_serial": sr},
                {"pallet_id": pid, "targets_revision": rev}, {"pallet_id": pid, "targets_revision": rev,
                                                              "register_serial": True}):
        assert r.loop._d5_refusal(bad) == "operator_start_needs_registration"  # noqa: SLF001
    reply, _ = ask(r, "operator_start", pallet_id=pid, targets_revision=rev - 1, register_serial=r.serial)
    assert reply["t"] == "reject" and reply["reason"] == "targets_changed", reply
    reply, _ = ask(r, "operator_start", pallet_id=pid, targets_revision=rev, register_serial=r.serial)
    assert reply["t"] == "ack" and reply["pallet_id"] == pid and reply["targets_revision"] == rev, reply
    assert reply["region"] == pytest.approx(REGION)
    ev = [e for e in r.events if e["t"] == "operator_start"]
    assert len(ev) == 1 and ev[0]["region"] == pytest.approx(REGION) and ev[0]["pallet_id"] == pid


def test_an_unregistered_pallet_cannot_be_confirmed():
    r = rig()
    pid = next(iter(r.core.targets.pallets))
    r.tick(30)
    reply, _ = ask(r, "operator_start", pallet_id=pid, targets_revision=int(r.core.targets.revision), register_serial=r.serial)
    assert reply["t"] == "reject" and reply["reason"] == "pallet_not_registered", reply


def test_the_start_check_is_not_bypassable_without_depth_only():
    r = rig()
    reply, _ = ask(r, "check_bypass", check="operator_start", on=True)
    assert reply["t"] == "reject" and reply["reason"] == "not_bypassable", reply
    r = rig(no_depth=False)  # a depth source: the check and its bypass as before
    reply, _ = ask(r, "check_bypass", check="operator_start", on=True)
    assert reply["t"] == "ack", reply


def test_the_submit_needs_the_d5_bypass():
    r = rig()
    pid, rev = register(r)
    ask(r, "operator_start", pallet_id=pid, targets_revision=rev, register_serial=r.serial)
    reply = submit(r, rev)
    assert reply["t"] == "reject" and reply["reason"] == "d5_pocket:no_depth_source", reply


def test_a_confirmation_of_an_older_registration_does_not_carry():
    r = rig()
    bypass_d5(r)
    pid, rev = register(r)
    ok, _ = ask(r, "operator_start", pallet_id=pid, targets_revision=rev, register_serial=r.serial)
    assert ok["t"] == "ack", ok
    _, rev2 = register(r, (PALLET[0] + 0.02, PALLET[1], PALLET[2]))  # registered again, still on the placement
    assert rev2 != rev
    reply = submit(r, rev2)
    assert reply["t"] == "reject" and reply["reason"].endswith("operator_start:unconfirmed"), reply


def test_a_confirmation_of_another_relocalization_generation_does_not_carry():
    r = rig()
    bypass_d5(r)
    pid, rev = register(r)
    ask(r, "operator_start", pallet_id=pid, targets_revision=rev, register_serial=r.serial)
    r.loop.reloc_gen = 5  # the place was found again after the check
    reply = submit(r, rev)
    assert reply["t"] == "reject" and reply["reason"].endswith("operator_start:unconfirmed"), reply


def test_the_confirmed_submit_runs_with_the_confirmed_region_and_records_its_use():
    r = rig()
    bypass_d5(r)
    pid, rev = register(r)
    ask(r, "operator_start", pallet_id=pid, targets_revision=rev, register_serial=r.serial)
    reply = submit(r, rev)
    assert reply["t"] == "ack", reply
    run_until(r, lambda rec: r.core.state.get("d5_bypass_check")
              or r.core.commands.status in ("cancelled", "failed", "recovery_required", "done"), 8000)
    assert r.core.state.get("d5_bypass_check"), r.core.commands.end_reason
    assert r.core.state["d5_bypass_check"][-1]["region"][:2] == pytest.approx(REGION[:2])
    used = [e for e in r.events if e["t"] == "operator_start_used"]
    assert len(used) == 1 and used[0]["mission_id"] == r.core.commands.mission_id
    assert used[0]["targets_revision"] == rev and used[0]["region"] == pytest.approx(REGION)
    assert r.loop.confirmed_region() == pytest.approx(REGION)
    assert not r.loop.operator_start.ok(r.lease)  # one submit only


def test_a_region_other_than_the_confirmed_one_ends_the_mission():
    r = rig()
    bypass_d5(r)
    pid, rev = register(r)
    ask(r, "operator_start", pallet_id=pid, targets_revision=rev, register_serial=r.serial)
    assert submit(r, rev)["t"] == "ack"
    r.tick()
    r.loop.operator_start.used["region"] = [PALLET[0] + 0.05, *REGION[1:]]  # what the eyes cleared is elsewhere
    run_until(r, lambda rec: r.core.commands.status in ("cancelled", "failed", "recovery_required", "done"), 8000)
    assert "d5_region_mismatch" in str(r.core.commands.end_reason)
    assert not r.core.state.get("d5_bypass_check")


# --- Codex round 1 -------------------------------------------------------------------------------------------------
def test_a_turned_axis_within_the_tolerance_exempts_the_confirmed_rectangle_itself():
    pickup = SimpleNamespace(x_m=PALLET[0], y_m=PALLET[1], yaw_rad=PALLET[2])
    check = _hooks(lambda: list(REGION)).build_bypass_pocket_check(PALLET[2] + math.radians(4.9), pickup)
    assert list(check.region_control()) == pytest.approx(REGION)  # P3-1: not the turned one


def _confirmed(r):
    bypass_d5(r)
    pid, rev = register(r)
    ok, _ = ask(r, "operator_start", pallet_id=pid, targets_revision=rev, register_serial=r.serial)
    assert ok["t"] == "ack", ok
    return pid, rev


def test_a_submit_without_a_revision_is_refused_not_latched():
    r = rig()
    _confirmed(r)
    reply, _ = ask(r, "mission_submit", pallet_id="P-S10", destination_id="D-S10")
    assert reply["t"] == "reject", reply  # P2-1: no int(None) inside the tick
    r.tick(5)
    assert r.permit.state != "LATCHED" and r.loop.permit.reason is None


def test_every_submit_of_a_batch_is_judged_on_its_own():
    r = rig()
    pid, rev = _confirmed(r)
    ok = SimpleNamespace(kind="submit", pallet_id=pid, targets_revision=rev)
    other = SimpleNamespace(kind="submit", pallet_id=pid, targets_revision=rev + 1)
    now = r.now
    assert r.loop._operating_submit_refusal([ok], now) is None  # noqa: SLF001
    for batch in ([ok, other], [other, ok]):  # P2-2: neither order lets the unconfirmed one ride the other
        assert r.loop._operating_submit_refusal(batch, now) == "operator_start:unconfirmed"  # noqa: SLF001
    assert r.loop._d5_key(SimpleNamespace(pallet_id=pid, targets_revision=True))[1] is None  # noqa: SLF001


def test_registering_again_at_the_same_pose_needs_the_eyes_again():
    r = rig()
    pid, rev = _confirmed(r)
    status = r.loop.real_status(r.now)
    assert status["operator_start"] is True and status["operator_start_binding"]["targets_revision"] == rev
    _, rev2 = register(r)  # a swapped pallet put on the same tape (P2-3)
    status = r.loop.real_status(r.now)
    assert status["operator_start"] is False and status["operator_start_binding"] is None
    reply = submit(r, rev2)
    assert reply["t"] == "reject" and reply["reason"].endswith("operator_start:unconfirmed"), reply


def test_the_status_shows_the_check_only_for_the_current_relocalization_generation():
    r = rig()
    _confirmed(r)
    assert r.loop.real_status(r.now)["operator_start"] is True
    r.loop.reloc_gen = 9
    assert r.loop.real_status(r.now)["operator_start"] is False


def test_moving_between_the_used_confirmation_and_the_first_arm_ends_the_mission():
    r = rig()
    pid, rev = _confirmed(r)
    assert submit(r, rev)["t"] == "ack"  # accepted, the confirmation used, preparing: the ARM is the next tick's
    assert any(e["t"] == "operator_start_used" for e in r.events) and r.loop._before_arm()  # noqa: SLF001
    r.mcu.lift = r.mcu.lift_target = 0.04  # the lift raised by hand before any ARM (P1)
    permits = []
    run_until(r, lambda rec: permits.append(r.permit.state) or r.core.commands.end_reason is not None, 400)
    assert r.core.commands.end_reason == "operator_start_moved", r.core.commands.end_reason
    r.tick(20)
    permits += [r.permit.state]
    assert set(permits) == {"OFF"}, permits  # no ARM
    assert not r.core.state.get("d5_bypass_check") and r.loop.confirmed_region() is None


# --- Codex round 2 -------------------------------------------------------------------------------------------------
def test_a_confirmation_sent_from_the_screen_before_a_same_pose_registration_is_stale():
    r = rig()
    bypass_d5(r)
    pid, rev = register(r)
    old = r.serial
    _, rev2 = register(r)  # the same pose: the same revision, a new registration
    assert rev2 == rev and r.serial == old + 1
    reply, _ = ask(r, "operator_start", pallet_id=pid, targets_revision=rev, register_serial=old)
    assert reply["t"] == "reject" and reply["reason"] == "targets_changed", reply
    reply, _ = ask(r, "operator_start", pallet_id=pid, targets_revision=rev, register_serial=r.serial)
    assert reply["t"] == "ack" and reply["register_serial"] == r.serial, reply


def test_a_run_with_depth_judges_only_the_first_submit_of_a_batch():
    r = rig(no_depth=False)
    pid = next(iter(r.core.targets.pallets))
    reply, _ = ask(r, "pallet_register", pallet_id=pid, pose=list(PALLET))
    rev = reply["targets_revision"]
    assert "register_serial" not in reply  # the D5 run's field only
    ok = SimpleNamespace(kind="submit", pallet_id=pid, targets_revision=rev)
    stray = SimpleNamespace(kind="submit", pallet_id="nope", targets_revision=rev)
    r.tick(30)
    assert r.loop._operating_submit_refusal([ok, stray], r.now) is None  # noqa: SLF001


def test_moving_with_the_first_arm_queued_but_not_written_refuses_it():
    r = rig()
    pid, rev = _confirmed(r)
    r.transport.eagain = {"ARM"}  # the first ARM meets a 0-byte EAGAIN: queued, never out (Codex round 2 P1)
    assert submit(r, rev)["t"] == "ack"
    r.tick(2)
    assert r.loop._d5_armed_for != r.core.commands.mission_id  # noqa: SLF001
    r.mcu.lift = r.mcu.lift_target = 0.04  # raised by hand while the ARM waits
    r.tick()
    r.transport.eagain = set()
    armed = []
    run_until(r, lambda rec: armed.append(r.mcu.mode) or r.core.commands.end_reason is not None, 400)
    assert r.core.commands.end_reason == "operator_start_moved", r.core.commands.end_reason
    r.tick(20)
    assert not r.core.state.get("d5_bypass_check")
    assert r.loop._d5_armed_for != "m1"  # noqa: SLF001 -- no ARM of that mission went out


def test_an_arm_written_ends_the_eyes_span():
    r = rig()
    pid, rev = _confirmed(r)
    assert submit(r, rev)["t"] == "ack"
    run_until(r, lambda rec: r.loop._d5_armed_for == r.core.commands.mission_id, 50)  # noqa: SLF001
    assert not r.loop._d5_disturbed()  # noqa: SLF001
