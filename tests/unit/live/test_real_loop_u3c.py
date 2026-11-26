"""The operating unlock U3c (docs/plans/2026-11-26-real-operating-assembly.md 2.9): real_status.preconditions on the
operating run -- the assembly's passed items and the run-time rows (C7 IMU, C8 D5 and, without depth, the per-pallet
confirmation, the scan source's newest refusal), each pass / blocked / bypassed (the scan row a note)."""

from forklift_core.live.real_preconditions import ASSEMBLY_CHECKS

import dataclasses

import pytest

from real_loop_rig import PALLET, Rig
from test_real_loop_console import ask
from test_real_loop_u3b2 import REGION, _confirmed, bypass_d5, register, rig, still


def rows(r):
    pre = r.loop.real_status(r.now)["preconditions"]
    return {(x["id"], x["item"]): (x["state"], x["why"]) for x in pre["runtime"]}


def test_only_the_operating_run_has_precondition_rows():
    r = Rig(real_requests=True)
    r.start()
    assert r.loop.real_status(r.now)["preconditions"] is None  # HIL: nothing the operating run judges


def test_without_depth_the_d5_rows_follow_the_bypass_and_the_confirmation():
    r = rig()
    assert rows(r) == {("C7", "imu"): ("pass", None),
                       ("C8", "d5_pocket"): ("blocked", "d5_pocket:no_depth_source"),
                       ("C8", "pallet_confirm"): ("blocked", "operator_start:unconfirmed"),
                       ("C5", "scan"): ("note", None)}
    _confirmed(r)  # the D5 bypass, a registration and its confirmation
    got = rows(r)
    assert got[("C8", "d5_pocket")] == ("bypassed", None) and got[("C8", "pallet_confirm")] == ("pass", None)
    register(r)  # registering again voids it
    assert rows(r)[("C8", "pallet_confirm")] == ("blocked", "operator_start:unconfirmed")


def test_a_run_with_depth_has_no_confirmation_row_and_shows_its_bypass():
    r = rig(no_depth=False)
    got = rows(r)
    assert got[("C8", "d5_pocket")] == ("pass", None) and ("C8", "pallet_confirm") not in got
    reply, _ = ask(r, "check_bypass", check="d5_pocket", on=True)
    assert reply["t"] == "ack", reply
    r.tick()
    assert r.core.d5_bypassed and rows(r)[("C8", "d5_pocket")] == ("bypassed", None)  # Codex round 1 P2-3


def test_the_registration_tells_its_region_before_the_confirmation():
    """Codex round 1 P1-2: the region the confirmation binds is on screen before the eyes check it."""
    r = rig()
    assert r.loop.real_status(r.now)["register_region"] is None
    pid = next(iter(r.core.targets.pallets))
    reply, _ = ask(r, "pallet_register", pallet_id=pid, pose=list(PALLET))
    assert reply["t"] == "ack" and reply["region"] == pytest.approx(REGION), reply
    shown = r.loop.real_status(r.now)["register_region"]
    assert shown["register_serial"] == reply["register_serial"] and shown["pallet_id"] == pid
    assert shown["targets_revision"] == reply["targets_revision"] and shown["region"] == pytest.approx(REGION)
    r.tick(30)
    ok, _ = ask(r, "operator_start", pallet_id=pid, targets_revision=reply["targets_revision"],
                register_serial=reply["register_serial"], still_serial=still(r))
    assert ok["t"] == "ack" and ok["region"] == pytest.approx(shown["region"])  # the region shown is the one bound
    assert rig(no_depth=False).loop.real_status(0)["register_region"] is None  # with depth: no confirmation


def test_an_unqualified_imu_is_blocked_until_bypassed():
    r = rig(imu_unqualified=True)
    assert rows(r)[("C7", "imu")] == ("blocked", "imu:unqualified")
    reply, _ = ask(r, "check_bypass", check="imu", on=True)
    assert reply["t"] == "ack", reply
    r.tick()
    assert rows(r)[("C7", "imu")] == ("bypassed", "imu:unqualified")


def test_the_scan_row_tells_the_newest_refusal_without_blocking():
    r = rig()
    r.loop.scan_refusal = lambda: "frame:stale"
    assert rows(r)[("C5", "scan")] == ("note", "frame:stale")


def test_the_summary_and_the_load_are_the_runs():
    r = rig()
    assert r.loop.real_status(r.now)["preconditions"]["passed"] == []  # a rig the assembly did not judge
    r.loop.preconditions_passed = ASSEMBLY_CHECKS
    r.loop.operator_load = {"lift_target_m": 0.2, "test_pallet": "P-S10", "pallet_mass_kg": 4.5}
    pre = r.loop.real_status(r.now)["preconditions"]
    assert pre["passed"] == list(ASSEMBLY_CHECKS)
    assert pre["load"] == {"lift_target_m": 0.2, "test_pallet": "P-S10", "pallet_mass_kg": 4.5}


def test_a_move_after_the_ticks_voids_them():
    """Codex round 2 P1: the confirmation names the still span its six lines were ticked in; a move and a new stand
    in between (even one the console missed) refuses it."""
    r = rig()
    bypass_d5(r)
    pid, rev = register(r)
    ticked = still(r)
    assert isinstance(ticked, int)
    r.loop.still_since_ns = None  # the truck moved ...
    r.tick(30)  # ... and stands again: a new span
    assert still(r) == ticked + 1
    reply, _ = ask(r, "operator_start", pallet_id=pid, targets_revision=rev, register_serial=r.serial,
                   still_serial=ticked)
    assert reply["t"] == "reject" and reply["reason"] == "still_changed", reply
    ok, _ = ask(r, "operator_start", pallet_id=pid, targets_revision=rev, register_serial=r.serial,
                still_serial=still(r))
    assert ok["t"] == "ack", ok


def test_a_list_moved_since_the_registration_is_not_confirmed():
    """Codex round 2 P2-1: a pose correction after the registration (a new revision, no new registration) is not
    what the eyes were shown."""
    from forklift_core.live.real_assembly import register_pallet

    r = rig()
    bypass_d5(r)
    pid, rev = register(r)
    register_pallet(r.core, pid, [PALLET[0] + 0.035, PALLET[1], PALLET[2]])  # moved by the list, not the console
    rev2 = int(r.core.targets.revision)
    assert rev2 != rev
    reply, _ = ask(r, "operator_start", pallet_id=pid, targets_revision=rev2, register_serial=r.serial,
                   still_serial=still(r))
    assert reply["t"] == "reject" and reply["reason"] == "targets_changed", reply


def test_a_registration_the_targets_did_not_take_changes_nothing():
    """Codex round 2 P2-2: a pallet with no pose after the registration (carried) is refused before the serial."""
    r = rig()
    _confirmed(r)
    before = r.loop.real_status(r.now)
    pid = before["register_region"]["pallet_id"]
    r.core.targets.pallets[pid] = dataclasses.replace(r.core.targets.pallets[pid], state="carried", pose=None)
    reply, _ = ask(r, "pallet_register", pallet_id=pid, pose=list(PALLET))
    assert reply["t"] == "reject" and reply["reason"] == "pallet_not_registered", reply
    after = r.loop.real_status(r.now)
    assert after["register_serial"] == before["register_serial"]
    assert after["register_region"] == before["register_region"] and after["operator_start"] is True


def test_a_slow_lift_creep_ends_the_still_span_and_voids_its_confirmation():
    """Codex round 3 P1: each sample window steady, but the lift crept past the port's tolerance since the span
    began -- a new span (number) and the confirmation given in the old one void."""
    r = rig()
    pid, rev = _confirmed(r)
    span = still(r)
    tol = float(r.loop.port.lift_still_m)
    for _ in range(5):  # 3x the tolerance, each step inside it and a whole sample window (128) apart
        r.mcu.lift = r.mcu.lift_target = r.mcu.lift + 0.6 * tol
        r.tick(140)
    status = r.loop.real_status(r.now)
    assert status["still_serial"] is not None and status["still_serial"] > span
    assert status["operator_start"] is False and status["operator_start_binding"] is None
    reply, _ = ask(r, "operator_start", pallet_id=pid, targets_revision=rev, register_serial=r.serial,
                   still_serial=span)
    assert reply["t"] == "reject" and reply["reason"] == "still_changed", reply


def test_a_relocalization_change_ends_the_still_span():
    """Codex round 3 P2: a confirmation checked before a relocalization changed the map place is not taken after."""
    r = rig()
    _confirmed(r)
    span = still(r)
    r.loop.reloc_gen = 9  # a relocalization generation came and went
    r.tick(30)
    status = r.loop.real_status(r.now)
    assert status["still_serial"] is not None and status["still_serial"] > span
    assert status["operator_start_binding"] is None

