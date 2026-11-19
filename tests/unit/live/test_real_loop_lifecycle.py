"""Codex 4c impl round 1 (P1-4, P2-3): a mission the head ends is collected before the console's apply; the
operator's start check approves one submit; no ARM while a wait holds."""

from dataclasses import replace

from forklift_core.live import real_assembly as asm
from forklift_core.live import real_autonomy as ra
from forklift_core.mcu_link.messages import Arm

from real_loop_rig import EPOCH, Rig


def run_until(r, cond, limit=4000):
    for _ in range(limit):
        rec = r.tick()
        if cond(rec):
            return rec
    raise AssertionError(f"not reached: {r.loop.records[-1]}")


def test_a_reboot_in_the_lift_is_collected_once_with_its_field_recovery():
    r = Rig(frames=True, operator=[])  # no answer: the truck waits at the carry height
    r.start()
    r.submit()
    run_until(r, lambda rec: r.core.ms.phase == "lift")
    assert r.core.cargo.pocket_entered
    r.mcu.boot = 0x7373
    r.tick(5)
    assert [e["outcome"] for e in r.loop.session.results] == ["recovery_required"]
    assert r.core.field_recovery is not None and r.permit.state in (ra.DISARMING, ra.LATCHED)


def test_the_operators_start_check_approves_one_submit_only():
    r = Rig(frames=True)
    start = asm.OperatorStart(EPOCH)
    r.loop.operator_start = start
    r.start()
    lease = r.arbiter.lease.lease_id
    r.loop.evidence = replace(r.loop.evidence, operator_start_ok=lambda n: start.ok(r.arbiter.lease.lease_id))
    assert start.confirm(EPOCH, lease)
    r.submit()
    run_until(r, lambda rec: r.core.commands.status in ("preparing", "running"), limit=200)
    assert not start.ok(lease)  # used up by the submit it approved
    r.say("mission_cancel", mission_id=r.core.commands.mission_id)
    run_until(r, lambda rec: r.core.commands.status in ("cancelled", "idle"), limit=200)
    r.submit()
    r.tick(20)
    assert r.core.commands.status in ("cancelled", "idle")  # the second submit has no check of its own


def test_no_arm_while_a_wait_holds_at_the_adoption():
    r = Rig(frames=True)
    r.start()
    r.submit()
    run_until(r, lambda rec: r.core.commands.status == "preparing", limit=100)
    ev = r.loop.evidence
    r.loop.evidence = replace(ev, snapshot=lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), n, True, 0, 0))
    r.tick(150)
    assert r.core.commands.status == "running" and r.permit.state == ra.OFF
    assert not any(isinstance(m, Arm) for _, _, _, m in r.transport.sent)
    r.loop.evidence = ev
    run_until(r, lambda rec: r.permit.state in (ra.ARMING, ra.READY, ra.DRIVING), limit=50)
    assert sum(isinstance(m, Arm) for _, _, _, m in r.transport.sent) == 1
