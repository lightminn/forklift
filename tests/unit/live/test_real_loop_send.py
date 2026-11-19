"""docs/design/2026-11-19-real-loop.md 6 (송신·출력 복원·제출 거부·수거): the loop's emergency paths and output record on
the fake truck."""

from forklift_core.live import real_autonomy as ra
from forklift_core.mcu_link.messages import Arm, Disarm, Drive

from real_loop_rig import Rig


def run_until(r, cond, limit=4000):
    for _ in range(limit):
        rec = r.tick()
        if cond(rec):
            return rec
    raise AssertionError(f"not reached: {r.loop.records[-1]}")


def driving():
    r = Rig(frames=True)
    r.start()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING and r.loop.book.confirmed is not None
              and r.loop.book.confirmed.speed_mps != 0.0)
    return r


def moving_after(r, k):
    return [m for _, _, _, m in r.transport.sent[k:] if isinstance(m, Drive) and m.left_wheel_rad_s]


def test_a_console_disarm_in_auto_is_the_emergency_branch_the_same_tick():
    r = driving()
    k = len(r.transport.sent)
    r.say("disarm")
    rec = r.tick()
    assert (rec.branch, rec.reason) == ("emergency", "console_disarm")
    assert any(isinstance(m, Disarm) for _, _, _, m in r.transport.sent[k:]) and not moving_after(r, k)
    assert r.permit.state == ra.LATCHED and r.arbiter.mode == "STOPPED"


def test_an_exception_before_the_output_is_the_emergency_branch():
    r = driving()
    k = len(r.transport.sent)

    def boom(*a, **kw):
        raise RuntimeError("hook")

    r.core.assemble_command = boom
    rec = r.tick()
    assert rec.branch == "emergency" and rec.reason.startswith("exception:RuntimeError")
    assert any(isinstance(m, Disarm) for _, _, _, m in r.transport.sent[k:]) and not moving_after(r, k)
    assert r.permit.state == ra.LATCHED


def test_an_assembly_exception_leaves_the_confirmed_target_as_applied_never_the_candidate():
    r = driving()
    real = r.core.assemble_command

    def half(*a, **kw):
        real(*a, **kw)
        r.core.state["applied_command"] = (9.9, [0.3])  # the candidate written over, then the failure
        r.core.state["command_speed"] = 9.9
        raise RuntimeError("after the write")

    r.core.assemble_command = half
    r.tick()
    applied = r.core.state["applied_command"][0]
    assert applied != 9.9 and r.core.state["command_speed"] != 9.9
    c = r.loop.book.confirmed  # this tick's confirmed target (a STATE in its head may have named a newer frame)
    assert applied == (0.0 if c is None else c.speed_mps)


def test_a_refused_submit_gets_no_ack_no_plan_request_and_no_arm():
    r = Rig(frames=True)
    r.start()
    r.loop.evidence = _with_slam_lost(r.loop.evidence)
    r.submit()
    r.tick(40)
    assert r.core.commands.status not in ("accepted", "preparing", "running")
    assert not r.jobs.submitted
    assert not any(isinstance(m, Arm) for _, _, _, m in r.transport.sent)


def _with_slam_lost(ev):
    from dataclasses import replace

    return replace(ev, slam=lambda n: ra.SlamView("lost", None, 0.0, 0))


def test_a_mission_ended_on_a_stale_slam_is_collected_once_and_its_release_deferred():
    """The tick before the end stood (stop_now), this tick's SLAM ran out and a STOP came: one collection, no
    release (the forks are freed only after a fresh stand at a new pose)."""
    r = driving()
    seen = {}
    real_end = r.core.end_reset

    def spy(t, release_allowed=None):
        seen.setdefault("calls", []).append(release_allowed)
        return real_end(t, release_allowed=release_allowed)

    r.core.end_reset = spy
    r.core.slam["stop_now"] = True
    r.loop.evidence = _with_slam_lost(r.loop.evidence)
    r.say("stop")
    r.tick(3)
    assert seen.get("calls") == [False]
    assert len(r.loop.session.results) == 1


def test_a_snapshot_wait_repeats_one_hold_frame_on_the_same_generation_then_auto_resumes():
    """Design 2.4 / 6 (대기 프레임): while a layer snapshot installs, the loop sends the wait frame (wheels 0 at the
    wait's target) on the generation its first frame opened -- no further generation change -- and at most one hold
    unsent; when the wait ends the normal AUTO DRIVE comes back."""
    from dataclasses import replace

    r = driving()
    ev = r.loop.evidence
    r.loop.evidence = replace(ev, snapshot=lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), n, True, 0, 0))
    k = len(r.transport.sent)
    rec = r.tick()
    assert (rec.branch, rec.reason) == ("wait", "wait:snapshot_installing")
    gen = r.port.control_gen
    recs = [r.tick() for _ in range(10)]
    assert all(x.branch == "wait" for x in recs) and r.port.control_gen == gen
    assert sum(o.kind == "DRIVE" for o in r.host._outbox) <= 1  # noqa: SLF001 -- the newest unsent hold only
    drives = [m for _, _, _, m in r.transport.sent[k:] if isinstance(m, Drive)]
    assert len(drives) >= 10 and all(m.left_wheel_rad_s == 0.0 == m.right_wheel_rad_s for m in drives)
    assert len({m.lift_m for m in drives}) == 1
    assert r.permit.state in (ra.READY, ra.DRIVING)  # a wait only closes the output
    r.loop.evidence = ev
    rec = run_until(r, lambda rec: rec.branch == "auto", limit=50)
    assert r.loop.waiting is None
