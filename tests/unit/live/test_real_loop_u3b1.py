"""The operating unlock U3b-1 (docs/plans/2026-11-26-real-operating-assembly.md 2.2, 2.4): case A on the operating run
-- the approved placement at the submit, the taped start at the submit and before the ARM whatever the start_pose
bypass, the adopted approach and insert forward only within the approved corridor -- and the run time: a submit with
too little time left, the insert's entry withheld, a resume inside the insert, the deadline and the stop at the
write."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.live import real_autonomy as ra
from forklift_core.live import real_case_a as rca

from real_loop_rig import MS, PALLET, Rig
from test_real_loop_console import ask, replies, run_until


# --- the pure rules -------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("pose, ok", [((3.0, 0.0, 0.0), True), ((3.09, 0.0, 0.0), True),
                                       ((3.11, 0.0, 0.0), False), ((3.0, 0.0, math.radians(5.5)), False)])
def test_the_placement_is_the_approved_one(pose, ok):
    assert (rca.placement_refusal(pose, (3.0, 0.0, 0.0)) is None) is ok
    assert rca.placement_refusal(None, (3.0, 0.0, 0.0)) == "case_a:placement_unset"


def _stage(xy, dirs=None):
    poses = np.array([[x, y, 0.0] for x, y in xy])
    return SimpleNamespace(poses=poses, directions=np.array(dirs if dirs is not None else [1] * len(xy)))


SEG = rca.corridor((0.0, 0.0, 0.0), (3.0, 0.0, 0.0), 1.0)  # start (0, 0) -> insert end (2, 0)


def test_a_straight_forward_plan_passes_and_extract_is_not_judged():
    plan = SimpleNamespace(approach=_stage([(0.0, 0.0), (1.0, 0.05)]), insert=_stage([(1.0, 0.0), (2.0, 0.0)]),
                           extract=_stage([(2.0, 0.0), (1.5, 0.0)], [-1, -1]))
    assert SEG == ((0.0, 0.0), (2.0, 0.0)) and rca.path_refusal(plan, SEG) is None


@pytest.mark.parametrize("approach, insert, why", [
    ([(0.0, 0.0), (1.0, 0.11)], [(1.0, 0.0), (2.0, 0.0)], "case_a:approach_outside_corridor"),  # an S within 15 deg
    ([(0.0, 0.0), (1.0, 0.0)], [(1.0, 0.0), (2.2, 0.0)], "case_a:insert_outside_corridor"),  # past the insert end
    (None, [(1.0, 0.0), (2.0, 0.0)], "case_a:approach_missing"),
])
def test_a_plan_off_the_corridor_or_missing_a_stage_is_refused(approach, insert, why):
    plan = SimpleNamespace(approach=None if approach is None else _stage(approach), insert=_stage(insert))
    assert rca.path_refusal(plan, SEG) == why


def test_a_reverse_piece_in_the_driven_stages_is_refused():
    plan = SimpleNamespace(approach=_stage([(0.0, 0.0), (0.5, 0.0), (0.4, 0.0)], [1, 1, -1]),
                           insert=_stage([(1.0, 0.0), (2.0, 0.0)]))
    assert rca.path_refusal(plan, SEG) == "case_a:approach_reverse"


def test_the_adoption_check_uses_the_approved_placement_not_the_plan():
    from forklift_core.live import real_run as rr
    from forklift_core.live.real_assembly import RealRequireFailed

    prep = SimpleNamespace(cfg=SimpleNamespace(start_rear=(0.0, 0.0, 0.0), check_pallet_pose=(3.0, 0.0, 0.0)),
                           geometry=SimpleNamespace(inserted_offset_m=1.0))
    started = rr._case_a_started(prep)  # noqa: SLF001
    started(0.0, {}, SimpleNamespace(approach=_stage([(0.0, 0.0), (1.0, 0.0)]),
                                     insert=_stage([(1.0, 0.0), (2.0, 0.0)])), None)
    with pytest.raises(RealRequireFailed, match="case_a:insert_outside_corridor"):
        started(0.0, {}, SimpleNamespace(approach=_stage([(0.0, 0.0), (1.0, 0.0)]),
                                         insert=_stage([(1.0, 0.0), (2.5, 0.0)])), None)
    prep.cfg.check_pallet_pose = None
    with pytest.raises(RealRequireFailed, match="placement_unset"):
        started(0.0, {}, None, None)


# --- the submit on the fake truck ------------------------------------------------------------------------------------
def rig(**kw):
    r = Rig(frames=True, real_requests=True, operating=True, **kw)
    r.start()
    r.console_out.clear()
    return r


def submit(r, pose=PALLET):
    pid = next(iter(r.core.targets.pallets))
    reply, _ = ask(r, "pallet_register", pallet_id=pid, pose=list(pose))
    assert reply["t"] == "ack", reply
    r.tick(30)
    ask(r, "operator_start")
    reply, _ = ask(r, "mission_submit", pallet_id="P-S10", destination_id="D-S10",
                   targets_revision=reply["targets_revision"])
    return reply


def test_a_pallet_off_the_approved_placement_is_refused_at_the_submit():
    r = rig()
    reply = submit(r, (PALLET[0] + 0.11, PALLET[1], PALLET[2]))
    assert reply["t"] == "reject" and reply["reason"] == "case_a:placement", reply


def test_the_start_is_checked_whatever_the_start_pose_bypass():
    r = rig()
    ask(r, "check_bypass", check="start_pose", on=True)
    r.tick(2)
    r.loop.start_rear = (r.loop.start_rear[0] - 0.2, r.loop.start_rear[1], r.loop.start_rear[2])  # truck 0.2 m off
    reply = submit(r)
    assert reply["t"] == "reject" and reply["reason"] == "case_a:not_at_start", reply


def test_too_little_run_time_refuses_the_submit():
    r = rig()
    r.loop.run_deadline_ns = r.now + int(200 * 1e9)
    reply = submit(r)
    assert reply["t"] == "reject" and reply["reason"] == "run_time_short", reply


def test_an_accepted_submit_with_time_and_placement():
    r = rig()
    r.loop.run_deadline_ns = r.now + int(1000 * 1e9)
    reply = submit(r)
    assert reply["t"] == "ack", reply


# --- the drive -----------------------------------------------------------------------------------------------------
def _driving():
    r = Rig(frames=True, operator=[(1, "confirm")], operating=True)
    r.start()
    r.tick(40)
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING and rec.branch == "auto", 3000)
    return r


def test_the_insert_entry_is_withheld_with_too_little_time():
    r = _driving()
    assert r.core.ms.phase == "approach"
    r.loop.run_deadline_ns = r.now + int(100 * 1e9)  # under the insert's 120 s, over the run's end
    for _ in range(1500):
        r.tick()
        assert r.core.ms.phase != "insert"
        if r.loop._wait_reason == "run_time_short:insert":  # noqa: SLF001
            break
    assert r.loop._wait_reason == "run_time_short:insert"  # noqa: SLF001
    r.tick(20)
    assert r.core.ms.phase == "approach"


def test_a_resume_inside_the_insert_waits_for_the_margin():
    r = _driving()
    run_until(r, lambda rec: r.core.ms.phase == "insert" and r.loop._insert_moving, 3000)  # noqa: SLF001
    since = len(r.transport.sent)
    r.loop._insert_moving = False  # noqa: SLF001 -- a zero frame was written: the next moving one is a resume
    r.loop.run_deadline_ns = r.now + int(100 * 1e9)
    r.tick(20)
    assert not any(d.left_wheel_rad_s or d.right_wheel_rad_s for d in r.loop_drives(since))


@pytest.mark.parametrize("what", ["deadline", "stop"])
def test_the_deadline_and_the_stop_refuse_every_motion_frame_at_the_write(what):
    r = _driving()
    if what == "deadline":
        r.loop.run_deadline_ns = r.now
    else:
        r.loop.run_stopping = lambda: True
    rec = run_until(r, lambda rec: rec.branch == "emergency", 20)
    assert ("run_deadline" if what == "deadline" else "run_stopping") in (rec.reason or ""), rec.reason
    r.tick(10)
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)


def test_off_the_operating_run_nothing_changes():
    r = Rig(frames=True, real_requests=True)
    r.start()
    r.console_out.clear()
    r.loop.run_deadline_ns = r.now + int(10 * 1e9)  # no effect off the operating run
    reply = submit(r, (PALLET[0] + 0.5, PALLET[1], PALLET[2]))
    assert reply["t"] == "ack", reply


# --- Codex U3b-1 round 1 ---------------------------------------------------------------------------------------------
@pytest.mark.parametrize("value", [float("nan"), float("inf"), 0, -5, True, "300", 10 ** 309])
def test_a_run_time_margin_that_is_not_a_finite_positive_number_refuses_the_run(value):
    from forklift_core.live import real_run as rr
    from forklift_core.live.real_assembly import AssemblyRefused

    with pytest.raises(AssemblyRefused, match="settings_unreadable:insert_time_min_s"):
        rr.run_time_setting({"insert_time_min_s": value}, "insert_time_min_s", 120.0)
    assert rr.run_time_setting({}, "insert_time_min_s", 120.0) == 120.0


def test_the_submit_is_judged_on_the_clock_at_the_judgement_not_the_ticks_start():
    """P2-3: with the tick's own processing time, 299.998 s left at the judgement refuses the submit."""
    r = rig()
    r.tick_cost_ns = 4 * MS
    pid = next(iter(r.core.targets.pallets))
    reply, _ = ask(r, "pallet_register", pallet_id=pid, pose=list(PALLET))
    r.tick(30)
    ask(r, "operator_start")
    r.loop.run_deadline_ns = r.now + 20 * MS + int(300 * 1e9) + 2 * MS  # the submit's tick starts 20 ms on
    rid = r.say("mission_submit", pallet_id="P-S10", destination_id="D-S10", targets_revision=reply["targets_revision"])
    r.tick()
    got, _ = replies(r)
    mine = [m for m in got if m.get("request_id") == rid]
    assert mine and mine[0]["t"] == "reject" and mine[0]["reason"] == "run_time_short", mine


def test_a_stop_set_inside_the_gate_refuses_the_write():
    """P2-2: the stop read again at the last read before the write."""
    r = _driving()
    since = len(r.transport.sent)
    flag = {"set": False}
    orig = r.loop._lease_valid  # noqa: SLF001

    def lease_valid(ns):
        flag["set"] = True  # the signal arrives while the gate judges
        return orig(ns)

    r.loop._lease_valid = lease_valid  # noqa: SLF001
    r.loop.run_stopping = lambda: flag["set"]
    r.tick(3)
    assert not any(d.left_wheel_rad_s or d.right_wheel_rad_s for d in r.loop_drives(since))


def test_a_resume_after_zero_frames_in_the_insert_waits_for_the_margin():
    """P2-1: zero frames going out inside the insert end its motion; the next moving frame is a resume."""
    r = Rig(frames=True, operator=[(1, "confirm")], operating=True, real_layer=True)
    r.start()
    r.tick(40)
    r.submit()
    run_until(r, lambda rec: r.core.ms.phase == "insert" and r.loop._insert_moving, 3000)  # noqa: SLF001
    r.loop.run_deadline_ns = r.now + int(100 * 1e9)
    r.layer.worker.held = True  # no new result: the drive waits with zero frames
    r.tick(15)
    assert not r.loop._insert_moving  # noqa: SLF001
    since = len(r.transport.sent)
    r.layer.worker.held = False
    r.layer.worker.step()
    r.tick(20)
    assert not any(d.left_wheel_rad_s or d.right_wheel_rad_s for d in r.loop_drives(since))


def test_a_zero_frame_that_never_went_out_does_not_count_as_a_stop():
    """Codex U3b-1 round 2 P2-2: a 0-byte EAGAIN on the zero frames leaves the insert moving -- the drive goes on
    without a resume wait once the wait clears."""
    r = Rig(frames=True, operator=[(1, "confirm")], operating=True, real_layer=True)
    r.start()
    r.tick(40)
    r.submit()
    run_until(r, lambda rec: r.core.ms.phase == "insert" and r.loop._insert_moving, 3000)  # noqa: SLF001
    r.loop.run_deadline_ns = r.now + int(100 * 1e9)
    # one tick of a layer overdue (a wait, not its end): this tick's frame is a zero one, and no byte of it goes out
    t = r.loop._t(r.now)  # noqa: SLF001
    r.layer._missed_expected = (t, t, r.layer._scan_id)  # noqa: SLF001
    r.layer._watch_cache = None  # noqa: SLF001
    r.transport.eagain = {"DRIVE"}
    r.tick(1)
    assert r.loop._zero_judged  # noqa: SLF001 -- a zero frame was judged this tick (the case under test)
    assert r.loop._insert_moving  # noqa: SLF001 -- nothing went out: the motion was never stopped
    r.transport.eagain = set()
    r.layer._missed_expected = None  # noqa: SLF001
    r.layer._overdue_since = None  # noqa: SLF001
    r.layer._watch_cache = None  # noqa: SLF001
    since = len(r.transport.sent)
    r.tick(20)
    assert any(d.left_wheel_rad_s or d.right_wheel_rad_s for d in r.loop_drives(since))
