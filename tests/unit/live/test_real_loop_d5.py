"""docs/design/2026-11-19-real-safety-wiring.md 7 (stage 5d) on the fake truck: the D5 query over every possible
direction (the real encoder model leaves both after a stand), the query at the write margin, the transmit gate's
write bound and stop-tick continuation, and the operator's D5 bypass (a bypass check in the pocket check's place)."""

import math

import pytest

from forklift_core.live import real_autonomy as ra
from forklift_core.live import real_loop as rl
from forklift_core.mission.core import MissionCore

from real_loop_rig import PALLET, DirCheck, Rig
from test_real_loop_console import ask, replies, run_until


def written_speeds(r):
    """A spy on the loop's book: (core time, booked speed) of every DRIVE written whole."""
    booked, written = {}, []
    ob, ow = r.loop._book, r.loop.book.written

    def book(now_ns, seq, speed, *a, **k):
        booked[int(seq) & 0xFFFF] = float(speed)
        return ob(now_ns, seq, speed, *a, **k)

    def wrote(seq, written_ns):
        written.append((r.loop._t(r.now), booked.get(int(seq) & 0xFFFF)))
        return ow(seq, written_ns)

    r.loop._book, r.loop.book.written = book, wrote
    return written


def test_the_write_margin_is_the_gates_latest_write():
    assert MissionCore.WRITE_MARGIN_S == pytest.approx(rl.SLOW_TICK_S + rl.EVIDENCE_READ_MAX_S)
    assert rl.WRITE_MARGIN_S == pytest.approx(MissionCore.WRITE_MARGIN_S)


# --- 7.1 ------------------------------------------------------------------------------------------------------------
def test_both_directions_clear_the_restart_goes_on_and_both_are_asked():
    check = DirCheck()
    r = Rig(frames=True, direction_evidence=False, check=check)
    r.start()
    written = written_speeds(r)
    r.submit()
    run_until(r, lambda rec: r.core.ms.phase in ("insert", "lift") or r.permit.state == ra.LATCHED, 6000)
    assert r.core.ms.phase in ("insert", "lift"), r.core.commands.end_reason
    assert {d for _, d, _ in check.calls} == {1, -1}  # the encoder model's both directions were asked
    assert any(v for _, v in written)


def test_the_other_direction_short_holds():
    check = DirCheck()
    r = Rig(frames=True, direction_evidence=False, check=check)
    r.start()
    r.submit()
    run_until(r, lambda rec: r.core.pocket.get("check") is check and not r.core.pocket.get("await_still_frame"), 6000)
    check.allow[-1] = 0.0  # backwards: not even the encoder's bound is allowed
    check.reason[-1] = "pocket_unverified"
    recs = [r.tick() for _ in range(40)]
    assert any(rec.branch == "wait" and "pocket_unverified" in (rec.reason or "") for rec in recs)


def test_the_still_frame_wait_asks_nothing():
    check = DirCheck()
    r = Rig(frames=True, direction_evidence=False, check=check)
    r.start()
    r.submit()
    run_until(r, lambda rec: r.core.pocket.get("check") is check, 6000)
    r.core.pocket["await_still_frame"] = True
    r.frames = None  # no frame releases it
    n = len(check.calls)
    r.tick(10)
    assert len(check.calls) == n  # the wait gives 0 before any query


def test_the_queries_go_at_the_write_margin():
    check = DirCheck()
    r = Rig(frames=True, direction_evidence=False, check=check)
    r.start()
    r.submit()
    run_until(r, lambda rec: len(check.calls) > 0, 6000)
    t_tick = r.loop._t(r.now)
    assert check.calls[-1][0] == pytest.approx(t_tick + MissionCore.WRITE_MARGIN_S)


def test_the_layer_judges_evidence_at_the_evidence_time_and_deadlines_at_now():
    r = Rig(frames=True, real_layer=True)
    r.start()
    r.tick(20)
    lay = r.layer
    t = r.loop._t(r.now)
    newest = lay._installed["newest_scan_s"]  # noqa: SLF001
    later = newest + lay.scan_max_age_s + 0.01
    a, why = lay.limit(t, current_pose=(0.0, 0.0, 0.0), curvature_inv_m=0.0, direction=1, loaded=False,
                       cap_mps=0.1, evidence_s=later)
    assert (a, why) == (0.0, "scan_stale")  # the evidence judged at its own instant
    assert lay.watchdog(t) in (None, "layer_overdue")  # no state advanced past now


# --- the transmit gate (7.1 6) -------------------------------------------------------------------------------------
def test_a_write_later_than_the_bound_is_held():
    r = Rig(frames=True)
    r.start()
    written = written_speeds(r)
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING and any(v for _, v in written), 6000)
    n = len([v for _, v in written if v])

    class LateAfterAuthority(dict):
        """The frame's authority binding is read after the lease and the readiness: the clock moves 6 ms there --
        past ns (start + 5 ms), well inside SLOW_TICK_S."""

        def get(self, key, default=None):
            r.tick_cost_ns = 6 * 1_000_000
            return super().get(key, default)

    r.loop._binding = LateAfterAuthority(r.loop._binding)
    rec = r.tick()
    r.tick_cost_ns = 0
    assert rec.branch == "wait" and rec.reason == "write_late"
    assert len([v for _, v in written if v]) == n  # no moving DRIVE written past min(ns, decision + W)


def test_a_tick_that_stops_does_not_finish_an_older_moving_frame():
    check = DirCheck()
    r = Rig(frames=True, check=check)
    orig = r.loop._book

    def book(now_ns, seq, speed, *a, **k):
        if speed != 0.0 and k.get("steer") is not None and r.core.ms.phase == "insert" and not getattr(r, "_st", False):
            r._st = True
            r.transport.stall = True  # an insert DRIVE gets only its first bytes out
        return orig(now_ns, seq, speed, *a, **k)

    r.loop._book = book
    r.start()
    r.submit()
    run_until(r, lambda rec: r.transport.partial_frame is not None and getattr(r, "_st", False), 8000)
    check.allow[1] = 0.0  # the next tick's D5 refuses: a hold
    check.reason[1] = "pocket_obstacle"
    r.transport.stall = False
    rec = run_until(r, lambda rec: rec.branch == "emergency" or r.transport.resyncs, 30)
    assert r.transport.resyncs == 1 and "link_stuck" in (rec.reason or "")  # abandoned, not finished


# --- 7.2 the D5 bypass ----------------------------------------------------------------------------------------------
def bypass_rig(**kw):
    r = Rig(real_requests=True, **kw)
    r.start()
    r.console_out.clear()
    reply, _ = ask(r, "check_bypass", check="d5_pocket", on=True)
    assert reply["t"] == "ack", reply
    r.tick()
    return r


def submit(r):
    pid = next(iter(r.core.targets.pallets))
    reply, _ = ask(r, "pallet_register", pallet_id=pid, pose=list(PALLET))
    r.tick(30)
    ok, _ = ask(r, "operator_start")
    assert ok["t"] == "ack", ok
    r.say("mission_submit", pallet_id="P-S10", destination_id="D-S10", targets_revision=reply["targets_revision"])


def test_without_frames_and_without_the_bypass_the_first_frame_ends_it():
    r = Rig(real_requests=True)
    r.start()
    submit(r)
    run_until(r, lambda rec: r.core.commands.status in ("cancelled", "failed", "recovery_required", "done"), 8000)
    assert "pocket_first_frame" in str(r.core.commands.end_reason)


def test_the_bypass_runs_s10_without_depth_capped_and_marked():
    r = bypass_rig(operator=[(1, "confirm")])
    written = written_speeds(r)
    asked = []
    od = r.loop._auto_drive

    def spy(now_ns, command_, steer, rec_):
        asked.append(abs(float(command_.drive.speed_mps)))
        return od(now_ns, command_, steer, rec_)

    r.loop._auto_drive = spy
    submit(r)
    run_until(r, lambda rec: r.core.ms.phase == "lift" or r.permit.state == ra.LATCHED, 9000)
    assert r.core.ms.phase == "lift" or "s10" in str(r.core.commands.end_reason), r.core.commands.end_reason
    first = r.core.state["pocket_check"]["first_frame"]
    assert first["source"] == "d5_bypass" and r.core.state["d5_bypass_check"]
    moving = [abs(v) for _, v in written if v]
    assert moving and max(moving) <= 0.06 + 1e-9 and max(asked) > 0.06
    assert "d5_pocket" in r.loop.real_status(r.now)["bypassed"]
    run_until(r, lambda rec: r.core.ms.phase != "insert" and r.core.pocket.get("check") is None, 2000)
    assert r.core.obstacle["layer"].exempt is None  # cleared with the insert's end


def test_the_bypass_region_is_the_submitted_pose():
    r = bypass_rig()
    pid = next(iter(r.core.targets.pallets))
    pose = [PALLET[0] + 0.05, PALLET[1] - 0.03, PALLET[2]]
    reply, _ = ask(r, "pallet_register", pallet_id=pid, pose=pose)
    r.tick(30)
    ask(r, "operator_start")
    r.say("mission_submit", pallet_id="P-S10", destination_id="D-S10", targets_revision=reply["targets_revision"])
    run_until(r, lambda rec: r.core.state.get("d5_bypass_check"), 8000)
    region = r.core.state["d5_bypass_check"][-1]["region"]
    assert region[0] == pytest.approx(pose[0]) and region[1] == pytest.approx(pose[1])


def test_a_failure_right_after_the_exemption_leaves_no_exemption_after_the_collection():
    r = bypass_rig()

    class Boom(dict):
        def setdefault(self, key, default=None):
            if key == "d5_bypass_check":
                raise RuntimeError("injected after the exemption was set")
            return super().setdefault(key, default)

    r.core.state = Boom(r.core.state)
    submit(r)
    run_until(r, lambda rec: r.core.commands.status in ("cancelled", "failed", "recovery_required", "done")
              or rec.branch == "emergency", 8000)
    r.tick(5)
    assert r.core.obstacle["layer"].exempt is None


def test_the_bypass_never_reads_the_depth_source():
    r = bypass_rig()

    class Raising:
        def frames(self, now_ns):
            raise RuntimeError("the depth source must not be read under the bypass")

    r.loop.depth_source = Raising()
    submit(r)
    run_until(r, lambda rec: r.core.state.get("d5_bypass_check"), 8000)
    r.tick(50)
    assert not any("depth source" in (rec.reason or "") for rec in r.loop.records)


def test_the_bypass_cannot_change_during_a_mission():
    r = bypass_rig()
    submit(r)
    run_until(r, lambda rec: r.core.commands.status in ("preparing", "running"), 2000)
    reply, _ = ask(r, "check_bypass", check="d5_pocket", on=False)
    assert reply["t"] == "reject"
