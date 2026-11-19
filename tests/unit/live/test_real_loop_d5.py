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


# --- impl round 1 ---------------------------------------------------------------------------------------------------
def test_the_bypass_check_is_installed_only_once_standing():
    r = bypass_rig(real_layer=True)
    stood = []
    orig = r.core.hooks.build_bypass_pocket_check

    def build(axis, pickup):
        stood.append(bool(r.core.slam["stop_now"]))
        return orig(axis, pickup)

    r.core.hooks.build_bypass_pocket_check = build
    submit(r)
    run_until(r, lambda rec: stood, 9000)
    assert stood == [True]


def test_the_bypass_runs_through_the_s10_stop_on_the_real_layer():
    r = bypass_rig(real_layer=True)
    written = written_speeds(r)
    submit(r)
    run_until(r, lambda rec: r.core.operator_open() is not None or r.permit.state == ra.LATCHED, 12000)
    op = r.core.operator_open()
    assert op is not None, r.core.commands.end_reason
    reply, _ = ask(r, "operator_answer", kind=op["kind"], serial=op["serial"], verdict="confirm")
    assert reply["t"] == "ack"
    run_until(r, lambda rec: r.permit.state == ra.LATCHED and r.loop.session.results, 4000)
    assert [e["reason"] for e in r.loop.session.results] == ["s10_lifted"]
    moving = [abs(v) for _, v in written if v]
    assert moving and max(moving) <= 0.06 + 1e-9
    assert r.core.obstacle["layer"].exempt is None


def test_an_obstacle_beside_the_truck_outside_the_region_stops_the_bypassed_approach():
    """In this rig the truck's front is already inside the region at the stand, so an outside obstacle in its way is at
    its body and blocks both ways; the requested-direction-only case is the core test below."""
    r = bypass_rig(real_layer=True)
    written = written_speeds(r)
    submit(r)
    run_until(r, lambda rec: r.core.state.get("d5_bypass_check"), 9000)
    x, y, yaw = r.loop.odom_pose
    region = r.core.state["d5_bypass_check"][-1]["region"]
    gap_x = (x + (region[0] - region[2] / 2)) / 2  # between the truck and the region's near edge
    r.extra_boxes = ((gap_x, y, 0.05, 0.3),)
    r.tick(30)  # new scans in
    n = len([v for _, v in written if v and v > 0])
    r.tick(60)
    assert len([v for _, v in written if v and v > 0]) == n  # no forward DRIVE into the outside obstacle


def test_a_layer_answer_below_the_possible_speed_holds_under_the_bypass():
    from types import SimpleNamespace

    from forklift_core.live.real_assembly import BypassPocketCheck
    from forklift_core.safety.truth_access import SafetySpeed

    from test_core_truth_split import _Layer, permission_core

    class Low(_Layer):
        def limit(self, t, *, current_pose, curvature_inv_m, direction, loaded, cap_mps, evidence_s=None):
            self.calls.append((direction, cap_mps, evidence_s))
            return (0.01, "ok") if direction > 0 else (0.5, "ok")  # 0 < allowed < the possible 0.05 forward

    check = BypassPocketCheck((0.0, 0.0), 1.8, 0.8, 0.0)
    c, _ = permission_core(SafetySpeed(0.05, 0.05, (1,)), phase="insert", pocket_check=check)
    c.obstacle["layer"] = Low()
    out = c.permission(1.0, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert out.allowed == 0.0 and out.requested_speed == 0.0 and c.state["motion_hold"] == "residual_over_allowed"
    assert all(ev == pytest.approx(1.0 + MissionCore.WRITE_MARGIN_S) for _, _, ev in c.obstacle["layer"].calls)


def test_the_layer_gets_the_evidence_instant_and_keeps_its_deadlines_at_now():
    r = Rig(frames=True, real_layer=True)
    r.start()
    r.tick(20)
    lay = r.layer
    seen = {}
    orig_allowed, orig_watch = lay.permission.allowed_speed, lay.watchdog

    def allowed(now_s, *a, **k):
        seen["allowed"] = now_s
        return orig_allowed(now_s, *a, **k)

    def watch(now_s):
        seen["watch"] = now_s
        return orig_watch(now_s)

    lay.permission.allowed_speed, lay.watchdog = allowed, watch
    t = r.loop._t(r.now)
    lay.limit(t, current_pose=(0.0, 0.0, 0.0), curvature_inv_m=0.0, direction=1, loaded=False, cap_mps=0.1,
              evidence_s=t + 0.035)
    assert seen == {"allowed": pytest.approx(t + 0.035), "watch": pytest.approx(t)}


def test_a_continuation_past_its_decision_plus_w_is_link_stuck():
    r = Rig(frames=True)
    orig = r.loop._book

    def book(now_ns, seq, speed, *a, **k):
        if speed != 0.0 and k.get("steer") is not None and not getattr(r, "_st", False) and r.permit.state == ra.DRIVING:
            r._st = True
            r.transport.stall = True  # a moving DRIVE gets only its first bytes out
        return orig(now_ns, seq, speed, *a, **k)

    r.loop._book = book
    r.start()
    r.submit()
    run_until(r, lambda rec: r.transport.partial_frame is not None and getattr(r, "_st", False), 8000)
    r.transport.stall = False
    r.tick_cost_ns = 16 * 1_000_000  # the next tick (+20 ms) continues at +36 ms of the frame's decision
    rec = r.tick()
    r.tick_cost_ns = 0
    assert "link_stuck" in (rec.reason or "") and r.transport.resyncs == 1


# --- impl round 2 ---------------------------------------------------------------------------------------------------
def test_the_permission_is_asked_with_no_request_under_a_pocket_check():
    from forklift_core.safety.truth_access import SafetySpeed

    r = Rig(frames=True)
    loop, core = r.loop, r.core
    core.pocket["check"] = DirCheck()
    core.state.pop("outstanding_motion", None)
    loop.speed = SafetySpeed(0.0, 0.0, (1,), valid=True)
    assert loop._permission_needed(0.0) is False  # nothing may move
    core.state["outstanding_motion"] = {1: 0.03}  # an unconfirmed output only
    assert loop._permission_needed(0.0) is True
    core.state.pop("outstanding_motion")
    loop.speed = SafetySpeed(0.0, 0.02, (1, -1), valid=True)  # the encoder's bound only, below 0.05 m/s
    assert loop._permission_needed(0.0) is True
    core.pocket["check"] = None
    assert loop._permission_needed(0.0) is False  # 4c's rule outside a pocket check


def test_layer_evidence_expiring_between_t_and_t_plus_w_is_refused_at_the_evidence_instant():
    """Mid-approach, the scans stop (the deadlines kept quiet so the evidence's own age is what is judged): there is
    a tick where the layer allows at t and refuses at t + W -- the real permission asks at t + W."""
    r = Rig(frames=True, real_layer=True)
    r.start()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING and r.core.ms.phase == "approach", 6000)
    r.tick(20)
    r.scan_on = False
    lay = r.layer
    lay.watchdog = lambda now_s: None
    found = None
    for _ in range(60):
        t = r.loop._t(r.now)
        kw = dict(current_pose=tuple(r.loop.odom_pose), curvature_inv_m=0.0, direction=1, loaded=False, cap_mps=0.06)
        at_t = lay.limit(t, **kw)
        at_w = lay.limit(t, evidence_s=t + MissionCore.WRITE_MARGIN_S, **kw)
        if at_t[0] > 0.0 and at_w[0] == 0.0:
            found = (at_t, at_w)
            break
        r.tick()
    assert found is not None, "no instant with the evidence valid at t and not at t + W"


def test_voxels_expiring_between_t_and_t_plus_w_hold_that_tick():
    class Expiring(DirCheck):
        until = math.inf

        def limit(self, now_s, rear, kappa, direction, speed, lift):
            self.calls.append((float(now_s), int(direction), float(speed)))
            return (math.inf, "ok") if now_s < self.until else (0.0, "pocket_unverified")

    check = Expiring()
    r = Rig(frames=True, check=check)
    r.start()
    written = written_speeds(r)
    r.submit()
    # mid-insert, driving: the last query was this tick's, forward at a speed well above zero
    run_until(r, lambda rec: r.core.ms.phase == "insert" and check.calls
              and check.calls[-1][0] == pytest.approx(r.loop._t(r.now) + MissionCore.WRITE_MARGIN_S)
              and check.calls[-1][2] > 0.03, 8000)
    t_next = r.loop._t(r.now + r.step_ns)
    check.until = t_next + MissionCore.WRITE_MARGIN_S / 2  # valid at the next tick's t, expired at its t + W
    n = len([v for _, v in written if v])
    rec = r.tick()
    assert rec.branch == "wait" and "pocket_unverified" in (rec.reason or "")
    assert len([v for _, v in written if v]) == n


def test_only_the_requested_direction_blocked_outside_the_region_holds_under_the_bypass():
    from forklift_core.live.real_assembly import BypassPocketCheck
    from forklift_core.safety.truth_access import SafetySpeed

    from test_core_truth_split import permission_core

    check = BypassPocketCheck((0.0, 0.0), 1.8, 0.8, 0.0)
    c, layer = permission_core(SafetySpeed(0.0, 0.02, (1, -1)), phase="insert", pocket_check=check, blocked=1)
    out = c.permission(1.0, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert out.requested_speed == 0.0 and out.allowed == 0.0 and out.why == "occupied"
    back = layer.limit(1.0, current_pose=(0.0, 0.0, 0.0), curvature_inv_m=0.0, direction=-1, loaded=False,
                       cap_mps=0.02)
    assert back[0] >= 0.02  # the other way's possible motion is allowed: only the requested direction blocks


# --- impl round 3 ---------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("kind", ["outstanding", "bound"])
def test_a_standing_tick_with_no_request_calls_the_permission_under_a_pocket_check(kind):
    from forklift_core.safety.truth_access import SafetySpeed

    check = DirCheck()
    r = Rig(frames=True, check=check)
    r.start()
    r.submit()
    run_until(r, lambda rec: r.core.ms.phase == "insert" and check.calls, 9000)
    asked = []
    orig_perm, orig_ctl, orig_cmd = r.core.permission, r.loop._control_and_output, r.core.command

    def command(*a, **k):
        out = orig_cmd(*a, **k)
        out.requested_speed = 0.0  # the follower asks nothing this tick
        return out

    r.core.command = command

    def perm(t, rear, kappa, requested_speed, loaded, cusp):
        asked.append(requested_speed)
        return orig_perm(t, rear, kappa, requested_speed, loaded, cusp)

    def ctl(now_ns, t, rec):
        if kind == "outstanding":
            r.loop.speed = SafetySpeed(0.0, 0.0, (1,), valid=True)
            r.core.state["outstanding_motion"] = {1: 0.03}  # an unconfirmed output only
        else:
            r.loop.speed = SafetySpeed(0.0, 0.02, (1, -1), valid=True)  # the encoder's bound only
            r.core.state.pop("outstanding_motion", None)
        return orig_ctl(now_ns, t, rec)

    r.core.permission, r.loop._control_and_output = perm, ctl
    r.tick()
    assert asked == [0.0]


def test_free_cells_expiring_between_t_and_t_plus_w_are_evidence_stale():
    from dataclasses import replace

    r = Rig(frames=True, real_layer=True)
    r.start()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING and r.core.ms.phase == "approach", 6000)
    r.tick(10)
    lay, t = r.layer, r.loop._t(r.now)
    kw = dict(current_pose=tuple(r.loop.odom_pose), curvature_inv_m=0.0, direction=1, loaded=False, cap_mps=0.06)
    first = lay.limit(t, **kw)
    assert first[0] > 0.0  # scans fresh, evidence fresh
    perm = lay.permission
    oldest = min(perm.last_estop.oldest_free_s, perm.path_check.oldest_free_s)
    # the FREE evidence's life set to expire 10 ms after t -- the scans stay valid
    perm.config = replace(perm.config, evidence_max_age_s=(t - oldest) + 0.010)
    assert lay.limit(t, **kw)[0] > 0.0
    assert lay.limit(t, evidence_s=t + MissionCore.WRITE_MARGIN_S, **kw) == (0.0, "evidence_stale")


def test_the_layers_zero_on_the_requested_direction_reaches_the_request_under_the_bypass():
    """Only a rearward motion is possible and the request is forward: no hold -- the layer's forward 0 must reach the
    request through the pocket branch's acting."""
    from forklift_core.live.real_assembly import BypassPocketCheck
    from forklift_core.safety.truth_access import SafetySpeed

    from test_core_truth_split import permission_core

    check = BypassPocketCheck((0.0, 0.0), 1.8, 0.8, 0.0)
    c, _ = permission_core(SafetySpeed(-0.02, 0.02, (-1,)), phase="insert", pocket_check=check, blocked=1)
    out = c.permission(1.0, (0.0, 0.0, 0.0), 0.0, 0.04, False, None)
    assert out.acting and out.requested_speed == 0.0 and out.why == "occupied"
    assert "motion_hold" not in c.state  # not a hold: the request itself is cut


# --- impl round 4 ---------------------------------------------------------------------------------------------------
def test_a_positive_d5_answer_below_the_requested_directions_possible_speed_holds():
    """Design 7.3: encoder bound 0.07, unconfirmed output 0.03, D5 allows 0.037 m/s forward -- a hold, not a request
    capped to 0.037 (a low positive target cannot guarantee the possible motion's stop)."""
    from types import SimpleNamespace

    from forklift_core.safety.truth_access import SafetySpeed

    from test_core_truth_split import permission_core

    check = SimpleNamespace(limit=lambda *a: (0.037, "ok") if a[3] > 0 else (math.inf, "ok"), summary=lambda: {})
    c, _ = permission_core(SafetySpeed(0.07, 0.07, (1,)), phase="insert", pocket_check=check)
    c.state["outstanding_motion"] = {1: 0.03}
    out = c.permission(1.0, (0.0, 0.0, 0.0), 0.0, 0.05, False, None)
    assert out.allowed == 0.0 and out.requested_speed == 0.0
    assert out.why == "residual_over_allowed" and c.state["motion_hold"] == "residual_over_allowed"


def test_fresh_frames_without_a_check_builder_end_at_the_approach_without_the_bypass():
    r = Rig(frames=True, real_requests=True, real_layer=True, check_builder=False)  # no PocketCheck builder (PR 7)
    r.start()
    phases = []
    submit(r)
    for _ in range(9000):
        rec = r.tick()
        phases.append(r.core.ms.phase)
        if r.core.commands.status in ("cancelled", "failed", "recovery_required", "done"):
            break
    assert "pocket_check_unavailable" in str(r.core.commands.end_reason)
    assert "insert" not in phases and r.core.obstacle["layer"].exempt is None
