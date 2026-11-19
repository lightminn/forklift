"""docs/design/2026-11-19-real-safety-wiring.md 1-3 (stage 5a) on the fake truck with a real RealLayer: S10-A end to
end on scans, the D5 exemption from the check hook, the planning grid gating the plan request, the layer forced on,
the layer's faults, scans dropped for an over-bound motion, a same-boot odometry break resetting the layer."""

import math

from forklift_core.live import real_autonomy as ra
from forklift_core.mcu_link.messages import FaultFlag
from forklift_core.perception.obstacle_grid import OCCUPIED

from real_loop_rig import PALLET, Rig


def run_until(r, cond, limit=6000):
    for _ in range(limit):
        rec = r.tick()
        if cond(rec):
            return rec
    raise AssertionError(f"not reached: {r.loop.records[-1]}")


def rig(**kw):
    r = Rig(frames=True, operator=[(1, "confirm")], real_layer=True, **kw)
    r.start()
    return r


def cell(snap, x, y):
    i = int((x - snap.origin_x_m) / snap.resolution_m)
    j = int((y - snap.origin_y_m) / snap.resolution_m)
    return snap.state[i, j]


def test_s10_a_runs_end_to_end_on_the_real_layer():
    r = rig()
    r.tick(40)  # the first scans wait for a history reaching back dt before their first beam (dropped at the start)
    start_drops = r.layer.counts["scans_dropped"]
    r.submit()
    run_until(r, lambda rec: r.core.ms.phase == "approach" and r.permit.state == ra.DRIVING)
    assert cell(r.layer.snapshot, PALLET[0] - 0.3, PALLET[1]) == OCCUPIED  # the pallet is an obstacle to the drive
    assert r.layer.exempt is None
    run_until(r, lambda rec: r.core.ms.phase == "insert")
    assert r.layer.exempt is not None  # the D5 check's region, installed by the check hook
    run_until(r, lambda rec: r.permit.state == ra.LATCHED and r.loop.session.results)
    assert [e["reason"] for e in r.loop.session.results] == ["s10_lifted"]
    assert r.layer.counts["scans_dropped"] == start_drops and r.layer.counts["installs"] > 0


def test_no_plan_is_asked_before_the_planning_grid_of_the_frozen_pallet_is_installed():
    """Design 1.2: the plan request waits for the planning grid of the frozen pallet at the applied correction
    generation; when it goes out, that grid is installed (and a grid-less plan is never asked)."""
    r = rig()
    r.submit()
    run_until(r, lambda rec: r.core.commands.status in ("accepted", "preparing"), limit=50)
    assert not r.jobs.submitted  # the frozen pallet's planning grid is not installed yet on the acceptance tick
    run_until(r, lambda rec: bool(r.jobs.submitted), limit=50)
    assert r.layer.planning_grid(r.loop._planning_key()) is not None  # noqa: SLF001


def test_turning_the_obstacle_layer_off_while_running_is_the_emergency():
    r = rig()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING)
    r.core.options.obstacle_act = False
    rec = r.tick()
    assert (rec.branch, rec.reason) == ("emergency", "obstacle_layer_off")
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)


def test_a_dead_layer_worker_while_preparing_latches_and_ends_the_preparation():
    r = Rig(frames=True, real_layer=True)
    r.start()
    r.tick(5)
    r.submit()
    run_until(r, lambda rec: r.core.commands.status in ("accepted", "preparing"), limit=50)
    r.layer.worker.alive = False
    run_until(r, lambda rec: rec.branch == "emergency", limit=20)
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)
    r.tick(2)
    assert r.core.commands.status not in ("accepted", "preparing", "running")


def test_a_truck_pushed_faster_than_the_scan_bound_drops_those_scans():
    r = rig()
    r.tick(10)
    dropped = r.layer.counts["scans_dropped"]
    r.mcu.counts[0] += 4000  # both wheels jump: a push far over v_scan between two STATEs
    r.mcu.counts[1] += 4000
    r.tick(15)
    assert r.layer.counts["scans_dropped"] > dropped


def test_a_same_boot_odometry_break_resets_the_layer():
    r = rig()
    r.tick(15)
    assert r.layer.grid.scans
    r.mcu.faults = FaultFlag.ENCODER_GAP
    r.tick(1)
    r.mcu.faults = FaultFlag(0)
    assert not r.layer.grid.scans or all(s.stamp_s > r.loop._t(r.now) - 0.2 for s in r.layer.grid.scans)  # noqa: SLF001
    assert r.layer.installing()


def _driving_rig():
    r = rig()
    r.tick(40)
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING and rec.branch == "auto")
    return r


def test_a_layer_deadline_passing_between_the_decision_and_the_write_holds_the_drive():
    """Codex 5a impl P1-1: the first byte reads the layer watchdog at the write -- a scan deadline passed 5 ms after
    the tick's decision makes its DRIVE a wait (wheels 0), not the moving frame decided."""
    from forklift_core.mcu_link.messages import Drive
    from test_teleop_bridge_cycle import MS

    from dataclasses import replace

    r = _driving_rig()
    ev = r.loop.evidence
    view = ev.snapshot
    # the loop's own read at the write, not the evidence's (which an assembly may not wire the watchdog into)
    r.loop.evidence = replace(ev, snapshot=lambda n: replace(view(n), layer_wait=None))
    orig = r.layer.watchdog
    r.layer.watchdog = lambda now: "layer_overdue" if now > r.loop._t(r.now) + 0.005 else orig(now)  # noqa: SLF001
    r.tick_cost_ns = 10 * MS
    k = len(r.transport.sent)
    r.tick()
    drives = [m for _, _, _, m in r.transport.sent[k:] if isinstance(m, Drive)]
    assert drives and all(m.left_wheel_rad_s == 0.0 == m.right_wheel_rad_s for m in drives)


def test_a_same_boot_odometry_break_asks_one_relocalization():
    """Codex 5a impl P1-3: the motion across a same-boot break was not integrated -- the map pose is not trusted
    until a relocalization; one generation per break, not one per tick."""
    r = rig()
    r.tick(15)
    assert r.core.relocalize_generation() is None
    r.mcu.faults = FaultFlag.ENCODER_GAP
    r.tick(3)
    r.mcu.faults = FaultFlag(0)
    r.tick(5)
    events = [e for e in r.core.state.get("field_recoveries", []) if e.get("reason") == "odometry_break"]
    assert len(events) == 1 and r.core.relocalize_generation() == events[0]["generation"]


def test_the_scan_motion_check_uses_the_encoder_bound_and_reaches_dt_before_the_first_beam():
    """Design 1.1 / Codex 5a impl P1-4: a 0.12 m/s approach at the STATE interval passes the scan check; an entry
    over the bound 80 ms before the first beam (inside dt = 2 period + state delay) drops the scan."""
    from types import SimpleNamespace

    loop = rig().loop
    loop.delay_bound = None
    loop.odom_history.clear()
    loop._odom_prev = None  # noqa: SLF001
    p, dt = loop.params, 0.01
    counts = [0.0, 0.0]
    for k in range(5):
        st = SimpleNamespace(boot_id=1, mcu_time_us=int(k * dt * 1e6),
                             wheel_count_left=int(counts[0]), wheel_count_right=int(counts[1]))
        loop._odom_entry(st, 0, SimpleNamespace(pose=(0.0, 0.0, 0.0), continuity_broken=False))  # noqa: SLF001
        for w in (0, 1):
            counts[w] += 0.12 * dt * p.counts_per_rev[w] / (2 * math.pi * p.radius_m[w])
    assert all(e[5] and e[6] for e in list(loop.odom_history)[1:])
    # the placer's window: entries every 10 ms from 0 to 1.0 s, all fine but one 80 ms before the first beam
    loop.odom_history.clear()
    first, last = 0.60, 0.70
    for k in range(101):
        t = k * 0.01
        loop.odom_history.append((t - 0.005, t, (0.0, 0.0, 0.0), True, True, True, True))
    assert loop.place_scan(first, last) is not None
    h = list(loop.odom_history)
    k = next(i for i, e in enumerate(h) if abs(e[1] - (first - 0.08)) < 1e-9)
    h[k] = h[k][:5] + (False, True)
    loop.odom_history.clear()
    loop.odom_history.extend(h)
    assert loop.place_scan(first, last) is None


def test_the_arm_goes_out_on_the_adopted_missions_installed_key():
    """Codex 5a impl P2-1: on the adoption tick the key follows the new mission before readiness is read -- the ARM
    is asked only once the installed result is the current key's."""
    r = rig()
    r.tick(40)
    seen = []
    orig = r.permit.adopted

    def adopted(now_ns, **kw):
        seen.append((r.layer.installing(), r.layer.installed_key().mission_gen, r.core.commands.mission_gen))
        return orig(now_ns, **kw)

    r.permit.adopted = adopted
    r.submit()
    run_until(r, lambda rec: bool(seen))
    installing, installed_gen, current_gen = seen[0]
    assert not installing and installed_gen == current_gen


def test_slam_lost_while_waiting_for_the_planning_grid_latches_and_ends_the_preparation():
    """Codex 5a impl P2-2: a pre-adoption fault latches whether or not a plan is out yet."""
    r = rig()
    r.tick(5)
    r.submit()
    run_until(r, lambda rec: r.core.commands.status in ("accepted", "preparing"), limit=50)
    r.tracker.mode = "lost"
    rec = run_until(r, lambda rec: rec.branch == "emergency", limit=20)
    assert rec.reason.startswith("prepare:slam")
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)
    r.tick(2)
    assert r.core.commands.status not in ("accepted", "preparing", "running")


def test_the_planning_key_holds_the_frozen_pallets_pose():
    """Codex 5a impl P2-4: the same pallet id registered elsewhere is another planning grid."""
    from types import SimpleNamespace

    loop = rig().loop
    loop.frozen = SimpleNamespace(pallet_id="P1", pose=(3.0, 0.0, 0.0))
    a = loop._planning_key()  # noqa: SLF001
    loop.frozen = SimpleNamespace(pallet_id="P1", pose=(3.4, 0.0, 0.0))
    assert loop._planning_key() != a  # noqa: SLF001


def test_a_same_boot_break_on_a_stuck_write_tick_still_locks():
    """Codex 5a impl round 2 P1-3: the odometry's part of the head runs before the stuck-write return."""
    r = rig()
    r.tick(15)
    orig = r.loop._cut_stuck_write  # noqa: SLF001
    r.loop._cut_stuck_write = lambda now_ns: "link_stuck"  # noqa: SLF001
    r.mcu.faults = FaultFlag.ENCODER_GAP
    r.tick(1)
    r.loop._cut_stuck_write = orig  # noqa: SLF001
    r.mcu.faults = FaultFlag(0)
    r.tick(3)
    assert r.core.relocalize_generation() is not None


def test_turning_the_layer_off_before_the_adoption_ends_the_preparation():
    """Codex 5a impl round 2 P2-3."""
    r = rig()
    r.tick(5)
    r.submit()
    run_until(r, lambda rec: r.core.commands.status in ("accepted", "preparing"), limit=50)
    r.core.options.obstacle_act = False
    r.tick(2)
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)
    assert r.core.commands.status not in ("accepted", "preparing", "running")


def test_after_a_reboot_the_session_check_runs_again_and_scans_are_placed():
    """Codex 5a impl round 2 P2-4: the new boot's session check and delay anchors under the same executor."""
    r = rig()
    r.tick(40)
    r.mcu.boot = 0x6262
    r.tick(1)
    assert not r.permit.session_started
    installs = r.layer.counts["installs"]
    r.tick(160)
    assert r.permit.session_started and r.loop.delay_bound.verified_boot == 0x6262
    assert r.layer.counts["installs"] > installs and r.layer.grid.scans


def test_the_refresh_carries_the_gear_legs_direction():
    """Codex 5a impl round 2 P2-5: the path check walks the leg's direction, not 0."""
    r = rig()
    r.tick(40)
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING and rec.branch == "auto")
    tracker = r.core.trackers[r.core.ms.phase]
    assert r.layer._request["direction"] == int(tracker.leg_ahead()[1]) != 0  # noqa: SLF001


def test_scans_keep_installing_at_0_12_mps_with_the_configured_delay_bound():
    """Design 1.1 (시험: 0.12 m/s 접근의 스캔이 채택됨) on the whole loop: the truck moved at 0.12 m/s for 1.6 s
    (here pushed, the fake MCU's counts turning at that speed), the real delay bound and STATE interval -- no scan
    dropped, results installed throughout."""
    from test_teleop_bridge_cycle import VEH

    r = rig()
    r.tick(40)
    drops, installs = r.layer.counts["scans_dropped"], r.layer.counts["installs"]
    orig = r.mcu.advance

    def advance(dt_us):
        orig(dt_us)
        for i in (0, 1):
            r.mcu.counts[i] += 0.12 / VEH.wheel_radius_m[i] * dt_us / 1e6 * VEH.counts_per_wheel_rev[i] / (2 * math.pi)

    r.mcu.advance = advance
    travel = r.loop.odom_travel
    r.tick(80)
    assert abs((r.loop.odom_travel - travel) - 0.12 * 1.6) < 0.02
    assert r.layer.counts["scans_dropped"] == drops
    assert r.layer.counts["installs"] >= installs + 14


def _adopted_waiting_rig():
    """Adopted and running, the permit still off: the new path's grid not yet installed (the ARM waits)."""
    r = rig()
    r.tick(40)
    r.submit()
    hold = {"on": False}
    step = r.layer.worker.step

    def held_step():
        if not hold["on"]:
            step()

    r.layer.worker.step = held_step
    run_until(r, lambda rec: r.core.commands.status == "running" and r.core.commands.adopted_id is not None
              and (hold.__setitem__("on", True) or True) and r.layer.installing())
    assert r.permit.state == ra.OFF
    return r, hold


def test_slam_lost_between_the_adoption_and_the_arm_latches_and_ends_the_mission():
    """Codex 5a impl round 3 P1: a fault while the adopted mission waits for its grid is the latch, not a wait the
    ARM goes out after."""
    r, hold = _adopted_waiting_rig()
    r.tracker.mode = "lost"
    rec = run_until(r, lambda rec: rec.branch == "emergency", limit=10)
    assert rec.reason.startswith("prepare:slam")
    r.tracker.mode = "tracking"
    hold["on"] = False
    arms = r.mcu.arms
    r.tick(20)
    assert r.mcu.arms == arms and r.permit.state in (ra.DISARMING, ra.LATCHED)
    assert r.core.commands.status not in ("accepted", "preparing", "running")


def test_a_dead_worker_between_the_adoption_and_the_arm_ends_the_mission():
    """Codex 5a impl round 3 P2-1."""
    r, hold = _adopted_waiting_rig()
    r.layer.worker.alive = False
    r.tick(3)
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)
    assert r.core.commands.status not in ("accepted", "preparing", "running")


def test_an_empty_tick_does_not_mend_a_continuing_break():
    """Codex 5a impl round 3 P2-2: one relocalization generation for one continuing break, an empty tick in it."""
    r = rig()
    r.tick(15)
    r.mcu.faults = FaultFlag.ENCODER_GAP
    r.tick(1)
    gen = r.core.relocalize_generation()
    orig = r.transport.receive
    r.transport.receive = lambda host, now_ns, limit=None: None  # no STATE this tick
    r.tick(1)
    r.transport.receive = orig
    assert not r.loop.odom_continuous
    r.tick(2)
    assert r.core.relocalize_generation() == gen
    r.mcu.faults = FaultFlag(0)


def test_an_explicit_source_loss_is_the_end_at_once():
    """Codex 5a impl round 3 P2-3 / design 3: the source saying it is disconnected latches without the watchdog's
    wait."""
    r = _driving_rig()
    r.layer.source_lost("lidar")
    rec = r.tick()
    assert (rec.branch, rec.reason) == ("emergency", "scan_source_lost")
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)
