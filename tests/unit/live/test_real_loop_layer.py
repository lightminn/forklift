"""docs/design/2026-11-19-real-safety-wiring.md 1-3 (stage 5a) on the fake truck with a real RealLayer: S10-A end to
end on scans, the D5 exemption from the check hook, the planning grid gating the plan request, the layer forced on,
the layer's faults, scans dropped for an over-bound motion, a same-boot odometry break resetting the layer."""

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
    r.submit()
    run_until(r, lambda rec: r.core.ms.phase == "approach" and r.permit.state == ra.DRIVING)
    assert cell(r.layer.snapshot, PALLET[0] - 0.3, PALLET[1]) == OCCUPIED  # the pallet is an obstacle to the drive
    assert r.layer.exempt is None
    run_until(r, lambda rec: r.core.ms.phase == "insert")
    assert r.layer.exempt is not None  # the D5 check's region, installed by the check hook
    run_until(r, lambda rec: r.permit.state == ra.LATCHED and r.loop.session.results)
    assert [e["reason"] for e in r.loop.session.results] == ["s10_lifted"]
    assert r.layer.counts["scans_dropped"] == 0 and r.layer.counts["installs"] > 0


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
