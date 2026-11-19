"""Codex 4c impl round 1 (P1-11, P1-12): the loop's watches -- the start interval and the tick's own time as design
3 states them, the layer latency watchdog in every phase (the lift included), the safety watch on each STATE."""

from forklift_core.live import real_autonomy as ra
from forklift_core.mcu_link.messages import Drive

from real_loop_rig import MS, ROOT, VEH, Rig


def run_until(r, cond, limit=4000):
    for _ in range(limit):
        rec = r.tick()
        if cond(rec):
            return rec
    raise AssertionError(f"not reached: {r.loop.records[-1]}")


def driving(**kw):
    r = Rig(frames=True, **kw)
    r.start()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING and r.loop.book.confirmed is not None
              and r.loop.book.confirmed.speed_mps != 0.0)
    return r


def test_three_start_intervals_over_30_ms_are_the_emergency():
    r = driving()
    r.step_ns = 40 * MS
    recs = [r.tick() for _ in range(3)]
    assert [x.branch for x in recs[:2]] != ["emergency", "emergency"]
    assert recs[2].reason == "loop_slow" and r.permit.state in (ra.DISARMING, ra.LATCHED)


def test_one_start_interval_over_60_ms_is_the_emergency():
    r = driving()
    r.step_ns = 70 * MS
    assert r.tick().reason == "loop_slow"


def test_a_tick_over_30_ms_before_its_first_write_waits_and_over_60_ms_is_the_emergency():
    r = driving()
    r.tick_cost_ns = 35 * MS
    rec = r.tick()
    assert (rec.branch, rec.reason) == ("wait", "tick_slow")
    r.tick_cost_ns = 65 * MS
    assert r.tick().reason == "tick_late" and r.permit.state in (ra.DISARMING, ra.LATCHED)


def test_the_layer_watchdog_holds_the_lift_and_its_end_is_the_emergency():
    r = Rig(frames=True, operator=[(1, "confirm")])
    r.start()
    r.submit()
    run_until(r, lambda rec: r.core.ms.phase == "lift")
    target = r.core.ms.lift_command
    r.core.obstacle["layer"].watchdog = lambda t: "layer_overdue"
    recs = [r.tick() for _ in range(5)]
    assert all(x.reason == "layer:layer_overdue" and x.branch == "wait" for x in recs)
    assert r.core.ms.lift_command == target  # the ramp waits
    r.core.obstacle["layer"].watchdog = lambda t: "layer_latency_exceeded"
    assert r.tick().reason == "layer_latency_exceeded" and r.permit.state in (ra.DISARMING, ra.LATCHED)


def test_the_safety_watch_runs_on_each_state_and_its_trip_is_the_emergency():
    from forklift_core.safety.real_watch import SafetyWatch, load_watch_config

    watch = SafetyWatch(load_watch_config(ROOT / "config/real/safety_watch.yaml"), wheel_radius_m=VEH.wheel_radius_m)
    r = driving(watch=watch)
    r.mcu.stuck = 0  # the left wheel does not turn whatever is asked
    for _ in range(200):
        k = len(r.transport.sent)
        rec = r.tick()
        if rec.branch == "emergency":
            break
    assert rec.reason.startswith("watch:") and r.permit.state in (ra.DISARMING, ra.LATCHED)
    assert not [m for _, _, _, m in r.transport.sent[k:] if isinstance(m, Drive) and m.left_wheel_rad_s]
