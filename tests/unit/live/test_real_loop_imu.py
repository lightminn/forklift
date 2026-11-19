"""docs/design/2026-11-19-real-safety-wiring.md 4 (stage 5b) on the fake truck: the IMU watch gates the ARM on a
steady baseline (a short wait while it gathers, one deadline per mission), freezes it before the ARM, and a lost or
tilted IMU while armed is the emergency -- at the tick head and at the first byte."""

import math

import pytest

from forklift_core.live import real_autonomy as ra
from forklift_core.mcu_link.messages import Arm, Drive

from real_loop_rig import Rig
from test_teleop_bridge_cycle import MS


def run_until(r, cond, limit=3000):
    for _ in range(limit):
        rec = r.tick()
        if cond(rec):
            return rec
    raise AssertionError(f"not reached: {r.loop.records[-1]}")


def rig(**kw):
    r = Rig(frames=True, operator=[(1, "confirm")], imu=True, **kw)
    r.start()
    return r


def tilted(deg):
    a = math.radians(deg)
    return (9.81 * math.sin(a), 0.0, 9.81 * math.cos(a))


def driving():
    r = rig()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING and rec.branch == "auto")
    return r


def test_required_needs_the_watch_and_its_source():
    from forklift_core.live.real_loop import RealLoop

    from forklift_core.live.real_assembly import Watches

    w = rig().loop
    required = Watches(w.watch, w.imu_watch, "required")
    with pytest.raises(ValueError, match="imu_required"):  # the assembled watches without a source
        RealLoop.__init__(object.__new__(RealLoop), **_loop_kwargs(watches=required, imu_source=None))
    with pytest.raises(ValueError, match="imu_required"):  # required, the IMU watch dropped
        RealLoop.__init__(object.__new__(RealLoop), **_loop_kwargs(watches=Watches(w.watch, None, "required"),
                                                                   imu_source=None))


def _loop_kwargs(**over):
    """The rig's RealLoop arguments, with some replaced (the constructor's own refusal checked)."""
    import inspect

    r = rig()
    loop = r.loop
    names = [p for p in inspect.signature(type(loop).__init__).parameters if p != "self"]
    kw = {}
    for n in names:
        if hasattr(loop, n):
            kw[n] = getattr(loop, n)
    kw.update(core=r.core, session=loop.session, permit=r.permit, port=r.port, host=r.host, link=r.link,
              channels=r.channels, transport=r.transport, params=loop.params, odometry=loop.odometry,
              kinematics=loop.kin, max_speed_mps=0.15, start_ns=loop.start_ns, slam_source=loop.slam_source,
              evidence=loop.evidence, base_scenario=loop.base_scenario, wait_pose=None)
    kw.update(over)
    return {k: v for k, v in kw.items() if k in names}


def test_the_baseline_is_frozen_before_the_arm_and_s10_runs():
    r = rig()
    r.submit()
    run_until(r, lambda rec: r.mcu.arms > 0)
    assert r.loop.imu_watch.armed
    run_until(r, lambda rec: r.permit.state == ra.LATCHED and r.loop.session.results)
    assert [e["reason"] for e in r.loop.session.results] == ["s10_lifted"]
    assert not r.loop.imu_watch.armed


def test_the_arm_waits_for_the_baseline_within_one_deadline_then_latches():
    """A stand with no IMU samples yet: the ARM waits (adopted_for unset), then past baseline_wait_s it latches and
    the mission ends."""
    r = rig()
    r.imu.paused = True
    r.submit()
    run_until(r, lambda rec: r.core.commands.status == "running" and r.core.commands.adopted_id is not None)
    first = r.now
    r.tick(5)
    assert r.permit.state == ra.OFF and r.mcu.arms == 0 and r.loop.adopted_for is None
    rec = run_until(r, lambda rec: rec.branch == "emergency", limit=60)
    assert "imu_no_baseline" in rec.reason and r.now - first >= 0.45e9
    assert r.mcu.arms == 0
    r.tick(2)
    assert r.core.commands.status not in ("running", "accepted", "preparing")


def test_a_baseline_that_comes_in_time_arms():
    r = rig()
    r.imu.paused = True
    r.submit()
    run_until(r, lambda rec: r.core.commands.status == "running" and r.core.commands.adopted_id is not None)
    r.tick(5)
    r.imu.paused = False
    run_until(r, lambda rec: r.mcu.arms > 0, limit=40)


def test_an_unsteady_stand_refuses_the_arm():
    r = rig()
    flip = {"k": 0}
    src = r.imu.samples

    def wobble(now_ns, limit):
        out = src(now_ns, limit)
        res = []
        for ns, a in out:
            flip["k"] += 1
            res.append((ns, tilted(6) if flip["k"] % 2 else a))
        return res

    r.imu.samples = wobble  # the stand's samples alternate 0 and 6 deg (each 3 deg off the mean > 2 deg spread)
    r.submit()
    rec = run_until(r, lambda rec: rec.branch == "emergency", limit=400)
    assert "imu_baseline_unsteady" in rec.reason and r.mcu.arms == 0


def test_a_lost_imu_while_driving_is_the_emergency():
    r = driving()
    r.imu.paused = True
    rec = run_until(r, lambda rec: rec.branch == "emergency", limit=20)
    assert rec.reason in ("watch:watch_imu_stale", "watch:watch_imu_gap") or "watch_imu" in rec.reason
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)


def test_a_held_tilt_while_driving_is_the_emergency():
    r = driving()
    r.imu.accel = tilted(12)
    rec = run_until(r, lambda rec: rec.branch == "emergency", limit=40)
    assert "watch_tilt" in rec.reason
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)


def test_a_failing_source_is_a_lost_watch_not_a_crash():
    r = driving()
    r.imu.fail = True
    rec = run_until(r, lambda rec: rec.branch == "emergency", limit=20)
    assert "watch_imu" in rec.reason


def test_the_first_byte_judges_the_imu_inside_its_budget():
    """A tilt hold completed by samples arriving after the tick head stops the frame at the first byte."""
    r = driving()
    w = r.loop.imu_watch
    orig = w.judge
    calls = {"n": 0}

    def judge(now_ns):
        calls["n"] += 1
        # the head's read passes; the first byte's read (later, inside the send) trips
        return "watch_tilt" if now_ns > r.now else orig(now_ns)

    w.judge = judge
    r.tick_cost_ns = 5 * MS
    k = len(r.transport.sent)
    rec = r.tick()
    moving = [m for _, _, _, m in r.transport.sent[k:] if isinstance(m, Drive)
              and (m.left_wheel_rad_s or m.right_wheel_rad_s)]
    assert not moving and calls["n"] >= 2
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)
    assert not [m for _, _, _, m in r.transport.sent[k:] if isinstance(m, Arm)]


def test_an_existing_pre_arm_refusal_latches_at_once_even_while_the_imu_gathers():
    """5b design round 3 P2-2: outstanding / mcu_not_disarmed are judged before the IMU's wait."""
    r = rig()
    r.imu.paused = True  # the baseline would wait
    r.loop._pre_arm_refusal = lambda now_ns: "outstanding"  # noqa: SLF001
    r.submit()
    rec = run_until(r, lambda rec: rec.branch == "emergency", limit=400)
    assert "outstanding" in rec.reason and r.mcu.arms == 0


def test_samples_are_read_up_to_the_clock_at_the_read():
    """5b design round 3 P2-3: the read's own instant bounds the samples (not the tick's start), so a sample taken
    after the tick began is not thrown away as future."""
    r = rig()
    seen = []
    src = r.imu.samples

    def spy(upto_ns, limit):
        seen.append(upto_ns)
        return src(upto_ns, limit)

    r.imu.samples = spy
    r.tick_cost_ns = 6 * MS
    r.tick()
    assert seen and all(u >= r.now + 6 * MS for u in seen)
    assert r.loop.imu_watch.counts["future"] == 0


def test_a_sample_with_a_broken_time_is_dropped_alone():
    """5b round 4 P2-2: a NaN acquisition time drops that sample, not the watch."""
    r = driving()
    src = r.imu.samples

    def with_nan(upto_ns, limit):
        out = src(upto_ns, limit)
        return out[:1] + [(float("nan"), (0.0, 0.0, 9.81))] + out[1:]

    r.imu.samples = with_nan
    recs = [r.tick() for _ in range(10)]
    assert all(x.branch != "emergency" for x in recs) and r.permit.state == ra.DRIVING
    assert r.loop.imu_watch.counts["not_finite"] >= 10


def test_a_pose_changed_behind_a_short_gap_in_the_stand_refuses_the_arm():
    """5b round 4 P1: level, an 80 ms IMU gap, then 14 deg in the same stand -- not a baseline."""
    r = rig()
    src = r.imu.samples
    state = {"n": 0}

    def changed(upto_ns, limit):
        out = []
        for ns, a in src(upto_ns, limit):
            state["n"] += 1
            if 40 <= state["n"] < 56:
                continue  # 16 samples (80 ms) lost
            out.append((ns, tilted(14) if state["n"] >= 56 else a))
        return out

    r.tick(5)  # the stand's level samples first
    r.imu.samples = changed
    r.submit()
    rec = run_until(r, lambda rec: rec.branch == "emergency" or r.mcu.arms > 0, limit=400)
    assert r.mcu.arms == 0 and "imu_baseline_unsteady" in rec.reason


def test_sample_times_stay_integers_and_a_non_integer_time_is_dropped_alone():
    """5b round 5 P2: host ns past 2**53 keep their precision; a float or a huge value is that sample's fault."""
    r = driving()
    src = r.imu.samples

    def odd(upto_ns, limit):
        out = src(upto_ns, limit)
        return out + [(1.5e18, (0.0, 0.0, 9.81)), ("x", (0.0, 0.0, 9.81))]

    r.imu.samples = odd
    recs = [r.tick() for _ in range(10)]
    assert all(x.branch != "emergency" for x in recs)
    assert r.loop.imu_watch.counts["not_finite"] >= 20
    w = r.loop.imu_watch
    big = 2 ** 60
    assert w.add(big, (0.0, 0.0, 9.81), big) is None and w.samples[-1][0] == big  # exact, not rounded to future
