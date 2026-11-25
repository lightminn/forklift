"""PR 7 design 5 (7b) on the fake truck: the loop takes ``(gen, lo, hi, accel)`` samples beside the 5b point
samples, asks a source with ``generation()`` for its current generation after each uncapped read (a restart with no
new sample still reaches the watch), and the run's IMU guard keeps that call behind it."""

from types import SimpleNamespace

import pytest

from forklift_core.live import real_autonomy as ra
from forklift_core.live import real_run as rr
from forklift_core.safety.real_watch import ImuWatch

from test_real_loop_imu import rig, run_until
from test_teleop_bridge_cycle import MS

G = (0.0, 0.0, 9.81)


def fresh_watch(loop):
    loop.imu_watch = ImuWatch(loop.imu_watch.cfg)
    return loop.imu_watch


def test_interval_samples_reach_the_watch_with_their_interval_and_generation():
    r = rig()
    loop = r.loop
    w = fresh_watch(loop)
    at = 10_000 * MS
    loop.imu_source = SimpleNamespace(samples=lambda upto, limit: [(4, upto - 10 * MS, upto, G)])
    loop._pull_imu(at)  # noqa: SLF001
    hi, _unit, lo, gen, _serial = w.samples[-1]
    assert (hi, lo, gen) == (at, at - 10 * MS, 4)


def test_point_samples_still_go_in_as_points_without_a_generation():
    r = rig()
    loop = r.loop
    w = fresh_watch(loop)
    loop.imu_source = SimpleNamespace(samples=lambda upto, limit: [(upto, G)])
    loop._pull_imu(7 * MS)  # noqa: SLF001
    hi, _unit, lo, gen, _serial = w.samples[-1]
    assert (hi, lo, gen) == (7 * MS, 7 * MS, None)


def test_interval_times_that_are_not_integers_are_that_samples_fault():
    r = rig()
    loop = r.loop
    w = fresh_watch(loop)
    bad = [(1, 1.0e9, 2 * 10**9, G), (1, 1, "x", G), (True, 1, 2, G), (1, 2, 3), (1, 3 * MS, 4 * MS, G)]
    loop.imu_source = SimpleNamespace(samples=lambda upto, limit: bad)
    loop._pull_imu(5 * MS)  # noqa: SLF001
    assert w.counts["not_finite"] == 4 and w.counts["valid"] == 1


def test_a_restart_with_no_new_sample_ends_a_drive_on_the_next_read():
    """design 5's second test on the loop: the previous sample still fresh (stale would take 100 ms, five ticks),
    the source reports a new generation and gives nothing -- watch_imu_gap at once."""
    r = rig()
    src = r.imu.samples
    gen = [1]
    r.loop.imu_source = SimpleNamespace(
        samples=lambda upto, limit: [(gen[0], ns, ns, a) for ns, a in src(upto, limit)],
        generation=lambda: gen[0])
    fresh_watch(r.loop)  # the rig's start fed it point samples: this source's samples from here on
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING and rec.branch == "auto")
    for _ in range(5):
        assert r.tick().branch != "emergency"
    r.imu.paused = True
    gen[0] = 2
    rec = run_until(r, lambda rec: rec.branch == "emergency", limit=2)
    assert "watch_imu_gap" in rec.reason


def test_a_capped_read_leaves_the_new_generation_for_after_the_older_samples():
    """64 old-generation samples still waiting: the generation is not noted until they are in (a flip back and
    forth would drop the baseline twice)."""
    r = rig()
    loop = r.loop
    w = fresh_watch(loop)
    waiting = [(1, k * MS, k * MS, G) for k in range(1, 100)]

    def samples(upto, limit):
        out = [s for s in waiting if s[2] <= upto][:limit]
        del waiting[: len(out)]
        return out

    loop.imu_source = SimpleNamespace(samples=samples, generation=lambda: 2)
    loop._pull_imu(200 * MS)  # noqa: SLF001
    assert loop.imu_backlog and w.counts["generation"] == 0 and len(w.samples) == 64
    loop._pull_imu(200 * MS)  # noqa: SLF001
    assert not loop.imu_backlog and w.counts["generation"] == 1 and not w.samples


def test_a_failing_generation_call_leaves_the_samples_in():
    r = rig()
    loop = r.loop
    w = fresh_watch(loop)

    def boom():
        raise OSError("gone")

    loop.imu_source = SimpleNamespace(samples=lambda upto, limit: [(1, upto, upto, G)], generation=boom)
    loop._pull_imu(5 * MS)  # noqa: SLF001
    assert w.counts["valid"] == 1


def test_the_imu_guard_keeps_a_sources_generation_behind_it():
    marks = []
    plain = SimpleNamespace(samples=lambda upto, limit: [])
    assert not hasattr(rr._ImuGuard(plain, marks.append), "generation")  # noqa: SLF001
    assert rr._ImuGuard(SimpleNamespace(samples=plain.samples, generation=lambda: 3),  # noqa: SLF001
                        marks.append).generation() == 3

    def fatal():
        raise rr.SourceFatal("usb gone")

    guard = rr._ImuGuard(SimpleNamespace(samples=plain.samples, generation=fatal), marks.append)  # noqa: SLF001
    with pytest.raises(rr.SourceFatal):
        guard.generation()
    assert marks == ["imu:usb gone"]
