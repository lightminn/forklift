"""M-c1: real-time pacing (plan C1)."""

import pytest

from forklift_core.live.pacing import Pacer

DT = 1 / 120


def run(p, wall, steps, cost):
    """Step ``steps`` times, each physics step taking ``cost`` of wall time, sleeping as told."""
    for _ in range(steps):
        wall += p.wait_s(wall)
        p.started(wall)
        wall += cost
        p.after_step(wall)
    return wall


def test_a_fast_simulation_is_held_to_the_declared_factor():
    p = Pacer(1.0, DT)
    wall = run(p, 0.0, 240, 0.001)
    assert wall == pytest.approx(2.0, abs=0.02) and p.realtime_factor() == pytest.approx(1.0, rel=0.01)
    half = Pacer(0.5, DT)
    assert run(half, 0.0, 120, 0.001) == pytest.approx(2.0, abs=0.03)


def test_a_late_step_is_not_made_up_by_running_faster():
    """Codex M-c plan round 1 P2: after a 200 ms delay the next steps keep their spacing."""
    p = Pacer(1.0, DT)
    wall = run(p, 0.0, 120, 0.001)
    wall += 0.2  # a synchronous call between two steps
    start = wall
    wall = run(p, wall, 27, 0.001)
    assert wall - start == pytest.approx(26 * DT + 0.001, rel=0.01)  # spaced: ~0.22 s for 27 steps, not 27 ms


def test_ok_needs_a_held_good_factor_and_a_stall_drops_it():
    p = Pacer(1.0, DT)
    wall = run(p, 0.0, 60, 0.001)
    assert not p.ok  # the window has not spanned yet / not held long enough
    wall = run(p, wall, 240, 0.001)
    assert p.ok
    wall += 0.3  # a synchronous call
    wall = run(p, wall, 1, 0.001)
    assert not p.ok and p.stalls == 1
    wall = run(p, wall, 60, 0.001)
    assert not p.ok  # the window still holds the stall's slow interval / the hold restarts
    run(p, wall, 300, 0.001)
    assert p.ok


def test_render_steps_that_are_slow_but_average_out_do_not_flip_ok():
    """agy T3: every 12th step costs 15 ms, the rest 2 ms (factor 0.94 on average): ok holds, it does not flip."""
    p = Pacer(1.0, DT)
    wall = 0.0
    flips = 0
    last = p.ok
    for k in range(1200):
        wall += p.wait_s(wall)
        p.started(wall)
        wall += 0.015 if k % 12 == 0 else 0.002
        p.after_step(wall)
        flips += p.ok != last
        last = p.ok
    assert p.ok and flips == 1


def test_a_simulation_too_slow_for_the_target_is_not_ok():
    p = Pacer(1.0, DT)
    run(p, 0.0, 240, 0.012)  # 12 ms a step: 0.69x
    assert not p.ok and p.realtime_factor() == pytest.approx(DT / 0.012, rel=0.02)



def test_a_late_wake_does_not_squeeze_the_next_steps():
    """Codex M-c review round 3 P2: the sleep ended 20 ms late; the next start is spaced from the real one."""
    p = Pacer(1.0, DT)
    wall = run(p, 0.0, 10, 0.001)
    wall += p.wait_s(wall) + 0.020  # woke late
    p.started(wall)
    first = wall
    wall += 0.001
    p.after_step(wall)
    wall += p.wait_s(wall)
    p.started(wall)
    assert wall - first >= DT - 1e-12
