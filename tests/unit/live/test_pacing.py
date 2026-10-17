"""M-c1: real-time pacing (plan C1)."""

import numpy as np
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


def test_catch_up_makes_up_a_small_lag_in_at_most_its_steps():
    """Real-time plan D7: 30 ms late, the next steps start at once until the lag is made up -- no wait between them."""
    p = Pacer(1.0, DT, catchup_steps=6)
    wall = run(p, 0.0, 120, 0.001)
    on_time = wall + p.wait_s(wall)
    wall = on_time + 0.030  # a slow step
    waits = []
    for _ in range(8):
        waits.append(p.wait_s(wall))
        wall += waits[-1]
        p.started(wall)
        wall += 0.001
        p.after_step(wall)
    burst = sum(1 for w in waits if w == 0.0)
    assert 3 <= burst <= 6  # ~30 ms / (8.3 - 1) ms per step
    assert waits[-1] > 0.0  # caught up: spaced again


def test_catch_up_never_makes_up_more_than_its_steps_nor_after_a_stall():
    p = Pacer(1.0, DT, catchup_steps=6)
    wall = run(p, 0.0, 120, 0.001)
    wall += 0.2  # late by 200 ms, not a stall (0.25 s)
    start = wall
    wall = run(p, wall, 27, 0.001)
    # some, but at most 6 steps' worth (50 ms), is made up: not 27 ms for 27 steps
    made_up = 26 * DT + 0.001 - (wall - start)
    assert 3 * DT <= made_up <= 6 * DT + 1e-9
    wall += 0.3  # a stall
    start = wall
    wall = run(p, wall, 27, 0.001)
    assert wall - start == pytest.approx(26 * DT + 0.001, rel=0.01)  # spaced: nothing made up after a stall


def test_catch_up_keeps_the_average_factor_when_slow_steps_average_out():
    """Every 12th step costs 30 ms, the rest 1.7 ms (0.5 s a sim second): with catch-up the factor stays ~1 and ok."""
    p = Pacer(1.0, DT, catchup_steps=6)
    wall = 0.0
    for k in range(2400):
        wall += p.wait_s(wall)
        p.started(wall)
        wall += 0.030 if k % 12 == 0 else 0.0017
        p.after_step(wall)
    assert p.realtime_factor() == pytest.approx(1.0, abs=0.02) and p.ok
    q = Pacer(1.0, DT)  # without it the same steps lose ~22 ms a slow step
    wall = 0.0
    for k in range(2400):
        wall += q.wait_s(wall)
        q.started(wall)
        wall += 0.030 if k % 12 == 0 else 0.0017
        q.after_step(wall)
    assert q.realtime_factor() < 0.85 and not q.ok


@pytest.mark.parametrize("cost", [0.001, 0.0031, 0.006, 0.008])
def test_a_burst_never_runs_more_than_its_steps_in_a_row(cost):
    """Codex real-time round 5 P1: 200 ms late, steps costing up to nearly their spacing -- at most 6 steps start
    ahead of the plain spacing in a row, so over any wall interval the sim runs at most 6 steps ahead."""
    p = Pacer(1.0, DT, catchup_steps=6)
    wall = run(p, 0.0, 120, cost)
    wall += 0.2
    starts = []
    for _ in range(200):
        wall += p.wait_s(wall)
        p.started(wall)
        starts.append(wall)
        wall += cost
        p.after_step(wall)
    gaps = np.diff(starts)
    row, longest = 0, 0
    for g in gaps:
        row = row + 1 if g < DT - 1e-9 else 0
        longest = max(longest, row)
    assert longest <= 6
    # over every interval between two starts the sim runs at most 6 steps more than wall x factor
    s = np.asarray(starts)
    k = np.arange(len(s))
    ahead = (k[None, :] - k[:, None]) * DT - (s[None, :] - s[:, None])
    assert ahead[np.triu_indices(len(s), 1)].max() <= 6 * DT + 1e-9


def test_repeated_small_delays_are_made_up_each_time():
    p = Pacer(1.0, DT, catchup_steps=6)
    wall = 0.0
    for k in range(2400):
        wall += p.wait_s(wall)
        p.started(wall)
        wall += 0.025 if k % 12 == 0 else 0.001  # a render-like step every 12, then cheap ones
        p.after_step(wall)
    assert p.realtime_factor() == pytest.approx(1.0, abs=0.02) and p.ok and p.stalls == 0
