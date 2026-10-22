import itertools
import math

import numpy as np
import pytest

from forklift_core.planning.fleet_traffic import (
    TrafficManager,
    arc_lengths,
    footprint_rectangle,
    pose_at,
    rectangles_overlap,
)
from forklift_core.planning.geometry import Footprint

TRUCK = Footprint(1.29, 0.17, 0.36)
SPEED = 1.0


def _line(x0, y0, x1, y1, step=0.1, yaw=None):
    n = max(2, int(round(math.hypot(x1 - x0, y1 - y0) / step)) + 1)
    heading = math.atan2(y1 - y0, x1 - x0) if yaw is None else yaw
    return np.column_stack((np.linspace(x0, x1, n), np.linspace(y0, y1, n), np.full(n, heading)))


def _path(*corners):
    legs = [_line(*a, *b) for a, b in zip(corners[:-1], corners[1:], strict=True)]
    return np.vstack([legs[0]] + [leg[1:] for leg in legs[1:]])


def _manager(routes, **kwargs):
    kwargs = {"margin_m": 0.3, "lookahead_m": 3.0, "speed_mps": SPEED, "accel_mps2": 100.0, "mode": "queues", **kwargs}
    manager = TrafficManager(**kwargs)
    for rank, (name, route) in enumerate(routes.items()):
        manager.add(name, rank, route[0], TRUCK)
    for name, route in routes.items():
        manager.set_route(name, [(route, TRUCK)])
    return manager


def _drive(manager, routes, steps, dt=0.1, paused=None, pace=False):
    """Followers at SPEED (or the permit's pace) inside their grants."""
    s = {n: arc_lengths(p) for n, p in routes.items()}
    progress = {n: 0.0 for n in routes}
    overlapped, log = False, []
    for k in range(steps):
        now = k * dt
        poses = {n: pose_at(routes[n], s[n], progress[n]) for n in routes}
        permits = manager.update(now, poses, progress)
        bodies = {n: footprint_rectangle(poses[n], TRUCK) for n in routes}
        for a, b in itertools.combinations(sorted(routes), 2):
            overlapped |= rectangles_overlap(bodies[a], bodies[b])
        moved = {}
        for n in routes:
            lo, hi = (paused or {}).get(n, (np.inf, np.inf))
            speed = SPEED
            if pace and permits[n].pace_mps is not None:
                speed = min(speed, permits[n].pace_mps)
            step = 0.0 if lo <= now < hi else min(speed * dt, permits[n].allowed_m)
            moved[n] = step
            progress[n] = min(progress[n] + step, s[n][-1])
        log.append((now, dict(progress), moved, {n: p.pace_mps for n, p in permits.items()}))
    return progress, overlapped, log


def _done(progress, routes):
    return {n: progress[n] >= arc_lengths(routes[n])[-1] - 1e-6 for n in routes}


def test_three_trucks_through_one_place_stand_in_one_line_each_with_a_window():
    # a east, b north and c south-east through (6, 0): three pairwise crossings, one place.
    routes = {"a": _line(0, 0, 14, 0), "b": _line(6, -7, 6, 7), "c": _line(1, 5, 11, -5)}
    manager = _manager(routes, issue_s=60.0)
    q = manager.windows
    assert len(q.crossings) == 3 and len(q.members) == 6  # a member per truck per crossing
    assert len(set(q.places().values())) == 1
    manager.update(0.0, {n: r[0] for n, r in routes.items()}, {n: 0.0 for n in routes})
    line = q.line(next(iter(q.members)))
    assert len(line) == 6 and all(m.issued_s is not None for m in line)  # every truck in line has a token
    index = q._index(q.members)
    for c in q.crossings.values():
        ma, mb = q._pair(c, index)
        first, second = (ma, mb) if c.first == c.a else (mb, ma)
        assert first.close_s < second.close_s  # each window after the one ahead
    progress, overlapped, _ = _drive(manager, routes, steps=600)
    assert not overlapped and all(_done(progress, routes).values())


def test_a_truck_passing_one_place_twice_is_in_its_line_twice():
    # a drives into a dead end and back out; b crosses its lane in between.
    routes = {"a": np.vstack((_line(0, 0, 12, 0), _line(12, 0, 0, 0, yaw=0.0)[1:])), "b": _line(6, -7, 6, 7)}
    manager = _manager(routes)
    q = manager.windows
    mine = [m for m in q.members.values() if m.truck == "a"]
    assert len(mine) == 2 and len(set(q.places().values())) == 1
    progress, overlapped, _ = _drive(manager, routes, steps=600)
    assert not overlapped and all(_done(progress, routes).values())


def test_an_expired_member_joins_the_line_again_behind_the_truck_due_earlier():
    routes = {"a": _line(0, 0, 14, 0), "b": _line(6, -8, 6, 8)}
    manager = _manager(routes, issue_s=30.0, slack_s=5.0)
    (c,) = manager.windows.crossings.values()
    assert c.first == "a" and not c.entered
    progress, overlapped, log = _drive(manager, routes, steps=700, paused={"a": (0.0, 40.0)})
    assert not overlapped and all(_done(progress, routes).values())
    expired = [e for e in manager.windows.events if e["kind"] == "window_expired"]
    assert expired and expired[0]["truck"] == "a" and expired[0]["other"] == "b"
    assert manager.windows.counts["requeued"] >= 1
    b_done = next(t for t, p, _, _ in log if p["b"] >= arc_lengths(routes["b"])[-1] - 1e-6)
    assert b_done < 40.0  # b went through while a stood


def test_a_member_late_only_for_the_truck_ahead_keeps_its_place():
    routes = {"a": _line(3, 0, 14, 0), "b": _line(6, -8, 6, 8)}
    manager = _manager(routes, issue_s=30.0, slack_s=5.0)
    for now in (0.0, 0.1):  # a's grant goes in: a is in, first
        manager.update(now, {"a": routes["a"][0], "b": routes["b"][0]}, {"a": 0.0, "b": 0.0})
    q = manager.windows
    b = next(m for m in q.members.values() if m.truck == "b")
    ticket = b.ticket
    progress, overlapped, log = _drive(manager, routes, steps=700, paused={"a": (0.0, 40.0)})
    assert not overlapped and all(_done(progress, routes).values())
    assert q.counts["renewed"] >= 1
    assert not [e for e in q.events if e["kind"] == "window_expired"]
    assert b.ticket == ticket


def test_a_follower_keeps_behind_without_touching():
    routes = {"a": _line(0, 0, 30, 0), "b": _line(-4, 0, 26, 0)}
    manager = _manager(routes)
    progress, overlapped, log = _drive(manager, routes, steps=500, paused={"a": (5.0, 15.0)})
    assert not overlapped and all(_done(progress, routes).values())
    gaps = [p["a"] + 4 - p["b"] for t, p, _, _ in log if 10.0 < t < 15.0]
    assert min(gaps) > 1.46 and max(gaps) < 3.5


def test_pace_lets_a_truck_arrive_as_the_one_ahead_clears_instead_of_stopping():
    # a reaches b's lane first and is still in it when b gets there: without
    # pace b drives up and stands; with pace it slows and does not stand.
    routes = {"a": _line(2, 0, 16, 0), "b": _line(8, -8, 8, 8)}
    end_b = arc_lengths(routes["b"])[-1]
    results = {}
    for pace in (False, True):
        manager = _manager(routes, pace=pace)
        assert {c.first for c in manager.windows.crossings.values()} == {"a"}
        progress, overlapped, log = _drive(manager, routes, steps=600, pace=pace)
        assert not overlapped and all(_done(progress, routes).values())
        stood = sum(1 for t, p, moved, _ in log if p["b"] < end_b - 1e-6 and moved["b"] < 1e-9)
        done = next(t for t, p, _, _ in log if p["b"] >= end_b - 1e-6)
        capped = [v for t, p, _, paces in log for v in [paces["b"]] if v is not None]
        results[pace] = (stood, done, capped)
    assert results[False][0] > 10 and results[True][0] <= 2  # it stood without pace, hardly with it
    assert results[True][1] <= results[False][1] + 0.5  # and got through no later
    assert results[True][2] and all(0.1 <= v < SPEED for v in results[True][2])


def test_pace_needs_queues_mode():
    with pytest.raises(ValueError):
        TrafficManager(mode="windows", pace=True)


def _costs_of_every_order(manager):
    q = manager.windows
    index = q._index(q.members)
    open_ = [c for c in q.crossings.values() if q._open(c, index)]
    best = np.inf
    keep = {c.id: c.first for c in q.crossings.values()}
    for firsts in itertools.product(*[(c.a, c.b) for c in open_]):
        for c, f in zip(open_, firsts, strict=True):
            c.first = f
        if not q._run(q.crossings, manager.trucks)[1]:
            best = min(best, q.cost(q.crossings, manager.trucks, 0.0))
    for cid, f in keep.items():
        q.crossings[cid].first = f
    return best


def test_the_global_solve_finds_the_best_orders_on_a_small_grid():
    # Four trucks over a 2 x 2 grid of lanes: four crossings, 16 orders.
    routes = {
        "a": _line(-12, 0, 20, 0), "b": _line(-20, 8, 12, 8),
        "c": _line(0, 16, 0, -14, yaw=-math.pi / 2), "d": _line(8, -18, 8, 14),
    }
    manager = _manager(routes, queue_opt="global", issue_s=1.0)
    q = manager.windows
    assert len(q.crossings) == 4
    best = _costs_of_every_order(manager)
    q._optimise_global(0.0, manager.trucks)
    assert not q._run(q.crossings, manager.trucks)[1]
    assert q.cost(q.crossings, manager.trucks, 0.0) == pytest.approx(best, abs=1e-6)


def test_the_order_model_is_the_one_its_dense_rows_gave(monkeypatch):
    # scipy turns dense rows into this very CSC matrix, so HiGHS sees the same model.
    import scipy.optimize
    from scipy.sparse import csc_array

    routes = {
        "a": _line(-12, 0, 20, 0), "b": _line(-20, 8, 12, 8),
        "c": _line(0, 16, 0, -14, yaw=-math.pi / 2), "d": _line(8, -18, 8, 14),
    }
    manager = _manager(routes, queue_opt="global", issue_s=1.0)
    seen, real = [], scipy.optimize.milp
    monkeypatch.setattr(scipy.optimize, "milp", lambda *a, **k: seen.append(k["constraints"].A) or real(*a, **k))
    manager.windows._optimise_global(0.0, manager.trucks)
    (matrix,) = seen
    dense = csc_array(matrix.toarray())
    for part in ("indptr", "indices", "data"):
        np.testing.assert_array_equal(getattr(matrix, part), getattr(dense, part))


def test_an_order_horizon_leaves_further_crossings_as_they_are():
    routes = {
        "a": _line(-12, 0, 20, 0), "b": _line(-20, 8, 12, 8),
        "c": _line(0, 16, 0, -14, yaw=-math.pi / 2), "d": _line(8, -18, 8, 14),
    }
    for horizon in (0.0, 1e6):
        manager = _manager(routes, queue_opt="global", issue_s=1.0, opt_horizon_s=horizon)
        q = manager.windows
        before = {c.id: c.first for c in q.crossings.values()}
        best = _costs_of_every_order(manager)
        q._optimise_global(0.0, manager.trucks)
        assert not q._run(q.crossings, manager.trucks)[1]
        if horizon == 0.0:  # every crossing is further on: nothing turns
            assert {c.id: c.first for c in q.crossings.values()} == before
        else:  # all of them within: the best orders, as without a horizon
            assert q.cost(q.crossings, manager.trucks, 0.0) == pytest.approx(best, abs=1e-6)


def test_the_global_solve_keeps_fixed_orders():
    routes = {"a": _line(3, 0, 14, 0), "b": _line(6, -8, 6, 8)}
    manager = _manager(routes, queue_opt="global")
    for now in (0.0, 0.1):  # a is in: its order is fixed
        manager.update(now, {"a": routes["a"][0], "b": routes["b"][0]}, {"a": 0.0, "b": 0.0})
    (c,) = manager.windows.crossings.values()
    assert c.first == "a" and c.entered
    manager.windows._optimise_global(0.2, manager.trucks)
    assert c.first == "a"


@pytest.mark.parametrize("opt", ["local", "global"])
@pytest.mark.parametrize("seed", [1, 2, 3])
def test_grid_traffic_with_random_stops_neither_touches_nor_locks(seed, opt):
    rng = np.random.default_rng(seed)
    rows, cols = rng.permutation([0.0, 8.0, 16.0]), rng.permutation([0.0, 8.0, 16.0])
    routes = {
        f"t{k}": _path((-10 - 4 * k, y), (x, y), (x, 30 + 4 * k)) for k, (y, x) in enumerate(zip(rows, cols, strict=True))
    }
    routes["w"] = _path((cols[0], 24), (cols[0], rows[1]), (-30, rows[1]))
    paused = {}
    for n in routes:
        start = float(rng.uniform(0, 40))
        paused[n] = (start, start + float(rng.uniform(5, 30)))
    manager = _manager(routes, issue_s=20.0, slack_s=3.0, pace=True, queue_opt=opt)
    progress, overlapped, log = _drive(manager, routes, steps=2500, paused=paused, pace=True)
    assert not overlapped
    assert all(_done(progress, routes).values()), progress
    assert manager.deadlocks(2500 * 0.1) == []


def test_a_truck_standing_in_a_long_shared_lane_does_not_hold_a_junction_further_along():
    # l shares its whole lane with s behind it and stands at its start; t crosses the
    # lane 30 m on and should not wait for l to get there.
    routes = {"l": _line(0, 0, 40, 0), "s": _line(-10, 0, 35, 0), "t": _line(30, -35, 30, 10)}
    manager = _manager(routes, issue_s=30.0, slack_s=10.0)
    progress, overlapped, log = _drive(manager, routes, steps=2500, paused={"l": (1.0, 150.0)})
    assert not overlapped and all(_done(progress, routes).values())
    end_t = arc_lengths(routes["t"])[-1]
    assert next(t for t, p, _, _ in log if p["t"] >= end_t - 1e-6) < 150.0
