import itertools
import math

import numpy as np
import pytest

from forklift_core.planning.fleet_tiles import route_tiles
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
MODES = ("adg", "sections", "mapf", "tiles")


def _line(x0, y0, x1, y1, step=0.1, yaw=None):
    n = max(2, int(round(math.hypot(x1 - x0, y1 - y0) / step)) + 1)
    heading = math.atan2(y1 - y0, x1 - x0) if yaw is None else yaw
    return np.column_stack((np.linspace(x0, x1, n), np.linspace(y0, y1, n), np.full(n, heading)))


def _path(*corners):
    legs = [_line(*a, *b) for a, b in zip(corners[:-1], corners[1:], strict=True)]
    return np.vstack([legs[0]] + [leg[1:] for leg in legs[1:]])


def _manager(routes, mode, **kwargs):
    kwargs = {"margin_m": 0.3, "lookahead_m": 3.0, "speed_mps": SPEED, "accel_mps2": 100.0, "mode": mode, **kwargs}
    manager = TrafficManager(**kwargs)
    for rank, (name, route) in enumerate(routes.items()):
        manager.add(name, rank, route[0], TRUCK)
    for name, route in routes.items():
        manager.set_route(name, [(route, TRUCK)])
    return manager


def _drive(manager, routes, steps, dt=0.1, paused=None, check=None):
    s = {n: arc_lengths(p) for n, p in routes.items()}
    progress = {n: 0.0 for n in routes}
    overlapped, log = False, []
    for k in range(steps):
        now = k * dt
        poses = {n: pose_at(routes[n], s[n], progress[n]) for n in routes}
        permits = manager.update(now, poses, progress)
        if check is not None:
            check(manager)
        bodies = {n: footprint_rectangle(poses[n], TRUCK) for n in routes}
        for a, b in itertools.combinations(sorted(routes), 2):
            overlapped |= rectangles_overlap(bodies[a], bodies[b])
        for n in routes:
            lo, hi = (paused or {}).get(n, (np.inf, np.inf))
            step = 0.0 if lo <= now < hi else min(SPEED * dt, permits[n].allowed_m)
            progress[n] = min(progress[n] + step, s[n][-1])
        log.append((now, dict(progress)))
    return progress, overlapped, log


def _done(progress, routes):
    return {n: progress[n] >= arc_lengths(routes[n])[-1] - 1e-6 for n in routes}


def _finish(log, routes, name):
    end = arc_lengths(routes[name])[-1]
    return next(t for t, p in log if p[name] >= end - 1e-6)


@pytest.mark.parametrize("mode", MODES)
def test_two_crossing_trucks_neither_touch_nor_lock(mode):
    routes = {"a": _line(0, 0, 14, 0), "b": _line(6, -8, 6, 8)}
    manager = _manager(routes, mode)
    progress, overlapped, _ = _drive(manager, routes, steps=600)
    assert not overlapped and all(_done(progress, routes).values())


@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize("seed", [1, 2, 3])
def test_grid_traffic_with_random_stops_neither_touches_nor_locks(seed, mode):
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
    manager = _manager(routes, mode)
    progress, overlapped, _ = _drive(manager, routes, steps=2500, paused=paused)
    assert not overlapped
    assert all(_done(progress, routes).values()), progress
    assert manager.deadlocks(2500 * 0.1) == []


def test_adg_keeps_the_plan_order_while_the_first_truck_stands():
    # a reaches the crossing first in the plan, then stands 40 s short of it:
    # the order is kept, so b waits for a.
    routes = {"a": _line(0, 0, 14, 0), "b": _line(6, -8, 6, 8)}
    manager = _manager(routes, "adg")
    (c,) = manager.windows.crossings.values()
    assert c.first == "a"
    progress, overlapped, log = _drive(manager, routes, steps=900, paused={"a": (0.0, 40.0)})
    assert not overlapped and all(_done(progress, routes).values())
    assert _finish(log, routes, "b") > 40.0


def test_sections_let_the_closer_truck_go_first():
    # The same stand: b gets closer to the crossing than a, so it goes first.
    routes = {"a": _line(0, 0, 14, 0), "b": _line(6, -8, 6, 8)}
    manager = _manager(routes, "sections")
    progress, overlapped, log = _drive(manager, routes, steps=900, paused={"a": (0.0, 40.0)})
    assert not overlapped and all(_done(progress, routes).values())
    assert _finish(log, routes, "b") < 40.0
    assert manager.windows.counts["turned"] >= 1


def test_mapf_lets_a_lower_priority_truck_through_only_if_it_clears_in_time():
    # a has priority. b is due at the crossing a little before a: it does not
    # clear before a's grant gets there, so it waits for a ...
    routes = {"a": _line(-1, 0, 14, 0), "b": _line(6, -5, 6, 8)}
    manager = _manager(routes, "mapf")
    manager.update(0.0, {n: r[0] for n, r in routes.items()}, {n: 0.0 for n in routes})
    (c,) = manager.windows.crossings.values()
    assert c.first == "a"
    # ... while a b due well before a goes first.
    routes = {"a": _line(-25, 0, 14, 0), "b": _line(6, -5, 6, 8)}
    manager = _manager(routes, "mapf")
    (c,) = manager.windows.crossings.values()
    c.first = "a"
    manager.update(0.0, {n: r[0] for n, r in routes.items()}, {n: 0.0 for n in routes})
    assert c.first == "b"
    progress, overlapped, _ = _drive(manager, routes, steps=700)
    assert not overlapped and all(_done(progress, routes).values())


def test_route_tiles_cover_the_footprint():
    rects = np.array([[0.5, 0.5, 0.4, 0.2, 1.0, 0.0], [2.0, 0.0, 1.0, 0.3, math.cos(0.5), math.sin(0.5)]])
    first, second = route_tiles(rects, 1.0)
    assert len(first) == 1
    assert len(second) >= 4


def test_tiles_are_never_held_by_two_trucks():
    routes = {"a": _line(0, 0, 14, 0), "b": _line(6, -8, 6, 8), "c": _line(14, 2, 0, 2, yaw=math.pi)}

    def check(manager):
        tiles = manager.windows
        held = {}
        for t in manager.trucks.values():
            for x in tiles._holding(t):
                assert held.setdefault(x, t.name) == t.name

    manager = _manager(routes, "tiles")
    progress, overlapped, _ = _drive(manager, routes, steps=700, check=check)
    assert not overlapped and all(_done(progress, routes).values())
    assert manager.windows.counts["shared"] == 0


def test_tiles_refuse_a_reservation_that_leaves_no_way_through():
    # Head-on in one lane: whichever truck enters first goes through; the
    # other is refused at the lane's mouth until the way is clear.
    routes = {"a": _path((-6, 0), (20, 0)), "b": _path((26, 0.0), (0, 0.0))}
    routes["b"][:, 2] = math.pi
    manager = _manager(routes, "tiles", tile_horizon_m=40.0)
    progress, overlapped, _ = _drive(manager, routes, steps=50)
    assert not overlapped
    assert manager.windows.counts["refused"] >= 1


def test_tiles_turn_down_a_new_route_that_leaves_no_way_through():
    # a drives east to where b stands; b's new route would run west through a.
    # Neither can get past the other, so the route is turned down (the job waits).
    a, b = _line(0, 0, 30, 0), _line(30, 0, 0, 0, yaw=math.pi)
    manager = TrafficManager(margin_m=0.3, lookahead_m=3.0, speed_mps=SPEED, accel_mps2=100.0, mode="tiles")
    manager.add("a", 0, a[0], TRUCK)
    manager.add("b", 1, b[0], TRUCK)
    manager.set_route("a", [(a, TRUCK)])
    manager.update(0.0, {"a": a[0], "b": b[0]}, {"a": 0.0, "b": 0.0})
    assert not manager.trial_route("b", [(b, TRUCK)])
    assert manager.trial_route("b", [(_line(30, 0, 30, 12, yaw=math.pi / 2), TRUCK)])


def test_tiles_turn_down_a_route_through_a_parked_truck():
    # p stands for good (no route): a route over its tiles would leave the
    # truck stuck behind it, one past it is taken.
    manager = TrafficManager(margin_m=0.3, lookahead_m=3.0, speed_mps=SPEED, accel_mps2=100.0, mode="tiles")
    manager.add("p", 0, (10.0, 0.0, 0.0), TRUCK)
    manager.add("b", 1, (0.0, 0.0, 0.0), TRUCK)
    manager.update(0.0, {"p": (10.0, 0.0, 0.0), "b": (0.0, 0.0, 0.0)}, {"p": 0.0, "b": 0.0})
    assert not manager.trial_route("b", [(_line(0, 0, 20, 0), TRUCK)])
    assert manager.trial_route("b", [(_path((0, 0), (3, 0), (3, 3), (20, 3)), TRUCK)])


def test_tiles_of_neighbouring_holding_places_are_not_shared():
    # Two trucks 1.6 m apart (world_gen's holding pitch) back out side by side.
    routes = {"a": _line(0, 0, 0, -8, yaw=math.pi / 2), "b": _line(1.6, 0, 1.6, -8, yaw=math.pi / 2)}
    manager = _manager(routes, "tiles")
    progress, overlapped, _ = _drive(manager, routes, steps=200)
    assert not overlapped and all(_done(progress, routes).values())
    assert manager.windows.counts["shared"] == 0
