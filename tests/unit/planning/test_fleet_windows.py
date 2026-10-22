import math
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.planning.fleet_traffic import (
    TrafficManager,
    arc_lengths,
    footprint_rectangle,
    oriented_rects,
    overlap_matrix,
    pose_at,
    rectangles_overlap,
)
from forklift_core.planning.fleet_queues import Queues
from forklift_core.planning.fleet_windows import (
    Crossing,
    Windows,
    time_profile,
    touch_groups,
)
from forklift_core.planning.geometry import Footprint

TRUCK = Footprint(1.29, 0.17, 0.36)
SPEED = 1.0


def _line(x0, y0, x1, y1, step=0.1, yaw=None):
    n = max(2, int(round(math.hypot(x1 - x0, y1 - y0) / step)) + 1)
    heading = math.atan2(y1 - y0, x1 - x0) if yaw is None else yaw
    return np.column_stack((np.linspace(x0, x1, n), np.linspace(y0, y1, n), np.full(n, heading)))


def _path(*corners):
    """Straight legs through the corners, heading along each leg."""
    legs = [_line(*a, *b) for a, b in zip(corners[:-1], corners[1:], strict=True)]
    return np.vstack([legs[0]] + [leg[1:] for leg in legs[1:]])


def _manager(routes, mode="windows", **kwargs):
    kwargs = {"margin_m": 0.3, "lookahead_m": 3.0, "speed_mps": SPEED, "accel_mps2": 100.0, **kwargs}
    manager = TrafficManager(mode=mode, **kwargs)
    for rank, (name, route) in enumerate(routes.items()):
        manager.add(name, rank, route[0], TRUCK)
    for name, route in routes.items():
        manager.set_route(name, [(route, TRUCK)])
    return manager


def _drive(manager, routes, steps, dt=0.1, paused=None):
    """Followers at SPEED inside their grants; paused[name] = (from_s, to_s)."""
    s = {n: arc_lengths(p) for n, p in routes.items()}
    progress = {n: 0.0 for n in routes}
    overlapped, log = False, []
    for k in range(steps):
        now = k * dt
        poses = {n: pose_at(routes[n], s[n], progress[n]) for n in routes}
        permits = manager.update(now, poses, progress)
        bodies = {n: footprint_rectangle(poses[n], TRUCK) for n in routes}
        names = sorted(routes)
        for i, a in enumerate(names):
            for b in names[i + 1 :]:
                overlapped |= rectangles_overlap(bodies[a], bodies[b])
        for n in routes:
            lo, hi = (paused or {}).get(n, (np.inf, np.inf))
            if not lo <= now < hi:
                progress[n] = min(progress[n] + min(SPEED * dt, permits[n].allowed_m), s[n][-1])
        log.append((now, dict(progress), {n: p.blocked_by for n, p in permits.items()}))
    return progress, overlapped, log


def _done(progress, routes):
    return {n: progress[n] >= arc_lengths(routes[n])[-1] - 1e-6 for n in routes}


def test_touch_groups_split_one_place_passed_twice():
    touch = np.zeros((12, 8), dtype=bool)
    touch[2:4, 5:7] = True  # a passes b's samples 5-6 early ...
    touch[8:10, 5:7] = True  # ... and again later
    touch[4, 6] = False
    ga, gb = np.arange(12) * 0.2, np.arange(8) * 0.2
    groups = touch_groups(touch, ga, gb)
    assert sorted((ia, ib) for ia, ib, _, _ in groups) == [((2, 3), (5, 6)), ((8, 9), (5, 6))]
    early = next(g for g in groups if g[0] == (2, 3))
    # b's sample 5 needs a past its sample 3; a's samples need b past its 6.
    assert early[3] == pytest.approx([ga[3], ga[3]])
    assert early[2] == pytest.approx([gb[6], gb[6]])


def test_free_flow_time_ramps_and_stops_at_a_gear_change():
    out = _line(0, 0, 10, 0)
    marks = np.array([0.0, 1.0, 10.0])
    t = time_profile(marks, out, 1.0, 0.5)
    assert t[0] == 0.0
    assert t[1] == pytest.approx(2.0)  # 1 m from rest at 0.5 m/s^2
    assert t[2] == pytest.approx(12.0)  # 2 s up, 8 m at 1 m/s, 2 s down
    back = np.vstack((out, _line(10, 0, 5, 0, yaw=0.0)[1:]))  # reverse out 5 m
    t = time_profile(np.array([15.0]), back, 1.0, 0.5)
    assert t[0] == pytest.approx(12.0 + 7.0)  # 2 s up, 1 m at 1 m/s, 2 s down


def test_a_cycle_in_the_orders_is_found_and_turning_one_clears_it():
    g = np.arange(20) * 0.2
    trucks = {
        n: SimpleNamespace(name=n, rank=k, g=g, eta=g.copy(), progress_m=0.0, granted_m=0.0, waiting_since_s=None)
        for k, n in enumerate("ab")
    }
    w = Windows(sample_m=0.2, lookahead_m=3.0)
    w.crossings = {
        0: Crossing(0, "a", "b", (10, 12), (2, 4), np.full(3, g[4]), np.full(3, g[12]), "a"),
        1: Crossing(1, "a", "b", (2, 4), (10, 12), np.full(3, g[12]), np.full(3, g[4]), "b"),
    }
    assert w._run(w.crossings, trucks)[1]
    assert w.timed(w.crossings, trucks, 0.0) is None
    w.crossings[1].first = "a"
    assert not w._run(w.crossings, trucks)[1]
    assert w.timed(w.crossings, trucks, 0.0) is not None


def test_a_near_truck_goes_ahead_of_a_far_one_that_ranks_would_wait_for():
    # b joins a's lane far ahead of a and drives along it for 30 m: more than
    # the rank rule scans for a stop off a's route, so under ranks b waits
    # until a has gone by; with windows b, much earlier there, goes first.
    routes = {"a": _line(-60, 0, 40, 0), "b": _path((0, -6), (0, 0), (30, 0), (30, 6))}
    end_b = arc_lengths(routes["b"])[-1]
    ranked = _manager(routes, mode="rank", scan_m=10.0)
    progress, overlapped, log = _drive(ranked, routes, steps=1800)
    assert not overlapped and all(_done(progress, routes).values())
    b_done_rank = next(t for t, p, _ in log if p["b"] >= end_b - 1e-6)
    assert any(blocked["b"] == "a" for _, _, blocked in log)

    windows = _manager(routes)
    first = {c.first for c in windows.windows.crossings.values()}
    assert first == {"b"}
    progress, overlapped, log = _drive(windows, routes, steps=1800)
    assert not overlapped and all(_done(progress, routes).values())
    b_done = next(t for t, p, _ in log if p["b"] >= end_b - 1e-6)
    assert b_done < b_done_rank - 30
    assert not any(blocked["b"] == "a" for _, _, blocked in log)


def test_a_follower_keeps_behind_without_touching():
    routes = {"a": _line(0, 0, 30, 0), "b": _line(-4, 0, 26, 0)}
    manager = _manager(routes)
    (c,) = manager.windows.crossings.values()
    assert c.first == "a" and c.entered  # a stands in it
    progress, overlapped, log = _drive(manager, routes, steps=500, paused={"a": (5.0, 15.0)})
    assert not overlapped and all(_done(progress, routes).values())
    # b caught up while a stood, and stayed a little over a body length behind.
    gaps = [p["a"] + 4 - p["b"] for t, p, _ in log if 10.0 < t < 15.0]
    assert min(gaps) > 1.46 and max(gaps) < 3.5


def test_a_truck_that_parks_on_another_route_first_is_a_body_to_go_round():
    # b, nearer, parks on a's lane: it goes first and then stands there; a,
    # coming later, is held by its body (TA's to go round), not by an order.
    routes = {"a": _line(0, 0, 20, 0), "b": _line(10, -6, 10, 0, yaw=math.pi / 2)}
    manager = _manager(routes, park_s=0.0)  # gone once parked: b first costs a nothing
    assert {c.first for c in manager.windows.crossings.values()} == {"b"}
    s = {n: arc_lengths(p) for n, p in routes.items()}
    progress = {n: 0.0 for n in routes}
    for k in range(300):
        poses = {n: pose_at(routes[n], s[n], progress[n]) for n in routes}
        permits = manager.update(k * 0.1, poses, progress)
        if progress["b"] >= s["b"][-1] - 1e-6 and len(manager.trucks["b"].g) > 1:
            manager.set_route("b", [(poses["b"].reshape(1, 3), TRUCK)])  # parked: idle
        for n in routes:
            progress[n] = min(progress[n] + min(SPEED * 0.1, permits[n].allowed_m), s[n][-1])
    assert progress["b"] == pytest.approx(s["b"][-1])
    assert permits["a"].blocked_by == "b" and manager.trucks["a"].held_by_body
    assert progress["a"] < 10 - 1.29


def test_an_unused_window_expires_and_the_other_truck_goes():
    routes = {"a": _line(0, 0, 14, 0), "b": _line(6, -8, 6, 8)}
    manager = _manager(routes, issue_s=30.0, slack_s=5.0)
    (c,) = manager.windows.crossings.values()
    assert c.first == "a" and not c.entered
    progress, overlapped, log = _drive(manager, routes, steps=700, paused={"a": (0.0, 40.0)})
    assert not overlapped and all(_done(progress, routes).values())
    expired = [e for e in manager.windows.events if e["kind"] == "window_expired"]
    assert expired and expired[0]["truck"] == "a" and expired[0]["turned"]
    b_done = next(t for t, p, _ in log if p["b"] >= arc_lengths(routes["b"])[-1] - 1e-6)
    assert b_done < 40.0  # b went through while a stood


def test_a_grant_already_in_is_not_taken_back():
    routes = {"a": _line(3, 0, 14, 0), "b": _line(6, -8, 6, 8)}
    manager = _manager(routes, issue_s=30.0, slack_s=5.0)
    for now in (0.0, 0.1):  # the grant goes in, then the crossing counts as entered
        manager.update(now, {"a": routes["a"][0], "b": routes["b"][0]}, {"a": 0.0, "b": 0.0})
    (c,) = manager.windows.crossings.values()
    assert c.first == "a" and c.entered
    progress, overlapped, log = _drive(manager, routes, steps=700, paused={"a": (0.0, 40.0)})
    assert not overlapped and all(_done(progress, routes).values())
    assert not [e for e in manager.windows.events if e["kind"] == "window_expired"]
    assert all(blocked["b"] == "a" for t, p, blocked in log if 20.0 < t < 39.0)


def test_a_route_with_no_lock_free_order_is_seen_before_it_is_taken():
    # a stands in b's way and b in a's, head-on in one lane: no order works.
    routes = {"a": _line(0, 0, 10, 0), "b": _line(4, 0, -6, 0)}
    manager = _manager({"a": routes["a"]})
    manager.add("b", 1, routes["b"][0], TRUCK)
    assert not manager.trial_route("b", [(routes["b"], TRUCK)])
    assert manager.trial_route("b", [(_path((4, 0), (4, 3), (-6, 3)), TRUCK)])
    assert all("b" not in (c.a, c.b) for c in manager.windows.crossings.values())


def test_a_way_out_of_orders_that_lock_already_is_taken_and_a_new_hold_is_not():
    # r2x multi_802_C5_20 (Q3): after a spill the orders locked with both grants in, so no
    # turn over helps; every route tried then was refused as the orders still locked.
    # Here a starts in b's last crossing and b first; a goes first where b crosses later.
    routes = {"a": _line(5, 0, 20, 0), "b": _path((15, -5), (15, 5), (5, 5), (5, -5))}
    manager = _manager(routes)
    w = manager.windows
    early, late = sorted(w.crossings.values(), key=lambda c: c.interval("a")[0])
    early.first, late.first = "b", "a"
    early.entered = late.entered = True
    assert w.held(manager.trucks) == {"a", "b"}
    manager.add("c", 2, (0, 10, 0), TRUCK)
    assert not w.replace("c", {}, manager.trucks, 0.0, dry=True)  # the old test: the orders lock
    assert manager.trial_route("c", [(_line(0, 10, 10, 10), TRUCK)])  # clear of both
    assert not manager.trial_route("c", [(_line(6, 8, 6, -8), TRUCK)])  # behind a, which cannot move
    assert manager.trial_route("b", [(_path((15, -5), (25, -5), (25, 5)), TRUCK)])  # b out of the lock
    assert all("c" not in (c.a, c.b) for c in w.crossings.values()) and w.held(manager.trucks) == {"a", "b"}


@pytest.mark.parametrize("mode", ["windows", "queues"])
def test_a_route_that_has_to_be_taken_gives_way_where_the_repair_finds_no_order(monkeypatch, mode):
    # r2x rack_804_C5_20 (A): a route re-planned round a spill got no order from the
    # repair, whose turns were left in place, and 19 trucks locked; from the orders
    # before the repair, the truck second at its new crossings locked none.
    routes = {"a": _line(0, 0, 14, 0), "b": _line(6, -3, 6, 7)}
    manager = _manager({"a": routes["a"]}, mode=mode)
    manager.add("b", 1, routes["b"][0], TRUCK)

    def gives_up(self, crossings, *args):
        for c in crossings.values():
            c.first = c.second
        return False

    monkeypatch.setattr(Queues if mode == "queues" else Windows, "_settle" if mode == "queues" else "_repair", gives_up)
    assert not manager.trial_route("b", [(routes["b"], TRUCK)])  # a trial does not give way
    manager.set_route("b", [(routes["b"], TRUCK)])
    (c,) = manager.windows.crossings.values()
    assert manager.windows.last_feasible and c.first == "a"  # b, due first, gives way


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_grid_traffic_with_random_stops_neither_touches_nor_locks(seed):
    rng = np.random.default_rng(seed)
    rows, cols = rng.permutation([0.0, 8.0, 16.0]), rng.permutation([0.0, 8.0, 16.0])
    routes = {
        f"t{k}": _path((-10 - 4 * k, y), (x, y), (x, 30 + 4 * k)) for k, (y, x) in enumerate(zip(rows, cols, strict=True))
    }
    # One the other way: south down a column, then west along a row (clear of
    # where the others park: a parked truck is a body for TA, not an order).
    routes["w"] = _path((cols[0], 24), (cols[0], rows[1]), (-30, rows[1]))
    paused = {}
    for n in routes:
        start = float(rng.uniform(0, 40))
        paused[n] = (start, start + float(rng.uniform(5, 30)))
    manager = _manager(routes, issue_s=20.0, slack_s=3.0)
    progress, overlapped, log = _drive(manager, routes, steps=2500, paused=paused)
    assert not overlapped
    assert all(_done(progress, routes).values()), progress
    assert manager.deadlocks(2500 * 0.1) == []


def test_a_leader_resting_short_of_its_grant_is_seen_and_the_lock_turned_over():
    # multi_100_C5_10 (v19 dev): b leads a through X but is held by Y, where a
    # goes first, at a grant end that releases a only if b reaches it exactly;
    # b rests 1 cm short, so each waits for the other.
    g = np.arange(40) * 0.2
    trucks = {
        "a": SimpleNamespace(name="a", rank=0, g=g, eta=g.copy(), progress_m=2.0, granted_m=2.2, waiting_since_s=None),
        "b": SimpleNamespace(name="b", rank=1, g=g, eta=g.copy(), progress_m=1.79, granted_m=1.8, waiting_since_s=None),
    }
    w = Windows(sample_m=0.2, lookahead_m=3.0, moving_m=0.05)
    w.crossings = {
        0: Crossing(0, "a", "b", (12, 15), (5, 8), np.full(4, 1.595), np.full(4, 3.0), "b", entered=True),
        1: Crossing(1, "a", "b", (20, 25), (10, 15), np.full(6, 2.9), np.full(6, 5.0), "a"),
    }
    assert w._run(w.crossings, trucks)[1]  # b held at 1.8 releases a only at 1.8 exactly
    w.step(0.0, trucks)
    assert w.crossings[1].first == "b"
    assert not w._run(w.crossings, trucks)[1]
    (event,) = [e for e in w.events if e["kind"] == "orders_locked"]
    assert event["healed"] and event["turned"] == [["b", "a"]]


def test_a_held_truck_loses_its_expired_token_where_free_flow_would_keep_it():
    # Two trucks the same way from one crossing: free flow alone, either order
    # costs the same, so an expired token stays (new window, same truck).
    routes = {"a": _line(0, 0, 12, 0), "b": _line(6, -6, 6, 6)}
    for held in (0.0, 30.0):
        manager = _manager(routes, issue_s=30.0, slack_s=5.0)
        manager.update(0.0, {n: r[0] for n, r in routes.items()}, {"a": 0.0, "b": 0.0})
        (c,) = manager.windows.crossings.values()
        assert c.first == "a" and not c.entered and c.deadline_s is not None
        manager.trucks["a"].held_s = held  # standing for a reason the orders do not know
        manager.windows._expire(c.deadline_s + 0.1, manager.trucks)
        assert c.first == ("b" if held else "a")


def test_standing_without_a_truck_to_wait_for_is_taken_as_held():
    routes = {"a": _line(0, 0, 12, 0), "b": _line(6, -6, 6, 6)}
    manager = _manager(routes, issue_s=30.0, slack_s=5.0, stall_s=20.0)
    for k in range(80):  # nobody moves for 8 s; a is not waiting for b
        manager.update(k * 0.1, {n: r[0] for n, r in routes.items()}, {"a": 0.0, "b": 0.0})
    a = manager.trucks["a"]
    assert a.waiting_on is None and a.moving and a.held_s == pytest.approx(7.9 - 0.0, abs=0.11)
    assert manager.windows._free(a, 5.0) == pytest.approx(5.0 + a.held_s, abs=0.2)


def test_a_truck_held_by_the_body_of_a_standing_truck_gets_a_ta():
    # b follows a down one lane; a stands for a reason the manager does not see.
    routes = {"a": _line(4, 0, 20, 0), "b": _line(0, 0, 16, 0)}
    manager = _manager(routes, stall_s=20.0, ta_s=20.0, cooldown_s=60.0)
    s = {n: arc_lengths(p) for n, p in routes.items()}
    progress = {"a": 0.0, "b": 0.0}
    advice = []
    for k in range(600):
        now = k * 0.1
        permits = manager.update(now, {n: pose_at(routes[n], s[n], progress[n]) for n in routes}, progress)
        progress["b"] = min(progress["b"] + min(SPEED * 0.1, permits["b"].allowed_m), s["b"][-1])
        advice += [(round(now, 1), a.kind, a.truck, a.other) for a in manager.advisories(now)]
    assert manager.trucks["b"].waiting_on == "a" and manager.trucks["b"].held_by_body
    assert not manager.trucks["a"].moving
    assert any(kind == "ta" and truck == "b" and other == "a" for _, kind, truck, other in advice)


def test_a_route_planned_again_keeps_clear_of_where_the_other_trucks_stand():
    routes = {"a": _line(0, 0, 12, 0), "b": _line(6, -6, 6, 6)}
    windows = _manager(routes)
    windows.update(0.0, {n: r[0] for n, r in routes.items()}, {"a": 0.0, "b": 0.0})
    keep = windows.replan_keepout("a", 0.15)
    bodies = {n: oriented_rects(np.array([r[0]]), TRUCK, 0.0) for n, r in routes.items()}
    assert overlap_matrix(bodies["b"], keep).any()  # round b
    assert not overlap_matrix(bodies["a"], keep).any()  # a's own spot is clear
    ranks = _manager(routes, mode="rank")
    ranks.update(0.0, {n: r[0] for n, r in routes.items()}, {"a": 0.0, "b": 0.0})
    assert np.array_equal(ranks.replan_keepout("a", 0.15), ranks.stop_keepout("a", 0.15))


@pytest.mark.parametrize("mode", ["windows", "queues"])
def test_a_route_planned_again_keeps_clear_of_what_the_other_trucks_hold(mode):
    # b holds the lane ahead of it: a route of a through there would meet b driving on.
    routes = {"a": _line(0, 0, 12, 0), "b": _line(6, -8, 6, 8)}
    manager = _manager(routes, mode=mode)
    for now in (0.0, 0.1):
        manager.update(now, {n: r[0] for n, r in routes.items()}, {"a": 0.0, "b": 0.0})
    b = manager.trucks["b"]
    assert b.granted_m > 2.0
    keep = manager.replan_keepout("a", 0.15)
    assert overlap_matrix(b.claim[-1:], keep).any()  # the far end of b's claim, not only its body


def test_stopping_a_truck_that_brakes_and_starts_slowly_costs_more():
    # Two trucks reach one crossing at the same time; making the one with the
    # larger stop cost (a heavy load: weak braking and starting) wait costs
    # its stop on top of the wait, so it should go first.
    g = np.arange(60) * 0.2
    trucks = {
        n: SimpleNamespace(name=n, rank=k, g=g, eta=g / 0.5, progress_m=0.0, granted_m=0.0, waiting_since_s=None,
                           restart=np.zeros(60), stopcost=np.full(60, cost), moving=True)
        for k, (n, cost) in enumerate((("light", 1.0), ("heavy", 6.0)))
    }
    w = Windows(sample_m=0.2, lookahead_m=3.0)
    need = np.full(6, g[25])
    crossings = {0: Crossing(0, "light", "heavy", (20, 25), (20, 25), need.copy(), need.copy(), "light")}
    light_first = w.cost(crossings, trucks, 0.0)
    crossings[0].first = "heavy"
    heavy_first = w.cost(crossings, trucks, 0.0)
    assert heavy_first < light_first
    assert light_first - heavy_first == pytest.approx(5.0, abs=1e-6)  # same wait, 6 s vs 1 s stop


def test_a_standing_truck_needs_time_to_get_going():
    g = np.arange(60) * 0.2
    t = SimpleNamespace(name="a", rank=0, g=g, eta=g / 0.5, progress_m=2.0, granted_m=2.0, waiting_since_s=None,
                        restart=np.full(60, 1.5), stopcost=np.zeros(60), moving=True)
    w = Windows(sample_m=0.2, lookahead_m=3.0)
    assert w._free(t, 6.0) == pytest.approx(8.0)
    t.moving = False
    assert w._free(t, 6.0) == pytest.approx(9.5)
