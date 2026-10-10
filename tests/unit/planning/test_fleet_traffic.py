import math

import numpy as np
import pytest

from forklift_core.planning.fleet_traffic import (
    TrafficManager,
    overlap_matrix,
    oriented_rects,
    arc_lengths,
    footprint_rectangle,
    pose_at,
    rectangles_overlap,
    tile_rects,
)
from forklift_core.planning.geometry import Footprint

TRUCK = Footprint(1.29, 0.17, 0.36)


def _line(x0, y0, x1, y1, step=0.1):
    n = max(2, int(round(math.hypot(x1 - x0, y1 - y0) / step)) + 1)
    yaw = math.atan2(y1 - y0, x1 - x0)
    return np.column_stack(
        (np.linspace(x0, x1, n), np.linspace(y0, y1, n), np.full(n, yaw))
    )


def _manager(routes, **kwargs):
    manager = TrafficManager(**kwargs)
    for rank, (name, route) in enumerate(routes.items()):
        manager.add(name, rank, route[0], TRUCK)
    for name, route in routes.items():
        manager.set_route(name, [(route, TRUCK)])
    return manager


def _drive(manager, routes, speed=0.5, dt=0.1, steps=600, check=None):
    """Kinematic followers: each advances min(speed*dt, allowed) along its route."""
    s = {n: arc_lengths(p) for n, p in routes.items()}
    progress = {n: 0.0 for n in routes}
    overlapped = False
    log = []
    for k in range(steps):
        now = k * dt
        poses = {n: pose_at(routes[n], s[n], progress[n]) for n in routes}
        permits = manager.update(now, poses, progress)
        if check is not None:
            check(manager)
        bodies = {n: footprint_rectangle(poses[n], TRUCK) for n in routes}
        names = sorted(routes)
        for i, a in enumerate(names):
            for b in names[i + 1 :]:
                overlapped |= rectangles_overlap(bodies[a], bodies[b])
        for n in routes:
            progress[n] += min(speed * dt, permits[n].allowed_m)
        log.append((now, dict(progress), {n: p.blocked_by for n, p in permits.items()}))
    return progress, overlapped, log


def test_arc_length_and_interpolation():
    path = _line(0, 0, 2, 0)
    s = arc_lengths(path)
    assert s[-1] == pytest.approx(2.0)
    assert pose_at(path, s, 0.55)[0] == pytest.approx(0.55)
    assert pose_at(path, s, 9.0)[0] == pytest.approx(2.0)


def test_vectorized_overlaps_agree_with_the_rectangle_check():
    rng = np.random.default_rng(3)
    a = rng.uniform([-3, -3, -np.pi], [3, 3, np.pi], (60, 3))
    b = rng.uniform([-3, -3, -np.pi], [3, 3, np.pi], (50, 3))
    other = Footprint(0.9, 0.4, 0.5)
    touch = overlap_matrix(oriented_rects(a, TRUCK, 0.1), oriented_rects(b, other, 0.0))
    expected = np.array(
        [
            [
                rectangles_overlap(
                    footprint_rectangle(p, TRUCK, 0.1), footprint_rectangle(q, other)
                )
                for q in b
            ]
            for p in a
        ]
    )
    assert touch.any() and not touch.all()
    assert np.array_equal(touch, expected)


def test_route_chunks_are_numbered_end_to_end():
    manager = TrafficManager(sample_m=0.2)
    manager.add("a", 0, (0, 0, 0), TRUCK)
    offsets = manager.set_route(
        "a", [(_line(0, 0, 3, 0), TRUCK), (_line(3, 0, 3, 2), TRUCK)]
    )
    assert offsets == pytest.approx([0.0, 3.0])
    assert manager.trucks["a"].g[-1] == pytest.approx(5.0)


def test_crossing_trucks_take_turns_and_never_touch():
    routes = {"a": _line(0, 0, 12, 0), "b": _line(6, -6, 6, 6)}
    manager = _manager(routes, margin_m=0.3, lookahead_m=3.0)
    progress, overlapped, log = _drive(manager, routes)
    assert not overlapped
    assert progress["a"] == pytest.approx(12.0)
    assert progress["b"] == pytest.approx(12.0)
    assert any(blocked["b"] == "a" for _, _, blocked in log)
    assert manager.route_conflicts() == []


def test_claims_of_different_trucks_never_overlap():
    routes = {
        "a": _line(0, 0, 12, 0),
        "b": _line(6, -6, 6, 6),
        "c": _line(12, 1.5, 0, 1.5),
    }
    manager = _manager(routes, margin_m=0.3, lookahead_m=4.0)

    def check(m):
        trucks = list(m.trucks.values())
        for i, t in enumerate(trucks):
            for u in trucks[i + 1 :]:
                assert not overlap_matrix(t.claim[1:], u.claim[1:]).any(), (t.name, u.name)

    progress, overlapped, _ = _drive(manager, routes, speed=0.5, steps=500, check=check)
    assert not overlapped
    assert all(progress[n] == pytest.approx(12.0) for n in routes)


def test_a_grant_is_not_taken_back_by_a_higher_rank():
    manager = TrafficManager(margin_m=0.2, lookahead_m=6.0)
    low, high = _line(0, 0, 10, 0), _line(5, -6, 5, 6)
    manager.add("high", 0, (5, -20, math.pi / 2), TRUCK)
    manager.add("low", 5, low[0], TRUCK)
    manager.set_route("low", [(low, TRUCK)])
    manager.update(
        0.0, {"low": low[0], "high": (5, -20, math.pi / 2)}, {"low": 0, "high": 0}
    )
    granted = manager.trucks["low"].granted_m
    assert granted == pytest.approx(6.0)
    manager.set_route("high", [(high, TRUCK)])
    permits = manager.update(
        0.1, {"low": low[0], "high": high[0]}, {"low": 0, "high": 0}
    )
    assert manager.trucks["low"].granted_m == pytest.approx(granted)
    assert permits["high"].blocked_by == "low"


def _junction():
    # b turns from a side corridor into a's lane against a's direction.
    a = _line(0, 0, 20, 0)
    b = np.vstack((_line(10, -8, 10, 0), _line(10, 0, -6, 0)[1:]))
    return {"a": a, "b": b}


def test_on_claims_alone_the_junction_locks(monkeypatch):
    # b claims the junction first, turns into a's lane and stops facing a.
    monkeypatch.setattr(
        TrafficManager, "_unsafe", lambda self, t: np.zeros(len(t.g), dtype=bool)
    )
    routes = _junction()
    manager = _manager(routes, margin_m=0.3, lookahead_m=3.0, cycle_s=2.0)
    progress, overlapped, _ = _drive(manager, routes, steps=900)
    assert not overlapped
    assert progress["a"] < 20 and progress["b"] < arc_lengths(routes["b"])[-1]
    assert manager.deadlocks(900 * 0.1) == [("b", "a")]


def test_a_lower_rank_never_stops_on_a_higher_route_and_a_junction_clears():
    # Under the safe-stop rule b waits short of the junction until a has gone
    # through, instead of the lock above.
    routes = _junction()
    b = routes["b"]
    manager = _manager(routes, margin_m=0.3, lookahead_m=3.0)

    def check(m):
        t = m.trucks["b"]
        if t.granted_m > 0:
            end = int(np.searchsorted(t.g, t.granted_m - 1e-9))
            assert not m._unsafe(t)[end]

    progress, overlapped, log = _drive(manager, routes, steps=900, check=check)
    assert not overlapped
    assert progress["a"] == pytest.approx(20.0)
    assert progress["b"] == pytest.approx(arc_lengths(b)[-1])
    waited = [p["b"] for _, p, blocked in log if blocked["b"] == "a"]
    assert waited and max(waited) < 8.0 - 1.29  # held short of a's lane
    assert manager.deadlocks(900 * 0.1) == []


def test_a_start_on_a_higher_route_is_reported_and_its_lock_seen():
    routes = {"a": _line(0, 0, 10, 0), "b": _line(10, 0, 0, 0)}
    manager = _manager(routes, margin_m=0.3, lookahead_m=3.0, cycle_s=2.0)
    assert {"truck": "b", "pose": "current", "route_of": "a"} in (
        manager.route_conflicts()
    )
    progress, overlapped, _ = _drive(manager, routes, steps=200)
    assert not overlapped
    assert progress["a"] < 10 and progress["b"] < 10
    assert manager.deadlocks(200 * 0.1) == [("b", "a")]


def test_waiting_on_a_standing_truck_becomes_a_stall_after_stall_s():
    path = _line(0, 0, 10, 0)
    manager = TrafficManager(margin_m=0.3, lookahead_m=3.0, stall_s=5.0)
    manager.add("mover", 0, path[0], TRUCK)
    manager.add("standing", 1, (6.0, 0.0, 0.0), TRUCK)
    manager.set_route("mover", [(path, TRUCK)])
    assert manager.route_conflicts() == [
        {"truck": "standing", "pose": "current", "route_of": "mover"}
    ]
    s = arc_lengths(path)
    progress = 0.0
    for k in range(200):
        permits = manager.update(
            k * 0.1,
            {"mover": pose_at(path, s, progress), "standing": (6.0, 0.0, 0.0)},
            {"mover": progress, "standing": 0.0},
        )
        progress += min(0.05, permits["mover"].allowed_m)
    assert permits["mover"].blocked_by == "standing"
    assert manager.deadlocks(200 * 0.1) == [("mover", "standing")]


def test_pruned_cover_tables_match_the_full_calculation():
    rng = np.random.default_rng(5)
    manager = TrafficManager(margin_m=0.3)
    routes = {}
    for rank in range(4):
        corners = rng.uniform(-15, 15, (4, 2))
        route = np.vstack(
            [_line(*a, *b) for a, b in zip(corners[:-1], corners[1:], strict=True)]
        )
        routes[f"t{rank}"] = route
        manager.add(f"t{rank}", rank, route[0], TRUCK)
    for name, route in routes.items():
        manager.set_route(name, [(route, TRUCK)])
    trucks = manager.trucks
    for (low, high), cover in manager._cover.items():
        touch = overlap_matrix(trucks[low].rects, trucks[high].rects)
        touch |= overlap_matrix(trucks[low].rest, trucks[high].rects)
        full = np.where(touch, trucks[high].g[None, :], -np.inf).max(axis=1)
        assert np.array_equal(cover, full), (low, high)
    assert any(np.isfinite(c).any() for c in manager._cover.values())


def test_a_truck_resting_a_little_short_of_its_grant_end_is_off_a_higher_route():
    # a (rank 0) stands at the head of the lane y = 0; b crosses it and c
    # stands just beyond, so b can stop only at its first sample past the
    # lane, 5 mm clear of it. b comes to rest 1 cm short, like the world
    # followers: there it would stand on a's route, so it must not be granted
    # through the lane and waits before it instead.
    a = _line(-20, 0, 20, 0)
    b = _line(0, -4.865, 0, 10)  # its sample 30 sits at y = 1.135
    manager = TrafficManager(margin_m=0.3, lookahead_m=3.0)
    manager.add("a", 0, a[0], TRUCK)
    manager.add("b", 1, b[0], TRUCK)
    manager.add("c", 2, (0.0, 3.3, math.pi / 2), TRUCK)
    manager.set_route("a", [(a, TRUCK)])
    manager.set_route("b", [(b, TRUCK)])
    s, progress = arc_lengths(b), 0.0
    for k in range(400):
        poses = {"a": a[0], "b": pose_at(b, s, progress), "c": (0.0, 3.3, math.pi / 2)}
        permits = manager.update(k * 0.1, poses, {"a": 0.0, "b": progress, "c": 0.0})
        progress += min(0.05, max(permits["b"].allowed_m - 0.01, 0.0))
    assert not manager.on_route("b", "a")
    assert manager.trucks["b"].waiting_on is not None
    assert pose_at(b, s, progress)[1] < -2.0


def test_a_new_rank_moves_a_truck_behind_the_others():
    # b goes first on fixed ranks; given a new job (rank 9) it must keep off
    # a's route instead, so it now waits short of the junction.
    routes = _junction()
    manager = TrafficManager(margin_m=0.3, lookahead_m=3.0)
    manager.add("b", 0, routes["b"][0], TRUCK)
    manager.add("a", 1, routes["a"][0], TRUCK)
    manager.set_route("a", [(routes["a"], TRUCK)])
    manager.set_route("b", [(routes["b"], TRUCK)], rank=9)
    assert manager.trucks["b"].rank == 9
    assert ("b", "a") in manager._cover and ("a", "b") not in manager._cover
    progress, overlapped, log = _drive(manager, routes, steps=900)
    assert not overlapped
    waited = [p["b"] for _, p, blocked in log if blocked["b"] == "a"]
    assert waited and max(waited) < 8.0 - 1.29
    assert progress["b"] == pytest.approx(arc_lengths(routes["b"])[-1])


# -- TA/RA advice (plan 2026-10-10) ---------------------------------------------


def test_a_head_on_lock_gets_one_ra_for_the_lower_rank_only():
    # b (rank 1) stands on a's route facing it: the lock fleet_traffic reports.
    routes = {"a": _line(0, 0, 10, 0), "b": _line(10, 0, 0, 0)}
    manager = _manager(routes, margin_m=0.3, lookahead_m=3.0, cycle_s=2.0)
    _drive(manager, routes, steps=200)
    advice = manager.advisories(200 * 0.1)
    assert [(a.kind, a.truck, a.other, a.why) for a in advice] == [("ra", "b", "a", "cycle")]
    assert manager.advisories(200 * 0.1 + 1.0) == []  # once per wait


def test_a_standing_truck_gets_a_ta_and_a_parked_one_is_never_asked_to_move():
    path = _line(0, 0, 10, 0)
    manager = TrafficManager(margin_m=0.3, lookahead_m=3.0, ta_s=3.0, ra_s=6.0, cooldown_s=1.0)
    manager.add("mover", 0, path[0], TRUCK)
    manager.add("parked", 1, (6.0, 0.0, 0.0), TRUCK)  # no route: it stays there
    manager.set_route("mover", [(path, TRUCK)])
    s, progress, seen = arc_lengths(path), 0.0, []
    for k in range(120):
        permits = manager.update(
            k * 0.1,
            {"mover": pose_at(path, s, progress), "parked": (6.0, 0.0, 0.0)},
            {"mover": progress, "parked": 0.0},
        )
        progress += min(0.05, permits["mover"].allowed_m)
        seen += [(round(k * 0.1, 1), a.kind, a.truck, a.why) for a in manager.advisories(k * 0.1)]
    assert len(seen) == 1 and seen[0][1:] == ("ta", "mover", "stall")


def test_the_wall_is_the_higher_route_near_the_truck_without_its_own_spot():
    routes = {"a": _line(0, 0, 30, 0)}
    manager = _manager(routes, margin_m=0.3)
    manager.add("b", 1, (12.0, 0.0, math.pi), TRUCK)  # standing in a's lane
    wall = manager.wall("b", "a", radius_m=8.0, keep_m=0.15)
    assert len(wall)
    here = oriented_rects(np.array([[12.0, 0.0, math.pi]]), TRUCK, 0.15)
    assert not overlap_matrix(wall, here).any()

    def covered(x, y):
        dot = np.array([[x, y, 0.01, 0.01, 1.0, 0.0]])
        return bool(overlap_matrix(dot, wall).any())

    assert covered(6.0, 0.0) and covered(17.0, 0.0)  # a's lane either side
    assert covered(6.0, 0.9)  # grown by twice the margin (0.36 + 0.6)
    assert not covered(6.0, 1.1)
    assert not covered(25.0, 0.0)  # beyond the radius


def test_back_to_is_the_nearest_spot_off_every_higher_route():
    # a (rank 0) will drive x 10..30; b (rank 1) came along x 0..15 and stands on it.
    routes = {"a": _line(10, 0, 30, 0), "b": _line(0, 0, 20, 0)}
    manager = _manager(routes, margin_m=0.3)
    back = manager.back_to("b", 0.0, 14.5)
    # b's sample at g is clear of a's grown route while g + 1.29 + 0.3 < 10 - 0.17 - 0.3.
    assert back is not None and 7.5 < back < 10 - 0.17 - 0.6 - 1.29
    assert manager.back_to("b", 9.0, 14.5) is None


def test_an_ra_truck_is_cleared_once_off_the_route_and_not_waiting():
    routes = {"a": _line(0, 0, 30, 0)}
    manager = _manager(routes, margin_m=0.3)
    manager.add("b", 1, (12.0, 0.0, math.pi), TRUCK)
    manager.resolved("b", "a")
    manager.update(0.0, {"a": routes["a"][0], "b": (12.0, 0.0, math.pi)}, {"a": 0.0, "b": 0.0})
    assert not [a for a in manager.advisories(0.0) if a.kind == "cc"]
    manager.update(0.1, {"a": routes["a"][0], "b": (12.0, 3.0, math.pi)}, {"a": 0.0, "b": 0.0})
    assert [(a.kind, a.truck, a.other) for a in manager.advisories(0.1)] == [("cc", "b", "a")]
    assert "b" not in manager.ra_against


def test_tiles_cover_the_rect_and_stay_inside_it():
    rect = oriented_rects(np.array([[1.0, 2.0, 0.7]]), TRUCK, 0.3)
    pieces = tile_rects(rect, 0.4)
    assert len(pieces) > 4
    rng = np.random.default_rng(0)
    c, s = rect[0, 4], rect[0, 5]
    for u, v in rng.uniform(-1, 1, (200, 2)) * rect[0, 2:4] * 0.999:
        x, y = rect[0, 0] + u * c - v * s, rect[0, 1] + u * s + v * c
        assert overlap_matrix(np.array([[x, y, 1e-4, 1e-4, 1.0, 0.0]]), pieces).any()
    assert np.isclose((4 * pieces[:, 2] * pieces[:, 3]).sum(), 4 * rect[0, 2] * rect[0, 3])


def test_a_lock_the_ra_left_gets_a_ta_after_ta_s():
    routes = {"a": _line(0, 0, 10, 0), "b": _line(10, 0, 0, 0)}
    manager = _manager(routes, margin_m=0.3, lookahead_m=3.0, cycle_s=2.0, ta_s=10.0, cooldown_s=60.0)
    seen = []

    def check(m):
        now = m.trucks["a"].waiting_since_s
        if now is not None:
            seen.extend((a.kind, a.truck, a.why) for a in m.advisories(check.t))
        check.t += 0.1

    check.t = 0.0
    _drive(manager, routes, steps=300, check=check)
    # b (lower) gets the RA; nobody moves it here, so a then gets a TA, and b,
    # still in its cooldown, does not.
    assert seen == [("ra", "b", "cycle"), ("ta", "a", "stall")]


def test_a_failed_ra_comes_again_after_the_cooldown_in_the_same_wait():
    routes = {"a": _line(0, 0, 10, 0), "b": _line(10, 0, 0, 0)}
    manager = _manager(routes, margin_m=0.3, lookahead_m=3.0, cycle_s=2.0, cooldown_s=5.0)
    _drive(manager, routes, steps=200)
    (advice,) = manager.advisories(20.0)
    manager.failed(advice)
    assert manager.advisories(21.0) == []  # cooldown
    assert [(a.kind, a.truck, a.why) for a in manager.advisories(26.0)] == [("ra", "b", "cycle")]


def test_a_new_route_may_not_run_over_where_a_lower_truck_stops():
    # a (rank 0) is to drive x 0..10; b (rank 1) stands at (5, 3) facing east.
    routes = {"a": _line(0, 0, 10, 0)}
    manager = _manager(routes, margin_m=0.3)
    manager.add("b", 1, (5.0, 3.0, 0.0), TRUCK)
    manager.update(0.0, {"a": routes["a"][0], "b": (5.0, 3.0, 0.0)}, {"a": 0.0, "b": 0.0})
    names, _ = manager.lower_stops("a")
    assert set(names) == {"b"} and manager.lower_stops("b")[1].shape == (0, 6)
    assert manager.path_conflicts("a", _line(0, 0, 10, 0)) == []
    over = np.vstack((_line(0, 0, 4, 3)[:-1], _line(4, 3, 10, 3)))
    assert manager.path_conflicts("a", over) == ["covers_lower"]
    keepout = manager.stop_keepout("a", keep_m=0.15)
    centre = 5.0 + (1.29 - 0.17) / 2

    def covered(x, y):
        return bool(overlap_matrix(np.array([[x, y, 0.01, 0.01, 1.0, 0.0]]), keepout).any())

    assert covered(centre, 3.0) and covered(centre, 3.0 - 0.36 - 0.55)  # grown by 2 x margin
    assert not covered(centre, 3.0 - 0.36 - 0.65)
    # b, the lower one, runs into a's claim: it would wait for a again.
    assert manager.path_conflicts("b", _line(5, 3, 0.5, 0.2), other="a") == ["meets_other"]


def test_a_moving_lower_truck_stops_at_the_end_of_its_grant_not_where_it_is():
    routes = {"a": _line(0, -5, 0, -4), "b": _line(0, 5, 20, 5)}
    manager = _manager(routes, margin_m=0.3, lookahead_m=3.0)
    manager.update(0.0, {n: r[0] for n, r in routes.items()}, {"a": 0.0, "b": 0.0})
    granted = manager.trucks["b"].granted_m
    assert manager.trucks["b"].moving and granted >= 3.0
    offset = (1.29 - 0.17) / 2
    (stop,) = manager.lower_stops("a")[1]
    assert stop[0] == pytest.approx(granted + offset, abs=0.21) and stop[1] == pytest.approx(5.0)


def test_a_truck_kept_off_a_route_gets_an_ra_not_a_ta():
    # a (rank 0) waits behind c (parked, rank 2) at the foot of a long lane;
    # b (rank 1) stands in that lane and has nowhere in it to stop: it waits
    # for a's route, not for a's body, so going round a's body would not help.
    routes = {"a": _line(0, -30, 0, 30), "b": _line(0.5, -10, 0.5, 30)}
    manager = _manager(routes, margin_m=0.3, ta_s=3.0, ra_s=6.0, cooldown_s=1.0)
    manager.add("c", 2, (0.0, -27.9, math.pi / 2), TRUCK)
    poses = {"a": routes["a"][0], "b": routes["b"][0], "c": (0.0, -27.9, math.pi / 2)}
    seen = []
    for k in range(100):
        manager.update(k * 0.1, poses, {"a": 0.0, "b": 0.0, "c": 0.0})
        seen += [(a.kind, a.truck, a.other, a.why) for a in manager.advisories(k * 0.1)]
    assert manager.trucks["b"].waiting_on == "a" and not manager.trucks["b"].held_by_body
    assert seen == [("ta", "a", "c", "stall"), ("ra", "b", "a", "long_wait")]
