import math

import numpy as np
import pytest

from forklift_core.planning.fleet_traffic import arc_lengths, pose_at
from forklift_core.planning.hybrid_astar import PlanResult, _advance
from forklift_core.planning.world_resolution import (
    make_room,
    rectangles,
    rest_of_path,
    reverse_path,
)


def _arcs(pieces, step=0.05):
    """A path of (gear, curvature, length) pieces driven from the origin, as the
    planner builds them: yaw changes by curvature per signed metre."""
    poses, gears, curvatures = [np.zeros(3)], [pieces[0][0]], [pieces[0][1]]
    for gear, curvature, length in pieces:
        for _ in range(int(round(length / step))):
            poses.append(np.asarray(_advance(tuple(poses[-1]), gear * step, curvature)))
            gears.append(gear)
            curvatures.append(curvature)
    poses = np.asarray(poses)
    return PlanResult(True, "success", poses, np.asarray(gears, np.int8), np.asarray(curvatures), 0.0, 0)


def _replay(result):
    """Drive result's controls from its first pose."""
    out = [np.asarray(result.poses[0], float)]
    for i in range(1, len(result.poses)):
        ds = float(np.hypot(*(result.poses[i, :2] - result.poses[i - 1, :2])))
        out.append(np.asarray(_advance(tuple(out[-1]), int(result.directions[i]) * ds, float(result.curvatures_inv_m[i]))))
    return np.asarray(out)


@pytest.mark.parametrize("pieces", [[(1, 0.25, 4.0)], [(1, -0.2, 2.0), (1, 0.0, 1.0), (-1, 0.3, 2.0)]])
def test_a_reversed_path_retraces_the_curve_with_the_gear_flipped(pieces):
    path = _arcs(pieces)
    s = arc_lengths(path.poses)
    back = reverse_path(path, s[-1] - 0.33, 0.47)
    assert np.allclose(back.poses[0], pose_at(path.poses, s, s[-1] - 0.33))
    assert np.allclose(back.poses[-1], pose_at(path.poses, s, 0.47))
    assert back.length_m == pytest.approx(s[-1] - 0.33 - 0.47)
    # The flipped gears with the same curvatures drive back along it (the end
    # points are interpolated, hence the tolerance); flipped curvatures do not.
    assert np.allclose(_replay(back), back.poses, atol=1e-3)
    wrong = PlanResult(True, "", back.poses, back.directions, -back.curvatures_inv_m, 0.0, 0)
    assert not np.allclose(_replay(wrong), back.poses, atol=1e-2)
    assert set(back.directions.tolist()) == {-g for g, _, _ in pieces}


def test_a_short_reverse_inside_one_segment():
    path = _arcs([(1, 0.1, 2.0)])
    back = reverse_path(path, 1.04, 1.01)  # samples every 0.05 m
    assert len(back.poses) == 2 and back.directions.tolist() == [-1, -1]
    with pytest.raises(ValueError):
        reverse_path(path, 0.5, 0.9)


def test_rects_become_planner_rectangles():
    (r,) = rectangles(np.array([[1.0, 2.0, 0.5, 0.25, math.cos(0.3), math.sin(0.3)]]))
    assert (r.x_m, r.y_m, r.length_m, r.width_m) == (1.0, 2.0, 1.0, 0.5)
    assert r.yaw_rad == pytest.approx(0.3)


def test_wait_stats_counts_waits_and_cycles_of_any_length():
    from forklift_core.planning.world_resolution import wait_stats

    blocked = np.full((400, 4), -1)
    blocked[10:80, 0] = 1  # a 70 s wait
    blocked[100:420, 1] = 2  # a 300 s wait to the end
    blocked[200:212, 2] = 3  # 1 -> 2 -> 3 -> 1 for 12 s: a three-truck cycle
    blocked[200:212, 3] = 1
    stats = wait_stats(blocked)
    assert stats["over_60"] == 2 and stats["over_300"] == 1 and stats["max_s"] == 300
    assert stats["cycles"] == 1 and stats["cycle_max_s"] == 12


def test_own_slots_holding_a_pallet_again_keep_a_new_route_out():
    from types import SimpleNamespace

    from forklift_core.planning.fleet_traffic import TrafficManager
    from forklift_core.planning.geometry import Footprint
    from forklift_core.planning.pallet_mission import PalletSite
    from forklift_core.planning.world_resolution import own_pallets_keepout

    # Source pallet at the origin (0.8 x 0.6 m across y; 0.3 m slot keep-out),
    # sink far off. The truck has backed out to y = 2 facing the source.
    geometry = SimpleNamespace(pallet_depth_m=0.6, pallet_width_m=0.8)
    job = SimpleNamespace(
        source=SimpleNamespace(name="S", site=PalletSite(0.0, 0.0, -math.pi / 2)),
        sink=SimpleNamespace(name="D", site=PalletSite(20.0, 0.0, -math.pi / 2)),
    )
    manager = TrafficManager(margin_m=0.3)
    manager.add("t", 0, (0.0, 2.0, -math.pi / 2), Footprint(1.29, 0.17, 0.36))

    def keepout(stage, occupied):
        return own_pallets_keepout(manager, "t", job, stage, occupied, geometry, keep_m=0.15)

    def covers(rects, x, y):
        return any(
            abs(x - r.x_m) <= r.length_m / 2 + 1e-9 and abs(y - r.y_m) <= r.width_m / 2 + 1e-9
            for r in rects
        )

    both = {"S": True, "D": True}
    assert keepout("to_pickup", both) == [] and keepout("lift", both) == []
    assert keepout("to_drop", {"S": False, "D": True}) == []
    restocked = keepout("to_drop", {"S": True, "D": False})
    # The grown slot is kept out, the pallet's near edge (y 0.3) too; only a
    # sliver within keep_m of the truck's front (at y 0.71) is left out.
    assert covers(restocked, 0.0, -0.5) and covers(restocked, 0.6, 0.3) and covers(restocked, 0.0, 0.4)
    assert not covers(restocked, 0.0, 0.58) and not covers(restocked, 20.0, 0.0)
    home = keepout("to_holding", both)
    assert covers(home, 20.0, 0.0) and covers(home, 0.0, -0.3)


def test_the_rest_of_a_path_starts_where_asked_and_drives_the_same_controls():
    path = _arcs([(1, -0.2, 2.0), (1, 0.0, 1.0), (-1, 0.3, 2.0)])
    s = arc_lengths(path.poses)
    rest = rest_of_path(path, 1.33)
    assert np.allclose(rest.poses[0], pose_at(path.poses, s, 1.33))
    assert np.allclose(rest.poses[-1], path.poses[-1])
    assert rest.length_m == pytest.approx(s[-1] - 1.33)
    assert np.allclose(_replay(rest), rest.poses, atol=1e-3)
    with pytest.raises(ValueError):
        rest_of_path(path, float(s[-1]))


def _room(stage_gear=1, behind=(), static=()):
    """A truck 5 m along a 10 m straight stage on +x (gear stage_gear), others
    standing at the poses in behind; make_room with up to 2.5 m."""
    from forklift_core.planning.fleet_traffic import TrafficManager
    from forklift_core.planning.geometry import Footprint

    truck = Footprint(1.29, 0.17, 0.36)
    poses = np.column_stack((np.linspace(0.0, 10.0, 101), np.zeros(101), np.zeros(101)))
    n = len(poses)
    path = PlanResult(True, "success", poses, np.full(n, stage_gear, np.int8), np.zeros(n), 10.0, 0)
    manager = TrafficManager(margin_m=0.3, mode="windows")
    manager.add("t", 0, (5.0, 0.0, 0.0), truck)
    for k, pose in enumerate(behind):
        manager.add(f"o{k}", k + 1, pose, truck)
    manager.update(0.0, {"t": (5.0, 0.0, 0.0), **{f"o{k}": p for k, p in enumerate(behind)}}, {"t": 0.0, **{f"o{k}": 0.0 for k in range(len(behind))}})
    return make_room(manager, "t", path, 5.0, stage_gear, 2.5, np.asarray(static, float).reshape(-1, 6))


def test_a_truck_held_by_its_field_backs_straight_off_and_comes_back():
    res = _room()
    assert res.how == "back"
    p = res.path
    turn = int(np.argmin(p.poses[:, 0]))
    assert p.poses[turn, 0] == pytest.approx(2.5) and np.allclose(p.poses[:, 1:], 0.0)
    assert set(p.directions[1 : turn + 1].tolist()) == {-1} and set(p.directions[turn + 1 :].tolist()) == {1}
    assert np.allclose(p.poses[-1], (10.0, 0.0, 0.0))
    assert p.length_m == pytest.approx(2.5 + 2.5 + 5.0) and res.left_m == pytest.approx(5.0)
    # Driving in reverse with the person behind, it backs off forward (the
    # first 26 poses: 0.1 m steps to 2.5 m).
    rev = _room(stage_gear=-1)
    assert rev.path.poses[25, 0] == pytest.approx(7.5) and set(rev.path.directions[1:26].tolist()) == {1}


def test_making_room_stops_short_of_what_stands_behind_and_fails_with_no_room():
    # A box at x 3.0-3.4: the body (rear 0.17 m behind the pose) clears it
    # from a 1 m back-off on (0.5 m steps from 2.5 m).
    box = [3.2, 0.0, 0.2, 0.5, 1.0, 0.0]
    assert _room(static=[box]).path.poses[:, 0].min() == pytest.approx(4.0)
    # Another truck behind at x 1 (its claim to x 2.59 with the margin; ours
    # reaches 0.47 m behind the pose): 1.5 m.
    assert _room(behind=[(1.0, 0.0, 0.0)]).path.poses[:, 0].min() == pytest.approx(3.5)
    tight = _room(static=[[4.7, 0.0, 0.1, 0.5, 1.0, 0.0]])
    assert tight.how == "failed" and tight.path is None
