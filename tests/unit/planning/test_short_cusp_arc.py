"""The short curved leg before a cusp as a planner cost (docs/plans/2026-11-16-aisle-short-cusp-arc.md): the leg
rule shared by the primitives and the goal shot, the closed-set bit, the off path unchanged."""

import dataclasses
import math

import numpy as np
import pytest

from forklift_core.mission import replay_codec
from forklift_core.planning import hybrid_astar as H
from forklift_core.planning.geometry import Bounds, Footprint, Pose2D

ON = dict(short_cusp_arc_m=0.6, short_cusp_arc_penalty_m=1.0)


def cfg(**kw):
    base = dict(curvature_limit_inv_m=0.29, primitive_length_m=0.25, goal_connection="reeds_shepp")
    base.update(kw)
    return H.PlannerConfig(**base)


# --- the leg rule -------------------------------------------------------------------------------------------------

def test_a_short_curved_leg_closed_by_a_gear_change_pays_the_penalty():
    c = cfg(**ON)
    pen, leg = H.leg_step(c, (0.25, 1, 1), -1, 0, 0.25)  # 0.25 m curved forward, then reverse
    assert pen == 1.0 and leg == (0.25, 0, -1)


def test_a_long_or_straight_leg_or_the_same_gear_pays_nothing():
    c = cfg(**ON)
    assert H.leg_step(c, (0.75, 1, 1), -1, 1, 0.25)[0] == 0.0  # long enough
    assert H.leg_step(c, (0.25, 0, 1), -1, 1, 0.25)[0] == 0.0  # straight leg end
    pen, leg = H.leg_step(c, (0.25, 1, 1), 1, -1, 0.25)  # same gear: the leg goes on
    assert pen == 0.0 and leg == (0.5, -1, 1)
    assert H.leg_step(c, (0.0, 0, 0), 1, 1, 0.25) == (0.0, (0.25, 1, 1))  # the root starts a leg


def test_off_never_pays():
    c = cfg()
    assert H.leg_step(c, (0.25, 1, 1), -1, 0, 0.25)[0] == 0.0
    assert not H.short_leg(c, (0.25, 1, 1))


def test_the_shot_walks_its_segments_from_the_node_leg():
    c = cfg(**ON)
    # node: 0.25 m curved forward; shot: forward 0.2 straight (continues: leg 0.45 ending straight), reverse curved
    # 0.3 (closes the straight leg: nothing), forward 1.0 to the goal (closes the 0.3 m curved reverse: penalty)
    segs = ((1, 0.0, 0.2), (-1, 0.29, 0.3), (1, 0.0, 1.0))
    assert H.shot_leg_penalty(c, (0.25, 1, 1), segs) == pytest.approx(1.0)
    # the shot's first gear differs: the node's own short curved leg closes (penalty), the last leg ends at the goal
    assert H.shot_leg_penalty(c, (0.25, 1, 1), ((-1, 0.29, 0.2),)) == pytest.approx(1.0)
    assert H.shot_leg_penalty(c, (0.25, 1, 1), ((1, 0.29, 0.2),)) == 0.0  # same gear, the goal ends it
    assert H.shot_leg_penalty(c, (0.25, 1, 1), ((1, 0.0, 0.0), (-1, 0.0, 1.0))) == pytest.approx(1.0)  # 0-length out
    assert H.shot_leg_penalty(cfg(), (0.25, 1, 1), ((-1, 0.29, 0.2),)) == 0.0


# --- configuration and records --------------------------------------------------------------------------------------

@pytest.mark.parametrize("bad", [dict(short_cusp_arc_m=-0.1), dict(short_cusp_arc_penalty_m=-1.0),
                                 dict(short_cusp_arc_m=math.nan), dict(short_cusp_arc_penalty_m=math.inf)])
def test_bad_values_are_refused(bad):
    with pytest.raises(ValueError):
        cfg(**bad)


def test_off_fields_are_left_out_of_records():
    off = cfg()
    names = {f.name for f in replay_codec.recorded_fields(off)}
    assert "short_cusp_arc_m" not in names and "short_cusp_arc_penalty_m" not in names
    assert "short_cusp_arc_m" not in H.planner_config_record(off)
    assert H.planner_config_record(off) == {k: v for k, v in dataclasses.asdict(off).items()
                                             if k not in ("short_cusp_arc_m", "short_cusp_arc_penalty_m")}
    on = cfg(**ON)
    assert {"short_cusp_arc_m", "short_cusp_arc_penalty_m"} <= {f.name for f in replay_codec.recorded_fields(on)}
    assert H.planner_config_record(on)["short_cusp_arc_m"] == 0.6


# --- the search ----------------------------------------------------------------------------------------------------

FOOT = Footprint(0.9, 0.3, 0.3)


def short_curved_cusps(res, length_m=0.6) -> int:
    """Legs (split at gear changes, the cusp at i-1) that end at a cusp, are shorter than length_m and end curved."""
    if not res.success:
        return 0
    poses, dirs, curv = np.asarray(res.poses), np.asarray(res.directions), np.asarray(res.curvatures_inv_m)
    n, start, out = len(dirs), 0, 0
    for i in range(1, n):
        if dirs[i] != dirs[i - 1]:
            seg = poses[max(start - 1, 0):i]  # from the previous cusp (pose start-1) to this one (pose i-1)
            length = float(np.sum(np.hypot(np.diff(seg[:, 0]), np.diff(seg[:, 1])))) if len(seg) > 1 else 0.0
            if abs(curv[i - 1]) > 1e-9 and length < length_m:
                out += 1
            start = i
    return out


def _scene():
    """A 3 m wide corridor, the goal turned round 2.2 m ahead: the off search turns with short curved cusp legs (two
    at 0.25 m/s primitives, found by a scan of small scenes)."""
    bounds = Bounds(0.0, 5.0, 0.0, 3.0)
    return Pose2D(1.0, 1.5, 0.0), Pose2D(3.2, 1.6, math.pi), [], bounds


def test_the_off_search_is_unchanged_and_the_on_search_avoids_the_short_curved_cusps():
    start, goal, obstacles, bounds = _scene()
    off = H.plan_hybrid_astar(start, goal, obstacles, FOOT, bounds, cfg(max_expansions=20000))
    same = H.plan_hybrid_astar(start, goal, obstacles, FOOT, bounds,
                               cfg(max_expansions=20000, short_cusp_arc_m=0.0, short_cusp_arc_penalty_m=0.0))
    assert off.success and np.array_equal(off.poses, same.poses) and off.expanded_nodes == same.expanded_nodes
    on = H.plan_hybrid_astar(start, goal, obstacles, FOOT, bounds, cfg(max_expansions=20000, **ON))
    assert on.success and short_curved_cusps(off) == 2 and short_curved_cusps(on) == 0


def test_with_no_other_way_the_short_curved_cusp_stays():
    """A cost, not a ban: the same scene with a penalty far above any detour still plans."""
    start, goal, obstacles, bounds = _scene()
    huge = H.plan_hybrid_astar(start, goal, obstacles, FOOT, bounds,
                               cfg(max_expansions=20000, short_cusp_arc_m=5.0, short_cusp_arc_penalty_m=100.0))
    assert huge.success  # every curved leg before a cusp is "short" at 5 m, so some stay


def test_the_on_search_is_deterministic():
    start, goal, obstacles, bounds = _scene()
    a = H.plan_hybrid_astar(start, goal, obstacles, FOOT, bounds, cfg(max_expansions=20000, **ON))
    b = H.plan_hybrid_astar(start, goal, obstacles, FOOT, bounds, cfg(max_expansions=20000, **ON))
    assert np.array_equal(a.poses, b.poses) and a.expanded_nodes == b.expanded_nodes


def test_the_threshold_is_strict():
    c = cfg(**ON)
    assert not H.short_leg(c, (0.6, 1, 1)) and H.short_leg(c, (0.59, 1, 1)) and H.short_leg(c, (0.5, -1, -1))


def test_a_turn_cycle_child_pays_each_closed_leg_once(monkeypatch):
    """D7b cycles: the first half closes the node's short curved leg (pen1), the cycle's own gear change closes the
    first half's 0.25 m curved leg (pen2); the child's cost holds each once (cost2 is built on cost1 before pen1)."""
    start, goal, obstacles, bounds = _scene()
    seen = []
    real = H.leg_step

    def spy(config, leg, direction, steer, length):
        out = real(config, leg, direction, steer, length)
        seen.append(out[0])
        return out

    monkeypatch.setattr(H, "leg_step", spy)
    on = cfg(max_expansions=3000, turn_cycles=True, turn_cycle_m=0.25, **ON)
    res = H.plan_hybrid_astar(start, goal, obstacles, FOOT, bounds, on)
    off = H.plan_hybrid_astar(start, goal, obstacles, FOOT, bounds, cfg(max_expansions=3000, turn_cycles=True,
                                                                         turn_cycle_m=0.25))
    assert res.success and off.success and any(p > 0 for p in seen)
    import inspect
    src = inspect.getsource(H.plan_hybrid_astar)
    i, j = src.index("cost2 = cost1 + length"), src.index("cost1 += pen1")
    assert i < j and "cost2 += pen1 + pen2" in src[j:]
