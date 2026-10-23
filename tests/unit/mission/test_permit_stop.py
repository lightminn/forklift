"""P6 (docs/plans/2026-10-23-planner-permission-clearance.md 3 P6): a permission stop in a scene layout -- the limit
0 for an occupied cell, held 3 s on a travel leg -- raises the scene-mode replan device's request with the blocked
cells as obstacles and the clearance 0.10 m wider; the cells become merged cell rectangles of the occupied region
they belong to."""

from types import SimpleNamespace

import numpy as np
import pytest
from test_core_scene_replan import make  # the device's core fixtures

from forklift_core.mission.failures import MISSION, classify
from forklift_core.mission.permit_stop import (
    PERMIT_STOP_CLEARANCE_ADD_M,
    PERMIT_STOP_S,
    blocked_rectangles,
)
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, GridSnapshot


def snapshot(occupied_cells, shape=(40, 40), origin=(-1.0, -1.0), res=0.05):
    state = np.full(shape, FREE, dtype=np.uint8)
    for i, j in occupied_cells:
        state[i, j] = OCCUPIED
    return GridSnapshot(state, np.zeros(shape), 0.0, (0.0, 0.0, 0.0), 0, 1, {}, origin[0], origin[1], res)


def cells_of(rects, snap):
    """The grid cells whose centres the rectangles cover."""
    out = set()
    for r in rects:
        assert r.yaw_rad == 0.0
        x0, x1 = r.x_m - r.length_m / 2, r.x_m + r.length_m / 2
        y0, y1 = r.y_m - r.width_m / 2, r.y_m + r.width_m / 2
        nx, ny = snap.state.shape
        for i in range(nx):
            for j in range(ny):
                cx = snap.origin_x_m + (i + 0.5) * snap.resolution_m
                cy = snap.origin_y_m + (j + 0.5) * snap.resolution_m
                if x0 < cx < x1 and y0 < cy < y1:
                    out.add((i, j))
    return out


def test_the_blocked_cells_grow_over_their_occupied_region_and_merge_into_rectangles():
    wall = {(i, 20) for i in range(5, 25)} | {(i, 21) for i in range(5, 25)}  # a 1.0 x 0.1 m wall
    other = {(30, 5), (31, 5)}  # another obstacle, not touching it
    snap = snapshot(wall | other)
    rects = blocked_rectangles(snap, [(12, 20)], grow_m=2.0)
    assert cells_of(rects, snap) == wall  # the whole connected wall, nothing else
    assert len(rects) <= 2  # merged, not one per cell


def test_the_growth_stops_at_its_radius():
    wall = {(i, 20) for i in range(0, 40)}
    snap = snapshot(wall)
    got = cells_of(blocked_rectangles(snap, [(20, 20)], grow_m=0.5), snap)
    assert (20, 20) in got and {(10, 20), (30, 20)} <= got and (9, 20) not in got and (31, 20) not in got


def test_no_blocked_cells_give_no_obstacles_and_unoccupied_seeds_are_kept_as_cells():
    snap = snapshot(set())
    assert blocked_rectangles(snap, [], grow_m=0.5) == []
    # a seed the snapshot no longer marks (an unknown block) still stands for itself
    assert cells_of(blocked_rectangles(snap, [(3, 4)], grow_m=0.5), snap) == {(3, 4)}


class Layer:
    def __init__(self, snap, path_cells=(), estop_cells=()):
        self.permission = SimpleNamespace(snapshot=snap, path_check=SimpleNamespace(blocked_cells=tuple(path_cells)),
                                          last_estop=SimpleNamespace(blocked_cells=tuple(estop_cells)))


def with_layer(core, layer, *, on="on"):
    core.obstacle = {"layer": layer}
    core.options.permit_stop_replan = on
    core.options.grid_planning = False
    core.options.obstacle_act = True
    return core


def hold(core, t0, seconds, *, allowed=0.0, why="occupied", acting=True, step=0.1):
    t = t0
    while t < t0 + seconds + 1e-9:
        core.permit_stop_tick(t, core.hooks.rear(), allowed, why, acting)
        t += step
    return t


def test_three_seconds_of_an_occupied_stop_raise_one_request_with_the_cells_and_a_wider_clearance():
    core, ms, state, calls, rear, held = make()
    snap = snapshot({(20, 20), (21, 20)})
    with_layer(core, Layer(snap, path_cells=[(20, 20)]))
    hold(core, 10.0, PERMIT_STOP_S - 0.2)
    assert not state.get("scene_replans")
    hold(core, 10.0 + PERMIT_STOP_S - 0.1, 0.2)
    rec = state["scene_replans"][-1]
    assert rec["trigger"] == "permit_stop" and len(state["scene_replans"]) == 1
    assert rec["clearance_m"] == pytest.approx(core.options.planner_config.clearance_m + PERMIT_STOP_CLEARANCE_ADD_M)
    assert rec["detail"]["why"] == "occupied" and rec["detail"]["held_s"] >= PERMIT_STOP_S - 1e-9
    extra = core.scene_replan.request_["extra_fn"]()
    assert cells_of(extra, snap) == {(20, 20), (21, 20)}


def test_the_callback_reads_the_permission_when_it_is_called():
    core, ms, state, calls, rear, held = make()
    layer = Layer(snapshot({(20, 20)}), path_cells=[(20, 20)])
    with_layer(core, layer)
    hold(core, 10.0, PERMIT_STOP_S + 0.1)
    fn = core.scene_replan.request_["extra_fn"]
    later = snapshot({(5, 5)})  # re-projected before the plan: other cells, the estop's only
    layer.permission = SimpleNamespace(snapshot=later, path_check=SimpleNamespace(blocked_cells=()),
                                       last_estop=SimpleNamespace(blocked_cells=((5, 5),)))
    assert cells_of(fn(), later) == {(5, 5)}


def test_any_break_restarts_the_three_seconds():
    for brk in ({"allowed": 0.1}, {"why": "unknown"}, {"acting": False}):
        core, ms, state, *_ = make()
        with_layer(core, Layer(snapshot(set())))
        t = hold(core, 10.0, PERMIT_STOP_S - 0.5)
        core.permit_stop_tick(t, core.hooks.rear(), **{"allowed": 0.0, "why": "occupied", "acting": True, **brk})
        hold(core, t + 0.1, PERMIT_STOP_S - 0.5)
        assert not state.get("scene_replans"), brk


def test_nothing_is_raised_off_outside_a_scene_on_a_grid_plan_or_while_the_device_runs():
    core, ms, state, *_ = make()
    with_layer(core, Layer(snapshot(set())), on="off")
    hold(core, 10.0, PERMIT_STOP_S + 1.0)
    assert not state.get("scene_replans")
    core, ms, state, *_ = make(scene=False)
    with_layer(core, Layer(snapshot(set())))
    hold(core, 10.0, PERMIT_STOP_S + 1.0)
    assert not state.get("scene_replans")
    core, ms, state, *_ = make()
    with_layer(core, Layer(snapshot(set())))
    core.options.grid_planning = True
    hold(core, 10.0, PERMIT_STOP_S + 1.0)
    assert not state.get("scene_replans")
    core, ms, state, calls, rear, held = make()
    with_layer(core, Layer(snapshot(set())))
    assert core.scene_replan_raise(9.0, "cusp_pose", rear["pose"]) is None  # the device is busy with another
    hold(core, 10.0, PERMIT_STOP_S + 1.0)
    assert [r["trigger"] for r in state["scene_replans"]] == ["cusp_pose"]
    assert state["scene_replans"][0]["ignored"] == []  # not even noted: the watch waits for the device


def test_a_spent_budget_ends_the_mission_with_the_device_s_reason():
    core, ms, state, *_ = make()
    with_layer(core, Layer(snapshot(set())))
    core.scene_replan.used_by_trigger["permit_stop"] = core.scene_replan.config.budget_per_trigger
    with pytest.raises(RuntimeError, match="scene_replan_exhausted:permit_stop"):
        hold(core, 10.0, PERMIT_STOP_S + 0.2)
    assert classify("scene_replan_exhausted:permit_stop") == MISSION


def test_an_old_record_s_options_without_the_switch_keep_it_off():
    core, ms, state, *_ = make()
    with_layer(core, Layer(snapshot(set())))
    del core.options.permit_stop_replan
    hold(core, 10.0, PERMIT_STOP_S + 1.0)
    assert not state.get("scene_replans")
