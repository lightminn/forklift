"""P6 (docs/plans/2026-10-23-planner-permission-clearance.md 3 P6): a permission stop in a scene layout -- the limit
0 for an occupied cell, held 3 s on a travel leg -- raises the scene-mode replan device's request with the blocked
cells as obstacles and the clearance 0.10 m wider; the cells become merged cell rectangles of the occupied region
they belong to."""

from types import SimpleNamespace

import numpy as np
import pytest
from test_core_scene_replan import make  # the device's core fixtures

from forklift_core.control.drive_permission import (
    DrivePermission,
    PermissionConfig,
    StoppingModel,
)
from forklift_core.mission.failures import MISSION, classify
from forklift_core.mission.permit_stop import (
    PERMIT_STOP_CLEARANCE_ADD_M,
    PERMIT_STOP_S,
    blocked_rectangles,
)
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, GridSnapshot
from forklift_core.planning import Footprint


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
    """A real DrivePermission on a hand-made snapshot; the loaded outline for both directions."""

    FOOT = Footprint(1.56, 0.17, 0.40)

    def __init__(self, snap):
        self.permission = DrivePermission(PermissionConfig(StoppingModel(0.1, 0.5, 0.05), 0.01, 1e9))
        self.permission.snapshot = snap

    def footprints(self, loaded, direction=0):
        return self.FOOT, self.FOOT


REAR = np.array([2.0, 0.3, 0.0])


class Straight:
    """The core's tracker in these tests: the leg ahead runs straight on from REAR."""

    config = SimpleNamespace(cruise_speed_mps=0.18)
    leg_index = 0

    def leg_ahead(self):
        xs = np.linspace(REAR[0], REAR[0] + 3.0, 61)
        return np.column_stack([xs, np.full_like(xs, REAR[1]), np.zeros_like(xs)]), 1


def grid(cells):
    """160 x 60 cells of 0.05 m from (0, -1): x 0-8, y -1-2."""
    return snapshot(cells, shape=(160, 60), origin=(0.0, -1.0))


def box(x0, x1, y0, y1):
    """The cells whose centres lie in [x0, x1] x [y0, y1] on grid()."""
    return {(i, j) for i in range(160) for j in range(60)
            if x0 <= (i + 0.5) * 0.05 <= x1 and y0 <= -1.0 + (j + 0.5) * 0.05 <= y1}


def with_layer(core, layer, *, on="on"):
    core.obstacle = {"layer": layer}
    core.options.permit_stop_replan = on
    core.options.grid_planning = False
    core.options.obstacle_act = True
    core.trackers["transport"] = Straight()
    core.hooks.rear = lambda: REAR
    return core


def hold(core, t0, seconds, *, allowed=0.0, why="occupied", acting=True, step=0.1):
    t = t0
    while t < t0 + seconds + 1e-9:
        core.permit_stop_tick(t, core.hooks.rear(), allowed, why, acting)
        t += step
    return t


AHEAD = box(4.0, 4.1, 0.2, 0.4)  # on the leg, past the outline's front (3.56)
LATER = box(4.5, 4.6, 0.2, 0.4)


def test_three_seconds_of_an_occupied_stop_raise_one_request_with_the_cells_and_a_wider_clearance():
    core, ms, state, calls, rear, held = make()
    snap = grid(AHEAD)
    with_layer(core, Layer(snap))
    hold(core, 10.0, PERMIT_STOP_S - 0.2)
    assert not state.get("scene_replans")
    hold(core, 10.0 + PERMIT_STOP_S - 0.1, 0.2)
    rec = state["scene_replans"][-1]
    assert rec["trigger"] == "permit_stop" and len(state["scene_replans"]) == 1
    assert rec["clearance_m"] == pytest.approx(core.options.planner_config.clearance_m + PERMIT_STOP_CLEARANCE_ADD_M)
    assert rec["detail"]["why"] == "occupied" and rec["detail"]["held_s"] >= PERMIT_STOP_S - 1e-9
    extra = core.scene_replan.request_["extra_fn"]()
    assert cells_of(extra, snap) == AHEAD


def test_the_callback_checks_again_on_the_snapshot_of_its_call_never_the_stored_checks():
    # Codex implementation review P2: a hold release re-projects and re-checks only the truck's own pose, so the
    # stored path check and e-stop arc still name the cells of the snapshot before
    core, ms, state, calls, rear, held = make()
    layer = Layer(grid(AHEAD))
    with_layer(core, layer)
    perm = layer.permission
    leg, direction = Straight().leg_ahead()
    perm.path_check = perm.check_path(perm.snapshot, leg, Layer.FOOT, Layer.FOOT, current_pose=tuple(REAR),
                                      direction=direction)
    assert perm.path_check.blocked == "occupied"  # the stored check: the cells of AHEAD
    hold(core, 10.0, PERMIT_STOP_S + 0.1)
    fn = core.scene_replan.request_["extra_fn"]
    later = grid(LATER)
    perm.snapshot = later  # the release's re-projection; path_check left from before
    assert cells_of(fn(), later) == LATER


def test_a_clear_snapshot_at_the_call_gives_no_obstacles():
    core, ms, state, *_ = make()
    layer = Layer(grid(AHEAD))
    with_layer(core, layer)
    hold(core, 10.0, PERMIT_STOP_S + 0.1)
    layer.permission.snapshot = grid(set())
    assert core.scene_replan.request_["extra_fn"]() == []


def test_two_blocked_regions_are_both_kept():
    wall_a, wall_b = {(i, 10) for i in range(5, 15)}, {(i, 30) for i in range(5, 15)}
    snap = snapshot(wall_a | wall_b)
    assert cells_of(blocked_rectangles(snap, [(7, 10), (12, 30)], grow_m=1.0), snap) == wall_a | wall_b


def test_a_gap_in_the_watch_or_a_new_tracker_starts_the_three_seconds_again():
    # Codex implementation review P3: the device owning ticks (permission() not called) left the clock running
    core, ms, state, *_ = make()
    with_layer(core, Layer(grid(AHEAD)))
    hold(core, 10.0, 2.0)
    hold(core, 14.0, PERMIT_STOP_S - 0.5)  # after a 2 s gap: a new stretch
    assert not state.get("scene_replans")
    core, ms, state, *_ = make()
    with_layer(core, Layer(grid(AHEAD)))
    t = hold(core, 10.0, 2.0)
    core.trackers["transport"] = Straight()  # a replan installed another tracker
    hold(core, t, PERMIT_STOP_S - 0.5)
    assert not state.get("scene_replans")


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


def test_a_spent_budget_stops_the_watch_for_the_mission_without_ending_it():
    core, ms, state, *_ = make()
    with_layer(core, Layer(snapshot(set())))
    core.scene_replan.used_by_trigger["permit_stop"] = core.scene_replan.config.budget_per_trigger
    hold(core, 10.0, PERMIT_STOP_S + 0.2)  # no RuntimeError: the mission goes on standing
    assert state["scene_replans"][-1]["outcome"] == "exhausted" and core.permit_stop_spent
    core.trackers["transport"] = Straight()  # even on another leg
    hold(core, 20.0, PERMIT_STOP_S + 1.0)
    assert len(state["scene_replans"]) == 1
    assert classify("scene_replan_exhausted:permit_stop") == MISSION  # the reason is still a mission's, if raised


def test_one_request_per_leg_a_request_handed_back_leaves_the_leg_to_the_permission():
    core, ms, state, *_ = make()
    with_layer(core, Layer(grid(AHEAD)))
    hold(core, 10.0, PERMIT_STOP_S + 0.1)
    assert len(state["scene_replans"]) == 1
    core.scene_replan.abort("handed_back")  # as a standing-safe end: the same tracker, nothing installed
    hold(core, 20.0, 2 * PERMIT_STOP_S)
    assert len(state["scene_replans"]) == 1
    core.trackers["transport"] = Straight()  # a replan installed a new tracker: watched again
    hold(core, 30.0, PERMIT_STOP_S + 0.1)
    assert len(state["scene_replans"]) == 2


def test_an_old_record_s_options_without_the_switch_keep_it_off():
    core, ms, state, *_ = make()
    with_layer(core, Layer(snapshot(set())))
    del core.options.permit_stop_replan
    hold(core, 10.0, PERMIT_STOP_S + 1.0)
    assert not state.get("scene_replans")


def _failing_plans(core):
    core.hooks.plan_transport_leg = lambda *a, **k: SimpleNamespace(success=False, status="invalid_start")


def test_the_core_hands_back_only_when_its_own_tracker_still_follows_from_the_pose():
    from test_core_scene_replan import _drive

    # on its path: the old tracker follows, the request ends standing and the mission goes on
    core, ms, state, calls, rear, held = make()
    _failing_plans(core)
    rear["pose"] = np.array(core.trackers["transport"].leg_ahead()[0][0], dtype=float)  # standing on its path
    assert core.scene_replan_raise(10.0, "permit_stop", rear["pose"]) is None
    outs = _drive(core, rear, held)
    assert outs[-1].owned is False and state["scene_replans"][-1]["mission_end"] is False
    # 1 m off it (what a hold release can leave): the old tracker fails, so the device's end stands
    core, ms, state, calls, rear, held = make()
    _failing_plans(core)
    rear["pose"] = np.array(core.trackers["transport"].leg_ahead()[0][0], dtype=float) + np.array([0.0, 1.0, 0.0])
    assert core.scene_replan_raise(10.0, "permit_stop", rear["pose"]) is None
    with pytest.raises(RuntimeError, match="scene_replan_failed:permit_stop:invalid_start"):
        _drive(core, rear, held)
    assert state["scene_replans"][-1]["not_resumable"] is True


def test_a_request_ended_with_its_plan_still_out_leaves_no_job_or_answer_behind():
    # PR #116 review P2 (the device's author): the request's time limit in wait_plan handed back, the worker's late
    # answer must not reach the mission's next request
    from test_core_scene_replan import Jobs

    class Recording(Jobs):
        def __init__(self):
            super().__init__()
            self.discarded = []

        def discard(self, job):
            self.discarded.append(job)

    core, ms, state, calls, rear, held = make(jobs=Recording())
    state["async_plans"] = []
    core.commands.begin_wait = lambda *a, **k: None
    core.commands.end_wait = lambda *a, **k: None
    rear["pose"] = np.array(core.trackers["transport"].leg_ahead()[0][0], dtype=float)
    assert core.scene_replan_raise(10.0, "permit_stop", rear["pose"]) is None
    core.scene_replan_tick(10.0, rear["pose"], 0.0, 1 / 120)
    assert core.pending is not None and core.pending.site == "scene_replan"
    job = core.pending.job
    out = core.scene_replan_tick(10.0 + core.scene_replan.config.request_limit_s + 1.0, rear["pose"], 0.0, 1 / 120)
    assert out.owned is False and state["scene_replans"][-1]["reason"] == "scene_replan_failed:permit_stop:timeout"
    assert core.pending is None and core.jobs.discarded == [job] and core.scene_mail is None
    assert state["async_plans"][-1]["dropped"] == "scene_replan_ended"


class Escape:
    """A scene replan escape installed in the leg's place: nothing ahead of it."""

    config = SimpleNamespace(cruise_speed_mps=0.18)
    leg_index = 0

    def leg_ahead(self):
        return np.zeros((0, 3)), -1


def test_after_an_escape_the_obstacles_are_found_on_the_leg_it_left():
    # Codex review of docs/plans/2026-10-25-escape-start-release.md P2-3: the escape replaces the leg's tracker and
    # stays installed until the request's next install, so the replan after it read no cells ahead (1 -> 0)
    core, ms, state, calls, rear, held = make()
    snap = grid(AHEAD)
    with_layer(core, Layer(snap))
    hold(core, 10.0, PERMIT_STOP_S + 0.1)
    fn = core.scene_replan.request_["extra_fn"]
    leg = core.trackers["transport"]
    escape = Escape()
    core.trackers["transport"] = escape
    core.scene_escape = {"status": "cleared", "tracker": escape, "leg_tracker": leg}
    assert cells_of(fn(), snap) == AHEAD
    core.scene_escape = None  # a later install consumed it: the tracker installed is the leg's own again
    assert fn() == []
