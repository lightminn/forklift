"""M-a1 slice 3b: one tick of the obstacle permission moved from the runner (behaviour, state, port requests and
their order, effects up to a refusal -- the plan's M-a1 criterion)."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState

REAR = np.array([1.0, 2.0, 0.1])


class Layer:
    def __init__(self, calls, limit=(0.5, "ok"), path_check=None, known_enabled=False, known_config=None):
        self.calls, self.answer = calls, limit
        self.permission = SimpleNamespace(path_check=path_check)
        self.known_enabled, self.known_config = known_enabled, known_config or {}
        self.snapshot = SimpleNamespace(state=np.zeros((2, 2), np.uint8), free_stamp=np.zeros((2, 2)), origin_x_m=0.0,
                                        origin_y_m=0.0, resolution_m=0.05, stamp_s=1.0)

    def refresh(self, *args, **kwargs):
        self.calls.append(("refresh", args, kwargs))

    def limit(self, *args, **kwargs):
        self.calls.append(("limit", args, kwargs))
        return self.answer


def make(phase="observe", *, limit=(0.5, "ok"), slam=None, remaining=5.0, truth_speed=0.3, act=True, grid=True,
         pocket_check=False, check=None, path_check=None, known_enabled=False, known_config=None, velocity=(0.3, 0.0)):
    calls = []

    def require(ok, reason):
        calls.append(("require", (bool(ok), reason), {}))
        if not ok:
            raise RuntimeError(reason)

    hooks = SimpleNamespace(
        require=require, truth_speed=lambda: truth_speed, truth_rear=lambda: (9.0, 8.0, 0.2),
        truth_velocity=lambda: np.array([*velocity, 0.0]), lift_position=lambda: 0.012,
        save_npz=lambda name, **arrays: calls.append(("save_npz", (name,), arrays)),
    )
    tracker = SimpleNamespace(remaining_to_goal_m=lambda: remaining, leg_ahead=lambda: ("LEG", -1))
    layer = Layer(calls, limit, path_check, known_enabled, known_config)
    obstacle = {"layer": layer, "applied": (0.0, 0.0, 0.0), "version": 2, "ticks": {}, "min_allowed": {},
                "slowed": {}, "reasons": {}}
    pocket = {"check": check, "frames_read": 4}
    ms = MissionState(phase=phase, phase_started=0.0)
    options = SimpleNamespace(grid_planning=grid, obstacle_act=act, pocket_check=pocket_check,
                              mid_still_decel_mps2=0.8)
    core = MissionCore(ms, state={"transitions": []}, obstacle=obstacle, slam=slam, pocket=pocket, paths={},
                       trackers={phase: tracker}, geometry=SimpleNamespace(alignment_straight_m=1.0,
                                                                           delivery_straight_m=0.8),
                       settings={}, pallet_geometry=None, hooks=hooks, options=options)
    # Moved into MissionCore (M-a1 slice 6e): this test's stand-ins go on the instance.
    for name_ in ['control_correction', 'known_pallet', 'return_turnaround_kwargs', 'realign_active', 'realign_install', 'start_near_realign', 'apply_tracker_profile', 'record_tracker_configs', 'build_near_tracker']:
        if hasattr(hooks, name_):
            setattr(core, name_, getattr(hooks, name_))
    return core, obstacle, pocket, calls


def test_a_correction_change_reprojects_the_grid_before_the_limit_is_asked():
    slam = {"tracker": SimpleNamespace(applied=(np.array([0.1, 0.2, 0.01]),)), "stop_now": False, "docking": {}}
    core, obstacle, _, calls = make(slam=slam)
    out = core.permission(10.0, REAR, 0.2, 0.4, True, None)
    assert [c[0] for c in calls] == ["refresh", "limit", "require"]  # the no-progress watch asks every tick
    assert calls[0][1] == (10.0, (0.1, 0.2, 0.01), 3) and calls[0][2] == {
        "current_pose": (1.0, 2.0, 0.1), "path_ahead": "LEG", "loaded": True, "direction": -1}
    assert (obstacle["version"], obstacle["applied"], obstacle["reprojections"]) == (3, (0.1, 0.2, 0.01), 1)
    assert calls[1][1] == (10.0,) and calls[1][2] == {"current_pose": (1.0, 2.0, 0.1), "curvature_inv_m": 0.2,
                                                    "direction": 1, "loaded": True, "cap_mps": 0.4}
    assert (out.allowed, out.why, out.acting, out.requested_speed, out.obstacle_hold) == (0.5, "ok", True, 0.4, False)
    assert obstacle["ticks"] == {"observe": 1} and obstacle["min_allowed"] == {"observe": 0.5}
    core.permission(10.1, REAR, 0.2, 0.4, True, None)  # same correction: no second re-projection
    assert [c[0] for c in calls if c[0] != "require"] == ["refresh", "limit", "limit"]


def test_the_direction_comes_from_the_command_then_a_handed_stall_then_the_truth_speed():
    core, _, _, calls = make(truth_speed=-0.3)
    core.permission(10.0, REAR, 0.0, 0.0, False, SimpleNamespace(hand_direction=1, prefix_occupied_now=False))
    core.permission(10.0, REAR, 0.0, 0.0, False, None)
    core.permission(10.0, REAR, 0.0, -0.2, False, None)
    assert [c[2]["direction"] for c in calls if c[0] == "limit"] == [1, -1, -1]
    assert [c[2]["cap_mps"] for c in calls if c[0] == "limit"] == [0.3, 0.3, 0.3]


def test_unknown_or_occupied_stops_are_dumped_four_times_and_then_once_per_new_phase():
    core, obstacle, _, calls = make(limit=(0.0, "unknown"))
    core.permission(4.0, REAR, 0.0, 0.0, False, None)  # not in the first 5 s
    for k in range(6):
        core.permission(6.0 + k, REAR, 0.0, 0.0, False, None)
    dumps = [c for c in calls if c[0] == "save_npz"]
    assert [d[1][0] for d in dumps] == [f"obstacle_dump_{k}.npz" for k in (1, 2, 3, 4)]
    assert dumps[0][2]["why"] == "unknown" and dumps[0][2]["phase"] == "observe" and dumps[0][2]["t"] == 6.0
    assert np.array_equal(dumps[0][2]["truth_rear"], [9.0, 8.0, 0.2]) and dumps[0][2]["leg"] == "LEG"
    core.ms.phase, core.trackers["transport"] = "transport", core.trackers["observe"]
    core.permission(20.0, REAR, 0.0, 0.0, False, None)
    assert [c[1][0] for c in calls if c[0] == "save_npz"][-1] == "obstacle_dump_5.npz"


@pytest.mark.parametrize("phase, remaining, extra, acting", [
    ("observe", 0.1, {}, True),
    ("approach", 1.5, {}, True),
    ("approach", 0.9, {}, False),  # the alignment straight: the docking segment, not acted on
    ("transport", 0.7, {}, False),  # the delivery straight
    ("transport", 0.7, {"backoff": True}, True),  # a backoff is never the delivery straight
    ("transport", 0.7, {"reapproach": True}, True),  # nor a D7c re-approach
    ("insert", 0.1, {}, False),
    ("withdraw", 0.1, {}, False),
    ("withdraw", 0.1, {"known": "certified"}, True),  # the D5 corridor
    ("withdraw", 0.1, {"known": "uncertified"}, False),
    ("withdraw", 0.1, {"known": "interim"}, False),  # only the certified corridor is acted on
])
def test_which_phases_the_permission_acts_on(phase, remaining, extra, acting):
    slam = {"tracker": SimpleNamespace(applied=None), "stop_now": False,
            "docking": {"reapproach": extra.get("reapproach", False)}}
    core, obstacle, _, _ = make(phase, remaining=remaining, slam=slam, known_enabled="known" in extra,
                                known_config={"withdraw_mode": extra.get("known", "certified")})
    if extra.get("backoff"):
        obstacle["backoff"] = {"phase": phase}
    out = core.permission(10.0, REAR, 0.0, 0.4, False, None)
    assert out.acting is acting and out.pocket_phase is False
    assert out.requested_speed == (0.4 if not acting else 0.4)  # 0.5 allowed: no cap either way


def test_the_pocket_check_answers_inside_the_pallet_region():
    core, _, pocket, _ = make("insert", pocket_check=True, check=None)
    out = core.permission(10.0, REAR, 0.0, 0.1, True, None)
    assert (out.pocket_phase, out.acting, out.allowed, out.why) == (True, True, 0.0, "pocket_missing")
    assert pocket["zero_since"] == 10.0
    with pytest.raises(RuntimeError, match="pocket_stop in insert: pocket_missing"):
        core.permission(15.0, REAR, 0.0, 0.1, True, None)
    asked = []
    check = SimpleNamespace(limit=lambda *a: asked.append(a) or (0.2, "voxel"), summary=lambda: {"frames": 9})
    core, _, pocket, _ = make("insert", pocket_check=True, check=check)
    pocket["zero_since"] = 3.0
    out = core.permission(10.0, REAR, 0.3, 0.4, True, None)
    assert (out.allowed, out.why) == (0.2, "voxel") and "zero_since" not in pocket
    assert asked == [(10.0, (1.0, 2.0, 0.1), 0.3, 1, 0.4, 0.012)]
    assert core.state["pocket_check"] == {"frames": 9, "frames_read": 4, "last_reason": "voxel", "last_reason_s": 10.0,
                                          "last_reason_rear": [1.0, 2.0, 0.1]}
    core.permission(10.05, REAR, 0.3, 0.4, True, None)
    assert core.state["pocket_check"]["last_reason_s"] == 10.0  # the same reason is kept once, when it began
    pocket["await_still_frame"] = True
    out = core.permission(10.1, REAR, 0.3, 0.4, True, None)
    assert (out.allowed, out.why) == (0.0, "pocket_settle")


def test_the_approach_straight_brakes_into_one_standing_frame_at_0_45_m():
    asked = []
    check = SimpleNamespace(limit=lambda *a: asked.append(a) or (0.9, "ok"), summary=lambda: {})
    core, _, pocket, _ = make("approach", remaining=0.5, pocket_check=True, check=check)
    out = core.permission(10.0, REAR, 0.0, 0.4, True, None)
    assert out.why == "pocket_settle_brake" and out.allowed == pytest.approx(max(0.05, math.sqrt(2 * 0.8 * 0.05)))
    assert "pocket_check" not in core.state  # an ok answer is not recorded
    core.trackers["approach"].remaining_to_goal_m = lambda: 0.44
    out = core.permission(10.1, REAR, 0.0, 0.4, True, None)
    assert (out.allowed, out.why) == (0.0, "pocket_settle") and pocket["mid_still_done"] and pocket["await_still_frame"]
    pocket["await_still_frame"] = False
    out = core.permission(10.2, REAR, 0.0, 0.4, True, None)
    assert (out.allowed, out.why) == (0.5, "ok")  # done once: the layer's answer stands


def test_a_blocked_path_for_36_ticks_stops_as_occupied_and_caps_the_command_as_an_emergency_stop():
    core, obstacle, _, _ = make("transport", path_check=SimpleNamespace(blocked="occupied"))
    for _ in range(35):
        out = core.permission(10.0, REAR, 0.0, 0.4, True, None)
    assert out.allowed == 0.5 and obstacle["path_blocked_ticks"] == 35
    out = core.permission(10.0, REAR, 0.0, 0.4, True, None)
    assert (out.allowed, out.why, out.requested_speed, out.obstacle_hold) == (0.0, "occupied", 0.0, True)
    assert obstacle["slowed"] == {"transport": 1} and obstacle["reasons"] == {"transport:occupied": 1}
    assert obstacle["permission_cap"] == (10.0, 0.5)  # the permission's own limit, before the blocked-path stop
    obstacle["layer"].permission.path_check = SimpleNamespace(blocked="unknown")
    core.permission(10.0, REAR, 0.0, 0.4, True, None)
    assert obstacle["path_blocked_ticks"] == 0


def test_a_slower_limit_caps_the_command_by_sign_only_when_acting_on_it():
    core, obstacle, _, _ = make("observe", limit=(0.2, "near"))
    out = core.permission(10.0, REAR, 0.0, -0.4, True, None)
    assert (out.requested_speed, out.obstacle_hold) == (-0.2, False)
    core, obstacle, _, _ = make("observe", limit=(0.2, "near"), act=False)
    out = core.permission(10.0, REAR, 0.0, -0.4, True, None)
    assert out.requested_speed == -0.4 and obstacle["slowed"] == {"observe": 1}
    assert obstacle["permission_cap"] is None


def test_sensor_silence_ends_the_run_after_one_second_standing():
    core, obstacle, _, calls = make("observe", limit=(0.0, "sensor_silent:lidar"), truth_speed=0.2)
    core.permission(10.0, REAR, 0.0, 0.0, False, None)
    trace = obstacle["silence_trace"]
    assert trace == {"first_zero_s": 10.0, "reason": "sensor_silent:lidar", "speed_at_zero_mps": 0.2,
                     "rear_at_zero": [9.0, 8.0, 0.2]}
    core.hooks.truth_speed = lambda: 0.0
    core.permission(11.0, REAR, 0.0, 0.0, False, None)
    assert trace["stopped_s"] == 11.0 and trace["rear_stopped"] == [9.0, 8.0, 0.2]
    core.hooks.truth_speed = lambda: 0.05  # moving again: that was not the final stop
    core.permission(11.5, REAR, 0.0, 0.0, False, None)
    assert trace["interrupted_stops_s"] == [11.0] and "stopped_s" not in trace and "rear_stopped" not in trace
    core.hooks.truth_speed = lambda: 0.0
    core.permission(12.0, REAR, 0.0, 0.0, False, None)
    with pytest.raises(RuntimeError, match="sensor_silence_stopped"):
        core.permission(13.0, REAR, 0.0, 0.0, False, None)
    core, obstacle, _, _ = make("observe", limit=(0.0, "sensor_silent:lidar"), act=False)
    core.permission(10.0, REAR, 0.0, 0.0, False, None)
    assert "silence_trace" not in obstacle  # not acting on it: no N9 outcome


def test_blocked_and_standing_counts_towards_the_replan_and_no_progress_ends_the_run():
    core, obstacle, _, _ = make("observe", limit=(0.0, "occupied"), velocity=(0.0, 0.0))
    for _ in range(3):
        core.permission(10.0, REAR, 0.0, 0.0, False, None)
    assert obstacle["blocked_ticks"] == 3
    core.hooks.truth_velocity = lambda: np.array([0.2, 0.0, 0.0])  # rolling: not yet
    core.permission(10.0, REAR, 0.0, 0.0, False, None)
    assert obstacle["blocked_ticks"] == 0
    slam = {"tracker": SimpleNamespace(applied=None), "stop_now": True, "docking": {}}
    core, obstacle, _, _ = make("transport", limit=(0.5, "ok"), slam=slam)
    core.permission(10.0, REAR, 0.0, 0.0, False, SimpleNamespace(hand_direction=1, prefix_occupied_now=True))
    assert obstacle["blocked_ticks"] == 1  # an occupied prefix of the next leg counts as blocked
    assert obstacle["progress"] == {"phase": "transport", "best_m": 5.0, "since_s": 10.0}
    core.permission(70.0, REAR, 0.0, 0.0, False, None)
    with pytest.raises(RuntimeError, match="obstacle_no_progress in transport"):
        core.permission(70.1, REAR, 0.0, 0.0, False, None)
    core.trackers["transport"].remaining_to_goal_m = lambda: 3.9  # a metre closer: the watch restarts
    core.permission(70.2, REAR, 0.0, 0.0, False, None)
    assert obstacle["progress"]["since_s"] == 70.2


def test_without_slam_standing_is_below_10_mm_per_second():
    """M-a2 sweep (boundaries): exactly 0.01 m/s is still moving -- no blocked-and-standing count."""
    core, obstacle, _, _ = make("observe", limit=(0.0, "occupied"), velocity=(0.01, 0.0))
    core.permission(10.0, REAR, 0.0, 0.0, False, None)
    assert obstacle["blocked_ticks"] == 0


def confirming(n=2):
    """make() with the restart confirmation on (docs/plans/2026-10-25-restart-confirmation.md) and one clearing LiDAR."""
    core, obstacle, pocket, calls = make("return_home", limit=(0.0, "occupied"), velocity=(0.0, 0.0))
    core.options.restart_confirm_scans = n
    layer = obstacle["layer"]
    layer.sensors = [SimpleNamespace(name="lidar", may_clear=True), SimpleNamespace(name="camera", may_clear=False)]
    layer.last_scan_s = {"lidar": 10.0, "camera": 10.0}
    return core, obstacle, layer


def test_an_occupied_stop_holds_a_one_scan_clearing_and_releases_on_n_new_scans():
    core, obstacle, layer = confirming(2)
    core.permission(10.0, REAR, 0.0, 0.0, False, None)  # the stop
    assert obstacle["blocked_ticks"] == 1
    layer.answer, layer.last_scan_s = (0.15, "occupied"), {"lidar": 10.1, "camera": 10.1}  # one scan clears the halo
    out = core.permission(10.1, REAR, 0.0, 0.2, False, None)
    assert out.allowed == 0.0 and out.why == "occupied" and out.obstacle_hold and out.requested_speed == 0.0
    assert obstacle["permission_cap"] == (10.1, 0.0)
    assert obstacle["blocked_ticks"] == 1  # neither counted nor reset by the confirmation alone
    layer.last_scan_s = {"lidar": 10.1, "camera": 10.2}  # only the camera (it never clears) is new
    assert core.permission(10.15, REAR, 0.0, 0.2, False, None).allowed == 0.0
    layer.last_scan_s = {"lidar": 10.2, "camera": 10.2}
    out = core.permission(10.2, REAR, 0.0, 0.2, False, None)
    assert out.allowed == 0.15 and obstacle["permission_cap"] == (10.2, 0.15)
    assert obstacle["restart_confirm"] == {"scans": 2, "arms": 1, "releases": 1, "gate_ticks": 2}


def test_without_the_option_nothing_of_the_confirmation_is_read():
    core, obstacle, _, _ = make("return_home", limit=(0.0, "occupied"), velocity=(0.0, 0.0))
    core.permission(10.0, REAR, 0.0, 0.0, False, None)
    obstacle["layer"].answer = (0.15, "occupied")
    out = core.permission(10.1, REAR, 0.0, 0.2, False, None)  # the stand-in layer has no sensors: none was read
    assert out.allowed == 0.15 and "restart_confirm" not in obstacle


def test_with_the_escape_presteer_on_a_running_scene_escape_s_first_stop_is_dumped_once_more():
    # escape presteer C4' (2026-10-28): an installed escape stopped at once and the phase's dumps were used up
    core, obstacle, _, calls = make(limit=(0.0, "occupied"))
    for k in range(4):
        core.permission(6.0 + k, REAR, 0.0, 0.0, False, None)
    core.scene_escape = {"status": "running"}
    core.permission(11.0, REAR, 0.0, 0.0, False, None)
    assert len([c for c in calls if c[0] == "save_npz"]) == 4  # off: as before
    core.options.escape_presteer = "on"
    obstacle["layer"].permission.release_stats = {"checks": 3, "passed": 1, "capped": 2}
    obstacle["layer"].permission.release_anchor = {"pose": (1.0, 2.0, 0.1)}
    core.permission(12.0, REAR, 0.0, 0.0, False, None)
    core.permission(13.0, REAR, 0.0, 0.0, False, None)
    dumps = [c for c in calls if c[0] == "save_npz"]
    assert len(dumps) == 5 and dumps[-1][2]["escape"] is True
    assert list(dumps[-1][2]["release_stats"]) == [3, 1, 2] and list(dumps[-1][2]["anchor_pose"]) == [1.0, 2.0, 0.1]
