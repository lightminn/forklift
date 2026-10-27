"""M-a1 slice 2: the arrival transitions, the follower's time limit, the lift ramp and the timed transitions moved
from the runner (behaviour, state, call order, effects up to a refusal -- the plan's M-a1 criterion)."""

import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState


class Log(list):
    def hook(self, name, result=None):
        def call(*args, **kwargs):
            self.append((name, args, kwargs))
            return result() if callable(result) else result
        return call


def make(phase, *, near=None, slam=None, trackers=None, stop_at=None, d7=False, dwells=None, safety_stops=None,
         truth=None):
    log = Log()
    truth = {"pallet_z": 0.1, "lift": 0.0, "rear": (1.0, 2.0, 0.1), "relative_yaw": 0.0, **(truth or {})}

    def require(ok, reason):
        log.append(("require", (bool(ok), reason), {}))
        if not ok:
            raise RuntimeError(reason)

    hooks = SimpleNamespace(
        echo=log.hook("echo"), record_json=lambda v: f"{v['from']}->{v['to']}", snapshot=log.hook("snapshot"),
        require=require, control_correction=log.hook("control_correction"), known_pallet=log.hook("known_pallet"),
        slam_odom_base=log.hook("slam_odom_base"), slam_error=log.hook("slam_error", (0.02, 0.0, 0.0)),
        fork_blades=log.hook("fork_blades", []),
        insertion_measured=log.hook("insertion_measured", {"depth_m": 0.33}),
        truth_relative_yaw=log.hook("truth_relative_yaw", lambda: truth["relative_yaw"]),
        dock_at_delivery_straight=log.hook("dock_at_delivery_straight"),
        slam_rear=log.hook("slam_rear", np.array([4.0, 5.0, 0.2])), set_rear=log.hook("set_rear"),
        pallet_z=log.hook("pallet_z", lambda: truth["pallet_z"]),
        lift_position=log.hook("lift_position", lambda: truth["lift"]),
        truth_rear=log.hook("truth_rear", lambda: truth["rear"]),
    )
    ms = MissionState(phase=phase, phase_started=10.0)
    state = {"transitions": [], "near_tracking": {}}
    options = SimpleNamespace(near_tracking_stop_at=stop_at, d7_docking=d7, dwells=dwells, safety_stops=safety_stops,
                              home_rear=SimpleNamespace(x_m=1.0, y_m=1.0, yaw_rad=0.0))
    core = MissionCore(ms, state=state, obstacle=None, slam=slam, pocket={"check": None}, paths={},
                       trackers=trackers if trackers is not None else {}, geometry=SimpleNamespace(),
                       settings={"lift_target_m": 0.1, "lift_rate_mps": 0.06}, pallet_geometry=None, hooks=hooks,
                       near=near, options=options)
    # Docking/withdrawal helpers are MissionCore methods since slice 6b; this test's stand-ins go on the instance (6c: the release and escape helpers too).
    for name_ in ['dock_at_delivery_straight', 'arm_docking', 'release_and_carry', 'transport_replan', 'start_docking_retry', 'release_and_stand', 'release_after_backoff', 'slam_release', 'escape_leg', 'find_escape', 'hold_count', 'near_tick', 'near_redraw', 'near_capture_redraw', 'near_protection_prediction', 'near_install', 'start_pickup_retry', 'control_correction', 'known_pallet', 'return_turnaround_kwargs', 'realign_active', 'realign_install', 'start_near_realign', 'apply_tracker_profile', 'record_tracker_configs', 'build_near_tracker']:
        if hasattr(hooks, name_):
            setattr(core, name_, getattr(hooks, name_))
    return core, ms, state, log


def track(position=0.004, yaw=0.01):
    return SimpleNamespace(position_error_m=position, yaw_error_rad=yaw)


@pytest.mark.parametrize("phase, to", [("approach", "insert"), ("extract", "transport"), ("withdraw", "settle"),
                                       ("return_home", "home_settle")])
def test_an_arrival_moves_to_the_next_phase_and_keeps_the_loop(phase, to):
    core, ms, state, _ = make(phase)
    result = core.arrive(20.0, track())
    assert (result.stop, result.rear) == (False, None)
    assert (ms.phase, ms.phase_started) == (to, 20.0)
    assert state["transitions"] == [{"from": phase, "to": to, "time_s": 20.0}]


@pytest.mark.parametrize("phase", ["observe", "lift", "lower", "settle"])
def test_an_arrival_in_a_phase_without_its_own_rule_changes_nothing(phase):
    core, ms, state, log = make(phase)
    result = core.arrive(20.0, track())
    assert (result.stop, result.rear, ms.phase, state["transitions"], list(log)) == (False, None, phase, [], [])


def test_near_tracking_stopping_at_insert_records_the_arrival_and_ends_the_loop():
    core, ms, state, _ = make("approach", near={"ticks": []}, stop_at="insert")
    result = core.arrive(21.0, track(0.003, -0.02))
    assert result.stop is True and ms.phase == "near_tracking_stopped"
    assert state["near_tracking"] == {"outcome": "stopped_at_insert",
                                      "arrival": {"time_s": 21.0, "position_error_m": 0.003, "yaw_error_rad": -0.02}}
    core, ms, _, _ = make("approach", near=None, stop_at="insert")  # no near tracker: the stop does not apply
    assert core.arrive(21.0, track()).stop is False and ms.phase == "insert"


def test_the_insertion_end_records_the_follower_error_the_measured_insertion_and_the_truth_yaw_then_lifts():
    slam = {"tracker": SimpleNamespace(mode="holding"), "holds": []}
    near = {"arrival": {"reason": "tick", "time_s": 30.5}, "ticks": [{"n": 1}, {"n": 2}]}
    core, ms, state, log = make("insert", near=near, slam=slam, truth={"relative_yaw": math.pi + 0.01})
    core.arrive(31.0, track(0.002, 0.005))
    assert state["insertion_error"] == {"position_m": 0.002, "yaw_rad": 0.005}
    assert state["insertion_measured"] == {"depth_m": 0.33}
    assert state["near_tracking"]["insert_arrival"] == {"reason": "tick", "time_s": 30.5, "last_tick": {"n": 2},
                                                        "insertion_measured": {"depth_m": 0.33}}
    assert state["insertion_truth_yaw_rad"] == pytest.approx(0.01)  # either way round along the pallet x axis
    assert state["slam_error_insert_end"] == (0.02, 0.0, 0.0)
    assert ms.phase == "lift"
    names = [n for n, *_ in log]
    assert names[:3] == ["insertion_measured", "truth_relative_yaw", "slam_error"]  # before the transition
    core, _, state, _ = make("insert", near={"ticks": []})
    core.arrive(31.0, track())
    assert state["near_tracking"]["insert_arrival"] == {"reason": "path", "time_s": 31.0, "last_tick": None,
                                                        "insertion_measured": {"depth_m": 0.33}}
    assert "slam_error_insert_end" not in state  # no SLAM, no error record


def test_transport_arrival_docks_when_armed_and_rereads_the_pose_under_d7():
    slam = {"tracker": SimpleNamespace(mode="holding"), "holds": [], "docking": {"status": "armed"},
            "control": [[0.0, 0.0, 0.0, 0.0]]}
    core, ms, _, log = make("transport", slam=slam, d7=True)
    result = core.arrive(40.0, track())
    assert ms.phase == "transport" and result.stop is False
    assert [n for n, *_ in log] == ["dock_at_delivery_straight", "slam_rear", "set_rear"]
    np.testing.assert_array_equal(result.rear, [4.0, 5.0, 0.2])
    assert slam["control"][-1] == [0.0, 4.0, 5.0, 0.2]
    slam["control"] = [[0.0, 0.0, 0.0, 0.0]]
    core, _, _, log = make("transport", slam=slam, d7=False)
    assert core.arrive(40.0, track()).rear is None and [n for n, *_ in log] == ["dock_at_delivery_straight"]
    assert slam["control"][-1] == [0.0, 0.0, 0.0, 0.0]
    slam["docking"]["status"] = "done"
    core, ms, _, _ = make("transport", slam=slam)
    core.arrive(40.0, track())
    assert ms.phase == "lower"


def test_the_tracking_limit_adds_the_scheduled_standing_on_the_approach_only():
    tracker = SimpleNamespace(nominal_duration_s=lambda: 12.0)
    asked = []
    dwells = SimpleNamespace(held_s=lambda t: asked.append(("dwells", t)) or t / 10)
    stops = SimpleNamespace(held_s=lambda t: asked.append(("stops", t)) or t / 100)
    core, ms, _, _ = make("approach", trackers={"approach": tracker, "transport": tracker}, dwells=dwells,
                          safety_stops=stops)
    assert core.tracking_limit(50.0) == pytest.approx(46.0 + 5.0 + 0.5)
    assert asked == [("dwells", 50.0), ("stops", 50.0)]
    ms.phase = "transport"
    assert core.tracking_limit(50.0) == pytest.approx(46.0)
    tracker.nominal_duration_s = lambda: 1.0
    assert core.tracking_limit(50.0) == 30.0


def test_the_lift_target_ramps_at_the_rate_and_drops_after_transport():
    core, ms, _, _ = make("lift")
    ms.phase_started = 100.0  # well inside the phase: no timed transition
    for _ in range(3):
        core.lift_and_timers(101.0, 0.5)
    assert ms.lift_command == pytest.approx(0.09)
    core.lift_and_timers(101.0, 0.5)
    assert ms.lift_command == pytest.approx(0.1)  # clipped at the target
    ms.phase = "lower"
    core.lift_and_timers(101.0, 0.5)
    assert ms.lift_command == pytest.approx(0.07)


def test_lift_and_lower_end_after_4_5_s_on_the_pallet_and_fork_checks():
    core, ms, _, _ = make("lift", truth={"pallet_z": 0.07})
    assert core.lift_and_timers(14.4, 0.01) is False and ms.phase == "lift"
    assert core.lift_and_timers(14.6, 0.01) is False and ms.phase == "extract"
    core, ms, _, log = make("lift", truth={"pallet_z": 0.05})
    with pytest.raises(RuntimeError, match="Pallet was not lifted"):
        core.lift_and_timers(14.6, 0.01)
    assert ms.phase == "lift"
    assert ms.lift_command == pytest.approx(0.06 * 0.01)  # this tick's ramp step comes before the refusal
    core, ms, _, log = make("lower", truth={"pallet_z": 0.002, "lift": 0.02})
    with pytest.raises(RuntimeError, match="Fork did not lower"):
        core.lift_and_timers(14.6, 0.01)
    assert [n for n, *_ in log] == ["pallet_z", "require", "lift_position", "require"] and ms.phase == "lower"
    core, ms, _, log = make("lower", truth={"pallet_z": 0.01})
    with pytest.raises(RuntimeError, match="destination floor"):
        core.lift_and_timers(14.6, 0.01)
    assert "lift_position" not in [n for n, *_ in log]  # the fork is not read once the pallet check refused
    core, ms, _, _ = make("lower", truth={"pallet_z": -0.002, "lift": 0.0})
    core.lift_and_timers(14.6, 0.01)
    assert ms.phase == "withdraw"


def test_settle_goes_home_or_completes_and_home_settle_records_the_return_error():
    core, ms, _, _ = make("settle", trackers={"return_home": object()})
    assert core.lift_and_timers(10.9, 0.01) is False and ms.phase == "settle"
    assert core.lift_and_timers(11.1, 0.01) is False and ms.phase == "return_home"
    core, ms, _, _ = make("settle")
    assert core.lift_and_timers(11.1, 0.01) is True and ms.phase == "complete"
    core, ms, state, _ = make("home_settle", truth={"rear": (4.0, 5.0, 2 * math.pi + 0.05)})
    assert core.lift_and_timers(11.1, 0.01) is True and ms.phase == "complete"
    assert state["return_home_error"]["position_m"] == pytest.approx(5.0)
    assert state["return_home_error"]["yaw_rad"] == pytest.approx(0.05)


# --- plan 2026-10-19 task 3: the bottom-deck insert permission at approach -> insert ---

def _bottom_deck(verdict, *, pickup=(3.0, 1.0, 0.3), check="on"):
    from forklift_core.perception import bottom_deck_permit as P
    from forklift_core.perception.bottom_deck_evidence import BottomDeckParams
    core, ms, state, log = make("approach")
    core.options.bottom_deck_check = check
    core.pallet_geometry = SimpleNamespace(geometry_version="epal6")
    core.pocket["planning_pickup"] = SimpleNamespace(x_m=pickup[0], y_m=pickup[1], yaw_rad=pickup[2])
    if verdict is not None:
        state["bottom_deck"] = {"record": P.BottomDeckRecord(
            verdict=verdict, faces=(), centre_xy=(3.0, 1.0), model_yaw_rad=0.3, shape="epal6", axis="x",
            support_z_m=0.0, candidate_id=None, track_id=None, captured_s=(5.0, 8.0), correction_s=None,
            params_sha256=P.params_digest(BottomDeckParams())).to_record()}
    return core, ms, state


@pytest.mark.parametrize("verdict, pickup, reason", [
    (None, (3.0, 1.0, 0.3), "bottom_deck_unobserved"),
    ("unobserved", (3.0, 1.0, 0.3), "bottom_deck_unobserved"),
    ("absent", (3.0, 1.0, 0.3), "no_bottom_deck"),
    ("present", (3.3, 1.0, 0.3), "bottom_deck_target_mismatch"),
])
def test_the_bottom_deck_check_stops_the_truck_before_the_forks_go_in(verdict, pickup, reason):
    core, ms, state = _bottom_deck(verdict, pickup=pickup)
    result = core.arrive(21.0, track())
    assert result.stop is True and ms.phase == "bottom_deck_refused"
    assert state["mission_result"] == "bottom_deck_refused"
    assert state["bottom_deck"]["refused"] == {"reason": reason, "time_s": 21.0, "target": list(pickup),
                                               "shape": "epal6"}
    assert state["transitions"] == [{"from": "approach", "to": "bottom_deck_refused", "time_s": 21.0}]
    assert json.loads(json.dumps(state)) == state  # the runner writes the state as JSON (Codex task 3 P2)


def test_a_present_record_on_the_same_target_lets_the_insertion_start():
    core, ms, state = _bottom_deck("present", pickup=(3.05, 0.95, 0.3 + math.pi))
    result = core.arrive(21.0, track())
    assert result.stop is False and ms.phase == "insert"
    assert state["bottom_deck"]["permitted"] == {"time_s": 21.0, "target": [3.05, 0.95, 0.3 + math.pi],
                                                 "shape": "epal6"}
    assert json.loads(json.dumps(state)) == state


def test_with_the_check_off_the_record_is_not_read():
    core, ms, state = _bottom_deck("absent", check="off")
    assert core.arrive(21.0, track()).stop is False and ms.phase == "insert"
    assert set(state["bottom_deck"]) == {"record"}


# --- plan 2026-10-28-adoption-zone-check: the planned pickup (and the latest tracked one) before the insertion ---

def _zoned(zone, *, pickup=(3.0, 1.0, 0.3), near=None):
    from forklift_core.mission.adoption_zone import AdoptionZone
    from forklift_core.planning import Pose2D

    core, ms, state, log = make("approach", near=near)
    core.options.adoption_zone_check = "on"
    core.options.adoption_zone_margin_m = 0.25
    core.options.adoption_zone = AdoptionZone(*zone, "scene_layout")
    core.pocket["planning_pickup"] = Pose2D(*pickup)
    # the tracked target as near_redraw reads it: the observation's centre placed with its own pose
    core.options.adapter = SimpleNamespace(estimate_pallet_center_m=lambda obs, depth: (obs.x, 0.0),
                                           estimate_world_pallet_site=lambda xy, yaw_, base, yaw: Pose2D(
                                               base[0] + xy[0], base[1], yaw))
    core.options.G2 = SimpleNamespace(planning_target=lambda mode, site, nominal: (site, "perception"))
    core.options.planning_target, core.options.scenario = "perception", SimpleNamespace(pickup=None)
    core.geometry = SimpleNamespace(pallet_depth_m=0.8)
    return core, ms, state


def tracked(x_ahead, pose=(2.0, 1.0, 0.0)):
    """A near record whose tracker's latest observation places the pallet x_ahead in front of ``pose``."""
    return {"tracker": SimpleNamespace(latest=SimpleNamespace(
        observation=SimpleNamespace(insertion_yaw_rad=0.0, x=x_ahead), pose=pose))}


def test_a_planned_pickup_inside_the_zone_starts_the_insertion_and_is_recorded():
    core, ms, state = _zoned((2.5, 3.5, 0.5, 1.5))
    assert core.arrive(21.0, track()).stop is False and ms.phase == "insert"
    assert [c["stage"] for c in state["adoption_zone"]["checks"]] == ["insert"]


def test_a_planned_pickup_outside_the_zone_ends_the_mission_before_the_insertion():
    core, ms, state = _zoned((0.0, 1.0, 0.0, 1.0))
    with pytest.raises(RuntimeError, match="^adoption_outside_zone:insert$"):
        core.arrive(21.0, track())
    assert ms.phase == "approach" and state["transitions"] == []


def test_with_near_tracking_the_latest_tracked_target_is_checked_too():
    core, ms, state = _zoned((2.5, 3.5, 0.5, 1.5), near=tracked(1.0))  # placed at (3.0, 1.0): inside
    core.arrive(21.0, track())
    assert [c["stage"] for c in state["adoption_zone"]["checks"]] == ["insert", "insert_tracked"]
    assert state["adoption_zone"]["checks"][1]["site"] == [3.0, 1.0]
    core, ms, state = _zoned((2.5, 3.5, 0.5, 1.5), near=tracked(2.0))  # at (4.0, 1.0): 0.5 m past x_max
    with pytest.raises(RuntimeError, match="^adoption_outside_zone:insert_tracked$"):
        core.arrive(21.0, track())
