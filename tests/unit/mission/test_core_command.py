"""M-a1 slice 4a-iv: the follower's command and the stops before the arrival, moved from the runner (behaviour,
state, port requests and their order, effects up to a refusal -- the plan's M-a1 criterion)."""

from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState

REAR = np.array([1.0, 2.0, 0.0])


@dataclass(frozen=True)
class Tracking:
    status: str = "tracking"
    speed_mps: float = 0.4
    curvature_inv_m: float = 0.1
    position_error_m: float = 0.02
    yaw_error_rad: float = 0.01


class Schedule:
    def __init__(self, answer, active=None):
        self.answer, self.active, self.asked = answer, active, []

    def update(self, t, **kwargs):
        self.asked.append((t, kwargs))
        return self.answer


class World:
    def __init__(self, phase="approach", *, slam=True, may_drive=True, stop_now=True, dwells=None, safety=None,
                 near=None, near_answer=(False, False, False), near_capture="done", remaining=2.0, obstacle=None):
        self.calls = []
        self.runner_rear = REAR

        def require(ok, reason):
            self.calls.append(("require", (bool(ok), reason), {}))
            if not ok:
                raise RuntimeError(reason)

        self.hooks = SimpleNamespace(
            require=require, dump_tracking=lambda reason, tr: self.calls.append(("dump_tracking", (reason,), {})),
            near_tick=lambda *a: self.calls.append(("near_tick", a, {})) or near_answer,
            release_after_backoff=lambda t, event: self.calls.append(("release_after_backoff", (t, event), {}))
            or np.array([1.5, 2.0, 0.0]),
            set_rear=lambda v: setattr(self, "runner_rear", v),
        )
        self.tracker = SimpleNamespace(remaining_to_goal_m=lambda: remaining,
                                       restart_speed_slew=lambda v: self.calls.append(("restart_speed_slew", (v,), {})))
        self.slam = {"tracker": SimpleNamespace(may_drive=lambda: may_drive), "stop_now": stop_now} if slam else None
        self.ms = MissionState(phase=phase, phase_started=0.0)
        self.state = {"transitions": [], "near_capture": {"status": near_capture}}
        self.obstacle = obstacle
        self.core = MissionCore(
            self.ms, state=self.state, obstacle=obstacle, slam=self.slam, pocket={"check": None}, paths={},
            trackers={phase: self.tracker, "approach": self.tracker}, geometry=None, settings={}, pallet_geometry=None,
            hooks=self.hooks, near=near,
            options=SimpleNamespace(dwells=dwells, safety_stops=safety, dwell_d_end_m=0.3, observe_arrival_m=0.10,
                                    observe_arrival_yaw_rad=0.10))
        # Helpers moved into MissionCore (slices 6b/6c): this test's stand-ins go on the instance.
        for name_ in ['dock_at_delivery_straight', 'arm_docking', 'release_and_carry', 'transport_replan', 'start_docking_retry', 'release_and_stand', 'release_after_backoff', 'slam_release', 'escape_leg', 'find_escape', 'hold_count', 'near_tick', 'near_redraw', 'near_capture_redraw', 'near_protection_prediction', 'near_install', 'start_pickup_retry', 'control_correction', 'known_pallet', 'return_turnaround_kwargs', 'realign_active', 'realign_install', 'start_near_realign', 'apply_tracker_profile', 'record_tracker_configs', 'build_near_tracker']:
            if hasattr(self.hooks, name_):
                setattr(self.core, name_, getattr(self.hooks, name_))

    def tick(self, tracking=Tracking(), t=50.0, speed=0.0, flags=(False, False, False)):
        return self.core.command(t, REAR, 49.99, tracking, speed, *flags)


def test_the_command_is_the_followers_and_a_failure_is_dumped_then_ends_the_run():
    out = World().tick()
    assert (out.requested_speed, out.curvature, out.tracking.status) == (0.4, 0.1, "tracking")
    assert (out.dwell_hold, out.dwell_standing, out.safety_hold) == (False, False, False) and out.rear is REAR
    w = World()
    with pytest.raises(RuntimeError, match="Tracking failed in approach: pos=0.0200,yaw=0.0100"):
        w.tick(Tracking(status="failed"))
    assert w.calls[0] == ("dump_tracking", ("failed",), {})


def test_slam_warming_up_holds_still():
    assert World(may_drive=False).tick().requested_speed == 0.0


def test_a_dwell_holds_the_wheels_and_steering_and_locks_only_once_standing():
    dwells = Schedule((True, False), active={"still_since_s": None})
    w = World(dwells=dwells)
    out = w.tick(Tracking(status="arrived"))
    assert (out.requested_speed, out.dwell_hold, out.dwell_standing, out.safety_hold) == (0.0, True, False, False)
    assert out.tracking.status == "tracking" and out.tracking.speed_mps == 0.0  # the arrival waits for the dwell
    assert dwells.asked == [(50.0, {"d_est_m": 2.3, "still": True, "arrived": True})]
    dwells.active = {"still_since_s": 49.0}
    assert w.tick().dwell_standing is True
    dwells.answer = (False, True)
    out = w.tick()
    assert out.requested_speed == 0.0 and w.calls[-1] == ("restart_speed_slew", (0.0,), {})
    w = World(dwells=Schedule((True, False)), near_capture="pending")
    assert w.tick().requested_speed == 0.4  # before the near capture: no dwell


def test_a_scheduled_safety_stop_steps_to_zero_without_a_wheel_lock():
    safety = Schedule((True, False))
    out = World(safety=safety).tick(Tracking(status="arrived"))
    assert (out.requested_speed, out.dwell_hold, out.safety_hold, out.dwell_standing) == (0.0, True, True, False)
    assert out.tracking.status == "tracking" and safety.asked == [(50.0, {"d_est_m": 2.3, "still": True})]
    w = World(safety=Schedule((False, True)))
    assert w.tick().requested_speed == 0.0 and w.calls[-1] == ("restart_speed_slew", (0.0,), {})


@pytest.mark.parametrize("answer, status, speed, holds", [
    ((False, False, True), "arrived", 0.0, (True, True)),  # the carriage-corner insertion end
    ((True, False, False), "tracking", 0.0, (True, True)),
    ((False, True, False), "tracking", 0.0, (False, False)),
    ((False, False, False), "tracking", 0.4, (False, False)),
])
def test_the_near_field_stop_decides_before_this_ticks_arrival(answer, status, speed, holds):
    w = World("insert", near={"tracker": object()}, near_answer=answer)
    out = w.tick(Tracking(status="tracking"))
    (_, args, _), = [c for c in w.calls if c[0] == "near_tick"]
    assert args == (50.0, 49.99, REAR, 0.4, "insert", 0.1, False)
    assert (out.tracking.status, out.requested_speed, (out.dwell_hold, out.safety_hold)) == (status, speed, holds)
    assert ("restart_speed_slew" in [c[0] for c in w.calls]) is (answer == (False, True, False))


def test_a_hold_defers_an_arrival_and_without_a_near_tracker_nothing_is_asked():
    w = World("approach", near={"tracker": object()}, near_answer=(True, False, False))
    assert w.tick(Tracking(status="arrived")).tracking.status == "tracking"
    (_, args, _), = [c for c in w.calls if c[0] == "near_tick"]
    assert args[-1] is True  # the near tick is told this tick's arrival
    w = World("approach", near={"tracker": None})
    w.tick()
    assert "near_tick" not in [c[0] for c in w.calls]


def test_an_observe_leg_standing_off_its_end_within_tolerance_arrives_after_a_second():
    w = World("observe", remaining=0.0)
    stopped = Tracking(status="tracking", speed_mps=0.0, position_error_m=0.06, yaw_error_rad=0.05)
    assert w.tick(stopped, t=10.0).tracking.status == "tracking"
    assert w.state["observe_loose_arrival"] == {"since_s": 10.0}
    out = w.tick(stopped, t=11.0)
    assert out.tracking.status == "arrived" and "observe_loose_arrival" not in w.state
    assert w.state["observe_loose_arrivals"] == [{"time_s": 11.0, "position_error_m": 0.06, "yaw_error_rad": 0.05}]
    w.tick(stopped, t=12.0)
    w.tick(Tracking(status="tracking", speed_mps=0.0, position_error_m=0.2), t=12.5)  # out of tolerance: restart
    assert "observe_loose_arrival" not in w.state
    w = World("observe", remaining=0.0, stop_now=False)
    w.tick(stopped, t=10.0)
    assert "observe_loose_arrival" not in w.state  # not standing


def test_a_backoffs_end_stands_releases_once_and_forces_a_replan_each_second():
    layer = SimpleNamespace(permission=SimpleNamespace(path_check=SimpleNamespace(blocked="occupied"), last_estop=None))
    obstacle = {"layer": layer, "backoff": {"phase": "transport", "time_s": 40.0}}
    w = World("transport", obstacle=obstacle)
    out = w.tick(Tracking(status="arrived"))
    back = obstacle["backoff"]
    assert (out.tracking.status, out.requested_speed) == ("tracking", 0.0)
    assert back["end_s"] == 50.0 and obstacle["backoff_records"] == [
        {"phase": "transport", "start_s": 40.0, "end_s": 50.0, "rear": [1.0, 2.0, 0.0], "escape": False}]
    assert back["released"] is True and back["stop_cause"] == {"path": "occupied", "estop": None}
    np.testing.assert_array_equal(out.rear, [1.5, 2.0, 0.0])
    np.testing.assert_array_equal(w.runner_rear, [1.5, 2.0, 0.0])
    assert obstacle["force_replan"] == "transport" and back["forced_s"] == 50.0
    obstacle.pop("force_replan")
    w.tick(Tracking(status="arrived"), t=50.5)
    assert "force_replan" not in obstacle and len(obstacle["backoff_records"]) == 1
    assert [c[0] for c in w.calls].count("release_after_backoff") == 1
    w.tick(Tracking(status="arrived"), t=51.0)
    assert obstacle["force_replan"] == "transport"
    w = World("transport", obstacle={"layer": layer, "backoff": {"phase": "transport", "time_s": 40.0}}, stop_now=False)
    w.tick(Tracking(status="arrived"))
    assert "released" not in w.obstacle["backoff"] and "force_replan" not in w.obstacle  # standing first


def test_a_loose_observe_arrival_needs_the_heading_within_its_tolerance_too():
    w = World("observe", remaining=0.0)
    w.tick(Tracking(status="tracking", speed_mps=0.0, position_error_m=0.05, yaw_error_rad=-0.11), t=10.0)
    assert "observe_loose_arrival" not in w.state


def test_the_loose_arrival_is_only_for_an_observation_leg():
    """M-a2 (Codex review P2): standing off a path end within the observe tolerance arrives only on an observe
    leg -- an insertion standing 60 mm off its end keeps tracking."""
    w = World("insert", remaining=0.0)
    stopped = Tracking(status="tracking", speed_mps=0.0, position_error_m=0.06, yaw_error_rad=0.05)
    w.tick(stopped, t=10.0)
    out = w.tick(stopped, t=11.5)
    assert out.tracking.status == "tracking" and "observe_loose_arrival" not in w.state
