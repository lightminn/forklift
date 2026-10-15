"""M-a1 slice 4a-i: the follower's tick and the rules that rewrite its result, moved from the runner (behaviour,
state, port requests and their order, effects up to a refusal -- the plan's M-a1 criterion)."""

from dataclasses import dataclass, replace
from types import SimpleNamespace

import pytest

from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState

REAR = (1.0, 2.0, 0.0)


@dataclass(frozen=True)
class Tracking:
    status: str = "tracking"
    speed_mps: float = 0.4
    cusp_pending: bool = False
    progress_m: float = 0.5


class Tracker:
    def __init__(self, calls, result, remaining=3.0):
        self.calls, self.result, self.remaining, self.leg_index = calls, result, remaining, 2

    def nominal_duration_s(self):
        return 10.0

    def update(self, rear, speed, dt):
        self.calls.append(("update", (rear, speed, dt), {}))
        return self.result

    def remaining_to_goal_m(self):
        return self.remaining


def make(phase="transport", *, result=Tracking(), slam=None, near=None, obstacle=None, remaining=3.0,
         cusp=lambda raw, tracking: (tracking, False), watch=None):
    calls = []

    def log(name, value=None):
        def call(*args, **kwargs):
            calls.append((name, args, kwargs))
            return value(*args) if callable(value) else value
        return call

    def require(ok, reason):
        calls.append(("require", (bool(ok), reason), {}))
        if not ok:
            raise RuntimeError(reason)

    holds = SimpleNamespace(sync=log("holds.sync"), tick=log("holds.tick"))
    hooks = SimpleNamespace(
        require=require, dump_tracking=log("dump_tracking"),
        cusp_holds=lambda: holds, slam_odom_base=lambda: "ODOM", slam_error=lambda: "ERR",
    )
    tracker = Tracker(calls, result, remaining)
    ms = MissionState(phase=phase, phase_started=0.0)
    state = {"transitions": [], "cusp_heading_watch": watch}
    core = MissionCore(ms, state=state, obstacle=obstacle, slam=slam, pocket={"check": None}, paths={},
                       trackers={phase: tracker}, geometry=None,
                       settings={"drive_acceleration_mps2": 0.5}, pallet_geometry=None, hooks=hooks, near=near,
                       options=SimpleNamespace(dwells=None, safety_stops=None,
                                               cusp_heading_hold_phases=("observe", "return_home")))
    # The cusp rules are MissionCore methods since slice 6a; stand-ins on the instance as before.
    core.cusp_heading_end = lambda tr: {"end": tr.status}
    core.cusp_heading_offset = log("offset", (0.01, 0.02))
    core.cusp_tick = lambda t, rear, raw, tracking, stand: calls.append(("cusp_tick", (raw, tracking, stand), {})) or cusp(
            raw, tracking)
    return core, ms, state, tracker, calls


def test_the_tracker_runs_once_and_its_result_passes_the_cusp_rule_into_last_tracking():
    core, ms, _, tracker, calls = make()
    out = core.follow(5.0, REAR, 0.3, 1 / 120)
    assert calls[1] == ("update", (REAR, 0.3, 1 / 120), {})
    assert calls[2][0] == "holds.sync" and calls[2][1][:2] == (5.0, "transport") and calls[2][1][3:] == (2, {"end": "tracking"})
    assert ("cusp_tick", (Tracking(), Tracking(), False), {}) in calls
    assert out.tracking == Tracking() and out.cusp_hold is False and ms.last_tracking == Tracking()
    assert "holds.tick" not in [c[0] for c in calls]  # transport holds no cusp heading


def test_a_timeout_dumps_the_last_result_then_refuses():
    core, ms, _, _, calls = make()
    ms.last_tracking = "LAST"
    with pytest.raises(RuntimeError, match="Tracking timeout in transport"):
        core.follow(40.0, REAR, 0.3, 1 / 120)  # limit max(30, 3 * 10 + 10) = 40
    assert calls[0] == ("dump_tracking", ("timeout", "LAST"), {})


def test_the_cusp_heading_watch_ticks_with_the_offset_only_for_the_watched_tracker():
    core, _, state, tracker, calls = make("observe", result=Tracking(cusp_pending=True))
    state["cusp_heading_watch"] = {"tracker": id(tracker)}
    core.follow(5.0, REAR, 0.3, 1 / 120)
    (tick,) = [c for c in calls if c[0] == "holds.tick"]
    assert tick[1][4:] == ({"end": "tracking"}, True, 0.4, (0.01, 0.02))
    state["cusp_heading_watch"] = {"tracker": -1}
    core.follow(5.1, REAR, 0.3, 1 / 120)
    assert [c for c in calls if c[0] == "holds.tick"][-1][1][-1] is None


def test_a_re_observation_stand_arrives_at_once_and_tells_the_cusp_rule():
    core, _, _, _, calls = make("observe", near={"retry_stand": True})
    out = core.follow(5.0, REAR, 0.3, 1 / 120)
    assert (out.tracking.status, out.tracking.speed_mps) == ("arrived", 0.0)
    assert [c for c in calls if c[0] == "cusp_tick"][0][1][2] is True


@pytest.mark.parametrize("backoff, status, speed, remaining, expect", [
    ({"end_s": 1.0}, "tracking", 0.3, 1.0, "arrived"),
    ({}, "arrived", 0.3, 1.0, "arrived"),
    ({}, "failed", 0.001, 1.0, "arrived"),  # standing on an endpoint failure
    ({}, "tracking", 0.001, 0.0, "arrived"),  # standing at its end
    ({"stand_ticks": 119}, "tracking", 0.001, 1.0, "arrived"),  # the 120th standing tick
    ({}, "failed", 0.3, 1.0, "braking"),  # still rolling: brake and take the end standing
    ({}, "tracking", 0.3, 1.0, "tracking"),
])
def test_an_escape_end_is_the_backoff_end(backoff, status, speed, remaining, expect):
    obstacle = {"backoff": {"phase": "transport", "escape": True, **backoff}}
    core, *_ = make(result=Tracking(status=status), obstacle=obstacle, remaining=remaining)
    out = core.follow(5.0, REAR, speed, 1 / 120)
    assert out.tracking.status == expect
    assert obstacle["backoff"]["stand_ticks"] == (backoff.get("stand_ticks", 0) + 1 if speed < 0.01 else 0)


def test_slam_holds_within_half_a_metre_plus_the_braking_distance_of_a_final_goal():
    tracker_ = SimpleNamespace(mode="tracking", hold=lambda **k: holds.append(k))
    holds = []
    slam = {"tracker": tracker_, "holds": [], "stop_now": False}
    core, *_ = make(slam=slam, remaining=0.5 + 0.4**2 / (2 * 0.5) + 0.001)
    core.follow(5.0, REAR, 0.4, 1 / 120)
    assert holds == []
    core, *_ = make(slam=slam, remaining=0.5 + 0.4**2 / (2 * 0.5))
    core.follow(5.0, REAR, 0.4, 1 / 120)
    assert holds == [{"odom_from_base": "ODOM"}]
    assert slam["holds"] == [{"phase": "transport", "time_s": 5.0, "event": "hold", "error": "ERR"}]
    for phase in ("insert", "withdraw"):  # docking and withdrawal phases are not judged at a final goal here
        core, *_ = make(phase, slam=slam, remaining=0.1)
        core.follow(5.0, REAR, 0.4, 1 / 120)
    assert len(holds) == 1


def test_the_cusp_rule_may_hold_the_leg():
    core, *_ = make(cusp=lambda raw, tracking: (replace(tracking, speed_mps=0.0), True))
    out = core.follow(5.0, REAR, 0.3, 1 / 120)
    assert out.cusp_hold is True and out.tracking.speed_mps == 0.0
