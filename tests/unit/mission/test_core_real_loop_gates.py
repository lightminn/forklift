"""docs/design/2026-11-19-real-loop.md (4c): the core's real-truck-only changes -- the insert end waits for the loop's
still span and re-checks the pose and a fresh D5 answer before the lift; end_reset takes the loop's release decision;
the direction checks read the outputs that may still act (same direction included); an unrecoverable hold opens the
relocalization lock. The simulation's paths are untouched."""

from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control.path_tracking import TrackerConfig
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.safety.truth_access import SafetySpeed

from test_core_arrival import make


class Check:
    def __init__(self, answer=(float("inf"), "ok")):
        self.answer, self.asked = answer, []

    def limit(self, *args):
        self.asked.append(args)
        return self.answer

    def summary(self):
        return {}


def insert_core(*, stop_now=True, rear=(2.0, 0.0, 0.0), check=None, real=True):
    goal = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    tracker = SimpleNamespace(path_arrays=lambda: (goal, np.array([1, 1]), np.zeros(2)),
                              config=TrackerConfig(position_tolerance_m=0.008, yaw_tolerance_rad=0.02))
    slam = {"stop_now": stop_now, "tracker": SimpleNamespace(mode="holding"), "holds": []}
    core, ms, state, log = make("insert", slam=slam, trackers={"insert": tracker})
    core.options.real_vehicle = real
    core.options.pocket_check = True
    core.hooks.rear = lambda: np.array(rear, dtype=float)
    core.pocket["check"] = check if check is not None else Check()
    core.pocket["frames_read"] = 3
    core.obstacle = {"layer": SimpleNamespace(exempt="E", depth_support="D", known_enabled=False)}
    core.commands.status, core.commands.mission_id = "running", "m1"
    return core, ms, state, log


def tracking():
    return SimpleNamespace(position_error_m=0.004, yaw_error_rad=0.01)


def test_the_real_insert_end_waits_for_the_still_span():
    core, ms, state, log = insert_core(stop_now=False)
    out = core.arrive(20.0, tracking())
    assert ms.phase == "insert" and not out.stop and "insertion_error" not in state


def test_the_real_insert_end_lifts_after_a_fresh_d5_answer_at_the_standing_pose():
    check = Check()
    core, ms, state, log = insert_core(check=check)
    core.arrive(20.0, tracking())
    assert ms.phase == "lift"
    (t, rear, kappa, direction, speed, lift) = check.asked[-1]
    assert t == 20.0 and tuple(rear) == (2.0, 0.0, 0.0) and speed == 0.0


@pytest.mark.parametrize("answer, why", [
    ((0.0, "depth_stale"), "insert_end:pocket:depth_stale"),
    ((0.0, "pocket_obstacle"), "insert_end:pocket:pocket_obstacle"),
    ((0.0, "pocket_invalid:x"), "insert_end:pocket:pocket_invalid:x"),
])
def test_a_stale_or_refusing_d5_at_the_standing_pose_ends_without_the_lift(answer, why):
    core, ms, state, log = insert_core(check=Check(answer))
    core.arrive(20.0, tracking())
    assert ms.phase == "insert" and core.commands.status == "recovery_required" and core.commands.end_reason == why


def test_a_residual_motion_past_the_arrival_window_ends_without_the_lift():
    core, ms, state, log = insert_core(rear=(2.03, 0.0, 0.0))  # 3 cm deeper than the goal after the first arrival
    core.arrive(20.0, tracking())
    assert ms.phase == "insert" and core.commands.end_reason == "insert_end:pose_out_of_range"


def test_the_simulation_insert_end_is_as_before():
    core, ms, state, log = insert_core(stop_now=False, real=False, check=Check((0.0, "depth_stale")))
    core.arrive(20.0, tracking())
    assert ms.phase == "lift"


# --- end_reset's release decision ---------------------------------------------------------------------------------

def released_core(stop_now):
    core, ms, state, log = make("transport", slam={"stop_now": stop_now, "tracker": SimpleNamespace(mode="holding"),
                                                   "holds": [], "pending_release": None,
                                                   "release_wait_from": None})
    calls = []
    core.end_release = lambda t, deferred: calls.append(("released", t))
    core.cargo.release_allowed = lambda: True
    core._held_estimates = lambda: (False, False)  # noqa: SLF001
    core.capture_recovery_context = lambda: None
    core._open_field_recovery = lambda t: None  # noqa: SLF001
    core.refresh_targets = lambda: None
    core.baseline = None
    core.commands.mission = SimpleNamespace(pallet_id="P", destination_id="D")
    return core, calls


@pytest.mark.parametrize("stop_now, allowed, released", [
    (True, None, True), (False, None, False),  # as before
    (True, False, False),  # the loop says no: deferred though the last tick's stop_now was true
    (False, True, True),
])
def test_end_reset_takes_the_loops_release_decision(stop_now, allowed, released):
    core, calls = released_core(stop_now)
    core.end_reset(30.0, release_allowed=allowed)
    assert bool(calls) is released
    assert (core.end_release_due is None) is released


def test_an_unrecoverable_hold_opens_the_relocalization_lock_and_its_confirmation_drops_the_deferred_release():
    core, calls = released_core(False)
    core.options.real_vehicle = True
    core.end_reset(30.0, release_allowed="relocalize")
    assert not calls and core.end_release_due is None
    gen = core.relocalize_generation()
    assert gen is not None and core.cargo.relocalize_required == gen
    assert core.relocalized(31.0, gen) is True and core.relocalize_generation() is None


# --- the direction checks: outputs that may still act ---------------------------------------------------------------

def test_the_real_direction_check_takes_the_same_directions_residual_speed():
    core, ms, state, log = make("approach")
    core.options.real_vehicle = True
    state["command_speed"] = 0.0
    state["outstanding_motion"] = {1: 0.15}  # a forward 0.15 m/s DRIVE may still act
    asked = []
    out = core._direction_limits(lambda d, cap: asked.append((d, cap)) or (1.0, "ok"),  # noqa: SLF001
                                 SafetySpeed(0.0, 0.0, (1,)), 1, 0.03)
    assert (1, 0.15) in asked  # checked at the residual 0.15, not the requested 0.03


def test_the_real_direction_check_takes_the_other_directions_residual_too():
    core, ms, state, log = make("approach")
    core.options.real_vehicle = True
    state["command_speed"] = 0.0
    state["outstanding_motion"] = {-1: 0.1}
    asked = []
    core._direction_limits(lambda d, cap: asked.append((d, cap)) or (1.0, "ok"),  # noqa: SLF001
                           SafetySpeed(0.0, 0.0, (1,)), 1, 0.05)
    assert (-1, 0.1) in asked


def test_the_simulation_direction_check_is_as_before():
    core, ms, state, log = make("approach")
    state["outstanding_motion"] = {1: 0.15}
    asked = []
    core._direction_limits(lambda d, cap: asked.append((d, cap)) or (1.0, "ok"),  # noqa: SLF001
                           SafetySpeed.exact(0.0), 1, 0.03)
    assert asked == [(1, 0.03)]


def test_a_faster_unconfirmed_output_the_same_way_than_allowed_holds_the_wheels():
    """Codex 4c impl P1-9: residual +0.15 forward, a request of +0.03, the layer allowing 0.05 that way -> a hold."""
    core, ms, state, log = make("approach")
    core.options.real_vehicle = True
    state["command_speed"] = 0.0
    state["outstanding_motion"] = {1: 0.15}
    allowed, why, hold, _, _ = core._direction_limits(lambda d, cap: (0.05, "ok"),  # noqa: SLF001
                                                      SafetySpeed(0.0, 0.0, (1,)), 1, 0.03)
    assert (allowed, why, hold) == (0.0, "residual_over_allowed", True)
    assert core._residual_speed(1) == 0.15 and core._residual_speed(-1) == 0.0  # noqa: SLF001
    core.options.real_vehicle = False
    assert core._residual_speed(1) == 0.0  # noqa: SLF001 -- the simulation has no such record


@pytest.mark.parametrize("short_m, lifts", [(0.0095, True), (0.0105, False)])
def test_the_insert_end_recheck_allows_a_small_settle_past_the_arrival_tolerance(short_m, lifts):
    """6a-2b: the arrival latches within 8 mm; a stand that settles the truck up to 2 mm further still lifts, more
    ends as pose_out_of_range (the HIL model drew back 0.3 mm from an arrival 8.0 mm short)."""
    core, ms, state, log = insert_core(rear=(2.0 - short_m, 0.0, 0.0))
    core.arrive(20.0, tracking())
    if lifts:
        assert ms.phase == "lift", core.commands.end_reason
    else:
        assert ms.phase == "insert" and core.commands.end_reason == "insert_end:pose_out_of_range"
