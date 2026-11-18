"""Design 2026-11-19-known-pallet-start 1 (the S10 case A): a submit whose pallet pose the operator entered in map
coordinates is planned through the preparation (the worker's ``plan_transport``), adopted only on a plan that
succeeded, installed as an observation's plan would be -- none of the observation's own after-steps -- and starts
in the approach, its pallet pose a map estimate recorded as operator_input, never recognition."""

from dataclasses import dataclass, field
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.mission.baseline import MissionBaseline
from forklift_core.mission.targets import Pose

from test_core_async_plan import REAR, Plan, preparing_core

STAGES = ("approach", "insert", "extract", "transport", "withdraw", "return_home")
PICKUP = (3.1, 0.65, 0.0)


@dataclass
class Leg(Plan):
    length_m: float = 1.0
    expanded_nodes: int = 3


@dataclass
class MissionPlan:
    status: str = "success"
    legs: dict = field(default_factory=lambda: {name: Leg("success") for name in STAGES})

    @property
    def success(self):
        return self.status == "success"

    def __getattr__(self, name):
        legs = self.__dict__.get("legs") or {}
        if name in legs:
            return legs[name]
        raise AttributeError(name)


class Tracker:
    def __init__(self):
        self.mode, self.applied, self.holds = "tracking", ((0.1, 0.0, 0.0), 0), 0

    def hold(self, odom_from_base=None):
        self.holds += 1
        self.mode = "holding"


def known_core(**opts):
    core, ms, state, started, inputs = preparing_core()
    calls = []
    core.hooks.show_path = lambda *a: calls.append(("show_path", a[1]))
    core.hooks.export_scene = lambda: calls.append(("export_scene",))
    core.hooks.echo = lambda line: calls.append(("echo", line.split()[0]))
    core.hooks.snapshot = lambda phase: None
    core.hooks.slam_odom_base = lambda: (0.0, 0.0, 0.0)
    core.hooks.slam_error = lambda: None
    core.hooks.planning_target = lambda target: calls.append(("planning_target",)) or (target, "oracle")
    core.hooks.final_straight_prefix = lambda *a: calls.append(("final_straight_prefix",))
    core.slam["tracker"] = Tracker()
    core.obstacle["layer"].known_enabled = False
    core.options.mission_stages, core.options.cusp_heading_hold_phases = STAGES, ("return_home",)
    core.options.speeds.update({name: 0.3 for name in STAGES})
    for k, v in opts.items():
        setattr(core.options, k, v)
    state["planning_wall_s"] = 0.0
    core.baseline = MissionBaseline.capture(core)
    return core, ms, state, started, inputs, calls


def ask(core, inputs, pickup=PICKUP, t=11.0):
    return core.known_pallet_request(t, REAR, inputs, pickup)


def answer(core, plan=None, t=12.0, ok=True):
    value = {"result": plan if plan is not None else MissionPlan(), "trace": [{"stage": "approach"}]}
    core.jobs.answers[core.pending.job] = (ok, value if ok else "worker died\nstack")
    return core.job_hold(t, REAR, 0.0, False)


def test_a_known_pallet_is_planned_by_the_worker_from_where_the_truck_stands():
    core, ms, state, started, inputs, calls = known_core()
    assert ask(core, inputs) is None
    assert core.commands.status == "preparing" and core.pending.site == "prepare" and core.pending.ctx["known"]
    name, payload = core.jobs.submitted[-1]
    assert name == "plan_transport"
    kw = payload["kwargs"]
    assert (kw["target_pickup"].x_m, kw["target_pickup"].y_m, kw["target_pickup"].yaw_rad) == PICKUP
    assert (kw["start_rear"].x_m, kw["start_rear"].y_m) == (1.0, 2.0) and kw["return_to"] == "WAIT"
    assert payload["args"][0] == ("world", "SC_NEW")  # the mission's own scene, not the last mission's
    assert core.pending.ctx["inputs"]["known_pickup"] == list(PICKUP)
    assert ms.phase == "complete" and not started  # nothing changes before the adoption


def test_a_valid_plan_is_adopted_into_the_approach_with_the_operators_pose_recorded():
    core, ms, state, started, inputs, calls = known_core()
    core.options.adoption_zone = "LAST_MISSIONS"
    ask(core, inputs)
    plan = MissionPlan()
    answer(core, plan)
    cmds = core.commands
    assert cmds.status == "running" and cmds.adopted_id == cmds.mission_id
    assert ms.phase == "approach" and ms.phase_started == 12.0
    assert state["transitions"][-1] == {"from": "complete", "to": "approach", "time_s": 12.0,
                                        "mission_id": cmds.mission_id}
    assert set(core.paths) == set(STAGES) and core.paths["approach"] is plan.approach
    assert set(core.trackers) == set(STAGES) and "observe" not in core.trackers
    assert state["planning_target_source"] == "operator_input"
    pickup = core.pocket["planning_pickup"]
    assert (pickup.x_m, pickup.y_m, pickup.yaw_rad) == PICKUP
    assert state["perception"]["perception_pickup_estimate_m"] == {"x_m": 3.1, "y_m": 0.65, "yaw_rad": 0.0}
    assert state["perception"]["source"] == "operator_input, not recognition"
    assert core.obstacle["pickup_estimate"] is pickup
    assert state["planning_traces"][-1] == [{"stage": "approach"}] and state["planning_status"] == "success"
    assert ("planning_target",) not in calls and ("final_straight_prefix",) not in calls  # no observation steps
    assert "near_capture" not in state
    assert ms.lift_command == 0.004 and started and started[0][2] is plan
    o = core.options
    assert (o.scenario, o.return_to_pose, o.observation_waypoints, o.adoption_zone, o.pickup_zone) == (
        "SC_NEW", "WAIT", [], None, "ZONE")


def test_the_approachs_hold_keeps_the_operators_pose_a_map_estimate():
    core, ms, state, started, inputs, calls = known_core()
    ask(core, inputs)
    answer(core)
    assert core.slam["tracker"].holds == 1 and core.in_hold
    pallet_basis = core.hold_basis["pallet"]
    assert pallet_basis[0] is core.pocket["planning_pickup"] and pallet_basis[1] is core.obstacle["pickup_estimate"]
    assert core._held_estimates()[0] is False  # noqa: SLF001 -- not found under the hold: not carried at its release


def test_the_targets_list_records_the_operator_not_recognition():
    core, ms, state, started, inputs, calls = known_core()
    ask(core, inputs)
    answer(core)
    core.slam.pop("transport_scenario", None)
    core.options.scenario = SimpleNamespace(destination=Pose(-5.0, -21.4, -1.57))
    core.refresh_targets()
    pallet = core.targets.pallets["P-1"]
    assert pallet.basis == "operator_input" and (pallet.pose.x_m, pallet.pose.y_m) == (3.1, 0.65)


@pytest.mark.parametrize("plan, ok, reason", [
    (MissionPlan("approach:no_path"), True, "prepare:known:approach:no_path"),
    (None, False, "prepare:error:worker died"),
])
def test_a_plan_that_failed_or_a_worker_error_adopts_nothing(plan, ok, reason):
    core, ms, state, started, inputs, calls = known_core()
    ask(core, inputs)
    answer(core, plan, ok=ok)
    assert core.commands.status == "failed" and core.commands.end_reason == reason
    assert ms.phase == "complete" and not started and core.paths == {} and "planning_target_source" not in state


@pytest.mark.parametrize("change, why", [
    (lambda core, k: setattr(core.slam["tracker"], "applied", ((0.2 * k, 0.0, 0.0), k)), "correction"),
    (lambda core, k: core.targets.add_pallet(f"P-{9 + k}", "epal6", Pose(9.0, 9.0 + k, 0.0)), "targets"),
])
def test_an_answer_on_a_changed_state_is_asked_again_then_fails(change, why):
    core, ms, state, started, inputs, calls = known_core()
    ask(core, inputs)
    change(core, 1)
    core.obstacle["applied"] = core.slam["tracker"].applied[0]
    answer(core)
    assert core.commands.status == "preparing" and not started  # dropped, asked again by the runner
    ask(core, inputs, t=12.5)
    change(core, 2)
    core.obstacle["applied"] = core.slam["tracker"].applied[0]
    answer(core, t=13.0)
    assert core.commands.status == "failed" and core.commands.end_reason == f"prepare:{why}" and not started


def test_a_late_answer_of_a_cancelled_preparation_is_not_taken():
    core, ms, state, started, inputs, calls = known_core()
    ask(core, inputs)
    core.commands.terminate("cancelled", 11.5, "operator")
    answer(core)
    assert core.commands.status == "cancelled" and not started and ms.phase == "complete"


@pytest.mark.parametrize("setup, why", [
    (lambda core: setattr(core, "near", {}), "near_tracking"),
    (lambda core: setattr(core.options, "bottom_deck_check", "structure"), "bottom_deck_check"),
    (lambda core: setattr(core, "slam", None), "no_slam"),
    (lambda core: setattr(core.slam["tracker"], "mode", "warming_up"), "slam_not_tracking"),
    (lambda core: setattr(core.slam["tracker"], "mode", "holding"), "slam_not_tracking"),
    (lambda core: setattr(core, "jobs", None), "no_worker"),
    (lambda core: setattr(core.obstacle["layer"], "known_enabled", True), "known_pallets_on"),
])
def test_a_known_start_is_refused_without_its_conditions(setup, why):
    core, ms, state, started, inputs, calls = known_core()
    setup(core)
    asked = len(core.jobs.submitted) if core.jobs is not None else 0
    assert ask(core, inputs) == why
    assert core.commands.status == "failed" and core.commands.end_reason == f"prepare:known:{why}"
    assert (len(core.jobs.submitted) if core.jobs is not None else 0) == asked and not started


def test_an_unlocated_pallet_registered_again_by_the_operator_is_available():
    core, ms, state, started, inputs, calls = known_core()
    tb = core.targets
    tb.unlocate("P-1")
    assert tb.pallets["P-1"].state == "unlocated"
    tb.update_pose("P-1", Pose(3.0, 0.6, 0.0))  # a pose alone (no basis) places nothing
    assert tb.pallets["P-1"].state == "unlocated"
    tb.update_pose("P-1", Pose(3.0, 0.6, 0.0), basis="operator_input", uncertainty_m=0.05)
    assert tb.pallets["P-1"].state == "available" and tb.pallets["P-1"].basis == "operator_input"
    assert np.isclose(tb.pallets["P-1"].pose.x_m, 3.0)


def test_a_preparation_asked_again_and_refused_ends_the_mission():
    """Codex 4a P2: the runner asks again after a dropped answer; a refusal then must not leave it preparing."""
    core, ms, state, started, inputs, calls = known_core()
    ask(core, inputs)
    core.slam["tracker"].applied = ((0.3, 0.0, 0.0), 1)
    answer(core)
    assert core.commands.status == "preparing" and core.pending is None
    core.slam["tracker"].mode = "warming_up"
    assert ask(core, inputs, t=12.5) == "slam_not_tracking"
    assert core.commands.status == "failed" and core.commands.end_reason == "prepare:known:slam_not_tracking"


def test_the_first_frame_stand_is_the_missions_and_starts_clean_in_the_next_one():
    core, ms, state, started, inputs, calls = known_core()
    core.pocket["first_frame_since"] = 3.0  # the last mission's stand, never finished
    core.baseline.restore(core)  # its end (end_reset)
    assert "first_frame_since" not in core.pocket
    ask(core, inputs)
    answer(core)
    assert "first_frame_since" not in core.pocket


def test_a_known_start_never_inserts_without_its_check():
    """Codex 4a P2: an approach arrival with no check built (the first-frame stand skipped) does not go in."""
    core, ms, state, started, inputs, calls = known_core(pocket_check=True)
    ask(core, inputs)
    answer(core)
    assert ms.phase == "approach" and core.pocket["check"] is None
    with pytest.raises(RuntimeError, match="pocket_check_missing_at_insert"):
        core.arrive(20.0, SimpleNamespace(position_error_m=0.0, yaw_error_rad=0.0))
    assert ms.phase == "approach"
