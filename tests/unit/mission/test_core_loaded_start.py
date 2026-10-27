"""Persistent console plan D5 6c (docs/plans/2026-10-18-persistent-live-console.md, items 4, 4a and 5): a loaded
submit's preparation asks the worker's ``loaded_delivery`` job, its plan is adopted straight into transport without
a transition (no SLAM release booked, the cargo untouched), and the truck stands in a start wait until the lift is
at the carry height, the first live leg is taken and the docking armed -- failed after 60 s outside the operator's
pauses, or when the cargo changed."""

import numpy as np
import pytest

from forklift_core.mission.baseline import CORE_FIELDS
from forklift_core.planning.pallet_mission import MissionPlan

from test_core_async_plan import REAR, Plan, preparing_core


def loaded_core(*, cargo_ok=True):
    core, ms, state, started, inputs = preparing_core()
    core.commands.mission_loaded = True
    core.settings.update(lift_target_m=0.08)
    core.options.speeds.update(transport=0.4, withdraw=0.2, return_home=0.5)
    core.options.cusp_heading_hold_phases = ("return_home",)
    core.loaded_lift_band = (0.003, 0.013)
    start = core.hooks.live_mission_start

    def live_mission_start(t, inputs, plan, ready):
        core.slam["docking"] = {"status": "pending"}  # the runner installs the destination's docking reference
        start(t, inputs, plan, ready)

    core.hooks.live_mission_start = live_mission_start
    checks = []
    core.hooks.loaded_cargo_ok = lambda: checks.append(True) or cargo_ok
    inputs = {**inputs, "waypoints": [], "pickup_zone": None}
    return core, ms, state, started, inputs, checks


def delivery(status="success"):
    if status != "success":
        return MissionPlan(False, status)
    return MissionPlan(True, "success", None, None, None, Plan("success"), Plan("success"), Plan("success"))


def adopt(core, inputs, plan=None, t=12.0):
    core.prepare_request(11.0, REAR, inputs, {"args": ()}, job="loaded_delivery")
    core.jobs.answers[core.pending.job] = (True, {"result": plan if plan is not None else delivery(), "wall_s": 0.1})
    core.job_hold(t, REAR, 0.0, False)


def test_a_loaded_preparation_asks_the_loaded_delivery_job():
    core, ms, state, started, inputs, _ = loaded_core()
    core.prepare_request(11.0, REAR, inputs, {"args": ()}, job="loaded_delivery")
    assert core.commands.status == "preparing" and core.jobs.submitted[-1][0] == "loaded_delivery"
    assert core.pending.site == "prepare" and core.pending.ctx["loaded"] is True


def test_the_loaded_plan_is_adopted_into_transport_without_a_transition():
    core, ms, state, started, inputs, _ = loaded_core()
    cargo_before = (core.cargo.relation, core.cargo.pallet_id)
    arms = []
    core.arm_docking = lambda: arms.append(ms.phase)
    core.options.adoption_zone = "LAST_MISSIONS"
    plan = delivery()
    adopt(core, inputs, plan)
    assert core.options.adoption_zone is None  # a loaded start adopts no pickup (plan 2026-10-28-adoption-zone-check)
    cmds = core.commands
    assert cmds.status == "running" and cmds.adopted_id == cmds.mission_id
    assert ms.phase == "transport" and ms.phase_started == 12.0 and state["phase"] == "transport"
    assert set(core.paths) == {"transport", "withdraw", "return_home"} and core.paths["transport"] is plan.transport
    assert set(core.trackers) == {"transport", "withdraw", "return_home"}
    assert core.trackers["transport"].config.cruise_speed_mps == 0.4
    assert core.trackers["transport"].config.position_tolerance_m == 0.008
    assert core.trackers["return_home"].config.yaw_tolerance_rad == 0.03
    assert core.slam.get("pending_release") is None  # transition() would have booked a release (Codex 6c P2-1)
    assert (core.cargo.relation, core.cargo.pallet_id) == cargo_before
    assert arms == ["transport"] and ms.lift_command == 0.004  # the ramp starts from the measured lift
    assert state["transitions"][-1] == {"from": "complete", "to": "transport", "time_s": 12.0,
                                        "mission_id": cmds.mission_id, "loaded": True}
    assert started and started[0][2] is plan
    assert core.loaded_wait is not None and state["loaded_lift_wait"] == [{"start_s": 12.0}]
    assert not core.obstacle.get("live_replanned", {}).get("transport")  # the live leg is still to plan
    o = core.options
    assert (o.scenario, o.return_to_pose, o.home_rear, o.pickup_bounds, o.observe_bounds, o.pickup_zone) == (
        "SC_NEW", "WAIT", "WAIT", None, None, None)


@pytest.mark.parametrize("status", ["transport:no_path", "transport:no_final_straight", "return_home:no_path"])
def test_a_loaded_plan_that_failed_fails_the_mission_with_its_stage(status):
    core, ms, state, started, inputs, _ = loaded_core()
    adopt(core, inputs, delivery(status))
    assert core.commands.status == "failed" and core.commands.end_reason == f"prepare:loaded:{status}"
    assert ms.phase == "complete" and not started and core.loaded_wait is None


def test_a_cargo_that_changed_before_the_answer_drops_it_and_fails_the_mission():
    core, ms, state, started, inputs, checks = loaded_core(cargo_ok=False)
    adopt(core, inputs)
    assert checks and core.commands.status == "failed"
    assert core.commands.end_reason == "prepare:loaded:cargo_changed" and not started


def ready_leg(core):
    core.obstacle.setdefault("live_replanned", {})["transport"] = True
    core.slam["docking"]["status"] = "armed"


def test_the_start_wait_holds_until_the_lift_the_live_leg_and_the_docking_are_ready():
    core, ms, state, started, inputs, checks = loaded_core()
    adopt(core, inputs)
    assert core.loaded_start_wait(13.0)  # lift at 0.004, no live leg
    core.hooks.lift_position = lambda: 0.072  # 0.08 - 0.003 sag, within 0.013
    assert core.loaded_start_wait(14.0)  # the live leg not yet taken
    core.obstacle.setdefault("live_replanned", {})["transport"] = True
    core.pending = type("P", (), {"site": "live_leg"})()
    assert core.loaded_start_wait(15.0)  # asked, not taken
    core.pending = None
    core.slam["docking"]["status"] = "pending"
    assert core.loaded_start_wait(16.0)  # not armed
    core.slam["docking"]["status"] = "armed"
    checks.clear()
    assert not core.loaded_start_wait(17.0)
    assert checks and core.loaded_wait is None and ms.phase_started == 17.0  # the tracking clock starts here
    assert state["loaded_lift_wait"] == [{"start_s": 12.0, "end_s": 17.0}]
    assert core.commands.status == "running" and not core.loaded_start_wait(18.0)


def test_a_lift_outside_the_loaded_band_keeps_the_wait():
    core, ms, state, started, inputs, _ = loaded_core()
    adopt(core, inputs)
    ready_leg(core)
    core.hooks.lift_position = lambda: 0.058  # 0.019 below the sagged target
    assert core.loaded_start_wait(13.0)
    core.hooks.lift_position = lambda: 0.091  # 0.014 above it
    assert core.loaded_start_wait(14.0)


def test_a_live_leg_with_no_delivery_straight_fails_the_mission():
    core, ms, state, started, inputs, _ = loaded_core()
    adopt(core, inputs)
    core.obstacle.setdefault("live_replanned", {})["transport"] = True
    core.slam["docking"]["status"] = "no_final_straight"
    assert core.loaded_start_wait(13.0)
    assert core.commands.status == "failed" and core.commands.end_reason == "prepare:loaded:no_final_straight"


def test_the_wait_fails_after_60_s_outside_the_operators_pauses_naming_what_is_left():
    core, ms, state, started, inputs, _ = loaded_core()
    adopt(core, inputs)
    core.commands.pauses = [[20.0, 50.0]]  # 30 s paused: not counted
    core.commands.waits = [[12.0, 80.0]]  # waiting for a plan: counted (Codex 6c round 5 P3)
    assert core.loaded_start_wait(100.0) and core.commands.status == "running"  # 88 - 30 = 58 s
    assert core.loaded_start_wait(103.0)
    assert core.commands.status == "failed" and core.commands.end_reason == "prepare:loaded:start_wait:lift,leg,docking"


def test_the_wait_does_nothing_while_the_mission_is_not_running():
    core, ms, state, started, inputs, _ = loaded_core()
    adopt(core, inputs)
    ready_leg(core)
    core.hooks.lift_position = lambda: 0.077
    core.commands.status = "paused"
    assert core.loaded_start_wait(13.0) and core.loaded_wait is not None


def test_a_cargo_that_changed_during_the_wait_fails_the_mission_at_its_release():
    core, ms, state, started, inputs, _ = loaded_core()
    adopt(core, inputs)
    ready_leg(core)
    core.hooks.lift_position = lambda: 0.077
    core.hooks.loaded_cargo_ok = lambda: False
    assert core.loaded_start_wait(13.0)
    assert core.commands.status == "failed" and core.commands.end_reason == "prepare:loaded:cargo_changed"


def test_the_wait_is_the_missions_and_goes_with_its_end():
    assert "loaded_wait" in CORE_FIELDS
    core, ms, state, started, inputs, _ = loaded_core()
    adopt(core, inputs)
    assert core.loaded_wait is not None
    core.baseline.restore(core)  # what end_reset ends with
    assert core.loaded_wait is None


def test_an_unloaded_mission_has_no_start_wait():
    core, ms, state, started, inputs = preparing_core()
    assert not core.loaded_start_wait(13.0) and getattr(core, "loaded_wait", None) is None


def test_the_wait_record_is_the_missions_handed_to_its_record_and_not_carried_on():
    """Codex 6c impl round 1 P3: the loaded_lift_wait record piled up across missions and missed the ended one's."""
    core, ms, state, started, inputs, _ = loaded_core()
    adopt(core, inputs)
    records = core.baseline.restore(core)
    assert records["state"]["loaded_lift_wait"] == [{"start_s": 12.0}] and "loaded_lift_wait" not in state


def test_the_lift_ramps_while_the_loaded_start_waits_for_a_plan():
    """Codex 6c impl round 1 P2: a plan wait stopped the lift ramp too, so a late first live leg failed start_wait:lift."""
    core, ms, state, started, inputs, _ = loaded_core()
    assert not core.lift_ramps_in_plan_wait()
    adopt(core, inputs)
    assert core.lift_ramps_in_plan_wait()
    ready_leg(core)
    core.hooks.lift_position = lambda: 0.077
    core.loaded_start_wait(13.0)
    assert not core.lift_ramps_in_plan_wait()  # the wait is over: a plan wait stands as before


def test_the_runner_ramps_the_lift_in_a_plan_wait_only_for_the_loaded_start():
    """Codex 6c impl round 2 P3: the helper alone passed with the runner's condition removed. The ramp itself moves the
    lift command toward the carry target during the wait, and the runner's lift call is gated on the helper."""
    import re
    from pathlib import Path

    core, ms, state, started, inputs, _ = loaded_core()
    core.settings.update(lift_rate_mps=0.05)
    adopt(core, inputs)
    before = ms.lift_command
    core.lift_and_timers(13.0, 0.1)
    assert ms.lift_command == pytest.approx(before + 0.005)
    source = (Path(__file__).resolve().parents[3] / "sim/isaac/run_transport.py").read_text()
    gate = re.search(r"if \(not paused_ and not live_abort_ and \(not plan_wait_ or core\.lift_ramps_in_plan_wait\(\)\)"
                     r"\s+and core\.lift_and_timers\(t, dt\)\):", source)
    assert gate is not None
