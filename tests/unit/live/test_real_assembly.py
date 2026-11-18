"""Design 2026-11-19-real-assembly 2, 4, 5: the real core assembled behind fail_closed_hooks, its boot inputs, the
S10-A targets (registered unlocated: only the operator's map pose makes a pick), and the known-pallet preparation and
adoption reading only the hooks of ADOPTION_HOOKS."""

import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.live import real_assembly as asm
from forklift_core.live import real_autonomy as ra
from forklift_core.mission.commands import Submit
from forklift_core.plant.kinematics import RealKinematics
from forklift_core.planning.pallet_mission import SyntheticMissionGeometry

from test_teleop_bridge_cycle import VEH

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tests/unit/mission"))
from test_core_async_plan import FakeJobs  # noqa: E402
from test_core_known_pallet import STAGES, MissionPlan, Tracker  # noqa: E402

KIN = RealKinematics(VEH)
CAP = min(-KIN.cfg.steering.curvature_range[0], KIN.cfg.steering.curvature_range[1])
SETTINGS = {"drive_acceleration_mps2": 0.3, "lift_rate_mps": 0.05, "lift_target_m": 0.08,
            "steering_command_rate_rad_s": 1.0, "withdraw_speed_mps": 0.05, "tracker_curvature_inv_m": CAP}
PALLET = (3.0, 0.2, 0.0)


class Recording(asm.RealHooks):
    def __init__(self, **kw):
        super().__init__(**kw)
        object.__setattr__(self, "read", set())

    def __getattribute__(self, name):
        if not name.startswith("_") and name != "read":
            object.__getattribute__(self, "read").add(name)
        return object.__getattribute__(self, name)


def cfg():
    base = ra.load_s10a(ROOT / "config/real/s10a.yaml")
    return replace(base, map_id="hall-1", pallet_depth_m=0.6, pallet_width_m=0.4, check_pallet_pose=(3.0, 0.0, 0.0))


GEOM = SyntheticMissionGeometry()
PLAN = SimpleNamespace(curvature_limit_inv_m=CAP)


def build(hooks_kw=None, **over):
    started, records = [], []
    hooks = Recording(lift_m=lambda: 0.003, odom_base=lambda: (0.0, 0.0, 0.0), ready=lambda t, inputs: None,
                      started=lambda *a: started.append(a), record=lambda *a: records.append(a), geometry=GEOM,
                      **(hooks_kw or {}))
    layer = SimpleNamespace(known_enabled=False, refresh=lambda *a, **k: None,
                            permission=SimpleNamespace(path_check=None))
    kw = dict(cfg=cfg(), loaded_map_id="hall-1", geometry=GEOM, pallet_geometry=None,
              kinematics=KIN, settings=SETTINGS, planner_config=PLAN, travel_config=PLAN,
              speeds={name: 0.1 for name in STAGES}, mission_stages=STAGES,
              curvatures={"preview": CAP}, layer=layer, applied=(0.0, 0.0, 0.0),
              slam_tracker=Tracker(), jobs=FakeJobs(), hooks=hooks,
              extra_options={"cusp_heading_hold_rad": 0.05, "cusp_heading_hold_phases": ("return_home",),
                             "tracker_profile": None})
    kw.update(over)
    return asm.assemble(**kw), hooks, started


def test_the_boot_inputs_have_their_values_and_meaning():
    core, hooks, _ = build()
    assert core.ms.phase == "idle" and core.near is None and core.paths == {} and core.trackers == {}
    assert core.state["transitions"] == [] and core.pocket == {"check": None, "frames_read": 0}
    assert core.obstacle["version"] == 0 and core.obstacle["applied"] == (0.0, 0.0, 0.0)
    assert core.slam["stop_now"] is False and core.slam["docking"] == {}
    assert core.options.dt == 1 / 50 and core.options.real_vehicle and core.baseline is not None
    assert type(core.hooks).__name__ == "FailClosedHooks"


def test_the_pallet_is_no_pick_until_the_operator_registers_its_map_pose():
    core, hooks, _ = build()
    p = core.targets.pallets["P-S10"]
    assert p.state == "unlocated" and p.pose is None
    assert asm.s10_submit_refusal(core, "P-S10") == "pallet_not_registered"
    assert core.targets.check_submit("P-S10", "D-S10", core.targets.revision) == "invalid"
    rev = asm.register_pallet(core, "P-S10", PALLET)
    assert asm.s10_submit_refusal(core, "P-S10") is None
    assert core.targets.check_submit("P-S10", "D-S10", rev) is None


@pytest.mark.parametrize("over, why", [
    ({"loaded_map_id": "hall-2"}, "map_id_mismatch:hall-2"),
    ({"settings": {k: v for k, v in SETTINGS.items() if k != "tracker_curvature_inv_m"}},
     "setting_missing:tracker_curvature_inv_m"),
    ({"speeds": {name: 0.5 for name in STAGES}}, "speed_over_cap:approach"),
    ({"curvatures": {"preview": CAP / 2}}, "curvature_caps_differ"),
])
def test_a_structural_problem_refuses_the_assembly(over, why):
    with pytest.raises(asm.AssemblyRefused) as err:
        build(**over)
    assert any(r.startswith(why) for r in err.value.reasons)


def test_the_check_pallet_pose_is_needed():
    with pytest.raises(asm.AssemblyRefused) as err:
        build(cfg=replace(cfg(), check_pallet_pose=None))
    assert "check_pallet_pose_unset" in err.value.reasons


def test_the_known_preparation_and_adoption_read_only_the_adoption_hooks():
    core, hooks, started = build()
    rev = asm.register_pallet(core, "P-S10", PALLET)
    core.commands.queue(Submit("P-S10", "D-S10", rev))
    core.commands.apply(10.0)
    assert core.commands.status == "accepted"
    frozen = asm.freeze_submit(core, Submit("P-S10", "D-S10", rev))
    inputs = {"scenario": core.options.scenario, "wait_pose": "WAIT", "pickup_bounds": None, "pickup_zone": None}
    core.slam["stop_now"] = True  # the loop's still span (4c sets it from new samples)
    hooks.read.clear()
    assert asm.known_request(core, 11.0, np.array([0.0, 0.0, 0.0]), inputs, frozen) is None
    kw = core.jobs.submitted[-1][1]["kwargs"]
    ob = kw["pickup_obstacle"]  # the registered pose, not the scenario's reference pallet (Codex 4b P2)
    assert (ob.x_m, ob.y_m, ob.yaw_rad) == PALLET and ob.length_m == GEOM.pallet_depth_m
    assert core.obstacle.get("pickup_estimate") is None  # the preparation wrote nothing
    core.jobs.answers[core.pending.job] = (True, {"result": MissionPlan(), "trace": []})
    core.job_hold(12.0, np.array([0.0, 0.0, 0.0]), 0.0, False)
    assert core.commands.status == "running" and core.ms.phase == "approach" and started
    assert core.state["planning_target_source"] == "operator_input"
    table = asm.ADOPTION_HOOKS | asm.ADOPTION_OPTIONAL_ABSENT
    assert hooks.read <= table, sorted(hooks.read - table)
    from forklift_core.mission.hook_contract import HOOKS

    assert all(HOOKS[name][1] for name in asm.ADOPTION_OPTIONAL_ABSENT)  # absent only where optional
    assert all(callable(getattr(asm.RealHooks, name)) for name in asm.ADOPTION_HOOKS)
    assert core.slam["tracker"].mode == "holding"  # the approach's hold, the pose a map estimate


def test_a_truth_hook_on_the_real_core_is_refused():
    from forklift_core.safety.truth_access import TruthAccessViolation

    core, hooks, _ = build()
    with pytest.raises(TruthAccessViolation):
        core.hooks.truth_velocity  # noqa: B018
    with pytest.raises(TruthAccessViolation):
        core.hooks.pallet_z  # noqa: B018
    assert core.hooks.truth_rear() is None  # an evaluation: unjudgeable


def test_a_failed_require_reaches_the_loop():
    core, hooks, _ = build()
    with pytest.raises(asm.RealRequireFailed):
        core.hooks.require(False, "x")


def test_the_dry_record_names_the_options_the_hooks_and_the_caps():
    core, hooks, _ = build()
    rec = asm.assembly_record(core, cfg(), {"planner": CAP})
    assert rec["options"]["dt"] == 1 / 50 and rec["options"]["near_tracking"] is None
    assert rec["adoption_hooks"] == sorted(asm.ADOPTION_HOOKS) and rec["s10a"]["map_id"] == "hall-1"


def test_the_submit_is_frozen_with_its_registered_pose_and_revision():
    core, hooks, _ = build()
    with pytest.raises(ValueError):
        asm.freeze_submit(core, Submit("P-S10", "D-S10", core.targets.revision))  # not registered yet
    rev = asm.register_pallet(core, "P-S10", PALLET)
    frozen = asm.freeze_submit(core, Submit("P-S10", "D-S10", rev))
    asm.register_pallet(core, "P-S10", (3.3, 0.2, 0.0))  # changed afterwards
    assert frozen.pose == PALLET and frozen.targets_revision == rev


def test_the_operator_start_check_is_one_lease_one_submit_and_voided_by_motion():
    op = asm.OperatorStart(7)
    assert not op.confirm(8, "L1")  # another session's
    assert op.confirm(7, "L1") and op.ok("L1") and not op.ok("L2")
    op.void("motion")
    assert not op.ok("L1") and op.voided == "motion"
    op.confirm(7, "L1")
    assert op.take("L1") and not op.ok("L1") and not op.take("L1")  # one submit only


def test_the_first_frame_stand_and_the_s10_end_read_only_their_hooks():
    answers = []
    core, hooks, _ = build(hooks_kw={"operator": lambda request: answers.pop(0) if answers else None})
    from forklift_core.mission.commands import Submit as _S

    rev = asm.register_pallet(core, "P-S10", PALLET)
    core.commands.queue(_S("P-S10", "D-S10", rev))
    core.commands.apply(10.0)
    frozen = asm.freeze_submit(core, Submit("P-S10", "D-S10", rev))
    core.slam["stop_now"] = True
    inputs = {"scenario": core.options.scenario, "wait_pose": "WAIT", "pickup_bounds": None, "pickup_zone": None}
    asm.known_request(core, 11.0, np.array([0.0, 0.0, 0.0]), inputs, frozen)
    core.jobs.answers[core.pending.job] = (True, {"result": MissionPlan(), "trace": []})
    core.job_hold(12.0, np.array([0.0, 0.0, 0.0]), 0.0, False)
    core.trackers["approach"] = SimpleNamespace(remaining_to_goal_m=lambda: 0.9)
    hooks.read.clear()
    assert core._known_first_frame(13.0, (0.0, 0.0, 0.0), None) == (0.0, "pocket_first_frame:none")  # noqa: SLF001
    with pytest.raises(asm.RealRequireFailed, match="pocket_first_frame"):  # no frame until PR 7: 5 s, then the end
        core._known_first_frame(18.5, (0.0, 0.0, 0.0), None)  # noqa: SLF001
    core.ms.phase, core.ms.phase_started = "lift", 20.0
    core.commands.status = "running"
    core.cargo.relation, core.cargo.load = "constrained", "acquiring"
    hooks._lift_m = lambda: SETTINGS["lift_target_m"]  # noqa: SLF001
    core.lift_and_timers(25.0, 0.02)  # the request
    answers.append((1, "confirm"))
    core.lift_and_timers(25.1, 0.02)
    assert core.commands.end_reason == "s10_lifted" and core.take_s10_stop() == 25.1
    assert hooks.read <= asm.STAND_AND_END_HOOKS | asm.ADOPTION_HOOKS, sorted(
        hooks.read - asm.STAND_AND_END_HOOKS - asm.ADOPTION_HOOKS)
    assert {"known_pocket_frame", "operator_check", "lift_position", "require"} <= hooks.read


def test_a_frozen_submit_of_another_destination_ends_the_preparation():
    core, hooks, _ = build()
    rev = asm.register_pallet(core, "P-S10", PALLET)
    core.commands.queue(Submit("P-S10", "D-S10", rev))
    core.commands.apply(10.0)
    wrong = asm.FrozenSubmit("P-S10", "D-OTHER", PALLET, rev)
    inputs = {"scenario": core.options.scenario, "wait_pose": "WAIT", "pickup_bounds": None, "pickup_zone": None}
    assert asm.known_request(core, 11.0, np.zeros(3), inputs, wrong) == "submit_mismatch"
    assert core.commands.status == "failed" and core.commands.end_reason == "prepare:known:submit_mismatch"
    assert not core.jobs.submitted


def test_the_docking_grid_clears_only_the_registered_pallets_band():
    from forklift_core.planning.grid_collision import OccupancyGrid

    occ = np.zeros((70, 50), bool)  # x from -1.0, y from -2.5, 0.1 m
    occ[38:42, 23:27] = True  # cells at the registered pallet (x 2.8-3.2, y -0.2-0.2)
    occ[10, 10] = True  # a far wall cell
    grid = OccupancyGrid(-1.0, -2.5, 0.1, occ)
    core, hooks, _ = build(hooks_kw={"occupancy": lambda: grid})
    out = core.hooks.grid_kwargs("mission", SimpleNamespace(x_m=3.0, y_m=0.0, yaw_rad=0.0))
    assert out["occupancy"] is grid and out["occupancy"].occupied[39, 24]  # the approach still sees it
    dock = out["docking_occupancy"].occupied
    assert not dock[38:42, 23:27].any() and dock[10, 10]
