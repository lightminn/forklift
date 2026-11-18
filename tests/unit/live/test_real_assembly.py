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


def build(**over):
    started, records = [], []
    hooks = Recording(lift_m=lambda: 0.003, odom_base=lambda: (0.0, 0.0, 0.0), ready=lambda t, inputs: None,
                      started=lambda *a: started.append(a), record=lambda *a: records.append(a))
    layer = SimpleNamespace(known_enabled=False, refresh=lambda *a, **k: None,
                            permission=SimpleNamespace(path_check=None))
    kw = dict(cfg=cfg(), loaded_map_id="hall-1", geometry=SyntheticMissionGeometry(), pallet_geometry=None,
              kinematics=KIN, settings=SETTINGS, planner_config="PC", travel_config="TC",
              speeds={name: 0.1 for name in STAGES}, mission_stages=STAGES,
              curvatures={"planner": CAP, "tracker": CAP, "preview": CAP}, layer=layer, applied=(0.0, 0.0, 0.0),
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
    ({"curvatures": {"planner": CAP, "tracker": CAP / 2}}, "curvature_caps_differ"),
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
    inputs = {"scenario": core.options.scenario, "wait_pose": "WAIT", "pickup_bounds": None, "pickup_zone": None}
    core.slam["stop_now"] = True  # the loop's still span (4c sets it from new samples)
    hooks.read.clear()
    assert core.known_pallet_request(11.0, np.array([0.0, 0.0, 0.0]), inputs, PALLET, targets_revision=rev) is None
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
