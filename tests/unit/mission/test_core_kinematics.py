"""Sim-to-real 4a': the core's command assembly through the injected kinematics
(docs/design/2026-11-11-p04a-kinematics-and-command.md D1-D3)."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control import ackermann_command
from forklift_core.control.path_tracking import AckermannGeometry
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.plant.kinematics import AckermannKinematics, RealKinematics
from forklift_core.plant.real_vehicle import real_vehicle_from_mapping

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "plant"))
from test_real_vehicle import CHASSIS, mapping  # noqa: E402

GEOMETRY = AckermannGeometry(wheelbase_m=1.0, track_m=0.6, wheel_radius_m=0.1, max_steering_rad=0.8,
                             max_wheel_rate_rad_s=50.0)
REAL = RealKinematics(real_vehicle_from_mapping(mapping(), CHASSIS))


def make(*, kinematics=None, steering=(0.0, 0.0), replan_allowed=False, phase="transport", tracker_serial=1):
    calls = []
    hooks = SimpleNamespace(steering_position=lambda: np.array(steering))
    tracker = SimpleNamespace(_book={"serial": tracker_serial})
    options = SimpleNamespace(drive_geometry=GEOMETRY, command_accel_mps2=0.8, obstacle_act=False)
    if kinematics is not None:
        options.kinematics = kinematics
    core = MissionCore(MissionState(phase=phase, phase_started=0.0), state={"transitions": []}, obstacle=None,
                       slam=None, pocket={"check": None}, paths={}, trackers={phase: tracker}, geometry=None,
                       settings={"steering_command_rate_rad_s": 1.2}, pallet_geometry=None, hooks=hooks, near=None,
                       options=options)
    core.scene_replan_allowed = lambda: replan_allowed
    core.scene_replan_request = lambda t, trigger, **kw: calls.append(("replan", t, trigger))
    queued = []
    core.commands = SimpleNamespace(adopted_id="m1", mission_id="m1", status="running", queue=queued.append)
    core.queued = queued
    return core, calls


def assemble(core, steering, *, speed=0.4, curvature=0.2, kappa=0.1, t=10.0, dt=0.01, holds=(False,) * 5):
    return core.assemble_command(t, dt, speed, curvature, kappa, steering, *holds)


def test_the_default_kinematics_is_the_simulations_and_nothing_extra_is_recorded():
    core, _ = make()
    assert isinstance(core.kinematics, AckermannKinematics)
    out = assemble(core, np.zeros(2))
    expected = ackermann_command(0.4, 0.2, GEOMETRY)
    assert out.drive.wheel_rates_rad_s == pytest.approx(ackermann_command(
        out.drive.speed_mps, 0.2, GEOMETRY).wheel_rates_rad_s)
    assert np.allclose(out.steering_command, np.clip(np.asarray(expected.steering_rad), -0.012, 0.012))
    assert core.refusal_events == [] and core._refusal is None  # noqa: SLF001


def test_the_real_kinematics_drives_one_steering_state():
    core, _ = make(kinematics=REAL, steering=(0.0,))
    steer = np.zeros(1)
    out = assemble(core, steer, curvature=0.2)
    assert steer.shape == (1,) and steer[0] == pytest.approx(REAL.steering_rate_rad_s * 0.01)
    assert len(out.drive.wheel_rates_rad_s) == 2
    k_now = math.tan(steer[0]) / 0.66  # the wheels follow the slewed q, not the target curvature
    assert out.drive.curvature_inv_m == pytest.approx(k_now)


def test_a_wrong_steering_shape_ends_the_tick():
    core, _ = make(kinematics=REAL, steering=(0.0, 0.0))
    with pytest.raises(ValueError):
        assemble(core, np.zeros(1))


def test_a_refusal_latches_a_stop_and_pauses_the_mission_once_per_plan():
    """Design D3 as built in 4a': stop, steering held, the mission paused for the operator; no automatic replan."""
    core, calls = make(kinematics=REAL, steering=(0.1,), replan_allowed=True)
    steer = np.array([0.1])
    out = assemble(core, steer, curvature=0.9)  # beyond the map
    assert out.drive.speed_mps == 0.0 and out.drive.wheel_rates_rad_s == (0.0, 0.0)
    assert steer[0] == pytest.approx(0.1)  # steering held
    assert calls == []  # no scene replan
    assert [c.request_id for c in core.queued] == ["kinematic_refusal"]
    assert core.refusal_events[-1]["reason"] == "curvature_infeasible"
    assemble(core, steer, curvature=0.9, t=10.01)
    assert assemble(core, steer, curvature=0.2, t=10.02).drive.speed_mps == 0.0  # latched on this plan
    assert len(core.queued) == 1 and len(core.refusal_events) == 1


def test_a_new_plan_or_the_operators_resume_clears_the_latch():
    core, _ = make(kinematics=REAL, steering=(0.1,))
    steer = np.array([0.1])
    assemble(core, steer, curvature=0.9)
    core.trackers["transport"] = SimpleNamespace(_book={"serial": 2})  # a new plan
    assert assemble(core, steer, curvature=0.1, t=11.0).drive.speed_mps > 0.0
    assemble(core, steer, curvature=0.9, t=12.0)
    assert core._refusal is not None  # noqa: SLF001
    core.commands.status = "resuming"  # the operator resumed
    assert assemble(core, steer, curvature=0.1, t=13.0).drive.speed_mps > 0.0


def test_an_unknown_measured_steering_is_a_refusal():
    """The measured q cannot be read as a curvature: stop and pause, even for a feasible request (Codex P1-2)."""
    core, _ = make(kinematics=REAL, steering=(float("nan"),))
    out = assemble(core, np.array([0.1]), curvature=0.1)
    assert out.drive.speed_mps == 0.0 and core.refusal_events[-1]["reason"] == "steering_unknown"
    assert [c.request_id for c in core.queued] == ["kinematic_refusal"]


def test_the_presteer_tolerance_is_the_given_kinematics_own():
    from forklift_core.mission.scene_replan import SceneReplanConfig

    core, _ = make(kinematics=REAL, steering=(0.1,))
    assert core.scene_replan.config.presteer_tolerance_rad == REAL.presteer_threshold_rad
    core_sim, _ = make()
    assert core_sim.scene_replan.config.presteer_tolerance_rad == SceneReplanConfig().presteer_tolerance_rad


def test_an_unknown_steering_brakes_without_a_curvature():
    core, _ = make(kinematics=REAL, steering=(0.1,), replan_allowed=False)
    steer = np.array([0.5])  # outside the map: its curvature is unknown
    out = assemble(core, steer, curvature=0.9)
    assert out.drive.speed_mps == 0.0 and out.drive.wheel_rates_rad_s == (0.0, 0.0)


def test_no_adopted_mission_clears_the_latch():
    core, _ = make(kinematics=REAL, steering=(0.1,))
    assemble(core, np.array([0.1]), curvature=0.9)
    core.commands = SimpleNamespace(adopted_id=None)
    assemble(core, np.array([0.1]), curvature=0.1, t=11.0)
    assert core._refusal is None  # noqa: SLF001


def test_the_creep_threshold_is_the_kinematics_own():
    sc = {"steering_rate_rad_s": 1.0, "creep_threshold_rad": 0.2, "presteer_threshold_rad": 0.02,
          "straight_threshold_rad": 0.01, "provenance": {"source": "t", "measured": False, "until": "S4"}}
    loose = RealKinematics(real_vehicle_from_mapping(mapping(steering_control=sc), CHASSIS))
    target_q = math.atan(0.66 * 0.2)
    for kin, creeps in ((REAL, True), (loose, False)):
        core, _ = make(kinematics=kin, steering=(target_q - 0.1,))  # 0.1 rad behind the target
        out = assemble(core, np.array([target_q]), speed=0.4, curvature=0.2)
        assert (abs(out.drive.speed_mps) < 0.2) is creeps


def _real_commands_core(near_tracker=None):
    """The core with the real MissionCommands, a running mission."""
    from forklift_core.mission.commands import MissionCommands

    core, _ = make(kinematics=REAL, steering=(0.1,))
    cmds = MissionCommands()
    cmds.status, cmds.mission_id, cmds.adopted_id = "running", "m1", "m1"
    core.commands = cmds
    core.near = {"tracker": near_tracker, "commands": SimpleNamespace(add=lambda *a: None)}
    return core, cmds


def test_with_the_real_commands_a_refusal_pauses_and_a_same_tick_resume_releases():
    """Pause → operator Resume → the resume checks pass in paused_tick (running again) → the assembly the same tick
    drives (Codex 4a' re-review P2-2)."""
    from forklift_core.mission.commands import Resume

    core, cmds = _real_commands_core()
    steer = np.array([0.1])
    assemble(core, steer, curvature=0.9, t=10.0)
    cmds.apply(10.01)
    assert cmds.status == "paused"
    cmds.queue(Resume())
    cmds.apply(11.0)
    assert cmds.status == "resuming"
    cmds.resumed(11.0)  # what paused_tick's finish() does when the checks pass
    cmds.status = "running"
    assert assemble(core, steer, curvature=0.1, t=11.0).drive.speed_mps > 0.0


def test_a_refusal_during_near_tracking_ends_the_mission():
    """A pause is refused while the near tracker runs: the mission ends instead (Codex 4a' re-review P2-1)."""
    core, cmds = _real_commands_core(near_tracker=object())
    assemble(core, np.array([0.1]), curvature=0.9, t=10.0)
    assert cmds.status == "failed" and cmds.log[-1]["reason"] == "kinematic_refusal"
