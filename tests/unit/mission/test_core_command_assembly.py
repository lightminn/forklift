"""M-a1 slice 5: the release gate at the tick's start and the command assembly, moved from the runner
(behaviour, state, port requests and their order, effects up to a refusal -- the plan's M-a1 criterion)."""

from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control import ackermann_command
from forklift_core.control.path_tracking import AckermannGeometry
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState

GEOMETRY = AckermannGeometry(wheelbase_m=1.0, track_m=0.6, wheel_radius_m=0.1, max_steering_rad=0.8,
                             max_wheel_rate_rad_s=50.0)


def make(*, slam=None, obstacle=None, act=True, near=None, steering=(0.0, 0.0)):
    calls = []
    pose = {"rear": np.array([2.0, 3.0, 0.0])}  # the estimate; the release moves it (Codex slice-5 P3)

    def release(t):
        calls.append(("slam_release", (t,)))
        pose["rear"] = np.array([3.0, 4.0, 0.1])

    def require(ok, reason):
        calls.append(("require", (bool(ok), reason)))
        if not ok:
            raise RuntimeError(reason)

    hooks = SimpleNamespace(
        require=require, slam_release=release,
        slam_rear=lambda t: pose["rear"].copy(), set_rear=lambda v: calls.append(("set_rear", (tuple(v),))),
        steering_position=lambda: np.array(steering),
    )
    state = {"transitions": []}
    core = MissionCore(MissionState(phase="transport", phase_started=0.0), state=state, obstacle=obstacle, slam=slam,
                       pocket={"check": None}, paths={}, trackers={}, geometry=None,
                       settings={"steering_command_rate_rad_s": 1.2}, pallet_geometry=None, hooks=hooks, near=near,
                       options=SimpleNamespace(drive_geometry=GEOMETRY, command_accel_mps2=0.8, obstacle_act=act))
    # Helpers moved into MissionCore (slices 6b/6c): this test's stand-ins go on the instance.
    for name_ in ['dock_at_delivery_straight', 'arm_docking', 'release_and_carry', 'transport_replan', 'start_docking_retry', 'release_and_stand', 'release_after_backoff', 'slam_release', 'escape_leg', 'find_escape']:
        if hasattr(hooks, name_):
            setattr(core, name_, getattr(hooks, name_))
    return core, state, calls


def test_a_pending_release_waits_for_the_stop_then_releases_and_rereads_the_pose():
    slam = {"pending_release": "transport", "release_wait_from": 40.0, "stop_now": False, "control": [[0, 0, 0, 0]],
            "tracker": SimpleNamespace(may_drive=lambda: True)}
    core, _, calls = make(slam=slam)
    out = core.release_gate(45.0, np.zeros(3))
    assert (out.releasing, out.warming) == (True, False) and [c[0] for c in calls] == ["require"]
    slam["stop_now"] = True
    out = core.release_gate(46.0, np.zeros(3))
    assert out.releasing is False and calls[-2:] == [("slam_release", (46.0,)), ("set_rear", ((3.0, 4.0, 0.1),))]
    np.testing.assert_array_equal(out.rear, [3.0, 4.0, 0.1])
    assert slam["control"][-1] == [0, 3.0, 4.0, 0.1]
    slam["stop_now"] = False
    with pytest.raises(RuntimeError, match="slam_release_no_stop after 10.0 s"):
        core.release_gate(50.0, np.zeros(3))
    slam["pending_release"], slam["tracker"] = None, SimpleNamespace(may_drive=lambda: False)
    out = core.release_gate(51.0, np.zeros(3))
    assert (out.releasing, out.warming) == (False, True)
    core, _, _ = make()
    out = core.release_gate(1.0, np.zeros(3))
    assert (out.releasing, out.warming) == (False, False)


def assemble(core, *, speed=0.4, curvature=0.2, kappa=0.1, steering=None, holds=(False,) * 5, t=10.0):
    steering = np.zeros(2) if steering is None else steering
    estop, obstacle_hold, dwell, cusp, safety = holds
    return core.assemble_command(t, 0.01, speed, curvature, kappa, steering, estop, obstacle_hold, dwell, cusp,
                                 safety), steering


def test_the_command_follows_the_tracker_and_the_steering_rate_moves_the_runners_array_in_place():
    expected = ackermann_command(0.4, 0.2, GEOMETRY)
    core, state, _ = make(act=False, steering=tuple(expected.steering_rad))  # measured at the target: no creep
    out, steering = assemble(core)
    assert out.drive.speed_mps == pytest.approx(expected.speed_mps) and out.requested_speed == 0.4
    assert out.steering_command is steering  # the same array, stepped in place
    np.testing.assert_allclose(steering, np.clip(np.asarray(expected.steering_rad), -0.012, 0.012))
    assert state["applied_command"] == (float(out.drive.speed_mps), [float(v) for v in steering])
    assert "command_speed" not in state  # no obstacle layer acting: no slew


def test_a_hold_zeroes_the_wheels_and_keeps_the_steering_on_the_held_curvature():
    core, _, _ = make(act=False)
    held = np.array([0.1, 0.12])
    out, steering = assemble(core, steering=held.copy(), holds=(False, False, True, False, False))
    assert out.requested_speed == 0.0 and out.drive.speed_mps == 0.0
    np.testing.assert_allclose(steering, held)  # target = the held steering: no step
    core, _, _ = make(act=False)
    out, _ = assemble(core, holds=(True, False, False, False, False))
    assert out.requested_speed == 0.0


def test_the_wheels_creep_while_the_steering_catches_up():
    core, _, _ = make(act=False, steering=(0.3, 0.3))
    out, _ = assemble(core, curvature=0.0)
    assert out.drive.speed_mps == pytest.approx(0.1) and out.actual_steering.tolist() == [0.3, 0.3]


def slewing(previous, **kw):
    layer = SimpleNamespace(permission=SimpleNamespace(config=SimpleNamespace(stopping=SimpleNamespace(decel_mps2=2.0))))
    obstacle = {"layer": layer, "permission_cap": kw.pop("cap", None)}
    core, state, _ = make(obstacle=obstacle, near=kw.pop("near", None))
    state["command_speed"] = previous
    out, _ = assemble(core, curvature=0.0, **kw)
    return out, state


def test_the_wheel_speed_is_slewed_capped_by_the_permission_and_stepped_to_zero_by_a_stop():
    out, state = slewing(0.2, speed=0.4)
    assert state["command_speed"] == pytest.approx(0.2 + 0.8 * 0.01)  # pulling away at COMMAND_ACCEL_MPS2
    out, state = slewing(0.4, speed=0.0)
    assert state["command_speed"] == pytest.approx(0.4 - 2.0 * 0.01)  # braking at the stopping model's decel
    out, state = slewing(0.3, speed=-0.3)
    assert state["command_speed"] == pytest.approx(0.3 - 2.0 * 0.01)  # a reversal first brakes to zero
    out, state = slewing(0.4, speed=0.4, cap=(10.0, 0.1))
    assert state["command_speed"] == pytest.approx(0.1)  # the permission's cap of this tick, as a step
    out, state = slewing(0.4, speed=0.4, cap=(9.99, 0.1))
    assert state["command_speed"] == pytest.approx(0.4)  # a cap from another tick does not apply
    out, state = slewing(0.4, speed=0.4, holds=(False, False, False, False, True))
    assert state["command_speed"] == 0.0 and out.drive.speed_mps == 0.0  # the safety stop's step


def test_the_near_tracker_is_told_the_commanded_speed_over_the_next_step():
    added = []
    near = {"tracker": object(), "commands": SimpleNamespace(add=lambda t, v: added.append((t, v)))}
    out, _ = slewing(0.4, speed=0.4, near=near)
    assert added == [(pytest.approx(10.01), out.drive.speed_mps)]


def test_a_dwell_brakes_its_wheels_on_the_held_steerings_curvature():
    out, state = slewing(0.4, speed=0.4, holds=(False, False, True, False, False), kappa=0.3)
    still_rolling = state["command_speed"]
    assert still_rolling == pytest.approx(0.4 - 2.0 * 0.01)  # braking on the slew, not a step
    expected = ackermann_command(still_rolling, 0.3, GEOMETRY)  # kappa, not the tracker's curvature (0.0)
    np.testing.assert_allclose(out.drive.wheel_rates_rad_s, expected.wheel_rates_rad_s)
