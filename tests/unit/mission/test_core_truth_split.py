"""Sim-to-real 4d-1: the safety speed and the truth access rules (docs/design/2026-11-11-p05-truth-audit.md 3a, 4)."""

from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.safety.truth_access import SafetySpeed, TruthAccessViolation


def test_the_exact_speed_is_todays_rule():
    for v in (0.3, -0.2, 0.0, -0.0, 1e-9):
        s = SafetySpeed.exact(v)
        assert s.signed_mps == v and s.bound_mps == abs(v) and s.valid
        assert s.directions == ((1,) if v > 0 else (-1,))  # a standing truck reads -1, as before
        assert s.standing(0.01) is (abs(v) < 0.01)


def test_a_safety_speed_must_cover_its_value():
    with pytest.raises(ValueError):
        SafetySpeed(0.3, 0.1, (1,))
    with pytest.raises(ValueError):
        SafetySpeed(0.0, 0.0, (2,))
    assert not SafetySpeed(0.0, 0.0, (1, -1), valid=False).standing(0.01)


def test_the_violation_is_not_absorbed_by_mission_failure_handling():
    assert not issubclass(TruthAccessViolation, Exception)
    with pytest.raises(TruthAccessViolation):
        try:
            raise TruthAccessViolation("truth_velocity")
        except Exception:  # the runner's mission-failure path
            pytest.fail("absorbed")


def core(*, real=False, hooks=None):
    hooks = hooks or SimpleNamespace(truth_speed=lambda: 0.25, truth_velocity=lambda: np.array([0.25, 0.0, 0.0]))
    opts = SimpleNamespace(drive_geometry=None, command_accel_mps2=0.8, obstacle_act=False, real_vehicle=real)
    return MissionCore(MissionState(phase="transport", phase_started=0.0), state={}, obstacle=None, slam=None,
                       pocket={"check": None}, paths={}, trackers={}, geometry=None,
                       settings={"steering_command_rate_rad_s": 1.0}, pallet_geometry=None, hooks=hooks, near=None,
                       options=opts)


def test_the_simulation_falls_back_to_the_true_speed_and_the_real_truck_never_does():
    assert core().safety_speed() == SafetySpeed.exact(0.25)
    with pytest.raises(TruthAccessViolation):
        core(real=True).safety_speed()
    given = SafetySpeed(0.1, 0.15, (1, -1))
    assert core(real=True, hooks=SimpleNamespace(safety_speed=lambda: given)).safety_speed() is given


def test_a_slam_off_truth_read_raises_on_the_real_truck():
    assert np.array_equal(core()._truth("truth_velocity"), [0.25, 0.0, 0.0])  # noqa: SLF001
    with pytest.raises(TruthAccessViolation):
        core(real=True)._truth("truth_velocity")  # noqa: SLF001


def test_the_simulation_checks_one_direction_as_before():
    c = core()
    assert c._check_directions(SafetySpeed.exact(0.3), None) == [1]  # noqa: SLF001
    assert c._check_directions(SafetySpeed.exact(0.0), None) == [-1]  # noqa: SLF001
    assert c._check_directions(SafetySpeed.exact(-0.3), 1) == [1]  # noqa: SLF001  the request wins, as before


def test_the_real_truck_checks_every_direction_it_may_move():
    c = core(real=True)
    unsure_moving = SafetySpeed(0.0, 0.05, (1, -1))
    assert c._check_directions(unsure_moving, 1) == [1, -1]  # noqa: SLF001  the request does not erase the doubt
    assert c._check_directions(unsure_moving, None) == [1, -1]  # noqa: SLF001
    assert c._check_directions(SafetySpeed(-0.2, 0.2, (-1,)), 1) == [1, -1]  # noqa: SLF001  rolling back, asked fwd
    standing = SafetySpeed(0.0, 0.005, (1, -1))
    assert c._check_directions(standing, -1) == [-1]  # noqa: SLF001  a standing truck moves only as asked
    assert c._check_directions(standing, None) == [1]  # noqa: SLF001
    assert c._check_directions(SafetySpeed(0.0, 0.0, (1, -1), valid=False), 1) == [1, -1]  # noqa: SLF001


def test_the_limit_over_several_directions_keeps_the_smallest():
    calls = []

    def limit(d):
        calls.append(d)
        return (0.4, "ok") if d > 0 else (0.1, "occupied")

    assert MissionCore._limit_any_direction(limit, [1, -1]) == (0.1, "occupied") and calls == [1, -1]  # noqa: SLF001
    calls.clear()
    assert MissionCore._limit_any_direction(limit, [1]) == (0.4, "ok") and calls == [1]  # noqa: SLF001


class _Tracker:
    def remaining_to_goal_m(self):
        return 5.0

    def progress_m(self):
        return 1.0

    def leg_ahead(self):
        return np.zeros((1, 3)), 1


class _Layer:
    known_enabled = False
    known_config = {}

    def __init__(self, blocked_direction=None):
        self.blocked_direction = blocked_direction
        self.calls = []
        self.permission = SimpleNamespace(path_check=None, config=SimpleNamespace())

    def limit(self, t, *, current_pose, curvature_inv_m, direction, loaded, cap_mps):
        self.calls.append(direction)
        return (0.0, "occupied") if direction == self.blocked_direction else (0.5, "ok")


def permission_core(speed, *, phase="transport", blocked=None, pocket_check=None):
    layer = _Layer(blocked)
    hooks = SimpleNamespace(safety_speed=lambda: speed, truth_rear=lambda: np.zeros(3), require=lambda ok, why: None,
                            save_npz=lambda *a, **k: None, lift_position=lambda: 0.0)
    opts = SimpleNamespace(drive_geometry=None, command_accel_mps2=0.8, obstacle_act=True, real_vehicle=True,
                           pocket_check=pocket_check is not None, grid_planning=False, restart_confirm_scans=0)
    obstacle = {"layer": layer, "applied": (0.0, 0.0, 0.0), "ticks": {}, "min_allowed": {}, "slowed": {},
                "reasons": {}, "version": 0}
    slam = {"tracker": SimpleNamespace(applied=None, mode="tracking"), "stop_now": False, "docking": {}}
    c = MissionCore(MissionState(phase=phase, phase_started=0.0), state={}, obstacle=obstacle, slam=slam,
                    pocket={"check": pocket_check, "frames_read": 0}, paths={}, trackers={phase: _Tracker()},
                    geometry=SimpleNamespace(alignment_straight_m=0.5, delivery_straight_m=0.5),
                    settings={"steering_command_rate_rad_s": 1.0}, pallet_geometry=None, hooks=hooks, near=None,
                    options=opts)
    c.commands.work_elapsed = lambda since, t: t - since
    return c, layer


def test_an_invalid_safety_speed_holds_the_wheels():
    c, _ = permission_core(SafetySpeed(0.0, 0.0, (1,), valid=False))
    out = c.permission(1.0, (0.0, 0.0, 0.0), 0.0, 0.4, False, None)
    assert out.allowed == 0.0 and out.why == "safety_speed_invalid"
    assert out.requested_speed == 0.0 and out.obstacle_hold
    assert c.obstacle["permission_cap"] == (1.0, 0.0)
    w = c.permission_watch(1.0, (0.0, 0.0, 0.0), 0.0, False)
    assert w.allowed == 0.0 and w.why == "safety_speed_invalid" and w.obstacle_hold


def test_a_forward_request_does_not_hide_an_unsure_rearward_motion():
    c, layer = permission_core(SafetySpeed(0.0, 0.05, (1, -1)), blocked=-1)
    out = c.permission(1.0, (0.0, 0.0, 0.0), 0.0, 0.4, False, None)
    assert layer.calls == [1, -1]
    assert out.allowed == 0.0 and out.why == "occupied" and out.requested_speed == 0.0


def test_the_pocket_check_does_not_answer_an_unsure_direction():
    asked = []
    check = SimpleNamespace(limit=lambda *a: asked.append(a[3]) or (0.5, "ok"))
    c, _ = permission_core(SafetySpeed(0.0, 0.05, (1, -1)), phase="insert", pocket_check=check)
    out = c.permission(1.0, (0.0, 0.0, 0.0), 0.0, 0.0, False, None)
    assert out.allowed == 0.0 and out.why == "direction_unsure" and asked == []
