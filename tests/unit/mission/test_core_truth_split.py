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


def test_an_unsure_direction_checks_both_and_keeps_the_smaller():
    calls = []

    def limit(d):
        calls.append(d)
        return (0.4, "ok") if d > 0 else (0.1, "occupied")

    assert MissionCore._limit_any_direction(limit, SafetySpeed(0.0, 0.05, (1, -1)), None) == (0.1, "occupied")
    assert calls == [1, -1]
    calls.clear()
    assert MissionCore._limit_any_direction(limit, SafetySpeed.exact(0.3), None) == (0.4, "ok") and calls == [1]
    calls.clear()
    assert MissionCore._limit_any_direction(limit, SafetySpeed(0.0, 0.05, (1, -1)), -1) == (0.1, "occupied")
    assert calls == [-1]  # a caller's direction (the request, a cusp hand-over) is kept
