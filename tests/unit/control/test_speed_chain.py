"""M-b4: the wheel-speed chain shared by the mission core and the arbiter (plan B8)."""

import pytest

from forklift_core.control.speed_chain import creep, slew_cap_stop

DT = 1 / 120


def chain(previous, target, **k):
    return slew_cap_stop(previous, target, decel_mps2=1.5, accel_mps2=0.8, dt=DT, **k)


def test_braking_is_the_stopping_deceleration_and_pulling_away_the_command_acceleration():
    assert chain(0.5, 0.0) == pytest.approx(0.5 - 1.5 * DT)
    assert chain(0.0, 0.5) == pytest.approx(0.8 * DT)


def test_a_sign_change_goes_through_zero():
    assert chain(0.1, -0.3) == pytest.approx(0.1 - 1.5 * DT)


def test_the_cap_cuts_after_the_slew_as_a_step_and_the_stop_zeroes():
    """Design 4.4: a cap of 0.1 m/s below a 0.5 m/s command is a step, not the slew's 0.4875."""
    assert chain(0.5, 0.5) == 0.5 and chain(0.5, 0.0) == pytest.approx(0.4875)
    assert chain(0.5, 0.5, cap_mps=0.1) == 0.1 and chain(-0.5, -0.5, cap_mps=0.1) == -0.1
    assert chain(0.5, 0.5, stop=True) == 0.0


def test_the_creep_comes_before_the_slew():
    """Codex M-b plan round 2: creeping after the slew would step 0.5 to 0.125; before it, the slew brakes."""
    assert chain(0.5, creep(0.5, 0.06)) == pytest.approx(0.4875)
    assert creep(0.5, 0.05) == 0.5
