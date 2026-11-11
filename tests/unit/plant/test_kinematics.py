"""P0-4 4a' vehicle kinematics (docs/design/2026-11-11-p04a-kinematics-and-command.md D1)."""

import math

import numpy as np
import pytest
from test_real_vehicle import CHASSIS, PROV, mapping  # noqa: E402

from forklift_core.control.path_tracking import AckermannGeometry, ackermann_command
from forklift_core.control.speed_chain import CREEP_STEERING_ERROR_RAD
from forklift_core.mission.scene_replan import SceneReplanConfig
from forklift_core.plant.kinematics import AckermannKinematics, RealKinematics
from forklift_core.plant.real_vehicle import DriveRefusal, real_vehicle_from_mapping

GEOM = AckermannGeometry(0.66, 0.53, 0.125, 0.2617993878, 8.0)
RATE = 1.0  # settings["steering_command_rate_rad_s"]


@pytest.fixture
def ack():
    return AckermannKinematics(GEOM, RATE)


@pytest.fixture
def real():
    return RealKinematics(real_vehicle_from_mapping(mapping(), CHASSIS))


def _held_today(steer, g=GEOM):  # sim/isaac/run_transport.py held_kappa, as written
    h, wb = g.track_m / 2, g.wheelbase_m
    return float(np.mean([math.tan(steer[0]) / (wb + math.tan(steer[0]) * h),
                          math.tan(steer[1]) / (wb - math.tan(steer[1]) * h)]))


def _bound_today(steer, g=GEOM):  # core.py near protection, per wheel
    h, wb = g.track_m / 2, g.wheelbase_m
    return max(abs(math.tan(steer[0]) / (wb + math.tan(steer[0]) * h)),
               abs(math.tan(steer[1]) / (wb - math.tan(steer[1]) * h)))


def test_ackermann_default_is_todays_arithmetic_bit_for_bit(ack):
    rng = np.random.default_rng(3)
    for _ in range(300):
        v, k = float(rng.uniform(-2, 2)), float(rng.uniform(-1, 1))
        got, want = ack.command(v, k), ackermann_command(v, k, GEOM)
        assert got == want  # same dataclass, same floats: no refusal on the Isaac path
        assert ack.steering_targets(k) == ackermann_command(0.0, k, GEOM).steering_rad
        steer = rng.uniform(-0.25, 0.25, 2)
        assert ack.held_curvature(steer) == _held_today(steer)
        assert ack.curvature_bound(steer) == _bound_today(steer)
        target, cmd = np.asarray(want.steering_rad), rng.uniform(-0.25, 0.25, 2)
        mine, theirs = cmd.copy(), cmd.copy()
        ack.slew_steering(mine, target, 1 / 120)
        rate = RATE * (1 / 120)
        theirs += np.clip(target - theirs, -rate, rate)
        assert np.array_equal(mine, theirs)
        assert ack.steering_error(target, cmd) == float(np.max(np.abs(target - cmd)))
        assert ack.finalize(want, mine) is want
    assert ack.steering_dim == 2
    assert ack.creep_threshold_rad == CREEP_STEERING_ERROR_RAD
    assert ack.presteer_threshold_rad == SceneReplanConfig().presteer_tolerance_rad


def test_held_mean_and_protection_bound_differ(ack):
    """(+0.08, -0.08): the mean hides it, the bound does not (Codex 4a' P1-3)."""
    assert ack.held_curvature((0.08, -0.08)) == pytest.approx(0.0, abs=1e-15)
    assert ack.curvature_bound((0.08, -0.08)) == pytest.approx(0.1176831600561432)


def test_real_command_is_k6_with_one_steering_state(real):
    d = real.command(0.3, 0.2)
    assert real.steering_dim == 1 and len(d.steering_rad) == 1 and len(d.wheel_rates_rad_s) == 2
    assert math.tan(d.steering_rad[0]) / 0.66 == pytest.approx(0.2)
    assert isinstance(real.command(0.3, 0.5), DriveRefusal)
    assert real.steering_targets(0.5) is None
    assert real.steering_targets(0.2) == pytest.approx((math.atan(0.66 * 0.2),))


def test_real_slew_moves_one_state_at_its_own_rate(real):
    cmd = np.array([0.0])
    real.slew_steering(cmd, (0.2,), 0.02)
    assert cmd[0] == pytest.approx(real.steering_rate_rad_s * 0.02)
    assert real.steering_error((0.2,), cmd) == pytest.approx(0.2 - cmd[0])


def test_real_finalize_drives_the_wheels_on_the_slewed_steering(real):
    target = real.command(0.3, 0.3)
    cmd = np.array([0.05])  # the steering has not got there yet
    fin = real.finalize(target, cmd)
    k_cmd = math.tan(0.05) / 0.66
    assert fin.curvature_inv_m == pytest.approx(k_cmd) and fin.steering_rad == (0.05,)
    assert fin.speed_mps == pytest.approx(0.3)
    u_l, u_r = fin.wheel_rates_rad_s[0] * 0.125, fin.wheel_rates_rad_s[1] * 0.125
    assert (u_r - u_l) / (0.53 * fin.speed_mps) == pytest.approx(k_cmd)


@pytest.mark.parametrize("bad", [float("nan"), 0.5])
def test_real_unknown_steering_is_none_and_finalize_brakes_without_a_curvature(real, bad):
    assert real.held_curvature((bad,)) is None and real.curvature_bound((bad,)) is None
    fin = real.finalize(real.command(0.3, 0.1), np.array([bad]))
    assert fin.speed_mps == 0.0 and fin.wheel_rates_rad_s == (0.0, 0.0)
    assert math.isnan(fin.steering_rad[0]) if math.isnan(bad) else fin.steering_rad == (bad,)


def test_real_thresholds_come_from_the_vehicle_config():
    sc = {"steering_rate_rad_s": 0.7, "creep_threshold_rad": 0.04, "presteer_threshold_rad": 0.015,
          "straight_threshold_rad": 0.008, "provenance": PROV}
    k = RealKinematics(real_vehicle_from_mapping(mapping(steering_control=sc), CHASSIS))
    assert (k.steering_rate_rad_s, k.creep_threshold_rad, k.presteer_threshold_rad, k.straight_threshold_rad) == \
        (0.7, 0.04, 0.015, 0.008)


@pytest.mark.parametrize("bad_shape", [(0.1, 0.1), ()])
def test_shapes_are_checked(real, ack, bad_shape):
    with pytest.raises(ValueError):
        real.steering_error((0.1,), np.asarray(bad_shape))
    with pytest.raises(ValueError):
        ack.held_curvature((0.1,))


def test_teleop_limits_take_the_real_kinematics_tolerances(ack):
    from forklift_core.control.speed_chain import creep
    from forklift_core.plant.kinematics import teleop_limit_overrides
    from forklift_core.safety.teleop_drive import TeleopLimits

    sc = {"steering_rate_rad_s": 0.7, "creep_threshold_rad": 0.2, "presteer_threshold_rad": 0.015,
          "straight_threshold_rad": 0.03, "provenance": PROV}
    k = RealKinematics(real_vehicle_from_mapping(mapping(steering_control=sc), CHASSIS))
    lim = TeleopLimits(**teleop_limit_overrides(k))
    assert (lim.creep_threshold_rad, lim.recovery_steer_tol_rad) == (0.2, 0.03)
    assert TeleopLimits(**teleop_limit_overrides(ack)) == TeleopLimits()
    assert creep(0.4, 0.1, lim.creep_threshold_rad) == 0.4 and creep(0.4, 0.1) == 0.1  # AUTO uses the same rule
