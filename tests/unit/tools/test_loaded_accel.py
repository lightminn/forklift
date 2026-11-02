"""The carried pallet's acceleration from the truck's truth pose (tools/loaded_accel.py)."""
import numpy as np
import pytest

from tools import loaded_accel as la


def _control(t, x, y, th):
    c = np.zeros((len(t), 7))
    c[:, 0], c[:, 4], c[:, 5], c[:, 6] = t, x, y, th
    return c


@pytest.mark.parametrize("n", [1, 3, 12])
def test_a_steady_turn_gives_the_point_its_distance_from_the_turn_centre_times_omega_squared(n):
    # the rear axle on a circle of radius R at speed v; a point d ahead of it is sqrt(R^2 + d^2) from the centre
    R, v, d, dt = 2.5, 0.3, 1.26, 1 / 120
    t = np.arange(0, 60, dt)
    w = v / R
    th = w * t
    x, y = R * np.sin(th), R * (1 - np.cos(th))
    tt, a = la.pallet_acceleration(_control(t, x, y, th), d, n)
    s = la.window_stats(tt, a, 10.0, 50.0, n)
    assert s["max"] == pytest.approx(w * w * np.hypot(R, d), rel=1e-3)


def test_a_straight_brake_is_the_deceleration_alone():
    dt, a0 = 1 / 120, 1.5
    t = np.arange(0, 20, dt)
    v = np.clip(1.0 - a0 * np.clip(t - 10.0, 0, None), 0, None)
    x = np.cumsum(v) * dt
    tt, a = la.pallet_acceleration(_control(t, x, np.zeros_like(t), np.zeros_like(t)), 1.26, 3)
    s = la.window_stats(tt, a, 10.1, 10.5, 3)
    assert s["max"] == pytest.approx(a0, rel=0.05)
