import math
from pathlib import Path

import numpy as np
import pytest

from forklift_core.planning.fleet_physics import (
    Chassis,
    Load,
    Mass,
    curvature,
    limits,
    speed_profile,
    stop_cost_s,
)

URDF = Path(__file__).resolve().parents[3] / "sim/models/dls08_measured/forklift.urdf"


def _box_chassis(**kw):
    # 10 kg at the origin, axles at +-0.5 m, track 0.5 m, centre of gravity 0.25 m up.
    return Chassis(
        parts=(Mass(10.0, 0.0, 0.25),), carriage=Mass(0.0, 0.0, 0.0), front_axle_x_m=0.5,
        rear_axle_x_m=-0.5, track_m=0.5, wheel_radius_m=0.1, fork_tip_x_m=1.0, fork_height_m=0.0,
        **{"wheel_speed_radps": 20.0, **kw},
    )


def test_tipping_bounds_follow_the_centre_of_gravity():
    lim = limits(_box_chassis(), share=1.0)
    assert lim.tip_backward == pytest.approx(9.81 * 0.5 / 0.25)
    assert lim.tip_forward == pytest.approx(9.81 * 0.5 / 0.25)
    assert lim.tip_lateral == pytest.approx(9.81 * 0.25 / 0.25)
    # A load ahead of the front axle moves the centre of gravity forward:
    # less braking (front tip), more speeding up (rear tip).
    load = Load(2.0, depth_m=0.0, insert_m=0.0, opening_top_m=0.0, com_height_m=0.25, fork_thickness_m=0.0)
    loaded = limits(_box_chassis(), load, share=1.0)
    x = 2.0 * 1.0 / 12.0
    assert loaded.cog_x_m == pytest.approx(x)
    assert loaded.tip_backward == pytest.approx(9.81 * (0.5 - x) / 0.25)
    assert loaded.tip_forward == pytest.approx(9.81 * (x + 0.5) / 0.25)


def test_lifting_raises_the_centre_of_gravity_and_cuts_braking():
    chassis = Chassis.from_urdf(URDF)
    low = limits(chassis, Load(10.0), lift_m=0.0)
    high = limits(chassis, Load(10.0), lift_m=0.2)
    assert high.cog_z_m > low.cog_z_m and high.backward < low.backward
    assert high.cog_x_m == pytest.approx(low.cog_x_m)


def test_the_measured_chassis_reads_from_its_urdf():
    chassis = Chassis.from_urdf(URDF)
    assert chassis.mass_kg == pytest.approx(24.0, abs=0.05)
    assert chassis.front_axle_x_m == pytest.approx(0.32)
    assert chassis.rear_axle_x_m == pytest.approx(-0.34)
    assert chassis.track_m == pytest.approx(0.53)
    assert chassis.wheel_radius_m == pytest.approx(0.125)
    # Too heavy a load tips it standing still: no braking left.
    assert limits(chassis, Load(14.0)).backward == 0.0


def test_caps_and_traction_bound_the_physics():
    lim = limits(_box_chassis(friction=0.1), share=1.0)
    assert lim.forward == pytest.approx(0.981) and lim.backward == pytest.approx(0.981)
    capped = limits(_box_chassis(), share=1.0, cap_mps2=0.3)
    assert capped.forward == capped.backward == pytest.approx(0.3)
    assert capped.lateral > 0.3  # the tracker cap is longitudinal only


def test_profile_stops_at_ends_and_gear_changes_and_brakes_with_its_limit():
    out = np.column_stack((np.linspace(0, 10, 101), np.zeros(101), np.zeros(101)))
    back = np.column_stack((np.linspace(10, 6, 41)[1:], np.zeros(40), np.zeros(40)))  # reverse
    path = np.vstack((out, back))
    lim = limits(_box_chassis(), share=1.0, cap_mps2=1.0)
    s, v, t = speed_profile(path, 1.0, lim)
    assert v[0] == v[100] == v[-1] == 0.0
    assert v.max() == pytest.approx(1.0)
    # 10 m out: 1 s up, 8.? m at 1 m/s, 1 s down -> 11 s; 4 m back: 1 + 3 + 1 = 5 s.
    assert t[100] == pytest.approx(11.0, abs=0.05)
    assert t[-1] == pytest.approx(16.0, abs=0.05)


def test_braking_harder_limits_than_speeding_up_shape_the_profile():
    chassis = _box_chassis()
    lim = limits(chassis, share=1.0)
    lim = type(lim)(**{**lim.__dict__, "forward": 1.0, "backward": 0.25})
    path = np.column_stack((np.linspace(0, 10, 201), np.zeros(201), np.zeros(201)))
    s, v, t = speed_profile(path, 1.0, lim)
    # Speeding up forward uses 1.0 (1 m to cruise), braking 0.25 (2 m).
    first = s[np.argmax(v >= 1.0 - 1e-6)]
    last = s[len(v) - 1 - np.argmax(v[::-1] >= 1.0 - 1e-6)]
    assert first == pytest.approx(0.5, abs=0.06)
    assert 10 - last == pytest.approx(2.0, abs=0.06)


def test_curves_cap_the_speed_by_lateral_acceleration():
    angle = np.linspace(0, math.pi / 2, 200)
    radius = 1.0
    arc = np.column_stack((radius * np.sin(angle), radius * (1 - np.cos(angle)), angle))
    assert np.median(curvature(arc)) == pytest.approx(1.0, rel=0.02)
    lim = limits(_box_chassis(), share=1.0)
    lim = type(lim)(**{**lim.__dict__, "lateral": 0.25, "forward": 10.0, "backward": 10.0})
    _, v, _ = speed_profile(arc, 2.0, lim)
    assert v.max() == pytest.approx(0.5, rel=0.03)


def test_stopping_costs_more_with_a_heavy_load():
    chassis = Chassis.from_urdf(URDF)
    light = limits(chassis, None, 0.2, cap_mps2=0.5)
    heavy = limits(chassis, Load(12.5), 0.2, cap_mps2=0.5)
    assert stop_cost_s(0.3, heavy) > stop_cost_s(0.3, light) > 0
    assert stop_cost_s(0.0, heavy) == 0.0
