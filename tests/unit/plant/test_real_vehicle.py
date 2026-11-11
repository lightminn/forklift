"""K6 real-vehicle drive conversion (docs/plans/2026-11-11-sim-to-real-bringup.md P0-6)."""

import copy
import math
from pathlib import Path

import numpy as np
import pytest

from forklift_core.control.path_tracking import AckermannGeometry, ackermann_command
from forklift_core.localization.wheel_odometry import (
    AckermannOdometryGeometry,
    advance_arc,
    integrate_wheel_odometry_increments,
)
from forklift_core.plant.real_vehicle import (
    DriveRefusal,
    RealOdometry,
    RearDrive,
    load_real_vehicle,
    real_vehicle_from_mapping,
    rear_drive_command,
)

REPO = Path(__file__).resolve().parents[3]
VEHICLE_YAML = REPO / "config" / "real" / "vehicle.yaml"
CHASSIS = {"dimensions": {"front_axle_x_m": 0.32, "rear_axle_x_m": -0.34, "wheel_radius_m": 0.125,
                          "wheel_track_m": 0.53},
           "assumptions": {"steering_limit_rad": 0.2617993878}}
PROV = {"source": "test", "measured": False, "until": "S2"}


def mapping(**over):
    data = {
        "schema": "real_vehicle/1",
        "wheel_counts": {side: {"motor_counts_per_rev": 4096, "gear_ratio": 80.0, "provenance": PROV}
                         for side in ("left", "right")},
        "wheel_radius": {"left": None, "right": None, "provenance": PROV},
        "steering_map": {"kind": "ideal_bicycle", "provenance": PROV},
        "wheel_rate_limit": {"rad_s": 8.0, "provenance": PROV},
        "steering_control": {"steering_rate_rad_s": 1.0, "creep_threshold_rad": 0.05, "presteer_threshold_rad": 0.02,
                             "straight_threshold_rad": 0.01, "provenance": PROV},
        "teleop_speed_limit": {"mps": 0.2, "provenance": PROV},
    }
    data.update(over)
    return data


@pytest.fixture
def cfg():
    return real_vehicle_from_mapping(mapping(), CHASSIS)


def test_repository_config_resolves_the_chassis_model_and_the_user_estimate():
    c = load_real_vehicle(VEHICLE_YAML)
    assert c.counts_per_wheel_rev == (327680.0, 327680.0)
    assert c.wheelbase_m == pytest.approx(0.66)
    assert c.track_m == pytest.approx(0.53)
    assert c.wheel_radius_m == (0.125, 0.125)
    assert c.wheel_rate_limit_rad_s == 8.0
    assert c.steering.kind == "ideal_bicycle"
    # Inner wheel at the model's 15 deg (Codex K6 round 1 recomputation).
    assert c.steering.curvature_range[1] == pytest.approx(0.366548, abs=1e-6)
    assert c.steering.angle_range[1] == pytest.approx(0.237361, abs=1e-6)
    assert c.provenance["wheel_counts.left"]["source"] == "user_estimate_2026_11_11"
    assert c.provenance["wheel_counts.left"]["measured"] is False


def test_loading_does_not_depend_on_the_working_directory(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    assert load_real_vehicle(VEHICLE_YAML).counts_per_wheel_rev[0] == 327680.0


def test_measured_counts_per_wheel_rev_may_be_fractional():
    counts = {side: {"counts_per_wheel_rev": 326041.6, "provenance": PROV} for side in ("left", "right")}
    c = real_vehicle_from_mapping(mapping(wheel_counts=counts), CHASSIS)
    assert c.counts_per_wheel_rev == (326041.6, 326041.6)


def test_effective_radius_per_side_overrides_the_nominal():
    radius = {"left": 0.124, "right": 0.1255, "provenance": PROV}
    assert real_vehicle_from_mapping(mapping(wheel_radius=radius), CHASSIS).wheel_radius_m == (0.124, 0.1255)


@pytest.mark.parametrize("bad", [
    {"schema": "real_vehicle/2"},
    {"wheel_counts": {"left": {"motor_counts_per_rev": 4096, "gear_ratio": 80.0, "counts_per_wheel_rev": 1.0,
                               "provenance": PROV},
                      "right": {"counts_per_wheel_rev": 1.0, "provenance": PROV}}},
    {"wheel_counts": {"left": {"provenance": PROV}, "right": {"counts_per_wheel_rev": 1.0, "provenance": PROV}}},
    {"wheel_counts": {"left": {"counts_per_wheel_rev": 0.0, "provenance": PROV},
                      "right": {"counts_per_wheel_rev": 1.0, "provenance": PROV}}},
    {"wheel_counts": {"left": {"counts_per_wheel_rev": 1.0},
                      "right": {"counts_per_wheel_rev": 1.0, "provenance": PROV}}},
    {"wheel_rate_limit": {"rad_s": float("nan"), "provenance": PROV}},
    {"steering_map": {"kind": "table", "points": [[-0.2, -0.3], [0.0, 0.0], [0.0, 0.1]], "provenance": PROV}},
    {"steering_map": {"kind": "table", "points": [[-0.2, 0.3], [0.2, -0.3]], "provenance": PROV}},
    {"steering_map": {"kind": "table", "points": [[0.0, 0.0]], "provenance": PROV}},
    {"steering_map": {"kind": "spline", "provenance": PROV}},
    {"extra": 1},
])
def test_malformed_configs_are_refused(bad):
    with pytest.raises(ValueError):
        real_vehicle_from_mapping(mapping(**bad), CHASSIS)


def test_straight_drive_runs_both_wheels_alike(cfg):
    d = rear_drive_command(0.5, 0.0, cfg)
    assert isinstance(d, RearDrive)
    assert d.left_wheel_rad_s == pytest.approx(4.0) and d.right_wheel_rad_s == pytest.approx(4.0)
    assert d.steering_rad == 0.0 and not d.speed_limited


@pytest.mark.parametrize("speed,kappa", [(0.3, 0.2), (-0.25, -0.3), (0.4, 0.36), (-0.1, 0.1)])
def test_rear_rates_and_curvature_match_the_sim_conversion_below_saturation(cfg, speed, kappa):
    d = rear_drive_command(speed, kappa, cfg)
    sim = ackermann_command(speed, kappa, AckermannGeometry(0.66, 0.53, 0.125, 0.2617993878, 8.0))
    assert d.curvature_inv_m == pytest.approx(sim.curvature_inv_m)
    assert d.speed_mps == pytest.approx(sim.speed_mps)
    assert (d.left_wheel_rad_s, d.right_wheel_rad_s) == pytest.approx(sim.wheel_rates_rad_s[2:])
    assert math.tan(d.steering_rad) / 0.66 == pytest.approx(kappa)  # ideal map: q is the bicycle angle


def test_speed_saturation_keeps_curvature_and_scales_both_wheels(cfg):
    kmax = cfg.steering.curvature_range[1]
    d = rear_drive_command(5.0, kmax, cfg)
    assert d.speed_limited and d.curvature_inv_m == kmax
    assert max(abs(d.left_wheel_rad_s), abs(d.right_wheel_rad_s)) == pytest.approx(8.0)
    assert d.right_wheel_rad_s / d.left_wheel_rad_s == pytest.approx((1 + kmax * 0.265) / (1 - kmax * 0.265))
    # Rear wheels only: a looser limit than the sim's four-wheel one (intended difference).
    assert d.speed_mps == pytest.approx(0.911465, abs=1e-6)
    sim = ackermann_command(5.0, kmax, AckermannGeometry(0.66, 0.53, 0.125, 0.2617993878, 8.0))
    assert sim.speed_mps == pytest.approx(0.890083, abs=1e-6)
    r = rear_drive_command(-5.0, -kmax, cfg)
    assert r.speed_mps == pytest.approx(-0.911465, abs=1e-6) and r.left_wheel_rad_s < 0


def test_infeasible_curvature_is_refused_not_slowed(cfg):
    kmax = cfg.steering.curvature_range[1]
    out = rear_drive_command(0.1, kmax * 1.001, cfg)
    assert isinstance(out, DriveRefusal)
    assert out.reason == "curvature_infeasible"
    assert out.requested_curvature_inv_m == pytest.approx(kmax * 1.001)
    assert out.curvature_range == cfg.steering.curvature_range
    edge = rear_drive_command(0.1, kmax + 1e-12, cfg)  # numerical noise at the edge is normalised
    assert isinstance(edge, RearDrive) and edge.curvature_inv_m == kmax
    assert isinstance(rear_drive_command(0.0, -kmax * 1.5, cfg), DriveRefusal)


def test_table_map_may_be_asymmetric_and_is_inverted(cfg):
    table = {"kind": "table", "points": [[-0.30, -0.33], [0.0, 0.0], [0.20, 0.25], [0.28, 0.36]],
             "provenance": PROV}
    c = real_vehicle_from_mapping(mapping(steering_map=table), CHASSIS)
    assert c.steering.curvature_range == (-0.33, 0.36)
    assert c.steering.angle_range == (-0.30, 0.28)
    d = rear_drive_command(0.2, 0.125, c)
    assert d.steering_rad == pytest.approx(0.10)
    assert c.steering.curvature(0.24) == pytest.approx(0.305)
    assert c.steering.curvature(0.29) is None  # outside the table: no extrapolation
    assert isinstance(rear_drive_command(0.2, -0.34, c), DriveRefusal)


def test_non_finite_requests_raise(cfg):
    with pytest.raises(ValueError):
        rear_drive_command(float("nan"), 0.0, cfg)
    with pytest.raises(ValueError):
        rear_drive_command(0.1, float("inf"), cfg)


def odo(cfg, **kw):
    return RealOdometry(cfg, max_gap_s=kw.pop("max_gap_s", 0.1))


def test_straight_counts_give_distance(cfg):
    o = odo(cfg)
    first = o.update(0, 7, 0, 0, 0.0)
    assert not first.integrated and not first.continuity_broken
    step = o.update(10_000, 7, 327680, 327680, 0.0)
    assert step.integrated and step.pose == pytest.approx((2 * math.pi * 0.125, 0.0, 0.0))


def test_counts_wrap_modulo_2_32(cfg):
    o = odo(cfg)
    o.update(0, 1, 2**31 - 100, 2**31 - 100, 0.0)
    step = o.update(10_000, 1, -(2**31) + 100, -(2**31) + 100, 0.0)  # +200 counts across the i32 boundary
    assert step.pose[0] == pytest.approx(200 * 2 * math.pi / 327680 * 0.125)


def test_boot_change_rereferences_without_a_jump(cfg):
    o = odo(cfg)
    o.update(0, 1, 0, 0, 0.0)
    o.update(10_000, 1, 1000, 1000, 0.0)
    x = o.pose[0]
    step = o.update(5_000, 2, 5, 5, 0.0)  # new boot: time and counts restart
    assert step.continuity_broken and step.reason == "boot_changed" and not step.integrated
    assert o.pose[0] == x
    nxt = o.update(15_000, 2, 105, 105, 0.0)
    assert nxt.integrated and o.pose[0] == pytest.approx(x + 100 * 2 * math.pi / 327680 * 0.125)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), 0.30])
def test_invalid_steering_breaks_the_reference_and_the_gap_is_not_integrated(cfg, bad):
    o = odo(cfg)
    o.update(0, 1, 0, 0, 0.0)
    o.update(10_000, 1, 1000, 1000, 0.0)
    x = o.pose[0]
    lost = o.update(20_000, 1, 2000, 2000, bad)  # 0.30 rad is outside the ideal map's range
    assert lost.continuity_broken and lost.reason == "steering_invalid" and not lost.integrated
    back = o.update(30_000, 1, 9000, 9000, 0.0)  # recovery sample only re-references
    assert not back.integrated and back.reason == "rereference" and o.pose[0] == x
    assert o.update(40_000, 1, 9100, 9100, 0.0).integrated


def test_long_gaps_break_the_reference(cfg):
    o = odo(cfg, max_gap_s=0.05)
    o.update(0, 1, 0, 0, 0.0)
    step = o.update(60_000, 1, 1000, 1000, 0.0)
    assert step.continuity_broken and step.reason == "gap" and o.pose == (0.0, 0.0, 0.0)


def test_non_increasing_time_is_ignored(cfg):
    o = odo(cfg)
    o.update(10_000, 1, 0, 0, 0.0)
    step = o.update(10_000, 1, 500, 500, 0.0)
    assert step.reason == "time_not_increasing" and not step.integrated and not step.continuity_broken
    assert o.update(20_000, 1, 1000, 1000, 0.0).pose[0] == pytest.approx(1000 * 2 * math.pi / 327680 * 0.125)


def test_matches_the_sim_integrator_with_changing_steering(cfg):
    """Same counts and a varying curvature: the real path (one mechanism angle) and the sim path (two
    front wheel angles of the same curvature) integrate the same poses (end-point mean curvature)."""
    n = 400
    t_us = np.arange(n) * 10_000
    kappa = 0.3 * np.sin(np.linspace(0, 3 * math.pi, n))
    counts = np.cumsum(np.r_[0, np.full(n - 1, 900)])
    o = odo(cfg)
    real = np.array([o.update(int(t), 3, int(c), int(c), math.atan(0.66 * k)).pose
                     for t, c, k in zip(t_us, counts, kappa, strict=True)])
    h = 0.265
    front = np.c_[np.arctan(0.66 * kappa / (1 - kappa * h)), np.arctan(0.66 * kappa / (1 + kappa * h))]
    inc = np.r_[0, np.diff(counts)] * 2 * math.pi / 327680
    sim = integrate_wheel_odometry_increments(t_us / 1e6, np.c_[inc, inc], front,
                                              AckermannOdometryGeometry(0.66, 0.53, 0.125))
    assert real == pytest.approx(sim, abs=1e-9)


def test_calibration_id_follows_the_resolved_values(cfg):
    same = real_vehicle_from_mapping(mapping(), copy.deepcopy(CHASSIS))
    assert same.calibration_id == cfg.calibration_id and len(cfg.calibration_id) == 16
    counts = {side: {"motor_counts_per_rev": 4096, "gear_ratio": 79.6, "provenance": PROV}
              for side in ("left", "right")}
    assert real_vehicle_from_mapping(mapping(wheel_counts=counts), CHASSIS).calibration_id != cfg.calibration_id
    prov = {"source": "other", "measured": False, "until": "S2"}
    other = mapping(wheel_rate_limit={"rad_s": 8.0, "provenance": prov})
    assert real_vehicle_from_mapping(other, CHASSIS).calibration_id == cfg.calibration_id  # values only


@pytest.mark.parametrize("table", [False, True])
def test_map_ends_survive_the_float32_wire(cfg, table):
    """The MCU reports q as an f32: a STATE at the map's end must not read as invalid (Codex K6 round 2)."""
    if table:
        sm = {"kind": "table", "points": [[-0.3000001, -0.33], [0.0, 0.0], [0.2800003, 0.36]], "provenance": PROV}
        cfg = real_vehicle_from_mapping(mapping(steering_map=sm), CHASSIS)
    for end in cfg.steering.angle_range:
        wire = float(np.float32(end))
        assert wire != end  # the case is real for these ends
        assert cfg.steering.curvature(wire) == pytest.approx(cfg.steering.curvature(end))
        o = odo(cfg)
        o.update(0, 1, 0, 0, wire)
        assert o.update(10_000, 1, 100, 100, wire).integrated
    beyond = cfg.steering.angle_range[1] * (1 + 1e-5)
    assert cfg.steering.curvature(beyond) is None


def test_a_valid_sample_ending_a_gap_is_the_next_reference(cfg):
    o = odo(cfg, max_gap_s=0.05)
    o.update(0, 1, 0, 0, 0.0)
    assert o.update(60_000, 1, 1000, 1000, 0.0).reason == "gap"
    nxt = o.update(70_000, 1, 1100, 1100, 0.0)
    assert nxt.integrated and o.pose[0] == pytest.approx(100 * 2 * math.pi / 327680 * 0.125)


@pytest.mark.parametrize("speed,kappa", [(0.4, 0.25), (-0.3, -0.2), (5.0, 0.3), (-5.0, 0.1)])
def test_unequal_sides_round_trip_through_counts(speed, kappa):
    """Different radii and count scales per side: the commanded rates, turned into counts, integrate back
    to the commanded (applied) speed and curvature."""
    counts = {"left": {"counts_per_wheel_rev": 326000.0, "provenance": PROV},
              "right": {"counts_per_wheel_rev": 328500.5, "provenance": PROV}}
    radius = {"left": 0.124, "right": 0.1262, "provenance": PROV}
    c = real_vehicle_from_mapping(mapping(wheel_counts=counts, wheel_radius=radius), CHASSIS)
    d = rear_drive_command(speed, kappa, c)
    assert isinstance(d, RearDrive)
    assert max(abs(d.left_wheel_rad_s), abs(d.right_wheel_rad_s)) <= 8.0 + 1e-12
    u_l, u_r = d.left_wheel_rad_s * 0.124, d.right_wheel_rad_s * 0.1262
    assert (u_l + u_r) / 2 == pytest.approx(d.speed_mps)
    assert (u_r - u_l) / (0.53 * d.speed_mps) == pytest.approx(d.curvature_inv_m)
    o = odo(c)
    dt = 0.01
    o.update(0, 1, 0, 0, d.steering_rad)
    n = 100
    for k in range(1, n + 1):
        cl = round(d.left_wheel_rad_s * k * dt * 326000.0 / (2 * math.pi))
        cr = round(d.right_wheel_rad_s * k * dt * 328500.5 / (2 * math.pi))
        o.update(k * 10_000, 1, cl, cr, d.steering_rad)
    expected = advance_arc(0.0, 0.0, 0.0, d.speed_mps * n * dt, d.curvature_inv_m)
    assert o.pose == pytest.approx(expected, abs=2e-5)


def test_the_teleop_speed_limit_is_read_with_its_provenance():
    """Lead decision 2026-11-11 (truth audit 3a): teleop without SLAM under an encoder-judged cap, provisional to S5."""
    c = load_real_vehicle(VEHICLE_YAML)
    assert c.teleop_speed_limit_mps == 0.2
    assert c.provenance["teleop_speed_limit"]["until"] == "S5" and c.provenance["teleop_speed_limit"]["measured"] is False
    with pytest.raises(ValueError):
        real_vehicle_from_mapping(mapping(teleop_speed_limit={"mps": 0.0, "provenance": PROV}), CHASSIS)


def test_an_encoder_continuity_cut_breaks_the_odometry_and_the_next_sample_rereferences():
    """mcu-link/2 bits 17/22/23 (Codex PR review P2): nothing is integrated across them."""
    from forklift_core.plant.real_vehicle import RealOdometry, load_real_vehicle

    cfg = load_real_vehicle(Path(__file__).resolve().parents[3] / "config" / "real" / "vehicle.yaml")
    odo = RealOdometry(cfg, max_gap_s=0.5)
    odo.update(0, 1, 0, 0, 0.0)
    assert odo.update(10_000, 1, 1000, 1000, 0.0).integrated
    pose = odo.pose
    cut = odo.update(20_000, 1, 900_000, 900_000, 0.0, encoder_cut=True)  # a jump: those counts are not motion
    assert not cut.integrated and cut.continuity_broken and cut.reason == "encoder_continuity" and odo.pose == pose
    re = odo.update(30_000, 1, 901_000, 901_000, 0.0)
    assert not re.integrated and re.reason == "rereference" and odo.pose == pose
    assert odo.update(40_000, 1, 902_000, 902_000, 0.0).integrated
