"""The ``m_analysis`` changes the G8 entry needs (docs/design/2026-11-15-m-analyze-entry.md 4, with 3.3 and 9): the
truth heading reference of M1/M2 beside the old ``imu_dyaw`` alias, the odometry pose, the relative pose with its
propagated covariance, the Allan deviation on actual timestamps and a ``quiet_window`` run that never spans a gap."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

from forklift_core.mcu_link.messages import FaultFlag
from forklift_core.plant import m_analysis as ma
from forklift_core.plant.real_vehicle import RealOdometry

_HERE = str(Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from m_synth import NS, drive_states, imu_series, ns, odometry_cfg, unit, wrap_i32  # noqa: E402

UP = unit([0.08, -0.99, 0.11])
BIAS = np.array([0.003, -0.002, 0.001])
CFG = odometry_cfg()


# --- (가) heading reference: truth / imu, and the imu_dyaw alias ---------------------------------------------------

M1 = {"odo_dyaw": 0.012, "distance_m": 3.0, "distance_source": "truth"}
M2 = {"distance_m": 1.5, "direction": 1, "kappa_measured_q": 0.33, "kappa_commanded_q": 0.36,
      "distance_source": "truth"}


@pytest.mark.parametrize("source", ["truth", "imu"])
def test_m1_new_path_returns_the_reference_difference_and_its_source(source):
    m = ma.m1_drift(heading_ref_dyaw=0.003, ref_source=source, **M1)
    assert m == {"rad_per_m": pytest.approx(0.003), "odo_minus_ref_rad": pytest.approx(0.009), "ref_source": source,
                 "distance_m": 3.0, "distance_source": "truth", "note": ""}
    assert "odo_minus_imu_rad" not in m  # not even when the reference is the IMU


def test_m1_alias_path_is_the_old_dict_plus_the_source():
    m = ma.m1_drift(imu_dyaw=0.003, **M1)
    assert m == {"rad_per_m": pytest.approx(0.003), "odo_minus_imu_rad": pytest.approx(0.009), "ref_source": "imu",
                 "distance_m": 3.0, "distance_source": "truth", "note": ""}
    new = ma.m1_drift(heading_ref_dyaw=0.003, ref_source="imu", **M1)
    assert m["rad_per_m"] == new["rad_per_m"] and m["odo_minus_imu_rad"] == new["odo_minus_ref_rad"]
    enc = ma.m1_drift(odo_dyaw=0.0, heading_ref_dyaw=0.0, ref_source="truth", distance_m=3.0,
                      distance_source="encoder")
    assert "radius" in enc["note"] and enc["ref_source"] == "truth"


def test_m2_truth_path_full_dict():
    g = ma.m2_gains(heading_ref_dyaw=0.45, ref_source="truth", **M2)
    assert g == {"kappa_true": pytest.approx(0.3), "gain_measured_q": pytest.approx(0.3 / 0.33),
                 "gain_commanded_q": pytest.approx(0.3 / 0.36), "distance_m": 1.5, "direction": 1,
                 "distance_source": "truth", "ref_source": "truth", "note": ""}


def test_m2_imu_reference_says_a_gyro_scale_is_needed_on_both_paths():
    new = ma.m2_gains(heading_ref_dyaw=0.45, ref_source="imu", **M2)
    alias = ma.m2_gains(imu_dyaw=0.45, **M2)
    assert new == alias
    assert alias["ref_source"] == "imu" and "gyro scale needed" in alias["note"]
    assert set(alias) == {"kappa_true", "gain_measured_q", "gain_commanded_q", "distance_m", "direction",
                          "distance_source", "ref_source", "note"}
    enc = ma.m2_gains(imu_dyaw=0.45, **{**M2, "distance_source": "encoder"})
    assert "radius" in enc["note"] and "gyro scale needed" in enc["note"]


@pytest.mark.parametrize("fn, base", [(ma.m1_drift, M1), (ma.m2_gains, M2)])
@pytest.mark.parametrize("ref", [
    {"heading_ref_dyaw": 0.1, "ref_source": "truth", "imu_dyaw": 0.1},  # both
    {"heading_ref_dyaw": 0.1, "imu_dyaw": 0.1},  # both, no source
    {},  # neither
    {"ref_source": "truth"},  # a source without a value
    {"imu_dyaw": 0.1, "ref_source": "imu"},  # the alias takes no source
    {"imu_dyaw": 0.1, "ref_source": "truth"},
    {"heading_ref_dyaw": 0.1},  # the new path needs its source
    {"heading_ref_dyaw": 0.1, "ref_source": "gyro"},
    {"heading_ref_dyaw": 0.1, "ref_source": "TRUTH"},
])
def test_the_heading_reference_is_exactly_one_with_a_known_source(fn, base, ref):
    with pytest.raises(ValueError):
        fn(**base, **ref)


def test_the_reference_arguments_stay_keyword_only():
    with pytest.raises(TypeError):
        ma.m1_drift(0.012, 0.003, "truth", None, 3.0, "truth")  # type: ignore[misc]


# --- (나) odometry_pose -------------------------------------------------------------------------------------------

_ENCODER_CUT = FaultFlag.ENCODER_GAP | FaultFlag.ENCODER_JUMP_L | FaultFlag.ENCODER_JUMP_R


def _previous_odometry_heading(states, cfg, *, max_gap_s):
    """The implementation before the 2026-11-15 change, kept verbatim as the wrapper's regression reference."""
    odo = RealOdometry(cfg, max_gap_s=max_gap_s)
    t, yaw, broken = [], [], []
    for s in states:
        cut = bool(int(s.fault_flags) & int(_ENCODER_CUT))
        step = odo.update(s.mcu_time_us, s.boot_id, s.wheel_count_left, s.wheel_count_right, s.steering_rad,
                          encoder_cut=cut)
        t.append(int(s.mcu_time_us))
        yaw.append(step.pose[2])
        if step.continuity_broken and len(t) > 1:
            broken.append((int(s.mcu_time_us), step.reason))
    return {"mcu_time_us": np.asarray(t, dtype=np.int64), "yaw": np.asarray(yaw), "broken": broken}


def _odd_records():
    plain = drive_states(1001, speed_mps=0.1, kappa=0.2, cfg=CFG)
    jump = drive_states(1001, speed_mps=0.1, kappa=-0.2, cfg=CFG, radius_error=(0.02, 0.0))
    jump[400].fault_flags = int(FaultFlag.ENCODER_JUMP_L)
    for s in jump[400:]:
        s.wheel_count_left = wrap_i32(s.wheel_count_left + 100_000)
    a = drive_states(301, speed_mps=0.1, kappa=0.2, cfg=CFG, t0_us=5_000_000, boot_id=7)
    b = drive_states(301, speed_mps=-0.1, kappa=0.2, cfg=CFG, t0_us=0, boot_id=8)
    bad_q = drive_states(201, speed_mps=0.1, kappa=0.2, cfg=CFG)
    bad_q[0].steering_rad = 0.9  # outside the map from the first sample
    bad_q[100].steering_rad = float("nan")
    return {"plain": plain, "gap": plain[:301] + plain[800:], "repeat": plain[:100] + [plain[99]] + plain[100:],
            "jump": jump, "boot": a + b, "steer": bad_q}


@pytest.mark.parametrize("name", ["plain", "gap", "repeat", "jump", "boot", "steer"])
def test_odometry_heading_is_a_wrapper_with_the_previous_values(name):
    st = _odd_records()[name]
    new, old = ma.odometry_heading(st, CFG, max_gap_s=0.1), _previous_odometry_heading(st, CFG, max_gap_s=0.1)
    assert set(new) == {"mcu_time_us", "yaw", "broken"}
    assert new["mcu_time_us"].dtype == np.int64 and np.array_equal(new["mcu_time_us"], old["mcu_time_us"])
    assert np.array_equal(new["yaw"], old["yaw"]) and new["broken"] == old["broken"]
    pose = ma.odometry_pose(st, CFG, max_gap_s=0.1)
    assert np.array_equal(pose["yaw"], new["yaw"]) and pose["broken"] == new["broken"]


def test_odometry_pose_of_a_straight_drive_and_an_arc():
    straight = ma.odometry_pose(drive_states(1001, speed_mps=0.1, kappa=0.0, cfg=CFG), CFG, max_gap_s=0.1)
    assert set(straight) == {"mcu_time_us", "x", "y", "yaw", "broken", "reasons"}
    assert straight["x"][-1] == pytest.approx(1.0, abs=1e-4) and straight["y"][-1] == pytest.approx(0.0, abs=1e-9)
    assert straight["broken"] == [] and straight["reasons"] == []
    kappa = 0.2
    arc = ma.odometry_pose(drive_states(1001, speed_mps=0.1, kappa=kappa, cfg=CFG), CFG, max_gap_s=0.1)
    th = arc["yaw"][-1]
    assert th == pytest.approx(kappa * 1.0, abs=1e-4)
    assert arc["x"][-1] == pytest.approx(math.sin(th) / kappa, abs=1e-6)  # on the circle of its own heading
    assert arc["y"][-1] == pytest.approx((1 - math.cos(th)) / kappa, abs=1e-6)
    rev = ma.odometry_pose(drive_states(1001, speed_mps=-0.1, kappa=0.0, cfg=CFG), CFG, max_gap_s=0.1)
    assert rev["x"][-1] == pytest.approx(-1.0, abs=1e-4)


def test_odometry_pose_reasons_name_what_the_breaks_do_not():
    rec = _odd_records()
    rep = ma.odometry_pose(rec["repeat"], CFG, max_gap_s=0.1)
    assert rep["broken"] == [] and rep["reasons"] == [(100, rec["repeat"][100].mcu_time_us, "time_not_increasing")]
    gap = ma.odometry_pose(rec["gap"], CFG, max_gap_s=0.1)
    assert gap["reasons"] == [(301, rec["gap"][301].mcu_time_us, "gap")]
    assert gap["broken"] == [(rec["gap"][301].mcu_time_us, "gap")]
    assert gap["x"][301] == gap["x"][300] and gap["y"][301] == gap["y"][300]
    jump = ma.odometry_pose(rec["jump"], CFG, max_gap_s=0.1)
    assert [r[1:] for r in jump["reasons"]] == [(rec["jump"][400].mcu_time_us, "encoder_continuity"),
                                                (rec["jump"][401].mcu_time_us, "rereference")]
    steer = ma.odometry_pose(rec["steer"], CFG, max_gap_s=0.1)
    # the first sample's invalid steering is a reason but (as before) not a break
    assert [(i, r) for i, _, r in steer["reasons"]] == [(0, "steering_invalid"), (1, "rereference"),
                                                       (100, "steering_invalid"), (101, "rereference")]
    assert [r for _, r in steer["broken"]] == ["steering_invalid"]
    boot = ma.odometry_pose(rec["boot"], CFG, max_gap_s=0.1)
    assert [(i, r) for i, _, r in boot["reasons"]] == [(301, "boot_changed")]


# --- (나) relative_pose ------------------------------------------------------------------------------------------

def test_relative_pose_from_a_rotated_start():
    r = ma.relative_pose((1.0, 2.0, math.pi / 2), (1.0, 3.0, math.pi / 2 + 0.1))
    assert (r["dx_fwd"], r["dy_lat"], r["dyaw"]) == pytest.approx((1.0, 0.0, 0.1), abs=1e-12)
    left = ma.relative_pose((1.0, 2.0, math.pi / 2), (0.0, 2.0, math.pi / 2))
    assert (left["dx_fwd"], left["dy_lat"]) == pytest.approx((0.0, 1.0), abs=1e-12)  # body left is +y
    back = ma.relative_pose((0.0, 0.0, -2.5), (-math.cos(-2.5), -math.sin(-2.5), -2.5))
    assert (back["dx_fwd"], back["dy_lat"], back["dyaw"]) == pytest.approx((-1.0, 0.0, 0.0), abs=1e-12)
    assert "cov" not in r


def test_relative_pose_yaw_wraps_across_pi():
    assert ma.relative_pose((0, 0, 3.13), (1, 0, -3.13))["dyaw"] == pytest.approx(2 * math.pi - 6.26, abs=1e-12)
    assert ma.relative_pose((0, 0, -3.13), (1, 0, 3.13))["dyaw"] == pytest.approx(6.26 - 2 * math.pi, abs=1e-12)
    assert ma.relative_pose((0, 0, 0.0), (1, 0, 4 * math.pi + 0.2))["dyaw"] == pytest.approx(0.2, abs=1e-12)


def _diag(sx, sy, sth):
    return np.diag([sx**2, sy**2, sth**2])


@pytest.mark.parametrize("length_m, lat_mm", [(1.32, 1.21), (3.0, 2.19)])
def test_relative_budget_jacobian_matches_the_registered_formula_when_diagonal(length_m, lat_mm):
    """Registration 6.1-6.3: sqrt(s_th0^2 + s_th1^2) and sqrt(s_y0^2 + s_y1^2 + (|dp| s_th0)^2) are the special
    case of independent, diagonal covariances (design 3.3 10, 9). Camera numbers of 6.2 (1)(3)."""
    s_th, s_y, s_x = 0.68e-3, 0.57e-3, 2.0e-3  # s_x differs on purpose: along the motion it must not leak in
    c0, c1 = _diag(s_x, s_y, s_th), _diag(s_x, s_y, s_th)
    r = ma.relative_pose((0.0, 0.0, 0.0), (length_m, 0.0, 0.004), c0, c1)
    lat = math.sqrt(s_y**2 + s_y**2 + (length_m * s_th) ** 2)
    assert r["sigma_lat"] == pytest.approx(lat, abs=1e-12) and r["sigma_lat"] == pytest.approx(lat_mm * 1e-3,
                                                                                               abs=5e-6)
    assert r["sigma_dyaw"] == pytest.approx(math.sqrt(2) * s_th, abs=1e-12)
    assert r["sigma_fwd"] == pytest.approx(math.sqrt(2) * s_x, abs=1e-12)
    assert r["cov"].shape == (3, 3) and np.allclose(r["cov"], r["cov"].T, rtol=0, atol=1e-20)
    # stop-to-moving heading (6.2): 0.68 / 0.74 mrad -> 1.005 mrad, over 1.0
    mixed = ma.relative_pose((0, 0, 0), (length_m, 0, 0), _diag(s_x, s_y, 0.68e-3), _diag(s_x, s_y, 0.74e-3))
    assert mixed["sigma_dyaw"] == pytest.approx(1.005e-3, abs=5e-7) and mixed["sigma_dyaw"] > 1.0e-3


def test_relative_budget_jacobian_from_a_rotated_start_with_round_position_errors():
    th0, length = 0.7, 2.4
    p0 = (0.3, -1.1, th0)
    p1 = (p0[0] + length * math.cos(th0), p0[1] + length * math.sin(th0), th0 - 0.05)
    c0, c1 = _diag(0.9e-3, 0.9e-3, 0.5e-3), _diag(0.4e-3, 0.4e-3, 0.3e-3)
    r = ma.relative_pose(p0, p1, c0, c1)
    assert r["sigma_lat"] == pytest.approx(math.sqrt(0.9e-3**2 + 0.4e-3**2 + (length * 0.5e-3) ** 2), abs=1e-12)
    assert r["sigma_dyaw"] == pytest.approx(math.hypot(0.5e-3, 0.3e-3), abs=1e-12)


def test_a_correlated_covariance_is_not_the_diagonal_formula():
    s_th, s_y, s_x, L = 0.68e-3, 0.57e-3, 0.57e-3, 1.32
    c0 = _diag(s_x, s_y, s_th)
    c0[1, 2] = c0[2, 1] = 0.6 * s_y * s_th  # lateral and heading errors of the start pose move together
    r = ma.relative_pose((0, 0, 0), (L, 0, 0), c0, _diag(s_x, s_y, s_th))
    formula = math.sqrt(2 * s_y**2 + (L * s_th) ** 2)
    assert abs(r["sigma_lat"] - formula) > 1e-5
    # the cross term: d(dy)/dy0 = -1, d(dy)/dth0 = -L, so var gains 2 * L * cov(y0, th0)
    assert r["sigma_lat"] == pytest.approx(math.sqrt(formula**2 + 2 * L * c0[1, 2]), abs=1e-12)


def test_the_propagated_covariance_is_the_numerical_jacobians():
    rng = np.random.default_rng(5)
    p0, p1 = np.array([0.4, -0.2, 2.9]), np.array([-1.3, 0.8, -2.8])
    a, b = rng.normal(size=(3, 3)) * 1e-3, rng.normal(size=(3, 3)) * 1e-3
    c0, c1 = a @ a.T, b @ b.T

    def f(q0, q1):
        r = ma.relative_pose(q0, q1)
        return np.array([r["dx_fwd"], r["dy_lat"], r["dyaw"]])

    h = 1e-7
    j0 = np.column_stack([(f(p0 + h * e, p1) - f(p0 - h * e, p1)) / (2 * h) for e in np.eye(3)])
    j1 = np.column_stack([(f(p0, p1 + h * e) - f(p0, p1 - h * e)) / (2 * h) for e in np.eye(3)])
    expected = j0 @ c0 @ j0.T + j1 @ c1 @ j1.T
    assert np.allclose(ma.relative_pose(p0, p1, c0, c1)["cov"], expected, rtol=1e-6, atol=1e-15)


@pytest.mark.parametrize("cov0, cov1", [(np.eye(3), None), (None, np.eye(3)), (np.eye(2), np.eye(3)),
                                        (np.eye(3), np.full((3, 3), np.nan)),
                                        (np.eye(3), np.array([[1.0, 0.5, 0], [0, 1, 0], [0, 0, 1]])),
                                        (np.eye(3), -np.eye(3))])
def test_relative_pose_refuses_a_bad_covariance(cov0, cov1):
    with pytest.raises(ValueError):
        ma.relative_pose((0, 0, 0), (1, 0, 0), cov0, cov1)


def test_relative_pose_refuses_a_bad_pose():
    with pytest.raises(ValueError):
        ma.relative_pose((0, 0), (1, 0, 0))
    with pytest.raises(ValueError):
        ma.relative_pose((0, 0, float("nan")), (1, 0, 0))


# --- (나) allan_dev_tau --------------------------------------------------------------------------------------------

def _jittered(duration_s, rate_hz=400.8, jitter=0.15, seed=0):
    rng = np.random.default_rng(seed)
    dt = 1.0 / rate_hz
    n = int(duration_s * rate_hz) + 1
    steps = dt * (1 + rng.uniform(-jitter, jitter, n - 1))
    t_ns = (2 * NS + np.round(np.concatenate([[0.0], np.cumsum(steps)]) * NS)).astype(np.int64)
    return t_ns, rng


def test_allan_irregular_t_white_rate_noise_gives_its_density_at_one_second():
    # the estimate's own spread at tau = 1 s is about 3.4 % over 300 s (12 seeds: 0.97-1.07, mean 1.009) and 1.7 %
    # over 1200 s, so 1200 s puts the 5 % tolerance near 3 sigma instead of 1.5
    n_density = 2.054e-4  # rad / sqrt(s), the desk N
    t, rng = _jittered(1200.0, seed=1)
    rate = 1.2e-3 + rng.normal(0.0, n_density * math.sqrt(400.8), len(t))  # a constant bias changes nothing
    assert ma.allan_dev_tau(t, rate, 1.0) == pytest.approx(n_density, rel=0.05)


def test_allan_uses_the_actual_times():
    # a rate ramp a * t: trapezoid-exact angle, Allan deviation a * tau / sqrt(2) whatever the sampling
    t, _ = _jittered(60.0, jitter=0.4, seed=2)
    a = 3e-4
    rate = a * (t - t[0]) / NS
    assert ma.allan_dev_tau(t, rate, 1.0) == pytest.approx(a / math.sqrt(2), rel=1e-6)
    assert ma.allan_dev_tau(t, rate, 2.5) == pytest.approx(a * 2.5 / math.sqrt(2), rel=1e-6)
    assert ma.allan_dev_tau(t, np.full(len(t), 0.02), 1.0) == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize("duration_s", [0.5, 1.9])
def test_allan_refuses_a_record_shorter_than_two_tau(duration_s):
    t, rng = _jittered(duration_s)
    with pytest.raises(ValueError):
        ma.allan_dev_tau(t, rng.normal(size=len(t)), 1.0)


def test_allan_refuses_bad_input():
    t, rng = _jittered(10.0)
    w = rng.normal(size=len(t))
    for bad_t, bad_w, tau in [(t[::-1], w, 1.0), (t, w[:-1], 1.0), (t, w, 0.0), (t, w, -1.0),
                              (t, np.where(np.arange(len(t)) == 7, np.nan, w), 1.0), (t[:1], w[:1], 1.0),
                              (t, np.outer(w, UP), 1.0)]:
        with pytest.raises(ValueError):
            ma.allan_dev_tau(bad_t, bad_w, tau)


# --- (나) quiet_window: a window is one contiguous run --------------------------------------------------------------

def _no_gap(t, s, e, gap_ns=10_000_000):
    return np.diff(t[s:e]).max() <= gap_ns


def test_quiet_window_never_spans_a_5_s_gap_in_the_samples():
    t, w, a = imu_series(0.0, 25.0, 200, lambda x: 0.0, up=UP, bias=BIAS, noise=0.001, accel_noise=0.005, seed=8)
    keep = (t < ns(15.0)) | (t >= ns(20.0))  # 5 s without IMU samples while the truck stood
    t, w, a = t[keep], w[keep], a[keep]
    kw = {"boundary_ns": ns(25.0), "side": "before", "max_s": 25.0, "standing": None}
    s, e = ma.quiet_window(t, w, a, min_s=2.0, **kw)
    assert t[s] >= ns(20.0) and _no_gap(t, s, e)
    s, e = ma.quiet_window(t, w, a, min_s=6.0, **kw)  # the 5 s after the gap is too short: the run before it
    assert t[e - 1] < ns(15.0) and (t[e - 1] - t[s]) / NS >= 6.0 and _no_gap(t, s, e)
    assert ma.quiet_window(t, w, a, min_s=16.0, **kw) is None  # 0-25 s across the gap would have been 25 s
    s, e = ma.quiet_window(t, w, a, boundary_ns=ns(0.0), side="after", max_s=25.0, min_s=2.0, standing=None)
    assert t[e - 1] < ns(15.0) and _no_gap(t, s, e)


def test_quiet_window_never_spans_a_5_s_standing_gap():
    # the IMU is quiet throughout (a steady straight drive looks quiet); the encoders say it drove 8-13 s
    t, w, a = imu_series(0.0, 25.0, 200, lambda x: 0.0, up=UP, bias=BIAS, noise=0.001, accel_noise=0.005, seed=9)
    standing = [(0, ns(8.0)), (ns(13.0), ns(25.0))]
    kw = {"boundary_ns": ns(25.0), "side": "before", "max_s": 25.0, "standing": standing}
    s, e = ma.quiet_window(t, w, a, min_s=2.0, **kw)
    assert t[s] >= ns(13.0) and t[e - 1] <= ns(25.0)
    s, e = ma.quiet_window(t, w, a, min_s=10.0, **kw)
    assert t[s] >= ns(13.0)
    assert ma.quiet_window(t, w, a, min_s=13.0, **kw) is None  # no one standing stretch is 13 s long
    s, e = ma.quiet_window(t, w, a, boundary_ns=0, side="after", max_s=25.0, min_s=2.0, standing=standing)
    assert t[e - 1] <= ns(8.0)


def test_quiet_window_gap_threshold_is_a_keyword():
    t, w, a = imu_series(0.0, 20.0, 200, lambda x: 0.0, up=UP, bias=BIAS, noise=0.001, accel_noise=0.005, seed=10)
    gap20 = (t < ns(12.0)) | (t >= ns(12.015))  # three samples lost: a 20 ms gap
    one = t != ns(6.0)  # one sample lost: a 10 ms gap, not over the default
    kw = {"boundary_ns": ns(20.0), "side": "before", "max_s": 20.0, "min_s": 2.0, "standing": None}
    s, e = ma.quiet_window(t[gap20], w[gap20], a[gap20], **kw)
    assert t[gap20][s] >= ns(12.0)
    s, e = ma.quiet_window(t[gap20], w[gap20], a[gap20], gap_max_ns=50_000_000, **kw)
    assert t[gap20][s] == 0
    s, e = ma.quiet_window(t[one], w[one], a[one], **kw)
    assert t[one][s] == 0
    for bad in (0, -1):
        with pytest.raises(ValueError):
            ma.quiet_window(t, w, a, gap_max_ns=bad, **kw)
