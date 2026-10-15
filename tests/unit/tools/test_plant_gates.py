"""Angle-based V2/V3 gate measures (fork spacing plan, 13th-draft gates revised 2026-10-15)."""

import importlib.util
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("plant_gates", ROOT / "tools/plant_gates.py")
G = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(G)
DT = 1 / 120


def test_a_steady_hold_is_one_window_and_too_short_is_none():
    q = np.full((120, 2), 0.2)
    assert G.steady_span(q, q.copy(), DT) == (0, 120)
    assert G.steady_span(q[:30], q[:30].copy(), DT) is None  # 29 pairs = 0.24 s < 0.3 s


def test_a_tick_to_tick_alternation_is_not_steady():
    """Codex (E results P2): +-0.05 mrad alternating each tick -- a central difference reads 0; the
    adjacent rate 0.012 rad/s passes the 0.02 limit but the error slope 0.012 > 0.005 does not."""
    q = 0.2 + np.tile([[0.00005], [-0.00005]], (60, 2))
    target = np.full_like(q, 0.2)
    assert np.allclose(np.gradient(q[:, 0] - target[:, 0], DT)[1:-1], 0.0)
    assert G.steady_span(q, target, DT) is None


def test_a_slow_drift_inside_the_limits_is_steady_and_a_faster_one_is_not():
    t = np.arange(120)[:, None] * DT
    slow = 0.2 + 0.004 * t * np.ones((1, 2))
    assert G.steady_span(slow, np.full_like(slow, 0.2), DT) == (0, 120)
    fast = 0.2 + 0.03 * t * np.ones((1, 2))
    assert G.steady_span(fast, fast.copy(), DT) is None  # error 0, but |dq|/dt 0.03 > 0.02


def test_the_window_metrics_read_curvature_residual_and_the_angle_rolling_ratio():
    cols = ("rear_x", "rear_y", "rear_yaw", "q_FL", "q_FR", "target_FL", "target_FR", "p_RL", "p_RR")
    c = {k: i for i, k in enumerate(cols)}
    n, kappa, speed, radius = 121, 0.3, -0.25, 0.125
    s = speed * np.arange(n) * DT
    rows = np.zeros((n, len(cols)))
    rows[:, c["rear_yaw"]] = kappa * s
    rows[:, c["rear_x"]] = np.sin(kappa * s) / kappa
    rows[:, c["rear_y"]] = (1 - np.cos(kappa * s)) / kappa
    wheelbase, track = 0.66, 0.53
    # Steering angles that imply exactly kappa at each wheel.
    q_l = math.atan(kappa * wheelbase / (1 - kappa * track / 2))
    q_r = math.atan(kappa * wheelbase / (1 + kappa * track / 2))
    rows[:, c["q_FL"]], rows[:, c["q_FR"]] = q_l, q_r
    rows[:, c["target_FL"]], rows[:, c["target_FR"]] = q_l - 0.001, q_r
    angle = s / (radius * 0.995)  # the wheels turn 0.5 % more than the axle travels
    rows[:, c["p_RL"]] = rows[:, c["p_RR"]] = np.mod(angle + np.pi, 2 * np.pi) - np.pi  # wrapped like the joints
    m = G.window_metrics(rows, c, DT, wheelbase_m=wheelbase, track_m=track, radius_m=radius, direction=-1.0)
    assert math.isclose(m["kappa_true"], kappa, rel_tol=1e-6)
    assert abs(m["residual"]) < 1e-6
    assert math.isclose(m["rolling_ratio"], 0.995, rel_tol=1e-4)
    assert math.isclose(m["err_FL_mrad"], 1.0, rel_tol=1e-6)


# --- 16th draft V2/V3 (fixed counterexamples from the plan block "16판") -----------------------------
COLS = ("rear_x", "rear_y", "rear_yaw", "q_FL", "q_FR", "target_FL", "target_FR", "p_RL", "p_RR")
C = {k: i for i, k in enumerate(COLS)}
L, T, R = 0.66, 0.53, 0.125


def steer_for(kappa):
    return (math.atan(kappa * L / (1 - kappa * T / 2)), math.atan(kappa * L / (1 + kappa * T / 2)))


def drive(kappa_ticks, speed=0.25, dt=DT, radius_scale=1.0):
    """Rows of a rear axle that follows the given curvature on each tick at a constant speed; rear wheel
    angles advance by travel / (R * radius_scale); steering angles imply the tick's curvature."""
    k = np.asarray(kappa_ticks, float)
    n = len(k) + 1
    rows = np.zeros((n, len(COLS)))
    x = y = yaw = 0.0
    ds = speed * dt
    for i, ki in enumerate(k):
        if abs(ki) < 1e-12:
            x, y = x + ds * math.cos(yaw), y + ds * math.sin(yaw)
        else:
            x, y = x + (math.sin(yaw + ki * ds) - math.sin(yaw)) / ki, y - (math.cos(yaw + ki * ds) - math.cos(yaw)) / ki
        yaw += ki * ds
        rows[i + 1, [C["rear_x"], C["rear_y"], C["rear_yaw"]]] = x, y, yaw
    for i in range(n):
        ql, qr = steer_for(k[min(i, len(k) - 1)])
        rows[i, [C["q_FL"], C["q_FR"], C["target_FL"], C["target_FR"]]] = ql, qr, ql, qr
    travel = np.r_[0.0, np.cumsum(np.full(len(k), ds))]  # signed: negative when reversing
    angle = travel / (R * radius_scale)
    rows[:, C["p_RL"]] = rows[:, C["p_RR"]] = np.mod(angle + np.pi, 2 * np.pi) - np.pi
    return rows


def judge(rows, kappa_cmd=0.36, direction=1.0):
    """rows[0] is the row before the window."""
    return G.v16_judge(rows, C, DT, kappa_cmd=kappa_cmd, direction=direction, wheelbase_m=L, track_m=T)


def chatter(n, pattern):
    """(n, 2) steering offsets repeating pattern (mrad) on both joints."""
    return np.resize(np.asarray(pattern, float) * 1e-3, n)[:, None] * np.ones((1, 2))


def test_v16_a_steady_arc_gives_a_whole_window_that_passes():
    rows = drive([0.36] * 121)
    span = G.v16_window(rows[1:, [C["q_FL"], C["q_FR"]]], DT)
    assert span == (0, 120)  # 10 whole blocks; the 121st row is an incomplete block and is dropped
    out = judge(rows[span[0]:span[1] + 1])  # with the row before the window
    assert out["pass"] and out["blocks"] == 10 and math.isclose(out["window_s"], 1.0)
    assert abs(out["residual"]) < 1e-7  # chord vs arc length per tick


def test_v16_an_opposite_sign_curvature_fails():
    rows = drive([-0.36] * 120)
    out = judge(rows, kappa_cmd=0.36)
    assert out["checks"]["kappa_min"] and out["checks"]["residual_mean"]
    assert not out["checks"]["same_sign"] and not out["pass"]


def test_v16_tick_alternation_inside_the_deviation_bound_keeps_the_window():
    """16.1: the deviation bound (3.1 mrad from the window mean) is the chatter limit -- a tick-to-tick
    jump is not limited by itself, so +-2 mrad (p-p 4) is accepted; +-3.2 mrad is not."""
    for amp in (0.05, 0.7, 2.0):
        for phase in (0, 1):
            q = 0.2 + chatter(120, np.roll([amp, -amp], phase))
            assert G.v16_window(q, DT) == (0, 120), (amp, phase)
    assert G.v16_window(0.2 + chatter(120, [3.2, -3.2]), DT) is None


def test_v16_boundary_chatter_is_judged_the_same_at_any_block_phase():
    """The 15th-draft review case: block A = [-0.258] * 11 + [2.838], B = [-2.838] + [0.258] * 11 --
    every block mean 0, a 5.676 mrad jump at the A/B boundary. Its largest deviation, 2.838 mrad, is
    inside the bound at phase 0 and at phase 6; scaled to 3.2 mrad it is outside at both."""
    def pattern(peak):
        small = peak / 11
        return np.r_[[-small] * 11, [peak], [-peak], [small] * 11]
    for peak, window in ((2.838, (0, 120)), (3.2, None)):
        for phase in (0, 6):
            q = 0.2 + chatter(120, np.roll(pattern(peak), phase))
            assert G.v16_window(q, DT) == window, (peak, phase)


def test_v16_block_means_swinging_and_steps_have_no_window():
    swing = np.repeat(np.resize([0.735, -0.735], 10), 12) * 1e-3
    assert G.v16_window(0.2 + swing[:, None] * np.ones((1, 2)), DT) is None  # neighbour change 1.47 mrad
    for step in (0.49, 0.6):
        stairs = np.repeat(np.arange(40) * step, 12) * 1e-3
        q = 0.2 + stairs[:, None] * np.ones((1, 2))
        assert G.v16_window(q, DT) is None, step  # 0.49: slope 4.9 mrad/s > 1; 0.6: neighbour > 0.5


def test_v16_only_the_last_three_blocks_steady_has_no_window():
    ramp = np.r_[np.linspace(0.0, 0.02, 84), np.full(36, 0.02)]
    q = 0.2 + ramp[:, None] * np.ones((1, 2))
    assert G.v16_window(q, DT) is None


def test_v16_opposite_left_right_chatter_on_a_static_split_fails_per_sample():
    rows = drive([0.36] * 120)
    alt = np.resize([1.54e-3, -1.54e-3], 120)
    rows[1:, C["q_FL"]] += 4.9e-3 + alt
    rows[1:, C["q_FR"]] += -4.9e-3 - alt
    assert G.v16_window(rows[1:, [C["q_FL"], C["q_FR"]]], DT) == (0, 120)
    out = judge(rows)
    assert out["checks"]["lr_mean"]  # mean difference 9.8 mrad
    assert not out["checks"]["lr_samples"] and not out["pass"]  # 12.88 mrad on alternate ticks
    assert math.isclose(out["lr_sample_max_mrad"], 12.88, rel_tol=1e-9)


def test_v16_block_body_curvature_alternating_around_the_angles_fails():
    body = np.resize(np.repeat([0.34, 0.38], 12), 120)
    rows = drive(body)
    ql, qr = steer_for(0.36)
    rows[:, [C["q_FL"], C["target_FL"]]] = ql
    rows[:, [C["q_FR"], C["target_FR"]]] = qr
    out = judge(rows)
    assert out["checks"]["residual_mean"]
    assert not out["checks"]["residual_blocks"] and not out["pass"]


def test_v16_every_tick_belongs_to_one_block():
    """Codex (16th-draft review, P2): 12-row blocks read only 11 ticks each and miss the boundary tick.
    Boundary ticks at 0.24 and 0.48 1/m cancel in the window mean; with the boundary tick in its block
    the block residuals are -0.010 and +0.010."""
    body = np.full(48, 0.36)
    body[12], body[24] = 0.24, 0.48
    rows = drive(body)
    ql, qr = steer_for(0.36)
    rows[:, [C["q_FL"], C["target_FL"]]] = ql
    rows[:, [C["q_FR"], C["target_FR"]]] = qr
    out = judge(rows)
    assert out["checks"]["residual_mean"] and not out["checks"]["residual_blocks"]
    assert np.allclose(out["block_residuals"], [0.0, -0.01, 0.01, 0.0], atol=1e-6)


def test_v16_straight_window_rolling_ratio_and_calibration():
    rows = drive([0.0] * 240, radius_scale=0.996)
    span = G.v16_straight_window(rows, C, DT)
    assert span == (0, 241)
    w = rows[span[0]:span[1]]
    r_cal = G.v16_rolling(w, C, 1.0)["ratio"]
    assert math.isclose(r_cal, R * 0.996, rel_tol=1e-9)
    assert math.isclose(G.v16_rolling(w, C, r_cal)["ratio"], 1.0, rel_tol=1e-12)
    back = drive([0.0] * 240, speed=-0.25, radius_scale=0.996)
    assert math.isclose(G.v16_rolling(back, C, r_cal)["ratio"], 1.0, rel_tol=1e-12)


def test_v16_straight_window_excludes_acceleration_reversal_turns_and_short_runs():
    rows = drive([0.0] * 240)
    # an acceleration over the first 60 ticks: the hops grow, so speed max / min > 1.05 there
    hop = np.r_[np.linspace(0.2, 1.0, 60), np.ones(180)] * 0.25 * DT
    rows[1:, C["rear_x"]] = np.cumsum(hop)
    span = G.v16_straight_window(rows, C, DT)
    assert span[0] > 40 and span[1] == 241
    assert np.ptp(np.diff(rows[span[0]:span[1], C["rear_x"]])) <= 0.05 * hop.max()
    # a steering angle off straight by 3.2 mrad splits the run; each half alone is 0.25 m
    rows = drive([0.0] * 240)
    rows[120, C["q_FL"]] = 0.0032
    span = G.v16_straight_window(rows, C, DT)
    assert span is not None and (span[1] <= 120 or span[0] >= 120)
    # a wheel that stops for one tick breaks the one-way run
    rows = drive([0.0] * 240)
    rows[121:, C["p_RR"]] = rows[120:-1, C["p_RR"]]
    span = G.v16_straight_window(rows, C, DT)
    assert span is not None and (span[1] <= 121 or span[0] >= 120)
    # a heading that drifts by more than 5 mrad
    rows = drive([0.02] * 240)  # 0.5 m at 0.02 1/m = 10 mrad of heading
    rows[:, [C["q_FL"], C["q_FR"]]] = 0.0
    span = G.v16_straight_window(rows, C, DT)
    yaw = rows[span[0]:span[1], C["rear_yaw"]]
    assert np.ptp(yaw) <= 0.005 + 1e-12 and span[1] - span[0] < 200
    rows = drive([0.03] * 240)  # 5 mrad is 0.167 m at 0.03 1/m: under 0.20 m
    rows[:, [C["q_FL"], C["q_FR"]]] = 0.0
    assert G.v16_straight_window(rows, C, DT) is None
    # under 0.20 m of travel: none
    assert G.v16_straight_window(drive([0.0] * 90), C, DT) is None  # 0.1875 m


def test_v16_mission_rows_mark_loaded_rows_from_the_samples(tmp_path):
    """Rows are loaded only where every 0.1 s sample within 0.1 s is in transport with lift >= 0.15 m and
    the pallet >= 0.10 m above its start; the rear axle sits rear_axle_x behind base_link."""
    import json as json_
    n, dt = 1200, DT
    stamps = (np.arange(n) + 1) * dt
    yaw = 0.3
    base = np.zeros((n, 7))
    base[:, 0] = 1.0 + 0.25 * stamps * math.cos(yaw)
    base[:, 1] = 2.0 + 0.25 * stamps * math.sin(yaw)
    base[:, 3], base[:, 6] = math.cos(yaw / 2), math.sin(yaw / 2)  # wxyz
    angles = np.zeros((n, 4))
    angles[:, 2] = angles[:, 3] = 0.25 * stamps / R
    np.savez(tmp_path / "slam_log.npz", joint_stamps_s=stamps, base_pose_world=base.astype(np.float32),
             steering_rad=np.zeros((n, 2)), wheel_angles_rad=angles)
    samples = []
    for k in range(100):
        t = k * 0.1
        loaded = 2.0 <= t < 7.0
        samples.append({"time_s": t, "phase": "transport" if loaded or t >= 8.0 else "extract",
                        "lift_m": 0.17 if loaded else (0.10 if t >= 8.0 else 0.0),
                        "pallet_position_m": [0, 0, 0.12 if loaded or t >= 8.0 else 0.0]})
    (tmp_path / "result.json").write_text(json_.dumps({"samples": samples}))
    (tmp_path / "meta.json").write_text(json_.dumps({
        "odometry_geometry": {"wheelbase_m": L, "track_m": T, "wheel_radius_m": R, "rear_axle_x_in_base_m": -0.34},
        "steering_order": ["front_left", "front_right"],
        "wheel_order": ["front_left", "front_right", "rear_left", "rear_right"]}))
    _, rows, c, loaded, dt_, radius = G.mission_rows(tmp_path)
    assert math.isclose(dt_, DT, rel_tol=1e-9) and radius == R
    assert math.isclose(rows[0, c["rear_x"]], base[0, 0] - 0.34 * math.cos(yaw), abs_tol=1e-6)
    t_loaded = stamps[loaded]
    # the samples at 1.9 and 7.0 s are not loaded: rows within 0.1 s of them are not either
    assert 2.0 < t_loaded.min() < 2.02 and 6.88 < t_loaded.max() < 7.0
    assert np.all(np.diff(np.flatnonzero(loaded)) == 1)
    assert not loaded[stamps >= 7.0].any()  # lift 0.10 after 8 s is not loaded
    out = G.v16_loaded(tmp_path)
    assert out["reverse"] is None and abs(out["forward"]["lateral_m"]) < 1e-4  # carried into the report
    f = out["forward"]
    assert math.isclose(f["raw_ratio"], 1.0, rel_tol=1e-5) and f["travel_m"] > 1.0


def test_v16_rolling_reads_longitudinal_travel_tick_by_tick_under_sideslip():
    """Codex (16.1, P2): 2 s at 120 Hz, yaw swinging over 5 mrad, body-frame longitudinal 0.04 m/s and a
    lateral +-0.20 m/s following the yaw -- every window condition holds; the end-to-end displacement on
    the mean heading reads 0.0790 m against 0.0800 m of longitudinal travel."""
    n = 240
    t = np.arange(n + 1) * DT
    yaw = 0.0025 * np.sin(2 * np.pi * t / 2.0) / np.max(np.abs(np.sin(2 * np.pi * t / 2.0)))
    mid = (yaw[1:] + yaw[:-1]) / 2
    v_lon, v_lat = 0.04, 0.20 * np.sign(mid)  # sliding sideways the way the body is yawed
    dx = (v_lon * np.cos(mid) - v_lat * np.sin(mid)) * DT
    dy = (v_lon * np.sin(mid) + v_lat * np.cos(mid)) * DT
    rows = np.zeros((n + 1, len(COLS)))
    rows[1:, C["rear_x"]], rows[1:, C["rear_y"]] = np.cumsum(dx), np.cumsum(dy)
    rows[:, C["rear_yaw"]] = yaw
    rows[:, C["p_RL"]] = rows[:, C["p_RR"]] = v_lon * t / R
    out = G.v16_rolling(rows, C, R)
    assert math.isclose(out["travel_m"], 0.08, rel_tol=1e-9)
    assert math.isclose(out["ratio"], 1.0, rel_tol=1e-9)
    assert "lateral_m" in out
    h = float(np.mean(yaw))  # the replaced reading: end-to-end displacement on the mean heading
    old = (rows[-1, C["rear_x"]] * math.cos(h) + rows[-1, C["rear_y"]] * math.sin(h)) / 0.08
    assert abs(old - 1.0) > 0.005  # Codex: 0.0790 / 0.0800


def test_v16_a_single_tick_over_the_yaw_limit_is_not_a_window():
    """Codex (16.1, P3): two rows 0.20 m apart with 6 mrad of yaw were returned as (0, 2)."""
    rows = np.zeros((2, len(COLS)))
    rows[1, C["rear_x"]], rows[1, C["rear_yaw"]] = 0.20, 0.006
    rows[1, C["p_RL"]] = rows[1, C["p_RR"]] = 1.6
    assert G.v16_straight_window(rows, C, DT) is None


def test_v16_report_serialises_to_json(tmp_path):
    """Codex (16.1 re-review): a valid V3 ratio's pass flag was numpy.bool_ and json.dumps failed."""
    import json as json_
    out = G.v16_judge(drive([0.36] * 120), C, DT, kappa_cmd=0.36, direction=1.0, wheelbase_m=L, track_m=T)
    json_.dumps(out)
    eff = G.v16_rolling(drive([0.0] * 240), C, 1.0)
    json_.dumps(eff)
