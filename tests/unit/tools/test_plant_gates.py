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
