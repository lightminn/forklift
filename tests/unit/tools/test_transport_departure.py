"""Baseline priority 3 transport departure classification
(docs/validation/2026-11-14-transport-departure/transport_departure.py) on synthetic records."""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
_spec = importlib.util.spec_from_file_location(
    "transport_departure", ROOT / "docs/validation/2026-11-14-transport-departure/transport_departure.py")
td = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(td)


def straight(n=41, step=0.05, kappa=0.0, cusp_at=None):
    poses = [[i * step, 0.0, 0.0] for i in range(n)]
    directions = [1] * n
    if cusp_at is not None:  # back over the same line after the cusp
        for i in range(cusp_at + 1, n):
            poses[i] = [(2 * cusp_at - i) * step, 0.0, 0.0]
            directions[i] = -1
    return {"poses": poses, "directions": directions, "curvatures_inv_m": [kappa] * n}


# ---- the ordered table ---------------------------------------------------------------------------------------------


@pytest.mark.parametrize("S, E0, P, M, label", [
    (None, True, True, True, "unresolved"),
    (False, True, None, None, "unsaturated_estimate_onset_marker"),
    (False, False, None, None, "tracking_unsaturated"),
    (False, None, None, None, "unresolved"),
    (True, None, None, True, "unresolved_saturated_no_curvature"),
    (True, False, True, True, "mixed_plan_margin_and_plant"),
    (True, False, True, False, "plant_shortfall"),
    (True, False, False, True, "plan_margin"),
    (True, False, False, False, "unresolved_saturated_margins_ok"),
])
def test_every_combination_has_one_label(S, E0, P, M, label):
    assert td.primary_cross(S, E0, P, M, False) == label


def test_an_unknown_path_is_unresolved_first():
    assert td.primary_cross(True, True, True, True, True) == "unresolved"


# ---- geometry ------------------------------------------------------------------------------------------------------


def test_the_segment_distance_clips_the_fraction():
    p = td.Path2(straight())
    d, s = p.on_segment([3.0, 0.1, 0.0], 39)  # beyond the last segment's end: the end point, not the line
    assert d == pytest.approx(math.hypot(3.0 - 2.0, 0.1)) and s == pytest.approx(2.0)


def test_the_forward_walk_never_goes_back():
    p = td.Path2(straight())
    walk = p.forward([[0.52, 0.02, 0.0], [0.30, 0.02, 0.0]], 0, 40)
    assert walk[0][2] == 10 and walk[1][2] == 10  # stays on segment 10 when the pose falls behind


def test_cusps_and_legs():
    p = td.Path2(straight(cusp_at=20))
    assert p.cusps == [20] and p.leg_range(0) == (0, 20) and p.leg_range(1) == (20, 40)


def test_a_far_part_of_the_path_as_close_is_ambiguous():
    path = straight(n=41)
    for i in range(21, 41):  # a hairpin: the second half runs back 0.004 m beside the first
        path["poses"][i] = [(40 - i) * 0.05, 0.004, math.pi]
    p = td.Path2(path)
    assert p.near_window([0.6, 0.002, 0.0], 0.6, 0, 40) is None  # 0.6 and its neighbour 1.4 m of path on
    assert p.near_window([0.6, 0.002, 0.0], 0.6, 0, 20) is not None  # inside one leg: unique


def test_wheel_curvatures_inner_and_outer():
    # rv3's saturated left turn: the left (inner) wheel at 0.2233 implies 0.315 /m, the right (outer) at 0.2133 0.36 /m
    kl, kr = td.wheel_curvatures([0.2233, 0.2133])
    assert kl == pytest.approx(0.3154, abs=5e-4) and kr == pytest.approx(0.3595, abs=5e-4)
    # an ideal Ackermann pair gives one curvature
    R = 1 / 0.36
    kl, kr = td.wheel_curvatures([math.atan(0.66 / (R - 0.265)), math.atan(0.66 / (R + 0.265))])
    assert kl == pytest.approx(0.36) and kr == pytest.approx(0.36)


# ---- a synthetic cross_track ---------------------------------------------------------------------------------------


def synthetic_cross(truth_lateral_end=0.30, est_extra=0.06, saturated=True, kappa_plan=0.29, achieved=0.30):
    """A leg along x; the truth drifts left linearly to truth_lateral_end, the estimate est_extra further."""
    path = straight(n=81, step=0.05, kappa=kappa_plan)
    t0, dt, n = 10.0, 0.1, 30
    rows, samples = [], []
    for k in range(n + 1):
        t = t0 + k * dt
        x = 0.1 + k * 0.1
        lat = truth_lateral_end * k / n
        yaw_t = achieved * x  # the truth's heading grows at the achieved curvature per metre
        for sub in range(12):  # control ticks between samples (only the sample rows matter)
            ts = t + sub * dt / 12
            if k == n and sub:
                break
            rows.append([ts, x, lat + est_extra * k / n, yaw_t, x, lat, yaw_t])
        if k < n:
            samples.append({"phase": "transport", "time_s": t, "signed_speed_mps": 0.3,
                            "curvature_inv_m": 0.36 if saturated else 0.1,
                            "steering_command_rad": [0.2569, 0.2136], "steering_actual_rad": [0.223, 0.2133],
                            "tracking": {"segment_index": min(79, int(x / 0.05))}})
    t_fail = t0 + n * dt
    x_end = 0.1 + n * 0.1
    est = [x_end, truth_lateral_end + est_extra, achieved * x_end]
    result = {"phase": "transport", "failure_reason": "Tracking failed in transport: pos=1.0,yaw=0.1",
              "transitions": [{"to": "transport", "time_s": 5.0}], "leg_switches": [], "samples": samples,
              "tracking_failure": {"reason": "failed", "time_s": t_fail, "phase_started_s": 5.0, "leg": 0,
                                   "rear_pose": est, "path": path, "config": {"max_cross_track_error_m": 0.35},
                                   "tracking": {"failure": "cross_track", "segment_index": min(79, int(x_end / 0.05)),
                                                "yaw_error_rad": 0.1, "at_cusp": False}}}
    return result, np.asarray(rows)


def test_a_saturated_drift_judged_early_by_the_estimate():
    r, sc = synthetic_cross()
    out = td.classify_run("lbg_B0_random_1_s0", r, sc)
    assert out["kind"] == "cross_track"
    assert out["dE"] == pytest.approx(0.36, abs=1e-3) and out["dT"] == pytest.approx(0.30, abs=1e-3)
    assert out["D"] is True and out["sat"]["S"] is True
    assert out["sat"]["ach_minus_plan"] == pytest.approx(0.01, abs=0.005)  # the achieved 0.30 against the plan 0.29
    assert out["sat"]["M"] is True
    assert out["primary"] in ("plan_margin", "mixed_plan_margin_and_plant")


def test_an_unsaturated_drift_is_tracking():
    r, sc = synthetic_cross(saturated=False, est_extra=0.0, truth_lateral_end=0.36)
    out = td.classify_run("x_B0_random_1_s0", r, sc)
    assert out["sat"]["S"] is False and out["D"] is False
    assert out["primary"] == "tracking_unsaturated"


def test_a_path_installed_before_the_failure_starts_the_window_there():
    """tracking_failure.path is the path installed last: its leg 0 starts at that install, not at transport."""
    r, sc = synthetic_cross()
    r["cusp_replans"] = [{"time_s": 11.0}]
    out = td.classify_run("x_B0_random_1_s0", r, sc)
    assert out["leg_start_s"] == 11.0 and out["installs_inside_window"] == []


def test_identical_failures_merge_and_different_ones_do_not():
    r, sc = synthetic_cross()
    a = td.classify_run("lbg_B0_random_1_s0", r, sc)
    b = td.classify_run("k1c_K1_random_1_s0", r, sc)
    r2, sc2 = synthetic_cross(est_extra=0.07)
    c = td.classify_run("k1c_B0_random_1_s1", r2, sc2)
    cases = td.merge([a, b, c])
    assert sorted(len(x["runs"]) for x in cases) == [1, 2]


def test_a_timeout_stalled_at_the_end_is_an_endpoint_stall():
    r = {"phase": "transport", "failure_reason": "Tracking timeout in transport",
         "samples": [{"phase": "transport", "time_s": 1.0, "speed_command_mps": 0.0,
                      "tracking": {"status": "tracking", "position_error_m": 0.02, "yaw_error_rad": 0.07,
                                   "at_cusp": True, "segment_index": 5}}]}
    assert td.classify_run("x_B0_aisle_rack_s1", r, np.zeros((1, 7)))["primary"] == "endpoint_stall"
    r["samples"][0]["tracking"]["position_error_m"] = 0.5
    assert td.classify_run("x_B0_aisle_rack_s1", r, np.zeros((1, 7)))["primary"] == "separate_ending"
