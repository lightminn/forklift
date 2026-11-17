"""docs/plans/2026-11-18-f2-i-reregistration.md 6.1: the mechanism metric's contract on synthetic runs."""

import importlib.util
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
_spec = importlib.util.spec_from_file_location(
    "f2i_mechanism", ROOT / "docs/validation/2026-11-18-f2-i-reregistration/mechanism.py")
M = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(M)

DT = 0.1


def arc(t0, n, speed, kcmd, kach, cmd=(0.30, 0.25), act=(0.28, 0.25), x0=0.0, y0=0.0, yaw0=0.0):
    """n transport samples and slam_control rows along an arc of curvature kach at speed (true pose)."""
    samples, rows = [], []
    x, y, yaw = x0, y0, yaw0
    for i in range(n + 1):
        t = t0 + i * DT
        samples.append({"time_s": t, "phase": "transport", "signed_speed_mps": speed, "curvature_inv_m": kcmd,
                        "steering_command_rad": list(cmd), "steering_actual_rad": list(act)})
        rows.append([t, x, y, yaw, x, y, yaw])
        ds = speed * DT
        x += ds * math.cos(yaw)
        y += ds * math.sin(yaw)
        yaw += kach * ds
    return samples, rows


def run(*parts, extra=None):
    samples, rows = [], []
    for s, r in parts:
        samples += s
        rows += r
    r = {"samples": samples, "transitions": [{"to": "transport", "time_s": samples[0]["time_s"]}]}
    r.update(extra or {})
    return r, np.asarray(rows, float)


def test_ratio_and_absolute_wheel_error_per_stratum():
    r, sc = run(arc(0.0, 30, 0.3, 0.36, 0.32))
    out = M.strata(r, sc)
    fl = out["fwd_left_slow"]
    assert fl["valid"]
    assert abs(fl["ratio"] - 0.32 / 0.36) < 1e-3
    assert abs(fl["mabs"][0] - 0.02) < 1e-9 and abs(fl["mabs"][1]) < 1e-9


def test_absolute_error_does_not_cancel_signs():
    a, ra = arc(0.0, 20, 0.3, 0.36, 0.32, cmd=(0.30, 0.25), act=(0.28, 0.25))
    b, rb = arc(2.1, 20, 0.3, 0.36, 0.32, cmd=(0.30, 0.25), act=(0.32, 0.25), x0=ra[-1][4], y0=ra[-1][5],
                yaw0=ra[-1][6])
    r, sc = run((a, ra), (b, rb))
    fl = M.strata(r, sc)["fwd_left_slow"]
    assert abs(fl["mabs"][0] - 0.02) < 1e-6  # signed mean would be 0


def test_composition_change_alone_does_not_move_the_pair_metric():
    # left turns achieve 0.85, right turns 1.00 in both arms; only the share of time changes
    def arm(n_left, n_right):
        left, rl = arc(0.0, n_left, 0.3, 0.36, 0.36 * 0.85)
        right, rr = arc(left[-1]["time_s"] + DT, n_right, 0.3, -0.36, -0.36, x0=rl[-1][4], y0=rl[-1][5],
                        yaw0=rl[-1][6])
        return M.strata(*run((left, rl), (right, rr)))
    o, x = arm(90, 10), arm(10, 90)
    pair = M.pair_metric(o, x)
    assert abs(pair["R_o"] - pair["R_x"]) < 1e-3
    assert sorted(pair["common"]) == ["fwd_left_slow", "fwd_right_slow"]


def test_samples_across_a_path_install_or_unpaired_rows_are_excluded():
    a, ra = arc(0.0, 20, 0.3, 0.36, 0.32)
    r, sc = run((a, ra), extra={"cusp_replans": [{"time_s": 1.05}]})
    out = M.strata(r, sc)
    assert out["excluded_s"]["path_install"] > 0
    sc2 = sc.copy()
    sc2[5, 0] += 0.01  # one row no longer pairs with its sample
    out2 = M.strata(run((a, ra))[0], sc2)
    assert out2["excluded_s"]["unpaired_row"] > 0


def test_a_stratum_with_mostly_disagreeing_signs_is_invalid():
    good, rg = arc(0.0, 6, 0.3, 0.36, 0.32)  # 0.6 s agreeing
    bad, rb = arc(0.7, 20, 0.3, 0.36, -0.10, x0=rg[-1][4], y0=rg[-1][5], yaw0=rg[-1][6])  # 2.0 s disagreeing
    out = M.strata(*run((good, rg), (bad, rb)))
    assert not out["fwd_left_slow"]["valid"]


def test_slow_or_unsaturated_samples_do_not_count():
    a, ra = arc(0.0, 20, 0.03, 0.36, 0.32)
    b, rb = arc(2.1, 20, 0.3, 0.20, 0.20, x0=ra[-1][4], y0=ra[-1][5], yaw0=ra[-1][6])
    out = M.strata(*run((a, ra), (b, rb)))
    assert out["fwd_left_slow"]["saturated_s"] == 0


def test_reverse_is_its_own_stratum():
    a, ra = arc(0.0, 20, -0.3, 0.36, 0.30)
    out = M.strata(*run((a, ra)))
    assert out["rev_left_slow"]["valid"] and out["fwd_left_slow"]["saturated_s"] == 0


def test_rule_needs_six_pairs_median_and_majority():
    pairs = [{"R_o": 0.88, "R_x": 0.97, "A_o": 0.02, "A_x": 0.005}] * 6
    v = M.judge(pairs)
    assert v["mechanism"] == "fixes" and v["asymmetry"] == "reduces"
    assert M.judge(pairs[:5])["mechanism"] == "undecided"
    worse = pairs[:4] + [{"R_o": 0.97, "R_x": 0.90, "A_o": 0.01, "A_x": 0.02}] * 2
    assert M.judge(worse)["mechanism"] == "does_not_fix"


def test_speed_bins_keep_slow_and_fast_apart():
    slow, rs = arc(0.0, 20, 0.3, 0.36, 0.36 * 0.98)
    fast, rf = arc(2.1, 20, 0.6, 0.36, 0.36 * 0.85, x0=rs[-1][4], y0=rs[-1][5], yaw0=rs[-1][6])
    out = M.strata(*run((slow, rs), (fast, rf)))
    assert abs(out["fwd_left_slow"]["ratio"] - 0.98) < 1e-3 and abs(out["fwd_left_fast"]["ratio"] - 0.85) < 1e-3


def test_a_stratum_the_candidate_loses_counts_against_it():
    good, rg = arc(0.0, 20, 0.3, 0.36, 0.30)
    o = M.strata(*run((good, rg)))
    bad, rb = arc(0.0, 20, 0.3, 0.36, -0.05)  # saturated, signs disagree: the stratum is lost in X
    x = M.strata(*run((bad, rb)))
    p = M.pair_metric(o, x)
    assert p["lost"] == ["fwd_left_slow"]
    v = M.judge([{**p, "R_o": 0.9, "R_x": 0.99, "A_o": 0.02, "A_x": 0.0}] * 6)
    assert v["mechanism"] == "does_not_fix" and v["asymmetry"] == "does_not_reduce"
