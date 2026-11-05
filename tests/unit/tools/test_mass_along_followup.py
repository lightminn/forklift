"""tools/mass_along_followup.py on synthetic records (docs/plans/2026-11-06-mass-along-followup-intervals-map.md)."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import mass_along_followup as F  # noqa: E402

L, W = 0.66, 0.53
RATE = 120.0


def ackermann(kappa):
    dl = np.arctan(kappa * L / (1 - kappa * W / 2))
    dr = np.arctan(kappa * L / (1 + kappa * W / 2))
    return dl, dr


def drive(kappa, v, gain=1.0, t_end=2.0):
    """A truth trajectory at speed v (signed) along curvature kappa(t), turning at gain x the kinematic rate."""
    t = np.arange(0.0, t_end, 1 / RATE)
    k = np.broadcast_to(np.asarray(kappa(t) if callable(kappa) else kappa, float), t.shape).copy()
    vv = np.broadcast_to(np.asarray(v(t) if callable(v) else v, float), t.shape).copy()
    ds = 0.5 * (vv[1:] + vv[:-1]) / RATE
    yaw = np.concatenate([[0.0], np.cumsum(gain * 0.5 * (k[1:] + k[:-1]) * ds)])
    mid = 0.5 * (yaw[1:] + yaw[:-1])
    x = np.concatenate([[0.0], np.cumsum(ds * np.cos(mid))])
    y = np.concatenate([[0.0], np.cumsum(ds * np.sin(mid))])
    control = np.column_stack([t, x, y, yaw, x, y, yaw])
    dl, dr = ackermann(k)
    log = {"joint_stamps_s": t, "steering_rad": np.column_stack([dl, dr]), "steering_measured_rad": np.column_stack([dl, dr])}
    return log, control


def windows(log, control, t_end=2.0):
    return F.transport_windows(log, control, (0.0, t_end), L, W)


def test_perfect_kinematics_give_zero_over_rotation_in_every_window():
    log, control = drive(lambda t: 0.3 + 0.1 * np.sin(t), 0.25)
    s2 = windows(log, control)
    inc = [w for w in s2["windows"] if w["included"]]
    assert len(inc) >= 6
    assert all(abs(w["over_rad"]) < 1e-9 for w in inc)


@pytest.mark.parametrize("kappa,v", [(0.5, 0.25), (-0.5, 0.25), (0.5, -0.25), (-0.5, -0.25)])
def test_over_rotation_is_positive_in_every_turn_and_direction(kappa, v):
    # gain 1.1: the truck turns 10 % more than the steering predicts, whichever way it turns or drives
    log, control = drive(kappa, v, gain=1.1)
    inc = [w for w in windows(log, control)["windows"] if w["included"]]
    assert inc and all(w["over_rad"] > 0 for w in inc)
    assert all(w["direction"] == ("fwd" if v > 0 else "rev") for w in inc)
    assert all(w["turn"] == ("left" if kappa * v > 0 else "right") for w in inc)
    # the signed residual has the sign of the rotation, the index does not
    assert all(math.copysign(1, w["residual_rad"]) == math.copysign(1, kappa * v) for w in inc)


def test_the_sign_follows_the_integrated_prediction_not_the_mean_curvature_times_net_motion():
    # Codex plan review 2: curvature and speed change together inside one window -- mean kappa > 0 and net motion > 0,
    # but most of the distance is rolled while kappa < 0, so the predicted rotation is negative
    def kappa(t):
        ph = np.mod(t, F.WINDOW_S) / F.WINDOW_S
        return np.where(ph < 0.3, 0.9, -0.2)

    def v(t):
        ph = np.mod(t, F.WINDOW_S) / F.WINDOW_S
        return np.where(ph < 0.3, 0.05, 0.4)

    log, control = drive(kappa, v, gain=0.8)
    ws = [w for w in windows(log, control)["windows"] if w["signed"]]
    flipped = [w for w in ws if w["kappa_mean"] > 0 and w["predicted_rad"] < 0]
    assert flipped
    # under-rotation (gain 0.8) reads negative with the integrated sign
    assert all(w["over_rad"] < 0 for w in flipped)


def test_the_partial_last_window_is_dropped_and_steps_belong_to_their_end_window():
    log, control = drive(0.4, 0.3, t_end=1.1)
    s2 = F.transport_windows(log, control, (0.0, 1.1), L, W)
    assert [round(w["t0"], 6) for w in s2["windows"]] == [0.0, 0.25, 0.5, 0.75]
    assert s2["rolled_m"] == pytest.approx(0.3 * (1.1 - 1 / RATE), rel=1e-3)


def test_inclusion_rules():
    slow, control = drive(0.5, 0.05)  # 0.0125 m per window: below the minimum travel
    assert not any(w["included"] for w in windows(slow, control)["windows"])
    straight, control = drive(0.02, 0.3)  # mean curvature below 0.05
    assert not any(w["included"] for w in windows(straight, control)["windows"])
    # a cusp inside a window is mixed and excluded
    cusp, control = drive(0.5, lambda t: 0.3 * np.cos(np.pi * t / 0.25 + 0.3))
    ws = windows(cusp, control)["windows"]
    assert any(w["direction"] == "mixed" for w in ws)
    assert not any(w["included"] for w in ws if w["direction"] == "mixed")


def test_cells_and_the_steady_flag():
    log, control = drive(0.45, 0.2)
    ws = windows(log, control)["windows"]
    assert {w["cell"] for w in ws if w["included"]} == {"fwd|left|k2|v1|steady"}
    log, control = drive(lambda t: 0.2 + 0.8 * t, 0.2)  # curvature ramp: 0.2 /m per window, not steady
    assert not any(w["steady"] for w in windows(log, control)["windows"])


def arm(gain, kappa=0.45, v=0.2, t_end=4.0):
    log, control = drive(kappa, v, gain=gain, t_end=t_end)
    return F.transport_windows(log, control, (0.0, t_end), L, W)


def test_a_pair_in_the_same_cell_gives_the_standardized_difference_and_the_staged_retention():
    a, b = arm(1.0), arm(1.1)
    cmp = F.compare_arms(a, b)
    assert cmp["common_cells"] == ["fwd|left|k2|v1|steady"]
    assert cmp["standardized_d"] == pytest.approx(0.1 * 0.45, rel=0.02)
    assert cmp["whole_d"] == pytest.approx(cmp["standardized_d"], rel=1e-9)
    assert cmp["observed_in_cells"]
    for x in ("A", "B"):
        s = cmp[x]
        assert s["rolled_m"] + 1e-12 >= s["signed_m"] >= s["included_m"] - 1e-12 >= s["common_m"] - 2e-12 > 0
        assert s["outside_m"] == 0


def test_no_common_cell_is_missing_not_zero():
    a, b = arm(1.0, kappa=0.45), arm(1.1, kappa=-0.45)  # different turn directions
    cmp = F.compare_arms(a, b)
    assert cmp["common_cells"] == [] and cmp["standardized_d"] is None and not cmp["observed_in_cells"]
    assert cmp["A"]["outside_m"] > 0


def test_a_short_common_cell_does_not_count():
    a, b = arm(1.0, t_end=4.0), arm(1.1, t_end=1.0)  # B rolls 0.15 m in the cell: below 0.3
    assert F.compare_arms(a, b)["common_cells"] == []


def test_balance_means_are_travel_weighted():
    c = F.cells([{"included": True, "cell": "x", "travel_m": 1.0, "over_rad": 0.1, "kappa_mean": 0.2, "speed_mean": 0.1,
                  "kappa_change": 0.0, "lr_mismatch": 0.0, "lift_m": None},
                 {"included": True, "cell": "x", "travel_m": 3.0, "over_rad": 0.1, "kappa_mean": 0.6, "speed_mean": 0.3,
                  "kappa_change": 0.0, "lr_mismatch": 0.0, "lift_m": 0.1}])["x"]
    assert c["index_rad_per_m"] == pytest.approx(0.05)
    assert c["kappa_mean"] == pytest.approx(0.5) and c["speed_mean"] == pytest.approx(0.25)
    assert c["lift_m"] == pytest.approx(0.1)


def test_the_lr_mismatch_is_zero_for_ackermann_and_visible_for_toe():
    log, control = drive(0.45, 0.2)
    assert all(abs(w["lr_mismatch"]) < 1e-9 for w in windows(log, control)["windows"])
    log["steering_rad"] = log["steering_rad"] + np.array([0.01, -0.01])
    assert all(w["lr_mismatch"] > 0 for w in windows(log, control)["windows"])


def test_scene_summary_counts():
    rows = [{"s2": {"standardized_d": 0.02, "whole_d": 0.03, "observed_in_cells": True}},
            {"s2": {"standardized_d": -0.01, "whole_d": 0.02, "observed_in_cells": False}},
            {"s2": {"standardized_d": None, "whole_d": 0.01, "observed_in_cells": False}}]
    s = F.scene_summary(rows, "s2")
    assert s["with_common_cells"] == 2 and s["standardized_positive"] == 1 and s["sign_agree"] == 1
    assert s["whole_positive"] == 3 and s["observed_in_cells"] == 1


def test_an_excursion_and_back_inside_a_window_is_not_steady():
    # independent plan review: kappa 0.2 -> 0.8 -> 0.2 inside each window has a net change of 0
    def kappa(t):
        ph = np.mod(t, F.WINDOW_S) / F.WINDOW_S
        return 0.2 + 0.6 * np.sin(np.pi * ph)

    log, control = drive(kappa, 0.25)
    ws = windows(log, control)["windows"]
    assert all(abs(w["kappa_change"]) < 0.5 * w["kappa_range"] for w in ws)  # the net change hides most of it
    assert not any(w["steady"] for w in ws)


def test_the_observation_rule_uses_the_overlap_not_the_common_cell_share():
    # every cell common, but each arm's distance mostly where the other has little: overlap is small
    def arm_two_cells(t_left, t_right):
        log1, c1 = drive(0.45, 0.2, t_end=t_left)
        a1 = F.transport_windows(log1, c1, (0.0, t_left), L, W)
        log2, c2 = drive(-0.45, 0.2, t_end=t_right)
        a2 = F.transport_windows(log2, c2, (0.0, t_right), L, W)
        return {"windows": a1["windows"] + a2["windows"], "rolled_m": a1["rolled_m"] + a2["rolled_m"]}

    a, b = arm_two_cells(20.0, 2.0), arm_two_cells(2.0, 20.0)
    cmp = F.compare_arms(a, b)
    assert len(cmp["common_cells"]) == 2
    assert cmp["A"]["common_share_of_rolled"] > 0.9
    assert cmp["A"]["overlap_share_of_rolled"] < 0.2 and not cmp["observed_in_cells"]
