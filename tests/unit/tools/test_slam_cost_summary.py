"""slam_link_failed plan E1 summary (docs/plans/2026-10-25-slam-link-failed.md 3-4): a replay's keyframe cost per
unit and its line against sim time extrapolated to sim 8 h, the SLAM pose (map<-odom composed with the odometry sent)
against the truth, and the validity check of an R0 replay against the live bridge record."""

import math

import pytest

from tools import slam_cost_summary as cs


def row(t, w, *, unit=0, src=None, mfo=(0.0, 0.0, 0.0), odom=(0.0, 0.0, 0.0), truth=(0.0, 0.0, 0.0)):
    return {"stamp_s": t, "slam_wall_s": w, "unit": unit, "src": src if src is not None else int(t * 10),
            "map_from_odom": list(mfo), "odom_base": list(odom), "truth_base": list(truth)}


def test_flat_cost_extrapolates_flat_and_a_slope_does_not():
    flat = [row(t, 0.03) for t in range(0, 1500, 10)]
    s = cs.cost_summary(flat)
    assert s["at_8h_s"] == pytest.approx(0.03, abs=1e-9)
    assert s["max_s"] == pytest.approx(0.03)
    # 0.02 + 0.009 per 83 s pass: passes the finite window, fails the 8 h extrapolation (Codex round 1 counterexample)
    rising = [row(t, 0.02 + 0.009 * t / 83.0) for t in range(0, 1500, 10)]
    r = cs.cost_summary(rising)
    assert r["max_s"] < 0.2 and r["at_8h_s"] > 1.0


def test_per_unit_means_and_last_two_passes_window():
    rows = [row(t, 0.1 * (1 + t // 100), unit=t // 100) for t in range(0, 300, 10)]
    s = cs.cost_summary(rows)
    assert s["unit_mean_s"] == pytest.approx([0.1, 0.2, 0.3])


def test_slam_pose_is_map_from_odom_times_odom_and_errors_against_truth():
    mfo, odom = (1.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0)
    assert cs.slam_pose(row(0, 0, mfo=mfo, odom=odom)) == pytest.approx((1.0, 1.0, math.pi / 2))
    rows = [row(0, 0, mfo=mfo, odom=odom, truth=(1.0, 1.05, math.pi / 2 + 0.01)),
            row(1, 0, mfo=(0, 0, 0), odom=(2.0, 0.0, 0.0), truth=(2.0, 0.0, 0.0))]
    e = cs.accuracy(rows)
    assert e["pos_max_m"] == pytest.approx(0.05) and e["pos_median_m"] == pytest.approx(0.025)
    assert e["yaw_max_deg"] == pytest.approx(math.degrees(0.01))


def test_accuracy_per_unit_growth():
    rows = [row(t, 0, unit=u, odom=(0.0, 0.0, 0.0), truth=(0.01 * u, 0.0, 0.0)) for u in range(3) for t in range(5)]
    e = cs.accuracy(rows)
    assert e["unit_pos_median_m"] == pytest.approx([0.0, 0.01, 0.02])
    assert e["growth_m"] == pytest.approx(0.02)


def test_validity_needs_the_live_keyframes_and_a_close_cost_curve():
    live = [{"status": "processed", "stamp_s": t, "wall_s": 0.1 + t / 1000} for t in range(0, 500, 10)]
    live += [{"status": "skipped", "stamp_s": 5.0, "wall_s": 0.0}]
    good = [row(t, 0.1 + t / 1000) for t in range(0, 500, 10)]
    v = cs.validity(good, live)
    assert v["same_keyframes"] and v["ok"]
    assert all(0.99 < x < 1.01 for x in v["tenth_ratio"])
    missing = good[:-1]
    assert not cs.validity(missing, live)["same_keyframes"]
    slow = [row(t, 2 * (0.1 + t / 1000)) for t in range(0, 500, 10)]
    assert not cs.validity(slow, live)["ok"]
