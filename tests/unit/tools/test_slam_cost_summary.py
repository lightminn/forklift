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


def test_per_unit_means_leave_out_the_partial_last_unit():
    rows = [row(t, 0.1 * (1 + t // 100), unit=t // 100) for t in range(0, 310, 10)]
    s = cs.cost_summary(rows)
    assert s["unit_mean_s"] == pytest.approx([0.1, 0.2, 0.3]) and s["units_complete"] == 3
    assert s["max_s"] == pytest.approx(0.4)  # the partial unit still counts in the maximum


def test_a_partial_last_unit_cannot_lower_the_max_line():
    # Codex round 3 counterexample: one slow keyframe late in each 500 s unit, the cut landing early in the next unit
    rows = []
    for u in range(3):
        for k in range(16):
            rows.append(row(500 * u + 30 * k, 0.02 + (0.02 * (u + 1) if k == 14 else 0.0), unit=u))
    rows.append(row(1500.0, 0.02, unit=3))
    s = cs.cost_summary(rows)
    assert s["units_complete"] == 3 and s["max_at_8h_s"] > 1.0


def test_slam_pose_is_map_from_odom_times_odom_and_errors_against_truth():
    mfo, odom = (1.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0)
    assert cs.slam_pose(row(0, 0, mfo=mfo, odom=odom)) == pytest.approx((1.0, 1.0, math.pi / 2))
    rows = [row(0, 0, mfo=mfo, odom=odom, truth=(1.0, 1.05, math.pi / 2 + 0.01)),
            row(1, 0, mfo=(0, 0, 0), odom=(2.0, 0.0, 0.0), truth=(2.0, 0.0, 0.0))]
    e = cs.accuracy(rows)
    assert e["pos_max_m"] == pytest.approx(0.05) and e["pos_median_m"] == pytest.approx(0.025)
    assert e["yaw_max_deg"] == pytest.approx(math.degrees(0.01))


def test_accuracy_growth_compares_complete_units_on_the_same_scans():
    rows = [row(t, 0, unit=u, src=t, truth=(0.01 * u, 0.0, 0.0)) for u in range(3) for t in range(5)]
    rows += [row(9, 0, unit=3, src=0, truth=(0.5, 0.0, 0.0))]  # partial last unit: left out
    e = cs.accuracy(rows)
    assert e["unit_pos_median_m"] == pytest.approx([0.0, 0.01, 0.02])
    assert e["growth_m"] == pytest.approx(0.02)
    assert e["pos_max_m"] == pytest.approx(0.5)


def _live(n_scans=500, every=10):
    runner = [{"scan_id": i, "stamp_s": 0.1 * i, "odom_base": [0.01 * i, 0.0, 0.0]} for i in range(n_scans)]
    bridge = [{"scan_id": i, "status": "processed" if i % every == 0 else "skipped", "stamp_s": 0.1 * i,
               "wall_s": 0.1 + i / 10000} for i in range(n_scans)]
    replay = [row(0.1 * i, 0.1 + i / 10000, src=i, odom=(0.01 * i, 0.0, 0.0)) for i in range(0, n_scans, every)]
    return runner, bridge, replay


def test_validity_needs_the_live_keyframes_odometry_and_a_close_cost_curve():
    runner, bridge, replay = _live()
    v = cs.validity(replay, bridge, runner)
    assert v["same_keyframes"] and v["same_odometry"] and v["ok"]
    assert all(0.99 < x < 1.01 for x in v["tenth_ratio"])
    assert not cs.validity(replay[:-1], bridge, runner)["same_keyframes"]
    slow = [dict(r, slam_wall_s=2 * r["slam_wall_s"]) for r in replay]
    assert not cs.validity(slow, bridge, runner)["ok"]
    renamed = [dict(r, src=r["src"] + 1) for r in replay]
    assert not cs.validity(renamed, bridge, runner)["ok"]
    moved = [dict(r, odom_base=[r["odom_base"][0] + 1e-9, 0.0, 0.0]) for r in replay]
    v = cs.validity(moved, bridge, runner)
    assert not v["same_odometry"] and not v["ok"]


def test_validity_leaves_out_a_late_bridge_record_after_the_runners_last_scan():
    runner, bridge, replay = _live()
    bridge.append({"scan_id": 500, "status": "processed", "stamp_s": 50.0, "wall_s": 1.2})  # the timed-out scan
    assert cs.validity(replay, bridge, runner)["ok"]


def test_one_slow_keyframe_per_pass_fails_the_unit_max_extrapolation():
    # Codex round 2 counterexample: 15 of 16 keyframes per 83 s pass at 0.02 s, one at 0.02 + 0.01 x pass; the mean
    # line stays low at 8 h, the slowest keyframe passes 1 s near pass 99 -- the per-unit maximum line catches it
    rows, unit_s = [], 6 * 83.0
    for p in range(18):
        for k in range(16):
            t_ = p * 83.0 + k * 5.0
            rows.append(row(t_, 0.02 + (0.01 * p if k == 8 else 0.0), unit=int(t_ // unit_s)))
    rows.append(row(3 * unit_s, 0.02, unit=3))  # a splice stops inside its last unit
    s = cs.cost_summary(rows)
    assert s["at_8h_s"] < 0.5
    assert s["unit_max_s"] == pytest.approx([0.07, 0.13, 0.19])
    assert s["max_at_8h_s"] > 1.0


def test_unit_lines_need_three_units():
    rows = [row(t, 0.03, unit=t // 500) for t in range(0, 1000, 10)]
    assert cs.cost_summary(rows)["max_at_8h_s"] is None
