"""C2(b) stage 1 estimate (docs/validation/2026-11-13-baseline-fragility/c2b_estimate.py,
docs/plans/2026-11-13-c2b-hold-release.md 2): the end-of-leg window, the record replay, the counterfactual seat offset,
the jump and the pass rule. Synthetic records, no Isaac."""

import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    "c2b_estimate", ROOT / "docs/validation/2026-11-13-baseline-fragility/c2b_estimate.py")
C = importlib.util.module_from_spec(spec)
spec.loader.exec_module(C)


def straight(t0=5.0, t1=15.0, speed=0.3, y_truth=-0.02):
    """Rows every 1/120 s, both poses driving +x at `speed`; the estimate on the target line y = 0, the truth y_truth
    beside it (so the estimate is -y_truth to the left of the truth)."""
    t = np.arange(t0, t1 + 1e-9, 1 / 120)
    x = speed * (t - t0)
    return np.column_stack([t, x, np.zeros_like(t), np.zeros_like(t), x, np.full_like(t, y_truth), np.zeros_like(t)])


def records(t0, t1, mfo_by_time, applied=(0.0, 0.0, 0.0)):
    """A record every 0.1 s; map_from_odom switches at the given times (processed rows), else kept (skipped)."""
    out, current = [], list(applied)
    for k in range(int(round((t1 - t0) / 0.1)) + 1):
        stamp = round(t0 + 0.1 * k, 6)
        status = "skipped"
        for t, m in mfo_by_time:
            if abs(stamp - t) < 1e-6:
                current, status = list(m), "processed"
        out.append({"stamp_s": stamp, "status": status, "mode": "holding", "map_from_odom": list(current),
                    "applied_map_from_odom": list(applied)})
    return out


def case(mfo_by_time):
    sc = straight()
    t_cap, t_ins, t_lift = 5.0, 12.0, 15.0
    V = [3.5, 0.0]
    E, A = sc[-1, 1:4], sc[-1, 4:7]
    decomp = {"name": "lbg_B0_random_1_s0", "scene": "random_1", "arm": "B0", "capture_t": t_cap, "t_insert": t_ins,
              "t_lift": t_lift, "V": V, "E": list(E), "A": list(A),
              "terms_m": {"S": C.lateral(V, A), "S_const": 0.0}}
    result = {"samples": [{"time_s": 15.0, "phase": "lift", "pallet_yaw_rad": 0.0}],
              "arguments": {"rear_axle_offset_m": -0.34},
              "chassis_model": {"drive_geometry": {"wheel_radius_m": 0.125}},
              "paths": {"approach": {"poses": [[0.0, 0.0, 0.0], [1.05, 0.0, 0.0], [2.1, 0.0, 0.0]]}},
              "slam_summary": {"holds": [{"event": "hold", "time_s": 4.0}]}}
    recs = records(3.0, 15.0, [(3.5, (0.0, 0.0, 0.0))] + list(mfo_by_time))
    stamps = sc[:, 0]
    log = {"joint_stamps_s": stamps,
           "wheel_increments_rad": np.full((len(stamps), 2), 0.3 / 120 / 0.125)}
    return decomp, result, sc, recs, log


def test_the_end_window_is_half_a_metre_plus_the_braking_distance():
    d, r, sc, rec, log = case([])
    e = C.run_estimate(d, r, sc, rec, log)["end_hold"]
    assert e["t_k"] == pytest.approx(12.0 - 0.65 / 0.3, abs=2 / 120 + 1e-9)
    assert e["t_k_travel_sum"] == pytest.approx(e["t_k"], abs=2 / 120 + 1e-9)
    t_k = C.end_hold_time(sc, 5.0, 12.0)
    # 0.3 m/s: 0.5 + 0.09 / 0.6 = 0.65 m before the insert entry, 0.65 / 0.3 s
    assert t_k == pytest.approx(12.0 - 0.65 / 0.3, abs=1 / 120 + 1e-9)
    assert C.end_hold_time(sc, 5.0, 12.0, 1.0) == pytest.approx(12.0 - 1.0 / 0.3, abs=1 / 120 + 1e-9)


def test_no_correction_change_leaves_the_offset():
    d, r, sc, rec, log = case([])
    e = C.run_estimate(d, r, sc, rec, log)["end_hold"]
    assert e["S_C2b_m"] == pytest.approx(d["terms_m"]["S"], abs=1e-12) and abs(e["check_identity_m"]) < 1e-12
    assert e["dB_C2b_m"] == pytest.approx(d["terms_m"]["S"] - d["terms_m"]["S_const"], abs=1e-12)
    assert e["jump_m"] == pytest.approx(0.0, abs=1e-12) and not e["over_jump"]


def test_a_correction_that_removes_the_estimate_error_removes_the_offset():
    """The estimate is 20 mm left of the truth; a correction 20 mm right, received before the window, takes it out:
    the truck stops on the target in truth, so S_C2b is the target's own offset in the estimate (zero here)."""
    d, r, sc, rec, log = case([(8.0, (0.0, -0.02, 0.0))])
    out = C.run_estimate(d, r, sc, rec, log)
    e = out["end_hold"]
    assert d["terms_m"]["S"] == pytest.approx(0.02)
    assert e["S_C2b_m"] == pytest.approx(C.lateral(d["V"], d["E"]), abs=1e-12) == pytest.approx(0.0, abs=1e-12)
    assert e["jump_m"] == pytest.approx(0.02, abs=1e-12) and not e["over_jump"]
    assert e["received_error_lateral_m"] == pytest.approx(0.0, abs=1e-12)
    assert e["processed_to_t_k"] == 1 and e["replies_to_t_k"] > 40 and out["held_correction_processed_age_s"] == \
        pytest.approx(0.5)
    assert e["last_received_age_s"] <= 0.1 and e["last_processed_age_s"] > 1.0


def test_a_correction_after_the_window_is_not_taken():
    d, r, sc, rec, log = case([(11.5, (0.0, -0.02, 0.0))])  # after t_k (about 9.83 s)
    assert C.run_estimate(d, r, sc, rec, log)["end_hold"]["S_C2b_m"] == pytest.approx(d["terms_m"]["S"], abs=1e-12)


def test_stale_and_moving_held_corrections_are_not_judgeable():
    d, r, sc, rec, log = case([])
    stale = [x for x in rec if not (9.0 < x["stamp_s"] < 10.5)]
    assert C.run_estimate(d, r, sc, stale, log)["end_hold"]["why"] == "no fresh correction at t_k"
    moved = [dict(x, applied_map_from_odom=[0.0, 0.001, 0.0]) if x["stamp_s"] > 13 else x for x in rec]
    out = C.run_estimate(d, r, sc, moved, log)
    assert out["judgeable"] is False and "not constant" in out["why"]


def test_the_jump_is_the_base_position_change_at_the_same_odometry_pose():
    # a rotation about the base pose itself: no position jump although Delta's translation is not zero
    base = (1.0, 2.0, 0.3)
    rear = C.pose_of(C.se2(base) @ C.se2([-0.34, 0.0, 0.0]))
    rot = C.se2(base) @ C.se2([0, 0, 0.03]) @ np.linalg.inv(C.se2(base))
    pos, yaw = C.jump((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), C.pose_of(rot), rear, -0.34)
    assert pos == pytest.approx(0.0, abs=1e-12) and yaw == pytest.approx(0.03)
    assert np.hypot(*C.pose_of(rot)[:2]) > 0.05


def run(S, S_after, over=False, scene="random_1", arm="B0", judgeable=True):
    e = {"S_C2b_m": S_after, "over_jump": over} if judgeable else {"why": "x"}
    return {"S_m": S, "scene": scene, "arm": arm, "end_hold": e}


def test_the_pass_rule_and_its_conservative_variant():
    good = [run(0.03, 0.005) for _ in range(30)] + [run(0.02, 0.025) for _ in range(3)] + [run(0.002, 0.003)] * 54
    j = C.judge(good)
    assert j["plain"]["selected"] == 33 and j["plain"]["better"] == 30 and j["pass"]
    jumpy = [run(0.03, 0.005, over=True) for _ in range(30)] + good[30:]
    assert C.judge(jumpy)["plain"]["all"] and not C.judge(jumpy)["conservative"]["all"] and not C.judge(jumpy)["pass"]
    # five normal runs worse by more than 10 mm (condition 4)
    c = C.judge(good[:33] + [run(0.002, 0.015)] * 5 + [run(0.002, 0.003)] * 49)["plain"]["conditions"]
    assert not c["4_worse_by_10mm_at_most_4"] and c["3_fewer_over_10mm"]
    # every normal run pushed just over 10 mm, each by less than 10 mm: more runs over 10 mm than before (condition 3)
    c = C.judge(good[:33] + [run(0.002, 0.011)] * 54)["plain"]["conditions"]
    assert not c["3_fewer_over_10mm"] and c["4_worse_by_10mm_at_most_4"]
    empty = C.judge([run(0.002, 0.003)])
    assert empty["plain"]["median_before_m"] is None and not empty["pass"]
    missing = [run(0.03, 0.0, judgeable=False) for _ in range(30)] + good[30:]
    assert C.judge(missing)["not_judgeable"] == 30 and not C.judge(missing)["pass"]


def test_the_cli(tmp_path):
    d, r, sc, rec, log = case([(8.0, (0.0, -0.02, 0.0))])
    run_dir = tmp_path / "check" / d["name"] / "run"
    run_dir.mkdir(parents=True)
    (run_dir / "result.json").write_text(json.dumps(r))
    np.save(run_dir / "slam_control.npy", sc)
    (run_dir / "slam_records.json").write_text(json.dumps(rec))
    np.savez(run_dir / "slam_log.npz", **log)
    decomp = tmp_path / "d.json"
    decomp.write_text(json.dumps({"runs": [dict(d, root="check", judgement={"status": "judged"}),
                                           {"name": "x", "judgement": {"status": "held"}}]}))
    out = tmp_path / "c.json"
    assert C.main([str(decomp), "--raw-root", str(tmp_path), "--out", str(out)]) == 0
    payload = json.loads(out.read_text())
    assert payload["n"] == 1 and math.isclose(payload["runs"][0]["end_hold"]["S_C2b_m"], 0.0, abs_tol=1e-12)


def test_a_large_jump_on_the_way_counts_even_if_the_end_jump_is_small():
    """Codex review round 2 P2: 30 mm out and back between the capture and t_k."""
    d, r, sc, rec, log = case([(7.0, (0.0, 0.03, 0.0)), (8.0, (0.0, 0.001, 0.0))])
    e = C.run_estimate(d, r, sc, rec, log)["end_hold"]
    assert e["jump_m"] == pytest.approx(0.001, abs=1e-9) and e["max_jump_m"] == pytest.approx(0.03, abs=1e-9)
    assert e["over_jump"] and [round(j["t"], 3) for j in e["jumps"]] == [7.0, 8.0]


def test_the_tracker_speed_and_the_path_progress():
    stamps = np.arange(1, 61) / 120
    inc = np.zeros((60, 2))
    inc[30:] = 0.24 / 120 / 0.125  # 0.24 m/s from the 31st sample
    t, v = C.tracker_speeds({"joint_stamps_s": stamps, "wheel_increments_rad": inc}, 0.125)
    assert v[10] == pytest.approx(0.012 * 1.01)  # standing: kept just above the stop speed
    assert v[-1] == pytest.approx(0.24) and v[35] == pytest.approx(0.24 * 6 / 12)
    path = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    rem = C.path_remaining(path, np.array([[0.5, 0.1], [0.4, 0.0], [1.0, 0.5], [0.9, 0.95]]))
    assert rem[0] == pytest.approx(1.5) and rem[1] == pytest.approx(1.5)  # progress does not go back
    assert rem[2] == pytest.approx(0.5) and rem[3] == pytest.approx(0.05)


def test_successive_jumps_recover_the_odometry_pose_with_the_held_correction():
    """Codex review round 3 P2: the record's estimate is M_hold * odom whatever was applied since; a second jump from a
    non-identity previous correction, with rotation and translation, is measured on that same odometry pose."""
    M_hold, M1, M2 = (0.01, -0.02, 0.005), (0.02, 0.0, 0.012), (0.015, 0.018, 0.03)
    E = (2.0, 1.0, 0.4)
    odom = np.linalg.inv(C.se2(M_hold)) @ C.se2(E)
    base = C.se2([0.34, 0.0, 0.0])
    a, b = C.pose_of(C.se2(M2) @ odom @ base), C.pose_of(C.se2(M1) @ odom @ base)
    pos, yaw = C.jump(M_hold, M1, M2, E, -0.34)
    assert pos == pytest.approx(np.hypot(a[0] - b[0], a[1] - b[1]), abs=1e-12) and yaw == pytest.approx(0.018)
    wrong = C.pose_of(C.se2(M2) @ np.linalg.inv(C.se2(M1)) @ C.se2(E) @ base)  # odom recovered with M1
    assert abs(np.hypot(wrong[0] - b[0], wrong[1] - b[1]) - pos) > 1e-4
