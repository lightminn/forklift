"""tools/mass_along_judge.py on synthetic records: the counterexamples the plan's reviews raised
(docs/plans/2026-11-05-pallet-mass-along-error-probe.md section 3 and 10)."""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import mass_along_judge as J  # noqa: E402
from forklift_core.localization.slam_pose import compose, invert  # noqa: E402

C0 = (0.0, 0.0, 0.0)
C1 = (0.01, -0.02, 0.003)
C2 = (-0.03, 0.01, -0.002)


def arc(t):
    """A truth rear pose turning left at 0.5 rad over 1 s while moving 0.4 m/s."""
    yaw = 0.5 * t
    return (0.8 * math.sin(yaw), 0.8 * (1 - math.cos(yaw)), yaw)


def run_records(*, err_at_c=(0.0, 0.10, 0.02), odo_scale=1.0, move_after_scan=0.0, t_c=0.3, t_end=1.0, dt=1 / 120):
    """Control rows, slam records and the round: one correction applied at t_c (tracking), the estimate afterwards is
    the corrected one moved with the odometry (``odo_scale`` x the true relative motion's translation); the match is at
    t_end, its scan at the last 0.1 s stamp, and the truth moves ``move_after_scan`` forward between the last control row
    and the match."""
    ts = np.arange(0.0, t_end + 1e-9, dt)
    rows = []
    e_c = compose(arc(t_c), err_at_c)
    for t in ts:
        T = arc(t)
        if t < t_c:
            E = compose(T, (0.0, 0.0, 0.0))
        else:
            rel = compose(invert(arc(t_c)), T)
            E = compose(e_c, (rel[0] * odo_scale, rel[1] * odo_scale, rel[2]))
        rows.append([t, *E, *T])
    control = np.array(rows)
    t_m = float(ts[-1] + dt)
    T_m = compose(arc(ts[-1]), (move_after_scan, 0.0, 0.0))
    t_s = math.floor(ts[-1] * 10) / 10
    E_s = control[np.searchsorted(ts, t_s), 1:4]
    round1 = {"round": 1, "time_s": t_m, "estimate_error_truck_frame": list(compose(invert(T_m), tuple(E_s))),
              "estimate_vs_truth_at_match": [float(E_s[0] - T_m[0]), float(E_s[1] - T_m[1])]}
    records = [{"scan_id": 1, "stamp_s": 0.1, "status": "processed", "mode": "tracking", "map_from_odom": list(C0),
                "applied_map_from_odom": list(C0)},
               {"scan_id": 2, "stamp_s": t_c, "status": "processed", "mode": "tracking", "map_from_odom": list(C1),
                "applied_map_from_odom": list(C1)},
               {"scan_id": 3, "stamp_s": t_c + 0.2, "status": "skipped", "mode": "tracking", "map_from_odom": list(C1),
                "applied_map_from_odom": list(C1)}]
    result = {"delivery_docking": {"rounds": [round1]}, "slam_summary": {"holds": []},
              "transitions": [{"to": "transport", "time_s": 0.0}]}
    return result, control, records


def test_perfect_odometry_through_a_turn_puts_nothing_in_e_new():
    # Codex round 1: the plain difference read about 5 mm here although nothing new accumulated
    result, control, records = run_records()
    sp = J.split(result, control, records)
    assert abs(sp["e_new"]) < 1e-12 and abs(sp["e_phase"]) < 1e-3
    assert sp["e_m"] == pytest.approx(sp["e_inh"] + sp["e_new"] + sp["e_phase"], abs=1e-15)
    plain = sp["e_m"] - J.along(control[np.searchsorted(control[:, 0], 0.3), 4:7],
                                control[np.searchsorted(control[:, 0], 0.3), 1:4])
    assert abs(plain) > 1e-3  # the rejected definition would have called the carried error new


def test_motion_after_the_matched_scan_goes_to_e_phase():
    # Codex round 2: 2 mm of truth motion after the scan is not odometry drift
    moved = J.split(*run_records(move_after_scan=0.002))
    base = J.split(*run_records())
    assert abs(moved["e_new"]) < 1e-12
    assert moved["e_phase"] - base["e_phase"] == pytest.approx(-0.002, abs=1e-9)
    assert (moved["e_inh"], moved["e_new"]) == (base["e_inh"], base["e_new"])


def test_odometry_scale_error_shows_in_e_new():
    result, control, records = run_records(err_at_c=(0.0, 0.0, 0.0), odo_scale=0.98)
    sp = J.split(result, control, records)
    assert sp["e_new"] < -1e-3 and abs(sp["e_inh"]) < 1e-12


def _rec(i, t, status, mode, received, applied):
    return {"scan_id": i, "stamp_s": t, "status": status, "mode": mode, "map_from_odom": list(received),
            "applied_map_from_odom": list(applied)}


@pytest.mark.parametrize("reply, event", [(C1, False), (C0, True), (C2, True)])
def test_after_a_release_the_next_reply_counts_only_when_it_changes_the_applied_transform(reply, event):
    # Codex round 3: C1 applied by the release; the next processed reply C1 / C0 / C2
    records = [_rec(1, 0.1, "processed", "tracking", C0, C0),
               _rec(2, 0.2, "processed", "holding", C1, C0),
               _rec(3, 0.4, "processed", "tracking", reply, reply)]
    holds = [{"event": "release", "time_s": 0.3, "jump_m": 0.02, "jump_rad": 0.003}]
    events, bad = J.correction_events(records, holds)
    assert bad is None
    assert [round(t, 3) for t, _ in events] == [0.1, 0.3] + ([0.4] if event else [])


def test_a_release_then_a_new_processed_correction_keeps_both():
    records = [_rec(1, 0.1, "processed", "tracking", C0, C0), _rec(2, 0.2, "processed", "holding", C1, C0),
               _rec(3, 0.5, "processed", "tracking", C2, C2)]
    holds = [{"event": "release_docking_retry", "time_s": 0.3, "jump_m": 0.02, "jump_rad": 0.0}]
    events, bad = J.correction_events(records, holds)
    assert bad is None and [k for _, k in events] == ["first", "release", "processed"]


def test_a_zero_jump_release_is_no_event_and_a_disagreeing_one_is_an_inconsistency():
    records = [_rec(1, 0.1, "processed", "tracking", C0, C0)]
    events, bad = J.correction_events(records, [{"event": "release", "time_s": 0.2, "jump_m": 0.0, "jump_rad": 0.0}])
    assert bad is None and len(events) == 1
    records = [_rec(1, 0.1, "processed", "tracking", C0, C0), _rec(2, 0.2, "processed", "holding", C1, C0)]
    events, bad = J.correction_events(records, [{"event": "release", "time_s": 0.3, "jump_m": 0.0, "jump_rad": 0.0}])
    assert bad is not None and "disagree" in bad


def test_a_record_whose_applied_value_the_replay_cannot_reach_is_an_inconsistency():
    records = [_rec(1, 0.1, "processed", "tracking", C0, C0), _rec(2, 0.2, "processed", "holding", C1, C1)]
    events, bad = J.correction_events(records, [])
    assert bad is not None and "differs" in bad


def test_records_come_before_releases_at_the_same_time():
    records = [_rec(1, 0.1, "processed", "tracking", C0, C0), _rec(2, 0.3, "processed", "holding", C1, C0)]
    events, bad = J.correction_events(records, [{"event": "release", "time_s": 0.3, "jump_m": 0.02, "jump_rad": 0.0}])
    assert bad is None and events[-1] == (0.3, "release")


def test_the_scale_error_is_signed_per_direction():
    # Codex round 1: a perfect 1 m reverse read k = -2 with unsigned truth distance
    ts = np.arange(0.0, 2.0001, 0.01)
    x = np.where(ts <= 1.0, ts, 2.0 - ts)  # 1 m forward then 1 m back, heading 0
    control = np.column_stack([ts, x, 0 * ts, 0 * ts, x, 0 * ts, 0 * ts])
    inc = np.diff(np.r_[0.0, x]) / 0.125
    log = {"joint_stamps_s": ts, "wheel_increments_rad": np.column_stack([inc, inc])}
    k = J.scale_errors(log, control, (0.0, 2.0), 0.125)
    assert k["k_fwd"] == pytest.approx(0.0, abs=1e-9) and k["k_rev"] == pytest.approx(0.0, abs=1e-9)
    log = {"joint_stamps_s": ts, "wheel_increments_rad": np.column_stack([inc, inc]) * 1.02}
    k = J.scale_errors(log, control, (0.0, 2.0), 0.125)
    assert k["k_fwd"] == pytest.approx(0.02, abs=1e-6) and k["k_rev"] == pytest.approx(-0.02, abs=1e-6)


def test_signed_pitch_and_roll():
    a = 0.01
    pitch, roll = J.pitch_roll((math.cos(a / 2), 0.0, math.sin(a / 2), 0.0))
    assert pitch == pytest.approx(a) and roll == pytest.approx(0.0, abs=1e-12)
    pitch, roll = J.pitch_roll((math.cos(a / 2), math.sin(a / 2), 0.0, 0.0))
    assert roll == pytest.approx(a) and pitch == pytest.approx(0.0, abs=1e-12)


def _pair(scene, seed, d, comps=None, same=True):
    p = {"scene": scene, "seed": seed, "valid": True, "primary": True, "delta_e_m": d, "same_path": same,
         "mech": {"A": {"x": 0.0}, "B": {"x": d}}}
    if comps is not None:
        p["delta"] = dict(zip(("e_inh", "e_new", "e_phase"), comps))
    return p


def test_seed_blocks_drop_a_seed_valid_in_one_scene_only():
    pairs = [_pair("line_2", s, -0.03) for s in range(10)] + [_pair("random_1", s, -0.01) for s in range(9)]
    b = J.blocks(pairs, lambda p: p["delta_e_m"])
    assert sorted(b) == list(range(9)) and b[0] == pytest.approx(-0.02)


def test_j1_and_j2_on_a_clear_new_drift_effect():
    rng = np.random.default_rng(0)
    pairs = []
    for s in range(10):
        for scene in J.SCENES:
            d = -0.03 + rng.normal(0, 0.002)
            pairs.append(_pair(scene, s, d, comps=(0.1 * d, 0.9 * d, 0.0)))
    out = J.verdict(pairs, set())
    assert out["J1"] == "confirmed (this condition)"
    assert out["J2"]["representative"] and out["J2"]["verdict"] == "new odometry drift leads"


def test_j2_refuses_a_decomposed_subset_that_lost_the_effect():
    # Codex round 1: 15 small pairs, 5 large ones whose split is missing
    pairs = []
    for s in range(10):
        big = s < 3
        for scene in J.SCENES:
            d = -0.05 if big else -0.004
            pairs.append(_pair(scene, s, d, comps=None if big else (0.0, d, 0.0)))
    out = J.verdict(pairs, set())
    assert out["J1"] == "confirmed (this condition)"
    assert not out["J2"]["representative"] and out["J2"]["verdict"].startswith("undecidable")


def test_j2_refuses_when_the_phase_share_is_large():
    pairs = [_pair(sc, s, -0.03, comps=(0.0, -0.015, -0.015)) for s in range(10) for sc in J.SCENES]
    pairs[0]["delta_e_m"] = -0.031  # a spread for the t interval
    out = J.verdict(pairs, set())
    assert not out["J2"]["representative"]


def test_j1_needs_the_point_estimate_threshold_and_stops_on_unconfirmed_determinism():
    pairs = [_pair(sc, s, -0.005 + 0.0001 * s) for s in range(10) for sc in J.SCENES]
    assert J.verdict(pairs, set())["J1"] == "not confirmed"
    assert J.verdict(pairs, {"line_2/A"})["J"].startswith("stopped")


def test_too_few_blocks_is_undecidable():
    pairs = [_pair(sc, s, -0.03 - 0.001 * s) for s in range(6) for sc in J.SCENES]
    assert J.verdict(pairs, set())["J1"].startswith("undecidable")


def test_cusps_and_divergence():
    ts = np.arange(0.0, 2.0001, 0.01)
    x = np.where(ts <= 1.0, ts, 2.0 - ts)
    control = np.column_stack([ts, x, 0 * ts, 0 * ts, x, 0 * ts, 0 * ts])
    assert J.cusps(control, (0.0, 2.0)) == 1
    other = control.copy()
    other[150:, 1] += 1e-9
    assert J.first_divergence(control, other) == pytest.approx(1.5)
    assert J.first_divergence(control, control) is None


def test_judge_reads_run_folders_and_checks_the_arm_identity(tmp_path):
    result, control, records = run_records()
    for arm, kg in (("A", 10.0), ("B", 4.93)):
        for scene in J.SCENES:
            run = tmp_path / f"mae_{arm}_{scene}_s0" / "run"
            run.mkdir(parents=True)
            r = dict(result, pallet_mass={"kg": kg, "shape": "epal6"}, deterministic={"enabled": True, "broken": []},
                     source_sha256="x", script_sha256="y", seed=0)
            (run / "result.json").write_text(json.dumps(r))
            np.save(run / "slam_control.npy", control)
            (run / "slam_records.json").write_text(json.dumps(records))
    compare = {f"{s}/{a}": True for s in J.SCENES for a in J.ARMS}
    out = J.judge(tmp_path, compare)
    assert all(p["valid"] and p["primary"] for p in out["pairs"]) and len(out["pairs"]) == 2
    assert out["pairs"][0]["delta_e_m"] == pytest.approx(0.0)
    bad = tmp_path / "mae_B_line_2_s0" / "run" / "result.json"
    r = json.loads(bad.read_text())
    r["pallet_mass"]["kg"] = 10.0
    bad.write_text(json.dumps(r))
    out = J.judge(tmp_path, compare)
    assert any(not p["valid"] and "not arm B" in p["why"] for p in out["pairs"])
