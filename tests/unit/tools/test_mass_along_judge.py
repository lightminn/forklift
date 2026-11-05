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
              "transitions": [{"from": "lift", "to": "transport", "time_s": 0.0}]}
    match = {"estimate": np.asarray(E_s), "truth_rear": np.asarray(T_m)}
    return result, control, records, match


def test_perfect_odometry_through_a_turn_puts_nothing_in_e_new():
    # Codex round 1: the plain difference read about 5 mm here although nothing new accumulated
    result, control, records, match = run_records()
    sp = J.split(result, control, records, match)
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
    sp = J.split(*run_records(err_at_c=(0.0, 0.0, 0.0), odo_scale=0.98))
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
    assert [round(e[0], 3) for e in events] == [0.1, 0.3] + ([0.4] if event else [])


def test_a_release_then_a_new_processed_correction_keeps_both():
    records = [_rec(1, 0.1, "processed", "tracking", C0, C0), _rec(2, 0.2, "processed", "holding", C1, C0),
               _rec(3, 0.5, "processed", "tracking", C2, C2)]
    holds = [{"event": "release_docking_retry", "time_s": 0.3, "jump_m": 0.02, "jump_rad": 0.0}]
    events, bad = J.correction_events(records, holds)
    assert bad is None and [e[1] for e in events] == ["first", "release", "processed"]


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
    assert bad is None and events[-1][:2] == (0.3, "release")


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
    # Codex tool review P2: a 1 mm reversal after a long run is no cusp (both sides must run 5 cm)
    x2 = np.where(ts <= 1.0, ts, np.maximum(1.0 - (ts - 1.0) * 0.001, 0.999))
    c2 = np.column_stack([ts, x2, 0 * ts, 0 * ts, x2, 0 * ts, 0 * ts])
    assert J.cusps(c2, (0.0, 2.0)) == 0
    other = control.copy()
    other[150:, 1] += 1e-9
    assert J.first_divergence(control, other) == pytest.approx(1.5)
    assert J.first_divergence(control, control) is None
    assert J.first_divergence(control, control[:150]) == pytest.approx(1.5)  # the first extra row, not the last common


def _write_run(folder, arm, kg, scene, seed=0, **over):
    result, control, records, match = run_records()
    run = folder / "run"
    run.mkdir(parents=True)
    ts = control[:, 0]
    samples = [{"time_s": float(t), "steering_command_rad": [0.01, 0.012], "steering_actual_rad": [0.011, 0.0115],
                "lift_m": 0.2, "pallet_yaw_rad": 0.0} for t in ts[::12]]
    r = dict(result, pallet_mass={"kg": kg, "shape": "epal6"}, deterministic={"enabled": True, "broken": []},
             seed=seed, samples=samples, initial_pallet_m=[1.0, 0.0, 0.07],
             arguments={"settings": f"/s/mae_0108c3ff_{arm}/config/x.yaml", "output": str(run), "rear_axle_offset_m": -0.34},
             settings_synthetic={"pallet_mass_kg": kg, "wheel_torque_nm": 3.0}, scene_layout={"sha256": "L"},
             **{k: "h" for k in ("source_sha256", "script_sha256", "forklift_urdf_sha256", "pallet_geometry_sha256",
                                 "pallet_urdf_sha256", "encoder_model", "chassis_model")},
             scene_sha256={"scene.usda": "s"}, plant={"m": 1}, plant_tyre_contact=[{"k": None}])
    r.update(over)
    (run / "result.json").write_text(json.dumps(r))
    np.save(run / "slam_control.npy", control)
    (run / "slam_records.json").write_text(json.dumps(records))
    np.savez(run / "delivery_match_1.npz", estimate=match["estimate"], truth_rear=match["truth_rear"])
    q = np.tile([1.0, 0.0, 0.0, 0.0], (len(ts), 1))
    np.savez(run / "slam_log.npz", joint_stamps_s=ts, base_pose_world=np.column_stack([control[:, 4:6], 0 * ts, q]),
             wheel_increments_rad=np.zeros((len(ts), 2)), steering_rad=np.zeros((len(ts), 2)),
             steering_measured_rad=np.full((len(ts), 2), 0.001))
    tyres = np.zeros((len(ts[::12]), 41))
    tyres[:, 0] = ts[::12]
    for i in range(4):
        tyres[:, 1 + 10 * i + 2] = -60.0 / 120.0  # 60 N each, the floor's push
    np.save(run / "tyre_contacts.npy", tyres)


def _batch(tmp_path, **over_b):
    for arm, kg in (("A", 10.0), ("B", 4.93)):
        for scene in J.SCENES:
            _write_run(tmp_path / f"mae_{arm}_{scene}_s0", arm, kg, scene, **(over_b if arm == "B" else {}))
    return {f"{s}/{a}": True for s in J.SCENES for a in J.ARMS}


def test_judge_reads_real_format_records(tmp_path):
    # Codex tool review P1: samples carry [FL, FR] steering arrays; the log, tyres and match file in their real shapes
    out = J.judge(tmp_path, _batch(tmp_path))
    assert all(p["valid"] and p["primary"] for p in out["pairs"]) and len(out["pairs"]) == 2
    p = out["pairs"][0]
    assert p["delta_e_m"] == pytest.approx(0.0)
    mech = p["mech"]["A"]
    assert mech["transport.steer_track_err_rad"] == pytest.approx(0.00075)
    assert mech["transport.steer_read_err_rad"] == pytest.approx(0.001)
    assert mech["transport.tyre_fz_rl_n"] == pytest.approx(60.0) and mech["transport.rear_load_share"] == pytest.approx(0.5)
    assert mech["static_fz_sum_n"] == pytest.approx(240.0)
    assert "transport.tyre_fz_fl_n" in out["J3"]  # per-tyre values reach J3, which runs whatever J1 says


@pytest.mark.parametrize("over, why", [
    ({"pallet_mass": {"kg": 10.0, "shape": "epal6"}}, "not arm B"),
    ({"arguments": {"settings": "/s/mae_0108c3ff_B/config/x.yaml", "output": "o", "rear_axle_offset_m": -0.34,
                    "approach_speed": 0.3}}, "arguments"),
    ({"settings_synthetic": {"pallet_mass_kg": 4.93, "wheel_torque_nm": 2.0}}, "settings_synthetic"),
    ({"scene_layout": {"sha256": "other"}}, "scene_layout"),
    ({"encoder_model": None}, "encoder_model"),
    ({"seed": 3}, "seed"),
    ({"deterministic": {"enabled": True, "broken": ["frame_sync:no_rgba"]}}, "broken"),
    ({"failure_reason": "slam_link_failed: no reply"}, "infrastructure")])
def test_a_pair_outside_the_allowed_difference_or_with_an_infra_end_is_invalid(tmp_path, over, why):
    # Codex tool review P2: identity beyond five hashes, a missing field, the infra allowlist
    out = J.judge(tmp_path, _batch(tmp_path, **over))
    assert all(not p["valid"] and why in p["why"] for p in out["pairs"]), [p.get("why") for p in out["pairs"]]


def test_a_match_file_that_disagrees_with_the_round_drops_the_primary(tmp_path):
    # Codex tool review P2: e_m is checked against delivery_match_1.npz
    compare = _batch(tmp_path)
    f = tmp_path / "mae_B_line_2_s0" / "run" / "delivery_match_1.npz"
    z = dict(np.load(f))
    z["estimate"] = z["estimate"] + np.array([0.001, 0.0, 0.0])
    np.savez(f, **z)
    out = J.judge(tmp_path, compare)
    bad = [p for p in out["pairs"] if p["scene"] == "line_2"][0]
    assert not bad["valid"] and "delivery_match_1" in bad["why"]


def test_the_odometry_yaw_error_leaves_out_the_correction():
    # Codex tool review P2: a 0.1 rad correction with no motion is no odometry yaw error
    ts = np.arange(0.0, 1.0001, 0.01)
    est_yaw = np.where(ts >= 0.5, 0.1, 0.0)
    control = np.column_stack([ts, 0 * ts, 0 * ts, est_yaw, 0 * ts, 0 * ts, 0 * ts])
    events = [(0.5, "processed", (0.0, 0.0, 0.0), (0.0, 0.0, 0.1), None)]
    assert J.odometry_yaw_error(control, events, (0.0, 1.0)) == pytest.approx(0.0, abs=1e-12)
    assert J.odometry_yaw_error(control, [], (0.0, 1.0)) == pytest.approx(0.1)


def test_the_pre_match_row_is_strictly_before_the_match():
    # Codex tool review: t_p <= t_m passed every test; the overwritten row at t_m must not be read
    result, control, records, match = run_records()
    t_m = result["delivery_docking"]["rounds"][0]["time_s"]
    over = np.vstack([control, [[t_m, 5.0, 5.0, 0.0, *control[-1, 4:7]]]])
    sp = J.split(result, over, records, match)
    assert sp["t_p"] < t_m and abs(sp["e_new"]) < 1e-12


def test_j2_refuses_a_subset_mean_out_of_the_ratio_band():
    # the ratio bound alone: 8 blocks decomposed near -0.045 against a J1 mean of about -0.03 (ratio 1.5)
    pairs = []
    for s in range(10):
        for scene in J.SCENES:
            d = -0.045 - 0.0005 * s if s < 8 else +0.03
            pairs.append(_pair(scene, s, d, comps=(0.0, d, 0.0) if s < 8 else None))
    out = J.verdict(pairs, set())
    assert out["J1"] == "confirmed (this condition)", out["J1_stats"]
    j2 = out["J2"]
    assert j2["blocks"] == 8 and j2["m_dec"]["upper"] < 0 and not j2["representative"]


def test_replans_count_the_obstacle_layer_too():
    r = {"stall_replans": [{"time_s": 1.0}], "obstacle_layer": {"replans": [{"time_s": 2.0}, {"time_s": 9.0}]}}
    assert J.replans_before(r, 5.0) == 2


def test_phase_before_the_first_transition_is_its_from():
    r = {"transitions": [{"from": "observe", "to": "approach", "time_s": 5.0}]}
    assert J.phase_at(r, 1.0) == "observe" and J.phase_at(r, 6.0) == "approach"


def _odom_rec(i, t, status, odom, applied=C0, truth=None):
    r = _rec(i, t, status, "tracking", applied, applied)
    r["odom_base"] = list(odom)
    if truth is not None:
        r["truth_base"] = list(truth)
    return r


@pytest.mark.parametrize("path, expected", [
    ([(0.0, 0, 0), (0.3, 0, 0), (0.0, 0, 0)], "distance"),  # forward 0.3 then back 0.3: 0.6 m summed
    ([(0.0, 0, 0), (0.0, 0, 0.3), (0.0, 0, 0.0)], "turn"),  # +0.3 then -0.3 rad: 0.6 rad summed
    ([(0.0, 0, 0), (0.1, 0, 0), (0.2, 0, 0)], "neither")])
def test_the_gate_condition_sums_motion_scan_to_scan(path, expected):
    # Codex tool review 2 P2: the net displacement between two processed scans hid back-and-forth motion
    recs = [_odom_rec(1, 0.0, "processed", path[0]), _odom_rec(2, 0.3, "skipped", path[1]),
            _odom_rec(3, 0.6, "processed", path[2])]
    assert J.gate_condition(recs, 0.6) == expected
    recs[2]["stamp_s"] = 0.4
    assert J.gate_condition(recs, 0.4) == "short"


def test_a_failed_replay_leaves_the_event_measures_missing():
    # Codex tool review 2 P2: a replay inconsistency read as zero events and an uncorrected yaw error
    result, control, records, match = run_records()
    records[1]["applied_map_from_odom"] = list(C2)  # the replay cannot reach this
    sp = J.split(result, control, records, match)
    assert "replay" in sp["missing"]
    mech = J.mechanisms(result, control, records, None, None, sp)
    assert mech["transport.correction_events"] is None and mech["transport.odom_yaw_err_rad"] is None


def test_the_post_event_error_belongs_to_its_own_event():
    # Codex tool review 2 P2: two processed corrections before the next control row gave both the second's error
    truth = (1.0, 0.0, 0.0)
    recs = [_odom_rec(1, 0.10, "processed", (0.9, 0.0, 0.0), C0, truth),
            _odom_rec(2, 0.105, "processed", (0.9, 0.0, 0.0), (0.2, 0.0, 0.0), truth),
            _odom_rec(3, 0.108, "processed", (0.9, 0.0, 0.0), (0.4, 0.0, 0.0), truth)]
    events, bad = J.correction_events(recs, [])
    assert bad is None
    errs = [J.post_event_error(e, events, recs, np.zeros((0, 7)), -0.34) for e in events[1:]]
    assert errs == [pytest.approx(0.1), pytest.approx(0.3)]
    assert J.post_event_error(events[1], events, recs, np.zeros((0, 7)), None) is None  # no offset, no rear axle


def test_a_batch_whose_pairs_each_agree_but_differ_from_each_other_is_caught(tmp_path):
    # Codex tool review 2 P2: seed 1's pair on other code is consistent inside the pair, not with the batch
    compare = {f"{s}/{a}": True for s in J.SCENES for a in J.ARMS}
    for seed, src in ((0, "h"), (1, "h"), (2, "other")):
        for arm, kg in (("A", 10.0), ("B", 4.93)):
            for scene in J.SCENES:
                _write_run(tmp_path / f"mae_{arm}_{scene}_s{seed}", arm, kg, scene, seed=seed, source_sha256=src)
    out = J.judge(tmp_path, compare)
    odd = [p for p in out["pairs"] if p["seed"] == 2]
    assert odd and all(not p["valid"] and "batch snapshot" in p["why"] for p in odd)
    assert all(p["valid"] for p in out["pairs"] if p["seed"] != 2)


def test_adoption_differences_keep_position_and_yaw_apart():
    assert J.pose_diff([1.0, 2.0, 0.0], [1.0, 2.0, 0.1]) == {"position_m": 0.0, "yaw_rad": pytest.approx(0.1)}


def test_the_static_weight_is_compared_with_the_given_truck_mass(tmp_path):
    out = J.judge(tmp_path, _batch(tmp_path), truck_mass_kg=240.0 / 9.81)
    assert out["pairs"][0]["mech"]["A"]["static_weight_ratio"] == pytest.approx(1.0)


def test_the_post_event_error_is_a_rear_axle_error():
    # Codex tool review 3 P2: base_link and rear-axle errors differ when the yaw is off
    truth = (1.0, 0.0, 0.0)
    rec = _odom_rec(2, 0.2, "processed", (1.0, 0.0, 0.2), (0.0, 0.0, 0.0), truth)
    ev = (0.2, "processed", (0.0, 0.0, 0.1), (0.0, 0.0, 0.0), None)
    got = J.post_event_error(ev, [ev], [rec], np.zeros((0, 7)), -0.4)
    expected = J.along(J.rear_of(truth, 0.4), J.rear_of((1.0, 0.0, 0.2), 0.4))
    assert got == pytest.approx(expected) and abs(got) > 1e-3


def test_the_hausdorff_distance_is_exact():
    # Codex tool review 3 P2: subsampling turned 0.095 m into 0.102 m and flipped the same-path call
    x = np.linspace(0.0, 10.0, 5001)
    a = np.column_stack([x, 0 * x])
    b = np.column_stack([x[::-1], 0 * x + 0.095])
    assert J.hausdorff(a, b) == pytest.approx(0.095, abs=1e-12)


def test_the_batch_checks_the_scene_hashes_and_refuses_a_tied_reference(tmp_path):
    compare = {f"{s}/{a}": True for s in J.SCENES for a in J.ARMS}
    for seed, over in ((0, {}), (1, {}), (2, {"pallet_urdf_sha256": "x"}), (3, {"scene_layout": {"sha256": "M"}})):
        for arm, kg in (("A", 10.0), ("B", 4.93)):
            for scene in J.SCENES:
                _write_run(tmp_path / f"mae_{arm}_{scene}_s{seed}", arm, kg, scene, seed=seed, **over)
    out = J.judge(tmp_path, compare)
    by = {(p["scene"], p["seed"]): p for p in out["pairs"]}
    assert by[("line_2", 2)]["why"].startswith("A: batch snapshot differs") and "pallet_urdf_sha256" in by[("line_2", 2)]["why"]
    assert "scene_layout:line_2" in by[("line_2", 3)]["why"]
    assert by[("line_2", 0)]["valid"] and by[("random_1", 1)]["valid"]
    assert J._mode(["a", "b"]) == J.TIED and J._mode(["a", "a", "b"]) == "a"


def test_the_gate_distance_is_the_gates_own():
    assert J.GATE_DISTANCE_M == pytest.approx(0.4472136, abs=1e-7)


def test_the_plant_may_differ_between_scenes_and_the_encoder_between_seeds(tmp_path):
    # plan amendment 3 (the real batch, validity only): the plant record lists the scene's props and the encoder model
    # carries the seed's phases, so neither is one value across the batch; each is still checked within its group
    compare = {f"{s}/{a}": True for s in J.SCENES for a in J.ARMS}
    for seed in (0, 1, 2):
        for arm, kg in (("A", 10.0), ("B", 4.93)):
            for scene in J.SCENES:
                _write_run(tmp_path / f"mae_{arm}_{scene}_s{seed}", arm, kg, scene, seed=seed,
                           plant={"materials": [scene]}, encoder_model={"phases_rad": [seed]})
    out = J.judge(tmp_path, compare)
    assert all(p["valid"] for p in out["pairs"]), [p.get("why") for p in out["pairs"]]
    _write_run(tmp_path / "mae_A_line_2_s3", "A", 10.0, "line_2", seed=3, plant={"materials": ["other"]},
               encoder_model={"phases_rad": [3]})
    _write_run(tmp_path / "mae_B_line_2_s3", "B", 4.93, "line_2", seed=3, plant={"materials": ["other"]},
               encoder_model={"phases_rad": [3]})
    out = J.judge(tmp_path, compare)
    bad = [p for p in out["pairs"] if p["seed"] == 3][0]
    assert not bad["valid"] and "plant:line_2" in bad["why"]
