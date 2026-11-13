"""Baseline fragility priority 1 (docs/plans/2026-11-13-baseline-fragility.md 1, 3, 3.1): the seat offset's split
S = T + F + L, T = D + C + P from saved records, the hold rules and the tally. Synthetic records, no Isaac."""

import copy
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    "seat_decompose", ROOT / "docs/validation/2026-11-13-baseline-fragility/seat_decompose.py")
R = importlib.util.module_from_spec(spec)
spec.loader.exec_module(R)


def rotate(pose, angle):
    """A planar pose or point rotated about the world origin."""
    c, s = math.cos(angle), math.sin(angle)
    out = [c * pose[0] - s * pose[1], s * pose[0] + c * pose[1]]
    return out + [pose[2] + angle] if len(pose) > 2 else out


def world(angle=0.0):
    """Pallet origin V at (2, 0); the insert path ends 4 mm right of it; the estimate 10 mm, the truth 20 mm right at
    lift; the capture's estimate 3 mm left of the truth and the detector 1 mm left of the pallet. Expected (m):
    T -0.004, F -0.006, L -0.010, S -0.020, D -0.001, C -0.003, P 0."""
    V = rotate([2.0, 0.0], angle)
    G = rotate([0.5, 0.004, 0.0], angle)
    E = rotate([0.5, 0.010, 0.0], angle)
    A = rotate([0.5, 0.020, 0.0], angle)
    truth_rear = rotate([-1.0, 0.0, 0.0], angle)
    est_rear = rotate([-1.0, 0.003, 0.0], angle)
    # base_link (0.34 ahead): truth (-0.66, 0), estimate (-0.66, 0.003); the observed centre (2.66, 0.001) in base_link
    # is the pocket front midpoint (2.36, 0.001) moved half of a 0.6 m depth along yaw 0.
    observation = {"status": "valid", "insertion_yaw_rad": 0.0,
                   "left": {"center_m": [2.36, 0.191, 0.06]}, "right": {"center_m": [2.36, -0.189, 0.06]}}
    Qe = rotate([2.0, 0.004], angle)
    # 0.1 s samples (as recorded): approach to 10 s, insert to 20.05 s, then lift
    samples = [{"time_s": round(0.1 * i + 0.01, 2), "phase": "approach" if i < 100 else "insert" if i < 201 else "lift",
                "pallet_position_m": V + [0.0], "pallet_yaw_rad": angle} for i in range(215)]
    result = {
        "transitions": [{"to": "approach", "time_s": 1.0}, {"to": "insert", "time_s": 10.0},
                        {"to": "lift", "time_s": 20.05}],
        "samples": samples,
        "paths": {"insert": {"poses": [rotate([0.0, 0.004, 0.0], angle), G]}},
        "slip_watch": {"baseline": {"seat": [1.0, -0.020, 0.0], "t": 20.05}},
        "near_capture": {"status": "done", "near_attempt": 2, "near_estimate_m": {"x_m": Qe[0], "y_m": Qe[1]}},
        "observation_attempts": [
            {"pocket_observation": None},
            {"pocket_observation": observation,
             "capture_control": {"time_s": 5.0, "control_rear": est_rear, "truth_rear": truth_rear}},
        ],
        "arguments": {"rear_axle_offset_m": -0.34},
        "geometry": {"pallet_depth_m": 0.6},
        "tracker_versions": [{"phase": "insert", "installed_s": 0.5, "replaced_s": 5.0},
                             {"phase": "insert", "installed_s": 5.0, "replaced_s": None},
                             {"phase": "approach", "installed_s": 5.0, "replaced_s": None}],
    }
    sc = np.array([[t, *E, *A] for t in (19.95, 20.05, 20.15)])
    return result, sc


EXPECTED = {"S": -0.020, "T": -0.004, "F": -0.006, "L": -0.010, "D": -0.001, "C": -0.003, "P": 0.0, "C+L": -0.013}


@pytest.mark.parametrize("angle", [0.0, math.pi / 3, -2.5])
def test_the_split_matches_the_constructed_offsets_in_any_world_frame(angle):
    d = R.decompose(*world(angle))
    for k, v in EXPECTED.items():
        assert d["terms_m"][k] == pytest.approx(v, abs=1e-12), k
    assert d["qe_vs_near_estimate_m"] == pytest.approx(0.0, abs=1e-12)
    assert all(abs(v) < 1e-12 for v in d["residuals_m"].values()) and len(d["residuals_m"]) == 3
    motion = d["pallet_motion"]
    assert motion["covered"] is True and motion["from_t"] == 5.01 and motion["to_t"] == 20.11
    assert d["insert_trackers"] == 1 and d["hold"] == [] and d["reached_lift"] is True


def test_the_contribution_names_the_largest_same_sign_term_and_the_cancelling_ones():
    j = R.judge(R.decompose(*world()))
    assert j["status"] == "judged" and j["leaf"]["largest_same_sign"] == "L" and j["leaf"]["cancelling"] == {}
    terms = {"S": -0.015, "D": 0.003, "C": 0.050, "P": 0.0, "F": -0.002, "L": -0.066, "T": 0.053}
    c = R.contributions(terms, R.LEAVES)
    assert c["largest_same_sign"] == "L" and set(c["cancelling"]) == {"D", "C"}
    assert R.contributions(terms, R.TOPS)["cancelling"] == {"T": 0.053}
    tiny = dict(terms, P=-5e-5, D=-9e-5)  # under the 0.1 mm resolution: neither same-sign nor cancelling
    assert set(R.contributions(tiny, R.LEAVES)["cancelling"]) == {"C"}
    assert R.contributions(dict(terms, L=-5e-5, F=-5e-5), R.LEAVES)["largest_same_sign"] is None


def test_the_selection_value_and_its_sensitivity():
    d = R.decompose(*world())  # |S| = 20 mm
    assert R.judge(d, 0.015)["status"] == "judged"
    assert R.judge(d, 0.025) == {"selected": False, "status": "not_selected"}


def hold_of(edit):
    result, sc = world()
    edit(result, sc)
    return R.decompose(result, sc)


def test_holds():
    def moved(r, sc):  # 15.01 s, during insert
        r["samples"][150]["pallet_position_m"] = [2.0, 0.0006, 0.0]
    assert hold_of(moved)["hold"] == ["pallet_moved"]

    def turned(r, sc):  # 20.11 s, the first lift sample
        r["samples"][201]["pallet_yaw_rad"] = 0.0011
    assert hold_of(turned)["hold"] == ["pallet_moved"]

    def moved_before_capture(r, sc):  # 4.01 s, before the capture's own time: outside the window
        r["samples"][40]["pallet_position_m"] = [2.0, 0.01, 0.0]
    assert hold_of(moved_before_capture)["hold"] == []

    def no_capture_time(r, sc):  # Codex review P2: no capture time must not skip the motion check
        del r["observation_attempts"][1]["capture_control"]["time_s"]
    d = hold_of(no_capture_time)
    assert d["missing"] == ["capture_time"] and d["hold"] == ["missing_terms"] and R.judge(d)["status"] == "held"

    def sparse(r, sc):  # only the lift samples left: no evidence the pallet stood still
        r["samples"] = r["samples"][201:]
    assert hold_of(sparse)["hold"] == ["insufficient_motion_coverage"]

    def gap(r, sc):
        del r["samples"][120:125]
    assert hold_of(gap)["hold"] == ["insufficient_motion_coverage"]

    def gap_of_exactly_two_tenths(r, sc):  # Codex review round 2 P3: 5.81 -> 6.01 s is 0.2 s up to float error
        del r["samples"][59]
    assert hold_of(gap_of_exactly_two_tenths)["hold"] == []

    def nan_sample(r, sc):  # Codex review round 2 P2: a NaN pallet sample mid-window
        r["samples"][150]["pallet_yaw_rad"] = float("nan")
    assert hold_of(nan_sample)["hold"] == ["nonfinite_terms"]

    def nan_sample_time(r, sc):  # Codex review round 3 P2: a NaN time must not drop the sample before the check
        r["samples"][150]["time_s"] = float("nan")
        r["samples"][150]["pallet_position_m"] = [2.0, 0.0006, 0.0]
    assert hold_of(nan_sample_time)["hold"] == ["nonfinite_terms"]

    def nan_sample_xy(r, sc):
        r["samples"][150]["pallet_position_m"] = [float("nan"), 0.0, 0.0]
    assert hold_of(nan_sample_xy)["hold"] == ["nonfinite_terms"]

    def inf_lift_yaw(r, sc):  # Codex review round 2 P2: an infinite yaw must hold, not raise
        sc[:, 3] = np.inf
    d = hold_of(inf_lift_yaw)
    assert d["missing"] == ["nonfinite_lift_inputs"] and "nonfinite_terms" in d["hold"]
    assert R.judge(d)["status"] == "held"

    def inf_capture_yaw(r, sc):
        r["observation_attempts"][1]["capture_control"]["truth_rear"][2] = float("inf")
    d = hold_of(inf_capture_yaw)
    assert d["missing"] == ["nonfinite_capture_inputs"] and "nonfinite_terms" in d["hold"] and "D" not in d["terms_m"]

    def slam_shifted(r, sc):  # Codex review P2: the nearest row 1000 s off is not the lift row
        sc[:, 0] -= 1000.0
    d = hold_of(slam_shifted)
    assert d["missing"] == ["slam_row_at_lift"] and d["hold"] == ["missing_terms"]

    def slam_bad_shape(r, sc):
        sc.resize((0, 7), refcheck=False)
    assert hold_of(slam_bad_shape)["missing"] == ["slam_row_at_lift"]

    def seat_time(r, sc):
        r["slip_watch"]["baseline"]["t"] = 19.0
    assert hold_of(seat_time)["hold"] == ["seat_time_mismatch"]

    def nan_estimate(r, sc):  # Codex review P2: NaN passes a > comparison
        sc[:, 1] = np.nan
    d = hold_of(nan_estimate)
    assert d["hold"] == ["nonfinite_terms"]
    assert d["missing"] == ["nonfinite_lift_inputs"] and "terms_m" not in d and R.judge(d)["status"] == "held"

    def nan_seat(r, sc):  # a non-finite seat passes the inputs but not the residuals
        r["slip_watch"]["baseline"]["seat"][1] = float("nan")
    d = hold_of(nan_seat)
    assert d["hold"] == ["nonfinite_terms"] and d["residuals_m"]["seat_minus_S"] is None

    def replaced(r, sc):
        r["tracker_versions"][1]["replaced_s"] = 15.0
        r["tracker_versions"].append({"phase": "insert", "installed_s": 15.0, "replaced_s": None})
    assert hold_of(replaced)["hold"] == ["insert_tracker_replaced"]

    def empty_version(r, sc):  # Codex review P3: a version replaced at its own install was never active
        r["tracker_versions"].append({"phase": "insert", "installed_s": 15.0, "replaced_s": 15.0})
    assert hold_of(empty_version)["hold"] == []

    def seat_off(r, sc):  # 3 mm against a bound of max(2 mm, 2 mm)
        r["slip_watch"]["baseline"]["seat"][1] = -0.017
    d = hold_of(seat_off)
    assert d["hold"] == ["residual"] and d["residual_bound_m"] == pytest.approx(0.002)

    def seat_slightly_off(r, sc):
        r["slip_watch"]["baseline"]["seat"][1] = -0.0185
    assert hold_of(seat_slightly_off)["hold"] == []

    def estimate_off(r, sc):
        r["near_capture"]["near_estimate_m"]["y_m"] += 1e-4
    assert hold_of(estimate_off)["hold"] == ["qe_mismatch"]

    def no_capture(r, sc):
        del r["observation_attempts"][1]["capture_control"]
    d = hold_of(no_capture)
    assert d["missing"] == ["capture_control"] and d["hold"] == ["missing_terms"] and "D" not in d["terms_m"]

    def no_seat(r, sc):
        del r["slip_watch"]
    d = hold_of(no_seat)
    assert d["missing"] == ["seat"] and d["hold"] == ["missing_terms"] and "T_minus_DCP" in d["residuals_m"]


def test_runs_without_lift_are_outside_the_data_and_holding_is_apart_from_selection():
    result, sc = world()
    result["transitions"] = result["transitions"][:2]
    no_lift = R.decompose(result, sc)
    assert no_lift["reached_lift"] is False and no_lift["hold"] == []
    assert R.judge(no_lift) == {"selected": None, "status": "not_reached_lift"}
    result["transitions"] = result["transitions"][:1]
    no_insert = R.decompose(result, sc)
    assert R.judge(no_insert)["status"] == "not_reached_insert"

    def no_capture(r, sc):
        del r["observation_attempts"][1]["capture_control"]
    held = hold_of(no_capture)  # |S| = 20 mm, still known
    assert R.judge(held, 0.025)["selected"] is False and R.judge(held, 0.010)["selected"] is True
    runs = [dict(no_lift, arm="Bb10"), dict(no_insert, arm="Pn"), dict(held, arm="Ub3"),
            dict(R.decompose(*world()), arm="B0")]
    t = R.tally(runs, 0.010)
    p = t["perturbed"]
    assert (p["not_reached_lift"], p["not_reached_insert"], p["held"], p["held_selected"], p["eligible"]) == \
        (1, 1, 1, 1, 0)
    assert p["held_reasons"] == {"missing_terms": 1}
    assert R.tally(runs, 0.025)["perturbed"]["held_selected"] == 0
    assert t["B0"]["judged"] == 1 and t["B0"]["largest_same_sign"] == {"L": 1}
    assert t["B0"]["terms_m"]["C+L"]["median"] == pytest.approx(-0.013)
    small = R.tally([dict(R.decompose(*world()), arm="B0")], 0.025)["B0"]
    assert small["eligible"] == 1 and small["judged"] == 0


def test_c_plus_l_is_not_a_bias_change_by_itself():
    """Codex review P3: one constant world bias b at the capture and at lift, the estimate's yaw at lift 0.015 rad off
    the path end's, gives C + L = (n_E - n_G).b -- non-zero with no change of the bias."""
    b, dyaw = (0.040, 0.060), 0.015
    result, sc = world()
    attempt = result["observation_attempts"][1]
    cap = attempt["capture_control"]
    cap["control_rear"] = [cap["truth_rear"][0] + b[0], cap["truth_rear"][1] + b[1], 0.0]
    est = R.to_world(R.observed_centre(attempt["pocket_observation"], 0.6), R.base_from_rear(cap["control_rear"], -0.34))
    result["near_capture"]["near_estimate_m"] = {"x_m": est[0], "y_m": est[1]}
    A = [0.5, 0.020, dyaw]
    sc[:, 4:7] = A
    sc[:, 1:4] = [A[0] + b[0], A[1] + b[1], dyaw]
    result["slip_watch"]["baseline"]["seat"][1] = R.lateral([2.0, 0.0], A)
    d = R.decompose(result, sc)
    n_E, n_G = (-math.sin(dyaw), math.cos(dyaw)), (0.0, 1.0)
    expected = (n_E[0] - n_G[0]) * b[0] + (n_E[1] - n_G[1]) * b[1]
    assert d["hold"] == [] and d["terms_m"]["C+L"] == pytest.approx(expected, abs=1e-12)
    assert abs(expected) > 5e-4
    # the bias did not change: dB is zero and S is all S_const
    assert d["terms_m"]["dB"] == pytest.approx(0.0, abs=1e-12)
    assert d["terms_m"]["S_const"] == pytest.approx(d["terms_m"]["S"], abs=1e-12)


def test_db_is_the_lateral_effect_of_the_bias_change():
    """The fixture's estimate is 3 mm left of the truth at the capture (W_c a 3 mm shift) and 10 mm left at lift
    (E - A = -10 mm lateral in the frame, i.e. the estimate is 10 mm right): the bias changed by 13 mm, all lateral,
    with no yaw, so dB = C + L = -13 mm and S_const = S - dB = -7 mm."""
    d = R.decompose(*world())
    assert d["terms_m"]["dB"] == pytest.approx(-0.013, abs=1e-12)
    assert d["terms_m"]["S_const"] == pytest.approx(-0.007, abs=1e-12)
    course = d["bias_course"]  # the fixture's slam rows start at 19.95 s: the course covers only those
    assert course["rows"] == 2 and course["dpsi_rad"] == pytest.approx(0.0, abs=1e-12)
    assert course["dB_max_step_m"] == pytest.approx(0.0, abs=1e-12)


def test_the_course_shows_a_jump_and_where_it_happened():
    """Rows every 0.1 s from the capture: the truth at lift's pose throughout, the estimate at the capture's bias
    (3 mm left) until 15 s, then 10 mm right -- one 13 mm step during insert."""
    result, sc = world()
    t = np.round(np.arange(5.0, 20.06, 0.05), 2)
    A = np.array([0.5, 0.020, 0.0])
    est = np.where(t[:, None] < 15.0, A + [0.0, 0.003, 0.0], A + [0.0, -0.010, 0.0])
    cap = result["observation_attempts"][1]["capture_control"]
    cap["truth_rear"], cap["control_rear"] = list(A), list(A + [0.0, 0.003, 0.0])
    q = R.to_world(R.observed_centre(result["observation_attempts"][1]["pocket_observation"], 0.6),
                   R.base_from_rear(cap["control_rear"], -0.34))
    result["near_capture"]["near_estimate_m"] = {"x_m": q[0], "y_m": q[1]}
    sc = np.column_stack([t, est, np.tile(A, (len(t), 1))])
    result["slip_watch"]["baseline"]["t"] = float(t[-1])
    result["transitions"][-1]["time_s"] = float(t[-1])
    d = R.decompose(result, sc)
    c = d["bias_course"]
    assert c["dB_at_capture_m"] == pytest.approx(0.0, abs=1e-12)
    assert c["dB_at_insert_m"] == pytest.approx(0.0, abs=1e-12)
    assert c["dB_max_step_m"] == pytest.approx(0.013, abs=1e-12)
    assert d["terms_m"]["dB"] == pytest.approx(-0.013, abs=1e-12)


def test_the_course_reports_the_yaw_of_the_bias_change():
    result, sc = world()
    sc[:, 3] += 0.01  # the estimate turns 0.01 rad more than the truth by lift
    d = R.decompose(result, sc)
    assert d["bias_course"]["dpsi_rad"] == pytest.approx(0.01, abs=1e-12)


def test_db_with_a_rotated_constant_bias_is_zero():
    """A constant rigid bias with a rotation (2 deg about a far point) at the capture and at lift: dB = 0."""
    result, sc = world()
    W = R.se2([0.3, -0.2, math.radians(2.0)])
    cap = result["observation_attempts"][1]["capture_control"]
    cap["control_rear"] = list(R.pose_of(W @ R.se2(cap["truth_rear"])))
    attempt = result["observation_attempts"][1]
    est = R.to_world(R.observed_centre(attempt["pocket_observation"], 0.6), R.base_from_rear(cap["control_rear"], -0.34))
    result["near_capture"]["near_estimate_m"] = {"x_m": est[0], "y_m": est[1]}
    sc[:, 1:4] = R.pose_of(W @ R.se2(sc[1, 4:7]))
    d = R.decompose(result, sc)
    assert d["hold"] == [] and d["terms_m"]["dB"] == pytest.approx(0.0, abs=1e-12)


def test_the_folders_are_read_and_written(tmp_path):
    raw = tmp_path / "raw"
    for name, angle in (("lbg_B0_random_1_s1", 0.0), ("dg_Pn_aisle_rack_s12", 1.0)):
        run = raw / name / "run"
        run.mkdir(parents=True)
        result, sc = world(angle)
        (run / "result.json").write_text(json.dumps(result))
        np.save(run / "slam_control.npy", sc)
    bad = raw / "lbg_Ub3_random_1_s0" / "run"
    bad.mkdir(parents=True)
    result, sc = world()
    sc[:, 1] = np.nan
    (bad / "result.json").write_text(json.dumps(result))
    np.save(bad / "slam_control.npy", sc)
    out, summary = tmp_path / "d.json", tmp_path / "d.md"
    assert R.main([str(raw), "--out", str(out), "--summary", str(summary)]) == 0
    payload = json.loads(out.read_text())  # standard JSON: the NaN run's terms are null
    names = {r["name"]: r for r in payload["runs"]}
    assert names["dg_Pn_aisle_rack_s12"]["arm"] == "Pn" and names["dg_Pn_aisle_rack_s12"]["scene"] == "aisle_rack"
    assert names["dg_Pn_aisle_rack_s12"]["seed"] == 12
    assert names["lbg_Ub3_random_1_s0"]["missing"] == ["nonfinite_lift_inputs"]
    assert names["lbg_Ub3_random_1_s0"]["judgement"]["status"] == "held"
    assert set(payload["tallies"]) == {"5mm", "10mm", "15mm"}
    assert "| lbg_B0_random_1_s1 | -20.0 | -4.0 | -1.0 | -3.0 | +0.0 | -6.0 | -10.0 | -13.0 | -13.0 | -20.0 | judged: L" \
        in summary.read_text()


def test_the_input_is_not_changed():
    result, sc = world()
    before, sc_before = copy.deepcopy(result), sc.copy()
    R.decompose(result, sc)
    assert result == before and np.array_equal(sc, sc_before)


def test_repeats_and_canaries_are_compared_with_their_original_and_not_counted_twice(tmp_path):
    check, diag = tmp_path / "check", tmp_path / "diag"
    for root, name, shift in ((check, "lbg_B0_random_1_s0", 0.0), (check, "lbg_B0_random_1_s0_rep", 0.0),
                              (diag, "lbd_B0_random_1_s0_can", 0.0005), (check, "lbg_Ub6_line_2_s0_rep", 0.0)):
        run = root / name / "run"
        run.mkdir(parents=True)
        result, sc = world()
        sc[:, 5] += shift  # the canary's truth rear 0.5 mm further left at lift
        result["slip_watch"]["baseline"]["seat"][1] = R.lateral([2.0, 0.0], sc[1, 4:7])
        (run / "result.json").write_text(json.dumps(result))
        np.save(run / "slam_control.npy", sc)
    live = diag / "lbd_live_s0" / "run"  # no slam_control: held as missing, not a crash
    live.mkdir(parents=True)
    (live / "result.json").write_text(json.dumps(world()[0]))
    runs = {r["name"]: r for r in R.collect([check, diag])}
    rep, can = runs["lbg_B0_random_1_s0_rep"], runs["lbd_B0_random_1_s0_can"]
    assert R.parse_name("lbd_B0_aisle_rack_s2_can") == {"arm": "B0", "scene": "aisle_rack", "seed": 2,
                                                        "duplicate": "can", "rerun": False}
    assert rep["duplicate_of"] == "lbg_B0_random_1_s0" and rep["duplicate_max_term_diff_m"] == 0.0
    assert can["duplicate_of"] == "lbg_B0_random_1_s0"
    assert can["duplicate_max_term_diff_m"] == pytest.approx(0.0005) and can["duplicate_status_match"] is True
    assert runs["lbg_Ub6_line_2_s0_rep"]["duplicate_max_term_diff_m"] is None  # its original is not in the data
    assert rep["judgement"]["status"] == "duplicate"
    assert runs["lbd_live_s0"]["missing"] == ["slam_row_at_lift"]
    t = R.tally(list(runs.values()), 0.010)
    assert t["B0"]["duplicates"] == 2 and t["B0"]["judged"] == 1 and t["perturbed"]["duplicates"] == 1


def test_a_resubmission_replaces_its_invalid_original(tmp_path):
    root = tmp_path / "check"
    for name in ("lbg_Ub6_aisle_rack_s0", "lat_r_lbg_Ub6_aisle_rack_s0"):
        run = root / name / "run"
        run.mkdir(parents=True)
        result, sc = world()
        (run / "result.json").write_text(json.dumps(result))
        np.save(run / "slam_control.npy", sc)
    runs = {r["name"]: r for r in R.collect([root])}
    rerun, orig = runs["lat_r_lbg_Ub6_aisle_rack_s0"], runs["lbg_Ub6_aisle_rack_s0"]
    assert (rerun["arm"], rerun["scene"], rerun["seed"], rerun["rerun"]) == ("Ub6", "aisle_rack", 0, True)
    assert orig["judgement"]["status"] == "superseded" and rerun["judgement"]["status"] == "judged"
    t = R.tally(list(runs.values()), 0.010)["perturbed"]
    assert (t["superseded"], t["judged"]) == (1, 1)
