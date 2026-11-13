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
    samples = [{"time_s": t, "phase": ph, "pallet_position_m": V + [0.0], "pallet_yaw_rad": angle}
               for t, ph in ((4.0, "approach"), (5.1, "approach"), (12.0, "insert"), (20.0, "insert"), (20.1, "lift"),
                             (21.0, "lift"))]
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


EXPECTED = {"S": -0.020, "T": -0.004, "F": -0.006, "L": -0.010, "D": -0.001, "C": -0.003, "P": 0.0}


@pytest.mark.parametrize("angle", [0.0, math.pi / 3, -2.5])
def test_the_split_matches_the_constructed_offsets_in_any_world_frame(angle):
    d = R.decompose(*world(angle))
    for k, v in EXPECTED.items():
        assert d["terms_m"][k] == pytest.approx(v, abs=1e-12), k
    assert d["C_plus_L_m"] == pytest.approx(-0.013, abs=1e-12)
    assert d["qe_vs_near_estimate_m"] == pytest.approx(0.0, abs=1e-12)
    assert all(abs(v) < 1e-12 for v in d["residuals_m"].values()) and len(d["residuals_m"]) == 3
    assert d["pallet_motion"]["from_t"] == 5.1 and d["pallet_motion"]["to_t"] == 20.1
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
    def moved(r, sc):
        r["samples"][3]["pallet_position_m"] = [2.0, 0.0006, 0.0]
    assert "pallet_moved" in hold_of(moved)["hold"]

    def turned(r, sc):
        r["samples"][4]["pallet_yaw_rad"] = 0.0011
    assert "pallet_moved" in hold_of(turned)["hold"]

    def moved_before_capture(r, sc):  # before the capture's own time: outside the window
        r["samples"][0]["pallet_position_m"] = [2.0, 0.01, 0.0]
    assert hold_of(moved_before_capture)["hold"] == []

    def replaced(r, sc):
        r["tracker_versions"][1]["replaced_s"] = 15.0
        r["tracker_versions"].append({"phase": "insert", "installed_s": 15.0, "replaced_s": None})
    assert hold_of(replaced)["hold"] == ["insert_tracker_replaced"]

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
    assert R.judge(d)["status"] == "held"

    def no_seat(r, sc):
        del r["slip_watch"]
    d = hold_of(no_seat)
    assert d["missing"] == ["seat"] and "missing_terms" in d["hold"] and "T_minus_DCP" in d["residuals_m"]


def test_a_run_without_lift_is_outside_the_data_not_held():
    result, sc = world()
    result["transitions"] = result["transitions"][:2]
    d = R.decompose(result, sc)
    assert d["reached_lift"] is False and d["hold"] == []
    assert R.judge(d) == {"selected": None, "status": "not_reached_lift"}
    runs = [dict(d, arm="Bb10"), dict(R.decompose(*world()), arm="B0")]
    t = R.tally(runs, 0.010)
    assert t["perturbed"]["not_reached_lift"] == 1 and t["perturbed"]["held"] == 0 and t["perturbed"]["selected"] == 0
    assert t["B0"]["judged"] == 1 and t["B0"]["largest_same_sign"] == {"L": 1}
    assert t["B0"]["terms_m"]["C+L"]["median"] == pytest.approx(-0.013)


def test_the_folders_are_read_and_written(tmp_path):
    raw = tmp_path / "raw"
    for name, angle in (("lbg_B0_random_1_s1", 0.0), ("dg_Pn_aisle_rack_s12", 1.0)):
        run = raw / name / "run"
        run.mkdir(parents=True)
        result, sc = world(angle)
        (run / "result.json").write_text(json.dumps(result))
        np.save(run / "slam_control.npy", sc)
    out, summary = tmp_path / "d.json", tmp_path / "d.md"
    assert R.main([str(raw), "--out", str(out), "--summary", str(summary)]) == 0
    payload = json.loads(out.read_text())
    names = {r["name"]: r for r in payload["runs"]}
    assert names["dg_Pn_aisle_rack_s12"]["arm"] == "Pn" and names["dg_Pn_aisle_rack_s12"]["scene"] == "aisle_rack"
    assert names["dg_Pn_aisle_rack_s12"]["seed"] == 12
    assert set(payload["tallies"]) == {"5mm", "10mm", "15mm"}
    assert "| lbg_B0_random_1_s1 | -20.0 | -4.0 | -1.0 | -3.0 | +0.0 | -6.0 | -10.0 | -13.0 | -20.0 | judged: L" \
        in summary.read_text()


def test_the_input_is_not_changed():
    result, sc = world()
    before, sc_before = copy.deepcopy(result), sc.copy()
    R.decompose(result, sc)
    assert result == before and np.array_equal(sc, sc_before)
