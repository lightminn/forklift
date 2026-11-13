"""Near-capture rejection classification (docs/validation/2026-11-13-baseline-fragility/near_reject_classify.py).
Synthetic records, no Isaac."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    "near_reject_classify", ROOT / "docs/validation/2026-11-13-baseline-fragility/near_reject_classify.py")
N = importlib.util.module_from_spec(spec)
spec.loader.exec_module(N)


def result(reason, *, candidates, offsets=None, redraw=None, site=(3.0, 0.0)):
    """Truck rear at the origin facing +x, base 0.34 ahead; the true pallet centre 3.34 m ahead of the base."""
    nc = {"status": "failed", "waypoint": [0.0, 0.0, 0.0]}
    if offsets:
        nc.update(offsets_to_new_line=offsets, box={"lateral_m": 0.05, "yaw_rad": 0.05, "length_m": 0.3})
    if redraw:
        nc["recoveries"] = [redraw]
    return {"failure_reason": reason, "arguments": {"rear_axle_offset_m": -0.34},
            "scenario": {"pickup": {"x_m": 3.34, "y_m": 0.0, "yaw_rad": 0.0}}, "near_capture": nc,
            "observation_attempts": [{"capture_control": {"truth_rear": [0.0, 0.0, 0.0]}, "arrived_pose": [0, 0, 0.0],
                                      "expected": {"site": list(site), "radius_m": 0.15, "candidates": candidates}}]}


def test_a_wrong_object_just_outside_the_radius_is_perception_by_mm():
    r = result("near_capture_failed:perception_no_pallet:no_expected_candidate",
               candidates=[{"centre_m": [3.0, 0.153], "distance_m": 0.153}])
    c = N.classify("lbg_B0_aisle_rack_s2", r)
    assert c["ended_at_near"] and c["label"] == "perception" and c["by_mm"]
    assert abs(c["site_vs_truth_m"]) < 1e-12 and abs(c["candidate_vs_truth_min_m"] - 0.153) < 1e-12


def test_a_good_detection_with_the_truck_off_the_line_is_position():
    r = result("near_capture_misaligned:{...}", candidates=[{"centre_m": [3.0, 0.001], "distance_m": 0.001}],
               offsets={"lateral_m": -0.107, "yaw_rad": 0.09},
               redraw={"stage": "redraw", "accepted": False, "max_lateral_m": 0.1, "record": {"refused": "box"}})
    c = N.classify("lbd_B0_random_1_s3", r)
    assert c["label"] == "position" and c["by_mm"] and c["redraw"]["refused"] == "box"
    far = result("near_capture_misaligned:{...}", candidates=[{"centre_m": [3.0, 0.001], "distance_m": 0.001}],
                 offsets={"lateral_m": -0.2, "yaw_rad": 0.09},
                 redraw={"stage": "redraw", "accepted": False, "max_lateral_m": 0.1, "record": {"refused": "box"}})
    assert N.classify("x_B0_random_1_s0", far)["by_mm"] is False
    exact = result("near_capture_misaligned:{...}", candidates=[{"centre_m": [3.0, 0.0], "distance_m": 0.0}],
                   offsets={"lateral_m": -0.107, "yaw_rad": 0.09},
                   redraw={"stage": "redraw", "accepted": False, "max_lateral_m": 0.1, "record": {"refused": "box"}})
    assert N.classify("x_B0_random_1_s0", exact)["label"] == "position"  # Codex P2: an exact 0.0 is not missing


def test_the_arrival_history_and_the_median():
    r = result(None, candidates=[{"centre_m": [3.0, 0.0], "distance_m": 0.0}])
    r["near_end_stand"] = {"since_s": 1.0}
    r["observe_loose_arrivals"] = [{"time_s": 0.0, "position_error_m": 0.06, "yaw_error_rad": -0.09}]
    r["observe_replans"] = [{"trigger": "near_end_stand"}, {"status": "success"}]
    r["observation_attempts"][0]["t_before_capture_s"] = 5.0
    c = N.classify("lbg_B0_random_1_s0", r)
    assert c["near_end_stand_waiting"] and c["loose_arrival"] and c["loose_yaw_error_rad"] == -0.09
    assert c["near_end_stand_replans"] == 1
    assert N.median([1, 2, 3, 4]) == 2.5 and N.median([3, 1, 2]) == 2


def test_a_loose_arrival_at_the_far_waypoint_is_not_counted():
    r = result(None, candidates=[{"centre_m": [3.0, 0.0], "distance_m": 0.0}])
    r["near_capture"]["far_attempt"] = 1
    far = {"t_before_capture_s": 10.0, "capture_control": {"truth_rear": [0.0, 0.0, 0.0]}}
    r["observation_attempts"] = [far] + r["observation_attempts"]
    r["observation_attempts"][1]["t_before_capture_s"] = 20.0
    r["observe_loose_arrivals"] = [{"time_s": 9.5, "position_error_m": 0.05, "yaw_error_rad": 0.05}]
    assert N.classify("lbd_Pn_random_1_s10", r)["loose_arrival"] is False
    r["observe_loose_arrivals"].append({"time_s": 19.0, "position_error_m": 0.06, "yaw_error_rad": -0.09})
    c = N.classify("lbd_Pn_random_1_s10", r)
    assert c["loose_arrival"] and c["loose_yaw_error_rad"] == -0.09


def test_a_passing_run_is_not_labelled_and_scenes_parse():
    r = result(None, candidates=[{"centre_m": [3.0, 0.0], "distance_m": 0.0}])
    c = N.classify("lbg_B0_line_2_s0", r)
    assert c["ended_at_near"] is False and "label" not in c
    assert N.scene_of("lat_r_lbg_Ub6_aisle_rack_s0") == "aisle_rack" and N.scene_of("lbd_Pn_random_1_s5") == "random_1"
