"""tools/low_clearance_report.py on synthetic records: the keep-out rectangle and the physical objects are reported
apart, an overlap is found by the overlap test (the carried pallet's too), the phase comes from the transitions, the stop
arc follows the truth motion, and the unevaluated ranges are listed (Codex review counterexamples)."""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))
import low_clearance_report as lcr  # noqa: E402
from forklift_core.control.drive_permission import StoppingModel  # noqa: E402

GEOM = {"loaded_footprint": {"front_m": 1.56, "rear_m": 0.17, "half_width_m": 0.40},
        "unloaded_footprint": {"front_m": 1.29, "rear_m": 0.17, "half_width_m": 0.36},
        "pallet_depth_m": 0.6, "pallet_width_m": 0.8}


def write_run(tmp_path, *, keepout_y=3.0, physical=(1.0, 3.0), speed=0.0, kappa=0.0, ticks=240, gap=False,
              transitions=(("insert", "transport", 0.0),), pallet_lat=0.0, sample_times=None, cmd_kappa=0.0):
    run = tmp_path / "run"
    run.mkdir(parents=True)
    keep = {"asset": {"uri": "riibotics:block", "length_m": 2.0, "width_m": 1.0, "height_m": 0.144},
            "rectangle": {"x_m": 1.0, "y_m": keepout_y, "length_m": 2.0, "width_m": 1.0, "yaw_rad": 0.0}}
    tall = {"asset": {"uri": "rack", "length_m": 0.1, "width_m": 0.1, "height_m": 1.5},
            "rectangle": {"x_m": 9.0, "y_m": 9.0, "length_m": 0.1, "width_m": 0.1, "yaw_rad": 0.0}}
    phys = {"x_m": physical[0], "y_m": physical[1], "length_m": 0.6, "width_m": 0.8, "yaw_rad": 0.0, "base_m": 0.0,
            "height_m": 0.144}
    t = np.arange(ticks) / 120.0
    if gap:
        t = np.concatenate([t[:100], t[110:]])
    if abs(kappa) < 1e-9:
        x, y, yaw = speed * t, 0 * t, 0 * t
    else:
        yaw = speed * kappa * t
        x, y = np.sin(yaw) / kappa, (1 - np.cos(yaw)) / kappa
    C = np.column_stack([t, x, y, yaw, x, y, yaw])
    np.save(run / "slam_control.npy", C)
    st = np.arange(0, ticks / 120.0, 0.1) if sample_times is None else np.asarray(sample_times)
    samples = []
    for tt in st:
        i = int(np.argmin(np.abs(t - tt)))
        c, s_ = np.cos(yaw[i]), np.sin(yaw[i])
        samples.append({"time_s": float(tt), "phase": "transport", "rear_pose": [0.0, 0.0, 0.0],
                        "signed_speed_mps": 0.0, "curvature_inv_m": cmd_kappa,
                        "pallet_position_m": [float(x[i] + 1.26 * c - pallet_lat * s_),
                                              float(y[i] + 1.26 * s_ + pallet_lat * c), 0.2],
                        "pallet_yaw_rad": float(yaw[i])})
    (run / "result.json").write_text(json.dumps({
        "scenario": {"props": [keep, tall]}, "scene_layout": {"obstacles": [phys]}, "geometry": GEOM,
        "samples": samples, "transitions": [{"from": a, "to": b, "time_s": tt} for a, b, tt in transitions]}))
    return run


def report(run):
    return lcr.run_report(str(run), StoppingModel(0.15, 1.5, 0.05), 0.01)


def test_keepout_and_physical_distances_are_apart(tmp_path):
    r = report(write_run(tmp_path, keepout_y=1.0, physical=(1.0, 1.2)))
    rows = r["rows"]
    assert abs(rows["truck_keepout"]["min_m"] - (1.0 - 0.5 - 0.40)) < 1e-6  # zone edge at y 0.5, loaded hull at 0.40
    assert abs(rows["truck_physical"]["min_m"] - (1.2 - 0.4 - 0.40)) < 1e-6  # pallet edge at y 0.8
    assert rows["truck_keepout"]["object"] == 0  # the tall prop is not a low object
    assert r["keepout_objects"] == 1 and r["physical_objects"] == 1
    assert not r["keepout_overlaps"] and r["reopen"] == {"a_physical_near": False, "b_keepout_overlap": False}


def test_an_overlap_is_found_by_the_overlap_test_not_the_distance(tmp_path):
    r = report(write_run(tmp_path, keepout_y=0.6))
    assert r["rows"]["truck_keepout"]["min_m"] == 0.0
    assert r["keepout_overlaps"] and r["keepout_overlaps"][0]["object"] == 0
    assert r["reopen"]["b_keepout_overlap"] and not r["reopen"]["a_physical_near"]


def test_the_carried_pallet_alone_overlapping_raises_b(tmp_path):
    # zone edge at y 0.43: the hull (0.40) clears it by 30 mm, the pallet shifted 50 mm sideways reaches 0.45
    r = report(write_run(tmp_path, keepout_y=0.93, pallet_lat=0.05))
    assert r["rows"]["truck_keepout"]["min_m"] > 0.0
    assert any(o["part"] == "pallet" for o in r["keepout_overlaps"]) and r["reopen"]["b_keepout_overlap"]


def test_physical_near_reopens_a(tmp_path):
    r = report(write_run(tmp_path, physical=(1.0, 0.83)))  # pallet edge at 0.43: 0.03 from the hull
    assert r["reopen"]["a_physical_near"]


def test_the_stop_arc_follows_the_truth_motion_not_the_command(tmp_path):
    # driving forward at 0.5 m/s toward an object ahead; the commanded curvature (sharp) is ignored
    r = report(write_run(tmp_path, speed=0.5, physical=(1.56 + 0.2 + 0.3 + 1.0, 0.0), cmd_kappa=0.36))
    assert r["physical_arc_violations"] and r["reopen"]["a_physical_near"]
    assert all(abs(h["curvature_inv_m"]) < 1e-6 for h in r["physical_arc_violations"])


def test_the_phase_comes_from_the_transitions(tmp_path):
    # insert -> lift at 0.05 s, between samples 0.0 and 0.1: from 0.05 s the loaded hull (0.40) is checked
    r = report(write_run(tmp_path, keepout_y=0.89, transitions=(("insert", "lift", 0.05),)))
    assert r["reopen"]["b_keepout_overlap"]  # loaded hull 0.40 > zone edge 0.39; the unloaded 0.36 would not
    assert r["keepout_overlaps"][0]["from_s"] >= 0.05 - 1e-9


def test_unloaded_reversing_arc_ticks_are_unevaluated(tmp_path):
    r = report(write_run(tmp_path, speed=-0.6, physical=(-0.17 - 0.2 - 0.3, 0.0),
                         transitions=(("lower", "return_home", 0.0),)))
    assert not r["physical_arc_violations"]
    assert any(isinstance(u, dict) and "arc_unloaded_reversing_s" in u for u in r["unevaluated"])


def test_unevaluated_ranges_are_listed(tmp_path):
    r = report(write_run(tmp_path, gap=True, sample_times=[0.0, 0.1, 1.0, 1.1]))
    assert any(isinstance(u, dict) and "control_gap_s" in u for u in r["unevaluated"])
    assert any(isinstance(u, dict) and u.get("sample_gap_s") == [0.1, 1.0] for u in r["unevaluated"])
    assert any(isinstance(u, dict) and "after_last_sample_s" in u for u in r["unevaluated"])
