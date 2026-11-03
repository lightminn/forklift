"""tools/low_clearance_report.py on synthetic records: the keep-out rectangle and the physical objects are reported
apart, an overlap is found by the overlap test, a physical stop-arc violation and the unevaluated ranges are listed."""
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


def write_run(tmp_path, *, keepout_y, physical_y, speed=0.0, gap=False, ticks=240):
    run = tmp_path / "r" / "run"
    run.mkdir(parents=True)
    # a truck driving along +x at y = 0, loaded; a low keep-out zone and a physical pallet beside it
    keep = {"asset": {"uri": "riibotics:block", "length_m": 2.0, "width_m": 1.0, "height_m": 0.144},
            "rectangle": {"x_m": 1.0, "y_m": keepout_y, "length_m": 2.0, "width_m": 1.0, "yaw_rad": 0.0}}
    tall = {"asset": {"uri": "rack", "length_m": 0.1, "width_m": 0.1, "height_m": 1.5},
            "rectangle": {"x_m": 9.0, "y_m": 9.0, "length_m": 0.1, "width_m": 0.1, "yaw_rad": 0.0}}
    phys = {"x_m": 1.0, "y_m": physical_y, "length_m": 0.6, "width_m": 0.8, "yaw_rad": 0.0, "base_m": 0.0, "height_m": 0.144}
    t = np.arange(ticks) / 120.0
    if gap:
        t = np.concatenate([t[:100], t[110:] + 0.0])
    x = 0.0 + 0.0 * t
    C = np.column_stack([t, x, 0 * t, 0 * t, x, 0 * t, 0 * t])
    np.save(run / "slam_control.npy", C)
    samples = [{"time_s": float(tt), "phase": "transport", "rear_pose": [0.0, 0.0, 0.0], "signed_speed_mps": speed,
                "curvature_inv_m": 0.0, "pallet_position_m": [1.26, 0.0, 0.2], "pallet_yaw_rad": 0.0}
               for tt in np.arange(0, ticks / 120.0, 0.1)]
    (run / "result.json").write_text(json.dumps({
        "scenario": {"props": [keep, tall]}, "scene_layout": {"obstacles": [phys]}, "geometry": GEOM, "samples": samples}))
    return run


def report(run):
    return lcr.run_report(str(run), StoppingModel(0.15, 1.5, 0.05), 0.01)


def test_keepout_and_physical_distances_are_apart(tmp_path):
    r = report(write_run(tmp_path, keepout_y=1.0, physical_y=1.2))
    rows = r["rows"]
    assert abs(rows["truck_keepout"]["min_m"] - (1.0 - 0.5 - 0.40)) < 1e-6  # zone edge at y 0.5, hull at 0.40
    assert abs(rows["truck_physical"]["min_m"] - (1.2 - 0.4 - 0.40)) < 1e-6  # pallet edge at y 0.8
    assert rows["truck_keepout"]["object"] == 0  # the tall prop is not a low object
    assert r["keepout_objects"] == 1 and r["physical_objects"] == 1
    assert not r["keepout_overlaps"] and r["reopen"] == {"a_physical_near": False, "b_keepout_overlap": False}


def test_an_overlap_is_found_by_the_overlap_test_not_the_distance(tmp_path):
    r = report(write_run(tmp_path, keepout_y=0.6, physical_y=3.0))
    assert r["rows"]["truck_keepout"]["min_m"] == 0.0
    assert r["keepout_overlaps"] and r["keepout_overlaps"][0]["object"] == 0
    assert r["reopen"]["b_keepout_overlap"] and not r["reopen"]["a_physical_near"]


def test_physical_near_and_a_stop_arc_violation_reopen_a(tmp_path):
    near = report(write_run(tmp_path / "n", keepout_y=3.0, physical_y=0.83))  # pallet edge at 0.43: 0.03 from the hull
    assert near["reopen"]["a_physical_near"]
    # driving forward at 0.5 m/s toward a physical object ahead: the stop arc meets it
    run = write_run(tmp_path / "a", keepout_y=3.0, physical_y=0.0, speed=0.5)
    data = json.loads((run / "result.json").read_text())
    data["scene_layout"]["obstacles"][0].update(x_m=1.56 + 0.2 + 0.3, y_m=0.0)
    (run / "result.json").write_text(json.dumps(data))
    arc = report(run)
    assert arc["physical_arc_violations"] and arc["reopen"]["a_physical_near"]


def test_unevaluated_ranges_are_listed(tmp_path):
    r = report(write_run(tmp_path, keepout_y=1.0, physical_y=1.2, gap=True))
    gaps = [u for u in r["unevaluated"] if isinstance(u, dict) and "control_gap_s" in u]
    assert gaps and gaps[0]["control_gap_s"][1] - gaps[0]["control_gap_s"][0] > 3 / 120
    assert any(isinstance(u, dict) and "after_last_sample_s" in u for u in r["unevaluated"])
