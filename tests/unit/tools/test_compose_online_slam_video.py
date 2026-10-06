"""Online SLAM video helpers (plan S3): map eligibility and the control estimate."""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("PIL")
ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    "compose_online_slam_video", ROOT / "tools/compose_online_slam_video.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_a_frame_shows_only_maps_stored_after_an_earlier_scan():
    after = np.array([0, 4, 9, 9, 15])
    assert MODULE.eligible_map(after, 0) == -1
    assert MODULE.eligible_map(after, 5) == 1
    assert MODULE.eligible_map(after, 9) == 1  # stored after scan 9 may postdate its reply
    assert MODULE.eligible_map(after, 10) == 3


def test_the_estimate_is_the_applied_correction_on_odometry():
    records = [
        {"scan_id": 0, "stamp_s": 0.0, "status": "processed",
         "applied_map_from_odom": [0.0, 0.0, 0.0], "odom_base": [1.0, 0.0, 0.0],
         "truth_base": [1.0, 0.0, 0.0]},
        {"scan_id": 1, "stamp_s": 0.1, "status": "skipped",
         "applied_map_from_odom": [0.1, 0.0, math.pi / 2], "odom_base": [1.0, 0.0, 0.0],
         "truth_base": [0.1, 1.0, math.pi / 2]},
        {"scan_id": 2, "stamp_s": 0.2, "status": "link_failure", "error": "x"},
    ]
    scans = MODULE.scan_estimates(records)
    assert len(scans["stamps"]) == 2
    np.testing.assert_allclose(scans["estimate"][1], (0.1, 1.0, math.pi / 2), atol=1e-12)
    assert scans["processed"].tolist() == [True, False]


def test_the_overlay_shows_the_plan_given_by_then_and_flags_a_fresh_replan():
    plans = [
        {"time_s": 10.0, "phase": "transport", "why": "live", "poses": [[0, 0], [1, 0]]},
        {"time_s": 50.0, "phase": "transport", "why": "replan", "poses": [[1, 0], [1, 1]]},
    ]
    spawns = [{"time_s": 47.0, "action": "spawn"}]
    early = MODULE.overlay_at(20.0, plans, spawns)
    assert early["current"]["why"] == "live" and early["previous"] is None and early["banner"] is None
    fresh = MODULE.overlay_at(51.0, plans, spawns)
    assert fresh["current"]["why"] == "replan" and fresh["previous"]["why"] == "live"
    assert "재계획" in fresh["banner"]
    assert "새 장애물" in MODULE.overlay_at(48.0, plans, spawns)["banner"]
    later = MODULE.overlay_at(60.0, plans, spawns)
    assert later["previous"] is None and later["banner"] is None
    assert MODULE.overlay_at(5.0, plans, spawns)["current"] is None


def test_the_grid_frame_is_the_latest_recorded_by_then():
    times = np.array([0.0, 0.5, 1.0])
    assert MODULE.grid_index(times, 0.7) == 1 and MODULE.grid_index(times, -1.0) == -1


def test_a_spawned_box_is_drawn_from_its_spawn_until_removed():
    log = [
        {"time_s": 10.0, "action": "spawn", "detail": ["b", 1.0, 2.0, 0.0, [0.4, 0.4, 0.5]]},
        {"time_s": 30.0, "action": "remove", "detail": ["b"]},
    ]
    assert MODULE.boxes_at(5.0, log) == []
    corners = MODULE.boxes_at(20.0, log)[0]
    np.testing.assert_allclose(np.sort(corners[:, 0]), [0.8, 0.8, 1.2, 1.2])
    assert MODULE.boxes_at(31.0, log) == []
