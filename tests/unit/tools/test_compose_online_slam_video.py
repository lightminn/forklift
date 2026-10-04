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
