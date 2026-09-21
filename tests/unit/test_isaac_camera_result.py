"""CPU tests of the final G1 decision, including height distance coverage."""

import runpy
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
FINISH = runpy.run_path(str(ROOT / "sim/isaac/verify_perception_camera.py"))[
    "finish_result"
]


@pytest.fixture
def complete_result():
    # Each panel observes one distance; other distances are genuinely unobserved.
    # Distinct dictionaries ensure a missing bin cannot be hidden by aliasing.
    return {
        "perception_camera_intrinsics": {"status": "PASS"},
        "mount": {"status": "PASS"},
        "intrinsics_fits": [{"gate_2a": "PASS", "gate_2b": "PASS"} for _ in range(18)],
        "boards": [{"gate_4": "PASS", "status": "PASS"} for _ in range(162)],
        "height_panels": [
            {
                "status": "PASS",
                "captures": [
                    {
                        "status": "PASS",
                        "per_distance": [
                            {
                                "distance_bin": distance,
                                "count": 20 if distance == panel // 9 else 0,
                                "status": "PASS"
                                if distance == panel // 9
                                else "UNOBSERVED",
                            }
                            for distance in range(6)
                        ],
                    }
                    for _ in range(3)
                ],
            }
            for panel in range(54)
        ],
        "gate_5": "PASS",
        "marker": [{"bright_centroid_outside_marker_bbox": True}],
    }


def test_complete_numerics_require_user_review_and_never_authorize_g2(complete_result):
    FINISH(complete_result)
    assert complete_result["gates"]["7a_prime"] == "PASS"
    assert complete_result["gates"]["7b"] == "RECORD_ONLY"
    assert complete_result["gates"]["6"] == "USER_JUDGMENT_REQUIRED"
    assert complete_result["numerical_status"] == "PASS"
    assert complete_result["status"] == "REVIEW_REQUIRED"
    assert complete_result["g2_allowed"] is False


@pytest.mark.parametrize("missing_bin", range(6))
def test_final_height_gate_refuses_each_unobserved_distance(
    complete_result, missing_bin
):
    for panel in complete_result["height_panels"]:
        for capture in panel["captures"]:
            group = capture["per_distance"][missing_bin]
            group.update(status="UNOBSERVED", count=0)
    FINISH(complete_result)
    assert complete_result["gates"]["7a_prime"] == "FAIL"
    assert complete_result["numerical_status"] == complete_result["status"] == "FAIL"
    assert complete_result["g2_allowed"] is False


@pytest.mark.parametrize("empty_capture_list", [False, True])
def test_final_height_gate_never_promotes_all_unobserved_panels(
    complete_result, empty_capture_list
):
    for panel in complete_result["height_panels"]:
        panel["status"] = "UNOBSERVED"
        for capture in panel["captures"]:
            capture["status"] = "UNOBSERVED"
            for group in capture["per_distance"]:
                group.update(status="UNOBSERVED", count=0)
        if empty_capture_list:
            panel["captures"] = []
    FINISH(complete_result)
    assert complete_result["gates"]["7a_prime"] == "FAIL"
    assert complete_result["status"] == "FAIL"


def test_local_unobserved_panel_is_allowed_when_all_distances_are_measured(
    complete_result,
):
    complete_result["height_panels"][0] = {"status": "UNOBSERVED", "captures": []}
    FINISH(complete_result)
    assert complete_result["gates"]["7a_prime"] == "PASS"
    assert complete_result["status"] == "REVIEW_REQUIRED"


@pytest.mark.parametrize(
    "section,field,gate",
    [
        ("intrinsics_fits", "gate_2a", "2a"),
        ("intrinsics_fits", "gate_2b", "2b"),
        ("boards", "gate_4", "4"),
        ("boards", "status", "7a"),
        ("height_panels", "status", "7a_prime"),
    ],
)
@pytest.mark.parametrize("defect", ["failed_record", "missing_record"])
def test_final_gate_refuses_one_failed_or_missing_record(
    complete_result, section, field, gate, defect
):
    if defect == "failed_record":
        complete_result[section][0][field] = "FAIL"
    else:
        complete_result[section].pop()
    FINISH(complete_result)
    assert complete_result["gates"][gate] == "FAIL"
    assert complete_result["status"] == "FAIL"
    assert complete_result["g2_allowed"] is False
