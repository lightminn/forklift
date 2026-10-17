"""tools/d5_replay.py helpers (plan D5 v11 replay tool) on synthetic inputs.

The end-to-end check is the reproduction of recorded runs (ws1); here the pieces it rests
on: the two-time displacement, the rear-axle pose, the reproduction comparison, the
first-demand summary and the source loader (in a subprocess -- it replaces the imported
forklift_core modules).
"""

import json
import math
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from tools import d5_replay as R

ROOT = Path(__file__).resolve().parents[3]


def yaw_quat(yaw):
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def test_displacement_is_zero_when_control_equals_truth_and_follows_a_drift():
    pts = np.array([[1.0, 0.5], [2.0, -0.3]])
    pose = (0.4, 0.2, 0.3)
    assert np.allclose(R.compose_inverse(pose, pose, pts), pts)
    # Truth 5 mm ahead of the control along x: every placed point moves 5 mm.
    moved = R.compose_inverse((0.405, 0.2, 0.3), pose, pts)
    assert np.allclose(np.hypot(*(moved - pts).T), 0.005)
    # A yaw error turns the points about the rear axle: displacement = lever x angle.
    turned = R.compose_inverse((0.4, 0.2, 0.3 + 0.001), pose, pts)
    lever = np.hypot(pts[:, 0] - 0.4, pts[:, 1] - 0.2)
    assert np.allclose(np.hypot(*(turned - pts).T), lever * 0.001, rtol=1e-6)


def test_rear_pose_from_base_pose7():
    row = np.array([1.0, 2.0, 0.1, *yaw_quat(math.pi / 2)])
    x, y, yaw = R.rear_pose(row, -0.34)
    assert math.isclose(x, 1.0, abs_tol=1e-12)
    assert math.isclose(y, 2.0 - 0.34, abs_tol=1e-12)
    assert math.isclose(yaw, math.pi / 2, abs_tol=1e-12)


def test_same_compares_nested_numbers_within_tolerance():
    assert R._same([1.0, [2.0, 3.0]], [1.0 + 1e-9, [2.0, 3.0]])
    assert not R._same([1.0, 2.0], [1.0, 2.1])
    assert not R._same([1.0], [1.0, 2.0])
    assert R._same("blade_left", "blade_left") and not R._same("blade_left", "body")
    assert not R._same(None, 0.0)


def recorded_summary():
    return {
        "first_frame": {"voxels": 10, "free": 8, "in_view": 10, "obstacle": 0, "erosion_voxels": 0},
        "last_reason": "pocket_unverified",
        "last_reason_s": 63.78333665989339,
        "last_fail": {"box": "blade_left", "arc_s": 0.02, "pose_insertion": [-1.4119, 0.0, -0.0002], "voxels": 1628,
                      "outside_v": 0, "never_certified": 0, "expired": 4,
                      "examples_insertion": [[-0.125, 0.095, 0.035]], "examples_last_code": [3]},
    }


def test_reproduction_needs_every_field():
    rec = recorded_summary()
    first = dict(rec["first_frame"])
    stop = {"time_s": rec["last_reason_s"], "reason": rec["last_reason"], "last_fail": dict(rec["last_fail"])}
    assert R.reproduction(rec, first, stop)["reproduced"]
    later = {**stop, "time_s": rec["last_reason_s"] + 1.0 / 120.0}
    out = R.reproduction(rec, first, later)
    assert not out["reproduced"] and not out["first_stop_time"]["same"]
    other = {**stop, "last_fail": {**stop["last_fail"], "expired": 3}}
    assert not R.reproduction(rec, first, other)["reproduced"]
    assert not R.reproduction(rec, {**first, "free": 7}, stop)["reproduced"]
    assert not R.reproduction(rec, first, None)["reproduced"]


def test_demand_summary_counts_ages_expiry_and_examples():
    shape = (3, 1, 1)
    arrays = {
        "fd_t": np.array([10.0, 30.0, np.nan]).reshape(shape),
        "fd_cert": np.array([9.5, 5.0, np.nan]).reshape(shape),
        "fd_frame": np.array([1, 0, -1], dtype=np.int32).reshape(shape),
        "fd_code": np.array([1, 3, -1], dtype=np.int8).reshape(shape),
        "fd_box": np.array([0, 1, -1], dtype=np.int8).reshape(shape),
        "fd_disp": np.array([0.001, 0.012, np.nan]).reshape(shape),
        "in_v": np.ones(shape, dtype=bool),
        "lo": np.zeros(3),
        "voxel_m": np.array(0.01),
    }
    frames = [{"camera_face_m": 1.5}, {"camera_face_m": 0.4}]
    check = SimpleNamespace(config=SimpleNamespace(lifetime_s=20.0))
    out = R.demand_summary(check, arrays, frames, None)
    assert out["asked_voxels"] == 2 and out["asked_never_certified"] == 0
    assert out["asked_expired"] == 1  # age 25 s
    ages = {tuple(b["age_s"]): b["voxels"] for b in out["age_bins"]}
    assert ages[(0.0, 1.0)] == 1 and ages[(20.0, 30.0)] == 1
    ex = out["expired_examples"][0]
    assert ex["age_s"] == 25.0 and ex["camera_face_at_certification_m"] == 1.5 and ex["last_code"] == 3
    assert math.isclose(out["displacement_m_quantiles"]["max"], 0.012)


def test_source_loader_imports_only_from_the_given_root():
    """In a subprocess: the loader replaces forklift_core; every module must come from --source
    and its hash is compared with the run's record."""
    code = (
        "import json, sys; from pathlib import Path; sys.path.insert(0, sys.argv[1]);"
        "from tools import d5_replay as R; import forklift_core;"
        "m, h = R.load_source(Path(sys.argv[1]), {'sim/isaac/pocket_check.py': 'x'});"
        "print(json.dumps({k: str(Path(v.__file__).resolve()) for k, v in m.items()}));"
        "print(json.dumps(h))"
    )
    out = subprocess.run([sys.executable, "-c", code, str(ROOT)], capture_output=True, text=True, check=True)
    files, hashes = (json.loads(line) for line in out.stdout.strip().splitlines()[-2:])
    assert all(Path(f).is_relative_to(ROOT) for f in files.values())
    assert set(hashes) == set(R.SOURCE_MODULES.values())
    assert hashes["sim/isaac/pocket_check.py"]["matches_run"] is False
    assert not any(h["matches_run"] for k, h in hashes.items() if k != "sim/isaac/pocket_check.py")
