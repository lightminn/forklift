# tests/unit/test_recognition_sequences.py
"""Approach sequences, their odometry and the tracker evaluation (stage 2B-3 plan sections 6-7)."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from tools import calibrate_track_bounds as C
from tools import evaluate_track_sequences as E
from tools import recognition_scenes as rs
from tools import recognition_sequences as R

ROOT = Path(__file__).resolve().parents[2]
TABLE = json.loads((ROOT / "config/obstacle_odometry_age.json").read_text())


class _Face:
    def __init__(self, left, right, yaw):
        self.left_m, self.right_m, self.insertion_yaw_rad = left, right, yaw


def _face(yaw=0.3):
    a = np.array([math.cos(yaw), math.sin(yaw)])
    n = np.array([-a[1], a[0]])
    mid = np.array([4.0, 1.0])
    return _Face((*(mid + 0.2 * n), 0.06), (*(mid - 0.2 * n), 0.06), yaw)


def _camera(p):
    x = R.instance_rig.PERCEPTION_CAMERA.xyz_m[0]
    return np.array([p.x + x * math.cos(p.yaw), p.y + x * math.sin(p.yaw)])


def test_a_straight_path_runs_on_the_face_normal_from_3_5_to_1_3_m_in_0_12_m_steps():
    face = _face()
    poses = R.path(face, "straight")
    a = np.array([math.cos(face.insertion_yaw_rad), math.sin(face.insertion_yaw_rad)])
    mid = (np.asarray(face.left_m[:2]) + np.asarray(face.right_m[:2])) / 2
    d = [float((mid - _camera(p)) @ a) for p in poses]
    assert d[0] == pytest.approx(3.5) and d[-1] >= 1.3 - 1e-9 and d[-1] < 1.3 + 0.12
    assert np.allclose(np.diff(d), -0.12)
    assert all(abs(p.yaw - face.insertion_yaw_rad) < 1e-12 for p in poses)


def test_a_curved_path_starts_offset_and_turned_and_ends_aligned():
    face = _face(0.0)
    poses = R.path(face, "curve")
    cam = _camera(poses[0])
    assert cam[1] - 1.0 == pytest.approx(0.4)
    assert poses[0].yaw == pytest.approx(-math.radians(15))
    assert abs(_camera(poses[-1])[1] - 1.0) < 0.002 and abs(poses[-1].yaw) < math.radians(1.5)


def test_sequence_seeds_stay_clear_of_the_catalogue_and_of_each_other():
    catalogue = {r.seed for r in rs.catalogue()}
    seeds = {R.SEQUENCE_BANDS[b] + c * R.CELL_STRIDE + i for b in R.SEQUENCE_BANDS for c in range(len(R.CELLS))
             for i in range(100)}
    assert not seeds & catalogue
    assert len(seeds) == 2 * len(R.CELLS) * 100
    assert len(R.CELLS) * R.CELL_STRIDE <= R.SEQUENCE_BANDS["evaluation"] - R.SEQUENCE_BANDS["calibration"]


def test_noisy_odometry_starts_true_and_stays_within_a_few_millimetres_per_second():
    poses = [(0.12 * k, 0.0, 0.0) for k in range(19)]
    reported = E.odometry(poses, "noise", seed=4)
    assert reported[0] == pytest.approx(poses[0])
    errs = [math.hypot(r[0] - p[0], r[1] - p[1]) for r, p in zip(reported, poses)]
    assert max(errs) < 0.02 and errs[-1] > 0
    assert E.odometry(poses, "noise", seed=4) == reported  # deterministic


def test_the_late_read_is_the_noisy_stream_30_ms_ahead():
    poses = [(0.12 * k, 0.0, 0.0) for k in range(5)]
    late = E.odometry(poses, "late", seed=1)
    assert late[0][0] == pytest.approx(0.6 * 0.03, abs=0.002)


def test_the_drift_stays_inside_every_window_of_the_odometry_table():
    poses = [(0.12 * k, 0.05 * k, 0.01 * k) for k in range(19)]
    assert E.drift_problem(poses, TABLE) is None
    drift = E.odometry(poses, "drift", 0, TABLE)
    assert math.hypot(drift[-1][0] - poses[-1][0], drift[-1][1] - poses[-1][1]) == pytest.approx(0.0327268 * 3.6, rel=1e-3)


def test_the_drift_check_reports_a_window_outside_the_table(monkeypatch):
    poses = [(0.12 * k, 0.0, 0.0) for k in range(5)]
    jump = [(x, y + (0.05 if k >= 2 else 0.0), yaw) for k, (x, y, yaw) in enumerate(poses)]
    monkeypatch.setattr(E, "odometry", lambda *a, **kw: jump)
    assert "frames 1-2" in E.drift_problem(poses, TABLE)  # 50 mm in 0.2 s > 31.5 mm


@pytest.fixture(scope="module")
def short_sequence(tmp_path_factory):
    """Three frames (3.5 / 3.38 / 3.26 m) of the first S1 EPAL 6 calibration recipe, recorded twice."""
    mp = pytest.MonkeyPatch()
    mp.setattr(R, "END_M", 3.26)
    try:
        recipe = R.recipe_for(R.CELLS[0], "calibration", 0)
        site = R.load_site()
        first = R.sequence(recipe, "straight", level="frame_848", clean=True, site=site)
        second = R.sequence(recipe, "straight", level="frame_848", clean=False, site=site)
    finally:
        mp.undo()
    return recipe, first, second


def test_a_sequence_is_deterministic(short_sequence):
    _, first, second = short_sequence
    assert first["skipped"] is None and len(first["frames"]) == 3
    for a, b in zip(first["frames"], second["frames"]):
        assert a["noise_seed"] == b["noise_seed"] and a["pose"] == b["pose"]
        assert [c.left.centre_m for c in a["candidates"]] == [c.left.centre_m for c in b["candidates"]]


def test_each_frames_truth_is_the_world_face_seen_from_that_pose(short_sequence):
    recipe, first, _ = short_sequence
    k, i = first["target"]
    world = R.choose_target(recipe, R.load_site())
    face = next(f for kk, ii, f in world if (kk, ii) == (k, i))
    for f in first["frames"]:
        pose = R.Pose(*f["pose"])
        got = f["faces"][k][i]
        assert got["left_m"][:2] == pytest.approx(pose.to_base(face.left_m[:2]), abs=1e-9)
        assert got["insertion_yaw_rad"] == pytest.approx(R._wrap(face.insertion_yaw_rad - pose.yaw), abs=1e-12)


def test_the_tracker_input_carries_no_truth_ids(short_sequence):
    _, first, _ = short_sequence
    for f in first["frames"]:
        for c in f["candidates"]:
            assert c.proposal_id.startswith("p") and len(c.proposal_id) == 13
        assert "clean" in f


def test_the_camera_never_enters_a_box():
    box = R.scene_rig.Box((1.0, 0.0, 0.2), (0.4, 0.4, 0.4))

    class _Recipe:
        instances, distractors = (), (box,)

    assert R._inside_any(_Recipe, (1.1, 0.1, 0.27))
    assert not R._inside_any(_Recipe, (1.3, 0.0, 0.27))


def test_the_evaluation_and_calibration_run_on_a_recorded_sequence(short_sequence, monkeypatch):
    _, first, _ = short_sequence
    monkeypatch.setattr(C, "MIN_SAMPLES", 1)
    rows, report = C.table([first])
    assert rows["known"], report
    bounds = E.TrackBounds.from_dict(C.config(rows, TABLE, 1.2))
    res = E.run(first, bounds, "V1", TABLE, seed=1)
    assert len(res["rows"]) == 3 and res["identity_errors"] == 0
    assert res["rows"][0]["initialised"] and res["rows"][-1]["retained"]
    s = E.summarise([res], 0)["V1"]
    assert s["sequences"] == 1 and s["verdict"]["pass"] is False  # one sequence cannot fill a bin
