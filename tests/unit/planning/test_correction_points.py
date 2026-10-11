"""Plan D8 S4c-3: where a rolled-out approach crosses the near-field correction points."""

import math

import numpy as np
import pytest

from forklift_core.planning.pallet_mission import (
    PalletSite,
    SyntheticMissionGeometry,
    correction_point_offsets,
    site_poses,
)

GEOMETRY = SyntheticMissionGeometry(alignment_straight_m=2.1)
SITES = site_poses(PalletSite(5.0, 1.0, 0.3), GEOMETRY)
FORK = GEOMETRY.axle_to_fork_tip_m
FACE = GEOMETRY.axle_to_fork_tip_m + GEOMETRY.approach_gap_m


def on_line(along, lateral=0.0, yaw=0.0):
    start = SITES["prealign"]
    c, s = math.cos(start.yaw_rad), math.sin(start.yaw_rad)
    return [
        start.x_m + along * c - lateral * s,
        start.y_m + along * s + lateral * c,
        start.yaw_rad + yaw,
    ]


def along_line_distance(pose):
    """The fork tip to face measured along the line: the face lies FACE ahead of the
    approach pose, the fork tip FORK ahead of the rear axle along the pose's heading."""
    start, end = SITES["prealign"], SITES["approach"]
    c, s = math.cos(start.yaw_rad), math.sin(start.yaw_rad)
    end_along = (end.x_m - start.x_m) * c + (end.y_m - start.y_m) * s
    along = (pose[0] - start.x_m) * c + (pose[1] - start.y_m) * s
    return end_along + FACE - (along + FORK * math.cos(pose[2] - start.yaw_rad))


def offsets(trajectory, points=(1.0, 0.6)):
    return correction_point_offsets(
        trajectory,
        SITES["prealign"],
        face_distance=along_line_distance,
        points_m=points,
    )


def test_on_the_line_the_fork_tip_reaches_each_point_where_the_geometry_puts_it():
    # The line is 2.1 m; at its end the fork tips stand the approach gap (0.10 m) from the face.
    trajectory = [on_line(a) for a in np.arange(0.0, 2.1 + 1e-9, 0.001)]
    out = offsets(trajectory, points=(2.2, 1.0, 0.6, 0.1))
    assert [e["point_m"] for e in out] == [2.2, 1.0, 0.6, 0.1]
    for e, along in zip(out, (0.0, 1.2, 1.6, 2.1), strict=True):
        assert e["offsets"]["along_m"] == pytest.approx(along, abs=1e-3)
        assert e["face_distance_m"] <= e["point_m"] + 1e-12
        assert e["offsets"]["lateral_m"] == pytest.approx(0.0, abs=1e-12)


def test_the_offsets_are_the_poses_own_and_the_heading_shortens_the_fork_reach():
    pose = on_line(1.0, lateral=0.03, yaw=0.1)
    out = offsets([pose], points=(5.0,))[0]
    assert out["pose"] == pytest.approx(pose)
    assert out["offsets"]["lateral_m"] == pytest.approx(0.03)
    assert out["offsets"]["yaw_rad"] == pytest.approx(0.1)
    # Fork tip to face = line end + face ahead - (along + fork x cos(yaw)).
    assert out["face_distance_m"] == pytest.approx(
        2.1 + FACE - (1.0 + FORK * math.cos(0.1))
    )


def test_a_trajectory_that_never_comes_close_has_no_offsets():
    out = offsets([on_line(a) for a in (0.0, 0.5, 1.0)])
    assert [e["offsets"] for e in out] == [None, None]
    assert [e["pose"] for e in out] == [None, None]


def test_the_crossing_records_the_pose_before_it():
    trajectory = [on_line(a) for a in (1.0, 1.1, 1.2, 1.3)]
    out = offsets(trajectory, points=(1.0,))[0]
    assert out["pose"] == pytest.approx(on_line(1.2))
    assert out["previous_pose"] == pytest.approx(on_line(1.1))
    assert out["previous_face_distance_m"] > 1.0 >= out["face_distance_m"]
    first = offsets([on_line(1.9)], points=(1.0,))[0]
    assert first["previous_pose"] is None and first["previous_face_distance_m"] is None
