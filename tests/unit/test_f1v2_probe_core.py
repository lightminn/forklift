"""F1 v2 delivery-match probe core: the registered grid, laser/base/rear pose inversion, fixed noise and the match
evaluation's agreement with dock()."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim" / "isaac"))

import f1v2_probe_core as P  # noqa: E402

from forklift_core.localization.scan_docking import dock, laser_points  # noqa: E402

MOUNT_XY, MOUNT_YAW, OFFSET = (-0.12, 0.0), 0.0, 0.399
DELIVERY = (0.678, -8.946, 0.0)


def test_grid_counts_and_definitions():
    g = P.grid(DELIVERY, OFFSET, MOUNT_XY, MOUNT_YAW)
    assert len(g) == 12 * 3 * 3 + 8 * 3 == 132
    assert len({p["label"] for p in g}) == 132
    p = next(p for p in g if p["label"] == "r1_d1.50_y-8.955_w0.02")
    assert p["rear"][0] == pytest.approx(DELIVERY[0] - 1.5)
    assert p["laser"][1] == pytest.approx(-8.955) and p["laser"][2] == pytest.approx(0.02)
    # the laser recomputed from the base agrees (laser_pose_2d's forward direction)
    bx, by, yaw = p["base"]
    lx = bx + math.cos(yaw) * MOUNT_XY[0] - math.sin(yaw) * MOUNT_XY[1]
    ly = by + math.sin(yaw) * MOUNT_XY[0] + math.cos(yaw) * MOUNT_XY[1]
    assert (lx, ly) == pytest.approx(p["laser"][:2])
    assert p["base"][0] - p["rear"][0] == pytest.approx(OFFSET * math.cos(yaw))
    r2 = [p for p in g if p["label"].startswith("r2")]
    assert {round(p["d_m"], 2) for p in r2} == set(P.ROUND2_D_M) and min(P.ROUND2_D_M) == 0.5
    with pytest.raises(ValueError):
        P.grid((0.0, 0.0, 0.3), OFFSET, MOUNT_XY, MOUNT_YAW)


def test_v1_stop_maps_into_the_grid_range():
    # v1 laser x -0.530..-0.494 -> stop distance 1.451..1.487, inside 1.38..1.60
    for lx in (-0.530, -0.494):
        base, rear = P.poses_from_laser((lx, -8.95, 0.0), OFFSET, MOUNT_XY, MOUNT_YAW)
        assert min(P.ROUND1_D_M) <= DELIVERY[0] - rear[0] <= max(P.ROUND1_D_M)


def test_noise_is_fixed_per_point_and_k_and_cut():
    a, b = P.noise_vector(1600, 3, 2), P.noise_vector(1600, 3, 2)
    assert np.array_equal(a, b) and not np.array_equal(a, P.noise_vector(1600, 3, 3))
    assert np.max(np.abs(a)) <= P.NOISE_CUT_M


def test_sent_ranges_only_touch_finite_and_clip():
    r = np.array([1.0, np.inf, 11.99, -np.inf])
    s = P.sent_ranges(r, np.array([0.05, 0.05, 0.05, 0.05]), 0.2, 12.0)
    assert s[0] == pytest.approx(1.05) and np.isinf(s[1]) and s[2] == 12.0 and np.isinf(s[3])


def test_evaluate_matches_dock_and_reports_weak_direction():
    xs = np.linspace(0.5, 4.0, 120)
    reference = np.vstack((np.column_stack((xs, np.full_like(xs, 1.5))),
                           np.column_stack((np.full(80, 4.0), np.linspace(-1.5, 1.5, 80))),
                           np.column_stack((xs, np.full_like(xs, -1.5)))))
    angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    # ranges of the reference seen from 0.3 m behind the reference laser: cast against the three walls
    ranges = np.full(len(angles), np.inf)
    for i, a in enumerate(angles):
        c, s = math.cos(a), math.sin(a)
        best = np.inf
        for t in ((1.5 / s) if s > 1e-9 else np.inf, (-1.5 / s) if s < -1e-9 else np.inf,
                  ((4.0 + 0.3) / c) if c > 1e-9 else np.inf):
            if t > 0:
                px, py = -0.3 + t * c, t * s
                if 0.5 <= px <= 4.0 + 1e-9 and -1.5 - 1e-9 <= py <= 1.5 + 1e-9:
                    best = min(best, t)
        ranges[i] = best
    kw = dict(rear_from_laser=(0.4, 0.0, 0.0), estimate_rear=(-0.3, 0.0, 0.0), delivery_rear=(0.0, 0.0, 0.0))
    out = P.evaluate(reference, ranges, angles, truth_rear=(-0.3, 0.0, 0.0), reference_rear=(0.0, 0.0, 0.0), **kw)
    ref = dock(reference, laser_points(ranges, angles), **kw)
    assert out["accepted"] == ref.accepted and out["reason"] == ref.reason
    assert [s["information"] for s in out["starts"]] == [d["min_information"] for d in ref.start_details]
    assert len(out["starts"]) == 7 and all(s["weak_vector"] is None or len(s["weak_vector"]) == 3 for s in out["starts"])
    if out["accepted"]:
        assert max(abs(v) for v in out["goal_error_vs_truth"][:2]) < 0.01
