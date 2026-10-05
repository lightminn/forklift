"""P0a measurement helpers (priority-5 plan): envelope line, relative drift, standoff."""

import importlib.util
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("p0a_measure", ROOT / "tools/p0a_measure.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_the_upper_line_lies_on_or_above_every_point_and_is_tight():
    x = np.array([0.5, 1.0, 2.0, 4.0])
    y = np.array([0.02, 0.025, 0.03, 0.06])
    e0, k = MODULE.upper_line(x, y)
    assert np.all(e0 + k * x >= y - 1e-12)
    # Through the two outer hull points (0.5, 0.02)-(4.0, 0.06) the sum is least.
    assert math.isclose(k, 0.04 / 3.5) and math.isclose(e0, 0.02 - 0.5 * 0.04 / 3.5)


def test_a_flat_series_gives_a_constant_line():
    e0, k = MODULE.upper_line([1.0, 2.0, 3.0], [0.05, 0.04, 0.05])
    assert (e0, k) == (0.05, 0.0)


def test_identical_trajectories_have_no_relative_error():
    t = np.linspace(0, 10, 200)
    poses = np.column_stack((t, 0.1 * t**1.5, 0.05 * t))
    d, p, y = MODULE.relative_errors(poses, poses.copy(), start_every=10, max_distance_m=5)
    assert len(d) and p.max() < 1e-12 and y.max() < 1e-12


def test_a_yaw_bias_grows_with_the_lever_and_not_with_a_common_offset():
    s = np.linspace(0, 6, 601)
    truth = np.column_stack((s, np.zeros_like(s), np.zeros_like(s)))
    # Odometry that is offset everywhere by a constant translation has no relative error.
    shifted = truth + [5.0, -3.0, 0.0]
    d, p, _ = MODULE.relative_errors(truth, shifted, start_every=50, max_distance_m=6)
    assert p.max() < 1e-12
    # A heading that drifts 0.01 rad/m bends the odometry away from the line.
    bent = truth.copy()
    bent[:, 2] = 0.01 * s
    bent[1:, 0] = np.cumsum(np.cos(bent[:-1, 2]) * np.diff(s))
    bent[1:, 1] = np.cumsum(np.sin(bent[:-1, 2]) * np.diff(s))
    d, p, y = MODULE.relative_errors(truth, bent, start_every=600, max_distance_m=6)
    assert math.isclose(y[-1], 0.06, rel_tol=1e-6)
    assert 0.17 < p[-1] < 0.19  # about 0.01/2 * 6^2


def test_standoff_depth_follows_the_similar_triangles():
    # Camera 0.27 m, deck underside 0.10 m, voxel top 0.062 m: L = D (0.208/0.170 - 1).
    depth = MODULE.max_visible_depth_m(1.0, camera_z_m=0.27, deck_bottom_m=0.10, voxel_z_m=0.062)
    assert math.isclose(depth, 0.208 / 0.170 - 1.0)
    # The plan's v8 figure (camera 0.20, voxel 0.05) gives L = D / 2.
    assert math.isclose(
        MODULE.max_visible_depth_m(0.72, camera_z_m=0.20, deck_bottom_m=0.10, voxel_z_m=0.05), 0.36
    )


def test_windows_end_at_the_age_limit():
    stamps = np.linspace(0, 20, 201)
    truth = np.column_stack((0.1 * stamps, np.zeros_like(stamps), np.zeros_like(stamps)))
    d, _, _ = MODULE.relative_errors(
        truth, truth.copy(), start_every=100, max_distance_m=12, stamps=stamps, max_age_s=5
    )
    assert d.max() <= 0.5 + 1e-9


def test_the_age_table_never_decreases_and_reads_each_age_from_the_start():
    stamps = np.arange(0, 5, 0.01)
    truth = np.column_stack((0.5 * stamps, np.zeros_like(stamps), np.zeros_like(stamps)))
    estimate = truth.copy()
    estimate[:, 0] *= 1.1  # 10 % odometry overshoot
    err = MODULE.relative_error_at(0, (0.1, 1.0, 10.0), stamps, truth, estimate)
    assert math.isclose(err[0, 0], 0.005, abs_tol=1e-9) and math.isclose(err[1, 0], 0.05, abs_tol=1e-9)
    assert np.isnan(err[2]).all()
    table = MODULE.cumulative_table(np.array([[0.03, 0.01], [0.02, 0.02], [np.nan, np.nan]]))
    assert table.tolist() == [[0.03, 0.01], [0.03, 0.02], [0.03, 0.02]]


def test_an_error_between_two_listed_ages_bounds_the_later_age():
    # Codex checkpoint 6: a 0.19 s age beat the 0.2 s value; every tick counts.
    stamps = np.arange(0, 2, 0.01)
    truth = np.column_stack((0.5 * stamps, np.zeros_like(stamps), np.zeros_like(stamps)))
    estimate = truth.copy()
    estimate[15, 0] += 0.2  # a spike at 0.15 s that is gone again by 0.2 s
    err = MODULE.relative_error_at(0, (0.1, 0.2, 1.0), stamps, truth, estimate)
    assert err[0, 0] < 1e-9 and math.isclose(err[1, 0], 0.2, abs_tol=1e-9) and math.isclose(err[2, 0], 0.2, abs_tol=1e-9)


def test_a_heading_error_at_the_present_swings_the_old_pose():
    # Codex L0 P1: same positions, the odometry heading 0.1 rad off at the end only.
    stamps = np.array([0.0, 1.0])
    truth = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    odom = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.1]])
    err = MODULE.relative_error_at(0, (1.0,), stamps, truth, odom)
    assert math.isclose(err[0, 0], 2 * math.sin(0.05), rel_tol=1e-9)  # about 0.10 m
    assert math.isclose(err[0, 1], 0.1, rel_tol=1e-9)


def test_triangle_cells_cover_every_cell_the_triangle_touches():
    spec = importlib.util.spec_from_file_location("p0b", ROOT / "tools/p0b_sensor_study.py")
    p0b = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(p0b)
    tri = np.array([[0.01, 0.01], [0.24, 0.01], [0.01, 0.12]])
    cells = {tuple(c) for c in p0b.triangle_cells(tri, 0.05)}
    assert (0, 0) in cells and (4, 0) in cells and (0, 2) in cells
    assert (4, 2) not in cells  # beyond the hypotenuse
