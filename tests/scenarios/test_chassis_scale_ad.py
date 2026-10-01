"""Chassis-scale A-D placements (tools/chassis_ad_cases.py): the narrow facts only.

What the planner does in C and D is recorded by the tool, not asserted here:
the planner has no contract for the gear it starts or ends in, so a result
that differs from the brief is a finding, not a failure.
"""

import math

import pytest

from tools import chassis_ad_cases as cases


@pytest.mark.parametrize(
    ("radius", "b_x", "c_x"),
    [(2.0, 0.749454, -0.795273), (1 / 0.29, 1.528002, -0.405999)],
)
def test_b_and_c_goals_follow_the_reference_radius(radius, b_x, c_x):
    placed = cases.placements(radius)
    assert placed["B"][0].x_m == pytest.approx(b_x, abs=1e-6)
    assert placed["C"][0].x_m == pytest.approx(c_x, abs=1e-6)
    assert placed["B"][0].y_m == placed["C"][0].y_m == 0.72
    assert placed["D"][0] == placed["C"][0]


def test_d_rear_block_is_inside_the_bay_and_0_30_m_behind_the_truck():
    block = cases.rear_obstacle()
    front = block.x_m + block.length_m / 2
    back = block.x_m - block.length_m / 2
    assert front == pytest.approx(cases.START.x_m - cases.FOOTPRINT.rear_m - 0.30)
    assert back > cases.BAY.x_min_m
    assert back == pytest.approx(-2.96)


def test_case_a_is_straight_in_for_both_planners_and_all_rows_are_reported():
    rows = cases.run_cases()
    predefined = [r for r in rows if r["definition"] == "predefined"]
    post_hoc = [r for r in rows if r["definition"] == "post_hoc"]
    assert len(predefined) == 16 and {r["case"] for r in predefined} == set("ABCD")
    assert len(post_hoc) == 8 and {r["case"] for r in post_hoc} == {"B0", "C0"}
    for row in rows:
        if row["case"] == "A":
            assert row["status"] == "success"
            assert (row["first_gear"], row["last_gear"], row["gear_changes"]) == (
                1,
                1,
                0,
            )
            assert math.isclose(row["max_abs_yaw_rad"], 0.0, abs_tol=1e-9)


def test_polygon_gap_sees_edges_not_only_corners():
    # Codex's case: the truck turned beside the D block; edge gap 0.120 m.
    block = cases.rear_obstacle()
    x0, x1 = block.x_m - block.length_m / 2, block.x_m + block.length_m / 2
    y0, y1 = block.y_m - block.width_m / 2, block.y_m + block.width_m / 2
    rect = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    import numpy as np

    truck = cases._footprint_polygon(-2.33, -0.43, np.pi / 2)
    assert cases.polygon_gap_m(truck, rect) == pytest.approx(0.120, abs=1e-3)
