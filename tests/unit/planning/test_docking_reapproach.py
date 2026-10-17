"""Plan D7c ③: the re-approach to the docking line's start, accepted only inside its box."""

import math

import numpy as np
import pytest

from forklift_core.planning import Bounds, Pose2D, Rectangle
from forklift_core.planning.pallet_mission import (
    AssetSpec,
    PalletSite,
    PlacedProp,
    SyntheticMissionGeometry,
    TransportScenario,
    make_transport_planner_config,
    plan_docking_reapproach,
)

GEOMETRY = SyntheticMissionGeometry(delivery_straight_m=1.5)
CONFIG = make_transport_planner_config(curvature_limit_inv_m=0.29, max_expansions=30000,
                                       goal_connection="reeds_shepp", shot_cap_m=4.0)
LINE = Pose2D(0.0, 0.0, 0.0)
OPEN = TransportScenario(0, Pose2D(-5, 0, 0), PalletSite(5, 5, 0), PalletSite(3, 0, 0), (), Bounds(-10, 10, -6, 6))


def offset(along, lateral, yaw):
    return Pose2D(LINE.x_m + along, LINE.y_m + lateral, LINE.yaw_rad + yaw)


@pytest.mark.parametrize("lateral,yaw", [(0.24, 0.0), (-0.30, 0.08), (0.12, -0.08)])
def test_a_lateral_miss_replans_to_the_line_start_inside_the_box(lateral, yaw):
    path, record = plan_docking_reapproach(OPEN, offset(0.0, lateral, yaw), LINE, CONFIG, geometry=GEOMETRY, keep_m=1.5)
    assert path is not None, record
    np.testing.assert_allclose(path.poses[-1], [LINE.x_m, LINE.y_m, LINE.yaw_rad], atol=1e-6)
    assert record["refused"] is None and path.length_m <= 6.0
    assert -4.0 <= record["along_min_m"] and record["along_max_m"] <= 1.7 and record["lateral_max_m"] <= 1.5


def test_a_start_at_the_goal_backs_out_inside_the_box():
    # Trigger (c): stalled at the end of the docked straight, 1.5 m ahead of the line start.
    path, record = plan_docking_reapproach(OPEN, offset(1.5, 0.02, 0.01), LINE, CONFIG, geometry=GEOMETRY, keep_m=1.5)
    assert path is not None, record
    assert record["along_max_m"] <= 1.7 and (path.directions < 0).any()


def test_a_start_outside_the_box_is_refused():
    path, record = plan_docking_reapproach(OPEN, offset(0.0, 2.0, 0.0), LINE, CONFIG, geometry=GEOMETRY, keep_m=1.5)
    assert path is None and record["refused"] == "leaves_box" and record["lateral_max_m"] > 1.5


def test_a_long_re_approach_is_refused():
    path, record = plan_docking_reapproach(OPEN, offset(0.0, 0.24, 0.0), LINE, CONFIG, geometry=GEOMETRY, keep_m=1.5,
                                           max_length_m=0.5)
    assert path is None and record["refused"] == "too_long" and record["length_m"] > 0.5
    # A refused plan keeps its search statistics (Codex D7c re-review P3).
    assert record["expansions"] >= 1 and record["search_attempts"] and record["gear_changes"] >= 1


def test_no_path_is_refused_with_the_search_status():
    # The line start itself is inside a prop.
    wall = PlacedProp(AssetSpec("test://crate.usd", 1.0, 1.0, 1.0), Rectangle(0.0, 0.0, 1.0, 1.0, 0.0))
    blocked = TransportScenario(0, OPEN.start_rear, OPEN.pickup, OPEN.destination, (wall,), OPEN.bounds)
    path, record = plan_docking_reapproach(blocked, offset(-2.0, 0.24, 0.0), LINE, CONFIG, geometry=GEOMETRY,
                                           keep_m=1.5)
    assert path is None and record["refused"] == "no_path" and record["status"] != "success"


def test_the_box_follows_the_line_start_frame():
    # The same miss with the line turned 90 degrees: the box turns with it.
    line = Pose2D(2.0, -1.0, math.pi / 2)
    start = Pose2D(2.0 - 0.24, -1.0, math.pi / 2)  # 0.24 m to the line's left
    path, record = plan_docking_reapproach(OPEN, start, line, CONFIG, geometry=GEOMETRY, keep_m=1.5)
    assert path is not None, record
    assert record["lateral_max_m"] == pytest.approx(0.24, abs=0.05)
