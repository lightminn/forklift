"""P1c (docs/plans/2026-10-23-planner-permission-clearance.md): the searches keep the LiDAR-visible props -- taller
than the LiDAR plane -- ``lidar_keepout_m`` wider on each side, the fixed straights keep them at their size, and a
loaded transport starting inside the grown band backs out straight first (the trailer exit prefix, generalised)."""

import dataclasses
import math

import numpy as np
import pytest

from forklift_core.mission.replay_codec import recorded_fields
from forklift_core.planning import Bounds, Footprint, PlannerConfig, Pose2D, Rectangle
from forklift_core.planning.pallet_mission import (
    AssetSpec,
    ExitPrefix,
    PalletSite,
    PlacedProp,
    SyntheticMissionGeometry,
    TransportScenario,
    footprint_clearance_m,
    leg_obstacles,
    lidar_visible_props,
    plan_loaded_delivery,
    plan_transport_leg,
    scenario_props,
)

# The Riibotics trailer scene as tests/unit/planning/test_exit_prefix.py builds it: two 1.36 m walls
WALL_PROPS = tuple(
    PlacedProp(AssetSpec("riibotics:trailer_wall", 3.322142857142857, 2.6000000000000005, 1.36),
               Rectangle(3.3, side * 2.3389285714285712, 3.322142857142857, 2.6000000000000005, -math.pi / 2))
    for side in (1, -1))
LOW = PlacedProp(AssetSpec("riibotics:block", 1.0, 1.0, 0.144), Rectangle(-3.8, 3.2, 1.0, 1.0, 0.0))  # away from every leg
BASE = dict(unloaded_footprint=Footprint(1.29, 0.17, 0.36), alignment_straight_m=2.1, delivery_straight_m=1.5,
            withdrawal_m=0.55, carriage_limit_m=0.346, insertion_reserve_m=0.016)
GEOMETRY = SyntheticMissionGeometry(**BASE)
CONFIG = PlannerConfig(curvature_limit_inv_m=0.29, xy_resolution_m=0.2, yaw_resolution_rad=math.radians(10),
                       primitive_length_m=0.25, collision_step_m=0.06, max_expansions=30000,
                       analytic_expansion_interval=8, heuristic_weight=1.8, reverse_penalty=1.3,
                       gear_change_penalty_m=1.0, steering_penalty=0.15, steering_change_penalty_m=0.15, clearance_m=0.1,
                       goal_connection="reeds_shepp", rs_min_segment_m=0.1, shot_cap_m=4.0, shot_cap_patience=2000)
EXTRACTED = Pose2D(0.5511635203935266, 0.11665803870177686, -0.03490658503988664)  # t1_trailer's extracted pose


def scenario(props=WALL_PROPS + (LOW,)):
    return TransportScenario(
        seed=0, start_rear=Pose2D(-1.0, 0.0, 0.0),
        pickup=PalletSite(2.46, 0.05, -0.0349), destination=PalletSite(-2.2, -2.6, -math.pi / 2),
        props=props, bounds=Bounds(-4.5, 4.6, -4.0, 4.0))


def keepout(g):
    return SyntheticMissionGeometry(**BASE, lidar_keepout_m=g)


def test_only_props_taller_than_the_lidar_plane_grow():
    grown = scenario_props(scenario(), keepout_m=0.15)
    plain = scenario_props(scenario())
    for g, p, prop in zip(grown, plain, scenario().props):
        tall = prop.asset.height_m > 1.05
        assert g.length_m == pytest.approx(p.length_m + (0.30 if tall else 0.0))
        assert g.width_m == pytest.approx(p.width_m + (0.30 if tall else 0.0))
        assert (g.x_m, g.y_m, g.yaw_rad) == (p.x_m, p.y_m, p.yaw_rad)
    assert [r for r in lidar_visible_props(scenario())] == plain[:2]


def test_no_keepout_gives_the_same_rectangles_as_before():
    assert scenario_props(scenario(), keepout_m=0.0) == scenario_props(scenario())
    rects, foot, bounds = leg_obstacles("transport", scenario(), GEOMETRY)
    assert rects == scenario_props(scenario())


def test_the_search_obstacles_of_every_leg_grow_with_the_geometry():
    for kind in ("observe", "transport", "return"):
        plain, _, _ = leg_obstacles(kind, scenario(), GEOMETRY)
        grown, _, _ = leg_obstacles(kind, scenario(), keepout(0.2))
        assert grown[0].length_m == pytest.approx(plain[0].length_m + 0.4)
        assert grown[2] == plain[2]  # the low block


def test_without_keepout_the_transport_leg_is_unchanged():
    a = plan_transport_leg(scenario(), EXTRACTED, CONFIG, geometry=GEOMETRY)
    b = plan_transport_leg(scenario(), EXTRACTED, CONFIG, geometry=keepout(0.0))
    assert a.status == b.status and np.array_equal(a.poses, b.poses) and a.exit_prefix == b.exit_prefix


def test_a_transport_leg_starting_in_the_grown_band_backs_out_first_and_keeps_the_keepout():
    g = 0.10
    plan = plan_transport_leg(scenario(), EXTRACTED, CONFIG, geometry=keepout(g))
    assert plan.success, plan.status
    record = plan.exit_prefix
    assert record["status"] == "prefix" and record["clearance_m"] == pytest.approx(1.5 * (g + 0.10) + 0.01)
    travelled = np.r_[0.0, np.cumsum(np.hypot(*np.diff(plan.poses[:, :2], axis=0).T))]
    after = plan.poses[travelled > record["length_m"] + 1e-9]
    walls = lidar_visible_props(scenario())
    gap = min(footprint_clearance_m(Pose2D(*p), GEOMETRY.loaded_footprint, walls) for p in after)
    assert gap >= g + 0.10 - 1e-6  # the search keeps the keepout plus its clearance from the walls


def test_an_explicit_exit_prefix_is_kept():
    explicit = ExitPrefix(tuple(lidar_visible_props(scenario())), 0.30)
    plan = plan_transport_leg(scenario(), EXTRACTED, CONFIG, geometry=keepout(0.10), exit_prefix=explicit)
    assert plan.exit_prefix["clearance_m"] == pytest.approx(0.30)


def test_the_loaded_delivery_uses_the_same_rule():
    plan = plan_loaded_delivery(scenario(), EXTRACTED, CONFIG, geometry=keepout(0.10))
    assert plan.success, plan.status
    assert plan.transport.exit_prefix["status"] == "prefix"


def test_the_prefix_straight_is_checked_at_the_props_real_size():
    # the extracted pose sits 0.2 m from the wall face: inside the 0.10 + 0.10 grown band, outside the real wall
    walls = lidar_visible_props(scenario())
    assert footprint_clearance_m(EXTRACTED, GEOMETRY.loaded_footprint, walls) > 0.0
    plan = plan_transport_leg(scenario(), EXTRACTED, CONFIG, geometry=keepout(0.10))
    assert plan.exit_prefix["status"] != "unmet:straight_collision"


def test_the_geometry_records_as_before_while_the_keepout_is_off():
    names = {f.name for f in recorded_fields(GEOMETRY)}
    assert "lidar_keepout_m" not in names and "lidar_plane_m" not in names
    assert "lidar_keepout_m" in {f.name for f in recorded_fields(keepout(0.1))}


def test_a_negative_keepout_is_refused():
    with pytest.raises(ValueError):
        keepout(-0.01)
