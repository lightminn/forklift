"""The transport exit prefix (docs/plans/2026-10-22-riibotics-trailer-exit.md D1): a loaded transport leg starts with a
straight reverse until the loaded footprint is clear of the scene's walls by the exit clearance, so the first turn
does not swing the pallet into the obstacle layer's band around a trailer mouth."""

import math
from dataclasses import replace

import numpy as np
import pytest

from forklift_core.planning import Bounds, Footprint, PlannerConfig, Pose2D, Rectangle
from forklift_core.planning import pallet_mission as pm
from forklift_core.planning.pallet_mission import (
    AssetSpec,
    ExitPrefix,
    PalletSite,
    PlacedProp,
    SyntheticMissionGeometry,
    TransportScenario,
    exit_prefix_length_m,
    footprint_clearance_m,
    plan_loaded_delivery,
    plan_transport_leg,
)

# The Riibotics trailer scene (scenes_riib_b35fcc9/trailer.json): the planner's keep-out rectangles of the two
# side walls and the thin walls Isaac collides with (the exit obstacles).
WALL_PROPS = tuple(
    PlacedProp(AssetSpec("riibotics:trailer_wall", 3.322142857142857, 2.6000000000000005, 1.36),
               Rectangle(3.3, side * 2.3389285714285712, 3.322142857142857, 2.6000000000000005, -math.pi / 2))
    for side in (1, -1))
EXIT_WALLS = tuple(Rectangle(3.34, side * 0.717857142857143, 0.08, 2.6800000000000006, -math.pi / 2) for side in (1, -1))
GEOMETRY = SyntheticMissionGeometry(unloaded_footprint=Footprint(1.29, 0.17, 0.36), alignment_straight_m=2.1,
                                    delivery_straight_m=1.5, withdrawal_m=0.55, carriage_limit_m=0.346,
                                    insertion_reserve_m=0.016)
CONFIG = PlannerConfig(curvature_limit_inv_m=0.29, xy_resolution_m=0.2, yaw_resolution_rad=math.radians(10),
                       primitive_length_m=0.25, collision_step_m=0.06, max_expansions=30000,
                       analytic_expansion_interval=8, heuristic_weight=1.8, reverse_penalty=1.3,
                       gear_change_penalty_m=1.0, steering_penalty=0.15, steering_change_penalty_m=0.15, clearance_m=0.1,
                       goal_connection="reeds_shepp", rs_min_segment_m=0.1, shot_cap_m=4.0, shot_cap_patience=2000)
EXTRACTED = Pose2D(0.5511635203935266, 0.11665803870177686, -0.03490658503988664)  # t1_trailer's extracted pose


def scenario(props=WALL_PROPS):
    return TransportScenario(
        seed=0, start_rear=Pose2D(-1.0, 0.0, 0.0),
        pickup=PalletSite(2.46, 0.05, -0.0349), destination=PalletSite(-2.2, -2.6, -math.pi / 2),
        props=props, bounds=Bounds(-4.5, 4.6, -4.0, 4.0))


def back(pose, d):
    return Pose2D(pose.x_m - d * math.cos(pose.yaw_rad), pose.y_m - d * math.sin(pose.yaw_rad), pose.yaw_rad)


def test_the_clearance_is_the_gap_between_the_footprint_and_the_nearest_rectangle():
    fp = Footprint(1.0, 0.0, 0.5)  # x 0..1, y -0.5..0.5 at the origin
    assert footprint_clearance_m(Pose2D(0, 0, 0), fp, [Rectangle(2.5, 0, 1.0, 1.0)]) == pytest.approx(1.0)
    assert footprint_clearance_m(Pose2D(0, 0, 0), fp, [Rectangle(2.0, 2.0, 1.0, 1.0)]) == pytest.approx(math.hypot(0.5, 1.0))
    assert footprint_clearance_m(Pose2D(0, 0, 0), fp, [Rectangle(0.5, 0, 0.2, 0.2)]) == 0.0  # inside: overlapping
    assert footprint_clearance_m(Pose2D(0, 0, 0), fp, []) == math.inf


def test_the_prefix_is_the_shortest_straight_reverse_that_clears_the_walls():
    exit_ = ExitPrefix(EXIT_WALLS, 0.30)
    loaded = GEOMETRY.loaded_footprint
    delta = exit_prefix_length_m(EXTRACTED, loaded, exit_)
    assert delta is not None and delta > 0
    assert footprint_clearance_m(back(EXTRACTED, delta), loaded, EXIT_WALLS) >= 0.30
    assert footprint_clearance_m(back(EXTRACTED, delta - exit_.step_m), loaded, EXIT_WALLS) < 0.30
    assert footprint_clearance_m(EXTRACTED, loaded, EXIT_WALLS) < 0.25  # where t1_trailer stopped


def test_a_transport_leg_with_an_exit_prefix_reverses_straight_first():
    exit_ = ExitPrefix(EXIT_WALLS, 0.30)
    plan = plan_transport_leg(scenario(), EXTRACTED, CONFIG, geometry=GEOMETRY, exit_prefix=exit_)
    assert plan.success and plan.exit_prefix["status"] == "prefix"
    delta = plan.exit_prefix["length_m"]
    assert delta == exit_prefix_length_m(EXTRACTED, GEOMETRY.loaded_footprint, exit_)
    travelled = np.r_[0.0, np.cumsum(np.hypot(*np.diff(plan.poses[:, :2], axis=0).T))]
    inside = travelled <= delta + 1e-9
    assert np.all(plan.directions[inside] == -1) and np.all(plan.curvatures_inv_m[inside] == 0.0)
    assert np.allclose(plan.poses[inside, 2], EXTRACTED.yaw_rad)
    assert plan.exit_prefix["min_clearance_m"] >= 0.30
    assert plan.poses[0].tolist() == pytest.approx([EXTRACTED.x_m, EXTRACTED.y_m, EXTRACTED.yaw_rad])


def test_without_an_exit_prefix_the_leg_is_planned_as_before():
    plain = plan_transport_leg(scenario(), EXTRACTED, CONFIG, geometry=GEOMETRY)
    explicit = plan_transport_leg(scenario(), EXTRACTED, CONFIG, geometry=GEOMETRY, exit_prefix=None)
    assert plain.exit_prefix is None and explicit.exit_prefix is None
    assert np.array_equal(plain.poses, explicit.poses) and plain.status == explicit.status


def test_walls_already_clear_add_no_prefix_and_the_path_is_the_plain_one():
    far = ExitPrefix((Rectangle(3.0, 3.6, 0.08, 0.5),), 0.30)
    plain = plan_transport_leg(scenario(), EXTRACTED, CONFIG, geometry=GEOMETRY)
    leg = plan_transport_leg(scenario(), EXTRACTED, CONFIG, geometry=GEOMETRY, exit_prefix=far)
    assert leg.exit_prefix["status"] == "clear" and leg.exit_prefix["length_m"] == 0.0
    assert np.array_equal(plain.poses, leg.poses) and np.array_equal(plain.directions, leg.directions)


def test_no_clear_pose_within_the_cap_plans_as_before_and_says_unmet():
    capped = ExitPrefix(EXIT_WALLS, 0.30, cap_m=0.10)
    plain = plan_transport_leg(scenario(), EXTRACTED, CONFIG, geometry=GEOMETRY)
    leg = plan_transport_leg(scenario(), EXTRACTED, CONFIG, geometry=GEOMETRY, exit_prefix=capped)
    assert leg.exit_prefix["status"] == "unmet:clearance" and leg.exit_prefix["length_m"] == 0.0
    assert np.array_equal(plain.poses, leg.poses)


def test_a_straight_that_would_hit_a_prop_is_not_taken():
    behind = PlacedProp(AssetSpec("test://post", 0.2, 0.2, 1.5), Rectangle(0.1, 0.12, 0.2, 0.2))  # in the reverse's sweep
    exit_ = ExitPrefix(EXIT_WALLS, 0.30)
    leg = plan_transport_leg(scenario(WALL_PROPS + (behind,)), EXTRACTED, CONFIG, geometry=GEOMETRY, exit_prefix=exit_)
    assert leg.exit_prefix["status"] == "unmet:straight_collision" and leg.exit_prefix["length_m"] == 0.0


def test_the_loaded_delivery_and_the_whole_mission_take_the_prefix_too():
    exit_ = ExitPrefix(EXIT_WALLS, 0.30)
    delivery = plan_loaded_delivery(scenario(), EXTRACTED, CONFIG, geometry=GEOMETRY, exit_prefix=exit_)
    assert delivery.success and delivery.transport.exit_prefix["status"] == "prefix"
    assert delivery.transport.exit_prefix["length_m"] == exit_prefix_length_m(EXTRACTED, GEOMETRY.loaded_footprint, exit_)
    mission = pm.plan_transport(scenario(), CONFIG, geometry=GEOMETRY, exit_prefix=exit_)  # the whole mission
    assert mission.success and mission.transport.exit_prefix["status"] == "prefix"
    assert mission.transport.exit_prefix["length_m"] > 0 and np.all(mission.transport.directions[:3] == -1)
    assert pm.plan_transport(scenario(), CONFIG, geometry=GEOMETRY).transport.exit_prefix is None


def test_a_plan_without_a_prefix_records_as_before_and_one_with_it_keeps_it(tmp_path):
    # Codex code review P2: a boundary record from before the field must replay -- the field is left out of the
    # record while it is None, and kept (and compared) when set
    from forklift_core.mission.boundary_record import BoundaryRecorder
    from forklift_core.mission.replay_codec import decode, encode

    from forklift_core.planning.hybrid_astar import PlanResult

    plain = PlanResult(True, "success", np.zeros((2, 3)), np.ones(2, dtype=np.int8), np.zeros(2), 0.1, 3)
    marked = PlanResult(True, "success", np.zeros((2, 3)), np.ones(2, dtype=np.int8), np.zeros(2), 0.1, 3,
                        exit_prefix={"status": "prefix", "length_m": 0.35})
    node, arrays = encode(plain, opaque_ok=True)
    assert "exit_prefix" not in node["fields"] and "turnaround" in node["fields"]
    assert decode(node, arrays).exit_prefix is None  # an old record's PlanResult comes back with the default
    node, arrays = encode(marked, opaque_ok=True)
    assert decode(node, arrays).exit_prefix == marked.exit_prefix
    recorder = BoundaryRecorder(tmp_path / "boundary")  # format 1: the comparison form every older record holds
    assert "exit_prefix" not in recorder._encode(plain) and recorder._encode(marked)["exit_prefix"]["length_m"] == 0.35


@pytest.mark.parametrize("kwargs", [dict(clearance_m=0.0), dict(clearance_m=-0.1), dict(clearance_m=math.nan),
                                    dict(clearance_m=0.3, step_m=0.0), dict(clearance_m=0.3, cap_m=-0.1),
                                    dict(clearance_m=0.3, cap_m=math.inf)])
def test_bad_exit_settings_are_refused(kwargs):
    with pytest.raises(ValueError):
        ExitPrefix(EXIT_WALLS, **kwargs)


def test_exit_obstacles_must_be_rectangles():
    with pytest.raises(TypeError):
        ExitPrefix(((3.0, 0.7, 0.08, 2.6, 0.0),), 0.3)


def test_a_failed_search_from_the_prefix_end_is_recorded_beside_the_plain_fallback(monkeypatch):
    """docs/validation/2026-10-31-invalid-start-diagnosis.md 2 (a): rxi_box15_aisle_rack_s1 found its prefix, the search
    from the prefix end failed, and the plain fallback from the in-slot start said invalid_start -- the record keeps the
    prefix-end search's own status and attempts, and the failure reason names both."""
    exit_ = ExitPrefix(EXIT_WALLS, 0.30)
    real = pm._search

    def search(start, goal, *a, **k):
        if abs(start.x_m - EXTRACTED.x_m) > 1e-9:  # the prefix end
            failed = real(start, goal, *a, **k)
            return replace(failed, success=False, status="no_path",
                           search_attempts=((8, "no_path", 353), (8, "expansion_limit", 30000)))
        return replace(real(start, goal, *a, **k), success=False, status="invalid_start")

    monkeypatch.setattr(pm, "_search", search)
    plan = plan_transport_leg(scenario(), EXTRACTED, CONFIG, geometry=GEOMETRY, exit_prefix=exit_)
    assert plan.status == "invalid_start" and plan.exit_prefix["status"] == "unmet:search_no_path"
    assert plan.exit_prefix["end_search"] == {"status": "no_path",
                                              "attempts": [[8, "no_path", 353], [8, "expansion_limit", 30000]]}
    assert pm.failed_plan_reason(plan) == "invalid_start (exit_prefix unmet:search_no_path)"


def test_the_failure_reason_is_the_status_alone_without_an_unmet_record():
    plain = plan_transport_leg(scenario(), EXTRACTED, CONFIG, geometry=GEOMETRY)
    failed = replace(plain, success=False, status="invalid_start")
    assert pm.failed_plan_reason(failed) == "invalid_start"
    capped = replace(failed, exit_prefix={"status": "unmet:clearance", "length_m": 0.0})
    assert pm.failed_plan_reason(capped) == "invalid_start (exit_prefix unmet:clearance)"
    backed = replace(failed, start_backoff={"status": "unmet:search_invalid_goal", "length_m": 0.0})
    assert pm.failed_plan_reason(backed) == "invalid_start (start_backoff unmet:search_invalid_goal)"
    assert pm.failed_plan_reason(replace(failed, exit_prefix={"status": "prefix"})) == "invalid_start"


def test_the_release_replan_failures_name_the_hidden_attempt():
    """Both release paths (the tick's and the real-time worker's) say what the final status may hide; a plan with no unmet
    record keeps the message every earlier record has (failed_plan_reason is the status alone)."""
    from pathlib import Path

    source = (Path(pm.__file__).resolve().parents[1] / "mission/core.py").read_text()
    assert source.count('f"slam_release_replan_failed: {failed_plan_reason(replanned)}"') == 2
    assert 'slam_release_replan_failed: {replanned.status}' not in source
