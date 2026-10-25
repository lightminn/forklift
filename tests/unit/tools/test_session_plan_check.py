"""tools/session_plan_check.py (plan 2026-10-22-session-scene-pallet-props 2.7): gaps between the planned outline and the
true pallets, between poses too, the stage table, and the order's prior_failed."""

import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.planning.geometry import Footprint, Rectangle
from tools import session_plan_check as C

FP = Footprint(0.5, 0.5, 0.5)


def test_the_gap_between_an_outline_and_a_box_is_exact_and_zero_when_they_overlap():
    box = C.rect_corners(Rectangle(3.0, 0.0, 1.0, 1.0, 0.0))
    assert C.distance(C.footprint_corners((0.0, 0.0, 0.0), Footprint(1.0, 0.0, 0.5)), box) == pytest.approx(1.5)
    assert C.distance(C.footprint_corners((2.4, 0.0, 0.0), Footprint(1.0, 0.0, 0.5)), box) == 0.0


def test_a_contact_between_two_clear_poses_is_caught():
    rect = {"P-2": Rectangle(0.0, 0.55, 0.1, 0.1, 0.0)}  # the outline's side passes through it half way
    out = C.leg_clearance([(-1.0, 0.0, 0.0), (1.0, 0.0, 0.0)], FP, rect)["P-2"]
    assert out["status"] == "contact_observed" and out["min_m"] == 0.0


def test_a_millimetre_parallel_gap_is_proven_clear_at_the_fine_step():
    rect = {"P-2": Rectangle(0.0, 0.5 + 0.001 + 0.05, 4.0, 0.1, 0.0)}
    out = C.leg_clearance([(-0.5, 0.0, 0.0), (0.5, 0.0, 0.0)], FP, rect)["P-2"]
    assert out["min_m"] == pytest.approx(0.001) and out["status"] == "clear"
    assert out["bound_m"] == pytest.approx(0.001 - C.FINE_STEP_M / 2)


def test_a_turn_is_sampled_on_its_arc_not_its_chord():
    """Codex PR 2 implementation round 1 P2: a half circle of radius 1/0.29 from (-R, R, -pi/2) to (R, R, pi/2) passes
    the box at its bottom; the chord between the two poses does not."""
    r = 1 / 0.29
    a, b = (-r, r, -math.pi / 2), (r, r, math.pi / 2)
    pose_at, length = C.arc(a, b, direction=1)
    assert length == pytest.approx(math.pi * r)
    assert pose_at(0.5)[:2] == pytest.approx((0.0, 0.0), abs=1e-9)
    rect = {"P-9": Rectangle(0.0, -0.759, 0.6, 0.8, 0.0)}
    out = C.leg_clearance([a, b], Footprint(1.29, 0.17, 0.36), rect, directions=[1, 1])["P-9"]
    assert out["status"] == "contact_observed"


@pytest.mark.parametrize("end_yaw", [math.pi, -math.pi])
def test_a_half_circle_turns_the_way_the_truck_drives_whatever_the_end_headings_sign(end_yaw):
    """Codex PR 2 implementation round 2 P2: forwards from (0, 0, 0) round a half circle to (0, 2R) the planner may
    write the end heading as -pi; the arc must still pass (R, R), where a box stands."""
    r = 1 / 0.29
    a, b = (0.0, 0.0, 0.0), (0.0, 2 * r, end_yaw)
    pose_at, _ = C.arc(a, b, direction=1)
    assert pose_at(0.5)[:2] == pytest.approx((r, r), abs=1e-9)
    rect = {"P-9": Rectangle(r + 0.759, r, 0.6, 0.8, math.pi / 2)}
    out = C.leg_clearance([a, b], Footprint(1.29, 0.17, 0.36), rect, directions=[1, 1])["P-9"]
    assert out["status"] == "contact_observed"
    with pytest.raises(ValueError, match="ambiguous"):
        C.arc(a, b)  # a half circle without its direction


def test_a_reversing_turn_is_sampled_on_its_arc():
    """Backwards a quarter circle about the origin: (R, 0) heading +y, reversing clockwise to (0, -R) heading +x."""
    r = 2.0
    a, b = (r, 0.0, math.pi / 2), (0.0, -r, 0.0)
    pose_at, length = C.arc(a, b, direction=-1)
    assert length == pytest.approx(math.pi / 2 * r)
    assert pose_at(0.5)[:2] == pytest.approx((r / math.sqrt(2), -r / math.sqrt(2)), abs=1e-9)
    assert pose_at(1.0)[:2] == pytest.approx(b[:2], abs=1e-9)


def test_an_arc_ends_where_the_next_pose_is():
    a, b = (0.0, 0.0, 0.0), (1.0, 0.3, 0.6)
    pose_at, _ = C.arc(a, b)
    assert pose_at(1.0)[:2] == pytest.approx(b[:2], abs=1e-9) and pose_at(1.0)[2] == pytest.approx(0.6)


# --- the stage table and the order, with stand-in planners -----------------------------------------------------------
def _session(tmp_path, n=2):
    from forklift_core.planning.pallet_mission import AssetSpec  # noqa: F401

    pallets = []
    for i in range(n):
        y = 3.0 * i
        site = {"x_m": 5.0, "y_m": y, "yaw_rad": 0.0}
        pallets.append({"pallet_id": f"P-{i + 1}", "nominal": site, "truth": site,
                        "zone": {"x_min_m": 4.75, "x_max_m": 5.25, "y_min_m": y - 0.25, "y_max_m": y + 0.25,
                                 "yaw_centre_rad": 0.0, "yaw_half_rad": 0.2},
                        "observation_waypoints": [[1.6, y, 0.0]]})
    record = {"format": "riibotics_isaac_session_v1", "name": "toy", "pallet": "epal6",
              "start_rear": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
              "bounds": {"x_min_m": -5.0, "x_max_m": 20.0, "y_min_m": -10.0, "y_max_m": 20.0}, "static_props": [],
              "pallets": pallets,
              "destinations": [{"destination_id": f"D-{i + 1}", "x_m": 15.0, "y_m": 3.0 * i, "yaw_rad": 0.0}
                               for i in range(n)],
              "sequence": [[f"P-{i + 1}", f"D-{i + 1}"] for i in range(n)], "visuals": []}
    path = tmp_path / "toy.json"
    path.write_text(json.dumps(record))
    return path


def _leg(poses, status="success", direction=1):
    poses = np.asarray(poses, dtype=float)
    return SimpleNamespace(success=status == "success", status=status, poses=poses,
                           directions=np.full(len(poses), direction), curvatures_inv_m=np.zeros(len(poses)))


def _planners(approach_end_x=None, insert_into=True, fail_first=False, legs_override=None):
    calls = {"n": 0}

    def observe(scenario, waypoint, config, **kw):
        return _leg([(0.0, scenario.pickup.y_m, 0.0), (waypoint.x_m, waypoint.y_m, 0.0)])

    def transport(scenario, config, **kw):
        calls["n"] += 1
        y = scenario.pickup.y_m
        if fail_first and calls["n"] == 1:
            return SimpleNamespace(success=False, status="approach:no_path")
        face = scenario.pickup.x_m - 0.3  # the pallet's near face (the scenes' EPAL 6: 0.6 m deep)
        front = 1.29  # the measured truck's axle to fork tip, about
        end = face - front - 0.10 if approach_end_x is None else approach_end_x
        legs = {"approach": _leg([(1.6, y, 0.0), (end, y, 0.0)]),
                "insert": _leg([(end, y, 0.0), (end + (0.4 if insert_into else 0.0), y, 0.0)]),
                "extract": _leg([(end + 0.4, y, 0.0), (end - 0.3, y, 0.0)], direction=-1),
                "transport": _leg([(end - 0.3, y, 0.0), (12.0, scenario.destination.y_m, 0.0)]),
                "withdraw": _leg([(12.0, scenario.destination.y_m, 0.0), (11.5, scenario.destination.y_m, 0.0)],
                                 direction=-1),
                "return_home": _leg([(11.5, scenario.destination.y_m, 0.0), (0.0, 0.0, 0.0)], direction=-1)}
        legs.update((legs_override or {})(scenario) if legs_override else {})
        return SimpleNamespace(success=True, status="success", **legs)

    return observe, transport


def test_the_insert_may_enter_its_own_pallet_but_the_approach_may_not(tmp_path):
    path = _session(tmp_path, n=1)
    ok = C.check_session(path, planner=_planners())
    assert ok["missions"][0]["outcome"] == "passed"
    assert ok["missions"][0]["stages"]["approach"]["nearest"] == "P-1"  # its own truth counts on the approach
    assert "P-1" not in ok["missions"][0]["stages"]["insert"]["all"]
    geometry = C.mission_geometry(C.URDF, C.load_pallet_geometry(C.PALLETS["epal6"]), C.RESERVE_M, **C.STRAIGHTS)
    deep = 5.0 - 0.3 - geometry.unloaded_footprint.front_m + 0.01  # the approach ends 1 cm inside the pallet
    bad = C.check_session(path, planner=_planners(approach_end_x=deep))
    assert bad["missions"][0]["outcome"] == "contact_observed"


def test_a_failed_mission_stops_the_order_and_a_passed_one_delivers_its_pallet(tmp_path):
    path = _session(tmp_path, n=2)
    out = C.check_session(path, planner=_planners(fail_first=True))
    assert [m["outcome"] for m in out["missions"]] == ["plan_failed", "prior_failed"]
    out = C.check_session(path, planner=_planners())
    assert [m["outcome"] for m in out["missions"]] == ["passed", "passed"]
    # the second mission sees the first pallet at its slot (15, 0), not where it stood
    second = out["missions"][1]["stages"]["observe"]["all"]["P-1"]
    assert second["min_m"] > 5.0
    assert out["settings"]["kappa"] == C.KAPPA and out["prior_grow_m"] == C.SESSION_PRIOR_GROW_M == 0.20


def test_a_loaded_leg_is_judged_with_the_loaded_outline(tmp_path):
    """A box the empty outline clears by 2 cm, the loaded (wider) one overlaps: transport catches it, observe would not."""
    path = _session(tmp_path, n=2)
    geometry = C.mission_geometry(C.URDF, C.load_pallet_geometry(C.PALLETS["epal6"]), C.RESERVE_M, **C.STRAIGHTS)
    empty, loaded = geometry.unloaded_footprint.half_width_m, geometry.loaded_footprint.half_width_m
    assert loaded > empty + 0.02
    # P-2 truth stands at (5, 3); a transport passing 0.4 m (the pallet's half width) + empty + 0.02 below its centre
    y = 3.0 - 0.4 - empty - 0.02
    over = lambda sc: {"transport": _leg([(2.0, y, 0.0), (8.0, y, 0.0)])} if sc.pickup.y_m == 0.0 else {}  # noqa: E731
    out = C.check_session(path, planner=_planners(legs_override=over))
    first = out["missions"][0]
    assert first["outcome"] == "contact_observed" and first["stages"]["transport"]["nearest"] == "P-2"
    assert [m["outcome"] for m in out["missions"]] == ["contact_observed", "prior_failed"]


def test_the_withdraw_meets_the_others_and_the_return_meets_its_own_delivered_pallet(tmp_path):
    path = _session(tmp_path, n=2)
    # the withdraw backs through P-2's place (5, 3)
    through = lambda sc: {"withdraw": _leg([(4.5, 3.0, 0.0), (3.5, 3.0, 0.0)], direction=-1)} if sc.pickup.y_m == 0.0 else {}  # noqa: E731
    out = C.check_session(path, planner=_planners(legs_override=through))
    assert out["missions"][0]["outcome"] == "contact_observed"
    assert out["missions"][0]["stages"]["withdraw"]["nearest"] == "P-2"
    # the return drives back over the slot it just filled (15, 0)
    back = lambda sc: {"return_home": _leg([(15.5, 0.0, 0.0), (13.0, 0.0, 0.0)], direction=-1)} if sc.pickup.y_m == 0.0 else {}  # noqa: E731
    out = C.check_session(path, planner=_planners(legs_override=back))
    assert out["missions"][0]["outcome"] == "contact_observed"
    assert out["missions"][0]["stages"]["return_home"]["nearest"] == "P-1"


def test_the_session_check_hands_the_planners_directions_to_the_arcs(tmp_path):
    """Codex PR 2 implementation round 3 P3: a half-circle transport whose bottom passes P-2's place is a contact only
    when check_session passes the leg's directions on (without them the half circle is refused)."""
    path = _session(tmp_path, n=2)
    r = 1 / 0.29

    def half(sc):
        if sc.pickup.y_m != 0.0:
            return {}
        return {"transport": _leg([(5.0 - r, 3.0 + r, -math.pi / 2), (5.0 + r, 3.0 + r, math.pi / 2)])}

    out = C.check_session(path, planner=_planners(legs_override=half))
    assert out["missions"][0]["outcome"] == "contact_observed"
    assert out["missions"][0]["stages"]["transport"]["nearest"] == "P-2"


def test_the_keepout_reaches_the_planners_and_the_tall_props_gap_is_recorded(tmp_path):
    # docs/validation/2026-10-25-ct1-session-keepout.md: a slot marker taller than the LiDAR plane beside the return
    path = _session(tmp_path, n=1)
    record = json.loads(path.read_text())
    record["static_props"] = [{"asset": {"uri": "riibotics:slot_marker", "length_m": 0.9, "width_m": 0.03,
                                         "height_m": 1.3},
                               "rectangle": {"x_m": 6.0, "y_m": 0.36 + 0.25 + 0.015, "length_m": 0.9, "width_m": 0.03,
                                             "yaw_rad": 0.0}}]
    path.write_text(json.dumps(record))
    seen = []
    observe, transport = _planners()

    def transport_(scenario, config, **kw):
        seen.append(kw["geometry"].lidar_keepout_m)
        return transport(scenario, config, **kw)

    out = C.check_session(path, planner=(observe, transport_), lidar_keepout_m=0.15)
    assert seen == [0.15] and out["settings"]["lidar_keepout_m"] == 0.15
    stage = out["missions"][0]["stages"]["return_home"]
    assert stage["tall_nearest"] == "tall:0" and stage["tall_bound_m"] == pytest.approx(0.25, abs=0.01)
    assert out["missions"][0]["outcome"] == "passed"  # a record beside the outcome, not part of it
