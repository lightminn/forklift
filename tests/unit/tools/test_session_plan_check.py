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
    out = C.leg_clearance([(-1.0, 0.0, 0.0), (1.0, 0.0, 0.0)], None, FP, rect)["P-2"]
    assert out["status"] == "contact_observed" and out["min_m"] == 0.0


def test_a_millimetre_parallel_gap_is_proven_clear_at_the_fine_step():
    rect = {"P-2": Rectangle(0.0, 0.5 + 0.001 + 0.05, 4.0, 0.1, 0.0)}
    out = C.leg_clearance([(-0.5, 0.0, 0.0), (0.5, 0.0, 0.0)], None, FP, rect)["P-2"]
    assert out["min_m"] == pytest.approx(0.001) and out["status"] == "clear" and out["bound_m"] > 0


def test_a_turn_counts_the_outlines_far_corner_and_the_arcs_sagitta():
    rect = {"P-2": Rectangle(0.0, 0.5 + 0.003 + 0.05, 4.0, 0.1, 0.0)}
    straight = C.leg_clearance([(-0.5, 0.0, 0.0), (0.5, 0.0, 0.0)], None, FP, rect)["P-2"]
    curved = C.leg_clearance([(-0.5, 0.0, 0.0), (0.5, 0.0, 0.0)], [0.0, 0.3], FP, rect)["P-2"]
    assert straight["status"] == "clear" and straight["bound_m"] == pytest.approx(0.003 - C.STEP_M / 2)
    # the arc's 37.5 mm sagitta is not covered by 3 mm, even sampled again at the fine step
    assert curved["bound_m"] == pytest.approx(0.003 - C.FINE_STEP_M / 2 - 0.3 * 1.0 / 8)
    assert curved["status"] == "clearance_unproven"


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


def _leg(poses, status="success"):
    return SimpleNamespace(success=status == "success", status=status, poses=np.asarray(poses, dtype=float),
                           curvatures_inv_m=np.zeros(len(poses)))


def _planners(approach_end_x=None, insert_into=True, fail_first=False):
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
                "extract": _leg([(end + 0.4, y, 0.0), (end - 0.3, y, 0.0)]),
                "transport": _leg([(end - 0.3, y, 0.0), (12.0, scenario.destination.y_m, 0.0)]),
                "withdraw": _leg([(12.0, scenario.destination.y_m, 0.0), (11.5, scenario.destination.y_m, 0.0)]),
                "return_home": _leg([(11.5, scenario.destination.y_m, 0.0), (0.0, 0.0, 0.0)])}
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
    assert out["settings"]["kappa"] == C.KAPPA and out["prior_grow_m"] == 0.25
