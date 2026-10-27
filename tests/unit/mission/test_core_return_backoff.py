"""The return-start backoff in the mission core and the planning worker (docs/plans/2026-10-27-return-start-withdraw-
extension.md R1, R2, R5, R6): the release and first-live replans get the cap and the slot-stage props moved with the
destination since the preparation; the worker keeps the first try's attempt when a tight retry replaces it; every
attempt is recorded with whether it was taken."""

import json
import math
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from forklift_core.mission.baseline import MISSION_KEYS
from forklift_core.mission.core import MissionCore, note_return_backoff, note_trace_backoffs
from forklift_core.planning import Footprint, PlannerConfig, Pose2D, Rectangle
from forklift_core.planning import pallet_mission as pm
from forklift_core.planning.pallet_mission import (
    AssetSpec,
    Bounds,
    PalletSite,
    PlacedProp,
    SyntheticMissionGeometry,
    TransportScenario,
    site_poses,
)
from forklift_core.planning.plan_worker import _run

FIXTURE = Path(__file__).resolve().parents[2] / "fixtures/return_start_backoff_rh_s8_m3.json"
GEOMETRY = SyntheticMissionGeometry(unloaded_footprint=Footprint(1.29, 0.17, 0.36), alignment_straight_m=2.1,
                                    delivery_straight_m=1.5, withdrawal_m=0.55, carriage_limit_m=0.346,
                                    insertion_reserve_m=0.016)
CONFIG = PlannerConfig(curvature_limit_inv_m=0.29, xy_resolution_m=0.2, yaw_resolution_rad=math.radians(10),
                       primitive_length_m=0.25, collision_step_m=0.06, max_expansions=30000,
                       analytic_expansion_interval=8, heuristic_weight=1.8, reverse_penalty=1.3,
                       gear_change_penalty_m=1.0, steering_penalty=0.15, steering_change_penalty_m=0.15, clearance_m=0.1,
                       goal_connection="reeds_shepp", rs_min_segment_m=0.1, shot_cap_m=4.0, shot_cap_patience=2000)


def recorded() -> TransportScenario:
    s = json.loads(FIXTURE.read_text())["mission_scenario"]
    return TransportScenario(
        seed=int(s["seed"]), start_rear=Pose2D(**s["start_rear"]), pickup=PalletSite(**s["pickup"]),
        destination=PalletSite(**s["destination"]),
        props=tuple(PlacedProp(AssetSpec(**p["asset"]), Rectangle(**p["rectangle"])) for p in s["props"]),
        bounds=Bounds(**s["bounds"]))


def kwargs_of(limit_m, prepared, world):
    host = SimpleNamespace(options=SimpleNamespace(return_start_backoff_m=limit_m, scenario=prepared))
    return MissionCore._return_backoff_kwargs(host, world)


def test_off_gives_the_replans_nothing_new():
    sc = recorded()
    assert kwargs_of(0.0, sc, sc) == {}
    assert MissionCore._return_backoff_kwargs(SimpleNamespace(options=SimpleNamespace(scenario=sc)), sc) == {}


def test_the_slot_props_move_with_the_destination_and_nothing_else_does():
    sc = recorded()
    moved = replace(sc, destination=replace(sc.destination, x_m=sc.destination.x_m + 0.05,
                                            y_m=sc.destination.y_m - 0.20))  # docking and release moved it
    kw = kwargs_of(pm.RETURN_START_BACKOFF_CAP_M, sc, moved)
    assert kw["return_start_backoff_m"] == pm.RETURN_START_BACKOFF_CAP_M
    got = kw["backoff_slot_props"]
    untagged = [p.rectangle for p in sc.props if not p.asset.uri.endswith((pm.SLOT_TAG, pm.SEARCH_TAG))]
    slots = [p.rectangle for p in sc.props if p.asset.uri.endswith(pm.SLOT_TAG)]
    assert len(got) == len(untagged) + len(slots)  # the @search estimates left out
    for r in untagged:  # walls, slot markers, priors: where they are
        assert r in got
    for r in slots:  # a pure translation here: each @slot moved by it
        assert any(abs(g.x_m - (r.x_m + 0.05)) < 1e-9 and abs(g.y_m - (r.y_m - 0.20)) < 1e-9
                   and (g.length_m, g.width_m, g.yaw_rad) == (r.length_m, r.width_m, r.yaw_rad) for g in got)


def test_the_slot_props_turn_about_the_destination():
    sc = recorded()
    d = sc.destination
    turned = replace(sc, destination=replace(d, yaw_rad=d.yaw_rad + 0.1))
    got = kwargs_of(0.3, sc, turned)["backoff_slot_props"]
    slot = next(p.rectangle for p in sc.props if p.asset.uri.endswith(pm.SLOT_TAG))
    dx, dy = slot.x_m - d.x_m, slot.y_m - d.y_m
    want = (d.x_m + dx * math.cos(0.1) - dy * math.sin(0.1), d.y_m + dx * math.sin(0.1) + dy * math.cos(0.1))
    assert any(abs(r.x_m - want[0]) < 1e-9 and abs(r.y_m - want[1]) < 1e-9
               and r.yaw_rad == pytest.approx(slot.yaw_rad + 0.1) for r in got)


def _leg_payload(backoff_m):
    sc = recorded()
    start = site_poses(sc.destination, GEOMETRY)["withdrawn"]
    extra = {} if backoff_m <= 0 else {"return_start_backoff_m": backoff_m}
    tight = replace(CONFIG, clearance_m=0.0)
    return {"kind": "return", "args": (sc, start, sc.start_rear, CONFIG),
            "kwargs": dict(geometry=GEOMETRY, travel_config=CONFIG, **extra), "retry_on": ("invalid_start",),
            "tight_args": (sc, start, sc.start_rear, tight),
            "tight_kwargs": dict(geometry=GEOMETRY, travel_config=tight, **extra)}


def test_the_worker_takes_the_backoff_before_the_tight_retry():
    answer = _run("leg", _leg_payload(pm.RETURN_START_BACKOFF_CAP_M))
    assert answer["result"].success and not answer["tight"]
    assert answer["result"].start_backoff["status"] == "prefix" and "first_backoff" not in answer


def test_the_worker_keeps_a_refused_attempt_the_tight_retry_replaced():
    answer = _run("leg", _leg_payload(0.10))  # no clear end within 0.10: refused, then the tight retry plans
    assert answer["tight"] and answer["result"].success
    assert answer["first_backoff"]["status"] == "unmet:no_advice"


def test_without_the_backoff_the_worker_answers_as_before():
    answer = _run("leg", _leg_payload(0.0))
    assert answer["tight"] and set(answer) == {"result", "tight"}


def _loaded_payload(backoff_m):
    sc = recorded()
    predelivery = site_poses(sc.destination, GEOMETRY)["predelivery"]
    start = Pose2D(predelivery.x_m - 1.0, predelivery.y_m, 0.0)
    extra = {} if backoff_m <= 0 else {"return_start_backoff_m": backoff_m}
    return {"args": (sc, start), "kwargs": {"config": CONFIG, "geometry": GEOMETRY, "return_to": sc.start_rear,
                                            **extra}, "keep_m": GEOMETRY.delivery_straight_m}


def test_the_loaded_job_returns_its_attempts():
    taken = _run("loaded_delivery", _loaded_payload(pm.RETURN_START_BACKOFF_CAP_M))
    assert taken["result"].success and [r["status"] for r in taken["return_backoffs"]] == ["prefix"]
    refused = _run("loaded_delivery", _loaded_payload(0.10))
    assert refused["result"].status == "return_home:invalid_start"
    assert [r["status"] for r in refused["return_backoffs"]] == ["unmet:no_advice"]
    assert "return_backoffs" not in _run("loaded_delivery", _loaded_payload(0.0))


def test_attempts_are_recorded_with_whether_they_were_taken():
    state = {}
    note_return_backoff(state, "release_replan", None, True)  # no attempt: nothing
    assert "return_start_backoffs" not in state
    note_return_backoff(state, "release_replan", {"status": "unmet:no_advice", "advice_m": None, "length_m": 0.0},
                        False)
    trace = [{"stage": "withdraw", "status": "success"},
             {"stage": "return_home", "status": "invalid_start", "start_backoff": {"status": "unmet:clipped"}},
             {"stage": "return_home", "status": "success", "start_backoff": {"status": "prefix", "length_m": 0.2}}]
    note_trace_backoffs(state, "mission_plan", trace, True)
    got = [(r["where"], r["status"], r["adopted"]) for r in state["return_start_backoffs"]]
    assert got == [("release_replan", "unmet:no_advice", False), ("mission_plan", "unmet:clipped", False),
                   ("mission_plan", "prefix", True)]
    note_trace_backoffs(state, "mission_plan", trace, False)  # a plan not taken: none of its attempts adopted
    assert not state["return_start_backoffs"][-1]["adopted"]


def test_the_records_and_the_runner_argument_are_registered():
    assert "return_start_backoffs" in MISSION_KEYS["state"]
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "sim/isaac"))
    from near_tracking import POST_MEASUREMENT_ARGUMENTS

    assert POST_MEASUREMENT_ARGUMENTS["return_start_backoff"] == "off"
