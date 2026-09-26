"""Execute the runner's planning boundary statements on CPU, without its SDK.

Only configuration/record assignments and the four planner calls are extracted;
this checks argument wiring and real core planning, not simulator execution.
"""

import ast
import json
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import pytest

from forklift_core.planning import Bounds, PlannerConfig, Pose2D, pallet_mission
from forklift_core.planning.pallet_mission import (
    PalletSite,
    SyntheticMissionGeometry,
    TransportScenario,
)

SCRIPT = Path(__file__).resolve().parents[2] / "sim/isaac/run_transport.py"


@pytest.mark.parametrize(
    "gap,clearance,approach_clearance",
    [(0.10, 0.12, 0.05), (0.04, 0.12, 0.02), (0.10, 0.01, 0.01)],
)
def test_all_runner_planning_calls_use_the_recorded_config(
    monkeypatch, gap, clearance, approach_clearance
):
    tree = ast.parse(SCRIPT.read_text())
    run = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "run"
    )
    assignments = []
    for node in run.body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if isinstance(target, ast.Name) and target.id == "planner_config":
            assignments.append(node)
        elif (
            isinstance(target, ast.Subscript)
            and isinstance(target.value, ast.Name)
            and target.value.id == "state"
            and isinstance(target.slice, ast.Constant)
            and target.slice.value in ("planner_config", "approach_clearance_m")
        ):
            assignments.append(node)
    assert len(assignments) == 3, "build once and record the effective planner settings"

    geometry = SyntheticMissionGeometry(approach_gap_m=gap)
    scenario = TransportScenario(
        0,
        Pose2D(-2.34, 0, 0),
        PalletSite(0, 0, 0),
        PalletSite(3.4, 0, 0),
        (),
        Bounds(-3, 4.7, -1.75, 3.05),
    )
    built = []

    def build(**kwargs):
        # Nondefault fields expose a log reconstructed from only known settings.
        config = pallet_mission.make_transport_planner_config(
            **kwargs, xy_resolution_m=0.13, reverse_penalty=1.6
        )
        built.append(config)
        return config

    state = {}
    namespace = dict(vars(pallet_mission))
    namespace.update(
        settings={"planner_curvature_inv_m": 0.45, "planning_clearance_m": clearance},
        geometry=geometry,
        state=state,
        asdict=asdict,
        make_transport_planner_config=build,
    )
    exec(
        compile(ast.Module(body=assignments, type_ignores=[]), str(SCRIPT), "exec"),
        namespace,
    )
    assert len(built) == 1
    config = built[0]
    record = json.loads(json.dumps(state))
    assert record["planner_config"] == asdict(
        PlannerConfig(
            primitive_length_m=0.25,
            clearance_m=clearance,
            max_expansions=30000,
            curvature_limit_inv_m=0.45,
            xy_resolution_m=0.13,
            reverse_penalty=1.6,
        )
    )
    assert record["approach_clearance_m"] == approach_clearance

    received = []
    real_plan = pallet_mission.plan_hybrid_astar

    def capture(*args):
        received.append(args[-1])
        return real_plan(*args)

    monkeypatch.setattr(pallet_mission, "plan_hybrid_astar", capture)
    namespace.update(
        scenario=scenario,
        waypoint=scenario.start_rear,
        target_pickup=scenario.pickup,
        start_rear_pose=scenario.start_rear,
        rear=np.array([-2.34, 0, 0]),
        return_to_pose=None,
        pickup_bounds=None,
        travel_config=None,
    )
    calls = [
        node
        for node in ast.walk(run)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id in ("plan_observation_leg", "plan_transport")
    ]
    assert sorted(node.func.id for node in calls) == [
        "plan_observation_leg",
        "plan_observation_leg",
        "plan_transport",
        "plan_transport",
    ]
    for call in calls:
        received.clear()
        result = eval(compile(ast.Expression(call), str(SCRIPT), "eval"), namespace)
        assert result.success, result.status
        assert received[-1] is config
        assert asdict(received[-1]) == record["planner_config"]
        if call.func.id == "plan_transport":
            assert received[0].clearance_m == record["approach_clearance_m"]
            assert received[0].primitive_length_m == 0.25
            assert result.return_home is None

    # The same two call sites plan the return leg under the same config when a
    # goal is supplied, so the flag cannot reach one runner branch only.
    namespace.update(return_to_pose=scenario.start_rear)
    for call in calls:
        if call.func.id != "plan_transport":
            continue
        received.clear()
        result = eval(compile(ast.Expression(call), str(SCRIPT), "eval"), namespace)
        assert result.success, result.status
        assert result.return_home is not None
        assert received[-1] is config

    # --layout factory: the same call sites confine the pickup side to the bay
    # and hand the travel config to the transport and return legs only.
    travel = replace(config, obstacle_heuristic_resolution_m=0.25)
    namespace.update(pickup_bounds=scenario.bounds, travel_config=travel)
    for call in calls:
        received.clear()
        result = eval(compile(ast.Expression(call), str(SCRIPT), "eval"), namespace)
        assert result.success, result.status
        if call.func.id == "plan_observation_leg":
            assert received == [config]
        else:
            assert received[0].obstacle_heuristic_resolution_m is None
            assert received[0].clearance_m == record["approach_clearance_m"]
            assert len(received) == 3
            assert all(item is travel for item in received[1:])


def test_trackers_rebuilt_after_detection_use_the_same_tolerance_rules():
    # Both places that build a tracker per planned stage must agree. Observing
    # and returning home are repositioning moves (0.03 m, 0.03 rad); docking
    # stages keep 8 mm and 0.02 rad. A car-like truck that stops 12 mm off a
    # repositioning goal cannot correct sideways, so the tighter tolerance only
    # deadlocks it (docs/validation/2026-09-23-return-to-start-runs.md,
    # 2026-09-26-factory-hall-and-isaac-slam.md).
    tree = ast.parse(SCRIPT.read_text())
    rules = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.DictComp):
            continue
        source = ast.unparse(node.generators[0].iter)
        if source != "paths.items()":
            continue
        for call in ast.walk(node.value):
            if (
                isinstance(call, ast.Call)
                and getattr(call.func, "id", "") == "TrackerConfig"
            ):
                keywords = {k.arg: k.value for k in call.keywords}
                rules.append(
                    (keywords["yaw_tolerance_rad"], keywords["position_tolerance_m"])
                )
    assert len(rules) == 2, "one comprehension per runner branch"
    for yaw_rule, position_rule in rules:
        yaw = compile(ast.Expression(yaw_rule), str(SCRIPT), "eval")
        position = compile(ast.Expression(position_rule), str(SCRIPT), "eval")
        for name in ("observe", "return_home"):
            assert eval(yaw, {"name": name}) == 0.03
            assert eval(position, {"name": name}) == 0.03
        for name in ("approach", "insert", "extract", "transport", "withdraw"):
            assert eval(yaw, {"name": name}) == 0.02
            assert eval(position, {"name": name}) == 0.008


def test_the_re_observation_tracker_uses_the_repositioning_tolerances():
    tree = ast.parse(SCRIPT.read_text())
    literal = [
        {k.arg: k.value for k in call.keywords}
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and getattr(call.func, "id", "") == "TrackerConfig"
        and isinstance(
            {k.arg: k.value for k in call.keywords}["yaw_tolerance_rad"],
            ast.Constant,
        )
    ]
    assert literal, "the observe re-plan builds its own tracker"
    for keywords in literal:
        assert keywords["yaw_tolerance_rad"].value == 0.03
        assert ast.literal_eval(keywords["position_tolerance_m"]) == 0.03


def test_every_runner_tracker_releases_gear_change_cusps_the_same_way():
    tree = ast.parse(SCRIPT.read_text())
    configs = [
        {k.arg: k.value for k in call.keywords}
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and getattr(call.func, "id", "") == "TrackerConfig"
    ]
    assert len(configs) == 3
    for keywords in configs:
        assert ast.literal_eval(keywords["cusp_position_tolerance_m"]) == 0.03
        assert ast.literal_eval(keywords["cusp_yaw_tolerance_rad"]) == 0.05


def test_every_runner_tracker_accepts_small_overshoot_except_insertion():
    tree = ast.parse(SCRIPT.read_text())
    for call in ast.walk(tree):
        if not (
            isinstance(call, ast.Call)
            and getattr(call.func, "id", "") == "TrackerConfig"
        ):
            continue
        rule = {k.arg: k.value for k in call.keywords}["overshoot_tolerance_m"]
        code = compile(ast.Expression(rule), str(SCRIPT), "eval")
        assert eval(code, {"name": "insert"}) in (None, 0.03)
        for name in ("observe", "approach", "transport", "withdraw", "return_home"):
            assert eval(code, {"name": name}) == 0.03
        if not isinstance(rule, ast.Constant):
            assert eval(code, {"name": "insert"}) is None


def test_every_runner_tracker_reads_the_same_optional_speed_caps():
    tree = ast.parse(SCRIPT.read_text())
    sources = set()
    for call in ast.walk(tree):
        if (
            isinstance(call, ast.Call)
            and getattr(call.func, "id", "") == "TrackerConfig"
        ):
            keywords = {k.arg: ast.unparse(k.value) for k in call.keywords}
            sources.add(
                (
                    keywords["max_lateral_acceleration_mps2"],
                    keywords["max_reverse_speed_mps"],
                )
            )
    assert sources == {
        (
            "settings.get('max_lateral_acceleration_mps2')",
            "settings.get('max_reverse_speed_mps')",
        )
    }
