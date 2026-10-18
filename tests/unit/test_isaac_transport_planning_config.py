"""Execute the runner's planning boundary statements on CPU, without its SDK.

Only configuration/record assignments and the four planner calls are extracted;
this checks argument wiring and real core planning, not simulator execution.
"""

import ast
import json
from dataclasses import asdict, replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.planning import Bounds, PlannerConfig, Pose2D, pallet_mission
from forklift_core.planning.pallet_mission import (
    PalletSite,
    SyntheticMissionGeometry,
    TransportScenario,
)

SCRIPT = Path(__file__).resolve().parents[2] / "sim/isaac/run_transport.py"
CORE = Path(__file__).resolve().parents[2] / "src/forklift_core/mission/core.py"  # moved mission logic (M-a1)



def runner_run(tree):
    """run() as one function again: its setup and then drive()'s statements (the M-a1 replay plan R2 split
    them at the initial plans; drive's prologue that binds the setup's names is left out)."""
    import copy

    run = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "run")
    drive = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "drive")
    body = list(drive.body)
    while body and (
        (isinstance(body[0], ast.Expr) and isinstance(body[0].value, ast.Constant))
        or ast.unparse(body[0]).startswith("_bound = ")
        or (isinstance(body[0], ast.If) and ast.unparse(body[0].test).endswith("in _bound"))
    ):
        body.pop(0)
    merged = copy.copy(run)
    merged.body = run.body[:-2] + body  # without the hand-over (import, return drive(...))
    return merged

def _pass_through(call) -> bool:
    """``f(*a, **k)``: the runner's hook for the core, every argument passed straight on."""
    return (bool(call.args) and all(isinstance(a, ast.Starred) and isinstance(a.value, ast.Name) for a in call.args)
            and all(k.arg is None and isinstance(k.value, ast.Name) for k in call.keywords))


@pytest.mark.parametrize("d7", [False, True])
@pytest.mark.parametrize(
    "gap,clearance,approach_clearance",
    [(0.10, 0.12, 0.05), (0.04, 0.12, 0.02), (0.10, 0.01, 0.01)],
)
def test_all_runner_planning_calls_use_the_recorded_config(
    monkeypatch, gap, clearance, approach_clearance, d7
):
    tree = ast.parse(SCRIPT.read_text())
    run = runner_run(tree)
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
        # --d7-planner: the 'D7 on' arm applies the judged options (plan D7).
        args=SimpleNamespace(d7_planner=d7),
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
            **(pallet_mission.D7_PLANNER_OPTIONS if d7 else {}),
        )
    )
    assert (record["planner_config"]["goal_connection"], record["planner_config"]["shot_cap_m"]) == (
        ("reeds_shepp", 4.0) if d7 else ("dubins", None)
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
        # The perception branch plans to planning_pickup (the estimate, or the
        # nominal pickup under --planning-target oracle_nominal).
        planning_pickup=scenario.pickup,
        start_rear_pose=scenario.start_rear,
        rear=np.array([-2.34, 0, 0]),
        return_to_pose=None,
        pickup_bounds=None,
        # The observation travel's bounds (persistent console plan D2): the pickup bounds in a batch run.
        observe_bounds_=None,
        self=SimpleNamespace(observe_bounds=lambda: None),
        travel_config=None,
        # The initial observation loop's ladder pass (earlier, then extended).
        extended=True,
        # The SLAM observe replan plans to the leg's own target.
        target=scenario.start_rear,
        PlanningPose=Pose2D,
        # Priority-5 grid planning off: the plan sees what it always saw.
        grid_world=lambda sc: sc,
        grid_kwargs=lambda *a, **k: {},
        start=scenario.start_rear,
        coordinates=(scenario.start_rear.x_m, scenario.start_rear.y_m, scenario.start_rear.yaw_rad),
        # The priority-5 mission replan after recognition plans to the same pickup.
        pocket={"planning_pickup": scenario.pickup},
    )
    # The obstacle replan's calls moved to the mission core (M-a1 slice 3c), its names bound to the same ports.
    calls = [
        node
        for node in [*ast.walk(run), *ast.walk(ast.parse(CORE.read_text()))]
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id in ("plan_observation_leg", "plan_transport")
        and not _pass_through(node)  # the runner's hook for the core passes straight through
    ]
    # Third observation leg call: the SLAM observe replan (online SLAM plan
    # v3.8, same recorded planner_config, earlier ladder); fourth and fifth:
    # the priority-5 obstacle replan of the same leg, and its zero-clearance
    # retry from a start inside the clearance; sixth: the other observation
    # candidates tried when that leg stays blocked. The last two plan_transport
    # calls: the mission replanned from where the truck stands after recognition
    # when that leg stays blocked, and its zero-clearance retry.
    assert sorted(node.func.id for node in calls) == [
        "plan_observation_leg",
        "plan_observation_leg",
        "plan_observation_leg",
        "plan_observation_leg",
        "plan_observation_leg",
        "plan_observation_leg",
        "plan_transport",
        "plan_transport",
        "plan_transport",
        "plan_transport",
    ]
    assert "tight = replace(planner_config, clearance_m=0.0)" in CORE.read_text()
    for call in calls:
        if any(ast.unparse(a) == "tight" for a in call.args[1:3]):
            continue  # the documented zero-clearance retry, built from planner_config
        received.clear()
        namespace["planning_trace"] = []
        result = eval(compile(ast.Expression(call), str(SCRIPT), "eval"), namespace)
        assert result.success, result.status
        assert received[-1] is config
        assert asdict(received[-1]) == record["planner_config"]
        if call.func.id == "plan_transport":
            assert received[0].clearance_m == record["approach_clearance_m"]
            assert received[0].primitive_length_m == 0.25
            assert result.return_home is None
            # Both branches hand the runner a trace that outlives a failed stage.
            stages = [entry["stage"] for entry in namespace["planning_trace"]]
            assert stages[:2] == ["approach_search", "approach_straight"]

    # The same two call sites plan the return leg under the same config when a
    # goal is supplied, so the flag cannot reach one runner branch only.
    namespace.update(return_to_pose=scenario.start_rear)
    for call in calls:
        if call.func.id != "plan_transport" or any(ast.unparse(a) == "tight" for a in call.args[1:3]):
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
        if any(ast.unparse(a) == "tight" for a in call.args[1:3]):
            continue  # the zero-clearance retry, as above
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
    rules = []  # (the mission plan after a detection moved to the mission core, M-a1 slice 4b-iv)
    for node in [*ast.walk(ast.parse(SCRIPT.read_text())), *ast.walk(ast.parse(CORE.read_text()))]:
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
        # An observation stop is judged like a cusp in heading: the next leg
        # and the capture start from the measured pose
        # (docs/plans/2026-10-02-second-eval-failure-fixes.md, P3).
        assert eval(yaw, {"name": "observe"}) == 0.05
        assert eval(yaw, {"name": "return_home"}) == 0.03
        assert eval(position, {"name": "observe"}) == 0.03
        # A 3 cm return stopped short before its heading settled (seed 23).
        assert eval(position, {"name": "return_home"}) == 0.008
        for name in ("approach", "insert", "extract", "transport", "withdraw"):
            assert eval(yaw, {"name": name}) == 0.02
            assert eval(position, {"name": name}) == 0.008


def test_the_re_observation_tracker_uses_the_repositioning_tolerances():
    # (the re-observation's candidate tracker moved to the mission core, M-a1 slice 4b-iii)
    literal = [
        {k.arg: k.value for k in call.keywords}
        for call in [*ast.walk(ast.parse(SCRIPT.read_text())), *ast.walk(ast.parse(CORE.read_text()))]
        if isinstance(call, ast.Call)
        and getattr(call.func, "id", "") == "TrackerConfig"
        and isinstance(
            {k.arg: k.value for k in call.keywords}["yaw_tolerance_rad"],
            ast.Constant,
        )
    ]
    assert literal, "the observe re-plan builds its own tracker"
    for keywords in literal:
        assert keywords["yaw_tolerance_rad"].value == 0.05
        assert ast.literal_eval(keywords["position_tolerance_m"]) == 0.03


def test_every_runner_tracker_releases_gear_change_cusps_the_same_way():
    # (the re-observation's candidate tracker moved to the mission core, M-a1 slice 4b-iii)
    configs = [
        {k.arg: k.value for k in call.keywords}
        for call in [*ast.walk(ast.parse(SCRIPT.read_text())), *ast.walk(ast.parse(CORE.read_text()))]
        if isinstance(call, ast.Call)
        and getattr(call.func, "id", "") == "TrackerConfig"
    ]
    assert len(configs) == 3
    for keywords in configs:
        assert ast.literal_eval(keywords["cusp_position_tolerance_m"]) == 0.03
        assert ast.literal_eval(keywords["cusp_yaw_tolerance_rad"]) == 0.05
        # Brake and judge cusps at 8 mm (2026-10-02 planner/tracker plan, P2).
        assert ast.literal_eval(keywords["cusp_brake_window_m"]) == 0.008


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


def test_path_record_keeps_the_interval_through_the_straight_tail():
    tree = ast.parse(SCRIPT.read_text())
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "path_record"
    )
    namespace = {}
    exec(
        compile(ast.Module(body=[function], type_ignores=[]), str(SCRIPT), "exec"),
        namespace,
    )
    scenario = TransportScenario(
        0,
        Pose2D(-2.34, 0, 0),
        PalletSite(0, 0, 0),
        PalletSite(3.4, 0, 0),
        (),
        Bounds(-3, 4.7, -1.75, 3.05),
    )
    plans = pallet_mission.plan_transport(scenario)
    assert plans.success, plans.status
    for stage in ("approach", "transport"):
        record = json.loads(json.dumps(namespace["path_record"](getattr(plans, stage))))
        assert record["analytic_expansion_interval"] == 8


def test_every_observation_candidate_record_keeps_its_search_attempts():
    """Both branches record every search behind a candidate, with its start."""
    records = []  # the runner's branches and the obstacle replan's fallback, now in the mission core (M-a1 slice 3c)
    for call in [*ast.walk(ast.parse(SCRIPT.read_text())), *ast.walk(ast.parse(CORE.read_text()))]:
        if (
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Attribute)
            and call.func.attr == "append"
            and "observation_candidates" in ast.unparse(call.func.value)
        ):
            (record,) = call.args
            records.append({key.value for key in record.keys})
    # Two observation branches plus the priority-5 obstacle-block fallback.
    assert len(records) == 3
    for keys in records:
        assert {"search_attempts", "start_rear", "status"} <= keys


def test_the_20260921_profile_restores_the_old_observation_heading():
    """The 9/21 baseline judged observation at 0.03 rad (git show 6f9fb82)."""
    from types import SimpleNamespace

    from forklift_core.control.path_tracking import TrackerConfig

    from forklift_core.mission.core import apply_tracker_profile_to  # a core function (M-a1 slice 6e)

    source = SCRIPT.read_text()
    first = source.index("apply_tracker_profile_to(trackers, args.tracker_profile)")
    assert first < source.index("record_tracker_configs_in(state, trackers)") < source.index("    core = MissionCore(")

    def build(profile):
        trackers = {
            "observe": SimpleNamespace(
                config=TrackerConfig(position_tolerance_m=0.03, yaw_tolerance_rad=0.05)
            ),
            "approach": SimpleNamespace(
                config=TrackerConfig(position_tolerance_m=0.008, yaw_tolerance_rad=0.02)
            ),
        }
        apply_tracker_profile_to(trackers, profile)
        return trackers

    old = build("20260921")
    assert old["observe"].config.yaw_tolerance_rad == 0.03
    assert old["observe"].config.position_tolerance_m == 0.008
    assert old["approach"].config.yaw_tolerance_rad == 0.02
    current = build("current")
    assert current["observe"].config.yaw_tolerance_rad == 0.05


def test_runtime_viewpoints_extend_the_default_list_before_any_candidate_is_planned():
    """Appended once, after the fixed list, and both loops read the same list.

    Fixed candidates keep their indices, so a run that ends inside the fixed
    list takes the same branches (docs/plans/2026-10-03-runtime-observation-viewpoints.md).
    """
    source = SCRIPT.read_text()
    extend = source.index("args.observation_waypoints = list(args.observation_waypoints) + [")
    first_loop = source.index("for candidate_index, coordinates, extended in initial_candidates:")
    # The re-observation moved to the mission core (M-a1 slice 4b-iii); it reads the same list, which the
    # runner hands it after the extension (the core is built later in run()).
    core = CORE.read_text()
    reobserve = core.index("while ms.next_candidate_index < len(")
    assert source.count("args.observation_waypoints = list(args.observation_waypoints) + [") == 1
    assert extend < first_loop < source.index("    core = MissionCore(")
    assert "observation_waypoints=args.observation_waypoints" in source
    assert "if args.runtime_viewpoints:" in source[source.rindex("if args.use_perception:", 0, extend) : extend]
    assert "options.observation_waypoints" in core[reobserve : reobserve + 120]


def test_runtime_viewpoints_only_follow_the_default_bay_list():
    tree = ast.parse(SCRIPT.read_text())
    (assign,) = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and ast.unparse(node.targets[0]) == "args.runtime_viewpoints"
    ]
    rule = ast.unparse(assign.value)
    assert "args.observation_waypoints is None" in rule
    assert "args.layout != 'factory'" in rule


def test_runtime_viewpoints_see_the_pallet_only_as_an_unlabelled_rectangle():
    """The call gets one occupied list; nothing passes the pickup pose separately."""
    tree = ast.parse(SCRIPT.read_text())
    (call,) = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and ast.unparse(node.func) == "runtime_viewpoints"
    ]
    keywords = {k.arg for k in call.keywords}
    assert [ast.unparse(a) for a in call.args] == [
        "occupied",
        "geometry.unloaded_footprint",
        "scenario.bounds",
    ]
    assert not any("pickup" in ast.unparse(k.value) for k in call.keywords)
    assert {"margin_m", "rear_to_camera_m", "half_fov_rad"} <= keywords


def test_runtime_viewpoints_need_both_the_default_list_and_the_bay():
    tree = ast.parse(SCRIPT.read_text())
    (assign,) = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and ast.unparse(node.targets[0]) == "args.runtime_viewpoints"
    ]
    assert isinstance(assign.value, ast.BoolOp) and isinstance(assign.value.op, ast.And)


def _cusp_replan_branch():
    tree = ast.parse(CORE.read_text())  # the cusp recoveries moved to the mission core (M-a1 slice 4a-iii)
    for node in ast.walk(tree):
        if isinstance(node, ast.If) and "max_cusp_replans" in ast.unparse(node.test):
            return node
    raise AssertionError("no cusp replan branch")


def test_only_a_transport_heading_failure_at_a_cusp_is_replanned():
    """docs/plans/2026-10-03-transport-stage-fixes.md, T1."""
    test = _cusp_replan_branch().test
    assert isinstance(test, ast.BoolOp) and isinstance(test.op, ast.And)
    parts = {ast.unparse(v) for v in test.values}
    assert parts == {
        "tracking.status == 'failed'",
        "ms.phase == 'transport'",
        "tracking.failure == 'endpoint_heading'",
        "tracking.at_cusp",
        "not tracking.off_path",
        # Plan D7c: after a docked straight or inside a re-approach the cusp is a
        # docking retry, outside the cusp budget (only with --d7-docking).
        "len(state['cusp_replans']) < max_cusp_replans or d7_owned",
    }


def test_the_replan_waits_for_a_stop_and_keeps_the_tracker_rules():
    branch = ast.unparse(_cusp_replan_branch())
    # Moving: keep braking, no plan yet. Stopped means zero command, planar
    # speed and yaw rate, held for 0.1 s -- not the forward speed alone.
    assert "tracking.speed_mps == 0.0" in branch
    assert "np.linalg.norm(hooks.truth_velocity()[:2])" in branch  # the runner's velocity, as a truth hook
    assert "hooks.truth_yaw_rate()" in branch  # robot.get_angular_velocity()[2]
    assert "cusp_stop_ticks < cusp_stop_needed" in branch
    assert "replace(tracking, status='braking')" in branch
    # The replan is the active plan everywhere afterwards.
    assert "state.setdefault('paths', {})[ms.phase] = path_record(replanned)" in branch
    assert "hooks.show_path(replanned, 'Transport'" in branch
    assert "state['planning_wall_s'] = " in branch
    # Stopped: the transport leg alone, same planner config, from the measured rear pose.
    # Under SLAM after docking the corrected drop (online SLAM plan v3.8).
    assert (
        "plan_transport_leg(grid_world(slam.get('transport_scenario', scenario) if slam is not None else scenario), PlanningPose("
        in branch
    )
    assert "planner_config" in branch and "travel_config=travel_config" in branch
    # Same tracker configuration as the leg it replaces.
    assert "trackers[ms.phase].config" in branch.split("RearAxlePathTracker(")[1]
    source = SCRIPT.read_text()
    assert "max_cusp_replans = 2" in source
    assert "cusp_stop_ticks, cusp_stop_needed = 0, 12" in source
    # A failure the branch does not clear still aborts as before.
    after = source.index("recover_ = core.cusp_and_observe_recovery(")
    assert source.index("command_ = core.command(", after) > after  # whose first rule is the failure dump (4a-iv)
    command = CORE.read_text()[CORE.read_text().index("    def command("):]
    assert 'if tracking.status == "failed":\n            dump_tracking("failed", tracking)' in command


def test_the_extended_ladder_is_a_second_pass_for_the_first_observation_only():
    """Initial candidates: earlier ladder for all, then extended for all;
    re-observation stays on the earlier ladder (transport-stage plan, T2)."""
    source = SCRIPT.read_text()
    assert "for extended in (False, True)\n            for index, coordinates in enumerate(args.observation_waypoints)" in source
    calls = [  # the obstacle replan's calls moved to the mission core (M-a1 slice 3c)
        node
        for node in [*ast.walk(ast.parse(source)), *ast.walk(ast.parse(CORE.read_text()))]
        if isinstance(node, ast.Call) and ast.unparse(node.func) == "plan_observation_leg"
        and not _pass_through(node)  # the runner's hook for the core passes straight through
    ]
    extended = {
        next(ast.unparse(k.value) for k in c.keywords if k.arg == "extended") for c in calls
    }
    assert extended == {"extended", "False"}


# --- perception mount selection (docs/plans/2026-10-03-carriage-mount-adoption.md) ---


def _mount_branch():
    tree = ast.parse(SCRIPT.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.If) and ast.unparse(node.test) == "args.perception_mount == 'legacy'":
            if any("mount_parent" in ast.unparse(t) for t in node.body):
                return node
    raise AssertionError("no mount branch")


def test_the_legacy_mount_keeps_the_recorded_camera_exactly():
    branch = _mount_branch()
    legacy = "\n".join(ast.unparse(n) for n in branch.body)
    assert "perception_mount = adapter.default_base_from_optical()" in legacy
    assert "mount_parent = '/World/Forklift/base_link'" in legacy
    assert "mount_xyzw = rig.OPTICAL_QUATERNION_XYZW" in legacy
    low = "\n".join(ast.unparse(n) for n in branch.orelse)
    assert "adapter.mount_base_from_optical(args.perception_mount)" in low
    assert "mount_parent = '/World/Forklift/fork_carriage'" in low
    assert "adapter.quaternion_xyzw(perception_mount.rotation)" in low


def test_both_cameras_use_the_same_parent_and_orientation():
    source = SCRIPT.read_text()
    assert 'prim_path=mount_parent + "/PerceptionCamera"' in source
    assert 'prim_path=mount_parent + "/PerceptionDisplayCamera"' in source
    # The priority-5 D5 pocket depth camera shares the mount too.
    assert 'prim_path=mount_parent + "/PocketDepthCamera"' in source
    # the perception, pocket and display cameras, and the live video camera (2026-10-18, the console's view)
    assert source.count("orientation=np.asarray(adapter.xyzw_to_wxyz(mount_xyzw))") == 4
    assert "OPTICAL_QUATERNION_XYZW))" not in source


def test_the_carriage_mount_is_guarded_and_checked():
    source = SCRIPT.read_text()
    assert "guard_fn=lift_guard," in source
    assert 'raise adapter.CaptureFailure("lift_not_zero")' in source
    assert "abs(float(cells.robot.get_joint_positions()[lift_joint])) > 0.001" in source
    assert '"Perception camera local pose differs from the planned mount"' in source
    for rule in (
        '"--perception-mount carriage_low needs --use-perception"',
        '"--perception-mount carriage_low needs ros camera axes"',
        '"--perception-mount carriage_low is defined for dls08_provisional only"',
    ):
        assert rule in source


def test_the_detector_sees_rounded_depth_only_when_asked_and_attempts_record_the_mount():
    source = SCRIPT.read_text()
    # Both detection paths go through the tested helper with the run's setting; the observe arrival's
    # moved to the mission core (M-a1 slice 4b-i), the repeat-capture diagnostic stays in the runner.
    core = CORE.read_text()
    assert source.count("adapter.detector_input(") + core.count("adapter.detector_input(") == 2
    assert "adapter.detector_input(\n            scene_input, options.depth_quantize_mm\n" in core
    assert "repeat_input, args.depth_quantize_mm" in source
    assert "detection = detect_pockets(detector_input, prior, params)" in core
    assert 'attempt["base_from_optical"] = {' in core
    assert 'attempt["depth_quantize_mm"] = options.depth_quantize_mm' in core
    tree = ast.parse(source)
    defaults = {
        ast.unparse(call.args[0]): next(
            (ast.unparse(k.value) for k in call.keywords if k.arg == "default"), None
        )
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and ast.unparse(call.func) == "parser.add_argument"
        and call.args
    }
    assert defaults["'--perception-mount'"] == "'legacy'"
    assert defaults["'--depth-quantize-mm'"] == "0"


def _runner_function(name):
    """One top-level function of the Isaac runner, without importing Isaac."""
    tree = ast.parse(SCRIPT.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
    scope = {"np": np}
    exec(compile(ast.Module([node], []), str(SCRIPT), "exec"), scope)
    return scope[name]


class _Path:
    def __init__(self, poses):
        self.poses = np.asarray(poses, dtype=float)


def _straight(x0, x1, y=0.0, yaw=0.0, step=0.025):
    xs = np.arange(x0, x1 + 1e-9, step)
    return _Path([[x, y, yaw] for x in xs])


def test_same_path_is_the_remaining_part_again():
    same_path = _runner_function("same_path")
    current = _straight(0.0, 4.0)
    # The truck stopped at x = 1.5: the replan from there is the same path.
    assert same_path(_straight(1.5, 4.0), current, remaining_m=2.5)
    # 2.5 cm aside (a detour that already misses the blocking cells), or a
    # different length, is a different path (Codex checkpoint 7 P2).
    assert not same_path(_straight(1.5, 4.0, y=0.025), current, remaining_m=2.5)
    assert not same_path(_straight(1.5, 4.5), current, remaining_m=2.5)
    assert not same_path(_straight(1.5, 4.0, yaw=0.2), current, remaining_m=2.5)
    # The same straight sampled from another start is still the same path
    # (Codex checkpoint 8: samples compared to samples read it as different).
    coarse = _straight(0.0, 4.0, step=0.05)
    shifted = _Path(np.column_stack((np.r_[np.arange(1.525, 4.0, 0.05), 4.0], np.zeros(51), np.zeros(51))))
    assert same_path(shifted, coarse, remaining_m=2.475)


def test_wheel_target_brakes_at_the_stopping_model_deceleration():
    runner = SCRIPT.read_text()
    source = CORE.read_text()  # the command assembly moved to the mission core (M-a1 slice 5)
    # The flag is args.obstacle_act (options.obstacle_act in the core): the obstacle dict has no "act" key
    # (L3c v37 never slewed).
    assert "obstacle_act=args.obstacle_act" in runner
    i = source.index("if obstacle is not None and options.obstacle_act:\n            # A step wheel target")
    block = source[i : source.index("drive = ackermann_command(wheel_speed", i)]
    assert "permission.config.stopping.decel_mps2" in block
    assert "COMMAND_ACCEL_MPS2" in block
    # The slew acts on the final wheel speed (creep included), the permission's
    # own limit and the e-stop probe cut it as a step afterwards (Codex checkpoint 6 P1).
    pre = source[source.rindex("steering_error = ", 0, i) : i]
    assert "wheel_speed = creep(requested_speed, steering_error)" in pre  # control/speed_chain.py (M-b4)
    assert "if creeping and" not in block  # the creep is slewed (L3c v43 seed 1)
    assert block.index("slew_cap_stop(") < block.index('obstacle.get("permission_cap")')  # its arithmetic: speed_chain
    # The e-stop and the plan D8 S4 safety stop (scheduled or near-field) cut it last.
    assert block.index('obstacle.get("permission_cap")') < block.index("if estop_holding or safety_hold:")
    j = source.index('obstacle["permission_cap"] = ')  # the permission tick (M-a1 slice 3b)
    assert j < source.index('if obstacle["path_blocked_ticks"] >= 36 and allowed > 0.0:')
    for text in (runner, source):
        assert 'obstacle.get("act")' not in text and 'obstacle["act"]' not in text


def test_a_failed_blocked_replan_stands_and_retries():
    source = SCRIPT.read_text()
    core = CORE.read_text()  # the obstacle replan moved to the mission core (M-a1 slice 3c)
    i = core.index("# Standing, the grid keeps updating: a failed replan is")
    block = core[i : core.index("elif same_path(replanned", i)]
    assert "failures < 5" in block and 'obstacle.setdefault("replan_wait", {})[ms.phase] = True' in block
    # Held on later ticks too, until a replan succeeds or the path clears (Codex checkpoint 7 P2).
    # The wait moved to the mission core with the live leg (M-a1 slice 3a); the runner stands on its answer.
    assert source.index("if core.live_leg(t, rear, loaded):") < source.index("command_ = core.assemble_command(")
    j = core.index('if obstacle is not None and obstacle.get("replan_wait", {}).get(ms.phase):')
    assert "check_.blocked is None" in core[j : j + 600]  # a verified path, not a zero count
    assert 'if replanned.status in ("invalid_start", "no_path"):' in core


def test_a_failed_replan_backs_off_a_bounded_distance_then_replans():
    source = SCRIPT.read_text()
    assert "BACKOFF_M = 0.30" in source and "BACKOFF_SPEED_MPS = 0.15" in source and "BACKOFF_PER_LEG = 2" in source
    replan = CORE.read_text()  # the obstacle replan moved to the mission core (M-a1 slice 3c)
    i = replan.index("# D4 delta: stood with the outline already on the")
    block = replan[replan.rindex("if ms.phase in", 0, i) : i + 2500]
    assert 'phase in ("transport", "return_home")' in block and "backs.get(ms.phase, 0) < BACKOFF_PER_LEG" in block
    assert "d = -1 if leg_dir >= 0 else 1" in block  # against the leg's direction
    # The end of a backoff is never the leg's arrival; it forces a replan.
    moved = CORE.read_text()  # the backoff end moved to the mission core (M-a1 slice 4a-iv)
    j = moved.index("# The end of a backoff is not the leg's arrival")
    assert source.index("command_ = core.command(") < source.index(
        'if tracking.status == "arrived":\n                        if args.use_perception and ms.phase == "observe":')  # the tick body sits one level in (D6)
    assert 'obstacle["force_replan"] = ms.phase' in moved[j : j + 2500]
    assert 'obstacle.pop("force_replan", None) == ms.phase' in source
    # Codex checkpoint 8: a backoff always acts under the permission, a forced
    # replan opens the obstacle block while standing, the leg direction is kept,
    # and the stall replan ends a backoff too.
    core = CORE.read_text()  # the permission tick moved to the mission core (M-a1 slice 3b)
    k = core.index('acting = (\n            ms.phase in ("observe", "approach", "transport", "return_home")')
    # Plan v10 D5: the withdrawal acts under the permission too when known pallets are on.
    assert '(ms.phase == "withdraw" and obstacle["layer"].known_enabled' in core[k : k + 400]
    assert 'known_config.get("withdraw_mode", "certified") == "certified"' in core[k : k + 400]
    assert ') and not docking_straight' in core[k : k + 500]
    assert 'obstacle.get("backoff", {}).get("phase") == ms.phase' in core[k - 2000 : k]
    assert 'or obstacle.get("force_replan") == ms.phase' in source
    assert 'leg_dir = obstacle.get("backoff", {}).get("leg_dir")' in core
    assert 'stall_config = obstacle.pop("backoff")["config"]' in core  # the stall replan too (M-a1 slice 4a-ii)


def test_new_obstacle_rules_get_the_leg_direction_the_pallet_face_and_can_silence_the_depth_camera():
    source = SCRIPT.read_text()
    i = source.index('due_ = new_obstacles["schedule"].update(')  # the call, assigned before its trace (M-a1 R3)
    call = source[i : i + 200]
    assert "leg_direction=int(leg_direction)" in call and "pallet_face=pallet_face" in call
    head = source[source.rindex("pallet_face = None", 0, i) : i]
    assert 'phase in ("approach", "insert")' in head and "geometry.pallet_depth_m / 2" in head
    j = source.index("def read_pocket_frame")
    assert '"pocket_camera" in new_obstacles["schedule"].silenced' in source[j : j + 900]


def test_pocket_bars_are_judged_against_the_block_columns_not_the_outline():
    # Codex checkpoint 9 P1: the 0.6 x 0.8 m outline holds the pockets.
    source = SCRIPT.read_text()
    assert 'new_obstacles.setdefault("pallet_frame", []).append(rect)' in source
    i = source.index('f"Measured pallet footprint overlap in {ms.phase}"')
    head = source[source.rindex("in_outline = []", 0, i) : i]
    assert "if not any(o is r_ for r_ in in_outline)" in head
    assert 'f"Measured pallet solid overlap in {ms.phase}: {box_.name}"' in source[i : i + 3500]
    assert "centre_p = (centre_w - np.asarray(ppos, dtype=float)) @ rot_" in source  # full pose, exact SAT
    assert "PALLET_BOXES = PALLET_BOXES_OF(args.pallet_geometry_loaded)" in source  # the canonical solids


def test_a_backoff_offers_no_leg_to_the_rules_and_silence_is_traced():
    source = SCRIPT.read_text()
    i = source.index('due_ = new_obstacles["schedule"].update(')  # the call, assigned before its trace (M-a1 R3)
    assert "leg_direction = 0  # a backoff is not a leg of the phase" in source[i - 2500 : i]
    core = CORE.read_text()  # the permission tick moved to the mission core (M-a1 slice 3b)
    assert 'trace.setdefault("first_zero_s", t)' in core and '"silence_trace": obstacle.get("silence_trace")' in source
    j = core.index('obstacle["permission_cap"] = (t, float(allowed))')
    assert 'trace.setdefault("first_zero_s", t)' in core[j : j + 900]  # at the cap the wheels take (Codex checkpoint 11)


def _bar_meets_pallet(pallet_xyz, roll, bar_xy, size):
    """The runner's pocket-bar check, rebuilt from its pieces for the counterexamples."""
    import math as m

    from forklift_core.perception.pallet_geometry import load_pallet_geometry, pallet_boxes

    meets = _runner_function("box_meets_obb")
    boxes = pallet_boxes(load_pallet_geometry(Path(__file__).resolve().parents[2] / "config/pallet_geometry_epal6.yaml"))
    c, s = m.cos(roll), m.sin(roll)
    rot = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])  # roll about x: pallet -> world
    centre_p = (np.array([bar_xy[0], bar_xy[1], size[2] / 2]) - np.asarray(pallet_xyz)) @ rot
    axes_p = rot.T @ np.eye(3)
    return [b.name for b in boxes if meets(b.centre_m, np.asarray(b.size_m) / 2, centre_p, axes_p, np.asarray(size) / 2)]


def test_pocket_bars_meet_the_pallet_solids_exactly():
    # Codex checkpoint 12: a 2.9 mm gap under a 0.1 rad roll is no contact.
    assert _bar_meets_pallet((0, 0, 0.04), 0.1, (0, 0.09), (0.03, 0.03, 0.09)) == []
    # Codex checkpoint 11: a -0.05 rad roll puts a 0.12 m bar 3 mm into stringer_1.
    assert "stringer_1" in _bar_meets_pallet((0, 0, 0.021), -0.05, (0, 0.145), (0.03, 0.03, 0.12))
    # Touching the stringer's underside (0.100 m) counts.
    assert "stringer_1" in _bar_meets_pallet((0, 0, 0.0), 0.0, (0, 0.145), (0.03, 0.03, 0.100))
    # The fixed N11 bar in its pocket (on the blade, 0.18625 m since the EPAL 6 fork spacing
    # plan), level pallet: clear.
    assert _bar_meets_pallet((0, 0, 0.0), 0.0, (0.25, 0.18625), (0.03, 0.03, 0.09)) == []


def test_the_n3_straight_follows_the_leg_direction():
    source = SCRIPT.read_text()
    i = source.index("def n_sweep_test(")
    block = source[i : i + 2500]
    assert "sgn_ = -1.0 if leg_direction < 0 else 1.0" in block
    assert "x0_ + sgn_ * d_ * math.cos(h0_)" in block


def test_grid_kwargs_never_reads_slam_before_it_exists():
    # L3c l5_video5: the first plans run before the SLAM link exists; a bare
    # `slam` in grid_kwargs raised "cannot access free variable 'slam'".
    tree = ast.parse(SCRIPT.read_text())
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "grid_kwargs")
    guarded = next(n for n in ast.walk(fn) if isinstance(n, ast.Try))
    inside = {id(n) for n in ast.walk(guarded)}
    bare = [n.lineno for n in ast.walk(fn) if isinstance(n, ast.Name) and n.id == "slam" and id(n) not in inside]
    assert bare == []
    attrs = [n.lineno for n in ast.walk(fn) if isinstance(n, ast.Attribute) and n.attr == "slam_ref"]
    assert attrs == []  # the memory's attribute is .slam (l5_video7: AttributeError)


def test_slam_ref_lives_only_inside_grid_kwargs():
    # L3c l5_video8: the rename to slam_ref also hit `slam = None` and the
    # link's `slam = {...}`, so the main loop's `slam` was never bound.
    tree = ast.parse(SCRIPT.read_text())
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "grid_kwargs")
    inside = {id(n) for n in ast.walk(fn)}
    stray = [n.lineno for n in ast.walk(tree) if isinstance(n, ast.Name) and n.id == "slam_ref" and id(n) not in inside]
    assert stray == []
    run = runner_run(tree)
    bound = [
        t.id
        for n in ast.walk(run)
        if isinstance(n, ast.Assign)
        for t in n.targets
        if isinstance(t, ast.Name)
    ]
    assert bound.count("slam") >= 2  # slam = None, then the link


def test_plans_project_with_the_grids_correction_and_a_release_passes_its_own():
    # l5_exp_A/B (2026-10-06): the tracker's applied correction moves between
    # scans; planning on it shifted the grid from the permission's. A release
    # passes the released correction explicitly (Codex review P1).
    source = SCRIPT.read_text()
    tree = ast.parse(source)
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "grid_kwargs")
    body = ast.get_source_segment(source, fn)
    assert '["tracker"].applied' not in body
    assert 'obstacle["applied"]' in body
    core = CORE.read_text()  # the SLAM release moved to the mission core (M-a1 slice 6c)
    rel = next(n for n in ast.walk(ast.parse(core)) if isinstance(n, ast.FunctionDef) and n.name == "slam_release")
    rel_src = ast.get_source_segment(core, rel)
    assert rel_src.count("grid_kwargs(None, own=after, applied=released_)") == 4
    assert "grid_kwargs(None)" not in rel_src


def test_every_path_replacement_restarts_the_no_progress_baseline():
    # Plan D7d (l8_measured seed 1 + N1): a stall replan's 23 m loop was judged against the
    # old path's near-zero remainder. Obstacle, stall and cusp replans that replace the phase
    # path all clear the watch after arm_docking, so it restarts on the path being driven.
    source = SCRIPT.read_text()
    moved = CORE.read_text()  # the stall replan moved to the mission core (M-a1 slice 4a-ii)
    stall = moved.index('require(replanned.success, f"stall_replan_failed:{replanned.status}")')
    window = moved[stall : stall + 2500]
    assert window.index("arm_docking()") < window.index('obstacle.setdefault("progress", {}).clear()')
    cusp = moved.index('"replaced_path": path_record(paths[ms.phase]),')  # the cusp replan (slice 4a-iii)
    window = moved[cusp : cusp + 1500]
    assert window.index("arm_docking()") < window.index('obstacle.setdefault("progress", {}).clear()')
    # ... and a D7c re-approach installed in place of the transport path (plan D7c ④).
    install = moved.index("    def install_reapproach(self, ")  # a MissionCore method since M-a1 slice 6b
    assert 'obstacle.setdefault("progress", {}).clear()' in moved[install : moved.index("\n    def ", install + 10)]
    # ... and a plan D8 S4b correction point's re-drawn docking straights.
    near_install = moved.index("    def near_install(self, ")  # a MissionCore method since M-a1 slice 6d
    assert 'obstacle.setdefault("progress", {}).clear()' in moved[near_install : near_install + 3500]
    # ... and every observe path a plan D8 S4c-3 realignment installs.
    realign = moved.index("    def realign_install(self")  # a MissionCore method since M-a1 slice 6e
    assert 'obstacle.setdefault("progress", {}).clear()' in moved[realign : realign + 2000]
    # the obstacle replan's one moved to the mission core (M-a1 slice 3c)
    replan = CORE.read_text()
    assert 'obstacle.setdefault("progress", {}).clear()' in replan[replan.index("    def blocked_replan("):]
    assert source.count('obstacle.setdefault("progress", {}).clear()') + replan.count(
        'obstacle.setdefault("progress", {}).clear()') == 6


def test_d7c_reapproach_is_under_the_permission_and_kept_by_obstacle_replans():
    source = SCRIPT.read_text()
    core = CORE.read_text()  # the permission tick moved to the mission core (M-a1 slice 3b)
    straight = core.index("docking_straight = (ms.phase == \"approach\"")
    window = core[straight : straight + 1500]
    assert 'if slam is not None and slam["docking"].get("reapproach"):\n' in window
    assert window.index("reapproach") < window.index("acting = (")
    # Both obstacle replans of the transport go through the dispatch, and a
    # re-approach is never re-armed into a docking straight.
    # (the obstacle replan moved to the mission core, M-a1 slice 3c)
    assert source.count("replanned = transport_replan(") + core.count("replanned = transport_replan(") == 2
    assert 'and not slam["docking"].get("reapproach"):\n                arm_docking()' in core


def test_a_destination_prior_error_moves_planning_only():
    # Plan D7c δ runs: the reference scan and the evaluation keep the true destination.
    source = SCRIPT.read_text()
    # the reference scan is taught at the destination's truth (a function since the persistent console plan D2,
    # which teaches each live mission at its own destination)
    assert 'taught = site_poses(taught_site, geometry)["delivery"]' in source
    assert "teach_docking_reference(scenario.destination, truth_destination)" in source
    # the delivery check (a function since the persistent console plan D1) judges on the true destination
    assert source.count("(truth_destination.x_m, truth_destination.y_m, truth_destination.yaw_rad)") == 2
    marker = source.index('state["destination_marker"] = add_destination(')
    assert source.index("truth_destination = scenario.destination") > marker
    assert source.index('(args.output / "scenario.json").write_text(') < marker


def test_d7c_docking_needs_slam():
    source = SCRIPT.read_text()
    assert ('if args.d7_docking and args.slam_feedback is None:\n'
            '        parser.error("--d7-docking retries the SLAM docking match; it needs --slam-feedback")') in source


def test_obstacle_recovery_v4_escapes_before_the_clearance_zero_replan():
    # Plan docs/plans/2026-10-04-lidar-obstacle-map.md, "장애물 복구–허가 정합 설계 4판" (smoke 2382).
    assert "ESCAPE_PER_LEG = 2" in SCRIPT.read_text()
    source = CORE.read_text()  # the obstacle replan moved to the mission core (M-a1 slice 3c)
    block = source[source.index("escape_, escape_cands_, escape_skips_back_ = None, None, False"):]
    e1 = block[: block.index('if replanned.status in ("invalid_start", "no_path"):')]
    # E1: only an invalid start in transport/return_home, with budget, and not while docking owns the transport.
    assert 'replanned.status == "invalid_start" and ms.phase in ("transport", "return_home")' in e1
    assert 'obstacle.setdefault("escapes", {}).get(ms.phase, 0) < ESCAPE_PER_LEG' in e1
    assert 'slam["docking"].get("reapproach") or slam["docking"].get("accepted_any")' in e1
    assert "escape_, escape_cands_ = find_escape(t, rear, leg_dir_e, kappa)" in e1
    # A chosen escape replaces the plan before the clearance-0 retry, whose statuses it does not match.
    assert 'True, "escape", poses_e' in e1
    # Installed on the backoff machinery: not a failure, its own budget, the end tolerance of E2's region.
    inst = block[block.index("if escape_ is not None:\n            # Plan v4 E2"):]
    inst = inst[: inst.index("elif not replanned.success:")]
    assert "backoff=True, failed=False, escape=escape_.record()" in inst
    assert '"leg_dir": leg_dir_e' in inst and '"escape": True' in inst
    for tol in ("position_tolerance_m=esc_.END_POSITION_TOL_M", "yaw_tolerance_rad=esc_.END_YAW_TOL_RAD",
                "overshoot_tolerance_m=esc_.END_POSITION_TOL_M", "cruise_speed_mps=BACKOFF_SPEED_MPS"):
        assert tol in inst
    # The 30 s and per-leg counts leave escapes out (the failure count already does: failed False).
    assert source.count('not r.get("escape")') == 2
    # The old backoff: skipped where E2 refused its straight by the checker; on the return only past
    # the rectangles (the delivered pallet the single LiDAR cannot see).
    assert "back_ok_ = not escape_skips_back_" in block
    assert "escape_leg(ms.phase)[2], esc_.straight(rear, d_b, BACKOFF_M)" in block
    assert "and back_ok_):" in block


def test_obstacle_recovery_v4_an_escape_end_bypasses_docking_stall_and_failure():
    source = SCRIPT.read_text()
    core = CORE.read_text()  # the follower's tick moved to the mission core (M-a1 slice 4a-i)
    e3 = core.index("escape_now_ = (")
    # Before each of the three ways an end could otherwise go: the docking stop, the heading miss and
    # stall replan, and the tracking-failure require -- the runner calls the follower first.
    follow = source.index("follow_ = core.follow(t, rear, signed_speed, dt)")
    docking = source.index("stall_ = core.docking_and_stall(")  # the docking stop and heading miss (slice 4a-ii)
    assert follow < docking and core.index("        dock_stop = (") < core.index("        goal_heading_miss = (")
    assert follow < source.index("command_ = core.command(")  # the tracking-failure require (slice 4a-iv)
    window = core[e3 : e3 + 2000]
    assert '"end_s" in back_e or tracking.status == "arrived"' in window
    # An endpoint failure is the end only standing; rolling, it brakes (Codex implementation review P2-1).
    assert '(standing_e and (tracking.status == "failed" or trackers[ms.phase].remaining_to_goal_m() <= 1e-6))' in window
    assert 'elif tracking.status == "failed":' in window and 'status="braking", speed_mps=0.0' in window
    assert 'back_e["stand_ticks"] >= 120' in window
    assert 'tracking = replace(tracking, status="arrived", speed_mps=0.0)' in window


def test_obstacle_recovery_v4_releases_a_hold_at_a_backoff_end_and_in_its_recovery():
    source = SCRIPT.read_text()
    moved_fn = CORE.read_text()  # release_after_backoff moved to the mission core (M-a1 slice 6c)
    fn = moved_fn[moved_fn.index("    def release_after_backoff(") :]
    fn = fn[:2500]
    assert 'slam["tracker"].mode != "holding"' in fn and "self.release_slam(" in fn  # M-a4: every release goes through the counting method
    assert 'slam["docking"].get("accepted_any") or slam["docking"].get("reapproach")' in fn
    assert 'obstacle["layer"].refresh(' in fn  # the grid re-projected before any plan reads it
    moved = CORE.read_text()  # the backoff end moved to the mission core (M-a1 slice 4a-iv)
    end = moved.index("# The end of a backoff is not the leg's arrival")
    end_block = moved[end : end + 2500]
    assert 'release_after_backoff(t, "release_after_backoff")' in end_block
    # Standing first, then the release and the forced replan (Codex implementation review P2-1).
    assert "if standing_b and not back_.get(\"released\"):" in end_block
    # (M-a3: the second is work time -- a pause does not count, plan E4)
    assert 'if standing_b and self.commands.work_elapsed(back_.get("forced_s", -1e9), t) >= 1.0:' in end_block
    core = CORE.read_text()  # the obstacle replan moved to the mission core (M-a1 slice 3c)
    rec = core.index('release_after_backoff(t, "release_in_backoff_recovery")')
    assert rec < core.index("start = PlanningPose(float(rear[0]), float(rear[1]), float(rear[2]))", rec)
    # D4: what the installed plan's clearance was and what stopped the truck -- read before a release
    # refreshes the permission (Codex implementation review P2-3), the clearance of whichever plan
    # was adopted (P2-4: the tight observe candidates and the tight mission replan too).
    assert '"clearance_m": clearance_used_' in core and '"stop": stop_cause_' in core
    cause = core.index("stop_cause_ = {")
    assert cause < core.index("released_ = release_and_carry(t)", cause)
    assert cause < core.index('release_after_backoff(t, "release_in_backoff_recovery")')
    # ... and at a backoff's end, which releases before the forced replan reads it (review 2 P2).
    keep = moved.index('back_["stop_cause"] = {')
    assert keep < moved.index('released_b = release_after_backoff(t, "release_after_backoff")')
    assert 'obstacle["backoff"].pop("stop_cause", None)' in core
    assert "clearance_used_ = 0.0  # the candidates are planned tight" in core
    assert "clearance_used_ = mission_clearance_" in core
    # find_escape moved to the mission core (M-a1 slice 6c); its dump goes through the runner's Sink.
    assert 'save_npz(\n                f"obstacle_recovery_{dumps_ + 1}.npz"' in moved_fn
    assert "band_pose=None, label=None" in source and "band_poses_ = rear if band_pose is None else band_pose" in source
    assert 'own=list(region), band_pose=list(region), label="escape_end"' in moved_fn


def test_cusp_position_stall_v7_is_wired_into_the_runner():
    # Plan docs/plans/2026-10-04-lidar-obstacle-map.md, "cusp 위치 정체 복구 설계 7판".
    source = SCRIPT.read_text()
    assert 'cusp_monitor = CUSP_STALL.CuspStallMonitor(state["cusp_stalls"])' in source
    assert 'state["cusp_stalls"] = []' in source
    # The tracker's own result goes in, after the escape override and before every later recovery rule.
    core = CORE.read_text()  # the follower's tick moved to the mission core (M-a1 slice 4a-i)
    call = core.index("tracking, cusp_hold = cusp_tick(t, rear, raw_tracking_, tracking, retry_stand_)")
    assert core.index("raw_tracking_ = tracking  # the tracker's own result") < call
    assert core.index("escape_now_ = (") < call
    follow = source.index("follow_ = core.follow(t, rear, signed_speed, dt)")
    for later in ("stall_ = core.docking_and_stall(", "recover_ = core.cusp_and_observe_recovery("):
        assert follow < source.index(later)
    # Exclusions: D7 transport whatever --d7-docking says, a held realignment, the legacy profile.
    # The cusp rules are MissionCore methods since M-a1 slice 6a (same indentation as the closures were).
    moved = CORE.read_text()

    def method(name):
        text = moved[moved.index(f"    def {name}(") :]
        return text[: text.index("\n    def ", 10)] if "\n    def " in text[10:] else text

    fn = method("cusp_excluded")
    assert 'options.tracker_profile == "20260921"' in fn
    assert 'slam["docking"].get("accepted_any") or slam["docking"].get("reapproach")' in fn
    assert 'slam["tracker"].mode == "holding" and realign_active()' in fn
    assert "args.d7_docking" not in fn
    # Ownership on the latest snapshot: the leg-ahead path, the e-stop arc at the permission's own cap
    # in the leg's direction, every sensor alive.
    own = method("cusp_ownership")
    assert "layer_.check_candidate(leg_, current_pose=pose_, direction=dir_, loaded=loaded)" in own
    assert "layer_.check_arc_candidate(" in own and "curvature_inv_m=kappa_, direction=dir_" in own
    assert "cap_mps=max(abs(float(command_mps)), abs(float(signed_speed)))" in own
    assert "sensor_timeout_s" in own
    # The prefix's planner obstacles: the leg's rectangles (observe as plan_observation_leg has them).
    leg = method("cusp_leg_checker")
    assert "escape_leg(ms.phase)[2]" in leg and 'leg_obstacles(\n            "observe"' in leg
    assert "pickup_bounds=self.observe_bounds()" in leg  # the observation travel's bounds (plan D2)
    # The prefix's sensor checks: the plain walk within the stopping distance, the walk past unknown
    # for occupied, the evidence age, sensors alive.
    sens = method("cusp_sensors")
    assert "unknown_passes=True" in sens and 'plain_.blocked in ("unknown", "edge")' in sens
    assert "age_ <= pconf_.evidence_max_age_s" in sens and "sensor_timeout_s" in sens
    # Every FREE cell either walk relied on (Codex implementation review P1).
    assert "age_ = now_s - min(plain_.oldest_free_s, passes_.oldest_free_s)" in sens
    # A short next leg's heading miss only within the existing rules' budgets.
    rec = method("cusp_end_recovered")
    assert 'len(state["cusp_replans"]) < max_cusp_replans' in rec
    assert 'goal and slam is not None' in rec and 'len(state["stall_replans"]) < 2' in rec
    # Stopped = standing with a zero applied at the wheels.
    assert 'state.get("applied_command", (None, None))[0] == 0.0' in method("cusp_stopped")
    # The hand-off: one forced replan per key, the block entered while handed, its direction, an occupied
    # prefix counted as a blocked path; ownership back once no replan is pending.
    tick = method("cusp_tick")
    # Forced only where the obstacle replan exists: with grid planning (Codex implementation review P1).
    assert 'if result_.force_replan and obstacle is not None:\n            if grid_planning:\n                obstacle["force_replan"] = ms.phase' in tick
    # A silent sensor sends no scan: liveness is passed in for every tick.
    assert "alive=lambda: cusp_alive(now_s)," in tick
    alive = method("cusp_alive")
    assert "now_s - last_ <= timeout_ for last_ in layer_.last_scan_s.values()" in alive[:600]
    assert 'obstacle.get("force_replan") == ms.phase or bool(obstacle.get("replan_wait", {}).get(ms.phase))' in tick
    assert "or cusp_handed_ is not None" in source
    core = CORE.read_text()  # the permission tick moved to the mission core (M-a1 slice 3b)
    assert "direction = cusp_handed_.hand_direction" in core
    assert '((allowed == 0.0 and why == "occupied") or cusp_prefix_blocked_)' in core
    # An end is dumped first, then the run ends.
    assert "dump_tracking(result_.end, result_.tracking)\n            require(False, result_.end)" in tick
    assert "release_update=lambda: tracker_.update(rear_now, signed_speed, dt)" in tick
    # The observe stop rule waits while the stall handling holds an observe stall.
    assert "and not cusp_monitor.holds_observe()" in core  # the observe stall rule (M-a1 slice 4a-iii)
    # Zero command with the steering held.
    assert "cusp_hold = False  # plan cusp position stall v7 K3" in source
    assembly = CORE.read_text()  # the command assembly moved to the mission core (M-a1 slice 5)
    assert "if estop_holding or obstacle_hold or dwell_hold or cusp_hold:" in assembly
    assert "wheel_curvature = kappa if (dwell_hold or cusp_hold) else curvature" in assembly
    assert "steering_command.copy() if (estop_holding or obstacle_hold or dwell_hold or cusp_hold)" in assembly


def test_near_protection_prediction_screens_only_the_last_correction():
    # Plan docs/plans/2026-10-04-lidar-obstacle-map.md, "근접 보호 예측 선별 설계 4판".
    source = SCRIPT.read_text()
    # The rule marker, always recorded with near tracking: the judgement (xii) applies by it.
    assert '"protection_prediction_rules": near_tracking.PREDICT_RULES,' in source
    core = CORE.read_text()  # near_tick and the prediction moved to the mission core (M-a1 slice 6d)
    decide = core[core.index('        elif corr_act["decide"] and terminal is None:'):]
    decide = decide[: decide.index("        if corr_act[\"event\"] == \"closed\"")]
    # After redraw_docking accepted, at the 0.6 m point only; a refusal turns into replan_rejected.
    assert decide.index("plans_, record_, target_ = near_redraw(latest_, rear_now)") < decide.index(
        "near_protection_prediction(")
    assert "if plans_ is not None and abs(corr.point_m - near_tracking.PREDICT_POINT_M) < 1e-9:" in decide
    assert 'if pred_["disposition"] == "refused":' in decide
    assert 'record_["refused"] = "protection_predicted"' in decide and "plans_ = None" in decide
    assert decide.index('record_["refused"] = "protection_predicted"') < decide.index('terminal = "replan_rejected"')
    fn = core[core.index("    def near_protection_prediction("):]
    fn = fn[: fn.index("    def near_install(")]
    # The retry permission is the terminal stop's own predicate; the lost injection the argument.
    assert ('near_tracking.retry_allowed("replan_rejected", near.get("retries", 0), near["protect"] is not None,'
            in fn)
    assert "lost_ = options.near_drop_frames_from_m is not None" in fn
    assert "near_tracking.prediction_disposition(" in fn
    for key in ('"rear"', '"approach"', '"tracker_config"', '"held"', '"widths_m"', '"bounds_sha256"',
                '"drive_terms_sha256"', '"constants"', '"camera_xy_m"', '"carriage_limit_m"', '"fork_tip_x_m"'):
        assert key in fn


def test_the_return_turnaround_runs_on_the_first_live_return_plan_only_after_the_release():
    """docs/plans/2026-10-12-return-forward-turnaround.md: the first live return plan waits for a
    pending SLAM release, re-projects the grid before it weighs the turnaround (a standing truck can
    skip the obstacle block's re-projection), and the release replans carry no turnaround."""
    src = SCRIPT.read_text()
    core = CORE.read_text()  # the live leg moved to the mission core (M-a1 slice 3a)
    start = core.index('and not obstacle.setdefault("live_replanned", {}).get(ms.phase)')
    block = core[start:core.index('obstacle["deadline"] = None', start)]
    assert 'slam.get("pending_release") is not None' in block.split("):", 1)[0]
    # The re-projection comes before every plan of the leg: the worker's request (real-time plan D8) and the
    # synchronous plan.
    assert block.index('obstacle["layer"].refresh(') < len(block) <= core.index("**return_turnaround_kwargs()", start) - start
    release = core[core.index("    def slam_release("):]  # moved to the mission core (M-a1 slice 6c)
    release = release[: release.index("\n    def ", 10)]
    assert "return_turnaround_kwargs" not in release and "plan_return_leg(" in release
    # the live return plan and its tight retry, and the worker's request of it (real-time plan D8)
    assert core.count("**return_turnaround_kwargs()") == 3
    assert src.count("**return_turnaround_kwargs()") == 0


def test_only_the_observe_and_return_trackers_hold_a_cusp_heading_miss_for_judgement():
    # EPAL 6 fork spacing plan, fix A (observe) and A' (return): 0.08 rad / 30 mm for the cusp being
    # judged; the docking phases and the loaded transport keep their rules.
    source = SCRIPT.read_text()
    assert "CUSP_HEADING_HOLD_RAD = 0.08\n" in source and "CUSP_HEADING_HOLD_POSITION_M = 0.03\n" in source
    assert 'CUSP_HEADING_HOLD_PHASES = ("observe", "return_home")\n' in source
    assert "cusp_heading_pending_rad=CUSP_HEADING_HOLD_RAD if name in CUSP_HEADING_HOLD_PHASES else None," in source
    moved = CORE.read_text()  # the cusp heading hold moved to the mission core (M-a1 slice 4a-iii)
    block = moved[moved.index("if ms.phase in CUSP_HEADING_HOLD_PHASES and tracking.cusp_pending:"):]
    block = block[: block.index("# Under SLAM, an observe leg judged out of heading")]
    assert "bounds=(CUSP_HEADING_HOLD_POSITION_M, CUSP_HEADING_HOLD_RAD)" in block
    assert "hold_tracker.release_cusp(True)" in block and "hold_tracker.fail_pending()" in block
    assert 'decision = "realign_held"' in block and "cusp_stopped()" in block
    # sync() and the watch on every tracker tick's own result, before any rule rewrites it or
    # replaces the tracker, then the hold; the watch once more on the release tick's own update
    # (Codex fix A implementation review 2 P3, 3 P2; A' design review 1 P2, 2 P2).
    core = CORE.read_text()  # the follower's tick moved to the mission core (M-a1 slice 4a-i)
    raw = core.index("raw_tracking_ = tracking  # the tracker's own result")
    whole = core[raw:]
    first_rule = whole.index("retry_stand_ = near is not None")
    assert whole.index("cusp_holds.sync(") < whole.index("cusp_holds.tick(") < first_rule
    assert source.index("follow_ = core.follow(t, rear, signed_speed, dt)") < source.index(
        "recover_ = core.cusp_and_observe_recovery(")
    after_update = block[block.index("tracking = hold_tracker.update(rear, signed_speed, dt)") :]
    assert 'if decision == "released":' in after_update and "cusp_holds.tick(" in after_update
    # Every observe and return tracker holds: the first ones, the candidate re-route's, the ones built
    # after the far observation (Codex fix A implementation review 1 P2); the near leg and the
    # replans reuse a tracker's config.
    moved = CORE.read_text()  # the candidate re-route's tracker moved to the mission core (M-a1 slice 4b-iii)
    assert source.count("cusp_heading_pending_rad=CUSP_HEADING_HOLD_RAD") + moved.count(
        "cusp_heading_pending_rad=CUSP_HEADING_HOLD_RAD") == 3
    # the next candidate's follower is made by observe_tracker() (persistent console plan D2 shares it)
    assert 'trackers["observe"] = self.observe_tracker(observe_plan)' in moved
    candidate = moved[moved.index("    def observe_tracker(self, plan):"):]
    assert "cusp_heading_pending_rad=CUSP_HEADING_HOLD_RAD," in candidate[: candidate.index("    def observe_succeeded(")]
    assert 'near_leg.curvatures_inv_m,\n                trackers["observe"].config,' in moved  # (M-a1 slice 4b-v)
    # The 2026-09-21 profile had neither cusp tolerances nor the hold.
    profile = moved[moved.index("def apply_tracker_profile_to(") :]  # a core function (M-a1 slice 6e)
    assert "cusp_heading_pending_rad=None," in profile[: profile.index("def record_tracker_configs_in(")]


def test_the_observe_replan_budget_is_per_leg_and_each_near_leg_opens_its_own():
    """Fork spacing plan, fix (1) after the S4 result: the observe-miss replan is budgeted per observe
    leg (OBSERVE_REPLANS_PER_LEG), the near-leg installation opens a new budget, the held-realignment
    no-recovery rule still comes first, and each record names its leg and the budget left."""
    source = SCRIPT.read_text()
    assert "OBSERVE_REPLANS_PER_LEG = 2" in source
    assert 'state["observe_replan_leg"] = {"serial": 0, "start": 0}' in source
    # No run-wide count of observe replans is left.
    assert 'len(state["observe_replans"]) < 2' not in source
    moved = CORE.read_text()  # the observe-miss replan moved to the mission core (M-a1 slice 4a-iii)
    assert 'len(state["observe_replans"]) < 2' not in moved
    miss = moved.index("observe_miss = (")
    assert moved.index(
        'len(state["observe_replans"]) - state["observe_replan_leg"]["start"]', miss
    ) < moved.index("if observe_miss:", miss)
    # The held-realignment refusal is checked before the replan rule.
    assert moved.index('require(False, "near_capture_realign_held_recovery")') < miss
    record = moved.index('state["observe_replans"].append(', miss)
    assert moved.index('"leg_serial": leg_["serial"]', record) < moved.index(
        "paths[\"observe\"] = replanned", record
    )
    assert '"budget_left": OBSERVE_REPLANS_PER_LEG' in moved[record:]
    # The near leg resets the budget where it is installed, not inside realign_install (which
    # returns at once outside a realignment), and only there.
    near = moved.index("if near_leg is not None:")  # the near leg install moved to the core (M-a1 slice 4b-v)
    reset = moved.index('state["observe_replan_leg"] = {\n', near)
    assert reset < moved.index("paths[\"observe\"] = near_leg", near)
    assert source.count('state["observe_replan_leg"] = {') + moved.count('state["observe_replan_leg"] = {') == 2


def test_the_steering_probe_runs_after_the_spawn_checks_and_ends_the_run():
    """Fork spacing plan, cause survey after the S4 rerun: --steering-probe drives scripted segments
    through the runner's command path once the spawn is checked, records them and returns."""
    source = SCRIPT.read_text()
    spawn = source.index('"Truck spawn displaced"')
    call = source.index("if args.steering_probe:", spawn)
    assert call < source.index("obstacle = None  # the priority-5 obstacle layer", spawn)
    body = source[call: source.index("return\n", call)]
    assert "run_steering_probe(" in body and 'state["phase"] = "steering_probe_done"' in body
    probe = source[source.index("def run_steering_probe("): source.index("def ", source.index("def run_steering_probe(") + 10)]
    # The runner's own command path: Ackermann targets, the steering rate limit, the wheel-rate slew.
    assert "ackermann(speed, kappa, geometry)" in probe
    assert 'settings["steering_command_rate_rad_s"] * dt' in probe
    assert "COMMAND_ACCEL_MPS2 * dt" in probe


def test_the_probe_segments_cover_each_curvature_standing_forward_and_reverse():
    import importlib.util as util

    spec = util.spec_from_file_location("_rt_probe", SCRIPT)
    # Read the pure helper without importing the runner (it needs Isaac): exec its source slice.
    source = SCRIPT.read_text()
    start = source.index("PROBE_CURVATURES = ")
    end = source.index("def run_steering_probe(")
    namespace = {}
    exec(source[start:end], namespace)  # noqa: S102 -- a constant tuple and a pure function
    segments = namespace["probe_segments"]()
    assert spec is not None
    for label in ("still", "fwd", "rev"):
        assert {k for lab, _, k, _ in segments if lab == label} >= {0.29, -0.29, 0.36, -0.36}
    assert all(seconds > 0 for *_, seconds in segments)
    assert {round(v, 3) for lab, v, _, _ in segments if lab == "fwd"} == {0.25}
    assert {round(v, 3) for lab, v, _, _ in segments if lab == "rev"} == {-0.25}


def _probe_helpers():
    source = SCRIPT.read_text()
    import math

    namespace = {"np": np, "math": math}
    exec(source[source.index("PROBE_CURVATURES = "): source.index("def run_steering_probe(")], namespace)  # noqa: S102
    return namespace


def test_the_long_probe_set_holds_each_run_long_enough_and_comes_back():
    """Fork spacing plan, diagnostic D-a (fixed before results): 0.25 m/s straight 4 s and 0.6 m/s
    straight 3.5 s, forward then reverse, and the 0.6 m/s +-0.36 arcs held 2.5 s, each forward run
    followed by the same reverse run."""
    segments = _probe_helpers()["probe_segments_long"]()
    moving = [(lab, v, k, s) for lab, v, k, s in segments if v != 0]
    assert (0.25, 0.0, 4.0) in {(v, k, s) for _, v, k, s in moving}
    assert (-0.25, 0.0, 4.0) in {(v, k, s) for _, v, k, s in moving}
    assert {(v, s) for _, v, k, s in moving if k == 0 and abs(v) > 0.5} == {(0.6, 3.5), (-0.6, 3.5)}
    for kappa in (0.36, -0.36):
        assert {(v, s) for _, v, k, s in moving if k == kappa} == {(0.6, 2.5), (-0.6, 2.5)}
    # Each forward run is followed (after a stop) by the same reverse run.
    for i, (_, v, k, s) in enumerate(moving):
        if v > 0:
            assert moving[i + 1][1:] == (-v, k, s)
    assert sum(1 for lab, v, *_ in segments if v == 0 and lab == "stop") == len(moving)


def test_the_probe_window_metrics_read_curvature_residual_and_rolling_ratio():
    helpers = _probe_helpers()
    columns = ("t", "segment", "speed_cmd", "kappa_cmd", "target_FL", "target_FR", "cmd_FL", "cmd_FR",
               "q_FL", "q_FR", "qd_FL", "qd_FR", "w_cmd_FL", "w_cmd_FR", "w_cmd_RL", "w_cmd_RR",
               "w_FL", "w_FR", "w_RL", "w_RR", "rear_x", "rear_y", "rear_yaw", "base_z", "roll", "pitch")
    c = {k: i for i, k in enumerate(columns)}
    dt, kappa, speed, radius = 1 / 120, 0.3, -0.6, 0.125
    t = np.arange(120) * dt
    s = speed * t
    rows = np.zeros((len(t), len(columns)))
    rows[:, c["rear_yaw"]] = kappa * s
    rows[:, c["rear_x"]] = np.sin(kappa * s) / kappa
    rows[:, c["rear_y"]] = (1 - np.cos(kappa * s)) / kappa
    rows[:, c["w_RL"]] = rows[:, c["w_RR"]] = speed / radius / 0.99  # wheels spin 1 % fast
    rows[:, c["q_FL"]] = rows[:, c["q_FR"]] = 0.1
    m = helpers["probe_window_metrics"](rows, c, dt, lambda l, r: (0.31, 0.29), speed, radius)
    assert m["kappa_true"] == pytest.approx(kappa, rel=1e-6)
    assert m["residual"] == pytest.approx(kappa - 0.30, abs=1e-6)
    assert m["rolling_ratio"] == pytest.approx(0.99, rel=1e-3)


def test_the_plant_backend_readback_uses_the_isaac_5_1_body_getters():
    """New plant candidate V0: Isaac Sim 5.1's isaacsim.core.prims Articulation has get_body_masses /
    get_body_coms / get_body_inertias; get_masses does not exist, and the record came back as an
    AttributeError in every run of fs_plant2/fs_plant3 (fork spacing plan, D-a)."""
    source = SCRIPT.read_text()
    block = source[source.index('state["plant_backend"] = {\n'): source.index('state["plant_backend"] = {"error"')]
    for getter in ("get_body_masses()", "get_body_coms()[0]", "get_body_inertias()"):
        assert f"view_.{getter}" in block
    assert "view_.get_masses(" not in source and "view_.get_coms(" not in source


def test_the_cause_separation_settings_are_probe_only_and_recorded():
    """Fork spacing plan, cause separation E: the scene and drive settings a settings file may set for a
    --steering-probe (friction correlation distance, PGS, stabilization, the tyre's damping combine mode,
    the wheel drive damping) are refused for any other run, applied after the World exists and recorded."""
    source = SCRIPT.read_text()
    main = source[source.index("def main() -> None:"):]
    guard = main.index("DIAGNOSTIC_PROBE_SETTINGS")
    assert "args.steering_probe or not any(k in settings for k in DIAGNOSTIC_PROBE_SETTINGS)" in main[guard - 200: guard + 200]
    namespace = {}
    exec(source[source.index("DIAGNOSTIC_PROBE_SETTINGS = "): source.index("TYRE_LINKS = ")], namespace)  # noqa: S102
    assert set(namespace["DIAGNOSTIC_PROBE_SETTINGS"]) == {
        "physx_friction_correlation_distance_m", "physx_solver_pgs", "physx_enable_stabilization",
        "tyre_damping_combine_max", "wheel_drive_damping", "physx_external_forces_every_iteration"}
    world = source.index("world = World(stage_units_in_meters=1.0")
    block = source[world: source.index('state["physics_scene"] = {', world) + 400]
    for call in ("set_friction_correlation_distance(", 'set_solver_type("PGS")', "enable_stablization(True)",
                 "get_friction_offset_threshold()", "is_stablization_enabled()"):
        assert call in block
    scene = (SCRIPT.parent / "scene.py").read_text()
    assert 'float(settings.get("wheel_drive_damping", 20.0)) if wheel' in scene
    assert 'if settings.get("tyre_damping_combine_max"):' in scene and 'CreateDampingCombineModeAttr("max")' in scene
    assert '"damping_combine_mode": value("physxMaterial:dampingCombineMode")' in scene


def test_the_reverse_first_long_set_swaps_each_pair_and_keeps_the_segments():
    """Cause separation E (Codex): order and contact history apart from the direction -- the same
    segments as the long set, each reverse run before its forward run."""
    helpers = _probe_helpers()
    long_, rev = helpers["probe_segments_long"](), helpers["probe_segments_long_reverse_first"]()
    assert sorted(long_) == sorted(rev) and len(long_) == len(rev)
    moving = [(lab, v, k, s) for lab, v, k, s in rev if v != 0]
    for i, (_, v, k, s) in enumerate(moving):
        if v < 0:
            assert moving[i + 1][1:] == (-v, k, s)
    assert moving[0][1] < 0


def test_the_encoder_model_feeds_the_odometry_increments_and_is_part_of_the_contract():
    """2026-10-15 (user: map the team's encoder specs into the simulation): --encoder-model
    team_spec_* reads the rear wheels' joint angles as MT6701 x4 counts and the steering as a
    KTH7801-C 12-bit angle each tick, integrates increments (no rate trapezoid, no Gaussian), records
    them, and differs from every measurement made before it (reported)."""
    import sys

    sys.path.insert(0, str(SCRIPT.parent))
    import near_tracking

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["encoder_model"] == "reported"
    source = SCRIPT.read_text()
    args = source[source.index('"--encoder-model",'): source.index('"--encoder-model",') + 300]
    assert 'choices=("reported", "team_spec_quantized", "team_spec_typical_inl")' in args
    assert 'default="reported"' in args
    assert 'if args.encoder_model != "reported" and not args.record_slam:' in source
    step = source[source.index("            rates_true = robot.get_joint_velocities()[wheels]"):]
    step = step[: step.index("            if recorder is not None:")]
    assert 'slam_log["wheel_angles_rad"].append(wheel_angles)' in step
    assert "encoder_model.read(wheel_angles[2:4], steer_true)" in step
    team = step[step.index("if slam is not None and encoder_model is not None:"): step.index("elif slam is not None:")]
    assert "update_increments(stamp_now, increments_, steering_measured_)" in team
    assert 'slam["odom_speed"] = slam["odometry"].interval_speed' in team
    assert "noise" not in team
    assert '"encoder_model": state.get("encoder_model") or {"name": "reported"}' in source


def test_the_pocket_check_is_refused_with_a_team_spec_encoder_model():
    """Codex (encoder implementation round 2, P2): the pocket check's still-frame test, frame-lag
    bound and d5_replay record use a one-tick odometry speed, which quantised counts with hysteresis
    do not bound; until a bound and a replay rule exist the combination is refused, and the reported
    path is unchanged."""
    source = SCRIPT.read_text()
    assert 'if args.encoder_model != "reported" and args.pocket_check:' in source
    assert ('if pocket.get("await_still_frame") and record.get("new") and speed_now < 0.01 and yaw_rate_now < 0.01:'
            in source)
    assert "still_ = bool(slam.get" not in source


def _tracker_book_class():
    source = SCRIPT.read_text()
    namespace = {"Path": Path}  # the replay helpers in the slice annotate with Path (M-a1 R3)
    exec(source[source.index("class TrackerBook(dict):"): source.index("def run(app, args")], namespace)  # noqa: S102
    return namespace["TrackerBook"]


def _cusp_tracker():
    from forklift_core.control.path_tracking import RearAxlePathTracker

    x = np.r_[np.linspace(0, 1, 51), np.linspace(0.98, 0.3, 35)]
    poses = np.column_stack((x, np.zeros_like(x), np.zeros_like(x)))
    return RearAxlePathTracker(poses, np.r_[np.ones(51), -np.ones(35)], np.zeros(len(x)))


def test_the_tracker_book_keeps_every_switch_with_its_owner_and_version():
    """Codex (leg-switch review P2): a switch must survive its tracker's replacement and the end of
    the run, and carry the phase that owned the tracker and a path-version serial -- not the phase
    current when it was collected."""
    book_class = _tracker_book_class()
    record, versions, clock = [], [], {"t": None}
    book = book_class(record, versions, clock, {"transport": _cusp_tracker()})
    assert versions[0]["serial"] == 1 and versions[0]["phase"] == "transport"
    assert [c["index"] for c in versions[0]["cusps"]] == [50] and versions[0]["installed_s"] is None
    clock["t"] = 12.5
    book["transport"].skip_legs(1)          # a switch ...
    book["transport"] = _cusp_tracker()     # ... and its tracker replaced before any collect
    assert [(e["serial"], e["phase"], e["stamped"], e["time_s"]) for e in record] == [(1, "transport", "replaced", 12.5)]
    assert versions[0]["replaced_s"] == 12.5 and versions[1]["serial"] == 2
    clock["t"] = 13.0
    book.update({"lower": _cusp_tracker()})  # update() goes through the book too
    assert versions[2]["phase"] == "lower"
    book["transport"].skip_legs(1)
    book.collect()
    book.collect()                           # nothing twice
    assert [(e["serial"], e["stamped"]) for e in record] == [(1, "replaced"), (2, "tick")]
    book["lower"].skip_legs(1)
    book.close()                             # the end drains what no tick collected
    assert record[-1]["serial"] == 3 and record[-1]["phase"] == "lower" and record[-1]["stamped"] == "end"


def test_the_runner_stamps_every_leg_switch_and_the_plant_record_carries_friction():
    """Fork spacing plan (Codex, batch R/C rules): the V5 cusp arrival is read at the switch; the
    runner collects each tracker's new leg switches every tick with the time and phase, and V0's
    collider census records the friction coefficients of every bound material."""
    source = SCRIPT.read_text()
    loop = source[source.index("for step in range(int(120 * args.max_sim_seconds)):"):]
    # (real-time plan D9c: the step also renders while a capture runs over the ticks)
    harvest = loop[: loop.index('stepper["fn"]((pocket_frames and step % 12 == 0) or capturing_)')]
    assert "trackers.collect()" in harvest and 'book_clock["t"] = t' in harvest
    assert 'trackers = TrackerBook(state.setdefault("leg_switches", []), state.setdefault("tracker_versions", [])' in source
    # Drained in the finally block after the D8b recorder (which stays first) and before the records are written.
    closing = source.index('tracker_book["book"].close()')
    assert source.index('state["pocket_recording"] = recorder.close(') < closing < source.index(
        "write_slam_record(args, state, replace(scenario")
    scene = (SCRIPT.parent / "scene.py").read_text()
    assert 'value("physics:staticFriction"), value("physics:dynamicFriction"))' in scene
    assert '"static_friction": k[5], "dynamic_friction": k[6]' in scene


def test_the_odometry_wheel_radius_is_the_odometrys_own_and_part_of_the_contract():
    """Fork spacing plan (16th draft): an odometry-only wheel radius -- the SLAM odometry integrates
    with --odometry-wheel-radius-m when given; drive commands, speed limits and the plant keep the
    geometric radius; meta.json records both so replays integrate with the same value; a run
    without it matches every earlier measurement record."""
    import sys
    from types import SimpleNamespace

    sys.path.insert(0, str(SCRIPT.parent))
    import near_tracking

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["odometry_wheel_radius_m"] is None
    radius = _probe_helpers()["odometry_wheel_radius"]
    geometry = SimpleNamespace(wheel_radius_m=0.125)
    assert radius(SimpleNamespace(odometry_wheel_radius_m=None, drive_geometry=geometry)) == 0.125
    assert radius(SimpleNamespace(odometry_wheel_radius_m=0.1245, drive_geometry=geometry)) == 0.1245
    source = SCRIPT.read_text()
    odometry = source[source.index('"odometry": IncrementalWheelOdometry('):]
    odometry = odometry[: odometry.index("initial_pose=slam_start")]
    assert "odometry_wheel_radius(args)" in odometry and "args.drive_geometry.wheel_radius_m" not in odometry
    meta = source[source.index('"odometry_geometry": {'):]
    meta = meta[: meta.index("},")]
    assert '"wheel_radius_m": odometry_wheel_radius(args)' in meta
    assert '"geometric_wheel_radius_m": args.drive_geometry.wheel_radius_m' in meta
    assert 'slam["odom_speed"] = float(np.mean(rates)) * odometry_wheel_radius(args)' in source
    assert '"odometry_wheel_radius_m": odometry_wheel_radius(args)' in source  # the feedback record
    # Nothing else reads it: the drive and the plant stay on the geometric radius. Four uses above and
    # one in age_table_problem (the obstacle layer's table must match).
    assert source.count("odometry_wheel_radius(args)") - source.count("def odometry_wheel_radius(args)") == 5
    checker = source[source.index("def age_table_problem("): source.index("def probe_segments()")]
    assert checker.count("odometry_wheel_radius(args)") == 1
    assert 'parser.error("--odometry-wheel-radius-m is the SLAM odometry\'s radius; it needs --record-slam")' in source
    assert 'parser.error("--pocket-check is not defined for --odometry-wheel-radius-m yet")' in source
    assert "require(0.9 * geometric_ <= args.odometry_wheel_radius_m <= 1.1 * geometric_," in source


def test_the_obstacle_layers_age_table_must_match_the_runs_odometry():
    """Codex (odometry-radius review, P2): the obstacle layer's odometry age-error table covers only
    odometry integrated the same way -- same encoder model, same wheel radius; a table that records no
    radius (made before the argument) only for a run on the geometric radius."""
    from types import SimpleNamespace

    problem = _probe_helpers()["age_table_problem"]
    geometry = SimpleNamespace(wheel_radius_m=0.125)

    def run(model="reported", radius=None):
        return SimpleNamespace(encoder_model=model, odometry_wheel_radius_m=radius, drive_geometry=geometry)

    old = {"noise": {"wheel_rate_std_rad_s": 0.2, "steering_std_rad": 0.005}}
    team = {"noise": {"encoder_model": "team_spec_typical_inl", "gaussian": None}, "odometry_wheel_radius_m": 0.125}
    assert problem(old, run()) is None
    assert "encoder model" in problem(old, run("team_spec_typical_inl"))
    assert problem(team, run("team_spec_typical_inl")) is None
    assert "records no wheel radius" in problem(old, run(radius=0.1245))
    assert "wheel radius 0.125" in problem(team, run("team_spec_typical_inl", 0.1245))
    assert problem(dict(team, odometry_wheel_radius_m=0.1245), run("team_spec_typical_inl", 0.1245)) is None
    source = SCRIPT.read_text()
    assert "table_problem_ = age_table_problem(age_table, args)" in source


def test_the_boundary_record_closes_an_exit_tick_with_its_branches_and_the_propagating_failure(tmp_path):
    """M-a0 (Codex round 3, P2): the run's finally closes the record before main() sets failure_reason, so
    the closer takes the reason from the exception propagating through that finally."""
    import json

    from forklift_core.mission.boundary_record import BoundaryRecorder, RecordingGate, read_record

    source = SCRIPT.read_text()
    block = source[source.index("_BOUNDARY_RECORD: dict = "): source.index("def run(app, args")]
    namespace: dict = {"time": object(), "Path": Path}  # the replay helpers in the slice annotate with Path
    exec(block, namespace)  # noqa: S102
    gate = RecordingGate(BoundaryRecorder(tmp_path))
    namespace["_BOUNDARY_RECORD"].update(gate=gate, probe=lambda: ["state.transitions+1"])
    gate.recorder.tick_begin(7, tick_t=0.1, work_t=0.1, commands=[], stop_kind="none")
    state = {"phase": "insert"}
    namespace["close_boundary_record"](state, RuntimeError("near_field_unbounded"))
    (tick,) = list(read_record(tmp_path))
    assert tick["outputs"] == {"exit": "insert", "failure_reason": "near_field_unbounded"}
    assert tick["branches"] == ["state.transitions+1"]
    assert namespace["_BOUNDARY_RECORD"]["gate"] is None
    assert json.loads((tmp_path / "boundary_meta.json").read_text())["ticks"] == 1
    namespace["close_boundary_record"](state)  # a second close (main's finally) is a no-op
    assert "close_boundary_record(state, sys.exc_info()[1])" in source
