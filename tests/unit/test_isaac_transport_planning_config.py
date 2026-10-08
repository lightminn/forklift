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


@pytest.mark.parametrize("d7", [False, True])
@pytest.mark.parametrize(
    "gap,clearance,approach_clearance",
    [(0.10, 0.12, 0.05), (0.04, 0.12, 0.02), (0.10, 0.01, 0.01)],
)
def test_all_runner_planning_calls_use_the_recorded_config(
    monkeypatch, gap, clearance, approach_clearance, d7
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
    calls = [
        node
        for node in ast.walk(run)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id in ("plan_observation_leg", "plan_transport")
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
    source = SCRIPT.read_text()
    assert "tight = replace(planner_config, clearance_m=0.0)" in source
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
        assert keywords["yaw_tolerance_rad"].value == 0.05
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
    tree = ast.parse(SCRIPT.read_text())
    records = []
    for call in ast.walk(tree):
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

    tree = ast.parse(SCRIPT.read_text())
    function = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "apply_tracker_profile"
    )

    def build(profile):
        trackers = {
            "observe": SimpleNamespace(
                config=TrackerConfig(position_tolerance_m=0.03, yaw_tolerance_rad=0.05)
            ),
            "approach": SimpleNamespace(
                config=TrackerConfig(position_tolerance_m=0.008, yaw_tolerance_rad=0.02)
            ),
        }
        namespace = {
            "args": SimpleNamespace(tracker_profile=profile),
            "trackers": trackers,
            "replace": replace,
        }
        exec(
            compile(ast.Module(body=[function], type_ignores=[]), str(SCRIPT), "exec"),
            namespace,
        )
        namespace["apply_tracker_profile"]()
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
    reobserve = source.index("while next_candidate_index < len(")
    assert source.count("args.observation_waypoints = list(args.observation_waypoints) + [") == 1
    assert extend < first_loop < reobserve
    assert "if args.runtime_viewpoints:" in source[source.rindex("if args.use_perception:", 0, extend) : extend]
    assert "args.observation_waypoints" in source[reobserve : reobserve + 120]


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
    tree = ast.parse(SCRIPT.read_text())
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
        "phase == 'transport'",
        "tracking.failure == 'endpoint_heading'",
        "tracking.at_cusp",
        "not tracking.off_path",
        "len(state['cusp_replans']) < max_cusp_replans",
    }


def test_the_replan_waits_for_a_stop_and_keeps_the_tracker_rules():
    branch = ast.unparse(_cusp_replan_branch())
    # Moving: keep braking, no plan yet. Stopped means zero command, planar
    # speed and yaw rate, held for 0.1 s -- not the forward speed alone.
    assert "tracking.speed_mps == 0.0" in branch
    assert "np.linalg.norm(velocity[:2])" in branch
    assert "robot.get_angular_velocity()[2]" in branch
    assert "cusp_stop_ticks < cusp_stop_needed" in branch
    assert "replace(tracking, status='braking')" in branch
    # The replan is the active plan everywhere afterwards.
    assert "state.setdefault('paths', {})[phase] = path_record(replanned)" in branch
    assert "add_path_display(stage, replanned, 'Transport'" in branch
    assert "state['planning_wall_s'] = " in branch
    # Stopped: the transport leg alone, same planner config, from the measured rear pose.
    # Under SLAM after docking the corrected drop (online SLAM plan v3.8).
    assert (
        "plan_transport_leg(grid_world(slam.get('transport_scenario', scenario) if slam is not None else scenario), PlanningPose("
        in branch
    )
    assert "planner_config" in branch and "travel_config=travel_config" in branch
    # Same tracker configuration as the leg it replaces.
    assert "trackers[phase].config" in branch.split("RearAxlePathTracker(")[1]
    source = SCRIPT.read_text()
    assert "max_cusp_replans = 2" in source
    assert "cusp_stop_ticks, cusp_stop_needed = 0, 12" in source
    # A failure the branch does not clear still aborts as before.
    after = source.index("< max_cusp_replans")
    assert source.index('if tracking.status == "failed":\n                    dump_tracking("failed", tracking)', after) > after


def test_the_extended_ladder_is_a_second_pass_for_the_first_observation_only():
    """Initial candidates: earlier ladder for all, then extended for all;
    re-observation stays on the earlier ladder (transport-stage plan, T2)."""
    source = SCRIPT.read_text()
    assert "for extended in (False, True)\n            for index, coordinates in enumerate(args.observation_waypoints)" in source
    tree = ast.parse(source)
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and ast.unparse(node.func) == "plan_observation_leg"
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
    assert source.count("orientation=np.asarray(adapter.xyzw_to_wxyz(mount_xyzw))") == 3
    assert "OPTICAL_QUATERNION_XYZW))" not in source


def test_the_carriage_mount_is_guarded_and_checked():
    source = SCRIPT.read_text()
    assert "guard_fn=lift_guard," in source
    assert 'raise adapter.CaptureFailure("lift_not_zero")' in source
    assert "abs(float(robot.get_joint_positions()[lift_joint])) > 0.001" in source
    assert '"Perception camera local pose differs from the planned mount"' in source
    for rule in (
        '"--perception-mount carriage_low needs --use-perception"',
        '"--perception-mount carriage_low needs ros camera axes"',
        '"--perception-mount carriage_low is defined for dls08_provisional only"',
    ):
        assert rule in source


def test_the_detector_sees_rounded_depth_only_when_asked_and_attempts_record_the_mount():
    source = SCRIPT.read_text()
    # Both detection paths go through the tested helper with the run's setting.
    assert source.count("adapter.detector_input(") == 2
    assert "adapter.detector_input(\n                            scene_input, args.depth_quantize_mm\n" in source
    assert "repeat_input, args.depth_quantize_mm" in source
    assert "detection = detect_pockets(detector_input, prior, params)" in source
    assert 'attempt["base_from_optical"] = {' in source
    assert 'attempt["depth_quantize_mm"] = args.depth_quantize_mm' in source
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


def test_same_path_waits_do_not_count_as_retries():
    source = SCRIPT.read_text()
    assert 'and not r.get("same_path") and not r.get("failed")]' in source
    assert 'if r["phase"] == phase and not r.get("same_path") and not r.get("failed"))' in source
    i = source.index("if same_path(replanned, paths[phase]")
    wait = source[i : source.index("else:", i)]
    assert "RearAxlePathTracker" not in wait and '"progress"' not in wait


def test_wheel_target_brakes_at_the_stopping_model_deceleration():
    source = SCRIPT.read_text()
    # The flag is args.obstacle_act: the obstacle dict has no "act" key (L3c v37 never slewed).
    i = source.index("if obstacle is not None and args.obstacle_act:\n                # A step wheel target")
    block = source[i : source.index("drive = ackermann_command(wheel_speed", i)]
    assert "permission.config.stopping.decel_mps2" in block
    assert "COMMAND_ACCEL_MPS2" in block
    # The slew acts on the final wheel speed (creep included), the permission's
    # own limit and the e-stop probe cut it as a step afterwards (Codex checkpoint 6 P1).
    pre = source[source.rindex("steering_error = ", 0, i) : i]
    assert "wheel_speed = requested_speed * 0.25 if creeping" in pre
    assert "if creeping and" not in block  # the creep is slewed (L3c v43 seed 1)
    assert block.index("np.clip(target_speed - previous") < block.index('obstacle.get("permission_cap")')
    assert block.index('obstacle.get("permission_cap")') < block.index("if estop_holding:")
    j = source.index('obstacle["permission_cap"] = ')
    assert j < source.index('if obstacle["path_blocked_ticks"] >= 36 and allowed > 0.0:')
    assert 'obstacle.get("act")' not in source and 'obstacle["act"]' not in source


def test_the_retry_limit_is_checked_after_the_same_path_test():
    source = SCRIPT.read_text()
    same = source.index("if same_path(replanned, paths[phase]")
    assert source.index('f"obstacle_blocked in {phase}') > same


def test_loose_arrival_only_at_observation_waypoints():
    source = SCRIPT.read_text()
    i = source.index("# The tracker stops at a path end it missed sideways")
    head = source[source.rindex("if (", 0, i) : i]
    assert 'phase == "observe"' in head and "OBSERVE_ARRIVAL_M" in head
    assert 'slam["stop_now"]' in head  # standing, not just a zero command (Codex checkpoint 6 P2)


def test_a_failed_live_plan_stands_and_retries():
    source = SCRIPT.read_text()
    i = source.index("# Standing, the grid keeps updating")
    block = source[i : source.index("waiting to retry the live plan: stand", i)]
    assert "tries < 5" in block and 'obstacle["live_retry_after_s"] = t + 1.0' in block
    assert 'and t >= obstacle.get("live_retry_after_s", 0.0)' in source


def test_a_failed_blocked_replan_stands_and_retries():
    source = SCRIPT.read_text()
    i = source.index("# Standing, the grid keeps updating: a failed replan is")
    block = source[i : source.index("elif same_path(replanned", i)]
    assert "failures < 5" in block and 'obstacle.setdefault("replan_wait", {})[phase] = True' in block
    # Held on later ticks too, until a replan succeeds or the path clears (Codex checkpoint 7 P2).
    j = source.index('if obstacle is not None and obstacle.get("replan_wait", {}).get(phase):')
    assert j < source.index("drive = ackermann_command(requested_speed")
    assert "check_.blocked is None" in source[j : j + 600]  # a verified path, not a zero count
    assert 'if replanned.status in ("invalid_start", "no_path"):' in source


def test_a_failed_replan_backs_off_a_bounded_distance_then_replans():
    source = SCRIPT.read_text()
    assert "BACKOFF_M = 0.30" in source and "BACKOFF_SPEED_MPS = 0.15" in source and "BACKOFF_PER_LEG = 2" in source
    i = source.index("# D4 delta: stood with the outline already on the")
    block = source[source.rindex("if phase in", 0, i) : i + 2500]
    assert 'phase in ("transport", "return_home")' in block and "backs.get(phase, 0) < BACKOFF_PER_LEG" in block
    assert "d = -1 if leg_dir >= 0 else 1" in block  # against the leg's direction
    # The end of a backoff is never the leg's arrival; it forces a replan.
    j = source.index("# The end of a backoff is not the leg's arrival")
    assert j < source.index('if tracking.status == "arrived":\n                    if args.use_perception and phase == "observe":')
    assert 'obstacle["force_replan"] = phase' in source[j : j + 900]
    assert 'obstacle.pop("force_replan", None) == phase' in source
    # Codex checkpoint 8: a backoff always acts under the permission, a forced
    # replan opens the obstacle block while standing, the leg direction is kept,
    # and the stall replan ends a backoff too.
    k = source.index('acting = (\n                    phase in ("observe", "approach", "transport", "return_home")')
    # Plan v10 D5: the withdrawal acts under the permission too when known pallets are on.
    assert '(phase == "withdraw" and obstacle["layer"].known_enabled' in source[k : k + 400]
    assert 'known_config.get("withdraw_mode", "certified") == "certified"' in source[k : k + 400]
    assert ') and not docking_straight' in source[k : k + 500]
    # (the plan v10 withdraw tracking check now sits between them, hence the wider window)
    assert 'obstacle.get("backoff", {}).get("phase") == phase' in source[k - 2000 : k]
    assert 'or obstacle.get("force_replan") == phase' in source
    assert 'leg_dir = obstacle.get("backoff", {}).get("leg_dir")' in source
    assert 'stall_config = obstacle.pop("backoff")["config"]' in source


def test_new_obstacle_rules_get_the_leg_direction_the_pallet_face_and_can_silence_the_depth_camera():
    source = SCRIPT.read_text()
    i = source.index('for action in new_obstacles["schedule"].update(')
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
    i = source.index('f"Measured pallet footprint overlap in {phase}"')
    head = source[source.rindex("in_outline = []", 0, i) : i]
    assert "if not any(o is r_ for r_ in in_outline)" in head
    assert 'f"Measured pallet solid overlap in {phase}: {box_.name}"' in source[i : i + 3500]
    assert "centre_p = (centre_w - np.asarray(ppos, dtype=float)) @ rot_" in source  # full pose, exact SAT
    assert "PALLET_BOXES = PALLET_BOXES_OF(args.pallet_geometry_loaded)" in source  # the canonical solids


def test_a_backoff_offers_no_leg_to_the_rules_and_silence_is_traced():
    source = SCRIPT.read_text()
    i = source.index('for action in new_obstacles["schedule"].update(')
    assert "leg_direction = 0  # a backoff is not a leg of the phase" in source[i - 2500 : i]
    assert 'trace.setdefault("first_zero_s", t)' in source and '"silence_trace": obstacle.get("silence_trace")' in source
    j = source.index('obstacle["permission_cap"] = (t, float(allowed))')
    assert 'trace.setdefault("first_zero_s", t)' in source[j : j + 900]  # at the cap the wheels take (Codex checkpoint 11)


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
    # The fixed N11 bar in its pocket, level pallet: clear.
    assert _bar_meets_pallet((0, 0, 0.0), 0.0, (0.25, 0.145), (0.03, 0.03, 0.09)) == []


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
    run = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "run")
    bound = [
        t.id
        for n in ast.walk(run)
        if isinstance(n, ast.Assign)
        for t in n.targets
        if isinstance(t, ast.Name)
    ]
    assert bound.count("slam") >= 2  # slam = None, then the link


def test_the_pickup_retraction_covers_the_estimate_and_the_range_noise():
    # Codex re-review 4 P2: a pallet endpoint at -0.059 m noise outlived a
    # 0.05 m retraction and blocked the loaded start.
    source = SCRIPT.read_text()
    i = source.index("retract_endpoints(est_.x_m")
    block = source[i - 900 : i + 200]
    assert 'grow_ = 0.05 + obstacle["layer"].noise_cut_m' in block
    assert "est_.yaw_rad, grow_)" in block


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
    rel = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "slam_release")
    rel_src = ast.get_source_segment(source, rel)
    assert rel_src.count("grid_kwargs(None, own=after, applied=released_)") == 4
    assert "grid_kwargs(None)" not in rel_src


def test_replans_record_their_path_before_the_mission_plan_exists():
    # l6_spec_mission seed 1: an observe replan under SLAM ran before the
    # perception branch created state["paths"] -> KeyError.
    source = SCRIPT.read_text()
    assert 'state["paths"]["observe"] = path_record(replanned)' not in source
    assert 'state["paths"][phase] = path_record(replanned)' not in source
    assert 'state.setdefault("paths", {})["observe"] = path_record(replanned)' in source


def test_every_path_replacement_restarts_the_no_progress_baseline():
    # Plan D7d (l8_measured seed 1 + N1): a stall replan's 23 m loop was judged against the
    # old path's near-zero remainder. Obstacle, stall and cusp replans that replace the phase
    # path all clear the watch after arm_docking, so it restarts on the path being driven.
    source = SCRIPT.read_text()
    stall = source.index('require(replanned.success, f"stall_replan_failed:{replanned.status}")')
    window = source[stall : stall + 2500]
    assert window.index("arm_docking()") < window.index('obstacle.setdefault("progress", {}).clear()')
    cusp = source.index('"replaced_path": path_record(paths[phase]),')
    window = source[cusp : cusp + 1500]
    assert window.index("arm_docking()") < window.index('obstacle.setdefault("progress", {}).clear()')
    assert source.count('obstacle.setdefault("progress", {}).clear()') == 3
