"""M-a1 slice 4b-iv: a valid observation's estimate, records, plan and followers, moved from the runner
(behaviour, state, port requests and their order, effects up to a refusal -- the plan's M-a1 criterion)."""

from dataclasses import dataclass, replace
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.planning import PlanResult, Pose2D

REAR = np.array([1.0, 2.0, 0.0])
BASE = np.array([1.5, 2.0, 0.0])


@dataclass(frozen=True)
class Observation:
    insertion_yaw_rad: float = 0.1
    lateral_m: float = 0.0


@dataclass(frozen=True)
class Config:
    clearance_m: float = 0.3


def leg(length=1.0):
    return PlanResult(True, "success", np.array([[0.0, 0.0, 0.0], [length, 0.0, 0.0]]), np.ones(2, np.int8),
                      np.zeros(2), length, 7)


class World:
    def __init__(self, *, slam=False, near=None, near_capture=None, plan_ok=True, inject=None, repeat=(0, None),
                 known=False, extra_views=False):
        self.calls = []
        log_ = self.log
        stages = ["approach", "insert", "extract", "transport", "withdraw"]
        self.plans = SimpleNamespace(success=plan_ok, status="success" if plan_ok else "approach:no_path",
                                     **{n: leg(1.0 + i) for i, n in enumerate(stages)})
        adapter = SimpleNamespace(
            estimate_pallet_center_m=lambda obs, depth: log_("center", obs) or (2.0, obs.lateral_m),
            estimate_world_pallet_site=lambda xy, yaw_, base, yaw: log_("site", xy) or Pose2D(
                base[0] + xy[0], base[1] + xy[1], yaw + yaw_))
        near_tracking = SimpleNamespace(
            biased_observation=lambda obs, depth, lat, yaw_: replace(obs, lateral_m=lat),
            biased_site=lambda site, lat, yaw_: Pose2D(site.x_m, site.y_m + lat, site.yaw_rad))

        def require(ok, reason):
            log_("require", bool(ok), reason)
            if not ok:
                raise RuntimeError(reason)

        def evaluation(target, attempt, number, diag, step, t):
            log_("record_capture_evaluation", (target.x_m, target.y_m), number, step, t)
            self.state["perception"]["perception_error"] = {"position_m": 0.01}

        self.hooks = SimpleNamespace(
            require=require, wall=lambda: 100.0, path_record=lambda p: {"length_m": p.length_m},
            record_json=lambda v, indent=None: "JSON", known_pallet=lambda *a: log_("known_pallet", *a),
            grid_world=lambda sc: ("world", sc), grid_kwargs=lambda *a: log_("grid_kwargs", *a) or {"g": a},
            plan_transport=lambda *a, **k: log_("plan_transport", **k) or self.plans,
            final_straight_prefix=lambda approach, length: log_("final_straight_prefix", length) or leg(0.6),
            apply_tracker_profile=lambda: log_("apply_tracker_profile"),
            record_tracker_configs=lambda: log_("record_tracker_configs"),
            record_capture_evaluation=evaluation,
            planning_target=lambda target: log_("planning_target") or (target, "perception"),
            set_inset_estimate=lambda *a: log_("set_inset_estimate", *a), write_paths=lambda: log_("write_paths"),
            show_path=lambda path, label, color: log_("show_path", label), export_scene=lambda: log_("export_scene"),
            echo=lambda text: log_("echo", text), now_s=lambda: 77.0, snapshot=lambda *a: None,
        )
        self.ms = MissionState(phase="observe", phase_started=0.0)
        self.state = {"transitions": [], "planning_wall_s": 1.0,
                      "near_tracking": {"injection": {"captures": []}, "retry": {"capture_attempt": 3}}}
        if near_capture is not None:
            self.state["near_capture"] = {"status": near_capture}
        layer = SimpleNamespace(known_enabled=known, known_config={"r_fix_pickup_m": 0.04})
        self.obstacle = {"layer": layer}
        self.pocket = {"check": None}
        settings = {"tracker_curvature_inv_m": 0.45, "drive_acceleration_mps2": 0.5}
        speeds = {n: 0.3 for n in stages + ["observe", "return_home"]}
        self.core = MissionCore(
            self.ms, state=self.state, obstacle=self.obstacle, slam={"tracker": None} if slam else None,
            pocket=self.pocket, paths={}, trackers={}, geometry=SimpleNamespace(pallet_depth_m=0.8,
                                                                                alignment_straight_m=1.0),
            settings=settings, pallet_geometry=None, hooks=self.hooks, near=near,
            options=SimpleNamespace(planner_config=Config(), travel_config=None, scenario="SC", pickup_bounds="B",
                                    return_to_pose="HOME", mission_stages=stages, speeds=speeds,
                                    cusp_heading_hold_rad=0.08, cusp_heading_hold_phases=("observe", "return_home"),
                                    adapter=adapter, near_tracking=near_tracking, repeat_captures=repeat[0],
                                    repeat_at_attempt=repeat[1], extra_views=extra_views, near_inject=inject,
                                    near_inject_lateral_m=0.05, near_inject_yaw_rad=0.0, initial_time=10.0))
        # Moved into MissionCore (M-a1 slice 6e): this test's stand-ins go on the instance.
        for name_ in ['control_correction', 'known_pallet', 'return_turnaround_kwargs', 'realign_active', 'realign_install', 'start_near_realign', 'apply_tracker_profile', 'record_tracker_configs', 'build_near_tracker']:
            if hasattr(self.hooks, name_):
                setattr(self.core, name_, getattr(self.hooks, name_))

    def log(self, name, *args, **kwargs):
        self.calls.append((name, args, kwargs))

    def names(self):
        return [c[0] for c in self.calls if c[0] not in ("require", "grid_kwargs", "center", "site")]

    def succeed(self, observation=Observation(), attempt_number=2):
        attempt = {"t_before_capture_s": 29.0, "candidate_index": 1}
        scene = SimpleNamespace(stamp_ns=int(39.5e9))
        return self.core.observe_succeeded(30.0, REAR, BASE, 0.0, attempt, attempt_number, observation, scene,
                                           "DIAG", 600), attempt


def test_the_estimate_is_recorded_evaluated_and_planned_in_the_runners_order():
    w = World()
    out, attempt = w.succeed()
    assert out.stop is False and out.near_leg is None and out.observation == Observation()
    assert w.names() == ["set_inset_estimate", "record_capture_evaluation", "planning_target", "plan_transport",
                         "apply_tracker_profile", "record_tracker_configs", "write_paths", "show_path", "show_path",
                         "export_scene", "echo"]
    assert w.state["perception"]["perception_pickup_estimate_m"] == {"x_m": 3.5, "y_m": 2.0, "yaw_rad": 0.1}
    assert w.calls[[c[0] for c in w.calls].index("record_capture_evaluation")][1] == ((3.5, 2.0), 2, 600, 30.0)
    assert w.pocket["planning_pickup"] == Pose2D(3.5, 2.0, 0.1) and w.state["planning_target_source"] == "perception"
    plan_call = [c for c in w.calls if c[0] == "plan_transport"][0][2]
    assert plan_call["target_pickup"] == Pose2D(3.5, 2.0, 0.1) and plan_call["start_rear"] == Pose2D(1.0, 2.0, 0.0)
    assert plan_call["return_to"] == "HOME" and plan_call["g"] == ("mission", Pose2D(3.5, 2.0, 0.1))
    assert w.state["planning_traces"] == [[]] and w.state["planning_status"] == "success"
    assert set(w.core.paths) == {"approach", "insert", "extract", "transport", "withdraw"}
    assert w.core.trackers["insert"].config.overshoot_tolerance_m is None
    assert w.core.trackers["transport"].config.yaw_tolerance_rad == 0.02
    assert w.core.trackers["approach"].config.cusp_heading_pending_rad is None
    assert w.state["paths"]["transport"] == {"length_m": 4.0}
    assert [c[1][0] for c in w.calls if c[0] == "show_path"] == ["Approach", "Transport"]
    assert w.calls[-1][1] == ("PLANNED JSON",)
    assert w.state["perception"] is not attempt and "perception_pickup_estimate_m" not in attempt


def test_a_failed_mission_plan_ends_the_run_after_recording_it():
    w = World(plan_ok=False)
    with pytest.raises(RuntimeError, match="approach:no_path"):
        w.succeed()
    assert w.state["planning_status"] == "approach:no_path" and w.core.paths == {}


def test_the_first_far_observation_under_slam_opens_the_near_capture():
    w = World(slam=True)
    out, _ = w.succeed()
    assert out.near_leg is not None and out.near_leg.length_m == 0.6
    # Where the approach's final alignment straight begins (its length, 1.0 m here).
    assert [c for c in w.calls if c[0] == "final_straight_prefix"] == [("final_straight_prefix", (1.0,), {})]
    assert w.state["near_capture"] == {"status": "pending", "far_attempt": 2,
                                       "far_estimate_m": {"x_m": 3.5, "y_m": 2.0, "yaw_rad": 0.1},
                                       "far_error": {"position_m": 0.01}}
    w = World(slam=True, near_capture="pending")
    assert w.succeed()[0].near_leg is None  # the near capture itself: no second one


def test_a_near_capture_records_its_observation_base_and_stamps():
    w = World(near={"tracker": None}, near_capture="pending")
    w.succeed()
    start = w.core.near["start"]
    assert start["pose"] == (1.5, 2.0, 0.0) and start["aligned_s"] == 29.0
    assert start["capture_stamp_s"] == pytest.approx(29.5) and start["attempt"] == 2


def test_a_re_observation_stand_records_where_it_stood_and_its_planned_target():
    w = World(near={"tracker": None})
    w.succeed(attempt_number=3)
    retry = w.state["near_tracking"]["retry"]
    assert retry["capture_base"] == [1.5, 2.0, 0.0] and retry["target"] == [3.5, 2.0, 0.1]
    assert retry["planned_target"] == [3.5, 2.0, 0.1]


@pytest.mark.parametrize("inject, target_y, lateral", [("mismatch", 2.05, 0.05), ("correction", 2.05, 0.0)])
def test_the_near_field_injection_biases_the_observation_or_the_target_once(inject, target_y, lateral):
    w = World(near={"tracker": None}, inject=inject)
    out, _ = w.succeed()
    (capture,) = w.state["near_tracking"]["injection"]["captures"]
    assert capture["raw_target"] == [3.5, 2.0, 0.1] and capture["target"][1] == pytest.approx(target_y)
    assert capture["phase"] == "far" and capture["planned_target"][1] == pytest.approx(target_y)
    assert out.observation.lateral_m == lateral
    w = World(near={"tracker": None, "injection_closed": True}, inject=inject)
    w.succeed()
    assert w.state["near_tracking"]["injection"]["captures"] == []


def test_a_known_pallet_is_fixed_again_at_each_detection():
    w = World(known=True)
    w.state["observation_attempts"] = [{}, {}]
    w.succeed()
    (call,) = [c for c in w.calls if c[0] == "known_pallet"]
    assert call[1] == ((3.5, 2.0, 0.1), 0.04, "recognised:attempt2")


def test_the_repeat_diagnostic_ends_the_loop_on_an_early_valid_detection():
    w = World(repeat=(5, 3))
    out, _ = w.succeed()
    assert out.stop is True and w.ms.phase == "repeat_target_not_reached"
    assert w.names() == ["echo"]  # the transition's console line, nothing planned
    assert w.state["repeat_capture"]["reason"] == "valid_detection_before_target"


def test_extra_views_record_the_acceptance_time():
    w = World(extra_views=True)
    _, attempt = w.succeed()
    assert attempt["acceptance_simulation_time_s"] == 77.0


def test_the_observe_follower_is_kept_and_an_injection_at_the_near_capture_is_labelled_near():
    w = World(near={"tracker": None}, near_capture="pending", inject="correction")
    observe = object()
    w.core.paths["observe"], w.core.trackers["observe"] = leg(), observe
    w.succeed()
    assert w.core.trackers["observe"] is observe  # the mission plan does not rebuild the leg being driven
    assert w.state["near_tracking"]["injection"]["captures"][0]["phase"] == "near"


def test_a_mission_without_a_final_straight_cannot_open_the_near_capture():
    w = World(slam=True)
    w.hooks.final_straight_prefix = lambda approach, length: None
    with pytest.raises(RuntimeError, match="near_capture_no_final_straight"):
        w.succeed()
    assert "near_capture" not in w.state


# M-a2 (docs/plans/2026-10-16-m-a2-source-tests-to-behaviour.md): the followers built after a detection, by rule.
def test_every_follower_built_after_a_detection_takes_its_stages_tolerances_cusp_and_speed_rules():
    w = World()
    w.plans.return_home = leg(9.0)
    w.core.options.mission_stages = ["approach", "insert", "extract", "transport", "withdraw", "return_home"]
    w.core.settings.update(max_lateral_acceleration_mps2=0.4, max_reverse_speed_mps=0.2)
    kept = object()
    w.core.trackers["observe"] = kept  # the observe follower is not rebuilt here
    w.succeed()
    trackers = w.core.trackers
    assert trackers["observe"] is kept
    for name in ("approach", "insert", "extract", "transport", "withdraw", "return_home"):
        c = trackers[name].config
        assert c.position_tolerance_m == 0.008, name
        assert c.yaw_tolerance_rad == (0.03 if name == "return_home" else 0.02), name
        # gear-change cusps released alike everywhere, braked and judged at 8 mm
        assert (c.cusp_position_tolerance_m, c.cusp_yaw_tolerance_rad, c.cusp_brake_window_m) == (0.03, 0.05, 0.008)
        assert c.overshoot_tolerance_m == (None if name == "insert" else 0.03), name
        assert (c.max_lateral_acceleration_mps2, c.max_reverse_speed_mps) == (0.4, 0.2), name  # read from settings
        assert c.cusp_heading_pending_rad == (0.08 if name == "return_home" else None), name


def test_without_speed_caps_in_the_settings_the_followers_have_none():
    w = World()
    w.succeed()
    c = w.core.trackers["transport"].config
    assert c.max_lateral_acceleration_mps2 is None and c.max_reverse_speed_mps is None


def test_the_mission_is_planned_with_the_recorded_planner_config_itself():
    """M-a2 (Codex review P2): the mission plan after a detection gets the run's planner config object -- the one
    the run records -- not a copy with other settings."""
    w = World()
    received = []
    plans = w.hooks.plan_transport
    w.hooks.plan_transport = lambda *a, **k: received.append(a) or plans(*a, **k)
    w.succeed()
    (args,) = received
    assert args[1] is w.core.options.planner_config


def test_the_tracker_profile_is_applied_to_the_followers_built_after_the_detection():
    """M-a2 sweep (order): the profile rewrites the new followers, not the ones they replace."""
    w = World()
    old = dict(w.core.trackers)
    seen = []
    w.core.apply_tracker_profile = lambda: seen.append(dict(w.core.trackers))
    w.succeed()
    (at_call,) = seen
    assert at_call["approach"] is w.core.trackers["approach"] and at_call["approach"] is not old.get("approach")
