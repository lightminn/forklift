"""M-a1 slice 6d: the near field's mission side, moved from run() closures into MissionCore methods
(behaviour, state, port requests and their order, effects up to a refusal -- the plan's M-a1 criterion).

The stop and correction rules are near_tracking's own (decide_stop, correction_update, retry_allowed);
only its geometry (the carried face, gaps, budgets) is stood in for, so each test steers one branch."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control.path_tracking import TrackerConfig
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.planning import PlanResult, Pose2D

ROOT = Path(__file__).resolve().parents[3]
if "near_tracking" in sys.modules:
    NT = sys.modules["near_tracking"]
else:
    SPEC = importlib.util.spec_from_file_location("near_tracking", ROOT / "sim/isaac/near_tracking.py")
    NT = importlib.util.module_from_spec(SPEC)
    sys.modules["near_tracking"] = NT
    SPEC.loader.exec_module(NT)


def leg(x0=0.0, length=1.0):
    poses = np.array([[x0, 0.0, 0.0], [x0 + length, 0.0, 0.0]])
    return PlanResult(True, "success", poses, np.ones(2, np.int8), np.zeros(2), length, 3)


class Geometry:
    """near_tracking with its geometry stood in for: the face is ``face_m`` ahead of the fork tip."""

    def __init__(self, face_m=2.0, budget_ok=True, entered=False, k_tip=1.0, gap_margin=0.01, carriage_gap=0.5,
                 sigma=0.005):
        self.face_m, self.budget_ok, self.entered_ = face_m, budget_ok, entered
        # The protection: the tip sweep factor, the worst blade-wall margin, the carriage corners' gap and
        # uncertainty.
        self.k_tip, self.gap_margin, self.carriage_gap, self.sigma = k_tip, gap_margin, carriage_gap, sigma

    def module(self):
        g = self
        ns = SimpleNamespace(**{k: v for k, v in vars(NT).items() if not k.startswith("__")})
        ns.face_min_x_m = lambda held, widths, base: g.face_m
        ns.section_entered = lambda face_x, tip: face_x <= 1.5
        ns.carried_face = lambda held, widths, base: {s_: {w_: (g.face_m + 0.9, 0.2) for w_ in ("outer", "inner")}
                                                      for s_ in ("left", "right")}
        ns.corner_sweep_factor = lambda *a: g.k_tip
        ns.blade_gaps = lambda face, blades, depths, rear_x: {"points": [{"gap_m": 0.05, "at": "left_inner"}],
                                                              "rho_m": 1.0}
        ns.lateral_sweep_m = lambda *a: 0.001
        ns.gap_threshold_m = lambda point, bound, odometry, rho, face, lat: 0.05 - g.gap_margin
        ns.carriage_gaps = lambda face, corners: [g.carriage_gap, g.carriage_gap]
        ns.carriage_uncertainty_m = lambda *a: g.sigma
        ns.face_distance_m = lambda face, tip: g.face_m
        ns.entered = lambda face, tip: g.entered_
        ns.result_wait_s = lambda *a: 0.05
        ns.travel_budget = lambda *a: SimpleNamespace(ok=g.budget_ok, reason=None if g.budget_ok else "travel",
                                                      len_term_m=0.0, d_m=0.0, horizon_s=0.1, s_term_m=0.0,
                                                      travel_m=0.0, total_m=0.0)
        return ns


class Tracker:
    def __init__(self, armed=False, lost=False, failed=False, source="front", frame_version=1, stamp_s=9.9):
        self.armed, self.mode = armed, "tracking"
        self.lost, self.failed = lost, failed
        self.config = SimpleNamespace(camera_xy_m=(0.4, 0.0))
        obs = SimpleNamespace(left=SimpleNamespace(width_m=0.2), right=SimpleNamespace(width_m=0.2))
        self.latest = SimpleNamespace(observation=obs, held={"left": (1.0, 0.1)}, stamp_s=stamp_s, source=source,
                                      frame_version=frame_version, aligned_s=stamp_s - 0.05,
                                      bound={"along_m": 0.01, "yaw_rad": 0.01}, pose=(0.0, 0.0, 0.0))

    def arm(self, now_s):
        self.armed = True

    def status(self, now_s):
        return SimpleNamespace(lost=self.lost, failed=self.failed)


class History:
    def add(self, stamp, rear, episode):
        self.last = (stamp, tuple(rear), episode)

    def base_at(self, stamp, episode):
        return (0.0, 0.0, 0.0)

    def path_length_since(self, t, episode):
        return 0.01


def make(geom=None, *, tracker=None, phase="approach", standing=False, retries=0, mode="holding", holds=1,
         stop_state=None, correction=None, drop_from=None):
    geom = geom or Geometry()
    calls = []

    def require(ok, reason):
        calls.append(("require", bool(ok), reason))
        if not ok:
            raise RuntimeError(reason)

    near = {
        "tracker": tracker or Tracker(), "path_revision": 0, "episode": holds, "history": History(),
        "bounds": SimpleNamespace(max_latency_s=0.1, render_period_s=0.1, read_delay_s=0.02,
                                  odometry=lambda age: (0.001, 0.001), align_latency_s=0.05),
        "terms": SimpleNamespace(delta_v_mps=0.01, delta_s_m=0.01, at=lambda *a: 0.01),
        "commands": SimpleNamespace(vbar=lambda *a, **k: 0.1), "last_stamp": 9.9, "fork_tip_x_m": 0.9,
        "blade_y_max_m": 0.3, "blades": (), "carriage_limit_m": 0.35, "corners": (), "protect": None,
        "stop_state": stop_state or NT.StopState(), "correction": correction or NT.CorrectionState(),
        "corrections": [], "events": [], "stop_events": [], "ticks": [], "stale_waits": {}, "retries": retries,
        "path_revisions": [],
    }
    slam = {"tracker": SimpleNamespace(mode=mode), "holds": [{"event": "hold"}] * holds, "stop_now": standing}
    hooks = SimpleNamespace(require=require, steering_position=lambda: np.zeros(2),
                            steering_command=lambda: np.zeros(2), echo=lambda text: None,
                            record_json=lambda v, indent=None: "{}", snapshot=lambda phase: None)
    state = {"transitions": [], "near_tracking": {}, "paths": {}}
    core = MissionCore(MissionState(phase=phase, phase_started=0.0), state=state, obstacle=None, slam=slam,
                       pocket={"check": None, "planning_pickup": Pose2D(5.0, 0.0, 0.0)}, paths={}, trackers={},
                       geometry=None, settings={}, pallet_geometry=None, hooks=hooks, near=near,
                       options=SimpleNamespace(near_tracking=geom.module(), rear_axle_offset_m=0.35,
                                               near_drop_frames_from_m=drop_from,
                                               drive_geometry=SimpleNamespace(wheelbase_m=0.8, track_m=0.5)))
    # Moved into MissionCore (M-a1 slice 6e): this test's stand-ins go on the instance.
    for name_ in ['control_correction', 'known_pallet', 'return_turnaround_kwargs', 'realign_active', 'realign_install', 'start_near_realign', 'apply_tracker_profile', 'record_tracker_configs', 'build_near_tracker']:
        if hasattr(hooks, name_):
            setattr(core, name_, getattr(hooks, name_))
    # The moved neighbours near_tick calls: stand-ins on the instance, logged.
    core.near_redraw = lambda latest, rear: calls.append(("near_redraw",)) or core.test_redraw
    core.near_protection_prediction = lambda *a: calls.append(("prediction",)) or core.test_prediction
    core.near_install = lambda now, plans, target, point, rear: calls.append(("install", point)) or {"revision": 1}
    core.start_pickup_retry = lambda *a: calls.append(("retry", a[1]))
    core.test_redraw = ({"approach": leg()}, {"redraw": 1}, Pose2D(5.1, 0.0, 0.0))
    core.test_prediction = {"disposition": "pass"}
    return core, near, calls


def tick(core, t=10.0, phase="approach", arrived=False):
    return core.near_tick(t, t, (0.0, 0.0, 0.0), 0.2, phase, 0.0, arrived)


def test_hold_count_is_the_slam_hold_episodes():
    core, _, _ = make(holds=3)
    core.slam["holds"] = [{"event": "hold"}, {"event": "release"}, {"event": "hold"}]
    assert core.hold_count() == 2
    core.slam = None
    assert core.hold_count() == 0


def test_a_released_hold_or_a_new_episode_ends_the_run_before_anything_is_recorded():
    core, near, _ = make(mode="tracking")
    with pytest.raises(RuntimeError, match="near_field_frame_released"):
        tick(core)
    core, near, _ = make(holds=2)
    near["episode"] = 1
    with pytest.raises(RuntimeError, match="near_field_frame_released"):
        tick(core)
    assert near["ticks"] == [] and near["events"] == []


def test_far_off_the_tracker_arms_at_the_section_and_a_clear_tick_neither_holds_nor_arrives():
    core, near, _ = make(Geometry(face_m=2.0))
    assert tick(core) == (False, False, False)
    assert not near["tracker"].armed and near["ticks"][-1]["reason"] is None
    core.options.near_tracking.face_min_x_m = lambda *a: 1.4  # inside the 1.5 m section
    tick(core, 10.1)
    assert near["tracker"].armed and near["events"][-1]["event"] == "armed"
    assert near["history"].last == (10.1, (0.0, 0.0, 0.0), 1)


def test_an_armed_tracker_waits_on_a_lost_view_or_a_spent_budget_and_records_the_new_stop():
    core, near, _ = make(Geometry(budget_ok=False), tracker=Tracker(armed=True, lost=True))
    assert tick(core) == (True, False, False)
    assert near["stop_state"].reason == "lost_wait" and near["ticks"][-1]["waits"] == ["lost_wait", "budget:travel"]
    (stop,) = near["stop_events"]
    assert stop["reason"] == "lost_wait" and stop["standing_s"] is None and stop["vbar_mps"] == 0.1
    near["tracker"].lost = False
    core.options.near_tracking = Geometry(budget_ok=True).module()
    assert tick(core, 10.1) == (True, False, False)  # nothing waits, but it releases only standing
    core.slam["stop_now"] = True
    assert tick(core, 10.2) == (False, True, False)
    assert near["events"][-1]["event"] == "release" and near["stop_events"][-1]["standing_s"] == 10.2


def test_a_correction_point_opens_waits_for_a_standing_observation_and_installs_the_redraw():
    core, near, calls = make(Geometry(face_m=0.95))
    assert tick(core, 10.0) == (True, False, False)  # the 1.0 m point opens: a waiting reason
    assert near["corrections"][0]["point_m"] == 1.0 and near["events"][-1]["event"] == "stop"
    core.slam["stop_now"] = True
    tick(core, 10.1)  # standing from here
    assert near["corrections"][0]["standing_s"] == [10.1] and "near_redraw" not in [c[0] for c in calls]
    near["tracker"].latest.stamp_s = 10.5  # an observation whose pixel time is after t_s
    tick(core, 10.6)
    assert [c[0] for c in calls if c[0] != "require"] == ["near_redraw", "install"]  # no prediction at 1.0 m
    rec = near["corrections"][0]
    assert rec["accepted"] is True and rec["revision"] == 1 and rec["observation"]["source"] == "front"
    assert near["correction"].installed_s == 10.6
    tick(core, 10.7)
    assert near["corrections"][0]["closed_s"] == 10.7 and near["correction"].next_index == 1


@pytest.mark.parametrize("zone, installed", [((4.5, 5.5, -0.5, 0.5), True), ((0.0, 1.0, -0.5, 0.5), False)])
def test_a_correction_redraw_is_checked_against_the_zone_before_it_is_installed(zone, installed):
    """Plan 2026-10-28-adoption-zone-check: the redraw's target (5.1, 0.0) against the rectangle + 0.25 m."""
    from forklift_core.mission.adoption_zone import AdoptionZone

    core, near, calls = make(Geometry(face_m=0.95))
    core.options.adoption_zone_check, core.options.adoption_zone_margin_m = "on", 0.25
    core.options.adoption_zone = AdoptionZone(*zone, "scene_layout")
    tick(core, 10.0)
    core.slam["stop_now"] = True
    tick(core, 10.1)
    near["tracker"].latest.stamp_s = 10.5
    if installed:
        tick(core, 10.6)
        assert [c[0] for c in calls if c[0] != "require"] == ["near_redraw", "install"]
        assert core.state["adoption_zone"]["checks"][0]["stage"] == "tracked_redraw"
    else:
        with pytest.raises(RuntimeError, match="^adoption_outside_zone:tracked_redraw$"):
            tick(core, 10.6)
        assert "install" not in [c[0] for c in calls]


def test_a_correction_from_the_roof_detector_or_another_hold_is_refused_into_a_terminal_stop():
    core, near, calls = make(Geometry(face_m=0.95), tracker=Tracker(source="roof"), standing=True,
                             correction=NT.CorrectionState(0, 1.0, 9.0, 9.0))
    near["corrections"].append({"point_m": 1.0, "standing_s": [9.0], "moved_s": [], "accepted": None})
    tick(core, 10.0)
    assert near["corrections"][-1]["refused"] == "source:roof" and near["stop_state"].reason == "replan_rejected"
    assert "near_redraw" not in [c[0] for c in calls]
    core, near, calls = make(Geometry(face_m=0.95), tracker=Tracker(frame_version=0), standing=True,
                             correction=NT.CorrectionState(0, 1.0, 9.0, 9.0))
    near["corrections"].append({"point_m": 1.0, "standing_s": [9.0], "moved_s": [], "accepted": None})
    tick(core, 10.0)
    assert near["corrections"][-1]["refused"] == "frame:0!=1"


def test_the_last_correction_point_screens_the_protection_and_a_predicted_shortfall_is_refused():
    core, near, calls = make(Geometry(face_m=0.55), standing=True, correction=NT.CorrectionState(1, 0.6, 9.0, 9.0))
    near["corrections"].append({"point_m": 0.6, "standing_s": [9.0], "moved_s": [], "accepted": None})
    core.test_prediction = {"disposition": "refused"}
    tick(core, 10.0)
    assert [c[0] for c in calls if c[0] != "require"] == ["near_redraw", "prediction"]
    rec = near["corrections"][-1]
    assert rec["accepted"] is False and rec["redraw"]["refused"] == "protection_predicted"
    assert rec["redraw"]["protection_prediction"] == {"disposition": "refused"}
    assert near["stop_state"].reason == "replan_rejected" and near["stop_state"].terminal


def test_a_terminal_stop_waits_its_hold_standing_then_retries_once_before_the_protection():
    core, near, calls = make(Geometry(face_m=0.8), standing=True)
    near["mismatch"] = True
    assert tick(core, 10.0) == (True, False, False)
    assert near["terminal_standing_since"] == 10.0 and "retry" not in [c[0] for c in calls]
    assert tick(core, 10.0 + NT.TERMINAL_HOLD_S) == (True, False, False)
    assert ("retry", "mismatch") in calls


def test_a_terminal_stop_without_a_retry_left_ends_the_run_with_its_outcome():
    core, near, calls = make(Geometry(face_m=0.8), standing=True, retries=1)
    near["mismatch"] = True
    tick(core, 10.0)
    with pytest.raises(RuntimeError, match="near_field_mismatch"):
        tick(core, 10.0 + NT.TERMINAL_HOLD_S)
    assert core.state["near_tracking"]["outcome"] == "near_field_mismatch"
    assert near["events"][-1] == {"time_s": 10.5, "event": "terminal_standing", "reason": "mismatch"}


def test_arriving_on_the_approach_with_a_point_never_opened_is_terminal():
    core, near, _ = make(Geometry(face_m=2.0))
    tick(core, 10.0, arrived=True)
    assert near["stop_state"].reason == "correction_missed"


def test_the_frame_drop_injection_latches_at_its_distance():
    core, near, _ = make(Geometry(face_m=1.2), drop_from=1.3)
    tick(core, 10.0)
    assert near["drop_latch"] == {"time_s": 10.0, "face_distance_m": 1.2, "latest_stamp_s": 9.9}
    tick(core, 10.1)
    assert [e["event"] for e in near["events"]].count("drop_latch") == 1


def install_core(tmp_path, *, obstacle=None):
    calls = []
    log = lambda *a: calls.append(a)  # noqa: E731
    paths = {n: leg() for n in ("approach", "insert", "extract")}
    trackers = {n: SimpleNamespace(config=TrackerConfig(cruise_speed_mps=0.3 + i / 10))
                for i, n in enumerate(paths)}
    hooks = SimpleNamespace(path_record=lambda p: {"length_m": p.length_m}, add_path_display=lambda *a: log("show", a[2]),
                            known_pallet=lambda rect, r, event: log("known", rect, r, event), loaded=lambda: False,
                            stage=lambda: "STAGE", write_paths=lambda: log("write_paths"), require=lambda ok, r: None)
    near = {"path_revision": 0, "episode": 2, "path_revisions": [], "events": []}
    core = MissionCore(MissionState(phase="approach", phase_started=0.0), state={"transitions": [], "paths": {}},
                       obstacle=obstacle, slam=None, pocket={"check": None, "planning_pickup": Pose2D(5.0, 0.0, 0.0)},
                       paths=paths, trackers=trackers, geometry=None, settings={}, pallet_geometry=None, hooks=hooks,
                       near=near, options=SimpleNamespace(near_tracking=NT))
    # Moved into MissionCore (M-a1 slice 6e): this test's stand-ins go on the instance.
    for name_ in ['control_correction', 'known_pallet', 'return_turnaround_kwargs', 'realign_active', 'realign_install', 'start_near_realign', 'apply_tracker_profile', 'record_tracker_configs', 'build_near_tracker']:
        if hasattr(hooks, name_):
            setattr(core, name_, getattr(hooks, name_))
    return core, near, calls


def test_an_accepted_correction_installs_all_three_straights_with_their_configs_and_a_revision(tmp_path):
    refreshed = []
    obstacle = {"applied": (0, 0, 0), "version": 3, "progress": {"x": 1},
                "layer": SimpleNamespace(known_enabled=True, known_config={}, refresh=lambda *a, **k: refreshed.append(k))}
    core, near, calls = install_core(tmp_path, obstacle=obstacle)
    plans = {n: leg(0.2, 0.8) for n in ("approach", "insert", "extract")}
    entry = core.near_install(11.0, plans, Pose2D(5.1, 0.02, 0.01), 0.6, (0.1, 0.0, 0.0))
    assert entry["revision"] == 1 and near["path_revision"] == 1 and near["path_revisions"] == [entry]
    for i, n in enumerate(("approach", "insert", "extract")):
        assert core.paths[n] is plans[n] and core.trackers[n].config.cruise_speed_mps == pytest.approx(0.3 + i / 10)
        assert entry["trackers"][n] == entry["paths"][n]["sha256"]  # the follower's copy is the path
    assert entry["previous_target"] == [5.0, 0.0, 0.0] and core.pocket["planning_pickup"] == Pose2D(5.1, 0.02, 0.01)
    assert calls[0] == ("write_paths",) and calls[1] == ("show", "Approach")
    assert calls[2] == ("known", (5.1, 0.02, 0.01), 0.05, "correction:0.6")
    assert obstacle["pickup_estimate"] == Pose2D(5.1, 0.02, 0.01) and obstacle["progress"] == {}
    assert refreshed[0]["current_pose"] == (0.1, 0.0, 0.0) and core.ms.phase_started == 11.0
    core, near, calls = install_core(tmp_path)
    core.near_install(11.0, plans, Pose2D(5.1, 0.0, 0.0), 1.0, (0.1, 0.0, 0.0))  # no obstacle layer
    assert [c[0] for c in calls] == ["write_paths", "show"]


def test_a_pickup_retry_closes_the_episode_releases_and_stands_in_observe():
    core, near, _ = make()
    near["ticks"] = [{"episode": 1, "face_distance_m": 0.9}, {"episode": 1, "face_distance_m": 0.7},
                     {"episode": 0, "face_distance_m": 0.1}]
    near.update(mismatch=True, handoff_missed=True, terminal_standing_since=9.0, protect={"x": 1})
    core.state["near_capture"] = {"status": "accepted"}
    order = []
    core.release_and_stand = lambda now, event: order.append(("release", near["tracker"], event)) or {"jump_m": 0.0}
    del core.start_pickup_retry  # the method itself, not the stand-in
    core.start_pickup_retry(12.0, "mismatch", 0.7, 1)
    retry = core.state["near_tracking"]["retry"]
    assert retry["min_face_distance_m"] == 0.7 and retry["jump_m"] == 0.0 and near["retries"] == 1
    assert order == [("release", None, "release_pickup_retry")]  # the tracker is gone before the release
    assert near["mismatch"] is False and near["protect"] is None and "terminal_standing_since" not in near
    assert "handoff_missed" not in near
    assert core.state["near_capture_history"] == [{"status": "accepted"}] and "near_capture" not in core.state
    assert core.ms.phase == "observe" and near["stand_record"] == "retry"


def test_the_near_capture_redraw_checks_the_correction_points_on_the_trajectories_its_port_returned():
    """In a replay the Planner response comes back without running the dry-run callbacks: the check
    must read the trajectories from that response (M-a1 slice 6d)."""
    checked = []
    nt = SimpleNamespace(**{k: v for k, v in vars(NT).items() if not k.startswith("__")})
    nt.rollout_face_distance = lambda held, widths, tip, offset: (lambda pose: 1.0)
    nt.correction_points_check = lambda trajs, start, fd: checked.append(trajs) or ([{"m": 1.0}], len(trajs) == 2)
    adapter = SimpleNamespace(estimate_pallet_center_m=lambda obs, depth: (1.0, 0.0),
                              estimate_world_pallet_site=lambda xy, yaw, base, base_yaw: Pose2D(5.0, 0.0, 0.0))
    asked, grids = [], []
    hooks = SimpleNamespace(
        grid_world=lambda sc: ("world", sc),
        correction_grid=lambda first, target, rear: grids.append((first, target, rear)) or {"occupancy": "GRID"},
        tracking_frame=lambda: SimpleNamespace(fork_tip_x_m=0.9),
        redraw_docking_traced=lambda *a, **k: asked.append((a, k)) or core.test_response,
    )
    obs = SimpleNamespace(left=SimpleNamespace(width_m=0.2, centre_xy_m=(1.0, 0.2), yaw_rad=0.0),
                          right=SimpleNamespace(width_m=0.2, centre_xy_m=(1.0, -0.2), yaw_rad=0.0),
                          insertion_yaw_rad=0.0)
    from forklift_core.planning.pallet_mission import SyntheticMissionGeometry

    geometry = SyntheticMissionGeometry()
    core = MissionCore(MissionState(phase="observe", phase_started=0.0), state={"transitions": []}, obstacle=None,
                       slam=None, pocket={"check": None, "planning_pickup": Pose2D(5.0, 0.0, 0.0)}, paths={},
                       trackers={}, geometry=geometry, settings={}, pallet_geometry=None, hooks=hooks, near={},
                       options=SimpleNamespace(near_tracking=nt, adapter=adapter, pickup_bounds="B", scenario="SC",
                                               planner_config="CFG", rear_axle_offset_m=0.35))
    # Moved into MissionCore (M-a1 slice 6e): this test's stand-ins go on the instance.
    for name_ in ['control_correction', 'known_pallet', 'return_turnaround_kwargs', 'realign_active', 'realign_install', 'start_near_realign', 'apply_tracker_profile', 'record_tracker_configs', 'build_near_tracker']:
        if hasattr(hooks, name_):
            setattr(core, name_, getattr(hooks, name_))
    trajs = [np.zeros((3, 3)), np.ones((3, 3))]
    core.test_response = ({"approach": leg()}, {"refused": None}, trajs)
    cfg_a, cfg_d = TrackerConfig(cruise_speed_mps=0.2), TrackerConfig(cruise_speed_mps=0.3)
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr("forklift_core.mission.core.asdict", lambda v: {"cfg": getattr(v, "cruise_speed_mps", None)})
        mp.setattr("forklift_core.perception.near_field_tracking.held_pockets", lambda obs_, base: {"held": base})
        plans, entry = core.near_capture_redraw((4.0, 0.0, 0.0), (3.9, 0.0, 0.0), obs, 2, cfg_a, cfg_d, 13.0)
        assert plans is not None and entry["accepted"] is True and checked == [trajs]
        (args, kwargs), = asked
        assert args[0] == ("world", "SC") and kwargs["dry_configs"] == (cfg_a, cfg_d)
        assert kwargs["start_rear"] == (4.0, 0.0, 0.0) and kwargs["occupancy"] == "GRID"
        assert entry["fork_tip_x_m"] == 0.9 and entry["dry_run_cruise_mps"] == [0.2, 0.3]
        # The grid clears this capture's own band only: the first target is the target being drawn.
        assert grids[0][0] is grids[0][1] and grids[0][2] == (4.0, 0.0, 0.0)
        core.test_response = ({"approach": leg()}, {"refused": None}, trajs[:1])  # one rollout: outside the box
        plans, entry = core.near_capture_redraw((4.0, 0.0, 0.0), (3.9, 0.0, 0.0), obs, 3, cfg_a, cfg_d, 14.0)
        assert plans is None and entry["record"]["refused"] == "correction_point" and entry["accepted"] is False


# --- The protection (Codex 6d review P2): its entry, fresh and stale evidence, the insertion end ---

def protected(geom, *, stamp_s=10.0, odometry=(0.001, 0.001), **kw):
    # Both correction points done, so none opens on the approach.
    core, near, calls = make(geom, tracker=Tracker(armed=True, stamp_s=stamp_s),
                             correction=NT.CorrectionState(next_index=2), **kw)
    near["bounds"].odometry = lambda age: odometry
    near["corners"] = ((0.35, 0.2), (0.35, -0.2))
    return core, near, calls


def test_the_protection_starts_within_w_of_the_face_and_a_short_tip_stopping_distance_is_terminal():
    core, near, _ = protected(Geometry(face_m=0.14))
    tick(core, 10.0, phase="approach")
    p = near["protect"]
    assert p["phase"] == "approach" and p["face_distance_m"] == 0.14 and p["kappa_b"] == 0.0
    assert near["events"][0]["event"] == "protect" and near["stop_state"].reason is None
    row = near["ticks"][-1]
    assert row["protect"] is True and row["protection"]["fresh"] is True and row["protection"]["inside"] is False
    # Entered with less than the along error plus the tip's stopping sweep left: stuck.
    core, near, _ = protected(Geometry(face_m=0.14, k_tip=100.0))
    tick(core, 10.0, phase="approach")
    assert near["stop_state"].reason == "stuck" and near["stop_state"].terminal
    # No odometry bound: unbounded, not a measured shortfall.
    core, near, _ = protected(Geometry(face_m=0.14), odometry=None)
    tick(core, 10.0, phase="approach")
    assert near["stop_state"].reason == "unbounded" and near["ticks"][-1]["protect"] is True


def test_a_gap_shortfall_ends_on_fresh_evidence_and_waits_on_stale_until_a_new_observation():
    core, near, _ = protected(Geometry(face_m=0.5, gap_margin=-0.01), stamp_s=10.0)
    tick(core, 10.0, phase="insert")
    assert near["stop_state"].reason == "stuck"  # a new observation confirms it
    core, near, _ = protected(Geometry(face_m=0.5, gap_margin=-0.01), stamp_s=9.5)
    assert tick(core, 10.0, phase="insert") == (True, False, False)
    assert near["stop_state"].reason == "gap_wait" and near["stale_waits"] == {"gap_wait": 9.5}
    core.options.near_tracking = Geometry(face_m=0.5).module()  # the margin is back
    tick(core, 10.1, phase="insert")
    assert near["stale_waits"] == {"gap_wait": 9.5}  # held: no observation newer than the one that raised it
    near["tracker"].latest.stamp_s = 10.15
    core.slam["stop_now"] = True
    assert tick(core, 10.2, phase="insert") == (False, True, False)  # decided afresh on it, and released standing
    assert near["stale_waits"] == {}


def test_the_insertion_end_latches_on_fresh_certain_evidence_and_arrives_standing():
    core, near, _ = protected(Geometry(face_m=0.1, carriage_gap=0.0))
    assert tick(core, 10.0, phase="insert") == (True, False, False)  # latched; not standing yet
    assert near["stop_state"].insert_end is True and near["stop_state"].reason == "insert_end"
    assert near.get("arrival") is None
    core.slam["stop_now"] = True
    near["tracker"].latest.stamp_s = 10.1  # a fresh observation this tick too (an old one would wait again)
    assert tick(core, 10.1, phase="insert") == (True, False, True)
    assert near["arrival"]["reason"] == "insert_end" and near["arrival"]["time_s"] == 10.1
    assert near["events"][-1]["event"] == "insert_end_arrival"
    # Unconfirmed (old evidence) or uncertain: it stands and waits, never latches.
    core, near, _ = protected(Geometry(face_m=0.1, carriage_gap=0.0), stamp_s=9.5, standing=True)
    tick(core, 10.0, phase="insert")
    assert near["stop_state"].insert_end is False and near["stop_state"].reason == "insert_end_unconfirmed"
    core, near, _ = protected(Geometry(face_m=0.1, carriage_gap=0.0, sigma=0.02, entered=True), standing=True)
    tick(core, 10.0, phase="insert")
    assert near["stop_state"].insert_end is False
    assert near["ticks"][-1]["waits"] == ["carriage_uncertain", "insert_end_unconfirmed"]
    # Only in the insertion: on the approach a small carriage gap does not end anything.
    core, near, _ = protected(Geometry(face_m=0.1, carriage_gap=0.0), standing=True)
    tick(core, 10.0, phase="approach")
    assert near["stop_state"].insert_end is False


# --- The real near_redraw, near_protection_prediction and the runner's redraw_docking_traced ---

def test_near_redraw_places_the_latest_observation_and_dry_runs_both_cruises_from_here():
    asked, rolled = [], []
    adapter = SimpleNamespace(estimate_pallet_center_m=lambda obs, depth: (2.0, 0.1),
                              estimate_world_pallet_site=lambda xy, yaw, base, base_yaw: Pose2D(xy[0] + base[0], xy[1] + base[1], yaw + base_yaw))
    g2 = SimpleNamespace(planning_target=lambda mode, site, pickup: (Pose2D(5.2, 0.1, 0.02), None))

    def redraw_docking(world, start, target, config, **k):
        asked.append((world, start, target, config, k))
        for dry in k["dry_runs"]:
            dry(SimpleNamespace(poses="P", directions="D", curvatures_inv_m="K"))
        return {"approach": leg()}, {"refused": None}

    driven = TrackerConfig(cruise_speed_mps=0.29)
    hooks = SimpleNamespace(
        redraw_docking=redraw_docking, grid_world=lambda sc: ("world", sc),
        bicycle_rollout=lambda poses, dirs, curv, config, start: rolled.append((config.cruise_speed_mps, start)),
        correction_grid=lambda first, target, rear: {"occupancy": ("GRID", first, target)})
    near = {"first_target": Pose2D(5.0, 0.0, 0.0)}
    core = MissionCore(MissionState(phase="approach", phase_started=0.0), state={"transitions": []}, obstacle=None,
                       slam=None, pocket={"check": None, "planning_pickup": Pose2D(5.1, 0.0, 0.0)}, paths={},
                       trackers={"approach": SimpleNamespace(config=driven)}, geometry=SimpleNamespace(pallet_depth_m=0.8),
                       settings={}, pallet_geometry=None, hooks=hooks, near=near,
                       options=SimpleNamespace(near_tracking=NT, adapter=adapter, G2=g2, planning_target="estimate",
                                               scenario=SimpleNamespace(pickup="PICKUP"), approach_straight_speed_mps=0.2,
                                               speeds={"approach": 0.2}, planner_config="CFG", pickup_bounds="B"))
    latest = SimpleNamespace(observation=SimpleNamespace(insertion_yaw_rad=0.01), pose=(1.0, 0.5, 0.1))
    rear_now = (1.3, 0.4, 0.05)  # where the truck stands now, apart from where the observation was placed
    plans, record, target = core.near_redraw(latest, rear_now)
    (world, start, tgt, config, k), = asked
    assert world == ("world", core.options.scenario) and (start.x_m, start.y_m, start.yaw_rad) == rear_now
    assert tgt == Pose2D(5.2, 0.1, 0.02) == target and config == "CFG" and k["max_yaw_rad"] == NT.CORRECTION_MAX_YAW_RAD
    assert k["occupancy"] == ("GRID", near["first_target"], target) and k["pickup_bounds"] == "B"
    assert [r[0] for r in rolled] == [0.2, 0.29] and all(np.allclose(r[1], rear_now) for r in rolled)
    assert record["previous_target"] == [5.1, 0.0, 0.0] and record["dry_run_cruise_mps"] == [0.2, 0.29]
    assert record["site"] == pytest.approx([3.0, 0.6, 0.11])  # the observation placed with its own pose
    core.options.approach_straight_speed_mps = None  # no slower approach straight: both dry runs at the driven cruise
    rolled.clear()
    core.near_redraw(latest, rear_now)
    assert [r[0] for r in rolled] == [0.29, 0.29]


def test_the_protection_prediction_carries_its_inputs_and_disposes_by_the_retry_rule():
    nt = SimpleNamespace(**{k: v for k, v in vars(NT).items() if not k.startswith("__")})
    asked = []
    nt.protection_prediction = lambda arrays, config, rear, held, widths, **k: asked.append(k) or {"margin_phys_m": -0.01}
    near = {"fork_tip_x_m": 0.9, "tracker": SimpleNamespace(config=SimpleNamespace(camera_xy_m=(0.4, 0.0))),
            "blades": "BLADES", "carriage_limit_m": 0.35, "blade_y_max_m": 0.3, "bounds": "BOUNDS", "terms": "TERMS",
            "protect": None, "retries": 0}
    core = MissionCore(MissionState(phase="approach", phase_started=0.0), state={
        "transitions": [], "near_tracking": {"bounds_sha256": "B1", "drive_terms_sha256": "D1"}}, obstacle=None, slam=None,
        pocket={"check": None}, paths={}, trackers={"approach": SimpleNamespace(config=TrackerConfig())}, geometry=None,
        settings={}, pallet_geometry=None, hooks=SimpleNamespace(record_json=lambda v, indent=None: '{"h": 1}'), near=near,
        options=SimpleNamespace(near_tracking=nt, rear_axle_offset_m=0.35, near_drop_frames_from_m=None))
    latest = SimpleNamespace(held={"h": 1}, stamp_s=9.9)
    pred = core.near_protection_prediction(leg(), latest, (0.2, 0.2), (1.0, 0.0, 0.0), 0.6)
    assert asked[0]["camera_xy"] == (0.4, 0.0) and asked[0]["rear_axle_offset_m"] == 0.35
    assert pred["retry_allowed"] is True and pred["lost_injection"] is False
    assert pred["disposition"] == NT.prediction_disposition(-0.01, lost_injection=False, retry_allowed=True)
    inputs = pred["inputs"]
    assert inputs["face_distance_m"] == 0.6 and inputs["retries"] == 0 and inputs["protected"] is False
    assert inputs["held"] == {"h": 1} and inputs["bounds_sha256"] == "B1" and inputs["drive_terms_sha256"] == "D1"
    assert inputs["approach"]["sha256"] == NT.path_digest(leg().poses, leg().directions, leg().curvatures_inv_m)
    near["retries"] = 1  # no retry left: the same shortfall is disposed of differently
    core.options.near_drop_frames_from_m = 0.4
    pred = core.near_protection_prediction(leg(), latest, (0.2, 0.2), (1.0, 0.0, 0.0), 0.6)
    assert pred["retry_allowed"] is False and pred["lost_injection"] is True
    assert pred["disposition"] == NT.prediction_disposition(-0.01, lost_injection=True, retry_allowed=False)


def test_the_runners_traced_redraw_returns_its_dry_runs_trajectories_in_call_order_and_is_one_planner_call():
    import ast

    source = (ROOT / "sim/isaac/run_transport.py").read_text()
    fn = next(n for n in ast.walk(ast.parse(source)) if isinstance(n, ast.FunctionDef)
              and n.name == "redraw_docking_traced")
    rolled = []

    def bicycle_rollout(poses, directions, curvatures, config, start):
        rolled.append((config, tuple(start)))
        return SimpleNamespace(trajectory=f"T{len(rolled)}")

    def redraw_docking(scenario, rear_pose, target, config, *, dry_runs, **k):
        assert k == {"max_yaw_rad": 0.12}
        results = [dry(SimpleNamespace(poses=0, directions=0, curvatures_inv_m=0)) for dry in dry_runs]
        return ("PLANS", [r.trajectory for r in results]), {"record": 1}

    namespace = {"np": np, "bicycle_rollout": bicycle_rollout, "redraw_docking": redraw_docking}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), "run_transport.py", "exec"), namespace)
    plans, record, trajectories = namespace["redraw_docking_traced"](
        "SC", "POSE", "TARGET", "CFG", start_rear=(1.0, 2.0, 0.0), dry_configs=("A", "D"), max_yaw_rad=0.12)
    assert plans == ("PLANS", ["T1", "T2"]) and record == {"record": 1} and trajectories == ["T1", "T2"]
    assert rolled == [("A", (1.0, 2.0, 0.0)), ("D", (1.0, 2.0, 0.0))]
    assert 'redraw_docking_traced, "Planner", "redraw_docking_traced", boundary,' in source


def test_a_correction_before_the_last_point_is_not_screened_by_the_protection_prediction():
    """M-a2: only the last correction point (near_tracking.PREDICT_POINT_M) asks the protection prediction."""
    core, near, calls = make(Geometry(face_m=0.95), standing=True, correction=NT.CorrectionState(0, 1.0, 9.0, 9.0))
    near["corrections"].append({"point_m": 1.0, "standing_s": [9.0], "moved_s": [], "accepted": None})
    core.test_prediction = {"disposition": "refused"}
    tick(core, 10.0)
    assert "prediction" not in [c[0] for c in calls]
    assert near["corrections"][-1]["accepted"] is True


def test_a_terminal_hold_restarts_when_the_truck_moves_before_it_ends():
    core, near, calls = make(Geometry(face_m=0.8), standing=True)
    near["mismatch"] = True
    tick(core, 10.0)
    assert near["terminal_standing_since"] == 10.0
    core.slam["stop_now"] = False  # not standing: no end this tick, the hold starts over
    tick(core, 10.2)
    assert "terminal_standing_since" not in near
    core.slam["stop_now"] = True
    tick(core, 10.0 + NT.TERMINAL_HOLD_S)
    assert near["terminal_standing_since"] == 10.0 + NT.TERMINAL_HOLD_S and "retry" not in [c[0] for c in calls]


def test_a_latched_terminal_stop_decides_no_correction_while_it_stands():
    core, near, calls = make(Geometry(face_m=0.95), standing=True, correction=NT.CorrectionState(0, 1.0, 9.0, 9.0),
                             stop_state=NT.StopState("mismatch", True, 9.5, False))
    near["corrections"].append({"point_m": 1.0, "standing_s": [9.0], "moved_s": [], "accepted": None})
    near["tracker"].latest.stamp_s = 9.95  # an observation that would decide the open point
    tick(core, 10.0)
    assert "near_redraw" not in [c[0] for c in calls] and "install" not in [c[0] for c in calls]
    assert near["corrections"][-1]["accepted"] is None and near["correction"] == NT.CorrectionState(0, 1.0, 9.0, 9.0)
    assert near["stop_state"].reason == "mismatch"


def test_the_held_steering_command_bounds_the_wheel_curvature():
    core, near, _ = make(Geometry(face_m=2.0))
    core.hooks.steering_command = lambda: np.array([0.3, 0.3])  # the measured steering stays at zero
    tick(core, 10.0)
    t_ = np.tan(0.3)
    assert near["ticks"][-1]["kappa_b"] == pytest.approx(max(t_ / (0.8 + t_ * 0.25), t_ / (0.8 - t_ * 0.25)))


def test_the_frame_drop_does_not_latch_before_its_distance():
    core, near, _ = make(Geometry(face_m=1.2), drop_from=1.1)
    tick(core, 10.0)
    assert near.get("drop_latch") is None
    core.options.near_tracking = Geometry(face_m=1.1).module()
    tick(core, 10.1)
    assert near["drop_latch"]["face_distance_m"] == 1.1


def test_each_tick_row_carries_the_path_revision_its_follower_ran_on():
    core, near, calls = make(Geometry(face_m=0.95), standing=True, correction=NT.CorrectionState(0, 1.0, 9.0, 9.0))
    near["corrections"].append({"point_m": 1.0, "standing_s": [9.0], "moved_s": [], "accepted": None})
    near["path_revision"] = 2

    def install(now, plans, target, point, rear):
        near["path_revision"] = 3  # this tick's install comes after the follower ran on revision 2
        return {"revision": 3}

    core.near_install = install
    near["tracker"].latest.stamp_s = 9.95
    tick(core, 10.0)
    assert near["corrections"][-1]["revision"] == 3 and near["ticks"][-1]["path_revision"] == 2


def test_a_pickup_retry_moves_to_observe_only_after_the_release_and_stand():
    """M-a2 (Codex review P2): a release that fails leaves the mission where it was, not already in observe."""
    core, near, _ = make()
    core.state["near_capture"] = {"status": "accepted"}

    def release(now, event):
        raise RuntimeError("release failed")

    core.release_and_stand = release
    del core.start_pickup_retry
    with pytest.raises(RuntimeError, match="release failed"):
        core.start_pickup_retry(12.0, "mismatch", 0.7, 1)
    assert core.ms.phase == "approach" and core.state["transitions"] == []


def test_the_terminal_hold_ends_exactly_at_its_length():
    """M-a2 sweep (boundaries): standing exactly TERMINAL_HOLD_S (less the 1e-9 slack) ends the hold."""
    core, near, calls = make(Geometry(face_m=0.8), standing=True)
    near["mismatch"] = True
    tick(core, 0.0)
    tick(core, NT.TERMINAL_HOLD_S - 1e-9)
    assert ("retry", "mismatch") in calls


# docs/plans/2026-10-28-near-front-range.md R2: a missed handoff is a terminal stop, never a retry ---------------
def test_a_missed_handoff_ends_the_run_after_its_hold_and_is_not_retried():
    core, near, calls = make(Geometry(face_m=0.8), standing=True)  # far enough and no retry yet: still none
    near["handoff_missed"] = True
    tick(core, 10.0)
    with pytest.raises(RuntimeError, match="near_field_handoff_missed"):
        tick(core, 10.0 + NT.TERMINAL_HOLD_S)
    assert "retry" not in [c[0] for c in calls]
    assert core.state["near_tracking"]["outcome"] == "near_field_handoff_missed"


@pytest.mark.parametrize("flags,reason", [
    ({"mismatch": True, "handoff_missed": True}, "mismatch"),
    ({"unbounded": True, "handoff_missed": True}, "unbounded"),
    ({"handoff_missed": True}, "handoff_missed"),
])
def test_the_terminal_reasons_keep_their_order(flags, reason):
    core, near, _ = make(Geometry(face_m=0.8), standing=True, retries=1)
    near.update(flags)
    tick(core, 10.0)
    assert near["stop_state"].reason == reason


def test_a_lost_failure_comes_before_a_missed_handoff_and_a_terminal_before_a_lost_wait():
    core, near, _ = make(Geometry(face_m=0.8), tracker=Tracker(armed=True, lost=True, failed=True), standing=True,
                         retries=1)
    near["handoff_missed"] = True
    tick(core, 10.0)
    assert near["stop_state"].reason == "lost"
    core, near, _ = make(Geometry(face_m=0.8), tracker=Tracker(armed=True, lost=True), standing=True, retries=1)
    near["handoff_missed"] = True
    tick(core, 10.0)
    assert near["stop_state"].reason == "handoff_missed" and near["stop_state"].terminal
