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

    def __init__(self, face_m=2.0, budget_ok=True, entered=False):
        self.face_m, self.budget_ok, self.entered_ = face_m, budget_ok, entered

    def module(self):
        g = self
        ns = SimpleNamespace(**{k: v for k, v in vars(NT).items() if not k.startswith("__")})
        ns.face_min_x_m = lambda held, widths, base: g.face_m
        ns.section_entered = lambda face_x, tip: face_x <= 1.5
        ns.carried_face = lambda held, widths, base: {"face": g.face_m}
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
    near.update(mismatch=True, terminal_standing_since=9.0, protect={"x": 1})
    core.state["near_capture"] = {"status": "accepted"}
    order = []
    core.release_and_stand = lambda now, event: order.append(("release", near["tracker"], event)) or {"jump_m": 0.0}
    del core.start_pickup_retry  # the method itself, not the stand-in
    core.start_pickup_retry(12.0, "mismatch", 0.7, 1)
    retry = core.state["near_tracking"]["retry"]
    assert retry["min_face_distance_m"] == 0.7 and retry["jump_m"] == 0.0 and near["retries"] == 1
    assert order == [("release", None, "release_pickup_retry")]  # the tracker is gone before the release
    assert near["mismatch"] is False and near["protect"] is None and "terminal_standing_since" not in near
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
    asked = []
    hooks = SimpleNamespace(
        grid_world=lambda sc: ("world", sc), correction_grid=lambda first, target, rear: {"occupancy": "GRID"},
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
        core.test_response = ({"approach": leg()}, {"refused": None}, trajs[:1])  # one rollout: outside the box
        plans, entry = core.near_capture_redraw((4.0, 0.0, 0.0), (3.9, 0.0, 0.0), obs, 3, cfg_a, cfg_d, 14.0)
        assert plans is None and entry["record"]["refused"] == "correction_point" and entry["accepted"] is False
