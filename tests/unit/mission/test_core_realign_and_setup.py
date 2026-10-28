"""M-a1 slice 6e: the realignment, the known pallet, the return turnaround's options, the tracker profile
and config record, and the near tracker's mission side, moved from run() into MissionCore (behaviour, state,
port requests and their order, effects up to a refusal -- the plan's M-a1 criterion)."""

import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control.path_tracking import TrackerConfig
from forklift_core.mission.core import MissionCore, apply_tracker_profile_to, record_tracker_configs_in
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


def leg(length=1.0):
    poses = np.array([[0.0, 0.0, 0.0], [length, 0.0, 0.0]])
    return PlanResult(True, "success", poses, np.ones(2, np.int8), np.zeros(2), length, 3)


class Tracker:
    def __init__(self, config=None, path=None):
        self.config = config or TrackerConfig()
        self.path = path or leg()

    def path_arrays(self):
        return self.path.poses, self.path.directions, self.path.curvatures_inv_m


def make(*, slam=None, obstacle=None, state=None, near=None, options=None, hooks=None, phase="observe"):
    calls = []

    def require(ok, reason):
        calls.append(("require", bool(ok), reason))
        if not ok:
            raise RuntimeError(reason)

    base_hooks = dict(require=require, record_json=lambda v, indent=None: json.dumps(v), echo=lambda t: None,
                      snapshot=lambda phase: None, world=lambda: SimpleNamespace(current_time=70.0))
    base_hooks.update(hooks or {})
    trackers = {n: Tracker() for n in ("observe", "approach", "insert", "extract")}
    core = MissionCore(
        MissionState(phase=phase, phase_started=0.0), state={"transitions": [], **(state or {})}, obstacle=obstacle,
        slam=slam, pocket={"check": None, "planning_pickup": Pose2D(5.0, 0.0, 0.0)},
        paths={"observe": leg()}, trackers=trackers, geometry=SimpleNamespace(pallet_depth_m=0.8, pallet_width_m=1.2),
        settings={}, pallet_geometry=None, hooks=SimpleNamespace(**base_hooks), near=near if near is not None else {},
        options=SimpleNamespace(**{"initial_time": 50.0, "near_tracking": NT, "grid_planning": True,
                                   "return_turnaround": True, "tracker_profile": "current", **(options or {})}))
    return core, calls


def test_the_control_correction_is_the_applied_one_or_none():
    assert make()[0].control_correction() == (0.0, 0.0, 0.0)
    slam = {"tracker": SimpleNamespace(applied=None)}
    assert make(slam=slam)[0].control_correction() == (0.0, 0.0, 0.0)
    slam["tracker"].applied = (np.array([0.1, -0.2, 0.03]),)
    assert make(slam=slam)[0].control_correction() == (0.1, -0.2, 0.03)


def test_a_known_pallet_fix_replaces_the_list_with_the_correction_applied_now():
    layer = SimpleNamespace(known=["OLD"])
    slam = {"tracker": SimpleNamespace(applied=(np.array([0.1, 0.0, 0.01]),))}
    core, _ = make(slam=slam, obstacle={"layer": layer, "last_stamp": 12.5})
    core.known_pallet((5.0, 0.5, 0.1), 0.05, "pickup")
    (rect,) = layer.known
    assert rect.rect == (5.0, 0.5, 0.8, 1.2, 0.1) and rect.fix_stamp_s == 12.5 and rect.r_fix_m == 0.05
    assert rect.frame_correction == (0.1, 0.0, 0.01) and rect.rho_m is None
    assert core.state["known_pallets"] == [{"event": "pickup", "time_s": 12.5, "rect": [5.0, 0.5, 0.1],
                                            "r_fix_m": 0.05}]


def test_the_return_turnaround_options_wait_for_the_release_and_check_arcs_on_the_permission():
    assert make(options={"return_turnaround": False})[0].return_turnaround_kwargs() == {}
    assert make(slam={"pending_release": "return_home"})[0].return_turnaround_kwargs() == {}
    asked = []
    layer = SimpleNamespace(snapshot="SNAP", check_turnaround_arc=lambda poses, current_pose: asked.append(
        current_pose) or "occupied")
    core, _ = make(slam={"pending_release": None}, obstacle={"layer": layer})
    kwargs = core.return_turnaround_kwargs()
    assert kwargs["forward_turnaround"] is True
    assert kwargs["turnaround_check"](np.array([[1.0, 2.0, 0.5], [1.5, 2.0, 0.5]])) == "occupied"
    assert asked == [(1.0, 2.0, 0.5)]
    layer.snapshot = None
    assert kwargs["turnaround_check"](np.zeros((2, 3))) == "no_snapshot"
    core, _ = make(obstacle={"layer": layer}, options={"grid_planning": False})
    assert core.return_turnaround_kwargs()["turnaround_check"](np.zeros((2, 3))) is None


def test_the_turnaround_check_and_the_dry_runs_describe_their_settings():
    from forklift_core.mission.core import rollout_callback_spec

    layer = SimpleNamespace(snapshot="SNAP", check_turnaround_arc=lambda poses, current_pose: None)
    core, _ = make(slam={"pending_release": None}, obstacle={"layer": layer, "version": 7})
    assert core.return_turnaround_kwargs()["turnaround_check"].callback_spec == {
        "kind": "turnaround_arc", "grid_planning": True, "obstacle_version": 7}
    spec = rollout_callback_spec(TrackerConfig(cruise_speed_mps=0.2), np.array([1.0, 2.0, 0.5]))
    assert spec["kind"] == "bicycle_rollout" and spec["start"] == [1.0, 2.0, 0.5]
    assert spec["config"]["cruise_speed_mps"] == 0.2
    assert spec != rollout_callback_spec(TrackerConfig(cruise_speed_mps=0.3), np.array([1.0, 2.0, 0.5]))


def realign_state(end=None):
    return {"near_tracking": {"realign": {"installs": [], "end": end}}}


def test_a_realignment_records_every_observe_install_and_refuses_one_too_long():
    obstacle = {"progress": {"since_s": 3.0}}
    slam = {"tracker": SimpleNamespace(mode="holding")}
    core, calls = make(slam=slam, obstacle=obstacle, state=realign_state())
    assert core.realign_active() is True
    core.realign_install("stand")
    (rec,) = core.state["near_tracking"]["realign"]["installs"]
    assert rec["n"] == 0 and rec["time_s"] == 20.0 and rec["reason"] == "stand" and rec["slam_mode"] == "holding"
    assert rec["length_m"] == 1.0 and rec["sha256"] == rec["tracker_sha256"] and obstacle["progress"] == {}
    core.paths["observe"] = leg(NT.REALIGN_MAX_LENGTH_M + 0.5)
    with pytest.raises(RuntimeError, match="near_capture_realign_too_long:6.500"):
        core.realign_install("backoff")
    assert len(core.state["near_tracking"]["realign"]["installs"]) == 2  # recorded before the refusal
    core, calls = make(state=realign_state(end=30.0))
    assert core.realign_active() is False
    core.realign_install("stand")
    assert core.state["near_tracking"]["realign"]["installs"] == [] and calls == []
    assert make()[0].realign_active() is False


def test_a_realignment_starts_by_setting_the_refused_capture_apart_then_releases_and_stands():
    order = []
    core, _ = make(slam={"tracker": SimpleNamespace(mode="holding")}, state={"near_tracking": {},
                                                                              "near_capture": {"status": "refused"}})
    core.release_and_stand = lambda now, event: order.append(("release", event, dict(core.state["near_tracking"]))) \
        or {"jump_m": 0.02}
    core.realign_install = lambda reason: order.append(("install", reason))
    core.start_near_realign(15.0, 3)
    nt = core.state["near_tracking"]
    assert nt["realigns"] == 1 and nt["realign"] == {"time_s": 15.0, "refused_attempt": 3, "installs": [],
                                                      "end": None, "jump_m": 0.02}
    assert core.state["near_capture_refused"] == [{"status": "refused", "refused_attempt": 3}]
    assert "near_capture" not in core.state and "near_capture_history" not in core.state
    assert [o[:2] for o in order] == [("release", "release_near_realign"), ("install", "stand")]
    assert "realign" in order[0][2]  # the record exists before the release fills it
    assert core.near["stand_record"] == "realign" and core.ms.phase == "observe"


def test_the_tracker_profile_and_its_record_are_shared_by_the_runner_and_the_core():
    core, _ = make(options={"tracker_profile": "20260921"})
    core.apply_tracker_profile()
    assert core.trackers["observe"].config.yaw_tolerance_rad == 0.03
    assert core.trackers["approach"].config.position_tolerance_m == 0.008
    assert core.trackers["approach"].config.cusp_position_tolerance_m is None
    core.record_tracker_configs()
    assert core.state["tracker_configs"]["observe"]["yaw_tolerance_rad"] == 0.03
    trackers = {"observe": Tracker(TrackerConfig(yaw_tolerance_rad=0.05))}
    apply_tracker_profile_to(trackers, "current")
    assert trackers["observe"].config.yaw_tolerance_rad == 0.05
    state = {"tracker_configs": {"old": {}}}
    record_tracker_configs_in(state, trackers)
    assert set(state["tracker_configs"]) == {"old", "observe"}  # updated, not replaced


class Recorder:
    def __init__(self):
        self.calls = []


def near_build(*, start=True, mode="holding", problems=None, holds=2):
    rec = Recorder()
    start_ = None if not start else {"observation": SimpleNamespace(), "pose": (1.0, 0.0, 0.0), "aligned_s": 9.0,
                                     "capture_stamp_s": 9.1, "attempt": 4}
    near = {"start": start_, "terms": SimpleNamespace(condition={"tracker_configs": {"x": 1}}),
            "history": SimpleNamespace(clear=lambda: rec.calls.append(("history_clear",))), "events": [],
            "terminal_standing_since": 3.0}
    service = SimpleNamespace(config=SimpleNamespace(handoff_start_front_x_m=1.47),
                              frame_geo=SimpleNamespace(fork_tip_x_m=0.9, carriage_limit_m=0.35))
    hooks = {
        "near_tracker_config": lambda: rec.calls.append(("config", dict(near))) or service,
        "near_tracker_create": lambda svc, st, ep: rec.calls.append(("create", svc, st, ep,
                                                                      near.get("injection_closed"))) or "TRACKER",
        "near_fork_geometry": lambda: rec.calls.append(("forks",)) or (((0, 0.4, -0.25, -0.15), (0, 0.4, 0.15, 0.3)),
                                                                       ((0.35, 0.2), (0.35, -0.2))),
    }
    nt = SimpleNamespace(**{k: v for k, v in vars(NT).items() if not k.startswith("__")})
    nt.tracker_config_problems = lambda expected, observed: rec.calls.append(("check", expected, observed)) or list(
        problems or [])
    slam = {"tracker": SimpleNamespace(mode=mode), "holds": [{"event": "hold"}] * holds}
    core, calls = make(slam=slam, near=near, hooks=hooks, options={"near_tracking": nt},
                       state={"near_tracking": {}, "tracker_configs": {"observe": {"a": 1}}})
    return core, near, rec


def test_the_near_tracker_is_built_after_its_checks_in_the_closures_order(monkeypatch):
    core, near, rec = near_build()
    monkeypatch.setattr("forklift_core.mission.core.asdict", lambda v: {"obs": 1})
    core.build_near_tracker(11.0)
    assert [c[0] for c in rec.calls] == ["config", "check", "create", "forks", "history_clear"]
    assert rec.calls[1][1:] == ({"x": 1}, {"observe": {"a": 1}})
    _, svc, st, episode, closed = rec.calls[2]
    assert episode == 2 and closed is True and st is near["start"]  # the injection latch before the tracker
    assert "injection_closed" not in rec.calls[0][1]
    assert near["tracker"] == "TRACKER" and near["episode"] == 2 and near["blade_y_max_m"] == 0.3
    assert near["corners"] == ((0.35, 0.2), (0.35, -0.2)) and near["carriage_limit_m"] == 0.35
    assert near["first_target"] == Pose2D(5.0, 0.0, 0.0) and near["path_revision"] == 0
    assert near["stop_events"] == [] and "terminal_standing_since" not in near
    start = core.state["near_tracking"]["start"]
    assert start["gate_front_x_m"] == 1.47 and start["episode"] == 2 and start["observation"] == {"obs": 1}
    assert set(start["paths0"]) == {"approach", "insert", "extract"}
    assert core.state["near_tracking"]["starts"] == [start]
    assert near["events"] == [{"time_s": 11.0, "event": "tracker_built", "episode": 2}]


def test_a_near_tracker_without_a_capture_a_hold_or_with_unmatched_followers_is_refused_before_it_exists():
    core, near, rec = near_build(start=False)
    with pytest.raises(RuntimeError, match="near_field_without_capture"):
        core.build_near_tracker(11.0)
    assert rec.calls == []
    core, near, rec = near_build(mode="tracking")
    with pytest.raises(RuntimeError, match="near_field_without_hold"):
        core.build_near_tracker(11.0)
    assert rec.calls == []
    core, near, rec = near_build(problems=["observe: yaw_tolerance_rad"])
    with pytest.raises(RuntimeError, match="near_field_contract:observe: yaw_tolerance_rad"):
        core.build_near_tracker(11.0)
    assert [c[0] for c in rec.calls] == ["config", "check"]  # the service read, then the refusal
    assert core.state["near_tracking"]["tracker_config_problems"] == ["observe: yaw_tolerance_rad"]
    assert "injection_closed" not in near and "tracker" not in near


def test_a_new_tracker_starts_with_clean_latches_and_keeps_the_earlier_episodes_stop_records(monkeypatch):
    core, near, _ = near_build()
    monkeypatch.setattr("forklift_core.mission.core.asdict", lambda v: {"obs": 1})
    earlier = {"start_s": 3.0, "episode": 1, "reason": "mismatch"}
    near.update(mismatch=True, unbounded=True, handoff_missed=True, stop_events=[earlier])  # a closed episode's
    core.build_near_tracker(11.0)
    assert near["mismatch"] is False and near["unbounded"] is False and near["handoff_missed"] is False
    assert near["stop_events"] == [earlier]


def test_a_realignment_install_of_exactly_the_maximum_length_is_kept():
    """M-a2 sweep (boundaries): REALIGN_MAX_LENGTH_M itself (with its 1e-9 slack) is allowed."""
    slam = {"tracker": SimpleNamespace(mode="holding")}
    core, calls = make(slam=slam, obstacle={"progress": {}}, state=realign_state())
    core.paths["observe"] = leg(NT.REALIGN_MAX_LENGTH_M + 1e-9)
    try:
        core.realign_install("backoff")
    except RuntimeError as exc:
        pytest.fail(f"an install of exactly the maximum was refused: {exc}")
    assert len(core.state["near_tracking"]["realign"]["installs"]) == 1


def test_a_realignment_whose_stand_install_refuses_has_not_moved_to_observe():
    """M-a2 sweep (order, Codex round 4): the stand is labelled first, the install may refuse, the phase moves last."""
    core, _ = make(slam={"tracker": SimpleNamespace(mode="holding")}, state={"near_tracking": {},
                                                                              "near_capture": {"status": "refused"}},
                   phase="approach")
    core.release_and_stand = lambda now, event: {"jump_m": 0.0}

    def refuse(reason):
        raise RuntimeError("near_capture_realign_too_long:9.000")

    core.realign_install = refuse
    with pytest.raises(RuntimeError, match="realign_too_long"):
        core.start_near_realign(15.0, 3)
    assert core.near["stand_record"] == "realign" and core.ms.phase == "approach"
