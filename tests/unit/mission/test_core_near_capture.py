"""M-a1 slice 4b-v: the near leg and the near capture's re-drawn straight, moved from the runner (behaviour,
state, port requests and their order, effects up to a refusal -- the plan's M-a1 criterion)."""

from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control.path_tracking import TrackerConfig
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.planning import PlanResult

REAR = np.array([1.0, 2.0, 0.0])
BASE = np.array([1.5, 2.0, 0.0])


def leg(x0=0.0, length=2.0):
    return PlanResult(True, "success", np.array([[x0, 2.0, 0.0], [x0 + length, 2.0, 0.0]]), np.ones(2, np.int8),
                      np.zeros(2), length, 0)


@dataclass(frozen=True)
class Dry:
    status: str = "arrived"
    position_error_m: float = 0.004
    yaw_error_rad: float = 0.01


class World:
    def __init__(self, *, near_capture="pending", near=None, straight=leg(1.0, 2.0), dry=Dry(), redraw="off",
                 redraw_plans=None, realign=False, realigns=0, slow=None, pocket=False, prefix=leg(3.0, 1.0)):
        self.calls = []
        log_ = self.log

        def require(ok, reason):
            log_("require", bool(ok), reason)
            if not ok:
                raise RuntimeError(reason)

        self.hooks = SimpleNamespace(
            require=require, path_record=lambda p: {"length_m": p.length_m},
            final_straight_prefix=lambda approach, length: log_("final_straight_prefix") or prefix,
            realign_active=lambda: realign, realign_install=lambda reason: log_("realign_install", reason),
            straight_from_pose=lambda *a, **k: log_("straight_from_pose", *a, **k) or (straight, {"lateral_m": 0.01}),
            bicycle_rollout=lambda *a, **k: log_("bicycle_rollout", a[3].cruise_speed_mps) or dry,
            near_capture_redraw=lambda *a: log_("near_capture_redraw", *a) or (redraw_plans, {"entry": 1}),
            start_near_realign=lambda *a: log_("start_near_realign", *a),
            record_tracker_configs=lambda: log_("record_tracker_configs"),
            build_pocket_check=lambda *a: log_("build_pocket_check", *a),
            build_near_tracker=lambda *a: log_("build_near_tracker", *a),
            set_inset_status=lambda s: log_("set_inset_status", s), write_paths=lambda: log_("write_paths"),
            show_path=lambda p, label, color: log_("show_path", label), echo=lambda *a: None,
            record_json=lambda v: str(v), snapshot=lambda *a: None,
        )
        self.ms = MissionState(phase="observe", phase_started=0.0)
        self.state = {"transitions": [], "observe_replan_leg": {"serial": 2, "start": 0}, "observe_replans": [{}],
                      "paths": {}, "near_tracking": {"realigns": realigns, "realign": {}},
                      "perception": {"perception_pickup_estimate_m": {"x_m": 4.0}, "perception_error": {"p": 1}}}
        if near_capture is not None:
            self.state["near_capture"] = {"status": near_capture}
        config = TrackerConfig(cruise_speed_mps=0.4)
        trackers = {name: SimpleNamespace(config=config, path_arrays=lambda: (np.zeros((2, 3)), np.ones(2), np.zeros(2)))
                    for name in ("observe", "approach", "insert", "extract")}
        paths = {name: leg() for name in ("observe", "approach", "insert", "extract")}
        near_tracking = SimpleNamespace(REALIGN_MIN_LEFT_M=1.5, path_digest=lambda *a: "SHA")
        self.core = MissionCore(
            self.ms, state=self.state, obstacle=None, slam=None, pocket={"check": None}, paths=paths,
            trackers=trackers, geometry=SimpleNamespace(alignment_straight_m=1.0), settings={}, pallet_geometry=None,
            hooks=self.hooks, near=near,
            options=SimpleNamespace(speeds={"approach": 0.4}, near_tracking=near_tracking,
                                    near_capture_box_yaw_rad=0.05, approach_straight_speed_mps=slow,
                                    near_capture_redraw=redraw, pocket_check=pocket))
        # near_capture_redraw moved into MissionCore (slice 6d): this test's stand-in goes on the instance.
        for name_ in ('near_capture_redraw', 'start_near_realign', 'build_near_tracker', 'record_tracker_configs',
                      'known_pallet', 'realign_install', 'apply_tracker_profile'):
            if hasattr(self.hooks, name_):
                setattr(self.core, name_, getattr(self.hooks, name_))

    def log(self, name, *args, **kwargs):
        self.calls.append((name, args, kwargs))

    def names(self):
        return [c[0] for c in self.calls if c[0] != "require"]

    def run(self, near_leg=None):
        self.core.near_leg_or_recapture(30.0, REAR, BASE, 0.0, "OBS", 3, near_leg)


def test_the_first_far_observation_drives_the_near_leg_with_its_own_replan_budget():
    w = World(near_capture="pending")  # observe_succeeded opened it just before
    near_leg = leg(1.0, 1.5)
    observe_config = w.core.trackers["observe"].config
    w.run(near_leg)
    assert w.names() == ["set_inset_status", "realign_install"] and w.calls[-1][1] == ("near_leg",)
    assert w.state["observe_replan_leg"] == {"serial": 3, "start": 1}
    assert w.core.paths["observe"] is near_leg and w.core.trackers["observe"].config is observe_config
    assert w.state["observation_waypoint_selected"] == [2.5, 2.0, 0.0] and w.ms.phase_started == 30.0
    assert w.state["near_capture"]["waypoint"] == [2.5, 2.0, 0.0]
    assert w.ms.phase == "observe"  # still observing: the near capture follows


def test_without_a_near_capture_the_observation_goes_straight_to_the_approach():
    w = World(near_capture=None)
    w.run()
    assert w.ms.phase == "approach" and w.names() == []


def test_the_near_capture_redraws_the_straight_from_here_and_installs_it_after_the_dry_run():
    w = World(near={"tracker": None}, pocket=True)
    w.run()
    _, args, kwargs = [c for c in w.calls if c[0] == "straight_from_pose"][0]
    assert (args[0].x_m, args[1].x_m, args[2].x_m) == (1.0, 4.0, 2.0)
    assert kwargs == {"max_lateral_m": 0.05, "max_yaw_rad": 0.05, "min_length_m": 0.3}
    nc = w.state["near_capture"]
    assert nc["offsets_to_new_line"] == {"lateral_m": 0.01} and nc["box"] == {"lateral_m": 0.05, "yaw_rad": 0.05,
                                                                              "length_m": 0.3}
    assert nc["dry_run"] == {"status": "arrived", "position_error_m": 0.004, "yaw_error_rad": 0.01}
    assert nc["status"] == "done" and nc["near_attempt"] == 3 and nc["near_error"] == {"p": 1}
    assert w.core.paths["approach"].length_m == 2.0 and w.state["paths"]["approach"] == {"length_m": 2.0}
    assert w.core.trackers["approach"].config.cruise_speed_mps == 0.4
    assert [n for n in w.names() if n != "final_straight_prefix"] == [
        "straight_from_pose", "bicycle_rollout", "write_paths", "show_path", "build_pocket_check", "build_near_tracker"]
    (_, pocket_args, _), = [c for c in w.calls if c[0] == "build_pocket_check"]
    assert pocket_args[0] == 0.0 and pocket_args[1] is REAR and pocket_args[2] is BASE
    assert w.ms.phase == "approach" and w.calls[-1] == ("build_near_tracker", (30.0,), {})


@pytest.mark.parametrize("dry, reason", [(Dry(status="failed"), "near_capture_misaligned:dry_run:"),
                                         (Dry(position_error_m=0.007), "near_capture_misaligned:dry_run:"),
                                         (Dry(yaw_error_rad=0.016), "near_capture_misaligned:dry_run:")])
def test_a_dry_run_out_of_its_tolerance_refuses_the_straight(dry, reason):
    w = World(dry=dry)
    with pytest.raises(RuntimeError, match=reason):
        w.run()
    assert w.ms.phase == "observe" and w.state["near_capture"]["status"] == "pending"


def test_no_straight_in_the_box_refuses_and_no_prefix_refuses_first():
    w = World(straight=None)
    with pytest.raises(RuntimeError, match=r"near_capture_misaligned:\{'lateral_m': 0.01\}"):
        w.run()
    w = World(prefix=None)
    with pytest.raises(RuntimeError, match="near_capture_no_final_straight"):
        w.run()


def test_a_refused_straight_is_redrawn_under_s4c3_and_its_install_recorded():
    plans = {"approach": leg(1.0, 2.5), "insert": leg(3.5, 0.3), "extract": leg(3.8, -0.3)}
    w = World(near={"tracker": None}, dry=Dry(status="failed"), redraw="on", redraw_plans=plans)
    w.run()
    (_, args, _), = [c for c in w.calls if c[0] == "near_capture_redraw"]
    assert args[0] is REAR and args[1:4] == ((1.5, 2.0, 0.0), "OBS", 3) and args[6] == 30.0
    assert args[4].cruise_speed_mps == 0.4 and args[5] is args[4]  # no slow cruise: driven = acceptance
    assert w.core.paths["approach"] is plans["approach"] and w.core.paths["insert"] is plans["insert"]
    assert w.state["near_capture"]["recoveries"] == [{"entry": 1, "installed": {
        name: {"sha256": "SHA", "tracker_sha256": "SHA", "poses": plans[name].poses.tolist(),
               "directions": plans[name].directions.tolist(),
               "curvatures_inv_m": plans[name].curvatures_inv_m.tolist()}
        for name in ("approach", "insert", "extract")}}]
    assert w.ms.phase == "approach"


def test_a_refusal_with_near_tracking_realigns_once_instead_of_ending():
    w = World(near={"tracker": None}, straight=None)
    w.run()
    assert w.calls[-1] == ("start_near_realign", (30.0, 3), {}) and w.ms.phase == "observe"
    w = World(near={"tracker": None}, straight=None, realigns=1)
    with pytest.raises(RuntimeError, match="near_capture_misaligned"):
        w.run()


def test_during_a_realignment_the_recapture_keeps_a_long_enough_straight_and_ends_the_realignment():
    w = World(near={"tracker": None}, realign=True, realigns=1, straight=leg(1.0, 1.4))
    with pytest.raises(RuntimeError, match="near_capture_realign_short:1.400"):
        w.run()
    w = World(near={"tracker": None}, realign=True, realigns=1, straight=leg(1.0, 1.6))
    w.run()
    assert w.state["near_tracking"]["realign"]["end"] == {"reason": "recaptured", "attempt": 3, "time_s": 30.0,
                                                          "length_left_m": 1.6, "observe_tracker_sha256": "SHA"}


def test_a_slow_approach_cruise_keeps_the_baseline_acceptance_and_records_the_driven_dry_run():
    w = World(slow=0.2)
    w.run()
    assert [c[1][0] for c in w.calls if c[0] == "bicycle_rollout"] == [0.4, 0.2]
    assert w.core.trackers["approach"].config.cruise_speed_mps == 0.2
    assert "dry_run_driven" in w.state["near_capture"] and "record_tracker_configs" in w.names()
