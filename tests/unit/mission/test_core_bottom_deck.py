"""Plan 2026-10-19 task 4: the bottom-deck side legs in the mission core (design rounds 1-5 with Codex): the side
legs before any mission plan or near capture, the side capture and its retakes, the two-face policy, the releases
before every side plan, the recoveries that would replace a side leg and the refusal that ends the mission."""

import json
import math
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control.path_tracking import TrackerConfig
from forklift_core.mission import bottom_deck_legs as L
from forklift_core.mission import core as core_mod
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.perception.bottom_deck_evidence import BottomDeckParams
from forklift_core.planning import PlanResult, Pose2D

REAR = np.array([0.0, 0.0, 0.0])


@dataclass(frozen=True)
class Config:
    clearance_m: float = 0.3
PICKUP = Pose2D(3.0, 0.0, 0.0)


def path_to(goal, start=(0.0, 0.0, 0.0)):
    poses = np.array([list(start), list(goal)], dtype=float)
    return PlanResult(True, "success", poses, np.ones(2, np.int8), np.zeros(2), 1.0, 3)


def failed(status="no_path"):
    return PlanResult(False, status, np.empty((0, 3)), np.empty(0, np.int8), np.empty(0), 0.0, 0)


class CaptureFailure(Exception):
    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


class World:
    """A real MissionCore with the runner's ports faked: planners answer by goal, the capture and the judge from
    scripts, every port call logged."""

    def __init__(self, *, verdicts=(), arrivals=None, slam=False, holding=False, obstacle=True, unreachable=(),
                 mission_ok=True, same_path=False, retreat_ok=True):
        self.calls, self.verdicts, self.arrivals = [], list(verdicts), list(arrivals or [])
        self.unreachable, self.same, self.retreat_ok = set(unreachable), same_path, retreat_ok
        self.runner_rear = REAR
        log = self.log

        def plan_observation_leg(world, target, config, **kw):
            goal = (target.x_m, target.y_m, target.yaw_rad)
            start = (kw["start_rear"].x_m, kw["start_rear"].y_m, kw["start_rear"].yaw_rad)
            log("plan_observation_leg", goal=goal, start=start, clearance=config.clearance_m,
                grid_rear=tuple(self.runner_rear))
            face = self.face_of(goal)
            if face is None and not self.retreat_ok:
                return failed("invalid_goal")  # the retreat point collides
            return failed() if face in self.unreachable else path_to(goal, start)

        def capture(max_attempts):
            log("capture")
            v = self.verdicts.pop(0)
            if v == "capture_failure":
                raise CaptureFailure("not_still")
            return SimpleNamespace(depth_m=np.zeros((2, 2)), stamp_ns=5, intrinsics=SimpleNamespace(
                width=2, height=2, fx=1.0, fy=1.0, cx=1.0, cy=1.0, frame_id="cam"), base_from_optical=SimpleNamespace(
                translation_m=np.zeros(3), rotation=np.eye(3)), verdict=v), None, 1

        stages = ["approach", "insert", "extract", "transport", "withdraw"]
        self.mission = SimpleNamespace(success=mission_ok, status="success" if mission_ok else "approach:no_path",
                                       **{n: path_to((5.0, 0.0, 0.0)) for n in stages})
        self.hooks = SimpleNamespace(
            require=self.require, wall=lambda: 100.0, path_record=lambda p: {"status": p.status},
            write_paths=lambda: log("write_paths"), grid_world=lambda sc: ("world", sc),
            grid_kwargs=lambda *a: log("grid_kwargs", *a) or {}, plan_observation_leg=plan_observation_leg,
            plan_transport=lambda *a, **k: log("plan_transport", start=k["start_rear"]) or self.mission,
            final_straight_prefix=lambda approach, length: path_to((4.0, 0.0, 0.0)),
            same_path=lambda new, cur, rem: self.same, set_rear=lambda r: setattr(self, "runner_rear", tuple(r)),
            set_inset_status=lambda text: log("inset", text), show_path=lambda *a: None, export_scene=lambda: None,
            echo=lambda text: None, record_json=lambda v, indent=None: "J", snapshot=lambda *a: None,
            perception_capture=lambda: SimpleNamespace(capture=capture), now_s=lambda: 50.0,
            slam_rear=lambda t: np.asarray(self.slam_pose, float), slam_odom_base=lambda: None,
            truth_rear=lambda: (0.0, 0.0, 0.0), save_npz=lambda name, **a: log("save_npz", name, a["verdict"]),
            loaded=lambda: False, straight_from_pose=lambda *a, **k: None, bicycle_rollout=lambda *a, **k: None,
            build_pocket_check=lambda *a, **k: None, truth_velocity=lambda: np.asarray(self.velocity, float),
            truth_yaw_rate=lambda: 0.0,
        )
        self.velocity = (0.0, 0.0, 0.0)
        self.slam_pose = (0.0, 0.0, 0.0)
        slam_ = None
        if slam:
            slam_ = {"tracker": SimpleNamespace(mode="holding" if holding else "tracking",
                                                applied=((0.0, 0.0, 0.0), 42.0, 7)), "holds": []}
        self.obstacle = {"layer": SimpleNamespace(known_enabled=False, refresh=lambda *a, **k: log("refresh")),
                         "version": 0, "applied": (0.0, 0.0, 0.0)} if obstacle else None
        self.ms = MissionState(phase="observe", phase_started=0.0)
        self.state = {"transitions": [], "planning_wall_s": 0.0, "observe_replans": [],
                      "observe_replan_leg": {"serial": 0, "start": 0},
                      "perception": {"perception_pickup_estimate_m": {}, "perception_error": {}}}
        config = TrackerConfig(cruise_speed_mps=0.3, max_curvature_inv_m=0.45, max_acceleration_mps2=0.5)
        from forklift_core.control import RearAxlePathTracker
        trackers = {"observe": RearAxlePathTracker(np.array([[0, 0, 0], [1, 0, 0]], float), np.ones(2),
                                                   np.zeros(2), config)}
        options = SimpleNamespace(
            bottom_deck_check="on", bottom_deck_params=BottomDeckParams(), bottom_deck_spec="SPEC",
            camera_mount_x_m=0.619, rear_axle_offset_m=0.37, planner_config=Config(),
            travel_config=None, scenario="SC", pickup_bounds="B", return_to_pose="HOME", mission_stages=stages,
            speeds={n: 0.3 for n in stages + ["observe", "return_home"]}, cusp_heading_hold_rad=0.08,
            cusp_heading_hold_phases=("observe",), perception_max_attempts=3,
            adapter=SimpleNamespace(CaptureFailure=CaptureFailure), near_tracking=None, backoff_m=0.3,
            backoff_speed_mps=0.1, backoff_per_leg=1, escape_per_leg=1, observation_waypoints=())
        settings = {"tracker_curvature_inv_m": 0.45, "drive_acceleration_mps2": 0.5}
        self.core = MissionCore(self.ms, state=self.state, obstacle=self.obstacle, slam=slam_,
                                pocket={"check": None, "planning_pickup": PICKUP}, paths={"observe": None},
                                trackers=trackers, geometry=SimpleNamespace(pallet_depth_m=0.8,
                                                                            alignment_straight_m=0.8),
                                settings=settings, pallet_geometry=SimpleNamespace(geometry_version="epal6"),
                                hooks=self.hooks, near=None, options=options)
        for name_ in ("apply_tracker_profile", "record_tracker_configs", "realign_install"):
            setattr(self.core, name_, lambda *a: None)
        self.core.release_slam = self.release_slam

    # -- the fakes ----------------------------------------------------------------------------------------------
    def log(self, name, *args, **kwargs):
        self.calls.append((name, args, kwargs))

    def require(self, ok, reason):
        if not ok:
            raise RuntimeError(reason)

    def release_slam(self, odom_base):
        self.log("release_slam")
        self.core.slam["tracker"].mode = "tracking"
        self.slam_pose = (self.slam_pose[0] + 0.2, self.slam_pose[1], self.slam_pose[2])  # the correction moves us
        return 0.2, 0.0

    def face_of(self, goal):
        deck = self.state.get("bottom_deck") or {}
        for f, st in (deck.get("stands") or {}).items():
            if np.hypot(st["rear"][0] - goal[0], st["rear"][1] - goal[1]) < 0.1:
                return int(f)
        tg = deck.get("target")
        if tg is not None:
            for f in (1, -1):
                s = L.stand(tg["centre_xy"], tg["insertion_yaw_rad"], tg["axis"], f, mount_x_m=0.619,
                            rear_axle_offset_m=0.37)
                if np.hypot(s.rear[0] - goal[0], s.rear[1] - goal[1]) < 0.1:
                    return f
        return None

    def names(self, *keep):
        return [c[0] for c in self.calls if not keep or c[0] in keep]

    # -- driving ------------------------------------------------------------------------------------------------
    def begin(self, attempt=None):
        return self.core._bottom_deck_begin(10.0, REAR, np.array([0.37, 0.0, 0.0]), 0.0, attempt or {}, 2,
                                            "OBS", PICKUP)

    def arrive(self, t=20.0):
        return self.core.bottom_deck_capture(t, np.asarray(self.core.paths["observe"].poses[-1]), 600)

    @property
    def deck(self):
        return self.state["bottom_deck"]


@pytest.fixture(autouse=True)
def scripted_judge(monkeypatch):
    """The judge answers from the scripted capture (its ``verdict``), arrival from the world's list."""
    worlds = []

    def arrival_ok(scene, target, *, face, axis):
        w = worlds[-1]
        return w.arrivals.pop(0) if w.arrivals else True

    fake = SimpleNamespace(
        arrival_ok=arrival_ok, camera_xyz=lambda scene: np.zeros(3), stand_angle_deg=lambda cam, tg, face: 35.0,
        evaluate_side=lambda scene, target, *, face, params: SimpleNamespace(verdict=scene.verdict, reason="r",
                                                                            gaps=()))
    monkeypatch.setattr(core_mod, "bottom_deck_evidence", fake)
    original = World.__init__

    def init(self, *a, **k):
        original(self, *a, **k)
        worlds.append(self)

    monkeypatch.setattr(World, "__init__", init)


# --- the start ---------------------------------------------------------------------------------------------------

def test_the_far_observation_starts_the_side_legs_without_a_mission_plan_or_a_near_capture():
    w = World()
    out = w.begin()
    assert (out.stop, out.handled, out.near_leg) == (False, True, None)
    assert "plan_transport" not in w.names() and "near_capture" not in w.state
    assert w.obstacle["pickup_estimate"] == PICKUP  # the side plans see the recognised pallet (round 1 P2)
    assert len(w.names("plan_observation_leg")) == 2  # both stands reachable before the first leg starts
    first = w.deck["order"][0]
    assert w.deck["leg"] == {"kind": "stand", "face": first, "captures": 0, "time_s": 10.0}
    assert np.allclose(w.core.paths["observe"].poses[-1], w.deck["stands"][str(first)]["rear"])
    assert w.state["observe_replan_leg"]["serial"] == 1 and w.core.bottom_deck_leg_active()
    assert json.loads(json.dumps(w.state["bottom_deck"])) == w.state["bottom_deck"]


def test_an_unreachable_stand_refuses_before_any_side_leg():
    w = World(unreachable={-1})
    out = w.begin()
    assert out.stop is True and w.ms.phase == "bottom_deck_refused"
    assert w.deck["refused"] == {"reason": "bottom_deck_unobserved", "time_s": 10.0, "stage": "side",
                                 "detail": "stand_unreachable"}
    assert w.state["mission_result"] == "bottom_deck_refused" and not w.core.bottom_deck_leg_active()


def test_the_far_recognitions_used_candidate_sets_the_target_t11_y_back_face_first():
    w = World()
    w.core.pallet_geometry = SimpleNamespace(geometry_version="t11_06")
    used = {"candidate_id": "c7", "hypothesis": "t11_06:y", "axis": "y", "support_z_m": 0.012}
    w.begin({"recognition": {"used": used, "selected_track": 3}})
    tg = w.deck["target"]
    assert (tg["shape"], tg["axis"], tg["support_z_m"], tg["candidate_id"], tg["track_id"]) == (
        "t11_06", "y", 0.012, "c7", "3")
    assert tg["model_yaw_rad"] == pytest.approx(-math.pi / 2) and w.deck["order"] == [1, -1]


@pytest.mark.parametrize("used, detail, reason", [
    ({}, "far_record_incomplete", "bottom_deck_unobserved"),
    ({"candidate_id": "c", "hypothesis": "epal6:x", "axis": "x"}, "far_record_incomplete", "bottom_deck_unobserved"),
    ({"candidate_id": "c", "hypothesis": "t11_06:x", "axis": "x", "support_z_m": 0.0}, "shape_mismatch",
     "bottom_deck_target_mismatch"),
    ({"candidate_id": "c", "hypothesis": "parametric:x", "axis": "x", "support_z_m": 0.0}, "shape_mismatch",
     "bottom_deck_target_mismatch"),
])
def test_a_far_record_without_what_the_side_evidence_needs_is_refused(used, detail, reason):
    w = World()
    out = w.begin({"recognition": {"used": used}})
    assert out.stop is True and w.deck["refused"]["reason"] == reason and w.deck["refused"]["detail"] == detail
    assert "plan_observation_leg" not in w.names()


# --- the captures --------------------------------------------------------------------------------------------------

def test_two_present_faces_plan_the_mission_from_the_second_stand_and_record_present():
    w = World(verdicts=["present", "present"])
    w.begin()
    first, second = w.deck["order"]
    out = w.arrive()
    assert (out.stop, w.deck["leg"]["face"], w.deck["leg"]["kind"]) == (False, second, "stand")
    stand2 = tuple(w.core.paths["observe"].poses[-1])
    out = w.arrive(30.0)
    assert out.stop is False and w.deck["leg"] is None and w.deck["status"] == "done"
    plan = [c for c in w.calls if c[0] == "plan_transport"]
    assert len(plan) == 1 and plan[0][2]["start"] == Pose2D(*stand2)
    rec = w.deck["record"]
    assert rec["verdict"] == "present" and [f["face"] for f in rec["faces"]] == [first, second]
    assert w.ms.phase == "approach"  # no SLAM: the near capture is skipped, the approach follows
    assert w.names("save_npz") == ["save_npz", "save_npz"]
    assert [a["npz"] for a in w.deck["attempts"]] == ["bottom_deck/side_01.npz", "bottom_deck/side_02.npz"]


def test_under_slam_the_near_capture_is_set_up_only_after_the_side_legs():
    w = World(verdicts=["present", "present"], slam=True)
    w.begin()
    assert "near_capture" not in w.state
    w.arrive()
    w.arrive(30.0)
    assert w.state["near_capture"]["status"] == "pending" and w.ms.phase == "observe"
    assert not w.core.bottom_deck_leg_active()  # the observe leg is the near leg now
    assert w.deck["record"]["correction_s"] == 42.0


def test_a_first_face_absent_ends_the_mission_without_the_second_face():
    w = World(verdicts=["absent"])
    w.begin()
    plans_before = len(w.names("plan_observation_leg"))
    out = w.arrive()
    assert out.stop is True and w.ms.phase == "bottom_deck_refused"
    assert w.deck["refused"]["reason"] == "no_bottom_deck" and w.deck["record"]["verdict"] == "absent"
    assert len(w.names("plan_observation_leg")) == plans_before and len(w.deck["faces"]) == 1


def test_an_unobserved_capture_retreats_and_arrives_afresh_three_times_at_most():
    w = World(verdicts=["unobserved", "capture_failure", "unobserved", "present"], arrivals=[True, False])
    w.begin()
    first, second = w.deck["order"]
    out = w.arrive()
    stand = w.deck["stands"][str(first)]["rear"]
    assert w.deck["leg"]["kind"] == "retreat"  # a planned leg to the point behind the stand (round 1 P1)
    assert tuple(w.core.paths["observe"].poses[-1]) == pytest.approx(L.retreat_pose(stand))
    w.core.bottom_deck_capture(21.0, np.asarray(w.core.paths["observe"].poses[-1]), 601)  # the retreat's end
    assert w.deck["leg"] == {"kind": "stand", "face": first, "captures": 1, "time_s": 21.0}
    w.arrive(22.0)  # a capture failure counts
    w.core.bottom_deck_capture(23.0, np.asarray(w.core.paths["observe"].poses[-1]), 602)
    w.arrive(24.0)  # the arrival check fails: the third capture, the face ends unobserved
    assert w.deck["faces"][0] == {"face": first, "verdict": "unobserved", "reason": "arrival", "captures": 3}
    assert w.deck["leg"]["face"] == second  # the second face is still seen (to record an absent)
    out = w.arrive(25.0)
    assert out.stop is True and w.deck["refused"]["reason"] == "bottom_deck_unobserved"
    assert [a["verdict"] for a in w.deck["attempts"]] == ["unobserved", "unobserved", "unobserved", "present"]
    assert w.deck["attempts"][1]["reason"] == "capture_failed:not_still"


def test_present_on_one_face_twice_is_not_two_faces():
    w = World(verdicts=["present"])
    w.begin()
    w.deck["order"] = [w.deck["order"][0]]  # only one face left to see
    out = w.arrive()
    assert out.stop is True and w.deck["record"]["verdict"] == "unobserved"


# --- the releases and the recoveries ---------------------------------------------------------------------------------

def test_a_held_pose_is_released_before_the_next_side_plan_and_the_target_moves_with_it():
    w = World(verdicts=["present", "present"], slam=True)
    w.begin()
    centre_before = list(w.deck["target"]["centre_xy"])
    w.core.slam["tracker"].mode = "holding"
    w.arrive()
    assert w.names("release_slam") == ["release_slam"]
    plan = [c for c in w.calls if c[0] == "plan_observation_leg"][-1][2]
    released = tuple(float(v) for v in w.runner_rear)
    assert released == pytest.approx((0.2, 0.0, 0.0))  # the estimate after the release, handed to the runner
    assert plan["start"] == pytest.approx(released) and tuple(plan["grid_rear"]) == pytest.approx(released)
    assert w.deck["target"]["centre_xy"][0] == pytest.approx(centre_before[0] + 0.2)
    assert w.obstacle["applied"] == (0.0, 0.0, 0.0) and "refresh" in w.names()


def test_a_blocked_retreat_makes_no_detour_and_ends_the_face():
    w = World(verdicts=["unobserved", "present"], slam=True)
    w.begin()
    w.arrive()
    assert w.deck["leg"]["kind"] == "retreat"
    n_plans = len(w.names("plan_observation_leg"))
    w.core.slam["tracker"].mode = "holding"
    out = w.core.blocked_replan(22.0, np.array([0.1, 0.0, 0.0]), 0.0, 0.1)
    assert out.requested_speed == 0.0 and w.names("release_slam") == ["release_slam"]
    assert w.deck["faces"][0]["reason"] == "retreat_blocked" and w.deck["leg"]["face"] == w.deck["order"][1]
    assert len(w.names("plan_observation_leg")) == n_plans + 1  # only the next face's leg, from the released pose
    # from the released estimate (0.2 here), not the held rear the runner passed (0.1)
    assert [c for c in w.calls if c[0] == "plan_observation_leg"][-1][2]["start"][0] == pytest.approx(0.2)
    assert tuple(w.runner_rear) == pytest.approx((0.2, 0.0, 0.0))


def test_a_blocked_stand_is_replanned_to_its_own_stand_and_a_failure_ends_the_face():
    w = World(verdicts=["present"])
    w.begin()
    first, second = w.deck["order"]
    w.unreachable = {first}
    w.core.blocked_replan(15.0, np.array([0.5, 0.0, 0.0]), 0.0, 0.1)
    assert w.deck["faces"][0] == {"face": first, "verdict": "unobserved", "reason": "blocked:no_path",
                                  "captures": 0}
    assert w.deck["leg"]["face"] == second and "plan_transport" not in w.names()


def test_the_same_path_again_is_a_wait_and_the_recoveries_per_face_are_bounded():
    w = World(same_path=True)
    w.begin()
    serial = w.state["observe_replan_leg"]["serial"]
    for i in range(L.RECOVERIES_PER_FACE):
        w.core.blocked_replan(15.0 + i, np.array([0.5, 0.0, 0.0]), 0.0, 0.1)
        assert w.state["observe_replan_leg"]["serial"] == serial and w.deck["leg"] is not None
    w.core.blocked_replan(19.0, np.array([0.5, 0.0, 0.0]), 0.0, 0.1)
    assert w.deck["faces"][0]["reason"] == "blocked:recovery_limit"


def test_a_new_side_leg_restarts_the_no_progress_watch():
    w = World(verdicts=["present"])
    w.begin()
    w.obstacle["progress"] = {"phase": "observe", "best_m": 0.2, "since_s": 0.0}
    w.arrive()
    assert w.obstacle["progress"] == {}


def test_a_heading_miss_on_a_side_leg_is_intercepted_before_the_observe_replan():
    import inspect

    source = inspect.getsource(MissionCore.cusp_and_observe_recovery)
    intercept = source.index('out_ = self.bottom_deck_recover(t, rear, "observe_miss")')
    assert intercept < source.index("replanned = plan_observation_leg(")
    assert source.index("tracking = trackers[\"observe\"].update(rear, signed_speed, dt)  # the new leg's") > intercept
    w = World(verdicts=["unobserved"], slam=True)
    w.begin()
    w.arrive()  # a retake: the retreat is the observe leg
    out = w.core.bottom_deck_recover(22.0, np.array([0.1, 0.0, 0.0]), "observe_miss")
    assert out.installed and w.deck["faces"][0]["reason"] == "retreat_observe_miss"


def test_the_side_record_permits_the_insertion_into_the_pallet_it_judged():
    w = World(verdicts=["present", "present"])
    w.begin()
    w.arrive()
    w.arrive(30.0)
    assert w.ms.phase == "approach"
    w.core.near = None
    w.core.options.near_tracking_stop_at = None
    w.core.arrive(40.0, SimpleNamespace(position_error_m=0.0, yaw_error_rad=0.0))
    assert w.ms.phase == "insert" and w.deck["permitted"]["shape"] == "epal6"
    w = World(verdicts=["present", "present"])
    w.begin()
    w.arrive()
    w.arrive(30.0)
    w.core.pocket["planning_pickup"] = Pose2D(3.3, 0.0, 0.0)  # the near target is another pallet
    w.core.options.near_tracking_stop_at = None
    assert w.core.arrive(40.0, SimpleNamespace(position_error_m=0.0, yaw_error_rad=0.0)).stop is True
    assert w.deck["refused"]["reason"] == "bottom_deck_target_mismatch"


def test_a_retreat_the_planner_refuses_ends_the_face_instead_of_driving_unchecked():
    w = World(verdicts=["unobserved"], retreat_ok=False)
    w.begin()
    w.arrive()
    assert w.deck["faces"][0]["reason"] == "retreat_blocked:invalid_goal" and w.deck["leg"]["kind"] == "stand"


def test_without_slam_a_side_legs_heading_miss_goes_through_the_recovery_entry_point():
    from dataclasses import replace as dc_replace

    w = World(verdicts=["unobserved"])
    w.begin()
    w.arrive()  # the retreat is the observe leg now
    opts = w.core.options
    opts.max_cusp_replans, opts.cusp_stop_needed, opts.cusp_stop_yaw_rate_radps = 2, 3, 0.05
    opts.cusp_heading_hold_position_m, opts.observe_replans_per_leg = 0.05, 2
    w.state["cusp_replans"] = []
    w.hooks.plan_transport_leg = lambda *a, **k: pytest.fail("no transport replan")
    w.hooks.cusp_holds = lambda: None
    w.hooks.cusp_monitor = lambda: SimpleNamespace(holds_observe=lambda: False)
    w.hooks.cusp_heading_motion = lambda v: {}
    tracking = w.core.trackers["observe"].update(np.array([0.0, 0.0, 0.0]), 0.0, 1 / 120)
    tracking = dc_replace(tracking, status="failed", failure="endpoint_heading", off_path=False, cusp_pending=False,
                          at_cusp=False)
    w.velocity = (-0.1, 0.0, 0.0)  # still rolling: braking, no plan, no face ended (Codex round 2 P2)
    n_plans = len(w.names("plan_observation_leg"))
    out = w.core.cusp_and_observe_recovery(29.0, np.array([0.0, 0.0, 0.0]), tracking, -0.1, 1 / 120, {}, False, 699)
    assert (out.tracking.status, out.tracking.speed_mps) == ("braking", 0.0)
    assert w.deck["faces"] == [] and len(w.names("plan_observation_leg")) == n_plans
    w.velocity = (0.0, 0.0, 0.0)
    out = w.core.cusp_and_observe_recovery(30.0, np.array([0.0, 0.0, 0.0]), tracking, 0.0, 1 / 120, {}, False, 700)
    assert w.deck["faces"][0]["reason"] == "retreat_observe_miss"  # not a tracker failure
    assert out.tracking.status != "failed" and w.deck["leg"]["face"] == w.deck["order"][1]


def test_the_record_holds_the_centre_the_planned_pickup_is_carried_to():
    # found with Codex implementation round 2: a release after the record would part the two
    w = World(verdicts=["present", "present"], slam=True)
    w.begin()
    w.arrive()
    w.core.slam["tracker"].mode = "holding"  # held again on the way to the second stand
    w.arrive(30.0)
    rec, pickup = w.deck["record"], w.core.pocket["planning_pickup"]
    assert w.deck["releases"][-1]["event"] == "release_at_bottom_deck_capture"
    assert rec["centre_xy"] == pytest.approx([pickup.x_m, pickup.y_m]) and rec["model_yaw_rad"] == pytest.approx(
        pickup.yaw_rad)
