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
from forklift_core.planning import Bounds, Footprint, PlanResult, Pose2D

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
                 mission_ok=True, same_path=False, retreat_ok=True, zone=None, pickup=None, return_ok=True):
        self.calls, self.verdicts, self.arrivals = [], list(verdicts), list(arrivals or [])
        self.unreachable, self.same, self.retreat_ok, self.return_ok = set(unreachable), same_path, retreat_ok, return_ok
        self.runner_rear = REAR
        log = self.log

        def plan_observation_leg(world, target, config, **kw):
            goal = (target.x_m, target.y_m, target.yaw_rad)
            start = (kw["start_rear"].x_m, kw["start_rear"].y_m, kw["start_rear"].yaw_rad)
            log("plan_observation_leg", goal=goal, start=start, clearance=config.clearance_m,
                grid_rear=tuple(self.runner_rear), bounds=kw["pickup_bounds"])
            face = self.face_of(goal)
            if np.allclose(goal[:2], REAR[:2], atol=1e-6):  # the return to the far stand
                return path_to(goal, start) if self.return_ok else failed("no_path")
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
            travel_config=None, scenario=SimpleNamespace(bounds=Bounds(-25.6, 4.7, -22.9, 8.3)), pickup_bounds=zone,
            return_to_pose="HOME", mission_stages=stages,
            speeds={n: 0.3 for n in stages + ["observe", "return_home"]}, cusp_heading_hold_rad=0.08,
            cusp_heading_hold_phases=("observe",), perception_max_attempts=3,
            adapter=SimpleNamespace(CaptureFailure=CaptureFailure), near_tracking=None, backoff_m=0.3,
            backoff_speed_mps=0.1, backoff_per_leg=1, escape_per_leg=1, observation_waypoints=())
        settings = {"tracker_curvature_inv_m": 0.45, "drive_acceleration_mps2": 0.5}
        self.core = MissionCore(self.ms, state=self.state, obstacle=self.obstacle, slam=slam_,
                                pocket={"check": None, "planning_pickup": pickup or PICKUP}, paths={"observe": None},
                                trackers=trackers, geometry=SimpleNamespace(
                                    pallet_depth_m=0.8, alignment_straight_m=0.8, approach_gap_m=0.1,
                                    unloaded_footprint=Footprint(1.29, 0.17, 0.36)),
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
        attempt = {"waypoint": list(REAR), **(attempt or {})}  # the first far observation's stand
        return self.core._bottom_deck_begin(10.0, REAR, np.array([0.37, 0.0, 0.0]), 0.0, attempt, 2,
                                            "OBS", self.core.pocket["planning_pickup"])

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
    assert len(w.names("plan_observation_leg")) == 3  # both stands and the way back reachable before the first leg
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


def test_two_present_faces_send_the_truck_back_to_the_far_stand_with_the_record_present():
    w = World(verdicts=["present", "present"])
    w.begin()
    first, second = w.deck["order"]
    out = w.arrive()
    assert (out.stop, w.deck["leg"]["face"], w.deck["leg"]["kind"]) == (False, second, "stand")
    out = w.arrive(30.0)
    assert out.stop is False and w.deck["status"] == "returning" and w.deck["leg"]["kind"] == "return"
    assert tuple(w.core.paths["observe"].poses[-1]) == pytest.approx(tuple(REAR))  # back to the far stand
    assert "plan_transport" not in w.names() and "near_capture" not in w.state  # the mission comes after
    rec = w.deck["record"]
    assert rec["verdict"] == "present" and [f["face"] for f in rec["faces"]] == [first, second]
    assert rec["centre_xy"] == pytest.approx([PICKUP.x_m, PICKUP.y_m])  # the map target judged, unmoved
    assert [a["npz"] for a in w.deck["attempts"]] == ["bottom_deck/side_01.npz", "bottom_deck/side_02.npz"]
    assert w.ms.phase == "observe" and not w.core.bottom_deck_side_leg() and w.core.bottom_deck_leg_active()


def test_under_slam_no_near_capture_is_set_up_by_the_side_legs():
    w = World(verdicts=["present", "present"], slam=True)
    w.begin()
    w.arrive()
    w.arrive(30.0)
    assert "near_capture" not in w.state and w.deck["status"] == "returning"
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


def test_a_held_pose_is_released_before_the_judgement_and_the_map_target_stays():
    # Codex return design rounds 1-2: the far target is a map target made after its own release; a release in the
    # side stage moves the truck's estimate, not the target, and the face is judged by the released estimate
    w = World(verdicts=["present", "present"], slam=True)
    w.begin()
    centre_before = list(w.deck["target"]["centre_xy"])
    w.core.slam["tracker"].mode = "holding"
    w.arrive()
    assert w.names("release_slam") == ["release_slam"]
    assert w.deck["attempts"][0]["rear_estimate"][0] == pytest.approx(0.2)  # judged after the release
    assert w.deck["target"]["centre_xy"] == pytest.approx(centre_before)
    assert w.core.pocket["planning_pickup"] == PICKUP and w.obstacle["pickup_estimate"] == PICKUP
    assert w.deck["releases"][0]["carried"] is False
    plan = [c for c in w.calls if c[0] == "plan_observation_leg"][-1][2]
    released = tuple(float(v) for v in w.runner_rear)
    assert released == pytest.approx((0.2, 0.0, 0.0)) and plan["start"] == pytest.approx(released)
    assert tuple(plan["grid_rear"]) == pytest.approx(released) and "refresh" in w.names()


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


def test_the_reobservation_must_be_the_pallet_judged_and_the_gate_then_permits_it():
    w = World(verdicts=["present", "present"])
    w.begin()
    w.arrive()
    w.arrive(30.0)
    assert w.core._bottom_deck_reobserved(35.0, REAR, {}, Pose2D(3.05, 0.02, 0.01)) is None
    assert w.deck["status"] == "done" and w.deck["reobservation"]["refusal"] is None
    w.ms.phase = "approach"
    w.core.near = None
    w.core.options.near_tracking_stop_at = None
    w.core.pocket["planning_pickup"] = Pose2D(3.05, 0.02, 0.01)
    w.core.arrive(40.0, SimpleNamespace(position_error_m=0.0, yaw_error_rad=0.0))
    assert w.ms.phase == "insert" and w.deck["permitted"]["shape"] == "epal6"
    w = World(verdicts=["present", "present"])
    w.begin()
    w.arrive()
    w.arrive(30.0)
    out = w.core._bottom_deck_reobserved(35.0, REAR, {}, Pose2D(3.3, 0.0, 0.0))  # another pallet
    assert out.stop is True and w.deck["refused"] == {"reason": "bottom_deck_target_mismatch", "time_s": 35.0,
                                                      "stage": "reobserve"}
    assert w.deck["record"]["centre_xy"] == pytest.approx([3.0, 0.0])  # never rebound


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



# --- the side stage's bounds (Isaac seed 0, Codex A') ---------------------------------------------------------------

ZONE = Bounds(-3.0, 4.7, -1.75, 3.05)  # the factory hall's pickup zone
SEED0 = Pose2D(3.222, 1.033, 0.020)    # the far estimate of the seed-0 run: its +y stand's corner leaves the zone


def test_every_side_plan_and_the_return_use_the_side_bounds():
    w = World(zone=ZONE, pickup=SEED0)
    out = w.begin()
    assert out.stop is False
    b = w.deck["bounds"]
    assert b["y_max_m"] > 3.05 + 0.2 and (b["x_min_m"], b["x_max_m"]) == (-3.0, 4.7)  # the zone, grown where needed
    assert b["y_min_m"] <= -1.75 and b["y_max_m"] <= 8.3  # never shrunk below the zone, clipped to the hall
    plans = [c[2] for c in w.calls if c[0] == "plan_observation_leg"]
    assert len(plans) == 3 and all(p["bounds"] == Bounds(**b) for p in plans)
    first, second = w.deck["order"]
    assert plans[1]["start"] == pytest.approx(tuple(w.deck["stands"][str(first)]["rear"]))
    assert plans[2]["start"] == pytest.approx(tuple(w.deck["stands"][str(second)]["rear"]))
    assert plans[2]["goal"] == pytest.approx(tuple(REAR))  # back to the far stand
    for f in ("1", "-1"):  # every stand and retreat footprint (with the start margin) fits the side bounds
        st = w.deck["stands"][f]["rear"]
        for pose in (st, L.retreat_pose(st)):
            assert L.inside(Bounds(**b), pose, Footprint(1.29, 0.17, 0.36), 0.05)


def test_stands_outside_the_zone_need_no_zone_check_now_the_truck_returns():
    # Codex return design round 2 P2 2: the mission is planned from the far stand, so the stands need not lie in the
    # zone; reaching them and coming back is what the plans check
    w = World(zone=Bounds(-3.0, 4.7, -1.0, 1.0), pickup=Pose2D(3.0, 0.0, 0.0))
    out = w.begin()
    assert out.stop is False and len(w.names("plan_observation_leg")) == 3


def test_a_return_that_cannot_be_planned_is_refused_at_the_return_stage():
    w = World(verdicts=["present", "present"], zone=ZONE, pickup=SEED0)
    w.begin()
    w.arrive()
    w.return_ok = False
    out = w.arrive(30.0)
    assert out.stop is True and w.deck["refused"]["reason"] == "bottom_deck_return_failed"
    assert w.deck["refused"]["stage"] == "return" and w.deck["record"]["verdict"] == "present"
    assert "plan_transport" not in w.names()


def test_the_observation_bounds_are_the_side_bounds_while_a_side_leg_or_the_return_drives():
    # every check of a side leg and of the return (its replans, the cusp check through observe_bounds) sees the bounds
    # it was planned in; before and after the side stage the observation bounds are the zone again
    w = World(zone=ZONE, pickup=SEED0, verdicts=["present", "present"])
    assert w.core.observe_bounds() == ZONE
    w.begin()
    assert w.core.bottom_deck_leg_active() and w.core.observe_bounds() == Bounds(**w.deck["bounds"])
    w.arrive()
    w.arrive(30.0)
    assert w.deck["leg"]["kind"] == "return" and w.core.observe_bounds() == Bounds(**w.deck["bounds"])
    w.deck["leg"] = None  # arrived back: the re-observation clears the leg
    assert w.core.observe_bounds() == ZONE


def test_a_blocked_return_is_replanned_to_the_far_stand_with_its_own_budget():
    w = World(verdicts=["present", "present"])
    w.begin()
    w.arrive()
    w.arrive(30.0)
    for i in range(L.RETURN_RECOVERIES):
        out = w.core.blocked_replan(31.0 + i, np.array([1.0, 0.5, 0.0]), 0.0, 0.1)
        assert w.deck["leg"]["kind"] == "return"
        assert tuple(w.core.paths["observe"].poses[-1]) == pytest.approx(tuple(REAR))  # never a side stand
    w.core.blocked_replan(40.0, np.array([1.0, 0.5, 0.0]), 0.0, 0.1)
    assert w.deck["refused"] == {"reason": "bottom_deck_return_failed", "time_s": 40.0, "stage": "return"}


def test_a_reobservation_without_the_pallet_is_refused_without_other_candidates():
    w = World(verdicts=["present", "present"])
    w.begin()
    w.arrive()
    w.arrive(30.0)
    w.deck["leg"] = None  # arrived back
    ended = w.core.observe_failed(36.0, REAR, {}, 3, SimpleNamespace(status="no_pallet", reason="no_candidate"))
    assert ended is True and w.deck["refused"]["reason"] == "bottom_deck_reobservation_failed"
    assert w.deck["refused"]["stage"] == "reobserve" and w.ms.phase == "bottom_deck_refused"


def test_a_failed_reobservation_capture_is_refused_by_name_and_ends_the_loop():
    w = World(verdicts=["present", "present"])
    w.begin()
    w.arrive()
    w.arrive(30.0)
    w.deck["leg"] = None
    exc = CaptureFailure("not_still")
    exc.diagnostics = SimpleNamespace(attempts=3)
    w.core.hooks.live_abort = None
    from dataclasses import dataclass as _dc

    @_dc
    class Diag:
        attempts: int = 3
    exc.diagnostics = Diag()
    out = w.core._capture_failed({}, exc)
    assert out.refused is True and out.aborted is False
    assert w.deck["refused"]["reason"] == "bottom_deck_reobservation_failed" and w.ms.phase == "bottom_deck_refused"



def test_the_capture_clears_the_return_leg_and_reprojects_after_its_release_only_with_the_check_on():
    import inspect

    source = inspect.getsource(MissionCore.capture_and_detect)
    clear = source.index('state["bottom_deck"]["leg"] = None  # arrived back')
    assert clear < source.index("attempt, attempt_number = self._new_attempt(")
    release = source.index('"event": "release_at_capture",')
    reproject = source.index("self._reproject_after_release(", release)
    assert source.rindex("if self.bottom_deck_on():", release, reproject) > release  # off runs unchanged



def test_the_return_legs_no_progress_watch_does_not_carry_into_the_near_leg():
    w = World(verdicts=["present", "present"])
    w.begin()
    w.arrive()
    w.arrive(30.0)
    w.obstacle["progress"] = {"phase": "observe", "best_m": 0.5, "since_s": 10.0}  # the return's, nearly arrived
    assert w.core._bottom_deck_reobserved(35.0, REAR, {}, PICKUP) is None
    assert w.obstacle["progress"] == {}



def test_the_return_narrows_the_far_recognitions_zone_to_the_pallet_judged():
    w = World(verdicts=["present", "present"])
    zones = []
    w.hooks.far_recognition_zone = lambda centre, radius: zones.append((centre, radius))
    w.begin()
    w.arrive()
    w.arrive(30.0)
    assert zones == [((PICKUP.x_m, PICKUP.y_m), L.REOBSERVE_RADIUS_M)]
    assert w.deck["reobservation_zone"]["radius_m"] == L.REOBSERVE_RADIUS_M


NEXT = (1.0, 1.0, 0.0)  # the next fixed observation point (the far stand is REAR)


def reobserve_attempt(status="reobserve", reason="uncertain_in_radius"):
    return {"recognition": {"status": status, "reason": reason, "leg": 1}}


def returned(waypoints=(tuple(REAR), NEXT), **kw):
    """Both faces present, arrived back at the far stand (the capture cleared the return leg)."""
    w = World(verdicts=["present", "present"], **kw)
    w.core.options.observation_waypoints = waypoints
    w.ms.next_candidate_index = 1  # the far stand was the first point
    w.begin()
    w.arrive()
    w.arrive(30.0)
    w.deck["leg"] = None
    return w


def test_a_look_again_decision_takes_the_next_observation_point_as_a_return_leg():
    # Isaac ecf4be5 s1: an unconfirmed proposal in reach at the re-observation stop asked to look again; the far
    # observation's own answer is the next fixed point (observe_failed's ladder), so the re-observation takes it.
    w = returned(zone=ZONE)
    w.obstacle["progress"] = {"phase": "observe", "best_m": 0.3, "since_s": 10.0}
    serial = w.state["observe_replan_leg"]["serial"]
    ended = w.core.observe_failed(36.0, REAR, reobserve_attempt(), 3,
                                  SimpleNamespace(status="no_pallet", reason="uncertain_in_radius"))
    assert ended is False and w.deck["status"] == "returning" and w.deck["leg"]["kind"] == "return"
    assert tuple(w.core.paths["observe"].poses[-1]) == pytest.approx(NEXT)
    assert w.ms.next_candidate_index == 2
    assert w.obstacle["progress"] == {} and w.state["observe_replan_leg"]["serial"] == serial + 1
    plans = [c for c in w.calls if c[0] == "plan_observation_leg"]
    assert plans[-1][2]["bounds"] == Bounds(**w.deck["bounds"])  # the side stage's bounds, as its recoveries
    assert w.deck["reobservation_attempts"][-1] == {
        "time_s": 36.0, "attempt": 3, "far_leg": 1, "status": "reobserve", "reason": "uncertain_in_radius",
        "outcome": "retry", "candidate_index": 1, "goal": list(NEXT)}
    assert "refused" not in w.deck


@pytest.mark.parametrize("status,reason", [("ask_operator", "multiple_in_radius"),
                                           ("none", "no_candidate_in_radius"),
                                           ("selected", "target_inconsistent")])
def test_a_reobservation_that_does_not_ask_to_look_again_is_refused(status, reason):
    w = returned()
    ended = w.core.observe_failed(36.0, REAR, reobserve_attempt(status, reason), 3,
                                  SimpleNamespace(status="no_pallet", reason=reason))
    assert ended is True and w.deck["refused"]["reason"] == "bottom_deck_reobservation_failed"
    assert w.deck["refused"]["detail"] == reason and w.ms.next_candidate_index == 1
    assert w.deck["reobservation_attempts"][-1]["outcome"] == "refused"


def test_a_blocked_retry_leg_is_replanned_to_its_own_point_with_its_own_budget():
    w = World(verdicts=["present", "present"])
    w.core.options.observation_waypoints = (tuple(REAR), NEXT)
    w.ms.next_candidate_index = 1
    w.begin()
    w.arrive()
    w.arrive(30.0)
    w.core.blocked_replan(31.0, np.array([0.5, 0.5, 0.0]), 0.0, 0.1)  # one recovery on the return itself
    assert tuple(w.core.paths["observe"].poses[-1]) == pytest.approx(tuple(REAR))
    w.deck["leg"] = None
    w.core.observe_failed(36.0, REAR, reobserve_attempt(), 3, SimpleNamespace(status="no_pallet", reason="x"))
    for i in range(L.RETURN_RECOVERIES):
        w.core.blocked_replan(37.0 + i, np.array([0.5, 0.5, 0.0]), 0.0, 0.1)
        assert w.deck["leg"]["kind"] == "return"
        assert tuple(w.core.paths["observe"].poses[-1]) == pytest.approx(NEXT)  # the retry's point, not the far stand
    w.core.blocked_replan(45.0, np.array([0.5, 0.5, 0.0]), 0.0, 0.1)
    assert w.deck["refused"]["reason"] == "bottom_deck_return_failed"
    assert "plan_transport" not in w.names()  # never the mission replan


def test_the_retries_end_at_the_bound_and_when_the_points_run_out():
    points = (tuple(REAR),) + tuple((1.0 + k, 1.0, 0.0) for k in range(L.REOBSERVE_RETRIES + 1))
    w = returned(waypoints=points)
    for k in range(L.REOBSERVE_RETRIES):
        assert w.core.observe_failed(36.0 + k, REAR, reobserve_attempt(), 3 + k,
                                     SimpleNamespace(status="no_pallet", reason="r")) is False
        w.deck["leg"] = None
    assert w.core.observe_failed(50.0, REAR, reobserve_attempt(), 9, SimpleNamespace(status="no_pallet", reason="r"))
    assert w.deck["refused"]["reason"] == "bottom_deck_reobservation_failed"
    w2 = returned(waypoints=(tuple(REAR),))
    assert w2.core.observe_failed(36.0, REAR, reobserve_attempt(), 3, SimpleNamespace(status="no_pallet", reason="r"))
    assert w2.deck["refused"]["detail"] == "candidates_exhausted"
    assert w2.deck["reobservation_attempts"][-1]["outcome"] == "exhausted"


def test_a_retried_reobservation_is_still_checked_against_the_record():
    w = returned()
    w.core.observe_failed(36.0, REAR, reobserve_attempt(), 3, SimpleNamespace(status="no_pallet", reason="r"))
    w.deck["leg"] = None
    out = w.core._bottom_deck_reobserved(40.0, REAR, reobserve_attempt("selected", None),
                                         Pose2D(PICKUP.x_m + 0.2, PICKUP.y_m, PICKUP.yaw_rad))
    assert out is not None and w.deck["refused"]["reason"] == "bottom_deck_target_mismatch"
    assert w.deck["reobservation_attempts"][-1]["outcome"] == "mismatch"
    w = returned()
    assert w.core._bottom_deck_reobserved(40.0, REAR, reobserve_attempt("selected", None), PICKUP) is None
    assert w.deck["reobservation_attempts"][-1]["outcome"] == "accepted"


def test_a_retry_point_the_mission_could_not_start_from_is_skipped():
    # Codex retry review round 5 P2: the side bounds reach a point whose footprint leaves the pickup zone, and the
    # mission planned there after the re-observation fails approach:invalid_start; the next point is taken instead.
    outside = (0.0, 3.0, 0.0)
    w = returned(waypoints=(tuple(REAR), outside, NEXT), zone=ZONE)
    assert w.core.observe_failed(36.0, REAR, reobserve_attempt(), 3,
                                 SimpleNamespace(status="no_pallet", reason="r")) is False
    assert tuple(w.core.paths["observe"].poses[-1]) == pytest.approx(NEXT)
    goals = [c[2]["goal"] for c in w.calls if c[0] == "plan_observation_leg"]
    assert not any(np.allclose(g, outside) for g in goals)
    skipped = [c for c in w.state["observation_candidates"] if c["pose"] == list(outside)]
    assert skipped and skipped[0]["status"] == "outside_mission_zone" and skipped[0]["success"] is False
    assert w.deck["reobservation_attempts"][-1]["candidate_index"] == 2


def test_without_a_pickup_zone_the_retry_point_is_checked_against_the_scenario_bounds():
    # Codex retry review round 6 P2: with no pickup bounds the mission plans in the scenario bounds (near_bounds);
    # a point whose footprint leaves them would fail approach:invalid_start just the same.
    edge = (4.0, 0.0, 0.0)  # the front of the footprint passes the scenario's x_max 4.7
    w = returned(waypoints=(tuple(REAR), edge, NEXT))
    assert w.core.options.pickup_bounds is None
    assert w.core.observe_failed(36.0, REAR, reobserve_attempt(), 3,
                                 SimpleNamespace(status="no_pallet", reason="r")) is False
    assert tuple(w.core.paths["observe"].poses[-1]) == pytest.approx(NEXT)
    assert [c["status"] for c in w.state["observation_candidates"] if c["pose"] == list(edge)] == ["outside_mission_zone"]


def test_a_near_capture_off_the_record_is_refused_before_it_is_adopted():
    # Plan 2026-10-21 task C (Codex plan round 1 P1): with the check on, the near estimate is checked against the
    # record before it becomes the planned pickup, the known pallet or a plan -- not first at the insert gate.
    w = returned()
    w.deck["status"] = "done"
    off = Pose2D(PICKUP.x_m + 0.30, PICKUP.y_m, PICKUP.yaw_rad)
    out = w.core._bottom_deck_near_check(60.0, REAR, off)
    assert out is not None and out.stop is True
    assert w.deck["refused"] == {"reason": "bottom_deck_target_mismatch", "time_s": 60.0, "stage": "near"}
    assert w.deck["near_check"]["refusal"] is not None and w.ms.phase == "bottom_deck_refused"
    w = returned()
    w.deck["status"] = "done"
    assert w.core._bottom_deck_near_check(60.0, REAR, PICKUP) is None
    assert w.deck["near_check"]["refusal"] is None and "refused" not in w.deck


def test_observe_succeeded_checks_the_near_capture_before_any_adoption():
    import inspect

    source = inspect.getsource(MissionCore.observe_succeeded)
    check = source.index("self._bottom_deck_near_check(")
    for later in ("hooks.set_inset_estimate(", "hooks.planning_target(", "known_pallet(", 'pocket["planning_pickup"] ='):
        assert check < source.index(later)
    assert source.index("self._bottom_deck_reobserved(") < check
    # plan 2026-10-28-adoption-zone-check: the zone check reads the estimate before anything adopts it as well
    zone = source.index("self.adoption_check_estimate(")
    for later in ("hooks.set_inset_estimate(", "hooks.planning_target(", "known_pallet(", 'pocket["planning_pickup"] ='):
        assert zone < source.index(later)


def test_a_pallet_the_fork_spacing_does_not_fit_ends_by_name_without_wandering():
    # T11 at 0.3725 m (2026-10-21): every far observation read not_pickable / fork_spacing_adjust and the mission went on
    # to the next observation points until obstacle_no_progress. Another viewpoint does not change the spacing.
    w = World()
    w.core.options.observation_waypoints = ((0.0, 0.0, 0.0), NEXT)
    w.ms.next_candidate_index = 1
    known = reobserve_attempt("not_pickable", "fork_spacing_adjust")
    known["recognition"]["decided"] = {"hypothesis": "t11_06:x"}
    with pytest.raises(RuntimeError, match="not_pickable:fork_spacing_adjust"):
        w.core.observe_failed(30.0, REAR, known, 2, SimpleNamespace(status="no_pallet", reason="fork_spacing_adjust"))
    assert w.ms.next_candidate_index == 1


@pytest.mark.parametrize("decided", [None, {"hypothesis": "parametric:3col"}])
def test_a_measured_or_unnamed_misfit_still_looks_again(decided):
    # Codex T11 review P2: a non-standard pallet's openings are measured, so another view can change the fit.
    w = World()
    w.core.options.observation_waypoints = ((0.0, 0.0, 0.0), NEXT)
    w.ms.next_candidate_index = 1
    w.state["observation_candidates"] = []
    w.hooks.set_inset = lambda *a: None
    attempt = reobserve_attempt("not_pickable", "fork_spacing_adjust")
    if decided is not None:
        attempt["recognition"]["decided"] = decided
    assert w.core.observe_failed(30.0, REAR, attempt, 2,
                                 SimpleNamespace(status="no_pallet", reason="fork_spacing_adjust")) is False
    assert w.ms.next_candidate_index == 2


def test_other_not_pickable_reasons_still_look_again():
    w = World()
    w.core.options.observation_waypoints = ((0.0, 0.0, 0.0), NEXT)
    w.ms.next_candidate_index = 1
    w.state["observation_candidates"] = []
    w.hooks.set_inset = lambda *a: None
    assert w.core.observe_failed(30.0, REAR, reobserve_attempt("not_pickable", "lift_alignment_needed"), 2,
                                 SimpleNamespace(status="no_pallet", reason="lift_alignment_needed")) is False
    assert w.ms.next_candidate_index == 2


# --- plan 2026-10-27 (bottomless pallets) §2.1 g: --bottom-deck-check structure --------------------------------------

def structure_world(verdicts):
    w = World(verdicts=verdicts)
    w.core.options.bottom_deck_check = "structure"
    return w


def test_structure_sees_the_second_face_after_an_absent_first_one_and_returns():
    w = structure_world(["absent", "absent"])
    w.begin()
    first, second = w.deck["order"]
    out = w.arrive()
    assert (out.stop, w.deck["leg"]["face"], w.deck["leg"]["kind"]) == (False, second, "stand")
    out = w.arrive(30.0)
    assert out.stop is False and w.deck["status"] == "returning" and w.deck["leg"]["kind"] == "return"
    rec = w.deck["record"]
    assert rec["verdict"] == "absent" and [f["verdict"] for f in rec["faces"]] == ["absent", "absent"]
    assert "refused" not in w.deck


@pytest.mark.parametrize("verdicts", [["present", "absent"], ["absent", "present"]])
def test_structure_takes_rows_with_and_without_bottom_boards(verdicts):
    w = structure_world(verdicts)
    w.begin()
    w.arrive()
    out = w.arrive(30.0)
    assert out.stop is False and w.deck["status"] == "returning" and w.deck["record"]["verdict"] == "absent"


def test_structure_refuses_a_face_it_could_not_see_even_after_an_absent_one():
    w = structure_world(["absent", "unobserved", "unobserved", "unobserved"])
    w.begin()
    w.arrive()
    w.arrive(21.0)  # the second face: unobserved, retreat, again
    for k in range(2):
        w.core.bottom_deck_capture(22.0 + 2 * k, np.asarray(w.core.paths["observe"].poses[-1]), 700 + k)
        out = w.arrive(23.0 + 2 * k)
    assert out.stop is True and w.ms.phase == "bottom_deck_refused"
    assert w.deck["refused"]["reason"] == "bottom_deck_unobserved" and w.deck["record"]["verdict"] == "unobserved"


def test_structure_permits_the_insert_on_an_absent_record_of_the_pallet_judged():
    w = structure_world(["absent", "absent"])
    w.begin()
    w.arrive()
    w.arrive(30.0)
    assert w.core._bottom_deck_reobserved(35.0, REAR, {}, Pose2D(3.05, 0.02, 0.01)) is None
    w.ms.phase = "approach"
    w.core.near = None
    w.core.options.near_tracking_stop_at = None
    w.core.pocket["planning_pickup"] = Pose2D(3.05, 0.02, 0.01)
    w.core.arrive(40.0, SimpleNamespace(position_error_m=0.0, yaw_error_rad=0.0))
    assert w.ms.phase == "insert" and w.deck["permitted"]["shape"] == "epal6"


def test_structure_waives_the_near_lower_band_on_an_absent_record_too():
    w = structure_world(["absent", "absent"])
    w.begin()
    w.arrive()
    w.arrive(30.0)
    w.core._bottom_deck_reobserved(35.0, REAR, {}, Pose2D(3.05, 0.02, 0.01))
    w.state["near_capture"] = {"status": "pending"}
    assert w.core._bottom_deck_lower_waived() is True


def test_on_still_ends_at_the_first_absent_face():
    w = World(verdicts=["absent"])
    assert w.core.options.bottom_deck_check == "on"
    w.begin()
    out = w.arrive()
    assert out.stop is True and w.deck["refused"]["reason"] == "no_bottom_deck" and len(w.deck["faces"]) == 1
