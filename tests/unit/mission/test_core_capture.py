"""M-a1 slice 4b-i: the capture and the detection at the observe arrival, moved from the runner (behaviour,
state, port requests and their order, effects up to a refusal -- the plan's M-a1 criterion)."""

import math
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.perception.pallet_prior import load_pallet_prior

PRIOR = load_pallet_prior(Path(__file__).resolve().parents[3] / "config/pallet_prior_epal6.yaml")

REAR = np.array([1.0, 2.0, 0.0])


class CaptureFailure(Exception):
    def __init__(self, reason, diagnostics):
        super().__init__(reason)
        self.reason, self.diagnostics = reason, diagnostics


@dataclass
class Diag:
    accepted_pose: tuple
    attempts: int = 2


@dataclass
class Frame:
    ok: bool = True


@dataclass
class Observation:
    status: str = "valid"


@dataclass
class DetectionDiag:
    n: int = 1


def yaw_and_tilt(q):
    return float(2 * math.atan2(q[3], q[0])), 0.0


class World:
    def __init__(self, *, slam_mode=None, near_capture=None, near=None, observation=Observation(), fail=False):
        self.calls = []
        log_ = self.log
        yaw = math.pi / 2
        q = np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])
        diag = Diag(accepted_pose=(np.array([3.0, 4.0, 0.2]), q))

        self.clock = 54.0  # the physics time; the capture steps it (Codex 4b P3)

        def capture(max_attempts):
            log_("capture", max_attempts)
            self.clock = 55.0
            if fail:
                raise CaptureFailure("not_still", Diag(accepted_pose=(), attempts=5))
            return SimpleNamespace(depth_m=np.array([[1.0, np.nan]]), base_from_optical=SimpleNamespace(
                translation_m=(0.1, 0.0, 1.0), rotation=np.eye(3))), Frame(), 3

        self.capture = SimpleNamespace(capture=capture, state=SimpleNamespace(diagnostics=diag))

        def require(ok, reason):
            log_("require", bool(ok), reason)
            if not ok:
                raise RuntimeError(reason)

        tracker = SimpleNamespace(mode=slam_mode,
                                  release=lambda **k: log_("slam.release", **k) or (0.02, 0.01))
        self.slam = {"tracker": tracker, "holds": []} if slam_mode else None
        self.hooks = SimpleNamespace(
            require=require, perception_capture=lambda: self.capture, slam_odom_base=lambda: "ODOM",
            slam_rear=lambda now: log_("slam_rear", now) or np.array([2.9, 3.6, 1.5]),
            slam_error=lambda: log_("slam_error") or (0.1, 0.4, 0.07),
            detect_pockets=lambda inp, prior, params: log_("detect_pockets", inp, prior) or SimpleNamespace(
                observation=observation, diagnostics=DetectionDiag()),
            save_capture=lambda number, scene: log_("save_capture", number), now_s=lambda: self.clock,
        )
        adapter = SimpleNamespace(CaptureFailure=CaptureFailure,
                                  detector_input=lambda scene, mm: log_("detector_input", mm) or "INPUT")
        G2 = SimpleNamespace(depth_sha256=lambda depth: "SHA")
        self.ms = MissionState(phase="observe", phase_started=0.0, next_candidate_index=2)
        self.state = {"transitions": [], "observation_attempts": [{}], "observation_waypoint_selected": (5.0, 1.0, 0.0),
                      "near_tracking": {"retry": {}, "realign": {}}}
        if near_capture is not None:
            self.state["near_capture"] = {"status": near_capture}
        self.pocket = {"check": None}
        self.core = MissionCore(
            self.ms, state=self.state, obstacle=None, slam=self.slam, pocket=self.pocket, paths={}, trackers={},
            geometry=None, settings={}, pallet_geometry=None, hooks=self.hooks, near=near,
            options=SimpleNamespace(adapter=adapter, G2=G2, yaw_and_tilt=yaw_and_tilt, perception_max_attempts=4,
                                    rear_axle_offset_m=-0.4, pallet_prior_loaded=PRIOR, depth_quantize_mm=1))
        # Moved into MissionCore (M-a1 slice 6e): this test's stand-ins go on the instance.
        for name_ in ['control_correction', 'known_pallet', 'return_turnaround_kwargs', 'realign_active', 'realign_install', 'start_near_realign', 'apply_tracker_profile', 'record_tracker_configs', 'build_near_tracker']:
            if hasattr(self.hooks, name_):
                setattr(self.core, name_, getattr(self.hooks, name_))

    def log(self, name, *args, **kwargs):
        self.calls.append((name, args, kwargs))

    def names(self):
        return [c[0] for c in self.calls if c[0] != "require"]

    def run(self):
        return self.core.capture_and_detect(30.0, REAR, 600)


def test_the_capture_is_recorded_dumped_and_its_accepted_pose_is_the_capture_base():
    w = World()
    out = w.run()
    assert w.names() == ["capture", "save_capture", "detector_input", "detect_pockets"]
    assert w.calls[0][1] == (4,) and w.calls[1][1] == (2,)
    attempt = out.attempt
    assert w.state["observation_attempts"][-1] is attempt and out.attempt_number == 2
    assert attempt["candidate_index"] == 1 and attempt["waypoint"] == [5.0, 1.0, 0.0]
    assert attempt["arrived_pose"] == [1.0, 2.0, 0.0] and (attempt["loop_step"], attempt["t_before_capture_s"]) == (600, 30.0)
    assert attempt["depth_sha256"] == "SHA" and w.pocket["capture"] is out.scene_input
    assert attempt["capture_diagnostics"]["attempts"] == 2 and attempt["capture_attempts"] == 3
    assert attempt["accepted_pose"][0] == [3.0, 4.0, 0.2]
    assert out.yaw == pytest.approx(math.pi / 2) and out.forward == pytest.approx([0.0, 1.0])
    np.testing.assert_allclose(out.rear, [3.0, 3.6, math.pi / 2])  # 0.4 m behind the base along the heading
    np.testing.assert_array_equal(out.base, [3.0, 4.0, 0.2])
    assert out.rear_stamp is None and "control_pose_source" not in attempt
    assert attempt["depth_quantize_mm"] == 1 and attempt["base_from_optical"]["translation_m"] == [0.1, 0.0, 1.0]
    assert attempt["pocket_observation"] == {"status": "valid"} and attempt["detection_diagnostics"] == {"n": 1}
    assert out.observation == Observation() and out.prior is PRIOR and w.state["detector_params"]
    assert w.calls[[c[0] for c in w.calls].index("detect_pockets")][1] == ("INPUT", PRIOR)


def test_a_capture_failure_is_recorded_then_ends_the_run():
    w = World(fail=True)
    with pytest.raises(RuntimeError, match="perception_capture_failed:not_still"):
        w.run()
    attempt = w.state["observation_attempts"][-1]
    assert attempt["capture_failure"] == "not_still" and attempt["capture_attempts"] == 5
    assert "save_capture" not in w.names()


def test_under_slam_the_hold_is_let_go_and_the_control_pose_is_the_estimate_with_the_accepted_q():
    w = World(slam_mode="holding")
    out = w.run()
    assert w.names()[:3] == ["capture", "save_capture", "slam.release"]
    assert w.slam["holds"] == [{"phase": "observe", "time_s": 55.0, "event": "release_at_capture", "jump_m": 0.02,
                                "jump_rad": 0.01}]
    assert [c[1] for c in w.calls if c[0] == "slam_rear"] == [(55.0,)]
    np.testing.assert_array_equal(out.rear, [2.9, 3.6, 1.5])
    assert out.rear_stamp == 55.0 and out.yaw == 1.5
    np.testing.assert_allclose(out.base, [2.9 + 0.4 * math.cos(1.5), 3.6 + 0.4 * math.sin(1.5), 0.2])
    assert out.q[3] == pytest.approx(math.sin(math.pi / 4))  # still the accepted (truth) orientation
    assert out.attempt["control_pose_source"] == "slam_estimate" and out.attempt["slam_error"] == (0.1, 0.4, 0.07)
    assert out.attempt["capture_control"]["truth_rear"] == pytest.approx([3.0, 3.6, math.pi / 2])
    assert out.attempt["capture_control"]["control_rear"] == [2.9, 3.6, 1.5]


def test_the_hold_stays_before_the_near_capture_and_while_tracking():
    w = World(slam_mode="holding", near_capture="pending")
    w.run()
    assert "slam.release" not in w.names() and w.slam["holds"] == []
    w = World(slam_mode="tracking")
    w.run()
    assert "slam.release" not in w.names() and "slam_rear" in w.names()


@pytest.mark.parametrize("stand, status, reason", [("retry", "no_detection", "near_field_retry_no_detection"),
                                                   ("realign", "no_detection", "near_capture_realign_no_detection")])
def test_a_re_observation_stand_must_see_the_pallet(stand, status, reason):
    near = {"retry_stand": True, "stand_record": stand}
    w = World(near=near, observation=Observation(status=status))
    with pytest.raises(RuntimeError, match=reason):
        w.run()
    assert near["retry_stand"] is False and w.state["near_tracking"]["outcome"] == reason
    assert w.state["near_tracking"][stand] == {"capture_attempt": 2, "capture_status": status}
    near = {"retry_stand": True}
    w = World(near=near)
    w.run()
    assert w.state["near_tracking"]["retry"] == {"capture_attempt": 2, "capture_status": "valid"}


def test_the_rear_axle_sits_behind_the_base_along_any_heading_and_the_detector_sees_rounded_depth():
    w = World()
    yaw = 0.3
    w.capture.state.diagnostics = Diag(accepted_pose=(np.array([3.0, 4.0, 0.2]),
                                                      np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])))
    out = w.run()
    np.testing.assert_allclose(out.rear, [3.0 - 0.4 * math.cos(yaw), 4.0 - 0.4 * math.sin(yaw), yaw])
    assert [c[1] for c in w.calls if c[0] == "detector_input"] == [(1,)]


def test_a_live_stop_inside_the_capture_ends_the_work_right_after_the_raw_capture():
    """M-c (Codex round 5 P2): a STOP at a physics step inside the capture -- no SLAM release, no detection."""
    w = World(slam_mode="holding")
    w.core.hooks.live_abort = lambda: True
    out = w.run()
    assert out.aborted is True and w.names() == ["capture"]
    assert w.core.state["observation_attempts"][-1]["aborted"] == "live_stop"


def test_a_live_stop_inside_a_failing_capture_pauses_instead_of_ending_the_run():
    """Codex M-c round 6 P2."""
    w = World(fail=True)
    w.core.hooks.live_abort = lambda: True
    assert w.run().aborted is True


def waiver_world(*, check="on", deck_status="done", verdict="present", near_capture="pending"):
    """Plan 2026-10-21 task C: a capture whose detector port records the keyword it was called with."""
    w = World(near_capture=near_capture)
    kwargs_ = []

    def detect(inp, prior, params, **kw):
        kwargs_.append(kw)
        return SimpleNamespace(observation=Observation(), diagnostics=DetectionDiag())

    w.hooks.detect_pockets = detect
    w.core.options.bottom_deck_check = check
    if deck_status is not None:
        w.state["bottom_deck"] = {"status": deck_status, "leg": None, "record": {"verdict": verdict}}
    w.kwargs = kwargs_
    return w


def test_the_near_capture_after_a_present_bottom_deck_waives_the_lower_band():
    w = waiver_world()
    out = w.run()
    assert w.kwargs == [{"lower_evidence": False}]
    assert out.attempt["lower_evidence"] is False


@pytest.mark.parametrize("kw", [dict(check="off"), dict(deck_status="returning"), dict(deck_status=None),
                                dict(verdict="absent"), dict(near_capture=None), dict(near_capture="done")])
def test_no_other_capture_waives_the_lower_band(kw):
    w = waiver_world(**kw)
    out = w.run()
    assert w.kwargs == [{}]  # the detector's default, called as before
    assert "lower_evidence" not in out.attempt


def expected_world(*, mode="on", near_capture="pending", slam_mode="holding", waiver=False):
    """Plan 2026-10-22 task A: a capture whose detector ports record how they were called."""
    from forklift_core.perception.pocket_detector import ExpectedCandidate

    w = World(near_capture=near_capture, slam_mode=slam_mode)
    w.plain, w.restricted = [], []

    def detect(inp, prior, params, **kw):
        w.plain.append(kw)
        return SimpleNamespace(observation=Observation(), diagnostics=DetectionDiag())

    def detect_expected(inp, prior, params, expected, **kw):
        w.restricted.append((expected, kw))
        return (SimpleNamespace(observation=Observation(), diagnostics=DetectionDiag()),
                (ExpectedCandidate(0, (2.0, 0.1), 0.01, 0.12, 0.02, 99.0, True),))

    w.hooks.detect_pockets = detect
    w.hooks.detect_pockets_expected = detect_expected
    w.core.options.near_capture_expected = mode
    w.pocket["planning_pickup"] = SimpleNamespace(x_m=3.4, y_m=6.1, yaw_rad=1.6)
    if waiver:
        w.core.options.bottom_deck_check = "on"
        w.state["bottom_deck"] = {"status": "done", "leg": None, "record": {"verdict": "present"}}
    return w


def test_the_near_capture_looks_for_the_pallet_where_the_plan_expects_it():
    w = expected_world()
    out = w.run()
    assert w.plain == [] and len(w.restricted) == 1
    site, kw = w.restricted[0]
    assert kw == {}
    # The control base after the capture: the SLAM rear (2.9, 3.6, 1.5) and 0.4 m ahead of it.
    yaw = 1.5
    bx, by = 2.9 + 0.4 * math.cos(yaw), 3.6 + 0.4 * math.sin(yaw)
    dx, dy = 3.4 - bx, 6.1 - by
    assert site.x_m == pytest.approx(math.cos(yaw) * dx + math.sin(yaw) * dy, abs=1e-12)
    assert site.y_m == pytest.approx(-math.sin(yaw) * dx + math.cos(yaw) * dy, abs=1e-12)
    assert site.yaw_rad == pytest.approx(0.1, abs=1e-12)
    assert (site.radius_m, site.axis_tolerance_rad) == (0.20, pytest.approx(math.radians(15.0)))
    record = out.attempt["expected"]
    assert record["site"] == [site.x_m, site.y_m, site.yaw_rad]
    assert record["radius_m"] == 0.20 and record["candidates"][0]["inside"] is True
    assert record["planning_pickup"] == [3.4, 6.1, 1.6]


def test_the_expected_site_follows_the_bottom_deck_waiver():
    w = expected_world(waiver=True)
    out = w.run()
    assert w.restricted[0][1] == {"lower_evidence": False} and out.attempt["lower_evidence"] is False


@pytest.mark.parametrize("kw", [dict(mode="off"), dict(near_capture=None), dict(near_capture="done")])
def test_no_other_capture_is_restricted_and_off_calls_the_detector_as_before(kw):
    w = expected_world(**kw)
    out = w.run()
    assert w.restricted == [] and w.plain == [{}]
    assert "expected" not in out.attempt


def test_an_options_object_without_the_switch_is_off():
    w = expected_world()
    del w.core.options.near_capture_expected
    w.run()
    assert w.restricted == [] and w.plain == [{}]


# --- plan 2026-10-22 task C: the near lower band waived with the check off (user decision, conditional) ---------

FAR_ESTIMATE = {"x_m": 3.4, "y_m": 6.1, "yaw_rad": 1.6}


@dataclass
class Seen:
    status: str
    reason: str | None = None
    insertion_yaw_rad: float = 0.02


def waiver_c_world(*, first="no_pallet", second="valid", far=None, near_capture="pending", lower="waive_far_confirmed",
                   inside=True):
    """A capture whose restricted detector is valid only with the lower band waived (or as given)."""
    from forklift_core.perception.pocket_detector import ExpectedCandidate

    w = World(near_capture=near_capture, slam_mode="holding")
    w.calls_c = []

    def detect(inp, prior, params, **kw):
        w.calls_c.append(("plain", None, kw))
        return SimpleNamespace(observation=Seen("valid"), diagnostics=DetectionDiag())

    def detect_expected(inp, prior, params, expected, **kw):
        w.calls_c.append(("restricted", expected, kw))
        status = second if kw.get("lower_evidence") is False else first
        reason = None if status == "valid" else "no_opening_pattern"
        return (SimpleNamespace(observation=Seen(status, reason), diagnostics=DetectionDiag()),
                (ExpectedCandidate(0, (2.0, 0.1), 0.01, 0.12 if inside else 0.18, 0.02, 99.0, True),))

    w.hooks.detect_pockets = detect
    w.hooks.detect_pockets_expected = detect_expected
    w.core.options.near_capture_expected = "on"
    w.core.options.near_capture_lower = lower
    w.core.options.adapter.estimate_pallet_center_m = lambda obs, depth: (2.5, 0.1)
    w.core.options.adapter.estimate_world_pallet_site = lambda xy, yaw, base_xy, base_yaw: SimpleNamespace(
        x_m=base_xy[0] + xy[0], y_m=base_xy[1] + xy[1], yaw_rad=base_yaw + yaw)
    w.pocket["planning_pickup"] = SimpleNamespace(**FAR_ESTIMATE)
    if near_capture == "pending":
        w.state["near_capture"].update(far_attempt=1, far_estimate_m=dict(FAR_ESTIMATE))
        record = {"pocket_observation": {"status": "valid"}, "detector_site": [3.42, 6.08, 1.61]}
        record.update(far or {})
        w.state["observation_attempts"][0] = record
    return w


def test_the_near_lower_band_is_waived_once_the_far_capture_passed_in_full_and_the_record_says_why():
    w = waiver_c_world()
    out = w.run()
    first, second = [c for c in w.calls_c if c[0] == "restricted"]
    assert first[2] == {} and second[2] == {"lower_evidence": False}
    assert (second[1].radius_m, second[1].axis_tolerance_rad) == (0.15, pytest.approx(math.radians(15.0)))
    assert (first[1].radius_m, second[1].x_m, second[1].y_m) == (0.20, first[1].x_m, first[1].y_m)
    assert out.observation.status == "valid"
    waiver = out.attempt["lower_waiver"]
    assert waiver["applied"] is True and waiver["far_attempt"] == 1
    assert waiver["far_detector_status"] == "valid" and waiver["far_link_m"] == pytest.approx(math.hypot(0.02, 0.02))
    assert (waiver["first"]["status"], waiver["first"]["reason"]) == ("no_pallet", "no_opening_pattern")
    assert waiver["first"]["expected"]["radius_m"] == 0.20  # the first pass, kept beside the waived one
    assert out.attempt["lower_evidence"] is False and out.attempt["expected"]["radius_m"] == 0.15


def test_a_near_capture_valid_on_the_first_pass_waives_nothing():
    w = waiver_c_world(first="valid")
    out = w.run()
    assert len([c for c in w.calls_c if c[0] == "restricted"]) == 1
    assert "lower_waiver" not in out.attempt and "lower_evidence" not in out.attempt


@pytest.mark.parametrize("far, why", [
    ({"pocket_observation": {"status": "no_pallet"}}, "far_detector:no_pallet"),
    ({"detector_reference": {"status": "invalid"}}, "far_detector:invalid"),
    ({"lower_evidence": False}, "far_lower_waived"),
    ({"expected": {"site": [0, 0, 0]}}, "far_restricted"),
    ({"detector_site": None}, "far_detector_site_missing"),
    ({"detector_site": [3.6, 6.1, 1.6]}, "far_link"),
    ({"detector_site": [3.4, 6.1, 1.95]}, "far_link"),
])
def test_each_far_condition_refuses_the_waiver_by_name(far, why):
    w = waiver_c_world(far=far)
    out = w.run()
    assert len([c for c in w.calls_c if c[0] == "restricted"]) == 1
    waiver = out.attempt["lower_waiver"]
    assert waiver["applied"] is False and waiver["why"] == why
    assert out.observation.status == "no_pallet" and "lower_evidence" not in out.attempt


def test_a_second_pass_without_a_valid_candidate_keeps_the_first_result():
    w = waiver_c_world(second="no_pallet")
    out = w.run()
    waiver = out.attempt["lower_waiver"]
    assert waiver["applied"] is False and waiver["why"] == "second:no_pallet:no_opening_pattern"
    assert out.observation.status == "no_pallet" and "lower_evidence" not in out.attempt
    assert out.attempt["expected"]["radius_m"] == 0.20


def test_the_far_capture_records_its_detector_site_only_with_the_waiver_option():
    w = waiver_c_world(near_capture=None)
    out = w.run()
    # the SLAM base (2.9, 3.6, 1.5) moved 0.4 m ahead, plus the stub's (2.5, 0.1) offset
    bx, by = 2.9 + 0.4 * math.cos(1.5), 3.6 + 0.4 * math.sin(1.5)
    assert out.attempt["detector_site"] == pytest.approx([bx + 2.5, by + 0.1, 1.52])
    w = waiver_c_world(near_capture=None, lower="gate")
    assert "detector_site" not in w.run().attempt


def test_the_gate_default_never_waives():
    w = waiver_c_world(lower="gate")
    out = w.run()
    assert len([c for c in w.calls_c if c[0] == "restricted"]) == 1
    assert "lower_waiver" not in out.attempt and out.observation.status == "no_pallet"


def test_the_check_on_waiver_is_not_this_one():
    w = waiver_c_world()
    w.core.options.bottom_deck_check = "on"
    w.state["bottom_deck"] = {"status": "done", "leg": None, "record": {"verdict": "present"}}
    out = w.run()
    (_, _, kw), = [c for c in w.calls_c if c[0] == "restricted"]
    assert kw == {"lower_evidence": False} and "lower_waiver" not in out.attempt
