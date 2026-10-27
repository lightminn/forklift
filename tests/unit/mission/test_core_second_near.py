"""C3, the second near observation on the approach straight (docs/plans/2026-10-27-insertion-lateral-drift.md, T2
사전 등록): the split at the gap, the capture with the hold kept and no far classification, the atomic adoption,
each refusal keeping the first state, the record, and the option off leaving the approach untouched."""

import math
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control import RearAxlePathTracker, TrackerConfig
from forklift_core.mission import second_near as SN
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.planning import Pose2D
from forklift_core.planning.pallet_mission import (
    PalletSite,
    SyntheticMissionGeometry,
    final_straight_prefix,
    site_poses,
    straight_from_pose,
)

PRIOR = load_pallet_prior(Path(__file__).resolve().parents[3] / "config/pallet_prior_epal6.yaml")
GEOMETRY = SyntheticMissionGeometry(alignment_straight_m=2.1)
CONFIG = TrackerConfig(cruise_speed_mps=0.6, max_curvature_inv_m=0.36, lookahead_m=0.28, position_tolerance_m=0.008,
                       yaw_tolerance_rad=0.02, stop_speed_mps=0.012, max_cross_track_error_m=0.35)
PICKUP = PalletSite(5.0, 0.0, 0.0)


def straight(site=PICKUP):
    poses = site_poses(site, GEOMETRY)
    path, _ = straight_from_pose(poses["prealign"], poses["prealign"], poses["approach"], max_lateral_m=0.05,
                                 max_yaw_rad=0.05, min_length_m=0.3)
    return path


def docking(site=PICKUP):
    poses = site_poses(site, GEOMETRY)
    insert, _ = straight_from_pose(poses["approach"], poses["approach"], poses["inserted"], max_lateral_m=0.05,
                                   max_yaw_rad=0.05, min_length_m=0.0)
    return insert


class CaptureFailure(Exception):
    pass


@dataclass
class Observation:
    status: str = "valid"
    reason: str | None = None
    insertion_yaw_rad: float = 0.0


@dataclass
class Diag:
    accepted_pose: tuple
    attempts: int = 1


@dataclass
class Frame:
    ok: bool = True


@dataclass
class DetectionDiag:
    n: int = 1


@dataclass
class Dry:
    status: str = "arrived"
    position_error_m: float = 0.002
    yaw_error_rad: float = 0.001
    time_s: float = 1.0
    trajectory: tuple = ()


class World:
    """A MissionCore standing at the first near capture's adoption, with the approach straight installed."""

    def __init__(self, *, gap=1.0, observed_site=PalletSite(5.0, 0.02, 0.0), observation=Observation(), dry=Dry(),
                 slam_mode="holding"):
        self.calls = []
        log_ = self.log
        self.clock = 60.0
        q = np.array([1.0, 0.0, 0.0, 0.0])
        self.stand = np.array([2.8, 0.0, 0.0])  # the rear where the split straight ends (filled by the test)

        def capture(max_attempts):
            log_("capture")
            return SimpleNamespace(depth_m=np.zeros((2, 2)), base_from_optical=SimpleNamespace(
                translation_m=(0.6, 0.0, 0.27), rotation=np.eye(3))), Frame(), 1

        diag = Diag(accepted_pose=(np.array([3.14, 0.0, 0.2]), q))
        self.capture = SimpleNamespace(capture=capture, state=SimpleNamespace(diagnostics=diag))

        def require(ok, reason):
            if not ok:
                raise RuntimeError(reason)

        tracker = SimpleNamespace(mode=slam_mode, release=lambda **k: log_("slam.release") or (0.0, 0.0))
        self.slam = {"tracker": tracker, "holds": []}
        self.far = []
        self.hooks = SimpleNamespace(
            require=require, perception_capture=lambda: self.capture, slam_odom_base=lambda: "ODOM",
            slam_rear=lambda now: self.stand.copy(), slam_error=lambda: (0.0, 0.0, 0.0),
            detect_pockets=lambda *a, **k: log_("detect_pockets", k) or SimpleNamespace(
                observation=observation, diagnostics=DetectionDiag()),
            detect_pockets_expected=lambda inp, prior, params, site, **k: log_("detect_expected", site, k) or (
                SimpleNamespace(observation=observation, diagnostics=DetectionDiag()), []),
            far_recognition=lambda *a: self.far.append(a),
            save_capture=lambda number, scene: None, now_s=lambda: self.clock,
            planning_target=lambda target: (target, "perception"),
            straight_from_pose=straight_from_pose, final_straight_prefix=final_straight_prefix,
            bicycle_rollout=lambda *a, **k: log_("dry_run") or dry,
            write_paths=lambda: log_("write_paths"), path_record=lambda p: {"length_m": float(p.length_m)},
            show_path=lambda *a: None,
        )
        adapter = SimpleNamespace(
            CaptureFailure=CaptureFailure, detector_input=lambda scene, mm: "INPUT",
            estimate_pallet_center_m=lambda obs, depth: "CENTRE",
            estimate_world_pallet_site=lambda centre, yaw, base_xy, base_yaw: observed_site)
        self.ms = MissionState(phase="approach", phase_started=50.0, next_candidate_index=1)
        self.state = {"transitions": [], "observation_attempts": [{}], "observation_waypoint_selected": (0, 0, 0),
                      "near_capture": {"status": "done", "far_attempt": 1}, "paths": {}}
        insert = docking()
        self.paths = {"approach": straight(), "insert": insert, "extract": insert}
        self.trackers = {name: RearAxlePathTracker(p.poses, p.directions, p.curvatures_inv_m, CONFIG)
                         for name, p in self.paths.items()}
        self.pocket = {"check": None, "planning_pickup": PICKUP}
        self.core = MissionCore(
            self.ms, state=self.state, obstacle=None, slam=self.slam, pocket=self.pocket, paths=self.paths,
            trackers=self.trackers, geometry=GEOMETRY, settings={}, pallet_geometry=None, hooks=self.hooks, near=None,
            options=SimpleNamespace(
                adapter=adapter, G2=SimpleNamespace(depth_sha256=lambda d: "SHA"),
                yaw_and_tilt=lambda q: (0.0, 0.0), perception_max_attempts=4, rear_axle_offset_m=-0.34,
                pallet_prior_loaded=PRIOR, depth_quantize_mm=1, near_capture_expected="on",
                near_capture_lower="waive", near_capture_box_yaw_rad=0.05, approach_straight_speed_mps=None,
                near_recapture_gap_m=gap))

    def log(self, name, *args):
        self.calls.append((name, args))

    def names(self):
        return [c[0] for c in self.calls]

    def split(self):
        return self.core.second_near_split(self.clock, self.paths["approach"], CONFIG)


def test_option_off_leaves_the_approach_whole():
    w = World(gap=None)
    full = w.trackers["approach"]
    w.split()
    assert w.trackers["approach"] is full
    assert "second" not in w.state["near_capture"]
    assert not w.core.second_near_due()


def test_the_split_stops_the_follower_where_the_tip_is_the_gap_before_the_face():
    w = World()
    w.split()
    second = w.state["near_capture"]["second"]
    assert second["status"] == "pending" and second["gap_m"] == 1.0
    end = w.trackers["approach"].path_arrays()[0][-1]
    approach_end = site_poses(PICKUP, GEOMETRY)["approach"]
    left = approach_end.x_m - end[0]  # the straight runs along +x
    assert left == pytest.approx(SN.keep_m(1.0, GEOMETRY.approach_gap_m), abs=0.05)
    assert left >= SN.keep_m(1.0, GEOMETRY.approach_gap_m) - 1e-6  # the stop is at or before the gap point
    assert w.core.second_near_due()


def test_a_straight_too_short_for_the_gap_is_skipped_and_kept_whole():
    w = World(gap=3.0)  # 2.9 m of straight needed, 2.1 m there
    full = w.trackers["approach"]
    w.split()
    assert w.state["near_capture"]["second"]["status"] == "skipped_short"
    assert w.trackers["approach"] is full
    assert not w.core.second_near_due()


def stand_at_split(w):
    w.split()
    end = w.trackers["approach"].path_arrays()[0][-1]
    w.stand = np.array(end, dtype=float)


def test_the_capture_keeps_the_hold_uses_the_expected_site_and_is_not_a_far_capture():
    w = World()
    stand_at_split(w)
    w.core.second_near_capture(w.clock, w.stand, 900)
    assert "slam.release" not in w.names()
    assert w.far == []
    expected = [c for c in w.calls if c[0] == "detect_expected"]
    assert expected and expected[0][1][1] == {"lower_evidence": False}  # the run's lower mode (waive)
    assert w.slam["holds"] == []


def test_an_accepted_redraw_is_adopted_with_the_pickup_and_the_docking_paths_together():
    w = World(observed_site=PalletSite(5.0, 0.02, 0.0))
    stand_at_split(w)
    old_insert = w.paths["insert"]
    out = w.core.second_near_capture(w.clock, w.stand, 900)
    second = w.state["near_capture"]["second"]
    assert second["status"] == "adopted"
    assert w.pocket["planning_pickup"].y_m == pytest.approx(0.02)
    assert w.paths["insert"] is not old_insert
    assert np.allclose(w.paths["insert"].poses[:, 1], old_insert.poses[:, 1] + 0.02)
    assert w.paths["approach"].poses[-1][1] == pytest.approx(0.02)
    for name in ("approach", "insert", "extract"):
        assert np.allclose(w.trackers[name].path_arrays()[0], w.paths[name].poses)
    assert w.ms.phase == "approach" and w.ms.phase_started == w.clock
    assert not w.core.second_near_due()
    assert out.rear is not None
    for key in ("reason", "offsets", "dry_run", "capture_attempt", "capture_time_s", "stop_s", "target",
                "previous_target"):
        assert key in second


@pytest.mark.parametrize("case, status", [
    ("no_detection", "no_detection"),
    ("box", "box_refused"),
    ("dry", "dry_run_refused"),
])
def test_a_refusal_keeps_the_first_pickup_paths_and_followers(case, status):
    kwargs = {}
    if case == "no_detection":
        kwargs["observation"] = Observation(status="invalid", reason="no_pallet")
    elif case == "box":
        kwargs["observed_site"] = PalletSite(5.0, 0.20, 0.0)  # 20 cm off the line: outside the 5 cm box
    else:
        kwargs["dry"] = Dry(position_error_m=0.0079)
    w = World(**kwargs)
    stand_at_split(w)
    before = {name: w.paths[name] for name in ("insert", "extract")}
    trackers = {name: w.trackers[name] for name in ("insert", "extract")}
    w.core.second_near_capture(w.clock, w.stand, 900)
    assert w.state["near_capture"]["second"]["status"] == status
    assert w.pocket["planning_pickup"] is PICKUP
    for name in ("insert", "extract"):
        assert w.paths[name] is before[name] and w.trackers[name] is trackers[name]
    # The approach resumes on the first line, from where the truck stands, to the first approach end.
    first_end = site_poses(PICKUP, GEOMETRY)["approach"]
    resumed = w.trackers["approach"].path_arrays()[0]
    assert resumed[-1][0] == pytest.approx(first_end.x_m) and resumed[-1][1] == pytest.approx(first_end.y_m)
    assert resumed[0][0] == pytest.approx(w.stand[0], abs=1e-6)
    assert not w.core.second_near_due()


def test_moved_path_carries_a_docking_path_with_the_site():
    old, new = PalletSite(5.0, 0.0, 0.0), PalletSite(5.1, -0.03, math.radians(1.0))
    moved = SN.moved_path(docking(old), old, new)
    expected = site_poses(new, GEOMETRY)
    assert moved.poses[0][:2] == pytest.approx([expected["approach"].x_m, expected["approach"].y_m], abs=1e-9)
    assert moved.poses[-1][:2] == pytest.approx([expected["inserted"].x_m, expected["inserted"].y_m], abs=1e-9)
