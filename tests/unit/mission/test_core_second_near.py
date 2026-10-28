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
from forklift_core.control.rollout import bicycle_rollout as real_rollout
from forklift_core.planning import Bounds, Rectangle
from forklift_core.planning.pallet_mission import (
    PalletSite,
    PlacedProp,
    SyntheticMissionGeometry,
    TransportScenario,
    final_straight_prefix,
    redraw_docking,
    site_poses,
    straight_from_pose,
)

PRIOR = load_pallet_prior(Path(__file__).resolve().parents[3] / "config/pallet_prior_epal6.yaml")
GEOMETRY = SyntheticMissionGeometry(alignment_straight_m=2.1)
CONFIG = TrackerConfig(cruise_speed_mps=0.6, max_curvature_inv_m=0.36, lookahead_m=0.28, position_tolerance_m=0.008,
                       yaw_tolerance_rad=0.02, stop_speed_mps=0.012, max_cross_track_error_m=0.35)
PICKUP = PalletSite(5.0, 0.0, 0.0)


def scenario(*rects):
    props = tuple(PlacedProp(SimpleNamespace(uri="post", height_m=1.0), r) for r in rects)
    return TransportScenario(0, Pose2D(0.0, 0.0, 0.0), PICKUP, PalletSite(12.0, 0.0, 0.0), props,
                             Bounds(-5.0, 15.0, -5.0, 5.0))


def redraw_docking_traced(scenario_, rear_pose, target_, config_, *, start_rear, dry_configs, **kwargs):
    """The runner's port, with real rollouts (sim/isaac/run_transport.py redraw_docking_traced)."""
    start_ = np.asarray(start_rear, dtype=float)
    dries = tuple((lambda c_: lambda st_: real_rollout(st_.poses, st_.directions, st_.curvatures_inv_m, c_, start_))(c)
                  for c in dry_configs)
    plans_, record_ = redraw_docking(scenario_, rear_pose, target_, config_, dry_runs=dries, **kwargs)
    return plans_, record_, []


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
                 slam_mode="holding", rollout=None, props=()):
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
            # The split's dry run first (arrives), then the second capture's (``dry``); or a real rollout.
            bicycle_rollout=rollout or (lambda *a, **k: (Dry() if self.splitting() else dry)),
            write_paths=lambda: log_("write_paths"), path_record=lambda p: {"length_m": float(p.length_m)},
            show_path=lambda *a: None, grid_world=lambda scenario_: scenario_, correction_grid=lambda *a: {},
            redraw_docking_traced=redraw_docking_traced,
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
                near_recapture_gap_m=gap, scenario=scenario(*props), planner_config=None, pickup_bounds=None))

    def splitting(self):
        return "second" not in self.state["near_capture"]

    def log(self, name, *args):
        self.calls.append((name, args))

    def names(self):
        return [c[0] for c in self.calls]

    def split(self, rear=None):
        start = self.paths["approach"].poses[0] if rear is None else rear
        return self.core.second_near_split(self.clock, np.asarray(start, dtype=float), self.paths["approach"], CONFIG)


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
    new_sites = site_poses(PalletSite(5.0, 0.02, 0.0), GEOMETRY)
    assert w.paths["insert"].poses[0][:2] == pytest.approx([new_sites["approach"].x_m, new_sites["approach"].y_m])
    assert w.paths["insert"].poses[-1][:2] == pytest.approx([new_sites["inserted"].x_m, new_sites["inserted"].y_m])
    assert second["docking"]["refused"] is None
    assert w.paths["approach"].poses[-1][1] == pytest.approx(0.02)
    for name in ("approach", "insert", "extract"):
        assert np.allclose(w.trackers[name].path_arrays()[0], w.paths[name].poses)
    assert w.ms.phase == "approach" and w.ms.phase_started == w.clock
    assert not w.core.second_near_due()
    assert out.rear is not None
    for key in ("reason", "offsets", "dry_run", "docking", "capture_attempt", "capture_time_s", "stop_s", "target",
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


def test_a_shortened_straight_the_follower_cannot_reach_keeps_the_whole_straight():
    # Codex C3 review P2: from 9.9 cm and -0.0595 rad off the line (inside the untracked re-draw's 0.10 m box) the
    # whole 2.1 m straight converges, but the 1.19 m cut for g = 1.0 ends in an endpoint heading failure.
    from forklift_core.control.rollout import bicycle_rollout

    riib = TrackerConfig(cruise_speed_mps=0.6, max_curvature_inv_m=0.36, max_acceleration_mps2=0.3, lookahead_m=0.28,
                         position_tolerance_m=0.008, yaw_tolerance_rad=0.02, stop_speed_mps=0.012,
                         max_cross_track_error_m=0.35, cusp_position_tolerance_m=0.03, cusp_yaw_tolerance_rad=0.05,
                         cusp_brake_window_m=0.008, overshoot_tolerance_m=0.03)
    w = World(rollout=bicycle_rollout)
    poses = site_poses(PICKUP, GEOMETRY)
    rear = np.array([poses["prealign"].x_m, poses["prealign"].y_m - 0.099, -0.0595])
    whole, _ = straight_from_pose(Pose2D(*rear), poses["prealign"], poses["approach"], max_lateral_m=0.10,
                                  max_yaw_rad=0.12, min_length_m=0.3)
    assert bicycle_rollout(whole.poses, whole.directions, whole.curvatures_inv_m, riib, rear).status == "arrived"
    w.paths["approach"] = whole
    w.trackers["approach"] = full = RearAxlePathTracker(whole.poses, whole.directions, whole.curvatures_inv_m, riib)
    w.core.second_near_split(w.clock, rear, whole, riib)
    second = w.state["near_capture"]["second"]
    assert second["status"] == "skipped_prefix" and second["split_dry_run"]["status"] != "arrived"
    assert w.trackers["approach"] is full
    assert not w.core.second_near_due()


def test_a_timed_out_dry_run_is_refused_and_recorded_as_strict_json():
    import json

    w = World(dry=Dry(status="timeout", position_error_m=float("nan"), yaw_error_rad=float("nan")))
    stand_at_split(w)
    w.core.second_near_capture(w.clock, w.stand, 900)
    second = w.state["near_capture"]["second"]
    assert second["status"] == "dry_run_refused"
    assert second["dry_run"] == {"status": "timeout", "position_error_m": None, "yaw_error_rad": None}
    json.dumps(w.state["near_capture"], allow_nan=False)


def test_a_redraw_whose_extraction_meets_a_prop_is_refused_and_keeps_the_first_state():
    # Codex C3 review round 2 P1: target (5, 0, 0) -> (5, 0.149, 0.049) passes the plan's box and dry run from the
    # gap, but its docking meets a post at (4.6, 0.555) (Codex: the loaded extraction; redraw_docking already
    # refuses its approach there) -- carried rigidly it was adopted, drawn and checked it is refused.
    w = World(observed_site=PalletSite(5.0, 0.149, 0.049), props=(Rectangle(4.6, 0.555, 0.04, 0.06),),
              rollout=real_rollout)
    stand_at_split(w)
    before = {name: w.paths[name] for name in ("insert", "extract")}
    w.core.second_near_capture(w.clock, w.stand, 900)
    second = w.state["near_capture"]["second"]
    assert second["dry_run"]["status"] == "arrived"  # the plan's own acceptance passed
    assert second["status"] == "docking_refused" and second["docking"]["refused"] is not None
    assert w.pocket["planning_pickup"] is PICKUP
    assert all(w.paths[name] is before[name] for name in before)


# Plan 2026-10-28-adoption-zone-check: the second near capture's pickup is checked only when it is adopted.
def zoned(w, zone):
    from forklift_core.mission.adoption_zone import AdoptionZone

    w.core.options.adoption_zone_check = "on"
    w.core.options.adoption_zone_margin_m = 0.25
    w.core.options.adoption_zone = AdoptionZone(*zone, "scene_layout")
    return w


def test_an_adopted_second_capture_inside_the_zone_is_checked():
    w = zoned(World(observed_site=PalletSite(5.0, 0.02, 0.0)), (4.5, 5.5, -0.5, 0.5))
    stand_at_split(w)
    w.core.second_near_capture(w.clock, w.stand, 900)
    assert w.state["near_capture"]["second"]["status"] == "adopted"
    (check_,) = w.state["adoption_zone"]["checks"]
    assert check_["stage"] == "second_near" and check_["inside"] and check_["site"] == pytest.approx([5.0, 0.02])


def test_an_adopted_second_capture_outside_the_zone_ends_the_mission_before_it_replaces_anything():
    w = zoned(World(observed_site=PalletSite(5.0, 0.02, 0.0)), (0.0, 1.0, -0.5, 0.5))
    stand_at_split(w)
    before = {name: w.paths[name] for name in ("insert", "extract")}
    with pytest.raises(RuntimeError, match="^adoption_outside_zone:second_near$"):
        w.core.second_near_capture(w.clock, w.stand, 900)
    assert w.pocket["planning_pickup"] is PICKUP and all(w.paths[n] is before[n] for n in before)


def test_a_second_capture_that_is_not_adopted_is_not_checked():
    w = zoned(World(observed_site=PalletSite(5.0, 0.20, 0.0)), (0.0, 1.0, -0.5, 0.5))  # box refusal, far outside
    stand_at_split(w)
    w.core.second_near_capture(w.clock, w.stand, 900)
    assert w.state["near_capture"]["second"]["status"] == "box_refused" and "adoption_zone" not in w.state
