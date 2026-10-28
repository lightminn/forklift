"""B-P2b: the runner's start relocalization (docs/plans/2026-10-29-start-relocalization.md 1.1, 1.2, 1.4) --
``--start-pose`` / ``--start-relocalize`` / ``--reloc-dir`` (off by default, not recorded off, in the near-field and
replay contracts), where the truck is placed and where the odometry, the obstacle layer and the bridge's start check
begin, the phase's failures as session failures, and that the localization path never reads the placed pose."""

import math
import re
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))
sys.path.insert(0, str(ROOT / "tests/unit/localization"))

import scan_capture as C  # noqa: E402
import test_global_match as TG  # noqa: E402
from test_reloc_phase import FakeTruck  # noqa: E402
from test_riibotics_isaac_scenes import _runner_args  # noqa: E402

from forklift_core.localization import global_match as G  # noqa: E402
from forklift_core.localization import start_reloc as R  # noqa: E402
from forklift_core.localization.slam_map import MapBundle  # noqa: E402

SCRIPT = ROOT / "sim/isaac/run_transport.py"
NEW = ("start_pose", "start_relocalize", "reloc_dir")
ANGLES = (0.0, 2 * math.pi / 360)  # the fake LiDAR's


def _live(tmp_path, *extra):
    return ["--live", "--slam-feedback", str(tmp_path / "s.sock"), "--slam-map-bundle", str(tmp_path),
            "--expected-map-id", "a" * 64, *extra]


def _reloc(tmp_path, pose="1.0,2.0,0.5"):
    return ["--start-pose", pose, "--start-relocalize", "--reloc-dir", str(tmp_path)]


def test_the_start_relocalization_is_off_by_default_and_not_recorded_off(tmp_path):
    import run_transport

    args = _runner_args(_live(tmp_path))
    assert (args.start_pose, args.start_relocalize, args.reloc_dir) == (None, False, None)
    assert not set(NEW) & set(run_transport.recorded_arguments(args))  # an off run records what it always did


def test_a_relocalizing_live_run_takes_its_placed_pose(tmp_path):
    import run_transport

    args = _runner_args(_live(tmp_path, *_reloc(tmp_path)))
    assert args.start_pose == (1.0, 2.0, 0.5) and args.start_relocalize and args.reloc_dir == tmp_path
    assert set(NEW) <= set(run_transport.recorded_arguments(args))


@pytest.mark.parametrize("extra", [
    ["--start-pose", "1,2,0.5"],  # the placed pose only with the relocalization
    ["--start-relocalize", "--reloc-dir", "d"],  # no placed pose
    ["--start-pose", "1,2,0.5", "--start-relocalize"],  # no epoch folder
    ["--reloc-dir", "d"],
    ["--start-pose", "1,2", "--start-relocalize", "--reloc-dir", "d"],
    ["--start-pose", "1,2,nan", "--start-relocalize", "--reloc-dir", "d"],
    ["--start-pose", "1,inf,0", "--start-relocalize", "--reloc-dir", "d"],
])
def test_the_start_arguments_are_refused_out_of_place(tmp_path, extra):
    with pytest.raises(SystemExit):
        _runner_args(_live(tmp_path, *extra))


def test_a_relocalizing_run_is_live_on_a_survey_map_with_the_shared_layer(tmp_path):
    for argv in (_reloc(tmp_path),  # not live
                 ["--live", "--slam-feedback", str(tmp_path / "s.sock"), *_reloc(tmp_path)],  # online mapping
                 _live(tmp_path, *_reloc(tmp_path), "--no-layer")):  # no obstacle layer (its LiDAR and noise)
        with pytest.raises(SystemExit):
            _runner_args(argv)


def test_the_near_field_and_replay_contracts_know_the_new_arguments():
    import near_tracking

    from tools import boundary_replay

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["start_pose"] is None
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["start_relocalize"] is False
    assert "reloc_dir" in near_tracking.RUN_SPECIFIC_ARGUMENTS  # the epoch folder is the run's own
    assert boundary_replay.SETUP_NAMES_ADDED["start_reloc"] == ("start_relocalize", False, None)


def _scenario(rear=(0.5, -1.0, 0.3)):
    return SimpleNamespace(start_rear=SimpleNamespace(x_m=rear[0], y_m=rear[1], yaw_rad=rear[2]))


def test_the_truck_is_placed_at_the_start_pose_or_else_at_the_scenes_start():
    import run_transport

    scenario = _scenario()
    off = SimpleNamespace(start_pose=None, rear_axle_offset_m=-0.34)
    x, y, yaw = run_transport.spawn_base_pose(off, scenario)
    assert (x, y, yaw) == pytest.approx((0.5 + 0.34 * math.cos(0.3), -1.0 + 0.34 * math.sin(0.3), 0.3))
    on = SimpleNamespace(start_pose=(4.0, 3.0, -2.0), rear_axle_offset_m=-0.34)
    assert run_transport.spawn_base_pose(on, scenario) == (4.0, 3.0, -2.0)


def test_the_map_start_is_the_estimate_of_a_relocalization_or_else_the_scenes_start():
    import run_transport

    scenario = _scenario()
    assert run_transport.map_start_rear(scenario, None) == (0.5, -1.0, 0.3)
    assert run_transport.map_start_rear(scenario, {"estimate_rear": [7.0, 8.0, 1.0]}) == (7.0, 8.0, 1.0)


def _source(start: str, end: str) -> str:
    text = SCRIPT.read_text()
    return text[text.index(start):text.index(end)]


def test_the_localization_path_never_reads_the_placed_pose():
    run_ = _source("\ndef run(app,", "\ndef drive(S, isaac)")
    drive_ = _source("\ndef drive(S, isaac)", "\ndef recorded_arguments(")
    assert "start_pose" not in drive_.replace("map_start_pose", "")  # the drive starts from map_start_rear only
    assert len(re.findall(r"args\.start_pose\b", run_)) == 0  # the setup places by spawn_base_pose...
    helper = _source("\ndef start_relocalization(", "\ndef run(app,")
    assert len(re.findall(r"args\.start_pose\b", helper)) == 1  # ...and the phase reads it for validity only
    assert "target=args.start_pose" in helper.replace(" ", "")


def test_the_setup_and_the_drive_start_from_map_start_rear():
    run_ = _source("\ndef run(app,", "\ndef drive(S, isaac)")
    drive_ = _source("\ndef drive(S, isaac)", "\ndef recorded_arguments(")
    assert "start_pose=map_start_rear(scenario,start_reloc)" in run_.replace(" ", "")  # the obstacle layer's prime
    assert "odom_rear=start_pose," in run_.replace(" ", "")
    assert "slam_start=map_start_rear(scenario,start_reloc)" in drive_.replace(" ", "")  # odometry, start check
    assert "base_start=spawn_base_pose(args, scenario)" not in run_  # (it builds the array from it)
    assert "spawn_base_pose(args, scenario)" in run_
    # every new call of the setup is behind the option (plan 1.4): the phase runs only when it is on
    assert re.search(r"\n(\s+)if args\.start_relocalize:\n\1    start_reloc = start_relocalization\(", run_)


def _bundle(grid, mfw=(0.0, 0.0, 0.0)):
    return MapBundle(folder=Path("b"), map_id="m" * 64, manifest={"map_from_world": list(mfw)}, grid=grid)


def _phase_args(tmp_path, base):
    return SimpleNamespace(start_pose=base, reloc_dir=tmp_path, seed=7, rear_axle_offset_m=-0.34)


def _engine(truck):
    return C.Engine(teleport=None, step=truck.step, pose=truck.pose, cast=truck.cast,
                    draw_noise=lambda rng, d: np.zeros(len(d)), ranges_from_hits=lambda d, h: d,
                    range_min_m=0.2, range_max_m=12.0, settle_z_m=0.1, laser_xyz_m=(-0.12, 0.0, 1.05))


def _frozen():
    return (G.MatchConfig(yaw_step_rad=math.radians(2.0)), R.RelocConfig(e_min=0.8, r_max=0.9, r_struct=0.8),
            {"match_config": {"sha256": "x"}})


def test_a_chosen_estimate_is_the_runs_map_start(tmp_path):
    import run_transport
    from test_reloc_phase import _go_later

    grid = TG._grid(*TG.ROOM)
    base = (2.42, 1.7, 0.6)
    _go_later(tmp_path)
    state = {}
    rec = run_transport.start_relocalization(_phase_args(tmp_path, base), state, _engine(FakeTruck(grid, base)),
                                             ANGLES, _bundle(grid), frozen=_frozen(), pace_s=0.0)
    assert rec is state["start_reloc"] and rec["status"] == "chosen" and rec["map_id"] == "m" * 64
    est = rec["estimate"]
    assert math.hypot(est[0] - base[0], est[1] - base[1]) <= 0.10
    assert rec["estimate_rear"] == pytest.approx(list(R.rear_from_base(est, 0.34)))
    assert rec["settings"] == {"match_config": {"sha256": "x"}}


def test_a_start_that_did_not_settle_ends_the_session_before_any_matching(tmp_path):
    import run_transport

    grid = TG._grid(*TG.ROOM)
    base = (2.42, 1.7, 0.6)
    state = {}
    with pytest.raises(run_transport.SessionFailure, match="^reloc_invalid_input: not_at_target"):
        run_transport.start_relocalization(_phase_args(tmp_path, base), state,
                                           _engine(FakeTruck(grid, base, drift=(0.05, 0.0))), ANGLES, _bundle(grid),
                                           frozen=_frozen(), pace_s=0.0)
    assert state["start_reloc"]["status"] == "invalid" and state["start_reloc"]["attempts"] == []


def test_a_bundle_whose_map_is_not_the_world_is_refused(tmp_path):
    import run_transport

    grid = TG._grid(*TG.ROOM)
    with pytest.raises(run_transport.SessionFailure, match="map_from_world"):
        run_transport.start_relocalization(_phase_args(tmp_path, (2.42, 1.7, 0.6)), {},
                                           _engine(FakeTruck(grid, (2.42, 1.7, 0.6))), ANGLES,
                                           _bundle(grid, (0.1, 0, 0)),
                                           frozen=_frozen(), pace_s=0.0)


def test_each_attempt_has_its_own_seeded_noise():
    import run_transport

    a = run_transport.reloc_noise(7)(0, 1).standard_normal(3)
    assert np.array_equal(a, run_transport.reloc_noise(7)(0, 1).standard_normal(3))  # reproducible per run seed
    assert not np.array_equal(a, run_transport.reloc_noise(7)(1, 1).standard_normal(3))
    assert not np.array_equal(a, run_transport.reloc_noise(8)(0, 1).standard_normal(3))
