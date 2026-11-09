"""The repro batch's H0 (docs/plans/2026-11-09-l4c-repro-cancel-other-target.md 1.3): ``--start-known`` places the
truck at ``--start-pose`` and starts the odometry, the obstacle layer and the bridge's start check at that pose's rear
axle, as at the scene's start -- no relocalization. Off by default and not recorded off; a live run on a survey map."""

import math
import re
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))
sys.path.insert(0, str(ROOT / "tests/unit/localization"))

from test_riibotics_isaac_scenes import _runner_args  # noqa: E402

SCRIPT = ROOT / "sim/isaac/run_transport.py"


def _live(tmp_path, *extra):
    return ["--live", "--slam-feedback", str(tmp_path / "s.sock"), "--slam-map-bundle", str(tmp_path),
            "--expected-map-id", "a" * 64, *extra]


def _scenario(rear=(0.5, -1.0, 0.3)):
    return SimpleNamespace(start_rear=SimpleNamespace(x_m=rear[0], y_m=rear[1], yaw_rad=rear[2]))


def test_the_known_start_is_off_by_default_and_not_recorded_off(tmp_path):
    import run_transport

    args = _runner_args(_live(tmp_path))
    assert args.start_known is False
    assert "start_known" not in run_transport.recorded_arguments(args)
    assert run_transport.known_start_rear(args) is None


def test_a_known_start_takes_its_placed_pose_and_is_recorded(tmp_path):
    import run_transport

    args = _runner_args(_live(tmp_path, "--start-pose=-0.934,0.9106,0.1706", "--start-known"))
    assert args.start_pose == (-0.934, 0.9106, 0.1706) and args.start_known and not args.start_relocalize
    assert {"start_pose", "start_known"} <= set(run_transport.recorded_arguments(args))
    assert "start_relocalize" not in run_transport.recorded_arguments(args)


@pytest.mark.parametrize("extra", [
    ["--start-known"],  # no placed pose
    ["--start-pose", "1,2,0.5", "--start-known", "--start-relocalize", "--reloc-dir", "d"],  # one start or the other
    ["--start-pose", "1,2,0.5", "--start-known", "--reloc-dir", "d"],
    ["--start-pose", "1,2,0.5", "--start-known", "--reloc-unattended"],
    ["--start-pose", "1,2,0.5", "--start-known", "--reloc-operator-timeout-s", "30"],
    ["--start-pose", "1,2,nan", "--start-known"],
])
def test_the_known_start_is_refused_out_of_place(tmp_path, extra):
    with pytest.raises(SystemExit):
        _runner_args(_live(tmp_path, *extra))


def test_a_known_start_is_live_on_a_survey_map(tmp_path):
    for argv in (["--start-pose", "1,2,0.5", "--start-known"],  # not live
                 ["--live", "--slam-feedback", str(tmp_path / "s.sock"), "--start-pose", "1,2,0.5",
                  "--start-known"]):  # online mapping: no map the pose is known on
        with pytest.raises(SystemExit):
            _runner_args(argv)


def test_the_known_rear_axle_is_the_placed_bases_and_the_map_start_takes_it():
    import run_transport

    args = SimpleNamespace(start_known=True, start_pose=(4.0, 3.0, -2.0), rear_axle_offset_m=-0.34)
    rear = run_transport.known_start_rear(args)
    assert rear == pytest.approx((4.0 - 0.34 * math.cos(-2.0), 3.0 - 0.34 * math.sin(-2.0), -2.0))
    # the inverse of spawn_base_pose's scene-start rule: the base is the rear axle plus the offset along the heading
    back = run_transport.spawn_base_pose(SimpleNamespace(start_pose=None, rear_axle_offset_m=-0.34), _scenario(rear))
    assert back == pytest.approx((4.0, 3.0, -2.0))
    scenario = _scenario()
    assert run_transport.map_start_rear(scenario, None, rear) == pytest.approx(rear)
    assert run_transport.map_start_rear(scenario, None, None) == (0.5, -1.0, 0.3)
    assert run_transport.map_start_rear(scenario, None) == (0.5, -1.0, 0.3)  # the scene's start, as before
    # a relocalization's estimate wins (the two never go together: the arguments refuse it)
    assert run_transport.map_start_rear(scenario, {"estimate_rear": [7.0, 8.0, 1.0]}, rear) == (7.0, 8.0, 1.0)
    off = SimpleNamespace(start_known=False, start_pose=(4.0, 3.0, -2.0), rear_axle_offset_m=-0.34)
    assert run_transport.known_start_rear(off) is None


def _source(start: str, end: str) -> str:
    text = SCRIPT.read_text()
    return text[text.index(start):text.index(end)]


def test_the_setup_and_the_drive_both_start_at_the_known_rear_axle():
    run_ = _source("\ndef run(app,", "\ndef drive(S, isaac)")
    drive_ = _source("\ndef drive(S, isaac)", "\ndef recorded_arguments(")
    assert "start_pose=map_start_rear(scenario,start_reloc,known_start_rear(args))" in run_.replace(" ", "")
    assert "slam_start=map_start_rear(scenario,start_reloc,known_start_rear(args))" in drive_.replace(" ", "")
    # the record is written whether or not the obstacle layer runs, and only when the option is on
    assert re.search(r"\n    if known_start_rear\(args\) is not None:[^\n]*\n        state\[\"start_known\"\] = ", run_)
    # the known start keeps the setup's 5 mm spawn check (only a relocalization leaves it to its phase)
    helper = _source("\ndef truck_spawn_problem(", "\nRELOC_SETTLE_STEPS")
    assert "start_known" not in helper


def test_the_near_field_contract_knows_the_option():
    import near_tracking

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["start_known"] is False
