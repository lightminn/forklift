"""The unconfirmed position check in the runner (docs/plans/2026-11-02-unconfirmed-runner.md 3): ``--pose-check``
(off by default, not recorded off, in the near-field contract) and its development values, required to turn it on."""

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))
sys.path.insert(0, str(ROOT / "tests/unit/localization"))

from test_riibotics_isaac_scenes import _runner_args  # noqa: E402
from test_start_reloc_runner import _live, _reloc  # noqa: E402

BOUNDS = {"odom": [0.02, 0.02, 0.005, 0.01], "e_still": [0.03, 0.01], "e_slam": [0.05, 0.025]}


def test_the_check_is_off_by_default_and_not_recorded_off(tmp_path):
    import run_transport

    off = _runner_args(_live(tmp_path, *_reloc(tmp_path)))
    assert off.pose_check == "off" and off.pose_check_bounds is None
    assert not {"pose_check", "pose_check_bounds"} & set(run_transport.recorded_arguments(off))


def test_on_needs_the_relocalization_and_every_development_value(tmp_path):
    import run_transport

    on = _runner_args(_live(tmp_path, *_reloc(tmp_path), "--pose-check", "on", "--pose-check-bounds",
                            json.dumps(BOUNDS)))
    assert on.pose_check == "on" and on.pose_check_bounds.e_slam == (0.05, 0.025)
    rec = run_transport.recorded_arguments(on)
    assert rec["pose_check"] == "on" and rec["pose_check_bounds"] == BOUNDS
    json.dumps(rec["pose_check_bounds"])  # plain lists in the record
    for extra in (["--pose-check", "on"],  # no development values (plan 0: no half state)
                  ["--pose-check", "on", "--pose-check-bounds", json.dumps({**BOUNDS, "e_still": [0.5, 0.01]})],
                  ["--pose-check", "on", "--pose-check-bounds", json.dumps({"odom": [0.0] * 4})],
                  ["--pose-check-bounds", json.dumps(BOUNDS)]):  # values without the check
        with pytest.raises(SystemExit):
            _runner_args(_live(tmp_path, *_reloc(tmp_path), *extra))
    with pytest.raises(SystemExit):  # not without the start relocalization
        _runner_args(_live(tmp_path, "--pose-check", "on", "--pose-check-bounds", json.dumps(BOUNDS)))


def test_the_bounds_may_come_from_a_file(tmp_path):
    f = tmp_path / "bounds.json"
    f.write_text(json.dumps(BOUNDS))
    on = _runner_args(_live(tmp_path, *_reloc(tmp_path), "--pose-check", "on", "--pose-check-bounds", str(f)))
    assert on.pose_check_bounds.odom == (0.02, 0.02, 0.005, 0.01)


def test_the_arguments_are_in_the_near_field_contract():
    import near_tracking

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["pose_check"] == "off"
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["pose_check_bounds"] is None


def _source():
    return (ROOT / "sim/isaac/run_transport.py").read_text()


def test_the_check_sits_where_the_plan_puts_it_in_the_tick():
    # plan 3.3 (Codex draft 1 P1-1): after the SLAM replies are applied and the control pose read, before any mission
    # work; again right before the output; the check's end before a session end request can make it a normal end
    src = _source()
    replies = src.index("core.slam_replies(t, replies_")
    first = src.index("pose_checkpoint(t)  # plan 3.3 (2)")
    work = src.index("if obstacle is not None:\n                    obstacle[\"control_rear\"]")
    assert replies < first < work
    second = src.index("pose_checkpoint(t)  # plan 3.3 (3)")
    assert second < src.index("live_safety_ = core.safety_state(t)\n                    out_ = live.tick_end(")
    top_end = src.index('if pose_box["end"] is not None:\n                # plan 3.3: the check ended')
    assert top_end < src.index("if live is not None and live.arbiter.session_request is not None:")


def test_every_new_path_is_behind_the_check():
    # the off path makes exactly the recorded calls (the replay gate): each new call is under pose_box
    src = _source()
    for needle in ("pose_box[\"rt\"].on_scan(", "pose_checkpoint(t)  # plan 3.3 (2)", "pose_checkpoint(t)  # plan 3.3 (3)",
                   "pose_box[\"rt\"] = make_pose_runtime(args"):
        at = src.index(needle)
        assert 'pose_box["rt"] is not None' in src[at - 260:at] or 'pose_box["sink"] is not None' in src[at - 260:at], \
            needle
    assert '**({} if pose_box["rt"] is None' in src  # tick_start's pose_ok only when on


def test_the_end_of_the_check_is_a_mission_failure_in_its_tick_and_a_session_failure_after():
    from forklift_core.mission.failures import classify

    assert classify("pose_ending:pose_revoked:int", strict=True) == "mission"
    assert classify("pose_revoked:int", strict=True) == "session"
    assert 'raise RuntimeError(f"pose_ending:{reason_}")' in _source()


def test_a_setup_recorded_before_the_check_replays_with_it_off():
    from tools import boundary_replay

    assert boundary_replay.SETUP_NAMES_ADDED["pose_box"] == ("pose_check", "off", {"rt": None, "sink": None,
                                                                                  "end": None})
    assert 'if "pose_box" not in _bound:' in _source()
