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
