"""The vehicle-side F1 block in the runner (docs/plans/2026-11-08-f1-vehicle-block.md 2.5, 3): ``--operator-placed-auto``
(block by default, recorded with the relocalization, in the near-field contract) and the decision handed to the
runtime."""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))
sys.path.insert(0, str(ROOT / "tests/unit/localization"))

from test_riibotics_isaac_scenes import _runner_args  # noqa: E402
from test_start_reloc_runner import _live, _reloc  # noqa: E402


def test_block_is_the_default_and_is_recorded_with_the_relocalization(tmp_path):
    import run_transport

    args = _runner_args(_live(tmp_path, *_reloc(tmp_path)))
    assert args.operator_placed_auto == "block"
    assert run_transport.recorded_arguments(args)["operator_placed_auto"] == "block"


def test_allow_is_taken_and_recorded(tmp_path):
    import run_transport

    args = _runner_args(_live(tmp_path, *_reloc(tmp_path), "--operator-placed-auto", "allow"))
    assert run_transport.recorded_arguments(args)["operator_placed_auto"] == "allow"


def test_a_run_without_the_relocalization_records_what_it_always_did(tmp_path):
    import run_transport

    args = _runner_args(_live(tmp_path))
    assert "operator_placed_auto" not in run_transport.recorded_arguments(args)


def test_another_value_is_refused(tmp_path):
    with pytest.raises(SystemExit):
        _runner_args(_live(tmp_path, *_reloc(tmp_path), "--operator-placed-auto", "maybe"))


def test_the_argument_is_in_the_near_field_contract():
    import near_tracking

    # every earlier run had no block (none of them placed a start and ran a mission)
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["operator_placed_auto"] == "allow"


def test_the_runner_hands_the_decision_to_the_runtime():
    src = (ROOT / "sim/isaac/run_transport.py").read_text()
    assert "start_block=start_block_" in src
    at = src.index("start_block_ = ")
    decision = src[at:at + 600]
    assert "autonomy_block(start_reloc, args.operator_placed_auto" in decision
    assert "estimate_source=" in decision
    assert src.index("start_block_ = ") < src.index("live = LiveRuntime(")
