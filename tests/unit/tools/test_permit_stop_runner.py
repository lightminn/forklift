"""The runner's P6 switch (docs/plans/2026-10-23-planner-permission-clearance.md 3 P6): ``--permit-stop-replan`` on
by default for a new run, a restored record without it off; the near-field contract's key; the core option."""

import sys
from argparse import Namespace
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from test_riibotics_isaac_scenes import _runner_args  # noqa: E402


def test_the_switch_is_on_for_a_new_run_and_takes_off():
    assert _runner_args([]).permit_stop_replan == "on"
    assert _runner_args(["--permit-stop-replan", "off"]).permit_stop_replan == "off"
    with pytest.raises(SystemExit):
        _runner_args(["--permit-stop-replan", "maybe"])


def test_the_near_field_contract_knows_the_switch():
    import near_tracking

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["permit_stop_replan"] == "on"


def test_the_core_option_reads_a_record_without_the_switch_as_off():
    source = (ROOT / "sim/isaac/run_transport.py").read_text()
    assert 'permit_stop_replan=getattr(args, "permit_stop_replan", "off")' in source
    assert getattr(Namespace(), "permit_stop_replan", "off") == "off"
