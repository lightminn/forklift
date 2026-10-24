"""The runner's leg-end diagnostics (docs/plans/2026-10-24-leg-end-budget-stands.md 3): ``--inject-observe-end-offset-m``
and ``--diag-return-stall-budget`` off by default, the near-field contract's keys, the return budget reaching the core
option only when given."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from test_riibotics_isaac_scenes import _runner_args  # noqa: E402


def test_both_diagnostics_are_off_for_a_new_run_and_take_values():
    args = _runner_args([])
    assert args.inject_observe_end_offset_m is None and args.diag_return_stall_budget is None
    args = _runner_args(["--inject-observe-end-offset-m", "0.12", "--diag-return-stall-budget", "0"])
    assert args.inject_observe_end_offset_m == 0.12 and args.diag_return_stall_budget == 0


def test_the_near_field_contract_knows_both():
    import near_tracking

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["inject_observe_end_offset_m"] is None
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["diag_return_stall_budget"] is None


def test_the_return_budget_reaches_the_core_only_when_given():
    source = (ROOT / "sim/isaac/run_transport.py").read_text()
    # read with getattr: a recorded run's arguments (the replay gate's) may predate the diagnostic
    assert ('**({} if getattr(args, "diag_return_stall_budget", None) is None\n'
            '                                   else {"return_stall_budget": int(args.diag_return_stall_budget)}),') in source
    assert "args.diag_return_stall_budget is None" not in source
