"""The runner's D7 re-approach caps (docs/plans/2026-10-25-leg-failures.md B1/B2): ``--d7-reapproach-curvature-inv-m``
and ``--d7-reapproach-cruise-mps`` unset by default, the near-field contract's keys, the core options read with getattr
(a recorded run's arguments may predate them)."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from test_riibotics_isaac_scenes import _runner_args  # noqa: E402


def test_both_caps_are_unset_for_a_new_run_and_take_values():
    args = _runner_args([])
    assert args.d7_reapproach_curvature_inv_m is None and args.d7_reapproach_cruise_mps is None
    args = _runner_args(["--d7-reapproach-curvature-inv-m", "0.24", "--d7-reapproach-cruise-mps", "0.15"])
    assert args.d7_reapproach_curvature_inv_m == 0.24 and args.d7_reapproach_cruise_mps == 0.15


def test_the_near_field_contract_knows_both():
    import near_tracking

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["d7_reapproach_curvature_inv_m"] is None
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["d7_reapproach_cruise_mps"] is None


def test_the_core_reads_both_with_getattr():
    source = (ROOT / "sim/isaac/run_transport.py").read_text()
    assert 'd7_reapproach_curvature_inv_m=getattr(args, "d7_reapproach_curvature_inv_m", None)' in source
    assert 'd7_reapproach_cruise_mps=getattr(args, "d7_reapproach_cruise_mps", None)' in source


def test_the_reapproach_hold_is_off_for_a_new_run_and_a_record_without_it():
    """docs/plans/2026-10-25-reapproach-slam-hold.md H1: --d7-reapproach-hold on|off, off by default, registered off."""
    import near_tracking

    assert _runner_args([]).d7_reapproach_hold == "off"
    assert _runner_args(["--d7-reapproach-hold", "on"]).d7_reapproach_hold == "on"
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["d7_reapproach_hold"] == "off"
    source = (ROOT / "sim/isaac/run_transport.py").read_text()
    assert 'd7_reapproach_hold=getattr(args, "d7_reapproach_hold", "off")' in source
