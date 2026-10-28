"""The runner's start-band and escape presteer options (docs/plans/2026-10-27-escape-presteer.md D1, D2, C4''): off by
default; the width and the approach budget only with the release on."""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from test_riibotics_isaac_scenes import _runner_args  # noqa: E402


def test_off_by_default():
    args = _runner_args([])
    assert args.start_band_m is None and args.start_band_approach_m is None and args.escape_presteer == "off"


def test_the_width_and_the_budget_with_the_release_on():
    args = _runner_args(["--start-band-release", "on", "--start-band-m", "0.03", "--start-band-approach-m", "0.002",
                         "--escape-presteer", "on"])
    assert (args.start_band_m, args.start_band_approach_m, args.escape_presteer) == (0.03, 0.002, "on")


@pytest.mark.parametrize("extra", [["--start-band-approach-m", "0.002"],
                                   ["--start-band-release", "on", "--start-band-approach-m", "0.0"],
                                   ["--start-band-release", "on", "--start-band-approach-m", "0.006"],
                                   ["--start-band-m", "0.03"]])
def test_a_width_or_a_budget_without_the_release_or_out_of_range_is_refused(extra):
    with pytest.raises(SystemExit):
        _runner_args(extra)


def test_the_runner_sets_the_budget_on_the_permission_config():
    source = (ROOT / "sim/isaac/run_transport.py").read_text()
    assert "start_band_approach_m=float(approach_) if approach_ is not None else 0.0" in source
