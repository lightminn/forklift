"""The runner's side of the RTX render preset (sim/isaac/render_preset.py): the argument, its near-field contract entry,
the preset written on every render product after the last camera is made, and the record with the end-of-run check."""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from test_riibotics_isaac_scenes import _runner_args  # noqa: E402

SOURCE = (ROOT / "sim/isaac/run_transport.py").read_text()


def test_default_unless_given():
    assert _runner_args([]).render_preset == "default"
    assert _runner_args(["--render-preset", "fast"]).render_preset == "fast"


def test_the_near_field_contract_compares_it_and_reads_its_absence_as_default():
    import near_tracking

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["render_preset"] == "default"
    assert "render_preset" not in near_tracking.RUN_SPECIFIC_ARGUMENTS


def test_written_after_the_last_camera_is_made():
    at = SOURCE.index('state["render_preset"] = render_preset.apply_all(stage,')
    cameras = [m.start() for m in re.finditer(r"(?<![A-Za-z_])Camera\(", SOURCE)]
    run_cameras = [i for i in cameras if SOURCE.rfind("\ndef ", 0, i) == SOURCE.rfind("\ndef ", 0, at)]
    assert run_cameras and max(run_cameras) < at
    assert max(SOURCE.rfind(".initialize()", 0, at), 0) > max(run_cameras)


def test_the_end_of_run_check_is_recorded():
    assert 'state["render_preset"]["drift_at_end"] = render_preset.drifted(' in SOURCE
