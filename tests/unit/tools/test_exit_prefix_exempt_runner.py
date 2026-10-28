"""The runner's ``--exit-prefix-exempt`` (docs/plans/2026-10-28-exit-prefix-extract-rules.md D1, D6): off by default and
registered off in the near-field contract, read by the core with getattr, scene layout with the obstacle layer only,
refused with --near-tracking; the obstacle scans carry their correction and version to the layer."""

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from test_riibotics_isaac_scenes import _runner_args, tool  # noqa: E402


def _scene(tmp_path):
    path = tmp_path / "aisle.json"
    path.write_text(json.dumps(tool.scene_record("aisle", tool.scenes()["aisle"])))
    return ["--layout", "scene", "--scene-file", str(path)]


def test_off_for_a_new_run_and_registered_off():
    import near_tracking

    assert _runner_args([]).exit_prefix_exempt == "off"
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["exit_prefix_exempt"] == "off"
    source = (ROOT / "sim/isaac/run_transport.py").read_text()
    assert 'exit_prefix_exempt=getattr(args, "exit_prefix_exempt", "off")' in source


def test_on_takes_a_scene_run(tmp_path):
    assert _runner_args(_scene(tmp_path) + ["--exit-prefix-exempt", "on"]).exit_prefix_exempt == "on"


@pytest.mark.parametrize("extra", [[], ["--near-tracking"]])
def test_on_is_refused_outside_a_scene_run_or_with_near_tracking(tmp_path, extra):
    argv = (_scene(tmp_path) if extra else []) + ["--exit-prefix-exempt", "on"] + extra
    with pytest.raises(SystemExit):
        _runner_args(argv)


def test_the_obstacle_scans_carry_their_correction_and_version():
    source = (ROOT / "sim/isaac/run_transport.py").read_text()
    assert ('layer.add_scans(stamp, raw, odom_rear=odom_rear, loaded=loaded_now, correction=obstacle["applied"],\n'
            '                            version=obstacle["version"])') in source
