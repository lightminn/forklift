"""The runner's P1c switch (docs/plans/2026-10-23-planner-permission-clearance.md): ``--lidar-keepout-m`` off by
default, finite and non-negative, not with grid planning; the near-field contract's key; the geometry it builds and
records (unchanged while off)."""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from test_riibotics_isaac_scenes import _runner_args  # noqa: E402


def test_the_keepout_is_off_unless_given_and_must_be_a_non_negative_length():
    assert _runner_args([]).lidar_keepout_m == 0.0
    assert _runner_args(["--lidar-keepout-m", "0.15"]).lidar_keepout_m == 0.15
    for bad in ("-0.01", "nan", "inf"):
        with pytest.raises(SystemExit):
            _runner_args(["--lidar-keepout-m", bad])


def test_the_keepout_is_refused_with_grid_planning():
    with pytest.raises(SystemExit):
        _runner_args(["--lidar-keepout-m", "0.1", "--grid-planning"])


def test_the_near_field_contract_knows_the_keepout():
    import near_tracking

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["lidar_keepout_m"] == 0.0


def test_the_recorded_geometry_gains_the_keepout_only_when_it_is_set():
    from dataclasses import asdict

    from forklift_core.mission.replay_codec import recorded_fields
    from forklift_core.planning.pallet_mission import SyntheticMissionGeometry

    def record(g):  # as run_transport records state["geometry"]
        kept = {f.name for f in recorded_fields(g)}
        return {k: v for k, v in asdict(g).items() if k in kept}

    off = record(SyntheticMissionGeometry())
    assert "lidar_keepout_m" not in off and "lidar_plane_m" not in off
    assert set(off) == set(asdict(SyntheticMissionGeometry())) - {"lidar_keepout_m", "lidar_plane_m"}
    assert record(SyntheticMissionGeometry(lidar_keepout_m=0.15))["lidar_keepout_m"] == 0.15
