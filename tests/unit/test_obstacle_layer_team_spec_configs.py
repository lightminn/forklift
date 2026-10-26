"""The team-spec obstacle layers (plan 2026-10-25-team-spec-default): each is its reported twin with the team-spec
odometry age table, nothing else."""
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("reported, team_spec", [
    ("config/obstacle_layer_single.yaml", "config/obstacle_layer_single_team_spec.yaml"),
    ("config/obstacle_layer_single_a2.yaml", "config/obstacle_layer_single_a2_team_spec.yaml")])
def test_a_team_spec_layer_is_its_twin_with_the_team_spec_age_table(reported, team_spec):
    a = yaml.safe_load((ROOT / reported).read_text())
    b = yaml.safe_load((ROOT / team_spec).read_text())
    assert a.pop("odometry_age") == "config/obstacle_odometry_age.json"
    assert b.pop("odometry_age") == "config/obstacle_odometry_age_team_spec.json"
    assert a == b
