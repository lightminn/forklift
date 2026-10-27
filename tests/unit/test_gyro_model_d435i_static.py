"""T2's C1 arm (docs/plans/2026-10-27-insertion-lateral-drift.md): the measured gyro model, its age table and the
obstacle layer that points at it pass the runner's own table contract, so a GYRO=measured run is not refused."""

import json
from pathlib import Path

import yaml

from forklift_core.localization.encoders import odometry_table_problem
from forklift_core.localization.gyro import GyroModel

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "config/gyro_model_d435i_static_20261023.yaml"
LAYER = ROOT / "config/obstacle_layer_single_team_spec_gyro_d435i_static.yaml"


def test_the_layer_table_and_model_match_for_a_team_spec_gyro_run():
    model, sha = GyroModel.load(MODEL)
    layer = yaml.safe_load(LAYER.read_text())
    table = json.loads((ROOT / layer["odometry_age"]).read_text())
    assert odometry_table_problem(
        table, encoder_model="team_spec_quantized", wheel_radius_m=table["odometry_wheel_radius_m"],
        geometric_radius=True, odometry_heading="gyro", gyro_model_sha256=sha, gyro_mount="carriage_low_measured",
    ) is None
    # the assumed model's table is not this model's
    old = json.loads((ROOT / "config/obstacle_odometry_age_team_spec_gyro.json").read_text())
    assert old["noise"]["gyro_model_sha256"] != sha


def test_the_layer_differs_from_the_assumed_models_only_in_its_age_table():
    strip = lambda p: {k: v for k, v in yaml.safe_load(p.read_text()).items() if k != "odometry_age"}  # noqa: E731
    assert strip(LAYER) == strip(ROOT / "config/obstacle_layer_single_team_spec_gyro.yaml")


def test_the_measured_model_keeps_the_assumed_models_algorithm_settings():
    measured, _ = GyroModel.load(MODEL)
    assumed, _ = GyroModel.load(ROOT / "config/gyro_model_spec_typical.yaml")
    for name in ("scale", "range_dps", "mount_error_deg", "zupt_s", "zupt_k", "gap_s"):
        assert getattr(measured, name) == getattr(assumed, name), name
