"""P0a measurement helpers (priority-5 plan): envelope line, relative drift, standoff."""

import importlib.util
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("p0a_measure", ROOT / "tools/p0a_measure.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_the_upper_line_lies_on_or_above_every_point_and_is_tight():
    x = np.array([0.5, 1.0, 2.0, 4.0])
    y = np.array([0.02, 0.025, 0.03, 0.06])
    e0, k = MODULE.upper_line(x, y)
    assert np.all(e0 + k * x >= y - 1e-12)
    # Through the two outer hull points (0.5, 0.02)-(4.0, 0.06) the sum is least.
    assert math.isclose(k, 0.04 / 3.5) and math.isclose(e0, 0.02 - 0.5 * 0.04 / 3.5)


def test_a_flat_series_gives_a_constant_line():
    e0, k = MODULE.upper_line([1.0, 2.0, 3.0], [0.05, 0.04, 0.05])
    assert (e0, k) == (0.05, 0.0)


def test_identical_trajectories_have_no_relative_error():
    t = np.linspace(0, 10, 200)
    poses = np.column_stack((t, 0.1 * t**1.5, 0.05 * t))
    d, p, y = MODULE.relative_errors(poses, poses.copy(), start_every=10, max_distance_m=5)
    assert len(d) and p.max() < 1e-12 and y.max() < 1e-12


def test_a_yaw_bias_grows_with_the_lever_and_not_with_a_common_offset():
    s = np.linspace(0, 6, 601)
    truth = np.column_stack((s, np.zeros_like(s), np.zeros_like(s)))
    # Odometry that is offset everywhere by a constant translation has no relative error.
    shifted = truth + [5.0, -3.0, 0.0]
    d, p, _ = MODULE.relative_errors(truth, shifted, start_every=50, max_distance_m=6)
    assert p.max() < 1e-12
    # A heading that drifts 0.01 rad/m bends the odometry away from the line.
    bent = truth.copy()
    bent[:, 2] = 0.01 * s
    bent[1:, 0] = np.cumsum(np.cos(bent[:-1, 2]) * np.diff(s))
    bent[1:, 1] = np.cumsum(np.sin(bent[:-1, 2]) * np.diff(s))
    d, p, y = MODULE.relative_errors(truth, bent, start_every=600, max_distance_m=6)
    assert math.isclose(y[-1], 0.06, rel_tol=1e-6)
    assert 0.17 < p[-1] < 0.19  # about 0.01/2 * 6^2


def test_standoff_depth_follows_the_similar_triangles():
    # Camera 0.27 m, deck underside 0.10 m, voxel top 0.062 m: L = D (0.208/0.170 - 1).
    depth = MODULE.max_visible_depth_m(1.0, camera_z_m=0.27, deck_bottom_m=0.10, voxel_z_m=0.062)
    assert math.isclose(depth, 0.208 / 0.170 - 1.0)
    # The plan's v8 figure (camera 0.20, voxel 0.05) gives L = D / 2.
    assert math.isclose(
        MODULE.max_visible_depth_m(0.72, camera_z_m=0.20, deck_bottom_m=0.10, voxel_z_m=0.05), 0.36
    )


def test_windows_end_at_the_age_limit():
    stamps = np.linspace(0, 20, 201)
    truth = np.column_stack((0.1 * stamps, np.zeros_like(stamps), np.zeros_like(stamps)))
    d, _, _ = MODULE.relative_errors(
        truth, truth.copy(), start_every=100, max_distance_m=12, stamps=stamps, max_age_s=5
    )
    assert d.max() <= 0.5 + 1e-9


def test_the_age_table_never_decreases_and_reads_each_age_from_the_start():
    stamps = np.arange(0, 5, 0.01)
    truth = np.column_stack((0.5 * stamps, np.zeros_like(stamps), np.zeros_like(stamps)))
    estimate = truth.copy()
    estimate[:, 0] *= 1.1  # 10 % odometry overshoot
    err = MODULE.relative_error_at(0, (0.1, 1.0, 10.0), stamps, truth, estimate)
    assert math.isclose(err[0, 0], 0.005, abs_tol=1e-9) and math.isclose(err[1, 0], 0.05, abs_tol=1e-9)
    assert np.isnan(err[2]).all()
    table = MODULE.cumulative_table(np.array([[0.03, 0.01], [0.02, 0.02], [np.nan, np.nan]]))
    assert table.tolist() == [[0.03, 0.01], [0.03, 0.02], [0.03, 0.02]]


def test_an_error_between_two_listed_ages_bounds_the_later_age():
    # Codex checkpoint 6: a 0.19 s age beat the 0.2 s value; every tick counts.
    stamps = np.arange(0, 2, 0.01)
    truth = np.column_stack((0.5 * stamps, np.zeros_like(stamps), np.zeros_like(stamps)))
    estimate = truth.copy()
    estimate[15, 0] += 0.2  # a spike at 0.15 s that is gone again by 0.2 s
    err = MODULE.relative_error_at(0, (0.1, 0.2, 1.0), stamps, truth, estimate)
    assert err[0, 0] < 1e-9 and math.isclose(err[1, 0], 0.2, abs_tol=1e-9) and math.isclose(err[2, 0], 0.2, abs_tol=1e-9)


def test_a_heading_error_at_the_present_swings_the_old_pose():
    # Codex L0 P1: same positions, the odometry heading 0.1 rad off at the end only.
    stamps = np.array([0.0, 1.0])
    truth = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    odom = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.1]])
    err = MODULE.relative_error_at(0, (1.0,), stamps, truth, odom)
    assert math.isclose(err[0, 0], 2 * math.sin(0.05), rel_tol=1e-9)  # about 0.10 m
    assert math.isclose(err[0, 1], 0.1, rel_tol=1e-9)


def test_triangle_cells_cover_every_cell_the_triangle_touches():
    spec = importlib.util.spec_from_file_location("p0b", ROOT / "tools/p0b_sensor_study.py")
    p0b = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(p0b)
    tri = np.array([[0.01, 0.01], [0.24, 0.01], [0.01, 0.12]])
    cells = {tuple(c) for c in p0b.triangle_cells(tri, 0.05)}
    assert (0, 0) in cells and (4, 0) in cells and (0, 2) in cells
    assert (4, 2) not in cells  # beyond the hypotenuse


def _team_spec_run(directory, *, garbage_rates=True, steps=1200, speed=0.4, model_name="team_spec_typical_inl",
                   still_s=0.0, turn_rad_s=0.0, command_zero=None):
    """A straight 10 s run whose reported rates are garbage (zero) while the wheel joint angles move
    with the truck: a replay that read the rates would see no motion."""
    import json

    from forklift_core.localization.encoders import TeamEncoderModel, encoder_model_spec, sensor_phases

    radius, rear_x = 0.125, -0.34
    t = np.arange(steps) / 120
    moving = np.clip(t - still_s, 0.0, None)
    s = speed * moving
    yaw = turn_rad_s * moving
    if turn_rad_s:  # an arc at the rear axle's speed (heading truth only; the steering stays recorded as 0)
        x, y = speed / turn_rad_s * np.sin(yaw), speed / turn_rad_s * (1 - np.cos(yaw))
    else:
        x, y = s, np.zeros(steps)
    base = np.column_stack((x, y, np.zeros(steps), np.cos(yaw / 2), np.zeros((steps, 2)), np.sin(yaw / 2)))
    theta = s / radius
    angles = np.column_stack((theta, theta, theta, theta))
    wrapped = np.mod(angles + 2 * np.pi, 4 * np.pi) - 2 * np.pi
    from forklift_core.localization.encoders import AngleUnwrapper

    unwrap = [AngleUnwrapper() for _ in range(4)]
    continuous = np.array([[u.update(v) for u, v in zip(unwrap, row, strict=True)] for row in wrapped])
    model = TeamEncoderModel(encoder_model_spec(model_name), sensor_phases(3))
    rows = [model.read(continuous[k, 2:4], (0.0, 0.0)) for k in range(steps)]
    np.savez_compressed(
        directory / "slam_log.npz", joint_stamps_s=t,
        wheel_rates_rad_s=np.zeros((steps, 4)) if garbage_rates else np.full((steps, 4), speed / radius),
        steering_rad=np.zeros((steps, 2)), base_pose_world=base, wheel_angles_rad=continuous,
        wheel_counts=np.array([r[2] for r in rows], dtype=np.int64),
        wheel_increments_rad=np.array([r[0] for r in rows]), steering_measured_rad=np.array([r[1] for r in rows]),
        scan_stamps_s=t[::12], drive_command_zero=(moving == 0) if command_zero is None else command_zero)
    (directory / "meta.json").write_text(json.dumps({
        "record_boundary": False,
        "encoder_model": model.record(),
        "odometry_geometry": {"wheelbase_m": 0.66, "track_m": 0.53, "wheel_radius_m": radius,
                              "rear_axle_x_in_base_m": rear_x}}))


def test_a_team_spec_run_replays_its_encoder_counts_not_the_reported_rates(tmp_path):
    from types import SimpleNamespace

    _team_spec_run(tmp_path)
    out = MODULE.odometry_command(SimpleNamespace(run=[tmp_path], draws=2, start_every=120, max_age_s=None))
    # Counts and INL (1 deg on a 0.125 m wheel = 2.2 mm) only: no Gaussian, no zero-rate drift.
    assert max(v for v in out["position_error_max_m"] if v == v) < 0.006
    assert out["noise"]["encoder_model"] == "team_spec_typical_inl" and out["noise"]["gaussian"] is None
    ages = MODULE.age_command(SimpleNamespace(run=[tmp_path], draws=2))
    assert max(ages["position_max_m"]) < 0.006 and "team-spec" in ages["source"]


def test_runs_with_different_encoder_models_are_not_mixed(tmp_path):
    import json
    from types import SimpleNamespace

    import pytest

    a, b = tmp_path / "a", tmp_path / "b"
    a.mkdir(), b.mkdir()
    _team_spec_run(a)
    _team_spec_run(b)
    meta = json.loads((b / "meta.json").read_text())
    meta.pop("encoder_model")
    (b / "meta.json").write_text(json.dumps(meta))
    with pytest.raises(SystemExit, match="different encoder models"):
        MODULE.odometry_command(SimpleNamespace(run=[a, b], draws=1, start_every=120, max_age_s=None))


def test_runs_with_different_odometry_wheel_radii_are_not_mixed(tmp_path):
    """2026-10-15 (fork spacing plan, odometry-only radius): a table records the radius its runs'
    odometry integrated with and mixes no two."""
    import json
    from types import SimpleNamespace

    import pytest

    a, b = tmp_path / "a", tmp_path / "b"
    a.mkdir(), b.mkdir()
    _team_spec_run(a)
    _team_spec_run(b)
    out = MODULE.odometry_command(SimpleNamespace(run=[a], draws=1, start_every=120, max_age_s=None))
    radius = json.loads((a / "meta.json").read_text())["odometry_geometry"]["wheel_radius_m"]
    assert out["odometry_wheel_radius_m"] == radius
    meta = json.loads((b / "meta.json").read_text())
    meta["odometry_geometry"]["wheel_radius_m"] = radius * 0.996
    (b / "meta.json").write_text(json.dumps(meta))
    with pytest.raises(SystemExit, match="different odometry wheel radii"):
        MODULE.odometry_command(SimpleNamespace(run=[a, b], draws=1, start_every=120, max_age_s=None))
    with pytest.raises(SystemExit, match="different odometry wheel radii"):
        MODULE.age_command(SimpleNamespace(run=[a, b], draws=1))


def test_a_boundary_recorded_run_is_refused_by_its_meta_even_without_result_json(tmp_path):
    """M-a0 (Codex round 6, P2): the tables read meta.json and slam_log.npz, so a copied pair from a
    recorded run is refused by the runner's record_boundary flag in meta.json."""
    import json
    from types import SimpleNamespace

    import pytest

    _team_spec_run(tmp_path)
    meta = json.loads((tmp_path / "meta.json").read_text())
    meta["record_boundary"] = True
    (tmp_path / "meta.json").write_text(json.dumps(meta))
    assert not (tmp_path / "result.json").exists()
    with pytest.raises(SystemExit, match="never measurement evidence"):
        MODULE.odometry_command(SimpleNamespace(run=[tmp_path], draws=1, start_every=120, max_age_s=None))


def test_a_run_whose_recording_cannot_be_verified_is_refused(tmp_path):
    """M-a0 (Codex round 7, P2): a meta.json written before the flag (possibly by a recording runner) needs
    result.json to decide; with neither, the input is refused; result.json decides when it is there."""
    import json
    from types import SimpleNamespace

    import pytest

    _team_spec_run(tmp_path)
    meta = json.loads((tmp_path / "meta.json").read_text())
    meta.pop("record_boundary")
    (tmp_path / "meta.json").write_text(json.dumps(meta))
    with pytest.raises(SystemExit, match="cannot verify"):
        MODULE.odometry_command(SimpleNamespace(run=[tmp_path], draws=1, start_every=120, max_age_s=None))
    (tmp_path / "result.json").write_text(json.dumps({"arguments": {"record_boundary": True}}))
    with pytest.raises(SystemExit, match="never measurement evidence"):
        MODULE.odometry_command(SimpleNamespace(run=[tmp_path], draws=1, start_every=120, max_age_s=None))
    (tmp_path / "result.json").write_text(json.dumps({"arguments": {}}))
    MODULE.odometry_command(SimpleNamespace(run=[tmp_path], draws=1, start_every=120, max_age_s=None))


def test_the_gyro_age_table_runs_the_runner_s_heading_and_records_its_model_and_mount(tmp_path):
    """Gyro-heading plan G2 (A): a team_spec_quantized run standing 1 s, then turning at 0.2 rad/s with the steering
    recorded as 0 -- the steering table sees no turn, the gyro table follows it (bias found in the standstill)."""
    import json
    from types import SimpleNamespace

    import pytest

    from forklift_core.localization.encoders import odometry_table_problem

    _team_spec_run(tmp_path, model_name="team_spec_quantized", still_s=1.0, turn_rad_s=0.2)
    gyro_args = SimpleNamespace(run=[tmp_path], draws=2, odometry_heading="gyro",
                                gyro_model=ROOT / "config/gyro_model_spec_typical.yaml", gyro_mount="carriage_low_measured")
    gyro = MODULE.age_command(gyro_args)
    steering = MODULE.age_command(SimpleNamespace(run=[tmp_path], draws=2))
    assert max(gyro["yaw_max_rad"]) < 0.03 < max(steering["yaw_max_rad"])
    assert gyro["noise"]["odometry_heading"] == "gyro" and gyro["noise"]["gyro_mount"] == "carriage_low_measured"
    assert all(n >= 1 for n in gyro["runs"][0]["gyro_bias_updates"]) and "odometry_heading" not in steering["noise"]
    kw = dict(encoder_model="team_spec_quantized", wheel_radius_m=0.125, geometric_radius=True, odometry_heading="gyro")
    sha = gyro["noise"]["gyro_model_sha256"]
    assert odometry_table_problem(gyro, **kw, gyro_model_sha256=sha, gyro_mount="carriage_low_measured") is None
    assert "gyro model" in odometry_table_problem(gyro, **kw, gyro_model_sha256="0" * 64, gyro_mount="carriage_low_measured")
    assert "mount" in odometry_table_problem(gyro, **kw, gyro_model_sha256=sha, gyro_mount="legacy")
    (tmp_path / "result.json").write_text(json.dumps({"arguments": {"perception_mount": "legacy"}}))
    with pytest.raises(SystemExit, match="recorded with --perception-mount legacy"):
        MODULE.age_command(gyro_args)


def test_a_gyro_age_table_follows_the_logged_command_and_refuses_what_the_runner_would_not_do(tmp_path):
    """Codex G2 (A) P2: the bias block's command condition comes from the log (a command on while the wheels stand
    makes no bias); a truck moving before the first bias, a run with no bias, or a log without the condition is refused."""
    from types import SimpleNamespace

    import numpy as np
    import pytest

    args = dict(draws=1, odometry_heading="gyro", gyro_model=ROOT / "config/gyro_model_spec_typical.yaml",
                gyro_mount="carriage_low_measured")
    late = np.ones(8000, bool)
    late[:61 * 120] = False  # standing, the command on until 61 s: the runner ends the session at 60 s
    cases = {"moving": (dict(still_s=0.0), "moved at"),
             "commanded": (dict(still_s=1.0, command_zero=np.zeros(1200, bool)), "moved at"),
             "never": (dict(still_s=20.0, command_zero=np.zeros(1200, bool)), "no standstill bias update"),
             "late": (dict(still_s=63.0, steps=8000, command_zero=late), "no standstill bias by 60 s")}
    for name, (kw, match) in cases.items():
        run = tmp_path / name
        run.mkdir()
        _team_spec_run(run, model_name="team_spec_quantized", **kw)
        with pytest.raises(SystemExit, match=match):
            MODULE.age_command(SimpleNamespace(run=[run], **args))
    run = tmp_path / "old"
    run.mkdir()
    _team_spec_run(run, model_name="team_spec_quantized", still_s=1.0)
    log = dict(np.load(run / "slam_log.npz"))
    log.pop("drive_command_zero")
    np.savez_compressed(run / "slam_log.npz", **log)
    with pytest.raises(SystemExit, match="no drive_command_zero"):
        MODULE.age_command(SimpleNamespace(run=[run], **args))


def test_the_gyro_age_table_reads_quantized_runs_only(tmp_path):
    from types import SimpleNamespace

    import pytest

    _team_spec_run(tmp_path, still_s=1.0)  # team_spec_typical_inl
    with pytest.raises(SystemExit, match="reads team_spec_quantized runs"):
        MODULE.age_command(SimpleNamespace(run=[tmp_path], draws=1, odometry_heading="gyro",
                                           gyro_model=ROOT / "config/gyro_model_spec_typical.yaml",
                                           gyro_mount="carriage_low_measured"))
